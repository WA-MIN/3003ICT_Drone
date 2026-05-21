# tribo_controller.py
# Tribo — Autonomous Search & Rescue Scout Drone (Mavic 2 Pro)
# FSM: TAKEOFF → PATROL → APPROACH → ASSESS → MONITORING ↔ ADJUST
#
# Built on Week 9 Mavic PID pattern.
# Fully autonomous after launch — no keyboard or supervisor needed.

from controller import Robot
import math

# ── Utility (from Wk9) ────────────────────────────────────────────────────────

def clamp(value, low, high):
    return max(low, min(value, high))

# ── Robot and timestep ────────────────────────────────────────────────────────

robot    = Robot()
timestep = int(robot.getBasicTimeStep())
print(f'[Tribo] Controller started — timestep={timestep} ms')

# ── Devices (same names as Wk9 Mavic) ────────────────────────────────────────

imu  = robot.getDevice('inertial unit'); imu.enable(timestep)
gps  = robot.getDevice('gps');           gps.enable(timestep)
gyro = robot.getDevice('gyro');          gyro.enable(timestep)

camera = robot.getDevice('camera'); camera.enable(timestep)
camera.recognitionEnable(timestep)

camera_roll_motor  = robot.getDevice('camera roll')
camera_pitch_motor = robot.getDevice('camera pitch')
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

CAMERA_CENTER_X = camera.getWidth() / 2

fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')
for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

prev_error_x=0 
yaw_direction = 1

# ── PID constants (from Wk9 — do not change) ─────────────────────────────────

K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P      = 3.0
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0

# ── Mission constants ─────────────────────────────────────────────────────────

PATROL_ALT      = 2.0    # metres — search altitude
HOVER_ALT       = 1.5    # metres — hover near survivor
ALT_REACHED     = 0.2    # metres — altitude tolerance (from Wk9)
STOP_DIST       = 1.5    # metres — horizontal distance to stop and assess
ADJUST_DIST     = 0.8    # metres — too close, ascend
APPROACH_SPEED  = 1.5    # pitch input during approach
PATROL_SPEED    = 3.0    # pitch input during patrol legs
SQUARE_LEG_TIME = 3.0    # seconds per unit leg
SQUARE_TURN_TIME= 1.2    # seconds per 90-degree yaw turn
ASSESS_DURATION = 3.0    # seconds to observe survivor movement
MOVE_THRESHOLD  = 0.3    # metres — displacement to classify as MOBILE
STATUS_INTERVAL = 5.0    # seconds between status prints in MONITORING

# ── Mobility classifications ──────────────────────────────────────────────────

MOBILE     = 'MOBILE'
STATIONARY = 'STATIONARY'
RETREATING = 'RETREATING'

# ── FSM state constants ───────────────────────────────────────────────────────

TAKEOFF    = 'TAKEOFF'
PATROL     = 'PATROL'
APPROACH   = 'APPROACH'
ASSESS     = 'ASSESS'
MONITORING = 'MONITORING'
ADJUST     = 'ADJUST'

# ── State variables ───────────────────────────────────────────────────────────

state          = TAKEOFF
target_alt     = PATROL_ALT

# Expanding square patrol
sq_leg_len     = 1       # current leg length multiplier
sq_leg_count   = 0       # legs completed at this length
sq_timer       = 0.0
sq_turning     = False

# Assessment
assess_timer       = 0.0
assess_start_pos   = None    # [x, z] survivor position at start of window
mobility           = None    # result: MOBILE / STATIONARY / RETREATING

# Monitoring
status_timer       = 0.0

# ── Helpers ───────────────────────────────────────────────────────────────────

def survivor_detected():
    """Returns (True, obj) if orange survivor is in camera view, else (False, None)."""
    for obj in camera.getRecognitionObjects():
        c = obj.getColors()
        if c and c[0] > 0.8 and c[1] < 0.5 and c[2] < 0.3:
            return True, obj
    return False, None

def horizontal_dist(obj):
    """Horizontal distance to survivor in robot frame (X-Z plane)."""
    pos = obj.getPosition()
    return math.sqrt(pos[0]**2 + pos[2]**2)

def survivor_angle(obj):
    """Lateral angle offset — used to steer toward survivor."""
    pos = obj.getPosition()
    return math.atan2(pos[0], pos[2])

def survivor_xz(obj):
    """Returns [x, z] position of survivor in robot frame."""
    pos = obj.getPosition()
    return [pos[0], pos[2]]

def classify_mobility(start_pos, end_pos):
    """
    Compares survivor position at start and end of assessment window.
    Returns MOBILE, STATIONARY, or RETREATING.
    """
    dx = end_pos[0] - start_pos[0]
    dz = end_pos[1] - start_pos[1]
    displacement = math.sqrt(dx**2 + dz**2)

    if displacement <= MOVE_THRESHOLD:
        return STATIONARY
    # If Z increased (moving away from drone in forward axis) = retreating
    if dz > 0.15:
        return RETREATING
    return MOBILE

def mobility_response(mob):
    """Returns (signal_label, message) based on mobility classification."""
    if mob == MOBILE:
        return '[GREEN — slow pulse]', 'Survivor is mobile. Help is on the way. Stay calm.'
    elif mob == STATIONARY:
        return '[RED — fast pulse]', 'Survivor is stationary. Urgent assistance required!'
    else:
        return '[YELLOW — medium pulse]', 'Survivor is moving away. Maintaining contact.'

def apply_motors(roll_d, pitch_d, yaw_d, altitude, roll, pitch, roll_vel, pitch_vel):
    """PID + motor mixer — carried directly from Wk9."""
    clamped_diff   = clamp(target_alt - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (clamped_diff ** 3)
    roll_input     = K_ROLL_P  * clamp(roll,  -1.0, 1.0) + roll_vel  + roll_d
    pitch_input    = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_d
    yaw_input      = yaw_d

    fl_v = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
    fr_v = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
    rl_v = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    rr_v = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

    fl.setVelocity( fl_v)
    fr.setVelocity(-fr_v)
    rl.setVelocity(-rl_v)
    rr.setVelocity( rr_v)

    camera_pitch_motor.setPosition(
        clamp(-0.1 * pitch_vel, -0.5, 0.5)
    )
    camera_roll_motor.setPosition(
        clamp(-0.115 * roll_vel, -0.5, 0.5)
    )

# ── Physics warmup (from Wk9) ─────────────────────────────────────────────────

while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print(f'[Tribo | {TAKEOFF}] Lifting off to {PATROL_ALT}m...')

# ── Main loop ─────────────────────────────────────────────────────────────────

while robot.step(timestep) != -1:

    t  = robot.getTime()
    dt = timestep / 1000.0

    # ── Sensor reads (top of loop — always current, same as Wk9) ─────────────
    altitude            = gps.getValues()[2]
    rpy                 = imu.getRollPitchYaw()
    roll, pitch         = rpy[0], rpy[1]
    gv                  = gyro.getValues()
    roll_vel  = gv[0]
    pitch_vel = gv[1]
    yaw_vel   = gv[2]

    # FSM writes these — PID reads them at the bottom every step
    roll_d  = 0.0
    pitch_d = 0.0
    yaw_d   = 0.0

    # ── TAKEOFF ───────────────────────────────────────────────────────────────
    if state == TAKEOFF:
        target_alt = PATROL_ALT
        if abs(altitude - PATROL_ALT) < ALT_REACHED:
            print(f'[Tribo | {TAKEOFF}] Reached {altitude:.2f}m. Starting PATROL.')
            state = PATROL

    # ── PATROL — expanding square ─────────────────────────────────────────────
    elif state == PATROL:
        target_alt    = PATROL_ALT
        detected, obj = survivor_detected()

        if detected:
            print(f'[Tribo | {PATROL}] Survivor detected! Switching to APPROACH.')
            state = APPROACH
        else:
            leg_duration = sq_leg_len * SQUARE_LEG_TIME
            if not sq_turning:
                pitch_d    = -PATROL_SPEED          # fly forward
                sq_timer  += dt
                if sq_timer >= leg_duration:
                    sq_turning   = True
                    sq_timer     = 0.0
                    sq_leg_count += 1
                    if sq_leg_count % 2 == 0:       # grow every 2 legs
                        sq_leg_len += 1
            else:
                yaw_d     = -1.3                    # turn right 90 degrees
                sq_timer += dt
                if sq_timer >= SQUARE_TURN_TIME:
                    sq_turning = False
                    sq_timer   = 0.0

    # ── APPROACH ──────────────────────────────────────────────────────────────
    
    elif state == APPROACH:

        target_alt = HOVER_ALT
        detected, obj = survivor_detected()

        if not detected:
            print(f'[Tribo | {APPROACH}] Lost survivor. Returning to PATROL.')
            target_alt = PATROL_ALT
            state = PATROL

        else:
            dist    = horizontal_dist(obj)
            img_pos = obj.getPositionOnImage()

            if img_pos is not None and len(img_pos) >= 2:
                error_x = img_pos[0] - CAMERA_CENTER_X

                if dist <= STOP_DIST:
                    # ── Close enough — stop and assess ──
                    print(f'[Tribo | {APPROACH}] Within {dist:.2f}m. Entering ASSESS.')
                    assess_timer     = 0.0
                    assess_start_pos = None
                    state            = ASSESS
                    # yaw_d, pitch_d, roll_d remain 0.0 (set at top of loop)

                elif abs(error_x) < 60:
                    # Centred — move forward AND correct drift
                    yaw_d   = clamp(error_x * 0.004, -0.3, 0.3)  # gentle proportional correction
                    yaw_d  += -yaw_vel * 2.5                       # kill residual spin
                    pitch_d = -APPROACH_SPEED
                    print(f'[Tribo | {APPROACH}] CENTERED (err={error_x:.1f}) yaw_d={yaw_d:.3f}')
                
                else:
                    # Far off-centre — rotate to face target
                    yaw_proportional = error_x * 0.008          # no upper clamp, let it be strong
                    yaw_damping      = -yaw_vel * 2.0
                    yaw_d            = clamp(yaw_proportional + yaw_damping, -3.0, 3.0)  # much higher ceiling
                    pitch_d          = 0.0
                    print(f'[Tribo | {APPROACH}] Centering (err={error_x:.1f}) yaw_vel={yaw_vel:.3f} yaw_d={yaw_d:.3f}')
    # ── APPROACH ──────────────────────────────────────────────────────────────


                  
        #target_alt    = HOVER_ALT                   # descend as we close in
        #detected, obj = survivor_detected()

        #if not detected:
        #    print(f'[Tribo | {APPROACH}] Lost survivor. Returning to PATROL.')
        #    target_alt = PATROL_ALT
        #    state      = PATROL
        #else:
        #    dist = horizontal_dist(obj)

        # Get target position on camera image
         #   img_pos = obj.getPositionOnImage()

         #   if img_pos is not None and len(img_pos) >= 2:

         #       target_x = img_pos[0]

            # Horizontal offset from image center
            #    error_x = target_x - CAMERA_CENTER_X

            #    print(f'error_x = {error_x}')
                # Move forward only if reasonably centered
             #   if abs(error_x) < 20:
             #       pitch_d = -APPROACH_SPEED
            #        yaw_d   = 0.0
            #    else:
             #       pitch_d = -0.1

            # Close enough -> stop and assess
             #   if dist <= STOP_DIST:

               #     print(f'[Tribo | {APPROACH}] Within {dist:.2f}m. Entering ASSESS.')

              #      assess_timer = 0.0
              #      assess_start_pos = None
               #     state = ASSESS

               # else:


                # Turn toward target
                
               #     yaw_d = clamp(error_x * 0.01, -0.3, 0.3)
               #     pitch_d = -0.1

        """
            dist  = horizontal_dist(obj)
            angle = survivor_angle(obj)
            print(angle)

            if dist <= STOP_DIST:
                print(f'[Tribo | {APPROACH}] Within {dist:.2f}m. Entering ASSESS.')
                assess_timer     = 0.0
                assess_start_pos = None
                state            = ASSESS
            else:
                #spd     = APPROACH_SPEED if dist > 2.5 else APPROACH_SPEED * 0.5
                #pitch_d = -spd
                #yaw_d   = -angle * 1.5            # steer toward survive

                # If target is centered -> move forward
                #if abs(angle) < 0.1:
                    pitch_d = -APPROACH_SPEED
                    yaw_d   = 0.0

                # Otherwise rotate until centered
                #else:
                    #pitch_d = 0.0
                    #yaw_d   = clamp(-angle * 0.6, -0.5, 0.5)
                    
          """

    # ── ASSESS — observe movement over 3 seconds ──────────────────────────────
    elif state == ASSESS:
        APPROACH_SPEED = 0
        print(APPROACH_SPEED, pitch_d)
        target_alt    = HOVER_ALT
        detected, obj = survivor_detected()

        if detected:
            # Record start position on first step
            if assess_start_pos is None:
                assess_start_pos = survivor_xz(obj)
                print(f'[Tribo | {ASSESS}] Observing survivor for {ASSESS_DURATION}s...')

            assess_timer += dt

            if assess_timer >= ASSESS_DURATION:
                end_pos    = survivor_xz(obj)
                mobility   = classify_mobility(assess_start_pos, end_pos)
                signal, msg = mobility_response(mobility)
                print(f'[Tribo | {ASSESS}] Assessment complete → {mobility}')
                print(f'[Tribo | {ASSESS}] {signal} {msg}')
                status_timer = 0.0
                state        = MONITORING
        else:
            # Lost survivor mid-assessment — restart
            print(f'[Tribo | {ASSESS}] Lost contact. Returning to PATROL.')
            state = PATROL

    # ── MONITORING ────────────────────────────────────────────────────────────
    elif state == MONITORING:
        target_alt    = HOVER_ALT
        detected, obj = survivor_detected()

        if detected and horizontal_dist(obj) < ADJUST_DIST:
            print(f'[Tribo | {MONITORING}] Survivor too close. Ascending.')
            state = ADJUST
        else:
            status_timer += dt
            APPROACH_SPEED = 0
            print(APPROACH_SPEED, pitch_d)

            # Refresh classification every STATUS_INTERVAL seconds
            if status_timer >= STATUS_INTERVAL:
                status_timer = 0.0
                if detected:
                    # Re-assess movement since last check
                    current_pos = survivor_xz(obj)
                    if assess_start_pos is not None:
                        mobility    = classify_mobility(assess_start_pos, current_pos)
                    assess_start_pos = current_pos if detected else assess_start_pos
                signal, msg = mobility_response(mobility)
                print(f'[Tribo | {MONITORING}] {signal} {msg}')

            # Follow retreating survivor to maintain contact
            if mobility == RETREATING and detected:
                angle   = survivor_angle(obj)
                pitch_d = -0.5                      # slow follow
                yaw_d   = -angle * 1.0

    # ── ADJUST — ascend if survivor too close ─────────────────────────────────
    elif state == ADJUST:
        target_alt   += 0.3 * dt                    # gradual climb
        detected, obj = survivor_detected()

        if not detected or horizontal_dist(obj) >= ADJUST_DIST:
            target_alt = HOVER_ALT
            print(f'[Tribo | {ADJUST}] Safe distance restored. Back to MONITORING.')
            state = MONITORING

    # ── PID + motor mixer — runs every step (from Wk9) ───────────────────────
    apply_motors(roll_d, pitch_d, yaw_d,
                 altitude, roll, pitch, roll_vel, pitch_vel)