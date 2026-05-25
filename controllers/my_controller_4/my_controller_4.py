from controller import Robot
import math

#Setup
def clamp(value, low, high):
    return max(low, min(value, high))
    
robot = Robot()
timestep = int(robot.getBasicTimeStep())
print(f'[Tribo] Controller started — timestep={timestep} ms')

#Drone setting
imu  = robot.getDevice('inertial unit'); imu.enable(timestep)
gps  = robot.getDevice('gps');           gps.enable(timestep)
gyro = robot.getDevice('gyro');          gyro.enable(timestep)

#Drone propellers
fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')
for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

#Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

camera_roll_motor  = robot.getDevice('camera roll')
camera_pitch_motor = robot.getDevice('camera pitch')

# Camera constants
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()
DESIRED_HEIGHT_RATIO = 0.40   # target object height as fraction of frame to decide drone-object distance

#Sonar to use for collision avoidance
sonar = robot.getDevice('sonar_front')
sonar.enable(timestep)

# Sonar constants
# Sonar returns 0 (obstacle) to maxValue (clear). maxValue=1000 (as default). Lower value means closer obstacle.
SONAR_DANGER  = 3  # emergency hover (obstacle very close)
SONAR_CAUTION = 5  # slow patrol (obstacle nearby)
SONAR_SMOOTH  = 5  # frames to average, avoid jitter

#LEDs Red, Blue, Yellow
led_blue   = robot.getDevice('led_blue')
led_yellow = robot.getDevice('led_yellow')
led_red    = robot.getDevice('led_red')

#Display
display = robot.getDevice('display')

# PID and Flight Constants
K_VERTICAL_THRUST  = 68.5
K_VERTICAL_OFFSET  = 0.6
K_VERTICAL_P       = 3.0
K_ROLL_P           = 50.0
K_PITCH_P          = 30.0

PATROL_ALT       = 2.0   # in meters. search altitude
HOVER_ALT        = 2.0   # in meters. TRACK ASSESS altitude
ALT_REACHED      = 0.2   # in meters. altitude tolerance
PATROL_SPEED     = 3.0   # pitch input during patrol legs
SQUARE_LEG_TIME  = 3.0   # seconds per unit leg
SQUARE_TURN_TIME = 1.2   # seconds per 90-degree yaw turn -> AVOID state turn 90 degrees

# Tracking constants
YAW_TRACK_GAIN = 0.006   # yaw rate per pixel of horizontal error
POS_HOLD_GAIN  = 2.0     # GPS position hold proportional gain used in ASSESS. Keep gentle to avoid oscillation

# Camera stabilisation gains
CAM_PITCH_GAIN = 0.1
CAM_ROLL_GAIN  = 0.115

# Avoid constants
AVOID_BRAKE_DURATION = 0.8   # seconds of reverse pitch to kill forward momentum
AVOID_COAST_TIME     = 0.5   # seconds to hold hover after turn before resuming patrol

#Motion detection constants
# NOTE: threshold applies to the bounding-box average only (not full frame),
# so it should be much lower than a full-frame threshold. A walking pedestrian
# in Webots typically produces 0.3-0.6 avg diff per pixel per channel.
MOTION_THRESHOLD   = 1.5   # average per-channel pixel diff within bounding box 0.3 was too sensitive. 
MOTION_SETTLE_TIME = 2.5   # seconds to hover still before sampling. Increased to allow PID to fully damp
ASSESS_DURATION    = 4.0   # seconds of observation — more samples = more reliable verdict

#FSM states
TAKEOFF = 'TAKEOFF'  # Beginning
PATROL  = 'PATROL'   # Search for human
TRACK   = 'TRACK'    # Follow / Center the human in the camera vision
ASSESS  = 'ASSESS'   # Assess health condition based on the movement
LAND    = 'LAND'     # Safety landing
AVOID   = 'AVOID'    # Collision avoidance

#State variables
state      = TAKEOFF      # Start
target_alt = PATROL_ALT
track_locked = False
locked_pos   = None

sq_leg_len   = 1
sq_leg_count = 0
sq_timer     = 0.0
sq_turning   = False

lost_frames  = 0
sonar_readings = []

# FIX #1: initialise avoid_timer at the top so it is always defined
# before the AVOID fail-safe or state block references it
avoid_timer = 0.0

assess_timer  = 0.0
prev_image    = None
motion_votes  = 0      # Use in ASSESS. assess human movement
total_votes   = 0
triage_done   = False
triage_result = None


#LED helpers
def leds_off():
    # Delegate to set_leds to avoid duplicating LED logic
    set_leds()

def set_leds(blue=0, yellow=0, red=0):
    led_blue.set(255 if blue else 0)
    led_yellow.set(255 if yellow else 0)
    led_red.set(255 if red else 0)

#Survivor detection
def survivor_detected():
    for obj in camera.getRecognitionObjects():
        model = obj.getModel().lower()
        if 'pedestrian' in model or 'human' in model:
            return True, obj
    return False, None

#Motion detection
def detect_motion(obj):
    """
    Detect motion using frame difference within the survivor's bounding box.
    Masking to the bounding box prevents background drift (from minor drone
    movement) inflating motion votes with false positives.
    Returns True if mean per-channel pixel change exceeds MOTION_THRESHOLD.
    """
    global prev_image
    # Copy bytes explicitly — in some Webots versions getImage() returns a
    # reference to an internal buffer updated in-place, so storing the raw
    # return value means prev_image and current always point to the same data
    # and the diff is always zero.
    current = bytes(camera.getImage())

    # Diagnostic — print once per call so we can see what state prev_image is in
    print(f'[DEBUG | MOTION] prev_image={"None" if prev_image is None else len(prev_image)}  current={len(current)}')

    if prev_image is None or len(current) != len(prev_image):
        # First call or resolution mismatch — store and skip this frame
        prev_image = current
        return False

    # Compute diff only within the survivor bounding box
    pos  = obj.getPositionOnImage()
    size = obj.getSizeOnImage()
    x1 = int(max(0,     pos[0] - size[0] / 2))
    y1 = int(max(0,     pos[1] - size[1] / 2))
    x2 = int(min(CAM_W, pos[0] + size[0] / 2))
    y2 = int(min(CAM_H, pos[1] + size[1] / 2))

    total_diff  = 0
    pixel_count = 0

    for py in range(y1, y2):
        for px in range(x1, x2):
            i = (py * CAM_W + px) * 4  # BGRA stride
            total_diff += abs(current[i]   - prev_image[i])
            total_diff += abs(current[i+1] - prev_image[i+1])
            total_diff += abs(current[i+2] - prev_image[i+2])
            pixel_count += 1

    prev_image = current

    if pixel_count == 0:
        return False

    avg_diff = total_diff / (pixel_count * 3)
    print(f'[DEBUG | MOTION] avg_diff={avg_diff:.3f}  threshold={MOTION_THRESHOLD}  bbox=({x1},{y1})-({x2},{y2})')
    return avg_diff > MOTION_THRESHOLD

#Sonar helpers
def update_sonar():
    # Reads the sensor once per frame and updates the sliding window
    global sonar_readings
    val = sonar.getValue()
    if val >= 0:
        sonar_readings.append(val)
    if len(sonar_readings) > SONAR_SMOOTH:
        sonar_readings.pop(0)
    print(f'[SONAR] raw={val:.2f}  avg={get_sonar_avg():.2f}  alt={altitude:.2f}')

def get_sonar_avg():
    if not sonar_readings:
        return 999.0  # no valid readings yet — assume clear
    return sum(sonar_readings) / len(sonar_readings)   # return average

def obstacle_near():
    return get_sonar_avg() < SONAR_DANGER

def obstacle_caution():
    return get_sonar_avg() < SONAR_CAUTION

#Motor mixer
def apply_motors(roll_d, pitch_d, yaw_d, altitude, roll, pitch, roll_vel, pitch_vel):
    clamped_diff   = clamp(target_alt - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (clamped_diff ** 3)
    roll_input     = K_ROLL_P  * clamp(roll,  -1.0, 1.0) + roll_vel + roll_d
    pitch_input    = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_d
    yaw_input      = yaw_d

    fl.setVelocity( K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input)
    fr.setVelocity(-(K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input))
    rl.setVelocity(-(K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input))
    rr.setVelocity( K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input)

    camera_pitch_motor.setPosition(clamp(-CAM_PITCH_GAIN * pitch_vel, -0.5, 0.5))
    camera_roll_motor.setPosition( clamp(-CAM_ROLL_GAIN  * roll_vel,  -0.5, 0.5))

#ASSESS reset to keep monitoring
def reset_assess():
    global assess_timer, prev_image, motion_votes, total_votes, triage_done, triage_result
    assess_timer  = 0.0
    prev_image    = None
    motion_votes  = 0
    total_votes   = 0
    triage_done   = False
    triage_result = None

#Display
def update_display(state, led_blue, led_yellow, led_red, gps_pos, triage_result=None, motion_ratio=None):
    display.setColor(0x111111)
    display.fillRectangle(0, 0, 256, 128)  # clear

    # State label
    display.setColor(0xFFFFFF)
    display.drawText(f'State: {state}', 10, 8)

    # LED indicators
    display.setColor(0x0000FF if led_blue   else 0x333333); display.fillRectangle(10, 30, 20, 20)
    display.setColor(0xFFFF00 if led_yellow else 0x333333); display.fillRectangle(40, 30, 20, 20)
    display.setColor(0xFF0000 if led_red    else 0x333333); display.fillRectangle(70, 30, 20, 20)
    display.setColor(0xFFFFFF)
    display.drawText('B', 15, 35)
    display.drawText('Y', 45, 35)
    display.drawText('R', 75, 35)

    # GPS
    display.drawText(f'GPS: ({gps_pos[0]:.1f}, {gps_pos[1]:.1f}, {gps_pos[2]:.1f})', 10, 62)

    # Triage result (only after ASSESS completes)
    if triage_result:
        display.setColor(0xFFFF00 if triage_result == 'ALIVE' else 0xFF0000)
        display.drawText(triage_result, 10, 88)

#Physics warmup
leds_off()
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print(f'[Tribo | TAKEOFF] Lifting off to {PATROL_ALT}m...')

#Main loop
while robot.step(timestep) != -1:
    dt      = timestep / 1000.0
    gps_pos = gps.getValues()
    altitude = gps_pos[2]
    rpy      = imu.getRollPitchYaw()
    roll     = rpy[0]; pitch = rpy[1]; yaw = rpy[2]
    gv       = gyro.getValues()
    roll_vel = gv[0];  pitch_vel = gv[1]

    roll_d  = 0.0
    pitch_d = 0.0
    yaw_d   = 0.0

    update_sonar()

    #SONAR FAIL-SAFE (overrides all states)
    if obstacle_near() and state not in [LAND, AVOID]:
        if state == TAKEOFF and robot.getTime() > 10.0:
            state = LAND
            print(f'[Tribo | SAFETY] Obstacle detected on takeoff. Landing.')
        else:
            print(f'[Tribo | SAFETY] Obstacle within {SONAR_DANGER}m! Braking and turning around.')
            state = AVOID
            avoid_timer = 0.0   # reset timer when newly entering AVOID

    #TAKEOFF State — drone takes off
    if state == TAKEOFF:
        target_alt = PATROL_ALT
        set_leds(blue=1)
        if abs(altitude - PATROL_ALT) < ALT_REACHED:
            print(f'[Tribo | TAKEOFF] Reached {altitude:.2f}m. Starting PATROL.')
            state = PATROL
            update_display(state, 1, 0, 0, gps_pos)

    #PATROL State — search for human
    elif state == PATROL:
        track_locked = False
        locked_pos   = None
        target_alt = PATROL_ALT
        set_leds(blue=1)
        update_display(state, 1, 0, 0, gps_pos)

        detected, obj = survivor_detected()
        if detected:
            print(f'[Tribo | PATROL] Survivor detected at GPS '
                  f'({gps_pos[0]:.2f}, {gps_pos[1]:.2f}, {gps_pos[2]:.2f}). '
                  f'Switching to TRACK.')
            state = TRACK
        else:
            # Sonar caution — slow patrol if obstacle nearby
            speed = PATROL_SPEED * 0.4 if obstacle_caution() else PATROL_SPEED

            leg_duration = sq_leg_len * SQUARE_LEG_TIME
            if not sq_turning:
                pitch_d   = -speed
                sq_timer += dt
                if sq_timer >= leg_duration:
                    sq_turning    = True
                    sq_timer      = 0.0
                    sq_leg_count += 1
                    if sq_leg_count % 2 == 0:
                        sq_leg_len += 1
            else:
                yaw_d     = -1.3
                sq_timer += dt
                if sq_timer >= SQUARE_TURN_TIME:
                    sq_turning = False
                    sq_timer   = 0.0

    #TRACK — adjust drone position, get ready to assess
    elif state == TRACK:
        set_leds(yellow=1)
        if target_alt > HOVER_ALT:
            target_alt = max(HOVER_ALT, target_alt - 0.5 * dt)

        detected, obj = survivor_detected()
        if not detected:
            lost_frames += 1
            if lost_frames > 60:
                print(f'[Tribo | TRACK] Lost survivor. Returning to PATROL.')
                state = PATROL
                lost_frames  = 0
                track_locked = False
                locked_pos   = None
            update_display(state, 0, 1, 0, gps_pos)
        else:
            lost_frames = 0
            pos  = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            obj_x = pos[0]
            obj_h = size[1]
            center_err_x = obj_x - (CAM_W / 2)
            height_ratio = obj_h / CAM_H
            print(f'[DEBUG | TRACK] center_err={center_err_x:.1f}  '
                  f'height_ratio={height_ratio:.3f}')

            if not track_locked:
                update_display(state, 0, 1, 0, gps_pos)
                # Yaw to centre horizontally
                yaw_d = clamp(-YAW_TRACK_GAIN * center_err_x, -0.8, 0.8) if abs(center_err_x) > 20 else 0.0
                # Pitch for distance, only when roughly centred
                if abs(center_err_x) > 10:
                    pitch_d = 0.0
                elif height_ratio > (DESIRED_HEIGHT_RATIO + 0.1):
                    pitch_d = 0.6
                elif height_ratio < (DESIRED_HEIGHT_RATIO - 0.1):
                    pitch_d = -1.0
                else:
                    # Good position — lock here
                    track_locked = True
                    locked_pos   = list(gps_pos)   # FIX #2a: list() for a reliable independent copy
                    print(f'[Tribo | TRACK] Position locked at '
                          f'({locked_pos[0]:.2f}, {locked_pos[1]:.2f}, {locked_pos[2]:.2f}). '
                          f'Entering ASSESS.')
                    state  = ASSESS
                    reset_assess()
                    leds_off()
                    # FIX #2b: zero ALL inputs, not just pitch_d, so the yaw
                    # correction computed earlier this frame does not carry through
                    # apply_motors and nudge the drone off position at transition
                    roll_d = pitch_d = yaw_d = 0.0
            else:
                # Locked — hold position, no movement
                update_display(state, 0, 1, 0, gps_pos)
                roll_d  = 0.0
                pitch_d = 0.0
                yaw_d   = 0.0

    #ASSESS — observe survivor and vote on motion
    elif state == ASSESS:
        target_alt = HOVER_ALT
        assess_timer += dt

        # Hold the locked GPS position.
        # The GPS error is in world frame (north/east), so rotate it into the
        # drone's body frame using current yaw before applying as pitch/roll inputs.
        # Without this, corrections fire on the wrong axis whenever the drone
        # is not facing north, which destabilises it immediately on entry.
        if locked_pos is not None:
            err_x = gps_pos[0] - locked_pos[0]   # world north error
            err_y = gps_pos[1] - locked_pos[1]   # world east error
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            # Rotate into body frame
            body_fwd   =  err_x * cos_y + err_y * sin_y   # forward axis -> pitch
            body_right = -err_x * sin_y + err_y * cos_y   # right axis   -> roll
            pitch_d = clamp(body_fwd   * POS_HOLD_GAIN, -1.0, 1.0)
            roll_d  = clamp(body_right * POS_HOLD_GAIN, -1.0, 1.0)

        detected, obj = survivor_detected()

        if not detected:
            lost_frames += 1
            if lost_frames > 30:
                print(f'[Tribo | ASSESS] Lost survivor. Returning to PATROL.')
                state = PATROL
                lost_frames = 0
                reset_assess()
        else:
            lost_frames = 0

            # Gently yaw to keep the pedestrian horizontally centred while holding
            # GPS position. Using a smaller dead zone and gentler gain than TRACK
            # so the yaw correction does not disturb the position hold or
            # introduce background motion that corrupts motion votes.
            obj_x        = obj.getPositionOnImage()[0]
            center_err_x = obj_x - (CAM_W / 2)
            if abs(center_err_x) > 30:   # wider dead zone than TRACK to stay stable
                yaw_d = clamp(-YAW_TRACK_GAIN * 0.5 * center_err_x, -0.4, 0.4)

            if not triage_done:
                if assess_timer <= MOTION_SETTLE_TIME:
                    # Settling — blink blue slowly, wait for PID to stabilise
                    set_leds(blue = 1 if int(assess_timer * 2) % 2 == 0 else 0)
                    update_display(state, 1, 0, 0, gps_pos)
                    print(f'[Tribo | ASSESS] Settling {assess_timer:.1f}s / {MOTION_SETTLE_TIME}s')
                else:
                    # Sampling — blink yellow rapidly
                    set_leds(yellow = 1 if int(assess_timer * 6) % 2 == 0 else 0)
                    update_display(state, 0, 1, 0, gps_pos)

                    # Pass obj into detect_motion so it masks diff to the bounding box
                    motion = detect_motion(obj)
                    total_votes += 1
                    if motion:
                        motion_votes += 1

                    print(f'[Tribo | ASSESS] Sampling  t={assess_timer:.1f}s  '
                          f'motion={motion}  votes={motion_votes}/{total_votes}')

                    if assess_timer >= MOTION_SETTLE_TIME + ASSESS_DURATION:
                        motion_ratio = motion_votes / total_votes if total_votes > 0 else 0

                        if motion_ratio > 0.3:
                            triage_result = 'ALIVE'
                            leds_off(); set_leds(yellow=1)   # yellow — conscious, moving
                            update_display(state, 0, 1, 0, gps_pos, triage_result, motion_ratio)
                        else:
                            triage_result = 'IMMOBILE'
                            leds_off(); set_leds(red=1)      # red — critical, not moving
                            update_display(state, 0, 0, 1, gps_pos, triage_result, motion_ratio)

                        triage_done = True

                        print(f'[Tribo | ASSESS] ╔TRIAGE RESULT ')
                        print(f'[Tribo | ASSESS] ║  {triage_result}')
                        print(f'[Tribo | ASSESS] ║  GPS : ({gps_pos[0]:.2f}, {gps_pos[1]:.2f}, {gps_pos[2]:.2f})')
                        print(f'[Tribo | ASSESS] ║  Motion ratio : {motion_ratio:.2f} ({motion_votes}/{total_votes} frames)')
                        print(f'[Tribo | ASSESS] ╚════════════════')
            else:
                # Hold 5s showing the result LED, then run another assessment cycle
                # in place. The drone stays here as long as the survivor is visible.
                # The lost-survivor check above handles the fallback to PATROL.
                if assess_timer >= MOTION_SETTLE_TIME + ASSESS_DURATION + 5.0:
                    print(f'[Tribo | ASSESS] Reassessing survivor.')
                    reset_assess()   # clears timer and votes — stays in ASSESS

    #LAND State — safety landing
    elif state == LAND:
        target_alt = 0
        set_leds(blue=1)
        update_display(state, 1, 0, 0, gps_pos)
        if altitude < ALT_REACHED:
            print(f'Safely landed')
            apply_motors(0, 0, 0, 0, 0, 0, 0, 0)
            continue   # skip motor application so they stay off

    #AVOID State — brake, spin 90, coast, resume patrol
    elif state == AVOID:
        set_leds(red=1, yellow=1)
        update_display(state, 0, 1, 1, gps_pos)
        target_alt = PATROL_ALT   # maintain current altitude
        avoid_timer += dt

        # Phase 1: Hard brake — pitch backward to kill forward momentum
        if avoid_timer < AVOID_BRAKE_DURATION:
            pitch_d = PATROL_SPEED
            yaw_d   = 0.0

        # Phase 2: Spin 90 degrees in place using the same yaw rate as patrol turns
        elif avoid_timer < (AVOID_BRAKE_DURATION + SQUARE_TURN_TIME):
            pitch_d = 0.0
            yaw_d   = -1.3

        # FIX #5: Coast phase — hold hover and let yaw momentum damp before
        # resuming patrol, otherwise the leftover yaw causes a sideways lurch
        elif avoid_timer < (AVOID_BRAKE_DURATION + SQUARE_TURN_TIME + AVOID_COAST_TIME):
            pitch_d = 0.0
            yaw_d   = 0.0

        # Phase 4: Resume patrol in the opposite direction
        else:
            print(f'[Tribo | AVOID] Turnaround complete. Resuming PATROL.')
            state      = PATROL
            sq_timer   = 0.0     # reset patrol leg timer
            sq_turning = False   # ensure it starts flying straight immediately

    #Motor mixer
    apply_motors(roll_d, pitch_d, yaw_d,
                 altitude, roll, pitch, roll_vel, pitch_vel)
