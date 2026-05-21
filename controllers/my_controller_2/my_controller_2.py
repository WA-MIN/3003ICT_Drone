# tribo_controller_fov_track.py
# FSM: TAKEOFF -> PATROL -> TRACK

from controller import Robot
import math


# ── Utility ───────────────────────────────────────────────────────────────────
def clamp(value, low, high):
    return max(low, min(value, high))


# ── Robot and timestep ────────────────────────────────────────────────────────
robot = Robot()
timestep = int(robot.getBasicTimeStep())
print(f'[Tribo] Controller started — timestep={timestep} ms')

# ── Devices ───────────────────────────────────────────────────────────────────
imu = robot.getDevice('inertial unit');
imu.enable(timestep)
gps = robot.getDevice('gps');
gps.enable(timestep)
gyro = robot.getDevice('gyro');
gyro.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

camera_roll_motor = robot.getDevice('camera roll')
camera_pitch_motor = robot.getDevice('camera pitch')

fl = robot.getDevice('front left propeller')
fr = robot.getDevice('front right propeller')
rl = robot.getDevice('rear left propeller')
rr = robot.getDevice('rear right propeller')
for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

# ── PID & Flight constants ────────────────────────────────────────────────────
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

PATROL_ALT = 2.0  # metres — search altitude
HOVER_ALT = 1.5  # metres — tracking altitude
ALT_REACHED = 0.2  # metres — altitude tolerance
PATROL_SPEED = 3.0  # pitch input during patrol legs
SQUARE_LEG_TIME = 3.0  # seconds per unit leg
SQUARE_TURN_TIME = 1.2  # seconds per 90-degree yaw turn

# ── Camera FOV Tracking Constants ─────────────────────────────────────────────
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

# 15% margin on the left and right sides
LEFT_LIMIT = CAM_W * 0.15
RIGHT_LIMIT = CAM_W * 0.85

# Desired height of the object on screen (30% of screen height)
DESIRED_HEIGHT_RATIO = 0.30

# ── FSM state constants ───────────────────────────────────────────────────────
TAKEOFF = 'TAKEOFF'
PATROL = 'PATROL'
TRACK = 'TRACK'

# ── State variables ───────────────────────────────────────────────────────────
state = TAKEOFF
target_alt = PATROL_ALT


sq_leg_len = 1
sq_leg_count = 0
sq_timer = 0.0
sq_turning = False



lost_frames  = 0


# ── Helpers ───────────────────────────────────────────────────────────────────
def survivor_detected():
    """Returns (True, obj) if orange survivor is in camera view, else (False, None)."""
    for obj in camera.getRecognitionObjects():
        c = obj.getColors()
        if c and c[0] > 0.8 and c[1] < 0.5 and c[2] < 0.3:
            return True, obj
    return False, None


def apply_motors(roll_d, pitch_d, yaw_d, altitude, roll, pitch, roll_vel, pitch_vel):
    """PID + motor mixer."""
    clamped_diff = clamp(target_alt - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (clamped_diff ** 3)
    roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_vel + roll_d
    pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_d
    yaw_input = yaw_d

    fl_v = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
    fr_v = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
    rl_v = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    rr_v = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

    fl.setVelocity(fl_v)
    fr.setVelocity(-fr_v)
    rl.setVelocity(-rl_v)
    rr.setVelocity(rr_v)

    camera_pitch_motor.setPosition(clamp(-0.1 * pitch_vel, -0.5, 0.5))
    camera_roll_motor.setPosition(clamp(-0.115 * roll_vel, -0.5, 0.5))


# ── Physics warmup ────────────────────────────────────────────────────────────
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print(f'[Tribo | {TAKEOFF}] Lifting off to {PATROL_ALT}m...')

# ── Main loop ─────────────────────────────────────────────────────────────────
while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = timestep / 1000.0

    # Sensor reads
    altitude = gps.getValues()[2]
    rpy = imu.getRollPitchYaw()
    roll = rpy[0]
    pitch = rpy[1]
    gv = gyro.getValues()
    roll_vel = gv[0]
    pitch_vel = gv[1]
    yaw_vel = gv[2]

    # Defaults (Hover)
    roll_d = 0.0
    pitch_d = 0.0
    yaw_d = 0.0

    # ── TAKEOFF ───────────────────────────────────────────────────────────────
    if state == TAKEOFF:
        target_alt = PATROL_ALT
        if abs(altitude - PATROL_ALT) < ALT_REACHED:
            print(f'[Tribo | {TAKEOFF}] Reached {altitude:.2f}m. Starting PATROL.')
            state = PATROL

    # ── PATROL — expanding square ─────────────────────────────────────────────
    elif state == PATROL:
        target_alt = PATROL_ALT
        detected, obj = survivor_detected()

        if detected:
            print(f'[Tribo | {PATROL}] Survivor detected! Switching to TRACK.')
            state = TRACK
        else:
            leg_duration = sq_leg_len * SQUARE_LEG_TIME
            if not sq_turning:
                pitch_d = -PATROL_SPEED
                sq_timer += dt
                if sq_timer >= leg_duration:
                    sq_turning = True
                    sq_timer = 0.0
                    sq_leg_count += 1
                    if sq_leg_count % 2 == 0:
                        sq_leg_len += 1
            else:
                yaw_d = -1.3
                sq_timer += dt
                if sq_timer >= SQUARE_TURN_TIME:
                    sq_turning = False
                    sq_timer = 0.0

    # ── TRACK — Image-based Distance & Centering ──────────────────────────────
        # ── TRACK — Image-based Distance & Centering ──────────────────────────────
        # ── TRACK — Image-based Distance & Centering ──────────────────────────────
    elif state == TRACK:
        # Smoothly ramp down altitude
        if target_alt > HOVER_ALT:
            target_alt = max(HOVER_ALT, target_alt - 0.5 * dt)

        detected, obj = survivor_detected()

        # FIX 1: The Patience Buffer. Don't instantly give up if we blink!
        if not detected:
            lost_frames += 1
            if lost_frames > 20:  # If we lose it for ~20 frames, THEN go back to patrol
                print(f'[Tribo | {TRACK}] Lost survivor completely. Returning to PATROL.')
                state = PATROL
                lost_frames = 0
            else:
                # We lost it momentarily! Hover completely still and wait for it to appear
                pitch_d = 0.0
                yaw_d = 0.0
        else:
            lost_frames = 0  # We see it! Reset the patience counter

            # Get 2D Pixel Data
            pos = obj.getPositionOnImage()
            size = obj.getSizeOnImage()

            obj_x, obj_y = pos[0], pos[1]
            obj_w, obj_h = size[0], size[1]

            obj_left = obj_x - (obj_w / 2)
            obj_right = obj_x + (obj_w / 2)

            # ── Centering Control (Yaw) ──
            center_err_x = obj_x - (CAM_W / 2)

            if abs(center_err_x) > 20:
                # FIX 2: If the drone STILL looks away after adding the patience buffer,
                # change the 0.003 below to -0.003. This flips the turn direction!
                yaw_d = clamp(-0.003 * center_err_x, -0.4, 0.4)
            else:
                yaw_d = 0.0

            # ── Distance Control (Pitch) ──
            if abs(center_err_x) > 60:
                pitch_d = 0.0  # Stop moving forward/back while aggressively turning
            else:
                if obj_left < LEFT_LIMIT or obj_right > RIGHT_LIMIT or (obj_h / CAM_H) > DESIRED_HEIGHT_RATIO:
                    pitch_d = 0.6
                elif (obj_h / CAM_H) < (DESIRED_HEIGHT_RATIO - 0.05):
                    pitch_d = -1.0
                else:
                    pitch_d = 0.0
                    
    # ── PID + motor mixer ─────────────────────────────────────────────────────
    apply_motors(roll_d, pitch_d, yaw_d,
                 altitude, roll, pitch, roll_vel, pitch_vel)