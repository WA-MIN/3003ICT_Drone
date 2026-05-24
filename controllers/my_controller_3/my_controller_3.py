from controller import Robot

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
DESIRED_HEIGHT_RATIO = 0.60   # target object height as fraction of frame to decide drone-obejct distance


#Sonar to use for collision avoidance
sonar = robot.getDevice('sonar_front')
sonar.enable(timestep)


# Sonar constants 
# Sonar returns 0 (obstacle) to maxValue (clear). maxValue=1000 (as default) Lower value means closer obstacle.
SONAR_DANGER  = 3  # emergency hover (obstacle very close)
SONAR_CAUTION = 5  # slow patrol (obstacle nearby)
SONAR_SMOOTH  = 5     # frames to average. avoid going 


#LEDs Red, Blue, Yellow
led_blue   = robot.getDevice('led_blue')
led_yellow = robot.getDevice('led_yellow')
led_red    = robot.getDevice('led_red')


# PID and Flight Constants
K_VERTICAL_THRUST  = 68.5
K_VERTICAL_OFFSET  = 0.6
K_VERTICAL_P       = 3.0
K_ROLL_P           = 50.0
K_PITCH_P          = 30.0

PATROL_ALT       = 2.0   # in meters. search altitude
HOVER_ALT        = 1.5   # in meters. TRACK ASSESS altitude
ALT_REACHED      = 0.2   # in meters. altitude tolerance
PATROL_SPEED     = 3.0   # pitch input during patrol legs
SQUARE_LEG_TIME  = 3.0   # seconds per unit leg
SQUARE_TURN_TIME = 1.2   # seconds per 90-degree yaw turn -> AVOID state turn 90 degrees

#Motion detection constants ────────────────────────────────────────────────
MOTION_THRESHOLD   = 1.5   # average per-channel pixel diff to count as motion
MOTION_SETTLE_TIME = 1.5   # seconds to hover still before sampling
ASSESS_DURATION    = 4.0   # seconds of observation before verdict

#FSM states 
TAKEOFF = 'TAKEOFF'  #Beginning
PATROL  = 'PATROL'   #Search for human
TRACK   = 'TRACK'    #Follow / Center the human in the camera vision
ASSESS  = 'ASSESS'   #Assess health condition based on the movement
LAND = "LAND"        #Safety landing
AVOID = "AVOID"      #Collision avoidance

#State variables
state      = TAKEOFF      #Start
target_alt = PATROL_ALT

sq_leg_len   = 1
sq_leg_count = 0
sq_timer     = 0.0
sq_turning   = False

lost_frames  = 0
sonar_readings = []

assess_timer  = 0.0
prev_image    = None
motion_votes  = 0      #Use in ASSESS. assess human movement
total_votes   = 0
triage_done   = False
triage_result = None

#LED helpers
def leds_off():
    led_blue.set(0)
    led_yellow.set(0)
    led_red.set(0)

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
def detect_motion():
    """
    Detect motion using frame difference compared to previous image. Only reliable when drone is hovering still.
    Returns True if mean per-channel pixel change exceeds MOTION_THRESHOLD. (To avoid false assessment due to constant drone position adjustment)
    """
    global prev_image
    current = camera.getImage()
    if prev_image is None or len(current) != len(prev_image):
        prev_image = current
        return False
    total_diff = 0
    for i in range(0, len(current), 4):  # BGRA
        total_diff += abs(current[i]   - prev_image[i])
        total_diff += abs(current[i+1] - prev_image[i+1])
        total_diff += abs(current[i+2] - prev_image[i+2])
    prev_image = current
    avg_diff = total_diff / (CAM_W * CAM_H * 3)
    return avg_diff > MOTION_THRESHOLD

#Sonar Check
def update_sonar():
    #Reads the sensor once per frame and updates the sliding window.
    global sonar_readings
    val = sonar.getValue()
    if val >= 0: 
        sonar_readings.append(val)
    if len(sonar_readings) > SONAR_SMOOTH:
        sonar_readings.pop(0)

def get_sonar_avg():
    
    if not sonar_readings:
        return 999.0  # no valid readings yet — assume clear
    return sum(sonar_readings) / len(sonar_readings)   #return average 

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

    camera_pitch_motor.setPosition(clamp(-0.1   * pitch_vel, -0.5, 0.5))
    camera_roll_motor.setPosition( clamp(-0.115 * roll_vel,  -0.5, 0.5))

#ASSESS reset to keep monitoring
def reset_assess():
    global assess_timer, prev_image, motion_votes, total_votes, triage_done, triage_result
    assess_timer  = 0.0
    prev_image    = None
    motion_votes  = 0
    total_votes   = 0
    triage_done   = False
    triage_result = None

#Physics warmup 
leds_off()
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print(f'[Tribo | TAKEOFF] Lifting off to {PATROL_ALT}m...')

#Main loop 

    dt      = timestep / 1000.0
    gps_pos = gps.getValues()
    altitude = gps_pos[2]
    rpy      = imu.getRollPitchYaw()
    roll     = rpy[0]; pitch = rpy[1]
    gv       = gyro.getValues()
    roll_vel = gv[0];  pitch_vel = gv[1]

    roll_d  = 0.0
    pitch_d = 0.0
    yaw_d   = 0.0
    
    update_sonar()

    #SONAR FAIL-SAFE (overrides all states) 
    if obstacle_near() and state not in [LAND, AVOID]:
        if state == TAKEOFF:
            state = LAND
            print(f'[Tribo | SAFETY] Obstacle detected on takeoff. Landing.')
        else:     
            print(f'[Tribo | SAFETY] Obstacle within {SONAR_DANGER}m! Braking and turning around.')
            state = AVOID
            avoid_timer = 0.0

    #TAKEOFF State Drone takes off
    if state == TAKEOFF:
        target_alt = PATROL_ALT
        set_leds(blue=1)
        if abs(altitude - PATROL_ALT) < ALT_REACHED:
            print(f'[Tribo | TAKEOFF] Reached {altitude:.2f}m. Starting PATROL.')
            state = PATROL

    #PATROL State search for human
    elif state == PATROL:
        target_alt = PATROL_ALT
        set_leds(blue=1)

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

    #TRACK adjust drone position, get ready to assess
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
                lost_frames = 0
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

            # Yaw to centre horizontally
            yaw_d = clamp(-0.003 * center_err_x, -0.4, 0.4) if abs(center_err_x) > 20 else 0.0

            # Pitch for distance, only when roughly centred
            if abs(center_err_x) > 30:
                pitch_d = 0.0
            elif height_ratio > DESIRED_HEIGHT_RATIO:
                pitch_d = 0.6    # too close — back away
            elif height_ratio < (DESIRED_HEIGHT_RATIO - 0.05):
                pitch_d = -1.0   # too far — approach
            else:
                # Good position → ASSESS
                print(f'[Tribo | TRACK] Survivor centred at GPS '
                      f'({gps_pos[0]:.2f}, {gps_pos[1]:.2f}, {gps_pos[2]:.2f}). '
                      f'Entering ASSESS.')
                state = ASSESS
                reset_assess()
                leds_off()
                pitch_d = 0.0

    #ASSESS
    elif state == ASSESS:
        target_alt = HOVER_ALT
        assess_timer += dt

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

            if not triage_done:
                if assess_timer <= MOTION_SETTLE_TIME:
                    # Settling — blink blue slowly
                    set_leds(blue = 1 if int(assess_timer * 2) % 2 == 0 else 0)
                    #print(f'[DEBUG | ASSESS] Settling {assess_timer:.1f}s / {MOTION_SETTLE_TIME}s')
                    print(f'[Tribo | ASSESS] Assessing')
                else:
                    # Sampling — blink yellow rapidly
                    set_leds(yellow = 1 if int(assess_timer * 6) % 2 == 0 else 0)

                    motion = detect_motion()
                    total_votes += 1
                    if motion:
                        motion_votes += 1

                    #print(f'[DEBUG | ASSESS] t={assess_timer:.1f}s  '
                    #     f'motion={motion}  votes={motion_votes}/{total_votes}')
                    print(f'[Tribo | ASSESS] Keep Assessing')

                    if assess_timer >= MOTION_SETTLE_TIME + ASSESS_DURATION:
                        motion_ratio = motion_votes / total_votes if total_votes > 0 else 0

                        if motion_ratio > 0.3:
                            triage_result = 'ALIVE'
                            leds_off(); set_leds(yellow=1)   # 🟡 conscious, moving
                        else:
                            triage_result = 'IMMOBILE'
                            leds_off(); set_leds(red=1)      # 🔴 critical, not moving

                        triage_done = True
                        print(f'[Tribo | ASSESS] ╔══ TRIAGE RESULT ══╗')
                        print(f'[Tribo | ASSESS] ║  {triage_result}')
                        print(f'[Tribo | ASSESS] ║  GPS : ({gps_pos[0]:.2f}, {gps_pos[1]:.2f}, {gps_pos[2]:.2f})')
                        print(f'[Tribo | ASSESS] ║  Motion ratio : {motion_ratio:.2f} ({motion_votes}/{total_votes} frames)')
                        print(f'[Tribo | ASSESS] ╚═══════════════════╝')
            else:
                # Hold 5 s with result LED go back into patrol mode. It detects the survivor again and run assessment. The drone keeps monitoring the survivor
                if assess_timer >= MOTION_SETTLE_TIME + ASSESS_DURATION + 5.0:
                    print(f'[Tribo | ASSESS] Assessment complete. Resuming PATROL.')
                    state = PATROL
                    reset_assess()
    
    #LAND State safety landing               
    elif state == LAND: 
        target_alt = 0
        set_leds(blue=1)
        # FIX: Compare actual altitude against the tolerance threshold
        if altitude < ALT_REACHED:
            print(f'Safely landed')
            apply_motors(0,0,0,0,0,0,0,0)
            continue # Skip motor application so they stay off
    #AVOID State  
    elif state == AVOID:
        set_leds(red=1, yellow=1)
        target_alt = PATROL_ALT  # Maintain current altitude
        avoid_timer += dt
        
        # Phase 1: Hard brake (0.0 to 0.8s)
        if avoid_timer < 0.8:
            pitch_d = PATROL_SPEED  # Pitch backward to counter forward momentum
            yaw_d = 0.0
            
        # Phase 2: Spin 180 degrees in place
        elif avoid_timer < (0.8 + SQUARE_TURN_TIME):
            pitch_d = 0.0
            yaw_d = -1.3            # Use the exact same yaw rate as your patrol turns
            
        # Phase 3: Resume patrol in the opposite direction
        else:
            print(f'[Tribo | AVOID] Turnaround complete. Resuming PATROL.')
            state = PATROL
            sq_timer = 0.0          # Reset patrol leg timer
            sq_turning = False      # Ensure it starts flying straight immediately
        

    #Motor mixer 
    apply_motors(roll_d, pitch_d, yaw_d,
                 altitude, roll, pitch, roll_vel, pitch_vel)