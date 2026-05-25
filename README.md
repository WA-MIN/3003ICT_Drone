# Tribo — Autonomous Search and Rescue Drone Controller

## Repository
https://github.com/WA-MIN/3003ICT_Drone.git

A Webots simulation of an autonomous drone that patrols a search area, detects survivors, assesses their condition using motion detection, and avoids obstacles — built on a finite state machine architecture.

---

## Dependencies

- Webots R2023a or later
- Python 3.8 or later
- Python modules: `math` (standard library), `controller` (provided by Webots)

---

## How to Replicate

1. Clone the repository:
   ```
   git clone https://github.com/WA-MIN/3003ICT_Drone.git
   ```
2. Place the project folder inside your Webots projects directory.
3. The folder structure should look like this:
   ```
   tribo-drone/
   ├── worlds/
   │   └── mavic_2_pro.wbt
   ├── controllers/
   │   └── my_controller_4/
   │       └── my_controller_4.py
   └── README.md
   ```
4. Open Webots and load `worlds/mavic_2_pro.wbt`.
5. In the robot node properties, confirm the controller field is set to `my_controller_4`.
6. Press play to run the simulation.

---

## How to Run

1. Open your Webots world file (`.wbt`) containing the Mavic 2 Pro drone.
2. In the robot node, set the controller to `my_controller_4` (or the filename of this script).
3. Place this script in your Webots project under `controllers/my_controller_4/my_controller_4.py`.
4. Press the play button in Webots to start the simulation.
5. Console output is prefixed with `[Tribo | STATE]` for easy filtering.

---

## FSM States

| State | Description | Transition |
|---|---|---|
| `TAKEOFF` | Lifts off to patrol altitude of 2m | → `PATROL` once altitude reached |
| `PATROL` | Flies an expanding square spiral searching for survivors | → `TRACK` on survivor detection |
| `TRACK` | Yaws and pitches to centre and lock onto the survivor | → `ASSESS` once position is locked |
| `ASSESS` | Hovers in place and samples motion to determine triage result | → `PATROL` if survivor lost, reassesses every 5s otherwise |
| `LAND` | Descends to ground and cuts motors | Terminal state |
| `AVOID` | Brakes, turns 90 degrees, coasts, then resumes patrol | → `PATROL` after avoidance sequence completes |

The sonar fail-safe runs every timestep and overrides any active state (except `LAND` and `AVOID`) if an obstacle is detected, immediately entering `AVOID`.

---

## Key Parameters

### Flight
| Parameter | Value | Description |
|---|---|---|
| `PATROL_ALT` | 2.0 m | Search and patrol altitude |
| `HOVER_ALT` | 2.0 m | Altitude held during Track and Assess |
| `PATROL_SPEED` | 3.0 | Pitch input magnitude during patrol legs |
| `SQUARE_LEG_TIME` | 3.0 s | Duration of each patrol leg unit |
| `SQUARE_TURN_TIME` | 1.2 s | Duration of each 90-degree yaw turn |

### Tracking
| Parameter | Value | Description |
|---|---|---|
| `YAW_TRACK_GAIN` | 0.006 | Yaw rate per pixel of horizontal error |
| `DESIRED_HEIGHT_RATIO` | 0.40 | Target bounding box height as fraction of frame |
| `POS_HOLD_GAIN` | 2.0 | GPS position hold proportional gain in Assess |

### Motion Detection
| Parameter | Value | Description |
|---|---|---|
| `MOTION_THRESHOLD` | 1.5 | Average per-channel pixel diff to count as motion |
| `MOTION_SETTLE_TIME` | 2.5 s | Settling time before sampling begins |
| `ASSESS_DURATION` | 4.0 s | Sampling window duration |

### Obstacle Avoidance
| Parameter | Value | Description |
|---|---|---|
| `SONAR_DANGER` | 3 | Sliding window average threshold to trigger Avoid |
| `SONAR_CAUTION` | 5 | Threshold to slow patrol speed to 40% |
| `SONAR_SMOOTH` | 5 | Sliding window size in frames |
| `AVOID_BRAKE_DURATION` | 0.8 s | Duration of reverse pitch braking phase |
| `AVOID_COAST_TIME` | 0.5 s | Coast duration after turn to damp yaw momentum |

---

## LED Indicators

| LED | State |
|---|---|
| Blue solid | Takeoff, Patrol, Land |
| Blue blinking | Assess settling phase |
| Yellow solid | Track, Alive triage result |
| Yellow blinking | Assess sampling phase |
| Red solid | Immobile triage result |
| Yellow + Red | Avoid (note: known issue — only yellow visible, see limitations) |

---

## Display

The onboard display shows the current FSM state, LED indicator status, current GPS coordinates, and the triage result once Assess completes.

---

## Known Limitations

- **Sonar field of view** — the sonar sensor is front-facing only. Obstacles to the sides or rear are not detected.
- **LED bug** — during the Avoid state, only the yellow LED is visible despite both red and yellow being set in code. The second `set_leds` call overwrites the first. Does not affect functionality.
- **Motion detection sensitivity** — motion detection assumes a mostly stationary drone. Residual PID oscillations during Assess can produce false motion votes. Mitigated by the 2.5 second settle time and the 30% vote threshold.
- **Sonar ground reflection** — at low altitudes the front sonar detects ground reflections and returns noisy readings. The sonar fail-safe is only reliable above 1 metre.
- **Single survivor** — the controller tracks and assesses the first recognised pedestrian object. Multiple survivors in frame are not handled.

---

## Architecture Overview

The controller follows a Sense-Think-Act loop running every simulation timestep:

- **Sense** — camera, sonar, GPS, gyro, IMU
- **Think** — FSM state machine computes roll, pitch, yaw corrections
- **Act** — motor velocities, LED colours, display output
