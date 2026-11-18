"""All tunable settings for the ultrasonic + IR MCL localizer.

Coordinates: inches, origin at the top-left corner of the maze grid, +x to the
right, +y DOWN (screen convention). Headings are radians measured from +x,
increasing CLOCKWISE on screen (i.e. turning right increases theta).
"""
import math

# --------------------------------------------------------------------------
# Serial link
# --------------------------------------------------------------------------
SERIAL_PORT = "COM18"         # the master Uno; override with --port COM5
SERIAL_BAUD = 9600            # override with --baud. Must match Serial.begin()
                              # in the master sketch (arduino/ folder: 9600)
# Text to send after connecting to put the firmware into its PC-control mode.
# The master sketch shows a menu at power-up and needs "c". Nothing else is
# sent until the first R line arrives; the text is repeated until then.
# Set to None for firmware that streams R lines straight away.
SERIAL_START_COMMAND = "c"
SERIAL_START_DELAY_S = 2.5    # wait for the Arduino to reboot after the port opens
SERIAL_START_RETRY_S = 2.0
# A drive command identical to the last one sent is only repeated this often
# (a keepalive for the firmware's 0.5 s watchdog). Changes go out at once.
# Sending every frame let lines pile up faster than the master read them, and
# commands arrived up to 18 s late.
SERIAL_CMD_KEEPALIVE_S = 0.2
SERIAL_DISTANCE_UNITS = "in"  # units of odometry distance and ranges in the
                              # robot's messages: "cm", "mm" or "in"
# Robot reports heading counter-clockwise positive (standard math / ROS
# convention: turning LEFT increases heading). Set False if your firmware
# reports clockwise positive.
HEADING_CCW_POSITIVE = True
# Send the pose estimate back to the robot as "P,<x_in>,<y_in>,<theta_deg>".
SEND_POSE_TO_ROBOT = False
# Odometry jumps bigger than this between two messages are treated as a glitch
# or firmware reset and ignored (the baseline is re-synced).
MAX_ODOM_JUMP_IN = 24.0

# --------------------------------------------------------------------------
# Range sensors (order MUST match the order of ranges in each message)
#   name, mount angle (deg, clockwise from robot front: right = +90, left = -90),
#   forward offset (in), right offset (in) of the sensor face from the robot's
#   rotation centre, sensor type (key of SENSOR_TYPES).
# Offsets estimated from a top-down photo (6 in top plate = scale), measured
# from the CENTRE OF THE TOP PLATE to each sonar's mesh face / IR's lens face.
# If the wheel axle (the real turning centre) is NOT under the plate centre,
# subtract the axle's forward offset from every forward offset below.
# IR angle: 30 deg off straight ahead, measured on the robot.
# The navigator looks sensors up by name: keep "front", "right", "back",
# "left"; "front_left" / "front_right" are optional.
# --------------------------------------------------------------------------
SENSORS = [
    ("front",         0.0,  3.0,   0.0,  "sonar"),
    ("right",        90.0, -0.15,  3.2,  "sonar"),
    ("back",        180.0, -3.35,  0.0,  "sonar"),
    ("left",        -90.0, -0.15, -3.1,  "sonar"),
    ("front_left",  -30.0,  1.7,  -1.65, "ir"),
    ("front_right",  30.0,  1.7,   1.65, "ir"),
]

# Per-type sensor model. Measurement likelihood = mixture of a Gaussian around
# the expected range (std = sigma_base + sigma_prop * range) and a uniform
# "random reading" component (echo off the wrong surface, crosstalk).
#   beam_deg         full cone angle (the nearest surface in the cone is read)
#   min_in / max_in  readings are clipped to this range
#   reliable_min_in  closer than this the sensor can't be trusted. The Sharp IR
#                    output FOLDS BACK below its minimum (4 cm for the
#                    GP2Y0A41SK, 10 cm for the GP2Y0A21YK: closer walls read
#                    farther), so wherever the map says the IR would be closer
#                    than this, that reading is treated as uninformative, and
#                    measured values below it are ignored.
#   sim_*            noise used by the simulated robot only
SENSOR_TYPES = {
    "sonar": dict(beam_deg=15.0, min_in=2.0 / 2.54, max_in=120.0 / 2.54,
                  reliable_min_in=2.0 / 2.54,
                  sigma_base=1.0, sigma_prop=0.04, random_prob=0.10,
                  sim_noise_in=0.5, sim_noise_prop=0.0, sim_outlier=0.03,
                  sim_folds=False),
    # Sharp GP2Y0A41SK0F: 4-30 cm, narrow beam, noise grows with distance.
    # (For a GP2Y0A21YK0F use 10 / 80 / 10 cm, and CTRL_IR_MODEL 21 in the
    # master sketch.) Readings beyond max_in count as "nothing in range".
    "ir": dict(beam_deg=5.0, min_in=4.0 / 2.54, max_in=30.0 / 2.54,
               reliable_min_in=4.0 / 2.54,
               sigma_base=0.5, sigma_prop=0.06, random_prob=0.05,
               sim_noise_in=0.2, sim_noise_prop=0.03, sim_outlier=0.01,
               sim_folds=True),
    # VL53L0X time-of-flight: ~25 deg field of view, 3-100 cm usable
    "tof": dict(beam_deg=25.0, min_in=3.0 / 2.54, max_in=100.0 / 2.54,
                reliable_min_in=3.0 / 2.54,
                sigma_base=0.3, sigma_prop=0.03, random_prob=0.05,
                sim_noise_in=0.15, sim_noise_prop=0.02, sim_outlier=0.01,
                sim_folds=False),
}

# --------------------------------------------------------------------------
# Block pickup. A 1 in block sits somewhere in target 1 (the green zone), too
# low for the sonars/IRs. A VL53L0X on the BACK of the robot (under the back
# sonar) finds it; the gripper is also on the back. The ToF is NOT used for
# localization, only for finding and approaching the block.
#   Same tuple format as SENSORS. Its reading is the LAST field of each R line.
# --------------------------------------------------------------------------
TOF_SENSOR = ("tof", 180.0, -3.0, 0.0, "tof")
BLOCK_SIZE_IN = 1.0
# The gripper works in a slot in the chassis: a gripped block sits touching the
# robot's back edge. MEASURE: distance from the rotation centre (midpoint
# between the wheels) to that back edge.
REAR_EDGE_IN = 3.0
GRIP_REACH_IN = REAR_EDGE_IN + BLOCK_SIZE_IN / 2   # rotation centre -> centre of a gripped block
# At contact the block is closer to the ToF than the VL53L0X can measure
# (~3 cm), so the last stretch is driven by odometry:
TOF_CLOSE_IN = 1.6                    # once a valid ToF reading is <= this (~4 cm)...
GRIP_CONTACT_EXTRA_IN = 0.2           # ...back up the measured remaining gap plus this, by odometry
GRIP_CONFIRM_MAX_IN = 2.0             # after G: ToF must read below this (or nothing) = block present
GRIP_TIME_S = 4.0                   # wait this long after G / L for the gripper to move
BLOCK_MIN_GAP_IN = 2.0                # ToF must read this much shorter than the map's
                                      # wall distance to count as "block"
BLOCK_SWEEP_DEG_S = 45.0              # turning speed while sweeping the ToF
BLOCK_REFINE_HALF_DEG = 30.0          # close-up sweep: +-this around the expected bearing
BLOCK_STAGE_GAP_IN = 6.0              # staging point: this much farther out than the pickup pose
BLOCK_REVERSE_SPEED_IN_S = 2.0        # backing onto the block
BLOCK_RELEASE_CLEAR_IN = 3.0          # after letting go at target 2, drive forward this far
BLOCK_MAX_ATTEMPTS = 3                # pickup attempts before giving up
BLOCK_TASK_TIMEOUT_S = 30.0           # any single search/approach step taking longer -> retry
SIM_GRIP_CAPTURE_IN = 1.0             # (sim) block must be this close to the slot to be gripped
# Stop this far OUTSIDE target 1 and sweep from there first: the ToF faces
# backwards, so driving into the zone blind could shove a block that sits
# near an entrance.
BLOCK_SEARCH_STANDOFF_IN = 6
# Raise each update's likelihood to this power (<1 = trust each reading less).
# Consecutive sonar readings have correlated errors; without this the filter
# gets overconfident and can lock onto a look-alike spot in the maze.
SENSOR_LIKELIHOOD_POWER = 0.5

# --------------------------------------------------------------------------
# Particle filter
# --------------------------------------------------------------------------
NUM_PARTICLES = 5000
# Only fuse sensor readings after the robot moved this much since the last
# fusion (repeated readings from the same spot are not independent evidence).
UPDATE_MIN_DIST_IN = 0.5
UPDATE_MIN_ANGLE_RAD = math.radians(5)
RESAMPLE_NEFF_RATIO = 0.5             # resample when N_eff < ratio * N

# Odometry noise (Thrun-style): std dev of distance / rotation error
ODOM_DIST_PER_DIST = 0.10             # in of error per in travelled
ODOM_DIST_PER_ROT = 0.20              # in of error per rad turned
ODOM_ROT_PER_ROT = 0.10               # rad of error per rad turned
ODOM_ROT_PER_DIST = 0.02              # rad of error per in travelled

# Particles whose centre is outside the drivable corridor get this log-weight
# penalty at each sensor update (the robot physically can't be there).
OFF_CORRIDOR_LOG_PENALTY = -5.0

# Augmented MCL: inject random particles when readings suddenly fit badly
# (recovers from a wrong convergence or the robot being picked up).
AUGMENT_ALPHA_SLOW = 0.02
AUGMENT_ALPHA_FAST = 0.2
MAX_RANDOM_INJECTION = 0.2

CONVERGED_SPREAD_IN = 4.0             # spread below this = "localized"
CONVERGED_MIN_WEIGHT = 0.6            # ...and the main cluster holds this much weight
CLUSTER_RADIUS_IN = 6.0               # pose estimate = mean of the main cluster

# --------------------------------------------------------------------------
# Precomputed range table
# --------------------------------------------------------------------------
TABLE_RES_IN = 0.5                    # spatial resolution of the table
TABLE_FILE = "sensor_table.npz"

# --------------------------------------------------------------------------
# Display
# --------------------------------------------------------------------------
PPI = 12                              # screen pixels per inch
MAX_PARTICLES_DRAWN = 1500
FPS = 60

# --------------------------------------------------------------------------
# Simulated robot (--sim): drive with W/A/S/D, emits the same serial lines
# --------------------------------------------------------------------------
SIM_RATE_HZ = 15
SIM_SPEED_IN_S = 6.0
SIM_TURN_DEG_S = 90.0
SIM_ODOM_SCALE_ERR = 0.03             # systematic odometry scale error
SIM_CMD_TIMEOUT_S = 0.5               # robot stops if no "C," command for this long
SIM_SAFETY_STOP_IN = 2.0              # robot refuses to drive forward when its own
                                      # front sonar reads less than this
SIM_CMD_LATENCY_S = 0.15              # (sim) delay before a command takes effect, like
                                      # the real serial + I2C + motor spin-up delay
# Robot body, as a circle around the turning centre: 7 in diameter (6 in plate
# plus sensors, board corners and a little slack). Sensor positions are set
# separately in SENSORS. Used for the path's wall margin (NAV_SAFE_MARGIN_IN),
# the room check before pulling away from the dropped block, and counting wall
# contacts in the simulator. Keep loose cables inside this circle!
ROBOT_RADIUS_IN = 3.5

# --------------------------------------------------------------------------
# Autopilot / maze solving. PC -> robot command:
#   C,<forward speed>,<turn rate deg/s>
# speed in SERIAL_DISTANCE_UNITS per second, turn rate in the same sign
# convention as the odometry heading (HEADING_CCW_POSITIVE).
# With a real robot the autopilot starts DISARMED; press N to arm it.
# --------------------------------------------------------------------------
# The robot cannot turn and drive forward at the same time: every command is
# either a pure drive (C,v,0) or a pure turn in place (C,0,w). Set True if the
# robot can drive curves.
ROBOT_CAN_ARC = False
NAV_ALIGN_START_DEG = 5.0             # (no arcs) heading error above this: stop, turn in place...
NAV_ALIGN_DONE_DEG = 2.0              # ...until it is below this, then drive straight
NAV_TURN_LEAD_S = 0.15                # (no arcs) end each turn this early (x turn rate) to allow
                                      # for the delay before the stop takes effect: raise it if
                                      # turns overshoot, lower it if they stop short
NAV_TURN_SETTLE_S = 0.3               # (no arcs) hold still this long after a turn, then re-check
NAV_CLEAR_AHEAD_IN = 3.0              # (no arcs) only drive if this much straight ahead is corridor
NAV_TURN_MIN_DEG_S = 20.0             # slowest in-place turn sent (beat motor deadband)
NAV_STRAIGHT_LOOKAHEAD_IN = 10.0      # (no arcs) aim at the farthest path point within
                                      # this distance that is reachable in a straight line
WANDER_SIDE_TURN_S = 0.3              # (no arcs) turn away this long when a side wall is too close
# Fixed-speed mode: instead of a range of speeds, every command uses one of
# two fixed magnitudes (direction kept). Simpler for the firmware's speed
# control and gives more repeatable odometry.
#   normal  - driving along paths, wandering, sweeps, big turns
#   precise - anything the autopilot asks to do slowly: the last few degrees
#             of lining up a heading, the close-up block sweep, and backing
#             onto the block (requests at or below the *_BELOW_* limits)
FIXED_SPEED_MODE = False
FIXED_SPEED_IN_S = 4.0
FIXED_TURN_DEG_S = 60.0
FIXED_PRECISE_SPEED_IN_S = 1.5
FIXED_PRECISE_TURN_DEG_S = 20.0
FIXED_PRECISE_BELOW_SPEED_IN_S = 2.5  # requested |speed| <= this -> precise speed
FIXED_PRECISE_BELOW_TURN_DEG_S = 35.0 # requested |turn| < this -> precise turn rate
CMD_RATE_HZ = 20                      # how often commands (and P, poses) are sent
DATA_STALE_S = 1.0                    # no "R," line for this long -> command stop
# Target 1 (the green zone): cells x0 <= x < x1, y0 <= y < y1 (inches).
GOAL_REGION = (4, 22, 4, 22)
# Target 2: a square of this size, chosen per run (right-click in the window,
# or --target2 X,Y for its centre in inches). The robot goes there after
# reaching target 1. If none is chosen by then, it stops and waits.
SECOND_GOAL_SIZE_IN = 12
NAV_SPEED_IN_S = 6.0                  # cruise speed
NAV_TURN_MAX_DEG_S = 90.0
NAV_TURN_GAIN = 2.5                   # turn rate (rad/s) per rad of heading error
NAV_TURN_IN_PLACE_DEG = 50.0          # heading error above this: stop and turn
NAV_LOOKAHEAD_IN = 6.0                # steer toward the path point this far ahead
NAV_EDGE_PENALTY = 2.0                # path cost near corridor edges (keeps it centred)
NAV_SAFE_MARGIN_IN = 1.5              # straight lines must keep the robot body (ROBOT_RADIUS_IN)
                                      # at least this far off every wall and wall corner
# Prefer driving along the maze's axes (north/south/east/west) instead of at
# an angle: a straight leg to the next path point becomes an L of two
# axis-aligned legs. Sideways offsets below NAV_CARDINAL_TOL_IN are ignored
# (no tiny sidesteps); larger ones are corrected with a short square leg.
NAV_CARDINAL = True
NAV_CARDINAL_TOL_IN = 1.5
# Hysteresis on the localizer's confidence
NAV_START_SPREAD_IN = 3.0             # start navigating below this spread...
NAV_START_CONF = 0.7                  # ...with at least this much weight in the cluster
NAV_START_HOLD_S = 1.0                # ...held for this long
NAV_MIN_TRAVEL_IN = 24.0              # ...and only after this much "travel" since (re)starting:
                                      # a fix from one spot can be a look-alike corridor
NAV_TURN_TRAVEL_PER_90DEG_IN = 6.0    # turning on the spot sweeps the sensors across the
                                      # walls, so every 90 deg turned counts as this many inches
NAV_MIN_STRAIGHT_IN = 12.0            # ...but at least this much must be real driving: spinning
                                      # in one spot can't tell two same-shaped spots apart
NAV_RETRY_COOLDOWN_S = 3.0            # after a "no progress" fallback, wander at least this long
NAV_STOP_SPREAD_IN = 6.0              # fall back to localizing above this spread
NAV_STOP_CONF = 0.4                   # ...or below this confidence
NAV_NO_PROGRESS_S = 8.0               # no progress toward goal this long -> re-localize
NAV_ARRIVE_HOLD_S = 0.5               # inside goal this long -> arrived
NAV_ARRIVE_MIN_DRIVEN_IN = 24.0       # ...and only after driving at least this far in total:
                                      # a wrong lock that already sits in the goal would
                                      # otherwise "arrive" instantly; if inside the goal
                                      # earlier, keep wandering to confirm, then come back
NAV_GOAL_MARGIN_IN = 2.0              # must get this far inside the goal's free area
                                      # (estimates are good to ~1-2 in)
# Start of the run: spin this far in place (measured by odometry) before
# driving anywhere, so all sensors sweep the walls first. 0 = no spin.
START_SPIN_DEG = 360.0
START_SPIN_DEG_S = 45.0               # slow enough for ~8 sensor readings per 45 deg at 10 Hz
# Wander along the maze axes (robots that can't arc): after the start spin
# the axis directions are known, so wandering only drives straight along an
# axis, turns in whole 90 deg steps (measured by odometry) and keeps to the
# middle between two walls.
WANDER_ON_AXES = True
WANDER_FRONT_STOP_IN = 3.75           # stop and turn when the front sonar reads this: the
                                      # centre is then about on the crossing corridor's centre
                                      # line (12 in corridors; allows ~0.75 in of stopping delay)
WANDER_CORRIDOR_MAX_IN = 6.0          # both side sonars below this = between two walls
WANDER_CENTER_TOL_IN = 0.75           # off-centre by more than this -> side-step back
WANDER_CENTER_ANGLE_DEG = 20.0        # side-step: turn this far, drive, turn back
WANDER_CENTER_MAX_STEP_IN = 8.0       # longest side-step leg
WANDER_CENTER_EVERY_IN = 4.0          # drive at least this far between side-steps
# Localize-mode wandering (uses raw sonar readings only)
WANDER_SPEED_IN_S = 5.0
WANDER_FRONT_CLEAR_IN = 7.0           # turn when the front reading is below this
WANDER_CENTER_GAIN = 0.15             # rad/s per inch of left/right imbalance
WANDER_SIDE_MIN_IN = 1.0              # steer away from a side wall closer than this
                                      # (side sonar 3 in out: 1 in = centre 4 in from the wall)
WANDER_ESCAPE_S = 0.6                 # (no arcs) after turning away from a wall, drive
                                      # straight this long before reacting to the sides again
# Turn away when a front-angled IR reads less than this AND the sonar on the
# same side reads farther than the IR, i.e. the IR sees a corner / wall ahead
# rather than the wall running alongside (which the side sonar reads closer).
# Keep it above the IR's fold-back point (reliable_min_in: 4 cm = 1.6 in).
WANDER_IR_MIN_IN = 4.5
# Stall recovery: commanded forward but odometry isn't moving (e.g. a corner
# clipped where no sonar points) -> back up, then turn.
STALL_TIME_S = 1.0
STALL_MIN_PROGRESS_IN = 0.3
RECOVER_BACKUP_S = 0.6
RECOVER_TURN_S = 0.6
RECOVER_SPEED_IN_S = 3.0
RECOVER_BACKUP_IF_AHEAD_IN = 4.0      # back up only if front sonar / IR reads below this,
                                      # otherwise just turn (the back is blind at the corners)
# Wall guard: whatever the position estimate says, never drive forward when
# the front sonar reads less than GUARD_FRONT_IN or a front IR less than
# GUARD_IR_IN, and never reverse when the back sonar reads less than
# GUARD_BACK_IN (readings = inches from the sensor face). A held-back drive
# counts as a stall, so the robot backs off and turns toward open space.
GUARD_FRONT_IN = 2.5
GUARD_IR_IN = 2.5
GUARD_BACK_IN = 1.5
