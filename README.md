# Ultrasonic + IR MCL

Localizes a robot in the 96 × 48 in maze with a particle filter, then drives it
to a goal. Over a serial (COM) port the robot streams wheel odometry, 4
ultrasonic ranges (front, right, back, left) and 2 Sharp GP2Y0A21YK IR ranges
(front-left and front-right, at 45°). The PC sends back drive commands.

```
pip install -r requirements.txt
python mcl.py --list-ports
python mcl.py --port COM5          # real robot (autopilot starts OFF, press N to arm)
python mcl.py --sim                # simulated robot (autopilot starts ON)
python mcl.py --port COM5 --target2 70,30   # preset target 2's centre (inches)
```

The first run builds `sensor_table.npz`, a precomputed table of expected sensor
readings (one per beam width). This takes about a minute and is redone automatically if the maze or
sensor settings change.

In the window, R scatters the particles again and left-click tells the filter
"the robot is about here". Red marks are particles, the blue (or grey) circle
is the estimate, and orange lines are the live sensor readings drawn from it.

## Files
| File | Purpose |
|---|---|
| `config.py` | Every setting: port, units, sensor mounting, noise, filter tuning |
| `maze.py` | Maze grid and the precomputed sensor range tables |
| `particle_filter.py` | The filter: odometry predict, sonar/IR update, resampling, estimate |
| `robot_link.py` | Serial protocol parser, COM-port reader thread, simulated robot |
| `navigator.py` | Maze solving: path planning to the goal, path following, modes |
| `mcl.py` | Display and main loop |
| `original_sim/` | The original keyboard-only simulation, kept for reference |

## Serial protocol (robot → PC)
115200 baud by default. Send one ASCII line per sample, ending in `\n`, at
about 10–20 Hz:

```
R,<odom_dist>,<odom_heading_deg>,<front>,<right>,<back>,<left>,<front_left>,<front_right>
```

| Field | Meaning |
|---|---|
| `odom_dist` | **Cumulative** signed distance since boot, forward positive, e.g. `(left_enc + right_enc) / 2` converted to cm |
| `odom_heading_deg` | **Cumulative** heading change since boot from the encoders, `(right - left) / wheelbase`, **counter-clockwise positive** (left turn = +) |
| `front` … `left` | Ultrasonic ranges in cm. Send the max-range value (or more) when there's no echo. Send `0` if a sensor failed and its reading must be ignored |
| `front_left`, `front_right` | Sharp IR ranges in cm, from your voltage-to-distance conversion, sent as-is. Send `80` or more for nothing in range, and `0` for a failed sensor |

Example: `R,153.20,-87.5,24.1,8.3,120.0,7.9,14.2,31.0`

- Cumulative values (rather than per-message deltas) mean a dropped or garbled
  line loses no motion.
- Lines that don't start with `R,` are printed to the console, so debug prints
  from the firmware are fine.
- Fire the sonars **one at a time** (not simultaneously) to avoid crosstalk.
- With `SEND_POSE_TO_ROBOT = True` the PC sends `P,<x_in>,<y_in>,<theta_deg>`
  back to the robot. Coordinates are in inches from the maze's top-left corner,
  +y points down the screen, and theta is clockwise from +x.

## Things to set in `config.py` for your robot
1. `SENSORS`: each sensor's mounting angle, its forward/right offset (in
   inches) from the robot's rotation centre (the midpoint between the drive
   wheels), and its type. The order must match the message. **Measure the IR
   offsets:** the defaults are a guess (3 in out along the 45° line).
2. `SERIAL_DISTANCE_UNITS` and `HEADING_CCW_POSITIVE` if your firmware differs.
3. `SENSOR_TYPES`: beam width, range limits and noise for each sensor type.
   Defaults: sonar 15° cone and 2–120 cm; IR 5° beam and 10–80 cm.
4. `WANDER_IR_MIN_IN`: how close an IR reading can get before the robot turns
   away while wandering. It depends on the IR mounting; see the comment in
   `config.py`.
5. The odometry noise `ODOM_*`, and the sensor noise (`sigma_*`) in
   `SENSOR_TYPES`. Start with the defaults. If the filter loses track, raise
   them. If the particle cloud stays large once the robot is clearly
   localized, lower them.

### How the IR sensors are used
- **Localization:** they're used like the sonars, but with a narrow beam and
  noise that grows with distance.
- **The 10 cm problem:** the GP2Y0A21YK's output folds back below 10 cm, so a
  wall at 5 cm reads like about 20 cm. The filter handles this on both sides:
  - Measured IR values under 10 cm are ignored.
  - Wherever a particle would put the IR closer than 10 cm, that reading counts
    as no evidence for or against it. A folded reading therefore can't push the
    filter toward the wrong place.

  In a 12 in corridor this matters: a centred robot's IRs read about 14 cm, and
  1 in off-centre is already about 10 cm.
- **Corner avoidance:** while wandering, the robot turns away from a side when
  that side's IR reads under `WANDER_IR_MIN_IN` **and** the same side's sonar
  reads farther than the IR.
  - A wall running alongside always reads closer on the side sonar, so it is
    ignored.
  - Corners ahead and walls approached at an angle make the IR the closer
    reading. The straight-ahead and sideways sonars can't see those.
  - During path following the robot relies on the map instead.

## Maze solving
The robot drives to two targets in order:
1. **Target 1**, the green zone `GOAL_REGION` in `config.py`, labelled "1".
2. **Target 2**, a 3 ft × 3 ft square (`SECOND_GOAL_SIZE_IN = 36`) that you
   choose for each run, drawn as the orange "2". Choose it in either of two
   ways:
   - **Right-click** anywhere on the map. The square is centred on the click
     and shifted to stay inside the maze. Right-click again at any time to
     move it, and the robot re-routes.
   - **`--target2 X,Y`** on the command line, giving the centre in inches
     (x 0–98 left to right, y 0–50 top to bottom).

The current target has the thicker outline, and the status bar shows
`target 1/2` or `target 2/2`. If target 2 hasn't been chosen when the robot
reaches target 1, the robot stops in **WAITING** mode
(`reached target 1 - right-click to choose target 2`). It sets off as soon as
you right-click.

The square only counts where it covers drivable corridor: the robot has
arrived once its estimate is at least 2 in inside any corridor part of the
square. If the square overlaps the green zone, target 2 can be reached almost
immediately after target 1.

The planned path is drawn in purple. The robot works through these modes:

1. **LOCALIZE**: while the filter is unsure, the robot wanders using only its
   raw sensor readings. It keeps to the middle of corridors, turns toward open
   space when blocked, and turns away from corners the IRs see.
2. **NAVIGATE**: it follows the path to the goal once all of these hold:
   - The estimate is tight: spread < 3 in and more than 70% of the weight in
     one cluster, held for 1 s.
   - It has at least 24 in of "travel" (`NAV_MIN_TRAVEL_IN`). Turning on the
     spot counts too, at 6 in per 90° (`NAV_TURN_TRAVEL_PER_90DEG_IN`), because
     it sweeps the sensors across the walls.
   - At least 12 in of that is real driving (`NAV_MIN_STRAIGHT_IN`). Spinning
     in one spot can't tell two same-shaped spots apart.

   The extra conditions exist because the filter can report 100% confidence in
   a look-alike corridor within a second or two.

   The path is planned once over the whole maze and favours the middle of
   corridors, so the robot can recompute its route from wherever the estimate
   says it is.
3. **Arriving at a target**: this happens once the estimate has been at least
   2 in inside the target (`NAV_GOAL_MARGIN_IN`) for 0.5 s, **and** the robot
   has driven at least 24 in in total (`NAV_ARRIVE_MIN_DRIVEN_IN`).
   - At target 1, the robot switches to target 2, or goes to **WAITING** if
     target 2 isn't chosen yet.
   - At target 2, it stops for good (**ARRIVED**).

   About the 24 in rule:
   - If the estimate is already in the goal before that, the robot keeps
     wandering to confirm, then comes back.
   - Otherwise a wrong lock that happens to sit inside the goal would "arrive"
     instantly. A wrong lock anywhere else gets caught while driving.

**Why isn't it moving on?** The status bar's `autopilot:` line shows what it's
waiting for, for example `waiting for: travel 11/24 in, driven 7/12 in`,
`spread 3.4>3 in`, `conf 55%<70%`, `holding 0.4/1 s`, `retry in 2.1 s` or
`confirming arrival: driven 15/24 in`. Note that "LOCALIZED" in the top line
uses looser thresholds (spread < 4 in, conf ≥ 60%) than navigating does.

### Robots that can't turn while driving (`ROBOT_CAN_ARC = False`, the default)
Every command is either a pure drive `C,v,0` or a pure turn on the spot
`C,0,w`, never both:
- **Navigating:** the robot turns on the spot until it faces the target
  (heading error under 3°), then drives straight. It stops to re-aim when the
  error grows past 8°.
- **Choosing the target:** it is the farthest path point within 10 in that the
  robot can reach in a straight line while staying about 1 in inside the
  corridor. This keeps it from cutting corners.
- **Clear-ahead check:** before driving, it checks that the next 3 in straight
  ahead is open corridor.
- **Wandering** works the same way:
  - Drive straight, and turn on the spot when the front is blocked.
  - If a side sonar reads under 1 in (`WANDER_SIDE_MIN_IN`), or an IR sees a
    corner, turn away briefly.
  - Then drive straight for 0.6 s (`WANDER_ESCAPE_S`) before reacting to the
    sides again. Turning on the spot alone doesn't move the robot away from
    the wall, and without this it could spin forever.
- **Safeguard:** as a final check, any command with both a speed and a turn
  rate has its speed set to 0.

Set `ROBOT_CAN_ARC = True` if the robot can drive curves, and it will steer
while moving instead. The simulated robot obeys the same limitation, so
`--sim` shows how the real robot will behave.

It goes back to LOCALIZE if confidence drops, or if it makes no progress
toward the goal for 8 s, which usually means it was following a wrong
estimate. If the robot is told to drive forward but its odometry doesn't
change for 1 s, it backs up and turns. That covers clipping a corner that no
sonar points at.

In the window, N arms and disarms the autopilot. With a real robot it starts
**disarmed**, so put the robot in the maze before pressing N. In `--sim`,
holding W/A/S/D overrides the autopilot.

### Drive commands (PC → robot)
```
C,<forward speed>,<turn rate deg/s>        e.g.  C,15.24,-45.0
```
- **Units and sign:** speed is in `SERIAL_DISTANCE_UNITS` per second (cm/s by
  default). Turn rate uses the same sign as the odometry heading
  (counter-clockwise positive).
- **Rate:** sent at `CMD_RATE_HZ` (20 Hz), only while armed.
- **When the PC sends `C,0.00,0.0` (stop):**
  - immediately on disarming (N)
  - when you press Esc or close the window
  - continuously whenever no `R,` line has arrived for `DATA_STALE_S`
    (1 s), so it never steers on old data

### What the robot's firmware must do
1. Read `C,` lines and store the latest speed `v` and turn rate `ω`
   (in rad/s, counter-clockwise positive).
2. Convert them to wheel speeds, with `b` = distance between the wheels:
   left = v − ω·b/2, right = v + ω·b/2. With `ROBOT_CAN_ARC = False`, one of
   `v` or `ω` is always 0, so this is either both wheels at the same speed or
   the wheels spinning in opposite directions.
3. Hold those wheel speeds with a PID loop on the encoders.
4. **Stop if no `C,` line has arrived for about 0.5 s** (PC crash, unplugged
   cable).
5. **Refuse to drive forward when the front sonar reads under about 2 in.**
6. Keep streaming `R,` lines the whole time.

The simulated robot follows these same rules.

## Notes
- The maze has look-alike spots. From some starting positions the filter needs
  some driving before it can tell them apart, and it may briefly lock onto the
  wrong one. Random particles are injected automatically when the readings stop
  fitting, which lets it recover. You can also click on the robot's location.
- Sonar readings are only fused after the robot has moved ≥ 0.5 in or turned
  ≥ 5°. Repeated readings from a robot standing still aren't new information.
