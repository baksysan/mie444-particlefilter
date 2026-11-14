# Ultrasonic + IR MCL

Localizes a robot in the 96 × 48 in maze with a particle filter. It then
drives to the green zone (target 1), finds a 1 in block there with a rear
VL53L0X time-of-flight sensor, grips it with the rear gripper, and delivers it
to target 2, a square you choose for each run.

Over a serial (COM) port the robot streams:
- wheel odometry
- 4 ultrasonic ranges (front, right, back, left)
- 2 Sharp GP2Y0A21YK IR ranges (front-left and front-right, at 45°)
- the ToF range

The PC sends back drive and gripper commands.

```
pip install -r requirements.txt
python mcl.py --list-ports
python mcl.py --port COM5          # real robot (autopilot starts OFF, press N to arm)
python mcl.py --sim                # simulated robot (autopilot starts ON)
python mcl.py --port COM5 --target2 70,30   # preset target 2's centre (inches)
python serial_tester.py --port COM5         # bench-test the serial link, no maze needed
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
| `navigator.py` | The mission: path planning, path following, modes, delivering the block |
| `block_task.py` | Finding the block with the ToF, backing onto it, gripping it |
| `mcl.py` | Display and main loop |
| `serial_tester.py` | Bench tester for the `R` / `C` / `G` / `L` serial lines (no maze needed) |
| `original_sim/` | The original keyboard-only simulation, kept for reference |

## Serial protocol (robot → PC)
115200 baud by default. Send one ASCII line per sample, ending in `\n`, at
about 10–20 Hz:

```
R,<odom_dist>,<odom_heading_deg>,<front>,<right>,<back>,<left>,<front_left>,<front_right>,<tof>
```

| Field | Meaning |
|---|---|
| `odom_dist` | **Cumulative** signed distance since boot in **inches**, forward positive, e.g. `(left_enc + right_enc) / 2` converted to inches |
| `odom_heading_deg` | **Cumulative** heading change since boot from the encoders, `(right - left) / wheelbase`, **counter-clockwise positive** (left turn = +) |
| `front` … `left` | Ultrasonic ranges in inches. Send `47.2` (120 cm) or more when there's no echo. Send `0` if a sensor failed and its reading must be ignored |
| `front_left`, `front_right` | Sharp IR ranges in inches, from your voltage-to-distance conversion, sent as-is. Send `31.5` (80 cm) or more for nothing in range, and `0` for a failed sensor |
| `tof` | Rear VL53L0X range in inches (the one under the back sonar; its mm output ÷ 25.4). Send `39.4` (100 cm) or more for nothing in range, and `0` for a failed or invalid reading |

Example: `R,60.31,-87.5,9.49,3.27,47.24,3.11,5.59,12.20,8.86`

All distances are in inches (`SERIAL_DISTANCE_UNITS = "in"` in `config.py`;
`"cm"` and `"mm"` are also supported). Decimals are fine.

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
6. **Block pickup, measure these:**
   - `TOF_SENSOR`: the ToF's offset from the rotation centre. The default is
     3 in straight back.
   - `REAR_EDGE_IN`: distance from the rotation centre to the back edge of
     the chassis, where a gripped block touches (the gripper works in a slot
     there). Default 3 in. If the ToF face is set back from that edge, set
     `TOF_SENSOR`'s offset to where the ToF really is; the difference is
     handled automatically.
   - If pickups miss, tune `BLOCK_MIN_GAP_IN` (how much shorter than the wall
     a reading must be to count as the block) and `BLOCK_REVERSE_SPEED_IN_S`.

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

## The mission
1. **Drive to target 1**, the green zone `GOAL_REGION` in `config.py`,
   labelled "1". The robot stops about 6 in **before** entering it
   (`BLOCK_SEARCH_STANDOFF_IN`). Its ToF faces backwards, so driving in blind
   could shove a block sitting near an entrance.
2. **Pick up the block** (`block_task.py`). The status bar shows each step:
   - **Sweep:** turn slowly on the spot so the rear ToF sweeps 360°. A reading
     at least `BLOCK_MIN_GAP_IN` (2 in) shorter than the map's wall distance is
     the block. Two or more readings in a row give its direction, and the
     shortest one its distance. If nothing is seen, it drives to other spots
     in the zone (centre, then quarter points) and sweeps again.
   - **Approach:** choose a side with room to back up. Drive (forwards, around
     the block) to a staging point 6 in out, then turn the back toward the
     block.
   - **Close-up sweep:** ±30° to pin down the block's position, then line the
     gripper up on it.
   - **Back up** slowly (2 in/s), guided by the ToF. At contact the block is
     closer than the VL53L0X can measure (about 3 cm). So once a reading is
     at or below `TOF_CLOSE_IN` (about 4 cm), or the readings drop out after
     being close, the robot uses that last good reading as the remaining gap.
     It backs up exactly that far by odometry, plus `GRIP_CONTACT_EXTRA_IN`
     (0.2 in), so the block ends up touching the back edge in the slot.
   - **Grip:** send `G` and wait `GRIP_TIME_S`. Then confirm: with the block
     against the edge, the ToF reads either nothing (firmware sends `0`) or
     less than `GRIP_CONFIRM_MAX_IN` (2 in). If it sees something farther
     away, the grip missed: the robot sends `L` and retries.
   - Any failed step (not seen, backed up too far, no block at the edge after
     gripping, timeout) starts a fresh sweep. After 3 failures (`BLOCK_MAX_ATTEMPTS`)
     the robot stops. Press R to try again.
3. **Carry it to target 2**, a `SECOND_GOAL_SIZE_IN` square (currently 12 in)
   that you choose for each run, drawn as the orange "2". The block hangs
   behind the robot, so the robot picks a drop spot near the square's centre
   and a robot position about 5 in from it. It drives there, turns the
   gripper toward the drop spot, sends `L` (leave behind), and pulls 3 in
   forward.
4. **Done**: the status bar shows `ARRIVED (done)`.

**Choosing target 2:**
- **Right-click** anywhere on the map. The square is centred on the click and
  shifted to stay inside the maze. Right-click again at any time to move it;
  while the robot is carrying the block, it re-plans.
- **`--target2 X,Y`** on the command line, giving the centre in inches (x 0–98
  left to right, y 0–50 top to bottom).

If target 2 isn't chosen by the time the block is gripped, the robot stops in
**WAITING** mode (`holding the block - right-click to choose target 2`). It
sets off as soon as you right-click.

**In the window:**
- The brown square (sim only) is the real block.
- The purple box is where the robot thinks the block is, and the purple circle
  is the staging point.
- The cyan line is the live ToF reading.
- The status bar shows the ToF reading and whether the gripper is holding.

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
3. **Arriving at target 1**: this happens once the estimate has been at least
   2 in inside the arrival area (`NAV_GOAL_MARGIN_IN`) for 0.5 s, **and** the
   robot has driven at least 24 in in total (`NAV_ARRIVE_MIN_DRIVEN_IN`). The
   arrival area ends 6 in before the green zone. Then the block pickup starts.

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

### Fixed-speed mode (`FIXED_SPEED_MODE`, off by default)
Normally the autopilot asks for a range of speeds. For example, turns slow
down as the robot nears the heading it wants, and it backs onto the block at
2 then 1 in/s. With `FIXED_SPEED_MODE = True` every command uses one of two
fixed magnitudes instead (the direction is kept):

| Level | Speed | Turn rate | Used for |
|---|---|---|---|
| normal | `FIXED_SPEED_IN_S` = 4 in/s | `FIXED_TURN_DEG_S` = 60°/s | Driving along paths, wandering, sweeps, big turns |
| precise | `FIXED_PRECISE_SPEED_IN_S` = 1.5 in/s | `FIXED_PRECISE_TURN_DEG_S` = 20°/s | The last ~14° of lining up a heading, the close-up block sweep, backing onto the block |

A request at or below `FIXED_PRECISE_BELOW_SPEED_IN_S` (2.5 in/s), or below
`FIXED_PRECISE_BELOW_TURN_DEG_S` (35°/s), gets the precise level; everything
else gets the normal level.

This keeps the firmware's speed control simple (two setpoints) and makes
odometry more repeatable. In simulation (15 full missions) all blocks were
delivered with no pickup retries. The median mission took 74 s, against 65 s
with variable speeds.

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

### Commands (PC → robot)
```
C,<forward speed in/s>,<turn rate deg/s>   e.g.  C,6.00,-45.0
G                                          grip: close the rear gripper
L                                          leave behind: open the rear gripper
```
Gripper commands are sent once, immediately, only while armed. Disarming or
quitting does **not** open the gripper, so a held block stays held.
- **Units and sign:** speed is in `SERIAL_DISTANCE_UNITS` per second (in/s
  with the current setting), negative for reversing. Turn rate is in deg/s
  and uses the same sign as the odometry heading (counter-clockwise positive).
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
   Don't apply this to reversing: the robot backs onto the block on purpose.
6. On `G` close the gripper, and on `L` open it. The PC waits
   `GRIP_TIME_S` (1 s) before moving on, so the gripper must finish within
   that time.
7. Keep streaming `R,` lines the whole time, including the ToF field.

The simulated robot follows these same rules.

## Bench-testing the firmware (`serial_tester.py`)
Check the serial link before going into the maze. Put the robot on blocks, or
on the floor with about 2 ft of clear space, and keep a hand near Space / Esc.

```
python serial_tester.py --port COM5
python serial_tester.py --port COM5 --speed 4 --turn 45 --log bench.csv
python serial_tester.py --sim          # try the tool without the robot
```

It uses the same parser, units and sensor list as `mcl.py`, so what it
accepts, the localizer accepts.

**`R` lines (robot → PC):**
- The message rate and the worst gap between lines (aim for 10–20/s).
- The raw text of the last line.
- Bad lines, with the reason, e.g. `expected 10 fields, got 9`.
- Odometry in inches and the heading exactly as sent, plus how much each has
  changed since you pressed Z.
- Every sensor, with its value, status, min/max seen and invalid count. The
  status is one of: ok, nothing in range, too close to trust, INVALID (0), or
  unchanged for a while (a stuck sensor or a value that's never updated).
- Lines that don't start with `R` are shown under "robot says".

**`C` lines (PC → robot):** hold W/S to drive at `--speed` and A/D to turn at
`--turn`. The exact line being sent is shown. G and L send the gripper
commands.

**Scripted tests:** each prints PASS, CHECK, FAIL or INCONCLUSIVE, with what
to fix.

| Key | Test | What it checks |
|---|---|---|
| 1 / 2 | Drive forward / backward for 12 in (speed × time) | Odometry counts the right way and the right amount. Measure the robot too: if both disagree with 12 in, the speed control is off. If only the odometry disagrees, fix the wheel size or encoder counts |
| 3 / 4 | Turn left / right 90° | The heading sign (left turn = + with `HEADING_CCW_POSITIVE = True`) and its scale. Watch that the robot really turned 90° the right way |
| 5 | Watchdog | Drives 1 s, then sends **nothing** for 2.5 s. The robot must stop by itself within about 0.5 s. FAIL means the firmware has no command timeout |

`--log file.csv` records every line sent and received, with timestamps, for
debugging.

## Notes
- The maze has look-alike spots. From some starting positions the filter needs
  some driving before it can tell them apart, and it may briefly lock onto the
  wrong one. Random particles are injected automatically when the readings stop
  fitting, which lets it recover. You can also click on the robot's location.
- Sonar readings are only fused after the robot has moved ≥ 0.5 in or turned
  ≥ 5°. Repeated readings from a robot standing still aren't new information.
