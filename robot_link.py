"""Communication with the robot.

Serial protocol (robot -> PC), one ASCII line per sample, '\\n' terminated:

    R,<odom_dist>,<odom_heading_deg>,<front>,<right>,<back>,<left>,<front_left>,<front_right>

  odom_dist         CUMULATIVE signed distance driven since boot (forward +),
                    in SERIAL_DISTANCE_UNITS. e.g. (left_enc + right_enc) / 2.
  odom_heading_deg  CUMULATIVE heading change since boot from the encoders,
                    counter-clockwise positive (see HEADING_CCW_POSITIVE).
                    Do not wrap it; wrapping is fine too.
  front..left       ultrasonic ranges, then the two 45-degree Sharp IR ranges,
                    in SERIAL_DISTANCE_UNITS, in the order of config.SENSORS.
                    Send the max-range value (or more) when nothing is in
                    range, and 0 or a negative number if the sensor failed and
                    the reading must be ignored. For the IR, send the distance
                    from your voltage-to-distance conversion as-is (no need to
                    handle the < 10 cm fold-back in firmware).

Any line not starting with "R," is printed as a robot log message.
Optional PC -> robot (SEND_POSE_TO_ROBOT): "P,<x_in>,<y_in>,<theta_deg>\\n".
"""
import math
import queue
import threading
from dataclasses import dataclass

import numpy as np

import config as cfg
import maze

UNIT_TO_IN = {"cm": 1 / 2.54, "mm": 1 / 25.4, "in": 1.0}


@dataclass
class Sample:
    odom_dist_in: float       # cumulative, inches
    odom_heading_rad: float   # cumulative, radians, CLOCKWISE positive (screen)
    ranges_in: np.ndarray     # inches, NaN = ignore


def parse_line(line):
    """Returns a Sample, or None if the line is not a valid data line."""
    parts = line.strip().split(",")
    n = len(cfg.SENSORS)
    if len(parts) != 3 + n or parts[0] != "R":
        return None
    try:
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    k = UNIT_TO_IN[cfg.SERIAL_DISTANCE_UNITS]
    heading = math.radians(vals[1])
    if cfg.HEADING_CCW_POSITIVE:
        heading = -heading
    ranges = np.array(vals[2:], dtype=np.float64) * k
    ranges[ranges <= 0] = np.nan
    return Sample(vals[0] * k, heading, ranges)


def format_command(v_in_s, w_rad_s):
    """PC -> robot drive command. v: in/s forward; w: rad/s CLOCKWISE (screen).
    Sent as C,<speed in SERIAL_DISTANCE_UNITS/s>,<turn deg/s, odometry sign>."""
    k = UNIT_TO_IN[cfg.SERIAL_DISTANCE_UNITS]
    w_deg = math.degrees(w_rad_s)
    if cfg.HEADING_CCW_POSITIVE:
        w_deg = -w_deg
    return f"C,{v_in_s / k + 0.0:.2f},{w_deg + 0.0:.1f}"   # + 0.0 avoids "-0.0"


def parse_command(line):
    """Inverse of format_command: returns (v in/s, w rad/s clockwise) or None."""
    parts = line.strip().split(",")
    if len(parts) != 3 or parts[0] != "C":
        return None
    try:
        v, w_deg = float(parts[1]), float(parts[2])
    except ValueError:
        return None
    if cfg.HEADING_CCW_POSITIVE:
        w_deg = -w_deg
    return v * UNIT_TO_IN[cfg.SERIAL_DISTANCE_UNITS], math.radians(w_deg)


class SerialRobot:
    """Reads lines from a COM port on a background thread."""

    def __init__(self, port, baud):
        import serial  # pyserial
        # serial_for_url accepts "COM5" as well as test URLs like "loop://"
        self.serial = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
        self.name = f"{port} @ {baud}"
        self.manual = False   # no keyboard driving for the real robot
        self.lines = queue.Queue()
        self.error = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.serial.read(self.serial.in_waiting or 1)
            except Exception as e:  # unplugged, etc.
                self.error = str(e)
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.lines.put(line.decode("ascii", errors="ignore").strip())

    def poll(self, dt=None, keys=None):
        out = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                return out

    def send(self, text):
        try:
            self.serial.write((text + "\n").encode("ascii"))
        except Exception as e:
            self.error = str(e)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        try:
            self.serial.flush()  # make sure a final stop command goes out
        except Exception:
            pass
        self.serial.close()


class SimulatedRobot:
    """A fake robot in the same maze. It emits the same text lines a real robot
    would, so the whole pipeline gets exercised. It drives from "C," commands
    sent to it (autopilot), or from W/A/S/D, which override the commands.
    Like the real firmware should, it stops when commands stop arriving and
    refuses to drive forward into an obstacle its front sonar can see."""

    def __init__(self, grid, range_model, rng=None):
        self.grid = grid
        self.model = range_model
        self.rng = rng if rng is not None else np.random.default_rng()
        self.name = "simulated robot"
        self.error = None
        cells = maze.free_cells(grid)
        c = cells[self.rng.integers(len(cells))]
        self.x, self.y = c[0] + 0.5, c[1] + 0.5
        self.theta = self.rng.uniform(-math.pi, math.pi)
        self.odom_dist = 0.0
        self.odom_heading_ccw = 0.0
        self.t_accum = 0.0
        self.scale_err = 1.0 + self.rng.normal(0, cfg.SIM_ODOM_SCALE_ERR)
        self.cmd = (0.0, 0.0)
        self.cmd_age = math.inf
        self.bumps = 0          # times it was blocked by a wall (for testing)
        self.arc_violations = 0  # combined drive+turn commands received (for testing)
        self.sim_noise = maze.sensor_param("sim_noise_in")
        self.sim_noise_prop = maze.sensor_param("sim_noise_prop")
        self.sim_outlier = maze.sensor_param("sim_outlier")
        self.sim_folds = maze.sensor_param("sim_folds").astype(bool)
        self.manual = False

    def drive(self, v, w, dt):
        """v: in/s forward, w: rad/s clockwise."""
        d = v * dt
        dth = w * dt
        nx = self.x + d * math.cos(self.theta + dth / 2)
        ny = self.y + d * math.sin(self.theta + dth / 2)
        if maze.is_free(self.grid, np.array([nx]), np.array([ny]))[0]:
            self.x, self.y = nx, ny
        else:
            d = 0.0  # bumped into a wall: wheels stall, encoders don't count
            self.bumps += 1
        self.theta = (self.theta + dth + math.pi) % (2 * math.pi) - math.pi
        self.odom_dist += d * self.scale_err + self.rng.normal(0, 0.01 * abs(d) + 1e-4)
        self.odom_heading_ccw -= math.degrees(dth) * self.scale_err

    def make_line(self):
        k = UNIT_TO_IN[cfg.SERIAL_DISTANCE_UNITS]
        m = self.model
        d = m.true_distance(np.array([self.x]), np.array([self.y]),
                            np.array([self.theta]))[0].astype(np.float64)
        r = d.copy()
        # Sharp IR fold-back: closer than its minimum, the output voltage drops
        # again and the reading looks FARTHER away (e.g. 5 cm reads ~25 cm)
        rmin = m.reliable_min_in
        fold = self.sim_folds & (d < rmin)
        r[fold] = rmin[fold] + 3.0 * (rmin[fold] - d[fold])
        r += self.rng.normal(0, 1, r.shape) * (self.sim_noise + self.sim_noise_prop * r)
        outlier = self.rng.random(r.shape) < self.sim_outlier
        r[outlier] = self.rng.uniform(m.min_in[outlier], m.max_in[outlier])
        r = np.clip(r, m.min_in, m.max_in)
        heading = self.odom_heading_ccw if cfg.HEADING_CCW_POSITIVE else -self.odom_heading_ccw
        fields = [self.odom_dist / k, heading] + list(r / k)
        return "R," + ",".join(f"{f:.2f}" for f in fields)

    def poll(self, dt, keys):
        self.cmd_age += dt
        self.manual = any(keys.get(k, 0) for k in "wasd")
        if self.manual:
            v = cfg.SIM_SPEED_IN_S * ((keys.get("w", 0)) - (keys.get("s", 0)))
            w = math.radians(cfg.SIM_TURN_DEG_S) * ((keys.get("d", 0)) - (keys.get("a", 0)))
        elif self.cmd_age <= cfg.SIM_CMD_TIMEOUT_S:
            v, w = self.cmd
        else:
            v, w = 0.0, 0.0  # watchdog: no fresh command -> stop
        if not cfg.ROBOT_CAN_ARC and v != 0 and w != 0:
            # like the real robot: can't drive and turn at once -> turn only
            if not self.manual:
                self.arc_violations += 1
            v = 0.0
        if v > 0:
            front = self.model.expected(np.array([self.x]), np.array([self.y]),
                                        np.array([self.theta]))[0][0]
            if front < cfg.SIM_SAFETY_STOP_IN:
                v = 0.0
        self.drive(v, w, dt)
        self.t_accum += dt
        out = []
        period = 1.0 / cfg.SIM_RATE_HZ
        if self.t_accum >= period:
            self.t_accum %= period
            out.append(self.make_line())
        return out

    def send(self, text):
        cmd = parse_command(text)
        if cmd is not None:
            self.cmd, self.cmd_age = cmd, 0.0

    def close(self):
        pass
