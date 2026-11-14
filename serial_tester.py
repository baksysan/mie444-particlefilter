"""Bench tester for the robot's serial link: check the R lines it sends and the
C (and G / L) lines it receives, without needing the maze.

    python serial_tester.py --port COM5
    python serial_tester.py --port COM5 --speed 4 --turn 45 --log bench.csv
    python serial_tester.py --sim                 # try it without the robot

Put the robot on blocks (wheels off the ground) or on the floor with room to
move, and keep a hand near Space / Esc.

Keys
  W / S        hold: drive forward / backward at --speed
  A / D        hold: turn left (counter-clockwise) / right at --turn
  Space        stop and cancel any running test
  G / L        gripper: Grip / Leave behind
  Z            zero the odometry counters shown on screen
  1            test: drive forward  12 in   (time based: speed x time)
  2            test: drive backward 12 in
  3            test: turn left  90 deg
  4            test: turn right 90 deg
  5            test: watchdog - stop sending commands, robot must stop itself
  Esc          stop and quit

The protocol, units and sensor list come from config.py, exactly as mcl.py
uses them.
"""
import argparse
import csv
import math
import time
from collections import deque

import numpy as np
import pygame

import config as cfg
from robot_link import (GRIP_CLOSE, GRIP_OPEN, UNIT_TO_IN, SerialRobot, format_command,
                        parse_line)

UNITS = cfg.SERIAL_DISTANCE_UNITS
K = UNIT_TO_IN[UNITS]                         # firmware units -> inches
ALL_SENSORS = list(cfg.SENSORS) + [cfg.TOF_SENSOR]
N_FIELDS = 3 + len(ALL_SENSORS)               # "R", odom, heading, ranges..., tof

BG, FG, DIM = (22, 24, 28), (230, 230, 230), (140, 140, 150)
OK_C, WARN_C, BAD_C, HEAD_C = (90, 200, 120), (240, 190, 70), (240, 90, 80), (120, 170, 255)


def fmt_units(v_in):
    """inches -> firmware units string"""
    return f"{v_in / K:.2f} {UNITS}"


class SensorStats:
    def __init__(self, name, kind):
        t = cfg.SENSOR_TYPES[kind]
        self.name, self.kind = name, kind
        self.min_in, self.max_in, self.rel_min = t["min_in"], t["max_in"], t["reliable_min_in"]
        self.value = math.nan
        self.seen_min, self.seen_max = math.inf, -math.inf
        self.last_change = time.time()
        self.invalid = 0

    def update(self, v_in):
        now = time.time()
        if not (np.isfinite(v_in) and np.isfinite(self.value) and abs(v_in - self.value) < 1e-9):
            self.last_change = now
        self.value = v_in
        if np.isfinite(v_in):
            self.seen_min, self.seen_max = min(self.seen_min, v_in), max(self.seen_max, v_in)
        else:
            self.invalid += 1

    def status(self):
        v = self.value
        if not np.isfinite(v):
            return "INVALID (0)", WARN_C
        if v >= self.max_in - 1e-6:
            return "nothing in range", DIM
        if v < self.rel_min:
            return "too close to trust", WARN_C
        if time.time() - self.last_change > 3.0:
            return f"unchanged {time.time() - self.last_change:.0f}s", WARN_C
        return "ok", OK_C


class Test:
    """Timed open-loop move, then compare with what the odometry reports."""

    def __init__(self, name, v_in_s, w_ccw_deg_s, duration, expect_dist_in, expect_turn_deg):
        self.name, self.v, self.w_ccw = name, v_in_s, w_ccw_deg_s
        self.duration = duration
        self.expect_dist, self.expect_turn = expect_dist_in, expect_turn_deg
        self.t0 = None
        self.start = None
        self.phase = "wait_sample"

    def command(self):
        # format_command takes w clockwise-positive (screen); w_ccw is counter-clockwise
        return format_command(self.v, -math.radians(self.w_ccw))


class Tester:
    def __init__(self, link, speed, turn_deg, log_path):
        self.link = link
        self.speed, self.turn = speed, turn_deg
        self.sensors = [SensorStats(s[0], s[4]) for s in ALL_SENSORS]
        self.raw_line = ""
        self.last_parsed = None             # (t, odom_in, heading_fw_deg)
        self.zero = None                    # odometry reference for the display
        self.arrivals = deque(maxlen=60)
        self.bad_lines = 0
        self.last_bad = ""
        self.robot_logs = deque(maxlen=4)
        self.results = deque(maxlen=9)
        self.cmd = format_command(0, 0)
        self.manual_cmd = self.cmd
        self.test = None
        self.watchdog = None
        self.sending = True
        self.next_send = 0.0
        self.history = deque(maxlen=400)    # (t, odom_in, heading_fw_deg)
        self.log = None
        if log_path:
            f = open(log_path, "w", newline="")
            self.log = (f, csv.writer(f))
            self.log[1].writerow(["time_s", "direction", "line"])
        self.t_start = time.time()

    # ------------------------------------------------------------- receiving
    def handle_line(self, line, now):
        if self.log:
            self.log[1].writerow([f"{now - self.t_start:.3f}", "rx", line])
        if not line.startswith("R"):
            if line:
                self.robot_logs.append(line)
            return
        self.raw_line = line
        parts = line.strip().split(",")
        if len(parts) != N_FIELDS:
            self.bad_lines += 1
            self.last_bad = f"expected {N_FIELDS} fields, got {len(parts)}: {line[:60]}"
            return
        s = parse_line(line)
        if s is None:
            self.bad_lines += 1
            self.last_bad = f"not numbers: {line[:60]}"
            return
        heading_fw = float(parts[2])        # degrees, exactly as the firmware sent it
        for st, v in zip(self.sensors, list(s.ranges_in) + [s.tof_in]):
            st.update(v)
        self.arrivals.append(now)
        self.last_parsed = (now, s.odom_dist_in, heading_fw)
        self.history.append(self.last_parsed)
        if self.zero is None:
            self.zero = (s.odom_dist_in, heading_fw)

    def rate(self):
        a = self.arrivals
        if len(a) < 2 or time.time() - a[-1] > 1.0:
            return 0.0, 0.0
        gaps = np.diff(np.array(a))
        return 1.0 / gaps.mean(), gaps.max() * 1000

    # --------------------------------------------------------------- sending
    def send(self, text, now):
        self.link.send(text)
        if self.log:
            self.log[1].writerow([f"{now - self.t_start:.3f}", "tx", text])

    def result(self, text, color=FG):
        self.results.append((text, color))
        print(text)

    def stop_all(self, now, why=None):
        self.test = None
        self.watchdog = None
        self.sending = True
        self.cmd = format_command(0, 0)
        self.send(self.cmd, now)
        if why:
            self.result(why, WARN_C)

    # ----------------------------------------------------------------- tests
    def start_test(self, n):
        if self.last_parsed is None:
            self.result("No R lines yet - can't run a test", BAD_C)
            return
        v, t = self.speed, self.turn
        dist, ang = 12.0, 90.0
        tests = {
            1: Test("forward 12 in", v, 0, dist / v, dist, 0),
            2: Test("backward 12 in", -v, 0, dist / v, -dist, 0),
            3: Test("turn left 90 deg", 0, t, ang / t, 0, ang),
            4: Test("turn right 90 deg", 0, -t, ang / t, 0, -ang),
        }
        self.watchdog = None
        self.test = tests[n]
        self.result(f"TEST {self.test.name}: {self.test.command()} for {self.test.duration:.1f} s", HEAD_C)

    def start_watchdog(self, now):
        if self.last_parsed is None:
            self.result("No R lines yet - can't run a test", BAD_C)
            return
        self.test = None
        self.watchdog = {"t0": now, "phase": "drive"}
        self.result(f"TEST watchdog: drive {self.speed:g} in/s for 1 s, then send NOTHING for 2.5 s", HEAD_C)

    def step_test(self, now):
        t = self.test
        if t is None or self.last_parsed is None:
            return
        _, odom, head = self.last_parsed
        if t.phase == "wait_sample":
            t.start, t.t0, t.phase = (odom, head), now, "run"
        if t.phase == "run":
            self.cmd = t.command()
            if now - t.t0 >= t.duration:
                t.phase, t.t_stop = "settle", now
        if t.phase == "settle":
            self.cmd = format_command(0, 0)
            if now - t.t_stop >= 0.8:              # let it coast to a stop, then read odometry
                d = odom - t.start[0]
                h = head - t.start[1]
                # what a CCW-positive heading should do for this test
                exp_h = t.expect_turn if cfg.HEADING_CCW_POSITIVE else -t.expect_turn
                self.report(t, d, h, exp_h)
                self.test = None

    def report(self, t, d, h, exp_h):
        self.result(f"  odometry: distance {d:+.2f} in ({fmt_units(d)}), heading {h:+.1f} deg", FG)
        if t.expect_dist:
            sign_ok = math.copysign(1, d) == math.copysign(1, t.expect_dist) and abs(d) > 0.5
            ratio = d / t.expect_dist
            ok = sign_ok and 0.85 <= ratio <= 1.15
            self.result(f"  {'PASS' if ok else 'CHECK'}: expected {t.expect_dist:+.1f} in "
                        f"({ratio:.0%} of it). Measure the robot: did it really move "
                        f"{abs(t.expect_dist):.0f} in? Heading drift {h:+.1f} deg.",
                        OK_C if ok else BAD_C)
            if not sign_ok:
                self.result("  -> wrong sign or no motion: odometry must count forward as +", BAD_C)
        else:
            sign_ok = math.copysign(1, h) == math.copysign(1, exp_h) and abs(h) > 5
            ratio = h / exp_h
            ok = sign_ok and 0.85 <= ratio <= 1.15
            side = "LEFT (counter-clockwise)" if t.w_ccw > 0 else "RIGHT (clockwise)"
            self.result(f"  {'PASS' if ok else 'CHECK'}: expected heading {exp_h:+.0f} deg "
                        f"({ratio:.0%} of it). Look: did it turn ~90 deg to the {side}? "
                        f"Distance drift {d:+.2f} in.", OK_C if ok else BAD_C)
            if not sign_ok:
                conv = "counter-clockwise" if cfg.HEADING_CCW_POSITIVE else "clockwise"
                self.result(f"  -> wrong sign or no motion: heading must increase turning {conv} "
                            f"(HEADING_CCW_POSITIVE = {cfg.HEADING_CCW_POSITIVE})", BAD_C)

    def step_watchdog(self, now):
        w = self.watchdog
        if w is None or self.last_parsed is None:
            return
        el = now - w["t0"]
        if w["phase"] == "drive":
            self.cmd = format_command(self.speed, 0)
            if el >= 1.0:
                w["phase"], w["t_quiet"] = "quiet", now
                self.sending = False               # stop sending anything at all
        elif w["phase"] == "quiet" and now - w["t_quiet"] >= 2.5:
            self.sending = True
            self.cmd = format_command(0, 0)
            self.analyse_watchdog(w["t_quiet"], now)
            self.watchdog = None

    def analyse_watchdog(self, t_quiet, t_end):
        pts = [(t, o) for t, o, _ in self.history if t >= t_quiet - 0.1]
        if len(pts) < 4:
            self.result("  CHECK: too few R lines during the test to judge", BAD_C)
            return
        before = [o for t, o, _ in self.history if t_quiet - 0.5 <= t <= t_quiet]
        if len(before) < 2 or abs(before[-1] - before[0]) < 0.5:
            self.result("  INCONCLUSIVE: the robot wasn't moving when commands stopped "
                        "(blocked? wheels not turning? odometry not counting?)", WARN_C)
            return
        # time of the last real odometry change after commands stopped
        moving_until = t_quiet
        for (t1, o1), (t2, o2) in zip(pts, pts[1:]):
            if abs(o2 - o1) > 0.02:
                moving_until = t2
        stopped_after = moving_until - t_quiet
        coasted = pts[-1][1] - pts[0][1]
        if t_end - moving_until < 0.4:
            self.result(f"  FAIL: still moving {t_end - t_quiet:.1f} s after the last command - "
                        f"the firmware needs a ~0.5 s command watchdog!", BAD_C)
        elif stopped_after <= cfg.SIM_CMD_TIMEOUT_S + 0.5:
            self.result(f"  PASS: stopped {stopped_after:.2f} s after the last command "
                        f"(moved {coasted:.2f} in meanwhile)", OK_C)
        else:
            self.result(f"  CHECK: took {stopped_after:.2f} s to stop (expected ~0.5 s); "
                        f"moved {coasted:.2f} in", WARN_C)

    # ------------------------------------------------------------------ loop
    def manual(self, pressed):
        v = self.speed * (pressed[pygame.K_w] - pressed[pygame.K_s])
        w_ccw = self.turn * (pressed[pygame.K_a] - pressed[pygame.K_d])
        if not cfg.ROBOT_CAN_ARC and v and w_ccw:
            v = 0.0                                # same rule as the autopilot: turn only
        return format_command(v, -math.radians(w_ccw))

    def tick(self, now, pressed):
        if self.test is None and self.watchdog is None:
            self.cmd = self.manual(pressed)
        self.step_test(now)
        self.step_watchdog(now)
        if self.sending and now >= self.next_send:
            self.next_send = now + 1.0 / cfg.CMD_RATE_HZ
            self.send(self.cmd, now)


def draw(win, font, big, tst, link):
    win.fill(BG)
    W = win.get_width()
    y = 10

    def text(s, x, y, c=FG, f=None):
        win.blit((f or font).render(s, True, c), (x, y))

    rate, worst = tst.rate()
    conn = "LINK ERROR: " + link.error if link.error else link.name
    text(f"Serial tester  |  {conn}", 12, y, BAD_C if link.error else HEAD_C, big)
    y += 30
    rc = OK_C if rate >= 8 else (WARN_C if rate > 0 else BAD_C)
    text(f"R lines: {rate:5.1f} /s  (worst gap {worst:4.0f} ms)   bad lines: {tst.bad_lines}"
         f"   units: {UNITS}   expected fields: {N_FIELDS}", 12, y, rc)
    y += 20
    if tst.last_bad:
        text(f"last bad line: {tst.last_bad}", 12, y, BAD_C)
        y += 20
    text(f"raw: {tst.raw_line[:110]}", 12, y, DIM)
    y += 28

    # odometry
    text("ODOMETRY", 12, y, HEAD_C)
    y += 20
    if tst.last_parsed:
        _, odom, head = tst.last_parsed
        z = tst.zero or (odom, head)
        text(f"distance  {odom:9.2f} in  ({fmt_units(odom)})    since Z: {odom - z[0]:+8.2f} in", 24, y)
        y += 20
        conv = "CCW +" if cfg.HEADING_CCW_POSITIVE else "CW +"
        text(f"heading   {head:9.1f} deg (as sent, {conv})       since Z: {head - z[1]:+8.1f} deg", 24, y)
    else:
        text("no R lines received yet", 24, y, BAD_C)
        y += 20
    y += 28

    # sensors
    text("SENSORS (inches)        value     status                seen min..max    invalid", 12, y, HEAD_C)
    y += 20
    for st in tst.sensors:
        label, c = st.status()
        v = "--" if not np.isfinite(st.value) else f"{st.value:6.2f}"
        mm = "" if st.seen_max < 0 else f"{st.seen_min:5.1f}..{st.seen_max:5.1f}"
        text(f"{st.name:12s} {st.kind:5s} {v:>8s}     {label:20s}  {mm:14s}  {st.invalid:5d}", 24, y, c)
        y += 19
    y += 14

    # command
    text("SENDING", 12, y, HEAD_C)
    y += 20
    if not tst.sending:
        text("(nothing - watchdog test: robot must stop by itself)", 24, y, WARN_C)
    else:
        busy = " [test running]" if (tst.test or tst.watchdog) else ""
        text(f"{tst.cmd} at {cfg.CMD_RATE_HZ} Hz{busy}", 24, y, FG)
    y += 28

    text("RESULTS", 12, y, HEAD_C)
    y += 20
    for s, c in tst.results:
        text(s[:118], 24, y, c)
        y += 18
    if tst.robot_logs:
        y += 6
        text("robot says: " + " | ".join(tst.robot_logs)[:110], 12, y, DIM)

    help_ = ("W/S drive  A/D turn  Space stop  G grip  L leave  Z zero  "
             "tests: 1 fwd  2 back  3 left  4 right  5 watchdog  Esc quit")
    text(help_, 12, win.get_height() - 24, DIM)
    pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=cfg.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD)
    ap.add_argument("--sim", action="store_true", help="test against the simulated robot")
    ap.add_argument("--speed", type=float, default=4.0, help="drive speed for keys/tests, in/s")
    ap.add_argument("--turn", type=float, default=45.0, help="turn rate for keys/tests, deg/s")
    ap.add_argument("--log", help="write every line sent/received to this CSV file")
    args = ap.parse_args()

    if args.sim:
        import maze
        from robot_link import SimulatedRobot
        grid = maze.init_grid()
        link = SimulatedRobot(grid, maze.RangeModel(grid))
        # start in a long straight corridor (facing up the left-hand one), so
        # the drive tests have room, like a robot on the bench
        link.x, link.y, link.theta = 7.0, 44.0, -math.pi / 2
    else:
        link = SerialRobot(args.port, args.baud)

    pygame.init()
    win = pygame.display.set_mode((1000, 720))
    pygame.display.set_caption("Robot serial tester")
    font = pygame.font.SysFont("consolas", 15)
    big = pygame.font.SysFont("consolas", 20, bold=True)
    clock = pygame.time.Clock()
    tst = Tester(link, args.speed, args.turn, args.log)
    print(f"Expecting R lines with {N_FIELDS} fields, distances in {UNITS}. Sending C at {cfg.CMD_RATE_HZ} Hz.")

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        now = time.time()
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_SPACE:
                    tst.stop_all(now, "stopped")
                elif ev.key == pygame.K_g:
                    tst.send(GRIP_CLOSE, now)
                    tst.result(f"sent {GRIP_CLOSE} (grip)")
                elif ev.key == pygame.K_l:
                    tst.send(GRIP_OPEN, now)
                    tst.result(f"sent {GRIP_OPEN} (leave behind)")
                elif ev.key == pygame.K_z and tst.last_parsed:
                    tst.zero = tst.last_parsed[1:]
                elif ev.key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4):
                    tst.start_test(ev.key - pygame.K_0)
                elif ev.key == pygame.K_5:
                    tst.start_watchdog(now)
        for line in link.poll(dt, {}):              # sim ignores keys here: it obeys C lines
            tst.handle_line(line, time.time())
        tst.tick(now, pygame.key.get_pressed())
        draw(win, font, big, tst, link)

    tst.stop_all(time.time())
    link.close()
    if tst.log:
        tst.log[0].close()
    pygame.quit()


if __name__ == "__main__":
    main()
