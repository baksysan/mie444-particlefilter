"""Localize a robot in the maze and drive it to target 1, then target 2.

    python mcl.py --port COM5                    # real robot over serial
    python mcl.py --sim                          # fake robot
    python mcl.py --sim --target2 60,30          # preset target 2's centre (inches)
    python mcl.py --list-ports

In the window: R = re-scatter particles (global localization),
left-click = "the robot is about here" (seeds particles around the click),
right-click = place target 2 (a 3 ft square centred on the click; can be
changed at any time), N = autopilot on/off, Esc = quit.
In --sim, W/A/S/D drive manually (overrides the autopilot while held).
"""
import argparse
import math
import time

import numpy as np
import pygame

import config as cfg
import maze
from navigator import Navigator
from particle_filter import ParticleFilter, wrap_angle
from robot_link import SerialRobot, SimulatedRobot, format_command, parse_line

WHITE, BLACK, RED = (255, 255, 255), (0, 0, 0), (220, 40, 40)
BLUE, LIGHT_BLUE = (40, 90, 220), (200, 215, 245)
GREEN, ORANGE, GREY = (0, 170, 0), (255, 140, 0), (90, 90, 90)
GOAL_FILL, PATH = (205, 240, 205), (150, 60, 200)
T2_COLOR, T2_FILL = (230, 120, 0), (255, 170, 60, 70)   # target 2 outline / translucent fill
STATUS_H = 104


class Localizer:
    """Turns cumulative odometry samples into filter steps."""

    def __init__(self, pf):
        self.pf = pf
        self.last = None
        self.samples = 0
        self.fusions = 0
        self.last_ranges = None

    def handle(self, s):
        self.samples += 1
        self.last_ranges = s.ranges_in
        if self.last is not None:
            d = s.odom_dist_in - self.last.odom_dist_in
            dth = wrap_angle(s.odom_heading_rad - self.last.odom_heading_rad)
            if abs(d) <= cfg.MAX_ODOM_JUMP_IN:
                self.pf.predict(d, dth)
            else:
                print(f"Ignoring odometry jump of {d:.1f} in (reset?)")
        self.last = s
        if self.pf.update(s.ranges_in):
            self.fusions += 1


def draw_pose(win, x, y, th, color, size):
    p = (int(x * cfg.PPI), int(y * cfg.PPI))
    pygame.draw.circle(win, color, p, size, 2)
    end = (p[0] + 2 * size * math.cos(th), p[1] + 2 * size * math.sin(th))
    pygame.draw.line(win, color, p, end, 3)


def make_map_surface(grid, goal=None):
    surf = pygame.Surface((grid.shape[1] * cfg.PPI, grid.shape[0] * cfg.PPI))
    colors = {maze.FREE: WHITE, maze.WALL: BLACK, maze.BLOCKED: LIGHT_BLUE}
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            pygame.draw.rect(surf, colors[int(grid[y, x])],
                             (x * cfg.PPI, y * cfg.PPI, cfg.PPI, cfg.PPI))
    if goal is not None:
        x0, x1, y0, y1 = goal
        rect = (x0 * cfg.PPI, y0 * cfg.PPI, (x1 - x0) * cfg.PPI, (y1 - y0) * cfg.PPI)
        pygame.draw.rect(surf, GOAL_FILL, rect)
        pygame.draw.rect(surf, GREEN, rect, 2)
    return surf


def px_rect(r):
    x0, x1, y0, y1 = r
    return pygame.Rect(x0 * cfg.PPI, y0 * cfg.PPI, (x1 - x0) * cfg.PPI, (y1 - y0) * cfg.PPI)


def draw_target2(win, nav, font, active):
    r = px_rect(nav.second_goal)
    fill = pygame.Surface(r.size, pygame.SRCALPHA)
    fill.fill(T2_FILL)
    win.blit(fill, r.topleft)
    pygame.draw.rect(win, T2_COLOR, r, 4 if active else 2)
    win.blit(font.render("2", True, T2_COLOR), (r.x + 6, r.y + 4))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=cfg.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD)
    ap.add_argument("--sim", action="store_true", help="use a simulated robot")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("--target2", metavar="X,Y",
                    help="centre of target 2 in inches (or right-click in the window)")
    args = ap.parse_args()

    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device}: {p.description}")
        return

    grid = maze.init_grid()
    model = maze.RangeModel(grid)
    pf = ParticleFilter(grid, model)
    loc = Localizer(pf)
    link = SimulatedRobot(grid, model) if args.sim else SerialRobot(args.port, args.baud)
    nav = Navigator(grid)
    if args.target2:
        try:
            tx, ty = (float(v) for v in args.target2.split(","))
        except ValueError:
            ap.error("--target2 must be X,Y in inches, e.g. --target2 60,30")
        err = nav.set_second_goal(tx, ty)
        if err:
            ap.error(err)
        print(f"Target 2: {nav.second_goal} (x0, x1, y0, y1 inches)")
    # The simulated robot starts driving straight away; a real robot starts
    # DISARMED and only moves after N is pressed.
    autopilot = args.sim
    STOP = format_command(0.0, 0.0)
    send_period = 1.0 / cfg.CMD_RATE_HZ
    next_send = 0.0
    last_sample_time = None
    cmd = STOP

    pygame.init()
    map_surf = make_map_surface(grid, nav.goal)
    W, H = map_surf.get_size()
    win = pygame.display.set_mode((W, H + STATUS_H))
    pygame.display.set_caption(f"Ultrasonic MCL - {link.name}")
    font = pygame.font.SysFont("consolas", 16)
    big_font = pygame.font.SysFont("consolas", 22, bold=True)
    g1 = px_rect(nav.goals[0].rect)
    map_surf.blit(big_font.render("1", True, GREEN), (g1.x + 6, g1.y + 4))
    notice, notice_until = "", 0.0       # short-lived message for the status bar
    clock = pygame.time.Clock()
    rng = np.random.default_rng()
    rate_t0, rate_n, rate = time.time(), 0, 0.0

    running = True
    while running:
        dt = clock.tick(cfg.FPS) / 1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r:
                pf.scatter()
                nav.reset()
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_n:
                autopilot = not autopilot
                nav.reset()
                if not autopilot:
                    link.send(STOP)   # disarm: stop immediately, don't wait for the timer
                    cmd = STOP
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and ev.pos[1] < H:
                pf.seed_near(ev.pos[0] / cfg.PPI, ev.pos[1] / cfg.PPI)
                nav.reset()
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 3 and ev.pos[1] < H:
                err = nav.set_second_goal(ev.pos[0] / cfg.PPI, ev.pos[1] / cfg.PPI)
                notice = err or f"target 2 set to {nav.second_goal} (x0, x1, y0, y1 in)"
                notice_until = time.time() + 4.0
                print(notice)

        pressed = pygame.key.get_pressed()
        keys = {k: pressed[getattr(pygame, "K_" + k)] for k in "wasd"}
        for line in link.poll(dt, keys):
            s = parse_line(line)
            if s is None:
                if line:
                    print(f"[robot] {line}")
                continue
            loc.handle(s)
            last_sample_time = time.time()
            rate_n += 1

        now = time.time()
        if now - rate_t0 >= 1.0:
            rate, rate_n, rate_t0 = rate_n / (now - rate_t0), 0, now

        est = pf.estimate()
        mx, my, mt, spread, conf = est
        localized = spread < cfg.CONVERGED_SPREAD_IN and conf >= cfg.CONVERGED_MIN_WEIGHT
        stale = last_sample_time is None or now - last_sample_time > cfg.DATA_STALE_S
        if autopilot:
            # The navigator runs every frame (its timers need dt); only the
            # latest command is sent, at CMD_RATE_HZ.
            odom = loc.last.odom_dist_in if loc.last is not None else None
            heading = loc.last.odom_heading_rad if loc.last is not None else None
            v, w = nav.step(est, loc.last_ranges, dt, odom, heading)
            cmd = STOP if stale else format_command(v, w)  # never steer on old data
        if now >= next_send:
            # fixed schedule (not now + period) so frame timing doesn't lower the rate
            next_send = max(next_send + send_period, now - send_period)
            if autopilot:
                link.send(cmd)
            if cfg.SEND_POSE_TO_ROBOT and loc.samples:
                link.send(f"P,{mx:.2f},{my:.2f},{math.degrees(mt):.1f}")

        # ---- draw
        win.blit(map_surf, (0, 0))
        if nav.stage == 0:
            pygame.draw.rect(win, GREEN, g1, 4)   # highlight the current target
        if nav.second_goal is not None:
            draw_target2(win, nav, big_font, active=nav.stage == 1)
        k = min(pf.n, cfg.MAX_PARTICLES_DRAWN)
        for i in rng.choice(pf.n, size=k, replace=False):
            px, py = int(pf.x[i] * cfg.PPI), int(pf.y[i] * cfg.PPI)
            ex = px + 0.8 * cfg.PPI * math.cos(pf.theta[i])
            ey = py + 0.8 * cfg.PPI * math.sin(pf.theta[i])
            pygame.draw.line(win, RED, (px, py), (ex, ey), 1)

        if len(nav.path) > 1:
            pygame.draw.lines(win, PATH, False,
                              [(px * cfg.PPI, py * cfg.PPI) for px, py in nav.path], 3)
            if nav.target:
                pygame.draw.circle(win, PATH, (int(nav.target[0] * cfg.PPI),
                                               int(nav.target[1] * cfg.PPI)), 6)

        if args.sim:
            draw_pose(win, link.x, link.y, link.theta, GREEN, cfg.PPI * 3)

        # measured ranges drawn from the estimated pose
        if loc.last_ranges is not None:
            sx, sy, beam = model.sensor_poses(mx, my, mt)
            for j, r in enumerate(loc.last_ranges):
                if not np.isfinite(r):
                    continue
                a = (sx[j] * cfg.PPI, sy[j] * cfg.PPI)
                b = (a[0] + r * cfg.PPI * math.cos(beam[j]), a[1] + r * cfg.PPI * math.sin(beam[j]))
                pygame.draw.line(win, ORANGE, a, b, 2)
                pygame.draw.circle(win, ORANGE, (int(b[0]), int(b[1])), 4)
        draw_pose(win, mx, my, mt, BLUE if localized else GREY, cfg.PPI * 3)

        # ---- status bar
        pygame.draw.rect(win, (30, 30, 30), (0, H, W, STATUS_H))
        state = "LOCALIZED" if localized else "searching"
        if link.manual:
            drive = "MANUAL"
        elif not autopilot:
            drive = "OFF (press N to arm)"
        elif stale and loc.samples:
            drive = "ARMED - no robot data, sending STOP"
        else:
            drive = f"ARMED {nav.mode} ({nav.target_label()})"
            if nav.wait_reason:
                drive += f"  waiting for: {nav.wait_reason}"
        if nav.second_goal is None and nav.mode != "WAITING":
            drive += "  | right-click to place target 2"
        if time.time() < notice_until:
            drive = notice
        if link.error:
            state = f"LINK ERROR: {link.error}"
        ranges = "-" if loc.last_ranges is None else "  ".join(
            f"{''.join(p[0] for p in n[0].split('_')).upper()}={'--' if not np.isfinite(r) else f'{r:.1f}'}"
            for n, r in zip(cfg.SENSORS, loc.last_ranges))
        lines = [
            f"{link.name} | {rate:4.1f} msg/s | samples {loc.samples} fused {loc.fusions} | {state}",
            f"pose x={mx:5.1f} y={my:5.1f} th={math.degrees(mt):6.1f}deg  spread {spread:4.1f} in  conf {conf:4.0%}",
            f"ranges (in): {ranges}",
            f"autopilot: {drive}",
            "R = re-scatter  click = seed here  right-click = target 2  N = autopilot  Esc = quit"
            + ("  WASD = drive" if args.sim else ""),
        ]
        for i, t in enumerate(lines):
            win.blit(font.render(t, True, WHITE), (8, H + 4 + 19 * i))
        pygame.display.flip()

    # Esc / window closed: always leave the robot stopped
    link.send(STOP)
    link.close()
    pygame.quit()


if __name__ == "__main__":
    main()
