"""Maze solving on top of the localizer: plan on the known map, follow the plan.

Mission (self.phase):
  TARGET1   drive to target 1 (GOAL_REGION, the green zone)
  BLOCK     find the 1 in block there with the rear ToF and grip it (block_task.py)
  TARGET2   carry it to target 2 (a SECOND_GOAL_SIZE_IN square the user picks per run)
  RELEASE   open the gripper (L, "leave behind") and pull forward
  DONE
Each target has its own cost-to-goal field.

Modes (how it is driving right now)
  LOCALIZE  filter not confident yet: wander using raw sensor readings only
  NAVIGATE  localized: carry out the current phase
  WAITING   holding the block but no target 2 chosen yet; stopped
  ARRIVED   mission complete; stopped
"""
import heapq
import math

import numpy as np

import config as cfg
import maze
from block_task import BlockTask
from particle_filter import wrap_angle
from robot_link import GRIP_OPEN

LOCALIZE, NAVIGATE, WAITING, ARRIVED = "LOCALIZE", "NAVIGATE", "WAITING", "ARRIVED"
TARGET1, BLOCK, TARGET2, RELEASE, DONE = "TARGET1", "BLOCK", "TARGET2", "RELEASE", "DONE"
NEIGHBOURS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def clearance_map(grid):
    """Chebyshev distance (cells) from each FREE cell to the nearest non-free cell."""
    return _distance_from(grid != maze.FREE)


def wall_distance_map(grid):
    """Euclidean distance (inches) from each cell's centre to the nearest WALL."""
    wy, wx = np.nonzero(grid == maze.WALL)
    h, w = grid.shape
    cy, cx = np.mgrid[0:h, 0:w]
    px, py = cx.ravel() + 0.5, cy.ravel() + 0.5
    best = np.full(px.shape, np.inf)
    for i in range(0, len(wx), 256):                       # chunks keep memory small
        bx, by = wx[i:i + 256, None].astype(float), wy[i:i + 256, None].astype(float)
        dx = np.maximum(np.maximum(bx - px, px - (bx + 1)), 0.0)
        dy = np.maximum(np.maximum(by - py, py - (by + 1)), 0.0)
        best = np.minimum(best, np.sqrt(dx * dx + dy * dy).min(axis=0))
    return best.reshape(h, w)


def safe_map(grid):
    """Cells where the robot's centre keeps its whole body (ROBOT_RADIUS_IN)
    at least NAV_SAFE_MARGIN_IN off every wall. Straight lines, the staging
    point for the block and the drop pose must stay on these cells."""
    return (grid == maze.FREE) & (wall_distance_map(grid) >= cfg.ROBOT_RADIUS_IN + cfg.NAV_SAFE_MARGIN_IN)


def goal_mask(grid, goal, margin_in):
    """FREE cells of the goal rectangle at least margin_in from its edge (falls
    back to the whole free part of the rectangle if it is too small)."""
    x0, x1, y0, y1 = goal
    m = np.zeros(grid.shape, dtype=bool)
    m[y0:y1, x0:x1] = grid[y0:y1, x0:x1] == maze.FREE
    if not m.any():
        raise ValueError(f"GOAL_REGION {goal} contains no free cells")
    inner = m & (_distance_from(~m) >= margin_in + 0.5)
    return inner if inner.any() else m


def _distance_from(blocked):
    """Chebyshev distance (cells) from every cell to the nearest blocked cell."""
    h, w = blocked.shape
    clear = np.full((h, w), np.inf)
    frontier = list(zip(*np.nonzero(blocked)))
    for y, x in frontier:
        clear[y, x] = 0
    d = 0
    while frontier:
        d += 1
        nxt = []
        for y, x in frontier:
            for dx, dy in NEIGHBOURS:
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and clear[ny, nx] == np.inf:
                    clear[ny, nx] = d
                    nxt.append((ny, nx))
        frontier = nxt
    return clear


def cost_to_goal(grid, goal_cells, avoid=None):
    """Dijkstra from the goal cells over FREE cells (8-connected). Cells near
    corridor edges cost more, so paths run down the middle of corridors.
    avoid = (x, y, r): treat cells within r inches of (x, y) as blocked (e.g.
    the loose block, so the robot doesn't drive over it)."""
    h, w = grid.shape
    if avoid is not None:
        ax, ay, ar = avoid
        yy, xx = np.mgrid[0:h, 0:w]
        near = (xx + 0.5 - ax) ** 2 + (yy + 0.5 - ay) ** 2 < ar ** 2
        grid = np.where(near & ~goal_cells, maze.BLOCKED, grid)
    clear = clearance_map(grid)
    dist = np.full((h, w), np.inf)
    heap = []
    for y, x in zip(*np.nonzero(goal_cells)):
        dist[y, x] = 0.0
        heap.append((0.0, y, x))
    heapq.heapify(heap)
    while heap:
        d, y, x = heapq.heappop(heap)
        if d > dist[y, x]:
            continue
        for dx, dy in NEIGHBOURS:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < h and 0 <= nx < w) or grid[ny, nx] != maze.FREE:
                continue
            step = math.hypot(dx, dy) * (1 + cfg.NAV_EDGE_PENALTY / clear[ny, nx] ** 2)
            if d + step < dist[ny, nx]:
                dist[ny, nx] = d + step
                heapq.heappush(heap, (d + step, ny, nx))
    return dist


def fixed_speed(v, w):
    """FIXED_SPEED_MODE: replace the requested speed / turn rate by one of two
    fixed magnitudes, keeping the direction. Slow requests (fine alignment,
    backing onto the block) map to the precise level, everything else to the
    normal level. v in/s, w rad/s; zero stays zero."""
    if v != 0.0:
        mag = (cfg.FIXED_PRECISE_SPEED_IN_S if abs(v) <= cfg.FIXED_PRECISE_BELOW_SPEED_IN_S
               else cfg.FIXED_SPEED_IN_S)
        v = math.copysign(mag, v)
    if w != 0.0:
        slow = abs(math.degrees(w)) < cfg.FIXED_PRECISE_BELOW_TURN_DEG_S
        mag = math.radians(cfg.FIXED_PRECISE_TURN_DEG_S if slow else cfg.FIXED_TURN_DEG_S)
        w = math.copysign(mag, w)
    return v, w


def square_around(grid, cx, cy, size):
    """Integer rectangle (x0, x1, y0, y1) of size x size inches centred near
    (cx, cy), shifted so it stays inside the maze's outer walls."""
    h, w = grid.shape
    x0 = int(round(cx - size / 2))
    y0 = int(round(cy - size / 2))
    x0 = min(max(x0, 1), max(1, w - 1 - size))
    y0 = min(max(y0, 1), max(1, h - 1 - size))
    return (x0, min(x0 + size, w - 1), y0, min(y0 + size, h - 1))


class Goal:
    """A target rectangle with its arrival cells and cost-to-goal field."""

    def __init__(self, grid, rect, avoid=None):
        self.rect = tuple(int(v) for v in rect)
        self.cells = goal_mask(grid, self.rect, cfg.NAV_GOAL_MARGIN_IN)
        self.dist = cost_to_goal(grid, self.cells, avoid)


def plan_drop(grid, rect, x, y, safe=None):
    """Where to put the block in target 2. The block hangs `reach` inches
    behind the robot centre, so choose a drop spot D inside the square (as
    central as possible) and a robot position P = D + reach * (direction phi)
    that is drivable. Returns (P, phi, D) or None. The robot drives to P,
    turns to heading phi (back toward D) and opens the gripper."""
    reach = cfg.GRIP_REACH_IN
    if safe is None:
        safe = safe_map(grid)
    x0, x1, y0, y1 = rect
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    m = cfg.BLOCK_SIZE_IN                 # keep the whole block inside the square
    best = None
    for dy in np.arange(y0 + m, y1 - m + 1e-9, 1.0):
        for dx in np.arange(x0 + m, x1 - m + 1e-9, 1.0):
            if grid[int(dy), int(dx)] == maze.WALL:
                continue
            for k in range(24):
                phi = k * math.pi / 12
                px, py = dx + reach * math.cos(phi), dy + reach * math.sin(phi)
                if not (0 <= py < grid.shape[0] and 0 <= px < grid.shape[1]) or not safe[int(py), int(px)]:
                    continue
                # prefer a central drop spot, then a short drive
                cost = 3.0 * math.hypot(dx - cx, dy - cy) + math.hypot(px - x, py - y) / 10
                if best is None or cost < best[0]:
                    best = (cost, (px, py), phi, (float(dx), float(dy)))
    return None if best is None else best[1:]


class Navigator:
    def __init__(self, grid, tof_model=None):
        """tof_model: RangeModel for config.TOF_SENSOR. Without it the block
        step is skipped (target 1 -> target 2 directly)."""
        self.grid = grid
        if tof_model is not None:
            # block mission: "reach" target 1 a little before entering it, so
            # the first ToF sweep happens before driving over the zone blind
            s = cfg.BLOCK_SEARCH_STANDOFF_IN
            x0, x1, y0, y1 = cfg.GOAL_REGION
            h, w = grid.shape
            g1 = Goal(grid, (max(x0 - s, 0), min(x1 + s, w), max(y0 - s, 0), min(y1 + s, h)))
            g1.rect = tuple(cfg.GOAL_REGION)    # drawn / searched as the real zone
        else:
            g1 = Goal(grid, cfg.GOAL_REGION)
        self.goals = [g1, None]                            # target 2 is set per run
        self.stage = 0                                     # index of the current target
        self._activate(0)
        self.phase = TARGET1
        self.block = BlockTask(self, grid, tof_model) if tof_model is not None else None
        self.holding = False
        self.outbox = []           # commands for the robot (gripper), sent by mcl.py
        self.release_time = 0.0
        self.release_clear = True  # room to pull away after letting go of the block
        self.drop = None           # (robot pos, heading, drop spot) for target 2
        self.drop_state = "goto"
        self.drop_timer = 0.0
        # Straight-line moves must stay ~1 in inside the corridor, so a small
        # estimate error doesn't clip an inside corner.
        self.wall_dist = wall_distance_map(grid)
        self.safe = (grid == maze.FREE) & (self.wall_dist >= cfg.ROBOT_RADIUS_IN + cfg.NAV_SAFE_MARGIN_IN)
        self.free = maze.free_cells(grid)
        self.mode = LOCALIZE
        self.good_time = 0.0
        self.arrive_time = 0.0
        self.best_dist = math.inf
        self.no_progress_time = 0.0
        self.wander_dir = 1
        self.path = []
        self.target = None
        self.last_v = 0.0
        self.stall_ref = None
        self.stall_time = 0.0
        self.recover_time = 0.0
        self.stalls = 0
        self.guarded = None        # why the wall guard last held back a drive (or None)
        self.aligning = False      # no-arc mode: currently turning in place
        self.turn_goal = None      # no-arc mode: odometry heading the current turn ends at
        self.start_spin_done = cfg.START_SPIN_DEG <= 0   # 360 deg spin at the start of the run
        self.spin_turned, self.spin_last = 0.0, None
        self.spin_samples = []     # (odometry heading, sensor readings) during the start spin
        self.axis = None           # odometry heading of a maze axis, mod 90 deg
        self.w_first = None        # first wander heading, picked from the start spin
        self.w_heading = None      # wander: current axis heading (odometry frame)
        self.w_state, self.w_next, self.w_goal = "drive", "drive", None
        self.w_timer, self.w_dist, self.w_start, self.w_last_fix = 0.0, 0.0, None, None
        self.settle_time = 0.0     # no-arc mode: hold still this long after a turn
        self._dt = 0.0
        self.side_turn_time = 0.0  # no-arc wander: timed turn away from a side wall
        self.escape_time = 0.0     # no-arc wander: drive straight after that turn
        self.travel = 0.0          # inches driven (+ turning credit) since (re)starting localization
        self.straight = 0.0        # inches actually driven since (re)starting localization
        self.last_odom = None
        self.last_heading = None
        self.cooldown = 0.0        # seconds left before NAVIGATE may be retried
        self.wait_reason = ""      # why it is not navigating (shown in the status bar)
        self.sensor_idx = maze.sensor_index()
        self.max_in = maze.sensor_param("max_in")

    # --------------------------------------------------------------- targets
    def _activate(self, stage):
        self.stage = stage
        self._use_goal(self.goals[stage])
        self.drop = None                  # (re)plan where to put the block

    def _use_goal(self, g):
        self.avoid_zone = None
        self.goal, self.goal_cells, self.dist = g.rect, g.cells, g.dist
        self.best_dist, self.no_progress_time, self.arrive_time = math.inf, 0.0, 0.0
        self.aligning, self.turn_goal, self.settle_time = False, None, 0.0

    def set_point_goal(self, x, y, avoid=None):
        """Drive to the 1 in cell containing (x, y) (used by the block task).
        avoid = (x, y, r): keep the path's centre line away from that spot."""
        cx, cy = int(math.floor(x)), int(math.floor(y))
        self._use_goal(Goal(self.grid, (cx, cx + 1, cy, cy + 1), avoid))
        self.avoid_zone = avoid

    def follow_current(self, x, y, theta):
        return self._follow(x, y, theta)

    def on_block_gripped(self):
        """Called by the block task once the block is in the gripper."""
        self.holding = True
        self.phase = TARGET2
        if self.goals[1] is None:
            self.mode = WAITING
            self._use_goal(self.goals[0])
        else:
            self._activate(1)

    def set_second_goal(self, cx, cy):
        """Place target 2: a SECOND_GOAL_SIZE_IN square centred near (cx, cy)
        inches. Returns an error message, or None on success. Can be called
        any time; if the robot is already heading there (or waiting for it),
        it re-routes to the new square."""
        rect = square_around(self.grid, cx, cy, cfg.SECOND_GOAL_SIZE_IN)
        try:
            self.goals[1] = Goal(self.grid, rect)
        except ValueError:
            return f"target 2 at {rect} has no drivable corridor inside it"
        if self.phase == TARGET2:
            self._activate(1)
            if self.mode == WAITING:
                self.mode = NAVIGATE
        return None

    @property
    def second_goal(self):
        return self.goals[1].rect if self.goals[1] is not None else None

    def target_label(self):
        return {TARGET1: "-> target 1", BLOCK: "picking up block",
                TARGET2: "-> target 2 with block", RELEASE: "releasing block",
                DONE: "done"}[self.phase]

    def reset(self):
        """Restart localization (keeps which phase it is in). A block task
        that gave up is started again."""
        if self.phase == BLOCK and self.block is not None and self.block.state == "failed":
            self.block.attempts = 0
            self.block._enter("sweep")
        self.mode = LOCALIZE
        self.good_time = self.arrive_time = self.no_progress_time = 0.0
        self.best_dist = math.inf
        self.aligning, self.turn_goal, self.settle_time = False, None, 0.0
        self.w_heading, self.w_state = None, "drive"     # wander starts afresh on an axis
        self.side_turn_time = 0.0
        self.escape_time = 0.0
        self.travel = 0.0
        self.straight = 0.0
        self.cooldown = 0.0

    # -------------------------------------------------------------- planning
    def _cell(self, x, y):
        cx, cy = int(math.floor(x)), int(math.floor(y))
        h, w = self.grid.shape
        if 0 <= cy < h and 0 <= cx < w and np.isfinite(self.dist[cy, cx]):
            return cx, cy
        # estimate sits just outside the corridor: use the nearest free cell
        i = np.argmin((self.free[:, 0] + 0.5 - x) ** 2 + (self.free[:, 1] + 0.5 - y) ** 2)
        return int(self.free[i, 0]), int(self.free[i, 1])

    def path_from(self, x, y):
        """Steepest descent on the cost field: list of cell centres to the goal."""
        cx, cy = self._cell(x, y)
        h, w = self.grid.shape
        pts = [(cx + 0.5, cy + 0.5)]
        while self.dist[cy, cx] > 0 and len(pts) < 1000:
            best = None
            for dx, dy in NEIGHBOURS:
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h and 0 <= nx < w and self.dist[ny, nx] < self.dist[cy, cx]:
                    if best is None or self.dist[ny, nx] < self.dist[best[1], best[0]]:
                        best = (nx, ny)
            if best is None:
                break
            cx, cy = best
            pts.append((cx + 0.5, cy + 0.5))
        return pts

    def in_goal(self, x, y):
        """True when (x, y) is inside the goal, at least NAV_GOAL_MARGIN_IN in."""
        cx, cy = int(math.floor(x)), int(math.floor(y))
        h, w = self.grid.shape
        return 0 <= cy < h and 0 <= cx < w and bool(self.goal_cells[cy, cx])

    def _line_clear(self, x0, y0, x1, y1):
        """Can the robot drive straight from (x0,y0) to (x1,y1) while staying
        ~1 in inside the corridor? If the robot starts closer to the edge than
        that, the leading stretch is ignored (it can't help where it is); once
        the line enters the safe area it must stay inside."""
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(2, int(length / 0.25) + 1)
        t = np.linspace(0.0, 1.0, n)
        xi = np.floor(x0 + t * (x1 - x0)).astype(int)
        yi = np.floor(y0 + t * (y1 - y0)).astype(int)
        h, w = self.grid.shape
        inside = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)
        free = np.zeros(n, dtype=bool)
        free[inside] = self.safe[yi[inside], xi[inside]]
        if self.avoid_zone is not None:
            ax, ay, ar = self.avoid_zone
            px, py = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            free &= (px - ax) ** 2 + (py - ay) ** 2 >= ar ** 2
        if not free.any():
            return False
        first = int(np.argmax(free))
        return bool(free[first:].all())

    def _body_clear(self, x, y, theta, dist):
        """Would the robot body (ROBOT_RADIUS_IN + 0.5 in) stay off the walls
        driving `dist` inches straight from (x, y) along theta (negative = back)?"""
        h, w = self.grid.shape
        for t in np.linspace(0.0, 1.0, max(2, int(abs(dist) / 0.25) + 1)):
            px, py = x + t * dist * math.cos(theta), y + t * dist * math.sin(theta)
            cx, cy = int(math.floor(px)), int(math.floor(py))
            if not (0 <= cy < h and 0 <= cx < w):
                return False
            if self.wall_dist[cy, cx] < cfg.ROBOT_RADIUS_IN + 0.5:
                return False
        return True

    def _straight_target(self, x, y):
        """Farthest path point within NAV_STRAIGHT_LOOKAHEAD_IN that can be
        reached in a straight line, so straight driving never cuts a corner.
        If none is, head for the centre of the current cell first."""
        if len(self.path) <= 2 and math.hypot(self.path[-1][0] - x, self.path[-1][1] - y) < 2.0:
            return self.path[-1]          # destination is right here: go straight at it
        best = self.path[0]
        travelled, px, py = 0.0, x, y
        for qx, qy in self.path[1:]:
            travelled += math.hypot(qx - px, qy - py)
            px, py = qx, qy
            if travelled > cfg.NAV_STRAIGHT_LOOKAHEAD_IN or not self._line_clear(x, y, qx, qy):
                break
            best = (qx, qy)
        if cfg.NAV_CARDINAL:
            return self._cardinal_leg(x, y, *best)
        return best

    def _cardinal_leg(self, x, y, tx, ty):
        """Turn a straight leg to (tx, ty) into axis-aligned driving.
        - Sideways offset within NAV_CARDINAL_TOL_IN: drive purely along the
          main axis (the small offset is simply kept).
        - Otherwise an L: the main axis first, then the other one (or the other
          way round if that corner isn't clear). Replanning every step then
          takes the second leg by itself.
        - Falls back to the diagonal when neither L is clear."""
        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < 1.0:
            return tx, ty
        x_main = abs(dx) >= abs(dy)
        minor = abs(dy) if x_main else abs(dx)
        if minor <= cfg.NAV_CARDINAL_TOL_IN:
            return (tx, y) if x_main else (x, ty)
        first, second = ((tx, y), (x, ty)) if x_main else ((x, ty), (tx, y))
        for cx, cy in (first, second):
            if self._line_clear(x, y, cx, cy) and self._line_clear(cx, cy, tx, ty):
                return cx, cy
        return tx, ty

    # ------------------------------------------------------------- behaviour
    def _reading(self, ranges, name):
        """Named reading in inches; missing sensor/reading -> its max range."""
        i = self.sensor_idx.get(name)
        if i is None or ranges is None or not np.isfinite(ranges[i]):
            return self.max_in[i] if i is not None else math.inf
        return float(ranges[i])

    def _corner_close(self, ranges):
        """-1 if a 45-degree IR on the LEFT sees something close, +1 if on the
        right, 0 if neither (or no IR sensors)."""
        fl = self._reading(ranges, "front_left")
        fr = self._reading(ranges, "front_right")
        # a wall running alongside reads closer on the side sonar than on the
        # 45-degree IR; only an obstacle ahead (corner, wall approached at an
        # angle) makes the IR the closer of the two
        left_hit = fl < cfg.WANDER_IR_MIN_IN and self._reading(ranges, "left") > fl
        right_hit = fr < cfg.WANDER_IR_MIN_IN and self._reading(ranges, "right") > fr
        if left_hit and (not right_hit or fl <= fr):
            return -1
        if right_hit:
            return 1
        return 0

    def _wander(self, ranges, dt):
        front = self._reading(ranges, "front")
        right = self._reading(ranges, "right")
        left = self._reading(ranges, "left")
        corner = self._corner_close(ranges)
        w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
        if not cfg.ROBOT_CAN_ARC:
            if cfg.WANDER_ON_AXES and self.axis is not None and self.last_heading is not None:
                return self._wander_axes(front, right, left, corner, dt)
            return self._wander_no_arc(front, right, left, corner, dt)
        if corner and front > cfg.WANDER_FRONT_CLEAR_IN:
            # wall corner ahead-left/right that the sonars can't see: veer away
            self.wander_dir = -corner
            return 0.5 * cfg.WANDER_SPEED_IN_S, self.wander_dir * 0.7 * w_max
        if front > cfg.WANDER_FRONT_CLEAR_IN:
            # drive on, nudging toward the middle of the corridor
            w = 0.0
            if left < 12 and right < 12:
                w = float(np.clip(cfg.WANDER_CENTER_GAIN * (right - left), -0.5 * w_max, 0.5 * w_max))
            elif left < cfg.WANDER_SIDE_MIN_IN:
                w = 0.5 * w_max     # too close on the left: veer right
            elif right < cfg.WANDER_SIDE_MIN_IN:
                w = -0.5 * w_max
            self.wander_dir = 1 if right >= left else -1
            return cfg.WANDER_SPEED_IN_S, w
        # blocked: turn in place toward the more open side (latched until clear)
        return 0.0, self.wander_dir * 0.7 * w_max

    # ---------------------------------------------------- wander along the axes
    def _snap(self, h):
        """Odometry heading -> the nearest maze axis direction (odometry frame)."""
        q = math.pi / 2
        return self.axis + round((h - self.axis) / q) * q

    def _w_turn_to(self, goal, then):
        self.w_goal, self.w_next, self.w_state = goal, then, "turn"
        return 0.0, 0.0

    def _side_gap(self, left, right):
        """How far the robot's centre is right of the corridor centre line
        (inches, + = right of centre), from the two side sonars; None if
        the robot isn't between two walls."""
        if left > cfg.WANDER_CORRIDOR_MAX_IN or right > cfg.WANDER_CORRIDOR_MAX_IN:
            return None
        lo = abs(dict((s[0], s[3]) for s in cfg.SENSORS).get("left", 0.0))
        ro = abs(dict((s[0], s[3]) for s in cfg.SENSORS).get("right", 0.0))
        # distance from the centre to each wall; centred when they're equal
        return ((left + lo) - (right + ro)) / 2

    def _wander_axes(self, front, right, left, corner, dt):
        """Wander for a robot that can't arc, once the maze's axis directions
        are known (from the start spin, refined by the localizer):
        - only ever drive along an axis; every turn is a whole number of 90 deg,
          measured with the odometry
        - at a wall ahead, stop about where the crossing corridor's centre line
          is, then turn 90 deg toward the more open side (180 at a dead end)
        - between two walls, keep to the centre: if off by more than
          WANDER_CENTER_TOL_IN, side-step with a short angled leg"""
        oh = self.last_heading
        w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
        w_min = math.radians(cfg.NAV_TURN_MIN_DEG_S)
        if self.w_heading is None:
            self.w_heading = self._snap(oh)
            if self.w_first is not None:                 # chosen from the start spin
                self.w_heading, self.w_first = self.w_first, None
            return self._w_turn_to(self.w_heading, "drive")

        if self.w_state == "turn":
            rem = wrap_angle(self.w_goal - oh)
            rate = float(np.clip(cfg.NAV_TURN_GAIN * abs(rem), w_min, w_max))
            if abs(rem) <= rate * cfg.NAV_TURN_LEAD_S:
                self.w_state, self.w_timer = "settle", cfg.NAV_TURN_SETTLE_S
                return 0.0, 0.0
            return 0.0, math.copysign(rate, rem)
        if self.w_state == "settle":
            self.w_timer -= dt
            if self.w_timer > 0:
                return 0.0, 0.0
            self.w_state = self.w_next
            self.w_start = self.last_odom

        if self.w_state == "sidestep":
            done = self.last_odom is None or self.w_start is None or \
                abs(self.last_odom - self.w_start) >= self.w_dist
            if done or front <= cfg.WANDER_FRONT_STOP_IN:
                self.w_last_fix = self.last_odom
                return self._w_turn_to(self.w_heading, "drive")
            return cfg.WANDER_SPEED_IN_S, 0.0

        # ---- driving along self.w_heading
        self.w_heading = self._snap(self.w_heading)      # follows refinements of the axis
        if front <= cfg.WANDER_FRONT_STOP_IN or corner:
            # wall (or a wall corner the IR sees) ahead: turn 90 deg toward the
            # more open side; both sides closed = dead end: turn around
            q = math.pi / 2
            if corner:
                turn = -corner * q                       # away from the corner
            elif max(left, right) < cfg.WANDER_CORRIDOR_MAX_IN:
                turn = 2 * q
            else:
                turn = q if right >= left else -q        # + = clockwise = right
            self.w_heading = self._snap(self.w_heading + turn)
            return self._w_turn_to(self.w_heading, "drive")
        # heading drifted off the axis (e.g. the axis estimate was refined)
        if abs(wrap_angle(self.w_heading - oh)) > max(math.radians(cfg.NAV_ALIGN_START_DEG),
                                                      w_min * cfg.NAV_TURN_LEAD_S + math.radians(1)):
            return self._w_turn_to(self.w_heading, "drive")
        # keep to the middle between the walls
        gap = self._side_gap(left, right)
        if gap is None:
            # one wall only: keep at least WANDER_SIDE_MIN_IN from it
            if left < cfg.WANDER_SIDE_MIN_IN:
                gap = -(cfg.WANDER_SIDE_MIN_IN + 1.0 - left)
            elif right < cfg.WANDER_SIDE_MIN_IN:
                gap = cfg.WANDER_SIDE_MIN_IN + 1.0 - right
        recent = (self.w_last_fix is not None and self.last_odom is not None
                  and abs(self.last_odom - self.w_last_fix) < cfg.WANDER_CENTER_EVERY_IN)
        if gap is not None and abs(gap) > cfg.WANDER_CENTER_TOL_IN and not recent:
            a = math.radians(cfg.WANDER_CENTER_ANGLE_DEG)
            side = -1 if gap > 0 else 1                  # right of centre -> step left (-)
            self.w_dist = min(abs(gap) / math.sin(a), cfg.WANDER_CENTER_MAX_STEP_IN)
            return self._w_turn_to(self.w_heading + side * a, "sidestep")
        return cfg.WANDER_SPEED_IN_S, 0.0

    def _set_axis_after_spin(self, theta, spread, conf):
        """End of the start spin: find the maze axes in the odometry frame.
        Best source: the localizer, if it's already confident (the axes are at
        estimate headings 0/90/180/270; that holds even for a look-alike lock).
        Otherwise the spin readings. Then pick the first wander direction: the
        axis direction that looked most open during the spin."""
        q = math.pi / 2
        if (self.last_heading is not None and spread < cfg.NAV_START_SPREAD_IN
                and conf >= cfg.NAV_START_CONF):
            self.axis = (self.last_heading - theta) % q
        else:
            self.axis = self._axis_from_spin()
        if self.axis is not None:
            self.w_first = self._most_open_axis_heading()

    def _most_open_axis_heading(self):
        idx = self.sensor_idx.get("front")
        if idx is None or not self.spin_samples:
            return None
        h = np.array([s[0] for s in self.spin_samples])
        f = np.array([s[1][idx] for s in self.spin_samples])
        f = np.where(np.isfinite(f), f, 0.0)
        best_front, first = -1.0, None
        for k in range(4):
            d = self.axis + k * math.pi / 2
            near = np.abs((h - d + math.pi) % (2 * math.pi) - math.pi) <= math.radians(10)
            if near.any() and float(np.median(f[near])) > best_front:
                best_front, first = float(np.median(f[near])), d
        return first

    def _axis_from_spin(self):
        """Maze axis direction (odometry frame, mod 90 deg) from the start spin:
        opposite sonars (front+back, left+right) read shortest when they point
        straight at the walls. The 15 deg sonar cone makes that minimum a
        flat bottom, so take the centre of the lowest +-8 deg window. (Only a
        fallback: the minimum is shallow, so this is good to ~10-15 deg.)"""
        if len(self.spin_samples) < 12:
            return None
        h = np.array([s[0] for s in self.spin_samples])
        r = np.array([s[1] for s in self.spin_samples])
        idx = self.sensor_idx
        if not all(k in idx for k in ("front", "back", "left", "right")):
            return None
        r = np.where(np.isfinite(r), r, 60.0)
        pair = np.minimum(r[:, idx["front"]] + r[:, idx["back"]], r[:, idx["left"]] + r[:, idx["right"]])
        best, best_a = math.inf, None
        for deg in range(90):
            a = math.radians(deg)
            near = np.abs((h - a + math.pi / 4) % (math.pi / 2) - math.pi / 4) <= math.radians(8)
            if near.sum() >= 2 and pair[near].mean() < best:
                best, best_a = pair[near].mean(), a
        return best_a

    def _update_axis(self, theta, spread, conf):
        """Refine the axis from the localizer: the maze axes are at estimate
        headings of 0/90/180/270, so (odometry heading - estimate heading)
        mod 90 deg is the axis. Holds even if the lock is a look-alike spot."""
        if self.last_heading is None or spread >= cfg.NAV_START_SPREAD_IN or conf < cfg.NAV_START_CONF:
            return
        q = math.pi / 2
        a = self.last_heading - theta
        if self.axis is None:
            self.axis = a % q
            return
        d = (a - self.axis + q / 2) % q - q / 2
        # big disagreement = the spin fallback was off: jump; small = smooth out noise
        self.axis = (self.axis + (1.0 if abs(d) > math.radians(6) else 0.05) * d) % q

    def _wander_no_arc(self, front, right, left, corner, dt):
        """Wander for a robot that can only drive straight or turn in place."""
        w_turn = 0.7 * math.radians(cfg.NAV_TURN_MAX_DEG_S)
        if self.side_turn_time > 0:            # finishing a short turn away from a wall
            self.side_turn_time -= dt
            if self.side_turn_time <= 0:
                # turning on the spot doesn't move it away from the wall:
                # now actually drive off before reacting to the sides again
                self.escape_time = cfg.WANDER_ESCAPE_S
            return 0.0, self.wander_dir * w_turn
        if front <= cfg.WANDER_FRONT_CLEAR_IN:  # blocked: turn toward open side until clear
            self.escape_time = 0.0
            return 0.0, self.wander_dir * w_turn
        if self.escape_time > 0:
            self.escape_time -= dt
            return cfg.WANDER_SPEED_IN_S, 0.0
        self.wander_dir = 1 if right >= left else -1
        if corner:
            # 45-degree IR sees a wall corner ahead on one side: turn away from it
            self.wander_dir = -corner
            self.side_turn_time = cfg.WANDER_SIDE_TURN_S
            return 0.0, self.wander_dir * w_turn
        if min(left, right) < cfg.WANDER_SIDE_MIN_IN:
            # drifting into a side wall: stop and turn away from it briefly
            self.side_turn_time = cfg.WANDER_SIDE_TURN_S
            return 0.0, self.wander_dir * w_turn
        return cfg.WANDER_SPEED_IN_S, 0.0

    def _follow(self, x, y, theta):
        self.path = self.path_from(x, y)
        travelled, tx, ty = 0.0, *self.path[-1]
        px, py = x, y
        for qx, qy in self.path[1:]:
            travelled += math.hypot(qx - px, qy - py)
            px, py = qx, qy
            if travelled >= cfg.NAV_LOOKAHEAD_IN:
                tx, ty = qx, qy
                break
        if not cfg.ROBOT_CAN_ARC:
            tx, ty = self._straight_target(x, y)
        self.target = (tx, ty)
        err = wrap_angle(math.atan2(ty - y, tx - x) - theta)
        w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
        if not cfg.ROBOT_CAN_ARC:
            return self._follow_no_arc(x, y, theta, err)
        w = float(np.clip(cfg.NAV_TURN_GAIN * err, -w_max, w_max))
        if abs(err) > math.radians(cfg.NAV_TURN_IN_PLACE_DEG):
            v = 0.0
        else:
            v = cfg.NAV_SPEED_IN_S * math.cos(err) ** 2
        return v, w

    def _follow_no_arc(self, x, y, theta, err):
        """Turn in place until facing the target, then drive straight at it.

        A turn is measured with the ODOMETRY, not the position estimate: when a
        turn starts, the needed angle is taken from the estimate once, and the
        robot turns until the encoders say it has turned that far. That is
        steadier than chasing the estimate, which jumps a little with every
        sensor update. The turn is ended NAV_TURN_LEAD_S early to allow for the
        delay before a stop command takes effect, then the robot holds still
        for NAV_TURN_SETTLE_S and checks the heading again."""
        w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
        w_min = math.radians(cfg.NAV_TURN_MIN_DEG_S)
        # the smallest turn the robot can make: even the slowest turn keeps
        # going for NAV_TURN_LEAD_S after the stop is sent
        smallest = w_min * cfg.NAV_TURN_LEAD_S + math.radians(1.0)
        odom_h = self.last_heading
        if self.settle_time > 0:                       # just finished a turn: let it stop
            self.settle_time -= self._dt
            return 0.0, 0.0
        if self.aligning:
            if odom_h is None or self.turn_goal is None:
                remaining = err                        # no odometry: fall back to the estimate
            else:
                remaining = wrap_angle(self.turn_goal - odom_h)
            rate = float(np.clip(cfg.NAV_TURN_GAIN * abs(remaining), w_min, w_max))
            if abs(remaining) <= rate * cfg.NAV_TURN_LEAD_S:
                # stop now: the delay carries it the rest of the way
                self.aligning, self.turn_goal = False, None
                self.settle_time = cfg.NAV_TURN_SETTLE_S
                return 0.0, 0.0
            return 0.0, math.copysign(rate, remaining)
        start = abs(err) > max(math.radians(cfg.NAV_ALIGN_START_DEG), smallest)
        if not start and abs(err) > smallest:
            # about to drive straight: make sure the current heading doesn't
            # run into a wall within the next few inches
            ax = x + cfg.NAV_CLEAR_AHEAD_IN * math.cos(theta)
            ay = y + cfg.NAV_CLEAR_AHEAD_IN * math.sin(theta)
            start = not self._line_clear(x, y, ax, ay)
        if start:
            self.aligning = True
            self.turn_goal = None if odom_h is None else odom_h + err
            return 0.0, 0.0                            # the turn starts on the next step
        return cfg.NAV_SPEED_IN_S, 0.0

    def _stall_check(self, odom, dt):
        """Detect 'driving forward but the wheels report no progress'."""
        if odom is None or self.last_v < 1.0:
            self.stall_ref, self.stall_time = odom, 0.0
            return
        if self.stall_ref is None or abs(odom - self.stall_ref) > cfg.STALL_MIN_PROGRESS_IN:
            self.stall_ref, self.stall_time = odom, 0.0
        else:
            self.stall_time += dt
            if self.stall_time > cfg.STALL_TIME_S:
                self.recover_time = cfg.RECOVER_BACKUP_S + cfg.RECOVER_TURN_S
                self.stall_ref, self.stall_time = None, 0.0
                self.stalls += 1

    def _recover(self, ranges, dt):
        self.recover_time -= dt
        if self.recover_time > cfg.RECOVER_TURN_S:
            # back up only if something is really close ahead: backing blind
            # into a wall behind is worse than just turning on the spot
            ahead = min(self._reading(ranges, "front"), self._reading(ranges, "front_left"),
                        self._reading(ranges, "front_right"))
            if ahead < cfg.RECOVER_BACKUP_IF_AHEAD_IN:
                return -cfg.RECOVER_SPEED_IN_S, 0.0
            self.recover_time = cfg.RECOVER_TURN_S          # skip straight to the turn
        if self.recover_time > cfg.RECOVER_TURN_S - dt:  # pick turn direction once
            corner = self._corner_close(ranges)
            if corner:
                self.wander_dir = -corner   # turn away from the corner the IR saw
            else:
                right = self._reading(ranges, "right")
                left = self._reading(ranges, "left")
                self.wander_dir = 1 if right >= left else -1
        return 0.0, self.wander_dir * 0.7 * math.radians(cfg.NAV_TURN_MAX_DEG_S)

    def step(self, estimate, ranges, dt, odom=None, odom_heading=None,
             tof=math.nan, sample_id=None):
        """estimate = ParticleFilter.estimate(); ranges = last sensor readings
        (inches, NaN = missing) or None; odom / odom_heading = latest
        cumulative odometry distance (inches) and heading (radians), or None;
        tof = latest rear ToF reading (inches, NaN = none); sample_id changes
        whenever a new sample arrived. Returns (v in/s, w rad/s clockwise).
        Gripper commands are left in self.outbox."""
        self._tof, self._sample_id = tof, sample_id
        v, w = self._step(estimate, ranges, dt, odom, odom_heading)
        if not cfg.ROBOT_CAN_ARC and v != 0.0 and w != 0.0:
            v = 0.0  # last-line guard: never send a combined drive + turn
        wanted_v = v
        v = self._wall_guard(v, ranges)
        if self.guarded:
            self.wait_reason = self.guarded
        if cfg.FIXED_SPEED_MODE:
            v, w = fixed_speed(v, w)
        # When the guard holds back a drive the planner wanted, report it as a
        # stall: the usual recovery (back up, turn toward open space) follows.
        # Not while picking up / dropping the block, where backing up is wrong.
        if self.phase in (BLOCK, RELEASE):
            self.last_v = v
        else:
            self.last_v = wanted_v if v == 0.0 and wanted_v > 0 else v
        return v, w

    def _wall_guard(self, v, ranges):
        """Last line of defence, from the LIVE sensor readings, whatever the
        position estimate says: never drive forward when the front sonar or a
        front IR sees a wall close ahead, and never reverse when the back sonar
        does. Thresholds are sensor readings (inches from the sensor face)."""
        self.guarded = None
        if ranges is None or v == 0.0:
            return v
        if v > 0:
            front = self._reading(ranges, "front")
            ir = min(self._reading(ranges, "front_left"), self._reading(ranges, "front_right"))
            if front < cfg.GUARD_FRONT_IN:
                self.guarded = f"wall guard: front {front:.1f} in"
                return 0.0
            if ir < cfg.GUARD_IR_IN:
                self.guarded = f"wall guard: front IR {ir:.1f} in"
                return 0.0
        else:
            back = self._reading(ranges, "back")
            if back < cfg.GUARD_BACK_IN:
                self.guarded = f"wall guard: back {back:.1f} in"
                return 0.0
        return v

    def _deliver(self, x, y, theta, dt):
        """Carry the block to target 2: drive to the planned drop pose, turn
        the gripper toward the drop spot, then hand over to RELEASE."""
        if self.drop is None:
            self.drop = plan_drop(self.grid, self.goals[1].rect, x, y, self.safe)
            self.drop_state, self.drop_timer = "goto", 0.0
            if self.drop is None:
                self.wait_reason = "no drivable spot to drop the block in target 2 - move target 2"
                self.path, self.target = [], None
                return 0.0, 0.0
            self.set_point_goal(*self.drop[0])
        (px, py), phi, _ = self.drop
        self.drop_timer += dt
        if self.drop_timer > 2 * cfg.BLOCK_TASK_TIMEOUT_S:
            self.drop = None              # taking too long: re-plan from here
            return 0.0, 0.0
        if self.drop_state == "goto":
            self.wait_reason = "carrying block to target 2"
            if self.in_goal(x, y):
                self.drop_state = "turn"
                self.path, self.target = [], None
                return 0.0, 0.0
            return self._follow(x, y, theta)
        # turn so the gripper (back) faces the drop spot
        self.wait_reason = "turning gripper toward drop spot"
        err = wrap_angle(phi - theta)
        if abs(err) > math.radians(3):
            w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
            rate = float(np.clip(cfg.NAV_TURN_GAIN * abs(err), math.radians(cfg.NAV_TURN_MIN_DEG_S), w_max))
            return 0.0, math.copysign(rate, err)
        self.phase = RELEASE
        self.release_time = 0.0
        self.outbox.append(GRIP_OPEN)
        self.holding = False
        return 0.0, 0.0

    def _update_travel(self, odom, odom_heading):
        if odom is not None:
            if self.last_odom is not None:
                self.travel += abs(odom - self.last_odom)
                self.straight += abs(odom - self.last_odom)
            self.last_odom = odom
        if odom_heading is not None:
            if self.last_heading is not None:
                turned = abs(wrap_angle(odom_heading - self.last_heading))
                self.travel += turned / (math.pi / 2) * cfg.NAV_TURN_TRAVEL_PER_90DEG_IN
            self.last_heading = odom_heading

    def _start_spin(self, odom_heading, ranges, dt):
        """Start of the run: spin a full 360 degrees in place before driving
        anywhere, so every sensor sweeps the walls and the localizer gets a
        first fix. Measured with the odometry (so it only counts real turning,
        e.g. not while the autopilot is off); without odometry, by time."""
        target = math.radians(cfg.START_SPIN_DEG)
        if odom_heading is not None:
            if self.spin_last is not None:
                self.spin_turned += abs(wrap_angle(odom_heading - self.spin_last))
            self.spin_last = odom_heading
            if ranges is not None and (not self.spin_samples or self.spin_samples[-1][0] != odom_heading):
                self.spin_samples.append((odom_heading, np.array(ranges, dtype=float)))
        else:
            self.spin_turned += math.radians(cfg.START_SPIN_DEG_S) * dt
        rate = math.radians(cfg.START_SPIN_DEG_S)
        # stop early by the command delay, like every other turn
        if self.spin_turned >= target - rate * cfg.NAV_TURN_LEAD_S:
            self.start_spin_done = True
            return 0.0, 0.0
        self.wait_reason = (f"start spin: {math.degrees(self.spin_turned):.0f}/"
                            f"{cfg.START_SPIN_DEG:g} deg")
        return 0.0, rate                              # clockwise on screen = turning right

    def _waiting_for(self, spread, conf, ranges):
        """Human-readable list of what still blocks LOCALIZE -> NAVIGATE."""
        if ranges is None:
            return "no sensor data yet"
        why = []
        if spread >= cfg.NAV_START_SPREAD_IN:
            why.append(f"spread {spread:.1f}>{cfg.NAV_START_SPREAD_IN:g} in")
        if conf < cfg.NAV_START_CONF:
            why.append(f"conf {conf:.0%}<{cfg.NAV_START_CONF:.0%}")
        if not why and self.good_time < cfg.NAV_START_HOLD_S:
            why.append(f"holding {self.good_time:.1f}/{cfg.NAV_START_HOLD_S:g} s")
        if self.travel < cfg.NAV_MIN_TRAVEL_IN:
            why.append(f"travel {self.travel:.0f}/{cfg.NAV_MIN_TRAVEL_IN:g} in")
        if self.straight < cfg.NAV_MIN_STRAIGHT_IN:
            why.append(f"driven {self.straight:.0f}/{cfg.NAV_MIN_STRAIGHT_IN:g} in")
        if self.cooldown > 0:
            why.append(f"retry in {self.cooldown:.1f} s")
        return ", ".join(why)

    def _step(self, estimate, ranges, dt, odom, odom_heading):
        x, y, theta, spread, conf = estimate
        self._dt = dt
        self._update_travel(odom, odom_heading)
        self.cooldown = max(0.0, self.cooldown - dt)
        self.wait_reason = ""
        if not self.start_spin_done:
            v, w = self._start_spin(odom_heading, ranges, dt)
            if self.start_spin_done:
                self._set_axis_after_spin(theta, spread, conf)
            return v, w
        self._update_axis(theta, spread, conf)
        if self.mode not in (ARRIVED, WAITING):
            self._stall_check(odom, dt)
            if self.recover_time > 0:
                self.wait_reason = "stall recovery: backing up"
                return self._recover(ranges, dt)
        good = spread < cfg.NAV_START_SPREAD_IN and conf >= cfg.NAV_START_CONF
        bad = spread > cfg.NAV_STOP_SPREAD_IN or conf < cfg.NAV_STOP_CONF
        self.good_time = self.good_time + dt if good else 0.0

        if self.mode in (ARRIVED, WAITING):
            if bad and self.mode == WAITING:
                self.reset()  # we weren't where we thought: start over
            else:
                if self.mode == WAITING:
                    self.wait_reason = ("holding the block - right-click to choose target 2"
                                        if self.holding else
                                        "reached target 1 - right-click to choose target 2")
                return 0.0, 0.0

        if (self.mode == LOCALIZE and self.good_time >= cfg.NAV_START_HOLD_S
                and self.travel >= cfg.NAV_MIN_TRAVEL_IN
                and self.straight >= cfg.NAV_MIN_STRAIGHT_IN and self.cooldown <= 0):
            self.mode = NAVIGATE
            self.best_dist, self.no_progress_time = math.inf, 0.0
            if self.phase == BLOCK:
                self.block.resume()
            elif self.phase == TARGET2 and self.goals[1] is None:
                self.mode = WAITING
                return 0.0, 0.0
        elif self.mode == NAVIGATE and bad:
            self.mode = LOCALIZE

        if self.mode == NAVIGATE and self.phase == BLOCK:
            v, w = self.block.step(x, y, theta, self._tof, self._sample_id,
                                   odom, odom_heading, dt)
            self.wait_reason = self.block.status
            if self.block.state not in ("goto_view", "goto_stage"):
                self.path, self.target = [], None
            return v, w

        if self.mode == NAVIGATE and self.phase == TARGET2 and self.holding:
            return self._deliver(x, y, theta, dt)

        if self.mode == NAVIGATE and self.phase == RELEASE:
            self.release_time += dt
            self.path, self.target = [], None
            if self.release_time < cfg.GRIP_TIME_S:
                self.wait_reason = "opening gripper"
                self.release_clear = self._body_clear(x, y, theta, cfg.BLOCK_RELEASE_CLEAR_IN)
                return 0.0, 0.0
            if (self.release_clear
                    and self.release_time < cfg.GRIP_TIME_S + cfg.BLOCK_RELEASE_CLEAR_IN / 3.0):
                # pull away from the block, only if the map says there's room
                self.wait_reason = "pulling away from the block"
                return 3.0, 0.0
            self.phase, self.mode = DONE, ARRIVED
            return 0.0, 0.0

        if self.mode == NAVIGATE:
            cx, cy = self._cell(x, y)
            d = self.dist[cy, cx]
            if d < self.best_dist - 1.0:
                self.best_dist, self.no_progress_time = d, 0.0
            else:
                self.no_progress_time += dt
            if self.no_progress_time > cfg.NAV_NO_PROGRESS_S:
                # stuck or chasing a wrong estimate: go back to gathering evidence
                self.mode = LOCALIZE
                self.cooldown = cfg.NAV_RETRY_COOLDOWN_S
            elif self.in_goal(x, y) and self.straight < cfg.NAV_ARRIVE_MIN_DRIVEN_IN:
                # looks like we're there, but too little evidence yet: a
                # same-shaped spot elsewhere would look identical. Keep
                # gathering evidence (the path brings it back afterwards).
                self.arrive_time = 0.0
                self.path, self.target = [], None
                self.wait_reason = (f"confirming arrival: driven {self.straight:.0f}/"
                                    f"{cfg.NAV_ARRIVE_MIN_DRIVEN_IN:g} in")
                return self._wander(ranges, dt) if ranges is not None else (0.0, 0.0)
            elif self.in_goal(x, y):
                self.arrive_time += dt
                if self.arrive_time >= cfg.NAV_ARRIVE_HOLD_S:
                    self.path, self.target = [], None
                    if self.phase == TARGET1 and self.block is not None:
                        self.phase = BLOCK           # now find and grip the block
                        self.block.start(x, y)
                    elif self.phase == TARGET1:      # no ToF configured: straight on
                        self.phase = TARGET2
                        if self.goals[1] is None:
                            self.mode = WAITING
                        else:
                            self._activate(1)
                    elif self.phase == TARGET2:
                        if self.holding:
                            self.phase = RELEASE     # let go of the block
                            self.release_time = 0.0
                            self.outbox.append(GRIP_OPEN)
                            self.holding = False
                        else:
                            self.phase, self.mode = DONE, ARRIVED
                    return 0.0, 0.0
            else:
                self.arrive_time = 0.0

        if self.mode == NAVIGATE:
            return self._follow(x, y, theta)

        self.path, self.target = [], None
        self.wait_reason = self._waiting_for(spread, conf, ranges)
        if ranges is None:
            return 0.0, 0.0
        return self._wander(ranges, dt)
