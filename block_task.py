"""Find the 1 in block in target 1 with the rear ToF, back onto it, grip it.

Runs once the robot has reached target 1 and is localized. Steps:

  sweep       turn on the spot so the rear ToF sweeps 360 deg. A reading clearly
              shorter than the map's wall distance is the block.
  goto_view   nothing found: drive to another viewpoint in the zone, sweep again
  goto_stage  block found: drive (forwards) to a staging point a few inches
              from it, on a side with room to back up
  align       turn so the back (ToF + gripper) faces the block
  refine      close-up sweep +-BLOCK_REFINE_HALF_DEG to pin down the block
  align2      turn so the back faces the refined block position
  reverse     back up slowly while the ToF can still measure the gap
  contact     the block is now too close for the VL53L0X (~3 cm): back up the
              last measured gap by odometry, until it touches the back edge
  grip        send G, wait, check the ToF sees the block right at the edge
  failed      gave up (BLOCK_MAX_ATTEMPTS); stops and says so

Commands for the robot (G = grip, L = leave behind) are put in Navigator.outbox.
"""
import math

import numpy as np

import config as cfg
import maze
from particle_filter import wrap_angle
from robot_link import GRIP_CLOSE, GRIP_OPEN


def _u(a):
    return math.cos(a), math.sin(a)


class BlockTask:
    def __init__(self, nav, grid, tof_model):
        self.nav = nav
        self.grid = grid
        self.model = tof_model
        self.mount = math.radians(cfg.TOF_SENSOR[1])
        self.max_in = float(tof_model.max_in[0])
        # robot centre -> block centre when gripped (block touching the back edge)
        self.reach = cfg.GRIP_REACH_IN
        # how far behind the ToF face the back edge is (0 if the ToF is flush)
        self.edge_gap = cfg.REAR_EDGE_IN - abs(cfg.TOF_SENSOR[2])
        self.last_valid, self.rev_sample, self.contact_odom = None, None, None
        self.block = None           # estimated block centre (x, y), inches
        self.state = "idle"
        self.status = ""
        self.attempts = 0
        self.viewpoints = []
        self.samples = []
        self.last_sample_id = None
        self.last_theta = None
        self.turned = 0.0
        self.timer = 0.0
        self.stage_pt = None
        self.phi = 0.0
        self.reverse_start = None
        self.reverse_max = 0.0
        self.latest_tof = math.nan

    # ------------------------------------------------------------- lifecycle
    def start(self, x, y):
        self.attempts = 0
        self.block = None
        self.viewpoints = self._viewpoints(x, y)
        self._enter("sweep")

    def resume(self):
        """Called after the robot had to re-localize mid-task."""
        if self.state in ("align", "refine", "align2", "reverse", "contact"):
            self._enter("goto_stage" if self.stage_pt is not None else "sweep")
        elif self.state in ("goto_view", "goto_stage"):
            self._enter(self.state)

    def _enter(self, state):
        self.state = state
        self.timer = 0.0
        self.samples = []
        self.turned = 0.0
        self.last_theta = None
        self.nav.aligning = False
        if state == "goto_view":
            vx, vy = self.viewpoints.pop(0)
            self.nav.set_point_goal(vx, vy)
        elif state == "goto_stage":
            # route around the block: body radius ~3 in + block + slack
            avoid = (self.block[0], self.block[1], 4.5)
            self.nav.set_point_goal(*self.stage_pt, avoid=avoid)
        elif state == "grip":
            self.nav.outbox.append(GRIP_CLOSE)

    def _retry(self, why):
        self.attempts += 1
        print(f"[block] {why} (attempt {self.attempts}/{cfg.BLOCK_MAX_ATTEMPTS})")
        if self.attempts >= cfg.BLOCK_MAX_ATTEMPTS:
            self._enter("failed")
        else:
            self._enter("sweep")

    def _viewpoints(self, x, y):
        """Search spots inside target 1: its centre and four quarter points,
        snapped to drivable cells, nearest first."""
        x0, x1, y0, y1 = self.nav.goals[0].rect
        pts = [((x0 + x1) / 2, (y0 + y1) / 2)]
        for fx in (0.25, 0.75):
            for fy in (0.25, 0.75):
                pts.append((x0 + fx * (x1 - x0), y0 + fy * (y1 - y0)))
        free = maze.free_cells(self.grid) + 0.5
        out = []
        for px, py in pts:
            i = int(np.argmin((free[:, 0] - px) ** 2 + (free[:, 1] - py) ** 2))
            p = (float(free[i, 0]), float(free[i, 1]))
            if p not in out:
                out.append(p)
        out.sort(key=lambda p: math.hypot(p[0] - x, p[1] - y))
        return out

    # ---------------------------------------------------------------- helpers
    def _record(self, x, y, theta, tof, sample_id):
        if sample_id == self.last_sample_id:
            return
        self.last_sample_id = sample_id
        sx, sy, beam = self.model.sensor_poses(x, y, theta)
        exp = float(self.model.expected(np.array([x]), np.array([y]), np.array([theta]))[0, 0])
        self.samples.append((float(sx[0]), float(sy[0]), float(beam[0]), tof, exp))

    def _turn_progress(self, odom_heading):
        """Accumulate how far we've turned, from the (smooth) odometry heading."""
        if odom_heading is None:
            return
        if self.last_theta is not None:
            self.turned += abs(wrap_angle(odom_heading - self.last_theta))
        self.last_theta = odom_heading

    def _find_block(self, region_margin=3.0):
        """Look through the recorded sweep for readings clearly shorter than
        the map predicts. Returns the block centre (x, y) or None."""
        if len(self.samples) < 3:
            return None
        s = np.array(self.samples)
        tof, exp = s[:, 3], s[:, 4]
        hit = np.isfinite(tof) & (tof < exp - cfg.BLOCK_MIN_GAP_IN) & (tof < self.max_in - 1)
        # group consecutive hits into runs; the block shows up as one run
        runs, cur = [], []
        for i, h in enumerate(hit):
            if h:
                cur.append(i)
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            runs.append(cur)
        if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == len(hit) - 1:
            runs[0] = runs.pop() + runs[0]          # the sweep wrapped around
        x0, x1, y0, y1 = self.nav.goals[0].rect
        for run in sorted(runs, key=len, reverse=True):
            if len(run) < 2:
                break
            r = s[run]
            bearing = math.atan2(np.sin(r[:, 2]).mean(), np.cos(r[:, 2]).mean())
            dist = float(np.nanmin(r[:, 3])) + cfg.BLOCK_SIZE_IN / 2
            sx, sy = r[:, 0].mean(), r[:, 1].mean()
            bx, by = sx + dist * math.cos(bearing), sy + dist * math.sin(bearing)
            inside = (x0 - region_margin <= bx <= x1 + region_margin
                      and y0 - region_margin <= by <= y1 + region_margin)
            cell_ok = maze.WALL != self.grid[int(np.clip(by, 0, self.grid.shape[0] - 1)),
                                             int(np.clip(bx, 0, self.grid.shape[1] - 1))]
            if inside and cell_ok:
                return bx, by
        return None

    def _plan_approach(self, x, y):
        """Pick the side to approach the block from: the pickup pose (block
        touching the back edge) and a staging point further out must both be drivable, with
        a clear straight line between them, and the staging point must be
        reachable without driving over the block. Shortest drive wins."""
        from navigator import cost_to_goal          # (avoid a circular import)
        bx, by = self.block
        here = np.zeros(self.grid.shape, dtype=bool)
        free = maze.free_cells(self.grid)
        i = int(np.argmin((free[:, 0] + 0.5 - x) ** 2 + (free[:, 1] + 0.5 - y) ** 2))
        here[int(free[i, 1]), int(free[i, 0])] = True
        reach_cost = cost_to_goal(self.grid, here, avoid=(bx, by, 4.5))
        best = None
        for k in range(24):
            phi = k * math.pi / 12
            ux, uy = _u(phi)
            px, py = bx + self.reach * ux, by + self.reach * uy
            far = self.reach + cfg.BLOCK_STAGE_GAP_IN
            stx, sty = bx + far * ux, by + far * uy
            t = np.linspace(0, 1, 12)
            lx, ly = px + t * (stx - px), py + t * (sty - py)
            if not maze.is_free(self.grid, lx, ly).all():
                continue
            if not (0 <= sty < self.grid.shape[0] and 0 <= stx < self.grid.shape[1]) \
                    or not self.nav.safe[int(sty), int(stx)]:
                continue                          # staging point must be ~1 in inside the corridor
            cost = reach_cost[int(sty), int(stx)]
            if not np.isfinite(cost):
                continue                          # can't get there without hitting the block
            if best is None or cost < best[0]:
                best = (cost, phi, (stx, sty))
        if best is None:
            return False
        _, self.phi, self.stage_pt = best
        return True

    def _turn_to(self, theta, target):
        """Turn-on-the-spot command toward heading `target`; None when there."""
        err = wrap_angle(target - theta)
        if abs(err) < math.radians(3):
            return None
        w_max = math.radians(cfg.NAV_TURN_MAX_DEG_S)
        rate = float(np.clip(cfg.NAV_TURN_GAIN * abs(err), math.radians(cfg.NAV_TURN_MIN_DEG_S), w_max))
        return 0.0, math.copysign(rate, err)

    # ------------------------------------------------------------------ step
    def step(self, x, y, theta, tof, sample_id, odom, odom_heading, dt):
        """Returns (v, w); sets self.status. Calls nav.on_block_gripped() on success."""
        self.timer += dt
        if np.isfinite(tof):
            self.latest_tof = tof
        st = self.state
        if st not in ("failed", "idle") and self.timer > cfg.BLOCK_TASK_TIMEOUT_S:
            self._retry(f"'{st}' took too long")
            return 0.0, 0.0
        sweep_w = math.radians(cfg.BLOCK_SWEEP_DEG_S)

        if st == "sweep":
            self._record(x, y, theta, tof, sample_id)
            self._turn_progress(odom_heading)
            need = 2 * math.pi + math.radians(cfg.SENSOR_TYPES["tof"]["beam_deg"])
            self.status = f"block: sweeping {math.degrees(self.turned):3.0f}/{math.degrees(need):.0f} deg"
            if self.turned < need:
                return 0.0, sweep_w
            found = self._find_block()
            if found is not None:
                self.block = found
                print(f"[block] found at ({found[0]:.1f}, {found[1]:.1f})")
                if self._plan_approach(x, y):
                    self._enter("goto_stage")
                else:
                    self._retry("no room to approach the block")
            elif self.viewpoints:
                self._enter("goto_view")
            else:
                self._retry("block not seen from any viewpoint")
                if self.state == "sweep":
                    self.viewpoints = self._viewpoints(x, y)
            return 0.0, 0.0

        if st in ("goto_view", "goto_stage"):
            self.status = ("block: moving to next search spot" if st == "goto_view"
                           else "block: driving to staging point")
            if self.nav.in_goal(x, y):
                self._enter("sweep" if st == "goto_view" else "align")
                return 0.0, 0.0
            self.nav.path = self.nav.path_from(x, y)
            return self.nav.follow_current(x, y, theta)

        if st == "align":
            self.status = "block: turning back toward block"
            cmd = self._turn_to(theta, self.phi + math.radians(cfg.BLOCK_REFINE_HALF_DEG))
            if cmd is None:
                self._enter("refine")
                return 0.0, 0.0
            return cmd

        if st == "refine":
            # sweep from phi+half to phi-half (turning anticlockwise on screen)
            self._record(x, y, theta, tof, sample_id)
            self._turn_progress(odom_heading)
            self.status = "block: close-up sweep"
            if self.turned < 2 * math.radians(cfg.BLOCK_REFINE_HALF_DEG):
                return 0.0, -0.6 * sweep_w
            found = self._find_block()
            if found is None:
                self._retry("block not seen in the close-up sweep")
                return 0.0, 0.0
            self.block = found
            self._enter("align2")
            return 0.0, 0.0

        if st == "align2":
            self.status = "block: lining up the gripper"
            bx, by = self.block
            target = math.atan2(y - by, x - bx)     # back (theta + pi) faces the block
            cmd = self._turn_to(theta, target)
            if cmd is None:
                self.reverse_start = odom
                self.reverse_max = math.hypot(x - bx, y - by) - self.reach + 2.0
                self.last_valid, self.rev_sample = None, None
                self._enter("reverse")
                return 0.0, 0.0
            return cmd

        if st == "reverse":
            # ToF-guided part: back up while the ToF can still measure the gap
            reversed_in = 0.0 if odom is None or self.reverse_start is None else self.reverse_start - odom
            self.status = f"block: backing up, ToF {self.latest_tof:4.1f} in"
            if sample_id != self.rev_sample and odom is not None:
                self.rev_sample = sample_id
                if np.isfinite(tof):
                    self.last_valid = (tof, odom)       # reading and odometry from the same sample
            close = np.isfinite(tof) and tof <= cfg.TOF_CLOSE_IN
            dropped_out = (not np.isfinite(tof) and self.last_valid is not None
                           and self.last_valid[0] < 3.0)
            if (close or dropped_out) and self.last_valid is not None:
                # From here the block is too close for the ToF: finish by odometry.
                # Contact = block surface at the back edge, edge_gap behind the ToF face.
                d, od = self.last_valid
                remaining = d - self.edge_gap + cfg.GRIP_CONTACT_EXTRA_IN
                self.contact_odom = od - remaining
                self._enter("contact")
                return 0.0, 0.0
            if reversed_in > self.reverse_max:
                self._retry("backed up too far without reaching the block")
                return 0.0, 0.0
            return -cfg.BLOCK_REVERSE_SPEED_IN_S, 0.0

        if st == "contact":
            self.status = "block: closing the last gap (odometry)"
            if odom is None or odom <= self.contact_odom:
                self._enter("grip")
                return 0.0, 0.0
            return -0.5 * cfg.BLOCK_REVERSE_SPEED_IN_S, 0.0

        if st == "grip":
            self.status = "block: closing gripper"
            if self.timer < cfg.GRIP_TIME_S:
                return 0.0, 0.0
            # block against the back edge: the ToF reads (almost) nothing, or
            # very close. A miss shows the wall/floor further back instead.
            if not np.isfinite(tof) or tof < cfg.GRIP_CONFIRM_MAX_IN:
                print("[block] gripped")
                self.state = "done"
                self.nav.on_block_gripped()
                return 0.0, 0.0
            self.nav.outbox.append(GRIP_OPEN)
            self._retry(f"no block at the back edge after gripping (ToF {tof:.1f} in)")
            return 0.0, 0.0

        if st == "failed":
            self.status = "block: NOT FOUND / could not pick up - stopped (R to retry)"
            return 0.0, 0.0
        return 0.0, 0.0
