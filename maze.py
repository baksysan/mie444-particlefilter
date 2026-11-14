"""Maze map and the precomputed range tables for the sonar and IR sensors."""
import hashlib
import math
import os
import time

import numpy as np

import config as cfg

GRID_WIDTH = 96 + 2   # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2

FREE, WALL, BLOCKED = 0, 1, 2   # BLOCKED: not a wall, but the robot's centre can't reach it


def init_grid():
    """1 cell = 1 inch. Same maze as the original simulation."""
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), BLOCKED, dtype=np.int8)

    # Corridors the robot's centre can occupy
    grid[4:46, 4:10] = FREE
    grid[4:22, 10:22] = FREE
    grid[4:10, 22:40] = FREE
    grid[28:40, 28:34] = FREE
    grid[4:22, 40:46] = FREE
    grid[16:22, 46:64] = FREE
    grid[4:46, 64:70] = FREE
    grid[40:46, 10:64] = FREE
    grid[16:22, 64:88] = FREE
    grid[4:46, 88:94] = FREE

    # Outer walls
    grid[0, :] = WALL
    grid[GRID_HEIGHT - 1, :] = WALL
    grid[:, 0] = WALL
    grid[:, GRID_WIDTH - 1] = WALL

    # Inner walls
    grid[25:37, 13:25] = WALL
    grid[13:25, 25:37] = WALL
    grid[25:37, 37:61] = WALL
    grid[1:13, 49:61] = WALL
    grid[1:13, 73:85] = WALL
    grid[25:49, 73:85] = WALL
    return grid


def is_free(grid, x, y):
    """Vectorised: True where (x, y) in inches lies in a FREE cell."""
    xi = np.floor(x).astype(int)
    yi = np.floor(y).astype(int)
    inside = (xi >= 0) & (xi < grid.shape[1]) & (yi >= 0) & (yi < grid.shape[0])
    out = np.zeros(np.shape(x), dtype=bool)
    out[inside] = grid[yi[inside], xi[inside]] == FREE
    return out


def free_cells(grid):
    """(N, 2) array of [x, y] integer cell corners that are FREE."""
    ys, xs = np.nonzero(grid == FREE)
    return np.stack([xs, ys], axis=1).astype(np.float64)


def sensor_param(key, sensors=None):
    """Per-sensor array of SENSOR_TYPES[type][key], in SENSORS order (or the
    order of the given sensor list)."""
    sensors = cfg.SENSORS if sensors is None else sensors
    return np.array([cfg.SENSOR_TYPES[s[4]][key] for s in sensors], dtype=np.float64)


def sensor_index():
    """{sensor name: column in the ranges array}."""
    return {s[0]: i for i, s in enumerate(cfg.SENSORS)}


# --------------------------------------------------------------------------
# Range tables: table[angle_deg, iy, ix] = what a sensor at (ix*res, iy*res)
# pointing at angle_deg would read, with its beam cone applied. One table per
# distinct beam width (sonar 15 deg, IR 5 deg); clipping to each sensor's
# min/max range happens at lookup time.
# --------------------------------------------------------------------------
def _beam_widths():
    return sorted({float(t["beam_deg"]) for t in cfg.SENSOR_TYPES.values()})


def _max_range():
    return max(t["max_in"] for t in cfg.SENSOR_TYPES.values())


def _table_key(grid):
    h = hashlib.sha1(grid.tobytes())
    h.update(repr((cfg.TABLE_RES_IN, _max_range(), _beam_widths(), 2)).encode())
    return h.hexdigest()


def _raw_rays(grid, max_range, verbose=True):
    """Single-ray distance to the nearest wall for every grid point and degree."""
    res = cfg.TABLE_RES_IN
    ny = int(round(grid.shape[0] / res)) + 1
    nx = int(round(grid.shape[1] / res)) + 1
    px, py = np.meshgrid(np.arange(nx) * res, np.arange(ny) * res)
    px = px.ravel().astype(np.float32)
    py = py.ravel().astype(np.float32)

    step = 0.1
    t = np.arange(0.0, max_range + step, step, dtype=np.float32)
    solid = grid == WALL
    h, w = grid.shape

    raw = np.empty((360, ny * nx), dtype=np.float32)
    t0 = time.time()
    for a in range(360):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        xs = np.floor(px[:, None] + t[None, :] * c).astype(np.int32)
        ys = np.floor(py[:, None] + t[None, :] * s).astype(np.int32)
        outside = (xs < 0) | (xs >= w) | (ys < 0) | (ys >= h)
        hit = outside.copy()
        ok = ~outside
        hit[ok] = solid[ys[ok], xs[ok]]
        any_hit = hit.any(axis=1)
        first = hit.argmax(axis=1)
        raw[a] = np.where(any_hit, t[first], max_range)
        if verbose and a % 45 == 0:
            print(f"  building range table: {a}/360 ({time.time() - t0:.0f}s)")
    return raw.reshape(360, ny, nx)


def _cone(raw, beam_deg):
    """A sensor reports the nearest surface anywhere inside its cone."""
    half = int(math.floor(beam_deg / 2 + 1e-9))
    table = raw.copy()
    for off in range(-half, half + 1):
        if off:
            np.minimum(table, np.roll(raw, off, axis=0), out=table)
    return table


_TABLE_CACHE = {}


def load_range_tables(grid):
    """{beam width (deg): table}. Cached, so several RangeModels share them."""
    key = _table_key(grid)
    if key not in _TABLE_CACHE:
        _TABLE_CACHE[key] = _load_range_tables(grid, key)
    return _TABLE_CACHE[key]


def _load_range_tables(grid, key):
    widths = _beam_widths()
    if os.path.exists(cfg.TABLE_FILE):
        data = np.load(cfg.TABLE_FILE)
        if "key" in data and str(data["key"]) == key:
            return {w: data[f"beam_{w:g}"] for w in widths}
        print("Range table settings changed; rebuilding.")
    print("Building sensor range tables (one-time, ~1 min)...")
    raw = _raw_rays(grid, _max_range())
    tables = {w: _cone(raw, w) for w in widths}
    np.savez_compressed(cfg.TABLE_FILE, key=key,
                        **{f"beam_{w:g}": t for w, t in tables.items()})
    print(f"Saved {cfg.TABLE_FILE}")
    return tables


class RangeModel:
    """Looks up expected sensor readings for many poses at once."""

    def __init__(self, grid, sensors=None):
        """sensors: list of SENSORS-style tuples (default cfg.SENSORS)."""
        sensors = cfg.SENSORS if sensors is None else sensors
        tables = load_range_tables(grid)
        self.res = cfg.TABLE_RES_IN
        self.sensor_tables = [tables[float(cfg.SENSOR_TYPES[s[4]]["beam_deg"])]
                              for s in sensors]
        _, self.ny, self.nx = self.sensor_tables[0].shape
        self.mount_angle = np.radians([s[1] for s in sensors])
        self.fwd = np.array([s[2] for s in sensors])
        self.right = np.array([s[3] for s in sensors])
        self.min_in = sensor_param("min_in", sensors)
        self.max_in = sensor_param("max_in", sensors)
        self.reliable_min_in = sensor_param("reliable_min_in", sensors)

    def sensor_poses(self, x, y, theta):
        """Returns sx, sy, beam_angle arrays of shape (N, num_sensors)."""
        x = np.asarray(x, dtype=np.float64)[..., None]
        y = np.asarray(y, dtype=np.float64)[..., None]
        th = np.asarray(theta, dtype=np.float64)[..., None]
        c, s = np.cos(th), np.sin(th)
        # robot "right" in screen coords (y down, clockwise theta) is theta + 90deg
        sx = x + self.fwd * c - self.right * s
        sy = y + self.fwd * s + self.right * c
        return sx, sy, th + self.mount_angle

    def true_distance(self, x, y, theta):
        """Distance (inches) from each sensor to the nearest surface in its
        cone, capped at its max range but NOT raised to its min range (the
        filter needs to know when the true distance is below the reliable
        minimum). Shape (N, num_sensors)."""
        sx, sy, beam = self.sensor_poses(x, y, theta)
        ix = np.clip(np.rint(sx / self.res).astype(int), 0, self.nx - 1)
        iy = np.clip(np.rint(sy / self.res).astype(int), 0, self.ny - 1)
        ia = np.rint(np.degrees(beam)).astype(int) % 360
        out = np.empty(ix.shape, dtype=np.float64)
        for j, table in enumerate(self.sensor_tables):
            out[..., j] = table[ia[..., j], iy[..., j], ix[..., j]]
        return np.minimum(out, self.max_in)

    def expected(self, x, y, theta):
        """Expected readings (inches) of an ideal sensor, clipped to each
        sensor's [min, max] range. Shape (N, num_sensors)."""
        return np.maximum(self.true_distance(x, y, theta), self.min_in)
