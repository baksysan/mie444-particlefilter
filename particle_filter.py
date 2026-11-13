"""Vectorised Monte Carlo Localization driven by odometry + ultrasonic ranges."""
import math

import numpy as np

import config as cfg
import maze


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ParticleFilter:
    def __init__(self, grid, range_model, n=cfg.NUM_PARTICLES, rng=None):
        self.grid = grid
        self.model = range_model
        self.n = n
        self.rng = rng if rng is not None else np.random.default_rng()
        self.cells = maze.free_cells(grid)
        self.sigma_base = maze.sensor_param("sigma_base")
        self.sigma_prop = maze.sensor_param("sigma_prop")
        self.random_prob = maze.sensor_param("random_prob")
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.pending_dist = 0.0
        self.pending_rot = 0.0
        self.first_update = True
        self.last_injected = 0
        self.scatter()

    # ------------------------------------------------------------------ init
    def _random_poses(self, k):
        idx = self.rng.integers(len(self.cells), size=k)
        xy = self.cells[idx] + self.rng.random((k, 2))
        th = self.rng.uniform(-math.pi, math.pi, size=k)
        return xy[:, 0], xy[:, 1], th

    def scatter(self):
        """Global localization: particles uniform over the free corridors."""
        self.x, self.y, self.theta = self._random_poses(self.n)
        self.logw = np.full(self.n, -math.log(self.n))
        self.w_slow = self.w_fast = 0.0
        self.first_update = True

    def seed_near(self, x, y, radius=4.0, theta=None):
        """Put all particles around a known spot (e.g. a mouse click)."""
        self.x = x + self.rng.normal(0, radius / 2, self.n)
        self.y = y + self.rng.normal(0, radius / 2, self.n)
        if theta is None:
            self.theta = self.rng.uniform(-math.pi, math.pi, self.n)
        else:
            self.theta = theta + self.rng.normal(0, math.radians(10), self.n)
        self.logw = np.full(self.n, -math.log(self.n))
        self.first_update = True

    # --------------------------------------------------------------- predict
    def predict(self, d, dtheta):
        """Apply an odometry increment: d inches forward, dtheta radians
        (clockwise positive, i.e. screen convention)."""
        ad, at = abs(d), abs(dtheta)
        sd = cfg.ODOM_DIST_PER_DIST * ad + cfg.ODOM_DIST_PER_ROT * at
        st = cfg.ODOM_ROT_PER_ROT * at + cfg.ODOM_ROT_PER_DIST * ad
        dn = d + self.rng.normal(0, sd, self.n) if sd > 0 else d
        tn = dtheta + self.rng.normal(0, st, self.n) if st > 0 else dtheta
        mid = self.theta + tn / 2
        self.x = self.x + dn * np.cos(mid)
        self.y = self.y + dn * np.sin(mid)
        self.theta = wrap_angle(self.theta + tn)
        self.pending_dist += ad
        self.pending_rot += at

    # ---------------------------------------------------------------- update
    def ready_for_update(self):
        return (self.first_update
                or self.pending_dist >= cfg.UPDATE_MIN_DIST_IN
                or self.pending_rot >= cfg.UPDATE_MIN_ANGLE_RAD)

    def update(self, ranges):
        """Fuse one set of ultrasonic readings (inches; NaN = unusable).
        Returns True if a fusion was performed."""
        if not self.ready_for_update():
            return False
        self.first_update = False
        self.pending_dist = self.pending_rot = 0.0

        m = self.model
        z = np.asarray(ranges, dtype=np.float64)
        # readings below a sensor's reliable minimum can't be trusted (Sharp IR
        # folds back there), so they are dropped
        use = np.isfinite(z) & (np.nan_to_num(z, nan=0.0) >= m.reliable_min_in)
        if use.any():
            zu = np.clip(z[use], m.min_in[use], m.max_in[use])
            true_d = m.true_distance(self.x, self.y, self.theta)[:, use]
            exp = np.maximum(true_d, m.min_in[use])
            sigma = self.sigma_base[use] + self.sigma_prop[use] * exp
            gauss = np.exp(-0.5 * ((zu - exp) / sigma) ** 2) / (sigma * math.sqrt(2 * math.pi))
            uniform = 1.0 / m.max_in[use]
            rp = self.random_prob[use]
            p = (1 - rp) * gauss + rp * uniform
            # where this particle would put the sensor closer than it can
            # measure reliably, the reading tells us nothing about the particle
            p = np.where(true_d < m.reliable_min_in[use], uniform, p)
            loglik = cfg.SENSOR_LIKELIHOOD_POWER * np.log(p).sum(axis=1)
        else:
            loglik = np.zeros(self.n)

        off = ~maze.is_free(self.grid, self.x, self.y)
        loglik[off] += cfg.OFF_CORRIDOR_LOG_PENALTY

        # Average likelihood of this reading under the prior -> augmented MCL
        prior = np.exp(self.logw - self.logw.max())
        prior /= prior.sum()
        w_avg = float(np.sum(prior * np.exp(loglik)))
        if self.w_slow == 0.0:
            self.w_slow = self.w_fast = w_avg
        else:
            self.w_slow += cfg.AUGMENT_ALPHA_SLOW * (w_avg - self.w_slow)
            self.w_fast += cfg.AUGMENT_ALPHA_FAST * (w_avg - self.w_fast)

        self.logw = self.logw + loglik
        self.logw -= np.logaddexp.reduce(self.logw)

        if self.n_eff() < cfg.RESAMPLE_NEFF_RATIO * self.n:
            self.resample()
        return True

    def weights(self):
        w = np.exp(self.logw - self.logw.max())
        return w / w.sum()

    def n_eff(self):
        w = self.weights()
        return 1.0 / np.sum(w * w)

    def resample(self):
        w = self.weights()
        # Systematic resampling (low variance, unbiased)
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        idx = np.searchsorted(np.cumsum(w), positions)
        idx = np.minimum(idx, self.n - 1)
        self.x = self.x[idx] + self.rng.normal(0, 0.1, self.n)
        self.y = self.y[idx] + self.rng.normal(0, 0.1, self.n)
        self.theta = wrap_angle(self.theta[idx] + self.rng.normal(0, math.radians(0.5), self.n))

        inject = 0.0
        if self.w_slow > 0:
            inject = min(cfg.MAX_RANDOM_INJECTION, max(0.0, 1.0 - self.w_fast / self.w_slow))
        k = int(inject * self.n)
        if k:
            sel = self.rng.choice(self.n, size=k, replace=False)
            self.x[sel], self.y[sel], self.theta[sel] = self._random_poses(k)
        self.logw = np.full(self.n, -math.log(self.n))
        self.last_injected = k

    # -------------------------------------------------------------- estimate
    def estimate(self):
        """Pose of the dominant particle cluster.

        Returns (x, y, theta, spread, confidence): spread is the RMS distance
        of the cluster's particles from its mean (inches), confidence is the
        fraction of total weight inside the cluster. A plain global mean would
        land between two hypotheses (often inside a wall) while the filter is
        still undecided.
        """
        w = self.weights()
        cell = cfg.CLUSTER_RADIUS_IN / 2
        ix = np.floor(self.x / cell).astype(int)
        iy = np.floor(self.y / cell).astype(int)
        ix -= ix.min()
        iy -= iy.min()
        keys = iy * (ix.max() + 1) + ix
        best = np.argmax(np.bincount(keys, weights=w))
        mx = float(np.mean(self.x[keys == best]))
        my = float(np.mean(self.y[keys == best]))
        r2 = cfg.CLUSTER_RADIUS_IN ** 2
        for _ in range(3):  # a few mean-shift steps
            m = (self.x - mx) ** 2 + (self.y - my) ** 2 < r2
            wc = w[m]
            mx = float(np.sum(wc * self.x[m]) / wc.sum())
            my = float(np.sum(wc * self.y[m]) / wc.sum())
        m = (self.x - mx) ** 2 + (self.y - my) ** 2 < r2
        wc = w[m] / w[m].sum()
        mt = math.atan2(float(np.sum(wc * np.sin(self.theta[m]))),
                        float(np.sum(wc * np.cos(self.theta[m]))))
        spread = math.sqrt(float(np.sum(wc * ((self.x[m] - mx) ** 2 + (self.y[m] - my) ** 2))))
        return mx, my, mt, spread, float(w[m].sum())

    def localized(self):
        _, _, _, spread, conf = self.estimate()
        return spread < cfg.CONVERGED_SPREAD_IN and conf >= cfg.CONVERGED_MIN_WEIGHT
