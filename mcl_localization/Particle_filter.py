import numpy as np
import cv2
import math
import yaml

class ParticleFilter:
    def __init__(self, map_path, yaml_path, n_particles=300):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        self.resolution = meta['resolution']      # 0.05 m/px
        self.origin     = meta['origin']          # [-5, -5, 0]
        self.map_img    = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        self.N          = n_particles
        self.map_h, self.map_w = self.map_img.shape

        self.particles = self._sample_free(self.N)
        self.weights   = np.ones(self.N) / self.N

    def _world_to_px(self, wx, wy):
        """Convierte coordenadas del mundo (metros) a píxeles del mapa."""
        px = int((wx - self.origin[0]) / self.resolution)
        py = self.map_h - int((wy - self.origin[1]) / self.resolution)
        return px, py

    def _px_to_world(self, px, py):
        """Convierte píxeles del mapa a coordenadas del mundo (metros)."""
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.map_h - py) * self.resolution
        return wx, wy

    def _sample_free(self, n):
        free = np.argwhere(self.map_img > 200)
        idx  = np.random.choice(len(free), n)
        py_arr, px_arr = free[idx, 0], free[idx, 1]
        wx, wy = self._px_to_world(px_arr, py_arr)
        th = np.random.uniform(-np.pi, np.pi, n)
        return np.column_stack([wx, wy, th])

    def score_particles(self, scan_ranges, angle_min, angle_increment):
        scores = np.zeros(self.N)
        step   = max(1, len(scan_ranges) // 72)

        for i, (x, y, th) in enumerate(self.particles):
            score = 0
            for j in range(0, len(scan_ranges), step):
                r = scan_ranges[j]
                if not math.isfinite(r) or r < 0.1 or r > 9.0:
                    continue
                angle = angle_min + j * angle_increment + th
                ex = x + r * math.cos(angle)
                ey = y + r * math.sin(angle)
                px, py = self._world_to_px(ex, ey)
                if 0 <= px < self.map_w and 0 <= py < self.map_h:
                    score += (255 - int(self.map_img[py, px]))
            scores[i] = score
        return scores

    def resample(self, scores):
        threshold = np.percentile(scores, 75)
        good      = np.where(scores >= threshold)[0]

        w = scores[good]
        if w.sum() == 0:
            self.particles = self._sample_free(self.N)
            return

        w = w / w.sum()
        idx = np.random.choice(len(good), self.N, p=w)
        self.particles = self.particles[good[idx]]

        self.particles[:, 0] += np.random.normal(0, 0.05, self.N)
        self.particles[:, 1] += np.random.normal(0, 0.05, self.N)
        self.particles[:, 2] += np.random.normal(0, 0.02, self.N)

    def move_particles(self, dx, dy, dth):
        self.particles[:, 0] += dx  + np.random.normal(0, 0.02, self.N)
        self.particles[:, 1] += dy  + np.random.normal(0, 0.02, self.N)
        self.particles[:, 2] += dth + np.random.normal(0, 0.01, self.N)

    def best_estimate(self):
        return np.mean(self.particles[:10], axis=0)