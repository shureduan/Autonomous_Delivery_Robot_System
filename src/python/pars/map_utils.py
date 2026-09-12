from pathlib import Path
import numpy as np


class PlatformMap:
    def __init__(self, npz_path: str | Path, occ_thresh: float = 0.65):
        npz_path = Path(npz_path)
        data = np.load(npz_path)

        self.prob_map = data["prob_map"].astype(np.float32)
        self.explored_mask = data["explored_mask"].astype(np.uint8)

        self.res = float(data["map_res"])
        self.x_min = float(data["map_x_min"])
        self.z_min = float(data["map_z_min"])

        self.h, self.w = self.prob_map.shape
        self.occ_thresh = occ_thresh

        self.occupied = (self.prob_map > occ_thresh)
        self.free = (self.explored_mask > 0) & (~self.occupied)

    def world_to_grid(self, x: float, z: float) -> tuple[int, int]:
        col = int(round((x - self.x_min) / self.res))
        row = int(round((z - self.z_min) / self.res))
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.x_min + col * self.res
        z = self.z_min + row * self.res
        return x, z

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.h and 0 <= col < self.w

    def is_free_world(self, x: float, z: float) -> bool:
        row, col = self.world_to_grid(x, z)
        if not self.in_bounds(row, col):
            return False
        return bool(self.free[row, col])

    def is_occupied_world(self, x: float, z: float) -> bool:
        return not self.is_free_world(x, z)

    def line_is_free(self, x0: float, z0: float, x1: float, z1: float, step_m: float = 0.05) -> bool:
        dx = x1 - x0
        dz = z1 - z0
        dist = float(np.hypot(dx, dz))

        if dist == 0:
            return self.is_free_world(x0, z0)

        n = max(2, int(np.ceil(dist / step_m)) + 1)

        for i in range(n):
            t = i / (n - 1)
            x = x0 + t * dx
            z = z0 + t * dz
            if not self.is_free_world(x, z):
                return False

        return True

    def circle_is_free(self, x: float, z: float, radius: float, sample_step: float = 0.05) -> bool:
        if radius <= 0:
            return self.is_free_world(x, z)

        if not self.is_free_world(x, z):
            return False

        n_r = max(2, int(np.ceil(radius / sample_step)) + 1)

        for ir in range(n_r + 1):
            r = radius * ir / n_r

            if r < 1e-8:
                if not self.is_free_world(x, z):
                    return False
                continue

            circumference = 2.0 * np.pi * r
            n_theta = max(8, int(np.ceil(circumference / sample_step)))

            for k in range(n_theta):
                theta = 2.0 * np.pi * k / n_theta
                px = x + r * np.cos(theta)
                pz = z + r * np.sin(theta)
                if not self.is_free_world(px, pz):
                    return False

        return True