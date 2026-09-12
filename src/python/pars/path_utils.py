import numpy as np


class PathHelper:
    def __init__(self, waypoints_xyz):
        self.waypoints = np.asarray(waypoints_xyz, dtype=float)  # [N, 3]
        self.xz = self.waypoints[:, [0, 2]]  # only x, z

        diffs = self.xz[1:] - self.xz[:-1]
        seg_lens = np.linalg.norm(diffs, axis=1)

        self.seg_lens = seg_lens
        self.cum_s = np.zeros(len(self.xz), dtype=float)
        if len(self.xz) > 1:
            self.cum_s[1:] = np.cumsum(seg_lens)

        self.total_length = float(self.cum_s[-1])

    def project_point(self, x: float, z: float):
        p = np.array([x, z], dtype=float)

        best_dist = float("inf")
        best_s = 0.0
        best_proj = self.xz[0].copy()
        best_seg_idx = 0

        for i in range(len(self.xz) - 1):
            a = self.xz[i]
            b = self.xz[i + 1]
            ab = b - a
            ab2 = float(np.dot(ab, ab))

            if ab2 < 1e-12:
                proj = a
                t = 0.0
            else:
                t = float(np.dot(p - a, ab) / ab2)
                t = max(0.0, min(1.0, t))
                proj = a + t * ab

            dist = float(np.linalg.norm(p - proj))
            s_here = float(self.cum_s[i] + t * self.seg_lens[i])

            if dist < best_dist:
                best_dist = dist
                best_s = s_here
                best_proj = proj
                best_seg_idx = i

        return {
            "dist": best_dist,
            "s": best_s,
            "proj_x": float(best_proj[0]),
            "proj_z": float(best_proj[1]),
            "seg_idx": best_seg_idx,
        }

    def sample_at_s(self, s_query: float):
        s_query = max(0.0, min(float(s_query), self.total_length))

        if s_query <= 0:
            return float(self.xz[0, 0]), float(self.xz[0, 1])
        if s_query >= self.total_length:
            return float(self.xz[-1, 0]), float(self.xz[-1, 1])

        for i in range(len(self.xz) - 1):
            s0 = self.cum_s[i]
            s1 = self.cum_s[i + 1]
            if s0 <= s_query <= s1:
                seg_len = self.seg_lens[i]
                if seg_len < 1e-12:
                    return float(self.xz[i, 0]), float(self.xz[i, 1])

                t = (s_query - s0) / seg_len
                p = self.xz[i] + t * (self.xz[i + 1] - self.xz[i])
                return float(p[0]), float(p[1])

        return float(self.xz[-1, 0]), float(self.xz[-1, 1])

    def get_return_waypoint(self, x: float, z: float, ds_forward: float = 1.0):
        proj = self.project_point(x, z)
        s_return = proj["s"] + ds_forward
        rx, rz = self.sample_at_s(s_return)
        return {
            "proj": proj,
            "return_x": rx,
            "return_z": rz,
            "return_s": s_return,
        }

    def get_local_segment(self, x: float, z: float, segment_length: float = 4.0, ds_sample: float = 0.5):
        proj = self.project_point(x, z)
        s0 = proj["s"]
        s1 = min(self.total_length, s0 + segment_length)

        s_values = list(np.arange(s0, s1, ds_sample))
        if len(s_values) == 0 or s_values[-1] < s1:
            s_values.append(s1)

        points = [self.sample_at_s(sv) for sv in s_values]
        final_x, final_z = points[-1]

        return {
            "proj": proj,
            "s_start": s0,
            "s_end": s1,
            "points": points,
            "final_x": final_x,
            "final_z": final_z,
        }