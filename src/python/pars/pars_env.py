from collections import deque
from pathlib import Path
import json
import random
import numpy as np

from map_utils import PlatformMap
from path_utils import PathHelper


def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


class RobotState:
    def __init__(self, x: float, z: float, yaw: float, v: float = 0.0, w: float = 0.0):
        self.x = float(x)
        self.z = float(z)
        self.yaw = float(yaw)
        self.v = float(v)
        self.w = float(w)

    def step(self, v_cmd: float, w_cmd: float, dt: float):
        # 差速/旋转底盘：允许原地转向
        self.v = float(v_cmd)
        self.w = float(w_cmd)
        self.x += self.v * np.cos(self.yaw) * dt
        self.z += self.v * np.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + self.w * dt)


class PARSEnv:
    def __init__(
        self,
        map_npz_path,
        route_files,
        robot_radius: float = 0.35,
        dt: float = 0.2,
        max_steps: int = 500,
        n_lidar_beams: int = 21,
        lidar_fov_deg: float = 180.0,
        lidar_max_range: float = 6.0,
        lidar_step_m: float = 0.05,
        return_ds_forward: float = 2.0,
        rejoin_check_inflation: float = 0.5,
        pp_takeover_inflation: float = 1.0,
        obstacle_lateral_range: float = 5.0,
        obstacle_min_center_dist: float = 5.0,
        n_circle_obs: int = 2,
        n_rect_obs: int = 2,
        pars_only_mode: bool = True,
        dynamic_n_obs: bool = False,
        seed: int | None = None,
    ):
        self.pm = PlatformMap(map_npz_path)
        self.route_files = [Path(p) for p in route_files]

        self.robot_radius = robot_radius
        self.dt = dt
        self.max_steps = max_steps

        self.n_lidar_beams = n_lidar_beams
        self.lidar_fov_deg = lidar_fov_deg
        self.lidar_max_range = lidar_max_range
        self.lidar_step_m = lidar_step_m

        self.return_ds_forward = return_ds_forward
        self.rejoin_check_inflation = rejoin_check_inflation
        self.pp_takeover_inflation = pp_takeover_inflation

        self.obstacle_lateral_range = obstacle_lateral_range
        self.obstacle_min_center_dist = obstacle_min_center_dist
        self.n_circle_obs = n_circle_obs
        self.n_rect_obs = n_rect_obs
        self.pars_only_mode = pars_only_mode
        self.dynamic_n_obs = dynamic_n_obs

        # =========================================================
        # REWARD / SUCCESS PARAMETERS
        # =========================================================
        self.w_goal_progress = 22.0
        self.w_s_progress = 8.0
        self.w_lateral = 0.05
        self.w_danger = 5.8
        self.w_smooth = 0.01
        self.w_stall = 3.0

        self.collision_penalty = 105.0
        self.rejoin_bonus = 30.0
        self.final_goal_bonus = 230.0

        self.safe_lidar_thresh = 0.9
        self.final_goal_success_radius = 0.5
        self.final_goal_bonus_radius = 0.5

        self.stall_s_thresh = 0.02
        self.stall_goal_thresh = 0.02

        # PP trigger / rejoin parameters
        self.rejoin_dist_thresh = 0.5

        # Pure Pursuit parameters
        self.pp_lookahead = 1.0
        self.pp_waypoint_gain = 1.2
        self.pp_max_v = 1.1
        self.pp_max_w = 1.2
        self.pp_turn_in_place_angle = np.deg2rad(35.0)
        self.pp_forward_speed_when_turning = 0.15
        # =========================================================

        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)

        self.route = None
        self.ph = None
        self.robot = None
        self.obstacles = None
        self.obs = None
        self.proj = None
        self.ret = None
        self.step_count = 0

        self.final_goal_x = None
        self.final_goal_z = None
        self.rejoin_point = None
        self._locked_rejoin_point = None
        self._approaching_rejoin = False
        self._pos_history: deque = deque(maxlen=15)  # last 3s at dt=0.2

        self.prev_action = np.zeros(2, dtype=np.float32)

        self.control_mode = "PP"
        self.pars_trigger_count = 0
        self.rejoin_count = 0

    def _load_route(self, json_path: Path):
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            data = data[0]
        return data

    @staticmethod
    def _point_dist(x1, z1, x2, z2):
        return float(np.hypot(x1 - x2, z1 - z2))

    def _estimate_path_yaw(self, s_query: float, ds: float = 0.3):
        s0 = max(0.0, s_query)
        s1 = min(self.ph.total_length, s_query + ds)
        x0, z0 = self.ph.sample_at_s(s0)
        x1, z1 = self.ph.sample_at_s(s1)
        if abs(x1 - x0) < 1e-8 and abs(z1 - z0) < 1e-8:
            return 0.0
        return np.arctan2(z1 - z0, x1 - x0)

    def _sample_point_near_polyline(self, route_waypoints, lateral_range=5.0):
        idx = random.randint(0, len(route_waypoints) - 2)
        x0, _, z0 = route_waypoints[idx]
        x1, _, z1 = route_waypoints[idx + 1]

        dx = x1 - x0
        dz = z1 - z0
        norm = np.hypot(dx, dz)

        if norm < 1e-8:
            tx, tz = 1.0, 0.0
        else:
            tx, tz = dx / norm, dz / norm

        nx, nz = -tz, tx

        u = random.uniform(0.0, 1.0)
        bx = x0 + u * dx
        bz = z0 + u * dz

        offset = random.uniform(-lateral_range, lateral_range)
        x = bx + offset * nx
        z = bz + offset * nz
        return x, z

    def _far_from_start_goal(self, x, z, route_waypoints, clearance=5.0):
        sx, _, sz = route_waypoints[0]
        gx, _, gz = route_waypoints[-1]
        return (
            self._point_dist(x, z, sx, sz) >= clearance
            and self._point_dist(x, z, gx, gz) >= clearance
        )

    def _generate_obstacles_along_route(self, route_waypoints):
        obstacles = []
        max_trials = 10000

        def center_ok(x, z):
            if not self._far_from_start_goal(x, z, route_waypoints, clearance=5.0):
                return False
            for ob in obstacles:
                if self._point_dist(x, z, ob["x"], ob["z"]) < self.obstacle_min_center_dist:
                    return False
            return True

        trials = 0
        count = 0
        while count < self.n_circle_obs and trials < max_trials:
            trials += 1
            x, z = self._sample_point_near_polyline(
                route_waypoints, lateral_range=self.obstacle_lateral_range
            )
            if not center_ok(x, z):
                continue
            r = random.uniform(0.45, 0.85)
            obstacles.append({"type": "circle", "x": x, "z": z, "r": r})
            count += 1

        trials = 0
        count = 0
        while count < self.n_rect_obs and trials < max_trials:
            trials += 1
            x, z = self._sample_point_near_polyline(
                route_waypoints, lateral_range=self.obstacle_lateral_range
            )
            if not center_ok(x, z):
                continue
            sx = random.uniform(2.5, 4.0)
            sz = random.uniform(0.7, 1.2)
            yaw = random.uniform(-np.pi, np.pi)
            obstacles.append({"type": "rect", "x": x, "z": z, "sx": sx, "sz": sz, "yaw": yaw})
            count += 1

        return obstacles

    def _make_forced_blocking_rectangle(self, s_start: float):
        if self.pars_only_mode:
            # Close enough so the 2m forward ref is blocked immediately at reset
            s_block = min(self.ph.total_length, s_start + random.uniform(2.2, 3.0))
        else:
            s_block = min(self.ph.total_length, s_start + random.uniform(3.5, 5.0))
        bx, bz = self.ph.sample_at_s(s_block)

        path_yaw = self._estimate_path_yaw(s_block, ds=0.5)

        nx = -np.sin(path_yaw)
        nz = np.cos(path_yaw)
        lateral_offset = random.uniform(-0.8, 0.8)

        cx = bx + lateral_offset * nx
        cz = bz + lateral_offset * nz

        sx = random.uniform(2.5, 3.8)
        sz = random.uniform(0.8, 1.2)

        yaw = wrap_angle(path_yaw + np.pi / 2.0 + random.uniform(-0.2, 0.2))

        return {
            "type": "rect",
            "x": cx,
            "z": cz,
            "sx": sx,
            "sz": sz,
            "yaw": yaw,
            "forced": True,
        }

    def _append_forced_blocking_obstacle(self, obstacles, s_start: float):
        obstacles.append(self._make_forced_blocking_rectangle(s_start))
        return obstacles

    def _robot_collides_with_obstacles(self):
        for ob in self.obstacles:
            if ob["type"] == "circle":
                if self._point_dist(self.robot.x, self.robot.z, ob["x"], ob["z"]) < (
                    self.robot_radius + ob["r"]
                ):
                    return True
            else:
                hx = ob["sx"] / 2.0 + self.robot_radius
                hz = ob["sz"] / 2.0 + self.robot_radius

                dx = self.robot.x - ob["x"]
                dz = self.robot.z - ob["z"]

                c = np.cos(-ob["yaw"])
                s = np.sin(-ob["yaw"])
                lx = c * dx - s * dz
                lz = s * dx + c * dz

                if abs(lx) <= hx and abs(lz) <= hz:
                    return True
        return False

    def _point_blocked_by_dynamic_obstacles(self, x: float, z: float):
        for ob in self.obstacles:
            if ob["type"] == "circle":
                if self._point_dist(x, z, ob["x"], ob["z"]) <= ob["r"]:
                    return True
            else:
                dx = x - ob["x"]
                dz = z - ob["z"]
                c = np.cos(-ob["yaw"])
                s = np.sin(-ob["yaw"])
                lx = c * dx - s * dz
                lz = s * dx + c * dz
                if abs(lx) <= ob["sx"] / 2.0 and abs(lz) <= ob["sz"] / 2.0:
                    return True
        return False

    def _circle_free_with_dynamic(self, x: float, z: float, radius: float):
        if not self.pm.circle_is_free(x, z, radius):
            return False

        for ob in self.obstacles:
            if ob["type"] == "circle":
                if self._point_dist(x, z, ob["x"], ob["z"]) < (radius + ob["r"]):
                    return False
            else:
                hx = ob["sx"] / 2.0 + radius
                hz = ob["sz"] / 2.0 + radius

                dx = x - ob["x"]
                dz = z - ob["z"]
                c = np.cos(-ob["yaw"])
                s = np.sin(-ob["yaw"])
                lx = c * dx - s * dz
                lz = s * dx + c * dz
                if abs(lx) <= hx and abs(lz) <= hz:
                    return False

        return True

    def _inflated_line_is_free(self, x0: float, z0: float, x1: float, z1: float, inflation: float):
        dx = x1 - x0
        dz = z1 - z0
        dist = float(np.hypot(dx, dz))
        if dist < 1e-8:
            return self._circle_free_with_dynamic(x0, z0, inflation)

        step_m = 0.05
        n = max(2, int(np.ceil(dist / step_m)) + 1)

        for i in range(n):
            t = i / (n - 1)
            x = x0 + t * dx
            z = z0 + t * dz
            if not self._circle_free_with_dynamic(x, z, inflation):
                return False
        return True

    def _lidar_scan(self):
        angles = np.linspace(
            -np.deg2rad(self.lidar_fov_deg) / 2.0,
            np.deg2rad(self.lidar_fov_deg) / 2.0,
            self.n_lidar_beams,
        )
        ranges = []

        for a in angles:
            beam_yaw = wrap_angle(self.robot.yaw + a)
            hit_dist = self.lidar_max_range

            n_steps = int(np.ceil(self.lidar_max_range / self.lidar_step_m))
            for i in range(1, n_steps + 1):
                d = i * self.lidar_step_m
                x = self.robot.x + d * np.cos(beam_yaw)
                z = self.robot.z + d * np.sin(beam_yaw)

                if (not self.pm.is_free_world(x, z)) or self._point_blocked_by_dynamic_obstacles(x, z):
                    hit_dist = d
                    break

            ranges.append(hit_dist)

        return np.array(ranges, dtype=np.float32)

    def _world_to_robot_frame(self, xw: float, zw: float):
        dx = xw - self.robot.x
        dz = zw - self.robot.z
        c = np.cos(self.robot.yaw)
        s = np.sin(self.robot.yaw)
        x_local = c * dx + s * dz
        y_local = -s * dx + c * dz
        return x_local, y_local

    def _update_path_geometry(self):
        self.proj = self.ph.project_point(self.robot.x, self.robot.z)
        self.ret = self.ph.get_return_waypoint(
            self.robot.x,
            self.robot.z,
            ds_forward=self.return_ds_forward,
        )

        if self._locked_rejoin_point is not None:
            # Keep the locked target fixed; only refresh the distance
            self.rejoin_point = {
                "x": self._locked_rejoin_point["x"],
                "z": self._locked_rejoin_point["z"],
                "s": self._locked_rejoin_point["s"],
                "dist": self._point_dist(
                    self.robot.x,
                    self.robot.z,
                    self._locked_rejoin_point["x"],
                    self._locked_rejoin_point["z"],
                ),
            }
        else:
            self.rejoin_point = {
                "x": self.ret["return_x"],
                "z": self.ret["return_z"],
                "s": self.ret["return_s"],
                "dist": self._point_dist(
                    self.robot.x,
                    self.robot.z,
                    self.ret["return_x"],
                    self.ret["return_z"],
                ),
            }

    def _pp_action(self):
        rx = self.rejoin_point["x"]
        rz = self.rejoin_point["z"]

        x_local, y_local = self._world_to_robot_frame(rx, rz)
        angle_to_target = np.arctan2(y_local, x_local)

        if abs(angle_to_target) > self.pp_turn_in_place_angle:
            v_cmd = 0.0
            w_cmd = np.clip(
                self.pp_waypoint_gain * angle_to_target,
                -self.pp_max_w,
                self.pp_max_w,
            )
        else:
            v_cmd = self.pp_max_v if x_local > 0 else self.pp_forward_speed_when_turning
            w_cmd = np.clip(
                self.pp_waypoint_gain * angle_to_target,
                -self.pp_max_w,
                self.pp_max_w,
            )

        return np.array([v_cmd, w_cmd], dtype=np.float32)

    def _should_trigger_pars(self, scan: np.ndarray):
        # 只看当前前方参考点 p_ref = rejoin_point 的安全膨胀连线是否可通
        blocked_rejoin = not self._inflated_line_is_free(
            self.robot.x,
            self.robot.z,
            self.rejoin_point["x"],
            self.rejoin_point["z"],
            self.rejoin_check_inflation,
        )
        return blocked_rejoin

    def _should_rejoin_pp(self):
        """
        Two-phase rejoin:
        1. Line to forward ref becomes clear → lock that point.
        2. Robot reaches locked point (dist < rejoin_dist_thresh) with clear line → rejoin.
        If line becomes blocked again while approaching, unlock and restart.
        """
        if self.rejoin_point is None:
            return False

        line_free = self._inflated_line_is_free(
            self.robot.x,
            self.robot.z,
            self.rejoin_point["x"],
            self.rejoin_point["z"],
            self.rejoin_check_inflation,
        )

        if not line_free:
            self._locked_rejoin_point = None  # obstacle back in the way, unlock
            return False

        if self._locked_rejoin_point is None:
            # Line just became clear: lock the current target
            self._locked_rejoin_point = {
                "x": self.rejoin_point["x"],
                "z": self.rejoin_point["z"],
                "s": self.rejoin_point["s"],
            }
            return False  # not rejoined yet, still need to reach the point

        # Already locked and line is still free: check arrival
        return self.rejoin_point["dist"] < self.rejoin_dist_thresh

    def _get_obs(self):
        scan = self._lidar_scan() / self.lidar_max_range  # normalize to [0, 1]
        self._update_path_geometry()

        goal_rel_x, goal_rel_y = self._world_to_robot_frame(self.final_goal_x, self.final_goal_z)
        ret_rel_x, ret_rel_y = self._world_to_robot_frame(
            self.rejoin_point["x"],
            self.rejoin_point["z"],
        )

        seg_idx = self.proj["seg_idx"]
        p0 = self.ph.xz[seg_idx]
        p1 = self.ph.xz[min(seg_idx + 1, len(self.ph.xz) - 1)]
        path_yaw = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
        heading_error = wrap_angle(path_yaw - self.robot.yaw)

        # normalize scalar features to [-1, 1] or [0, 1]
        _goal_scale = 50.0
        _ref_scale = 6.0

        obs = np.concatenate([
            scan,
            np.array([
                float(np.clip(goal_rel_x / _goal_scale, -1.0, 1.0)),
                float(np.clip(goal_rel_y / _goal_scale, -1.0, 1.0)),
                float(np.clip(ret_rel_x / _ref_scale, -1.0, 1.0)),
                float(np.clip(ret_rel_y / _ref_scale, -1.0, 1.0)),
                float(np.clip(self.proj["dist"] / 10.0, 0.0, 1.0)),
                float(heading_error / np.pi),
                float(self.robot.v / self.pp_max_v),
                float(np.clip(self.robot.w / self.pp_max_w, -1.0, 1.0)),
            ], dtype=np.float32),
        ])
        return obs

    def _compute_reward(self, prev_s, prev_goal_dist, prev_action, collision, pars_active_this_step, rejoined_this_step):
        final_goal_dist = self._point_dist(
            self.robot.x,
            self.robot.z,
            self.final_goal_x,
            self.final_goal_z,
        )

        goal_progress = prev_goal_dist - final_goal_dist
        r_goal_progress = self.w_goal_progress * goal_progress

        ds_progress = self.proj["s"] - prev_s
        r_s_progress = self.w_s_progress * ds_progress

        r_lateral = -self.w_lateral * (self.proj["dist"] ** 2)

        # lidar obs is normalized [0,1]; convert back to metres for threshold
        scan_norm = self.obs[:self.n_lidar_beams]
        dmin_m = float(np.min(scan_norm)) * self.lidar_max_range
        if dmin_m < self.safe_lidar_thresh:
            gap = self.safe_lidar_thresh - dmin_m
            r_danger = -self.w_danger * (gap ** 2)
        else:
            r_danger = 0.0

        current_action = np.array([self.robot.v, self.robot.w], dtype=np.float32)
        da = np.linalg.norm(current_action - prev_action)
        r_smooth = -self.w_smooth * da

        # static penalty: penalise if robot barely moved over last 3 s (15 steps)
        self._pos_history.append((self.robot.x, self.robot.z))
        if len(self._pos_history) == self._pos_history.maxlen:
            x0, z0 = self._pos_history[0]
            disp = self._point_dist(self.robot.x, self.robot.z, x0, z0)
            r_stall = -self.w_stall if disp < 1.0 else 0.0
        else:
            r_stall = 0.0

        reward = (
            r_goal_progress
            + r_s_progress
            + r_lateral
            + r_danger
            + r_smooth
            + r_stall
        )

        reached_final_goal = final_goal_dist < self.final_goal_bonus_radius
        success = final_goal_dist < self.final_goal_success_radius
        timeout = self.step_count >= self.max_steps

        if collision:
            reward -= self.collision_penalty

        if rejoined_this_step:
            reward += self.rejoin_bonus

        if reached_final_goal:
            reward += self.final_goal_bonus

        done = collision or success or timeout

        info = {
            "final_goal_dist": final_goal_dist,
            "goal_progress": goal_progress,
            "s_progress": ds_progress,
            "proj_s": self.proj["s"],
            "success": success,
            "reached_final_goal_bonus_radius": reached_final_goal,
            "timeout": timeout,
            "min_lidar": dmin_m,
            "control_mode": self.control_mode,
            "pars_active_this_step": pars_active_this_step,
            "rejoined_this_step": rejoined_this_step,
            "pars_trigger_count": self.pars_trigger_count,
            "rejoin_count": self.rejoin_count,
            "reward_terms": {
                "r_goal_progress": r_goal_progress,
                "r_s_progress": r_s_progress,
                "r_lateral": r_lateral,
                "r_danger": r_danger,
                "r_smooth": r_smooth,
                "r_stall": r_stall,
                "collision_penalty": -self.collision_penalty if collision else 0.0,
                "rejoin_bonus": self.rejoin_bonus if rejoined_this_step else 0.0,
                "final_goal_bonus": self.final_goal_bonus if reached_final_goal else 0.0,
            },
        }
        return reward, done, info

    def reset(self):
        route_file = random.choice(self.route_files)
        self.route = self._load_route(route_file)
        self.ph = PathHelper(self.route["waypoints"])

        if self.dynamic_n_obs:
            L = self.ph.total_length
            n = 1 if L < 10 else (2 if L <= 20 else 3)
            self.n_circle_obs = n
            self.n_rect_obs = max(0, n - 1)  # forced blocking counts as the 1st rect

        s_upper = max(0.5, 0.2 * self.ph.total_length)
        s_start = random.uniform(0.0, s_upper)

        x0, z0 = self.ph.sample_at_s(s_start)
        yaw0 = self._estimate_path_yaw(s_start, ds=0.5)

        x0 += random.uniform(-0.15, 0.15)
        z0 += random.uniform(-0.15, 0.15)
        yaw0 = wrap_angle(yaw0 + random.uniform(-0.15, 0.15))

        self.robot = RobotState(x=x0, z=z0, yaw=yaw0, v=0.0, w=0.0)

        self.obstacles = self._generate_obstacles_along_route(self.route["waypoints"])
        self.obstacles = self._append_forced_blocking_obstacle(self.obstacles, s_start)

        self.step_count = 0
        self.prev_action = np.zeros(2, dtype=np.float32)

        gx, _, gz = self.route["waypoints"][-1]
        self.final_goal_x = float(gx)
        self.final_goal_z = float(gz)

        self.pars_trigger_count = 0
        self.rejoin_count = 0
        self._locked_rejoin_point = None
        self._approaching_rejoin = False
        self._pos_history.clear()

        if self.pars_only_mode:
            # Start already in PARS mode — obstacle is pre-placed close enough
            self.control_mode = "PARS"
            self.pars_trigger_count = 1
        else:
            self.control_mode = "PP"

        self.obs = self._get_obs()
        return self.obs

    def step(self, action):
        prev_s = self.proj["s"]
        prev_goal_dist = self._point_dist(
            self.robot.x,
            self.robot.z,
            self.final_goal_x,
            self.final_goal_z,
        )
        prev_action = self.prev_action.copy()

        rejoined_this_step = False
        scan_before = self._lidar_scan()
        pars_active_this_step = False

        if self.control_mode == "PP":
            # Don't re-trigger while approaching the locked rejoin point
            if not self._approaching_rejoin and self._should_trigger_pars(scan_before):
                self.control_mode = "PARS"
                self.pars_trigger_count += 1
                pars_active_this_step = True
                v_cmd, w_cmd = float(action[0]), float(action[1])
            else:
                pp_action = self._pp_action()
                v_cmd, w_cmd = float(pp_action[0]), float(pp_action[1])
        else:  # PARS
            pars_active_this_step = True
            # Takeover when gap is clearly sufficient (larger inflation = stricter clearance)
            line_free = self._inflated_line_is_free(
                self.robot.x, self.robot.z,
                self.rejoin_point["x"], self.rejoin_point["z"],
                self.pp_takeover_inflation,
            )
            if line_free:
                # Immediately switch to PP and lock target
                self._locked_rejoin_point = {
                    "x": self.rejoin_point["x"],
                    "z": self.rejoin_point["z"],
                    "s": self.rejoin_point["s"],
                }
                self._approaching_rejoin = True
                self.control_mode = "PP"
                pars_active_this_step = False
                pp_action = self._pp_action()
                v_cmd, w_cmd = float(pp_action[0]), float(pp_action[1])
            else:
                v_cmd, w_cmd = float(action[0]), float(action[1])

        self.robot.step(v_cmd, w_cmd, self.dt)
        self.step_count += 1

        map_collision = not self.pm.circle_is_free(self.robot.x, self.robot.z, self.robot_radius)
        dyn_collision = self._robot_collides_with_obstacles()
        collision = map_collision or dyn_collision

        self.obs = self._get_obs()  # also updates self.rejoin_point (locked if set)

        # Check arrival at locked rejoin point → give bonus
        if self._approaching_rejoin and self.rejoin_point["dist"] < self.rejoin_dist_thresh:
            rejoined_this_step = True
            self._approaching_rejoin = False
            self._locked_rejoin_point = None
            self.rejoin_count += 1

        reward, done, extra = self._compute_reward(
            prev_s=prev_s,
            prev_goal_dist=prev_goal_dist,
            prev_action=prev_action,
            collision=collision,
            pars_active_this_step=pars_active_this_step,
            rejoined_this_step=rejoined_this_step,
        )

        if self.pars_only_mode:
            # Rejoin → PP takes over, episode continues; ends only on collision/timeout/goal
            done = collision or (self.step_count >= self.max_steps) or extra["success"]

        self.prev_action = np.array([self.robot.v, self.robot.w], dtype=np.float32)

        info = {
            "map_collision": map_collision,
            "dyn_collision": dyn_collision,
            "proj_dist": self.proj["dist"],
            "proj_s": self.proj["s"],
            "final_goal": (self.final_goal_x, self.final_goal_z),
            "rejoin_point": self.rejoin_point,
            "rejoin_success": rejoined_this_step,
            **extra,
        }
        return self.obs, reward, done, info