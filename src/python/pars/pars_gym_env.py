import numpy as np
import gymnasium as gym
from gymnasium import spaces

from pars_env import PARSEnv


class PARSGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        map_npz_path,
        route_files,
        seed: int | None = None,
        dynamic_n_obs: bool = False,
        obstacle_lateral_range: float = 5.0,
        obstacle_min_center_dist: float = 5.0,
    ):
        super().__init__()

        self.inner_env = PARSEnv(
            map_npz_path=map_npz_path,
            route_files=route_files,
            pars_only_mode=True,
            dynamic_n_obs=dynamic_n_obs,
            obstacle_lateral_range=obstacle_lateral_range,
            obstacle_min_center_dist=obstacle_min_center_dist,
            seed=seed,
        )

        # 21 lidar beams + 8 extra features = 29
        obs_dim = 29

        # action = [v_cmd, w_cmd]
        # 先用简单范围：
        # v in [0, 1]
        # w in [-1, 1]
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([0.8, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32,
        )

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)

        obs = self.inner_env.reset()
        obs = np.asarray(obs, dtype=np.float32)

        info = {
            "route": f"{self.inner_env.route['start_id']} -> {self.inner_env.route['end_id']}",
            "num_obstacles": len(self.inner_env.obstacles),
        }
        return obs, info

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)

        obs, reward, done, info = self.inner_env.step(action)
        obs = np.asarray(obs, dtype=np.float32)
        reward = float(reward)

        terminated = bool(done)
        truncated = False

        return obs, reward, terminated, truncated, info
