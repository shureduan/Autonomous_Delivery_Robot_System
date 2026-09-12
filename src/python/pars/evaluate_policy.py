import argparse
from pathlib import Path
import sys
import numpy as np
from stable_baselines3 import SAC

THIS_FILE = Path(__file__).resolve()
sys.path.append(str(THIS_FILE.parent))

from pars_gym_env import PARSGymEnv


SUCCESS_RADIUS_M = 3.0
N_EPISODES = 20
MAX_STEPS_PER_EPISODE = 600


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained PARS policy.")
    parser.add_argument(
        "model_path",
        type=Path,
        help="Path to a locally supplied Stable-Baselines3 SAC model archive.",
    )
    parser.add_argument(
        "--map",
        dest="map_path",
        type=Path,
        required=True,
        help="Occupancy-map NPZ used by the evaluation environment.",
    )
    parser.add_argument(
        "--route",
        dest="route_paths",
        type=Path,
        action="append",
        required=True,
        help="Reference-route JSON; repeat this option to supply multiple routes.",
    )
    args = parser.parse_args()

    env = PARSGymEnv(
        map_npz_path=args.map_path,
        route_files=args.route_paths,
        seed=123,
        dynamic_n_obs=True,
        obstacle_lateral_range=4.5,
        obstacle_min_center_dist=4.0,
    )
    model = SAC.load(str(args.model_path))

    n_success = 0
    n_collision = 0
    n_timeout = 0

    final_dists = []
    ep_lengths = []
    ep_rewards = []

    for ep in range(N_EPISODES):
        obs, info = env.reset()
        total_reward = 0.0

        final_dist = None
        step_count = 0
        collided = False
        timeout = False

        for step in range(MAX_STEPS_PER_EPISODE):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step_count += 1

            inner = env.inner_env
            final_dist = np.hypot(
                inner.robot.x - inner.final_goal_x,
                inner.robot.z - inner.final_goal_z
            )

            if info.get("map_collision", False) or info.get("dyn_collision", False):
                collided = True

            if info.get("timeout", False):
                timeout = True

            if terminated or truncated:
                break
        else:
            # external evaluation cutoff reached
            timeout = True

        success = (
            final_dist is not None
            and final_dist < SUCCESS_RADIUS_M
            and (not collided)
        )

        # mutually exclusive outcome: collision / success / timeout
        if collided:
            outcome = "collision"
            n_collision += 1
        elif success:
            outcome = "success"
            n_success += 1
        else:
            outcome = "timeout"
            n_timeout += 1
            timeout = True

        final_dists.append(float(final_dist if final_dist is not None else 999.0))
        ep_lengths.append(step_count)
        ep_rewards.append(total_reward)

        print(
            f"ep={ep+1:03d} | "
            f"route={info.get('route', 'n/a') if isinstance(info, dict) else 'n/a'} | "
            f"steps={step_count:03d} | "
            f"reward={total_reward:8.2f} | "
            f"final_dist={final_dist:6.2f} | "
            f"outcome={outcome} | "
            f"success={success} | "
            f"collision={collided} | "
            f"timeout={timeout}"
        )

    success_rate = n_success / N_EPISODES
    collision_rate = n_collision / N_EPISODES
    timeout_rate = n_timeout / N_EPISODES

    avg_final_dist = float(np.mean(final_dists))
    avg_ep_len = float(np.mean(ep_lengths))
    avg_ep_reward = float(np.mean(ep_rewards))

    print(f"episodes           : {N_EPISODES}")
    print(f"success rate       : {success_rate:.3f}")
    print(f"collision rate     : {collision_rate:.3f}")
    print(f"timeout rate       : {timeout_rate:.3f}")
    print(f"avg step reward      : {avg_ep_reward/avg_ep_len:.3f}")


if __name__ == "__main__":
    main()
