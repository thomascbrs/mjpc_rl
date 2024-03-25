from build_release.libmjpc_rl_pywrap import MujocoSimulator, loadData

import numpy as np
from time import sleep
from time import perf_counter as clock

from envs.BaseEnv import BaseEnv

# from gymnasium import spaces
# from gymnasium.wrappers import FlattenObservation, RescaleAction

if __name__ == "__main__":
    env = BaseEnv(render_mode="h")
    # observation, info = env.reset()
    for k in range(500):
        actions = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(actions)
        # print(observation["ori_ref"])
        if terminated:
            print("Terminated due to collision")
            env.reset()
        if truncated:
            print("Truncated due to end time reached.")
            env.reset()

    # sleep(2.)
    # observation, reward, terminated, truncated, info = env.step(action)
