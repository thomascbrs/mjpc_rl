from build_release.libmjpc_rl_pywrap import MujocoSimulator, loadData

import numpy as np
from time import sleep
from time import perf_counter as clock
import copy
import os

from envs.BaseEnv import BaseEnv
from envs.wrapper import Wrapper
from scripts.plots import *

from stable_baselines3 import PPO
# from sbx import PPO
from gymnasium import spaces

current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "../../mjpc_rl/logs/models_stream/model_30.zip"

# Construct the absolute path
filename = os.path.join(current_dir, relative_path)
# Check if the file exists
if not os.path.exists(filename):
    error = "File does not exist: {}".format(filename)
    raise RuntimeError(error)
model = PPO.load(filename)

base = BaseEnv(render_mode="human", logger=True)
wrapper_env = Wrapper(base)

obs, info = wrapper_env.reset()
done = False

list_cos = []
list_sin = []
list_cos_ref = []
list_sin_ref = []
yaw_ref = []
yaw_filt = []
yaw = []


def create_r_logger():
    return {
        "reward": [],
        "dgoal": [],
        "r_dgoal": [],
        "r_bias": [],
        "vel_toward_goal": [],
        "r_termination": [],
        "r_task": [],
        "timestep": [],
        "r_height": [],
        "r_angle": [],
        "r_vel": [],
        "r_control": [],
        "r_stall": [],
    }


def register_rewards(r_logger, reward, info):
    r_logger["reward"].append(reward)
    for key, value in info.items():
        if key in r_logger.keys():
            r_logger[key].append(value)


def do_step(obs):
    action, _ = model.predict(
        obs, deterministic=True)  # Use deterministic=True for deterministic actions according to the policy
    observation, reward, terminated, truncated, info = wrapper_env.step(action)

    return observation, reward, terminated, truncated, info, action


def run_episode(options=None):
    rewards = []
    actions = []
    infos = []
    obs, info = wrapper_env.reset(options=options)
    done = False
    truncated = False

    r_logger = create_r_logger()

    while done == False and truncated == False and done == False:
        print("\n----")
        obs, reward, done, truncated, info, action = do_step(obs)
        print("reward : ", reward)
        print("done : ", done)
        print("action : ", action)
        print_obs(obs)
        print_dict(info)
        # actions.append(action)
        # infos.append(copy.deepcopy(info))
        # rewards.append(reward)
        register_rewards(r_logger, reward, info)
        print("----\n")

    return r_logger


def unscale_obs(obs):
    # obs_unscale = spaces.unflatten(wrapper_env.observation_space, obs)
    obs_unscale = wrapper_env._env2.inverse_observation(obs)
    return spaces.unflatten(wrapper_env.unwrapped.observation_space, obs_unscale)


def print_dict(dt):
    for key in dt.keys():
        toprint = key + " = " + str(dt[key])
        print(toprint)


def print_obs(obs):
    obs_un = unscale_obs(obs)
    print_dict(obs_un)


def plot_reward(r_logger):
    import matplotlib.pyplot as plt
    plt.ion()
    plt.figure()

    for key, value in r_logger.items():
        if key != "timestep":
            plt.plot(value, label=key)
    plt.legend()


if __name__ == "__main__":

    # obs, info = wrapper_env.reset(options=None)
    # obs, reward, done, truncated, info, action = do_step(obs)

    r_logger = run_episode()

    # Plotting the logged informations
    data = wrapper_env.unwrapped.simulator.getLoggerData()
    import matplotlib.pyplot as plt
    # plot_contact_MPCs(data)
    # plot_contact(data)
    # plot_velocity(data)
    # plot_state_filter(data)
    # plot_state_filterEval(data,wrapper)
    # plot_contact_forces(data)
    # plot_state_simple(data)
    # plot_state_mpc(data)
    # plot_angular_velocities_MPCs(data)
    # plot_angular_position_MPCs(data)
