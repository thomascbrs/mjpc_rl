# from envs.BaseEnv import BaseEnv
from envs.BaseEnv import BaseEnv
from envs.wrapper import Wrapper
from time import perf_counter as clock

import gymnasium as gym
from gymnasium import spaces
from gymnasium.spaces import Box
from gymnasium.wrappers import FlattenObservation, RescaleAction, NormalizeReward
from stable_baselines3 import PPO
import torch.nn as nn

# import flax.linen as nn
# from sbx import PPO,SAC

from stable_baselines3.common.callbacks import BaseCallback
import jax
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.utils import set_random_seed

import numpy as np

def make_env(env_id, rank, seed=0, mode="rgb_array"):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param rank: (int) index of the subprocess
    :param seed: (int) the inital seed for RNG
    :return: (callable)
    """
    def _init():
        base = BaseEnv(render_mode = mode)
        env = Wrapper(base)
        # env.seed(seed + rank)
        return env
    return _init

class TensorboardCallback(BaseCallback):
    """
    Custom callback for plotting additional values in tensorboard.
    """

    def __init__(self,  total_timesteps = 1000000, n_save = 1000, log_path="logs/models/model_" , verbose=2):
        super().__init__(verbose)
        self.mean_len_ep = 0.
        self.N_ = 0.
        # self.t0 = clock()
        # self.t1 = clock()
        # self.num_timesteps_previous = 0

        # Save purpose.
        self.total_timesteps = total_timesteps
        self.n_save = n_save
        self.k_ = 1
        self.log_path = log_path

        # Mean r_task
        self.r_task_mean = 0.
        self.N_task = 0

    def _on_step(self) -> bool:

        envId = []
        # Log scalar value (here a random variable)
        for i in range(len(self.locals.get("infos"))):
            for key,value in self.locals.get("infos")[i].items():
                self.logger.record("infos/" + key, value)
                if key == "envId":
                    envId.append(value)
        for i in range(len(self.locals.get("rewards"))):
            self.logger.record("infos/" + "rewards", self.locals.get("rewards")[i])

        # get envId mean:
        # self.logger.record("infos/" + "envId", np.mean(envId))

        # Get average number of steps.
        for i,done in enumerate(self.locals.get("dones")):
            if done:
                self.mean_len_ep = 1/(self.N_+1) * (self.N_ * self.mean_len_ep + self.locals.get("infos")[i].get("timestep"))
                self.N_ += 1.
        self.logger.record("infos/" + "mean_len_episode", self.mean_len_ep)

        # if self.num_timesteps > 100000 and self.num_timesteps < 100500 :
        #     # Sure to catch event with high number of cpus
        #     self.training_env.env_method("update_curriculum", 3)

        # self.r_task_mean = 1/(self.N_task +1) * (self.N_task * self.r_task_mean + self.locals.get("infos")[i].get("r_task"))
        # self.logger.record("infos/" + "r_task_mean",self.r_task_mean)

        # Saving model.
        if self.num_timesteps > self.k_ * int(self.total_timesteps / self.n_save):
            filename = self.log_path + str(self.k_)
            print(filename)
            print(self.k_)
            print(self.num_timesteps)
            print("\n\n")
            self.model.save(filename)
            self.k_ += 1

        return True

def main(num_cpu=1, mode ="rgb_array", timesteps = 20000, model_log="logs/models/model_1", n_steps = 2048):

    if num_cpu > 1:
        # Multiprocessing : Create the vectorized environment
        env = SubprocVecEnv([make_env('YourCustomEnv-v0', i, mode) for i in range(num_cpu)])
        # env = DummyVecEnv([make_env('YourCustomEnv-v0', i, mode) for i in range(num_cpu)])
    else:
        base = BaseEnv(render_mode = mode)
        env = Wrapper(base)

    # Checking the env.
    # from stable_baselines3.common.env_checker import check_env
    # check = check_env(env)

    policy_kwargs = dict(
    net_arch=[128, 64],  # This specifies the size of the MLP (number of units for each layer).
    # activation_fn=nn.elu # SBX
    activation_fn=nn.ELU # torch
    )
    learning_rate=0.0003
    n_steps= 16
    batch_size= int((n_steps * num_cpu) / 4)
    n_epochs=5
    gamma=0.99
    gae_lambda=0.95
    clip_range=0.2
    target_kl = None
    clip_range_vf=None
    normalize_advantage=True
    ent_coef=0.02
    vf_coef=0.5
    max_grad_norm=0.5
    use_sde=False

    tensorboard_log = "logs/tensorboard/"
    model = PPO("MlpPolicy", env, verbose=1, 
                tensorboard_log=tensorboard_log,
                learning_rate=learning_rate,
                clip_range=clip_range,
                n_epochs=n_epochs,
                batch_size=batch_size,
                ent_coef=ent_coef,
                vf_coef=vf_coef, 
                n_steps = n_steps,
                gamma=gamma,
                policy_kwargs=policy_kwargs,
                gae_lambda=gae_lambda,
                clip_range_vf=clip_range_vf,
                normalize_advantage=normalize_advantage,
                target_kl=target_kl,
                use_sde = use_sde,
                max_grad_norm=max_grad_norm,
                device="cuda")
    # model = SAC("MlpPolicy", env, verbose=1, buffer_size=1000000, tensorboard_log=tensorboard_log, device="cpu")
    model.learn(total_timesteps=timesteps, callback=TensorboardCallback(timesteps, int(timesteps / 4000), model_log))
    model.save(model_log)

if __name__ == '__main__':
    # Optional, but necessary if you want to produce an executable
    # freeze_support()

    # Parameters of training.
    num_cpu = 32  # Nb of processes to use (nb * 4, mpc uses 4 cpus).
    mode = "" # or mode = "human"
    timesteps = 4000000
    # n_steps = int(2048 / num_cpu)
    # n_steps = int(512 / num_cpu)
    n_steps = int(1024 / num_cpu)
    model_log = "logs/models_stream/model_"

    # train
    main(num_cpu, mode, timesteps, model_log, n_steps)
