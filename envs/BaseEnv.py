import numpy as np
import copy
from time import perf_counter as clock

import gymnasium as gym
from gymnasium import spaces
from collections import OrderedDict
import pinocchio

import os

from build_release.libmjpc_rl_pywrap import MujocoSimulator

class BaseEnv(gym.Env):
  metadata = {"render_modes": ["human", "rgb_array", "logger"], "render_fps": 4}

  def __init__(self, render_mode=None):
    super().__init__()

    # Action type.
    # lb_vel = np.array([-1., -1., -1.])
    # ub_vel = np.array([1., 1., 1.])

    # lb_ang = np.array([-1., -1., -1.])
    # ub_ang = np.array([1., 1., 1.])

    # self._lb = np.concatenate([lb_vel, lb_ang])
    # self._ub = np.concatenate([ub_vel, ub_ang])

    self._lb = np.array([-0.5,-0.5])
    self._ub = np.array([0.5,0.5])
    self.action_space = spaces.Box(low=self._lb, high=self._ub, dtype=np.float32)

    # Observation.
    lb = np.zeros(4)
    ub = np.ones(4)
     # TODO: in local frame wrt shoulder position ?
    lb_feet = np.tile([-0.4, -0.4, -0.4], 4)
    ub_feet = np.tile([0.4, 0.4, 0.4], 4)
    self.observation_space = spaces.Dict({
        "t":spaces.Box(0.,10.,shape=(1,), dtype=np.float32),
        "lgoal": spaces.Box(-3.5, 3.5, shape=(2, ), dtype=np.float32),
        # "contact_state": spaces.Box(lb, ub, dtype=np.float32),
        # "lfeet": spaces.Box(lb_feet, ub_feet, dtype=np.float32),
        "pose": spaces.Box(-2., 2.,shape=(4,), dtype=np.float32),
        "collision_status": spaces.Box(0., 1.,shape=(1,), dtype=np.float32),
        "lvel":spaces.Box(-3.,3.,shape=(6,), dtype=np.float32),
        "lvref":spaces.Box(-3.,3.,shape=(3,), dtype=np.float32),
        "ori_ref":spaces.Box(-3.14,3.14,shape=(3,), dtype=np.float32)
        # "time":spaces.Box(0.,11., dtype=np.float32),
        # "vel_b":spaces.Box(-3.,3.,shape=(3,), dtype=np.float32)
    })

    # Informations for tensoarboard callbacks.
    self.general_infos = dict({
        "dgoal": 0.,
        "r_dgoal": 0.,
        "r_bias": 0.,
        "vel_toward_goal": 0.,
        "r_termination": 0.,
        "timestep":0,
        "r_height":0.,
        "r_angle":0.,
        "r_vel":0.,
        "r_control":0.
    })

    self.infos = dict({
        "t":0.,
        "lgoal":np.zeros(3),
        "robot_pose":np.zeros(3),
        "goal":np.zeros(3),
        "velxy":np.zeros(2),
        "collision_status":0,
        "goal_reached":False
    })

    self.shoulder = {
        'HL': [0.18, -0.133, 0.],
        'HR': [0.18, 0.133, 0.],
        'FL': [-0.18, -0.133, 0.],
        'FR': [-0.18, 0.133, 0.]
    }

    self.feet_names = ["FL", "FR", "HL", "HR"] # Order matter in observation.

    # filename = "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml"
    current_dir = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../../mjpc_rl/unitree_a1/task_hill.xml"

    # Construct the absolute path
    filename = os.path.join(current_dir, relative_path)
    self.RENDERING = (render_mode == "human" )
    self.simulator = MujocoSimulator(1, self.RENDERING, False, filename)

    self.bias = True

    self.reset()

  def _get_info(self):
    return self.general_infos

  def _get_obs(self):

    # Observations from simulator
    obs = self.simulator.getObervation()

    # Feet position in local frame
    lfeet_pos = []
    for name in self.feet_names:
      lfeet_pos.extend(obs.lfeet_pos[name])
    lfeet = np.clip(lfeet_pos, -0.4,0.4).tolist()

    # Goal position in local frame. Using filtered end poisiton.
    R = pinocchio.rpy.rpyToMatrix(obs.filtered_pose[3], obs.filtered_pose[4], obs.filtered_pose[5])
    T = np.array(obs.end_pose)[:3]

    # Velocity in local frame. Using filtered end velocity.
    wvel = np.array(obs.filtered_vel[:])
    lvel = np.zeros([6])
    lvel[:2] = R[:2,:2] @ wvel[:2]
    lvel[2] = wvel[2] # Only x,y in local frame.
    lvel[3:] = wvel[3:]
    lvel = np.clip(lvel, -3., 3., dtype=np.float32).tolist()

    # heightmap =  np.clip(self.environment.get_observation(),0.,1.,dtype=np.float32) # no need to clip.
    contact_state = [status for status in obs.contact_status.values()]
    contact_state = np.clip(contact_state, 0.,1.).tolist()
    collision_status = np.clip(self.infos["collision_status"], 0.,1.)

    lvref = np.clip(obs.lvref, -3., 3., dtype=np.float32).tolist()
    ori_ref = np.clip(obs.orientation_ref, -3.14, 3.14, dtype=np.float32).tolist()

    lgoal = np.clip(self.infos["lgoal"], -3.5, 3.5, dtype=np.float32)[:2].tolist()

    pose = np.zeros(4)
    pose[:] = obs.filtered_pose[2:]
    pose = np.clip(pose, -2., 2., dtype=np.float32).tolist()

    observations = {
        "t": np.array([self.infos["t"]],dtype=np.float32),
        "lgoal": np.array(lgoal,dtype=np.float32) ,
        # "contact_state": np.array(contact_state,dtype=np.float32),
        # "lfeet": np.array(lfeet, dtype=np.float32),
        "pose":np.array(pose, dtype=np.float32),
        "collision_status": np.array([collision_status], dtype=np.float32),
        "lvel":np.array(lvel, dtype=np.float32),
        "lvref":np.array(lvref, dtype=np.float32),
        "ori_ref":np.array(ori_ref, dtype=np.float32),
        # "time":np.array([self.infos["t"] * 0.01], dtype=np.float32),
        # "gait_info":np.array(self.gait_info, dtype=np.float32),
        # "heightmap":heightmap
    }

    # Assertion for observation space validation
    assert self.observation_space.contains(observations), f"Observations are outside the defined observation space : {observations}"

    return observations

  def _update_infos(self):

    self.infos["t"] += 0.24
    self.general_infos["timestep"] += 1

    obs = self.simulator.getObervation()

    self.infos["robot_pose"][:] = obs.filtered_pose[:3]

    # Goal position in local frame. Using filtered end poisiton.
    R = pinocchio.rpy.rpyToMatrix(obs.filtered_pose[3], obs.filtered_pose[4], obs.filtered_pose[5])
    T = np.array(obs.end_pose)[:3]
    self.infos["lgoal"] = R.T @ (self.infos["goal"][:3] - T)

    self.infos["velxy"] = np.array(obs.filtered_vel[:2])

    self.infos["dgoal"] = np.linalg.norm(self.infos["robot_pose"][:2] - self.infos["goal"][:2])
    self.general_infos["dgoal"] = self.infos["dgoal"]

    self.infos["goal_reached"] = self.infos["dgoal"] < 0.15

    # Update general info to terminate episode if necessary
    if obs.collision_status > 0.:
      self.infos["collision_status"] = 1

  def step(self, actions):

    action_6D = [0.]*6
    action_6D[0] = actions[0]
    action_6D[5] = actions[1]
    self.simulator.step(action_6D)

    # Update new infos based on the internal observer.
    self._update_infos()

    observation = self._get_obs()
    info = self._get_info()
    reward = 0.
    if self.bias:
      reward += self._reward_bias(2.5)
      # reward += self._reward01(0.6)

    reward += self._reward_stall()
    # reward += self._reward_behaviour()

    # Early termination
    terminated = False
    if self.infos["collision_status"] > 0:
      terminated = True
      # Negative penalty when colliding with the ground.
      reward -= 5. * ( (5. - self.infos["t"]) / 5.)
      self.general_infos["r_termination"] = - 5. * ( (5. - self.infos["t"]) / 5.)

    truncated = False
    if self.infos["t"] > 5.:
      truncated = True

    if self.infos["goal_reached"]:
      terminated = True
      reward += 4.
      self.general_infos["r_termination"] = 4.

    return observation, reward, terminated, truncated, info

  def reset(self, seed=None, options=None):

    assert isinstance(options, dict) or options is None, f"options type is {type(options)}. Should either None or dict."

    # We need the following line to seed self.np_random
    super().reset(seed=seed)

    self.infos["t"] = 0.
    self.general_infos["timestep"] = 0

    # Reset environment around origin.
    q = [0.]*6
    q[2] = 0.3
    q[4] = -0.1 # Pitch angle
    # q[5] = 1.9
    self.infos["robot_pose"] = np.zeros(3)
    self.simulator.reset(q)

    # Reset goal position.
    self.infos["goal"] = np.zeros(6)
    self.infos["goal"][0] = 1.5 + (2. - 1.5) * self.np_random.random()
    self.infos["goal"][1] = 0.
    self.infos["goal"][2] = 0.248
    self.infos["collision_status"] = 0
    self.infos["goal_reached"] = False

    # Reset general infos
    self.general_infos["r_termination"] = 0
    self.general_infos["dgoal"] = np.linalg.norm(self.infos["robot_pose"][:2] - self.infos["goal"][:2])
    self.general_infos["r_height"] = 0.
    self.general_infos["r_angle"] = 0.
    self.general_infos["r_vel"] = 0.
    self.general_infos["r_control"] = 0.

    observation = self._get_obs()
    info = self._get_info()

    if self.render_mode == "human" or self.render_mode == "logger":
      self._render_frame()

    return observation, info

  def _render_frame(self):
    pass

  def _reward_bias(self, alpha=1.):
    """ Reward term encouraging exploration at the beginning of training as decribed in
    https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9981198.
    """
    reward = 0
    # Approximate velocity on x,y.
    d_goal = self.infos["goal"][:2] - self.infos["robot_pose"][:2]
    vel_b = self.infos["velxy"]
    # TODO : Which velocity to use ?

    # From ETH paper.
    # norm_velb = np.linalg.norm(vel_b)
    # norm = norm_velb * np.linalg.norm(d_goal)
    # reward = 0.
    # if norm_velb > 0.1:
    #     reward = np.clip(alpha * (vel_b.T @ d_goal) / norm, -1.,1.)

    # From Extreme parkour paper.
    # vref = 0.25
    # direction_vec = d_goal / np.linalg.norm(d_goal)
    # vell_diff = vref - vel_b.T @ direction_vec
    # # if np.linalg.norm(vel_b) > 0.05:
    # reward += min(vel_b.T @ direction_vec, vref)
    vref = 0.6
    direction_vec = d_goal / np.linalg.norm(d_goal)
    vell_diff = vref - vel_b.T @ direction_vec
    if np.linalg.norm(vel_b) > 0.05:
        reward += min(vel_b.T @ direction_vec, vref) / vref

    self.general_infos["r_bias"] = reward
    self.general_infos["vel_toward_goal"] = vel_b.T @ d_goal # Along the goal direction.
    return reward

  def _reward01(self, alpha=1.):
    """ Positive reward for tracking a reference velocity.
    """
    reward = 0
    # TODO : Avoid using getObservation, make common interface instead
    l_goal = self.infos["lgoal"]
    yaw_diff = np.arctan2(l_goal[1], l_goal[0])
    r =  - (alpha)  * (1 - np.exp(- 5 * yaw_diff**2))
    # r =  alpha * np.exp(- 5 * yaw_diff**2)
    self.general_infos["r_yaw_track"] = r
    reward += r

    return reward

  def _reward_stall(self):
    """ Penalty waiting while being far away from the target as described in
    https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9981198.
    """
    reward = 0.
    # Approximate velocity on x,y.
    if np.linalg.norm(self.infos["velxy"]) <= 0.1 and self.infos["dgoal"] > 0.5:
      if self.infos["t"] > 0.48: # Not negative during 2 first rewards
        reward = -1.
    self.general_infos["r_stall"] = reward
    return reward

  def _reward_behaviour(self):
    """ Penalty based on sum of squared informations accumulated during the step.
    """
    obs = self.simulator.getObervation()

    r_height = -0.1 * obs.sq_height[0]
    self.general_infos["r_height"] = r_height

    r_angle = -0.1 * (obs.sq_angle[0] + obs.sq_angle[1] + obs.sq_angle[2])
    self.general_infos["r_angle"] = r_angle

    r_vel = -0.01 * np.sum(obs.sq_vel)
    self.general_infos["r_vel"] = r_vel

    r_control = -0.01 * obs.sq_control[0]
    self.general_infos["r_control"] = r_control

    reward = r_height + r_angle + r_vel + r_control

    return reward

  # def _reward_task(self, Tr=4., T=2.4, alpha=1.):
  #   """ Task reward to reach the desired location as described in
  #   https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9981198.
  #   """
  #   # Reward on x,y axis.
  #   if self.infos["t"] > T:
  #       # reward = (1 / (Tr*self._T_nodes)) / (1 + np.linalg.norm(self._goal[:2] - self._robot_pose[:2], 2))
  #       reward = alpha / (1 + np.linalg.norm(2 * (self.infos["lgoal"]), 2))
  #       # reward = (1 / (Tr*self._T_nodes)) * self.function_n(np.linalg.norm(self._goal[:2] - self._robot_pose[:2])  )
  #       self.general_infos["r_task"] = reward
  #       return reward
  #   else:
  #       self.general_infos["r_task"] = 0.
  #       return 0.
