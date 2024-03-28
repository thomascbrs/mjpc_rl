import numpy as np
import copy
from time import perf_counter as clock

import gymnasium as gym
from gymnasium import spaces
from collections import OrderedDict
import pinocchio

import os

from build_release.libmjpc_rl_pywrap import MujocoSimulator
from envs.environment import Rectangle, create_environment

class BaseEnv(gym.Env):
  metadata = {"render_modes": ["human", "rgb_array", "logger"], "render_fps": 4}

  def __init__(self, render_mode=None, logger=False):
    super().__init__()

    # Action type.
    # lb_vel = np.array([-1., -1., -1.])
    # ub_vel = np.array([1., 1., 1.])

    # lb_ang = np.array([-1., -1., -1.])
    # ub_ang = np.array([1., 1., 1.])

    # self._lb = np.concatenate([lb_vel, lb_ang])
    # self._ub = np.concatenate([ub_vel, ub_ang])

    self._lb = np.array([-0.2,-0.2,-0.4])
    self._ub = np.array([0.5,0.2,0.4])
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
        "height": spaces.Box(-0.1, 2.,shape=(1,), dtype=np.float32),
        "roll": spaces.Box(-3.14, 3.14,shape=(1,), dtype=np.float32),
        "pitch": spaces.Box(-3.14, 3.14,shape=(1,), dtype=np.float32),
        "yaw": spaces.Box(-1., 1.,shape=(2,), dtype=np.float32),
        "collision_status": spaces.Box(0., 1.,shape=(1,), dtype=np.float32),
        "lvel":spaces.Box(-3.,3.,shape=(6,), dtype=np.float32),
        "lvref":spaces.Box(-3.,3.,shape=(3,), dtype=np.float32),
        "roll_ref":spaces.Box(-3.14,3.14,shape=(1,), dtype=np.float32),
        "pitch_ref":spaces.Box(-3.14,3.14,shape=(1,), dtype=np.float32),
        "yaw_ref":spaces.Box(-1.,1.,shape=(2,), dtype=np.float32),
        "heightmap":spaces.Box(-1.,0.,shape=(91,), dtype=np.float32),
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
        "r_task":0.,
        "timestep":0,
        "r_height":0.,
        "r_angle":0.,
        "r_vel":0.,
        "r_control":0.,
        "r_action":0.,
        "envId":0.
    })

    self.infos = dict({
        "t":0.,
        "lgoal":np.zeros(2),
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

    current_dir = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../../mjpc_rl/unitree_a1/task_hill.xml"

    # Construct the absolute path
    filename = os.path.join(current_dir, relative_path)
    # Check if the file exists
    if not os.path.exists(filename):
        toprint = "File does not exist: {}".format(filename)
        raise RuntimeError("File does not exist: {}".format(filename))

    self.RENDERING = (render_mode == "human" )
    self.simulator = MujocoSimulator(2, self.RENDERING, logger, filename)

    self.bias = True

    # Import environement
    self.envId = 0 # start at 0.
    self.counter_failure = 0
    self.counter_success = 0
    self.environments, self.start_zones, self.goal_zones = create_environment()

    self.reset_options = {
      "envId":0,
      "q0":[0.]*6,
      "goal":[0.]*2,
      "mpc":[1.,2.,5.]
    }

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

    lgoal = np.clip(self.infos["lgoal"], -3.5, 3.5, dtype=np.float32)[:2].tolist()

    pose = np.zeros(4)
    pose[:] = obs.filtered_pose[2:]
    pose = np.clip(pose, -2., 2., dtype=np.float32).tolist()

    height = np.clip(obs.filtered_pose[2], -0.1, 2., dtype=np.float32).tolist()
    roll = np.clip(obs.filtered_pose[3], -3.14, 3.14, dtype=np.float32).tolist()
    pitch = np.clip(obs.filtered_pose[4], -3.14, 3.14, dtype=np.float32).tolist()
    yaw = np.clip([np.cos(obs.filtered_pose[5]), np.sin(obs.filtered_pose[5])], -1., 1., dtype=np.float32).tolist()

    roll_ref = np.clip(obs.orientation_ref[0], -3.14, 3.14, dtype=np.float32).tolist()
    pitch_ref = np.clip(obs.orientation_ref[1], -3.14, 3.14, dtype=np.float32).tolist()
    yaw_ref = np.clip([np.cos(obs.orientation_ref[2]), np.sin(obs.orientation_ref[2])], -1., 1., dtype=np.float32).tolist()

    heightmap = np.clip(self.simulator.getHeightmap(), -1.,0., dtype=np.float32)
    observations = {
        "t": np.array([self.infos["t"]],dtype=np.float32),
        "lgoal": np.array(lgoal,dtype=np.float32) ,
        # "contact_state": np.array(contact_state,dtype=np.float32),
        # "lfeet": np.array(lfeet, dtype=np.float32),
        "height": np.array([height], dtype=np.float32),
        "roll": np.array([roll], dtype=np.float32),
        "pitch": np.array([pitch], dtype=np.float32),
        "yaw": np.array(yaw, dtype=np.float32),
        "collision_status": np.array([collision_status], dtype=np.float32),
        "lvel":np.array(lvel, dtype=np.float32),
        "lvref":np.array(lvref, dtype=np.float32),
        "roll_ref":np.array([roll_ref], dtype=np.float32),
        "pitch_ref":np.array([pitch_ref], dtype=np.float32),
        "yaw_ref":np.array(yaw_ref, dtype=np.float32),
        "heightmap":heightmap
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
    R = pinocchio.rpy.rpyToMatrix(0., 0., obs.filtered_pose[5])[:2,:2]
    T = np.array(obs.end_pose)[:2]
    self.infos["lgoal"][:] = R.T @ (self.infos["goal"][:2] - T)

    self.infos["velxy"] = np.array(obs.filtered_vel[:2])

    self.infos["dgoal"] = np.linalg.norm(self.infos["robot_pose"][:2] - self.infos["goal"][:2])
    self.general_infos["dgoal"] = self.infos["dgoal"]

    self.infos["goal_reached"] = self.infos["dgoal"] < 0.2

    # Update general info to terminate episode if necessary
    if obs.collision_status > 0.:
      self.infos["collision_status"] = 1

  def step(self, actions):

    action_6D = [0.]*6
    action_6D[0] = actions[0]
    action_6D[4] = actions[1]
    action_6D[5] = actions[2]
    self.simulator.step(action_6D)

    # Update new infos based on the internal observer.
    self._update_infos()

    reward = 0.
    if self.bias:
      reward += self._reward_bias(1.)
      # reward += self._reward01(0.6)

    # reward += self._reward_stall()
    reward += self._reward_task(Tr=5., T=3.,alpha=1.)
    # reward += self._reward_action(actions, 0.5)
    # reward += self._reward_behaviour()

    # Early termination
    terminated = False
    if self.infos["collision_status"] > 0:
      terminated = True
      alpha = 4
      # Negative penalty when colliding with the ground.
      reward -= 5. * ( (5. - self.infos["t"]) / 5.)
      self.general_infos["r_termination"] = - 5. * ( (5. - self.infos["t"]) / 5.) - 1.5

    truncated = False
    if self.infos["t"] > 5.:
      terminated = True
      self.general_infos["r_termination"] = -1.

    if self.infos["goal_reached"]:
      print("goal reached")
      terminated = True
      reward += 6.
      self.general_infos["r_termination"] = 5.

    observation = self._get_obs()
    info = self._get_info()

    return observation, reward, terminated, truncated, info

  def reset(self, seed=None, options=None):

    assert isinstance(options, dict) or options is None, f"options type is {type(options)}. Should either None or dict."

    # We need the following line to seed self.np_random
    super().reset(seed=seed)

    self.infos["t"] = 0.
    self.general_infos["timestep"] = 0

    # Reset environment around origin.
    # Environment curriculum
    q = [0.]*6
    if self.infos["goal_reached"] :
      # Increase the environement.
      self.counter_success += 1
      if self.counter_success >= 2:
        if self.envId == len(self.environments) - 1 :
          self.envId = self.np_random.integers(0, 5, size=1)[0]
        else:
          self.envId += 1
        self.counter_failure = 0
        self.counter_success = 0
    else:
      self.counter_failure += 1
      if self.counter_failure > 8 and self.envId != 0:
        self.envId -= 1
        self.counter_failure = 0
        self.counter_success = 0

    # For replay purposes. Bypass the curriculum.
    if isinstance(options, dict) and "envId" in options:
      self.envId = options["envId"]

    # Update reset_options_dict to replay the episode
    self.reset_options["envId"] = self.envId

    # Set starting position
    q = [0.]*6
    if isinstance(options, dict) and "q0" in options:
      assert isinstance(options["q0"],list), "q0 attribute should be a list."
      assert len(options["q0"]) == 6, "q0 should be size 6."
      q[:] = options["q0"][:]
    else:
      [[xlim_min, xlim_max],[ylim_min, ylim_max]] = self.start_zones[self.envId].get_boundaries()
      q[0] = xlim_min + (xlim_max - xlim_min) * self.np_random.random()
      q[1] = ylim_min + (ylim_max - ylim_min) * self.np_random.random()
      q[2] = 0.3
      q[4] = -0.1 # Pitch angle
      q[5] = -0.5 + (0.5 + 0.5) * self.np_random.random()
      self.infos["robot_pose"] = np.array(q[:3])
    # Update reset_options_dict to replay the episode
    self.reset_options["q0"][:] = q[:] # copy
    self.simulator.reset(q, self.envId)

    # Reset goal position.
    self.infos["goal"] = np.zeros(6)
    if isinstance(options, dict) and "goal" in options:
      assert isinstance(options["goal"],list), "q0 attribute should be a list."
      assert len(options["goal"]) == 2, "q0 should be size 6."
      self.infos["goal"][0] = options["goal"][0]
      self.infos["goal"][1] = options["goal"][1]
    else:
      [[xlim_min, xlim_max],[ylim_min, ylim_max]] = self.goal_zones[self.envId].get_boundaries()
      self.infos["goal"][0] = xlim_min + (xlim_max - xlim_min) * self.np_random.random()
      self.infos["goal"][1] = ylim_min + (ylim_max - ylim_min) * self.np_random.random()
      # Not get a goal too close to the starting point
      while self.envId == 0 and np.linalg.norm(self.infos["robot_pose"][:2] - self.infos["goal"][:2]) <= 0.3:
        self.infos["goal"][0] = xlim_min + (xlim_max - xlim_min) * self.np_random.random()
        self.infos["goal"][1] = ylim_min + (ylim_max - ylim_min) * self.np_random.random()
    self.infos["goal"][2] = 0.248
    self.infos["collision_status"] = 0
    self.infos["goal_reached"] = False
    self.simulator.update_goal_position(self.infos["goal"].tolist())
    # Update reset_options_dict to replay the episode
    self.reset_options["goal"][0] = self.infos["goal"][0] # copy
    self.reset_options["goal"][1] = self.infos["goal"][1]

    # MPC options
    if isinstance(options, dict) and "mpc" in options:
      assert isinstance(options["mpc"], list), "mpc param in option should be a list"
      assert len(options["mpc"]) == 3, "mpc param should be size 3"
      self.simulator.set_mpc_params(options["mpc"][0],options["mpc"][1],options["mpc"][2])
      self.reset_options["mpc"][:] = options["mpc"][:] # copy

    # Reset general infos
    self.general_infos["r_termination"] = 0
    self.general_infos["dgoal"] = np.linalg.norm(self.infos["robot_pose"][:2] - self.infos["goal"][:2])
    self.general_infos["r_height"] = 0.
    self.general_infos["r_angle"] = 0.
    self.general_infos["r_vel"] = 0.
    self.general_infos["r_control"] = 0.
    self.general_infos["r_task"] = 0.
    self.general_infos["r_action"] = 0.
    self.general_infos["envId"] = self.envId

    self._update_infos()

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
    # print("vel_b : ", vel_b)
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

    # Normalise goal direction vector.
    norm = np.linalg.norm(d_goal)
    if norm > 0.:
      direction_goal = d_goal / np.linalg.norm(d_goal)
    else:
      direction_goal = d_goal

    # Heading vector. Project base velocity along heading vector.
    obs = self.simulator.getObervation()
    direction_heading = np.array([np.cos(obs.filtered_pose[5]), np.sin(obs.filtered_pose[5])])
    value_vel_heading = vel_b.T @ direction_heading
    vel_heading = value_vel_heading * direction_heading

    # Project velocity along goal direction vector.
    vref = 0.6
    vell_diff = vref - vel_heading.T @ direction_goal
    if value_vel_heading < 0:
      reward += value_vel_heading # Moving in oppisite direction of the head.
    elif np.linalg.norm(vel_heading) > 0.05:
      reward += min(vel_heading.T @ direction_goal, vref) / vref
    else:
      pass
    
    reward *= alpha
    reward = np.clip(reward, -alpha,alpha)
    self.general_infos["r_bias"] = reward
    self.general_infos["vel_toward_goal"] = vel_heading.T @ d_goal # Along the goal direction.
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

    r_angle = -0.05 * (obs.sq_angle[0] + obs.sq_angle[1])
    self.general_infos["r_angle"] = r_angle

    # r_vel = -0.001 * np.sum(obs.sq_vel)
    r_vel = -0.02 * (obs.sq_vel[2])
    self.general_infos["r_vel"] = r_vel

    r_control = -0.001 * obs.sq_control[0]
    self.general_infos["r_control"] = r_control

    reward = r_height + r_vel + r_control + r_angle

    return reward

  def _reward_task(self, Tr=5., T=4., alpha=1.):
    """ Task reward to reach the desired location as described in
    https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9981198.
    """
    # Reward on x,y axis.
    if self.infos["t"] > T:
        # reward = (1 / (Tr*self._T_nodes)) / (1 + np.linalg.norm(self._goal[:2] - self._robot_pose[:2], 2))
        reward = alpha / (1 + np.linalg.norm(2 * (self.infos["lgoal"]), 2))
        # reward = (1 / (Tr*self._T_nodes)) * self.function_n(np.linalg.norm(self._goal[:2] - self._robot_pose[:2])  )
        self.general_infos["r_task"] = reward
        return reward
    else:
        self.general_infos["r_task"] = 0.
        return 0.

  def _reward_action(self,actions,alpha = 1.):
    reward = - alpha * np.linalg.norm(actions)
    self.general_infos["r_action"] = reward
    return reward
