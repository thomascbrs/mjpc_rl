import numpy as np
import copy
from time import perf_counter as clock

import gymnasium as gym
from gymnasium import spaces
from collections import OrderedDict
import pinocchio

from build_release.libmjpc_rl_pywrap import MujocoSimulator

class BaseEnv(gym.Env):
  metadata = {"render_modes": ["human", "rgb_array", "logger"], "render_fps": 4}

  def __init__(self, render_mode=None):
    super().__init__()

    # Action type.
    lb_vel = np.array([-1., -1., -1.])
    ub_vel = np.array([1., 1., 1.])

    lb_ang = np.array([-1., -1.])
    ub_ang = np.array([1., 1.])

    self._lb = np.concatenate([lb_vel, lb_ang])
    self._ub = np.concatenate([ub_vel, ub_ang])
    self.action_space = spaces.Box(low=self._lb, high=self._ub, dtype=np.float32)

    # Observation.
    lb = np.zeros(4)
    ub = np.ones(4)
     # TODO: in local frame wrt shoulder position ?
    lb_feet = np.tile([-0.4, -0.4, -0.4], 4)
    ub_feet = np.tile([0.4, 0.4, 0.4], 4)
    self.observation_space = spaces.Dict({
        "lgoal": spaces.Box(-3.5, 3.5, shape=(2, ), dtype=np.float32),
        "contact_state": spaces.Box(lb, ub, dtype=np.float32),
        "lfeet": spaces.Box(lb_feet, ub_feet, dtype=np.float32),
        "collision_status": spaces.Box(0., 1.,shape=(1,), dtype=np.float32),
        "lvel":spaces.Box(-3.,3.,shape=(6,), dtype=np.float32),
        "lvref":spaces.Box(-3.,3.,shape=(3,), dtype=np.float32),
        "ori_ref":spaces.Box(-3.14,3.14,shape=(3,), dtype=np.float32)
        # "time":spaces.Box(0.,11., dtype=np.float32),
        # "vel_b":spaces.Box(-3.,3.,shape=(3,), dtype=np.float32)
    })

    # Informations for tensoarboard callbacks.
    self.general_infos = dict({
        "v_dgoal": 0.,
        "r_termination": 0.
    })

    self.infos = dict({
        "t":0.,
        "robot_pose_previous":np.zeros(6),
        "robot_pose":np.zeros(6),
        "robot_velocity":np.zeros(6),
        "goal":np.zeros(6),
        "dgoal":0.,
        "goal_reached":False,
        "velxy":np.zeros(2),
        "vref":np.zeros(6),
        "lgoal":np.zeros(6)
    })

    self.shoulder = {
        'HL': [0.18, -0.133, 0.],
        'HR': [0.18, 0.133, 0.],
        'FL': [-0.18, -0.133, 0.],
        'FR': [-0.18, 0.133, 0.]
    }

    self.feet_names = ["FL", "FR", "HL", "HR"] # Order matter in observation.


    filename = "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml"
    self.RENDERING = (render_mode == "human" )
    self.simulator = MujocoSimulator(1, self.RENDERING, False, filename)

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
    lgoal = R.T @ (self.infos["goal"][:3] - T)
    lgoal = np.clip(lgoal, -3.5, 3.5, dtype=np.float32)[:2].tolist()

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
    collision_status = np.clip(obs.collision_status, 0.,1.)

    lvref = np.clip(obs.lvref, -3., 3., dtype=np.float32).tolist()
    ori_ref = np.clip(obs.orientation_ref, -3.14, 3.14, dtype=np.float32).tolist()

    observations = {
        "lgoal": np.array(lgoal,dtype=np.float32) ,
        "contact_state": np.array(contact_state,dtype=np.float32),
        "lfeet": np.array(lfeet, dtype=np.float32),
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

  def step(self):

    observation = self._get_obs()
    info = self._get_info()
    reward = 0.
    terminated = False
    truncated = False

    return observation, reward, terminated, truncated, info

  def reset(self, seed=None, options=None):

    assert isinstance(options, dict) or options is None, f"options type is {type(options)}. Should either None or dict."

    # We need the following line to seed self.np_random
    super().reset(seed=seed)

    # Reset environment around origin.
    q = [0.]*6
    q[2] = 0.3
    # q[5] = 1.9
    self.simulator.reset(q)

    # Reset goal position.
    self.infos["goal"] = np.zeros(6)
    self.infos["goal"][0] = 1.5 + (2. - 1.5) * self.np_random.random()
    self.infos["goal"][1] = 0.
    self.infos["goal"][2] = 0.248

    observation = self._get_obs()
    info = self._get_info()

    if self.render_mode == "human" or self.render_mode == "logger":
      self._render_frame()

    return observation, info
