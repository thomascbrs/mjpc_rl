import gymnasium as gym
from gymnasium.spaces import Box
import numpy as np

from gymnasium import Env
from typing import TypeVar, Generic, Union

from gymnasium.wrappers import FlattenObservation, RescaleAction, NormalizeReward, NormalizeObservation

WrapperObsType = TypeVar("WrapperObsType")
WrapperActType = TypeVar("WrapperActType")
ObsType = TypeVar("ObsType")
ActType = TypeVar("ActType")


class RescaleObservation(gym.ObservationWrapper, gym.utils.RecordConstructorArgs):
    def __init__(self, env: gym.Env, min_observation: Union[float, int, np.ndarray],
                 max_observation: Union[float, int, np.ndarray]):
        """Initializes the :class:`RescaleObservation` wrapper.

        Args:
            env (Env): The environment to apply the wrapper
            min_observation (float, int or np.ndarray): The min values for each observation. This may be a numpy array or a scalar.
            max_observation (float, int or np.ndarray): The max values for each observation. This may be a numpy array or a scalar.
        """
        assert isinstance(env.observation_space,
                          Box), f"expected Box observation space, got {type(env.observation_space)}"
        assert np.less_equal(min_observation, max_observation).all(), (min_observation, max_observation)

        gym.utils.RecordConstructorArgs.__init__(self,
                                                 min_observation=min_observation,
                                                 max_observation=max_observation)
        gym.ObservationWrapper.__init__(self, env)

        self.min_observation = np.zeros(env.observation_space.shape,
                                        dtype=env.observation_space.dtype) + min_observation
        self.max_observation = np.zeros(env.observation_space.shape,
                                        dtype=env.observation_space.dtype) + max_observation

        self.observation_space = Box(
            low=min_observation,
            high=max_observation,
            shape=env.observation_space.shape,
            dtype=env.observation_space.dtype,
        )

    def observation(self, observation):
        """Rescales the observation affinely from [:attr:`min_observation`, :attr:`max_observation`] to the observation space of the base environment, :attr:`env`.

        Args:
            observation: The observation to rescale

        Returns:
            The rescaled observation
        """
        assert np.all(np.greater_equal(observation,
                                       self.env.observation_space.low)), (observation, self.env.observation_space.low)
        assert np.all(np.less_equal(observation,
                                    self.env.observation_space.high)), (observation, self.env.observation_space.high)

        low = self.env.observation_space.low
        high = self.env.observation_space.high
        observation = self.min_observation + (self.max_observation - self.min_observation) * ((observation - low) /
                                                                                              (high - low))
        observation = np.clip(observation, self.min_observation, self.max_observation)
        return observation

    def inverse_observation(self, observation):
        low = self.env.observation_space.low
        high = self.env.observation_space.high
        observation = low + (high - low) * ((observation - self.min_observation) / (self.max_observation - self.min_observation))
        observation = np.clip(observation, low, high)
        return observation


class MultiDiscretizeActionWrapper(gym.ActionWrapper):
    def __init__(self, env, num_discrete_actions_per_dim):
        super(MultiDiscretizeActionWrapper, self).__init__(env)

        assert len(num_discrete_actions_per_dim) == len(
            env.action_space.low), "Discretization should have same state space as continuous"
        self.num_discrete_actions_per_dim = num_discrete_actions_per_dim

        assert isinstance(env.action_space, gym.spaces.Box)

        # This will create a multi-dimensional discrete action space
        self.action_space = gym.spaces.MultiDiscrete(num_discrete_actions_per_dim)

    def action(self, discrete_actions):
        continuous_actions = []
        for i, discrete_action in enumerate(discrete_actions):
            action_low = self.env.action_space.low[i]
            action_high = self.env.action_space.high[i]
            continuous_action = action_low + (discrete_action /
                                              (self.num_discrete_actions_per_dim[i] - 1.0)) * (action_high -
                                                                                               action_low)
            continuous_actions.append(continuous_action)
        return np.array(continuous_actions)


class Wrapper(gym.Wrapper[WrapperObsType, WrapperActType, ObsType, ActType]):
    def __init__(self, env: Env[ObsType, ActType]):
        # Store the initial environment
        self._env2 = env

        # Apply observation wrappers
        self._env2 = FlattenObservation(self._env2)  # Flatten before rescaling (cannot handle dict)
        # self._env2 = NormalizeObservation(self._env2)  # Rescale observation between [-1,1]

        # Apply action wrappers
        self._env2 = RescaleAction(self._env2, -1., 1.)  # Rescale action between [-1,1]

        self._env2 = RescaleObservation(self._env2,-1.,1.)
        # Apply normalize reward
        # self._env2 = NormalizeReward(self._env2)

        # Apply normalize observations
        # self._env2 = NormalizeObservation(self._env2)


        # Discretize the action space.
        # d_gait = [7]
        # d_timings = np.tile([7, 7], 4)
        # d_vel = [11, 11, 11]
        # discretization = np.concatenate([d_gait, d_timings, d_vel])
        # self._env2 = MultiDiscretizeActionWrapper(self._env2, discretization)  # Discretize after rescaling

        # gym.ActionWrapper.__init__(self, self._env)
        super().__init__(self._env2)

    # This will forward all method calls to the underlying environment (i.e., _env)
    def __getattr__(self, name):
        return getattr(self._env2, name)