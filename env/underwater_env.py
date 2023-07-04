from typing import Dict, List
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.side_channel import SideChannel
from mlagents_envs.environment import UnityEnvironment
from scipy.spatial.transform import Rotation as R
import numpy as np

class UnderwaterEnv:
    def __init__(
        self,
        file_name: str,
        worker_id: int,
        base_port: int,
        seed: int,
        no_graphics: bool,
        timeout_wait: int,
        side_channels: List[SideChannel],
        log_folder: str,
        max_steps: int,
        reward_type: str,
        max_reward: float,
        nsubsteps: int = 50,
        behavior_name: str = None,
    ):
        print("Start the environment with worker_id: ", worker_id, " and base_port: ", base_port)
        self.env = UnityEnvironment(file_name=file_name, 
                                    worker_id=worker_id,
                                    base_port=base_port,
                                    seed=seed,
                                    no_graphics=no_graphics,
                                    timeout_wait=timeout_wait,
                                    side_channels=side_channels,
                                    log_folder=log_folder)
        self.env.reset()
        if behavior_name is None:
            behavior_name = list(self.env.behavior_specs)[0]
        else:
            if behavior_name not in self.env.behavior_specs:
                raise RuntimeError(f"Unknown behavior name {behavior_name}")
        self.behavior_name = behavior_name
        self.action_space = self.env.behavior_specs[behavior_name].action_spec
        self.observation_space = self.env.behavior_specs[behavior_name].observation_specs[0]
        self.max_steps = max_steps
        self.action_max = 1.0 # hard coded for now
        self.reward_type = reward_type
        self.max_reward = max_reward
        self.nsubsteps = nsubsteps

    def reset(self):
        self.env.reset()
        return self.step()[0]
    
    def step(self, action=None):
        if action is not None:
            self.env.set_actions(self.behavior_name, self._process_action(action))
        for _ in range(self.nsubsteps):
            self.env.step()
        get_obs = self.get_obs()
        obs = {
            'observation': get_obs['observation'],
            'achieved_goals': get_obs['achieved_goals'],
            'desired_goal': get_obs['desired_goal']
        }
        reward = get_obs['reward']
        is_done = (reward > 0.0)
        info = {
            'is_success': is_done,
        }
        return obs, reward, is_done, info
    
    def close(self):
        self.env.close()

    def get_obs(self):
        decision_steps, terminal_steps = self.env.get_steps(self.behavior_name)
        obs = decision_steps.obs[0]
        achieved_goals = self._process_achieved_goals(obs[0][0:10])
        desired_goal = obs[0][11:18]
        reward = decision_steps.reward[0]
        return {
            'observation': obs[0],
            'achieved_goals': achieved_goals, # multi goals implementation
            'desired_goal': desired_goal,
            'reward': reward
        }
    
    def _goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)
    
    def compute_reward(self, achieved_goal, desired_goal, info=None):
        distance = self._goal_distance(achieved_goal, desired_goal)
        if self.reward_type == "sparse":
            success = -(distance > 0.05).astype(np.float32) # hard coded for now
            success[success == 0] = self.max_reward
            return success
        else:
            success = -distance
            success[success >= -0.05] = self.max_reward # hard coded for now
            return success
            
    def _process_action(self, action):
        action = np.clip(action, -self.action_max, self.action_max)
        action[3] *= np.pi
        action[4] *= np.pi/2
        action[5] = 1.0 if action[5] > 0.0 else 0.0
        action_tuple = ActionTuple(
            continuous=np.reshape(action[0:self.action_space.continuous_size], (1, self.action_space.continuous_size)),
            discrete=np.reshape(action[self.action_space.continuous_size:], (1, self.action_space.discrete_size))
        )
        return action_tuple

    def _process_achieved_goals(self, achieved_goal):
        pos = achieved_goal[0:3]
        rot = R.from_euler('xyz', achieved_goal[3:6], degrees=False)
        # generate the desired goals by rotating the rot (in radian) around z-axis
        achieved_goals = []
        for i in np.arange(0, 2*np.pi, np.pi/2):
            r = R.from_euler('z', i, degrees=False)
            r = r * rot
            achieved_goals.append(np.concatenate((pos, r.as_euler('xyz', degrees=False), [1.0])))

        # generate the desired goals by rotating the rot (in radian) around x-axis 90 degrees and z-axis 4 times
        r_x_90 = R.from_euler('x', np.pi/2, degrees=False)
        rot_x_90 = r_x_90 * rot
        for i in np.arange(0, 2*np.pi, np.pi/2):
            r = R.from_euler('z', i, degrees=False)
            r = r * rot_x_90
            achieved_goals.append(np.concatenate((pos, r.as_euler('xyz', degrees=False), [1.0])))

        # generate the desired goals by rotating the rot_x_90 (in radian) around y-axis 90 degrees
        r_y_90 = R.from_euler('y', np.pi/2, degrees=False)
        rot_y_90_x_90 = r_y_90 * rot_x_90
        achieved_goals.append(np.concatenate((pos, rot_y_90_x_90.as_euler('xyz', degrees=False), [1.0])))

        # generate the desired goals by rotating the rot_x_90 (in radian) around y-axis -90 degrees
        r_y_neg_90 = R.from_euler('y', -np.pi/2, degrees=False)
        rot_y_neg_90_x_90 = r_y_neg_90 * rot_x_90
        achieved_goals.append(np.concatenate((pos, rot_y_neg_90_x_90.as_euler('xyz', degrees=False), [1.0])))

        # generate the desired goals by rotating the rot (in radian) around y-axis 90 degrees
        rot_y_90 = r_y_90 * rot
        achieved_goals.append(np.concatenate((pos, rot_y_90.as_euler('xyz', degrees=False), [1.0])))

        # generate the desired goals by rotating the rot (in radian) around y-axis -90 degrees
        rot_y_neg_90 = r_y_neg_90 * rot
        achieved_goals.append(np.concatenate((pos, rot_y_neg_90.as_euler('xyz', degrees=False), [1.0])))

        return achieved_goals

