from typing import Dict, List
from mlagents_envs.side_channel.side_channel import SideChannel
from mlagents_envs.environment import UnityEnvironment

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
        behavior_name: str = None,
    ):
        self.env = UnityEnvironment(
            file_name,
            worker_id,
            base_port,
            seed,
            no_graphics,
            timeout_wait,
            side_channels,
            log_folder,
        )
        if behavior_name is None:
            behavior_name = list(self.env.behavior_specs)[0]
        else:
            if behavior_name not in self.env.behavior_specs:
                raise RuntimeError(f"Unknown behavior name {behavior_name}")
        self.behavior_name = behavior_name


    def reset(self):
        self.env.reset()
        return self_get_obs()
    
    def step(self):
        self.env.step()
        return self_get_obs()
    
    def close(self):
        self.env.close()

    def self_get_obs(self):
        decision_steps, terminal_steps = self.env.get_steps(self.behavior_name)
        obs = decision_steps.obs
        achieved_goal = obs[0][0]