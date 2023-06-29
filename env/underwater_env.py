from typing import Dict, List
from mlagents_envs.side_channel.side_channel import SideChannel
from mlagents_envs.environment import UnityEnvironment

class UnderwaterEnv:
    def __init__(
        self,
        file_name: str,
        worker_id: int = 0,
        base_port: int = 0,
        seed: int = 0,
        no_graphics: bool = False,
        timeout_wait: int = 60,
        additional_args: List[str] = None,
        side_channels: List[SideChannel] = None,
        log_folder: str = None,
        num_areas: int = 1
    ):
        self.env = UnityEnvironment(
            file_name,
            worker_id,
            base_port,
            seed,
            no_graphics,
            timeout_wait,
            additional_args,
            side_channels,
            log_folder,
            num_areas
        )

    def reset(self):
        self.env.reset()
        return self.get_steps()
    
    def step(self, action):
        self.env.set_actions(action)
        self.env.step()
        return self.get_steps()
    
    def get_steps(self):
        decision_steps, terminal_steps = self.env.get_steps()
        return decision_steps, terminal_steps