from typing import Dict, List
from mlagents_envs.side_channel.side_channel import SideChannel
from mlagents_envs.environment import UnityEnvironment

class UnderwaterEnv:
    def __init__(
        self,
        file_name: str
    ):
        self.env = UnityEnvironment(
            file_name
        )

    def reset(self):
        self.env.reset()
    
    def step(self):
        self.env.step()
    
    def close(self):
        self.env.close()