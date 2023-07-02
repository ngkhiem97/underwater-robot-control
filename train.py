from env.underwater_env import UnderwaterEnv
from rl_modules.ddpg_agent import ddpg_agent
from arguments import get_args
import os

MAX_TIMESTEPS = 100 # hard coded for now

if __name__ == "__main__":
    # take the configuration for the HER
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ['MKL_NUM_THREADS'] = '1'
    os.environ['IN_MPI'] = '1'

    args = get_args()
    env = UnderwaterEnv(file_name=args.file_name, 
                        worker_id=0, 
                        base_port=5005, 
                        seed=1, 
                        no_graphics=True, 
                        timeout_wait=60, 
                        side_channels=[],
                        log_folder='logs/', 
                        max_steps=MAX_TIMESTEPS, 
                        behavior_name=None,
                        reward_type=args.reward_type)
    obs = env.reset()
    env_params = {
        'obs': obs['observation'].shape[0],
        'goal': obs['desired_goals'][0].shape[0],
        'action': env.action_space.continuous_size + env.action_space.discrete_size,
        'action_max': env.action_max, # hard coded for now
        'max_timesteps': env.max_steps, # hard coded for now
    }
    ddpg_ag = ddpg_agent(args, env, env_params)
    ddpg_ag.learn()