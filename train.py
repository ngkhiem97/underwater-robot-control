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
    no_graphics = True if args.headless else False
    env = UnderwaterEnv(file_name=args.file_name, 
                        worker_id=0, 
                        base_port=None, 
                        seed=args.seed, 
                        no_graphics=no_graphics, 
                        timeout_wait=60, 
                        side_channels=[],
                        log_folder='logs/', 
                        max_steps=MAX_TIMESTEPS, 
                        behavior_name=None,
                        reward_type=args.reward_type,
                        max_reward=args.max_reward,
                        nsubsteps=args.nsubsteps)
    # stop for 5 seconds to wait for the environment to be ready
    obs = env.get_obs()
    env_params = {
        'obs': obs['observation'].shape[0],
        'goal': 3,
        'action': env.action_space.continuous_size + env.action_space.discrete_size,
        'action_max': env.action_max, # hard coded for now
        'max_timesteps': args.max_timesteps
    }
    ddpg_ag = ddpg_agent(args, env, env_params)
    ddpg_ag.learn()