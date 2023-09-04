import torch
from rl_modules.models import actor, critic
from arguments import get_args
from env.underwater_env import UnderwaterEnv
import numpy as np
from rl_modules.float_log_channel import FloatLogChannel

# process the inputs
def process_inputs(o, g, o_mean, o_std, g_mean, g_std, args):
    o_clip = np.clip(o, -args.clip_obs, args.clip_obs)
    g_clip = np.clip(g, -args.clip_obs, args.clip_obs)
    o_norm = np.clip((o_clip - o_mean) / (o_std), -args.clip_range, args.clip_range)
    g_norm = np.clip((g_clip - g_mean) / (g_std), -args.clip_range, args.clip_range)
    inputs = np.concatenate([o_norm, g_norm])
    inputs = torch.tensor(inputs, dtype=torch.float32)
    return inputs

if __name__ == '__main__':
    args = get_args()
    model_path = args.save_dir + args.load_dir
    o_mean, o_std, g_mean, g_std, actor_model, critic_model = torch.load(model_path, map_location=lambda storage, loc: storage)
    env = UnderwaterEnv(file_name=args.file_name, 
                        worker_id=0, 
                        base_port=5004, 
                        seed=args.seed, 
                        no_graphics=False, 
                        timeout_wait=60, 
                        log_folder='logs/', 
                        max_steps=args.max_timesteps, 
                        behavior_name=None,
                        reward_type=args.reward_type,
                        max_reward=args.max_reward,
                        nsubsteps=args.nsubsteps)
    obs = env.get_obs()
    env_params = {
        'obs': obs['observation'].shape[0],
        'goal': obs['desired_goal'].shape[0],
        'action': env.action_space.continuous_size,
        'action_max': env.action_max, 
        'max_timesteps': args.max_timesteps
    }
    actor_network = actor(env_params)
    actor_network.load_state_dict(actor_model)
    actor_network.eval()
    for i in range(args.n_test_rollouts):
        observation = env.reset(80, 5) # we want to reset multiple times to avoid the bug in the environment, 80 is the substeps, 5 is the number of resets
        obs = observation['observation']
        g = observation['desired_goal']
        for t in range(env_params['max_timesteps']):
            print('the time step is: ', t)
            print('the observation is: ', obs)
            print('the goal is: ', g)
            inputs = process_inputs(obs, g, o_mean, o_std, g_mean, g_std, args)
            with torch.no_grad():
                pi = actor_network(inputs)
            action = pi.detach().numpy().squeeze()
            print('the action is: ', action)
            observation_new, reward, is_done, info = env.step(action)
            obs = observation_new['observation']
            g = observation_new['desired_goal']
        print('the episode is: {}, is success: {}, is done: {}'.format(i, info['is_success'], is_done))
