from mlagents_envs.environment import UnityEnvironment
import os

print("Start verifying the environment...")
# unity_env = UnityEnvironment(os.path.abspath("./") + "/simulation/build/UnderwaterArm", worker_id=0, seed=1)
unity_env = UnityEnvironment(file_name=None, seed=1, side_channels=[])

unity_env.reset()

print("Get the behavior specs...")
behavior_name = list(unity_env.behavior_specs)[0]
print(f"Name of the behavior : {behavior_name}")

spec = unity_env.behavior_specs[behavior_name]
print("Number of observations : ", len(spec.observation_specs))
print("Number of actions : ", spec.action_spec.discrete_size)

if spec.action_spec.is_continuous():
  print("The action is continuous")

if spec.action_spec.is_discrete():
  print("The action is discrete")

# for episode in range(3):
# print(f"Starting episode {episode}...")
unity_env.reset()
decision_steps, terminal_steps = unity_env.get_steps(behavior_name)
tracked_agent = -1 # -1 indicates not yet tracking
done = False # For the tracked_agent
episode_rewards = 0 # For the tracked_agent
while not done:
  # Track the first agent we see if not tracking
  # Note : len(decision_steps) = [number of agents that requested a decision]
  if tracked_agent == -1 and len(decision_steps) >= 1:
    tracked_agent = decision_steps.agent_id[0]
    
  # Generate an action for all agents
  action = spec.action_spec.random_action(len(decision_steps))

  # Set the actions
  # unity_env.set_actions(behavior_name, action)

  # Move the simulation forward
  unity_env.step()
  # Get the new simulation results
  decision_steps, terminal_steps = unity_env.get_steps(behavior_name)
  print("Observations: ", decision_steps.obs, "\nRewards: ", decision_steps.reward, end="\r")
  if tracked_agent in decision_steps: # The agent requested a decision
    episode_rewards += decision_steps[tracked_agent].reward
  if tracked_agent in terminal_steps: # The agent terminated its episode
    episode_rewards += terminal_steps[tracked_agent].reward
    done = True
# print(f"Total rewards for episode {episode} is {episode_rewards}")

# for _ in range(100):
#   print("Performing random actions...")
#   unity_env.step()
#   decision_steps, terminal_steps = unity_env.get_steps(behavior_name)
#   print("Observations: ", decision_steps.obs)
#   print("Rewards: ", decision_steps.reward)