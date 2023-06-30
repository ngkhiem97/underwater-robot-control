from env.underwater_env import UnderwaterEnv
from arguments import get_args

if __name__ == "__main__":
    args = get_args()

    print("Create the environment...")
    env = UnderwaterEnv(file_name=None)
    env.reset()

    print("Get the behavior specs...")
    behavior_name = list(env.env.behavior_specs)[0]
    print(f"Name of the behavior : {behavior_name}")

    spec = env.env.behavior_specs[behavior_name]
    print("Number of observations : ", len(spec.observation_specs))
    print("Number of actions : ", spec.action_spec.discrete_size)

    if spec.action_spec.is_continuous():
        print("The action is continuous")

    if spec.action_spec.is_discrete():
        print("The action is discrete")

    for _ in range(80):
        print("step...")
        # action = env.env.get_steps(behavior_name)[0].action
        env.step()
    env.close()