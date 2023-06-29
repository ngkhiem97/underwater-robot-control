from env.underwater_env import UnderwaterEnv

if __name__ == "__main__":
    env = UnderwaterEnv("env/Underwater.exe", num_areas=2)
    env.reset()
    for _ in range(10):
        action = env.env.get_steps()[0].action
        env.step(action)
    env.env.close()