# Underwater Robot Control

Run the training with sparse rewards and headless mode:
```
python train.py --nsubsteps 15 --max-timesteps 25 --n-episodes 10 --n-epochs 40 --reward-type dense --batch-size 1024 --n-test-rollouts 10 --file-name ./simulation/build/UnderwaterArm --polyak 0.9
```