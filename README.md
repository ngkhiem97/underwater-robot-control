# Underwater Robot Control

Run the training with sparse rewards and headless mode:
```
python3 train.py --num-rollouts-per-mpi 1 --max-timesteps 3 --nsubsteps 40 --n-episodes 100 --n-epochs 1000 --reward-type sparse  --file-name ./simulation/build/UnderwaterArm --headless
```