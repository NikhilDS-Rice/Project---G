# Project - G: Stochastic Motion Roadmap

## Quick Start Guide

### 1. Installation

```powershell
# Navigate to project directory
cd "e:\COMP 550\Project - G"

# Install dependencies
pip install -r requirements.txt
```

### 2. Setup Environments

First, create the environment configuration files:

```powershell
python main.py --setup
```

This creates four test environments:
- `simple_env.json` - Simple environment with 2 obstacles
- `corridor_env.json` - Corridor with narrow passage
- `complex_env.json` - Complex environment with multiple obstacles
- `maze_env.json` - Maze-like environment

### 3. Run Part 1: SMR Implementation and Policy Execution

Basic run with default parameters:
```powershell
python part1.py
```

Custom parameters:
```powershell
python part1.py --env environments/complex_env.json --n_states 3000 --n_simulations_test 200 --std 0.15
```

Run sensitivity analysis:
```powershell
python part1.py --sensitivity --n_states 2000
```

Parameters:
- `--env`: Environment file (default: `environments/simple_env.json`)
- `--n_states`: Number of states to sample (default: 2000)
- `--n_simulations_learning`: Simulations per state-action pair (default: 100)
- `--n_simulations_test`: Number of test simulations (default: 100)
- `--std`: Noise standard deviation (default: 0.1)
- `--sensitivity`: Run sensitivity analysis
- `--save_dir`: Directory to save results (default: `results/part1`)

### 4. Run Part 2: Probability Accuracy Analysis

Basic run:
```powershell
python part2.py
```

Custom parameters:
```powershell
python part2.py --env environments/simple_env.json --n_trials 30 --n_simulations_test 1500
```

Parameters:
- `--env`: Environment file
- `--n_trials`: Number of SMR structures per n value (default: 20)
- `--n_simulations_learning`: Simulations per state-action pair (default: 100)
- `--n_simulations_test`: Number of test simulations per SMR (default: 1000)
- `--std`: Noise standard deviation (default: 0.1)
- `--save_dir`: Directory to save results (default: `results/part2`)

### 5. Expected Outputs

#### Part 1
- Visualization of multiple trajectory executions
- Value function heatmap showing probability of success
- Sensitivity analysis plots (if `--sensitivity` flag used)
- Files saved to `results/part1/`

#### Part 2
- Plots showing expected vs actual probability
- Error analysis graphs
- Statistical trends as n increases
- Results saved to `results/part2/`

### 6. Running Tests

```powershell
python -m pytest tests/
# or
python -m unittest discover tests/
```

## Example Workflow

```powershell
# 1. Setup
python main.py --setup

# 2. Test basic SMR on simple environment
python part1.py --env environments/simple_env.json --n_states 1500

# 3. Run sensitivity analysis
python part1.py --env environments/complex_env.json --sensitivity --n_states 2000

# 4. Analyze probability accuracy
python part2.py --env environments/simple_env.json --n_trials 25

# 5. Test on challenging environment
python part1.py --env environments/maze_env.json --n_states 3000 --n_simulations_test 500
```

## Interpreting Results

### Part 1
- **Does the system always follow the same path?** No, due to actuation uncertainty, different executions follow different paths.
- **Does it always reach the goal?** No, success depends on noise level and environment complexity.
- **Sensitivity to noise?** Higher noise → lower success probability.

### Part 2
- **Accuracy vs n:** As n increases, the difference between expected and actual probability should decrease.
- **Convergence:** Larger n provides better approximation of continuous MDP.
- **Diminishing returns:** Beyond a certain n, improvement becomes marginal.

## Troubleshooting

### Issue: "No module named 'shapely'"
```powershell
pip install shapely
```

### Issue: Slow execution
- Reduce `n_states` (try 1000 instead of 2000)
- Reduce `n_simulations_learning` (try 50 instead of 100)
- Use simpler environment

### Issue: Low success rate
- Increase `n_states` for better coverage
- Decrease noise `--std`
- Check if start and goal are too far apart

## File Structure Reference

```
Project - G/
├── src/                          # Core implementation
│   ├── smr.py                   # SMR algorithm
│   ├── steerable_needle.py      # Robot model
│   ├── mdp.py                   # Value iteration
│   ├── collision_checker.py     # Collision detection
│   ├── environment.py           # Environment creation
│   └── utils.py                 # Utilities
├── visualization/               # Visualization tools
│   ├── visualize_smr.py
│   └── plot_results.py
├── environments/                # Environment configs (created by setup)
├── results/                     # Output results
│   ├── part1/
│   └── part2/
├── tests/                       # Unit tests
├── part1.py                     # Part 1 script
├── part2.py                     # Part 2 script
└── main.py                      # Main entry point
```

## Citation

Based on: R. Alterovitz, T. Siméon, and K. Goldberg, *The Stochastic Motion Roadmap: A Sampling Framework for Planning with Markov Motion Uncertainty*. In Robotics: Science and Systems, pp. 233–241, 2007.
