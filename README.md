# COMP 550 Project - Stochastic Motion Roadmap (SMR)

## Project Overview
Implementation of the Stochastic Motion Roadmap (SMR) algorithm for motion planning under action uncertainty with a 2D steerable needle.

**Two implementations available:**
1. **OMPL-based** (recommended if Docker available) - Uses Open Motion Planning Library
2. **Pure Python** (fallback) - Standalone implementation with NumPy/SciPy

## Project Structure
```
Project - G/
├── src/                    # Source code
│   ├── smr.py             # Main SMR algorithm implementation
│   ├── steerable_needle.py # Steerable needle model (Dubins-car-like)
│   ├── mdp.py             # MDP and value iteration implementation
│   ├── collision_checker.py # Collision detection
│   └── utils.py           # Utility functions
├── environments/          # Environment configurations
│   ├── simple_env.json
│   ├── complex_env.json
│   └── corridor_env.json
├── visualization/         # Visualization scripts
│   ├── visualize_smr.py
│   └── plot_results.py
├── results/               # Output results and figures
├── tests/                 # Unit tests
├── main.py               # Main execution script
├── part1.py              # Part 1: Policy execution and sensitivity analysis
├── part2.py              # Part 2: Probability accuracy analysis
└── requirements.txt      # Python dependencies
```

## Requirements

### Option 1: Docker with OMPL (Recommended)
- Docker Desktop
- See `DOCKER.md` for setup instructions

### Option 2: Pure Python
- Python 3.8+
- NumPy, Matplotlib, SciPy, Shapely, tqdm

## Installation

### Using Docker (with OMPL)
```powershell
# Build Docker image
docker build -t ompl-smr .

# Run container
docker run -it --rm -v "${PWD}:/workspace" ompl-smr

# Inside container
python3 main.py --setup
python3 part1_ompl.py
```

See **DOCKER.md** for detailed Docker instructions.

### Using Pure Python (no OMPL)
```powershell
pip install -r requirements.txt
python main.py --setup
python part1.py  # or part1_ompl.py --force-python
```

## Usage

### Part 1: SMR Implementation and Policy Execution

**With OMPL (in Docker):**
```bash
python3 part1_ompl.py --env environments/simple_env.json --n_states 2000 --std 0.1
```

**Pure Python:**
```bash
python part1.py --env environments/simple_env.json --n_states 2000 --std 0.1
# or
python part1_ompl.py --force-python --env environments/simple_env.json
```

### Part 2: Probability Accuracy Analysis

**With OMPL:**
```bash
python3 part2_ompl.py --env environments/simple_env.json --n_trials 20
```

**Pure Python:**
```bash
python part2.py --env environments/simple_env.json --n_trials 20
# or
python part2_ompl.py --force-python --env environments/simple_env.json
```

## Deliverables

### Part 1
- Implement SMR algorithm for 2D steerable needle
- Create interesting 2D environments
- Visualize execution under optimal policy
- Analyze sensitivity to Gaussian noise standard deviation

### Part 2
- Compare expected vs actual probability of success
- Analyze how accuracy changes with number of sampled states (n)
- Generate statistical trends

## Reference
R. Alterovitz, T. Siméon, and K. Goldberg, *The Stochastic Motion Roadmap: A Sampling Framework for Planning with Markov Motion Uncertainty*. In Robotics: Science and Systems, pp. 233–241, 2007.
