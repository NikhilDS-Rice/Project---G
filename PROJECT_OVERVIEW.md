# COMP 550 Project - Stochastic Motion Roadmap
## Complete Project Structure and Implementation Guide

---

## 📁 Project Structure

```
Project - G/
├── 📄 README.md                      # Main project documentation
├── 📄 QUICKSTART.md                  # Quick start guide
├── 📄 TODO.md                        # Task checklist
├── 📄 requirements.txt               # Python dependencies
├── 📄 config.py                      # Configuration parameters
├── 📄 .gitignore                     # Git ignore rules
│
├── 🎯 main.py                        # Main entry point
├── 🎯 part1.py                       # Part 1: Policy execution & sensitivity
├── 🎯 part2.py                       # Part 2: Accuracy analysis
├── 🎯 example.py                     # Simple example script
│
├── 📂 src/                           # Core implementation
│   ├── __init__.py                  # Package initialization
│   ├── steerable_needle.py          # Steerable needle model (Dubins-car)
│   ├── mdp.py                       # MDP and value iteration
│   ├── smr.py                       # SMR algorithm (main)
│   ├── collision_checker.py         # Collision detection
│   ├── environment.py               # Environment creation/loading
│   └── utils.py                     # Utility functions
│
├── 📂 visualization/                 # Visualization tools
│   ├── visualize_smr.py             # SMR visualizer class
│   └── plot_results.py              # Result plotting utilities
│
├── 📂 tests/                         # Unit tests
│   ├── test_steerable_needle.py     # Needle model tests
│   ├── test_mdp.py                  # MDP tests
│   └── test_collision_checker.py    # Collision checker tests
│
├── 📂 environments/                  # Environment configurations
│   ├── simple_env.json              # Simple test environment
│   ├── corridor_env.json            # Narrow corridor
│   ├── complex_env.json             # Multiple obstacles
│   └── maze_env.json                # Maze-like layout
│
├── 📂 results/                       # Output results
│   ├── part1/                       # Part 1 results
│   └── part2/                       # Part 2 results
│
└── 📄 SMR_paper.pdf                  # Reference paper

```

---

## 🚀 Quick Start

### 1. Install Dependencies
```powershell
pip install -r requirements.txt
```

### 2. Setup Environments
```powershell
python main.py --setup
```

### 3. Run Example
```powershell
python example.py
```

### 4. Run Part 1
```powershell
python part1.py --env environments/simple_env.json
```

### 5. Run Part 2
```powershell
python part2.py --env environments/simple_env.json
```

---

## 📚 Core Components

### 1. Steerable Needle (`src/steerable_needle.py`)
- **Purpose**: Robot model with Dubins-car-like dynamics
- **Configuration Space**: Q = (x, y, θ)
- **Control Set**: 5 curvature values
- **Key Methods**:
  - `apply_control()`: Apply control with Gaussian noise
  - `sample_random_state()`: Sample valid configurations
  - `is_valid_state()`: Check workspace bounds

### 2. MDP (`src/mdp.py`)
- **Purpose**: Markov Decision Process implementation
- **Components**: States, controls, transitions, rewards
- **Key Methods**:
  - `add_transition()`: Define P(s'|s,a)
  - `value_iteration()`: Compute optimal policy
  - `get_optimal_control()`: Query policy

### 3. SMR (`src/smr.py`)
- **Purpose**: Main SMR algorithm
- **Phases**:
  - **Learning**: Sample states, estimate transitions
  - **Query**: Compute optimal policy via value iteration
- **Key Methods**:
  - `learning_phase()`: Build roadmap
  - `query_phase()`: Extract policy
  - `get_optimal_control()`: Get action for state

### 4. Collision Checker (`src/collision_checker.py`)
- **Purpose**: Detect collisions with obstacles
- **Uses**: Shapely library for geometric operations
- **Key Methods**:
  - `is_point_in_collision()`: Point collision check
  - `is_state_in_collision()`: State collision check

### 5. Visualizer (`visualization/visualize_smr.py`)
- **Purpose**: Visualization utilities
- **Key Methods**:
  - `plot_environment()`: Draw workspace and obstacles
  - `plot_trajectory()`: Visualize execution paths
  - `plot_value_function()`: Show probability heatmap
  - `plot_multiple_trajectories()`: Overlay many paths

---

## 🎯 Project Deliverables

### Part 1: Implementation & Policy Execution

**Objectives:**
1. ✅ Implement SMR for 2D steerable needle
2. ✅ Create interesting 2D environments
3. ✅ Visualize policy execution
4. ✅ Analyze sensitivity to noise

**Key Questions:**
- Does system always follow same path? → **No** (stochastic)
- Does it always reach goal? → **Depends on noise/environment**
- Sensitivity to noise? → **Higher noise = lower success**

**Outputs:**
- Multiple trajectory visualizations
- Value function heatmaps
- Sensitivity analysis plots

### Part 2: Probability Accuracy

**Objectives:**
1. ✅ Compare expected vs actual success probability
2. ✅ Analyze accuracy as function of n (sample size)
3. ✅ Show statistical trends

**Key Analysis:**
- Generate 20+ SMR structures per n value
- Test each with 1000+ simulations
- Plot error vs n (logarithmic scale)
- Show convergence with larger n

**Outputs:**
- Expected vs actual probability plots
- Error analysis graphs
- Statistical trend analysis

---

## 🔬 Algorithm Details

### SMR Learning Phase
```python
for each sampled state q:
    for each control u:
        for m simulations:
            q' = simulate(q, u, noise)
            if collision(q'):
                count_transitions[q][u][obstacle] += 1
            else:
                nearest = find_nearest(q', states)
                count_transitions[q][u][nearest] += 1
        
        # Convert counts to probabilities
        P(·|q,u) = counts / m
```

### Value Iteration
```python
V_0(q) = R(q)
repeat until convergence:
    for each state q:
        V_{t+1}(q) = R(q) + max_u Σ_q' P(q'|q,u) V_t(q')
        π(q) = argmax_u Σ_q' P(q'|q,u) V_t(q')
```

### Policy Execution
```python
current_state = start
while not goal_reached:
    u = π(current_state)
    current_state = apply_control(current_state, u, noise)
    if collision(current_state):
        return FAILURE
return SUCCESS
```

---

## 📊 Expected Results

### Part 1
- **Simple Environment**: ~80-90% success rate
- **Complex Environment**: ~50-70% success rate
- **Noise Sensitivity**: Linear decrease in success
- **Path Diversity**: High variance in trajectories

### Part 2
- **Error Trend**: Decreasing with n
- **Convergence**: Logarithmic improvement
- **Optimal n**: Around 3000-5000 for good balance

---

## ⚙️ Configuration Parameters

### Default Settings (in `config.py`)
```python
n_states = 2000              # Sample size
n_simulations_learning = 100 # Transition estimates
noise_std = 0.1              # Actuation uncertainty
connection_radius = 2.0      # State connectivity
max_iterations = 1000        # Value iteration
tolerance = 1e-6             # Convergence threshold
```

### Recommended Settings

**Quick Test:**
- n_states: 1000
- n_simulations_learning: 50
- n_simulations_test: 50

**Production Run:**
- n_states: 3000
- n_simulations_learning: 100
- n_simulations_test: 1000

**High Accuracy:**
- n_states: 5000+
- n_simulations_learning: 200
- n_simulations_test: 2000+

---

## 🧪 Testing

### Run Unit Tests
```powershell
python -m unittest discover tests/
```

### Test Coverage
- ✅ Steerable needle dynamics
- ✅ MDP value iteration
- ✅ Collision detection
- ✅ State sampling

---

## 📈 Performance Notes

### Computational Complexity
- **Learning Phase**: O(n × m × |U|) where:
  - n = number of states
  - m = simulations per state-action
  - |U| = control set size (5)
  
- **Value Iteration**: O(k × n × |U|) where:
  - k = iterations to converge (~50-100)

### Typical Runtime
- n=1000: ~2-5 minutes
- n=2000: ~5-10 minutes
- n=5000: ~20-30 minutes

### Memory Usage
- Sparse transition matrix
- ~100-500 MB for typical runs

---

## 🎓 Theory Review

### Markov Property
State transitions depend only on current state and action, not history.

### Bellman Equation
V*(s) = R(s) + max_a Σ_s' P(s'|s,a) V*(s')

### Optimal Policy
π*(s) = argmax_a Σ_s' P(s'|s,a) V*(s')

### Probability of Success
V(s) represents expected probability of reaching goal from state s under optimal policy.

---

## 📖 Reference

**Paper**: R. Alterovitz, T. Siméon, and K. Goldberg, *The Stochastic Motion Roadmap: A Sampling Framework for Planning with Markov Motion Uncertainty*. In Robotics: Science and Systems, pp. 233–241, 2007.

**Key Contributions:**
- Sampling-based approach to continuous MDP
- Approximation of Markov motion uncertainty
- Application to medical robotics (steerable needles)

---

## 💡 Tips & Tricks

1. **Start Small**: Test with n=1000 before scaling up
2. **Visualize Early**: Check value function makes sense
3. **Monitor Convergence**: Value iteration should converge in <100 iterations
4. **Balance Accuracy**: More states ≠ always better (diminishing returns)
5. **Use Appropriate Noise**: std=0.1 is good starting point

---

## 🐛 Troubleshooting

### Low Success Rate
- ✓ Increase n_states
- ✓ Decrease noise_std
- ✓ Check environment difficulty

### Slow Performance
- ✓ Reduce n_states
- ✓ Reduce n_simulations_learning
- ✓ Use simpler environment

### Value Iteration Not Converging
- ✓ Check transition probabilities sum to 1
- ✓ Verify goal states marked correctly
- ✓ Increase max_iterations

---

## 📝 Report Sections

### Introduction
- Problem statement
- SMR overview
- Steerable needle model

### Methods
- Algorithm description
- Implementation details
- Parameters chosen

### Results - Part 1
- Environment descriptions
- Policy visualizations
- Sensitivity analysis

### Results - Part 2
- Accuracy analysis
- Statistical trends
- Convergence behavior

### Discussion
- Answer key questions
- Analyze trade-offs
- Limitations and future work

### Conclusion
- Summary of findings
- Insights gained

---

## ✅ Final Checklist

- [ ] All code implemented and tested
- [ ] Environments created
- [ ] Part 1 experiments completed
- [ ] Part 2 experiments completed
- [ ] Figures generated
- [ ] Results analyzed
- [ ] Report written
- [ ] Code documented
- [ ] Repository organized

---

**Good luck with your project! 🚀**
