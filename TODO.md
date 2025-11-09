# SMR Project TODO List

## Implementation Checklist

### Core Implementation
- [x] Steerable needle model (Dubins-car-like)
- [x] MDP and value iteration
- [x] Collision checker
- [x] SMR algorithm (learning + query phases)
- [x] Environment configuration
- [x] Visualization tools

### Part 1 Tasks
- [x] Basic SMR implementation
- [x] Policy execution simulation
- [x] Multiple trajectory visualization
- [x] Sensitivity analysis for noise
- [ ] Test on all 4 environments
- [ ] Generate report figures

### Part 2 Tasks
- [x] Probability accuracy analysis
- [x] Multi-trial experiments
- [x] Statistical analysis
- [ ] Run full experiments (n=500 to 10000)
- [ ] Generate accuracy plots
- [ ] Write analysis report

### Testing
- [x] Unit tests for steerable needle
- [x] Unit tests for MDP
- [x] Unit tests for collision checker
- [ ] Integration tests
- [ ] Performance benchmarks

### Documentation
- [x] README.md
- [x] QUICKSTART.md
- [x] Code documentation
- [ ] Final report
- [ ] Presentation slides

## Experiments to Run

### Basic Tests
```powershell
# 1. Quick test on simple environment
python example.py

# 2. Full Part 1 on simple environment
python part1.py --env environments/simple_env.json --n_states 2000

# 3. Sensitivity analysis
python part1.py --sensitivity --n_states 1500
```

### Full Experiments
```powershell
# Part 1: All environments
python part1.py --env environments/simple_env.json --n_states 2000 --n_simulations_test 200
python part1.py --env environments/corridor_env.json --n_states 2500 --n_simulations_test 200
python part1.py --env environments/complex_env.json --n_states 3000 --n_simulations_test 200
python part1.py --env environments/maze_env.json --n_states 3000 --n_simulations_test 200

# Part 2: Accuracy analysis
python part2.py --env environments/simple_env.json --n_trials 25 --n_simulations_test 1500
```

## Questions to Answer

### Part 1
1. Does the system always follow the same path?
   - **Answer**: No, due to actuation uncertainty
   
2. Does it always reach the goal?
   - **Answer**: No, success probability depends on noise and environment
   
3. How sensitive is success to noise std?
   - **To measure**: Run sensitivity analysis

### Part 2
1. How does accuracy change with n?
   - **Expected**: Error decreases as n increases
   
2. What is the statistical trend?
   - **To analyze**: Plot error vs n on log scale

## Notes

- Learning phase is computationally expensive (O(n × m × |U|))
- Larger n → better approximation but slower
- Trade-off between accuracy and computation time
- Noise std significantly affects success rate
- Value iteration usually converges quickly (<100 iterations)

## Performance Tips

- Use KD-tree for nearest neighbor (already implemented)
- Sparse representation for transition probabilities (already implemented)
- Consider parallel execution for multiple trials
- Cache SMR structures to avoid rebuilding

## Future Enhancements

- [ ] Adaptive sampling (more states in difficult regions)
- [ ] Different robot models
- [ ] 3D extension
- [ ] Real-time visualization/animation
- [ ] Interactive parameter tuning
- [ ] Comparison with other methods (RRT, PRM)
