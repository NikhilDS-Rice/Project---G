# SMR Project with OMPL Integration

## Overview

This project now supports **two implementations**:

1. **OMPL-based** (recommended) - Uses the Open Motion Planning Library for:
   - State space representation (SE2)
   - Control space (curvature-based)
   - Built-in state validity checking
   - Optimized sampling strategies

2. **Pure Python** (fallback) - Custom implementation using NumPy/SciPy

Both implementations use the same MDP and value iteration code.

---

## Quick Start with Docker + OMPL

### 1. Build Docker Image
```powershell
cd "e:\COMP 550\Project - G"
docker build -t ompl-smr .
```

This will install:
- Ubuntu 22.04
- OMPL library
- Python 3 with all dependencies
- Project files

### 2. Run Container
```powershell
docker run -it --rm -v "${PWD}:/workspace" ompl-smr
```

### 3. Inside Docker, Setup and Run
```bash
# Setup environments
python3 main.py --setup

# Run example with OMPL
python3 example_ompl.py

# Run Part 1 with OMPL
python3 part1_ompl.py --env environments/simple_env.json

# Run Part 2 with OMPL
python3 part2_ompl.py --env environments/simple_env.json --n_trials 10
```

---

## File Structure (Updated)

```
Project - G/
├── Dockerfile                    # Docker setup with OMPL
├── DOCKER.md                     # Docker instructions
│
├── src/
│   ├── smr.py                   # Pure Python SMR
│   ├── smr_ompl.py              # OMPL-based SMR ⭐ NEW
│   ├── steerable_needle.py      # Pure Python needle
│   ├── mdp.py                   # MDP (shared by both)
│   └── ...
│
├── part1.py                      # Pure Python Part 1
├── part1_ompl.py                 # OMPL/Python Part 1 ⭐ NEW
├── part2.py                      # Pure Python Part 2
├── part2_ompl.py                 # OMPL/Python Part 2 ⭐ NEW
├── example.py                    # Pure Python example
└── example_ompl.py               # OMPL/Python example ⭐ NEW
```

---

## Comparison: OMPL vs Pure Python

| Feature | OMPL Implementation | Pure Python |
|---------|-------------------|-------------|
| State Space | `ob.SE2StateSpace()` | `np.array([x,y,θ])` |
| Sampling | OMPL's optimized sampler | `np.random.uniform()` |
| Validity Check | `ob.StateValidityChecker` | Custom collision check |
| Propagation | `oc.StatePropagator` | Custom dynamics |
| Performance | Faster sampling | Good enough |
| Dependencies | Requires OMPL | NumPy/SciPy only |
| Learning Curve | Need OMPL knowledge | Easier to understand |

---

## Usage Examples

### Example 1: Quick Test with OMPL
```bash
# Inside Docker
python3 example_ompl.py
```

### Example 2: Force Pure Python (no OMPL)
```bash
# On host machine (no Docker)
python part1_ompl.py --force-python
```

### Example 3: Sensitivity Analysis with OMPL
```bash
# Inside Docker
python3 part1_ompl.py --sensitivity --n_states 2000
```

### Example 4: Compare OMPL vs Python
```bash
# Inside Docker - OMPL
python3 part1_ompl.py --env environments/complex_env.json --save_dir results/ompl

# On host - Python
python part1_ompl.py --force-python --env environments/complex_env.json --save_dir results/python
```

---

## Docker Commands Reference

### Build and Run
```powershell
# Build
docker build -t ompl-smr .

# Run interactive
docker run -it --rm -v "${PWD}:/workspace" ompl-smr

# Run specific command
docker run --rm -v "${PWD}:/workspace" ompl-smr python3 example_ompl.py
```

### Verify OMPL Installation
```bash
# Inside container
python3 -c "from ompl import base as ob; print('✓ OMPL installed:', ob.OMPL_VERSION)"
```

### Access Results
All results saved in `results/` are automatically available on your host machine due to the volume mount.

---

## Benefits of Using OMPL

1. **Industry Standard**: OMPL is widely used in robotics research
2. **Optimized**: C++ backend for performance
3. **Extensible**: Easy to add other planners (RRT, PRM, etc.)
4. **Professional**: Learn a real robotics library
5. **Reproducible**: Docker ensures consistent environment

---

## Fallback to Pure Python

If OMPL is not available (or `--force-python` flag used):
- Automatically uses pure Python implementation
- Same algorithms, just custom code
- No performance difference for small problems
- Easier to debug and understand

---

## Troubleshooting

### OMPL not found in Docker
```bash
# Inside container
ldconfig
python3 -c "import ompl"
```

### Results not saving
- Check volume mount: `-v "${PWD}:/workspace"`
- Ensure `results/` directory exists

### Graphics not showing
- Docker runs headless by default
- Figures are saved to files automatically
- View on host machine

---

## Development Workflow

1. **Develop locally** (pure Python) for quick testing
2. **Test in Docker** (OMPL) for final experiments
3. **Compare results** from both implementations
4. **Report findings** with OMPL results as primary

---

## Next Steps

1. ✅ Build Docker image
2. ✅ Verify OMPL installation
3. ✅ Run `example_ompl.py`
4. ✅ Run Part 1 experiments
5. ✅ Run Part 2 experiments
6. ✅ Compare OMPL vs Python results
7. ✅ Generate report figures

Good luck! 🚀
