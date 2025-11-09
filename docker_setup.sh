#!/bin/bash
# Quick setup script for Docker environment
# Run this inside your Docker container

echo "=========================================="
echo "SMR Project - Quick Setup for Docker"
echo "=========================================="

# Check if OMPL is available
echo -e "\n1. Checking OMPL installation..."
python3 -c "from ompl import base as ob; print('   ✓ OMPL version:', ob.OMPL_VERSION)" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ OMPL is ready!"
else
    echo "   ✗ OMPL not found. Will use pure Python."
fi

# Install Python dependencies
echo -e "\n2. Installing Python dependencies..."
pip3 install -q numpy matplotlib scipy shapely tqdm seaborn 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Dependencies installed!"
else
    echo "   ⚠ Some dependencies may already be installed"
fi

# Create environments
echo -e "\n3. Creating environment configurations..."
python3 main.py --setup
if [ $? -eq 0 ]; then
    echo "   ✓ Environments created!"
else
    echo "   ✗ Failed to create environments"
fi

# Run a quick test
echo -e "\n4. Running quick test..."
python3 -c "
import numpy as np
from src.collision_checker import CollisionChecker
print('   ✓ Project modules loaded successfully!')
"

echo -e "\n=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Try these commands:"
echo "  python3 example_ompl.py              # Quick example"
echo "  python3 part1_ompl.py                # Run Part 1"
echo "  python3 part2_ompl.py --n_trials 10  # Run Part 2"
echo ""
echo "Results will be saved to results/ directory"
echo "=========================================="
