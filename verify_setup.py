"""
Verification script to check if everything is set up correctly.
"""

import sys
import os

def check_imports():
    """Check if all required packages are installed."""
    print("Checking required packages...")
    required_packages = [
        ('numpy', 'NumPy'),
        ('matplotlib', 'Matplotlib'),
        ('scipy', 'SciPy'),
        ('shapely', 'Shapely'),
        ('tqdm', 'tqdm'),
    ]
    
    all_good = True
    for module_name, display_name in required_packages:
        try:
            __import__(module_name)
            print(f"  ✓ {display_name}")
        except ImportError:
            print(f"  ✗ {display_name} - NOT INSTALLED")
            all_good = False
    
    return all_good


def check_project_structure():
    """Check if all required files and directories exist."""
    print("\nChecking project structure...")
    
    required_dirs = [
        'src',
        'visualization',
        'tests',
        'environments',
        'results'
    ]
    
    required_files = [
        'src/__init__.py',
        'src/steerable_needle.py',
        'src/mdp.py',
        'src/smr.py',
        'src/collision_checker.py',
        'src/environment.py',
        'src/utils.py',
        'part1.py',
        'part2.py',
        'main.py',
        'requirements.txt'
    ]
    
    all_good = True
    
    for directory in required_dirs:
        if os.path.isdir(directory):
            print(f"  ✓ {directory}/")
        else:
            print(f"  ✗ {directory}/ - MISSING")
            all_good = False
    
    for filepath in required_files:
        if os.path.isfile(filepath):
            print(f"  ✓ {filepath}")
        else:
            print(f"  ✗ {filepath} - MISSING")
            all_good = False
    
    return all_good


def test_imports():
    """Test if project modules can be imported."""
    print("\nTesting project imports...")
    
    try:
        from src.steerable_needle import SteerableNeedle
        print("  ✓ SteerableNeedle")
    except Exception as e:
        print(f"  ✗ SteerableNeedle - {e}")
        return False
    
    try:
        from src.mdp import MDP
        print("  ✓ MDP")
    except Exception as e:
        print(f"  ✗ MDP - {e}")
        return False
    
    try:
        from src.collision_checker import CollisionChecker
        print("  ✓ CollisionChecker")
    except Exception as e:
        print(f"  ✗ CollisionChecker - {e}")
        return False
    
    try:
        from src.smr import StochasticMotionRoadmap
        print("  ✓ StochasticMotionRoadmap")
    except Exception as e:
        print(f"  ✗ StochasticMotionRoadmap - {e}")
        return False
    
    try:
        from visualization.visualize_smr import SMRVisualizer
        print("  ✓ SMRVisualizer")
    except Exception as e:
        print(f"  ✗ SMRVisualizer - {e}")
        return False
    
    return True


def run_basic_test():
    """Run a basic functionality test."""
    print("\nRunning basic functionality test...")
    
    try:
        import numpy as np
        from src.steerable_needle import SteerableNeedle
        from src.mdp import MDP
        from src.collision_checker import CollisionChecker
        
        # Test steerable needle
        needle = SteerableNeedle((0, 10, 0, 10))
        state = np.array([5, 5, 0])
        new_state = needle.apply_control(state, 0, noise_std=0.0)
        assert needle.is_valid_state(new_state), "Invalid state generated"
        print("  ✓ Steerable needle working")
        
        # Test MDP
        mdp = MDP(n_states=5, n_controls=3)
        mdp.add_transition(0, 0, 1, 1.0)
        mdp.set_goal_state(1)
        values, policy = mdp.value_iteration()
        assert values[1] == 1.0, "Goal state value incorrect"
        print("  ✓ MDP working")
        
        # Test collision checker
        obstacles = [[(2, 2), (4, 2), (4, 4), (2, 4)]]
        checker = CollisionChecker(obstacles, (0, 10, 0, 10))
        assert checker.is_point_in_collision((3, 3)), "Collision detection failed"
        assert not checker.is_point_in_collision((5, 5)), "False collision detected"
        print("  ✓ Collision checker working")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all verification checks."""
    print("="*60)
    print("SMR Project Verification")
    print("="*60)
    
    checks = [
        ("Package Dependencies", check_imports),
        ("Project Structure", check_project_structure),
        ("Module Imports", test_imports),
        ("Basic Functionality", run_basic_test),
    ]
    
    results = []
    for name, check_func in checks:
        result = check_func()
        results.append((name, result))
    
    # Summary
    print("\n" + "="*60)
    print("VERIFICATION SUMMARY")
    print("="*60)
    
    all_passed = True
    for name, result in results:
        status = "✓ PASSED" if result else "✗ FAILED"
        print(f"{name:.<40} {status}")
        if not result:
            all_passed = False
    
    print("="*60)
    
    if all_passed:
        print("\n✓ All checks passed! Your project is ready to run.")
        print("\nNext steps:")
        print("  1. python main.py --setup       # Create environments")
        print("  2. python example.py            # Run simple example")
        print("  3. python part1.py              # Run Part 1")
        print("  4. python part2.py              # Run Part 2")
    else:
        print("\n✗ Some checks failed. Please fix the issues above.")
        print("\nTo install missing packages:")
        print("  pip install -r requirements.txt")
        sys.exit(1)


if __name__ == "__main__":
    main()
