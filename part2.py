"""
Part 2: Probability Accuracy Analysis
- Compare expected vs actual probability of success
- Analyze how accuracy changes with number of sampled states (n)
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from typing import List, Tuple

from src.steerable_needle import SteerableNeedle
from src.collision_checker import CollisionChecker
from src.smr import StochasticMotionRoadmap
from src.environment import load_environment
from part1 import simulate_policy_execution


def evaluate_smr_accuracy(env_config: dict,
                          n_states: int,
                          n_trials: int,
                          n_simulations_learning: int,
                          n_simulations_test: int,
                          noise_std: float) -> Tuple[float, float, float]:
    """
    Evaluate SMR accuracy for a given number of states.
    
    Args:
        env_config: Environment configuration
        n_states: Number of states to sample
        n_trials: Number of SMR structures to generate
        n_simulations_learning: Simulations per state-action pair
        n_simulations_test: Number of test simulations per SMR
        noise_std: Noise standard deviation
    
    Returns:
        (mean_expected_prob, mean_actual_prob, mean_abs_error)
    """
    workspace_bounds = tuple(env_config['workspace_bounds'])
    obstacles = env_config['obstacles']
    start_state = np.array(env_config['start'])
    goal_center = np.array(env_config['goal_center'])
    goal_radius = env_config['goal_radius']
    
    # Setup robot and collision checker
    robot = SteerableNeedle(workspace_bounds)
    collision_checker = CollisionChecker(obstacles, workspace_bounds)
    
    expected_probs = []
    actual_probs = []
    
    for trial in range(n_trials):
        print(f"  Trial {trial + 1}/{n_trials}...", end=' ')
        
        # Create and train SMR
        smr = StochasticMotionRoadmap(robot, collision_checker, noise_std=noise_std)
        smr.learning_phase(n_states, n_simulations_learning,
                          (goal_center, goal_radius), connection_radius=2.0)
        start_idx, expected_prob = smr.query_phase(start_state)
        
        # Run test simulations
        successes = 0
        for _ in range(n_simulations_test):
            _, success, _ = simulate_policy_execution(
                smr, start_state, goal_center, goal_radius
            )
            if success:
                successes += 1
        
        actual_prob = successes / n_simulations_test
        
        expected_probs.append(expected_prob)
        actual_probs.append(actual_prob)
        
        print(f"Expected: {expected_prob:.4f}, Actual: {actual_prob:.4f}")
    
    mean_expected = np.mean(expected_probs)
    mean_actual = np.mean(actual_probs)
    mean_abs_error = np.mean(np.abs(np.array(expected_probs) - np.array(actual_probs)))
    
    return mean_expected, mean_actual, mean_abs_error


def main():
    parser = argparse.ArgumentParser(description='Part 2: Probability Accuracy Analysis')
    parser.add_argument('--env', type=str, default='environments/simple_env.json',
                       help='Environment configuration file')
    parser.add_argument('--n_trials', type=int, default=20,
                       help='Number of SMR structures per n value')
    parser.add_argument('--n_simulations_learning', type=int, default=100,
                       help='Simulations per state-action pair')
    parser.add_argument('--n_simulations_test', type=int, default=1000,
                       help='Number of test simulations per SMR')
    parser.add_argument('--std', type=float, default=0.1,
                       help='Noise standard deviation')
    parser.add_argument('--save_dir', type=str, default='results/part2',
                       help='Directory to save results')
    
    args = parser.parse_args()
    
    # Create save directory
    os.makedirs(args.save_dir, exist_ok=True)
    
    # Load environment
    env_config = load_environment(args.env)
    print(f"Loaded environment: {env_config['name']}")
    
    # Range of n values to test (logarithmic scale)
    n_values = [500, 1000, 2000, 3000, 5000, 7000, 10000]
    
    print(f"\nTesting {len(n_values)} different values of n with {args.n_trials} trials each")
    print(f"Simulations per state-action: {args.n_simulations_learning}")
    print(f"Test simulations per trial: {args.n_simulations_test}")
    print(f"Noise std: {args.std}\n")
    
    results = []
    
    for n in n_values:
        print(f"\n{'='*60}")
        print(f"Testing with n = {n} states")
        print(f"{'='*60}")
        
        mean_expected, mean_actual, mean_abs_error = evaluate_smr_accuracy(
            env_config, n, args.n_trials, args.n_simulations_learning,
            args.n_simulations_test, args.std
        )
        
        print(f"\nResults for n = {n}:")
        print(f"  Mean expected probability: {mean_expected:.4f}")
        print(f"  Mean actual probability: {mean_actual:.4f}")
        print(f"  Mean absolute error: {mean_abs_error:.4f}")
        
        results.append({
            'n': n,
            'mean_expected': mean_expected,
            'mean_actual': mean_actual,
            'mean_abs_error': mean_abs_error
        })
    
    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    n_vals = [r['n'] for r in results]
    expected = [r['mean_expected'] for r in results]
    actual = [r['mean_actual'] for r in results]
    errors = [r['mean_abs_error'] for r in results]
    
    # Plot 1: Expected vs Actual Probability
    axes[0, 0].plot(n_vals, expected, 'b-o', label='Expected', linewidth=2)
    axes[0, 0].plot(n_vals, actual, 'r-s', label='Actual', linewidth=2)
    axes[0, 0].set_xlabel('Number of Sampled States (n)')
    axes[0, 0].set_ylabel('Probability of Success')
    axes[0, 0].set_title('Expected vs Actual Probability')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].set_xscale('log')
    
    # Plot 2: Absolute Error
    axes[0, 1].plot(n_vals, errors, 'g-o', linewidth=2)
    axes[0, 1].set_xlabel('Number of Sampled States (n)')
    axes[0, 1].set_ylabel('Mean Absolute Error')
    axes[0, 1].set_title('Prediction Error vs Sample Size')
    axes[0, 1].grid(True)
    axes[0, 1].set_xscale('log')
    
    # Plot 3: Error on linear scale
    axes[1, 0].plot(n_vals, errors, 'g-o', linewidth=2)
    axes[1, 0].set_xlabel('Number of Sampled States (n)')
    axes[1, 0].set_ylabel('Mean Absolute Error')
    axes[1, 0].set_title('Prediction Error (Linear Scale)')
    axes[1, 0].grid(True)
    
    # Plot 4: Difference between expected and actual
    diff = [e - a for e, a in zip(expected, actual)]
    axes[1, 1].plot(n_vals, diff, 'm-o', linewidth=2)
    axes[1, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1, 1].set_xlabel('Number of Sampled States (n)')
    axes[1, 1].set_ylabel('Expected - Actual Probability')
    axes[1, 1].set_title('Bias in Probability Estimate')
    axes[1, 1].grid(True)
    axes[1, 1].set_xscale('log')
    
    plt.tight_layout()
    plt.savefig(os.path.join(args.save_dir, 'accuracy_analysis.png'), dpi=300)
    plt.show()
    
    # Save numerical results
    with open(os.path.join(args.save_dir, 'results.txt'), 'w') as f:
        f.write(f"Environment: {env_config['name']}\n")
        f.write(f"Trials per n: {args.n_trials}\n")
        f.write(f"Simulations per state-action: {args.n_simulations_learning}\n")
        f.write(f"Test simulations: {args.n_simulations_test}\n")
        f.write(f"Noise std: {args.std}\n\n")
        f.write(f"{'n':>8} {'Expected':>10} {'Actual':>10} {'Error':>10}\n")
        f.write(f"{'-'*42}\n")
        for r in results:
            f.write(f"{r['n']:>8} {r['mean_expected']:>10.4f} "
                   f"{r['mean_actual']:>10.4f} {r['mean_abs_error']:>10.4f}\n")
    
    print(f"\n{'='*60}")
    print(f"Analysis complete! Results saved to {args.save_dir}")
    print(f"{'='*60}")
    
    # Print summary
    print("\nSummary:")
    print(f"  Best accuracy (lowest error) at n = {results[np.argmin(errors)]['n']}")
    print(f"  Error decreased from {errors[0]:.4f} to {errors[-1]:.4f}")
    print(f"  Improvement: {(1 - errors[-1]/errors[0])*100:.1f}%")


if __name__ == "__main__":
    main()
