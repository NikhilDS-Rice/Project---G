"""
Part 1: SMR Implementation and Policy Execution
- Implement SMR algorithm for 2D steerable needle
- Visualize execution under optimal policy
- Analyze sensitivity to noise standard deviation
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from typing import List

from src.steerable_needle import SteerableNeedle
from src.collision_checker import CollisionChecker
from src.smr import StochasticMotionRoadmap
from src.environment import load_environment
from visualization.visualize_smr import SMRVisualizer


def simulate_policy_execution(smr: StochasticMotionRoadmap, 
                              start_state: np.ndarray,
                              goal_center: np.ndarray,
                              goal_radius: float,
                              max_steps: int = 200) -> tuple:
    """
    Simulate execution of the optimal policy.
    
    Args:
        smr: Trained SMR
        start_state: Initial state
        goal_center: Goal region center
        goal_radius: Goal region radius
        max_steps: Maximum number of steps
    
    Returns:
        (trajectory, success, num_steps)
    """
    trajectory = [start_state.copy()]
    current_state = start_state.copy()
    
    for step in range(max_steps):
        # Check if reached goal
        dist_to_goal = smr.robot.distance(current_state, goal_center)
        if dist_to_goal <= goal_radius:
            return trajectory, True, step + 1
        
        # Get optimal control
        control_idx = smr.get_optimal_control(current_state)
        
        if control_idx is None:
            # State not in roadmap
            return trajectory, False, step + 1
        
        # Apply control with noise
        next_state = smr.robot.apply_control(
            current_state, control_idx, smr.noise_std
        )
        
        # Check collision
        if smr.collision_checker.is_state_in_collision(next_state):
            return trajectory, False, step + 1
        
        current_state = next_state
        trajectory.append(current_state.copy())
    
    return trajectory, False, max_steps


def run_multiple_simulations(smr: StochasticMotionRoadmap,
                            start_state: np.ndarray,
                            goal_center: np.ndarray,
                            goal_radius: float,
                            n_simulations: int = 100) -> dict:
    """
    Run multiple simulations and collect statistics.
    
    Args:
        smr: Trained SMR
        start_state: Initial state
        goal_center: Goal region center
        goal_radius: Goal region radius
        n_simulations: Number of simulations
    
    Returns:
        Dictionary with statistics
    """
    trajectories = []
    successes = []
    num_steps_list = []
    
    for i in range(n_simulations):
        trajectory, success, num_steps = simulate_policy_execution(
            smr, start_state, goal_center, goal_radius
        )
        trajectories.append(trajectory)
        successes.append(success)
        num_steps_list.append(num_steps)
    
    success_rate = sum(successes) / n_simulations
    avg_steps = np.mean(num_steps_list)
    
    return {
        'trajectories': trajectories,
        'successes': successes,
        'num_steps': num_steps_list,
        'success_rate': success_rate,
        'avg_steps': avg_steps
    }


def sensitivity_analysis(env_config: dict, n_states: int, 
                        n_simulations_learning: int,
                        n_simulations_test: int,
                        std_values: List[float],
                        save_dir: str):
    """
    Analyze sensitivity to noise standard deviation.
    
    Args:
        env_config: Environment configuration
        n_states: Number of states to sample
        n_simulations_learning: Simulations per state-action pair
        n_simulations_test: Number of test simulations
        std_values: List of noise standard deviations to test
        save_dir: Directory to save results
    """
    workspace_bounds = tuple(env_config['workspace_bounds'])
    obstacles = env_config['obstacles']
    start_state = np.array(env_config['start'])
    goal_center = np.array(env_config['goal_center'])
    goal_radius = env_config['goal_radius']
    
    # Setup robot and collision checker
    robot = SteerableNeedle(workspace_bounds)
    collision_checker = CollisionChecker(obstacles, workspace_bounds)
    
    results = []
    
    for std in std_values:
        print(f"\n{'='*60}")
        print(f"Testing with noise std = {std}")
        print(f"{'='*60}")
        
        # Create and train SMR
        smr = StochasticMotionRoadmap(robot, collision_checker, noise_std=std)
        smr.learning_phase(n_states, n_simulations_learning, 
                          (goal_center, goal_radius))
        start_idx, expected_prob = smr.query_phase(start_state)
        
        # Run simulations
        stats = run_multiple_simulations(
            smr, start_state, goal_center, goal_radius, n_simulations_test
        )
        
        actual_prob = stats['success_rate']
        
        print(f"Expected probability: {expected_prob:.4f}")
        print(f"Actual probability: {actual_prob:.4f}")
        print(f"Average steps: {stats['avg_steps']:.2f}")
        
        results.append({
            'std': std,
            'expected_prob': expected_prob,
            'actual_prob': actual_prob,
            'avg_steps': stats['avg_steps'],
            'trajectories': stats['trajectories'][:10]  # Keep only first 10
        })
    
    # Plot results
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    stds = [r['std'] for r in results]
    expected_probs = [r['expected_prob'] for r in results]
    actual_probs = [r['actual_prob'] for r in results]
    avg_steps = [r['avg_steps'] for r in results]
    
    # Success probability vs noise
    axes[0].plot(stds, expected_probs, 'b-o', label='Expected')
    axes[0].plot(stds, actual_probs, 'r-s', label='Actual')
    axes[0].set_xlabel('Noise Standard Deviation')
    axes[0].set_ylabel('Probability of Success')
    axes[0].set_title('Success Probability vs Noise')
    axes[0].legend()
    axes[0].grid(True)
    
    # Average steps vs noise
    axes[1].plot(stds, avg_steps, 'g-o')
    axes[1].set_xlabel('Noise Standard Deviation')
    axes[1].set_ylabel('Average Number of Steps')
    axes[1].set_title('Path Length vs Noise')
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'sensitivity_analysis.png'), dpi=300)
    plt.close()
    
    print(f"\nSensitivity analysis saved to {save_dir}")


def main():
    parser = argparse.ArgumentParser(description='Part 1: SMR Implementation')
    parser.add_argument('--env', type=str, default='environments/simple_env.json',
                       help='Environment configuration file')
    parser.add_argument('--n_states', type=int, default=2000,
                       help='Number of states to sample')
    parser.add_argument('--n_simulations_learning', type=int, default=100,
                       help='Simulations per state-action pair')
    parser.add_argument('--n_simulations_test', type=int, default=100,
                       help='Number of test simulations')
    parser.add_argument('--std', type=float, default=0.1,
                       help='Noise standard deviation')
    parser.add_argument('--sensitivity', action='store_true',
                       help='Run sensitivity analysis')
    parser.add_argument('--save_dir', type=str, default='results/part1',
                       help='Directory to save results')
    
    args = parser.parse_args()
    
    # Create save directory
    os.makedirs(args.save_dir, exist_ok=True)
    
    # Load environment
    env_config = load_environment(args.env)
    print(f"Loaded environment: {env_config['name']}")
    
    workspace_bounds = tuple(env_config['workspace_bounds'])
    obstacles = env_config['obstacles']
    start_state = np.array(env_config['start'])
    goal_center = np.array(env_config['goal_center'])
    goal_radius = env_config['goal_radius']
    
    # Setup robot and collision checker
    robot = SteerableNeedle(workspace_bounds)
    collision_checker = CollisionChecker(obstacles, workspace_bounds)
    visualizer = SMRVisualizer(collision_checker, workspace_bounds)
    
    if args.sensitivity:
        # Run sensitivity analysis
        std_values = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
        sensitivity_analysis(env_config, args.n_states, 
                           args.n_simulations_learning,
                           args.n_simulations_test,
                           std_values, args.save_dir)
    else:
        # Single SMR run
        print(f"\nBuilding SMR with {args.n_states} states...")
        smr = StochasticMotionRoadmap(robot, collision_checker, noise_std=args.std)
        smr.learning_phase(args.n_states, args.n_simulations_learning,
                          (goal_center, goal_radius))
        start_idx, expected_prob = smr.query_phase(start_state)
        
        # Run multiple simulations
        print(f"\nRunning {args.n_simulations_test} test simulations...")
        stats = run_multiple_simulations(
            smr, start_state, goal_center, goal_radius, args.n_simulations_test
        )
        
        print(f"\nResults:")
        print(f"Expected probability of success: {expected_prob:.4f}")
        print(f"Actual probability of success: {stats['success_rate']:.4f}")
        print(f"Average number of steps: {stats['avg_steps']:.2f}")
        
        # Visualize results
        fig, axes = plt.subplots(1, 2, figsize=(20, 10))
        
        # Plot 1: Environment with multiple trajectories
        ax = axes[0]
        visualizer.plot_environment(ax)
        visualizer.plot_goal_region(goal_center, goal_radius, ax)
        visualizer.plot_multiple_trajectories(stats['trajectories'][:50], ax)
        ax.plot(start_state[0], start_state[1], 'ro', markersize=10, label='Start')
        ax.set_title(f'Multiple Trajectory Executions (n={min(50, len(stats["trajectories"]))})')
        ax.legend()
        
        # Plot 2: Value function
        ax = axes[1]
        visualizer.plot_environment(ax)
        visualizer.plot_goal_region(goal_center, goal_radius, ax)
        visualizer.plot_value_function(smr.states, smr.mdp.values[:-1], ax)
        ax.set_title('Value Function (Probability of Success)')
        
        plt.tight_layout()
        plt.savefig(os.path.join(args.save_dir, 'smr_results.png'), dpi=300)
        plt.show()
        
        print(f"\nResults saved to {args.save_dir}")


if __name__ == "__main__":
    main()
