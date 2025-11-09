"""
Example script demonstrating SMR usage.
"""

import numpy as np
import matplotlib.pyplot as plt

from src.steerable_needle import SteerableNeedle
from src.collision_checker import CollisionChecker
from src.smr import StochasticMotionRoadmap
from src.utils import generate_rectangle_obstacle, generate_circle_obstacle
from visualization.visualize_smr import SMRVisualizer


def create_example_environment():
    """Create a simple example environment."""
    workspace_bounds = (0, 10, 0, 10)
    
    obstacles = [
        generate_rectangle_obstacle((5, 5), 2, 3),
        generate_circle_obstacle((7, 7), 1),
    ]
    
    return workspace_bounds, obstacles


def main():
    """Run a simple SMR example."""
    print("SMR Example: Simple Environment")
    print("=" * 60)
    
    # Create environment
    workspace_bounds, obstacles = create_example_environment()
    
    # Setup components
    robot = SteerableNeedle(workspace_bounds)
    collision_checker = CollisionChecker(obstacles, workspace_bounds)
    visualizer = SMRVisualizer(collision_checker, workspace_bounds)
    
    # Define start and goal
    start_state = np.array([1, 1, 0])
    goal_center = np.array([9, 9, 0])
    goal_radius = 0.5
    
    # Create SMR
    print("\nCreating SMR...")
    smr = StochasticMotionRoadmap(robot, collision_checker, noise_std=0.1)
    
    # Learning phase
    print("Learning phase...")
    smr.learning_phase(
        n_states=1500,
        n_simulations=50,
        goal_region=(goal_center, goal_radius)
    )
    
    # Query phase
    print("Query phase...")
    start_idx, prob_success = smr.query_phase(start_state)
    
    print(f"\nProbability of success: {prob_success:.4f}")
    
    # Simulate execution
    print("\nSimulating 10 executions...")
    from part1 import simulate_policy_execution
    
    trajectories = []
    successes = 0
    
    for i in range(10):
        traj, success, steps = simulate_policy_execution(
            smr, start_state, goal_center, goal_radius
        )
        trajectories.append(traj)
        if success:
            successes += 1
        print(f"  Trial {i+1}: {'Success' if success else 'Failed'} ({steps} steps)")
    
    print(f"\nActual success rate: {successes}/10 = {successes/10:.2f}")
    
    # Visualize
    print("\nGenerating visualization...")
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot trajectories
    ax = axes[0]
    visualizer.plot_environment(ax)
    visualizer.plot_goal_region(goal_center, goal_radius, ax)
    visualizer.plot_multiple_trajectories(trajectories, ax)
    ax.plot(start_state[0], start_state[1], 'ro', markersize=10, label='Start')
    ax.set_title('Trajectory Executions')
    ax.legend()
    
    # Plot value function
    ax = axes[1]
    visualizer.plot_environment(ax)
    visualizer.plot_goal_region(goal_center, goal_radius, ax)
    visualizer.plot_value_function(smr.states, smr.mdp.values[:-1], ax)
    ax.set_title('Value Function')
    
    plt.tight_layout()
    plt.savefig('example_result.png', dpi=150)
    print("Saved visualization to example_result.png")
    plt.show()
    
    print("\nExample complete!")


if __name__ == "__main__":
    main()
