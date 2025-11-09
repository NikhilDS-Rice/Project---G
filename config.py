"""
Configuration file for SMR experiments.
"""

# Default SMR parameters
SMR_CONFIG = {
    # Learning phase
    'n_states': 2000,              # Number of states to sample
    'n_simulations_learning': 100, # Simulations per state-action pair
    'connection_radius': 2.0,      # Radius for state connections
    
    # Robot parameters
    'max_curvature': 1.0,          # Maximum steering curvature
    'step_size': 0.5,              # Distance per control action
    
    # Noise parameters
    'noise_std': 0.1,              # Standard deviation of Gaussian noise
    
    # Value iteration
    'max_iterations': 1000,        # Maximum value iteration steps
    'tolerance': 1e-6,             # Convergence tolerance
    
    # Testing
    'n_simulations_test': 100,     # Number of test simulations
    'max_execution_steps': 200,    # Maximum steps per execution
}

# Part 1 specific parameters
PART1_CONFIG = {
    'sensitivity_analysis': {
        'std_values': [0.05, 0.1, 0.15, 0.2, 0.25, 0.3],
        'n_states': 2000,
        'n_simulations_test': 100,
    }
}

# Part 2 specific parameters
PART2_CONFIG = {
    'n_values': [500, 1000, 2000, 3000, 5000, 7000, 10000],
    'n_trials': 20,
    'n_simulations_test': 1000,
}

# Visualization parameters
VIS_CONFIG = {
    'figure_size': (10, 10),
    'dpi': 300,
    'alpha_trajectories': 0.3,
    'show_orientation_arrows': True,
    'colormap': 'viridis',
}

# Environment parameters
ENV_CONFIG = {
    'workspace_bounds': [0, 10, 0, 10],
    'goal_radius': 0.5,
}
