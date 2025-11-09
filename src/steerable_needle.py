"""
Steerable Needle Model (Dubins-car-like dynamics)
Based on Section III of the SMR paper by Alterovitz et al.
"""

import numpy as np
from typing import Tuple, List


class SteerableNeedle:
    """
    2D Steerable needle model with configuration space and control set.
    
    Configuration space: Q = (x, y, θ)
    - (x, y): position in 2D workspace
    - θ: orientation angle
    
    Control space: U = {u1, u2, u3, ...}
    - Each control specifies a curvature and distance to travel
    """
    
    def __init__(self, workspace_bounds: Tuple[float, float, float, float],
                 max_curvature: float = 1.0,
                 step_size: float = 0.5):
        """
        Initialize the steerable needle model.
        
        Args:
            workspace_bounds: (x_min, x_max, y_min, y_max)
            max_curvature: Maximum curvature the needle can achieve
            step_size: Distance traveled per control action
        """
        self.workspace_bounds = workspace_bounds
        self.max_curvature = max_curvature
        self.step_size = step_size
        
        # Define control set (curvature values)
        # Similar to Dubins car: left turn, straight, right turn, plus intermediate values
        self.control_set = self._generate_control_set()
    
    def _generate_control_set(self) -> List[float]:
        """
        Generate a discrete set of control inputs (curvatures).
        
        Returns:
            List of curvature values
        """
        # Use 5 controls: max left, slight left, straight, slight right, max right
        return [
            -self.max_curvature,      # Max left turn
            -self.max_curvature / 2,  # Slight left turn
            0.0,                       # Straight
            self.max_curvature / 2,   # Slight right turn
            self.max_curvature         # Max right turn
        ]
    
    def apply_control(self, state: np.ndarray, control_idx: int, 
                     noise_std: float = 0.0) -> np.ndarray:
        """
        Apply a control to the current state with optional Gaussian noise.
        
        Args:
            state: Current configuration [x, y, θ]
            control_idx: Index of the control in the control set
            noise_std: Standard deviation of Gaussian noise
        
        Returns:
            New state after applying control with noise
        """
        x, y, theta = state
        curvature = self.control_set[control_idx]
        
        # Add Gaussian noise to the control
        if noise_std > 0:
            curvature += np.random.normal(0, noise_std)
        
        # Compute new state based on curvature
        if abs(curvature) < 1e-6:  # Straight line motion
            x_new = x + self.step_size * np.cos(theta)
            y_new = y + self.step_size * np.sin(theta)
            theta_new = theta
        else:  # Arc motion
            # Radius of curvature
            radius = 1.0 / curvature
            
            # Arc length equals step_size
            delta_theta = self.step_size * curvature
            
            # New position (circular arc)
            x_new = x + radius * (np.sin(theta + delta_theta) - np.sin(theta))
            y_new = y - radius * (np.cos(theta + delta_theta) - np.cos(theta))
            theta_new = theta + delta_theta
        
        # Normalize theta to [-π, π]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        
        return np.array([x_new, y_new, theta_new])
    
    def is_valid_state(self, state: np.ndarray) -> bool:
        """
        Check if a state is within workspace bounds.
        
        Args:
            state: Configuration [x, y, θ]
        
        Returns:
            True if state is within bounds
        """
        x, y, _ = state
        x_min, x_max, y_min, y_max = self.workspace_bounds
        return x_min <= x <= x_max and y_min <= y <= y_max
    
    def sample_random_state(self) -> np.ndarray:
        """
        Sample a random valid state within the workspace.
        
        Returns:
            Random configuration [x, y, θ]
        """
        x_min, x_max, y_min, y_max = self.workspace_bounds
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        theta = np.random.uniform(-np.pi, np.pi)
        return np.array([x, y, theta])
    
    def distance(self, state1: np.ndarray, state2: np.ndarray) -> float:
        """
        Compute distance between two states (Euclidean distance in (x,y) space).
        
        Args:
            state1: First configuration [x, y, θ]
            state2: Second configuration [x, y, θ]
        
        Returns:
            Euclidean distance
        """
        return np.linalg.norm(state1[:2] - state2[:2])
    
    def state_to_dict(self, state: np.ndarray) -> dict:
        """Convert state array to dictionary."""
        return {'x': state[0], 'y': state[1], 'theta': state[2]}
    
    def dict_to_state(self, state_dict: dict) -> np.ndarray:
        """Convert state dictionary to array."""
        return np.array([state_dict['x'], state_dict['y'], state_dict['theta']])
