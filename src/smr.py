"""
Stochastic Motion Roadmap (SMR) implementation.
Based on Alterovitz et al., RSS 2007.
"""

import numpy as np
from typing import List, Tuple, Optional
from tqdm import tqdm
import pickle

from src.steerable_needle import SteerableNeedle
from src.mdp import MDP
from src.collision_checker import CollisionChecker
from src.utils import nearest_neighbor


class StochasticMotionRoadmap:
    """
    Stochastic Motion Roadmap for motion planning under uncertainty.
    
    The SMR consists of two phases:
    1. Learning phase: Sample states and estimate transition probabilities
    2. Query phase: Compute optimal policy using value iteration
    """
    
    def __init__(self, robot: SteerableNeedle, 
                 collision_checker: CollisionChecker,
                 noise_std: float = 0.1):
        """
        Initialize SMR.
        
        Args:
            robot: Steerable needle robot model
            collision_checker: Collision checker for the environment
            noise_std: Standard deviation of Gaussian noise in controls
        """
        self.robot = robot
        self.collision_checker = collision_checker
        self.noise_std = noise_std
        
        # Sampled states
        self.states: List[np.ndarray] = []
        
        # MDP representation
        self.mdp: Optional[MDP] = None
        
        # State indices for special states
        self.obstacle_state_idx: Optional[int] = None
        self.goal_state_indices: List[int] = []
        
        # Hyperparameters
        self.connection_radius = 2.0  # Radius for considering state transitions
    
    def learning_phase(self, n_states: int, n_simulations: int,
                      goal_region: Tuple[np.ndarray, float],
                      connection_radius: Optional[float] = None):
        """
        Learning phase: Sample states and estimate transition probabilities.
        
        Args:
            n_states: Number of states to sample
            n_simulations: Number of simulations per state-action pair
            goal_region: (goal_center, goal_radius) defining the goal region
            connection_radius: Radius for considering state transitions
        """
        if connection_radius is not None:
            self.connection_radius = connection_radius
        
        goal_center, goal_radius = goal_region
        
        print(f"SMR Learning Phase: Sampling {n_states} states...")
        
        # Sample valid states
        self.states = []
        attempts = 0
        max_attempts = n_states * 100
        
        with tqdm(total=n_states) as pbar:
            while len(self.states) < n_states and attempts < max_attempts:
                state = self.robot.sample_random_state()
                
                # Check if state is valid (collision-free and in goal region if close enough)
                if not self.collision_checker.is_state_in_collision(state):
                    self.states.append(state)
                    pbar.update(1)
                
                attempts += 1
        
        if len(self.states) < n_states:
            print(f"Warning: Only sampled {len(self.states)} valid states")
        
        # Add obstacle state (index 0)
        self.obstacle_state_idx = len(self.states)
        
        # Initialize MDP
        n_controls = len(self.robot.control_set)
        self.mdp = MDP(len(self.states) + 1, n_controls)  # +1 for obstacle state
        
        # Mark obstacle state
        self.mdp.set_obstacle_state(self.obstacle_state_idx)
        
        # Identify goal states
        self.goal_state_indices = []
        for idx, state in enumerate(self.states):
            dist = self.robot.distance(state, goal_center)
            if dist <= goal_radius:
                self.goal_state_indices.append(idx)
                self.mdp.set_goal_state(idx)
        
        print(f"Found {len(self.goal_state_indices)} goal states")
        
        # Estimate transition probabilities
        print(f"Estimating transition probabilities with {n_simulations} simulations per state-action pair...")
        
        with tqdm(total=len(self.states) * n_controls) as pbar:
            for state_idx, state in enumerate(self.states):
                # Skip goal and obstacle states (terminal states)
                if self.mdp.is_terminal_state(state_idx):
                    pbar.update(n_controls)
                    continue
                
                for control_idx in range(n_controls):
                    # Count transitions to each state
                    transition_counts = {}
                    
                    for _ in range(n_simulations):
                        # Simulate control with noise
                        next_state = self.robot.apply_control(
                            state, control_idx, self.noise_std
                        )
                        
                        # Check if resulting state is in collision
                        if self.collision_checker.is_state_in_collision(next_state):
                            # Transition to obstacle state
                            next_state_idx = self.obstacle_state_idx
                        else:
                            # Find nearest neighbor in sampled states
                            next_state_idx = nearest_neighbor(
                                next_state, self.states, self.connection_radius
                            )
                            
                            # If no neighbor within radius, go to obstacle state
                            if next_state_idx is None:
                                next_state_idx = self.obstacle_state_idx
                        
                        # Update count
                        transition_counts[next_state_idx] = \
                            transition_counts.get(next_state_idx, 0) + 1
                    
                    # Convert counts to probabilities
                    for next_state_idx, count in transition_counts.items():
                        probability = count / n_simulations
                        self.mdp.add_transition(
                            state_idx, control_idx, next_state_idx, probability
                        )
                    
                    pbar.update(1)
        
        print("Learning phase complete!")
    
    def query_phase(self, start_state: np.ndarray) -> Tuple[int, float]:
        """
        Query phase: Compute optimal policy for a start state.
        
        Args:
            start_state: Initial configuration
        
        Returns:
            Tuple of (start_state_idx, probability_of_success)
        """
        if self.mdp is None:
            raise ValueError("Must run learning_phase before query_phase")
        
        print("SMR Query Phase: Computing optimal policy...")
        
        # Find nearest state to start
        start_state_idx = nearest_neighbor(start_state, self.states, 
                                          self.connection_radius)
        
        if start_state_idx is None:
            print("Warning: Start state not near any sampled state")
            return None, 0.0
        
        # Run value iteration
        values, policy = self.mdp.value_iteration()
        
        # Get probability of success from start state
        prob_success = values[start_state_idx]
        
        print(f"Optimal policy computed. Probability of success: {prob_success:.4f}")
        
        return start_state_idx, prob_success
    
    def get_optimal_control(self, state: np.ndarray) -> Optional[int]:
        """
        Get optimal control for a given state.
        
        Args:
            state: Current configuration
        
        Returns:
            Optimal control index, or None if state not in roadmap
        """
        if self.mdp is None:
            return None
        
        # Find nearest sampled state
        state_idx = nearest_neighbor(state, self.states, self.connection_radius)
        
        if state_idx is None:
            return None
        
        return self.mdp.get_optimal_control(state_idx)
    
    def save(self, filename: str):
        """Save SMR to file."""
        data = {
            'states': self.states,
            'mdp': self.mdp,
            'obstacle_state_idx': self.obstacle_state_idx,
            'goal_state_indices': self.goal_state_indices,
            'connection_radius': self.connection_radius,
            'noise_std': self.noise_std
        }
        with open(filename, 'wb') as f:
            pickle.dump(data, f)
    
    def load(self, filename: str):
        """Load SMR from file."""
        with open(filename, 'rb') as f:
            data = pickle.load(f)
        
        self.states = data['states']
        self.mdp = data['mdp']
        self.obstacle_state_idx = data['obstacle_state_idx']
        self.goal_state_indices = data['goal_state_indices']
        self.connection_radius = data['connection_radius']
        self.noise_std = data['noise_std']
