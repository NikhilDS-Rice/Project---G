"""
Markov Decision Process (MDP) and Value Iteration implementation.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional


class MDP:
    """
    Markov Decision Process implementation.
    
    Components:
    - Q: Set of states
    - U: Set of controls/actions
    - P: Transition probabilities P(q'|q,u)
    - R: Reward function R(q)
    """
    
    def __init__(self, n_states: int, n_controls: int):
        """
        Initialize MDP.
        
        Args:
            n_states: Number of states in the MDP
            n_controls: Number of control actions
        """
        self.n_states = n_states
        self.n_controls = n_controls
        
        # Transition probabilities: P[state_idx][control_idx][next_state_idx]
        # Store as dict of dicts for sparse representation
        self.transitions: Dict[int, Dict[int, Dict[int, float]]] = {}
        
        # Reward function
        self.rewards = np.zeros(n_states)
        
        # State values (computed by value iteration)
        self.values = np.zeros(n_states)
        
        # Optimal policy: policy[state_idx] = control_idx
        self.policy = np.zeros(n_states, dtype=int)
        
        # Goal and obstacle state indices
        self.goal_states = set()
        self.obstacle_states = set()
    
    def add_transition(self, state_idx: int, control_idx: int, 
                      next_state_idx: int, probability: float):
        """
        Add a transition probability.
        
        Args:
            state_idx: Current state index
            control_idx: Control action index
            next_state_idx: Next state index
            probability: Transition probability
        """
        if state_idx not in self.transitions:
            self.transitions[state_idx] = {}
        if control_idx not in self.transitions[state_idx]:
            self.transitions[state_idx][control_idx] = {}
        
        self.transitions[state_idx][control_idx][next_state_idx] = probability
    
    def set_reward(self, state_idx: int, reward: float):
        """Set reward for a state."""
        self.rewards[state_idx] = reward
    
    def set_goal_state(self, state_idx: int):
        """Mark a state as a goal state with reward 1."""
        self.goal_states.add(state_idx)
        self.rewards[state_idx] = 1.0
    
    def set_obstacle_state(self, state_idx: int):
        """Mark a state as an obstacle state with reward 0."""
        self.obstacle_states.add(state_idx)
        self.rewards[state_idx] = 0.0
    
    def is_terminal_state(self, state_idx: int) -> bool:
        """Check if state is terminal (goal or obstacle)."""
        return state_idx in self.goal_states or state_idx in self.obstacle_states
    
    def value_iteration(self, max_iterations: int = 1000, 
                       tolerance: float = 1e-6) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform value iteration to compute optimal policy.
        
        Bellman equation:
        V_{t+1}(q) = R(q) + max_u ∑_{q'} P(q'|q,u) V_t(q')
        
        Args:
            max_iterations: Maximum number of iterations
            tolerance: Convergence threshold
        
        Returns:
            Tuple of (values, policy)
        """
        # Initialize values
        self.values = self.rewards.copy()
        
        for iteration in range(max_iterations):
            old_values = self.values.copy()
            
            # Update value for each state
            for state_idx in range(self.n_states):
                # Skip terminal states (goal and obstacle)
                if self.is_terminal_state(state_idx):
                    continue
                
                # Skip states with no transitions
                if state_idx not in self.transitions:
                    continue
                
                # Compute Q-value for each action
                q_values = []
                for control_idx in range(self.n_controls):
                    if control_idx not in self.transitions[state_idx]:
                        q_values.append(0.0)
                        continue
                    
                    # Compute expected value: ∑_{q'} P(q'|q,u) V(q')
                    expected_value = 0.0
                    for next_state_idx, prob in self.transitions[state_idx][control_idx].items():
                        expected_value += prob * old_values[next_state_idx]
                    
                    q_values.append(expected_value)
                
                # Update value with best action
                if q_values:
                    best_control = np.argmax(q_values)
                    self.values[state_idx] = self.rewards[state_idx] + q_values[best_control]
                    self.policy[state_idx] = best_control
            
            # Check convergence
            max_change = np.max(np.abs(self.values - old_values))
            if max_change < tolerance:
                print(f"Value iteration converged after {iteration + 1} iterations")
                break
        else:
            print(f"Value iteration did not converge after {max_iterations} iterations")
        
        return self.values, self.policy
    
    def get_optimal_control(self, state_idx: int) -> int:
        """
        Get the optimal control for a given state.
        
        Args:
            state_idx: State index
        
        Returns:
            Optimal control index
        """
        return self.policy[state_idx]
    
    def get_state_value(self, state_idx: int) -> float:
        """
        Get the value (expected probability of success) for a state.
        
        Args:
            state_idx: State index
        
        Returns:
            State value
        """
        return self.values[state_idx]
    
    def get_transition_probability(self, state_idx: int, control_idx: int, 
                                   next_state_idx: int) -> float:
        """
        Get transition probability P(next_state | state, control).
        
        Args:
            state_idx: Current state index
            control_idx: Control action index
            next_state_idx: Next state index
        
        Returns:
            Transition probability (0 if not defined)
        """
        if state_idx not in self.transitions:
            return 0.0
        if control_idx not in self.transitions[state_idx]:
            return 0.0
        return self.transitions[state_idx][control_idx].get(next_state_idx, 0.0)
