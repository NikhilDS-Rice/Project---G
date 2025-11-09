"""
OMPL-based Stochastic Motion Roadmap implementation.
Requires: ompl library (install via Docker or conda)
"""

import numpy as np
from typing import List, Tuple, Optional
import pickle

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    OMPL_AVAILABLE = True
except ImportError:
    OMPL_AVAILABLE = False
    print("Warning: OMPL not available. Please install it or use Docker.")


class OMPLSteerableNeedle:
    """
    OMPL-based steerable needle implementation using control space.
    """
    
    def __init__(self, workspace_bounds: Tuple[float, float, float, float],
                 max_curvature: float = 1.0,
                 step_size: float = 0.5,
                 noise_std: float = 0.1):
        """
        Initialize OMPL-based steerable needle.
        
        Args:
            workspace_bounds: (x_min, x_max, y_min, y_max)
            max_curvature: Maximum curvature
            step_size: Distance per control action
            noise_std: Noise standard deviation
        """
        if not OMPL_AVAILABLE:
            raise ImportError("OMPL is required for this implementation")
        
        self.workspace_bounds = workspace_bounds
        self.max_curvature = max_curvature
        self.step_size = step_size
        self.noise_std = noise_std
        
        # Create OMPL state space (SE2: x, y, theta)
        self.space = ob.SE2StateSpace()
        
        # Set bounds
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, workspace_bounds[0])  # x_min
        bounds.setHigh(0, workspace_bounds[1]) # x_max
        bounds.setLow(1, workspace_bounds[2])  # y_min
        bounds.setHigh(1, workspace_bounds[3]) # y_max
        self.space.setBounds(bounds)
        
        # Create control space (curvature)
        self.control_space = oc.RealVectorControlSpace(self.space, 1)
        
        # Set control bounds
        cbounds = ob.RealVectorBounds(1)
        cbounds.setLow(-max_curvature)
        cbounds.setHigh(max_curvature)
        self.control_space.setBounds(cbounds)
        
        # Create simple setup for control-based planning
        self.ss = oc.SimpleSetup(self.control_space)
        
    def propagate(self, start, control, duration, result):
        """
        Propagate function for OMPL control space.
        Implements the steerable needle dynamics with noise.
        """
        # Extract state
        x = start.getX()
        y = start.getY()
        theta = start.getYaw()
        
        # Extract control (curvature)
        curvature = control[0]
        
        # Add Gaussian noise
        if self.noise_std > 0:
            curvature += np.random.normal(0, self.noise_std)
        
        # Compute new state
        if abs(curvature) < 1e-6:  # Straight line
            x_new = x + self.step_size * np.cos(theta)
            y_new = y + self.step_size * np.sin(theta)
            theta_new = theta
        else:  # Arc motion
            radius = 1.0 / curvature
            delta_theta = self.step_size * curvature
            x_new = x + radius * (np.sin(theta + delta_theta) - np.sin(theta))
            y_new = y - radius * (np.cos(theta + delta_theta) - np.cos(theta))
            theta_new = theta + delta_theta
        
        # Set result state
        result.setX(x_new)
        result.setY(y_new)
        result.setYaw(theta_new)
    
    def setup_planning_problem(self, start_state: np.ndarray, 
                               goal_state: np.ndarray,
                               collision_checker):
        """
        Setup OMPL planning problem.
        
        Args:
            start_state: [x, y, theta]
            goal_state: [x, y, theta]
            collision_checker: Collision checker object
        """
        # Set state validity checker
        def isStateValid(state):
            x = state.getX()
            y = state.getY()
            return not collision_checker.is_point_in_collision((x, y))
        
        self.ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
        
        # Set propagation function
        self.ss.setStatePropagator(oc.StatePropagatorFn(self.propagate))
        
        # Set start state
        start = ob.State(self.space)
        start().setX(start_state[0])
        start().setY(start_state[1])
        start().setYaw(start_state[2])
        
        # Set goal state
        goal = ob.State(self.space)
        goal().setX(goal_state[0])
        goal().setY(goal_state[1])
        goal().setYaw(goal_state[2])
        
        self.ss.setStartAndGoalStates(start, goal, 0.5)  # 0.5 threshold
    
    def state_to_array(self, state) -> np.ndarray:
        """Convert OMPL state to numpy array."""
        return np.array([state.getX(), state.getY(), state.getYaw()])
    
    def array_to_state(self, arr: np.ndarray):
        """Convert numpy array to OMPL state."""
        state = ob.State(self.space)
        state().setX(arr[0])
        state().setY(arr[1])
        state().setYaw(arr[2])
        return state


class OMPLStochasticMotionRoadmap:
    """
    OMPL-based SMR implementation combining OMPL's sampling
    with custom MDP and value iteration.
    """
    
    def __init__(self, ompl_needle: OMPLSteerableNeedle,
                 collision_checker,
                 noise_std: float = 0.1):
        """
        Initialize OMPL-based SMR.
        
        Args:
            ompl_needle: OMPL steerable needle instance
            collision_checker: Collision checker
            noise_std: Noise standard deviation
        """
        if not OMPL_AVAILABLE:
            raise ImportError("OMPL is required for this implementation")
        
        self.needle = ompl_needle
        self.collision_checker = collision_checker
        self.noise_std = noise_std
        
        # Sampled states (as numpy arrays)
        self.states: List[np.ndarray] = []
        
        # OMPL sampler
        self.sampler = self.needle.space.allocStateSampler()
        
        # Control set
        self.control_set = self._generate_control_set()
        
        # MDP will be created during learning phase
        self.mdp = None
        self.obstacle_state_idx = None
        self.goal_state_indices = []
        self.connection_radius = 2.0
    
    def _generate_control_set(self) -> List[float]:
        """Generate discrete control set (curvatures)."""
        return [
            -self.needle.max_curvature,
            -self.needle.max_curvature / 2,
            0.0,
            self.needle.max_curvature / 2,
            self.needle.max_curvature
        ]
    
    def sample_state_ompl(self) -> np.ndarray:
        """Sample a state using OMPL's sampler."""
        ompl_state = self.needle.space.allocState()
        self.sampler.sample(ompl_state)
        return self.needle.state_to_array(ompl_state())
    
    def learning_phase_ompl(self, n_states: int, n_simulations: int,
                           goal_region: Tuple[np.ndarray, float],
                           connection_radius: Optional[float] = None):
        """
        Learning phase using OMPL sampling and custom transition estimation.
        
        Args:
            n_states: Number of states to sample
            n_simulations: Simulations per state-action pair
            goal_region: (goal_center, goal_radius)
            connection_radius: Radius for state connections
        """
        from src.mdp import MDP
        from src.utils import nearest_neighbor
        from tqdm import tqdm
        
        if connection_radius is not None:
            self.connection_radius = connection_radius
        
        goal_center, goal_radius = goal_region
        
        print(f"OMPL-SMR Learning Phase: Sampling {n_states} states...")
        
        # Sample valid states using OMPL
        self.states = []
        attempts = 0
        max_attempts = n_states * 100
        
        with tqdm(total=n_states) as pbar:
            while len(self.states) < n_states and attempts < max_attempts:
                state = self.sample_state_ompl()
                
                # Check validity
                if not self.collision_checker.is_state_in_collision(state):
                    self.states.append(state)
                    pbar.update(1)
                
                attempts += 1
        
        print(f"Sampled {len(self.states)} valid states using OMPL")
        
        # Add obstacle state
        self.obstacle_state_idx = len(self.states)
        
        # Initialize MDP
        n_controls = len(self.control_set)
        self.mdp = MDP(len(self.states) + 1, n_controls)
        self.mdp.set_obstacle_state(self.obstacle_state_idx)
        
        # Identify goal states
        self.goal_state_indices = []
        for idx, state in enumerate(self.states):
            dist = np.linalg.norm(state[:2] - goal_center[:2])
            if dist <= goal_radius:
                self.goal_state_indices.append(idx)
                self.mdp.set_goal_state(idx)
        
        print(f"Found {len(self.goal_state_indices)} goal states")
        
        # Estimate transitions using OMPL propagation
        print(f"Estimating transitions with OMPL propagation...")
        
        with tqdm(total=len(self.states) * n_controls) as pbar:
            for state_idx, state in enumerate(self.states):
                if self.mdp.is_terminal_state(state_idx):
                    pbar.update(n_controls)
                    continue
                
                for control_idx, curvature in enumerate(self.control_set):
                    transition_counts = {}
                    
                    for _ in range(n_simulations):
                        # Create OMPL state and control
                        ompl_state = self.needle.array_to_state(state)
                        ompl_result = self.needle.space.allocState()
                        
                        # Create control
                        control = oc.Control(self.needle.control_space)
                        control()[0] = curvature
                        
                        # Propagate with noise
                        self.needle.propagate(ompl_state(), control(), 
                                            self.needle.step_size, ompl_result())
                        
                        next_state = self.needle.state_to_array(ompl_result())
                        
                        # Check collision
                        if self.collision_checker.is_state_in_collision(next_state):
                            next_state_idx = self.obstacle_state_idx
                        else:
                            next_state_idx = nearest_neighbor(
                                next_state, self.states, self.connection_radius
                            )
                            if next_state_idx is None:
                                next_state_idx = self.obstacle_state_idx
                        
                        transition_counts[next_state_idx] = \
                            transition_counts.get(next_state_idx, 0) + 1
                    
                    # Convert to probabilities
                    for next_state_idx, count in transition_counts.items():
                        probability = count / n_simulations
                        self.mdp.add_transition(
                            state_idx, control_idx, next_state_idx, probability
                        )
                    
                    pbar.update(1)
        
        print("OMPL-SMR learning phase complete!")
    
    def query_phase(self, start_state: np.ndarray) -> Tuple[int, float]:
        """Query phase with value iteration."""
        from src.utils import nearest_neighbor
        
        if self.mdp is None:
            raise ValueError("Must run learning_phase_ompl first")
        
        print("OMPL-SMR Query Phase: Computing optimal policy...")
        
        start_state_idx = nearest_neighbor(start_state, self.states,
                                          self.connection_radius)
        
        if start_state_idx is None:
            print("Warning: Start state not near any sampled state")
            return None, 0.0
        
        values, policy = self.mdp.value_iteration()
        prob_success = values[start_state_idx]
        
        print(f"Optimal policy computed. Probability of success: {prob_success:.4f}")
        
        return start_state_idx, prob_success
    
    def get_optimal_control(self, state: np.ndarray) -> Optional[int]:
        """Get optimal control for a given state."""
        from src.utils import nearest_neighbor
        
        if self.mdp is None:
            return None
        
        state_idx = nearest_neighbor(state, self.states, self.connection_radius)
        if state_idx is None:
            return None
        
        return self.mdp.get_optimal_control(state_idx)


def check_ompl_availability():
    """Check if OMPL is available."""
    return OMPL_AVAILABLE


if __name__ == "__main__":
    if check_ompl_availability():
        print("✓ OMPL is available!")
        print("  - State space: SE2 (x, y, theta)")
        print("  - Control space: RealVector (curvature)")
        print("  - Ready for OMPL-based SMR")
    else:
        print("✗ OMPL is not available")
        print("  Please install OMPL or use Docker")
