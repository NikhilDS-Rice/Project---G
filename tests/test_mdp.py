"""
Unit tests for MDP implementation.
"""

import unittest
import numpy as np
from src.mdp import MDP


class TestMDP(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures."""
        self.mdp = MDP(n_states=5, n_controls=3)
    
    def test_initialization(self):
        """Test MDP initialization."""
        self.assertEqual(self.mdp.n_states, 5)
        self.assertEqual(self.mdp.n_controls, 3)
        self.assertEqual(len(self.mdp.values), 5)
    
    def test_add_transition(self):
        """Test adding transitions."""
        self.mdp.add_transition(0, 1, 2, 0.5)
        self.mdp.add_transition(0, 1, 3, 0.5)
        
        prob1 = self.mdp.get_transition_probability(0, 1, 2)
        prob2 = self.mdp.get_transition_probability(0, 1, 3)
        
        self.assertEqual(prob1, 0.5)
        self.assertEqual(prob2, 0.5)
    
    def test_rewards(self):
        """Test reward setting."""
        self.mdp.set_reward(0, 1.0)
        self.mdp.set_goal_state(1)
        
        self.assertEqual(self.mdp.rewards[0], 1.0)
        self.assertEqual(self.mdp.rewards[1], 1.0)
        self.assertTrue(1 in self.mdp.goal_states)
    
    def test_value_iteration_simple(self):
        """Test value iteration on a simple MDP."""
        # Create a simple chain: 0 -> 1 -> 2 (goal)
        self.mdp.set_goal_state(2)
        
        # Deterministic transitions
        self.mdp.add_transition(0, 0, 1, 1.0)
        self.mdp.add_transition(1, 0, 2, 1.0)
        
        values, policy = self.mdp.value_iteration()
        
        # State 2 is goal, should have value 1
        self.assertEqual(values[2], 1.0)
        # State 1 should lead to goal with probability 1
        self.assertAlmostEqual(values[1], 1.0, places=5)
        # State 0 should lead to goal with probability 1
        self.assertAlmostEqual(values[0], 1.0, places=5)
    
    def test_terminal_states(self):
        """Test terminal state handling."""
        self.mdp.set_goal_state(0)
        self.mdp.set_obstacle_state(1)
        
        self.assertTrue(self.mdp.is_terminal_state(0))
        self.assertTrue(self.mdp.is_terminal_state(1))
        self.assertFalse(self.mdp.is_terminal_state(2))


if __name__ == '__main__':
    unittest.main()
