"""
Unit tests for Steerable Needle model.
"""

import unittest
import numpy as np
from src.steerable_needle import SteerableNeedle


class TestSteerableNeedle(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures."""
        self.workspace_bounds = (0, 10, 0, 10)
        self.needle = SteerableNeedle(self.workspace_bounds)
    
    def test_initialization(self):
        """Test steerable needle initialization."""
        self.assertEqual(len(self.needle.control_set), 5)
        self.assertEqual(self.needle.workspace_bounds, self.workspace_bounds)
    
    def test_straight_motion(self):
        """Test straight line motion (zero curvature)."""
        state = np.array([5.0, 5.0, 0.0])  # Start at (5,5) facing right
        control_idx = 2  # Straight motion
        
        new_state = self.needle.apply_control(state, control_idx, noise_std=0.0)
        
        # Should move right by step_size
        expected_x = 5.0 + self.needle.step_size
        self.assertAlmostEqual(new_state[0], expected_x, places=5)
        self.assertAlmostEqual(new_state[1], 5.0, places=5)
        self.assertAlmostEqual(new_state[2], 0.0, places=5)
    
    def test_curved_motion(self):
        """Test curved motion."""
        state = np.array([5.0, 5.0, 0.0])
        control_idx = 0  # Max left turn
        
        new_state = self.needle.apply_control(state, control_idx, noise_std=0.0)
        
        # Should not be at the same position
        self.assertNotEqual(new_state[0], state[0])
        # Orientation should change
        self.assertNotEqual(new_state[2], state[2])
    
    def test_valid_state(self):
        """Test state validity checking."""
        valid_state = np.array([5.0, 5.0, 0.0])
        invalid_state = np.array([15.0, 5.0, 0.0])
        
        self.assertTrue(self.needle.is_valid_state(valid_state))
        self.assertFalse(self.needle.is_valid_state(invalid_state))
    
    def test_random_sampling(self):
        """Test random state sampling."""
        for _ in range(100):
            state = self.needle.sample_random_state()
            self.assertTrue(self.needle.is_valid_state(state))
    
    def test_distance(self):
        """Test distance computation."""
        state1 = np.array([0.0, 0.0, 0.0])
        state2 = np.array([3.0, 4.0, 0.0])
        
        dist = self.needle.distance(state1, state2)
        self.assertAlmostEqual(dist, 5.0, places=5)


if __name__ == '__main__':
    unittest.main()
