"""
Tests for collision checker.
"""

import unittest
import numpy as np
from src.collision_checker import CollisionChecker


class TestCollisionChecker(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a simple environment with one rectangular obstacle
        self.workspace_bounds = (0, 10, 0, 10)
        obstacles = [
            [(2, 2), (4, 2), (4, 4), (2, 4)]  # Square obstacle
        ]
        self.checker = CollisionChecker(obstacles, self.workspace_bounds)
    
    def test_point_collision(self):
        """Test point collision detection."""
        # Point inside obstacle
        self.assertTrue(self.checker.is_point_in_collision((3, 3)))
        
        # Point outside obstacle
        self.assertFalse(self.checker.is_point_in_collision((5, 5)))
        
        # Point outside workspace
        self.assertTrue(self.checker.is_point_in_collision((15, 5)))
    
    def test_state_collision(self):
        """Test state collision detection."""
        # State in collision
        state1 = np.array([3, 3, 0])
        self.assertTrue(self.checker.is_state_in_collision(state1))
        
        # State collision-free
        state2 = np.array([5, 5, 0])
        self.assertFalse(self.checker.is_state_in_collision(state2))
    
    def test_buffer_radius(self):
        """Test collision checking with buffer radius."""
        # Point just outside obstacle
        point = (4.1, 3)
        
        # No buffer - should be free
        self.assertFalse(self.checker.is_point_in_collision(point, buffer_radius=0))
        
        # With buffer - might collide
        self.assertTrue(self.checker.is_point_in_collision(point, buffer_radius=0.2))


if __name__ == '__main__':
    unittest.main()
