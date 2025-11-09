"""
Collision Checker for 2D environments with obstacles.
"""

import numpy as np
from typing import List, Tuple
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union


class CollisionChecker:
    """
    Collision checker for 2D workspace with polygonal obstacles.
    """
    
    def __init__(self, obstacles: List[List[Tuple[float, float]]], 
                 workspace_bounds: Tuple[float, float, float, float]):
        """
        Initialize collision checker.
        
        Args:
            obstacles: List of obstacles, where each obstacle is a list of vertices
            workspace_bounds: (x_min, x_max, y_min, y_max)
        """
        self.workspace_bounds = workspace_bounds
        self.obstacles = [Polygon(obs) for obs in obstacles]
        
        # Union of all obstacles for efficient collision checking
        if self.obstacles:
            self.obstacle_union = unary_union(self.obstacles)
        else:
            self.obstacle_union = None
    
    def is_point_in_collision(self, point: Tuple[float, float], 
                             buffer_radius: float = 0.0) -> bool:
        """
        Check if a point is in collision with obstacles.
        
        Args:
            point: (x, y) coordinates
            buffer_radius: Safety margin around obstacles
        
        Returns:
            True if point is in collision
        """
        # Check workspace bounds
        x, y = point
        x_min, x_max, y_min, y_max = self.workspace_bounds
        if not (x_min <= x <= x_max and y_min <= y <= y_max):
            return True
        
        # Check obstacle collision
        if self.obstacle_union is None:
            return False
        
        pt = Point(point)
        if buffer_radius > 0:
            pt = pt.buffer(buffer_radius)
        
        return self.obstacle_union.intersects(pt)
    
    def is_state_in_collision(self, state: np.ndarray, 
                             buffer_radius: float = 0.0) -> bool:
        """
        Check if a state is in collision.
        
        Args:
            state: [x, y, theta] configuration
            buffer_radius: Safety margin around obstacles
        
        Returns:
            True if state is in collision
        """
        return self.is_point_in_collision((state[0], state[1]), buffer_radius)
    
    def is_path_in_collision(self, path: List[np.ndarray], 
                            buffer_radius: float = 0.0) -> bool:
        """
        Check if a path is in collision.
        
        Args:
            path: List of states [x, y, theta]
            buffer_radius: Safety margin around obstacles
        
        Returns:
            True if any part of the path is in collision
        """
        for state in path:
            if self.is_state_in_collision(state, buffer_radius):
                return True
        return False
    
    def get_obstacles(self) -> List[Polygon]:
        """Get list of obstacle polygons."""
        return self.obstacles
    
    def add_obstacle(self, vertices: List[Tuple[float, float]]):
        """
        Add a new obstacle.
        
        Args:
            vertices: List of vertices defining the obstacle polygon
        """
        obstacle = Polygon(vertices)
        self.obstacles.append(obstacle)
        
        # Update obstacle union
        if self.obstacle_union is None:
            self.obstacle_union = obstacle
        else:
            self.obstacle_union = unary_union([self.obstacle_union, obstacle])
