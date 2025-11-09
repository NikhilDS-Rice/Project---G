"""
Utility functions for SMR implementation.
"""

import numpy as np
from typing import List, Optional
from scipy.spatial import cKDTree


def nearest_neighbor(query_state: np.ndarray, 
                     states: List[np.ndarray],
                     max_radius: Optional[float] = None) -> Optional[int]:
    """
    Find the nearest neighbor to a query state.
    
    Args:
        query_state: Query configuration [x, y, θ]
        states: List of sampled states
        max_radius: Maximum search radius (None for no limit)
    
    Returns:
        Index of nearest neighbor, or None if no neighbor within max_radius
    """
    if len(states) == 0:
        return None
    
    # Use only (x, y) for nearest neighbor search
    positions = np.array([s[:2] for s in states])
    query_pos = query_state[:2]
    
    # Use KD-tree for efficient nearest neighbor search
    tree = cKDTree(positions)
    dist, idx = tree.query(query_pos)
    
    # Check if within max radius
    if max_radius is not None and dist > max_radius:
        return None
    
    return idx


def compute_path_length(path: List[np.ndarray]) -> float:
    """
    Compute the total length of a path.
    
    Args:
        path: List of states [x, y, θ]
    
    Returns:
        Total path length
    """
    if len(path) < 2:
        return 0.0
    
    length = 0.0
    for i in range(len(path) - 1):
        length += np.linalg.norm(path[i][:2] - path[i+1][:2])
    
    return length


def interpolate_path(path: List[np.ndarray], num_points: int) -> List[np.ndarray]:
    """
    Interpolate a path to have a specific number of points.
    
    Args:
        path: List of states [x, y, θ]
        num_points: Desired number of points
    
    Returns:
        Interpolated path
    """
    if len(path) < 2:
        return path
    
    # Compute cumulative arc lengths
    arc_lengths = [0.0]
    for i in range(len(path) - 1):
        dist = np.linalg.norm(path[i][:2] - path[i+1][:2])
        arc_lengths.append(arc_lengths[-1] + dist)
    
    total_length = arc_lengths[-1]
    
    # Interpolate
    interpolated_path = []
    for i in range(num_points):
        target_length = (i / (num_points - 1)) * total_length
        
        # Find segment
        for j in range(len(arc_lengths) - 1):
            if arc_lengths[j] <= target_length <= arc_lengths[j + 1]:
                # Interpolate within segment
                t = (target_length - arc_lengths[j]) / (arc_lengths[j + 1] - arc_lengths[j])
                state = (1 - t) * path[j] + t * path[j + 1]
                interpolated_path.append(state)
                break
    
    return interpolated_path


def angle_difference(theta1: float, theta2: float) -> float:
    """
    Compute the smallest angle difference between two angles.
    
    Args:
        theta1: First angle in radians
        theta2: Second angle in radians
    
    Returns:
        Angle difference in range [-π, π]
    """
    diff = theta2 - theta1
    return np.arctan2(np.sin(diff), np.cos(diff))


def generate_circle_obstacle(center: tuple, radius: float, num_points: int = 20) -> List[tuple]:
    """
    Generate vertices for a circular obstacle.
    
    Args:
        center: (x, y) center of circle
        radius: Radius of circle
        num_points: Number of vertices to approximate circle
    
    Returns:
        List of (x, y) vertices
    """
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    vertices = []
    for angle in angles:
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        vertices.append((x, y))
    return vertices


def generate_rectangle_obstacle(center: tuple, width: float, height: float) -> List[tuple]:
    """
    Generate vertices for a rectangular obstacle.
    
    Args:
        center: (x, y) center of rectangle
        width: Width of rectangle
        height: Height of rectangle
    
    Returns:
        List of (x, y) vertices
    """
    x, y = center
    hw, hh = width / 2, height / 2
    return [
        (x - hw, y - hh),
        (x + hw, y - hh),
        (x + hw, y + hh),
        (x - hw, y + hh)
    ]
