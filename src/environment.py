"""
Environment configurations for SMR testing.
"""

import json
from typing import List, Tuple
from src.utils import generate_circle_obstacle, generate_rectangle_obstacle


def create_simple_environment() -> dict:
    """
    Create a simple environment with a few obstacles.
    """
    obstacles = [
        generate_rectangle_obstacle((5, 5), 2, 4),
        generate_circle_obstacle((8, 8), 1.5),
    ]
    
    config = {
        "name": "Simple Environment",
        "workspace_bounds": [0, 10, 0, 10],
        "obstacles": obstacles,
        "start": [1, 1, 0],
        "goal_center": [9, 9, 0],
        "goal_radius": 0.5
    }
    
    return config


def create_corridor_environment() -> dict:
    """
    Create a corridor environment.
    """
    obstacles = [
        # Top wall with gap
        generate_rectangle_obstacle((2.5, 8), 5, 0.5),
        generate_rectangle_obstacle((7.5, 8), 5, 0.5),
        # Bottom wall with gap
        generate_rectangle_obstacle((2.5, 2), 5, 0.5),
        generate_rectangle_obstacle((7.5, 2), 5, 0.5),
    ]
    
    config = {
        "name": "Corridor Environment",
        "workspace_bounds": [0, 10, 0, 10],
        "obstacles": obstacles,
        "start": [1, 5, 0],
        "goal_center": [9, 5, 0],
        "goal_radius": 0.5
    }
    
    return config


def create_complex_environment() -> dict:
    """
    Create a complex environment with multiple obstacles.
    """
    obstacles = [
        generate_rectangle_obstacle((3, 3), 1.5, 1.5),
        generate_rectangle_obstacle((7, 3), 1.5, 1.5),
        generate_circle_obstacle((5, 5), 1.2),
        generate_rectangle_obstacle((3, 7), 1.5, 1.5),
        generate_rectangle_obstacle((7, 7), 1.5, 1.5),
        generate_circle_obstacle((2, 8), 0.8),
        generate_circle_obstacle((8, 2), 0.8),
    ]
    
    config = {
        "name": "Complex Environment",
        "workspace_bounds": [0, 10, 0, 10],
        "obstacles": obstacles,
        "start": [0.5, 0.5, 0.785],  # 45 degrees
        "goal_center": [9.5, 9.5, 0],
        "goal_radius": 0.4
    }
    
    return config


def create_maze_environment() -> dict:
    """
    Create a maze-like environment.
    """
    obstacles = [
        # Vertical walls
        generate_rectangle_obstacle((3, 5), 0.3, 8),
        generate_rectangle_obstacle((7, 5), 0.3, 8),
        # Horizontal walls
        generate_rectangle_obstacle((5, 3), 3, 0.3),
        generate_rectangle_obstacle((5, 7), 3, 0.3),
        # Central obstacle
        generate_circle_obstacle((5, 5), 0.8),
    ]
    
    config = {
        "name": "Maze Environment",
        "workspace_bounds": [0, 10, 0, 10],
        "obstacles": obstacles,
        "start": [1, 1, 0],
        "goal_center": [9, 9, 0],
        "goal_radius": 0.5
    }
    
    return config


def save_environment(config: dict, filename: str):
    """Save environment configuration to JSON file."""
    # Convert numpy arrays to lists for JSON serialization
    json_config = {
        "name": config["name"],
        "workspace_bounds": config["workspace_bounds"],
        "obstacles": config["obstacles"],
        "start": config["start"],
        "goal_center": config["goal_center"],
        "goal_radius": config["goal_radius"]
    }
    
    with open(filename, 'w') as f:
        json.dump(json_config, f, indent=2)


def load_environment(filename: str) -> dict:
    """Load environment configuration from JSON file."""
    with open(filename, 'r') as f:
        config = json.load(f)
    return config


if __name__ == "__main__":
    # Generate and save environment configurations
    import os
    
    env_dir = "environments"
    os.makedirs(env_dir, exist_ok=True)
    
    environments = [
        (create_simple_environment(), "simple_env.json"),
        (create_corridor_environment(), "corridor_env.json"),
        (create_complex_environment(), "complex_env.json"),
        (create_maze_environment(), "maze_env.json"),
    ]
    
    for env_config, filename in environments:
        filepath = os.path.join(env_dir, filename)
        save_environment(env_config, filepath)
        print(f"Saved {filepath}")
