"""
Visualization utilities for SMR.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MPLPolygon, FancyArrow
from matplotlib.collections import LineCollection
from typing import List, Optional, Tuple

from src.collision_checker import CollisionChecker


class SMRVisualizer:
    """
    Visualizer for Stochastic Motion Roadmap.
    """
    
    def __init__(self, collision_checker: CollisionChecker, 
                 workspace_bounds: Tuple[float, float, float, float]):
        """
        Initialize visualizer.
        
        Args:
            collision_checker: Collision checker with obstacles
            workspace_bounds: (x_min, x_max, y_min, y_max)
        """
        self.collision_checker = collision_checker
        self.workspace_bounds = workspace_bounds
    
    def plot_environment(self, ax: Optional[plt.Axes] = None, 
                        show_grid: bool = True) -> plt.Axes:
        """
        Plot the environment with obstacles.
        
        Args:
            ax: Matplotlib axes (creates new if None)
            show_grid: Whether to show grid
        
        Returns:
            Matplotlib axes
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
        # Set workspace bounds
        x_min, x_max, y_min, y_max = self.workspace_bounds
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        
        # Plot obstacles
        for obstacle in self.collision_checker.get_obstacles():
            coords = list(obstacle.exterior.coords)
            polygon = MPLPolygon(coords, facecolor='gray', 
                               edgecolor='black', alpha=0.7)
            ax.add_patch(polygon)
        
        if show_grid:
            ax.grid(True, alpha=0.3)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        
        return ax
    
    def plot_states(self, states: List[np.ndarray], ax: plt.Axes,
                   color: str = 'blue', alpha: float = 0.3, size: float = 10):
        """
        Plot sampled states.
        
        Args:
            states: List of states [x, y, θ]
            ax: Matplotlib axes
            color: Color of points
            alpha: Transparency
            size: Point size
        """
        positions = np.array([s[:2] for s in states])
        ax.scatter(positions[:, 0], positions[:, 1], 
                  c=color, alpha=alpha, s=size, label='Sampled States')
    
    def plot_goal_region(self, goal_center: np.ndarray, goal_radius: float,
                        ax: plt.Axes):
        """
        Plot goal region.
        
        Args:
            goal_center: Center of goal region [x, y, θ]
            goal_radius: Radius of goal region
            ax: Matplotlib axes
        """
        circle = Circle(goal_center[:2], goal_radius, 
                       facecolor='green', edgecolor='darkgreen',
                       alpha=0.3, linewidth=2, label='Goal Region')
        ax.add_patch(circle)
    
    def plot_trajectory(self, trajectory: List[np.ndarray], ax: plt.Axes,
                       color: str = 'red', alpha: float = 0.8, 
                       linewidth: float = 2, show_orientation: bool = True,
                       label: Optional[str] = None):
        """
        Plot a trajectory.
        
        Args:
            trajectory: List of states [x, y, θ]
            ax: Matplotlib axes
            color: Line color
            alpha: Transparency
            linewidth: Line width
            show_orientation: Whether to show orientation arrows
            label: Legend label
        """
        if len(trajectory) < 2:
            return
        
        positions = np.array([s[:2] for s in trajectory])
        ax.plot(positions[:, 0], positions[:, 1], 
               color=color, alpha=alpha, linewidth=linewidth, label=label)
        
        # Plot start and end points
        ax.plot(positions[0, 0], positions[0, 1], 'o', 
               color=color, markersize=8, label='Start' if label is None else None)
        ax.plot(positions[-1, 0], positions[-1, 1], 's', 
               color=color, markersize=8, label='End' if label is None else None)
        
        # Plot orientation arrows at intervals
        if show_orientation:
            arrow_interval = max(1, len(trajectory) // 10)
            for i in range(0, len(trajectory), arrow_interval):
                x, y, theta = trajectory[i]
                dx = 0.3 * np.cos(theta)
                dy = 0.3 * np.sin(theta)
                arrow = FancyArrow(x, y, dx, dy, 
                                  width=0.1, head_width=0.2, 
                                  head_length=0.15,
                                  color=color, alpha=alpha * 0.7)
                ax.add_patch(arrow)
    
    def plot_multiple_trajectories(self, trajectories: List[List[np.ndarray]], 
                                   ax: plt.Axes, cmap: str = 'viridis',
                                   alpha: float = 0.3):
        """
        Plot multiple trajectories with a color map.
        
        Args:
            trajectories: List of trajectories
            ax: Matplotlib axes
            cmap: Colormap name
            alpha: Transparency
        """
        colormap = plt.cm.get_cmap(cmap)
        
        for i, trajectory in enumerate(trajectories):
            if len(trajectory) < 2:
                continue
            color = colormap(i / max(len(trajectories) - 1, 1))
            positions = np.array([s[:2] for s in trajectory])
            ax.plot(positions[:, 0], positions[:, 1], 
                   color=color, alpha=alpha, linewidth=1)
    
    def plot_value_function(self, states: List[np.ndarray], 
                           values: np.ndarray, ax: plt.Axes,
                           cmap: str = 'RdYlGn', vmin: float = 0.0, 
                           vmax: float = 1.0):
        """
        Plot value function as a heatmap.
        
        Args:
            states: List of states [x, y, θ]
            values: Value for each state
            ax: Matplotlib axes
            cmap: Colormap name
            vmin: Minimum value for colormap
            vmax: Maximum value for colormap
        """
        positions = np.array([s[:2] for s in states])
        scatter = ax.scatter(positions[:, 0], positions[:, 1], 
                           c=values, cmap=cmap, s=50, 
                           vmin=vmin, vmax=vmax, alpha=0.8)
        plt.colorbar(scatter, ax=ax, label='Probability of Success')
    
    def create_animation_frame(self, current_state: np.ndarray,
                              trajectory_history: List[np.ndarray],
                              goal_center: np.ndarray, goal_radius: float,
                              title: str = "") -> plt.Figure:
        """
        Create a single animation frame.
        
        Args:
            current_state: Current state [x, y, θ]
            trajectory_history: History of states
            goal_center: Goal region center
            goal_radius: Goal region radius
            title: Plot title
        
        Returns:
            Matplotlib figure
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Plot environment
        self.plot_environment(ax)
        
        # Plot goal region
        self.plot_goal_region(goal_center, goal_radius, ax)
        
        # Plot trajectory history
        if len(trajectory_history) > 1:
            positions = np.array([s[:2] for s in trajectory_history])
            ax.plot(positions[:, 0], positions[:, 1], 
                   'b-', alpha=0.5, linewidth=2, label='Trajectory')
        
        # Plot current state
        x, y, theta = current_state
        ax.plot(x, y, 'ro', markersize=10, label='Current Position')
        
        # Plot orientation
        dx = 0.5 * np.cos(theta)
        dy = 0.5 * np.sin(theta)
        arrow = FancyArrow(x, y, dx, dy, width=0.15, 
                          head_width=0.3, head_length=0.2,
                          color='red', alpha=0.8)
        ax.add_patch(arrow)
        
        ax.set_title(title)
        ax.legend()
        
        return fig
