"""
Plotting utilities for generating publication-quality figures.
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from typing import List, Dict
import os


def set_plot_style():
    """Set publication-quality plot style."""
    sns.set_style("whitegrid")
    plt.rcParams['figure.figsize'] = (10, 8)
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['axes.titlesize'] = 16
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 12
    plt.rcParams['legend.fontsize'] = 12
    plt.rcParams['figure.titlesize'] = 18


def plot_success_rate_comparison(data: Dict, save_path: str = None):
    """
    Plot comparison of expected vs actual success rates.
    
    Args:
        data: Dictionary with 'expected' and 'actual' lists
        save_path: Path to save figure
    """
    set_plot_style()
    
    fig, ax = plt.subplots()
    
    x = np.arange(len(data['expected']))
    width = 0.35
    
    ax.bar(x - width/2, data['expected'], width, label='Expected', alpha=0.8)
    ax.bar(x + width/2, data['actual'], width, label='Actual', alpha=0.8)
    
    ax.set_ylabel('Success Rate')
    ax.set_title('Expected vs Actual Success Rate')
    ax.set_xticks(x)
    ax.set_xticklabels([f"Trial {i+1}" for i in x])
    ax.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    
    plt.close()


def plot_convergence(values_history: List[np.ndarray], save_path: str = None):
    """
    Plot value iteration convergence.
    
    Args:
        values_history: List of value arrays at each iteration
        save_path: Path to save figure
    """
    set_plot_style()
    
    fig, ax = plt.subplots()
    
    max_values = [np.max(v) for v in values_history]
    mean_values = [np.mean(v) for v in values_history]
    
    iterations = np.arange(len(values_history))
    
    ax.plot(iterations, max_values, 'b-', label='Max Value', linewidth=2)
    ax.plot(iterations, mean_values, 'r-', label='Mean Value', linewidth=2)
    
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Value')
    ax.set_title('Value Iteration Convergence')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    
    plt.close()


def create_results_summary(results: Dict, output_file: str):
    """
    Create a summary report of results.
    
    Args:
        results: Dictionary containing results
        output_file: Path to output text file
    """
    with open(output_file, 'w') as f:
        f.write("="*60 + "\n")
        f.write("SMR EXPERIMENT RESULTS SUMMARY\n")
        f.write("="*60 + "\n\n")
        
        for key, value in results.items():
            if isinstance(value, (int, float)):
                f.write(f"{key}: {value}\n")
            elif isinstance(value, list):
                f.write(f"{key}: {len(value)} items\n")
            elif isinstance(value, dict):
                f.write(f"{key}:\n")
                for k, v in value.items():
                    f.write(f"  {k}: {v}\n")
            else:
                f.write(f"{key}: {type(value)}\n")
        
        f.write("\n" + "="*60 + "\n")


def plot_path_diversity(trajectories: List[List[np.ndarray]], 
                        save_path: str = None):
    """
    Visualize diversity of execution paths.
    
    Args:
        trajectories: List of trajectories
        save_path: Path to save figure
    """
    set_plot_style()
    
    fig, ax = plt.subplots()
    
    # Compute path lengths
    lengths = []
    for traj in trajectories:
        length = 0
        for i in range(len(traj) - 1):
            length += np.linalg.norm(traj[i][:2] - traj[i+1][:2])
        lengths.append(length)
    
    ax.hist(lengths, bins=20, alpha=0.7, edgecolor='black')
    ax.set_xlabel('Path Length')
    ax.set_ylabel('Frequency')
    ax.set_title('Distribution of Path Lengths')
    ax.axvline(np.mean(lengths), color='r', linestyle='--', 
               linewidth=2, label=f'Mean: {np.mean(lengths):.2f}')
    ax.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    
    plt.close()


if __name__ == "__main__":
    # Example usage
    print("Plotting utilities loaded successfully!")
