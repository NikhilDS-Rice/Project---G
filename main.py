"""
Main script to run SMR experiments.
"""

import argparse
import os

from src.environment import (
    create_simple_environment,
    create_corridor_environment,
    create_complex_environment,
    create_maze_environment,
    save_environment
)


def setup_environments():
    """Create and save all environment configurations."""
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
        print(f"Created {filepath}")


def main():
    parser = argparse.ArgumentParser(description='SMR Main Script')
    parser.add_argument('--setup', action='store_true',
                       help='Setup environment files')
    parser.add_argument('--part', type=int, choices=[1, 2],
                       help='Run part 1 or part 2')
    parser.add_argument('--env', type=str, default='environments/simple_env.json',
                       help='Environment configuration file')
    
    args = parser.parse_args()
    
    if args.setup:
        print("Setting up environments...")
        setup_environments()
        print("\nEnvironment setup complete!")
        print("\nYou can now run:")
        print("  python part1.py --env environments/simple_env.json")
        print("  python part2.py --env environments/simple_env.json")
    elif args.part == 1:
        print("Running Part 1...")
        os.system(f"python part1.py --env {args.env}")
    elif args.part == 2:
        print("Running Part 2...")
        os.system(f"python part2.py --env {args.env}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
