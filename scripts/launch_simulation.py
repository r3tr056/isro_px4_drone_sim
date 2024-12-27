#!/usr/bin/env python3

import subprocess
import os
import argparse
import sys
from ament_index_python.packages import get_package_share_directory

class SimulationLauncher:
    def __init__(self):
        self.package_name = 'px4_drone_sim'
        try:
            self.package_path = get_package_share_directory(self.package_name)
        except Exception as e:
            print(f"Error: Could not find package '{self.package_name}'. Make sure it's installed correctly.")
            sys.exit(1)

    def get_available_models(self):
        """Get list of available drone models."""
        model_path = os.path.join(self.package_path, "models")
        if not os.path.exists(model_path):
            print(f"Error: Drone models directory not found at {model_path}")
            sys.exit(1)
        return [d for d in os.listdir(model_path) if os.path.isdir(os.path.join(model_path, d))]

    def get_available_worlds(self):
        """Get list of available simulation worlds."""
        worlds_path = os.path.join(self.package_path, "worlds")
        if not os.path.exists(worlds_path):
            print(f"Error: Worlds directory not found at {worlds_path}")
            sys.exit(1)
        return [d for d in os.listdir(worlds_path) if os.path.isdir(os.path.join(worlds_path, d))]

    def validate_paths(self):
        """Validate that all required paths exist."""
        required_paths = {
            'Launch file': os.path.join(self.package_path, 'launch', 'px4_sim.launch.py'),
            'Models directory': os.path.join(self.package_path, 'models'),
            'Worlds directory': os.path.join(self.package_path, 'worlds')
        }
        
        for name, path in required_paths.items():
            if not os.path.exists(path):
                print(f"Error: {name} not found at {path}")
                sys.exit(1)

    def select_model(self, available_models):
        """Interactive model selection."""
        print("\nAvailable Drone Models:")
        for i, model in enumerate(available_models, 1):
            print(f"{i}. {model}")
        while True:
            try:
                model_index = int(input("\nChoose a drone model number: "))
                if 1 <= model_index <= len(available_models):
                    return available_models[model_index - 1]
                print("Error: Invalid model number. Please try again.")
            except ValueError:
                print("Error: Please enter a valid number.")

    def select_world(self, available_worlds):
        """Interactive world selection."""
        print("\nAvailable Worlds:")
        for i, world in enumerate(available_worlds, 1):
            print(f"{i}. {world}")
        while True:
            try:
                world_index = int(input("\nChoose a world number: "))
                if 1 <= world_index <= len(available_worlds):
                    return available_worlds[world_index - 1]
                print("Error: Invalid world number. Please try again.")
            except ValueError:
                print("Error: Please enter a valid number.")

    def generate_launch_command(self, vehicle, world, headless, standalone):
        """Generate the ROS 2 launch command."""
        launch_file = os.path.join(self.package_path, 'launch', 'px4_sim.launch.py')
        return [
            'ros2', 'launch', launch_file,
            f'vehicle:={vehicle}',
            f'world:={world}',
            f'headless:={str(headless).lower()}',
            f'standalone:={str(standalone).lower()}'
        ]

def main():
    parser = argparse.ArgumentParser(
        description='Launch PX4 drone simulation with custom models and worlds.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--vehicle', type=str, help='Vehicle model to use')
    parser.add_argument('--world', type=str, help='World model to use')
    parser.add_argument('--headless', action='store_true', help='Run Gazebo in headless mode')
    parser.add_argument('--standalone', action='store_true', help='Run PX4 and Gazebo in standalone mode')
    args = parser.parse_args()

    try:
        launcher = SimulationLauncher()
        launcher.validate_paths()

        available_models = launcher.get_available_models()
        available_worlds = launcher.get_available_worlds()

        if not available_models:
            print("Error: No drone models found.")
            sys.exit(1)
        if not available_worlds:
            print("Error: No world models found.")
            sys.exit(1)

        # Handle vehicle selection
        vehicle_model = args.vehicle
        if vehicle_model is None:
            vehicle_model = launcher.select_model(available_models)
        elif vehicle_model not in available_models:
            print(f"Error: Vehicle model '{vehicle_model}' not found.")
            sys.exit(1)

        # Handle world selection
        world_model = args.world
        if world_model is None:
            world_model = launcher.select_world(available_worlds)
        elif world_model not in available_worlds:
            print(f"Error: World model '{world_model}' not found.")
            sys.exit(1)

        print(f"\nLaunching simulation with:")
        print(f"- Vehicle model: {vehicle_model}")
        print(f"- World model: {world_model}")
        print(f"- Headless mode: {args.headless}")
        print(f"- Standalone mode: {args.standalone}")

        launch_command = launcher.generate_launch_command(
            vehicle_model, world_model, args.headless, args.standalone
        )

        subprocess.run(launch_command, check=True)

    except KeyboardInterrupt:
        print("\nSimulation launch cancelled by user.")
        sys.exit(0)
    except subprocess.CalledProcessError as e:
        print(f"\nError launching simulation: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()