import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, SetEnvironmentVariable, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory    

def validate_paths(paths_dict):
    for name, path in paths_dict.items():
        if not os.path.exists(path):
            raise FileNotFoundError(f"{name} not found: {path}")

def launch_setup(context, *args, **kwargs):
    # Get arguments
    vehicle = LaunchConfiguration('vehicle').perform(context)
    standalone = LaunchConfiguration('standalone').perform(context)
    
    # Set up paths
    HOME = '/home/retro'
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    pkg_px4 = os.path.abspath('/home/retro/Projects/PX4-Autopilot')
    pkg_px4_sim = get_package_share_directory('px4_drone_sim')

    os.makedirs(PX4_RUN_DIR)

    paths = {
        'Vehicle model': os.path.join(pkg_px4_sim, 'models', vehicle, 'model.sdf'),
        'PX4_BINARY': os.path.join(pkg_px4, 'build', 'px4_sitl_default', 'bin', 'px4')
    }
    validate_paths(paths)

    env_vars = {
        'PX4_SIM_MODEL': vehicle,
        'PX4_HOME_LAT': '47.397742',
        'PX4_HOME_LON': '8.545594',
        'PX4_HOME_ALT': '488.0',
        'LD_LIBRARY_PATH': os.path.join(pkg_px4, 'build/px4_sitl_default/build_gazebo')
    }
    
    # Get existing environment paths
    current_path = os.environ.get('PATH', '')
    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    
    # Define Gazebo model and plugin paths
    gz_model_path = os.path.join(pkg_px4, 'Tools', 'simulation', 'gz', 'models')
    gz_plugin_path = os.path.join(pkg_px4, 'build', 'px4_sitl_default', 'build_gz')
    gz_world_path = os.path.join(pkg_px4, 'Tools', 'simulation', 'gz', 'worlds')

    # Environment variables for PX4 SITL and Gazebo
    env_vars = {
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_SIMULATOR': 'gz',
        'PX4_GZ_MODEL': vehicle,
        'PX4_GZ_MODEL_POSE': '0,0,0.2,0,0,0',
        'PX4_GZ_WORLD': world,
        'GZ_SIM_RESOURCE_PATH': f"{gz_model_path}:{gz_world_path}",
        'IGN_GAZEBO_RESOURCE_PATH': f"{gz_model_path}:{gz_world_path}",
        'GZ_SIM_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': gz_plugin_path,
        'GAZEBO_PLUGIN_PATH': gz_plugin_path,
        'PATH': current_path,
        'LD_LIBRARY_PATH': current_ld_path,
    }

    if headless == 'True':
        env_vars['HEADLESS'] = '1'

    # Source setup script and run PX4
    px4_cmd = [
        f'{pkg_px4}/build/px4_sitl_default/bin/px4'
    ]

    # Create the PX4 SITL process
    sitl_process = ExecuteProcess(
        cmd=px4_cmd,
        env=env_vars,
        output='screen',
        shell=True
    )

    # # Create the MicroXRCE-DDS Agent process with a delay
    # microxrcedds_process = ExecuteProcess(
    #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    #     output='screen'
    # )

    # # Add a delayed start for the MicroXRCE-DDS Agent
    # delayed_microxrcedds = TimerAction(
    #     period=5.0,  # 5 second delay
    #     actions=[microxrcedds_process]
    # )

    # Create log messages for debugging
    start_msg = LogInfo(msg='Starting PX4 SITL simulation...')
    agent_msg = LogInfo(msg='Starting MicroXRCE-DDS Agent...')

    return [
        SetEnvironmentVariable(name=key, value=value)
        for key, value in env_vars.items()
    ] + [
        start_msg,
        sitl_process,
        agent_msg,
        # delayed_microxrcedds
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle',
            default_value='x500',
            description='Vehicle model to spawn (x500, x500_depth, x500_vision, standard_vtol, rc_cessna)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='default',
            description='World name to load'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run simulation in headless mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])