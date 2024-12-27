import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def validate_paths(paths_dict):
    for name, path in paths_dict.items():
        if not os.path.exists(path):
            raise FileNotFoundError(f"{name} not found: {path}")

def get_env_variables(pkg_px4, model_path, vehicle, world, standalone):
    """Generate environement variables for PX4 simulation"""
    env_vars = {
        'PX4_SIM_MODEL': vehicle,
        'PX4_HOME_LAT': '47.397742',
        'PX4_HOME_LON': '8.545594',
        'PX4_HOME_ALT': '488.0',
        'GAZEBO_MODEL_PATH': f"{model_path}:$GAZEBO_MODEL_PATH",
        'GZ_SIM_RESOURCE_PATH': f"{model_path}:$GZ_SIM_RESOURCE_PATH",
        'IGN_GAZEBO_RESOURCE_PATH': f"{model_path}:$IGN_GAZEBO_RESOURCE_PATH",
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': f"{pkg_px4}/build/px4_sitl_default/build_gazebo",
        'GAZEBO_PLUGIN_PATH': os.path.join(pkg_px4, 'build/px4_sitl_default/build_gazebo'),
        'LD_LIBRARY_PATH': os.path.join(pkg_px4, 'build/px4_sitl_default/build_gazebo'),
        'PX4_RCSHELL_DIRECTORY': f"{pkg_px4}/ROMFS/px4fmu_common/init.d-posix"
    }

    if standalone == 'True':
        env_vars['PX4_GZ_STANDALONE'] = '1'
    else:
        env_vars['PX4_GX_WORLD'] = world

    return env_vars


def launch_setup(context, *args, **kwargs):
    # Get launch configurations
    vehicle = LaunchConfiguration('vehicle').perform(context)
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    standalone = LaunchConfiguration('standalone').perform(context)

    # set up package paths
    HOME = '/home/retro'
    PX4_RUN_DIR = os.path.join(HOME, 'tmp/px4_run_dir')
    pkg_px4 = os.path.abspath('/home/retro/Projects/PX4-Autopilot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_px4_sim = get_package_share_directory('px4_drone_sim')

    # create the run directory
    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    paths = {
        'World file': os.path.join(pkg_px4_sim, 'worlds', world, f'{world}.world'),
        'Vehicle model': os.path.join(pkg_px4_sim, 'models', vehicle, 'model.sdf'),
        'PX4 binary': os.path.join(pkg_px4, 'build', 'px4_sitl_default', 'bin', 'px4')
    }
    validate_paths(paths)

    model_path = PathJoinSubstitution([pkg_px4_sim])
    env_vars = get_env_variables(pkg_px4, model_path, vehicle, world, standalone)

    # launch the gazebo world
    gz_launch_path = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': paths['World file'],
        }.items(),
    )
    # gazebo_cmd = ExecuteProcess(
    #     cmd=['gz', 'sim', '-v', '4', '-r', paths['World file']],
    #     output='screen',
    #     env=env_vars
    # )

    # # spawn drone model
    # spawn_drone = ExecuteProcess(
    #     cmd=[
    #         'gz', 'sim',
    #         '-s', '--render-engine', 'ogre',
    #         '-g', 'libgz-sim-factory-system.so',
    #         '-i', paths['Vehicle model'],
    #         '--model', 'drone',
    #         '--x', '0', '--y', '0', '--z', '0',
    #         '--roll', '0', '--pitch', '0', '--yaw', '0'
    #     ],
    #     prefix="bash -c 'sleep 5s; $0 $@'",
    #     output='screen',
    # )
    spawn_cmd = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'sdf_filename: "{pkg_px4_sim}/models/drones/{vehicle}/model.sdf", name: "drone", pose: {{position: {{x: 0, y: 0, z: 0.5}}}}'
        ],
        output='screen'
    )


    # launch px4 sitl
    px4_process = ExecuteProcess(
        cmd=[
            paths['PX4 binary'],
            "-i", "0",
            "-d",
            os.path.join(pkg_px4, 'ROMFS/px4fmu_common/init.d-posix/rcS')
        ],
        env=env_vars,
        cwd=f"{pkg_px4}/build/px4_sitl_default/rootfs",
        output='screen'
    )

    # microrcrtps_agent_process = ExecuteProcess(
    #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    #     output='screen'
    # )

    return [
        SetEnvironmentVariable(name=key, value=value) for key, value in env_vars.items()
    ] + [
        SetLaunchConfiguration(
            name='world_file',
            value=[LaunchConfiguration('world'), TextSubstitution(text='.world')]
        ),
        gazebo_launch,
        px4_process,
        spawn_cmd,
        # microrcrtps_agent_process
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        DeclareLaunchArgument(
            'vehicle',
            default_value='x500',
            description='Vehicle name to simulate'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='mars',
            description='World name to simulate'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run Gazebo in headless mode.'
        ),
        DeclareLaunchArgument(
            'standalone',
            default_value='False',
            description='Run PX4 and Gazebo in standalone mode.'
        ),
        OpaqueFunction(function=launch_setup)
    ])