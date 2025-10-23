import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Package directories ---
    pkg_maze = get_package_share_directory('maze_simulation')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # --- Launch configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.25')
    y_pose = LaunchConfiguration('y_pose', default='0.25')

    world_file = os.path.join(pkg_maze, 'worlds', 'maze_world.sdf')
    map_file = os.path.join(pkg_maze, 'map', 'my_map_save.yaml')
    nav2_params = os.path.join(pkg_maze, 'config', 'nav2_params.yaml')
    twist_mux_yaml = os.path.join(pkg_maze, 'config', 'twist_mux.yaml')
    rviz_config = os.path.join(pkg_maze, 'rviz', 'tb3_navigation2.rviz')

    # --- Environment setup for Gazebo ---
    set_env_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_tb3_gazebo, 'models'))
    set_env_maze_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_maze, 'models'))

    # --- Gazebo simulation (headless + GUI) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v4 {world_file}'}.items()
    )

    # --- Spawn TurtleBot3 model ---
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # --- Robot state publisher ---
    state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_bringup, 'launch', 'turtlebot3_state_publisher.launch.py')),
        launch_arguments={''
            'use_sim_time': use_sim_time,
            'namespace': ''
        }.items()
    )

    # --- Twist mux (velocity control arbitration) ---
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_yaml],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        output='screen'
    )

    # --- Nav2 Localization (AMCL + Map Server) ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )

    # --- Nav2 Navigation (Planner, Controller, BT Navigator) ---
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params
        }.items()
    )

    # --- RViz for visualization ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --- Delayed actions (wait for Gazebo to load fully) ---
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_tb3])
    delayed_localization = TimerAction(period=10.0, actions=[localization])
    delayed_navigation = TimerAction(period=15.0, actions=[navigation])
    delayed_rviz = TimerAction(period=20.0, actions=[rviz])
    delayed_twist_mux = TimerAction(period=25.0, actions=[twist_mux])

    # --- Launch description ---
    ld = LaunchDescription()
    for action in [
        set_env_tb3_models, set_env_maze_models, gz_sim, state_pub,
        delayed_spawn, delayed_localization, delayed_navigation,
        delayed_rviz, delayed_twist_mux
    ]:
        ld.add_action(action)

    return ld
