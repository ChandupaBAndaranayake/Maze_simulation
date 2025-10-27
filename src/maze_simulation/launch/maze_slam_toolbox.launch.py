import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Paths ---
    pkg_maze = get_package_share_directory('maze_simulation')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # --- Config ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.25')
    y_pose = LaunchConfiguration('y_pose', default='0.25')
    world_file = os.path.join(pkg_maze, 'worlds', 'maze_world.sdf')


    # --- Environment setup for Gazebo resources ---
    set_env_tb3_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_tb3_gazebo, 'models'))
    set_env_maze_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_maze, 'models'))

    # --- Gazebo (server + client) ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s -v4 {world_file}'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )

    # --- Spawn robot into world ---
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # --- Publish TFs (state publisher only) ---
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_bringup, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': ''
        }.items()
    )

    # --- SLAM Toolbox (online async) ---
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_maze, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # --- RViz2 (delayed start) ---
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_maze, 'rviz', 'slam_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Delay RViz launch by 8 seconds
    delayed_rviz = TimerAction(period=15.0, actions=[rviz_launch])
    delayed_slam = TimerAction(period=10.0, actions=[slam_toolbox])

    # --- LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(set_env_tb3_models)
    ld.add_action(set_env_maze_models)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_turtlebot)
    ld.add_action(state_publisher)
    ld.add_action(delayed_slam)
    ld.add_action(delayed_rviz)

    return ld
