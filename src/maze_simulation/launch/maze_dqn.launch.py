#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    stage_num = 1        # default stage
    max_training_episodes = 1000  # default episodes

    return LaunchDescription([
        # DQN Gazebo Environment Node
        Node(
            package='turtlebot3_dqn',
            executable='dqn_gazebo',
            name='dqn_gazebo',
            output='screen',
            arguments=[str(stage_num)]
        ),
        
        # DQN Environment Node
        Node(
            package='turtlebot3_dqn',
            executable='dqn_environment',
            name='dqn_environment',
            output='screen'
        ),
        
        # DQN Agent Node
        Node(
            package='turtlebot3_dqn',
            executable='dqn_agent',
            name='dqn_agent',
            output='screen',
            arguments=[str(stage_num), str(max_training_episodes)]
        ),
    ])
