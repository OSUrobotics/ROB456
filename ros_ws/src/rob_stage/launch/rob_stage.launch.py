#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Directories where we find the world files
    stage_ros2_directory = get_package_share_directory('stage_ros2')
    rob_stage_directory = get_package_share_directory('rob_stage')

    # This is a stage parameter - use Twist or TwistStamped. We're going to use the latter
    use_stamped_velocity = LaunchConfiguration('use_stamped_velocity')
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='true',
        description='on true stage will accept TwistStamped command messages')
    
    # If using stage world only, this is where cave lives
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world')

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    
    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!')

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='on true all times are sim times')
    
    def stage_world_configuration(context):
        file = os.path.join(
            stage_ros2_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    def rob_stage_world_configuration(context):
        file = os.path.join(
            rob_stage_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    rob_stage_world_configuration_arg = OpaqueFunction(function=rob_stage_world_configuration)
    print(f"directories {rob_stage_directory, rob_stage_world_configuration}")

    return LaunchDescription([
        use_stamped_velocity_arg,
        stage_world_arg,
        one_tf_tree_arg, 
        use_sim_time_arg,
        enforce_prefixes_arg, 
        use_static_transformations_arg, 
        rob_stage_world_configuration_arg,   # Change this back to stage_world_configuration_arg for using stage_ros2 worlds
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{'one_tf_tree': one_tf_tree,
                        'enforce_prefixes': enforce_prefixes,
                        'use_stamped_velocity': use_stamped_velocity,
                        'use_static_transformations': use_static_transformations,
                        'use_sim_time': use_sim_time,
                "world_file": [LaunchConfiguration('world_file')]}],
        )
    ])
