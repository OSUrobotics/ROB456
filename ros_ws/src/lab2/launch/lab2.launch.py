#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration


def generate_launch_description():

    # Fill this in as we go
    ld = LaunchDescription()

    rob_stage_directory = get_package_share_directory('rob_stage')

    # This is a stage parameter - use Twist or TwistStamped. We're going to use the latter
    use_stamped_velocity = LaunchConfiguration('use_stamped_velocity')
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='true',
        description='on true stage will accept TwistStamped command messages')
    ld.add_action(use_stamped_velocity_arg)

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    ld.add_action(enforce_prefixes_arg)
    
    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!')
    ld.add_action(use_static_transformations_arg)

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_arg = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
    ld.add_action(one_tf_tree_arg)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='on true all times are sim times')
    ld.add_action(use_sim_time_arg)

     # If using stage world only, this is where cave lives
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='empty'),
        description='World file relative to the project world file, without .world')
    ld.add_action(stage_world_arg)
   
    def rob_stage_world_configuration(context):
        file = os.path.join(
            rob_stage_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    rob_stage_world_configuration_arg = OpaqueFunction(function=rob_stage_world_configuration)
    ld.add_action(rob_stage_world_configuration_arg)

    stage_node = Node(
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
    ld.add_action(stage_node)    

    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='if true launch rviz')
    ld.add_action(launch_rviz_arg)

    #rviz_config_file = PathJoinSubstitution(
    #    ["$(find lab2)", "config", "driver2.rviz"]
    #)
    
    rviz_config_file = os.path.join(get_package_share_directory("lab2"), 'config', 'driver.rviz')

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        executable="rviz2",
        name="rviz2_driver",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    ld.add_action(rviz_node)

    send_points_node = Node(
        package="lab2",
        executable="send_points"
    )
    ld.add_action(send_points_node)

    driver_node = Node(
        package="lab2",
        executable="driver"
    )

    ld.add_action(driver_node)

    return ld
