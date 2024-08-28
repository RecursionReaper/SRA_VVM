#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch_ros.descriptions import ParameterValue
import random

# this is the function launch  system will look for
def generate_launch_description():
    ####### DATA INPUT ##########
    # urdf_file = 'MAIN_vvm.urdf'
    # xacro_file = "open_manipulator_robot.urdf.xacro"
    sdf_file = "MAIN_vvm.urdf"
    package_description = "simulation_vvm"
    ####### DATA INPUT END ##########
    config = os.path.join( get_package_share_directory(package_description),
    'config',
    'vvm.yaml'
    )
    
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", sdf_file)
    print("Fetching URDF ==>")
    robot_description_content = Command(['xacro ',robot_desc_path])
    
    # robot_description_config = xacro.process_file(xacro_file)
    # robot_description = robot_description_config.toxml()

#########################################################3

    # robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", sdf_file)
    # # Directly read the SDF content
    # with open(robot_desc_path, 'r') as sdf_file:
    #     robot_description_content = sdf_file.read()
    robot_description = {"robot_description": robot_description_content}
    

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(Command(['xacro ',robot_desc_path]), value_type=str)}],
        output="screen"
    )


    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.0]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "VoiceVidManipulator"
    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_base_name,
                    '-x', str(position[0]), '-y', str(position[1]
                                                        ), '-z', str(position[2]),
                    '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                    '-topic', '/robot_description'
                    ]
    )
    

    delay_joint_state_broadcaster_spawner_after_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    return LaunchDescription([  
        # load_joint_position_controller,
        control_node,
        robot_state_publisher_node,
        spawn_robot,
        delay_joint_state_broadcaster_spawner_after_spawn_robot,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        
    ])