#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction,DeclareLaunchArgument,  ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch_ros.descriptions import ParameterValue
import xacro
 

# this is the function launch  system will look for
def generate_launch_description():
    ####### DATA INPUT ##########
    xacro_file = 'open_manipulator_robot.urdf.xacro'
    
    package_description = 'simulation_vvm'
    ####### DATA INPUT END ##########
    config = os.path.join(get_package_share_directory(package_description),'config','vvm.yaml')
 

    # Earlier approach:
    # assert os.path.exists(xacro_path), f"The file {xacro_file} does not exist in {os.path.dirname(xacro_path)}"  
    # urdf_content = xacro.process_file(xacro_path)
    # robot_description_content = urdf_content.toxml()

    # MOdified:
    # xacro_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc, mappings={'config_path': config})
    # robot_description_content = doc.toxml()
    # robot_description = {"robot_description": robot_description_content}
    
    #Latest
    xacro_path = os.path.join(get_package_share_directory(package_description), 'urdf/', xacro_file)    
    assert os.path.exists(xacro_path), "The box_bot.xacro doesnt exist in "+str(xacro_path)

    robot_description_config = xacro.process_file(xacro_path)
    robot_description_content = robot_description_config.toxml()

    print(robot_description_content)
    spawn_node = Node(
        package='simulation_vvm', 
        executable='spawn_node.py', 
        arguments=[robot_description_content], 
        output='screen',
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_content}, 
                    config],
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
        parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(Command(['xacro ',xacro_path]), value_type=None)}],
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
                    '-x', str(position[0]), '-y', str(position[1] ), '-z', str(position[2]),
                    '-R', str(orientation[0]), '-P', str(orientation[1] ), '-Y', str(orientation[2]),
                    '-topic', '/robot_description'
                    ],
        parameters=[{'qos_overrides./robot_description.subscription.durability': 'transient_local'}]
    )
    
    delayed_spawn = TimerAction(
    period=2.0,
    actions=[spawn_robot]
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

    control_node_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[control_node],
        )
    )


    return LaunchDescription([  
        spawn_robot,
        # spawn_node,
        robot_state_publisher_node,
        # control_node_after_gazebo,
        # delay_joint_state_broadcaster_spawner_after_spawn_robot,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ])


   