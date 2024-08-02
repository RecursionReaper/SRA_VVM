from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Include the open_manipulator_upload.launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('open_manipulator_description'), '/launch/open_manipulator_upload.launch']
            ),
            launch_arguments={}.items()
        ),

        # Send joint values
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states']}]
        ),from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import IncludeLaunchDescription
from launch.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Include the open_manipulator_upload.launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('open_manipulator_description'), '/launch/open_manipulator_upload.launch']
            )
        ),

        # Send joint values
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_states']}]
        ),

        # Combine joint values to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),

        # Show in Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare('open_manipulator_description'), '/rviz/open_manipulator.rviz']]
        )
    ])


        # Combine joint values to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
        ),

        # Show in Rviz
        Node(
            package='rviz',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare('open_manipulator_description'), '/rviz/open_manipulator.rviz']],
        ),
    ])

