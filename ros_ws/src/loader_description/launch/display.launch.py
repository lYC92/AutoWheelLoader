from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('loader_description')
    xacro_file = os.path.join(package_share, 'urdf', 'loader.urdf.xacro')
    model_fidelity = LaunchConfiguration('model_fidelity')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' model_fidelity:=', model_fidelity]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_fidelity',
            default_value='nominal',
            description='Model pedigree label; use validated only after the documented checks pass.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
