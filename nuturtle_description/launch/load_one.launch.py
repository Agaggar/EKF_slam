from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
                name='model',
                default_value=str(
                    get_package_share_path('nuturtle_description') \
                        / 'urdf/turtlebot3_burger.urdf.xacro'),
                description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=str(
                get_package_share_path('nuturtle_description') \
                    / 'config/basic_purple.rviz'),
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description='Choices for joint state publisher gui, defaults to true'),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Choices for whether to launch rviz, defaults to true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=LaunchConfigurationEquals('use_jsp', 'true')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(
                    ['xacro ', LaunchConfiguration('model')]),
                    value_type=str)}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        )
    ])
