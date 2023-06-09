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
            name='color',
            default_value='purple',
            choices=['red', 'green', 'blue', 'purple'],
            description='Choices for the color of baselink, and the name of the'
                        'namespace; defaults to purple.'),
        DeclareLaunchArgument(
                name='model',
                default_value=str(
                    get_package_share_path('nuturtle_description')
                    / 'urdf/turtlebot3_burger.urdf.xacro'),
                description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=[str(get_package_share_path('nuturtle_description')),
                           '/config/basic_', LaunchConfiguration('color'), '.rviz'],
            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description='Choices for joint state publisher, defaults to true'),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Choices for whether to launch rviz, defaults to true'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=LaunchConfiguration('color'),
            condition=LaunchConfigurationEquals('use_jsp', 'true')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('color'),
            parameters=[
                {'robot_description': ParameterValue(Command(
                        ['xacro ', LaunchConfiguration('model'),
                            ' color:=', LaunchConfiguration('color')]),
                        value_type=str)},
                {'frame_prefix': [LaunchConfiguration('color'), '/']}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=LaunchConfiguration('color'),
            arguments=['-d', LaunchConfiguration('rvizconfig'),
                       '-f', [LaunchConfiguration('color'), '/base_footprint']],
            condition=LaunchConfigurationEquals('use_rviz', 'true'),
            on_exit=Shutdown()
        )
    ])
