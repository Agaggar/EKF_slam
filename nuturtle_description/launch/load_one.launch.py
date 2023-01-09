from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.events import Shutdown


def generate_launch_description():
    ddrive_pkg_path = get_package_share_path('nuturtle_description')
    robot_model_path = ddrive_pkg_path / 'turtlebot3_burger.urdf.xacro'
    default_rviz_config_path = ddrive_pkg_path / 'basic_purple.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(robot_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    use_jsp = DeclareLaunchArgument(name='use_jsp', default_value='true',
                                      choices=['true', 'false'],
                                      description='Choices for joint state publisher gui, defaults to true')
    use_rviz = DeclareLaunchArgument(name='use_rviz', default_value='true',
                                      choices=['true', 'false'],
                                      description='Choices for whether to launch rviz, defaults to true')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    # robot_configs = ddrive_pkg_path / 'ddrive.yaml'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=LaunchConfigurationEquals('use_jsp', 'true')
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
        on_exit=Shutdown()
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_jsp,
        use_rviz,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
