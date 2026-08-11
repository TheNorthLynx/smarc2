from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from smarc_msgs.msg import Topics as SmarcTopics
from evolo_msgs.msg import Topics as evoloTopics
from smarc_control_msgs.msg import Topics as ControlTopics


def generate_launch_description():

    robot_ns = LaunchConfiguration('robot_name')
    log_name = LaunchConfiguration('log_name')
    log_time = LaunchConfiguration('log_time')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )
    log_name_arg = DeclareLaunchArgument('log_name', default_value="evolo_log_2026_06_25_nx")
    log_time_arg = DeclareLaunchArgument('log_time', default_value="10.0") #[s]

    ghost = Node(
        package='evolo_logger',
        namespace=robot_ns,
        executable="logger",
        name='logger',
        parameters=[{"robot_name": robot_ns,
                     "log_name": log_name,
                     "log_time": log_time,
                     }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        log_name_arg,
        log_time_arg,
        ghost
    ])
