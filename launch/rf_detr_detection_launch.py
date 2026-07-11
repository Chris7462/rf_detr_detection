from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # or 'true' if appropriate
        description='Use simulation time'
    )

    params = join(
        get_package_share_directory('rf_detr_detection'), 'param',
        'rf_detr_detection.yaml'
    )

    rf_detr_detection_node = Node(
        package='rf_detr_detection',
        executable='rf_detr_detection_node',
        name='rf_detr_detection_node',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rf_detr_detection_node
    ])
