import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/nav2_bringup/launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': '/home/dyan/map.yaml',
            'params_file': os.path.join(os.getenv('HOME'), 'ros2_ws/src/turtle_ws/src/tf/params/nav2_params.yaml')
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/nav2_bringup/launch',
                'rviz_launch.py'
            )
        )
    )

    static_transform_broadcaster = ExecuteProcess(
        cmd=['ros2', 'run', 'tf', 'static_transform_tf2_broadcaster', '0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        cwd=os.path.join(os.getenv('HOME'), 'ros2_ws/src/turtle_ws/src/tf/src'),
        output='screen'
    )

    armor_detector_node = ExecuteProcess(
        cmd=['ros2', 'run', 'armor_detector', 'armor_detector_node'],
        cwd=os.path.join(os.getenv('HOME'), 'ros2_ws/src/turtle_ws/src/armor_detector/src'),
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        rviz_launch,
        static_transform_broadcaster,
        armor_detector_node
    ])