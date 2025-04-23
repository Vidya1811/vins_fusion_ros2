from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        os.getenv('HOME'),
        'vins_fusion_ws',
        'src',
        'VINS-Fusion-ROS2',
        'config',
        'euroc',
        'euroc_mono_imu_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='vins',
            executable='vins_node',
            name='vins_node',
            output='screen',
            emulate_tty=True,
            arguments=[config_file],
            parameters=[{'use_sim_time': True}],
        )
    ])

