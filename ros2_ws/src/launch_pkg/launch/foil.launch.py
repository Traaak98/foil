import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbg_driver'),
        'config',
        'sbg_device_uart_default.yaml'
    )

    sbg_node = Node(
        package='sbg_driver',
        #	name='sbg_device_1',
        executable = 'sbg_device',
        output = 'screen',
        parameters = [config]
    )

    foil_state_node = Node(
        package='foil_state',
        executable = 'foil_state_node',
        output = 'screen'
    )

    foil_objective_node = Node(
        package='foil_objective',
        executable = 'foil_objective_node',
        output = 'screen'
    )

    foil_consigne_node = Node(
        package='foil_consigne',
        executable = 'foil_consigne_node',
        output = 'screen'
    )

    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a'],output='screen')


    return LaunchDescription([
        sbg_node,
        foil_state_node,
        foil_objective_node,
        foil_consigne_node,
        rosbag
    ])
