import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbg_driver'),
        'config',
        'sbg_device_uart_default.yaml'
    )

    sbg_node = launch_ros.actions.Node(
        package='sbg_driver',
        #	name='sbg_device_1',
        executable = 'sbg_device',
        output = 'screen',
        parameters = [config]
    )

    senix_node = launch_ros.actions.Node(
        package='senix_driver',
        executable = 'senix_node',
        output = 'screen'
    )

    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a'],output='screen')


    return launch.LaunchDescription([
        sbg_node,
        senix_node,
        rosbag
    ])
