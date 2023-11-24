import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sbg_driver'),
        'config',
        'sbg_device_uart_default.yaml'
    )

    sbg_node = launch_ros.action.Node(
        package='sbg_driver',
        #	name='sbg_device_1',
        executable = 'sbg_device',
        output = 'screen',
        parameters = [config]
    )

    senix_node = launch_ros.action.Node(
        package='senix_driver',
        executable = 'senix_node',
        output = 'screen'
    )


    return launch.LaunchDescription([
        sbg_node,
        senix_node
    ])
