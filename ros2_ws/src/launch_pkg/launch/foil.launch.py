import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import yaml
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

    driver_share_dir = get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_share_dir = get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    laserscan_share_dir = get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])

    esp_nuc_node = launch_ros.actions.Node(
        package='esp_nuc',
        executable = 'node_esp_nuc',
        output = 'screen'
    )

    foil_consigne_node = launch_ros.actions.Node(
        package='foil_consigne',
        executable = 'foil_consigne_node',
        output = 'screen'
    )

    foil_objective_node = launch_ros.actions.Node(
        package='foil_objective',
        executable = 'foil_objective_node',
        output = 'screen'
    )

    foil_state_node = launch_ros.actions.Node(
        package='foil_state',
        executable = 'foil_state_node',
        output = 'screen'
    )

    uart_py_node = launch_ros.actions.Node(
        package='uart_py',
        executable = 'node_uart_py',
        output = 'screen'
    )

    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a'],output='screen')


    return launch.LaunchDescription([
        sbg_node,
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        #esp_nuc_node,
        foil_consigne_node,
        foil_objective_node,
        foil_state_node,
        uart_py_node,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
        rosbag
    ])
