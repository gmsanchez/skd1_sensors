import os
import launch
import launch_ros.actions
import launch.substitutions
import yaml

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ARGUMENTS = [
         DeclareLaunchArgument('use_mag', default_value='False',
            description='Do not use magnetometer for orientation estimation by default. Set to True for outdoor use'),
    ]

    spatial_param_config = os.path.join(get_package_share_directory('phidgets_spatial_bringup'), 'config', 'phidgets_spatial.yaml')
    imu_filter_param_config = os.path.join(get_package_share_directory('phidgets_spatial_bringup'), 'config', 'imu_filter.yaml')
    use_mag = LaunchConfiguration('use_mag', default='True')

    # https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf
    # https://github.com/ros2/rclcpp/issues/715#issuecomment-490425249
    # Composable Nodes use different yaml parsing than a standalone node.
    # This code will load the parameters from the yaml (removing the namespace/nodename/ros__parameters heading) so
    # that the parameters are parsed and named properly for the composable node.
    with open(spatial_param_config, 'r') as f:
        spatial_params = yaml.safe_load(f)['phidgets_spatial']['ros__parameters']

    with open(imu_filter_param_config, 'r') as f:
        imu_filter_params = yaml.safe_load(f)['imu_filter']['ros__parameters']

    container = ComposableNodeContainer(
        name='spatial_imu_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                namespace='phidgets_373223',
                name='phidgets_spatial',
                parameters=[spatial_params],
            ),
            ComposableNode(
                package='imu_filter_madgwick',
                plugin='ImuFilterMadgwickRos',
                namespace='phidgets_373223',
                name='imu_filter',
                parameters=[imu_filter_params,
                            {'use_mag': use_mag}],
            )
        ],
        output='screen',
    )

    ld = launch.LaunchDescription(ARGUMENTS)
    ld.add_action(container)

    return ld
