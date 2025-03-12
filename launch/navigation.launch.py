import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import rclpy

def generate_launch_description():
    logger = rclpy.logging.get_logger('navigation_launch')

    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')
    navigation_bringup_path = get_package_share_directory('nav2_bringup')
    stretch_navigation_extension_path = get_package_share_directory('stretch_nav2_extension')
    print(stretch_navigation_extension_path)
    
    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")
    
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Whether to autostart lifecycle nodes on launch')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stretch_navigation_path,
                                   'map', 'home2.yaml'),
        description='Full path to the map.yaml file to use for navigation')

    # Disable Error out if the map file does not exist
    #def map_file_check(context: LaunchContext):
    #    map_path = LaunchConfiguration('map').perform(context)
    #    if not os.path.exists(map_path):
    #        msg='Map file not found in given path: {}'.format(map_path)
    #        logger.error(msg)
    #        raise FileNotFoundError(msg)
    #    if not map_path.endswith('.yaml'):
    #        msg = 'Map file is not a yaml file: {}'.format(map_path)
    #        logger.error(msg)
    #        raise FileNotFoundError(msg)
        
    #map_path_check_action = OpaqueFunction(function=map_file_check)

    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stretch_navigation_extension_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_extension_path, '/launch/bringup_launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 
                          'autostart': LaunchConfiguration('autostart'),
                          'map': LaunchConfiguration('map'),
                          'params_file': LaunchConfiguration('params_file'),
                          'use_rviz': LaunchConfiguration('use_rviz')}.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_bringup_path, '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        teleop_type_param,
        use_sim_time_param,
        autostart_param,
        map_path_param,
        params_file_param,
        rviz_param,
        stretch_driver_launch,
        rplidar_launch,
        base_teleop_launch,
        navigation_bringup_launch,
        rviz_launch,
        #map_path_check_action,
    ])
