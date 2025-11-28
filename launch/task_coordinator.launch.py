"""
Launch file for Prometheus Task Coordinator
Starts both the task coordinator and web dashboard nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('prometheus_task_coordinator')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker address'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    mqtt_simulate_arg = DeclareLaunchArgument(
        'mqtt_simulate',
        default_value='true',
        description='Use MQTT simulation mode'
    )
    
    api_host_arg = DeclareLaunchArgument(
        'api_host',
        default_value='0.0.0.0',
        description='Web API host address'
    )
    
    api_port_arg = DeclareLaunchArgument(
        'api_port',
        default_value='5000',
        description='Web API port'
    )
    
    # Task Coordinator Node
    task_coordinator_node = Node(
        package='prometheus_task_coordinator',
        executable='task_coordinator',
        name='task_coordinator',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'mqtt_simulate': LaunchConfiguration('mqtt_simulate'),
            'nav_success_rate': 0.95,
            'nav_speed': 1.0,
            'check_interval': 1.0,
        }],
        emulate_tty=True,
    )
    
    # Web Dashboard Node
    web_dashboard_node = Node(
        package='prometheus_task_coordinator',
        executable='web_dashboard',
        name='web_dashboard',
        output='screen',
        parameters=[{
            'api_host': LaunchConfiguration('api_host'),
            'api_port': LaunchConfiguration('api_port'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        mqtt_simulate_arg,
        api_host_arg,
        api_port_arg,
        task_coordinator_node,
        web_dashboard_node,
    ])