import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file demonstrating how to launch flight_manager_node with parameters.
    
    This shows how to:
    1. Load parameters from YAML file
    2. Override specific parameters
    3. Launch the flight manager with custom configuration
    """
    
    # Get package directory
    package_dir = get_package_share_directory('uav_offboard')
    
    # Path to the configuration file
    config_file = os.path.join(package_dir, 'config', 'flight_params.yaml')
    
    # Create the flight manager node with parameters
    flight_manager_node = Node(
        package='uav_offboard',
        executable='flight_manager_node',
        name='flight_manager_node',
        parameters=[
            config_file,
            {
                # Override specific parameters if needed
                # 'navigation.position_tolerance': 0.15,
                # 'navigation.takeoff_climb_rate': 1.0,
                # 'control.setpoint_mode': 'position',
                # 'logging.debug_navigation': True
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        flight_manager_node
    ]) 