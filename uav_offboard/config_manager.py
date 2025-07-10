"""
Configuration Manager for UAV Offboard Package

This module provides centralized configuration management using ROS2 parameters.
All configurable parameters are loaded from YAML files or set via ROS2 parameters.
"""

from dataclasses import dataclass
from rclpy.node import Node


@dataclass
class NavigationConfig:
    """Navigation configuration parameters"""
    position_tolerance: float = 0.2
    takeoff_climb_rate: float = 0.5
    hold_duration: float = 2.0
    default_takeoff_altitude: float = 3.0
    altitude_threshold: float = 0.75  # Limiar para navegação gradual de altitude


@dataclass
class ControlConfig:
    """Control system configuration parameters"""
    setpoint_mode: str = "velocity"
    heartbeat_frequency: float = 10.0
    navigation_frequency: float = 10.0
    status_frequency: float = 5.0
    realtime_status: bool = True


@dataclass
class CallbackConfig:
    """Callback system configuration parameters"""
    position_threshold: float = 0.1
    velocity_threshold: float = 0.1


@dataclass
class MissionConfig:
    """Mission system configuration parameters"""
    command_delay: float = 0.05
    command_timeout: float = 30.0


@dataclass
class SafetyConfig:
    """Safety system configuration parameters"""
    max_altitude: float = 10.0
    max_velocity: float = 5.0
    emergency_land_battery: float = 0.15


@dataclass
class LoggingConfig:
    """Logging configuration parameters"""
    debug_navigation: bool = False
    debug_callbacks: bool = False
    debug_mission: bool = True


@dataclass
class FlightConfig:
    """Complete flight configuration"""
    navigation: NavigationConfig
    control: ControlConfig
    callbacks: CallbackConfig
    mission: MissionConfig
    safety: SafetyConfig
    logging: LoggingConfig


class ConfigManager:
    """
    Configuration manager for UAV Offboard package.
    
    Loads configuration from ROS2 parameters and provides
    centralized access to all configurable parameters.
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.config = self._load_config()
        
        # Log configuration loaded
        self.node.get_logger().info("ConfigManager: Configuration loaded successfully")
        if self.config.logging.debug_navigation:
            self.node.get_logger().info(f"Navigation config: {self.config.navigation}")
        if self.config.logging.debug_callbacks:
            self.node.get_logger().info(f"Callback config: {self.config.callbacks}")
    
    def _load_config(self) -> FlightConfig:
        """Load configuration from ROS2 parameters"""
        
        # Declare all parameters with defaults
        self._declare_parameters()
        
        # Load navigation parameters
        navigation = NavigationConfig(
            position_tolerance=self.node.get_parameter('navigation.position_tolerance').value,
            takeoff_climb_rate=self.node.get_parameter('navigation.takeoff_climb_rate').value,
            hold_duration=self.node.get_parameter('navigation.hold_duration').value,
            default_takeoff_altitude=self.node.get_parameter('navigation.default_takeoff_altitude').value,
            altitude_threshold=self.node.get_parameter('navigation.altitude_threshold').value
        )
        
        # Load control parameters
        control = ControlConfig(
            setpoint_mode=self.node.get_parameter('control.setpoint_mode').value,
            heartbeat_frequency=self.node.get_parameter('control.heartbeat_frequency').value,
            navigation_frequency=self.node.get_parameter('control.navigation_frequency').value,
            status_frequency=self.node.get_parameter('control.status_frequency').value,
            realtime_status=self.node.get_parameter('control.realtime_status').value
        )
        
        # Load callback parameters
        callbacks = CallbackConfig(
            position_threshold=self.node.get_parameter('callbacks.position_threshold').value,
            velocity_threshold=self.node.get_parameter('callbacks.velocity_threshold').value
        )
        
        # Load mission parameters
        mission = MissionConfig(
            command_delay=self.node.get_parameter('mission.command_delay').value,
            command_timeout=self.node.get_parameter('mission.command_timeout').value
        )
        
        # Load safety parameters
        safety = SafetyConfig(
            max_altitude=self.node.get_parameter('safety.max_altitude').value,
            max_velocity=self.node.get_parameter('safety.max_velocity').value,
            emergency_land_battery=self.node.get_parameter('safety.emergency_land_battery').value
        )
        
        # Load logging parameters
        logging = LoggingConfig(
            debug_navigation=self.node.get_parameter('logging.debug_navigation').value,
            debug_callbacks=self.node.get_parameter('logging.debug_callbacks').value,
            debug_mission=self.node.get_parameter('logging.debug_mission').value
        )
        
        return FlightConfig(
            navigation=navigation,
            control=control,
            callbacks=callbacks,
            mission=mission,
            safety=safety,
            logging=logging
        )
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values"""
        
        # Navigation parameters
        self.node.declare_parameter('navigation.position_tolerance', 0.2)
        self.node.declare_parameter('navigation.takeoff_climb_rate', 0.5)
        self.node.declare_parameter('navigation.hold_duration', 2.0)
        self.node.declare_parameter('navigation.default_takeoff_altitude', 3.0)
        self.node.declare_parameter('navigation.altitude_threshold', 0.75)
        
        # Control parameters
        self.node.declare_parameter('control.setpoint_mode', "velocity")
        self.node.declare_parameter('control.heartbeat_frequency', 10.0)
        self.node.declare_parameter('control.navigation_frequency', 10.0)
        self.node.declare_parameter('control.status_frequency', 5.0)
        self.node.declare_parameter('control.realtime_status', True)
        
        # Callback parameters
        self.node.declare_parameter('callbacks.position_threshold', 0.1)
        self.node.declare_parameter('callbacks.velocity_threshold', 0.1)
        
        # Mission parameters
        self.node.declare_parameter('mission.command_delay', 0.05)
        self.node.declare_parameter('mission.command_timeout', 30.0)
        
        # Safety parameters
        self.node.declare_parameter('safety.max_altitude', 10.0)
        self.node.declare_parameter('safety.max_velocity', 5.0)
        self.node.declare_parameter('safety.emergency_land_battery', 0.15)
        
        # Logging parameters
        self.node.declare_parameter('logging.debug_navigation', False)
        self.node.declare_parameter('logging.debug_callbacks', False)
        self.node.declare_parameter('logging.debug_mission', True)
    
    def get_config(self) -> FlightConfig:
        """Get the current configuration"""
        return self.config
    
    def reload_config(self):
        """Reload configuration from parameters"""
        self.config = self._load_config()
        self.node.get_logger().info("ConfigManager: Configuration reloaded")
    
    def validate_config(self) -> bool:
        """Validate configuration parameters"""
        
        # Check navigation parameters
        if self.config.navigation.position_tolerance <= 0:
            self.node.get_logger().error("Invalid position_tolerance: must be > 0")
            return False
        
        if self.config.navigation.takeoff_climb_rate <= 0:
            self.node.get_logger().error("Invalid takeoff_climb_rate: must be > 0")
            return False
        
        if self.config.navigation.hold_duration < 0:
            self.node.get_logger().error("Invalid hold_duration: must be >= 0")
            return False
        
        # Check control parameters
        if self.config.control.setpoint_mode not in ["position", "velocity"]:
            self.node.get_logger().error("Invalid setpoint_mode: must be 'position' or 'velocity'")
            return False
        
        if self.config.control.heartbeat_frequency <= 0:
            self.node.get_logger().error("Invalid heartbeat_frequency: must be > 0")
            return False
        
        # Check safety parameters
        if self.config.safety.max_altitude <= 0:
            self.node.get_logger().error("Invalid max_altitude: must be > 0")
            return False
        
        if self.config.safety.emergency_land_battery < 0 or self.config.safety.emergency_land_battery > 1:
            self.node.get_logger().error("Invalid emergency_land_battery: must be between 0 and 1")
            return False
        
        return True 