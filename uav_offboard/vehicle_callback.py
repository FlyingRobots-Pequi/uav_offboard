"""
Optimized Vehicle Callback for PX4 UAV

This module provides an optimized callback system for PX4 vehicle telemetry
with thread-safe state management and efficient data processing.
"""

import threading
from typing import Callable, List, Optional
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleOdometry, VehicleStatus, FailsafeFlags,
    VehicleCommandAck, OffboardControlMode, HomePosition, TakeoffStatus,
    VehicleLandDetected, BatteryStatus
)
from rclpy.qos import qos_profile_sensor_data

from .vehicle_state import (
    VehicleState, ArmingState, NavState, TakeoffState, BatteryWarning,
    CommandResult, FailsafeFlag
)

class VehicleCallback:
    """
    Optimized callback system for PX4 vehicle telemetry.
    
    Features:
    - Thread-safe state management
    - Efficient data processing
    - Event-driven notifications
    - Minimal memory footprint
    """
    
    def __init__(self, node, config, uav_namespace=""):
        self.node = node
        self.config = config
        self.uav_namespace = uav_namespace
        self._lock = threading.RLock()
        self._state = VehicleState()
        
        # Event callbacks
        self._position_change_callbacks: List[Callable] = []
        self._state_change_callbacks: List[Callable] = []
        self._critical_alert_callbacks: List[Callable] = []
        
        # Thresholds for notifications from config
        self._position_threshold = config.position_threshold
        self._velocity_threshold = config.velocity_threshold
        
        # Previous values for change detection
        self._prev_position = (0.0, 0.0, 0.0)
        self._prev_arming_state = ArmingState.DISARMED
        self._prev_nav_state = NavState.MANUAL
        self._prev_takeoff_state = TakeoffState.EMPTY
        
        self._create_subscriptions()
        
        self.node.get_logger().info("VehicleCallback initialized with optimized callbacks")

    def _build_uav_topic(self, topic):
        """Build complete topic name with namespace prefix."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{topic}"
        return topic
    
    def _create_subscriptions(self):
        """Create all PX4 message subscriptions"""
        
        # Position and movement data
        self.sub_local_position = self.node.create_subscription(
            VehicleLocalPosition,
            self._build_uav_topic('/fmu/out/vehicle_local_position'),
            self.cb_local_position,
            qos_profile_sensor_data
        )
        
        self.sub_odometry = self.node.create_subscription(
            VehicleOdometry,
            self._build_uav_topic('/fmu/out/vehicle_odometry'),
            self.cb_odometry,
            qos_profile_sensor_data
        )
        
        # Vehicle status
        self.sub_vehicle_status = self.node.create_subscription(
            VehicleStatus,
            self._build_uav_topic('/fmu/out/vehicle_status'),
            self.cb_vehicle_status,
            qos_profile_sensor_data
        )
        
        # Safety and failsafe
        self.sub_failsafe_flags = self.node.create_subscription(
            FailsafeFlags,
            self._build_uav_topic('/fmu/out/failsafe_flags'),
            self.cb_failsafe_flags,
            qos_profile_sensor_data
        )
        
        # Command acknowledgments
        self.sub_command_ack = self.node.create_subscription(
            VehicleCommandAck,
            self._build_uav_topic('/fmu/out/vehicle_command_ack'),
            self.cb_command_ack,
            qos_profile_sensor_data
        )
        
        # Control modes
        self.sub_offboard_mode = self.node.create_subscription(
            OffboardControlMode,
            self._build_uav_topic('/fmu/out/offboard_control_mode'),
            self.cb_offboard_mode,
            qos_profile_sensor_data
        )
        
        # Home and takeoff
        self.sub_home_position = self.node.create_subscription(
            HomePosition,
            self._build_uav_topic('/fmu/out/home_position'),
            self.cb_home_position,
            qos_profile_sensor_data
        )
        
        self.sub_takeoff_status = self.node.create_subscription(
            TakeoffStatus,
            self._build_uav_topic('/fmu/out/takeoff_status'),
            self.cb_takeoff_status,
            qos_profile_sensor_data
        )
        
        # Landing detection
        self.sub_land_detected = self.node.create_subscription(
            VehicleLandDetected,
            self._build_uav_topic('/fmu/out/vehicle_land_detected'),
            self.cb_land_detected,
            qos_profile_sensor_data
        )
        
        # Battery status
        self.sub_battery_status = self.node.create_subscription(
            BatteryStatus,
            self._build_uav_topic('/fmu/out/battery_status'),
            self.cb_battery_status,
            qos_profile_sensor_data
        )
    
    # ============================================================================
    # PROPERTIES (Thread-safe access to state)
    # ============================================================================
    
    @property
    def current_position(self) -> tuple:
        """Current position as (x, y, z) tuple"""
        with self._lock:
            return self._state.position
    
    @property
    def current_velocity(self) -> tuple:
        """Current velocity as (vx, vy, vz) tuple"""
        with self._lock:
            return self._state.velocity
    
    @property
    def current_heading(self) -> float:
        """Current heading in radians"""
        with self._lock:
            return self._state.heading
    
    @property
    def arming_state(self) -> ArmingState:
        """Current arming state"""
        with self._lock:
            return self._state.arming_state
    
    @property
    def nav_state(self) -> NavState:
        """Current navigation state"""
        with self._lock:
            return self._state.nav_state
    
    @property
    def takeoff_state(self) -> TakeoffState:
        """Current takeoff state"""
        with self._lock:
            return self._state.takeoff_state
    
    @property
    def is_armed(self) -> bool:
        """Check if vehicle is armed"""
        with self._lock:
            return self._state.arming_state == ArmingState.ARMED
    
    @property
    def is_flying(self) -> bool:
        """Check if vehicle is flying"""
        with self._lock:
            return self._state.is_flying
    
    @property
    def is_ready_for_offboard(self) -> bool:
        """Check if vehicle is ready for offboard control"""
        with self._lock:
            return self._state.is_ready_for_offboard
    
    @property
    def is_landed(self) -> bool:
        """Check if vehicle is landed"""
        with self._lock:
            return self._state.landed
    
    @property
    def failsafe_active(self) -> bool:
        """Check if failsafe is active"""
        with self._lock:
            return self._state.failsafe
    
    @property
    def battery_level(self) -> float:
        """Battery level as percentage (0-100)"""
        with self._lock:
            return self._state.battery_level_percent
    
    @property
    def home_position(self) -> tuple:
        """Home position as (x, y, z) tuple"""
        with self._lock:
            return (self._state.home_x, self._state.home_y, self._state.home_z)
    
    @property
    def status_summary(self) -> str:
        """Human-readable status summary"""
        with self._lock:
            return self._state.status_summary
    
    # ============================================================================
    # CALLBACK REGISTRATION
    # ============================================================================
    
    def add_position_change_callback(self, callback: Callable):
        """Add callback for position changes"""
        self._position_change_callbacks.append(callback)
    
    def add_state_change_callback(self, callback: Callable):
        """Add callback for state changes"""
        self._state_change_callbacks.append(callback)
    
    def add_critical_alert_callback(self, callback: Callable):
        """Add callback for critical alerts"""
        self._critical_alert_callbacks.append(callback)
    
    def remove_position_change_callback(self, callback: Callable):
        """Remove position change callback"""
        if callback in self._position_change_callbacks:
            self._position_change_callbacks.remove(callback)
    
    def remove_state_change_callback(self, callback: Callable):
        """Remove state change callback"""
        if callback in self._state_change_callbacks:
            self._state_change_callbacks.remove(callback)
    
    def remove_critical_alert_callback(self, callback: Callable):
        """Remove critical alert callback"""
        if callback in self._critical_alert_callbacks:
            self._critical_alert_callbacks.remove(callback)
    
    # ============================================================================
    # PX4 MESSAGE CALLBACKS
    # ============================================================================
    
    def cb_local_position(self, msg):
        """Local position callback"""
        with self._lock:
            # Update position
            self._state.x = msg.x
            self._state.y = msg.y
            self._state.z = msg.z
            self._state.heading = msg.heading
            
            # Update velocity
            self._state.vx = msg.vx
            self._state.vy = msg.vy
            self._state.vz = msg.vz
            
            # Update acceleration
            self._state.ax = msg.ax
            self._state.ay = msg.ay
            self._state.az = msg.az
            
            self._state.timestamp = msg.timestamp
        
        # Check for significant position change
        current_pos = (msg.x, msg.y, msg.z)
        if self._position_changed(current_pos, self._prev_position):
            self._prev_position = current_pos
            self._notify_position_change(current_pos)
    
    def cb_odometry(self, msg):
        """Odometry callback - additional position and velocity data"""
        # Note: This provides additional data that may be more accurate
        # For now, we'll use local_position as primary source
        pass
    
    def cb_vehicle_status(self, msg):
        """Vehicle status callback"""
        state_changed = False
        
        with self._lock:
            # Check for state changes
            new_arming = ArmingState(msg.arming_state)
            new_nav = NavState(msg.nav_state)
            
            if new_arming != self._prev_arming_state:
                self._prev_arming_state = new_arming
                self._state.arming_state = new_arming
                self._state.armed = (new_arming == ArmingState.ARMED)
                state_changed = True
                self.node.get_logger().info(f"Arming state changed: {new_arming}")
            
            if new_nav != self._prev_nav_state:
                self._prev_nav_state = new_nav
                self._state.nav_state = new_nav
                self._state.offboard_enabled = (new_nav == NavState.OFFBOARD)
                state_changed = True
                self.node.get_logger().info(f"Navigation state changed: {new_nav}")
            
            # Update other status fields
            self._state.failsafe = msg.failsafe
            
            # Check for critical conditions
            if msg.failsafe and not self._state.failsafe:
                self._notify_critical_alert("Failsafe activated!")
        
        if state_changed:
            self._notify_state_change()
    
    def cb_failsafe_flags(self, msg):
        """Failsafe flags callback"""
        critical_flags = []
        
        # Check for critical failsafe conditions
        if msg.offboard_control_signal_lost:
            critical_flags.append(FailsafeFlag.get_text("offboard_control_signal_lost", True))
        if msg.battery_warning:
            critical_flags.append(FailsafeFlag.get_text("battery_warning", True))
        if msg.local_position_invalid:
            critical_flags.append(FailsafeFlag.get_text("local_position_invalid", True))
        if msg.fd_critical_failure:
            critical_flags.append(FailsafeFlag.get_text("fd_critical_failure", True))
        if msg.fd_motor_failure:
            critical_flags.append(FailsafeFlag.get_text("fd_motor_failure", True))
        if msg.fd_esc_arming_failure:
            critical_flags.append(FailsafeFlag.get_text("fd_esc_arming_failure", True))
        if msg.fd_imbalanced_prop:
            critical_flags.append(FailsafeFlag.get_text("fd_imbalanced_prop", True))
        if msg.gcs_connection_lost:
            critical_flags.append(FailsafeFlag.get_text("gcs_connection_lost", True))
        if msg.manual_control_signal_lost:
            critical_flags.append(FailsafeFlag.get_text("manual_control_signal_lost", True))
        if msg.home_position_invalid:
            critical_flags.append(FailsafeFlag.get_text("home_position_invalid", True))
        
        # Notify critical alerts with descriptive messages
        for flag in critical_flags:
            self._notify_critical_alert(flag)
    
    def cb_command_ack(self, msg):
        """Command acknowledgment callback"""
        result = CommandResult(msg.result)
        
        if result != CommandResult.ACCEPTED:
            self.node.get_logger().warn(f"Command {msg.command} result: {result}")
        else:
            self.node.get_logger().debug(f"Command {msg.command} accepted")
    
    def cb_offboard_mode(self, msg):
        """Offboard control mode callback"""
        with self._lock:
            self._state.offboard_enabled = (msg.position or msg.velocity or 
                                          msg.acceleration or msg.attitude)
    
    def cb_home_position(self, msg):
        """Home position callback"""
        with self._lock:
            self._state.home_x = msg.x
            self._state.home_y = msg.y
            self._state.home_z = msg.z
            self._state.home_yaw = msg.yaw
        
        self.node.get_logger().info(f"Home position set: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
    
    def cb_takeoff_status(self, msg):
        """Takeoff status callback"""
        with self._lock:
            new_takeoff_state = TakeoffState(msg.takeoff_state)
            
            if new_takeoff_state != self._prev_takeoff_state:
                self._prev_takeoff_state = new_takeoff_state
                self._state.takeoff_state = new_takeoff_state
                self.node.get_logger().info(f"Takeoff state changed: {new_takeoff_state}")
                self._notify_state_change()
    
    def cb_land_detected(self, msg):
        """Land detection callback"""
        prev_landed = None
        
        # Debug: sempre loga recebimento do tópico
        self.node.get_logger().debug(f"Land detected msg: landed={msg.landed}, ground_contact={msg.ground_contact}, freefall={msg.freefall}")
        
        with self._lock:
            prev_landed = self._state.landed
            self._state.landed = msg.landed
            
            if prev_landed != msg.landed:
                status_text = FailsafeFlag.get_text("landed", msg.landed)
                self.node.get_logger().info(f"LAND STATUS CHANGED: {status_text}")
                self._notify_state_change()
        
        # Check for other critical conditions (outside lock)
        if msg.freefall:
            self._notify_critical_alert(FailsafeFlag.get_text("freefall", True))
        if msg.ground_contact and not prev_landed:
            self.node.get_logger().info(FailsafeFlag.get_text("ground_contact", True))
    
    def cb_battery_status(self, msg):
        """Battery status callback"""
        with self._lock:
            self._state.battery_voltage = msg.voltage_v
            self._state.battery_current = msg.current_a
            self._state.battery_remaining = msg.remaining
            self._state.battery_warning = BatteryWarning(msg.warning)
        
        # Check for battery warnings
        if msg.warning > BatteryWarning.NONE:
            warning_text = BatteryWarning(msg.warning)
            self._notify_critical_alert(f"Battery warning: {warning_text}")
    
    # ============================================================================
    # HELPER METHODS
    # ============================================================================
    
    def _position_changed(self, current: tuple, previous: tuple) -> bool:
        """Check if position changed significantly"""
        return (abs(current[0] - previous[0]) > self._position_threshold or
                abs(current[1] - previous[1]) > self._position_threshold or
                abs(current[2] - previous[2]) > self._position_threshold)
    
    def _notify_position_change(self, position: tuple):
        """Notify position change callbacks"""
        for callback in self._position_change_callbacks:
            try:
                callback(position)
            except Exception as e:
                # self.node.get_logger().error(f"Error in position callback: {e}")
                pass
    
    def _notify_state_change(self):
        """Notify state change callbacks"""
        for callback in self._state_change_callbacks:
            try:
                callback(self._state)
            except Exception as e:
                # self.node.get_logger().error(f"Error in state callback: {e}")
                pass
    
    def _notify_critical_alert(self, message: str):
        """Notify critical alert callbacks"""
        # self.node.get_logger().error(f"CRITICAL ALERT: {message}")
        for callback in self._critical_alert_callbacks:
            try:
                callback(message)
            except Exception as e:
                # self.node.get_logger().error(f"Error in critical alert callback: {e}")
                pass
    
    def get_state_snapshot(self) -> VehicleState:
        """Get a copy of the current state (thread-safe)"""
        with self._lock:
            # Create a copy of the state
            import copy
            return copy.deepcopy(self._state)
    
    def is_position_reached(self, target_x: float, target_y: float, target_z: float, 
                           tolerance: float = 0.5) -> bool:
        """Check if target position is reached within tolerance"""
        with self._lock:
            return (abs(self._state.x - target_x) < tolerance and
                    abs(self._state.y - target_y) < tolerance and
                    abs(self._state.z - target_z) < tolerance)
    
    def get_distance_to_home(self) -> float:
        """Get distance to home position"""
        with self._lock:
            dx = self._state.x - self._state.home_x
            dy = self._state.y - self._state.home_y
            dz = self._state.z - self._state.home_z
            return (dx*dx + dy*dy + dz*dz) ** 0.5
