"""
Vehicle State Management for PX4 UAV

This module provides optimized state management for PX4 vehicle telemetry,
including Enum classes for PX4 states and a centralized VehicleState dataclass.
"""

from enum import IntEnum
from dataclasses import dataclass


class ArmingState(IntEnum):
    """PX4 Arming State Enum"""
    DISARMED = 1
    ARMED = 2
    
    def __str__(self):
        return {
            1: "Vehicle Disarmed",
            2: "Vehicle Armed"
        }.get(self.value, f"Unknown ({self.value})")


class NavState(IntEnum):
    """PX4 Navigation State Enum - Only modes used in vehicle_commander"""
    MANUAL = 0
    ALTCTL = 1          # Altitude Control
    POSCTL = 2         # Position Control  
    AUTO_MISSION = 3         # Auto Mission
    HOLD = 4         # Hold
    ACRO = 10            # Acro Mode
    OFFBOARD = 14        # Offboard
    STABILIZED = 15      # Stabilized
    TAKEOFF = 17
    LAND = 18
    
    def __str__(self):
        return {
            0: "Manual",
            1: "Altitude Control",
            2: "Position Control",
            3: "Mission",
            4: "Hold",
            10: "Acro",
            14: "Offboard",
            15: "Stabilized",
            17: "Takeoff",
            18: "Land"
        }.get(self.value, f"Unknown ({self.value})")
    

    
    @property
    def is_auto_mode(self) -> bool:
        """Check if current state is an autonomous mode"""
        return self == self.MISSION
    
    @property
    def is_manual_mode(self) -> bool:
        """Check if current state is a manual mode"""
        return self in [
            self.MANUAL, self.ALTCTL, self.POSCTL, self.ACRO, self.STABILIZED
        ]
    
    @property
    def is_offboard_mode(self) -> bool:
        """Check if current state is offboard mode"""
        return self == self.OFFBOARD
    
    @property
    def allows_offboard_control(self) -> bool:
        """Check if current state allows offboard control"""
        return self in [
            self.OFFBOARD, self.POSCTL, self.ALTCTL
        ]


class TakeoffState(IntEnum):
    """PX4 Takeoff State Enum"""
    EMPTY = 0
    READY = 1
    RAMP_UP = 2
    CLIMBING = 3
    FLIGHT = 4
    REJECTED = 5
    
    def __str__(self):
        return {
            0: "Empty",
            1: "Ready for takeoff",
            2: "During ramp-up",
            3: "Climbing",
            4: "Flight",
            5: "Rejected or failed"
        }.get(self.value, f"Unknown ({self.value})")


class BatteryWarning(IntEnum):
    """PX4 Battery Warning Enum"""
    NONE = 0
    LOW = 1
    CRITICAL = 2
    EMERGENCY = 3
    FAILED = 4
    UNHEALTHY = 6
    CHARGING = 7
    
    def __str__(self):
        return {
            0: "No Warning",
            1: "Low Voltage Warning",
            2: "Critical Voltage Warning",
            3: "Emergency Voltage Warning",
            4: "Battery Failed",
            6: "Battery Unhealthy",
            7: "Battery Charging"
        }.get(self.value, f"Unknown ({self.value})")


class CommandResult(IntEnum):
    """PX4 Command Result Enum"""
    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    DENIED = 2
    UNSUPPORTED = 3
    FAILED = 4
    IN_PROGRESS = 5
    CANCELLED = 6
    
    def __str__(self):
        return {
            0: "Command Accepted and Executed",
            1: "Command Temporarily Rejected",
            2: "Command Permanently Denied",
            3: "Command Unsupported",
            4: "Command Execution Failed",
            5: "Command In Progress",
            6: "Command Cancelled"
        }.get(self.value, f"Unknown ({self.value})")


class FailsafeFlag(IntEnum):
    """PX4 Failsafe Flags Enum"""
    OK = 0
    ACTIVE = 1
    
    @staticmethod
    def get_text(flag_name: str, value: bool) -> str:
        """Get human-readable text for failsafe flags"""
        flag_texts = {
            "offboard_control_signal_lost": {
                False: "Offboard Control Signal OK",
                True: "Offboard Control Signal Lost"
            },
            "battery_warning": {
                False: "Battery OK",
                True: "Battery Warning"
            },
            "local_position_invalid": {
                False: "Local Position OK",
                True: "Local Position Invalid"
            },
            "fd_critical_failure": {
                False: "No Critical Failure",
                True: "Critical Failure Detected"
            },
            "fd_motor_failure": {
                False: "Motor Status OK",
                True: "Motor Failure Detected"
            },
            "fd_esc_arming_failure": {
                False: "ESC Arming OK",
                True: "ESC Arming Failure"
            },
            "fd_imbalanced_prop": {
                False: "Propeller Balanced",
                True: "Imbalanced Propeller"
            },
            "gcs_connection_lost": {
                False: "GCS Connected",
                True: "GCS Connection Lost"
            },
            "manual_control_signal_lost": {
                False: "Manual Control Signal OK",
                True: "Manual Control Signal Lost"
            },
            "home_position_invalid": {
                False: "Home Position Set",
                True: "Home Position Invalid"
            },
            "landed": {
                False: "In Air",
                True: "Landed"
            },
            "ground_contact": {
                False: "No Ground Contact",
                True: "Ground Contact Detected"
            },
            "freefall": {
                False: "Stable",
                True: "Free-Fall Detected"
            }
        }
        
        return flag_texts.get(flag_name, {}).get(value, f"Unknown {flag_name}: {value}")


@dataclass
class VehicleState:
    """Centralized vehicle state data structure"""
    
    # Position (NED frame)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    
    # Velocity (NED frame)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    
    # Acceleration (NED frame)
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    
    # Vehicle states
    arming_state: ArmingState = ArmingState.DISARMED
    nav_state: NavState = NavState.MANUAL
    takeoff_state: TakeoffState = TakeoffState.EMPTY
    
    # Critical flags
    failsafe: bool = False
    landed: bool = False  # Initialize as False - will be updated from land detector
    armed: bool = False
    offboard_enabled: bool = False
    
    # Battery information
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: float = 0.0
    battery_warning: BatteryWarning = BatteryWarning.NONE
    
    # Home position
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 0.0
    home_yaw: float = 0.0
    
    # Timestamp
    timestamp: int = 0
    
    # Status message
    status_message: str = ""
    
    @property
    def position(self) -> tuple:
        """Current position as (x, y, z) tuple"""
        return (self.x, self.y, self.z)
    
    @property
    def velocity(self) -> tuple:
        """Current velocity as (vx, vy, vz) tuple"""
        return (self.vx, self.vy, self.vz)
    
    @property
    def is_ready_for_offboard(self) -> bool:
        """Check if vehicle is ready for offboard control"""
        return (self.arming_state == ArmingState.ARMED and
                not self.failsafe and
                self.nav_state == NavState.OFFBOARD)
    
    @property
    def is_flying(self) -> bool:
        """Check if vehicle is flying"""
        return (self.arming_state == ArmingState.ARMED and
                not self.landed and
                self.takeoff_state == TakeoffState.FLIGHT)
    
    @property
    def battery_level_percent(self) -> float:
        """Battery level as percentage (0-100)"""
        return self.battery_remaining * 100.0
    
    @property
    def status_summary(self) -> str:
        """Human-readable status summary"""
        return (f"Arming State: {self.arming_state}, "
                f"Mode: {self.nav_state}, "
                f"Takeoff: {self.takeoff_state}, "
                f"Battery: {self.battery_level_percent:.1f}%")
    
    def get_failsafe_text(self, flag_name: str, value: bool) -> str:
        """Get human-readable failsafe flag text"""
        return FailsafeFlag.get_text(flag_name, value) 