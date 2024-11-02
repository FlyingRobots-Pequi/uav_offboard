import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Timer to manage control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        
        self.offboard_setpoint_counter_ = 0
        
        self.current_altitude = 0.0  # Track the current altitude
        
        self.max_altitude = -1.0  # Target altitude for takeoff (-1m in NED)
        
        self.current_x = 0.0
        self.current_x = 0.0
        
        self.landing_altitude = -0.1  # Stop landing when altitude is near 0 (ground level)
        
        self.hover_altitude = -1.0  # Hover altitude after takeoff
 
    def vehicle_local_position_callback(self, msg):
        # Get the current altitude from the VehicleLocalPosition message
        self.current_altitude = msg.z  # Altitude is in the z field (in NED, z is negative upward)
        self.current_yaw = msg.heading  # Get the current yaw angle
        self.current_x = msg.x
        self.current_y = msg.y
        

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        if self.offboard_setpoint_counter_ == 50:
            # Engage offboard mode and arm the drone after 1 second (50 setpoints)
            self.engage_offBoard_mode()
            self.arm()
        
        # Control takeoff
        if self.offboard_setpoint_counter_ < 550:
            self.publish_offboard_control_mode()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_takeoff_setpoint()
        
        # Hover at the target altitude after takeoff
        if self.offboard_setpoint_counter_ < 850:
            self.publish_offboard_control_mode()                
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
               self.hover()

        # After reaching 550 setpoints, enter the landing phase
        if self.offboard_setpoint_counter_ >= 850:
            self.publish_offboard_control_mode()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.land()

        self.offboard_setpoint_counter_ += 1


    def arm(self):
        print("Arm command sent")
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True  # We are controlling position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def engage_offBoard_mode(self):
        print('Offboard mode command sent')
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publish_vehicle_command(msg)

    def publish_takeoff_setpoint(self):
        msg = TrajectorySetpoint()
        # Ascend to the target altitude (e.g., -1.0 meters in NED frame)
        if self.current_altitude > self.max_altitude:
            msg.position = [0.0, 0.0, self.max_altitude]  # Hold at 1 meter above ground
            print(f"Ascending to {self.max_altitude} meters...")
        else:
            msg.position = [0.0, 0.0, self.current_altitude]  # Hold position
            print("Reached target altitude.")

        msg.yaw = self.current_yaw  # Keep the yaw fixed

        self.publish_vehicle_command(msg)


    def hover(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, self.hover_altitude]
        msg.yaw = self.current_yaw
        print("Hovering... at altitude: ", self.hover_altitude)

        self.publish_vehicle_command(msg)

    def land(self):
        msg = TrajectorySetpoint()
        msg.position = [self.current_x, self.current_y, self.landing_altitude]
        msg.yaw = self.current_yaw
        
        if self.current_altitude < self.landing_altitude:
            print("Landed. Disarming...")
            self.disarm()
            self.timer.cancel()

        self.publish_vehicle_command(msg)

    def disarm(self):
        print('Disarm command sent')
        msg = VehicleCommand()
        msg.param1 = 0.0  # Disarm the drone
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()