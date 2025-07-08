# offboard_controller.py

import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class OffboardController():
    def __init__(self, node):
        self.node = node
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # Create publishers
        self.offboard_control_mode_publisher_ = self.node.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
        
        self.trajectory_setpoint_publisher_ = self.node.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)
        

    def publish_offboard_control_heartbeat_signal(self, position_control, velocity_control):
        """
            Publish the offboard control mode based on the control type.
            
            Args:
                position_control (bool): Set to True to enable position setpoints.
                velocity_control (bool): Set to True to enable velocity setpoints.
        """
        # Initialize the message with the selected control modes
        msg = OffboardControlMode()
        msg.position = position_control
        msg.velocity = velocity_control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Publish the control mode message
        self.offboard_control_mode_publisher_.publish(msg)
        
        # Get log information about the control mode
        # control_type = "position" if position_control else "velocity" if velocity_control else "none"
        # self.node.get_logger().info(f"[PUBLISH_OFFBOARD_CONTROL_HEARTBEAT_SIGNAL] Offboard control mode heartbeat signal set to {control_type}")

    def publish_position_control_setpoint(self, target_x, target_y, target_z, yaw=0.0):
        
        """
            Publishes a position setpoint to move the drone to a specific (x, y, z) coordinate.
            
            Args:
                target_x (float): Target x-coordinate in meters.
                target_y (float): Target y-coordinate in meters.
                target_z (float): Target z-coordinate in meters.
        """
        
        msg = TrajectorySetpoint()
        msg.position = [target_x, target_y, target_z]  # x, y, and z positions
        msg.yaw = yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        # self.node.get_logger().info(f"[PUB_POSITION_CONTROL_SETPOINT]Position control: Target position: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
        
    def publish_velocity_control_setpoint(self, target_vx, target_vy, target_vz, yaw=0.0):

        """
            Publishes a velocity setpoint to move the drone to a specific (x, y, z) coordinate.
            
            Args:
                target_vx (float): Target vx-velocity in meters/second.
                target_vy (float): Target vy-velocity in meters/second.
                target_vz (float): Target vz-velocity in meters/second.
        """

        # Construct and publish the velocity command
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [target_vx, target_vy, target_vz]  # Set x, y, and z velocities
        msg.yaw = yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

        # self.node.get_logger().info(f"[PUB_VELOCITY_CONTROL_SETPOINT]Velocity control: Target velocity: ({target_vx:.2f}, {target_vy:.2f}, {target_vz:.2f})")