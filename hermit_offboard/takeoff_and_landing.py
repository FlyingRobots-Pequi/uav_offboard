import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
import time
class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Create subscribers
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
        
        # Create publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile)
        
        # Timer to manage control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.offboard_setpoint_counter_ = 0
        self.current_altitude = 0.0  # Track the current altitude
        self.takeoff_altitude = -1.5  # Target altitude for takeoff (-1m in NED)
        self.landing_altitude = -0.1  # Stop landing when altitude is near 0 (ground level)
        self.takeoff_success = False
        self.arming_state = False
        self.hover_timing = 5
        self.hover_counter = 0
        self.hold_position = True
        self.landing = False
        self.takeoff_again = False
        self.takeland = False
        self.detected_land = False
        self.immediate_takeoff = False
        self.goto = True
        self.current_yaw = 0.0
        self.tolerance = 0.05
        
        
        # self.current_x = 0.0
        # self.current_y = 0.0
        # self.current_vertical_velocity = 0.0       
        
        # Define points to go to
        self.gotopoints = [
            [1.0, 1.0, -2.0],  # Example waypoints
            [2.0, 2.0, -2.0],
            [3.0, 1.0, -2.0],
            [1.0, -1.0, -2.0],
            [0.0, 0.0, -2.0]
        ]
        self.current_point_index = 0  # Start at the first waypoint
        self.state = "ARM"  # Initial state
        self.hold_start_time = None  # Timer for hold states
            

    def cmdloop_callback(self):
        # Check arming state and initiate offboard mode only once
        if not self.arming_state:
            self.engage_offBoard_mode()
            self.arm()

        # Proceed with takeoff phase
        if not self.takeoff_success and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_offboard_control_mode()
            self.publish_takeoff_setpoint(0.0, 0.0, self.takeoff_altitude)

        # Hover phase
        elif self.takeoff_success and not self.immediate_takeoff:
            if self.hold_position:
            # Initialize hover start time only once
                if not hasattr(self, 'hover_start_time'):
                    self.hover_start_time = time.time()
                    self.get_logger().info("Starting hover phase.")

                self.publish_offboard_control_mode()
                self.hover(self.current_x, self.current_y, self.takeoff_altitude)
                
                # Check if hold time is over
                if time.time() - self.hover_start_time >= self.hover_timing:
                    self.hold_position = False  # Move to the landing phase
                    self.landing = True
                    # self.goto = True
                    # self.takeland = True
                    self.get_logger().info("Hover time complete, proceeding to landing.")
                    del self.hover_start_time  # Clean up attribute to avoid reuse

            if self.landing:
                self.publish_offboard_control_mode()
                self.publish_landing_setpoint()
            
            if self.takeland:
                self.publish_offboard_control_mode()
                self.landing_and_takeoff()
                
        elif self.immediate_takeoff:
            self.publish_offboard_control_mode()
            self.publish_takeoff_setpoint(self.current_x, self.current_y, self.takeoff_altitude)
            
        # elif not self.hold_position and self.goto:
        #     self.publish_offboard_control_mode()
        #     self.goto_setpoint(3.0, -1.0, -2.0)


        # Landing phase
        # elif not self.hold_position and not self.goto and self.landing:
        # elif self.landing:
        #     self.publish_offboard_control_mode()
        #     if self.current_altitude < self.landing_altitude:
        #         self.publish_landing_setpoint()
        if self.detected_land:
            self.disarm()
            rclpy.shutdown()  # Shut down after landing

    def vehicle_local_position_callback(self, msg):
        # Get the current altitude from the VehicleLocalPosition message
        self.current_altitude = msg.z  # Altitude is in the z field (in NED, z is negative upward)
        self.current_yaw = msg.heading  # Get the current yaw angle
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_vertical_velocity = msg.vz
        
    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def arm(self):
        print("Arm command sent")
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)
        self.arming_state = True
    
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

    def publish_trajectory_setpoint_publisher(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_takeoff_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()

        # Check if the drone is below the takeoff altitude and has not yet achieved success
        msg.position = [x, y, z]
        self.get_logger().info(f"Taking off: Current Altitude: {self.current_altitude:.2f}, Target Altitude: {z}")
        if z - self.tolerance < self.current_altitude < z + self.tolerance:
            if self.immediate_takeoff:
                self.get_logger().info("Reached immediate takeoff altitude after landing.")
                self.immediate_takeoff = False  # Reset flag after reaching target altitude
            else:
            # Mark takeoff as successful when the target altitude is reached
                self.takeoff_success = True
                self.get_logger().info("Takeoff altitude reached successfully.")

        msg.yaw = self.current_yaw  # Keep the yaw fixed
        self.publish_trajectory_setpoint_publisher(msg)
        # msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.trajectory_setpoint_publisher_.publish(msg)

    def publish_landing_setpoint(self, x, y):
        msg = TrajectorySetpoint()

        # Set a threshold to detect when the drone is close to the ground (e.g., 0.1 meters)
        landing_threshold = 0.3  # You can adjust this based on how close to the ground you want to detect

        new_altitude = self.current_altitude + landing_threshold  # Decrease altitude in small steps 
        msg.position = [x, y, new_altitude]  # Set new target position
        print(f"Landing... Current altitude: {self.current_altitude}, Target: {new_altitude}")

        # Parameters for landing detection
        landing_velocity_threshold = 0.05  # Threshold for detecting near-zero vertical velocity
        detection_delay = 2.0  # Delay in seconds before enabling landing detection

        # Initialize the detection delay timer if not already set
        if not hasattr(self, 'landing_detection_start_time'):
            self.landing_detection_start_time = time.time()
            self.get_logger().info("Landing detection delay timer started.")

        # Check if the detection delay has passed
        if time.time() - self.landing_detection_start_time >= detection_delay:
            # Begin landing detection based on vertical velocity after the delay
            if abs(self.current_vertical_velocity) < landing_velocity_threshold:
                print("Landing detected based on vertical velocity.")
                self.detected_land = True  # Flag indicating landing detected
                del self.landing_detection_start_time  # Reset timer for future use        

        msg.yaw = self.current_yaw  # Maintain current yaw
        self.publish_trajectory_setpoint_publisher(msg)
        # msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.trajectory_setpoint_publisher_.publish(msg)
                
    def disarm(self):
        print('Disarm command sent')
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
        msg.target_system = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publish_vehicle_command(msg)

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def failsafe_vehicle_startup_position(self, x, y, z):

        tolerance_xy = 0.2
        tolerance_z = self.tolerance
        
        if tolerance_xy > x > -tolerance_xy:
            if tolerance_xy > y > -tolerance_xy:
                if (tolerance_z + self.landing_altitude)  > z > (self.landing_altitude - tolerance_z):
                    return True;
        else:
            return False

    def hover(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]  # Hold position
        msg.yaw = self.current_yaw  # Maintain current yaw
        print(f"Hovering at {x, y, z}")
        self.publish_trajectory_setpoint_publisher(msg)
        # msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.trajectory_setpoint_publisher_.publish(msg)   
        
    def goto_setpoint(self, x,y,z):
        msg = TrajectorySetpoint()
        msg.position = [x,y,z]
        msg.yaw = self.current_yaw  # Maintain current yaw
        
        if (abs(self.current_x - x) < self.tolerance and
            abs(self.current_y - y) < self.tolerance and
            abs(self.current_altitude - z) < self.tolerance):
            print("Goto Setpoint reached.")
            self.goto = False
        
        print(f"Going to {x,y,z}")
        self.publish_trajectory_setpoint_publisher(msg)
        # msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.trajectory_setpoint_publisher_.publish(msg) 

    def landing_and_takeoff(self, x, y, z):
        # Create TrajectorySetpoint message
        if not self.detected_land:
            self.publish_landing_setpoint(x,y)
        else:
            # Set flag to indicate immediate takeoff after landing
            self.immediate_takeoff = True
            self.publish_takeoff_setpoint(x, y, z)
            self.hold_position = True  # Transition back to hover mode once at altitude
            self.detected_land = False  # Reset for next use    
    
def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()