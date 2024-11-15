# drone_controller.py
import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
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
        

        # Drone state variables
        self.current_altitude = 0.0
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vertical_velocity = 0.0
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
        self.tolerance = 0.05
        self.landing_altitude = -0.1
        self.takeoff_altitude = -1.5
        self.landing_and_takeoff_sequence = False
        self.land_start_time = None
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

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

    # Functions as written
    def arm(self):
        print("Arm command sent")
        msg = VehicleCommand(param1=1.0, command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        self.publish_vehicle_command(msg)
        self.arming_state = True

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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode(position=False, velocity=True)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def hover(self, x, y, z):
        msg = TrajectorySetpoint(position=[x, y, z], yaw=self.current_yaw)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        # print(f"Hovering at {x:.2f}, {y:.2f}, {z:.2f}")

    def goto_setpoint(self, x, y, z):
        msg = TrajectorySetpoint(position=[x, y, z], yaw=self.current_yaw)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        if abs(self.current_x - x) < self.tolerance and abs(self.current_y - y) < self.tolerance and abs(self.current_altitude - z) < self.tolerance:
            print("Goto Setpoint reached.")
            self.goto = False
        # print(f"Going to {x, y, z}")

    def publish_takeoff_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()

        # Check if the drone is below the takeoff altitude and has not yet achieved success
        msg.position = [x, y, z]
        self.get_logger().info(f"Taking off: Current Altitude: {self.current_altitude:.2f}, Target Altitude: {z:.2f}")
        if z - self.tolerance < self.current_altitude < z + self.tolerance:

            self.takeoff_success = True
            self.get_logger().info("Takeoff altitude reached successfully.")
        else: 
            self.takeoff_success = False
        msg.yaw = self.current_yaw  # Keep the yaw fixed
        self.publish_trajectory_setpoint_publisher(msg)

    def publish_landing_setpoint(self, x, y):
        msg = TrajectorySetpoint()

        # Set a threshold to detect when the drone is close to the ground (e.g., 0.1 meters)
        landing_threshold = 0.3  # You can adjust this based on how close to the ground you want to detect

        new_altitude = self.current_altitude + landing_threshold  # Decrease altitude in small steps 
        msg.position = [x, y, new_altitude]  # Set new target position
        print(f"Landing... Current altitude: {self.current_altitude:.2f}, Target: {new_altitude:.2f}")

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
            else:
                self.detected_land = False

        msg.yaw = self.current_yaw  # Maintain current yaw
        self.publish_trajectory_setpoint_publisher(msg)

    def landing_and_takeoff(self, x, y, z):
        # Reset takeoff success flag at the start of each sequence
        self.takeoff_success = False
        # Landing phase
        if not self.detected_land:
            self.publish_landing_setpoint(x, y)  # Continue publishing landing setpoint
            self.landing_and_takeoff_sequence = False  # Reset start time if not in hover mode
            self.land_start_time = time.time()
        else:
            # Continue hovering for 3 seconds
            hold_for_takeoff = 5
            if time.time() - self.land_start_time < hold_for_takeoff:
                print(f"Landing detected! Preparing to takeoff in {(hold_for_takeoff - (time.time() - self.land_start_time)):.0f} seconds...")
                self.hover(self.current_x, self.current_y, self.current_altitude)  # Keep calling hover during the 3 seconds
            else:
                # Takeoff phase once landing is detected
                if not self.takeoff_success:
                    # Begin takeoff sequence after landing is detected
                    self.publish_takeoff_setpoint(x, y, z)  # Publish the takeoff setpoint
                    self.landing_and_takeoff_sequence = False

                    # If takeoff altitude has been reached, mark sequence as complete
                    if self.takeoff_success:
                        self.get_logger().info("Takeoff after landing complete, transitioning to next state.")
                        self.detected_land = False  # Reset for future landings
                        self.landing_and_takeoff_sequence = True

    def vehicle_local_position_callback(self, msg):
        self.current_altitude = msg.z
        self.current_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_vertical_velocity = msg.vz

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def failsafe_vehicle_startup_position(self, x, y, z):

        tolerance_xy = 0.2
        tolerance_z = self.tolerance
        
        if tolerance_xy > x > -tolerance_xy:
            if tolerance_xy > y > -tolerance_xy:
                if (tolerance_z + self.landing_altitude)  > z > (self.landing_altitude - tolerance_z):
                    return True;
        else:
            return False
        
    def publish_trajectory_setpoint_publisher(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg) 

    def battery_callback(self, msg):
        """Callback function to update the battery percentage."""
        if msg.remaining >= 0:  
            self.battery_percentage = msg.remaining * 100 
            self.get_logger().info(f"Battery level: {self.battery_percentage:.2f}%")
        else:
            self.get_logger().info("Battery level unknown")

    def check_battery(self):
        if self.battery_percentage <= 11:
            self.get_logger().info(f"Low battery: {self.battery_percentage:.2f}%")
            return True
        return False

### Silver Controll ###

    def move_forward(self, speed: float):
        """Move the drone forward at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [speed, 0.0, 0.0]
        msg.yaw = self.current_yaw  # Maintain current yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def move_backward(self, speed: float):
        """Move the drone backward at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [-speed, 0.0, 0.0] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def move_left(self, speed: float):
        """Move the drone left at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, -speed, 0.0] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def move_right(self, speed: float):
        """Move the drone right at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, speed, 0.0] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def move_up(self, speed: float):
        """Move the drone up at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, -speed] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def move_down(self, speed: float):
        """Move the drone down at a specified speed."""
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, speed] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def stop(self):
        """Stop the drone's movement by zeroing velocities."""
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, 0.0] 
        msg.yaw = self.current_yaw
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
