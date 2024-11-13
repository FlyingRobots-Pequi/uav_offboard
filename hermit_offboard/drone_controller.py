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

        # Initialize callback variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        # PID controller variables
        self.k_p = 0.1
        self.k_i = 0.01
        self.k_d = 0.1
        
        self.integral_x = 0.0
        self.integral_y = 0.0
        
        
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0

        self.dt = 0.1

        self.max_vertical_velocity_z = 0.5
        self.max_vertical_velocity_xy = 0.5
        
        
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
    def get_current_position(self):
        return self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z
    
    def get_current_velocity(self):
        return self.vehicle_local_position.vx, self.vehicle_local_position.vy, self.vehicle_local_position.vz
    
    def get_current_yaw(self):
        return self.vehicle_local_position.heading
    
    def get_current_acceleration(self):
        return self.vehicle_local_position.ax, self.vehicle_local_position.ay, self.vehicle_local_position.az
    
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def set_mode(self, mode: str):
        """
            Set the vehicle mode based on a string input.
            
            Args:
                mode (str): The desired mode as a string. Options include:
                            - "MANUAL"
                            - "ALTCTL" - Altitude control
                            - "POSCTL" - Position control
                            - "MISSION"
                            - "ACRO"
                            - "OFFBOARD"
                            - "STABILIZED"
        """
        # Define a dictionary to map mode strings to param2 values
        mode_mapping = {
            "MANUAL": 1.0,
            "ALTCTL": 2.0,
            "POSCTL": 3.0,
            "MISSION": 4.0,
            "ACRO": 5.0,
            "OFFBOARD": 6.0,
            "STABILIZED": 7.0
        }
        
        # Look up the mode code in the dictionary
        if mode.upper() not in mode_mapping:
            print(f"Error: Mode '{mode}' is not recognized.")
            return
        
        param2_value = mode_mapping[mode.upper()]
        
        # Set up the message with the determined mode
        msg = VehicleCommand()
        msg.param1 = 1.0  # Base mode, often set to 1.0 for enabled command
        msg.param2 = param2_value  # Custom mode from the mapping
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info(f"Mode '{mode}' command sent with param2={param2_value}")

    # Functions as written
    def arm(self):
        msg = VehicleCommand(param1=1.0, command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info("Arm command sent")
        
    ## TODO: Implement the disarm function. Logic is simular to the arm function but it is still not working.
    # def disarm(self):
    #     print("Arm command sent")
    #     msg = VehicleCommand(param1=1.0, command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
    #     self.publish_vehicle_command(msg)
    #     self.arming_state = False

    def flight_termination(self):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
        msg.target_system = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        
        self.get_logger().info("Flight termination command sent")

    def publish_offboard_control_heartbeat_signal(self, position_control: bool = True, velocity_control: bool = False):
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
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # Publish the control mode message
        self.offboard_control_mode_publisher_.publish(msg)
        
        # Get log information about the control mode
        # control_type = "position" if position_control else "velocity" if velocity_control else "none"
        # self.get_logger().info(f"Offboard control mode heartbeat signal set to {control_type}")

    def publish_z_velocity_control_setpoint(self, current_height, target_height):
        """
            Controls the drone's height using velocity setpoints to reach the target height.
            
            Args:
                current_height (float): The current height of the drone in meters.
        """
        
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)
        
        height_error =  - target_height + current_height
        
        velocity_vertical_setpoint = self.k_p * height_error
        
        vertical_velocity_setpoint = max(-self.max_vertical_velocity_z, min(velocity_vertical_setpoint, self.max_vertical_velocity_z))
        
        # Publish the velocity setpoint
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, vertical_velocity_setpoint]  # x, y, and z velocities
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        self.get_logger().info(f"Height control: Current height: {current_height:.2f} m, Target height: {target_height:.2f} m")
        
    def publish_xy_velocity_control_setpoint(self, current_x, current_y, target_x, target_y):
        """
            Control the x and y position of the drone precisely using a PID controller.
            
            Args:
                current_x (float): The current x-coordinate of the drone.
                current_y (float): The current y-coordinate of the drone.
        """        

        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)
        
        error_x = target_x - current_x
        error_y = target_y - current_y
        
        p_term_x = self.k_p * error_x
        p_term_y = self.k_p * error_y
        
        integral_x  += error_x * self.dt
        integral_y  += error_y * self.dt
        
        i_term_x = self.k_i * integral_x
        i_term_y = self.k_i * integral_y
        
        d_term_x = self.k_d * (error_x - self.previous_error_x) / self.dt
        d_term_y = self.k_d * (error_y - self.previous_error_y) / self.dt
        
        velocity_x = p_term_x + i_term_x + d_term_x
        velocity_y = p_term_y + i_term_y + d_term_y
        
        velocity_x = max(-self.max_vertical_velocity_xy, min(velocity_x, self.max_vertical_velocity_xy))
        velocity_y = max(-self.max_vertical_velocity_xy, min(velocity_y, self.max_vertical_velocity_xy))
        
        msg = TrajectorySetpoint()
        msg.velocity = [velocity_x, velocity_y, 0.0]  # x, y, and z velocities (z = 0 for no vertical movement)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

        self.previous_error_x = error_x
        self.previous_error_y = error_y
        
        # self.get_logger().info(f"Position control: Current position: ({current_x:.2f}, {current_y:.2f}), Target position: ({target_x:.2f}, {target_y:.2f})")
        # self.get_logger().info(f"Velocity setpoint: ({velocity_x:.2f}, {velocity_y:.2f})")
        
    def publish_position_control_setpoint(self, target_x, target_y, target_z):
        
        self.publish_offboard_control_heartbeat_signal(position_control=True, velocity_control=False)
        
        """
            Publishes a position setpoint to move the drone to a specific (x, y, z) coordinate.
            
            Args:
                target_x (float): Target x-coordinate in meters.
                target_y (float): Target y-coordinate in meters.
                target_z (float): Target z-coordinate in meters.
        """
        
        msg = TrajectorySetpoint()
        msg.position = [target_x, target_y, target_z]  # x, y, and z positions
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        self.get_logger().info(f"Position control: Target position: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
        
        
    def takeoff(self, target_altitude=2.0, ascent_velocity=0.3, altitude_threshold=0.1, velocity_threshold=0.1, acceleration_threshold=0.1, stable_time=2.0):
        """
        Command the drone to take off to a target altitude without specifying exact altitude control, by ascending until stability is detected.
        
        Args:
            target_altitude (float): Approximate target altitude for the takeoff (in meters).
            ascent_velocity (float): Velocity (in m/s) at which the drone ascends.
            altitude_threshold (float): Altitude range within which takeoff is considered successful.
            velocity_threshold (float): Threshold for vertical velocity indicating stopped ascent.
            acceleration_threshold (float): Threshold for vertical acceleration indicating stability.
            stable_time (float): Duration for which conditions must be met to confirm takeoff completion.
        """
        # Start ascending by setting offboard mode for velocity control
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)

        # Initialize timer for stable conditions
        takeoff_stable_start_time = None

        # Control loop for adaptive takeoff
        while rclpy.ok():
            # Publish a small upward velocity setpoint
            self.publish_z_velocity_control_setpoint(self.get_current_position()[2], ascent_velocity)

            # Get current altitude, vertical velocity, and acceleration
            current_altitude = self.get_current_position()[2]  # Assuming z is at index 2
            vertical_velocity = self.get_current_velocity()[2]  # Assuming z is at index 2
            vertical_acceleration = self.get_current_acceleration()[2]  # Assuming z is at index 2

            # Check if altitude exceeds target threshold
            if current_altitude >= target_altitude - altitude_threshold:
                # Check if conditions indicate stability at target altitude
                if abs(vertical_velocity) < velocity_threshold and abs(vertical_acceleration) < acceleration_threshold:
                    # Start or check the stable time counter
                    if takeoff_stable_start_time is None:
                        takeoff_stable_start_time = time.time()
                    elif time.time() - takeoff_stable_start_time >= stable_time:
                        self.get_logger().info("Takeoff completed and stable at target altitude.")
                        return True
            else:
                # Reset stable time counter if conditions are not met
                takeoff_stable_start_time = None

            # Small delay to keep the loop from running too fast
            rclpy.spin_once(self, timeout_sec=0.1)

        
        
    def hold_position(self, target_hold_x, target_hold_y, target_hold_z):
        """
        Command the drone to hold its current position indefinitely until another function is activated.
        
        Args:
            target_hold_x (float): The x-coordinate to hold.
            target_hold_y (float): The y-coordinate to hold.
            target_hold_z (float): The z-coordinate to hold.
        """
        # Set offboard control mode for position control
        self.publish_offboard_control_heartbeat_signal(position_control=True, velocity_control=False)

        # Set the hold mode flag to True, which we will monitor in the loop
        self.is_holding_position = True
        
        # Start an infinite hold loop
        while rclpy.ok() and self.is_holding_position:
            # Publish the position hold setpoint
            self.publish_position_control_setpoint(target_hold_x, target_hold_y, target_hold_z)

            # Log the holding position message periodically
            self.get_logger().info(f"Holding position at ({target_hold_x:.2f}, {target_hold_y:.2f}, {target_hold_z:.2f})")
            
            # Small delay to prevent overloading the control loop
            rclpy.spin_once(self, timeout_sec=0.1)

        # Log exit from hold mode
        self.get_logger().info("Exiting hold position mode.")

    def goto_setpoint(self, target_x, target_y, target_z, position_threshold=0.1, velocity_threshold=0.1, acceleration_threshold=0.1, stable_time=2.0):
        """
        Command the drone to move to a specific (x, y, z) coordinate and confirm when it arrives.
        
        Args:
            target_x (float): Target x-coordinate in meters.
            target_y (float): Target y-coordinate in meters.
            target_z (float): Target z-coordinate in meters.
            position_threshold (float): Distance threshold to consider the drone at the target.
            velocity_threshold (float): Velocity threshold for stability at the target.
            acceleration_threshold (float): Acceleration threshold for stability at the target.
            stable_time (float): Duration the drone must remain within thresholds to confirm arrival.
        """
        # Set offboard control mode for position control
        self.publish_offboard_control_heartbeat_signal(position_control=True, velocity_control=False)

        # Initialize timer for stability
        arrival_stable_start_time = None

        # Control loop for adaptive movement to setpoint
        while rclpy.ok():
            # Publish the target position setpoint
            self.publish_position_control_setpoint(target_x, target_y, target_z)

            # Get current position, velocity, and acceleration
            current_x, current_y, current_z = self.get_current_position()
            velocity_x, velocity_y, velocity_z = self.get_current_velocity()
            accel_x, accel_y, accel_z = self.get_current_acceleration()

            # Calculate position error
            position_error = ((target_x - current_x) ** 2 + (target_y - current_y) ** 2 + (target_z - current_z) ** 2) ** 0.5

            # Check if the position is within the target threshold
            if position_error < position_threshold:
                # Check if the drone is stable within velocity and acceleration thresholds
                if (abs(velocity_x) < velocity_threshold and abs(velocity_y) < velocity_threshold and abs(velocity_z) < velocity_threshold and
                    abs(accel_x) < acceleration_threshold and abs(accel_y) < acceleration_threshold and abs(accel_z) < acceleration_threshold):
                    
                    # Start or check the stable time counter
                    if arrival_stable_start_time is None:
                        arrival_stable_start_time = time.time()
                    elif time.time() - arrival_stable_start_time >= stable_time:
                        self.get_logger().info(f"Arrived at setpoint: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                        return True
            else:
                # Reset stable time counter if conditions are not met
                arrival_stable_start_time = None

            # Small delay to keep the loop from running too fast
            rclpy.spin_once(self, timeout_sec=0.1)        

    def land(self, descent_velocity=0.3, velocity_threshold=0.1, acceleration_threshold=0.1, stable_time=2.0):
        """
        Command the drone to land without specifying a target altitude by gradually descending and monitoring landing conditions.
        
        Args:
            descent_velocity (float): The velocity (in m/s) at which the drone descends.
            velocity_threshold (float): Threshold for vertical velocity to indicate stopped descent.
            acceleration_threshold (float): Threshold for vertical acceleration indicating stability.
            stable_time (float): Duration for which the conditions must be met to confirm landing.
        """
        # Start descent by setting offboard mode for velocity control
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)

        # Initialize timer for stable conditions
        landing_stable_start_time = None

        # Control loop for adaptive landing
        while rclpy.ok():
            # Publish a small downward velocity setpoint
            self.publish_z_velocity_control_setpoint(self.get_current_position()[2], -descent_velocity)

            # Get current vertical velocity and acceleration
            vertical_velocity = self.get_current_velocity()[2]  # Assuming z is at index 2
            vertical_acceleration = self.get_current_acceleration()[2]  # Assuming z is at index 2

            # Check if conditions indicate that landing is complete
            if abs(vertical_velocity) < velocity_threshold and abs(vertical_acceleration) < acceleration_threshold:
                # Start or check the stable time counter
                if landing_stable_start_time is None:
                    landing_stable_start_time = time.time()
                elif time.time() - landing_stable_start_time >= stable_time:
                    self.get_logger().info("Landing detected.")
                    return True
            else:
                # Reset stable time counter if conditions are not met
                landing_stable_start_time = None

            # Small delay to keep the loop from running too fast
            rclpy.spin_once(self, timeout_sec=0.1)


