# drone_controller.py
import rclpy
import time
import math
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
        self.integral_z = 0.0
        
        
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_error_z = 0.0

        self.dt = 0.1

        # Define max velocities for horizontal and vertical movement
        self.max_horizontal_velocity = 0.5  # Adjust as needed
        self.max_vertical_velocity_z = 0.5  # Adjust as needed
        
        
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
    #     self.vehicle_command_publisher_.publish(msg)
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
        # self.get_logger().info(f"[PUBLISH_OFFBOARD_CONTROL_HEARTBEAT_SIGNAL] Offboard control mode heartbeat signal set to {control_type}")

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
        msg.yaw = self.get_current_yaw()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        # self.get_logger().info(f"[PUB_POSITION_CONTROL_SETPOINT]Position control: Target position: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
        
    def publish_velocity_control_setpoint(self, target_vx, target_vy, target_vz):
                    
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)
        
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
        msg.yaw = self.get_current_yaw()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

        # self.get_logger().info(f"[PUB_VELOCITY_CONTROL_SETPOINT]Velocity control: Target velocity: ({target_vx:.2f}, {target_vy:.2f}, {target_vz:.2f})")
    
    def position_correction_controller(self, target_x, target_y, target_z):
        """
        Controls the x, y, and z position of the drone using a PID controller.
        
        Args:
            current_x (float): The current x-coordinate of the drone.
            current_y (float): The current y-coordinate of the drone.
            current_z (float): The current z-coordinate of the drone.
            target_x (float): The target x-coordinate for the drone.
            target_y (float): The target y-coordinate for the drone.
            target_z (float): The target z-coordinate for the drone (negative for heights above the ground).
        """
        
        current_x, current_y, current_z = self.get_current_position()
        
        # Ensure offboard mode is set for velocity control
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)

        # Error calculations for PID controller
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_z = target_z - current_z  # Z-axis oriented down, so positive error moves downward

        # Proportional terms
        p_term_x = self.k_p * error_x
        p_term_y = self.k_p * error_y
        p_term_z = self.k_p * error_z

        # Integral terms (summing error over time)
        self.integral_x += error_x * self.dt
        self.integral_y += error_y * self.dt
        self.integral_z += error_z * self.dt

        i_term_x = self.k_i * self.integral_x
        i_term_y = self.k_i * self.integral_y
        i_term_z = self.k_i * self.integral_z

        # Derivative terms (rate of change of error)
        d_term_x = self.k_d * (error_x - self.previous_error_x) / self.dt
        d_term_y = self.k_d * (error_y - self.previous_error_y) / self.dt
        d_term_z = self.k_d * (error_z - self.previous_error_z) / self.dt

        # Calculated velocities with PID terms
        velocity_x = p_term_x + i_term_x + d_term_x
        velocity_y = p_term_y + i_term_y + d_term_y
        velocity_z = p_term_z + i_term_z + d_term_z

        # Limit velocities to the maximum allowed
        velocity_x = max(-self.max_horizontal_velocity, min(velocity_x, self.max_horizontal_velocity))
        velocity_y = max(-self.max_horizontal_velocity, min(velocity_y, self.max_horizontal_velocity))
        velocity_z = max(-self.max_vertical_velocity_z, min(velocity_z, self.max_vertical_velocity_z))

        # Publish the velocity setpointland
        self.publish_velocity_control_setpoint(velocity_x, velocity_y, velocity_z)

        # Store errors for the next derivative calculation
        self.previous_error_x = error_x
        self.previous_error_y = error_y
        self.previous_error_z = error_z

        # Log the current velocity setpoints for debugging
        # self.get_logger().info(
        #     f" [POSITION_CORRECTION] Velocity setpoint: ({velocity_x:.2f}, {velocity_y:.2f}, {velocity_z:.2f}) for target ({target_x}, {target_y}, {target_z})"
        # )

        
    def takeoff(self, target_z, ascent_velocity=-0.5, altitude_threshold=0.1, velocity_threshold=0.03, acceleration_threshold=0.03, stable_time=2.0):
        """
        Command the drone to take off to a target altitude by ascending at a specified velocity 
        until stability is detected.

        Args:
            target_z (float): Target altitude for takeoff (in meters, negative for above-ground targets).
            ascent_velocity (float): Vertical ascent velocity (in m/s).
            altitude_threshold (float): Altitude range within which takeoff is considered successful.
            velocity_threshold (float): Threshold for vertical velocity indicating stopped ascent.
            acceleration_threshold (float): Threshold for vertical acceleration indicating stability.
            stable_time (float): Duration for which conditions must be met to confirm takeoff completion.
        """
        # Set offboard mode for velocity control
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)

        # Initialize timer for stability conditions
        takeoff_stable_start_time = None

        # Takeoff control loop
        while rclpy.ok():
            # Command upward velocity to achieve takeoff
            self.publish_velocity_control_setpoint(target_vx=0.0, target_vy=0.0, target_vz=ascent_velocity)

            # Get current altitude, vertical velocity, and acceleration
            current_altitude = self.get_current_position()[2]  # Z-axis altitude
            vertical_velocity = self.get_current_velocity()[2]
            vertical_acceleration = self.get_current_acceleration()[2]

            # Debugging: Log the altitude and velocity status
            self.get_logger().info(f"Current Altitude: {current_altitude}, Target Altitude: {target_z}, Altitude Threshold: {altitude_threshold}")

            #TODO: If the current altitude goes above the target_z, the drone will not stop ascending.

            # Check if altitude is close enough to the target
            if abs(abs(current_altitude) - abs(target_z)) <= altitude_threshold:
                ascent_velocity = 0.0  # Stop ascending once at target altitude 
                self.get_logger().info("[TAKEOFF]Target altitude reached. Checking for stability.")
                
                # Check stability (low velocity and acceleration)
                if abs(vertical_velocity) < velocity_threshold and abs(vertical_acceleration) < acceleration_threshold:
                    # Start or check the stable time counter
                    if takeoff_stable_start_time is None:
                        takeoff_stable_start_time = time.time()
                        self.get_logger().info("[TAKEOFF]Stability detected, starting stable time counter.")
                    elif time.time() - takeoff_stable_start_time >= stable_time:
                        self.get_logger().info("[TAKEOFF]Takeoff completed and stable at target altitude.")
                        return True
            else:
                # Reset stable time counter if stability conditions are not met
                takeoff_stable_start_time = None
                self.get_logger().info("[TAKEOFF]Ascending to target altitude.")

            # Prevent the loop from running too fast
            rclpy.spin_once(self, timeout_sec=0.1)


    def hold(self, target_hold_x, target_hold_y, target_hold_z):
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
        # self.is_holding_position = True
        
        # Start an infinite hold loop
        # while rclpy.ok():
        # Publish the position hold setpoint
        self.publish_position_control_setpoint(target_hold_x, target_hold_y, target_hold_z)

        # Log the holding position message periodically
        self.get_logger().info(f"[HOLD]Holding position at ({target_hold_x:.2f}, {target_hold_y:.2f}, {target_hold_z:.2f})")
        
        # Small delay to prevent overloading the control loop
        # rclpy.spin_once(self, timeout_sec=0.1)

        # Log exit from hold mode
        # self.get_logger().info("[HOLD]Exiting hold position mode.")

    def goto_setpoint(self, target_x, target_y, target_z, position_threshold=0.05, velocity_threshold=0.03, acceleration_threshold=0.03, stable_time=2.0):
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
            
            self.get_logger().info(f"[GOTO]Current position: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})")
            
            # Check if the position is within the target threshold
            if position_error < position_threshold:
                # Check if the drone is stable within velocity and acceleration thresholds
                if (abs(velocity_x) < velocity_threshold and abs(velocity_y) < velocity_threshold and abs(velocity_z) < velocity_threshold and
                    abs(accel_x) < acceleration_threshold and abs(accel_y) < acceleration_threshold and abs(accel_z) < acceleration_threshold):
                    
                    # Start or check the stable time counter
                    if arrival_stable_start_time is None:
                        arrival_stable_start_time = time.time()
                    elif time.time() - arrival_stable_start_time >= stable_time:
                        self.get_logger().info(f"[GOTO]Arrived at setpoint: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
                        return True
            else:
                # Reset stable time counter if conditions are not met
                arrival_stable_start_time = None
            
            self.get_logger().info(f"[GOTO]Moving to setpoint: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")


            # Small delay to keep the loop from running too fast
            rclpy.spin_once(self, timeout_sec=0.1)

    def land(self, descent_velocity=0.25, velocity_threshold=0.05, acceleration_threshold=0.05, stable_time=2.0):
        """
        Command the drone to land by gradually descending until stability is detected near the ground.

        Args:
            descent_velocity (float): Downward velocity (m/s) to initiate descent.
            velocity_threshold (float): Threshold for vertical velocity indicating a stable descent.
            acceleration_threshold (float): Threshold for vertical acceleration indicating stability.
            stable_time (float): Duration for which conditions must be met to confirm landing.
        """
        # Set offboard mode to velocity control for descent
        self.publish_offboard_control_heartbeat_signal(position_control=False, velocity_control=True)

        # Initialize stable time counter for landingland
        landing_stable_start_time = None

        # Begin controlled descent loop
        while rclpy.ok():
            # Command a downward velocity to descend
            self.publish_velocity_control_setpoint(target_vx=0.0, target_vy=0.0, target_vz=descent_velocity)

            # Retrieve the current vertical velocity and acceleration
            vertical_velocity = self.get_current_velocity()[2]
            vertical_acceleration = self.get_current_acceleration()[2]

            # Debugging: Log the vertical descent progress and current metrics
            self.get_logger().info(f"[LAND]Vertical velocity: {vertical_velocity:.2f} m/s, acceleration: {vertical_acceleration:.2f} m/s^2")

            # Check if stability conditions indicate landing (low velocity and acceleration)
            if   0.0 < abs(vertical_velocity) < velocity_threshold and 0.0 < abs(vertical_acceleration) < acceleration_threshold:
                # Start or continue the stability timer if conditions are met
                if landing_stable_start_time is None:
                    landing_stable_start_time = time.time()
                    self.get_logger().info("[LAND]Landing stability detected, starting stable time counter.")
                elif time.time() - landing_stable_start_time >= stable_time:
                    # Confirm landing as stable for required duration
                    self.get_logger().info("[LAND]Landing complete and stable.")
                    return True
            else:
                # Reset the stability counter if conditions are not met continuously
                landing_stable_start_time = None
                self.get_logger().info("[LAND]Descent in progress, conditions not yet stable.")

            # Small delay to avoid running the loop too fast
            rclpy.spin_once(self, timeout_sec=0.1)

    