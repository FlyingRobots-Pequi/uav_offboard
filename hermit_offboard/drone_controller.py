# drone_controller.py
import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition, BatteryStatus
import math

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
            '/pequi/hermit/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/pequi/hermit/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/pequi/hermit/fmu/out/battery_status',
            self.battery_status_callback,
            qos_profile)

        # Create publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode,
            '/pequi/hermit/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint,
            '/pequi/hermit/fmu/in/trajectory_setpoint',
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            '/pequi/hermit/fmu/in/vehicle_command',
            qos_profile)   

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
        self.takeoff_altitude = -3.0
        self.landing_and_takeoff_sequence = False
        self.land_start_time = None
        self.landed_x = 0
        self.landed_y = 0
        self.landed_z = 0

        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.target_yaw = None

        self.reached_xy = False
        self.reached_z = False
        self.reached_xyz = False
        self.reached_yaw = False
        self.reached_pose = False

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.low_battery = False
        self.orbit_active = False

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
        # print("Arm command sent")
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
        msg = OffboardControlMode(position=True)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def hover(self, x, y, z, target_yaw=None):
        
        self.publish_offboard_control_mode()
        if target_yaw == None:
            target_yaw = self.target_yaw
        
        msg = TrajectorySetpoint(position=[x, y, z], yaw=target_yaw)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def goto_setpoint(self, x, y, z):

        self._yaw_rotation_initialized = False
        self.reached_yaw = False

        self.publish_offboard_control_mode()

        msg = TrajectorySetpoint(position=[x, y, z], yaw=self.current_yaw)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        if abs(self.current_x - x) < self.tolerance and abs(self.current_y - y) < self.tolerance and abs(self.current_altitude - z) < self.tolerance:
            print("Goto Setpoint reached.")
            self.goto = False

    def publish_takeoff_setpoint(self, x, y, z):

        self.publish_offboard_control_mode()

        msg = TrajectorySetpoint()

        # Check if the drone is below the takeoff altitude and has not yet achieved success
        msg.position = [x, y, z]
        # self.get_logger().info(f"Taking off: Current Altitude: {self.current_altitude:.2f}, Target Altitude: {z:.2f}")
        if z - self.tolerance < self.current_altitude < z + self.tolerance:

            self.takeoff_success = True
            # self.get_logger().info("Takeoff altitude reached successfully.")
        else: 
            self.takeoff_success = False
        msg.yaw = self.target_yaw  # Keep the yaw fixed
        self.publish_trajectory_setpoint_publisher(msg)

    def publish_landing_setpoint(self, x, y):

        self.publish_offboard_control_mode()

        msg = TrajectorySetpoint()

        # Set a threshold to detect when the drone is close to the ground (e.g., 0.1 meters)
        landing_threshold = 0.3  # You can adjust this based on how close to the ground you want to detect

        new_altitude = self.current_altitude + landing_threshold  # Decrease altitude in small steps 
        msg.position = [x, y, new_altitude]  # Set new target position
        # print(f"Landing... Current altitude: {self.current_altitude:.2f}, Target: {new_altitude:.2f}")

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
                self.landed_x = self.current_x
                self.landed_y = self.current_y
                self.landed_z = self.current_altitude
                self.detected_land = True 
                del self.landing_detection_start_time
            else:
                self.detected_land = False

        msg.yaw = self.target_yaw
        self.publish_trajectory_setpoint_publisher(msg)

    def _compute_xy_step(self, target_x, target_y):
        step_xy = 0.3
        threshold_xy = 0.05
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_xy = (dx**2 + dy**2)**0.5

        if distance_xy < threshold_xy:
            self.reached_xy = True
            return target_x, target_y
        else:
            self.reached_xy = False
            dir_x = dx / distance_xy
            dir_y = dy / distance_xy
            return self.current_x + step_xy * dir_x, self.current_y + step_xy * dir_y

    def _compute_z_step(self, target_z):
        step_z = 0.3
        threshold_z = 0.05
        dz = target_z - self.current_altitude

        if abs(dz) < threshold_z:
            self.reached_z = True
            return target_z
        else:
            self.reached_z = False
            return self.current_altitude + step_z * (dz / abs(dz))

    def publish_yaw_setpoint(self, x, y, z, delta_yaw):
        if not hasattr(self, "_yaw_rotation_initialized") or not self._yaw_rotation_initialized:
            self._yaw_start_yaw = self.current_yaw
            self._yaw_target_delta = delta_yaw
            self._yaw_accumulated = 0.0
            self._yaw_last = self.current_yaw
            self._yaw_rotation_initialized = True

        delta_step = self._compute_yaw_step_incremental()

        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = delta_step
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint_publisher(msg)

        # print(f"Rotating: Yaw now {self.current_yaw:.3f}, Accumulated: {self._yaw_accumulated:.3f}, Target ΔYaw: {self._yaw_target_delta:.3f}")

        if abs(self._yaw_accumulated) >= abs(self._yaw_target_delta):
            self.reached_yaw = True
            self.target_yaw = self.current_yaw
            self._yaw_rotation_initialized = False

    def _compute_yaw_step_incremental(self):
        yaw_step = 0.1  # rad
        current = self.current_yaw
        last = self._yaw_last

        # Compute how much yaw has changed since last step
        delta = self._normalize(current - last)
        self._yaw_accumulated += delta
        self._yaw_last = current

        remaining = self._yaw_target_delta - self._yaw_accumulated

        if abs(remaining) < 0.01:
            return self._normalize(current + remaining)

        direction = 1.0 if remaining > 0 else -1.0
        return self._normalize(current + direction * yaw_step)

    def _normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_z_setpoint(self, x, y, target_z):
        self.publish_offboard_control_mode()
        new_z = self._compute_z_step(target_z)
        
        msg = TrajectorySetpoint()
        msg.position = [x, y, new_z]
        msg.yaw = self.target_yaw
        self.publish_trajectory_setpoint_publisher(msg)

    def publish_xy_setpoint(self, target_x, target_y, z):
        self.publish_offboard_control_mode()
        new_x, new_y = self._compute_xy_step(target_x, target_y)
        
        msg = TrajectorySetpoint()
        msg.position = [new_x, new_y, z]
        msg.yaw = self.target_yaw
        self.publish_trajectory_setpoint_publisher(msg)

    def landing_and_takeoff(self, x, y, z):

        self.publish_offboard_control_mode()

        # Reset takeoff success flag at the start of each sequence
        self.takeoff_success = False
        # Landing phase
        if not self.detected_land:
            self.publish_landing_setpoint(x, y)  # Continue publishing landing setpoint
            self.landing_and_takeoff_sequence = False
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
                    return True
        else:
            return False

    def battery_status_callback(self, msg: BatteryStatus):
        self.low_battery = msg.remaining < 0.15
        if self.low_battery:
            self.get_logger().warn(f'Battery low: {msg.remaining * 100:.1f}%')

    def publish_trajectory_setpoint_publisher(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg) 