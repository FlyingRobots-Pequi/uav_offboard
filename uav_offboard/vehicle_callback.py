from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus, FailsafeFlags, VehicleCommandAck, OffboardControlMode, HomePosition, TakeoffStatus, VehicleLandDetected, BatteryStatus
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import os
import yaml

class VehicleCallback:
    def __init__(self, node):
        self.node = node
        self.status_labels = self.load_yaml_file('uav_offboard', 'config/uav_status.yaml')

        self.sub_local_position_ = self.node.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.cb_local_position,
            qos_profile_sensor_data
        )

        self.sub_odometry_ = self.node.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.cb_odometry,
            qos_profile_sensor_data
        )

        self.sub_vehicle_status_ = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.cb_vehicle_status,
            qos_profile_sensor_data
        )

        self.sub_failsafe_flags_ = self.node.create_subscription(
            FailsafeFlags,
            '/fmu/out/failsafe_flags',
            self.cb_failsafe_flags,
            qos_profile_sensor_data
        )

        self.sub_vehicle_command_ack_ = self.node.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.cb_vehicle_command_ack,
            qos_profile_sensor_data
        )

        self.sub_offboard_control_mode_ = self.node.create_subscription(
            OffboardControlMode,
            '/fmu/out/offboard_control_mode',
            self.cb_offboard_control_mode,
            qos_profile_sensor_data
        )

        self.sub_home_position_ = self.node.create_subscription(
            HomePosition,
            '/fmu/out/home_position',
            self.cb_home_position,
            qos_profile_sensor_data
        )

        self.sub_takeoff_status_ = self.node.create_subscription(
            TakeoffStatus,
            '/fmu/out/takeoff_status',
            self.cb_takeoff_status,
            qos_profile_sensor_data
        )

        self.sub_land_detected_ = self.node.create_subscription(
            VehicleLandDetected,
            '/fmu/out/land_detected',
            self.cb_land_detected,
            qos_profile_sensor_data
        )

        self.sub_battery_status_ = self.node.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.cb_battery_status,
            qos_profile_sensor_data
        )

    def cb_local_position(self, msg):
        self.local_position = msg
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz
        self.current_ax = msg.ax
        self.current_ay = msg.ay
        self.current_az = msg.az
        self.current_heading = msg.heading
        # self.self.node.get_logger().info(f"[local_pos] x={msg.x:.2f}")

    def cb_odometry(self, msg):
        self.odometry = msg
        
        self.position = msg.position
        self.quaternion = msg.q

        self.velocity = msg.velocity
        self.angular_velocity = msg.angular_velocity

        self.var_position = msg.position_variance
        self.var_velocity = msg.velocity_variance
        self.var_orientation = msg.orientation_variance

    def cb_vehicle_status(self, msg):
        self.vehicle_status = msg 
        self.arming_state = msg.arming_state # uint8
        self.latest_disarming_reason = msg.latest_disarming_reason # uint8
        self.nav_state = msg.nav_state # uint8
        self.failure_detector_status = msg.failure_detector_status # uint16
        self.failsafe = msg.failsafe # bool
        self.safety_off = msg.safety_off # bool
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass # bool

        # code = msg.arming_state
        # label = self.status_labels["vehicle_status"]["arming_state"].get(code, "Unknown")
        # self.self.node.get_logger().info(f"[arming_state] Takeoff={code}: {label}")

    def cb_failsafe_flags(self, msg):
        self.failsafe_flags = msg
        self.attitude_invalid = msg.attitude_invalid
        self.local_altitude_invalid = msg.local_altitude_invalid
        self.local_position_invalid = msg.local_position_invalid
        self.local_position_invalid_relaxed = msg.local_position_invalid_relaxed
        self.local_velocity_invalid = msg.local_velocity_invalid
        self.global_position_invalid = msg.global_position_invalid
        self.auto_mission_missing = msg.auto_mission_missing
        self.offboard_control_signal_lost = msg.offboard_control_signal_lost
        self.home_position_invalid = msg.home_position_invalid
        self.manual_control_signal_lost = msg.manual_control_signal_lost
        self.gcs_connection_lost = msg.gcs_connection_lost
        self.battery_warning = msg.battery_warning
        self.battery_low_remaining_time = msg.battery_low_remaining_time
        self.mission_failure = msg.mission_failure
        self.flight_time_limit_exceeded = msg.flight_time_limit_exceeded
        self.local_position_accuracy_low = msg.local_position_accuracy_low
        self.fd_critical_failure = msg.fd_critical_failure
        self.fd_esc_arming_failure = msg.fd_esc_arming_failure
        self.fd_imbalanced_prop = msg.fd_imbalanced_prop
        self.fd_motor_failure = msg.fd_motor_failure

    def cb_vehicle_command_ack(self, msg):
        self.vehicle_command_ack = msg
        self.command_ack = msg.command
        self.result = msg.result
        self.target_system = msg.target_system
        self.target_component = msg.target_component
        self.timestamp = msg.timestamp

    def cb_offboard_control_mode(self, msg):
        self.offboard_control_mode = msg
        self.timestamp = msg.timestamp
        self.position = msg.position
        self.velocity = msg.velocity
        self.acceleration = msg.acceleration
        self.attitude = msg.attitude
        self.body_rate = msg.body_rate
        self.thrust_and_torque = msg.thrust_and_torque
        self.direct_actuator = msg.direct_actuator

    def cb_home_position(self, msg):
        self.home_position = msg
        self.home_x = msg.x
        self.home_y = msg.y
        self.home_z = msg.z
        self.home_yaw = msg.yaw

    def cb_takeoff_status(self, msg):
        self.takeoff_status = msg
        self.takeoff_state = msg.takeoff_state

        # code = msg.takeoff_state
        # label = self.status_labels["takeoff_state"].get(code, "Unknown")
        # self.self.node.get_logger().info(f"[takeoff_state] Takeoff={code}: {label}")


    def cb_land_detected(self, msg):
        self.land_detected = msg
        self.freefall = msg.freefall
        self.ground_contact = msg.ground_contact
        self.maybe_landed = msg.maybe_landed
        self.landed = msg.landed
        self.in_ground_effect = msg.in_ground_effect
        self.in_descend = msg.in_descend
        self.has_low_throttle = msg.has_low_throttle
        self.vertical_movement = msg.vertical_movement
        self.horizontal_movement = msg.horizontal_movement
        self.rotational_movement = msg.rotational_movement
        self.close_to_ground_or_skipped_check = msg.close_to_ground_or_skipped_check
        self.at_rest = msg.at_rest

    def cb_battery_status(self, msg):
        self.battery_status = msg
        self.connected = msg.connected
        self.voltage_v = msg.voltage_v
        self.current_a = msg.current_a
        self.remaining = msg.remaining
        self.temperature = msg.temperature
        self.cell_count = msg.cell_count

    def load_yaml_file(self, package: str, relative_path: str):

        base = get_package_share_directory(package)
        full_path = os.path.join(base, relative_path)
        with open(full_path, 'r') as f:
            return yaml.safe_load(f)
