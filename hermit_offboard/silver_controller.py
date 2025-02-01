# drone_controller.py
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition

class SilverController(Node):
    
    def __init__(self):
        super().__init__('silver_controller')
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
        
        # Define drone state variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_ax = 0.0
        self.current_ay = 0.0
        self.current_az = 0.0
        self.takeoff_success = False
        self.landing_success = False
        self.stop_success = False
        self.hover_state = False
        self.takeoff_altitude = -1.0
        
 
        
        self.xy_error_threshold = 0.05
        self.z_error_threshold = 0.05
        self.yaw_error_threshold = 0.05
        
        self.tolerance_z = 0.05
        
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
        
    def publish_offboard_control_mode(self, set_position=True, set_velocity=False):
        msg = OffboardControlMode(position=set_position, velocity=set_velocity)
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)  
    
    def publish_trajectory_setpoint_publisher(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg) 
    
    def vehicle_local_position_callback(self, msg):
        
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        
        self.current_yaw = msg.heading
        
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz
        
        self.current_ax = msg.ax
        self.current_ay = msg.ay
        self.current_az = msg.az
        
    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def failsafe_vehicle_startup_position(self, x, y, z):

        tolerance_xy = 0.2
        tolerance_z = self.tolerance_z
        
        if tolerance_xy > x > -tolerance_xy:
            if tolerance_xy > y > -tolerance_xy:
                if (tolerance_z + self.landing_altitude)  > z > (self.landing_altitude - tolerance_z):
                    return True;
        else:
            return False
    
    def up(self, vz):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, 0.0, -vz], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)
        
    def down(self, vz):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, 0.0, vz], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)

    def left(self, vy):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, -vy, 0.0], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)
        
    def right(self, vy):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, vy, 0.0], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)

    def forward(self, vx):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[vx, 0.0, 0.0], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)

    def backward(self, vx):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[-vx, 0.0, 0.0], yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)

    def clockwise(self, vyaw):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, 0.0, 0.0], yaw=self.current_yaw + vyaw)
        self.publish_trajectory_setpoint_publisher(msg)
        
    def counter_clockwise(self, vyaw):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(velocity=[0.0, 0.0, 0.0], yaw=self.current_yaw - vyaw)
        self.publish_trajectory_setpoint_publisher(msg)

    def takeoff(self, x, y, z):
        self.publish_offboard_control_mode(set_position=True, set_velocity=False)
        msg = TrajectorySetpoint(
            position=[x, y, z],
            yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)
        if self.z_error_threshold > abs(self.current_z - z):
            self.takeoff_success = True
            self.hold()
        else:    
            self.takeoff_success = False
    
    def land(self, vz):
        self.publish_offboard_control_mode(set_position=False, set_velocity=True)
        msg = TrajectorySetpoint(
            velocity=[0.0, 0.0, vz],
            yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)
        if abs(self.current_vz) < 0.05:
            self.landing_success = True
        else:
            self.landing_success = False
    
    def hold(self, x, y, z):
        self.publish_offboard_control_mode(set_position=True, set_velocity=True)
        msg = TrajectorySetpoint(
            position=[x, y, z],
            velocity=[0.0, 0.0, 0.0],
            yaw=self.current_yaw)
        self.publish_trajectory_setpoint_publisher(msg)