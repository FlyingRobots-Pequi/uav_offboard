# vehicle_commander.py

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleCommand

class VehicleCommander:
    """
    VehicleCommander class to handle vehicle commands.

    Args:
        node (Node): The ROS2 node instance.
        uav_namespace (str): Namespace prefix for UAV topics (e.g., "/pequi/hermit" or "").
    """

    def __init__(self, node, uav_namespace=""):
        self.node = node
        self.uav_namespace = uav_namespace
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Build topic name with namespace
        topic_name = self._build_uav_topic('/fmu/in/vehicle_command')
        
        self.vehicle_command_publisher_ = self.node.create_publisher(
            VehicleCommand,
            topic_name,
            qos_profile)

    def _build_uav_topic(self, topic):
        """Build complete topic name with namespace prefix."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{topic}"
        return topic


    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        
        # self.node.get_logger().info(f"Published VehicleCommand: command={command}")

    def set_mode(self, mode: str):
        """
        Set the vehicle mode based on a string input.

        Args:
            mode (str): The desired mode as a string. Options include:
                        - "manual"
                        - "altctl"
                        - "posctl"
                        - "mission"
                        - "acro"
                        - "offboard"
                        - "stabilized"
        """
        mode_mapping = {
            "manual": 1.0,
            "altctl": 2.0,
            "posctl": 3.0,
            "mission": 4.0,
            "acro": 5.0,
            "offboard": 6.0,
            "stabilized": 7.0,
        }

        mode_upper = mode.lower()
        if mode_upper not in mode_mapping:
            self.node.get_logger().error(f"Mode '{mode}' is not recognized.")
            return

        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=mode_mapping[mode_upper]
        )
        self.node.get_logger().info(f"Mode '{mode_upper}' command sent.")


    def arm(self):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=0.0
        )
        self.node.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=0.0
        )
        self.node.get_logger().info('Disarm command sent')

    def flight_termination(self):
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION,
            param1=1.0,
            param2=0.0
        )
        self.node.get_logger().info('Flight termination command sent')

    def takeoff(self, param1: float):
        """
        
        Take off to the specified MSL altitude.
        
        Args:
            alt (float): The desired altitude in meters.
        
        """

        alt = param1

        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=-1.0,
            param2=0.0,
            param3=0.0,
            param4=float('nan'),
            param5=float('nan'),
            param6=float('nan'),
            param7=alt
        )
        self.node.get_logger().info('Takeoff command sent')

    def land(self):
        """Initiate landing."""
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.node.get_logger().info("Switching to land mode")
