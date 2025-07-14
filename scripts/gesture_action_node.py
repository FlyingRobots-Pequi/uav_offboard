import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_interfaces.srv import VehicleCommander, SetpointControl

GESTURE_TO_COMMAND = {
    "forward":   {"vx": 0.3, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
    "back":      {"vx": -0.3, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
    "left":      {"vx": 0.0, "vy": -0.3, "vz": 0.0, "yaw": 0.0},
    "right":     {"vx": 0.0, "vy": 0.3, "vz": 0.0, "yaw": 0.0},
    "up":        {"vx": 0.0, "vy": 0.0, "vz": -0.6, "yaw": 0.0},
    "down":      {"vx": 0.0, "vy": 0.0, "vz": 0.3, "yaw": 0.0},
    # "clockwise": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": -3.14},
    # "counter-clockwise": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 3.14},
    "hold":      {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
}

GESTURE_TO_ACTION = {
    "takeoff":       ("takeoff",       {"param1": 1.0}),
    "land":          ("land",          {}),
    "return":        ("return",        {}),
    "arm":           ("arm",           {}),
    "disarm":        ("disarm",        {}),
    "kill":          ("flight_termination", {}),
}

class GestureActionNode(Node):
    def __init__(self):
        super().__init__('gesture_action_node')

        self.cmd_cli = self.create_client(VehicleCommander, 'vehicle_commander')
        self.setpoint_cli = self.create_client(SetpointControl, 'setpoint_controller')
        self.create_subscription(String, '/gesture_detected', self.cb_gesture, 10)

        self.last_gesture = None
        self.cmd_cli.wait_for_service()
        self.setpoint_cli.wait_for_service()
        self.get_logger().info("GestureActionNode ready.")

    def cb_gesture(self, msg):
        gesture = msg.data.strip()
        if gesture == self.last_gesture:
            return  # evita repetição contínua

        self.last_gesture = gesture
        self.get_logger().info(f"Gesto detectado: {gesture}")

        if gesture in GESTURE_TO_COMMAND:
            self.send_velocity_setpoint(GESTURE_TO_COMMAND[gesture])
        elif gesture in GESTURE_TO_ACTION:
            command, kwargs = GESTURE_TO_ACTION[gesture]
            self.send_vehicle_command(command, **kwargs)

    def send_velocity_setpoint(self, cmd):
        req = SetpointControl.Request()
        req.type = 'vel'
        req.vx = cmd["vx"]
        req.vy = cmd["vy"]
        req.vz = cmd["vz"]
        req.yaw = cmd["yaw"]
        self.setpoint_cli.call_async(req)

    def send_vehicle_command(self, command, **kwargs):
        req = VehicleCommander.Request()
        req.command = command
        req.mode = kwargs.get('mode', "")
        req.param1 = float(kwargs.get('param1', 0.0))
        self.cmd_cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    manager = GestureActionNode()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
