#!/usr/bin/python3
import time
import rclpy
from hermit_offboard.drone_controller import DroneController
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

'''
    Precisa conferir como usar o takeoff e o land, eles usam a posição do
    drone e o drone_controller.py está usando velocity ao invés de position.
'''

class MissionTaskManager(Node):
    def __init__(self, drone_controller):
        super().__init__('mission_task_manager')
        self.drone = drone_controller
        self.state = "ARM"
        self.hold_start_time = None
        self.default_speed = 0.3
        self.last_command_time = time.time()

        # Subscriber for gesture recognition
        self.gesture_subscription = self.create_subscription(
            String,
            '/gesture_recognition',
            self.gesture_command,
            10
        )

    def execute_mission_step(self):
        if self.state == "ARM":
            self.arm_drone()
        elif self.state == "TAKEOFF":
            self.takeoff_drone()
        elif self.state == "GESTURE":
            if time.time() - self.last_command_time > 5:
                self.get_logger().warn("No command received for 5 seconds. Initiating landing.")
                self.state = "LAND"
        elif self.state == "LAND":
            self.land()
        elif self.state == "DISARM":
            self.disarm_drone()

    def arm_drone(self):
        self.drone.arm()
        self.drone.engage_offBoard_mode()
        self.state = "TAKEOFF"

    def takeoff_drone(self):
        self.drone.publish_offboard_control_mode()
        self.drone.publish_takeoff_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
        if self.drone.takeoff_success:
            self.hold_start_time = time.time()
            self.state = "GESTURE"

    def gesture_command(self, msg):
        self.last_command_time = time.time()  # Update the timestamp when a command is received
        command = msg.data if msg else None
        if command == "FORWARD":
            self.drone.move_forward(self.default_speed)
            self.get_logger().info("Moving forward.")
        elif command == "BACKWARD":
            self.drone.move_backward(self.default_speed)
            self.get_logger().info("Moving backward.")
        elif command == "LEFT":
            self.drone.move_left(self.default_speed)
            self.get_logger().info("Moving left.")
        elif command == "RIGHT":
            self.drone.move_right(self.default_speed)
            self.get_logger().info("Moving right.")
        elif command == "UP":
            self.drone.move_up(self.default_speed)
            self.get_logger().info("Moving up.")
        elif command == "DOWN":
            self.drone.move_down(self.default_speed)
            self.get_logger().info("Moving down.")
        elif command == "HOLD":
            self.drone.stop()
            self.get_logger().info("Hold position.")
        else:
            self.get_logger().warn("Invalid command received.")

    def land(self):
        self.drone.publish_offboard_control_mode()
        self.drone.publish_landing_setpoint(0.0, 0.0)
        if self.drone.detected_land:
            self.state = "DISARM"

    def disarm_drone(self):
        self.drone.disarm()
        self.get_logger().info("Mission complete. Drone disarmed.")

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    mission_manager = MissionTaskManager(drone)

    while rclpy.ok():
        rclpy.spin_once(mission_manager)
        mission_manager.execute_mission_step()

    mission_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
