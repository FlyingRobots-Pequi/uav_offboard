#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from drone_controller import DroneController

class MissionTaskManager(Node):
    def __init__(self, drone_controller):
        super().__init__('mission_task_manager')
        self.drone = drone_controller
        self.state = "ARM"
        self.waypoints = []  # Holds the list of waypoints
        self.current_point_index = 0
        self.hold_start_time = None

        # Subscribe to /hermit_waypoints topic to receive waypoints
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hermit_waypoints',
            self.waypoints_callback,
            10
        )
        self.waypoints_received = False

    def waypoints_callback(self, msg):
        """Callback to handle incoming waypoints."""
        # Process the flat list of waypoints into a list of (x, y, z) coordinate groups
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().error("Invalid waypoints data. Must be a multiple of 3.")
            return
        
        # Convert flat list to a list of waypoints [[x1, y1, z1], [x2, y2, z2], ...]
        self.waypoints = [[data[i], data[i + 1], data[i + 2]] for i in range(0, len(data), 3)]
        self.get_logger().info(f"Received waypoints: {self.waypoints}")
        self.waypoints_received = True

    def execute_mission_step(self):
        # Wait until waypoints are received before proceeding
        if not self.waypoints_received:
            self.get_logger().info("Waiting for waypoints...")
            return

        # State machine for mission steps
        if self.state == "ARM":
            self.drone.arm()
            self.drone.engage_offBoard_mode()  # Engage offboard mode immediately after arming
            self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            self.drone.publish_offboard_control_mode()  # Keep offboard mode active
            self.drone.publish_takeoff_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
            if self.drone.takeoff_success:
                self.hold_start_time = time.time()
                self.state = "HOVER"

        elif self.state == "HOVER":
            self.drone.publish_offboard_control_mode()
            self.drone.hover(0.0, 0.0, self.drone.takeoff_altitude)
            if time.time() - self.hold_start_time >= 5:
                self.state = "GOTO"

        elif self.state == "GOTO":
            if self.current_point_index < len(self.waypoints):
                point = self.waypoints[self.current_point_index]
                self.drone.publish_offboard_control_mode()
                self.drone.goto_setpoint(*point)
                if not self.drone.goto:
                    self.hold_start_time = time.time()
                    self.current_point_index += 1
                    self.state = "HOVER_AFTER_GOTO"
            else:
                self.state = "FINAL_LAND"

        elif self.state == "HOVER_AFTER_GOTO":
            point = self.waypoints[self.current_point_index - 1]
            self.drone.publish_offboard_control_mode()
            self.drone.hover(*point)
            if time.time() - self.hold_start_time >= 5:
                self.state = "GOTO"

        elif self.state == "FINAL_LAND":
            self.drone.publish_offboard_control_mode()
            self.drone.publish_landing_setpoint(self.drone.current_x, self.drone.current_y)
            if self.drone.detected_land:
                self.state = "DISARM"

        elif self.state == "DISARM":
            self.drone.disarm()
            self.drone.get_logger().info("Mission complete. Drone disarmed.")

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    mission = MissionTaskManager(drone)

    while rclpy.ok():
        rclpy.spin_once(mission)  # Use mission node for spinning
        mission.execute_mission_step()

    mission.destroy_node()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
