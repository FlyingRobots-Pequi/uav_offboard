#!/usr/bin/python3
import time
import yaml
import rclpy
from hermit_offboard.drone_controller import DroneController
import os
import math
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String  # coloque no topo

class MissionTaskManager(Node):
    def __init__(self, drone_controller, config_file_path):
        super().__init__('mission_task_manager')

        self.log_pub = self.create_publisher(String, '/mission_log', 10)

        self.drone = drone_controller
        self.state = "ARM"
        self.search_point_index = 0
        self.config_file_path = config_file_path

        (   
            self.position_setpoints,
            self.leftward_setpoints,
            self.rightward_setpoints
        ) = self.load_goto_setpoints(config_file_path)

        self.search_points = self.position_setpoints
        # self.search_setpoints = self.leftward_setpoints
        # self.search_points = self.rightward_setpoints

        self.hold_start_time = None
        self.hold_time = 3
        self.takeoff_altitude = -1.5

        for i in range(len(self.search_points)):
            point = self.search_points[i]
            if len(point) >= 2:
                point[0], point[1] = point[1], point[0]  # swap x <-> y

    def log(self, message: str):
        self.get_logger().info(message)
        self.log_pub.publish(String(data=message))


    def execute_mission_step(self):
        if self.state == "ARM":
            self.arm_drone()
        elif self.state == "TAKEOFF":
            self.takeoff_drone()
        elif self.state == "HOLD":
            self.hold_position()
        elif self.state == "SEARCH_POINTS":
            self.navigate_search_points()
        elif self.state == "HOLD_AT_SEARCH_POINT":
            self.hold_at_search_point()
        elif self.state == "FINAL_RETURN":
            self.final_return()
        elif self.state == "FINAL_LAND":
            self.final_land()
        elif self.state == "DISARM":
            self.disarm_drone()

    def arm_drone(self):
        self.drone.engage_offBoard_mode()
        self.drone.arm()
        self.log(f"Arm command sent")

        if self.drone.current_yaw is not None:
            if not hasattr(self.drone, "initial_yaw") or self.drone.initial_yaw is None:
                self.drone.initial_yaw = self.drone.current_yaw
                self.drone.initial_x = self.drone.current_x
                self.drone.initial_y = self.drone.current_y
                self.log(f"Registering home position: x:{self.drone.initial_x:.2f} m, y:{self.drone.initial_y:.2f} m, {self.drone.initial_yaw:.2f} rad")

            if not hasattr(self.drone, "target_yaw") or self.drone.target_yaw is None:
                self.drone.target_yaw = self.drone.initial_yaw

            self.state = "TAKEOFF"
        else:
            self.log("Waiting for current_yaw to be available...")

    def takeoff_drone(self):
        self.drone.publish_takeoff_setpoint(self.drone.initial_x, self.drone.initial_y, self.takeoff_altitude)
        self.log(f"Taking off... Target Altitude: {self.takeoff_altitude:.2f}, Current Altitude: {self.drone.current_altitude:.2f}")
        if self.drone.takeoff_success:
            self.log(f"Takeoff Altitude reached successfully.")
            self.hold_start_time = time.time()
            self.state = "HOLD"

    def hold_position(self):
        self.drone.hover(self.drone.initial_x, self.drone.initial_y, self.takeoff_altitude)
        self.log(
            f"Holding at takeoff position: x: {self.drone.initial_x:.3f}, y: {self.drone.initial_y:.3f}, "
            f"z: {self.drone.takeoff_altitude:.3f}, yaw: {self.drone.initial_yaw:.3f}"
        )
        if time.time() - self.hold_start_time >= self.hold_time:
            self.state = "SEARCH_POINTS"

    def navigate_search_points(self):
        if self.search_point_index < len(self.search_points):
            point = self.search_points[self.search_point_index]

            if len(point) == 4:
                self.log(f"Rotating: Yaw now {self.drone.current_yaw:.3f}, Target ΔYaw: {point[3]:.3f}")
                self.drone.publish_yaw_setpoint(*point)
            else:
                self.drone.goto_setpoint(*point)

            self.log(f"Navigating to: {point}")

            if len(point) == 4:
                if self.drone.reached_yaw:
                    self.search_point_index += 1
                    self.hold_start_time = time.time()
                    self.state = "HOLD_AT_SEARCH_POINT"
                    self.drone.reached_yaw = False
            else:
                if not self.drone.goto:
                    self.search_point_index += 1
                    self.hold_start_time = time.time()
                    self.state = "HOLD_AT_SEARCH_POINT"
                    self.drone.goto = True
        else:
            self.state = "FINAL_RETURN"

    def hold_at_search_point(self):
        point = self.search_points[self.search_point_index - 1]
        self.drone.hover(point[0], point[1], point[2], self.drone.target_yaw)
        self.log(f"Holding at search point {self.search_point_index - 1} position: {point}")

        if time.time() - self.hold_start_time >= self.hold_time:
            self.state = "SEARCH_POINTS"

    def final_return(self):
        self.drone.goto_setpoint(self.drone.initial_x, self.drone.initial_y, self.takeoff_altitude)
        self.log(
            f"Returning to takeoff base at ({self.drone.initial_x:.3f}, {self.drone.initial_y:.3f})"
        )
        if not self.drone.goto:
            self.drone.detected_land = False
            self.state = "FINAL_LAND"

    def final_land(self):
        self.drone.publish_landing_setpoint(self.drone.initial_x, self.drone.initial_y)
        self.log(f"Landing... Current altitude: {self.drone.current_altitude:.2f}")
        if self.drone.detected_land:
            self.log("Landed. Disarm Command sent.")
            self.state = "DISARM"

    def disarm_drone(self):
        self.drone.disarm()
        self.log("Mission complete. Drone disarmed.")

    def load_goto_setpoints(self, file_path):
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        return (
            config.get('position_setpoints', []),
            config.get('traverse_leftward', []),
            config.get('traverse_rightward', [])
        )

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    node = rclpy.create_node('mission_task_manager')

    package_path = get_package_share_directory('hermit_offboard')
    config_file_path = os.path.join(package_path, 'config', 'goto_setpoints.yaml')

    mission = MissionTaskManager(drone, config_file_path=config_file_path)
    node.destroy_node()

    while rclpy.ok():
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
