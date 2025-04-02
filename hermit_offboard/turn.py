#!/usr/bin/python3
import time
import yaml
import rclpy
from hermit_offboard.drone_controller import DroneController
import os
from std_msgs.msg import String
import math

class MissionTaskManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "ARM"
        self.search_point_index = 0
        self.hold_start_time = None
        self.takeoff_altitude = -3.0

        self.search_points = [
            [0.0, 0.0, self.takeoff_altitude, -math.pi / 2],
            [0.0, 0.0, self.takeoff_altitude, 0.0],
            [0.0, 0.0, self.takeoff_altitude, math.pi / 2],
            [0.0, 0.0, self.takeoff_altitude, 0.0],
            [0.0, 0.0, self.takeoff_altitude]
        ]

    # Swap x and y in simulation (only once, no code change needed)
        for i in range(len(self.search_points)):
            point = self.search_points[i]
            if len(point) >= 2:
                point[0], point[1] = point[1], point[0]  # swap x <-> y

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

        if self.drone.current_yaw is not None:
            if not hasattr(self.drone, "initial_yaw") or self.drone.initial_yaw is None:
                self.drone.initial_yaw = self.drone.current_yaw
                print(f"Initial yaw locked: {self.drone.initial_yaw:.2f} rad")

            if not hasattr(self.drone, "target_yaw") or self.drone.target_yaw is None:
                self.drone.target_yaw = self.drone.initial_yaw

            self.state = "TAKEOFF"
        else:
            print("Waiting for current_yaw to be available...")


    def takeoff_drone(self):
        self.drone.publishing_setpoint(0.0, 0.0, self.takeoff_altitude, self.drone.target_yaw)
        if self.drone.reached_pose:
            self.hold_start_time = time.time()
            self.drone.reached_pose = False
            self.state = "HOLD"

    def hold_position(self):
        self.drone.publishing_setpoint(0.0, 0.0, self.takeoff_altitude)
        print("Holding at takeoff position")
        if time.time() - self.hold_start_time >= 3:
            self.state = "SEARCH_POINTS"

    # FUNCIONA MAS TEM BUG
    def navigate_search_points(self):
        if self.search_point_index < len(self.search_points):
            point = self.search_points[self.search_point_index]
            x, y, z = point[:3]

            if len(point) > 3 and point[3] is not None:
                yaw_offset = point[3]
                yaw_absolute = math.atan2(math.sin(self.drone.initial_yaw + yaw_offset), math.cos(self.drone.initial_yaw + yaw_offset))
                self.drone.target_yaw = yaw_absolute
            else:
                yaw_absolute = self.drone.target_yaw

            self.drone.publishing_setpoint(x, y, z, yaw_absolute)
            print(f"Navigating to search point {self.search_point_index}: {point}")

            if self.drone.reached_pose:
                self.search_point_index += 1
                self.hold_start_time = time.time()
                self.drone.reached_pose = False
                self.state = "HOLD_AT_SEARCH_POINT"
        else:
            self.state = "FINAL_RETURN"

    def hold_at_search_point(self):
        point = self.search_points[self.search_point_index - 1]
        x, y, z = point[:3]
        self.drone.publishing_setpoint(x, y, z)
        print(f"Holding at search point {self.search_point_index - 1} position: {point}")
        
        if time.time() - self.hold_start_time >= 5:
            self.state = "SEARCH_POINTS"

    def final_return(self):
        self.drone.publishing_setpoint(0.0, 0.0, -2.0, self.drone.target_yaw)
        if self.drone.reached_pose:
            self.drone.detected_land = False
            self.drone.reached_pose = False
            self.state = "FINAL_LAND"

    def final_land(self):
        self.drone.publish_landing_setpoint(0.0, 0.0)
        print("Descending for final landing...")
        if self.drone.detected_land:
            self.state = "DISARM"

    def disarm_drone(self):
        self.drone.disarm()
        self.drone.get_logger().info("Mission complete. Drone disarmed.")

    def battery_failsafe(self):
        if self.drone.low_battery:
            self.state = "FINAL_RETURN"

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    node = rclpy.create_node('mission_task_manager')

    mission = MissionTaskManager(drone)
    node.destroy_node()

    while rclpy.ok():
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
