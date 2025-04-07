#!/usr/bin/python3
import time
import yaml
import rclpy
from hermit_offboard.drone_controller import DroneController
import os

class MissionTaskManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "ARM"
        self.search_point_index = 0
        self.hold_start_time = None
        self.takeoff_altitude = -1.5
        
        self.z1 = -3.0

        self.x0 = 0.8
        self.x1 = 3.0
        self.x2 = 6.0
        self.x3 = 9.0
        self.x4 = 11.2

        self.y_m = 0.0
        self.y_l = 2.7
        self.y_r = -2.7

        # Path1

        # self.search_points = [
        #     [self.x0, self.y_m, self.z1],
        #     [self.x1, self.y_m, self.z1],
        #     [self.x1, self.y_m, self.z1],
        #     [self.x2, self.y_m, self.z1],
        #     [self.x1, self.y_m, self.z1],
        #     [self.x2, self.y_m, self.z1],
        #     [self.x0, self.y_m, self.z1]
        # ]

        # Orbit clock-wise
        self.search_points = [
            [self.x0, self.y_m, self.z1],
            [self.x1, self.y_l, self.z1],
            [self.x2, self.y_l, self.z1],
            [self.x3, self.y_l, self.z1],
            [self.x4, self.y_m, self.z1],
            [self.x3, self.y_r, self.z1],
            [self.x2, self.y_r, self.z1],
            [self.x1, self.y_r, self.z1],
            [self.x0, self.y_m, self.z1]
        ]

        # Orbit counter clock-wise
        # self.search_points = [
        #     [self.x0, self.y_m, self.z1],
        #     [self.x1, -self.y_l, self.z1],
        #     [self.x2, -self.y_l, self.z1],
        #     [self.x3, -self.y_l, self.z1],
        #     [self.x4, -self.y_m, self.z1],
        #     [self.x3, -self.y_r, self.z1],
        #     [self.x2, -self.y_r, self.z1],
        #     [self.x1, -self.y_r, self.z1],
        #     [self.x0, self.y_m, self.z1]
        # ]

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
        self.drone.publish_takeoff_setpoint(0.0, 0.0, self.takeoff_altitude)
        if self.drone.takeoff_success:
            self.hold_start_time = time.time()
            self.state = "HOLD"

    def hold_position(self):
        self.drone.hover(0.0, 0.0, self.takeoff_altitude)
        print("Holding at takeoff position")
        if time.time() - self.hold_start_time >= 3:
            self.state = "SEARCH_POINTS"

    def navigate_search_points(self):
        # Navigate through search points
        if self.search_point_index < len(self.search_points):
            point = self.search_points[self.search_point_index]
            self.drone.goto_setpoint(*point)
            print("Going to search point at setpoint: ", point)

            # Check if the drone has reached the setpoint
            if not self.drone.goto:
                self.search_point_index += 1  # Move to the next search point
                self.hold_start_time = time.time()  # Start hold timer
                self.state = "HOLD_AT_SEARCH_POINT"  # Transition to hold state
                self.drone.goto = True  # Reset the goto flag
        else:
            self.state = "FINAL_RETURN"

    def hold_at_search_point(self):
        # Hold the drone at the current search point for 5 seconds
        point = self.search_points[self.search_point_index - 1]
        self.drone.hover(*point)
        print(f"Holding at search point {self.search_point_index - 1} position: {point}")
        
        if time.time() - self.hold_start_time >= 3:
            self.state = "SEARCH_POINTS"  # Return to SEARCH_POINTS after holding

    def final_return(self):
        self.drone.goto_setpoint(0.0, 0.0, self.takeoff_altitude)
        print("Returning to takeoff base at (0.0, 0.0)")
        if not self.drone.goto:
            self.drone.detected_land = False
            self.state = "FINAL_LAND"

    def final_land(self):
        self.drone.publish_landing_setpoint(0.0, 0.0)
        if self.drone.detected_land:
            self.state = "DISARM"

    def disarm_drone(self):
        self.drone.disarm()
        self.drone.get_logger().info("Mission complete. Drone disarmed.")


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
