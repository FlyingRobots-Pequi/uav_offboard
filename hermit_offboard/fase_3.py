#!/usr/bin/python3
import time
import yaml
import rclpy
from hermit_offboard.drone_controller import DroneController
import os
from std_msgs.msg import String

class MissionTaskManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "ARM"
        self.search_point_index = 0
        self.hold_start_time = None
        self.takeoff_altitude = -1.0
        
        self.base_A = [3.0, -0.5, -1.15]
        self.base_B = [2.0, -1.5, -1.15]
        self.base_C = [0.0, -1.5, -1.15]
        self.base_D = [3.0, -3.5, -1.15]
        self.base_E = [1.0, -3.5, -1.5]
        
        self.search_points = [
            self.base_A
        ]
        self.counter_qr_code_appended = 0
        self.visited_bases = set(["A"]) 
        
        # Set up QR code detection subscriber
        self.qr_code_subscriber = self.drone.create_subscription(
            String,
            'pequi/hermit/qr_code_detector',
            self.qr_code_callback,
            10
        )

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

    def qr_code_callback(self, msg):
        """Process QR code only during HOLD state."""
        if self.state == "HOLD_AT_SEARCH_POINT":
            self.detected_qr_code = msg.data
            print(self.detected_qr_code)
            print(f"Detected QR code: {self.detected_qr_code}")
            print(self.search_points)

            # Append the corresponding base to search_points if not already added
            if self.detected_qr_code == 'A' and 'A' not in self.visited_bases:
                self.search_points.append(self.base_A)
                self.visited_bases.add("A")
                self.counter_qr_code_appended += 1
                print("Appended base A to search points. Next base: A")

            elif self.detected_qr_code == 'B' and 'B' not in self.visited_bases:
                self.search_points.append(self.base_B)
                self.visited_bases.add("B")
                self.counter_qr_code_appended += 1
                print("Appended base B to search points. Next base: B")

            elif self.detected_qr_code == 'C' and 'C' not in self.visited_bases:
                self.search_points.append(self.base_C)
                self.visited_bases.add("C")
                self.counter_qr_code_appended += 1
                print("Appended base C to search points. Next base: C")

            elif self.detected_qr_code == 'D' and 'D' not in self.visited_bases:
                self.search_points.append(self.base_D)
                self.visited_bases.add("D")
                self.counter_qr_code_appended += 1
                print("Appended base D to search points. Next base: D")

            elif self.detected_qr_code == 'E' and 'E' not in self.visited_bases:
                self.search_points.append(self.base_E)
                self.visited_bases.add("E")
                self.counter_qr_code_appended += 1
                print("Appended base E to search points. Next base: E")

            else:
                print(f"QR code {self.detected_qr_code} is invalid or base already visited.")

    def arm_drone(self):
        self.drone.engage_offBoard_mode()
        self.drone.arm()
        self.state = "TAKEOFF"

    def takeoff_drone(self):
        self.drone.publish_takeoff_setpoint(0.0, 0.0, self.takeoff_altitude)
        if self.drone.takeoff_success:
            self.hold_start_time = time.time()
            self.state = "HOLD"

    def hold_position(self):
        self.drone.hover(0.0, 0.0, self.takeoff_altitude)
        print("Holding at takeoff position")
        if time.time() - self.hold_start_time >= 5:
            self.state = "SEARCH_POINTS"

    def navigate_search_points(self):
        # Navigate through search points
        if self.search_point_index < len(self.search_points):
            point = self.search_points[self.search_point_index]
            self.drone.goto_setpoint(*point)
            print("Going to base at setpoint: ", point)

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
        
        if time.time() - self.hold_start_time >= 5:
            self.state = "SEARCH_POINTS"  # Return to SEARCH_POINTS after holding

    def final_return(self):
        self.drone.goto_setpoint(0.0, 0.0, -1.0)
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
