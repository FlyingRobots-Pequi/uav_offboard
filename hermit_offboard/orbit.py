import time
import math
import rclpy
from hermit_offboard.drone_controller import DroneController

class MissionTaskManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "ARM"
        self.search_point_index = 0
        self.hold_start_time = None
        self.takeoff_altitude = -3.0
        self.orbit_active = False
        self.orbit_start_time = None
        self.orbit_duration = 45.0  # segundos para 1 volta
        self.orbit_radius = 3.0

        self.search_points = [
            [0.0, 0.0, self.takeoff_altitude],                       # Takeoff
            [0.0, 0.0, self.takeoff_altitude, "orbit_realtime"],    # Full orbit realtime
            [0.0, 0.0, -0.5]                                         # Land
        ]

    def execute_mission_step(self):
        print(f"[STATE] Current: {self.state}")
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
        print("[ARM] Engaging offboard mode and arming drone")
        self.drone.engage_offBoard_mode()
        self.drone.arm()
        if self.drone.current_yaw is not None:
            self.drone.initial_yaw = self.drone.current_yaw
            self.drone.target_yaw = self.drone.initial_yaw
            print(f"[ARM] Initial yaw locked: {self.drone.initial_yaw:.2f} rad")
            self.state = "TAKEOFF"

    def takeoff_drone(self):
        print("[TAKEOFF] Ascending to altitude")
        self.drone.publishing_setpoint(0.0, 0.0, self.takeoff_altitude, self.drone.target_yaw)
        if self.drone.reached_pose:
            self.hold_start_time = time.time()
            self.drone.reached_pose = False
            self.state = "HOLD"

    def hold_position(self):
        print("[HOLD] Holding position")
        self.drone.publishing_setpoint(0.0, 0.0, self.takeoff_altitude)
        if time.time() - self.hold_start_time >= 2:
            self.state = "SEARCH_POINTS"

    def navigate_search_points(self):
        if self.search_point_index < len(self.search_points):
            point = self.search_points[self.search_point_index]
            x, y, z = point[:3]
            print(f"[NAV] Navigating to point {self.search_point_index}: {point}")

            if len(point) > 3 and point[3] == "orbit_realtime":
                if not self.orbit_active:
                    print("[ORBIT_REALTIME] Starting continuous orbit")
                    self.orbit_active = True
                    self.orbit_start_time = time.time()
                t = time.time() - self.orbit_start_time
                if t <= self.orbit_duration:
                    theta = 2 * math.pi * (t / self.orbit_duration)
                    ox = x + self.orbit_radius * math.cos(theta)
                    oy = y + self.orbit_radius * math.sin(theta)
                    self.drone.publishing_setpoint(ox, oy, z, self.drone.target_yaw)
                else:
                    print("[ORBIT_REALTIME] Orbit complete")
                    self.orbit_active = False
                    self.search_point_index += 1
                    self.hold_start_time = time.time()
                    self.state = "HOLD_AT_SEARCH_POINT"
            else:
                self.drone.publishing_setpoint(x, y, z, self.drone.target_yaw)
                if self.drone.reached_pose:
                    print(f"[NAV] Reached point {self.search_point_index}")
                    self.search_point_index += 1
                    self.hold_start_time = time.time()
                    self.drone.reached_pose = False
                    self.state = "HOLD_AT_SEARCH_POINT"
        else:
            print("[NAV] All search points completed. Proceeding to land.")
            self.state = "FINAL_RETURN"

    def hold_at_search_point(self):
        point = self.search_points[self.search_point_index - 1]
        x, y, z = point[:3]
        print(f"[HOLD] Holding at point {self.search_point_index - 1}: {point}")
        self.drone.publishing_setpoint(x, y, z)
        if time.time() - self.hold_start_time >= 2:
            self.state = "SEARCH_POINTS"

    def final_return(self):
        self.drone.publishing_setpoint(0.0, 0.0, -2.0, self.drone.target_yaw)
        if self.drone.reached_pose:
            self.drone.detected_land = False
            self.drone.reached_pose = False
            self.state = "FINAL_LAND"

    def final_land(self):
        print("[LAND] Descending for final landing")
        self.drone.publish_landing_setpoint(0.0, 0.0, -0.5, self.drone.target_yaw)
        if self.drone.detected_land:
            self.state = "DISARM"

    def disarm_drone(self):
        print("[DISARM] Disarming drone")
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