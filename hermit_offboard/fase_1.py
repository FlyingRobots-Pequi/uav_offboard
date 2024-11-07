#!/usr/bin/python3
import time
import rclpy
from drone_controller import DroneController

class MissionTaskManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "ARM"
        self.gotopoints = [
            [2.0, 0.0, -2.0],
            [2.0, 2.0, -2.0],
            [3.0, -1.0, -2.0],
            [1.0, -1.0, -2.0],
            [0.0, 0.0, -2.0]
        ]
        self.current_point_index = 0
        self.hold_start_time = None

    def execute_mission_step(self):
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
            point = self.gotopoints[self.current_point_index]
            self.drone.publish_offboard_control_mode()
            self.drone.goto_setpoint(*point)
            if not self.drone.goto:
                self.hold_start_time = time.time()
                self.state = "LAND"

        elif self.state == "LAND":
            self.drone.publish_offboard_control_mode()
            complete = self.drone.landing_and_takeoff(self.drone.current_x, self.drone.current_y, self.drone.takeoff_altitude)
            
            # Transition to the next state only if the sequence is fully completed
            if complete:
                self.hold_start_time = time.time()
                self.state = "HOVER_AFTER_TAKEOFF"


        elif self.state == "HOVER_AFTER_TAKEOFF":
            point = self.gotopoints[self.current_point_index]
            self.drone.publish_offboard_control_mode()
            self.drone.hover(*point)
            if time.time() - self.hold_start_time >= 5:
                if self.current_point_index < len(self.gotopoints) - 1:
                    self.current_point_index += 1
                    self.state = "GOTO"
                else:
                    self.state = "FINAL_LAND"

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
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
