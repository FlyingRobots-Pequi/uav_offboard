#!/usr/bin/python3
import time
import yaml
import rclpy
from drone_controller import DroneController

class MissionTaskManager:
    def __init__(self, drone_controller, config_file_path):
        self.drone = drone_controller
        self.state = "ARM"
        self.config_file_path = config_file_path
        self.gotopoints = self.load_goto_setpoints(config_file_path)
        self.goto_point_index = 0
        self.search_point_index = 0
        self.hold_start_time = None

        self.search_points = [
            # [2.0, 0.0, -2.0],
            # [0.0, -2.0, -2.0],
            # [0.0, 2.0, -2.0],
            [0.0, 0.0, -2.0],
            [-1.0, 1.0, -2.0]
        ]

    def execute_mission_step(self):
        # State machine for mission steps
        if self.state == "ARM":
            self.drone.arm()
            self.drone.engage_offBoard_mode()
            self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            self.drone.publish_offboard_control_mode()
            self.drone.publish_takeoff_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
            if self.drone.takeoff_success:
                self.hold_start_time = time.time()
                self.state = "HOLD"

        elif self.state == "HOLD":
            self.drone.publish_offboard_control_mode()
            self.drone.hover(0.0, 0.0, self.drone.takeoff_altitude)
            if time.time() - self.hold_start_time >= 5:
                self.state = "SEARCH_POINTS"

        elif self.state == "SEARCH_POINTS":
            # Navigate through search points
            if self.search_point_index < len(self.search_points) - 1:
                point = self.search_points[self.search_point_index]
                self.drone.publish_offboard_control_mode()
                self.drone.goto_setpoint(*point)
                
                if not self.drone.goto:
                    self.search_point_index += 1  # Move to the next search point
                    self.drone.goto = True  # Reset the goto flag
            else:
                self.state = "HOLD_AT_SEARCH_END"
                self.hold_start_time = time.time()

        elif self.state == "HOLD_AT_SEARCH_END":
            self.drone.publish_offboard_control_mode()
            self.drone.hover(*self.search_points[-1])
            if time.time() - self.hold_start_time >= 5:
                self.goto_point_index = 0  # Reset to start navigating `gotopoints`
                self.state = "GOTO_BASE"

        elif self.state == "GOTO_BASE":
            # Reload goto setpoints if modified
            self.reload_goto_setpoints(self.config_file_path)

            # Ensure there are points to go to and the index is within range
            if self.goto_point_index < len(self.gotopoints):
                point = self.gotopoints[self.goto_point_index]
                self.drone.publish_offboard_control_mode()
                self.drone.goto_setpoint(*point)

                # Check if the drone has reached the point
                if not self.drone.goto:
                    print("Arrived at setpoint: ", point)
                    # Reset landing-related flags
                    self.drone.detected_land = False
                    self.landing_and_takeoff_sequence = False
                    self.state = "LAND_TAKEOFF_BASE"  # Transition to landing
            else:
                # If all points are visited or no points are loaded, move to the final return state
                self.state = "FINAL_RETURN"

        elif self.state == "LAND_TAKEOFF_BASE":
            self.drone.publish_offboard_control_mode()
            # Wait until the landing and takeoff sequence completes before advancing
            
            point = self.gotopoints[self.goto_point_index]
            
            
            if not self.drone.landing_and_takeoff_sequence:
                # self.drone.landing_and_takeoff(point[0], point[1], self.drone.takeoff_altitude)
                self.drone.landing_and_takeoff(point[0], point[1], self.drone.takeoff_altitude)
                # self.drone.landing_and_takeoff(self.drone.current_x, self.drone.current_y, self.drone.takeoff_altitude)
                # Sequence is complete; move to the next point if available
            else:
                self.drone.landing_and_takeoff_sequence = False
                self.drone.goto = True
                if self.goto_point_index < len(self.gotopoints) - 1:
                    self.goto_point_index += 1
                      # Reset for the next movement
                    self.state = "GOTO_BASE"
                else:
                    # If no more points, transition to final return
                    self.state = "FINAL_RETURN"

        elif self.state == "FINAL_RETURN":
            # Return to starting position (0.0, 0.0) and prepare to land
            self.drone.publish_offboard_control_mode()
            self.drone.goto_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
            if not self.drone.goto:
                self.drone.detected_land = False
                self.state = "FINAL_LAND"

        elif self.state == "FINAL_LAND":
            self.drone.publish_offboard_control_mode()
            self.drone.publish_landing_setpoint(0.0, 0.0)
            if self.drone.detected_land:
                self.state = "DISARM"

        elif self.state == "DISARM":
            self.drone.disarm()
            self.drone.get_logger().info("Mission complete. Drone disarmed.")

    def load_goto_setpoints(self, file_path):
        """Loads the setpoints from a YAML file."""
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        return config.get('setpoints', [])

    def reload_goto_setpoints(self, file_path):
        """Reloads the setpoints if the file has been updated."""
        new_gotopoints = self.load_goto_setpoints(file_path)
        
        # Update `gotopoints` only if new data is valid and not shorter than the current index
        if new_gotopoints and len(new_gotopoints) > self.goto_point_index:
            self.gotopoints = new_gotopoints

def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    mission = MissionTaskManager(drone, config_file_path='/home/ros2_ws/src/hermit_offboard/config/goto_setpoints.yaml')

    while rclpy.ok():
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
