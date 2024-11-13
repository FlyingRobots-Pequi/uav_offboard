#!/usr/bin/python3
import time
import yaml
import rclpy
from hermit_offboard.drone_controller import DroneController
import os
from ament_index_python.packages import get_package_share_directory

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
            [0.0, -0.0, -1.5]
        ]
        
        self.takeoff_altitude = -1.5
        self.hold_time = 5

    def execute_mission_step(self):
        if self.state == "ARM":
            self.arm_drone()
        elif self.state == "TAKEOFF":
            self.initial_takeoff_drone()
        elif self.state == "HOLD":
            self.hold_position("SEARCH_POINTS", self.search_points, self.search_point_index)
        elif self.state == "SEARCH_POINTS":
            self.navigate_search_points()
        elif self.state == "HOLD_ABOVE_POINT":
            self.hold_position("GOTO_EACH_BASE", self.gotopoints, self.goto_point_index)     
        elif self.state == "GOTO_EACH_BASE":
            self.navigate_goto_points()
        elif self.state == "HOLD_ABOVE_BASE":
            self.hold_position("LANDING", self.gotopoints, self.goto_point_index)    
        elif self.state == "LANDING":
            self.landing_sequence()
        elif self.state == "TAKEOFF_BASE":
            self.base_takeoff()
        elif self.state == "FINAL_RETURN":
            self.final_return()
        elif self.state == "FINAL_LAND":
            self.final_land()
        elif self.state == "DISARM":
            self.disarm_drone()
        elif self.state == "MISSION_COMPLETE":
            self.drone.get_logger().info("Mission completed.")

    def arm_drone(self):
        self.drone.set_mode("OFFBOARD")
        self.drone.arm()
        self.state = "TAKEOFF"
        self.drone.get_logger().info("Armed and transitioned to TAKEOFF state.")

    def initial_takeoff_drone(self):
        
        takeoff_successful = self.drone.takeoff(target_altitude=self.takeoff_altitude)
        
        if takeoff_successful:
            self.hold_start_time = time.time()
            self.state = "HOLD"
            self.drone.get_logger().info("Takeoff successful. Transitioned to HOLD state.")

    def base_takeoff(self):
        
        takeoff_successful = self.drone.takeoff(target_altitude=self.takeoff_altitude)
        
        if takeoff_successful:
            self.hold_start_time = time.time()
            self.state = "HOLD_ABOVE_BASE"
            self.drone.get_logger().info("Takeoff successful. Transitioned to HOLD state.")

    #TODO Improve logig to not repeat code
    def hold_position(self, next_state, points, point_index):
        """
        Holds the drone's position for the specified duration and then transitions to the next state.
        
        Args:
            next_state (str): The state to transition to after the hold duration is complete.
        """
        # Check if the hold duration has passed
        if time.time() - self.hold_start_time >= self.hold_time:
            # Update the next state
            self.state = next_state
            self.drone.get_logger().info(f"Hold complete. Transitioning to {next_state}.")
            
            # Update index if the next state is SEARCH_POINTS or GOTO_EACH_BASE
            if next_state == "SEARCH_POINTS":
                self.search_point_index += 1
            elif next_state == "GOTO_EACH_BASE":
                self.goto_point_index += 1
            if point_index >= len(points):
                self.state = "FINAL_RETURN"

        else:
            # Keep holding position until the hold time is complete
            current_position = self.drone.get_current_position()
            self.drone.hold(*current_position)
            self.drone.get_logger().info(f"Holding position for {self.hold_time} seconds.")

    def navigate_to_points(self, points, point_index, next_state):
        """
        Generic function to navigate to a list of points and transition to HOLD state upon reaching each point.
        
        Args:
            points (list): The list of target points to navigate.
            point_index (int): The current index in the points list.
            next_state (str): The state to transition to after the hold completes.
        """
        if point_index < len(points):
            target_x, target_y, target_z = points[point_index]
            point_reached = self.drone.goto_setpoint(target_x, target_y, target_z)
            
            if point_reached:
                self.state = "HOLD_ABOVE_POINT"
                self.hold_start_time = time.time()
                self.next_state = next_state
                self.drone.get_logger().info(f"Reached point {point_index} in {next_state}. Holding position.")
        else:
            self.state = next_state
            self.drone.get_logger().info(f"All points in {next_state} completed. Moving to next phase.")

    def navigate_search_points(self):
        """
        Navigate through search points in sequence.
        """
        self.navigate_to_points(self.search_points, self.search_point_index, "SEARCH_POINTS")

    def navigate_goto_points(self):
        """
        Navigate through goto points in sequence.
        """
        self.navigate_to_points(self.gotopoints, self.goto_point_index, "GOTO_EACH_BASE")
        

    def landing_sequence(self):
        """
        Executes the landing process at the current base location, 
        applies correction to center on the base, and then initiates the landing descent.
        """
        # Get the target base center position
        target_x, target_y, _ = self.gotopoints[self.goto_point_index]

        # Step 1: Correct XY position to the center of the base
        current_x, current_y, _ = self.drone.get_current_position()
        self.drone.publish_xy_velocity_control_setpoint(
            current_x=current_x,
            current_y=current_y,
            target_x=target_x,
            target_y=target_y
        )
        
        # Check if the drone is close enough to the target center (within a small threshold)
        position_tolerance = 0.1  # meters, adjust as needed for precision
        if abs(current_x - target_x) < position_tolerance and abs(current_y - target_y) < position_tolerance:
            self.drone.get_logger().info("Position corrected above landing base. Starting descent.")

            # Step 2: Start landing with descent
            landing_successful = self.drone.land()

            if landing_successful:
                self.drone.get_logger().info("Landing successful. Taking off again.")
                self.state = "TAKEOFF_BASE"

                # Transition to HOLD_ABOVE_BASE after takeoff
                self.state = "HOLD_ABOVE_BASE"
                self.hold_start_time = time.time()
        else:
            # If not yet centered, remain in the LANDING state and adjust position
            self.drone.get_logger().info("Adjusting position to center above base.")

    def land_takeoff_sequence(self):
        self.drone.publish_offboard_control_mode()
        # Wait until the landing and takeoff sequence completes before advancing

        point = self.gotopoints[self.goto_point_index]

        if not self.drone.landing_and_takeoff_sequence:
            self.drone.landing_and_takeoff(point[0], point[1], self.drone.takeoff_altitude)
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

    def final_return(self):
        """
        Navigate the drone to the home location (0.0, 0.0, takeoff_altitude) and prepare for landing.
        """
        # Define the home location coordinates
        home_x, home_y, home_z = 0.0, 0.0, self.takeoff_altitude

        # Navigate to the home location
        home_reached = self.drone.goto_setpoint(home_x, home_y, home_z)

        if home_reached:
            # Once at the home location, transition to the FINAL_LAND state
            self.state = "FINAL_LAND"
            self.drone.get_logger().info("Home location reached. Preparing for final landing.")

    def final_land(self):
        """
        Executes the final landing at the home location and disarms the drone after landing.
        """
        # Initiate landing at the current position
        landing_successful = self.drone.land()

        if landing_successful:
            # Once landed, transition to the DISARM state
            self.state = "DISARM"
            self.drone.get_logger().info("Final landing complete. Transitioning to disarm.")

    def disarm_drone(self):
        """
        Disarms the drone, marking the end of the mission.
        """
        self.drone.disarm()
        self.state = "MISSION_COMPLETE"
        self.drone.get_logger().info("Drone disarmed. Mission complete.")

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
