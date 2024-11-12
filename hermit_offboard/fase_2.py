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
            print("Holding at takeoff position")
            if time.time() - self.hold_start_time >= 5:
                self.state = "SEARCH_SHELF"

        elif self.state == "SEARCH_SHELF":
            self.drone.publish_offboard_control_mode()
            # Acredito que os valores iniciais sejam X e Y, adicionei 2m para o drone ir ao centro da arena
            self.drone.goto_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
            print("Moving 4 meters forward on X-axis (0.0, 0.0)")

            # Variando a altura para cada andar da prateleira
            search_points = [
             #    x    y     z
                [0.0, 0.0, -2.5],
                [0.0, 0.0, -2.0],
                [0.0, 0.0, -1.5],
                [0.0, 0.0, -1.0],
            ]

            lateral_steps = 3 
            lateral_distance = 1.0  #lembrar de reduzir valores de acordo com a documentação
            self.wait_time = None
            
            if self.search_point_index < len(search_points):
                x, y, z = search_points[self.search_point_index]
                
                self.drone.goto_setpoint(x, y, z)
                print(f"Moving to search position at (x: {x}, y: {y}, z: {z})")

                if not self.drone.goto:
                    if self.wait_time is None:
                        self.wait_time = time.time()
                    
                    elif time.time() - self.wait_time <= 2:
                        print(f"Arrived at search position ({x}, {y}, {z}). Performing search...")

                        for i in range(lateral_steps):  
                            new_y = y + lateral_distance * (i + 1)
                            self.drone.goto_setpoint(x, new_y, z)
                            print(f"Moving to lateral position (x: {x}, y: {new_y}, z: {z})")
                            # time.sleep(1.5)

                            if self.wait_time is None:
                                self.wait_time = time.time()
                            elif time.time() - self.wait_time <= 2:
                                self.wait_time = None
                                continue

                        # Volta Y inicial
                        self.drone.goto_setpoint(x, y, z)
                        print(f"Returning to center position (x: {x}, y: {y}, z: {z})")
                        # time.sleep(1)
                        
                        self.wait_time = None
                        self.search_point_index += 1
                else:
                    self.wait_time = None
                    
            else:
                self.state = "FINAL_RETURN"
                print("Completed searching all shelves. Returning to base.")

        elif self.state == "FINAL_RETURN":
            # Return to starting position (0.0, 0.0) and prepare to land
            self.drone.publish_offboard_control_mode()
            self.drone.goto_setpoint(0.0, 0.0, self.drone.takeoff_altitude)
            print("Returning to takeoff base at (0.0, 0.0)")
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
    node = rclpy.create_node('mission_task_manager')
    
    package_path = get_package_share_directory('hermit_offboard')
    config_file_path = os.path.join(package_path, 'config', 'goto_setpoints.yaml')
    
    mission = MissionTaskManager(drone, config_file_path=config_file_path)
    node.destroy_node()  # Clean up parameter node

    while rclpy.ok():
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
