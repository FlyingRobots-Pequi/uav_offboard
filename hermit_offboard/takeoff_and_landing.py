#!/usr/bin/python3

import time
import rclpy
from hermit_offboard.drone_controller import DroneController

class MissionTaskManager:
    def __init__(self, drone_controller, node):
        self.node = node
        self.drone = drone_controller
        self.state = "ARM"
        self.hold_start_time = None
        
        self.takeoff_altitude = self.node.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.hold_time = self.node.get_parameter('hold_time').get_parameter_value().double_value
        
    def execute_mission_step(self):
        if self.state == "ARM":
            self.arm_drone()
        elif self.state == "TAKEOFF":
            self.takeoff_drone()
        elif self.state == "HOLD":
            self.hold_drone(hold_drone_time=self.hold_time)
        elif self.state == "LAND":
            self.land_drone()
        elif self.state == "DISARM":
            self.disarm_drone()

    def arm_drone(self):
        self.drone.set_mode('OFFBOARD')
        self.drone.arm()
        self.state = "TAKEOFF"

    def takeoff_drone(self):
        takeoff = self.drone.takeoff(self.takeoff_altitude)
        if takeoff:
            self.hold_start_time = time.time()
            self.state = "HOLD"

    def hold_drone(self, hold_drone_time):
        self.drone.hold(0.0, 0.0, self.takeoff_altitude)
        elapsed_time = time.time() - self.hold_start_time
        self.drone.get_logger().info(f"Exiting hold position in {(hold_drone_time - elapsed_time):.0f}")
        if elapsed_time >= hold_drone_time:
            self.state = "LAND"

    def land_drone(self):
        landed = self.drone.land()
        if landed:
            self.state = "DISARM"

    def disarm_drone(self):
        self.drone.flight_termination()
        self.drone.get_logger().info("Mission complete. Drone disarmed.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('mission_task_manager')

    # Declare and set default parameters
    node.declare_parameter("takeoff_altitude", -3.0)
    node.declare_parameter("hold_time", 5.0)

    drone = DroneController()

    mission = MissionTaskManager(drone, node)

    node.destroy_node()

    while rclpy.ok():
        rclpy.spin_once(drone)
        mission.execute_mission_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()