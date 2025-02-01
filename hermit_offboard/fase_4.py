#!/usr/bin/python3

import time
import yaml
import rclpy
from silver_control.silver_controller import SilverController
import os
from ament_index_python.packages import get_package_share_directory

class MissionTaskManager:
    def __init__(self, silver_controller):
        self.silver = silver_controller
        self.state = "ARM"
        self.takeoff_altitude = -1.5
        self.vel_x = 0.1
        self.vel_y = 0.1
        self.vel_z = 0.1
        self.landing_vel = 0.3
        self.vel_yaw = 0.1
        
    def execute_mission_step(self):
        
        # State machine for mission steps
        if self.state == "ARM":
            self.drone.arm()
            self.drone.engage_offBoard_mode()
            self.state = "TAKEOFF"
        
        if self.state == "TAKEOFF":
            self.drone.takeoff(0.0, 0.0, self.takeoff_altitude)
            if self.drone.takeoff_success:
                self.state = "HOLD"
        
        if self.state == "HOLD":
            self.drone.hold(self.drone.current_x, self.drone.current_y, self.drone.current_z)
            
        if self.state == "RIGHT":
            self.drone.right(self.vel_y)
        
        if self.state == "LEFT":
            self.drone.left(self.vel_y)
        
        if self.state == "FORWARD":
            self.drone.forward(self.vel_x)
            
        if self.state == "BACKWARD":
            self.drone.backward(self.vel_x)
        
        if self.state == "UP":
            self.drone.up(self.vel_z)
        
        if self.state == "DOWN":
            self.drone.down(self.vel_z)
            
        if self.state == "CLOCKWISE":
            self.drone.clockwise(self.vel_yaw)
            
        if self.state == "COUNTER_CLOCKWISE":
            self.drone.counter_clockwise(self.vel_yaw)
            
        if self.state == "LAND":
            self.drone.land(self.landing_vel)
            
        if self.state == "DISARM":
            self.drone.disarm()


def main(args=None):
    rclpy.init(args=args)
    drone = SilverController()
    
    node = rclpy.create_node('silver_controller')
    
    mission = MissionTaskManager(drone)
    
    while rclpy.ok():    
        rclpy.spin_once(drone)
        mission.execute_mission_step()
    
    drone.destroy_node()    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()