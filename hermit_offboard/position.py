import time
import rclpy
from hermit_offboard.drone_controller import DroneController

class FlightTest:
    def __init__(self, drone_controller):
        self.drone = drone_controller
        self.state = "INIT"
        self.step_index = 0
        self.S = 3  # Distância fornecida no documento
        self.start_time = None
        self.waiting = False
        self.wait_time = 2  # Tempo de espera entre cada passos
        self.steps = [
            (self.launch_hover, "Launch and Hover"),
            (self.yaw_left_360, "Yaw Left 360 over Stand #1"),
            (self.yaw_right_360, "Yaw Right 360 over Stand #1"),
            (self.climb_vertically, "Climb Vertically over Stand #1"),
            (self.descend_vertically, "Descend Vertically over Stand #1"),
            (self.pitch_forward_1, "Pitch Forward over Stand #2"),
            (self.pitch_backward_1, "Pitch Backward over Stand #1"),
            (self.pitch_forward_2, "Pitch Forward over Stand #2 then Yaw Left 180"),
            (self.pitch_forward_3, "Pitch Forward over Landing then Yaw Right 180"),
            (self.land, "Land in Circle"),
        ]

    def execute_step(self):
        if self.step_index < len(self.steps):
            if not self.waiting:
                action, description = self.steps[self.step_index]
                print(f"Executing step {self.step_index + 1}: {description}")
                action()
                self.start_time = time.time()
                self.waiting = True
            elif time.time() - self.start_time >= self.wait_time:
                self.waiting = False
                self.step_index += 1
        else:
            print("Flight test completed.")
            self.state = "COMPLETE"

    def launch_hover(self):
        self.drone.arm()
        self.drone.engage_offBoard_mode()
        self.drone.publish_takeoff_setpoint(0.0, 0.0, -self.S)

    def yaw_left_360(self):
        # self.drone.yaw(-360)
        pass

    def yaw_right_360(self):
        # self.drone.yaw(360)
        pass

    def climb_vertically(self):
        self.drone.goto_setpoint(self.S, 0.0, 2 * -self.S)

    def descend_vertically(self):
        self.drone.goto_setpoint(0.0, 0.0, -self.S)

    def pitch_forward_1(self):
        self.drone.goto_setpoint(2 * self.S, 0.0, -self.S)

    def pitch_backward_1(self):
        self.drone.goto_setpoint(self.S, 0.0, -self.S)

    def pitch_forward_2(self):
        self.drone.goto_setpoint(2 * self.S, 0.0, -self.S)
        # self.drone.yaw(-180)

    def pitch_forward_3(self):
        self.drone.goto_setpoint(0.0, 0.0, -self.S)
        # self.drone.yaw(180)

    def land(self):
        self.drone.publish_landing_setpoint(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    drone = DroneController()
    flight_test = FlightTest(drone)

    while rclpy.ok() and flight_test.state != "COMPLETE":
        rclpy.spin_once(drone)
        flight_test.execute_step()

    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
