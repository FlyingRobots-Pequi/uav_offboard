import rclpy
from rclpy.node import Node
from uav_interfaces.srv import VehicleCommander, SetpointControl
import curses


class UAVTeleopKeyboard(Node):
    def __init__(self, stdscr):
        super().__init__('uav_teleop_keyboard')
        self.cmd_cli = self.create_client(VehicleCommander, 'vehicle_commander')
        self.setpoint_cli = self.create_client(SetpointControl, 'setpoint_controller')

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0

        self.stdscr = stdscr
        stdscr.nodelay(True)
        curses.cbreak()
        curses.noecho()

        stdscr.addstr(0, 0, "UAV Teleop Keyboard - Commands:")
        stdscr.addstr(1, 0, "o: offboard | t: takeoff | a: arm | d: disarm | l: land | k: kill")
        stdscr.addstr(2, 0, "↑: +vx | ↓: -vx | →: +vy | ←: -vy | w: -vz | x: +vz")
        stdscr.addstr(3, 0, "s: stop all velocity")
        stdscr.addstr(5, 0, "Waiting for service servers...")

        self.wait_for_services()
        self.control_loop()

    def wait_for_services(self):
        self.cmd_cli.wait_for_service()
        self.setpoint_cli.wait_for_service()
        self.stdscr.addstr(5, 0, "Service servers ready. Press keys...          ")

    def send_vehicle_command(self, command, **kwargs):
        req = VehicleCommander.Request()
        req.command = command
        req.mode = kwargs.get('mode', "")
        req.param1 = float(kwargs.get('param1', 0.0))
        future = self.cmd_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def send_velocity_setpoint(self):
        req = SetpointControl.Request()
        req.type = 'vel'
        req.vx = self.vx
        req.vy = self.vy
        req.vz = self.vz
        req.yaw = self.yaw
        future = self.setpoint_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def control_loop(self):
        while rclpy.ok():
            key = self.stdscr.getch()
            updated = False

            if key == ord('o'):
                self.send_vehicle_command('set_mode', mode='offboard')
            elif key == ord('t'):
                self.send_vehicle_command('takeoff', param1=1.0)
            elif key == ord('k'):
                self.send_vehicle_command('flight_termination')
            elif key == ord('a'):
                self.send_vehicle_command('arm')
            elif key == ord('d'):
                self.send_vehicle_command('disarm')
            elif key == ord('l'):
                self.send_vehicle_command('land')
            elif key == curses.KEY_UP:
                self.vx += 0.2
                updated = True
            elif key == curses.KEY_DOWN:
                self.vx -= 0.2
                updated = True
            elif key == curses.KEY_RIGHT:
                self.vy += 0.2
                updated = True
            elif key == curses.KEY_LEFT:
                self.vy -= 0.2
                updated = True
            elif key == ord('w'):
                self.vz -= 0.2
                updated = True
            elif key == ord('x'):
                self.vz += 0.2
                updated = True
            elif key == ord('s'):
                self.vx = self.vy = self.vz = 0.0
                updated = True

            if updated:
                self.send_velocity_setpoint()
                self.stdscr.addstr(6, 0, f"Setpoint -> vx: {self.vx:.1f}, vy: {self.vy:.1f}, vz: {self.vz:.1f}     ")

def main():
    curses.wrapper(run_teleop)

def run_teleop(stdscr):
    rclpy.init()
    try:
        UAVTeleopKeyboard(stdscr)
    finally:
        rclpy.shutdown()
