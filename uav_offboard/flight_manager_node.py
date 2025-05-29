import rclpy
from rclpy.node import Node
import inspect

from uav_offboard.vehicle_commander import VehicleCommander
from uav_offboard.vehicle_callback import VehicleCallback
from uav_offboard.offboard_controller import OffboardController

from uav_interfaces.srv import VehicleCommander as VehicleCommanderService
from uav_interfaces.srv import SetpointControl

class FlightManagerNode(Node):

    """
    FlightManagerNode class to manage vehicle commands, offboard and callbacks.

    """

    def __init__(self):
        super().__init__('flight_manager_node')
        self.get_logger().info('FlightManagerNode initialized')

        self.vehicle_commander = VehicleCommander(self)
        self.vehicle_callback = VehicleCallback(self)
        self.offboard_controller = OffboardController(self)

        self.vehicle_commander_srv = self.create_service(VehicleCommanderService, 'vehicle_commander', self.vehicle_commander_service_callback)
        self.setpoint_controller_srv =self.create_service(SetpointControl, 'setpoint_controller', self.setpoint_controller_service_callback)

        # Heartbeat at 10Hz (must never stop)
        self.create_timer(0.1, self.publish_heartbeat)

        self.setpoint_mode = "velocity"  # ou "position"

    def publish_heartbeat(self):
        self.offboard_controller.publish_offboard_control_heartbeat_signal(
            position_control=(self.setpoint_mode == "position"),
            velocity_control=(self.setpoint_mode == "velocity")
        )

    def vehicle_commander_service_callback(self, request, response):

        """ Service callback to handle vehicle commands.
        Args:
            request (VehicleCommanderService.Request): The service request.
            response (VehicleCommanderService.Response): The service response.
        """

        command = request.command.lower()

        try:
            method = getattr(self.vehicle_commander, command)
        except AttributeError:
            response.success = False
            response.message = f"Unknown command: {command}"
            return response

        try:
            sig = inspect.signature(method)
            args = []

            # Map fields from request to potential args
            request_args = {
                'mode': request.mode,
                'param1': request.param1,
                'param2': request.param2,
                'param3': request.param3,
                'param4': request.param4,
                'param5': request.param5,
                'param6': request.param6,
                'param7': request.param7,
            }

            for param in sig.parameters.values():
                if param.name in request_args:
                    args.append(request_args[param.name])
                elif param.default is param.empty:
                    raise ValueError(f"Missing required param: {param.name}")
                else:
                    args.append(param.default)

            method(*args)

            response.success = True
            response.message = f"Command '{command}' executed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Execution error: {str(e)}"

        return response

    def setpoint_controller_service_callback(self, request, response):
        """
        Service callback to handle setpoint control requests.

        Args:
            request (SetpointController.Request): The service request containing setpoint type and values.
            response (SetpointController.Response): The service response indicating success or failure.

        This service handles two types of setpoints:
            - 'pos': Position control setpoint (x, y, z, yaw)
            - 'vel': Velocity control setpoint (vx, vy, vz, yaw)
        The service will publish the appropriate setpoint using the OffboardController.
        If the type is not recognized, it will return an error message.

        """

        if request.type == 'pos':  # position control
            self.setpoint_mode = "position"
            position = [request.x, request.y, request.z]
            self.offboard_controller.publish_position_control_setpoint(*position, yaw=request.yaw)
            response.accepted = True
            response.message = f"Position setpoint accepted: {position}"


        elif request.type == 'vel':  # velocity control
            self.setpoint_mode = "velocity"
            velocity = [request.vx, request.vy, request.vz]
            self.offboard_controller.publish_velocity_control_setpoint(*velocity, yaw=request.yaw)
            response.accepted = True
            response.message = f"Velocity setpoint accepted: {velocity}"
    
        else:
            response.accepted = False
            response.message = "Unsupported setpoint type. Use 0 for position or 1 for velocity."

        return response

def main(args=None):
    rclpy.init(args=args)
    manager = FlightManagerNode()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
