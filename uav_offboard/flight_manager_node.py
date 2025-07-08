import rclpy
from rclpy.node import Node
import inspect
import time

from uav_offboard.vehicle_commander import VehicleCommander
from uav_offboard.vehicle_callback import VehicleCallback
from uav_offboard.offboard_controller import OffboardController
from uav_offboard.vehicle_state import VehicleState, ArmingState, NavState
from uav_offboard.navigation_manager import NavigationManager
from uav_offboard.config_manager import ConfigManager

from uav_interfaces.srv import VehicleCommander as VehicleCommanderService
from uav_interfaces.srv import SetpointControl
from uav_interfaces.msg import MissionState, MissionCommand, UavStatus

class FlightManagerNode(Node):

    """
    FlightManagerNode class to manage vehicle commands, offboard and callbacks.
    Includes basic navigation functions: takeoff, land, hold_position, emergency_stop.
    """

    def __init__(self):
        super().__init__('flight_manager_node')
        self.get_logger().info('FlightManagerNode initialized')
        
        # Declare and get FMU namespace parameter
        self.declare_parameter('uav_namespace', '')
        self.uav_namespace = self.get_parameter('uav_namespace').value
        
        if self.uav_namespace:
            self.get_logger().info(f'Using UAV namespace: {self.uav_namespace}')
        else:
            self.get_logger().info('Using default UAV namespace (no prefix)')

        # Load configuration
        self.config_manager = ConfigManager(self)
        self.config = self.config_manager.get_config()
        
        # Validate configuration
        if not self.config_manager.validate_config():
            self.get_logger().error("Invalid configuration detected. Using defaults.")
        
        self.vehicle_commander = VehicleCommander(self, self.uav_namespace)
        self.vehicle_callback = VehicleCallback(self, self.config.callbacks, self.uav_namespace)
        self.offboard_controller = OffboardController(self, self.uav_namespace)
        self.navigation_manager = NavigationManager(self.vehicle_callback, self.offboard_controller, self.config.navigation)

        self.vehicle_commander_srv = self.create_service(VehicleCommanderService, 'vehicle_commander', self.vehicle_commander_service_callback)
        self.setpoint_controller_srv = self.create_service(SetpointControl, 'setpoint_controller', self.setpoint_controller_service_callback)
        
        # Navigation state (para status display)
        self.navigation_active = False
        self.navigation_command = None
        self.navigation_start_time = None
        
        # Heartbeat timer (must never stop)
        heartbeat_period = 1.0 / self.config.control.heartbeat_frequency
        self.create_timer(heartbeat_period, self.publish_heartbeat)

        # Navigation control loop timer
        navigation_period = 1.0 / self.config.control.navigation_frequency
        self.create_timer(navigation_period, self.navigation_manager.navigation_control_loop)

        self.setpoint_mode = self.config.control.setpoint_mode

        # Variáveis de controle da missão
        self.current_mission_command = None
        self.mission_status = "IDLE"
        self.mission_start_time = None

        # UAV Status publisher - publishes vehicle state information
        uav_status_topic = self._build_uav_topic('/uav_status')
        self.uav_status_pub = self.create_publisher(
            UavStatus,
            uav_status_topic,
            10
        )
        
        # Subscriber para comandos de missão
        mission_cmd_topic = self._build_uav_topic('/mission_cmd')
        self.mission_cmd_sub = self.create_subscription(
            MissionCommand,
            mission_cmd_topic,
            self.mission_command_callback,
            10
        )

        # Publisher para status da missão
        mission_state_topic = self._build_uav_topic('/mission_state')
        self.mission_status_pub = self.create_publisher(
            MissionState,
            mission_state_topic,
            10
        )
        
        # Setup vehicle callback event handlers
        self._setup_vehicle_callbacks()
        
        # Timer to publish UAV status
        status_period = 1.0 / self.config.control.status_frequency
        self.create_timer(status_period, self.publish_uav_status)
        
        # Timer to monitor mission status
        self.create_timer(status_period, self.monitor_mission_status)
        
        self.get_logger().info('FlightManagerNode setup complete with integrated navigation functions')

    def _build_uav_topic(self, topic):
        """Build complete topic name with namespace prefix."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{topic}"
        return topic

    def _setup_vehicle_callbacks(self):
        """Setup event callbacks for vehicle state changes"""
        
        # State change callback - logs important state transitions
        self.vehicle_callback.add_state_change_callback(self._on_vehicle_state_change)
        
        # Position change callback - for mission monitoring
        self.vehicle_callback.add_position_change_callback(self._on_position_change)
        
        # Critical alert callback - for safety monitoring
        self.vehicle_callback.add_critical_alert_callback(self._on_critical_alert)
    
    def _on_vehicle_state_change(self, state: VehicleState):
        """Callback for vehicle state changes"""
        self.get_logger().info(f"Vehicle state changed: {state.status_summary}")
        
        # Publish updated UAV status immediately on state change
        self.publish_uav_status()
        
        # Log specific state transitions
        if state.arming_state == ArmingState.ARMED:
            self.get_logger().info("Vehicle ARMED - Ready for operations")
        elif state.arming_state == ArmingState.DISARMED:
            self.get_logger().info("Vehicle DISARMED")
            
        if state.nav_state == NavState.OFFBOARD:
            self.get_logger().info("OFFBOARD mode active - Accepting setpoints")
        
        if state.takeoff_state == TakeoffState.FLIGHT:
            self.get_logger().info("Vehicle in FLIGHT state")
        
    def _on_position_change(self, position: tuple):
        """Callback for significant position changes"""
        x, y, z = position
        self.get_logger().debug(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def _on_critical_alert(self, message: str):
        """Callback for critical alerts"""
        # Publish immediate UAV status update on critical alerts
        self.publish_uav_status()
    
    def publish_uav_status(self):
        """Publish current UAV status to /uav_status topic"""
        try:
            # Get current vehicle state snapshot
            state = self.vehicle_callback.get_state_snapshot()
            
            # Create UAV status message
            msg = UavStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            
            # Main vehicle states
            msg.arming_state = int(state.arming_state)
            msg.nav_state = int(state.nav_state)
            msg.takeoff_state = int(state.takeoff_state)
            
            # Critical status flags
            msg.armed = state.armed
            msg.landed = state.landed
            msg.failsafe = state.failsafe
            msg.offboard_enabled = state.offboard_enabled
            
            # Position (NED frame)
            msg.x = state.x
            msg.y = state.y
            msg.z = state.z
            msg.heading = state.heading
            
            # Velocity (NED frame)
            msg.vx = state.vx
            msg.vy = state.vy
            msg.vz = state.vz
            
            # Battery information
            msg.battery_voltage = state.battery_voltage
            msg.battery_current = state.battery_current
            msg.battery_remaining = state.battery_remaining
            msg.battery_warning = int(state.battery_warning)
            
            # Home position
            msg.home_x = state.home_x
            msg.home_y = state.home_y
            msg.home_z = state.home_z
            msg.home_yaw = state.home_yaw
            
            # Computed status checks
            msg.is_ready_for_offboard = state.is_ready_for_offboard
            msg.is_flying = state.is_flying
            msg.allows_offboard_control = state.nav_state.allows_offboard_control
            
            # Human-readable text fields
            msg.arming_state_text = str(state.arming_state)
            msg.nav_state_text = str(state.nav_state)
            msg.takeoff_state_text = str(state.takeoff_state)
            msg.status_summary = state.status_summary
            
            # Add navigation status to status message
            if self.navigation_active:
                nav_info = f"Navigation: {self.navigation_command} (elapsed: {time.time() - self.navigation_start_time:.1f}s)"
                msg.status_message = f"{state.status_message} | {nav_info}"
            else:
                msg.status_message = state.status_message
            
            # Publish the message
            self.uav_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing UAV status: {e}")

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

    def mission_command_callback(self, msg: MissionCommand):
        """Callback para processar comandos de missão"""
        
        command = msg.command.upper()
        
        self.get_logger().info(f"Recebido comando: {command}")
        
        # Armazena o comando atual
        self.current_mission_command = msg
        self.mission_status = "ONGOING"
        self.mission_start_time = time.time()
        
        # Publica status inicial
        self.publish_mission_status("ONGOING", f"Executando comando {command}")
        
        # Interpreta e executa o comando
        success = self.execute_mission_command(command, msg)
        
        if not success:
            self.mission_status = "FAILED"
            self.publish_mission_status("FAILED", f"Falha ao executar {command}")
            

    def execute_mission_command(self, command: str, msg: MissionCommand) -> bool:
        """Executa um comando de missão específico"""
        
        try:
            if command == "OFFBOARD":
                self.vehicle_commander.set_mode("offboard")
                return True
                
            elif command == "ARM":
                self.vehicle_commander.arm()
                return True
                
            elif command == "TAKEOFF":
                # Usa a altitude do target_pose ou padrão do config
                alt = msg.target_pose.pose.position.z if msg.target_pose.pose.position.z != 0 else self.config.navigation.default_takeoff_altitude
                self.navigation_manager.takeoff(alt)
                # Comando contínuo - não retorna SUCCESS imediatamente
                return True
                
            elif command == "GOTO":
                # Navega para posição especificada
                x = msg.target_pose.pose.position.x
                y = msg.target_pose.pose.position.y
                z = msg.target_pose.pose.position.z
                self.navigation_manager.goto(x, y, z)
                # Comando contínuo - não retorna SUCCESS imediatamente
                return True
                
            elif command == "HOLD":
               
                x, y, z = self.vehicle_callback.current_position

                self.navigation_manager.hold(x, y, z)
                # Comando contínuo - não retorna SUCCESS imediatamente
                return True
                
            elif command == "LAND":
                # Para o navigation_manager para não interferir com o pouso
                self.navigation_manager.stop_navigation()
                # Usa comando LAND nativo do PX4
                self.vehicle_commander.land()
                return True
                
            elif command == "DISARM":
                self.vehicle_commander.disarm()
                return True
                
            else:
                self.get_logger().error(f"Comando desconhecido: {command}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erro ao executar {command}: {str(e)}")
            return False

    def publish_mission_status(self, status: str, info: str = ""):
        """Publica o status atual da missão"""
        
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"        
        msg.status = status
        msg.info = info
        
        # Publica no tópico mission_status
        self.mission_status_pub.publish(msg)

    def monitor_mission_status(self):
        """Monitora o status da missão e publica SUCCESS/FAILED quando apropriado"""
        if self.mission_status != "ONGOING" or self.current_mission_command is None:
            return
            
        command = self.current_mission_command.command.upper()
        vehicle_state = self.vehicle_callback.get_state_snapshot()
        
        # Verifica condições específicas para cada comando
        if command == "OFFBOARD":
            # Comando instantâneo - verifica se mudou para offboard
            if vehicle_state.nav_state == 14:  # OFFBOARD
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Modo OFFBOARD ativado")
                
        elif command == "ARM":
            # Comando instantâneo - verifica se armou
            if vehicle_state.armed:
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Drone armado")
                
        elif command == "TAKEOFF":
            # Comando contínuo - verifica se navigation_manager completou
            if self.navigation_manager.is_command_completed():
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Decolagem completa")
                
        elif command == "GOTO":
            # Comando contínuo - verifica se navigation_manager completou
            if self.navigation_manager.is_command_completed():
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Posição alvo alcançada")
                
        elif command == "HOLD":
            # Comando contínuo - verifica se navigation_manager completou
            if self.navigation_manager.is_command_completed():
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Posição mantida")
                
        elif command == "LAND":
            # Comando contínuo - usa cb_land_detected para atualizar vehicle_state.landed
            if vehicle_state.landed:
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Pouso detectado via land_detected")
                
        elif command == "DISARM":
            # Comando instantâneo - verifica se desarmou
            if not vehicle_state.armed:
                self.mission_status = "SUCCESS"
                self.publish_mission_status("SUCCESS", "Drone desarmado")

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
