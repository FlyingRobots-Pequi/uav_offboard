import math
import time

class NavigationManager:
    def __init__(self, vehicle_callback, offboard_controller, config):
        self.vehicle_callback = vehicle_callback
        self.offboard_controller = offboard_controller
        self.config = config
        
        # Navigation state
        self.active_command = None
        self.target_position = None
        self.command_start_time = None
        self.tolerance = config.position_tolerance
        self.command_completed = False
        
        # Gradual altitude change configuration
        self.altitude_threshold = config.altitude_threshold  # metros - limiar para mudança gradual
        self.gradual_navigation_active = False
        
    def takeoff(self, altitude: float):
        """Takeoff para altitude especificada"""
        current_pos = self.vehicle_callback.current_position
        
        # Target position: mesma x,y, mas altitude negativa (NED)
        target_z = -abs(altitude)  # NED: negativo para cima
        
        self.target_position = (current_pos[0], current_pos[1], target_z)
        self.active_command = "TAKEOFF"
        self.command_start_time = time.time()
        self.command_completed = False
        self.gradual_navigation_active = False
        
        # Publica setpoint
        self.offboard_controller.publish_position_control_setpoint(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2]
        )
        
    def goto(self, x: float, y: float, z: float):
        """Navega para posição especificada com controle gradual de altitude"""
        # Converte altitude para NED se necessário
        if z > 0:
            z = -z  # NED: negativo para cima
            
        current_pos = self.vehicle_callback.current_position
        altitude_difference = abs(current_pos[2] - z)
        
        self.target_position = (x, y, z)
        self.active_command = "GOTO"
        self.command_start_time = time.time()
        self.command_completed = False
        
        # Verifica se precisa de controle gradual de altitude
        if altitude_difference > self.altitude_threshold:
            print(f"Diferença de altitude detectada: {altitude_difference:.2f}m > {self.altitude_threshold}m")
            print(f"Iniciando navegação com controle gradual de altitude")
            
            self.gradual_navigation_active = True
            # Não precisamos de fases separadas - apenas controle gradual de Z
        else:
            # Navegação normal - vai direto para o ponto
            self.gradual_navigation_active = False
            
        # Sempre publica o setpoint inicial
        self.offboard_controller.publish_position_control_setpoint(x, y, z)
        
    def hold(self, x: float = None, y: float = None, z: float = None):
        """Mantém posição especificada ou atual"""
        if x is None or y is None or z is None:
            # Usa posição atual
            current_pos = self.vehicle_callback.current_position
            x = x if x is not None else current_pos[0]
            y = y if y is not None else current_pos[1]
            z = z if z is not None else current_pos[2]
            
        self.target_position = (x, y, z)
        self.active_command = "HOLD"
        self.command_start_time = time.time()
        self.command_completed = False
        self.gradual_navigation_active = False
        
        # Publica setpoint
        self.offboard_controller.publish_position_control_setpoint(x, y, z)

    def navigation_control_loop(self):
        """Loop de controle de navegação - chamado a 10Hz"""
        if self.active_command is None or self.target_position is None:
            return
            
        current_pos = self.vehicle_callback.current_position
        
        if self.active_command == "TAKEOFF":
            # Subida gradual para takeoff
            self._handle_gradual_takeoff(current_pos)
            
        elif self.active_command == "GOTO" and self.gradual_navigation_active:
            # Navegação gradual para GOTO com grande diferença de altitude
            self._handle_gradual_navigation(current_pos)
            
        else:
            # Para GOTO normal e HOLD, usa a lógica normal
            self.offboard_controller.publish_position_control_setpoint(
                self.target_position[0],
                self.target_position[1],
                self.target_position[2]
            )
            
            # Verifica se chegou na posição
            if not self.command_completed:
                if self.is_position_reached():
                    if self.active_command == "HOLD":
                        # Para HOLD, aguarda o tempo configurado na posição
                        elapsed = time.time() - self.command_start_time
                        if elapsed >= self.config.hold_duration:
                            self.command_completed = True
                    else:
                        # Para GOTO, completa imediatamente
                        self.command_completed = True
                        
    def _handle_gradual_navigation(self, current_pos):
        """Maneja navegação com controle gradual de altitude"""
        target_altitude = self.target_position[2]
        current_altitude = current_pos[2]
        
        # Taxa de mudança de altitude (baseada na configuração de takeoff)
        altitude_rate = self.config.takeoff_climb_rate  # Assumindo 10Hz
        
        # Calcula próxima altitude com controle gradual
        if current_altitude > target_altitude:  # Precisa subir (NED: mais negativo)
            next_altitude = current_altitude - altitude_rate
            if next_altitude < target_altitude:
                next_altitude = target_altitude
        else:  # Precisa descer (NED: menos negativo)
            next_altitude = current_altitude + altitude_rate
            if next_altitude > target_altitude:
                next_altitude = target_altitude
        
        # Publica setpoint com altitude controlada, mas X,Y direto para o alvo
        self.offboard_controller.publish_position_control_setpoint(
            self.target_position[0],  # X vai direto para o alvo
            self.target_position[1],  # Y vai direto para o alvo
            next_altitude             # Z controlado gradualmente
        )
        
        # Verifica se chegou na posição final completa
        if self.is_position_reached():
            print(f"Navegação gradual concluída. Posição final alcançada.")
            self.gradual_navigation_active = False
            self.command_completed = True
        
    def _handle_gradual_takeoff(self, current_pos):
        """Maneja subida gradual durante takeoff"""
        target_altitude = self.target_position[2]  # Altitude alvo (negativa no NED)
        current_altitude = current_pos[2]
        
        # Taxa de subida baseada na configuração
        # Negativo porque NED (para cima), dividido pela frequência do loop
        climb_rate = -self.config.takeoff_climb_rate  # Assumindo 10Hz
        
        # Calcula próxima altitude
        if current_altitude > target_altitude:  # Ainda precisa subir (NED: valores mais negativos = mais alto)
            next_altitude = current_altitude + climb_rate
            # Não passa da altitude alvo
            if next_altitude < target_altitude:
                next_altitude = target_altitude
        else:
            next_altitude = target_altitude
        
        # Publica setpoint com nova altitude
        self.offboard_controller.publish_position_control_setpoint(
            self.target_position[0],  # x fixo
            self.target_position[1],  # y fixo
            next_altitude
        )
        
        # Verifica se chegou na altitude alvo
        if not self.command_completed and abs(current_altitude - target_altitude) < self.tolerance:
            self.command_completed = True
            print(f"Takeoff completed - reached altitude: {current_altitude:.2f}m")

    def is_position_reached(self) -> bool:
        """Verifica se a posição alvo foi alcançada"""
        if self.target_position is None:
            return False
            
        current_pos = self.vehicle_callback.current_position
        
        distance = math.sqrt(
            (current_pos[0] - self.target_position[0])**2 +
            (current_pos[1] - self.target_position[1])**2 +
            (current_pos[2] - self.target_position[2])**2
        )
        
        return distance < self.tolerance
        
    def is_command_completed(self) -> bool:
        """Verifica se o comando atual foi completado"""
        return self.command_completed
        
    def stop_navigation(self):
        """Para navegação atual"""
        self.active_command = None
        self.target_position = None
        self.command_start_time = None
        self.command_completed = False
        self.gradual_navigation_active = False
        