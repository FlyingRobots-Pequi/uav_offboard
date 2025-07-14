# Sistema de Navegação Autônoma em Tempo Real

Este documento explica como usar o novo sistema de navegação autônoma baseado em tópicos para o UAV.

## Visão Geral

O sistema foi evoluído de um modelo baseado em serviços (síncrono) para um modelo baseado em tópicos (assíncrono) que permite navegação autônoma em tempo real.

### Componentes Principais

1. **MissionState.msg** - Mensagem personalizada que contém todas as informações da missão
2. **MissionStateManager** - Gerenciador de estado da missão
3. **FlightManagerNode** - Nó principal com navegação autônoma integrada
4. **Scripts de exemplo e monitoramento**

## Arquitetura do Sistema

```
┌─────────────────────┐    /uav/mission_command    ┌──────────────────────┐
│  Mission Control    │────────────────────────────▶│  FlightManagerNode   │
│  (External Apps)    │                             │                      │
└─────────────────────┘                             │  + MissionStateManager │
                                                     │  + VehicleCommander    │
┌─────────────────────┐    /uav/mission_state      │  + OffboardController  │
│  Mission Monitor    │◀────────────────────────────│  + VehicleCallback     │
│  (Real-time info)   │                             └──────────────────────┘
└─────────────────────┘                                        │
                                                                ▼
                                                     ┌──────────────────────┐
                                                     │      PX4 Autopilot   │
                                                     └──────────────────────┘
```

## Tópicos ROS2

### `/uav/mission_state` (Publisher)
- **Tipo:** `uav_interfaces/MissionState`
- **Frequência:** 10Hz
- **Descrição:** Publica o estado atual da missão em tempo real

### `/uav/mission_command` (Subscriber)
- **Tipo:** `uav_interfaces/MissionState`
- **Descrição:** Recebe comandos externos para controlar a missão

## Campos da Mensagem MissionState

```python
# Estado da missão
string mission_state        # ARM, TAKEOFF, NAVIGATE, HOLD, LAND, DISARM, EMERGENCY
uint8 mission_id           # ID da missão atual
uint8 waypoint_index       # Índice do waypoint atual
uint8 total_waypoints      # Total de waypoints na missão

# Posições
geometry_msgs/Point target_position      # Posição alvo (x, y, z)
geometry_msgs/Point current_position     # Posição atual (x, y, z)
geometry_msgs/Vector3 target_velocity    # Velocidade alvo (vx, vy, vz)
geometry_msgs/Vector3 current_velocity   # Velocidade atual (vx, vy, vz)
float32 target_heading                   # Heading alvo (rad)
float32 current_heading                  # Heading atual (rad)

# Status de voo
string flight_mode          # OFFBOARD, MANUAL, ALTITUDE, etc.
bool armed                  # Estado de armamento
bool offboard_enabled       # Modo offboard ativo

# Tolerâncias
float32 position_tolerance  # Tolerância de posição (metros)
float32 heading_tolerance   # Tolerância de heading (radianos)
float32 velocity_tolerance  # Tolerância de velocidade (m/s)

# Flags de controle
bool hold_position         # Manter posição atual
bool emergency_stop        # Parada de emergência
bool mission_complete      # Missão concluída

# Informações adicionais
float32 battery_level      # Nível da bateria (0.0 - 1.0)
float32 mission_progress   # Progresso da missão (0.0 - 1.0)
string status_message      # Mensagem de status
```

## Como Usar

### 1. Compilar o Sistema

```bash
cd ~/ros2_ws
colcon build --packages-select uav_interfaces uav_offboard
source install/setup.bash
```

### 2. Executar o Flight Manager

```bash
ros2 run uav_offboard flight_manager_node
```

### 3. Monitorar o Estado da Missão

Em outro terminal:
```bash
ros2 run uav_offboard mission_monitor
```

### 4. Executar Exemplo de Missão Autônoma

Em outro terminal:
```bash
ros2 run uav_offboard autonomous_mission_example
```

## Estados da Missão

- **IDLE** - Sistema inativo
- **ARM** - Armando o drone
- **TAKEOFF** - Decolando
- **NAVIGATE** - Navegando para waypoints
- **HOLD** - Mantendo posição
- **LAND** - Pousando
- **DISARM** - Desarmando
- **EMERGENCY** - Parada de emergência
- **MISSION_COMPLETE** - Missão concluída

## Comandos de Missão

Você pode enviar comandos via tópico `/uav/mission_command`:

```python
# Iniciar missão
mission_cmd = MissionState()
mission_cmd.mission_state = "START_MISSION"
mission_cmd.mission_id = 1

# Pausar missão
mission_cmd.mission_state = "PAUSE_MISSION"

# Retomar missão
mission_cmd.mission_state = "RESUME_MISSION"

# Abortar missão
mission_cmd.mission_state = "ABORT_MISSION"

# Parada de emergência
mission_cmd.mission_state = "EMERGENCY_STOP"
```

## Carregando Waypoints

```python
# Carrega waypoints na missão
waypoints = [
    [0.0, 0.0, -2.0],    # Decolagem
    [5.0, 0.0, -2.0],    # Primeiro waypoint
    [5.0, 5.0, -2.0],    # Segundo waypoint
    [0.0, 5.0, -2.0],    # Terceiro waypoint
    [0.0, 0.0, -2.0],    # Retornar ao início
]

# Via código Python
flight_manager.load_mission_waypoints(waypoints)
flight_manager.start_autonomous_mission(mission_id=1)
```

## Monitoramento em Tempo Real

O sistema publica informações a 10Hz incluindo:
- Estado atual da missão
- Posição atual vs posição alvo
- Progresso da missão (%)
- Nível de bateria
- Status de voo (armado, offboard, etc.)
- Mensagens de status

## Compatibilidade

- O sistema mantém compatibilidade com o sistema de serviços anterior
- Pode ser usado tanto para teleoperação quanto navegação autônoma
- Integração transparente com PX4 v1.15 e ROS2 Humble

## Exemplo de Uso Completo

```python
import rclpy
from rclpy.node import Node
from uav_interfaces.msg import MissionState

class MyMissionController(Node):
    def __init__(self):
        super().__init__('my_mission_controller')
        
        # Publisher para comandos
        self.cmd_pub = self.create_publisher(MissionState, '/uav/mission_command', 10)
        
        # Subscriber para monitorar estado
        self.state_sub = self.create_subscription(
            MissionState, '/uav/mission_state', self.mission_callback, 10
        )
        
    def mission_callback(self, msg):
        print(f"Mission State: {msg.mission_state}")
        print(f"Progress: {msg.mission_progress:.1%}")
        
    def start_mission(self):
        cmd = MissionState()
        cmd.mission_state = "START_MISSION"
        cmd.mission_id = 1
        self.cmd_pub.publish(cmd)
```

## Troubleshooting

### Problemas Comuns

1. **Mensagem não encontrada**: Verifique se `uav_interfaces` foi compilado corretamente
2. **Tópico não disponível**: Verifique se o `flight_manager_node` está rodando
3. **Drone não responde**: Verifique conexão PX4 e modo offboard

### Logs

Use `ros2 topic echo /uav/mission_state` para monitorar o estado em tempo real.

### Debug

```bash
# Listar tópicos
ros2 topic list | grep uav

# Verificar mensagens
ros2 interface show uav_interfaces/msg/MissionState

# Monitorar tópico
ros2 topic echo /uav/mission_state
``` 