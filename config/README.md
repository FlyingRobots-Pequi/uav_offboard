# UAV Offboard Configuration

Este diretório contém arquivos de configuração para o pacote `uav_offboard`, permitindo parametrizar o comportamento do sistema de controle UAV.

## Arquivo de Configuração

### `flight_params.yaml`

Arquivo principal de configuração contendo todos os parâmetros ajustáveis do sistema.

## Categorias de Parâmetros

### 🛩️ Navigation (Navegação)
- **`position_tolerance`**: Tolerância para considerar que chegou na posição alvo (metros)
- **`takeoff_climb_rate`**: Velocidade de subida durante takeoff (m/s)
- **`hold_duration`**: Tempo para manter posição no comando HOLD (segundos)
- **`default_takeoff_altitude`**: Altitude padrão de takeoff (metros)

### 🎮 Control (Controle)
- **`setpoint_mode`**: Modo de setpoint ("position" ou "velocity")
- **`heartbeat_frequency`**: Frequência do heartbeat offboard (Hz)
- **`navigation_frequency`**: Frequência do loop de navegação (Hz)
- **`status_frequency`**: Frequência de monitoramento de status (Hz)

### 📡 Callbacks (Callbacks)
- **`position_threshold`**: Limiar para detectar mudança de posição (metros)
- **`velocity_threshold`**: Limiar para detectar mudança de velocidade (m/s)

### 🎯 Mission (Missão)
- **`command_delay`**: Delay entre comandos de missão (segundos)
- **`command_timeout`**: Timeout para comandos (segundos)

### 🛡️ Safety (Segurança)
- **`max_altitude`**: Altitude máxima permitida (metros)
- **`max_velocity`**: Velocidade máxima permitida (m/s)
- **`emergency_land_battery`**: Nível de bateria para pouso de emergência (0-1)

### 📝 Logging (Logs)
- **`debug_navigation`**: Log detalhado de navegação
- **`debug_callbacks`**: Log detalhado de callbacks
- **`debug_mission`**: Log detalhado de missão

## Como Usar

### 1. Lançamento com Arquivo YAML
```bash
ros2 launch uav_offboard flight_manager_with_params.launch.py
```

### 2. Lançamento com Parâmetros Customizados
```bash
ros2 run uav_offboard flight_manager_node --ros-args --params-file /path/to/custom_params.yaml
```

### 3. Override de Parâmetros Específicos
```bash
ros2 run uav_offboard flight_manager_node --ros-args \
  -p navigation.position_tolerance:=0.15 \
  -p navigation.takeoff_climb_rate:=1.0 \
  -p control.setpoint_mode:=position
```

### 4. Verificar Parâmetros Ativos
```bash
ros2 param list /flight_manager_node
ros2 param get /flight_manager_node navigation.position_tolerance
```

### 5. Modificar Parâmetros Durante Execução
```bash
ros2 param set /flight_manager_node navigation.position_tolerance 0.25
```

## Exemplos de Configuração

### Configuração Conservadora (Lenta e Segura)
```yaml
navigation:
  position_tolerance: 0.1
  takeoff_climb_rate: 0.3
  hold_duration: 5.0

control:
  setpoint_mode: "position"
  navigation_frequency: 20.0

safety:
  max_altitude: 5.0
  max_velocity: 2.0
```

### Configuração Agressiva (Rápida)
```yaml
navigation:
  position_tolerance: 0.3
  takeoff_climb_rate: 1.0
  hold_duration: 1.0

control:
  setpoint_mode: "velocity"
  navigation_frequency: 30.0

safety:
  max_altitude: 15.0
  max_velocity: 8.0
```

### Configuração de Debug
```yaml
logging:
  debug_navigation: true
  debug_callbacks: true
  debug_mission: true

control:
  status_frequency: 10.0  # Mais status updates
```

## Validação de Parâmetros

O sistema automaticamente valida os parâmetros:
- Valores numéricos devem ser positivos onde apropriado
- `setpoint_mode` deve ser "position" ou "velocity"
- `emergency_land_battery` deve estar entre 0 e 1
- Frequências devem ser > 0

Parâmetros inválidos resultarão em log de erro e uso dos valores padrão.

## Estrutura do Arquivo YAML

```yaml
flight_manager_node:
  ros__parameters:
    categoria:
      parametro: valor
```

⚠️ **Importante**: Mantenha a estrutura hierárquica do YAML para que os parâmetros sejam carregados corretamente. 