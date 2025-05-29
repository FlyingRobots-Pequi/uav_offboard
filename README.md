## `uav_offboard`

Pacote ROS 2 para controle e gerenciamento de voo de drones PX4 via serviços, incluindo teleop por teclado e gerenciamento offboard.

---

### Estrutura

```bash
uav_offboard/
├── uav_offboard/
│   ├── flight_manager_node.py         # Gerenciador principal (serviços e heartbeat)
│   ├── vehicle_callback.py            # Subscrição de estados da aeronave
│   ├── vehicle_commander.py           # Implementa comandos via serviços
│   └── offboard_controller.py         # Controle de setpoints (posição/velocidade)
├── scripts/
│   └── uav_teleop_keyboard.py         # Interface de teclado estilo teleop_twist
├── config/
│   └── (opcional: arquivos YAML)
├── resource/
├── setup.py
├── setup.cfg
├── package.xml
```

---

### Build do Workspace

1. Clone ou coloque o pacote em `ros2_ws/src/`:

```bash
cd ~/ros2_ws/src
git clone <repositório>  # ou mova os arquivos manualmente
```

> Esse pacote depende do `uav_interfaces` para serviços customizados, certifique-se de que ele esteja presente no workspace.

2. Compile:

```bash
cd ~/ros2_ws
colcon build --packages-select uav_offboard
```

3. Fonte o ambiente:

```bash
source install/setup.bash
```

---

### Execução dos Nós

#### Gerenciador de Voo:

Inicia os serviços e o heartbeat necessário para manter o modo offboard ativo.

```bash
ros2 run uav_offboard flight_manager_node
```

---

#### Teleop por Teclado:

Interface por teclado para controle de velocidade (`vx`, `vy`, `vz`) e comandos de voo.

```bash
ros2 run uav_offboard uav_teleop_keyboard
```

##### Teclas disponíveis:

* `o` → Setar modo **offboard**
* `a` → **Arm**
* `d` → **Disarm**
* `t` → **Takeoff** (subida vertical 1m)
* `l` → **Land**
* `k` → **Kill** (emergência)
* `↑ ↓ → ←` → Controle de `vx`, `vy`
* `w/x` → Controle de `vz`
* `s` → Zera velocidades

> As chamadas são feitas via serviços: `/vehicle_commander` e `/setpoint_controller`.

---

### Dependências

* `uav_interfaces` (serviços customizados)
* Drone PX4 com ROS 2 bridge publicando:

  * `/fmu/out/vehicle_odometry`, `/vehicle_status`, etc.
* ROS 2 Humble

---

###  Observações

* `uav_teleop_keyboard` exige terminal compatível com `curses`.
* Para executar com teclado, rode em terminal **não multiplexado** (não funciona bem via IDEs).
* Requisitos mínimos do PX4:

  * Modo offboard habilitado
  * Arming via comando permitido
