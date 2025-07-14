# Navigation Controller Guide 🚁

## Overview

The NavigationController provides safe, high-level navigation commands for UAV operations. This guide explains the **mandatory sequence** for using navigation commands.

## ⚠️ CRITICAL SAFETY REQUIREMENT

**Navigation commands will ONLY work when the vehicle is:**
1. **ARMED** ✅
2. **In OFFBOARD mode** ✅  
3. **Not in failsafe** ✅

Any deviation from this state will result in **immediate command rejection** or **emergency stop**.

## 📋 Required Sequence

### Step 1: ARM the Vehicle
```bash
ros2 service call /vehicle_commander uav_interfaces/srv/VehicleCommander "{command: 'arm'}"
```

### Step 2: Set OFFBOARD Mode
```bash
ros2 service call /vehicle_commander uav_interfaces/srv/VehicleCommander "{command: 'offboard'}"
```

### Step 3: Navigation Commands (Now Accepted!)
```bash
# Takeoff to 5 meters
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'takeoff', altitude: 5.0}"

# Hold position at current location
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'hold_position'}"

# Move to specific position
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'hold_position', x: 10.0, y: 5.0, z: 5.0}"

# Land at current position
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'land'}"
```

## 🛠️ Tools and Scripts

### Automated Preparation Tool
The easiest way to prepare your vehicle:
```bash
# Install and build first
colcon build --packages-select uav_offboard uav_interfaces

# Run preparation tool
ros2 run uav_offboard prepare_vehicle
```

This tool will:
1. Check vehicle connection
2. ARM the vehicle
3. Set OFFBOARD mode  
4. Verify readiness
5. Confirm navigation commands are accepted

### Navigation Example Script
```bash
ros2 run uav_offboard navigation_example
```

### Manual Monitoring
Monitor vehicle status in real-time:
```bash
ros2 topic echo /uav_status
```

## 🚨 Safety Features

### Automatic Rejections
Navigation commands are **automatically rejected** if:
- Vehicle is not armed
- Vehicle is not in OFFBOARD mode
- Vehicle is in failsafe state

### Continuous Monitoring
During navigation execution, the system continuously monitors:
- Vehicle remains ARMED
- Vehicle stays in OFFBOARD mode
- No failsafe conditions occur

### Emergency Actions
If safety conditions are violated during navigation:
1. **Immediate velocity stop** (0, 0, 0)
2. **Emergency state** activation
3. **Action abort** with detailed error

### Cancellation Support
Actions can be cancelled at any time:
```bash
ros2 action cancel_goal /navigation_command
```

## 📊 Available Commands

### Takeoff
```bash
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'takeoff',
  altitude: 5.0,
  heading: 0.0
}"
```

### Hold Position
```bash
# Hold current position
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'hold_position'
}"

# Hold specific position
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'hold_position',
  x: 10.0,
  y: 5.0, 
  z: 8.0,
  heading: 1.57
}"
```

### Landing
```bash
# Land at current position
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'land'
}"

# Land at specific XY position  
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'land',
  x: 0.0,
  y: 0.0,
  heading: 0.0
}"
```

### Emergency Stop
```bash
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{
  command: 'emergency_stop'
}"
```

## 📈 Action Feedback

All navigation actions provide continuous feedback:

```yaml
# Real-time feedback every 100ms
nav_state: "takeoff"           # Current navigation state
progress: 0.75                 # Progress 0.0-1.0 (75%)
current_x: 2.3                 # Current position
current_y: 1.1
current_z: 3.7
current_heading: 0.0
target_x: 2.0                  # Target position
target_y: 1.0  
target_z: 5.0
position_error: 1.34           # Distance to target (meters)
elapsed_time: 8.5              # Time elapsed (seconds)
status_message: "Takeoff in progress - 75% complete"
```

## 🔧 Configuration

Navigation parameters can be customized in:
```
ros_packages/uav_offboard/config/navigation_controller.yaml
```

Key parameters:
- `takeoff.target_altitude`: Default takeoff altitude
- `takeoff.climb_speed`: Climb velocity
- `landing.descent_speed`: Descent velocity  
- `control.feedback_rate`: Control loop frequency
- `tolerances.position_xy`: XY position tolerance
- `safety.max_altitude`: Maximum safe altitude

## 🐛 Troubleshooting

### "Navigation goal REJECTED" 
**Cause:** Vehicle not ready (not armed or not in offboard mode)  
**Solution:** Follow the required sequence (ARM → OFFBOARD → Navigate)

### "Vehicle DISARMED during navigation"
**Cause:** Vehicle lost arm state during command execution  
**Solution:** Re-arm vehicle and restart navigation

### "Vehicle left OFFBOARD mode during navigation" 
**Cause:** Mode was changed externally during navigation  
**Solution:** Ensure no other systems change vehicle mode

### Action timeouts
**Cause:** Command taking longer than configured timeout  
**Solution:** Check vehicle responsiveness and increase timeout in config

## 📝 Examples

### Complete Flight Sequence
```bash
# 1. Prepare vehicle
ros2 run uav_offboard prepare_vehicle

# 2. Execute mission
ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'takeoff', altitude: 5.0}"
# Wait for completion...

ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'hold_position', x: 10.0, y: 10.0, z: 5.0}"
# Wait for completion...

ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'land'}"
# Wait for completion...

# 3. Safely disarm
ros2 service call /vehicle_commander uav_interfaces/srv/VehicleCommander "{command: 'disarm'}"
```

### Programmatic Usage (Python)
```python
import rclpy
from rclpy.action import ActionClient
from uav_interfaces.action import NavigationCommand

# Send takeoff command
goal = NavigationCommand.Goal()
goal.command = 'takeoff'
goal.altitude = 5.0

client = ActionClient(node, NavigationCommand, 'navigation_command')
future = client.send_goal_async(goal, feedback_callback=feedback_cb)
```

## ⭐ Best Practices

1. **Always check vehicle status** before sending commands
2. **Use the preparation tool** for consistent setup
3. **Monitor feedback** during long operations
4. **Have emergency procedures** ready
5. **Test in simulation** before real flights
6. **Respect altitude and safety limits**
7. **Keep manual override** capability available

## 🆘 Emergency Procedures

If something goes wrong:

1. **Cancel current action:**
   ```bash
   ros2 action cancel_goal /navigation_command
   ```

2. **Emergency stop:**
   ```bash
   ros2 action send_goal /navigation_command uav_interfaces/action/NavigationCommand "{command: 'emergency_stop'}"
   ```

3. **Manual mode (external safety pilot):**
   ```bash
   ros2 service call /vehicle_commander uav_interfaces/srv/VehicleCommander "{command: 'manual'}"
   ```

4. **Disarm (if safe):**
   ```bash
   ros2 service call /vehicle_commander uav_interfaces/srv/VehicleCommander "{command: 'disarm'}"
   ```

Remember: **Safety first!** Always have a manual override ready when testing. 