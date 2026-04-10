# Motor Speed Control via ROS2

This document describes how to control motor speed parameters externally from your ROS2 workspace.

## Overview

The ESP32 firmware now supports dynamic motor speed control through a new ROS2 topic `/motor_speed_params`. This allows you to set different speed profiles for different parts of your motor sequences without modifying the ESP32 code.

## ROS2 Topic

### `/motor_speed_params` (subscriber)

- **Message Type**: `std_msgs/Float32MultiArray`
- **Format**: `[motor_id, vmax, amax, dmax, vstart, vstop]`
- **Parameters**:
  - `motor_id` (int): Motor number (0, 1, or 2)
  - `vmax` (int): Maximum velocity in steps/s
  - `amax` (int): Acceleration in steps/s²
  - `dmax` (int): Deceleration in steps/s²
  - `vstart` (int): Starting velocity in steps/s
  - `vstop` (int): Stop velocity in steps/s

## Default Values

Each motor starts with these default parameters:
```
VMAX:   50000 steps/s
AMAX:   10000 steps/s²
DMAX:   10000 steps/s²
VSTART: 0 steps/s
VSTOP:  10 steps/s
```

## Usage Methods

### Method 1: Command Line (Quick Testing)

Use the `set_motor_speed.py` script for quick parameter changes:

```bash
# Set motor 0 to slow speed for precise movements
python3 set_motor_speed.py 0 20000 5000 5000 0 10

# Set motor 1 to fast speed
python3 set_motor_speed.py 1 80000 15000 15000 0 10

# Set motor 2 to medium speed
python3 set_motor_speed.py 2 50000 10000 10000 0 10
```

### Method 2: Python ROS2 Node

Use the provided classes in your ROS2 nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.speed_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_speed_params',
            10
        )
    
    def set_motor_speed(self, motor_id, vmax, amax, dmax):
        msg = Float32MultiArray()
        msg.data = [float(motor_id), float(vmax), float(amax), 
                    float(dmax), 0.0, 10.0]
        self.speed_pub.publish(msg)
```

### Method 3: Motor Sequences (Recommended)

Use the `motor_sequence.py` module for complex multi-step sequences:

```python
from motor_sequence import MotorSequence, SpeedProfiles

# Create a sequence for motor 0
sequence = MotorSequence(motor_id=0)

# Slow precise loading
sequence.add_move(
    position=10000,
    speed_profile=SpeedProfiles.SLOW,
    delay_after=0.5
)

# Fast compression
sequence.add_move(
    position=50000,
    speed_profile=SpeedProfiles.FAST,
    delay_after=1.0
)

# Execute the sequence
sequence.execute(node, wait_for_completion=True)
```

## Predefined Speed Profiles

The `SpeedProfiles` class provides common configurations:

| Profile | VMAX | AMAX | DMAX | Use Case |
|---------|------|------|------|----------|
| ULTRA_SLOW | 10000 | 2000 | 2000 | Ultra-precise positioning |
| SLOW | 20000 | 5000 | 5000 | Precise movements |
| MEDIUM | 50000 | 10000 | 10000 | Default/balanced |
| FAST | 80000 | 15000 | 15000 | Rapid movements |
| ULTRA_FAST | 120000 | 20000 | 20000 | Maximum speed (use carefully) |

## Complete Example

The `example_pump_controller.py` demonstrates a full pump sequence with variable speeds:

```bash
python3 example_pump_controller.py
```

This runs a sequence that:
1. Homes all motors
2. Performs slow precise loading (Motor 0)
3. Fast compression (Motor 1)
4. Ultra-fast return (Motor 2)
5. Slow return to home (All motors)

## Integration with Your ROS2 Workspace

### Option 1: Copy to your ROS2 package

```bash
# Copy scripts to your ROS2 workspace
cp set_motor_speed.py ~/ros2_ws/src/your_package/scripts/
cp motor_sequence.py ~/ros2_ws/src/your_package/scripts/
cp example_pump_controller.py ~/ros2_ws/src/your_package/scripts/

# Make executable
chmod +x ~/ros2_ws/src/your_package/scripts/*.py
```

### Option 2: Import as a module

```python
# In your ROS2 node
import sys
sys.path.append('/home/joseph/Documents/PlatformIO/Projects/esp32')

from motor_sequence import MotorSequence, SpeedProfiles
```

## Workflow for Complex Sequences

1. **Home the motors** first using `/pump/home` topic
2. **Set speed parameters** for the first movement segment
3. **Send position command** via `/motor_cmd` topic
4. **Wait for completion** (monitor `/motor_pos`)
5. **Repeat steps 2-4** for each segment with different speeds

## Tips

- **Speed Changes**: Speed parameters take effect immediately on the motor
- **Timing**: Add a small delay (~100ms) after setting speed before sending position commands
- **Safety**: Start with slower speeds and gradually increase
- **Monitoring**: Subscribe to `/motor_pos` to track actual positions
- **Force Feedback**: Subscribe to `/force` for load monitoring during movements

## Troubleshooting

**Motors not responding to speed changes:**
- Ensure micro-ROS agent is running
- Check topic names match exactly
- Verify motor_id is 0, 1, or 2

**Unexpected behavior:**
- Speed changes apply immediately - set speed before sending position
- Each motor maintains its own speed parameters independently
- Parameters persist until changed or ESP32 is reset

## Advanced: Custom Speed Profiles

Create custom profiles for your specific application:

```python
from motor_sequence import MotorSpeedProfile

# Custom profile for delicate sample handling
GENTLE_PROFILE = MotorSpeedProfile(
    vmax=5000,
    amax=1000,
    dmax=1000,
    vstart=0,
    vstop=5
)

# Custom profile for empty chamber movement
RAPID_PROFILE = MotorSpeedProfile(
    vmax=150000,
    amax=25000,
    dmax=25000,
    vstart=0,
    vstop=20
)
```

## Hardware Limits

**TMC5272 Constraints:**
- Maximum VMAX: ~200,000 steps/s (depends on microstepping)
- Maximum AMAX/DMAX: Hardware limited by motor/load
- Recommended starting values: 50000/10000/10000
- Test incrementally when increasing speeds

## See Also

- Main firmware: [src/main.cpp](src/main.cpp)
- Motor command topic: `/motor_cmd`
- Position monitoring: `/motor_pos`
- Encoder positions: `/encoder_pos`
