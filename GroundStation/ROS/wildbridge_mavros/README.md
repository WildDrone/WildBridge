# WildBridge MAVROS Bridge

A MAVROS-compatible ROS 2 interface for controlling DJI drones through WildBridge.

This package allows applications built for PX4/ArduPilot drones (using MAVROS) to control DJI drones with minimal changes.

## Features

- **MAVROS-compatible Topics**: Standard topics like `/mavros/local_position/pose`, `/mavros/global_position/global`, `/mavros/battery`
- **MAVROS-compatible Services**: `mavros/cmd/takeoff`, `mavros/cmd/land`, `mavros/cmd/rtl`, `mavros/cmd/arming`
- **Setpoint Interfaces**: Position, velocity, and attitude setpoints
- **Multi-drone Support**: Launch multiple drones with namespaces
- **Flight Mode Translation**: Automatic mapping between DJI and PX4 flight modes

## Installation

```bash
cd ~/ros2_ws/src
# Assuming WildBridge is already cloned
cd WildBridge/GroundStation/ROS

# Build the package
cd ~/ros2_ws
colcon build --packages-select wildbridge_mavros
source install/setup.bash
```

## Usage

### Single Drone

```bash
ros2 launch wildbridge_mavros mavros_bridge.launch.py drone_ip:=192.168.1.100
```

### Multi-Drone

Edit `config/drones.yaml` to configure your drones:

```yaml
drones:
  - id: 1
    ip: "192.168.1.100"
    namespace: "drone_1"
  - id: 2
    ip: "192.168.1.101"
    namespace: "drone_2"
```

Then launch:

```bash
ros2 launch wildbridge_mavros multi_drone.launch.py
```

## Topic Mapping

### Published Topics

| MAVROS Topic | Description |
|--------------|-------------|
| `mavros/state/connected` | Connection status |
| `mavros/state/armed` | Armed status |
| `mavros/state/mode` | Current flight mode (PX4-style) |
| `mavros/local_position/pose` | Local position (relative to home) |
| `mavros/local_position/velocity_local` | Local velocity |
| `mavros/global_position/global` | GPS position (NavSatFix) |
| `mavros/global_position/compass_hdg` | Compass heading |
| `mavros/global_position/rel_alt` | Relative altitude |
| `mavros/global_position/satellites` | GPS satellite count |
| `mavros/home_position/home` | Home position |
| `mavros/battery` | Battery state |
| `mavros/imu/data` | IMU data |

### WildBridge-specific Topics

| Topic | Description |
|-------|-------------|
| `wildbridge/waypoint_reached` | Waypoint reached flag |
| `wildbridge/distance_to_home` | Distance to home (meters) |
| `wildbridge/flight_time_remaining` | Estimated remaining flight time |

### Subscribed Topics (Setpoints)

| Topic | Description |
|-------|-------------|
| `mavros/setpoint_position/local` | Local position setpoint |
| `mavros/setpoint_position/global` | Global position setpoint (GPS) |
| `mavros/setpoint_velocity/cmd_vel` | Velocity setpoint |
| `mavros/setpoint_attitude/attitude` | Attitude setpoint (gimbal) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `mavros/cmd/arming` | SetBool | Arm/disarm (DJI auto-arms on takeoff) |
| `mavros/cmd/takeoff` | Trigger | Takeoff command |
| `mavros/cmd/land` | Trigger | Land command |
| `mavros/cmd/rtl` | Trigger | Return to launch |
| `mavros/set_mode/offboard` | Trigger | Enable offboard (virtual stick) mode |
| `wildbridge/enable_virtual_stick` | Trigger | Enable virtual stick |
| `wildbridge/abort_mission` | Trigger | Abort current mission |

## Flight Mode Mapping

| DJI Mode | MAVROS/PX4 Mode |
|----------|-----------------|
| MANUAL | MANUAL |
| GPS | POSCTL |
| ATTI | ALTCTL |
| GO_HOME | AUTO.RTL |
| AUTO_LANDING | AUTO.LAND |
| AUTO_TAKEOFF | AUTO.TAKEOFF |
| VIRTUAL_STICK | OFFBOARD |
| WAYPOINT | AUTO.MISSION |

## Example: Sending a Waypoint

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.pub = self.create_publisher(NavSatFix, 'mavros/setpoint_position/global', 10)
    
    def send_waypoint(self, lat, lon, alt):
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.pub.publish(msg)

# Usage
rclpy.init()
node = WaypointSender()
node.send_waypoint(49.306254, 4.593728, 20.0)
```

## Example: Takeoff and Land

```bash
# Takeoff
ros2 service call /mavros/cmd/takeoff std_srvs/srv/Trigger

# Wait for drone to reach altitude...

# Land
ros2 service call /mavros/cmd/land std_srvs/srv/Trigger

# Or Return to Launch
ros2 service call /mavros/cmd/rtl std_srvs/srv/Trigger
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_ip` | 192.168.1.100 | WildBridge RC IP address |
| `system_id` | 1 | MAVLink system ID |
| `component_id` | 1 | MAVLink component ID |
| `telemetry_rate` | 20.0 | Telemetry publishing rate (Hz) |

## Notes

- DJI drones arm automatically on takeoff, so the `arming` service is mainly for compatibility
- Virtual stick mode must be enabled for velocity/position setpoints to work
- The coordinate conversions assume a local flat-Earth approximation, suitable for typical drone operations

## License

MIT License - Part of the WildBridge project.
