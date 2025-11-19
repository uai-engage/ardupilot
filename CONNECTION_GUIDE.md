# ArduPilot SITL Connection Guide

Quick reference for connecting to your ArduPilot SITL multi-vehicle swarm.

---

## Port Overview

Each vehicle uses **5 different ports** for different purposes:

| Port Type | Protocol | Purpose | Used By |
|-----------|----------|---------|---------|
| **MAVLink GCS** | TCP | Ground Control Station communication | Mission Planner, QGroundControl |
| **MAVLink ROS** | TCP | ROS2 MAVLink bridge communication | MAVROS2 |
| **SITL** | UDP | External simulator control | Gazebo, RealFlight, X-Plane |
| **DDS** | UDP | ROS2 DDS communication | ROS2 Micro Agent |
| **MAVProxy** | UDP | Telemetry routing/multiplexing (optional) | MAVProxy GCS |

**Note**: Each vehicle gets dedicated MAVLink TCP ports to avoid conflicts when multiple clients (GCS and ROS2) connect simultaneously.

---

## 1. Connecting Mission Planner (GCS)

### Connection Method: Direct TCP

**Purpose**: Control and monitor your vehicles

**Connection String Format**:
```
tcp:<IP>:<PORT>
```

### Examples:

**Copter 1 (Instance 0)**:
```
Connection Type: TCP
IP Address: 127.0.0.1
Port: 5760

Connection String: tcp:127.0.0.1:5760
```

**Copter 2 (Instance 1)**:
```
Connection Type: TCP
IP Address: 127.0.0.1
Port: 5770

Connection String: tcp:127.0.0.1:5770
```

**Plane 1 (Instance 2)**:
```
Connection Type: TCP
IP Address: 127.0.0.1
Port: 5780

Connection String: tcp:127.0.0.1:5780
```

### Port Formula:
```
MAVLink Port = 5760 + (Instance × 10)
```

### Mission Planner Steps:

1. **Open Mission Planner**
2. **Top-right dropdown**: Select "TCP"
3. **Click "Connect"** button
4. **Enter connection details**:
   - Host: `127.0.0.1`
   - Port: `5760` (for first vehicle)
5. **Click "Connect"**

### Multi-Vehicle View:

To monitor multiple vehicles simultaneously:

**Option 1: Separate Windows**
- Open multiple Mission Planner instances
- Connect each to different port (5760, 5770, 5780, etc.)

**Option 2: Multi-Vehicle Mode**
- File → New Window
- Each window connects to different vehicle port
- All vehicles shown on same map

---

## 2. Connecting ROS2 (DDS Communication)

### Connection Method: Micro ROS Agent (UDP)

**Purpose**: Exchange data between ArduPilot and ROS2 nodes

**Important**: Each vehicle needs its **OWN** Micro ROS Agent on a **unique port**

### Examples:

**Copter 1 (Instance 0)**:
```bash
# Terminal 1
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Copter 2 (Instance 1)**:
```bash
# Terminal 2
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2020
```

**Plane 1 (Instance 2)**:
```bash
# Terminal 3
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2021
```

### Port Formula:
```
DDS Port = 2019 + Instance
```

### ROS2 Topic Namespaces:

Each vehicle publishes to its own namespace based on **SYSID**:

```bash
# List all topics
ros2 topic list

# Expected output:
/vehicle_1/gps_global_origin/...
/vehicle_1/navsat/fix
/vehicle_2/gps_global_origin/...
/vehicle_2/navsat/fix
/vehicle_3/gps_global_origin/...
```

### Subscribe to Specific Vehicle:

```bash
# Copter 1 GPS data
ros2 topic echo /vehicle_1/navsat/fix

# Copter 2 GPS data
ros2 topic echo /vehicle_2/navsat/fix

# Plane 1 attitude
ros2 topic echo /vehicle_3/ap/attitude/quaternion
```

---

## 2B. Alternative: Connecting ROS2 via MAVROS2

### Connection Method: MAVROS2 (MAVLink-ROS2 Bridge)

**Purpose**: Alternative to Micro ROS Agent - connects ROS2 to ArduPilot using MAVLink protocol

**Key Difference**:
- **Micro ROS Agent**: Uses DDS protocol (port 2019+), ArduPilot acts as DDS client
- **MAVROS2**: Uses MAVLink protocol (port 5761+), connects to dedicated MAVLink ROS port

**Important**: Each vehicle has **separate MAVLink TCP ports** for GCS (5760, 5770...) and ROS2 (5761, 5771...). This prevents connection conflicts when both Mission Planner and MAVROS2 are used simultaneously.

### When to Use MAVROS2 vs Micro ROS Agent:

| Feature | Micro ROS Agent (DDS) | MAVROS2 (MAVLink) |
|---------|----------------------|-------------------|
| **Protocol** | DDS/XRCE-DDS | MAVLink |
| **Connection** | Dedicated DDS port | Existing MAVLink port |
| **Setup** | Requires Micro ROS Agent running | Requires MAVROS2 package installed |
| **Topics** | Native ArduPilot DDS topics | Standard MAVROS topics |
| **Maturity** | Newer (experimental) | Mature, well-tested |
| **Performance** | Lower overhead | Slightly higher overhead |
| **Use Case** | Modern DDS-based systems | Traditional ROS/MAVLink workflows |

### Installation:

**Install MAVROS2** (if not already installed):

```bash
# Install from apt (ROS2 Humble)
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets (required)
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### Configuration for Multi-Vehicle:

MAVROS2 uses **launch files** to configure each vehicle connection.

#### Example 1: Single Vehicle (Copter 1)

**Create**: `~/ros2_ws/src/ardupilot_mavros/launch/copter1.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_copter1',
            namespace='copter1',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5761@',  # Dedicated ROS port for Copter 1
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
            output='screen',
        ),
    ])
```

**Launch**:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ardupilot_mavros copter1.launch.py
```

#### Example 2: Multi-Vehicle Setup (2 Copters)

**Create**: `~/ros2_ws/src/ardupilot_mavros/launch/multi_vehicle.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Copter 1
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_copter1',
            namespace='copter1',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5761@',  # Dedicated ROS port for Copter 1
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
            output='screen',
        ),
        # Copter 2
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_copter2',
            namespace='copter2',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5771@',  # Dedicated ROS port for Copter 2
                'target_system_id': 2,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
            output='screen',
        ),
    ])
```

**Launch**:
```bash
ros2 launch ardupilot_mavros multi_vehicle.launch.py
```

### Quick Launch (Without Launch File):

**Copter 1**:
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5761@ \
  -p target_system_id:=1
```

**Copter 2**:
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter2 \
  -p fcu_url:=tcp://127.0.0.1:5771@ \
  -p target_system_id:=2
```

**Plane 1**:
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/plane1 \
  -p fcu_url:=tcp://127.0.0.1:5781@ \
  -p target_system_id:=3
```

### Port Configuration for MAVROS2:

| Vehicle | Instance | SYSID | GCS Port | ROS Port | MAVROS2 Connection |
|---------|----------|-------|----------|----------|-------------------|
| Copter 1 | 0 | 1 | 5760 | **5761** | `tcp://127.0.0.1:5761@` |
| Copter 2 | 1 | 2 | 5770 | **5771** | `tcp://127.0.0.1:5771@` |
| Copter 3 | 2 | 3 | 5780 | **5781** | `tcp://127.0.0.1:5781@` |
| Plane 1 | 3 | 4 | 5790 | **5791** | `tcp://127.0.0.1:5791@` |

**Formula**: `tcp://127.0.0.1:<5761 + Instance × 10>@`

**Note**: Use the **ROS Port** (odd numbers) for MAVROS2, not the GCS Port. This prevents connection conflicts when Mission Planner is also connected.

### MAVROS2 Topic Structure:

MAVROS2 publishes to namespaced topics:

```bash
# List all MAVROS topics
ros2 topic list

# Expected output:
/copter1/mavros/state
/copter1/mavros/global_position/global
/copter1/mavros/imu/data
/copter2/mavros/state
/copter2/mavros/global_position/global
/copter2/mavros/imu/data
```

### Common MAVROS2 Topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/copter1/mavros/state` | `mavros_msgs/State` | Vehicle connection state |
| `/copter1/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS position |
| `/copter1/mavros/imu/data` | `sensor_msgs/Imu` | IMU data |
| `/copter1/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Local position |
| `/copter1/mavros/battery` | `sensor_msgs/BatteryState` | Battery status |
| `/copter1/mavros/rc/in` | `mavros_msgs/RCIn` | RC inputs |

### Subscribe to MAVROS2 Topics:

```bash
# Copter 1 GPS position
ros2 topic echo /copter1/mavros/global_position/global

# Copter 2 IMU data
ros2 topic echo /copter2/mavros/imu/data

# Plane 1 state
ros2 topic echo /plane1/mavros/state
```

### Sending Commands via MAVROS2:

**Arm Vehicle**:
```bash
ros2 service call /copter1/mavros/cmd/arming \
  mavros_msgs/srv/CommandBool \
  "{value: true}"
```

**Set Mode**:
```bash
ros2 service call /copter1/mavros/set_mode \
  mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}"
```

**Takeoff**:
```bash
ros2 service call /copter1/mavros/cmd/takeoff \
  mavros_msgs/srv/CommandTOL \
  "{altitude: 10.0}"
```

### Comparison: Micro ROS Agent vs MAVROS2 Topics

| Data | Micro ROS Agent (DDS) | MAVROS2 (MAVLink) |
|------|----------------------|-------------------|
| GPS | `/vehicle_1/navsat/fix` | `/copter1/mavros/global_position/global` |
| IMU | `/vehicle_1/ap/imu/experimental/data` | `/copter1/mavros/imu/data` |
| Battery | `/vehicle_1/ap/battery/battery0` | `/copter1/mavros/battery` |
| State | `/vehicle_1/ap/mode` | `/copter1/mavros/state` |

### Complete Multi-Vehicle Example with MAVROS2:

**Scenario**: 2 Copters with MAVROS2

**Terminal 1 - Start Docker Swarm**:
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Terminal 2 - Start MAVROS2 for Copter 1**:
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760@ \
  -p target_system_id:=1
```

**Terminal 3 - Start MAVROS2 for Copter 2**:
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter2 \
  -p fcu_url:=tcp://127.0.0.1:5770@ \
  -p target_system_id:=2
```

**Terminal 4 - Monitor Topics**:
```bash
# List all topics
ros2 topic list

# Monitor Copter 1 GPS
ros2 topic echo /copter1/mavros/global_position/global

# Monitor Copter 2 state
ros2 topic echo /copter2/mavros/state
```

### Using Both Micro ROS Agent AND MAVROS2:

You **can** use both simultaneously if needed:

**Configuration**:
- **Micro ROS Agent**: Connects to DDS ports (2019, 2020, 2021...)
- **MAVROS2**: Connects to MAVLink ports (5760, 5770, 5780...)

**Example Setup**:

**Terminal 1 - Docker**:
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Terminal 2 - Micro ROS Agent (Copter 1)**:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Terminal 3 - MAVROS2 (Copter 1)**:
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760@ \
  -p target_system_id:=1
```

**Result**: You'll have BOTH sets of topics available:
- DDS topics: `/vehicle_1/*`
- MAVROS topics: `/copter1/mavros/*`

**Use Case**: Use DDS for high-frequency data (IMU, attitude) and MAVROS2 for commands/services (arm, takeoff, set_mode).

### Choosing Between Micro ROS Agent and MAVROS2:

**Use Micro ROS Agent (DDS) when**:
- You want native ArduPilot DDS integration
- Lower latency is critical
- You're building new DDS-based systems
- You need high-frequency sensor data

**Use MAVROS2 (MAVLink) when**:
- You have existing MAVROS/ROS1 code to port
- You need mature, well-documented ROS integration
- You want standard MAVROS topics/services
- You're integrating with existing MAVLink-based tools

**Use Both when**:
- You need both DDS and MAVLink capabilities
- You want DDS for data and MAVROS for commands
- You're transitioning from MAVROS to DDS gradually

---

## 3. Connecting External Simulator (SITL Port)

### Connection Method: UDP

**Purpose**: Control vehicle physics from external simulator (Gazebo, RealFlight, X-Plane)

**Note**: Only needed if you're using an **external** simulator. With built-in SITL physics (default), this port is **NOT actively used**.

### Examples:

**Copter 1 (Instance 0)**:
```
Protocol: UDP
IP: 127.0.0.1
Port: 5501
```

**Copter 2 (Instance 1)**:
```
Protocol: UDP
IP: 127.0.0.1
Port: 5511
```

### Port Formula:
```
SITL Port = 5501 + (Instance × 10)
```

### When to Use:

- **External Gazebo**: Configure Gazebo to send physics data to SITL port
- **RealFlight**: Configure RealFlight connection to SITL port
- **X-Plane**: Configure X-Plane plugin to SITL port

### Current Setup:

In your current configuration, vehicles run with **built-in SITL physics**, so external simulator connection is **optional**.

---

## 4. MAVProxy Telemetry Output

### Connection Method: UDP

**Purpose**: Additional telemetry output for routing to multiple GCS or logging

### Examples:

**Copter 1 (Instance 0)**:
```
Protocol: UDP
IP: 127.0.0.1
Port: 14550
```

**Copter 2 (Instance 1)**:
```
Protocol: UDP
IP: 127.0.0.1
Port: 14551
```

### Port Formula:
```
MAVProxy OUT Port = 14550 + Instance
```

### Usage:

MAVProxy can forward telemetry to multiple destinations:

```bash
# Example: Listen to Copter 1 telemetry via MAVProxy
mavproxy.py --master=udp:127.0.0.1:14550
```

### Disable MAVProxy:

If you don't need MAVProxy, set in `.env.generated`:
```bash
COPTER1_MAVPROXY_ENABLED=0
COPTER2_MAVPROXY_ENABLED=0
```

---

## Complete Connection Examples

### Example 1: Using Micro ROS Agent (DDS)

**Scenario**: 2 Copters with Mission Planner and ROS2 via Micro ROS Agent

**Terminal 1 - Start ROS2 Agent for Copter 1**:
```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Terminal 2 - Start ROS2 Agent for Copter 2**:
```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2020
```

**Terminal 3 - Start Docker Swarm**:
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Mission Planner Instance 1 - Copter 1**:
```
Connection: tcp:127.0.0.1:5760
```

**Mission Planner Instance 2 - Copter 2**:
```
Connection: tcp:127.0.0.1:5770
```

**Terminal 4 - Monitor ROS2 Topics**:
```bash
# List all topics
ros2 topic list

# Echo Copter 1 GPS (DDS topics)
ros2 topic echo /vehicle_1/navsat/fix

# Echo Copter 2 GPS (DDS topics)
ros2 topic echo /vehicle_2/navsat/fix
```

---

### Example 2: Using MAVROS2 (MAVLink)

**Scenario**: 2 Copters with Mission Planner and ROS2 via MAVROS2

**Terminal 1 - Start Docker Swarm**:
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Terminal 2 - Start MAVROS2 for Copter 1**:
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760@ \
  -p target_system_id:=1
```

**Terminal 3 - Start MAVROS2 for Copter 2**:
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter2 \
  -p fcu_url:=tcp://127.0.0.1:5770@ \
  -p target_system_id:=2
```

**Mission Planner**: You can still connect Mission Planner to the same MAVLink ports, but MAVROS2 will also be connected. Both can share the connection.

**Terminal 4 - Monitor ROS2 Topics**:
```bash
# List all topics
ros2 topic list

# Echo Copter 1 GPS (MAVROS topics)
ros2 topic echo /copter1/mavros/global_position/global

# Echo Copter 2 GPS (MAVROS topics)
ros2 topic echo /copter2/mavros/global_position/global

# Arm Copter 1 via MAVROS service
ros2 service call /copter1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

---

### Example 3: Using Both (Hybrid Approach)

**Scenario**: Use DDS for high-frequency data and MAVROS2 for commands

**Terminal 1 - Start Docker Swarm**:
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Terminal 2 - Start Micro ROS Agent (for DDS data)**:
```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Terminal 3 - Start MAVROS2 (for commands/services)**:
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760@ \
  -p target_system_id:=1
```

**Terminal 4 - Use Both Topic Types**:
```bash
# Monitor DDS topics for high-frequency data
ros2 topic echo /vehicle_1/ap/imu/experimental/data

# Monitor MAVROS topics
ros2 topic echo /copter1/mavros/state

# Send commands via MAVROS services
ros2 service call /copter1/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /copter1/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

---

## Port Assignment Reference Table

For quick lookup, here's a complete port table for the first 4 vehicles:

| Vehicle | Instance | SYSID | MAVLink (TCP) | SITL (UDP) | DDS (UDP) | MAVProxy (UDP) |
|---------|----------|-------|---------------|------------|-----------|----------------|
| Copter 1 | 0 | 1 | 5760 | 5501 | 2019 | 14550 |
| Copter 2 | 1 | 2 | 5770 | 5511 | 2020 | 14551 |
| Copter 3 | 2 | 3 | 5780 | 5521 | 2021 | 14552 |
| Copter 4 | 3 | 4 | 5790 | 5531 | 2022 | 14553 |
| Plane 1 | 4 | 5 | 5800 | 5541 | 2023 | 14554 |
| Plane 2 | 5 | 6 | 5810 | 5551 | 2024 | 14555 |
| VTOL 1 | 6 | 7 | 5820 | 5561 | 2025 | 14556 |
| VTOL 2 | 7 | 8 | 5830 | 5571 | 2026 | 14557 |

---

## Viewing Port Information at Startup

When you run `docker compose up`, each vehicle will display its port information:

```
=========================================
ArduPilot Copter 1 (Instance 0, SYSID 1)
-----------------------------------------
PORT ASSIGNMENTS:
  MAVLink:  5760 (TCP) - For Mission Planner/GCS
  SITL:     5501 (UDP) - For external simulator
  DDS:      2019 (UDP) - For ROS2 communication
  MAVProxy: 14550 (UDP) - For telemetry output

CONNECTIONS:
  Mission Planner: tcp:127.0.0.1:5760
  ROS2 Agent: ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
=========================================
```

---

## Troubleshooting

### Mission Planner Won't Connect

**Check**:
1. Docker container is running: `docker ps`
2. Port is correct: `5760 + (Instance × 10)`
3. Connection type is **TCP**
4. IP is `127.0.0.1` or `localhost`

**Test with MAVProxy**:
```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

### ROS2 Topics Not Appearing

**Check**:
1. Micro ROS Agent is running on correct port
2. Agent started BEFORE Docker containers
3. DDS is enabled in `.env.generated`: `ENABLE_DDS=1`
4. Wait 10-15 seconds after vehicle startup

**Verify**:
```bash
# Check agent is running
ps aux | grep micro_ros_agent

# Check topics
ros2 topic list
```

### Port Already in Use

**Find conflicting process**:
```bash
# Check what's using port 5760
sudo netstat -tulpn | grep 5760

# Kill the process
sudo kill -9 <PID>
```

**Or generate swarm with different vehicles** to use different ports.

---

## Summary

| What You Want | Use This Port | Protocol | Example Connection |
|---------------|---------------|----------|-------------------|
| **Control with Mission Planner** | MAVLink | TCP | `tcp:127.0.0.1:5760` |
| **ROS2 via Micro ROS Agent** | DDS | UDP | `ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019` |
| **ROS2 via MAVROS2** | MAVLink | TCP | `ros2 run mavros mavros_node -p fcu_url:=tcp://127.0.0.1:5760@` |
| **Connect external simulator** | SITL | UDP | Configure Gazebo to `127.0.0.1:5501` |
| **Route telemetry with MAVProxy** | MAVProxy OUT | UDP | `mavproxy.py --master=udp:127.0.0.1:14550` |

**Most Common**: You'll primarily use **MAVLink (TCP)** for Mission Planner and either **DDS (UDP)** or **MAVROS2 (TCP)** for ROS2.

### ROS2 Connection Options:

**Option 1: Micro ROS Agent (DDS)** - Native ArduPilot DDS, lower latency, newer
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

**Option 2: MAVROS2 (MAVLink)** - Mature, well-tested, standard ROS topics
```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/copter1 -p fcu_url:=tcp://127.0.0.1:5760@
```

**Option 3: Both** - Use DDS for data and MAVROS2 for commands simultaneously

---

## Need More Help?

- **System Architecture**: See `SYSTEM_ARCHITECTURE.md` for complete system documentation
- **Swarm Generator**: See `SWARM_GENERATOR_GUIDE.md` for generating custom swarms
- **ROS2 Setup**: See `docker/ROS2_CONNECTION.md` for ROS2 configuration details
