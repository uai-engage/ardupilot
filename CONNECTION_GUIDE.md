# ArduPilot SITL Connection Guide

Quick reference for connecting to your ArduPilot SITL multi-vehicle swarm.

---

## Port Overview

Each vehicle uses **4 different ports** for different purposes:

| Port Type | Protocol | Purpose | Used By |
|-----------|----------|---------|---------|
| **MAVLink** | TCP | Ground Control Station communication | Mission Planner, QGroundControl |
| **SITL** | UDP | External simulator control | Gazebo, RealFlight, X-Plane |
| **DDS** | UDP | ROS2 communication | ROS2 Micro Agent |
| **MAVProxy** | UDP | Telemetry routing/multiplexing | MAVProxy GCS |

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

## Complete Connection Example

### Scenario: 2 Copters with Mission Planner and ROS2

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

# Echo Copter 1 GPS
ros2 topic echo /vehicle_1/navsat/fix

# Echo Copter 2 GPS
ros2 topic echo /vehicle_2/navsat/fix
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
| **Send ROS2 commands/data** | DDS | UDP | `ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019` |
| **Connect external simulator** | SITL | UDP | Configure Gazebo to `127.0.0.1:5501` |
| **Route telemetry with MAVProxy** | MAVProxy OUT | UDP | `mavproxy.py --master=udp:127.0.0.1:14550` |

**Most Common**: You'll primarily use **MAVLink (TCP)** for Mission Planner and **DDS (UDP)** for ROS2.

---

## Need More Help?

- **System Architecture**: See `SYSTEM_ARCHITECTURE.md` for complete system documentation
- **Swarm Generator**: See `SWARM_GENERATOR_GUIDE.md` for generating custom swarms
- **ROS2 Setup**: See `docker/ROS2_CONNECTION.md` for ROS2 configuration details
