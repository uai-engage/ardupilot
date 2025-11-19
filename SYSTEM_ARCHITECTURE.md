# ArduPilot SITL Multi-Vehicle System Architecture

**Complete Guide to Understanding Ports, Connections, and Components**

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Port Assignments](#port-assignments)
3. [Understanding MAVProxy Parameters](#understanding-mavproxy-parameters)
4. [Connection Scenarios](#connection-scenarios)
5. [Built-in vs External Simulators](#built-in-vs-external-simulators)
6. [ROS2/DDS Integration](#ros2dds-integration)
7. [Mission Planner Connection Methods](#mission-planner-connection-methods)
8. [Complete Data Flow Diagrams](#complete-data-flow-diagrams)
9. [Common Use Cases](#common-use-cases)
10. [Troubleshooting Guide](#troubleshooting-guide)

---

## System Overview

This multi-vehicle swarm system consists of multiple ArduPilot SITL instances running in Docker containers, each with:

- **Unique SYSID** (System ID) for vehicle identification
- **Unique MAVLink port** for Ground Control Station connections
- **Unique DDS port** for ROS2 communication
- **Unique SITL port** for simulation physics control
- **Optional MAVProxy** for telemetry routing

### Key Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Container (per vehicle)           │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │            ArduPilot SITL                            │  │
│  │  - Flight control logic                             │  │
│  │  - Built-in physics simulation (default)            │  │
│  │  - Parameter management                             │  │
│  │  - Sensor simulation                                │  │
│  └────┬─────────────┬──────────────┬──────────────┬────┘  │
│       │             │              │              │        │
│   MAVLink         SITL           DDS          MAVProxy     │
│   Port 5760      Port 5501     Port 2019    UDP 14550     │
└───────┼─────────────┼──────────────┼──────────────┼────────┘
        │             │              │              │
        ▼             ▼              ▼              ▼
  Mission Planner  (Physics)    ROS2 Agent    (Optional GCS)
```

---

## Port Assignments

### Port Allocation Formula

For a vehicle at **Instance N** (starting from 0):

| Port Type | Formula | Example (Instance 0) | Example (Instance 1) |
|-----------|---------|---------------------|---------------------|
| **MAVLink** | `5760 + (N × 10)` | 5760 | 5770 |
| **SITL** | `5501 + (N × 10)` | 5501 | 5511 |
| **DDS** | `2019 + N` | 2019 | 2020 |
| **MAVProxy OUT** | `14550 + N` | 14550 | 14551 |

### Complete Port Map (4 Vehicle Example)

| Vehicle | Instance | SYSID | MAVLink | SITL | DDS | MAVProxy OUT |
|---------|----------|-------|---------|------|-----|--------------|
| Copter 1 | 0 | 1 | 5760 | 5501 | 2019 | 14550 |
| Copter 2 | 1 | 2 | 5770 | 5511 | 2020 | 14551 |
| Plane 1 | 2 | 3 | 5780 | 5521 | 2021 | 14552 |
| VTOL 1 | 3 | 4 | 5790 | 5531 | 2022 | 14553 |

---

## Understanding MAVProxy Parameters

MAVProxy is a MAVLink ground station software that acts as a **router/multiplexer** between ArduPilot and Ground Control Stations.

### 1. MAVPROXY_OUT

**Purpose:** Where MAVProxy **broadcasts** telemetry (outbound to GCS)

**What it does:**
- MAVProxy sends vehicle telemetry to this UDP address
- Ground Control Stations listen on this port
- Multiple GCS can connect simultaneously
- One-to-many distribution

**Format:** `IP:Port` (e.g., `127.0.0.1:14550`)

**Example:**
```bash
COPTER1_MAVPROXY_OUT=127.0.0.1:14550
COPTER2_MAVPROXY_OUT=127.0.0.1:14551
```

**When to use:**
- You want UDP connection to Mission Planner
- You need multiple GCS to see the same vehicle
- You're using MAVProxy features (like command forwarding)

**When NOT needed:**
- Direct TCP connection to ArduPilot (bypass MAVProxy entirely)
- Single GCS per vehicle

---

### 2. MAVPROXY_MASTER

**Purpose:** Where MAVProxy **connects to** ArduPilot (inbound source)

**What it does:**
- This is the MAVLink port where ArduPilot is listening
- MAVProxy reads telemetry from this source
- Acts as the "master" connection for MAVProxy
- One-to-one connection

**Format:** `tcp:IP:Port` (e.g., `tcp:127.0.0.1:5760`)

**Example:**
```bash
COPTER1_MAVPROXY_MASTER=tcp:127.0.0.1:5760
COPTER2_MAVPROXY_MASTER=tcp:127.0.0.1:5770
```

**When to use:**
- MAVProxy is enabled and needs to know where ArduPilot is
- Automatically configured by `sim_vehicle.py` based on instance number

**When NOT needed:**
- MAVProxy is disabled (`MAVPROXY_ENABLED=0`)
- You're connecting directly to ArduPilot

---

### 3. MAVPROXY_SITL

**Purpose:** SITL physics simulation control port

**What it does:**
- Used for sending simulation control commands
- NOT for telemetry - for physics manipulation only
- Commands like: set wind, inject GPS errors, change time-of-day
- Internal communication between MAVProxy and SITL

**Format:** `IP:Port` (e.g., `127.0.0.1:5501`)

**Example:**
```bash
COPTER1_MAVPROXY_SITL=127.0.0.1:5501
COPTER2_MAVPROXY_SITL=127.0.0.1:5511
```

**When to use:**
- Running external simulators (RealFlight, X-Plane, Gazebo)
- Using MAVProxy simulation control commands
- Advanced testing scenarios

**When NOT needed:**
- Built-in SITL physics (default mode)
- Direct TCP connection to Mission Planner
- Standard testing/development

---

### MAVProxy Data Flow

```
┌─────────────────┐
│ ArduPilot SITL  │
│  MAVLink: 5760  │ ← MAVPROXY_MASTER (MAVProxy reads from here)
│  SITL: 5501     │ ← MAVPROXY_SITL (MAVProxy controls physics)
└────────┬────────┘
         │
    ┌────▼────────┐
    │  MAVProxy   │
    │  (Router)   │
    └────┬────────┘
         │
         ├─→ UDP:14550 (MAVPROXY_OUT) → Mission Planner
         ├─→ UDP:14550 (MAVPROXY_OUT) → QGroundControl
         └─→ UDP:14550 (MAVPROXY_OUT) → Other GCS
```

---

## Connection Scenarios

### Scenario 1: Direct TCP (Most Common, Simplest)

**You're using this if:** You connect Mission Planner to `tcp:localhost:5760`

```
Mission Planner
     │
     │ TCP Connection (Port 5760)
     │
     ▼
ArduPilot SITL
```

**Characteristics:**
- ✅ Lowest latency
- ✅ Simplest setup
- ✅ Most reliable
- ❌ One GCS per vehicle
- ❌ No MAVProxy features

**Parameters used:**
- MAVLink Port (5760, 5770, etc.)

**Parameters NOT used:**
- MAVPROXY_OUT
- MAVPROXY_MASTER
- MAVPROXY_SITL

**In .env.generated:**
```bash
COPTER1_MAVPROXY_ENABLED=0  # Disable MAVProxy
# OR connect directly even if MAVProxy is running
```

---

### Scenario 2: UDP via MAVProxy

**You're using this if:** You connect Mission Planner to `udp:localhost:14550`

```
Mission Planner (UDP:14550)
     │
     ▼
MAVProxy (Router)
     │
     │ TCP to MAVPROXY_MASTER
     │
     ▼
ArduPilot SITL (TCP:5760)
```

**Characteristics:**
- ✅ Multiple GCS can connect
- ✅ MAVProxy features (scripting, logging)
- ✅ UDP auto-discovery
- ❌ Slightly higher latency
- ❌ More complex setup

**Parameters used:**
- MAVPROXY_OUT (14550, 14551, etc.)
- MAVPROXY_MASTER (tcp:5760, tcp:5770, etc.)
- MAVLink Port (5760, 5770, etc.)

**Parameters optionally used:**
- MAVPROXY_SITL (if using sim control)

**In .env.generated:**
```bash
COPTER1_MAVPROXY_ENABLED=1
COPTER1_MAVPROXY_OUT=127.0.0.1:14550
COPTER1_MAVPROXY_MASTER=tcp:127.0.0.1:5760
```

---

### Scenario 3: Multiple GCS, Same Vehicle

**Use case:** Multiple people watching the same drone

```
Mission Planner 1 ─┐
                   │
Mission Planner 2 ─┼─→ UDP:14550 → MAVProxy → ArduPilot
                   │
QGroundControl ────┘
```

**Setup:**
```bash
COPTER1_MAVPROXY_ENABLED=1
COPTER1_MAVPROXY_OUT=127.0.0.1:14550
```

All GCS listen on UDP port 14550 and receive the same data.

---

## Built-in vs External Simulators

### Built-in SITL Physics (Your Current Setup)

**What it is:**
- Physics simulation built into ArduPilot
- Runs inside the Docker container
- Default mode when you run `sim_vehicle.py`

**Architecture:**
```
┌─────────────────────────────┐
│  Docker Container           │
│  ┌───────────────────────┐  │
│  │  ArduPilot SITL       │  │
│  │  ┌─────────────────┐  │  │
│  │  │ Built-in Physics│  │  │
│  │  │  - Motors       │  │  │
│  │  │  - Aerodynamics │  │  │
│  │  │  - Sensors      │  │  │
│  │  └─────────────────┘  │  │
│  └───────────────────────┘  │
└─────────────────────────────┘
        │
        ▼
  Mission Planner (TCP:5760)
```

**Parameters:**
- MAVPROXY_SITL: **Defined but NOT actively used**
- Physics runs internally
- No external simulator needed

**When to use:**
- Standard development/testing
- Quick prototyping
- Most common use case

---

### External Simulator (RealFlight, Gazebo, X-Plane)

**What it is:**
- Physics simulation runs in external software
- ArduPilot receives sensor data from simulator
- More realistic physics
- Better graphics

**Architecture:**
```
┌────────────────────┐
│  RealFlight        │
│  (Windows PC)      │
│  - 3D Graphics     │
│  - Physics Engine  │
└─────────┬──────────┘
          │ UDP (sensor data)
          │
┌─────────▼──────────────────┐
│  Docker Container          │
│  ┌──────────────────────┐  │
│  │  ArduPilot SITL      │  │
│  │  (No built-in phys.) │  │
│  └──────────────────────┘  │
└────────────────────────────┘
        │
        ▼
  MAVProxy → Mission Planner
```

**Parameters:**
- MAVPROXY_SITL: **Actively used** (port 5501, 5511, etc.)
- SIM_ADDRESS: **Set to RealFlight IP** (e.g., 192.168.1.100)

**In .env.generated:**
```bash
SIM_ADDRESS=192.168.1.100  # RealFlight host
COPTER1_MAVPROXY_SITL=127.0.0.1:5501  # Used for sim control
```

**When to use:**
- Realistic flight testing
- Hardware-in-the-loop testing
- Training with visual simulation
- Testing with actual simulator hardware

---

## ROS2/DDS Integration

### Why Separate DDS Ports?

**Problem with shared port:**
- Multiple ArduPilot instances cannot share one Micro ROS Agent
- Causes "No ping response, disconnecting" errors
- Connection instability

**Solution:**
- Each vehicle gets unique DDS port
- Each port runs its own Micro ROS Agent
- Stable, isolated connections

### DDS Connection Architecture

```
┌──────────────┐                ┌──────────────┐
│  Copter 1    │                │  Copter 2    │
│  SYSID: 1    │                │  SYSID: 2    │
│  DDS: 2019   │                │  DDS: 2020   │
└──────┬───────┘                └──────┬───────┘
       │                               │
       │ UDP                           │ UDP
       │                               │
┌──────▼───────┐                ┌──────▼───────┐
│ ROS2 Agent   │                │ ROS2 Agent   │
│ Port 2019    │                │ Port 2020    │
└──────┬───────┘                └──────┬───────┘
       │                               │
       └───────────┬───────────────────┘
                   │
            ┌──────▼────────┐
            │   ROS2 Node   │
            │               │
            │ /vehicle_1/*  │
            │ /vehicle_2/*  │
            └───────────────┘
```

### Starting ROS2 Agents

**For 2 Copters:**

Terminal 1:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

Terminal 2:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2020
```

### ROS2 Topic Namespaces

Each vehicle publishes to its own namespace:

```bash
# Copter 1 (SYSID 1)
/vehicle_1/gps_global_origin/indoor
/vehicle_1/navsat/fix
/vehicle_1/imu/data

# Copter 2 (SYSID 2)
/vehicle_2/gps_global_origin/indoor
/vehicle_2/navsat/fix
/vehicle_2/imu/data
```

### Subscribing to Vehicle Data

```bash
# Listen to Copter 1 GPS
ros2 topic echo /vehicle_1/navsat/fix

# Listen to Copter 2 IMU
ros2 topic echo /vehicle_2/imu/data
```

---

## Mission Planner Connection Methods

### Method 1: Direct TCP (Recommended)

**For each vehicle, open separate Mission Planner instance:**

```
Window 1: TCP → localhost:5760 (Copter 1)
Window 2: TCP → localhost:5770 (Copter 2)
Window 3: TCP → localhost:5780 (Plane 1)
Window 4: TCP → localhost:5790 (VTOL 1)
```

**Steps:**
1. Open Mission Planner
2. Top-right dropdown: Select "TCP"
3. Click "TCP" → Enter `localhost:5760`
4. Click "Connect"
5. Repeat in new windows for other vehicles

**Pros:**
- Direct connection (lowest latency)
- Independent windows per vehicle
- Most reliable
- Simplest troubleshooting

**Cons:**
- Multiple Mission Planner instances (uses more RAM)
- Separate map views

---

### Method 2: UDP via MAVProxy

**Connect via MAVProxy UDP output:**

```
Mission Planner: UDP → 14550 (Copter 1)
Mission Planner: UDP → 14551 (Copter 2)
```

**Steps:**
1. Ensure `MAVPROXY_ENABLED=1` in .env.generated
2. Mission Planner → Select "UDP"
3. Port: `14550` (for Copter 1)
4. Click "Connect"

**Pros:**
- Can have multiple GCS on same vehicle
- MAVProxy features available
- UDP auto-discovery possible

**Cons:**
- Requires MAVProxy running
- Slightly higher latency
- More complex troubleshooting

---

### Method 3: Multi-Vehicle in One Mission Planner

**Mission Planner can show multiple vehicles on one map:**

1. Connect to first vehicle (TCP:5760)
2. Add additional connections via dropdown
3. Switch between vehicles with SYSID selector

**Note:** This feature depends on Mission Planner version.

---

## Complete Data Flow Diagrams

### Current Setup (Built-in SITL, Direct TCP)

```
┌────────────────────────────────────────────────────────┐
│                 Docker Container - Copter 1            │
│                                                        │
│  ┌──────────────────────────────────────────────┐    │
│  │         ArduPilot SITL (Instance 0)          │    │
│  │                                              │    │
│  │  - Built-in physics simulation               │    │
│  │  - MAVLink server on TCP 5760               │    │
│  │  - DDS client on UDP 2019                   │    │
│  │  - SYSID: 1                                 │    │
│  │                                              │    │
│  │  MAVProxy running (optional):               │    │
│  │    - Listening on TCP 5760                  │    │
│  │    - Broadcasting to UDP 14550              │    │
│  └──┬───────────────────────┬──────────────────┘    │
│     │                       │                        │
└─────┼───────────────────────┼────────────────────────┘
      │ TCP 5760              │ DDS UDP 2019
      │                       │
      ▼                       ▼
┌──────────────┐      ┌──────────────────┐
│   Mission    │      │  Micro ROS Agent │
│   Planner    │      │   (Port 2019)    │
│              │      │                  │
│ tcp:5760     │      │  └─→ ROS2 Node  │
└──────────────┘      │      /vehicle_1/* │
                      └──────────────────┘
```

### With MAVProxy Enabled

```
┌────────────────────────────────────────────────────────┐
│                 Docker Container - Copter 1            │
│                                                        │
│  ┌──────────────────────────────────────────────┐    │
│  │         ArduPilot SITL (Instance 0)          │    │
│  │  - MAVLink server on TCP 5760               │    │
│  │  - DDS client on UDP 2019                   │    │
│  └──┬───────────────────────┬──────────────────┘    │
│     │                       │                        │
│     │ TCP 5760              │ DDS UDP 2019          │
│     │                       │                        │
│  ┌──▼────────────────────┐  │                        │
│  │     MAVProxy          │  │                        │
│  │  - Master: tcp:5760   │  │                        │
│  │  - Out: udp:14550     │  │                        │
│  └──┬────────────────────┘  │                        │
│     │ UDP 14550             │                        │
└─────┼───────────────────────┼────────────────────────┘
      │                       │
      ▼                       ▼
┌──────────────┐      ┌──────────────────┐
│   Mission    │      │  Micro ROS Agent │
│   Planner    │      │   (Port 2019)    │
│              │      │                  │
│ udp:14550    │      │  └─→ ROS2 Node  │
│      OR      │      │      /vehicle_1/* │
│ tcp:5760     │      └──────────────────┘
└──────────────┘
```

---

## Common Use Cases

### Use Case 1: Development & Testing

**Scenario:** Testing multi-vehicle behaviors

**Setup:**
- Built-in SITL physics
- Direct TCP to Mission Planner
- MAVProxy disabled
- ROS2 for logging/analysis

**Configuration:**
```bash
COPTER1_MAVPROXY_ENABLED=0
COPTER2_MAVPROXY_ENABLED=0
```

**Connections:**
- Mission Planner: TCP:5760, TCP:5770
- ROS2: Agents on 2019, 2020

---

### Use Case 2: Multi-Operator Training

**Scenario:** Multiple operators watching same vehicle

**Setup:**
- Built-in SITL physics
- MAVProxy enabled
- Multiple GCS via UDP

**Configuration:**
```bash
COPTER1_MAVPROXY_ENABLED=1
COPTER1_MAVPROXY_OUT=127.0.0.1:14550
```

**Connections:**
- Instructor: UDP:14550
- Student 1: UDP:14550
- Student 2: UDP:14550
- All see the same vehicle data

---

### Use Case 3: Realistic Flight Testing

**Scenario:** Testing with RealFlight simulator

**Setup:**
- External RealFlight physics
- MAVProxy for sim control
- Mission Planner via TCP or UDP

**Configuration:**
```bash
SIM_ADDRESS=192.168.1.100  # RealFlight PC
COPTER1_MAVPROXY_ENABLED=1
COPTER1_MAVPROXY_SITL=127.0.0.1:5501  # Used for control
```

**Connections:**
- RealFlight: UDP to ArduPilot
- MAVProxy: Controls SITL port 5501
- Mission Planner: TCP:5760 or UDP:14550

---

### Use Case 4: ROS2 Autonomous Control

**Scenario:** ROS2 sends commands, Mission Planner monitors

**Setup:**
- Built-in SITL physics
- Direct TCP to Mission Planner
- ROS2 for control commands

**Configuration:**
```bash
COPTER1_MAVPROXY_ENABLED=0
```

**Connections:**
- Mission Planner: TCP:5760 (monitoring only)
- ROS2 Agent: UDP:2019 (sending commands)
- ROS2 node publishes to `/vehicle_1/cmd_vel`

---

## Troubleshooting Guide

### Problem: "Port 5760 already in use"

**Cause:** Another ArduPilot instance or GCS is using the port

**Solution:**
```bash
# Check what's using the port
sudo netstat -tulpn | grep 5760

# Kill the process or use different instance number
```

---

### Problem: "DDS: No ping response, disconnecting"

**Cause:** Multiple vehicles trying to use same DDS port

**Solution:**
- Ensure each vehicle has unique DDS port (2019, 2020, etc.)
- Start separate Micro ROS Agent for each vehicle

```bash
# Terminal 1
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# Terminal 2
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2020
```

---

### Problem: "Connection timeout" in Mission Planner

**Cause:** Wrong IP, wrong port, or ArduPilot not started

**Debug steps:**
```bash
# 1. Check if ArduPilot is running
docker ps

# 2. Check if port is listening
netstat -an | grep 5760

# 3. Check Docker logs
docker logs ardupilot-copter-1

# 4. Try telnet
telnet localhost 5760
```

---

### Problem: "MAVProxy not broadcasting"

**Cause:** MAVProxy disabled or wrong configuration

**Solution:**
1. Check `MAVPROXY_ENABLED=1` in .env.generated
2. Verify `MAVPROXY_OUT` is correct
3. Check Docker logs for MAVProxy errors
4. Ensure UDP port 14550+ is not blocked

---

### Problem: "ROS2 topics not appearing"

**Cause:** Micro ROS Agent not running or wrong port

**Solution:**
```bash
# 1. Check agent is running
ps aux | grep micro_ros_agent

# 2. Check port matches DDS_UDP_PORT
cat .env.generated | grep DDS_UDP_PORT

# 3. Restart agent on correct port
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# 4. Wait 10-15 seconds for connection
ros2 topic list | grep vehicle
```

---

### Problem: "Multiple vehicles showing same data"

**Cause:** Incorrect SYSID or DDS port configuration

**Solution:**
1. Verify each vehicle has unique SYSID (1, 2, 3, 4)
2. Verify each vehicle has unique DDS port (2019, 2020, 2021)
3. Check `.env.generated` for correct values
4. Restart containers

---

## Quick Reference

### Port Summary

| Purpose | Port Range | Usage |
|---------|------------|-------|
| MAVLink (TCP) | 5760-5790+ | Mission Planner, QGC |
| SITL Control | 5501-5531+ | Physics simulation |
| DDS (UDP) | 2019-2022+ | ROS2 communication |
| MAVProxy OUT (UDP) | 14550-14553+ | GCS broadcast |

### Parameter Quick Reference

| Parameter | Required When | Default |
|-----------|---------------|---------|
| MAVPROXY_ENABLED | Always set | 1 (enabled) |
| MAVPROXY_OUT | MAVProxy enabled | 127.0.0.1:14550+ |
| MAVPROXY_MASTER | MAVProxy enabled | tcp:127.0.0.1:5760+ |
| MAVPROXY_SITL | External sim only | 127.0.0.1:5501+ |

### Connection Quick Reference

| Use Case | Connection String | Port Type |
|----------|------------------|-----------|
| Mission Planner (Direct) | `tcp:localhost:5760` | MAVLink TCP |
| Mission Planner (MAVProxy) | `udp:localhost:14550` | MAVProxy UDP |
| ROS2 Agent | `udp4 -p 2019` | DDS UDP |

---

## Summary

### When is MAVPROXY_SITL Used?

✅ **Used when:**
- Running external simulators (RealFlight, X-Plane, Gazebo)
- Sending simulation control commands via MAVProxy
- Advanced testing with physics manipulation

❌ **NOT used when:**
- Built-in SITL physics (default)
- Direct TCP connection to Mission Planner
- Standard development/testing

### Default Setup (Your Current Configuration)

```
Built-in Physics: ✅ Yes
MAVProxy: ✅ Enabled (but can bypass)
Direct TCP: ✅ Primary connection method
ROS2: ✅ Separate DDS ports per vehicle
MAVPROXY_SITL: ⚠️  Defined but not actively used
```

### Best Practices

1. **Use direct TCP** for simplest, most reliable connections
2. **One Micro ROS Agent per vehicle** for stable DDS
3. **Unique SYSID** for each vehicle (1, 2, 3, 4...)
4. **Unique DDS port** for each vehicle (2019, 2020, 2021...)
5. **MAVProxy optional** - only enable if needed
6. **MAVPROXY_SITL** - ignore unless using external simulator

---

**Document Version:** 1.0
**Last Updated:** 2024
**For:** ArduPilot Multi-Vehicle Swarm System
