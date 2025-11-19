# ArduPilot Multi-Vehicle Swarm - Complete Setup Guide

This comprehensive guide covers everything you need to run a multi-vehicle swarm with ArduPilot SITL, ROS2/DDS, and Mission Planner.

## Table of Contents
- [Overview](#overview)
- [Quick Start](#quick-start)
- [Step-by-Step Setup](#step-by-step-setup)
- [ENV File Configuration](#env-file-configuration)
- [Starting the Swarm](#starting-the-swarm)
- [ROS2 Connection](#ros2-connection)
- [Mission Planner Configuration](#mission-planner-configuration)
- [Spawn Location Patterns](#spawn-location-patterns)
- [Troubleshooting](#troubleshooting)

---

## Overview

### What You Get

**4 Different Vehicles Running Simultaneously:**
- **Copter 1** (Quadcopter) - SYSID 1, MAVLink Port 5760
- **Copter 2** (Hexacopter) - SYSID 2, MAVLink Port 5770
- **Plane 1** (Fixed Wing) - SYSID 3, MAVLink Port 5780
- **VTOL 1** (QuadPlane) - SYSID 4, MAVLink Port 5790

### Key Features
✅ **Single DDS Port** - All vehicles connect to ONE Micro ROS Agent (port 2019)
✅ **Different Spawn Locations** - 4 pre-configured formation patterns
✅ **Unique System IDs** - Each vehicle differentiated by SYSID
✅ **Mixed Vehicle Types** - Run copters, planes, and VTOLs together
✅ **Mission Planner Support** - Connect all vehicles to Mission Planner

---

## Quick Start

```bash
# 1. Copy environment template
cp .env.swarm.example .env.swarm

# 2. Create log directories with proper permissions
mkdir -p sitl_logs/copter-1 sitl_logs/copter-2 sitl_logs/plane-1 sitl_logs/vtol-1
chmod -R 777 sitl_logs

# 3. (Optional) Edit spawn locations - skip for default behavior
nano .env.swarm

# 4. Start ROS2 Micro ROS Agent (ONE agent for all vehicles)
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# 5. Start the swarm in Docker
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up --build
```

---

## Step-by-Step Setup

### Step 1: Prepare Environment File

Copy the template:
```bash
cd /path/to/ardupilot
cp .env.swarm.example .env.swarm
```

### Step 2: Create Log Directories

Create log directories with proper permissions before starting the swarm:
```bash
mkdir -p sitl_logs/copter-1 sitl_logs/copter-2 sitl_logs/plane-1 sitl_logs/vtol-1
chmod -R 777 sitl_logs
```

**Why?** The Docker containers need writable directories on the host for logs. Creating them beforehand ensures proper permissions.

### Step 3: Configure ENV File (Optional)

The default configuration works out of the box. To customize, edit `.env.swarm`:

```bash
nano .env.swarm
```

See [ENV File Configuration](#env-file-configuration) section below for details.

### Step 4: Start ROS2 Micro ROS Agent

**Important:** Start ONE agent for all vehicles (not 4 separate agents):

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Start the agent
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

Expected output:
```
[1234567890.123456] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 2019
[1234567890.123456] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### Step 5: Start the Swarm

In a new terminal:

```bash
# Start all vehicles
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up --build

# Or start specific vehicles only
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2
```

### Step 5: Verify Connection

**Check ROS2 Topics:**
```bash
ros2 topic list
```

Expected output:
```
/ap/airspeed
/ap/battery
/ap/clock
/ap/geopose/filtered
/ap/imu/experimental/data
/ap/navsat
/ap/pose/filtered
...
```

**Check Agent Connection:**
In the Micro ROS Agent terminal, you should see:
```
[...] info | ProxyClient.cpp | create_client | client created | client_key: 0x00000001, session_id: 0x81
[...] info | ProxyClient.cpp | create_client | client created | client_key: 0x00000002, session_id: 0x82
[...] info | ProxyClient.cpp | create_client | client created | client_key: 0x00000003, session_id: 0x83
[...] info | ProxyClient.cpp | create_client | client created | client_key: 0x00000004, session_id: 0x84
```

---

## ENV File Configuration

### Required Parameters

The `.env.swarm` file contains all configuration. Here's what you need to know:

### Global Settings

```bash
# Simulation speed
SPEEDUP=1  # 1 = real-time, 10 = 10x faster

# External simulator IP (for Gazebo)
SIM_ADDRESS=127.0.0.1

# Wipe EEPROM on startup
WIPE=0  # 0 = keep parameters, 1 = wipe and reset

# Synthetic clock
SYNTHETIC_CLOCK=0  # 0 = real clock, 1 = synthetic for faster simulation
```

### ROS2/DDS Settings (Single Port for All)

```bash
# Enable DDS
ENABLE_DDS=1

# SINGLE DDS PORT - All vehicles connect here
DDS_UDP_PORT=2019

# Micro ROS Agent IP (where agent is running)
DDS_IP0=127  # 127.0.0.1 = localhost
DDS_IP1=0
DDS_IP2=0
DDS_IP3=1

# ROS Domain ID (must match your ROS2 environment)
DDS_DOMAIN_ID=0

# Connection settings
DDS_TIMEOUT_MS=2000
DDS_MAX_RETRY=0  # 0 = unlimited retries
```

### Individual Vehicle Configuration

#### Copter 1 (Quadcopter)
```bash
COPTER1_MODEL=quad
COPTER1_SYSID=1
COPTER1_HOME=  # Leave empty for default location
```

#### Copter 2 (Hexacopter)
```bash
COPTER2_MODEL=hexa
COPTER2_SYSID=2
COPTER2_HOME=  # Leave empty or set custom location
```

#### Plane 1 (Fixed Wing)
```bash
PLANE1_MODEL=plane
PLANE1_SYSID=3
PLANE1_HOME=  # Leave empty or set custom location
```

#### VTOL 1 (QuadPlane)
```bash
VTOL1_MODEL=quadplane
VTOL1_SYSID=4
VTOL1_HOME=  # Leave empty or set custom location
```

### Changing Vehicle Models

You can change vehicle models to any supported type:

**ArduCopter Models:**
- `quad` - Quadcopter (X configuration)
- `hexa` - Hexacopter
- `octa` - Octocopter
- `tri` - Tricopter
- `y6` - Y6 configuration

**ArduPlane Models:**
- `plane` - Standard fixed-wing
- `quadplane` - VTOL QuadPlane
- `plane-elevon` - Elevon configuration
- `plane-vtail` - V-tail configuration

**Example - All Quadcopters:**
```bash
COPTER1_MODEL=quad
COPTER2_MODEL=quad
# Then either disable plane-1 and vtol-1 or change them:
PLANE1_MODEL=quad  # Changes plane to copter
VTOL1_MODEL=quad   # Changes VTOL to copter
```

---

## Starting the Swarm

### Option 1: Start All Vehicles

```bash
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up --build
```

This will:
1. Build ArduCopter for Copter 1 and Copter 2
2. Build ArduPlane for Plane 1 and VTOL 1
3. Start all 4 vehicles in parallel
4. Connect all vehicles to Micro ROS Agent on port 2019

### Option 2: Start Specific Vehicles

```bash
# Only start copters
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2

# Start copters and plane
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2 plane-1
```

### Option 3: Start in Background

```bash
# Start and detach
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up -d

# View logs
docker-compose -f docker-compose.swarm.yml logs -f

# Stop all
docker-compose -f docker-compose.swarm.yml down
```

### Build Options

**First Run (Full Build):**
```bash
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up --build
```

**Subsequent Runs (Skip Build):**
```bash
# Set SKIP_BUILD=1 in .env.swarm
SKIP_BUILD=1

# Or pass as environment variable
SKIP_BUILD=1 docker-compose -f docker-compose.swarm.yml up
```

---

## ROS2 Connection

### Single Agent Setup (Recommended)

**Start ONE Micro ROS Agent for all vehicles:**

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### Verify Connection

```bash
# List all topics
ros2 topic list

# Echo specific topic
ros2 topic echo /ap/pose/filtered

# Monitor topic rate
ros2 topic hz /ap/time

# List services
ros2 service list
```

### ROS2 Topic Examples

All vehicles publish to the same topic names. They are differentiated by the message data (SYSID field):

```bash
# Battery status (all vehicles)
ros2 topic echo /ap/battery

# Position (all vehicles)
ros2 topic echo /ap/pose/filtered

# GPS (all vehicles)
ros2 topic echo /ap/navsat

# IMU (all vehicles)
ros2 topic echo /ap/imu/experimental/data
```

### Filtering by Vehicle

To filter messages for a specific vehicle, use ROS2 message filters or check the SYSID in the message:

```python
# Python example
import rclpy
from sensor_msgs.msg import NavSatFix

def callback(msg):
    # Filter based on header frame_id or other identifier
    print(f"Received: {msg}")

# Subscribe to topic
subscription = node.create_subscription(
    NavSatFix,
    '/ap/navsat',
    callback,
    10)
```

### ROS2 Services (Control Vehicles)

```bash
# Arm Copter 1 (needs SYSID filtering in your ROS2 node)
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

# Switch mode
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"

# Check pre-arm status
ros2 service call /ap/prearm_check std_srvs/srv/Trigger
```

**Note:** For multi-vehicle control, you'll need to implement SYSID filtering in your ROS2 application to target specific vehicles.

---

## Mission Planner Configuration

### Connecting All Vehicles to Mission Planner

Mission Planner can connect to multiple vehicles simultaneously using different TCP connections.

### Step 1: Open Mission Planner

Launch Mission Planner on your Windows/Linux machine.

### Step 2: Connect First Vehicle (Copter 1)

1. In the top-right corner, find the connection dropdown
2. Select **TCP**
3. Click the connection settings icon
4. Enter:
   - **Host:** `localhost` (or Docker host IP if remote)
   - **Port:** `5760`
5. Click **Connect**

You should see Copter 1 connect with SYSID 1.

### Step 3: Add Additional Vehicles

Mission Planner supports viewing multiple vehicles:

**Method 1: Using Multiple Instances**

1. Start a second instance of Mission Planner
2. Connect to TCP `localhost:5770` (Copter 2)
3. Repeat for Plane (port 5780) and VTOL (port 5790)

**Method 2: Single Instance Multi-Vehicle (Advanced)**

Mission Planner can show multiple vehicles in one instance:

1. Connect to first vehicle (port 5760)
2. Go to **CONFIG → Full Parameter List**
3. The sidebar should show all connected vehicles with different SYSIDs
4. Click on each SYSID to switch between vehicles

### Connection Table

| Vehicle | SYSID | Connection String | Usage |
|---------|-------|-------------------|-------|
| Copter 1 | 1 | `tcp:localhost:5760` | Primary quadcopter |
| Copter 2 | 2 | `tcp:localhost:5770` | Secondary hexacopter |
| Plane 1 | 3 | `tcp:localhost:5780` | Fixed-wing aircraft |
| VTOL 1 | 4 | `tcp:localhost:5790` | VTOL quadplane |

### Map Display (All Vehicles)

In Mission Planner's Flight Data screen:
1. All connected vehicles will appear on the map
2. Each vehicle shows its own icon (plane, copter, etc.)
3. Different colors indicate different SYSIDs
4. Click on a vehicle icon to select it

### HUD Display

To switch between vehicles in HUD:
1. Use the dropdown menu near the top (shows SYSID)
2. Select the vehicle you want to view
3. HUD will update to show that vehicle's telemetry

### Mission Planning for Swarm

**Individual Missions:**
1. Connect to vehicle (e.g., port 5760 for Copter 1)
2. Go to **Flight Plan** tab
3. Create waypoint mission
4. Click **Write WPs** to upload to that vehicle
5. Repeat for each vehicle on their respective ports

**Coordinated Missions:**
For swarm coordination, you'll need to:
- Upload missions with synchronized timing
- Use ROS2 for high-level coordination
- Set different home locations for formation

### Arming Vehicles from Mission Planner

**Arm Single Vehicle:**
1. Select the vehicle (switch SYSID if needed)
2. Click **Actions** tab
3. Click **Arm/Disarm**
4. Vehicle will arm (check PreArm messages)

**Arm All Vehicles:**
You'll need to arm each vehicle individually by switching between their connections or using separate Mission Planner instances.

---

## Spawn Location Patterns

### Pattern 1: Line Formation (East-West)

Perfect for search patterns or convoy operations.

```bash
# Uncomment in .env.swarm:
COPTER1_HOME=35.363261,149.165230,584,0  # Origin
COPTER2_HOME=35.363261,149.165240,584,0  # 10m east
PLANE1_HOME=35.363261,149.165250,684,0   # 20m east, 100m higher
VTOL1_HOME=35.363261,149.165260,584,0    # 30m east
```

**Spacing:** 10 meters apart
**Formation:** ● ● ● ●
**Best For:** Search & rescue, area coverage

### Pattern 2: Square Formation

Perfect for area monitoring or perimeter patrol.

```bash
# Uncomment in .env.swarm:
COPTER1_HOME=35.363261,149.165230,584,0  # Southwest
COPTER2_HOME=35.363261,149.165250,584,0  # Southeast
PLANE1_HOME=35.363281,149.165230,684,0   # Northwest
VTOL1_HOME=35.363281,149.165250,584,0    # Northeast
```

**Spacing:** 20m x 20m square
**Formation:**
```
●        ●
   20m
●        ●
```
**Best For:** Perimeter security, area monitoring

### Pattern 3: Diamond Formation

Perfect for exploration with a leader vehicle.

```bash
# Uncomment in .env.swarm:
COPTER1_HOME=35.363261,149.165230,584,0   # Center (Leader)
COPTER2_HOME=35.363261,149.165215,584,90  # 15m west
PLANE1_HOME=35.363276,149.165230,684,180  # 15m north, higher altitude
VTOL1_HOME=35.363261,149.165245,584,270   # 15m east
```

**Spacing:** 15m from center
**Formation:**
```
       ●
       ^
   ●  ●  ●
    ←   →
```
**Best For:** Exploration, escort missions

### Pattern 4: Staggered Altitude

Perfect for vertical coverage or layered surveillance.

```bash
# Uncomment in .env.swarm:
COPTER1_HOME=35.363261,149.165230,584,0   # Ground level (584m)
COPTER2_HOME=35.363261,149.165230,604,0   # +20m altitude
PLANE1_HOME=35.363261,149.165230,634,0    # +50m altitude
VTOL1_HOME=35.363261,149.165230,664,0     # +80m altitude
```

**Spacing:** Vertical separation (20m, 50m, 80m)
**Formation:**
```
●  664m
●  634m
●  604m
●  584m
```
**Best For:** Multi-altitude operations, communications relay

### Custom Locations

To set your own spawn locations:

1. Choose base location (latitude, longitude, altitude MSL)
2. Calculate offsets:
   - **~0.00001° longitude ≈ 10 meters east/west** (at mid-latitudes)
   - **~0.00001° latitude ≈ 11 meters north/south**
3. Set heading (0-359 degrees, 0=North, 90=East, 180=South, 270=West)

**Example - 50m Triangle:**
```bash
# Vertex A (south)
COPTER1_HOME=35.363261,149.165230,584,0

# Vertex B (northeast, 50m away)
# Calculate: 50m = ~0.00005° at mid-latitudes
COPTER2_HOME=35.363311,149.165280,584,120

# Vertex C (northwest, 50m away)
VTOL1_HOME=35.363311,149.165180,584,240
```

---

## Troubleshooting

### Issue 1: Vehicles Not Connecting to ROS2

**Symptoms:**
- `ros2 topic list` shows no topics
- Micro ROS Agent shows no client connections

**Solution:**
```bash
# 1. Check agent is running
ps aux | grep micro_ros_agent

# 2. Check port 2019 is listening
netstat -tuln | grep 2019

# 3. Check DDS configuration in logs
docker-compose -f docker-compose.swarm.yml logs | grep DDS_UDP_PORT

# Should show: DDS_UDP_PORT 2019

# 4. Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 0 (or match DDS_DOMAIN_ID in .env.swarm)

# 5. Restart agent with verbose logging
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019 -v 6
```

### Issue 2: Vehicles Spawn at Same Location

**Symptoms:**
- All vehicles appear at same GPS coordinates
- Vehicles collide in simulation

**Solution:**
```bash
# Edit .env.swarm and uncomment one of the PATTERN sections:
nano .env.swarm

# Uncomment Pattern 1 for line formation:
COPTER1_HOME=35.363261,149.165230,584,0
COPTER2_HOME=35.363261,149.165240,584,0
PLANE1_HOME=35.363261,149.165250,684,0
VTOL1_HOME=35.363261,149.165260,584,0

# Restart swarm
docker-compose -f docker-compose.swarm.yml down
docker-compose -f docker-compose.swarm.yml up
```

### Issue 3: Port Already in Use

**Symptoms:**
- Error: `Address already in use`
- Container fails to start

**Solution:**
```bash
# Check what's using the ports
netstat -tuln | grep -E '5760|5770|5780|5790|2019'

# Kill any existing processes
# Find PID
lsof -ti:5760
# Kill it
kill -9 <PID>

# Or stop all containers
docker-compose -f docker-compose.swarm.yml down

# Then restart
docker-compose -f docker-compose.swarm.yml up
```

### Issue 4: Mission Planner Not Connecting

**Symptoms:**
- Connection timeout
- No heartbeat

**Solution:**
```bash
# 1. Verify SITL is running
docker-compose -f docker-compose.swarm.yml ps

# 2. Check MAVLink ports are listening
netstat -tuln | grep -E '5760|5770|5780|5790'

# 3. Test connection with MAVProxy first
mavproxy.py --master=tcp:127.0.0.1:5760

# 4. Check firewall (if Docker host is remote)
sudo ufw allow 5760/tcp
sudo ufw allow 5770/tcp
sudo ufw allow 5780/tcp
sudo ufw allow 5790/tcp

# 5. In Mission Planner, try UDP instead of TCP
# Use UDP port 14550, 14560, 14570, 14580
```

### Issue 5: PreArm: Logging Failed

**Symptoms:**
- Cannot arm vehicles
- PreArm check fails with "Logging failed"

**Solution:**
This is already fixed in the current configuration, but if you see it:

```bash
# Check logs directory exists
docker-compose -f docker-compose.swarm.yml exec copter-1 ls -la /ardupilot/logs

# Verify LOG parameters in generated config
docker-compose -f docker-compose.swarm.yml logs copter-1 | grep LOG_

# Should show:
# LOG_DISARMED 1
# LOG_BACKEND_TYPE 1

# If not, restart with WIPE=1 to reset parameters
WIPE=1 docker-compose -f docker-compose.swarm.yml up
```

### Issue 6: Build Failures

**Symptoms:**
- `waf configure` fails
- Submodule errors

**Solution:**
```bash
# Clean build and try again
docker-compose -f docker-compose.swarm.yml down
docker-compose -f docker-compose.swarm.yml build --no-cache

# Or manually initialize submodules on host
git submodule update --init --recursive

# Then rebuild
docker-compose -f docker-compose.swarm.yml up --build
```

### Issue 7: Different Vehicles Same SYSID

**Symptoms:**
- Mission Planner shows only one vehicle
- Vehicles interfere with each other

**Solution:**
```bash
# Verify SYSID configuration in .env.swarm
grep SYSID .env.swarm

# Should show:
# COPTER1_SYSID=1
# COPTER2_SYSID=2
# PLANE1_SYSID=3
# VTOL1_SYSID=4

# If incorrect, fix and restart
nano .env.swarm
docker-compose -f docker-compose.swarm.yml restart
```

---

## Advanced Configuration

### Running More Than 4 Vehicles

To add a 5th vehicle (e.g., Rover):

1. **Edit `docker-compose.swarm.yml`:**
   - Copy the `copter-1` service
   - Rename to `rover-1`
   - Change `INSTANCE=4`, `SYSID=5`
   - Ports: MAVLink 5800, SITL 5541
   - Set `VEHICLE=ArduRover`, `MODEL=rover`

2. **Edit `.env.swarm`:**
   ```bash
   ROVER1_MODEL=rover
   ROVER1_SYSID=5
   ROVER1_HOME=35.363261,149.165270,584,0
   ```

3. **Add volume:**
   ```yaml
   volumes:
     rover-1-eeprom:
       driver: local
   ```

### Different Speed Settings Per Vehicle

Currently all vehicles share `SPEEDUP` setting. To customize per vehicle, you would need to modify the docker-compose.swarm.yml to accept individual speedup variables.

### Remote Gazebo Integration

To connect swarm to remote Gazebo:

```bash
# In .env.swarm, set Gazebo host IP:
SIM_ADDRESS=192.168.1.100

# On Gazebo host, allow connections:
sudo ufw allow 5501/udp
sudo ufw allow 5511/udp
sudo ufw allow 5521/udp
sudo ufw allow 5531/udp
```

---

## Summary

### What We Covered

✅ **ENV Configuration** - All parameters explained
✅ **Starting the Swarm** - Multiple start options
✅ **ROS2 Connection** - Single agent for all vehicles
✅ **Mission Planner** - Multi-vehicle connection guide
✅ **Spawn Patterns** - 4 ready-to-use formations
✅ **Troubleshooting** - Common issues and fixes

### Quick Reference Card

```
┌─────────────────────────────────────────────────────────────┐
│              ArduPilot Swarm Quick Reference                │
├─────────────┬────────┬─────────┬──────┬──────────┬──────────┤
│ Vehicle     │ SYSID  │ MAVLink │ SITL │ DDS Port │ Model    │
├─────────────┼────────┼─────────┼──────┼──────────┼──────────┤
│ Copter 1    │   1    │  5760   │ 5501 │   2019   │ quad     │
│ Copter 2    │   2    │  5770   │ 5511 │   2019   │ hexa     │
│ Plane 1     │   3    │  5780   │ 5521 │   2019   │ plane    │
│ VTOL 1      │   4    │  5790   │ 5531 │   2019   │ quadplane│
└─────────────┴────────┴─────────┴──────┴──────────┴──────────┘

Start Commands:
  ROS2 Agent:  ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
  Docker:      docker-compose -f docker-compose.swarm.yml up --build

Mission Planner Connections:
  Copter 1:    tcp:localhost:5760
  Copter 2:    tcp:localhost:5770
  Plane 1:     tcp:localhost:5780
  VTOL 1:      tcp:localhost:5790
```

### Files Reference

- **docker-compose.swarm.yml** - Multi-vehicle orchestration
- **.env.swarm.example** - Configuration template
- **.env.swarm** - Your configuration (create this)
- **docker/SWARM_SETUP.md** - Detailed technical docs
- **docker/SWARM_QUICK_START.md** - Quick reference
- **docker/SWARM_COMPLETE_GUIDE.md** - This file

### Next Steps

1. ✅ Configure `.env.swarm` with your desired settings
2. ✅ Start Micro ROS Agent
3. ✅ Launch swarm with Docker Compose
4. ✅ Connect Mission Planner to each vehicle
5. ✅ Upload missions and fly!

**Enjoy flying your swarm! 🚁✈️🚁**
