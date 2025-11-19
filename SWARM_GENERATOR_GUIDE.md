# ArduPilot Swarm Generator - Complete Guide

The `generate_swarm.sh` script automatically generates Docker Compose configurations for multi-vehicle swarms with **any number** of copters, planes, and VTOLs.

## Why Use This Script?

**Before (Manual):**
- Edit docker-compose.yml for each vehicle
- Manually calculate ports and SYSID
- Create log directories by hand
- Edit .env file for each vehicle
- Easy to make mistakes with port conflicts

**After (Automated):**
```bash
./generate_swarm.sh --copters=5 --planes=2 --vtols=1
```
- ✅ All files generated automatically
- ✅ Ports and SYSIDs calculated correctly
- ✅ Log directories created with proper permissions
- ✅ ENV file configured for all vehicles
- ✅ Ready to run immediately

---

## Quick Start

### 1. Basic Usage

```bash
# Generate swarm with 4 copters
./generate_swarm.sh --copters=4

# Generate swarm with mixed vehicles
./generate_swarm.sh --copters=2 --planes=1 --vtols=1

# Generate and auto-start
./generate_swarm.sh --copters=3 --start
```

### 2. Start ROS2 Agent

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### 3. Start the Swarm

```bash
docker-compose -f docker-compose-generated.yml --env-file .env.generated up
```

---

## Command Line Options

| Option | Description | Default | Example |
|--------|-------------|---------|---------|
| `--copters=N` | Number of copter instances | 0 | `--copters=4` |
| `--planes=N` | Number of plane instances | 0 | `--planes=2` |
| `--vtols=N` | Number of VTOL instances | 0 | `--vtols=1` |
| `--start` | Auto-start swarm after generation | false | `--start` |
| `--help` | Show help message | - | `--help` |

---

## What Gets Generated

### 1. `docker-compose-generated.yml`
- Complete Docker Compose configuration
- One service per vehicle
- Proper dependencies (copters build first, planes reuse build)
- Unique ports for each vehicle
- Separate log volumes per vehicle

### 2. `.env.generated`
- Global settings (speedup, DDS config)
- Per-vehicle configuration (MODEL, SYSID, HOME location)
- Spawn locations (automatically offset 10m apart)

### 3. `sitl_logs/` Directory Structure
```
sitl_logs/
├── copter-1/
├── copter-2/
├── plane-1/
└── vtol-1/
```
Each vehicle gets its own log directory with 777 permissions.

---

## Port Allocation

The script automatically assigns ports based on instance number:

**Formula:**
- MAVLink Port: `5760 + (instance * 10)`
- SITL Port: `5501 + (instance * 10)`
- DDS Port: `2019` (shared by all)

**Example with `--copters=2 --planes=1 --vtols=1`:**

| Vehicle | Instance | SYSID | MAVLink Port | SITL Port | DDS Port |
|---------|----------|-------|--------------|-----------|----------|
| Copter 1 | 0 | 1 | 5760 | 5501 | 2019 |
| Copter 2 | 1 | 2 | 5770 | 5511 | 2019 |
| Plane 1 | 2 | 3 | 5780 | 5521 | 2019 |
| VTOL 1 | 3 | 4 | 5790 | 5531 | 2019 |

---

## Spawn Locations

Vehicles are automatically spawned in a **line formation** (east-west):

**Default Location:** CMAC, Canberra, Australia
- **Latitude:** 35.363261° S
- **Longitude:** 149.165230° E
- **Altitude:** 584m

**Spacing:** 10 meters apart (approximately 0.00001° longitude)

**Example Layout:**
```
West ←─────────────────────────────────→ East
     Copter1  Copter2  Plane1  VTOL1
        ●────10m─●────10m─●────10m─●
```

**Planes start 100m higher** than copters/VTOLs for safety.

### Customizing Spawn Locations

After generation, edit `.env.generated`:

```bash
# Format: latitude,longitude,altitude,heading
COPTER1_HOME=35.363261,149.165230,584,0
COPTER2_HOME=35.363261,149.165240,584,0  # 10m east
PLANE1_HOME=35.363261,149.165250,684,0   # 20m east, 100m higher
```

**Common Patterns:**

**Square Formation (20m sides):**
```bash
COPTER1_HOME=35.363261,149.165230,584,0  # Southwest
COPTER2_HOME=35.363261,149.165250,584,0  # Southeast
COPTER3_HOME=35.363441,149.165230,584,0  # Northwest
COPTER4_HOME=35.363441,149.165250,584,0  # Northeast
```

**Diamond Formation:**
```bash
COPTER1_HOME=35.363261,149.165240,584,0  # Center
COPTER2_HOME=35.363261,149.165250,584,0  # East
COPTER3_HOME=35.363361,149.165240,584,0  # North
COPTER4_HOME=35.363261,149.165230,584,0  # West
```

---

## Complete Workflow Examples

### Example 1: Simple Copter Swarm

```bash
# Generate 5 copters
./generate_swarm.sh --copters=5

# Start ROS2 agent
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# Start swarm
docker-compose -f docker-compose-generated.yml --env-file .env.generated up

# Connect Mission Planner to each:
# tcp:localhost:5760 (Copter 1)
# tcp:localhost:5770 (Copter 2)
# tcp:localhost:5780 (Copter 3)
# tcp:localhost:5790 (Copter 4)
# tcp:localhost:5800 (Copter 5)
```

### Example 2: Mixed Fleet

```bash
# Generate 3 copters, 2 planes, 1 VTOL
./generate_swarm.sh --copters=3 --planes=2 --vtols=1

# Edit spawn locations if needed
nano .env.generated

# Start ROS2 agent
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# Start swarm
docker-compose -f docker-compose-generated.yml --env-file .env.generated up
```

### Example 3: Quick Test (Auto-Start)

```bash
# Terminal 1: Start ROS2 agent
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# Terminal 2: Generate and auto-start
./generate_swarm.sh --copters=2 --start
```

---

## ROS2 Integration

### Single Agent for All Vehicles

All vehicles connect to **ONE** Micro ROS Agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### Topic Namespaces

Each vehicle publishes to its own namespace based on SYSID:

```bash
# List all topics
ros2 topic list

# Expected output:
/vehicle_1/gps_global_origin/...
/vehicle_1/navsat/fix
/vehicle_2/gps_global_origin/...
/vehicle_2/navsat/fix
...
```

### Subscribe to Specific Vehicle

```bash
# Listen to Copter 1 GPS
ros2 topic echo /vehicle_1/navsat/fix

# Listen to Plane 1 GPS
ros2 topic echo /vehicle_3/navsat/fix
```

---

## Mission Planner Configuration

### Connecting Multiple Vehicles

**Option 1: Separate Windows**
- Open Mission Planner
- Connect to `tcp:localhost:5760` (Vehicle 1)
- Open new Mission Planner instance
- Connect to `tcp:localhost:5770` (Vehicle 2)
- Repeat for each vehicle

**Option 2: Multi-Vehicle View**
- File → New Window
- Each window connects to different port
- View all vehicles on same map

### Port Reference

The script outputs port assignments when you run it:

```
Port Assignments:
  Copter 1: MAVLink 5760, SYSID 1
  Copter 2: MAVLink 5770, SYSID 2
  Plane 1:  MAVLink 5780, SYSID 3
  VTOL 1:   MAVLink 5790, SYSID 4
```

---

## Stopping and Restarting

### Stop the Swarm

```bash
# Ctrl+C in the docker-compose terminal
# Or in another terminal:
docker-compose -f docker-compose-generated.yml down
```

### Restart with Same Configuration

```bash
# Just restart (preserves EEPROM/parameters)
docker-compose -f docker-compose-generated.yml --env-file .env.generated up
```

### Wipe and Restart

Edit `.env.generated`:
```bash
WIPE=1  # Change from 0 to 1
```

Then restart:
```bash
docker-compose -f docker-compose-generated.yml --env-file .env.generated up
```

### Generate New Configuration

Simply run the script again:
```bash
./generate_swarm.sh --copters=6  # Overwrites previous config
```

---

## Advanced Configuration

### Change Simulation Speed

Edit `.env.generated`:
```bash
SPEEDUP=2  # 2x speed
SPEEDUP=4  # 4x speed (may be unstable)
```

### Use Custom DDS Port

Edit `.env.generated`:
```bash
DDS_UDP_PORT=2020  # Change from 2019
```

Then update your ROS2 agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2020
```

### Different Vehicle Models

Edit `.env.generated`:
```bash
COPTER1_MODEL=quad     # Quadcopter (default)
COPTER2_MODEL=hexa     # Hexacopter
COPTER3_MODEL=octa     # Octacopter
COPTER4_MODEL=X        # X configuration

PLANE1_MODEL=plane     # Standard plane (default)
PLANE2_MODEL=quadplane # VTOL plane

VTOL1_MODEL=quadplane  # QuadPlane (default)
```

---

## Troubleshooting

### Error: "Must specify at least one vehicle"

You didn't provide any vehicle counts:
```bash
# Wrong
./generate_swarm.sh

# Correct
./generate_swarm.sh --copters=1
```

### Error: "Permission denied" on logs

The script creates log directories with 777 permissions automatically. If you still see this:
```bash
sudo chmod -R 777 sitl_logs
```

### Vehicles Not Building

Check if git submodules are initialized:
```bash
git submodule update --init --recursive
```

### Port Conflicts

If you're running other services:
```bash
# Check what's using port 5760
sudo netstat -tulpn | grep 5760

# Kill the process or generate with different vehicles
```

### ROS2 Topics Not Appearing

1. Verify agent is running:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

2. Check DDS is enabled in `.env.generated`:
```bash
ENABLE_DDS=1
DDS_UDP_PORT=2019
```

3. Wait 10-15 seconds after vehicle startup for connection

---

## Performance Considerations

### System Requirements Per Vehicle

**Minimum:**
- CPU: 1 core @ 2 GHz
- RAM: 512 MB
- Disk: 100 MB

**Recommended for 4 Vehicles:**
- CPU: 4+ cores @ 2.5 GHz
- RAM: 4+ GB
- Disk: 1+ GB

### Large Swarms (10+ Vehicles)

For 10+ vehicles:

1. **Increase speedup** to reduce CPU load:
```bash
SPEEDUP=2  # or higher
```

2. **Disable logging** for some vehicles in `.env.generated`:
```bash
# Add these to specific vehicle param files in docker-compose-generated.yml
LOG_DISARMED 0
LOG_BACKEND_TYPE 0
```

3. **Use SSD** for faster builds and logs

4. **Monitor CPU usage**:
```bash
htop  # Watch CPU per container
```

---

## Files Reference

### Generated by Script

| File | Purpose | Edit? |
|------|---------|-------|
| `docker-compose-generated.yml` | Container configuration | ❌ Regenerate instead |
| `.env.generated` | Environment variables | ✅ Safe to edit |
| `sitl_logs/` | Vehicle logs | ❌ Auto-managed |

### Original Files (Preserved)

| File | Purpose |
|------|---------|
| `docker-compose.swarm.yml` | Original 4-vehicle config |
| `.env.swarm.example` | Original env template |
| `generate_swarm.sh` | This generator script |

The script creates **new** files and doesn't modify your original static configuration.

---

## Best Practices

### 1. **Always Start ROS2 Agent First**
Start the Micro ROS Agent before starting Docker containers.

### 2. **Use Meaningful Counts**
Start small and scale up:
```bash
# Start with 2-3 vehicles
./generate_swarm.sh --copters=2

# Scale up after testing
./generate_swarm.sh --copters=5
```

### 3. **Check Logs Regularly**
```bash
# View logs for specific vehicle
cat sitl_logs/copter-1/ArduCopter.log

# Or monitor in real-time (shown in docker-compose output)
```

### 4. **Version Control Your ENV**
```bash
# Save your custom configuration
cp .env.generated .env.my_mission

# Restore later
cp .env.my_mission .env.generated
```

### 5. **Document Your Setup**
```bash
# Add comment to .env.generated
# Formation: Diamond pattern for search mission
# Created: 2024-01-15
# Mission: Area surveillance
```

---

## Examples Library

### Search and Rescue (Grid Pattern)

```bash
./generate_swarm.sh --copters=9

# Edit .env.generated for 3x3 grid:
COPTER1_HOME=35.363200,149.165200,584,0
COPTER2_HOME=35.363200,149.165230,584,0
COPTER3_HOME=35.363200,149.165260,584,0
COPTER4_HOME=35.363300,149.165200,584,0
COPTER5_HOME=35.363300,149.165230,584,0
COPTER6_HOME=35.363300,149.165260,584,0
COPTER7_HOME=35.363400,149.165200,584,0
COPTER8_HOME=35.363400,149.165230,584,0
COPTER9_HOME=35.363400,149.165260,584,0
```

### Aerial Imaging (Line Formation)

```bash
./generate_swarm.sh --copters=5

# Already in line formation by default!
# Adjust altitude for different perspective:
# Edit .env.generated and add 50m to altitude
```

### Delivery Fleet (Mixed)

```bash
./generate_swarm.sh --copters=3 --vtols=2

# Copters for short range
# VTOLs for long range deliveries
```

---

## Next Steps

1. **Generate your first swarm:**
   ```bash
   ./generate_swarm.sh --copters=2
   ```

2. **Start ROS2 and Docker**

3. **Connect Mission Planner**

4. **Fly your swarm!**

For more help, see:
- `SWARM_COMPLETE_GUIDE.md` - Original 4-vehicle static guide
- `docker/SWARM_QUICK_START.md` - Quick reference
- `Claude.md` - Docker and ROS2 basics

Happy swarming! 🚁🚁🚁
