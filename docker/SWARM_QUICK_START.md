# Quick Reference: Swarm Spawn Locations & Single DDS Port

## ✅ Answer to Your Questions

### 1. Can vehicles spawn at different locations?
**YES!** Edit `.env.swarm` and uncomment one of the 4 formation patterns:

```bash
# PATTERN 1: Line Formation (East-West, 10m spacing)
COPTER1_HOME=35.363261,149.165230,584,0  # Origin
COPTER2_HOME=35.363261,149.165240,584,0  # 10m east
PLANE1_HOME=35.363261,149.165250,684,0   # 20m east, 100m higher
VTOL1_HOME=35.363261,149.165260,584,0    # 30m east
```

### 2. Can all vehicles connect to ONE DDS port?
**YES! This is now the default!** All vehicles share port 2019.

## How It Works

### Single Micro ROS Agent Setup

```bash
# Start ONE agent (instead of 4)
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### Vehicle Differentiation

Each vehicle has unique **System ID (SYSID)**:
- Copter 1: SYSID 1
- Copter 2: SYSID 2
- Plane 1: SYSID 3
- VTOL 1: SYSID 4

ROS2 automatically namespaces topics by vehicle.

## Configuration (.env.swarm)

```bash
# Single DDS port for ALL vehicles
DDS_UDP_PORT=2019

# Micro ROS Agent IP
DDS_IP0=127  # localhost
DDS_IP1=0
DDS_IP2=0
DDS_IP3=1
```

## Quick Start

```bash
# 1. Copy template
cp .env.swarm.example .env.swarm

# 2. Uncomment a spawn pattern (optional)
nano .env.swarm
# Example: Uncomment PATTERN 1 lines

# 3. Start ROS2 agent (ONE agent for all vehicles)
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# 4. Start swarm
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up
```

## Available Spawn Patterns

### PATTERN 1: Line Formation
Vehicles in a row, 10 meters apart (good for search patterns)

### PATTERN 2: Square Formation
Vehicles at corners of 20m square (good for area coverage)

### PATTERN 3: Diamond Formation
One leader, 3 followers at 15m distance (good for exploration)

### PATTERN 4: Staggered Altitude
Same position, different heights (good for testing altitude control)

## Port Summary

| Vehicle | Instance | MAVLink | SITL | DDS | SYSID |
|---------|----------|---------|------|-----|-------|
| Copter 1 | 0 | 5760 | 5501 | **2019** | 1 |
| Copter 2 | 1 | 5770 | 5511 | **2019** | 2 |
| Plane 1 | 2 | 5780 | 5521 | **2019** | 3 |
| VTOL 1 | 3 | 5790 | 5531 | **2019** | 4 |

**Note:** All share DDS port 2019!

## ROS2 Topics

With one agent on port 2019, you'll see all vehicles:

```bash
ros2 topic list

# Shows (example):
# /ap/battery
# /ap/clock
# /ap/pose/filtered
# ... (topics from all 4 vehicles)
```

Topics are differentiated by SYSID in the message data.

## Coordinate Reference

**Base Location:** Canberra, Australia (CMAC)
- Latitude: 35.363261
- Longitude: 149.165230
- Altitude: 584m MSL

**Offset Guide:**
- ~0.00001° longitude ≈ 10 meters east/west
- ~0.00001° latitude ≈ 11 meters north/south

## Example: Set Custom Formation

Edit `.env.swarm`:

```bash
# Triangle formation
COPTER1_HOME=35.363261,149.165230,584,0    # Point A
COPTER2_HOME=35.363261,149.165250,584,120  # Point B (20m east, facing 120°)
VTOL1_HOME=35.363271,149.165240,584,240    # Point C (10m north, 10m east, facing 240°)

# Disable plane-1 (only use 3 vehicles)
# Comment out plane-1 in docker-compose.swarm.yml or don't start it
```

Start only 3 vehicles:
```bash
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2 vtol-1
```

## Verification

```bash
# Check if all vehicles connected
docker-compose -f docker-compose.swarm.yml logs | grep "SYSID_THISMAV"

# Should show:
# copter-1: SYSID_THISMAV 1
# copter-2: SYSID_THISMAV 2
# plane-1: SYSID_THISMAV 3
# vtol-1: SYSID_THISMAV 4

# Check ROS2 connection
ros2 topic hz /ap/time
# Should show data from all connected vehicles
```

## Files

- `docker-compose.swarm.yml` - Multi-vehicle orchestration (updated)
- `.env.swarm.example` - Configuration template with patterns
- `.env.swarm` - Your configuration (create this)
- `docker/SWARM_SETUP.md` - Full documentation

## Troubleshooting

**Q: Vehicles spawn at same location?**
A: Uncomment HOME_LOCATION lines in `.env.swarm`

**Q: ROS2 topics not visible?**
A: Check one agent is running on port 2019, not multiple agents on different ports

**Q: Port conflict on 2019?**
A: Only ONE micro_ros_agent should be running, not four!
