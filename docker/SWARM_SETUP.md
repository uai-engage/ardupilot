# ArduPilot Multi-Vehicle Swarm Setup

This guide covers running multiple ArduPilot SITL instances simultaneously in Docker, enabling swarm simulation with different vehicle types.

## Quick Start

```bash
# 1. Copy swarm environment template
cp .env.swarm.example .env.swarm

# 2. Edit configuration (optional)
nano .env.swarm

# 3. Start all vehicles
docker-compose -f docker-compose.swarm.yml --env-file .env.swarm up --build

# Or start specific vehicles
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2
```

## Default Swarm Configuration

The default setup includes:

| Vehicle | Type | Instance | SYSID | MAVLink | SITL | DDS |
|---------|------|----------|-------|---------|------|-----|
| copter-1 | Quadcopter | 0 | 1 | 5760 | 5501 | 2019 |
| copter-2 | Hexacopter | 1 | 2 | 5770 | 5511 | 2029 |
| plane-1 | Fixed Wing | 2 | 3 | 5780 | 5521 | 2039 |
| vtol-1 | QuadPlane | 3 | 4 | 5790 | 5531 | 2049 |

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    Host Machine                          │
│                                                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  Copter 1   │  │  Copter 2   │  │   Plane 1   │    │
│  │ Instance 0  │  │ Instance 1  │  │ Instance 2  │    │
│  │ Port 5760   │  │ Port 5770   │  │ Port 5780   │    │
│  │ DDS 2019    │  │ DDS 2029    │  │ DDS 2039    │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
│         │                │                │            │
│         └────────────────┼────────────────┘            │
│                          │                             │
│                  ┌───────▼───────┐                     │
│                  │   ROS2 Agent  │                     │
│                  │  (Multi-Port) │                     │
│                  └───────────────┘                     │
└──────────────────────────────────────────────────────────┘
```

## Port Allocation

Ports are automatically offset by `10 * instance_number`:

### Instance 0 (Copter 1)
- MAVLink: 5760 (TCP)
- SITL Sim: 5501 (UDP)
- DDS: 2019 (UDP)
- GCS: 14550 (UDP)

### Instance 1 (Copter 2)
- MAVLink: 5770 (TCP)
- SITL Sim: 5511 (UDP)
- DDS: 2029 (UDP)
- GCS: 14560 (UDP)

### Instance 2 (Plane 1)
- MAVLink: 5780 (TCP)
- SITL Sim: 5521 (UDP)
- DDS: 2039 (UDP)
- GCS: 14570 (UDP)

### Instance 3 (VTOL 1)
- MAVLink: 5790 (TCP)
- SITL Sim: 5531 (UDP)
- DDS: 2049 (UDP)
- GCS: 14580 (UDP)

## ROS2 Multi-Vehicle Connection

Each vehicle connects to ROS2 on a unique DDS port. You need to run multiple Micro ROS Agents:

### Option 1: Separate Agent Processes

```bash
# Terminal 1 - Copter 1
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019

# Terminal 2 - Copter 2
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2029

# Terminal 3 - Plane 1
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2039

# Terminal 4 - VTOL 1
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2049
```

### Option 2: Script to Start All Agents

```bash
#!/bin/bash
# start_swarm_agents.sh

source /opt/ros/humble/setup.bash

# Start agents in background
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019 > agent_copter1.log 2>&1 &
echo "Copter 1 agent started (PID $!)"

ros2 run micro_ros_agent micro_ros_agent udp4 -p 2029 > agent_copter2.log 2>&1 &
echo "Copter 2 agent started (PID $!)"

ros2 run micro_ros_agent micro_ros_agent udp4 -p 2039 > agent_plane1.log 2>&1 &
echo "Plane 1 agent started (PID $!)"

ros2 run micro_ros_agent micro_ros_agent udp4 -p 2049 > agent_vtol1.log 2>&1 &
echo "VTOL 1 agent started (PID $!)"

echo "All agents started. Press Ctrl+C to stop."
wait
```

### Verify ROS2 Topics

```bash
# List all topics (should show /ap/copter1/*, /ap/copter2/*, etc.)
ros2 topic list

# Echo specific vehicle's position
ros2 topic echo /ap/pose/filtered
```

## Connecting Ground Control Stations

### QGroundControl

1. Open QGroundControl
2. Go to **Application Settings → Comm Links**
3. Add links for each vehicle:
   - **Copter 1**: TCP, localhost:5760
   - **Copter 2**: TCP, localhost:5770
   - **Plane 1**: TCP, localhost:5780
   - **VTOL 1**: TCP, localhost:5790

### Mission Planner

1. Connection dropdown → TCP
2. Enter: `localhost:5760` for Copter 1
3. Repeat for other vehicles on their respective ports

### MAVProxy (Multiple Instances)

```bash
# Terminal 1 - Copter 1
mavproxy.py --master=tcp:127.0.0.1:5760

# Terminal 2 - Copter 2
mavproxy.py --master=tcp:127.0.0.1:5770

# Terminal 3 - Plane 1
mavproxy.py --master=tcp:127.0.0.1:5780

# Terminal 4 - VTOL 1
mavproxy.py --master=tcp:127.0.0.1:5790
```

## Configuration Examples

### Example 1: All Quadcopters (4 identical copters)

Edit `.env.swarm`:
```bash
COPTER1_MODEL=quad
COPTER1_SYSID=1
COPTER1_HOME=35.363261,149.165230,584,0

COPTER2_MODEL=quad
COPTER2_SYSID=2
COPTER2_HOME=35.363261,149.165240,584,0  # 10m east

# Modify docker-compose.swarm.yml to add copter-3 and copter-4
# Or just run 2 copters
```

Start only copters:
```bash
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2
```

### Example 2: Mixed Fleet (Different vehicle types)

```bash
COPTER1_MODEL=quad       # Fast delivery
COPTER1_SYSID=1

COPTER2_MODEL=hexa       # Heavy lift
COPTER2_SYSID=2

PLANE1_MODEL=plane       # Long range surveillance
PLANE1_SYSID=3

VTOL1_MODEL=quadplane    # Versatile operations
VTOL1_SYSID=4
```

### Example 3: Formation Flying

Set home locations in formation:
```bash
# Line formation (10m spacing)
COPTER1_HOME=35.363261,149.165230,584,0
COPTER2_HOME=35.363261,149.165240,584,0
COPTER3_HOME=35.363261,149.165250,584,0
COPTER4_HOME=35.363261,149.165260,584,0

# Diamond formation
COPTER1_HOME=35.363261,149.165230,584,0  # Front
COPTER2_HOME=35.363251,149.165240,584,0  # Left
COPTER3_HOME=35.363271,149.165240,584,0  # Right
COPTER4_HOME=35.363261,149.165250,584,0  # Rear
```

## Managing Individual Vehicles

### Start specific vehicles

```bash
# Only copters
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2

# Only one vehicle
docker-compose -f docker-compose.swarm.yml up copter-1

# All except VTOL
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2 plane-1
```

### Stop specific vehicle

```bash
docker-compose -f docker-compose.swarm.yml stop copter-2
```

### View logs for specific vehicle

```bash
# Follow logs
docker-compose -f docker-compose.swarm.yml logs -f copter-1

# View all logs
docker-compose -f docker-compose.swarm.yml logs

# Logs are also saved to ./sitl_logs/
tail -f ./sitl_logs/copter-1/ArduCopter.log
```

### Restart single vehicle

```bash
docker-compose -f docker-compose.swarm.yml restart copter-1
```

## Adding More Vehicles

To add a 5th vehicle (e.g., Rover):

1. Edit `docker-compose.swarm.yml`:

```yaml
  rover-1:
    build:
      context: .
      dockerfile: Dockerfile
      # ... (copy from copter-1 and modify)
    container_name: ardupilot-rover-1
    environment:
      - VEHICLE=ArduRover
      - MODEL=rover
      - INSTANCE=4
      - SYSID=5
      - DDS_UDP_PORT=2059  # 2019 + 40
    # ... rest of config
```

2. Update `.env.swarm`:

```bash
ROVER1_MODEL=rover
ROVER1_SYSID=5
ROVER1_HOME=35.363261,149.165230,584,0
```

3. Add volume:

```yaml
volumes:
  rover-1-eeprom:
    driver: local
```

## Troubleshooting

### Issue: Port Conflicts

**Error**: `Address already in use`

**Solution**: Check which ports are in use:
```bash
netstat -tuln | grep -E '5760|5770|5780|5790|2019|2029|2039|2049'
```

Kill any processes using those ports or change INSTANCE numbers.

### Issue: Vehicles Not Building

**Solution**: Build vehicles sequentially:
```bash
# Build copter first
docker-compose -f docker-compose.swarm.yml up copter-1
# Ctrl+C after build

# Then build plane
docker-compose -f docker-compose.swarm.yml up plane-1
# Ctrl+C after build

# Now start all
docker-compose -f docker-compose.swarm.yml up
```

### Issue: ROS2 Topics Not Visible

**Solution**:
1. Verify agents are running on all ports:
```bash
netstat -tuln | grep -E '2019|2029|2039|2049'
```

2. Check DDS_DOMAIN_ID matches:
```bash
echo $ROS_DOMAIN_ID  # Should match DDS_DOMAIN_ID in .env.swarm
```

3. Check vehicle logs:
```bash
docker-compose -f docker-compose.swarm.yml logs copter-1 | grep -i dds
```

### Issue: Vehicles Have Same Position

**Solution**: Set different HOME locations in `.env.swarm`:
```bash
COPTER1_HOME=35.363261,149.165230,584,0
COPTER2_HOME=35.363261,149.165240,584,0  # Offset east
```

### Issue: Memory/CPU High Usage

**Solution**:
1. Reduce SPEEDUP:
```bash
SPEEDUP=1  # Run at real-time
```

2. Run fewer vehicles:
```bash
docker-compose -f docker-compose.swarm.yml up copter-1 copter-2
```

3. Disable DDS if not needed:
```bash
ENABLE_DDS=0
```

## Performance Optimization

### Build Once, Run Many

The docker-compose.swarm.yml is configured so that:
- `copter-1` builds ArduCopter
- `copter-2` reuses copter-1's build (SKIP_BUILD=1)
- `plane-1` builds ArduPlane
- `vtol-1` reuses plane-1's build

This saves significant build time.

### Resource Limits

Add resource limits to prevent system overload:

```yaml
services:
  copter-1:
    # ... existing config
    deploy:
      resources:
        limits:
          cpus: '1.0'
          memory: 1G
```

## Swarm Control Examples

### Arm All Vehicles via ROS2

```bash
# Arm Copter 1
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

# For multi-vehicle, you'd need namespaced topics
# Or use MAVLink for traditional swarm control
```

### Waypoint Mission for Swarm

Use MAVProxy or Mission Planner to upload missions to each vehicle individually through their respective MAVLink ports.

### Formation Control

Consider using:
- ROS2 for high-level coordination
- MAVLink for individual vehicle control
- External scripts to maintain formation

## Files Reference

```
ardupilot/
├── docker-compose.swarm.yml    # Multi-vehicle configuration
├── .env.swarm.example          # Swarm environment template
├── .env.swarm                  # Your swarm configuration (create this)
├── docker/
│   └── SWARM_SETUP.md          # This file
└── sitl_logs/
    ├── copter-1/               # Copter 1 logs
    ├── copter-2/               # Copter 2 logs
    ├── plane-1/                # Plane 1 logs
    └── vtol-1/                 # VTOL 1 logs
```

## Next Steps

1. **Test Single Vehicle**: Start with one vehicle to verify setup
2. **Add Second Vehicle**: Test multi-vehicle coordination
3. **Configure ROS2**: Set up all ROS2 agents
4. **Test GCS**: Connect ground stations to each vehicle
5. **Run Missions**: Upload and execute missions on swarm

## Related Documentation

- [Single Vehicle Setup](README.md)
- [ROS2 Connection](ROS2_CONNECTION.md)
- [Main Documentation](../Claude.md)
