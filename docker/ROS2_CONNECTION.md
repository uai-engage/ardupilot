# Connecting ArduPilot SITL to ROS2 (Both in Docker on Same Host)

This guide covers connecting ArduPilot SITL and ROS2 when both are running in Docker containers on the same host using `network_mode: host`.

## Architecture

```
┌────────────────────────────────────────────┐
│           Host Machine                     │
│                                            │
│  ┌─────────────────┐  ┌─────────────────┐│
│  │ ArduPilot SITL  │  │   ROS2 Docker   ││
│  │    Container    │  │    Container    ││
│  │ network: host   │  │  network: host  ││
│  │                 │  │                 ││
│  │  DDS Client ────┼──┼──► Micro ROS   ││
│  │  127.0.0.1:2019 │  │    Agent       ││
│  │                 │  │    :2019       ││
│  └─────────────────┘  └─────────────────┘│
│                                            │
│         Both share host network            │
│         (localhost = 127.0.0.1)           │
└────────────────────────────────────────────┘
```

## Configuration

### 1. ArduPilot Container (.env)

Since both containers use `network_mode: host`, they share the host's network namespace. Configure ArduPilot to connect to localhost:

```bash
# .env file for ArduPilot
ENABLE_DDS=1
DDS_UDP_PORT=2019

# Connect to localhost (same host network)
DDS_IP0=127
DDS_IP1=0
DDS_IP2=0
DDS_IP3=1

DDS_DOMAIN_ID=0
DDS_TIMEOUT_MS=2000
DDS_MAX_RETRY=0  # Unlimited retries
```

### 2. ROS2 Container

Your ROS2 container should also use `network_mode: host`:

```yaml
# docker-compose.yml for ROS2 container
services:
  ros2:
    image: your-ros2-image
    network_mode: host  # Important!
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
      "
```

## Step-by-Step Setup

### Step 1: Start Micro ROS Agent (ROS2 Container)

```bash
# In your ROS2 container terminal
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

Expected output:
```
[1699999999.999999] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 2019
[1699999999.999999] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### Step 2: Start ArduPilot SITL

```bash
# In ardupilot directory
docker-compose up
```

### Step 3: Verify Connection

#### Check Agent Connection
In the Micro ROS Agent terminal, you should see:
```
[1699999999.999999] info     | ProxyClient.cpp    | create_client            | client created         | client_key: 0x00000001, session_id: 0x81
```

#### Check ROS2 Topics
```bash
# In ROS2 container
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
/ap/time
/ap/twist/filtered
...
```

#### Check ROS2 Services
```bash
ros2 service list
```

Expected output:
```
/ap/arm_motors
/ap/mode_switch
/ap/prearm_check
```

## Troubleshooting

### Issue: Agent Not Receiving Connection

**Check 1: Verify both containers use host network**
```bash
# Check ArduPilot container
docker inspect ardupilot-sitl | grep NetworkMode

# Check ROS2 container
docker inspect <ros2-container-name> | grep NetworkMode

# Both should show: "NetworkMode": "host"
```

**Check 2: Verify port not in use**
```bash
# On host machine
netstat -tuln | grep 2019

# Should show Micro ROS Agent listening:
# udp  0  0  0.0.0.0:2019  0.0.0.0:*
```

**Check 3: Check ArduPilot DDS parameters**
```bash
# View generated parameters
docker-compose logs | grep -A 15 "Generated parameter file"

# Should show:
# DDS_ENABLE 1
# DDS_UDP_PORT 2019
# DDS_IP0 127
# DDS_IP1 0
# DDS_IP2 0
# DDS_IP3 1
```

**Check 4: View ArduPilot logs for DDS connection**
```bash
docker-compose exec ardupilot-sitl tail -f /tmp/ArduCopter.log | grep -i dds
```

### Issue: ROS_DOMAIN_ID Mismatch

Both ArduPilot and ROS2 must use the same ROS_DOMAIN_ID:

```bash
# In ArduPilot .env
DDS_DOMAIN_ID=0

# In ROS2 container
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### Issue: No Topics Visible

```bash
# Check if agent is receiving data
docker-compose logs -f | grep -i dds

# Check ROS2 environment
ros2 doctor --report | grep ROS_DOMAIN_ID

# Try listing with verbose
ros2 topic list -v
```

## Testing the Connection

### 1. Echo ArduPilot Time
```bash
ros2 topic echo /ap/time
```

Expected:
```
sec: 1699999999
nanosec: 123456000
---
```

### 2. Arm Motors via ROS2
```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
```

Expected:
```
response:
ardupilot_msgs.srv.ArmMotors_Response(result=True)
```

### 3. Check Battery Status
```bash
ros2 topic echo /ap/battery --once
```

### 4. Monitor Position
```bash
ros2 topic echo /ap/pose/filtered
```

## Complete docker-compose Example (Both Services)

If you want to run both in a single docker-compose:

```yaml
version: '3.8'

services:
  # ArduPilot SITL
  ardupilot-sitl:
    build: ./ardupilot
    network_mode: host
    volumes:
      - ./ardupilot:/ardupilot
    env_file:
      - ./ardupilot/.env

  # ROS2 with Micro ROS Agent
  ros2-agent:
    image: osrf/ros:humble-desktop
    network_mode: host
    command: >
      bash -c "
        apt-get update &&
        apt-get install -y ros-humble-micro-ros-agent &&
        source /opt/ros/humble/setup.bash &&
        ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
      "
    depends_on:
      - ardupilot-sitl
```

Then run:
```bash
docker-compose up
```

## Port Reference (with network_mode: host)

All services bind to the host's ports directly:

| Service | Port | Protocol | Accessible From |
|---------|------|----------|-----------------|
| MAVLink | 5760 | TCP | Host, other containers |
| SITL Sim | 5501 | UDP | Host, other containers |
| DDS/ROS | 2019 | UDP | Host, other containers |
| MAVProxy | 14550 | UDP | Host, other containers |

## Verification Checklist

- [ ] Both containers use `network_mode: host`
- [ ] Micro ROS Agent starts before ArduPilot SITL
- [ ] DDS_IP is set to 127.0.0.1 in ArduPilot config
- [ ] DDS_UDP_PORT matches agent port (2019)
- [ ] ROS_DOMAIN_ID matches on both sides (default 0)
- [ ] Port 2019 is not blocked by firewall
- [ ] `ros2 topic list` shows /ap/* topics
- [ ] ArduPilot logs show DDS connection successful

## Alternative: Using Docker Internal Network

If you don't want to use `network_mode: host`, you can use a custom network:

```yaml
version: '3.8'

networks:
  ardupilot-ros:
    driver: bridge

services:
  ardupilot-sitl:
    networks:
      - ardupilot-ros
    environment:
      # Connect to ros2-agent container by name
      - DDS_IP0=ros2-agent  # Will be resolved by Docker DNS

  ros2-agent:
    networks:
      - ardupilot-ros
    hostname: ros2-agent
```

However, `network_mode: host` is simpler and has better performance.
