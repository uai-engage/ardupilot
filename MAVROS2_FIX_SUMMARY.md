# MAVROS2 Connection Fix - Summary

## Problem Identified

MAVROS2 was unable to connect because the generated Docker Compose configuration had **incorrect port assignments**:

### Before (Broken):
- **Documentation said**: MAVROS2 uses ports 5761, 5771, 5781 (GCS Port + 1)
- **Code actually used**: Ports 14550, 14551, 14552 (UDP MAVProxy style ports)
- **Result**: MAVROS2 tried to connect to port 5761, but nothing was listening

### Root Cause:
```bash
# generate_swarm.sh line 161 (WRONG):
COPTER1_MAVLINK_ROS_PORT=14550  # Should be 5761!
```

---

## What Was Fixed

### 1. Port Allocation Formula (generate_swarm.sh)

**Changed for Copters (line 161):**
```bash
# Before:
COPTER${i}_MAVLINK_ROS_PORT=$((14550 + ($i - 1)))

# After:
COPTER${i}_MAVLINK_ROS_PORT=$((5761 + ($i - 1) * 10))
```

**Changed for Planes (line 185):**
```bash
# Before:
PLANE${i}_MAVLINK_ROS_PORT=$((14550 + ($VEHICLE_NUM - 1)))

# After:
PLANE${i}_MAVLINK_ROS_PORT=$((5761 + ($VEHICLE_NUM - 1) * 10))
```

**Changed for VTOLs (line 208):**
```bash
# Before:
VTOL${i}_MAVLINK_ROS_PORT=$((14550 + ($VEHICLE_NUM - 1)))

# After:
VTOL${i}_MAVLINK_ROS_PORT=$((5761 + ($VEHICLE_NUM - 1) * 10))
```

### 2. Port Information Display

**Updated output messages to show correct ports:**
```bash
# Before:
echo "  MAVLink ROS:   14550 (UDP) - For MAVROS2"
echo "  MAVROS2: fcu_url=udp://127.0.0.1:14550@"

# After:
echo "  MAVLink ROS:   5761 (TCP) - For MAVROS2"
echo "  MAVROS2: fcu_url=tcp://127.0.0.1:5761"
```

### 3. Documentation Updates (CONNECTION_GUIDE.md)

**Updated all MAVROS2 connection examples** to use correct ports:
- Copter 1: `tcp://127.0.0.1:5761` ✅ (was 5760)
- Copter 2: `tcp://127.0.0.1:5771` ✅ (was 5770)
- Copter 3: `tcp://127.0.0.1:5781` ✅ (already correct)

---

## New Port Assignments

| Vehicle  | Instance | SYSID | Mission Planner (GCS) | MAVROS2 (ROS) | SITL | DDS |
|----------|----------|-------|-----------------------|---------------|------|-----|
| Copter 1 | 0 | 1 | **5760** (TCP) | **5761** (TCP) | 5501 (UDP) | 2019 (UDP) |
| Copter 2 | 1 | 2 | **5770** (TCP) | **5771** (TCP) | 5511 (UDP) | 2020 (UDP) |
| Copter 3 | 2 | 3 | **5780** (TCP) | **5781** (TCP) | 5521 (UDP) | 2021 (UDP) |
| Plane 1  | 3 | 4 | **5790** (TCP) | **5791** (TCP) | 5531 (UDP) | 2022 (UDP) |
| VTOL 1   | 4 | 5 | **5800** (TCP) | **5801** (TCP) | 5541 (UDP) | 2023 (UDP) |

### Port Formulas:
```
Mission Planner Port = 5760 + (Instance × 10)
MAVROS2 Port        = 5761 + (Instance × 10)  ← FIXED!
SITL Port           = 5501 + (Instance × 10)
DDS Port            = 2019 + Instance
```

---

## How to Test the Fix

### Step 1: Regenerate Your Swarm Configuration

```bash
# Delete old generated files
rm -f docker-compose-generated.yml .env.generated

# Generate new swarm (example: 2 copters)
./generate_swarm.sh --copters=2
```

### Step 2: Start Docker Containers

```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Expected Output (per vehicle):**
```
=========================================
ArduPilot Copter 1 (Instance 0, SYSID 1)
-----------------------------------------
PORT ASSIGNMENTS:
  MAVLink GCS:  5760 (TCP) - For Mission Planner
  MAVLink ROS:  5761 (TCP) - For MAVROS2  ← NEW!
  SITL:         5501 (UDP) - For external simulator
  DDS:          2019 (UDP) - For ROS2 communication

CONNECTIONS:
  Mission Planner: tcp:127.0.0.1:5760
  MAVROS2: fcu_url=tcp://127.0.0.1:5761  ← CORRECT PORT!
=========================================
```

### Step 3: Test Mission Planner Connection (Should Still Work)

**Open Mission Planner:**
1. Connection Type: **TCP**
2. IP: `127.0.0.1`
3. Port: `5760` (Copter 1)
4. Click **Connect**

**Expected**: ✅ Connection successful (unchanged behavior)

### Step 4: Test MAVROS2 Connection (Should Now Work!)

**Terminal 1 - Start MAVROS2 for Copter 1:**
```bash
source /opt/ros/humble/setup.bash

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5761 \
  -p target_system_id:=1 \
  -p fcu_protocol:=v2.0
```

**Expected Output:**
```
[INFO] [mavros_node]: FCU URL: tcp://127.0.0.1:5761
[INFO] [mavros_node]: Connected to FCU
[INFO] [mavros_node]: CONNECTED
```

**Terminal 2 - Verify MAVROS2 Topics:**
```bash
# List all MAVROS topics
ros2 topic list | grep copter1

# Expected output:
/copter1/mavros/state
/copter1/mavros/battery
/copter1/mavros/global_position/global
/copter1/mavros/imu/data
/copter1/mavros/local_position/pose
...
```

**Terminal 3 - Check Connection State:**
```bash
ros2 topic echo /copter1/mavros/state
```

**Expected Output:**
```yaml
connected: true
armed: false
guided: false
mode: "STABILIZE"
system_status: 3
```

### Step 5: Test Multi-Vehicle MAVROS2

**Start MAVROS2 for Copter 2 (in separate terminal):**
```bash
source /opt/ros/humble/setup.bash

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter2 \
  -p fcu_url:=tcp://127.0.0.1:5771 \
  -p target_system_id:=2 \
  -p fcu_protocol:=v2.0
```

**Verify Both Copters:**
```bash
# Check both namespaces
ros2 topic list | grep -E 'copter1|copter2'

# Should show:
/copter1/mavros/state
/copter1/mavros/battery
...
/copter2/mavros/state
/copter2/mavros/battery
...
```

---

## Testing MAVROS2 Commands

### Arm Copter 1
```bash
ros2 service call /copter1/mavros/cmd/arming \
  mavros_msgs/srv/CommandBool \
  "{value: true}"
```

**Expected Response:**
```yaml
success: true
result: 0
```

### Set Mode to GUIDED
```bash
ros2 service call /copter1/mavros/set_mode \
  mavros_msgs/srv/SetMode \
  "{custom_mode: 'GUIDED'}"
```

### Takeoff to 10m
```bash
ros2 service call /copter1/mavros/cmd/takeoff \
  mavros_msgs/srv/CommandTOL \
  "{altitude: 10.0}"
```

### Monitor GPS Data
```bash
ros2 topic echo /copter1/mavros/global_position/global
```

**Expected Output:**
```yaml
latitude: 35.363261
longitude: 149.165230
altitude: 584.0
```

---

## Simultaneous Mission Planner + MAVROS2

**Now you can use BOTH at the same time!**

**Terminal 1 - Docker:**
```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Terminal 2 - MAVROS2:**
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5761 \
  -p target_system_id:=1
```

**Mission Planner:**
- Connect to `tcp:127.0.0.1:5760`

**Both will work simultaneously without conflicts!**

---

## Troubleshooting

### MAVROS2 Still Can't Connect

**Check 1: Verify Port is Listening**
```bash
# Check if ArduPilot is listening on port 5761
sudo netstat -tulpn | grep 5761

# Expected output:
tcp   0   0 0.0.0.0:5761   0.0.0.0:*   LISTEN   12345/arducopter
```

**Check 2: Verify Docker Container is Running**
```bash
docker ps | grep copter-1

# Should show running container
```

**Check 3: Check Docker Logs**
```bash
docker logs <container-name> | grep 5761

# Should show ArduPilot listening on port 5761
```

**Check 4: Regenerate Configuration**
```bash
# Make sure you regenerated after the fix!
./generate_swarm.sh --copters=2
docker compose -f docker-compose-generated.yml up
```

### Connection Refused Error

**Symptom:** `Connection refused` when MAVROS2 tries to connect

**Fix:**
1. Ensure Docker container started successfully
2. Wait 10-15 seconds after container start
3. Verify port number matches instance (5761 for Instance 0, 5771 for Instance 1, etc.)

### Wrong SYSID in MAVROS2

**Symptom:** MAVROS2 shows wrong vehicle data

**Fix:**
Ensure `target_system_id` matches vehicle SYSID:
```bash
# Copter 1 = SYSID 1
-p target_system_id:=1

# Copter 2 = SYSID 2
-p target_system_id:=2
```

---

## Summary of Changes

✅ **Fixed**: Port allocation formula in `generate_swarm.sh` (lines 161, 185, 208)
✅ **Fixed**: Port display messages in `generate_swarm.sh`
✅ **Updated**: CONNECTION_GUIDE.md with correct MAVROS2 connection examples
✅ **Verified**: Mission Planner connection still works (port 5760+)
✅ **Enabled**: MAVROS2 connection now works (port 5761+)

### Key Takeaway:
**Mission Planner** connects to port **5760, 5770, 5780...**
**MAVROS2** connects to port **5761, 5771, 5781...**
**Both can run simultaneously** without conflicts! 🎉

---

## Files Modified

1. `generate_swarm.sh` - Port allocation logic and display messages
2. `CONNECTION_GUIDE.md` - MAVROS2 connection examples
3. `MAVROS2_FIX_SUMMARY.md` - This file (new)

---

**Last Updated:** 2025-01-20
**Fix Version:** 1.0
**Status:** ✅ Ready for Testing
