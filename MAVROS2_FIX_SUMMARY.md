# MAVROS2 Connection Fix - Final Summary

## Problem Identified

MAVROS2 was unable to connect due to **incorrect MAVProxy configuration** that used invalid syntax for the `--out` parameter.

### Root Cause:

The `generate_swarm.sh` script was generating invalid MAVProxy commands:

```bash
# WRONG - tcpin/udpin are NOT valid for --out parameter:
mavproxy.py --master tcp:127.0.0.1:5760 --out tcpin:0.0.0.0:5761,udpin:0.0.0.0:14550

# Error:
ValueError: TCP ports must be specified as host:port
```

**Why it failed:**
- `tcpin:` and `udpin:` formats are ONLY valid for `--master` (input/server mode)
- The `--out` parameter only accepts simple `IP:port` format for UDP output
- Attempting to use `tcpin:` in `--out` caused MAVProxy to crash

---

## What Was Fixed

### 1. Port Allocation in .env Generation (generate_swarm.sh lines 161, 185, 208)

**Before:**
```bash
COPTER${i}_MAVPROXY_OUT=tcpin:0.0.0.0:$((5760 + ($i - 1) * 10)),udpin:0.0.0.0:$((14550 + ($i - 1)))
COPTER${i}_MAVPROXY_MASTER=tcp:127.0.0.1:$((5501 + ($i - 1) * 10))
```

**After:**
```bash
COPTER${i}_MAVPROXY_OUT=127.0.0.1:$((14550 + ($i - 1)))
COPTER${i}_MAVPROXY_MASTER=tcp:127.0.0.1:$((5760 + ($i - 1) * 10))
```

**Changes:**
- ✅ Removed invalid `tcpin:` and `udpin:` prefixes from MAVPROXY_OUT
- ✅ Changed to simple UDP format: `IP:port`
- ✅ Fixed MAVPROXY_MASTER to connect to MAVLink port (5760) instead of SITL physics port (5501)

### 2. Docker Compose Default Values (lines 303-304, 460-461, 615-616)

**Updated fallback defaults for all vehicle types:**
```bash
MAVPROXY_OUT=\${COPTER${i}_MAVPROXY_OUT:-127.0.0.1:14550}
MAVPROXY_MASTER=\${COPTER${i}_MAVPROXY_MASTER:-tcp:127.0.0.1:$MAVLINK_PORT}
```

### 3. Display Messages (lines 325-336, 482-493, 637-648)

**Updated to show correct architecture:**
```bash
echo "  MAVLink:       5760 (TCP) - Mission Planner + MAVROS2"
echo "  MAVProxy UDP:  14550 (UDP) - Monitoring output"
echo ""
echo "  Mission Planner: tcp:127.0.0.1:5760"
echo "  MAVROS2: fcu_url=tcp://127.0.0.1:5760"
```

### 4. Updated Comments (line 367)

**Before:** "Use MAVProxy to create separate TCP ports for GCS and MAVROS2"

**After:** "Add UDP output for monitoring/logging (GCS and MAVROS2 connect directly to ArduPilot TCP server)"

---

## Final Architecture

### Key Concept: Multi-Client TCP Server

ArduPilot's SITL creates a **TCP server on port 5760** that supports **multiple simultaneous client connections**. This means:

- ✅ Mission Planner and MAVROS2 can BOTH connect to port 5760 at the same time
- ✅ No need for MAVProxy to create separate TCP listeners
- ✅ Simpler architecture with fewer ports
- ✅ MAVProxy only provides optional UDP output for monitoring

### Port Assignments

| Vehicle  | Instance | SYSID | MAVLink TCP (Shared) | MAVProxy UDP | SITL | DDS |
|----------|----------|-------|----------------------|--------------|------|-----|
| Copter 1 | 0        | 1     | **5760** (TCP)       | 14550 (UDP)  | 5501 | 2019 |
| Copter 2 | 1        | 2     | **5770** (TCP)       | 14551 (UDP)  | 5511 | 2020 |
| Copter 3 | 2        | 3     | **5780** (TCP)       | 14552 (UDP)  | 5521 | 2021 |
| Plane 1  | 3        | 4     | **5790** (TCP)       | 14553 (UDP)  | 5531 | 2022 |
| VTOL 1   | 4        | 5     | **5800** (TCP)       | 14554 (UDP)  | 5541 | 2023 |

### Port Formulas:
```
MAVLink TCP Port    = 5760 + (Instance × 10)  ← Mission Planner + MAVROS2
MAVProxy UDP Output = 14550 + Instance         ← Optional monitoring
SITL Port           = 5501 + (Instance × 10)
DDS Port            = 2019 + Instance
```

### Connection Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     ArduPilot SITL                          │
│                                                             │
│  ┌──────────────────────────────────────────────────┐      │
│  │     TCP Server on Port 5760                      │      │
│  │     (Supports Multiple Clients)                  │      │
│  └──────────────────────────────────────────────────┘      │
│           ▲                           ▲                     │
│           │                           │                     │
│           │                           │                     │
└───────────┼───────────────────────────┼─────────────────────┘
            │                           │
            │                           │
    ┌───────┴────────┐          ┌──────┴────────┐
    │                │          │               │
    │  Mission       │          │   MAVROS2     │
    │  Planner       │          │   ROS2 Node   │
    │                │          │               │
    │ tcp:5760       │          │ tcp:5760      │
    └────────────────┘          └───────────────┘

    Both connect to the SAME TCP port!

    Optional UDP Output (monitoring):
    │
    │ UDP 14550
    ▼
    ┌────────────────┐
    │   Monitoring   │
    │   / Logging    │
    └────────────────┘
```

---

## How to Test the Fix

### Step 1: Regenerate Your Swarm Configuration

```bash
# Delete old generated files
rm -f docker-compose-generated.yml .env.generated

# Generate new swarm (example: 1 copter for testing)
./generate_swarm.sh --copters=1
```

### Step 2: Start Docker Containers

```bash
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

**Expected Output:**
```
=========================================
ArduPilot Copter 1 (Instance 0, SYSID 1)
-----------------------------------------
PORT ASSIGNMENTS:
  MAVLink:       5760 (TCP) - Mission Planner + MAVROS2
  MAVProxy UDP:  14550 (UDP) - Monitoring output
  SITL:          5501 (UDP) - For external simulator
  DDS:           2019 (UDP) - For ROS2 communication

CONNECTIONS:
  Mission Planner: tcp:127.0.0.1:5760
  MAVROS2: fcu_url=tcp://127.0.0.1:5760
=========================================
```

### Step 3: Test Mission Planner Connection

**Open Mission Planner:**
1. Connection Type: **TCP**
2. IP: `127.0.0.1`
3. Port: `5760`
4. Click **Connect**

**Expected:** ✅ Connection successful

### Step 4: Test MAVROS2 Connection (Simultaneously!)

**Terminal 1 - Start MAVROS2 for Copter 1:**
```bash
source /opt/ros/humble/setup.bash

ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760 \
  -p target_system_id:=1 \
  -p fcu_protocol:=v2.0
```

**Expected Output:**
```
[INFO] [mavros_node]: FCU URL: tcp://127.0.0.1:5760
[INFO] [mavros_node]: Connected to FCU
[INFO] [mavros_node]: CONNECTED
```

**Terminal 2 - Verify MAVROS2 Topics:**
```bash
ros2 topic list | grep copter1

# Expected output:
/copter1/mavros/state
/copter1/mavros/battery
/copter1/mavros/global_position/global
/copter1/mavros/imu/data
/copter1/mavros/local_position/pose
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

### Step 5: Verify Both Connections Work Simultaneously

**With Mission Planner still connected:**

1. In Mission Planner, arm the vehicle
2. In MAVROS2 terminal, verify the state changes:
   ```bash
   ros2 topic echo /copter1/mavros/state --once
   # Should show: armed: true
   ```

**With MAVROS2 still connected:**

1. In Terminal 4, arm via ROS2:
   ```bash
   ros2 service call /copter1/mavros/cmd/arming \
     mavros_msgs/srv/CommandBool \
     "{value: true}"
   ```
2. In Mission Planner, verify the HUD shows "ARMED"

**Both clients can control the same vehicle!** ✅

---

## Testing MAVROS2 Commands

### Arm Copter 1
```bash
ros2 service call /copter1/mavros/cmd/arming \
  mavros_msgs/srv/CommandBool \
  "{value: true}"
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

---

## Multi-Vehicle MAVROS2 Testing

### Generate 2-Vehicle Swarm

```bash
./generate_swarm.sh --copters=2
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

### Start MAVROS2 for Both Copters

**Terminal 1 - Copter 1:**
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter1 \
  -p fcu_url:=tcp://127.0.0.1:5760 \
  -p target_system_id:=1
```

**Terminal 2 - Copter 2:**
```bash
ros2 run mavros mavros_node --ros-args \
  -r __ns:=/copter2 \
  -p fcu_url:=tcp://127.0.0.1:5770 \
  -p target_system_id:=2
```

**Verify Both:**
```bash
ros2 topic list | grep -E 'copter1|copter2'

# Should show both:
/copter1/mavros/state
/copter1/mavros/battery
...
/copter2/mavros/state
/copter2/mavros/battery
...
```

---

## Troubleshooting

### MAVROS2 Can't Connect

**Check 1: Verify Port is Listening**
```bash
sudo netstat -tulpn | grep 5760

# Expected output:
tcp   0   0 0.0.0.0:5760   0.0.0.0:*   LISTEN   12345/arducopter
```

**Check 2: Verify Docker Container Running**
```bash
docker ps | grep copter-1
```

**Check 3: Check Docker Logs**
```bash
docker logs <container-name> | grep -E '5760|MAVProxy'
```

**Check 4: Regenerate Configuration**
```bash
# Make sure you regenerated after pulling the latest fixes!
./generate_swarm.sh --copters=1
docker compose -f docker-compose-generated.yml --env-file .env.generated up
```

### Connection Refused Error

**Symptom:** `Connection refused` when MAVROS2 tries to connect

**Fix:**
1. Ensure Docker container started successfully (check logs)
2. Wait 15-20 seconds after container start for ArduPilot to initialize
3. Verify port number matches instance:
   - Instance 0: port 5760
   - Instance 1: port 5770
   - Instance 2: port 5780

### Wrong Vehicle Data in MAVROS2

**Symptom:** MAVROS2 shows data from different vehicle

**Fix:** Ensure `target_system_id` matches vehicle SYSID:
```bash
# Copter 1 = SYSID 1
-p target_system_id:=1

# Copter 2 = SYSID 2
-p target_system_id:=2
```

### MAVProxy ValueError on Startup

**Symptom:** Container crashes with "ValueError: TCP ports must be specified as host:port"

**Fix:** This means you have old generated files. Regenerate:
```bash
rm -f docker-compose-generated.yml .env.generated
./generate_swarm.sh --copters=1
```

---

## Summary of Changes

### Commits Applied:

1. **5b8a023847** - Fix MAVROS2 port allocation for multi-vehicle swarms
   - Initial port allocation changes

2. **941860c073** - Fix MAVProxy port configuration to prevent conflicts
   - Fixed MAVPROXY_MASTER to use port 5760 instead of 5501

3. **24ac364a22** - Fix MAVProxy --out parameter format (remove invalid tcpin/udpin)
   - Removed invalid `tcpin:` and `udpin:` syntax from MAVPROXY_OUT
   - Changed to simple UDP format

4. **675d82cb27** - Update Docker Compose defaults and display messages for MAVROS2
   - Fixed docker-compose template default values
   - Updated display messages to show correct architecture
   - Updated comments to reflect actual behavior

### Files Modified:

1. ✅ `generate_swarm.sh` - Port allocation, defaults, and display messages
2. ✅ `MAVROS2_FIX_SUMMARY.md` - This file (updated to reflect final architecture)

### Key Takeaway:

**ArduPilot's TCP server on port 5760 supports multiple simultaneous clients!**

- Mission Planner → `tcp://127.0.0.1:5760`
- MAVROS2 → `tcp://127.0.0.1:5760` (same port!)
- MAVProxy UDP → `udp://127.0.0.1:14550` (optional monitoring)

**Both can run simultaneously without conflicts!** 🎉

---

**Last Updated:** 2025-01-20
**Fix Version:** 2.0 (Final)
**Status:** ✅ Complete and Ready for Testing
