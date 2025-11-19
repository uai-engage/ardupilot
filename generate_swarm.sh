#!/bin/bash

# ArduPilot Swarm Generator
# Dynamically generates docker-compose and .env files for multi-vehicle swarms
# Usage: ./generate_swarm.sh --copters=2 --planes=1 --vtols=1 [--start]

set -e

# Default values
NUM_COPTERS=0
NUM_PLANES=0
NUM_VTOLS=0
AUTO_START=false
OUTPUT_COMPOSE="docker-compose-generated.yml"
OUTPUT_ENV=".env.generated"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Parse command line arguments
for arg in "$@"; do
    case $arg in
        --copters=*)
            NUM_COPTERS="${arg#*=}"
            ;;
        --planes=*)
            NUM_PLANES="${arg#*=}"
            ;;
        --vtols=*)
            NUM_VTOLS="${arg#*=}"
            ;;
        --start)
            AUTO_START=true
            ;;
        --help|-h)
            echo "ArduPilot Swarm Generator"
            echo ""
            echo "Usage: ./generate_swarm.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --copters=N    Number of copter instances (default: 0)"
            echo "  --planes=N     Number of plane instances (default: 0)"
            echo "  --vtols=N      Number of VTOL instances (default: 0)"
            echo "  --start        Automatically start the swarm after generation"
            echo "  --help, -h     Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./generate_swarm.sh --copters=4"
            echo "  ./generate_swarm.sh --copters=2 --planes=1 --vtols=1"
            echo "  ./generate_swarm.sh --copters=3 --start"
            echo ""
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $arg${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Validate input
TOTAL_VEHICLES=$((NUM_COPTERS + NUM_PLANES + NUM_VTOLS))
if [ $TOTAL_VEHICLES -eq 0 ]; then
    echo -e "${RED}Error: Must specify at least one vehicle!${NC}"
    echo "Use --help for usage information"
    exit 1
fi

if [ $TOTAL_VEHICLES -gt 10 ]; then
    echo -e "${YELLOW}Warning: Running more than 10 vehicles may cause performance issues${NC}"
fi

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}ArduPilot Swarm Generator${NC}"
echo -e "${BLUE}=========================================${NC}"
echo -e "Copters: ${GREEN}$NUM_COPTERS${NC}"
echo -e "Planes:  ${GREEN}$NUM_PLANES${NC}"
echo -e "VTOLs:   ${GREEN}$NUM_VTOLS${NC}"
echo -e "Total:   ${GREEN}$TOTAL_VEHICLES${NC} vehicles"
echo -e "${BLUE}=========================================${NC}"
echo ""

# Create log directories
echo -e "${YELLOW}Creating log directories...${NC}"
mkdir -p sitl_logs
VEHICLE_INDEX=0

for ((i=0; i<NUM_COPTERS; i++)); do
    mkdir -p "sitl_logs/copter-$((i+1))"
done

for ((i=0; i<NUM_PLANES; i++)); do
    mkdir -p "sitl_logs/plane-$((i+1))"
done

for ((i=0; i<NUM_VTOLS; i++)); do
    mkdir -p "sitl_logs/vtol-$((i+1))"
done

chmod -R 777 sitl_logs
echo -e "${GREEN}✓ Log directories created${NC}"
echo ""

# Default spawn location (CMAC, Canberra, Australia)
DEFAULT_LAT="35.363261"
DEFAULT_LON="149.165230"
DEFAULT_ALT="584"
DEFAULT_HDG="0"

# Generate .env file
echo -e "${YELLOW}Generating $OUTPUT_ENV...${NC}"
cat > "$OUTPUT_ENV" << 'EOF'
# ArduPilot Swarm Configuration - Auto-Generated
# DO NOT EDIT MANUALLY - Use generate_swarm.sh to regenerate

# ============================================
# GLOBAL SETTINGS
# ============================================
SPEEDUP=1
SIM_ADDRESS=127.0.0.1
WIPE=0
SYNTHETIC_CLOCK=0

# ROS2/DDS Configuration (SHARED by all vehicles)
ENABLE_DDS=1
DDS_UDP_PORT=2019
DDS_IP0=127
DDS_IP1=0
DDS_IP2=0
DDS_IP3=1
DDS_DOMAIN_ID=0
DDS_TIMEOUT_MS=1000
DDS_MAX_RETRY=10

# ============================================
# VEHICLE-SPECIFIC SETTINGS
# ============================================
EOF

# Generate vehicle-specific env variables
SYSID=1
INSTANCE=0

# Copters
for ((i=1; i<=NUM_COPTERS; i++)); do
    # Calculate spawn offset (10m east per vehicle)
    LON_OFFSET=$(echo "$DEFAULT_LON + ($i - 1) * 0.00001" | bc -l)

    cat >> "$OUTPUT_ENV" << EOF

# Copter $i
COPTER${i}_MODEL=quad
COPTER${i}_SYSID=$SYSID
COPTER${i}_HOME=$DEFAULT_LAT,$LON_OFFSET,$DEFAULT_ALT,$DEFAULT_HDG
COPTER${i}_MAVPROXY_ENABLED=1
COPTER${i}_MAVPROXY_OUT=127.0.0.1:$((14550 + ($i - 1)))
COPTER${i}_MAVPROXY_MASTER=tcp:127.0.0.1:$((5760 + ($i - 1) * 10))
COPTER${i}_MAVPROXY_SITL=127.0.0.1:$((5501 + ($i - 1) * 10))
EOF

    SYSID=$((SYSID + 1))
done

# Planes
for ((i=1; i<=NUM_PLANES; i++)); do
    # Calculate spawn offset
    LON_OFFSET=$(echo "$DEFAULT_LON + ($NUM_COPTERS + $i - 1) * 0.00001" | bc -l)
    ALT_OFFSET=$((DEFAULT_ALT + 100)) # Planes start 100m higher
    VEHICLE_NUM=$((NUM_COPTERS + i))

    cat >> "$OUTPUT_ENV" << EOF

# Plane $i
PLANE${i}_MODEL=plane
PLANE${i}_SYSID=$SYSID
PLANE${i}_HOME=$DEFAULT_LAT,$LON_OFFSET,$ALT_OFFSET,$DEFAULT_HDG
PLANE${i}_MAVPROXY_ENABLED=1
PLANE${i}_MAVPROXY_OUT=127.0.0.1:$((14550 + ($VEHICLE_NUM - 1)))
PLANE${i}_MAVPROXY_MASTER=tcp:127.0.0.1:$((5760 + ($VEHICLE_NUM - 1) * 10))
PLANE${i}_MAVPROXY_SITL=127.0.0.1:$((5501 + ($VEHICLE_NUM - 1) * 10))
EOF

    SYSID=$((SYSID + 1))
done

# VTOLs
for ((i=1; i<=NUM_VTOLS; i++)); do
    # Calculate spawn offset
    LON_OFFSET=$(echo "$DEFAULT_LON + ($NUM_COPTERS + $NUM_PLANES + $i - 1) * 0.00001" | bc -l)
    VEHICLE_NUM=$((NUM_COPTERS + NUM_PLANES + i))

    cat >> "$OUTPUT_ENV" << EOF

# VTOL $i
VTOL${i}_MODEL=quadplane
VTOL${i}_SYSID=$SYSID
VTOL${i}_HOME=$DEFAULT_LAT,$LON_OFFSET,$DEFAULT_ALT,$DEFAULT_HDG
VTOL${i}_MAVPROXY_ENABLED=1
VTOL${i}_MAVPROXY_OUT=127.0.0.1:$((14550 + ($VEHICLE_NUM - 1)))
VTOL${i}_MAVPROXY_MASTER=tcp:127.0.0.1:$((5760 + ($VEHICLE_NUM - 1) * 10))
VTOL${i}_MAVPROXY_SITL=127.0.0.1:$((5501 + ($VEHICLE_NUM - 1) * 10))
EOF

    SYSID=$((SYSID + 1))
done

echo -e "${GREEN}✓ Generated $OUTPUT_ENV${NC}"
echo ""

# Generate docker-compose.yml
echo -e "${YELLOW}Generating $OUTPUT_COMPOSE...${NC}"

cat > "$OUTPUT_COMPOSE" << 'EOF'
version: '3.8'

# ArduPilot Multi-Vehicle Swarm - Auto-Generated
# DO NOT EDIT MANUALLY - Use generate_swarm.sh to regenerate

services:
EOF

# Track if we need to build copter/plane
COPTER_BUILT=false
PLANE_BUILT=false
INSTANCE=0
SYSID=1

# Generate copter services
for ((i=1; i<=NUM_COPTERS; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    SITL_PORT=$((5501 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))

    # Determine if this is the first copter (needs to build)
    if [ "$COPTER_BUILT" = false ]; then
        SKIP_BUILD=0
        DEPENDS_ON=""
        COPTER_BUILT=true
    else
        SKIP_BUILD=1
        DEPENDS_ON="copter-1"
    fi

    cat >> "$OUTPUT_COMPOSE" << EOF

  copter-$i:
    build:
      context: .
      dockerfile: Dockerfile
      args:
        BASE_IMAGE: ubuntu
        TAG: "22.04"
        SKIP_AP_GRAPHIC_ENV: 1
        SKIP_AP_EXT_ENV: 0
        SKIP_AP_COV_ENV: 1
        SKIP_AP_GIT_CHECK: 1
        DO_AP_STM_ENV: 0
    container_name: ardupilot-copter-$i
    network_mode: host
    working_dir: /ardupilot
    volumes:
      - .:/ardupilot
      - ./logs:/tmp/buildlogs
      - ./sitl_logs/copter-$i:/ardupilot/logs
      - copter-$i-eeprom:/ardupilot/eeprom
    environment:
      - VEHICLE=ArduCopter
      - MODEL=\${COPTER${i}_MODEL:-quad}
      - INSTANCE=$INSTANCE
      - SYSID=\${COPTER${i}_SYSID:-$SYSID}
      - SPEEDUP=\${SPEEDUP:-1}
      - SIM_ADDRESS=\${SIM_ADDRESS:-127.0.0.1}
      - WIPE=\${WIPE:-0}
      - SYNTHETIC_CLOCK=\${SYNTHETIC_CLOCK:-0}
      - HOME_LOCATION=\${COPTER${i}_HOME:-}
      - ENABLE_DDS=\${ENABLE_DDS:-1}
      - DDS_ENABLE=1
      - DDS_UDP_PORT=$DDS_PORT
      - DDS_IP0=\${DDS_IP0:-127}
      - DDS_IP1=\${DDS_IP1:-0}
      - DDS_IP2=\${DDS_IP2:-0}
      - DDS_IP3=\${DDS_IP3:-1}
      - DDS_DOMAIN_ID=\${DDS_DOMAIN_ID:-0}
      - DDS_TIMEOUT_MS=\${DDS_TIMEOUT_MS:-1000}
      - DDS_MAX_RETRY=\${DDS_MAX_RETRY:-10}
      - SKIP_BUILD=$SKIP_BUILD
      - BUILD_TARGET=sitl
      - MAVPROXY_ENABLED=\${COPTER${i}_MAVPROXY_ENABLED:-1}
      - MAVPROXY_OUT=\${COPTER${i}_MAVPROXY_OUT:-127.0.0.1:14550}
      - MAVPROXY_MASTER=\${COPTER${i}_MAVPROXY_MASTER:-tcp:127.0.0.1:5760}
      - MAVPROXY_SITL=\${COPTER${i}_MAVPROXY_SITL:-127.0.0.1:5501}
EOF

    if [ -n "$DEPENDS_ON" ]; then
        cat >> "$OUTPUT_COMPOSE" << EOF
    depends_on:
      - $DEPENDS_ON
EOF
    fi

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
    entrypoint: ["/bin/bash", "-c"]
    command:
      - |
        set -e
        source /home/ardupilot/.ardupilot_env

        echo "========================================="
EOF

    cat >> "$OUTPUT_COMPOSE" << EOF
        echo "ArduPilot Copter $i (Instance $INSTANCE, SYSID $SYSID)"
        echo "-----------------------------------------"
        echo "PORT ASSIGNMENTS:"
        echo "  MAVLink:  $MAVLINK_PORT (TCP) - For Mission Planner/GCS"
        echo "  SITL:     $SITL_PORT (UDP) - For external simulator"
        echo "  DDS:      $DDS_PORT (UDP) - For ROS2 communication"
        echo "  MAVProxy: $((14550 + INSTANCE)) (UDP) - For telemetry output"
        echo ""
        echo "CONNECTIONS:"
        echo "  Mission Planner: tcp:127.0.0.1:$MAVLINK_PORT"
        echo "  ROS2 Agent: ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT"
EOF

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
        echo "========================================="

        PARAM_FILE="/tmp/copter$${INSTANCE}_params.parm"
        echo "# Copter $${INSTANCE} Parameters" > $$PARAM_FILE
        echo "LOG_DISARMED 1" >> $$PARAM_FILE
        echo "LOG_BACKEND_TYPE 1" >> $$PARAM_FILE
        echo "DDS_ENABLE 1" >> $$PARAM_FILE
        echo "DDS_UDP_PORT $${DDS_UDP_PORT}" >> $$PARAM_FILE
        echo "DDS_IP0 $${DDS_IP0}" >> $$PARAM_FILE
        echo "DDS_IP1 $${DDS_IP1}" >> $$PARAM_FILE
        echo "DDS_IP2 $${DDS_IP2}" >> $$PARAM_FILE
        echo "DDS_IP3 $${DDS_IP3}" >> $$PARAM_FILE
        echo "DDS_DOMAIN_ID $${DDS_DOMAIN_ID}" >> $$PARAM_FILE
        echo "SYSID_THISMAV $${SYSID}" >> $$PARAM_FILE

        cd /ardupilot

        if [ "$$SKIP_BUILD" != "1" ]; then
          echo "Building ArduCopter..."
          git submodule update --init --recursive 2>/dev/null || true
          ./waf configure --board sitl --enable-DDS || (sleep 2 && ./waf configure --board sitl --enable-DDS)
          ./waf copter
        fi

        SIM_CMD="python3 Tools/autotest/sim_vehicle.py -v ArduCopter --model $${MODEL} --speedup $${SPEEDUP} --instance $${INSTANCE} --sim-address=$${SIM_ADDRESS} --add-param-file $$PARAM_FILE --enable-DDS"

        if [ "$${MAVPROXY_ENABLED}" = "0" ]; then
          SIM_CMD="$$SIM_CMD --no-mavproxy"
        else
          SIM_CMD="$$SIM_CMD --out=$${MAVPROXY_OUT}"
        fi

        if [ -n "$${HOME_LOCATION}" ]; then
          SIM_CMD="$$SIM_CMD --custom-location $${HOME_LOCATION}"
        fi

        if [ "$${WIPE}" = "1" ]; then
          SIM_CMD="$$SIM_CMD -w"
        fi

        touch /tmp/ArduCopter-$${INSTANCE}.log
        ln -sf /tmp/ArduCopter-$${INSTANCE}.log /ardupilot/logs/ArduCopter.log
        tail -f /tmp/ArduCopter-$${INSTANCE}.log &
        TAIL_PID=$$!
        trap "kill $$TAIL_PID 2>/dev/null || true" EXIT INT TERM

        exec $$SIM_CMD
EOF

    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

# Generate plane services
for ((i=1; i<=NUM_PLANES; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    SITL_PORT=$((5501 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))

    # Determine if this is the first plane (needs to build)
    if [ "$PLANE_BUILT" = false ]; then
        SKIP_BUILD=0
        if [ $NUM_COPTERS -gt 0 ]; then
            DEPENDS_ON="copter-1"
        else
            DEPENDS_ON=""
        fi
        PLANE_BUILT=true
    else
        SKIP_BUILD=1
        DEPENDS_ON="plane-1"
    fi

    cat >> "$OUTPUT_COMPOSE" << EOF

  plane-$i:
    build:
      context: .
      dockerfile: Dockerfile
      args:
        BASE_IMAGE: ubuntu
        TAG: "22.04"
        SKIP_AP_GRAPHIC_ENV: 1
        SKIP_AP_EXT_ENV: 0
        SKIP_AP_COV_ENV: 1
        SKIP_AP_GIT_CHECK: 1
        DO_AP_STM_ENV: 0
    container_name: ardupilot-plane-$i
    network_mode: host
    working_dir: /ardupilot
    volumes:
      - .:/ardupilot
      - ./logs:/tmp/buildlogs
      - ./sitl_logs/plane-$i:/ardupilot/logs
      - plane-$i-eeprom:/ardupilot/eeprom
    environment:
      - VEHICLE=ArduPlane
      - MODEL=\${PLANE${i}_MODEL:-plane}
      - INSTANCE=$INSTANCE
      - SYSID=\${PLANE${i}_SYSID:-$SYSID}
      - SPEEDUP=\${SPEEDUP:-1}
      - SIM_ADDRESS=\${SIM_ADDRESS:-127.0.0.1}
      - WIPE=\${WIPE:-0}
      - HOME_LOCATION=\${PLANE${i}_HOME:-}
      - ENABLE_DDS=\${ENABLE_DDS:-1}
      - DDS_ENABLE=1
      - DDS_UDP_PORT=$DDS_PORT
      - DDS_IP0=\${DDS_IP0:-127}
      - DDS_IP1=\${DDS_IP1:-0}
      - DDS_IP2=\${DDS_IP2:-0}
      - DDS_IP3=\${DDS_IP3:-1}
      - DDS_DOMAIN_ID=\${DDS_DOMAIN_ID:-0}
      - SKIP_BUILD=$SKIP_BUILD
      - BUILD_TARGET=sitl
      - MAVPROXY_ENABLED=\${PLANE${i}_MAVPROXY_ENABLED:-1}
      - MAVPROXY_OUT=\${PLANE${i}_MAVPROXY_OUT:-127.0.0.1:14550}
      - MAVPROXY_MASTER=\${PLANE${i}_MAVPROXY_MASTER:-tcp:127.0.0.1:5760}
      - MAVPROXY_SITL=\${PLANE${i}_MAVPROXY_SITL:-127.0.0.1:5501}
EOF

    if [ -n "$DEPENDS_ON" ]; then
        cat >> "$OUTPUT_COMPOSE" << EOF
    depends_on:
      - $DEPENDS_ON
EOF
    fi

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
    entrypoint: ["/bin/bash", "-c"]
    command:
      - |
        set -e
        source /home/ardupilot/.ardupilot_env

        echo "========================================="
EOF

    cat >> "$OUTPUT_COMPOSE" << EOF
        echo "ArduPilot Plane $i (Instance $INSTANCE, SYSID $SYSID)"
        echo "-----------------------------------------"
        echo "PORT ASSIGNMENTS:"
        echo "  MAVLink:  $MAVLINK_PORT (TCP) - For Mission Planner/GCS"
        echo "  SITL:     $SITL_PORT (UDP) - For external simulator"
        echo "  DDS:      $DDS_PORT (UDP) - For ROS2 communication"
        echo "  MAVProxy: $((14550 + INSTANCE)) (UDP) - For telemetry output"
        echo ""
        echo "CONNECTIONS:"
        echo "  Mission Planner: tcp:127.0.0.1:$MAVLINK_PORT"
        echo "  ROS2 Agent: ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT"
EOF

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
        echo "========================================="

        PARAM_FILE="/tmp/plane$${INSTANCE}_params.parm"
        echo "# Plane $${INSTANCE} Parameters" > $$PARAM_FILE
        echo "LOG_DISARMED 1" >> $$PARAM_FILE
        echo "LOG_BACKEND_TYPE 1" >> $$PARAM_FILE
        echo "DDS_ENABLE 1" >> $$PARAM_FILE
        echo "DDS_UDP_PORT $${DDS_UDP_PORT}" >> $$PARAM_FILE
        echo "DDS_IP0 $${DDS_IP0}" >> $$PARAM_FILE
        echo "DDS_IP1 $${DDS_IP1}" >> $$PARAM_FILE
        echo "DDS_IP2 $${DDS_IP2}" >> $$PARAM_FILE
        echo "DDS_IP3 $${DDS_IP3}" >> $$PARAM_FILE
        echo "DDS_DOMAIN_ID $${DDS_DOMAIN_ID}" >> $$PARAM_FILE
        echo "SYSID_THISMAV $${SYSID}" >> $$PARAM_FILE

        cd /ardupilot

        if [ "$$SKIP_BUILD" != "1" ]; then
          echo "Building ArduPlane..."
          git submodule update --init --recursive 2>/dev/null || true
          ./waf configure --board sitl --enable-DDS || (sleep 2 && ./waf configure --board sitl --enable-DDS)
          ./waf plane
        fi

        SIM_CMD="python3 Tools/autotest/sim_vehicle.py -v ArduPlane --model $${MODEL} --speedup $${SPEEDUP} --instance $${INSTANCE} --sim-address=$${SIM_ADDRESS} --add-param-file $$PARAM_FILE --enable-DDS"

        if [ "$${MAVPROXY_ENABLED}" = "0" ]; then
          SIM_CMD="$$SIM_CMD --no-mavproxy"
        else
          SIM_CMD="$$SIM_CMD --out=$${MAVPROXY_OUT}"
        fi

        if [ -n "$${HOME_LOCATION}" ]; then
          SIM_CMD="$$SIM_CMD --custom-location $${HOME_LOCATION}"
        fi

        if [ "$${WIPE}" = "1" ]; then
          SIM_CMD="$$SIM_CMD -w"
        fi

        touch /tmp/ArduPlane-$${INSTANCE}.log
        ln -sf /tmp/ArduPlane-$${INSTANCE}.log /ardupilot/logs/ArduPlane.log
        tail -f /tmp/ArduPlane-$${INSTANCE}.log &
        TAIL_PID=$$!
        trap "kill $$TAIL_PID 2>/dev/null || true" EXIT INT TERM

        exec $$SIM_CMD
EOF

    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

# Generate VTOL services (VTOLs are ArduPlane with quadplane model)
for ((i=1; i<=NUM_VTOLS; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    SITL_PORT=$((5501 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))

    # VTOLs reuse plane build
    SKIP_BUILD=1
    if [ $NUM_PLANES -gt 0 ]; then
        DEPENDS_ON="plane-1"
    elif [ $NUM_COPTERS -gt 0 ]; then
        # If no planes, build from scratch but depend on copter
        SKIP_BUILD=0
        DEPENDS_ON="copter-1"
    else
        # First vehicle, build from scratch
        SKIP_BUILD=0
        DEPENDS_ON=""
    fi

    cat >> "$OUTPUT_COMPOSE" << EOF

  vtol-$i:
    build:
      context: .
      dockerfile: Dockerfile
      args:
        BASE_IMAGE: ubuntu
        TAG: "22.04"
        SKIP_AP_GRAPHIC_ENV: 1
        SKIP_AP_EXT_ENV: 0
        SKIP_AP_COV_ENV: 1
        SKIP_AP_GIT_CHECK: 1
        DO_AP_STM_ENV: 0
    container_name: ardupilot-vtol-$i
    network_mode: host
    working_dir: /ardupilot
    volumes:
      - .:/ardupilot
      - ./logs:/tmp/buildlogs
      - ./sitl_logs/vtol-$i:/ardupilot/logs
      - vtol-$i-eeprom:/ardupilot/eeprom
    environment:
      - VEHICLE=ArduPlane
      - MODEL=\${VTOL${i}_MODEL:-quadplane}
      - INSTANCE=$INSTANCE
      - SYSID=\${VTOL${i}_SYSID:-$SYSID}
      - SPEEDUP=\${SPEEDUP:-1}
      - SIM_ADDRESS=\${SIM_ADDRESS:-127.0.0.1}
      - WIPE=\${WIPE:-0}
      - HOME_LOCATION=\${VTOL${i}_HOME:-}
      - ENABLE_DDS=\${ENABLE_DDS:-1}
      - DDS_ENABLE=1
      - DDS_UDP_PORT=$DDS_PORT
      - DDS_IP0=\${DDS_IP0:-127}
      - DDS_IP1=\${DDS_IP1:-0}
      - DDS_IP2=\${DDS_IP2:-0}
      - DDS_IP3=\${DDS_IP3:-1}
      - DDS_DOMAIN_ID=\${DDS_DOMAIN_ID:-0}
      - SKIP_BUILD=$SKIP_BUILD
      - MAVPROXY_ENABLED=\${VTOL${i}_MAVPROXY_ENABLED:-1}
      - MAVPROXY_OUT=\${VTOL${i}_MAVPROXY_OUT:-127.0.0.1:14550}
      - MAVPROXY_MASTER=\${VTOL${i}_MAVPROXY_MASTER:-tcp:127.0.0.1:5760}
      - MAVPROXY_SITL=\${VTOL${i}_MAVPROXY_SITL:-127.0.0.1:5501}
EOF

    if [ -n "$DEPENDS_ON" ]; then
        cat >> "$OUTPUT_COMPOSE" << EOF
    depends_on:
      - $DEPENDS_ON
EOF
    fi

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
    entrypoint: ["/bin/bash", "-c"]
    command:
      - |
        set -e
        source /home/ardupilot/.ardupilot_env

        echo "========================================="
EOF

    cat >> "$OUTPUT_COMPOSE" << EOF
        echo "ArduPilot VTOL $i (Instance $INSTANCE, SYSID $SYSID)"
        echo "-----------------------------------------"
        echo "PORT ASSIGNMENTS:"
        echo "  MAVLink:  $MAVLINK_PORT (TCP) - For Mission Planner/GCS"
        echo "  SITL:     $SITL_PORT (UDP) - For external simulator"
        echo "  DDS:      $DDS_PORT (UDP) - For ROS2 communication"
        echo "  MAVProxy: $((14550 + INSTANCE)) (UDP) - For telemetry output"
        echo ""
        echo "CONNECTIONS:"
        echo "  Mission Planner: tcp:127.0.0.1:$MAVLINK_PORT"
        echo "  ROS2 Agent: ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT"
EOF

    cat >> "$OUTPUT_COMPOSE" << 'EOF'
        echo "========================================="

        PARAM_FILE="/tmp/vtol$${INSTANCE}_params.parm"
        echo "# VTOL $${INSTANCE} Parameters" > $$PARAM_FILE
        echo "LOG_DISARMED 1" >> $$PARAM_FILE
        echo "LOG_BACKEND_TYPE 1" >> $$PARAM_FILE
        echo "DDS_ENABLE 1" >> $$PARAM_FILE
        echo "DDS_UDP_PORT $${DDS_UDP_PORT}" >> $$PARAM_FILE
        echo "DDS_IP0 $${DDS_IP0}" >> $$PARAM_FILE
        echo "DDS_IP1 $${DDS_IP1}" >> $$PARAM_FILE
        echo "DDS_IP2 $${DDS_IP2}" >> $$PARAM_FILE
        echo "DDS_IP3 $${DDS_IP3}" >> $$PARAM_FILE
        echo "DDS_DOMAIN_ID $${DDS_DOMAIN_ID}" >> $$PARAM_FILE
        echo "SYSID_THISMAV $${SYSID}" >> $$PARAM_FILE

        cd /ardupilot

        if [ "$$SKIP_BUILD" != "1" ]; then
          echo "Building ArduPlane (for VTOL)..."
          git submodule update --init --recursive 2>/dev/null || true
          ./waf configure --board sitl --enable-DDS || (sleep 2 && ./waf configure --board sitl --enable-DDS)
          ./waf plane
        fi

        SIM_CMD="python3 Tools/autotest/sim_vehicle.py -v ArduPlane --model $${MODEL} --speedup $${SPEEDUP} --instance $${INSTANCE} --sim-address=$${SIM_ADDRESS} --add-param-file $$PARAM_FILE --enable-DDS"

        if [ "$${MAVPROXY_ENABLED}" = "0" ]; then
          SIM_CMD="$$SIM_CMD --no-mavproxy"
        else
          SIM_CMD="$$SIM_CMD --out=$${MAVPROXY_OUT}"
        fi

        if [ -n "$${HOME_LOCATION}" ]; then
          SIM_CMD="$$SIM_CMD --custom-location $${HOME_LOCATION}"
        fi

        if [ "$${WIPE}" = "1" ]; then
          SIM_CMD="$$SIM_CMD -w"
        fi

        touch /tmp/ArduPlane-$${INSTANCE}.log
        ln -sf /tmp/ArduPlane-$${INSTANCE}.log /ardupilot/logs/ArduPlane.log
        tail -f /tmp/ArduPlane-$${INSTANCE}.log &
        TAIL_PID=$$!
        trap "kill $$TAIL_PID 2>/dev/null || true" EXIT INT TERM

        exec $$SIM_CMD
EOF

    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

# Add volumes section
cat >> "$OUTPUT_COMPOSE" << EOF

volumes:
EOF

for ((i=1; i<=NUM_COPTERS; i++)); do
    cat >> "$OUTPUT_COMPOSE" << EOF
  copter-$i-eeprom:
    driver: local
EOF
done

for ((i=1; i<=NUM_PLANES; i++)); do
    cat >> "$OUTPUT_COMPOSE" << EOF
  plane-$i-eeprom:
    driver: local
EOF
done

for ((i=1; i<=NUM_VTOLS; i++)); do
    cat >> "$OUTPUT_COMPOSE" << EOF
  vtol-$i-eeprom:
    driver: local
EOF
done

echo -e "${GREEN}✓ Generated $OUTPUT_COMPOSE${NC}"
echo ""

# Print summary
echo -e "${BLUE}=========================================${NC}"
echo -e "${GREEN}Generation Complete!${NC}"
echo -e "${BLUE}=========================================${NC}"
echo ""
echo -e "${YELLOW}Generated files:${NC}"
echo -e "  - $OUTPUT_COMPOSE"
echo -e "  - $OUTPUT_ENV"
echo -e "  - sitl_logs/ (with subdirectories for each vehicle)"
echo ""
echo -e "${YELLOW}Port Assignments:${NC}"

INSTANCE=0
SYSID=1

for ((i=1; i<=NUM_COPTERS; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "  Copter $i: MAVLink ${GREEN}$MAVLINK_PORT${NC}, DDS ${GREEN}$DDS_PORT${NC}, SYSID ${GREEN}$SYSID${NC}"
    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

for ((i=1; i<=NUM_PLANES; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "  Plane $i:  MAVLink ${GREEN}$MAVLINK_PORT${NC}, DDS ${GREEN}$DDS_PORT${NC}, SYSID ${GREEN}$SYSID${NC}"
    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

for ((i=1; i<=NUM_VTOLS; i++)); do
    MAVLINK_PORT=$((5760 + INSTANCE * 10))
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "  VTOL $i:   MAVLink ${GREEN}$MAVLINK_PORT${NC}, DDS ${GREEN}$DDS_PORT${NC}, SYSID ${GREEN}$SYSID${NC}"
    INSTANCE=$((INSTANCE + 1))
    SYSID=$((SYSID + 1))
done

echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo -e "  1. Start Micro ROS Agent (one per vehicle):"
echo ""

INSTANCE=0
TERMINAL_NUM=1

for ((i=1; i<=NUM_COPTERS; i++)); do
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "     ${GREEN}# Terminal $TERMINAL_NUM - Copter $i${NC}"
    echo -e "     ${GREEN}ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT${NC}"
    echo ""
    INSTANCE=$((INSTANCE + 1))
    TERMINAL_NUM=$((TERMINAL_NUM + 1))
done

for ((i=1; i<=NUM_PLANES; i++)); do
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "     ${GREEN}# Terminal $TERMINAL_NUM - Plane $i${NC}"
    echo -e "     ${GREEN}ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT${NC}"
    echo ""
    INSTANCE=$((INSTANCE + 1))
    TERMINAL_NUM=$((TERMINAL_NUM + 1))
done

for ((i=1; i<=NUM_VTOLS; i++)); do
    DDS_PORT=$((2019 + INSTANCE))
    echo -e "     ${GREEN}# Terminal $TERMINAL_NUM - VTOL $i${NC}"
    echo -e "     ${GREEN}ros2 run micro_ros_agent micro_ros_agent udp4 -p $DDS_PORT${NC}"
    echo ""
    INSTANCE=$((INSTANCE + 1))
    TERMINAL_NUM=$((TERMINAL_NUM + 1))
done

echo -e "  2. Start the swarm (in a new terminal):"
echo -e "     ${GREEN}docker compose -f $OUTPUT_COMPOSE --env-file $OUTPUT_ENV up${NC}"
echo ""

# Auto-start if requested
if [ "$AUTO_START" = true ]; then
    echo -e "${YELLOW}Auto-starting swarm...${NC}"
    echo ""
    docker compose -f "$OUTPUT_COMPOSE" --env-file "$OUTPUT_ENV" up
fi
