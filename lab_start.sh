#!/bin/bash
# Run on lab PC — builds bridge image, starts CARLA, bridge, and tod_vehicle.
# Laptop runs tod_operator separately (TUM Visual → Connect to 10.88.209.223).

set -e
cd "$(dirname "$0")"

[ -f .env ] && set -a && source .env && set +a

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ -z "$VEHICLE_NETWORK_IFACE" ]; then
    echo "[lab] ERROR: Set VEHICLE_NETWORK_IFACE in .env (should be ztqtitvw44)."
    echo "[lab] Run: ip link show | grep zt"
    exit 1
fi

echo "[lab] Building CARLA bridge image (uses layer cache — fast if only bridge_node.py changed)..."
docker build -t jazzy-project-group-test1-carla-bridge:latest \
    -f "${REPO_DIR}/bridge/Dockerfile" "${REPO_DIR}/bridge"

echo "[lab] Starting CARLA..."
docker start carla-sim

echo "[lab] Waiting for CARLA to be ready on port 2000..."
until ss -tlnp | grep -q ':2000'; do
    if [ "$(docker inspect -f '{{.State.Status}}' carla-sim 2>/dev/null)" != "running" ]; then
        echo "[lab] CARLA crashed, restarting..."
        docker restart carla-sim
    fi
    echo "[lab] still waiting..."
    sleep 5
done

echo "[lab] CARLA ready."
echo "[lab] Starting CARLA bridge (fresh container from rebuilt image)..."
docker rm -f carla-ros2-bridge 2>/dev/null || true

docker run -d \
    --name carla-ros2-bridge \
    --network=host \
    -e ROS_DOMAIN_ID=7 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e "CYCLONEDDS_URI=file:///tmp/cyclonedds.xml" \
    -e CARLA_HOST=127.0.0.1 \
    -e CARLA_PORT=2000 \
    -e CARLA_VEHICLE=vehicle.tesla.model3 \
    -e CARLA_SPAWN_INDEX=0 \
    jazzy-project-group-test1-carla-bridge:latest

echo "[lab] Starting tod_vehicle..."
docker start tod_vehicle_latest

echo ""
pkill -f reconnect_watchdog.sh 2>/dev/null || true
nohup bash "${REPO_DIR}/reconnect_watchdog.sh" >> /tmp/reconnect_watchdog.log 2>&1 &
echo "[lab] All started."
echo "[lab]   CARLA:     docker logs -f carla-sim"
echo "[lab]   Bridge:    docker logs -f carla-ros2-bridge"
echo "[lab]   Vehicle:   docker logs -f tod_vehicle_latest"
echo "[lab]   Monitor:   ./sim_monitor.sh"
echo "[lab]   RTSP watchdog (optional): ./rtsp_watchdog.sh &"
echo "[lab] Waiting for laptop (TUM Visual → Connect to 10.88.209.223)..."
