#!/bin/bash
# Run on lab PC — starts CARLA, bridge, and tod_vehicle.
# Laptop runs tod_operator separately.
set -e
cd "$(dirname "$0")"

# shellcheck source=.env
[ -f .env ] && set -a && source .env && set +a

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ -z "$VEHICLE_NETWORK_IFACE" ]; then
    echo "[lab] ERROR: Set VEHICLE_NETWORK_IFACE in .env to your ZeroTier interface name."
    echo "[lab] Run: ip link show | grep zt"
    exit 1
fi

echo "[lab] Starting CARLA..."
docker start carla-sim

echo "[lab] Waiting for CARLA to be ready..."
until python3 -c "import carla; c=carla.Client('127.0.0.1',2000); c.set_timeout(3.0); c.get_world()" 2>/dev/null; do
    if [ "$(docker inspect -f '{{.State.Status}}' carla-sim 2>/dev/null)" != "running" ]; then
        echo "[lab] CARLA crashed, restarting..."
        docker restart carla-sim
    fi
    sleep 5
done
echo "[lab] CARLA ready."

echo "[lab] Starting CARLA bridge..."
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
  -v "${REPO_DIR}/config/dds/zerotier.xml:/tmp/cyclonedds.xml:ro" \
  -v "${REPO_DIR}/bridge/bridge_node.py:/bridge/bridge_node.py:ro" \
  jazzy-project-group-test1-carla-bridge:latest

echo "[lab] Starting tod_vehicle..."
docker compose -f "${REPO_DIR}/docker-compose.yaml" up -d tod_vehicle

echo "[lab] All started. Waiting for laptop to connect..."
