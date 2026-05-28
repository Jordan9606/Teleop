#!/bin/bash
set -e
cd "$(dirname "$0")"

echo "[sim] Starting CARLA..."
docker start carla-sim

echo "[sim] Waiting for CARLA to be ready..."
until python3 -c "import carla; c=carla.Client('127.0.0.1',2000); c.set_timeout(3.0); c.get_world()" 2>/dev/null; do
    # Restart if crashed (happens on first boot sometimes)
    if [ "$(docker inspect -f '{{.State.Status}}' carla-sim 2>/dev/null)" != "running" ]; then
        echo "[sim] CARLA crashed, restarting..."
        docker restart carla-sim
    fi
    sleep 5
done
echo "[sim] CARLA ready."

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[sim] Starting CARLA bridge..."
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
  -v "${REPO_DIR}/software-stack-main/bridge/bridge_node.py:/bridge/bridge_node.py:ro" \
  -v /home/jordan/teleoperated_driving/.dev_artifacts/install:/tum_install:ro \
  -v /home/jordan/teleoperated_driving/.dev_artifacts/build:/build:ro \
  jazzy-project-group-test1-carla-bridge:latest

echo "[sim] Starting TUM containers..."
docker compose up -d tod_vehicle tod_operator

echo "[sim] All started. Open TUM Visual and click Connect."
