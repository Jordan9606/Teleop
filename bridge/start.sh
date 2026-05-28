#!/bin/bash
set -e

echo "[bridge] Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Source full baked-in TUM install (tod_vehicle_msgs + autoware_auto_msgs + tier4 msgs)
if [ -f /tum_install_baked/setup.bash ]; then
    echo "[bridge] Sourcing baked-in TUM install (autoware msgs included)..."
    source /tum_install_baked/setup.bash
fi

# Source volume-mounted TUM install if present (dev builds, overrides baked if newer)
if [ -f /tum_install/setup.bash ]; then
    echo "[bridge] Sourcing volume-mounted TUM install directory..."
    source /tum_install/setup.bash
fi

echo "[bridge] Starting BEV monitor server on port 8080..."
python3 /bridge/bev_server.py &

echo "[bridge] Starting CARLA bridge node..."
exec python3 /bridge/bridge_node.py \
    --ros-args \
    -p carla_host:="${CARLA_HOST:-127.0.0.1}" \
    -p carla_port:="${CARLA_PORT:-2000}" \
    -p spawn_point_index:="${CARLA_SPAWN_INDEX:-0}" \
    -p vehicle_filter:="${CARLA_VEHICLE:-vehicle.tesla.model3}"
