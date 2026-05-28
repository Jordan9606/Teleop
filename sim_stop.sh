#!/bin/bash
# Stop all simulation containers (CARLA, bridge, tod_vehicle).

cd "$(dirname "$0")"

echo "[sim] Stopping tod_vehicle..."
docker stop tod_vehicle_latest 2>/dev/null || true

echo "[sim] Stopping CARLA bridge..."
docker rm -f carla-ros2-bridge 2>/dev/null || true

echo "[sim] Stopping CARLA..."
docker stop carla-sim 2>/dev/null || true

echo "[sim] Stopping watchdogs (if running)..."
pkill -f rtsp_watchdog.sh 2>/dev/null || true
pkill -f reconnect_watchdog.sh 2>/dev/null || true

echo "[sim] All stopped."
