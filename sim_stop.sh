#!/bin/bash
cd "$(dirname "$0")"

echo "[sim] Stopping TUM containers..."
docker compose stop tod_vehicle tod_operator

echo "[sim] Stopping CARLA bridge..."
docker rm -f carla-ros2-bridge 2>/dev/null || true

echo "[sim] Stopping CARLA..."
docker stop carla-sim

echo "[sim] All stopped."
