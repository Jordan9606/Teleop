#!/bin/bash
set -e

echo "[lab] Starting CARLA..."
docker compose -f "$(dirname "$0")/docker-compose.yml" up -d carla

echo "[lab] Waiting for CARLA to be ready on port 2000..."
until ss -tlnp | grep -q 2000; do
    echo "[lab] still waiting..."
    sleep 5
done

echo "[lab] CARLA ready. Starting bridge..."
docker compose -f "$(dirname "$0")/docker-compose.yml" up -d carla-bridge

echo "[lab] Done. Running containers:"
docker ps --filter name=carla
