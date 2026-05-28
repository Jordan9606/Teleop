#!/bin/bash
# Run on laptop — starts tod_operator only.
# Lab PC must already be running CARLA + bridge + tod_vehicle.
set -e
cd "$(dirname "$0")"

# shellcheck source=.env
[ -f .env ] && set -a && source .env && set +a

if [ -z "$OPERATOR_NETWORK_IFACE" ]; then
    echo "[laptop] ERROR: Set OPERATOR_NETWORK_IFACE in .env to your ZeroTier interface name."
    echo "[laptop] Run: ip link show | grep zt"
    exit 1
fi

echo "[laptop] Starting tod_operator (interface: $OPERATOR_NETWORK_IFACE)..."
docker compose up tod_operator
