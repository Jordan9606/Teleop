#!/bin/bash
# Watchdog for VehicleRtspServer — restarts tod_vehicle_latest if the process dies.
# Run before the demo: ./rtsp_watchdog.sh &
# Stop with: kill %1  or  pkill -f rtsp_watchdog.sh

CHECK_INTERVAL=5   # seconds between checks
RESTART_COOLDOWN=30  # seconds to wait after a restart before checking again

echo "[watchdog] Starting. Monitoring VehicleRtspServer in tod_vehicle_latest..."

while true; do
    if ! docker exec tod_vehicle_latest pgrep -af VehicleRtspServer | grep -qv grep 2>/dev/null; then
        echo "[watchdog] $(date '+%H:%M:%S') VehicleRtspServer not found — restarting tod_vehicle_latest..."
        docker restart tod_vehicle_latest
        echo "[watchdog] Restart issued. Waiting ${RESTART_COOLDOWN}s for startup..."
        sleep "$RESTART_COOLDOWN"
        echo "[watchdog] Resumed monitoring."
    fi
    sleep "$CHECK_INTERVAL"
done
