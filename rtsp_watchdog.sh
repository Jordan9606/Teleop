#!/bin/bash
# Watches tod_vehicle_latest for VehicleRtspServer crashes and auto-restarts.
# Run in background: nohup ./rtsp_watchdog.sh >> /tmp/rtsp_watchdog.log 2>&1 &

CONTAINER="tod_vehicle_latest"
echo "[rtsp_watchdog] Monitoring $CONTAINER for RtspServer crashes..."

docker logs -f "$CONTAINER" 2>&1 | while read -r line; do
    if echo "$line" | grep -q "VehicleRtspServer.*process has died"; then
        echo "[rtsp_watchdog] $(date): VehicleRtspServer crashed — restarting $CONTAINER..."
        docker restart "$CONTAINER"
        echo "[rtsp_watchdog] $(date): $CONTAINER restarted. Reconnect in TUM Visual."
        # Re-attach to the new log stream after restart
        exec "$0"
    fi
done
