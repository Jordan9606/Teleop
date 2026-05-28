#!/bin/bash
# Monitors tod_vehicle_latest for VehicleRtspServer crashes.
# ROS2 launch respawns VehicleRtspServer automatically — no container restart needed.
# Just logs the event so you know to reconnect in TUM Visual.
# Run in background: nohup ./rtsp_watchdog.sh >> /tmp/rtsp_watchdog.log 2>&1 &

CONTAINER="tod_vehicle_latest"
echo "[rtsp_watchdog] Monitoring $CONTAINER for RtspServer crashes..."

docker logs -f "$CONTAINER" 2>&1 | while read -r line; do
    if echo "$line" | grep -q "VehicleRtspServer.*process has died"; then
        echo "[rtsp_watchdog] $(date): RtspServer crashed — ROS2 launch will respawn it. Reconnect in TUM Visual once streams are back."
    fi
    if echo "$line" | grep -q "RtspServer.*available at rtsp://"; then
        echo "[rtsp_watchdog] $(date): RtspServer ready — reconnect in TUM Visual now."
    fi
done
