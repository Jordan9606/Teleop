#!/bin/bash
# Monitors bridge logs for heartbeat loss and auto-restarts tod_vehicle_latest
# so its TCP command socket is re-opened before the operator tries to reconnect.
#
# Usage: ./reconnect_watchdog.sh &
# Stop:  pkill -f reconnect_watchdog.sh

COOLDOWN=15   # seconds to wait before watching for another event
RESTART_DELAY=1  # seconds to wait after HB loss before restarting

echo "[reconnect-watchdog] Started. Monitoring carla-ros2-bridge for heartbeat loss..."

while true; do
    # Block until a "Heartbeat lost" line appears in the bridge log
    if ! docker container inspect carla-ros2-bridge > /dev/null 2>&1; then
        sleep 3
        continue
    fi

    docker logs -f --since 0s carla-ros2-bridge 2>&1 | grep --line-buffered "Heartbeat lost" | \
    while IFS= read -r line; do
        echo "[reconnect-watchdog] $(date '+%H:%M:%S') — $line"
        echo "[reconnect-watchdog] Restarting tod_vehicle_latest in ${RESTART_DELAY}s..."
        sleep "$RESTART_DELAY"
        docker restart tod_vehicle_latest > /dev/null
        echo "[reconnect-watchdog] tod_vehicle_latest restarted — TCP socket ready. Reconnect operator now."
        sleep "$COOLDOWN"
    done

    # docker logs -f exited (bridge container stopped/restarted), loop and retry
    sleep 3
done
