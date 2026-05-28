#!/bin/bash
# Graphical view:
#   rqt_graph      — node connection graph (proves Autoware path)
#   rqt_plot       — live speed + steering over time
#   rqt_image_view — live CARLA camera feed as ROS topic
# Run from any directory. Requires sim running and DISPLAY set.

if [ -z "$DISPLAY" ]; then
    echo "[error] DISPLAY not set. Run from a desktop terminal."
    exit 1
fi

echo "[graph] Launching rqt_graph (node connections)..."
docker exec -e DISPLAY="$DISPLAY" tod_operator_latest bash -c \
    "source /opt/ros/jazzy/setup.bash && source /home/tum/wsp/install/setup.bash && \
     ros2 run rqt_graph rqt_graph" &

sleep 1

echo "[graph] Launching rqt_plot (live speed + steering)..."
docker exec -e DISPLAY="$DISPLAY" carla-ros2-bridge bash -c \
    "source /opt/ros/jazzy/setup.bash && source /tum_install_baked/setup.bash && \
     ros2 run rqt_plot rqt_plot \
       /vehicle/status/velocity_status/longitudinal_velocity \
       /external/selected/control_cmd/longitudinal/speed \
       /external/selected/control_cmd/lateral/steering_tire_angle" &

sleep 1

echo "[graph] Launching rqt_image_view (CARLA front camera)..."
docker exec -e DISPLAY="$DISPLAY" carla-ros2-bridge bash -c \
    "source /opt/ros/jazzy/setup.bash && source /tum_install_baked/setup.bash && \
     ros2 run rqt_image_view rqt_image_view /carla/ego_vehicle/frontcenter/image" &

echo "[graph] All windows launched. Ctrl+C to close."
wait
