#!/bin/bash
# Usage: ./mode.sh teleop | ./mode.sh auto
MODE=${1:-teleop}
[[ "$MODE" == "auto" ]] && FRAME="autonomous" || FRAME="teleop"

docker exec carla-ros2-bridge bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 topic pub --once /mode_switch geometry_msgs/msg/PoseStamped \
   '{header: {frame_id: \"'$FRAME'\"}}'" > /dev/null 2>&1

sleep 0.3
echo "--- Bridge logs ---"
docker logs carla-ros2-bridge 2>&1 | grep -E "Mode →|bridge ready" | tail -5
