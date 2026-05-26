FROM gitlab.lrz.de:5005/teleoperiertes_fahren/tod/tod_vehicle:latest AS tum_msgs

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ackermann-msgs \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ── CARLA Python API ──────────────────────────────────────────────────────────
RUN pip3 install --break-system-packages carla==0.9.16 \
    || pip3 install --break-system-packages carla==0.9.15 \
    || pip3 install --break-system-packages carla

# ── TUM message packages (full install — includes autoware_auto_msgs) ────────
# Copying the full install ensures autoware_auto_vehicle_msgs and
# autoware_auto_control_msgs are available for the Autoware integration path.
COPY --from=tum_msgs /home/tum/wsp/install /tum_install_baked

# ── CycloneDDS config ─────────────────────────────────────────────────────────
COPY cyclonedds.xml /tmp/cyclonedds.xml

# ── Bridge source ─────────────────────────────────────────────────────────────
COPY bridge_node.py /bridge/bridge_node.py
COPY start.sh /bridge/start.sh
RUN chmod +x /bridge/start.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///tmp/cyclonedds.xml
ENV ROS_DOMAIN_ID=7

CMD ["/bridge/start.sh"]
