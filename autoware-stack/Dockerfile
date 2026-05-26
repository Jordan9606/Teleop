FROM ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy

COPY config /config

RUN export DEBIAN_FRONTEND=noninteractive TZ="Etc/UTC" && \
    apt -y update && \
    apt -y install terminator ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro ros-jazzy-topic-tools ros-jazzy-rviz2 ros-jazzy-rviz-common ros-jazzy-rviz-default-plugins && \
    apt install -y ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher ros-jazzy-tf-transformations python3-yaml \
    net-tools iputils-ping psmisc vim nano && \
    apt -y install wget && \
    wget https://github.com/tsl0922/ttyd/releases/download/1.6.3/ttyd.x86_64 && \
    cp ttyd.x86_64 /usr/local/bin/ttyd && \
    chmod +x /usr/local/bin/ttyd && \
    cp /config/cyclonedds.xml /tmp/cyclonedds.xml && chmod +x /tmp/cyclonedds.xml && \
    apt -y install websockify && \
    apt -y install xfce4 xfce4-goodies && \
    apt -y install x11vnc && \
    apt -y clean && \
    apt -y remove && \
    apt -y autoclean && \
    apt -y autoremove && \
    rm -rf /var/cache/apt

WORKDIR /home/ros2_ws/src

RUN /bin/bash -c "git clone https://github.com/carla-simulator/ros-carla-msgs.git"

WORKDIR /home/ros2_ws

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Environment setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "ROS_DOMAIN_ID=0" >> ~/.bashrc
RUN echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "CYCLONEDDS_URI=file:///tmp/cyclonedds.xml" >> ~/.bashrc


CMD bash /config/init.sh && bash