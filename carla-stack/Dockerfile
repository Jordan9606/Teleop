FROM carlasim/carla:0.9.16

USER root

COPY config /config

# Fix broken NVIDIA repo
RUN rm -f /etc/apt/sources.list.d/cuda*.list \
    && rm -f /etc/apt/sources.list.d/nvidia*.list

ENV DEBIAN_FRONTEND=noninteractive TZ="Etc/UTC"

RUN apt-get update && apt-get install -y \
    terminator \
    net-tools \
    iputils-ping \
    psmisc \
    vim \
    nano \
    wget \
    websockify \
    xfce4 \
    xfce4-goodies \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

RUN cp /config/cyclonedds.xml /tmp/cyclonedds.xml && chmod +x /tmp/cyclonedds.xml

# ttyd
RUN wget https://github.com/tsl0922/ttyd/releases/download/1.6.3/ttyd.x86_64 && \
    mv ttyd.x86_64 /usr/local/bin/ttyd && \
    chmod +x /usr/local/bin/ttyd

# Optional init script
CMD ["bash"]