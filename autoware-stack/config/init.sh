#!/bin/bash
export TTYD_PORT="5000"
export HOME="/root"
export DEBIAN_FRONTEND=noninteractive TZ="Etc/UTC" &&

apt -y update && 

rm -rf /tmp/.X1-lock
# killall websockify Xtigervnc ttyd launch.sh

ttyd -p $TTYD_PORT -t 'theme={"foreground":"#fff","background":"#111", "cursor":"#943124"}' bash &
