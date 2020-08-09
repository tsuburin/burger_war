#!/bin/bash

sudo dbus-uuidgen --ensure=/etc/machine-id

source /opt/ros/kinetic/setup.bash
source /home/burger/catkin_ws/devel/setup.bash

exec "$@"
