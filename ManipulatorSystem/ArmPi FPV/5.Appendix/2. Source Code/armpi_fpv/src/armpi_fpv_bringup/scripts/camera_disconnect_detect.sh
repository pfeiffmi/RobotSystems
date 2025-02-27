#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/armpi_fpv/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

python3 /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/scripts/camera_disconnect_detect.py
