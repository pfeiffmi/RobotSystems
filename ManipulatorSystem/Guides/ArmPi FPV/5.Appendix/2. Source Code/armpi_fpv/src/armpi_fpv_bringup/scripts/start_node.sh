#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ubuntu/armpi_fpv/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

roslaunch /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/launch/start_dependence.launch &
sleep 10
sudo systemctl start start_camera.service &
sleep 10
sudo /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/scripts/start_sensor_node.sh &
sleep 10
roslaunch /home/ubuntu/armpi_fpv/src/hiwonder_servo_controllers/launch/start.launch &
sleep 10
roslaunch /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/launch/start_functions.launch
