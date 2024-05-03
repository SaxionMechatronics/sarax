#!/bin/bash

# sourceing 
echo "Tring to source ROS and the CATKIN workspace."
source ~/.bashrc

# Activate CAN
sudo ip link set can0 up type can bitrate 1000000
echo "Tried to connect to CAN, the CAN LED should be blinking."

# ROS Launch file
echo "Launching m4e_mani_impedance_control exp_all.launch ......"
roslaunch m4e_mani_base sarax_plus_exp_gps_nuc.launch

# exit gracefully by returning a status 
exit 0