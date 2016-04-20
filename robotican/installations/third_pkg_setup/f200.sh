#!/bin/bash
source ~/catkin_ws/devel/setup.bash
cd ~
git clone https://github.com/richardw347/RealSense-ROS.git
cd ~/RealSense-ROS/f200_install
mkdir ~/ivcam
sh install.sh ~/ivcam
cd ~/RealSense-ROS/realsense_dist/2.3
./install_realsense_ros.sh


  


