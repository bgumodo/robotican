#!/bin/bash

#Terminator
#sudo apt-get -y install terminator
#END 

#ROS install
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
#sudo apt-get update
#sudo apt-get -y install ros-indigo-desktop-full
#sudo rosdep init
#rosdep update
#sudo apt-get -y install python-rosinstall
#source /opt/ros/indigo/setup.bash
#mkdir -p ~/catkin_ws/src
#cd ~/catkin_ws/src
#catkin_init_workspace
#cd ~/catkin_ws/
#catkin_make
#source devel/setup.bash
#echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd ~/catkin_ws/src
if [ $? == 0 ]; then
	
	#Third party apt-get pkgs
	cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
	sudo ./setup.sh
	
	#USB rules setup
	cd ~/catkin_ws/src/robotican/robotican/installations/usb_rules_setup
	sudo ./setup.sh
	
	#Changing to user.
	sudo chown -R $(logname):$(logname) ~/catkin_ws
	
	echo "Do you want to install f200/r200 camera package [y/n]: "
	read asf
	
	if [ $asf == "y" ]; then
		cd ~/catkin_ws/src/robotican/robotican/installations/third_pkg_setup
		./f200.sh
	fi
	
	chown -R $(logname):$(logname) ~/catkin_ws
	
	#Do this in the end of the installation.
	if [ $asf == "y" ]; then
		echo -e "\e[31mWarning: To complete f200/r200 installation the PC need to reboot."
		echo -en "\e[39mReboot the PC [y/n]:"
		read as

		if [ $as == "y" ]; then
			sudo reboot
		fi
	fi

else
	echo -e "\e[31m[Error]: ROS catkin_ws not found"
	echo -en "\e[39m"
	exit 1
fi

exit
#END
