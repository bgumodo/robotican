#!/bin/bash
cd ~
git clone https://github.com/OpenKinect/libfreenect2.git
sudo chown -R $(logname):$(logname) ~/libfreenect2

cd libfreenect2
cd depends; ./download_debs_trusty.sh
sudo apt-get -y install build-essential cmake pkg-config
sudo dpkg -i debs/libusb*deb
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev
sudo dpkg -i debs/libglfw3*deb; sudo apt-get -y install -f
sudo apt-add-repository ppa:floe/beignet; sudo apt-get update; sudo apt-get -y  install beignet-dev; sudo dpkg -i debs/ocl-icd*deb
sudo dpkg -i debs/{libva,i965}*deb; sudo -y apt-get install -f
sudo apt-get -y  install libopenni2-dev
cd ..
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON
make
make install

cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
sudo chown -R $(logname):$(logname) ~/catkin_ws
cd iai_kinect2
rosdep install -r --from-paths .
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"

echo -e "\e[34mInstalltion completed...\e[0m"
echo -en "\e[39m"
echo "Add following command to /etc/rc.local and reboot your PC"
echo "sudo sh -c \"echo 0 > /sys/module/i915/parameters/enable_cmd_parser\""
