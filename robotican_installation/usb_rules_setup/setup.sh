#!/bin/bash

#echo -e "\e[34mStarting setup...\e[0m"
#echo -e "\e[34mInstalling python-tk package...\e[0m"
#apt-get -y install python-tk
#echo -e "\e[34mFinish installing python-tk package...\e[0m"
#echo -e "\e[34mInstalling idle package...\e[0m"
#apt-get -y install idle
#apt-get -y install python-qt4
#echo -e "\e[34mFinish installing idle package..\e[0m"
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

echo -e "\e[34mSetting usb rules..\e[0m"
cp $DIR/ric_usb.rules /etc/udev/rules.d
echo -e "\e[34mRestarting rules..\e[0m"
/etc/init.d/udev reload
echo -e "\e[34mInstallation is complete..\e[0m"



