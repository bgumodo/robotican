#!/bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

echo -e "\e[34mSetting usb rules..\e[0m"
cp $DIR/ric_usb.rules /etc/udev/rules.d
cp $DIR/hokuyo.rules /etc/udev/rules.d
sudo adduser $(logname) dialout
adduser 
echo -e "\e[34mRestarting rules..\e[0m"
/etc/init.d/udev reload
echo -e "\e[34mInstallation is complete..\e[0m"



