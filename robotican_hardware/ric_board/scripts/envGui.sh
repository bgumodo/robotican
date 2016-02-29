#!/bin/bash	
export ROS_MASTER_URI="http://$1:11311"
export ROS_HOSTNAME="$2"
echo "http://$1:11311"
echo "$2"
/bin/bash
