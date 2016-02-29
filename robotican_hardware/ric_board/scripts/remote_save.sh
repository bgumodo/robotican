#!/bin/bash

#TODO enter the password
scp ~/catkin_ws/src/ric/ric_board/config/$1.yaml $2@$3:~/catkin_ws/src/ric/ric_board/config/
scp ~/catkin_ws/src/ric/ric_board/DATA/$1.RIC $2@$3:~/catkin_ws/src/ric/ric_board/DATA/
scp ~/catkin_ws/src/ric/ric_board/launch/$1.launch $2@$3:~/catkin_ws/src/ric/ric_board/launch/
