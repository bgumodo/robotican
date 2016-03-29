#!/bin/bash

function usage() {
	echo "Usage: ./setup -n <Name of root> -p <Robot P.N> -s <Robot S.N>"
}

OPTIND=1

if [ $# == 0 ]; then
	usage
	exit 1
fi

while getopts "n:p:s:" opt; do
	case $opt in
		n)
			name=$OPTARG
			;;
		p)
			pn=$OPTARG
			;;
		s)
			sn=$OPTARG
			;;
		:) 
			echo "Option $OPTARG requires an argument." >&2
			exit 1
			;;
		?)
			usage
			exit 1
			;;
	esac
done

if [ -z $pn ]; then
	echo -e "\e[31m[Error]: P.N can't be empty."
	echo -en "\e[39m"
	exit 1
fi

if [ -z $sn ]; then
	echo -e "\e[31m[Error]: S.N can't be empty."
	echo -en "\e[39m"
	exit 1
fi


numOfPaths=$(echo $ROS_PACKAGE_PATH | awk -F ':' '{print NF}')

if [ $numOfPaths -le 0 ]; then
	echo -e "\e[31m[Error]: ROS not found on the system please install see the manual http://wiki.ros.org/robotican/Tutorials/PC%20installation" 
	echo -en "\e[39m"
	exit 4
fi
 
CATKIN=$(echo $ROS_PACKAGE_PATH | tr ":" "\n" | awk -F '/' '{if ($2 == "home") {print}}') 
DIR=$CATKIN/robotican/robotican_robots/$name

if [ -d $DIR ]; then
	rm -R $DIR
fi 

wget --quiet https://raw.githubusercontent.com/robotican/robots/master/$pn"_"$sn/doc/manual -P $DIR/Manual
if [ $? != 0 ]; then
	echo -e "\e[31m[Error]: Manual not found" 
	echo -en "\e[39m"
	exit 2
fi

wget --quiet https://raw.githubusercontent.com/robotican/robots/master/$pn"_"$sn/setup/setup.bash -P $DIR/Setup
if [ $? != 0 ]; then
	echo -e "\e[31m[Error]: Setup not found" 
	echo -en "\e[39m"
	exit 2
fi

if [ $? == 0 ]; then
	echo -e "\e[32mCustom setup was downloaded successfully"
	echo -e "\e[32mBeginning robot custom setup..."
	echo -en "\e[39m"
	
	chmod +x $DIR/Setup/setup.bash
	sudo $DIR/Setup/setup.bash $name
	if [ $? == 0 ]; then
		echo -e "\e[32mCustom setup complete"
		echo -en "\e[39m"
	else
		echo -e "\e[31m[Error]: could not complate setup" 
		echo -en "\e[39m"
		exit 3
	fi
fi

exit 0 
