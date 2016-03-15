#!/bin/bash

echo -n "Please enter your robot P.N and press [ENTER]: "
read pn
echo -n "Please enter your robot S.N and press [ENTER]: "
read sn

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

DIR=./Robot

if [ -d $DIR ]; then
	rm -R $DIR
fi 

wget --quiet https://raw.githubusercontent.com/robotican/robots/master/$pn"_"$sn/doc/test -P $DIR/Manual
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
	sudo $DIR/Setup/setup.bash
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
