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

set DIR=./test

if [ -d $DIR ]; then
	rm -R ./test
fi 

wget --quiet https://raw.githubusercontent.com/robotican/robots/master/$pn"_"$sn/doc/test -P ./test
if [ $? != 0 ]; then
	echo -e "\e[31m[Error]: Manual not found" 
	echo -en "\e[39m"
	exit 2
fi

wget --quiet https://raw.githubusercontent.com/robotican/robots/master/$pn"_"$sn/setup/setup.bash -P ./test
if [ $? != 0 ]; then
	echo -e "\e[31m[Error]: Setup not found" 
	echo -en "\e[39m"
	exit 2
fi

if [ $? == 0 ]; then
	echo -e "\e[32mCustom setup was downloaded successfully"
	echo -en "\e[39m"
fi

exit 0 
