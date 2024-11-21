#!/bin/bash

user_dir="/home/opi"
user="opi"
# user_dir="/root"
# user="root"

if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Removing old testing files from $user@$hostname"
		ssh $user@$hostname "rm -r $user_dir/ros2_ws/src/blimp_imu_calibration"
		
		echo ">>Copying files to $user@$hostname"
		ssh $user@$hostname "mkdir -p $user_dir/ros2_ws/src/blimp_imu_calibration"
		scp -r ./* $user@$hostname:$user_dir/ros2_ws/src/blimp_imu_calibration
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
