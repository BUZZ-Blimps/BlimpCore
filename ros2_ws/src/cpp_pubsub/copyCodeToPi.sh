#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Removing old testing files from opi@$hostname"
		ssh opi@$hostname 'rm -r /home/opi/ros2_ws/src/cpp_pubsub'
		
		echo ">>Copying files to opi@$hostname"
		ssh opi@$hostname 'mkdir -p /home/opi/ros2_ws/src/cpp_pubsub'
		scp -r ./* opi@$hostname:/home/opi/ros2_ws/src/cpp_pubsub
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi