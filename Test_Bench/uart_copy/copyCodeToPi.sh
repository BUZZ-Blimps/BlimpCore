#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Removing old testing files from opi@$hostname"
		ssh opi@$hostname 'rm -r /home/opi/uart/'
		
		echo ">>Copying files to opi@$hostname"
		ssh opi@$hostname 'mkdir -p /home/opi/uart'
		scp -r ./* opi@$hostname:/home/opi/uart/
		
		echo "Compiling code."
		ssh opi@$hostname 'make -C /home/opi/uart'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi