#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Removing old testing files from root@$hostname"
		ssh root@$hostname 'rm -r /root/testing_uart/'
		
		echo ">>Copying files to root@$hostname"
		ssh root@$hostname 'mkdir -p /root/testing_uart'
		scp -r ./* root@$hostname:/root/testing_uart/
		
		echo "Compiling code."
		ssh root@$hostname 'make -C /root/testing_uart'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
