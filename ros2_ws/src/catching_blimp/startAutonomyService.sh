#!/bin/bash

# user="opi"
user="root"

if [ "$1" != "" ]; then
    hostname=$1

    # Check if device is online
    timeout=2
    ping $hostname -c 1 -W $timeout > /dev/null
    if [ $? == 0 ]; then
        ssh $user@$hostname 'systemctl start catching_blimp'
        echo "Started autonomy service on $hostname."

    else
        echo "$hostname is offline :("
    fi
else
    echo "Usage: startAutonomyService.sh [hostname]"
fi
