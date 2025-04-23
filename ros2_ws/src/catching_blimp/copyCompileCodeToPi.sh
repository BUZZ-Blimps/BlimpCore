#!/bin/bash

# user_dir="/home/opi"
# user="opi"
user_dir="/root"
user="root"

if [ "$1" != "" ]; then
    hostname=$1

    # Check if device is online
    timeout=2
    ping $hostname -c 1 -W $timeout > /dev/null
    if [ $? == 0 ]; then
        # echo ">>Removing old testing files from $user@$hostname"
        # ssh $user@$hostname "rm -r $user_dir/ros2_ws/src/catching_blimp"
        
        # echo ">>Copying files to $user@$hostname"
        # ssh $user@$hostname "mkdir -p $user_dir/ros2_ws/src/catching_blimp"
        # scp -r ./* $user@$hostname:$user_dir/ros2_ws/src/catching_blimp
        
        # echo ">>Done."

        rsync -rt ../catching_blimp $user@$hostname:$user_dir/ros2_ws/src
        echo "Copied code to $hostname."

        ssh $user@$hostname 'cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build'

    else
        echo "$hostname is offline :("
    fi
else
    echo "Usage: copyCompileCodeToPi.sh [hostname]"
fi
