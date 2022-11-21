#!/bin/bash
jetson="$1"

if [ -z $1 ]; then
        echo "Jetson name is empty"
elif [ "$jetson" = "tx2" ]; then
        export ROS_IP=192.168.2.11
        export ROS_MASTER_URI=http://192.168.2.11:11311
elif [ "$jetson" = "nano" ]; then
        export ROS_IP=192.168.2.69
        export ROS_MASTER_URI=http://192.168.2.11:11311        
else
        echo "invalid Jetson name"
fi