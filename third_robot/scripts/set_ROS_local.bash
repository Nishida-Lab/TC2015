#!/bin/bash
eth0_addr=$(ip -f inet -o addr show eth0|cut -d\  -f 7 | cut -d/ -f 1)
wlan0_addr=$(ip -f inet -o addr show wlan0|cut -d\  -f 7 | cut -d/ -f 1)

if [ -z "$eth0_addr" ]; then
	echo "No eth0 connection..."
	if [ -z  "$wlan0_addr" ]; then
		echo "No wlan0 connection..."
		export ROS_MASTER_URI=http://localhost:11311
		unset ROS_HOST_NAME
		unset ROS_IP
	else
		echo "There are wlan0 connection"
		export ROS_MASTER_URI=http://${wlan0_addr}:11311
		export ROS_HOST_NAME=${wlan0_addr}
		export ROS_IP=${wlan0_addr}		
	fi	
else
	echo "There are eth0 connection"
	export ROS_MASTER_URI=http://${eth0_addr}:11311
	export ROS_HOST_NAME=${eth0_addr}
	export ROS_IP=${eth0_addr}		
fi

env | grep "ROS_"
