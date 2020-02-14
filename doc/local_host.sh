#!/bin/bash
export ROS_MASTER_URI='http://localhost:11311'
echo ROS_MASTER_URI=$ROS_MASTER_URI
MY_IP=$(ifconfig wlp2s0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
export ROS_IP=$MY_IP
echo ROS_IP=$ROS_IP
export ROS_HOSTNAME=$MY_IP
echo ROS_HOSTNAME=$ROS_HOSTNAME
