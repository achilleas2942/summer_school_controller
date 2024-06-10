#!/bin/bash
# Source the ROS setup script
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Function to set up network conditions
setup_tc() {
    # Remove existing configurations if they exist
    tc qdisc del dev lo root 2>/dev/null

    # Set up new network configurations
    tc qdisc add dev lo root handle 1: htb default 10
    tc class add dev lo parent 1: classid 1:1 htb rate 100mbit
    tc qdisc add dev lo parent 1:1 handle 10: netem delay 25ms
    tc filter add dev lo protocol ip parent 1:0 prio 1 u32 match ip protocol 17 0xff flowid 1:1
}

# Set up network conditions
setup_tc

# # Start the Python scripts in the background
# python3 /root/catkin_ws/src/summer_school_controller/src/odometry_server.py &
# python3 /root/catkin_ws/src/summer_school_controller/src/command_client.py &

# Execute the main command (ROS core or another ROS node)
exec "$@"

