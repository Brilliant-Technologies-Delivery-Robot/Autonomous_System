#!/bin/bash

# Define your ROS launch file
LAUNCH_FILE="integration.launch"
PACKAGE_NAME="integration"

# Number of restart attempts
MAX_RESTARTS=2

# Function to check errors
check_errors() {
    # Check if lidar is connected
    if ! ls /dev/ttyUSB0 &>/dev/null; then
        echo "LIDAR not connected or cannot find port, please reconnect LIDAR. If the problem persists, check the USB port number"
        echo "system will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if imu is connected
    if ! ls /dev/ttyUSB1 &>/dev/null; then
        echo "IMU not connected or cannot find port, please reconnect IMU. If the problem persists, check the USB port number"
        echo "system will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if arduino is connected
    if ! ls /dev/ttyACM0 &>/dev/null; then
        echo "Arduino not connected or cannot find port, please reconnect Arduino. If the problem persists, check the USB port number"
        echo "system will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if /position topic is being published
    if ! rostopic list | grep -q "/position"; then
        echo "/position topic not being published, please check the rosserial node"
        echo "system will restart in 10 seconds"
        sleep 10
        return 1
    fi

    return 0
}

# Function to kill all ROS nodes
kill_nodes() {
    echo "Killing all ROS nodes..."
    rosnode kill -a
}

# Function to start the ROS launch file
start_launch_file() {
    echo "Starting ROS launch file..."
    roslaunch $PACKAGE_NAME $LAUNCH_FILE &
    LAUNCH_PID=$!
}

# Main loop
restart_count=0

while true; do
    start_launch_file

    # Wait for a few seconds to allow nodes to start
    sleep 10

    # Check for errors
    check_errors
    if [ $? -eq 0 ]; then
        echo "System running without errors"
        break
    else
        echo "Error detected, restarting launch file..."
        kill_nodes
        kill $LAUNCH_PID
        restart_count=$((restart_count + 1))

        if [ $restart_count -ge $MAX_RESTARTS ]; then
            echo "Max restarts reached. Exiting..."
            exit 1
        fi
    fi

    # Wait before next restart attempt
    sleep 5
done
