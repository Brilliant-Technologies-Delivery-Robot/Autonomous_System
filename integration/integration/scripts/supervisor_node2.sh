#!/bin/bash

# Define your ROS launch file
LAUNCH_FILE="integration.launch"
PACKAGE_NAME="integration"

# Number of restart attempts
MAX_RESTARTS=2

# Trap SIGINT (Ctrl+C) and define cleanup actions
trap "echo 'Caught SIGINT. Exiting...'; kill_nodes; exit" SIGINT

# Variable to store the last goal received
LAST_GOAL=""

# Function to check errors
check_errors() {
    # Check if lidar is connected
    if ! ls /dev/ttyUSB0 &>/dev/null; then
        echo "LIDAR not connected or cannot find port, please reconnect LIDAR. If the problem persists, check the USB port number"
        echo "System will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if imu is connected
    if ! ls /dev/ttyUSB1 &>/dev/null; then
        echo "IMU not connected or cannot find port, please reconnect IMU. If the problem persists, check the USB port number"
        echo "System will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if Arduino is connected
    if ! ls /dev/ttyACM0 &>/dev/null; then
        echo "Arduino not connected or cannot find port, please reconnect Arduino. If the problem persists, check the USB port number"
        echo "System will restart in 10 seconds"
        sleep 10
        return 1
    fi
    # Check if /position topic is being published
    if ! rostopic list | grep -q "/position"; then
        echo "/position topic not being published, please check the rosserial node"
        echo "System will restart in 10 seconds"
        sleep 10
        return 1
    fi

    return 0
}

# Function to kill all ROS nodes
kill_nodes() {
    echo "Killing all ROS nodes..."
    rosnode kill -a &>/dev/null
    if [[ -n $LAUNCH_PID ]]; then
        kill $LAUNCH_PID &>/dev/null
    fi
}

# Function to start the ROS launch file
start_launch_file() {
    echo "Starting ROS launch file..."
    roslaunch $PACKAGE_NAME $LAUNCH_FILE &
    LAUNCH_PID=$!
}

# Function to clear costmaps periodically
clear_costmaps_periodically() {
    while true; do
        echo "Clearing costmaps using bash script..."
        rosservice call /move_base/clear_costmaps "{}"
        sleep 30  # Adjust the interval as needed (e.g., every 30 seconds)
    done
}

# Function to get the last goal given to move_base and store the full message
get_last_goal() {
    echo "Waiting for the last goal to be published..."
    LAST_GOAL=$(rostopic echo -n 1 /move_base/goal)
    echo "Last goal received and stored."
}

# Function to monitor the terminal for specific errors and respond accordingly
monitor_for_errors() {
    # Define error patterns to search for
    ERROR_PATTERNS=("Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00"
                    "Aborting because a valid control could not be found. Even after executing all recovery behaviors"
                    "DWA planner failed to produce path")
    
    # Continuously monitor the ROS logs for errors
    tail -n 0 -f /delivery_robot/log | while read -r line; do
        for pattern in "${ERROR_PATTERNS[@]}"; do
            if [[ "$line" == *"$pattern"* ]]; then
                echo "Error detected: $pattern"
                echo "Clearing costmaps error pattern detected..."
                rosservice call /clear_costmaps "{}"
                sleep 2  # Wait for 2 seconds before publishing the last goal
                if [[ -n $LAST_GOAL ]]; then
                    echo "Republishing last goal..."
                    rostopic pub /move_base/goal geometry_msgs/PoseStamped "$LAST_GOAL"
                else
                    echo "No last goal stored. Unable to republish."
                fi
                break
            fi
        done
    done
}

# Main loop
restart_count=0

while true; do
    start_launch_file

    # Start the clear_costmaps_periodically function in the background
    clear_costmaps_periodically &

    # Get the last goal given to move_base
    get_last_goal

    # Start the error monitoring function in the background
    monitor_for_errors &

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
        restart_count=$((restart_count + 1))

        if [ $restart_count -ge $MAX_RESTARTS ]; then
            echo "Max restarts reached. Exiting..."
            kill_nodes
            exit 1
        fi
    fi

    # Wait before next restart attempt
    sleep 5
done
