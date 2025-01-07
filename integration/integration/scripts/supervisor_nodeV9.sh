#!/bin/bash

# Define your ROS launch file
LAUNCH_FILE="integration_2.launch"
PACKAGE_NAME="integration"

# Number of restart attempts
MAX_RESTARTS=2

# Variable to store the last goal received
LAST_GOAL=""
LOG_FILE_PATH=""
LAUNCH_PID=""
RUNNING=true

# Trap SIGINT (Ctrl+C) and define cleanup actions
trap "echo 'Caught SIGINT. Exiting...'; RUNNING=false; kill_nodes; exit" SIGINT

# Function to kill all ROS nodes and background processes
kill_nodes() {
    echo "Killing all ROS nodes and background processes..."

    # Kill all ROS nodes gracefully
    rosnode list | grep -v "rosout" | xargs -I {} rosnode kill {}

    # Ensure the launch process is killed if it is running
    if [[ -n $LAUNCH_PID ]]; then
        kill $LAUNCH_PID &>/dev/null
    fi

    # Kill all other background processes (e.g., rviz)
    pkill -f rviz
    pkill -f "roscore"
    pkill -P $$  # Kill any remaining child processes of the script

    # Kill specific background tasks if running
    if [[ -n $COSTMAP_PID ]]; then
        kill $COSTMAP_PID &>/dev/null
    fi

    if [[ -n $MONITOR_PID ]]; then
        kill $MONITOR_PID &>/dev/null
    fi
}

# Function to check if move_base is initialized and running
is_move_base_initialized() {
    if rosnode list | grep -q "/move_base"; then
        return 0  # True: move_base node is running
    else
        return 1  # False: move_base node is not running
    fi
}

# Function to check errors and attempt to fix them
check_errors() {
    if ! ls /dev/ttyUSB0 &>/dev/null; then
        echo "LIDAR not connected or cannot find port. Please check the connection."
        return 1
    fi

    if ! ls /dev/ttyUSB1 &>/dev/null; then
        echo "IMU not connected or cannot find port. Please check the connection."
        return 1
    fi

    if ! ls /dev/ttyACM0 &>/dev/null; then
        echo "Arduino not connected or cannot find port. Please check the connection."
        return 1
    fi

    if ! rostopic list | grep -q "/position"; then
        echo "/position topic not being published, please check the rosserial node."
        return 1
    fi

    return 0
}

# Function to start the ROS launch file in a non-blocking way
start_launch_file() {
    echo "Starting ROS launch file..."
    
    # Run roslaunch in the background
    roslaunch $PACKAGE_NAME $LAUNCH_FILE &
    LAUNCH_PID=$!
}

# Function to get the most recent log directory
get_latest_log_dir() {
    LATEST_LOG_DIR=$(ls -td /home/brilliant-technologies/.ros/log/*/ | head -n 1)
    echo "Most recent log directory: $LATEST_LOG_DIR"
    LOG_FILE_PATH="$LATEST_LOG_DIR/roslaunch-*.log"
    echo "Log file path: $LOG_FILE_PATH"
}

clear_costmaps_periodically() {
    while $RUNNING; do
        if is_move_base_initialized; then
            echo "Clearing costmaps using bash script..."
            rosservice call /move_base/clear_costmaps "{}"
        else
            echo "move_base node is not initialized. Skipping costmap clear."
        fi
        sleep 1
    done
}

get_last_goal() {
    echo "Waiting for the last goal to be published..."
    LAST_GOAL=$(rostopic echo -n 1 /move_base/goal)
    echo "Last goal received and stored."
}

monitor_for_errors() {
    if [ -z "$LOG_FILE_PATH" ]; then
        echo "Log file path is not set. Exiting error monitoring."
        return
    fi

    ERROR_PATTERNS=("Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00"
                    "Aborting because a valid control could not be found. Even after executing all recovery behaviors"
                    "DWA planner failed to produce path")
    
    tail -n 0 -f "$LOG_FILE_PATH" | while read -r line; do
        for pattern in "${ERROR_PATTERNS[@]}"; do
            if [[ "$line" == *"$pattern"* ]]; then
                echo "Error detected: $pattern"
                if is_move_base_initialized; then
                    echo "Clearing costmaps..."
                    rosservice call /move_base/clear_costmaps "{}"
                    if [[ -n $LAST_GOAL ]]; then
                        echo "Publishing last goal..."
                        rostopic pub /move_base/goal geometry_msgs/PoseStamped "$LAST_GOAL"
                    else
                        echo "No last goal stored. Unable to republish."
                    fi
                else
                    echo "move_base node is not initialized. Skipping costmap clear and goal republish."
                fi
                break
            fi
        done
    done
}

# Main loop
restart_count=0

while $RUNNING; do
    start_launch_file

    # Get the most recent log directory
    get_latest_log_dir

    # Start the clear_costmaps_periodically function in the background
    clear_costmaps_periodically &
    COSTMAP_PID=$!

    # Get the last goal given to move_base
    get_last_goal

    # Start the error monitoring function in the background
    monitor_for_errors &
    MONITOR_PID=$!

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
