#!/bin/bash

# Define your ROS launch file
LAUNCH_FILE="integration_2.launch"
PACKAGE_NAME="integration"

# Number of restart attempts
MAX_RESTARTS=2
restart_count=0

# Running state
RUNNING=true

# Track PIDs for background processes
declare -a BACKGROUND_PIDS

# Trap SIGINT (Ctrl+C) and define cleanup actions
trap "terminate_script" SIGINT



kill_nodes() {
    echo "Killing all ROS nodes and background processes..."

    # Kill all ROS nodes gracefully
    rosnode list | grep -v "rosout" | xargs -I {} rosnode kill {} &>/dev/null

    # Kill specific background processes
    for pid in "${BACKGROUND_PIDS[@]}"; do
        kill $pid &>/dev/null || true
    done

    # Ensure launch process is killed if running
    if [[ -n $LAUNCH_PID ]]; then
        kill $LAUNCH_PID &>/dev/null || true
    fi

    # Kill all other related processes
    pkill -f rviz
    pkill -f "roscore"
    pkill -P $$  # Kill remaining child processes
}

start_launch_file() {
    echo "Starting ROS launch file..."
    roslaunch $PACKAGE_NAME $LAUNCH_FILE &
    LAUNCH_PID=$!
}

is_move_base_initialized() {
    if rosnode list | grep -q "/move_base"; then
        return 0  # True: move_base node is running
    else
        return 1  # False: move_base node is not running
    fi
}

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




get_latest_log_dir() {
    LATEST_LOG_DIR=$(ls -td /home/brilliant-technologies/.ros/log/*/ | head -n 1)
    echo "Most recent log directory: $LATEST_LOG_DIR"
    LOG_FILE_PATH="$LATEST_LOG_DIR/roslaunch-*.log"
    echo "Log file path: $LOG_FILE_PATH"
}

get_last_goal() {
    echo "Waiting for the last goal to be published..."
    LAST_GOAL=$(rostopic echo -n 1 /move_base_simple/goal)
    LAST_GOAL1=$(rostopic echo -n 1 /move_base/current_goal)
    echo "Last goal received and stored."
}

monitor_for_errors() {
    if [[ -z "$LOG_FILE_PATH" ]]; then
        echo "Log file path is not set. Exiting error monitoring."
        return
    fi

    ERROR_PATTERNS=("Aborting because a valid control could not be found. Even after executing all recovery behaviors")

    tail -n 0 -f "$LOG_FILE_PATH" | while read -r line; do
        for pattern in "${ERROR_PATTERNS[@]}"; do
            if [[ "$line" == *"$pattern"* ]]; then
                echo "Error detected: $pattern"
                # Ensure move_base is initialized before clearing costmaps and republishing the last goal
                if is_move_base_initialized; then
                    echo "Clearing costmaps..."
                    #rosservice call /move_base/clear_costmaps "{}"
                    clear_costmaps_periodically()
                    #sleep 2  # Wait for 2 seconds before publishing the last goal
                    if [[ -n $LAST_GOAL ]]; then
                        echo "Publishing last goal..."
                        rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "$LAST_GOAL"
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

clear_costmaps_periodically() {
    while $RUNNING; do
        # Check the status of the RUNNING flag explicitly
        if [[ "$RUNNING" == "false" ]]; then
            echo "Exiting clear_costmaps_periodically loop."
            break
        fi

        if is_move_base_initialized; then
            echo "Clearing costmaps periodiclly .."
            rosservice call /move_base/clear_costmaps "{}"
        else
            echo "move_base node is not initialized. Skipping costmap clear."
        fi
        sleep 0.2
    done
}

terminate_script() {
    echo "Caught SIGINT. Exiting gracefully..."
    RUNNING=false
    kill_nodes
    exit 0
}

# Main loop
while true; do

    
    start_launch_file

    # Get the most recent log directory
    get_latest_log_dir
    get_last_goal

    # Start periodic tasks in the background
    clear_costmaps_periodically &
    BACKGROUND_PIDS+=($!)

    monitor_for_errors &
    BACKGROUND_PIDS+=($!)

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
            terminate_script
        fi
    fi

    # Wait before next restart attempt
    sleep 5
done
wait 
