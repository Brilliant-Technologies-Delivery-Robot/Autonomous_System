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

terminate_script() {
    echo "Caught SIGINT. Exiting gracefully..."
    RUNNING=false
    kill_nodes
    exit 0
}

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

start_launch_file() {
    echo "Starting ROS launch file..."
    roslaunch $PACKAGE_NAME $LAUNCH_FILE &
    LAUNCH_PID=$!
}

clear_costmaps_periodically() {
    while $RUNNING; do
        if is_move_base_initialized; then
            echo "Clearing costmaps..."
            rosservice call /move_base/clear_costmaps "{}"
        else
            echo "move_base node is not initialized. Skipping costmap clear."
        fi
        sleep 1
    done
}

get_latest_log_dir() {
    # Use 'ls -td' to get the most recent directory
    LATEST_LOG_DIR=$(ls -td /home/brilliant-technologies/.ros/log/*/ | head -n 1)
    echo "Most recent log directory: $LATEST_LOG_DIR"
    LOG_FILE_PATH="$LATEST_LOG_DIR/roslaunch-*.log"
    echo "Log file path: $LOG_FILE_PATH"
}

get_last_goal() {
    echo "Waiting for the last goal to be published..."
    LAST_GOAL=$(rostopic echo -n 1 /move_base/goal)
    echo "Last goal received and stored."
}

monitor_for_errors() {
    if [[ -z "$LOG_FILE_PATH" ]]; then
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
                # Ensure move_base is initialized before clearing costmaps and republishing the last goal
                if is_move_base_initialized; then
                    echo "Clearing costmaps..."
                    #rosservice call /move_base/clear_costmaps "{}"
                    clear_costmaps_periodically()
                    #sleep 2  # Wait for 2 seconds before publishing the last goal
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
while $RUNNING; do
    start_launch_file

    get_latest_log_dir
    
    get_last_goal


    # Start periodic tasks in the background
   # clear_costmaps_periodically &
 #   BACKGROUND_PIDS+=($!)
    monitor_for_errors &
    BACKGROUND_PIDS+=($!)

    # Wait before restarting
    sleep 10

    if ! check_errors; then
        echo "Error detected. Restarting launch file..."
        kill_nodes
        restart_count=$((restart_count + 1))

        if [[ $restart_count -ge $MAX_RESTARTS ]]; then
            echo "Max restarts reached. Exiting..."
            terminate_script
        fi
    else
        echo "System running without errors."
        break
    fi
done
