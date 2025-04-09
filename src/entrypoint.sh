#!/bin/bash
set -e

# Load ROS environment
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# Environment configuration
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/ros_ws/install/json_mqtt_bridge/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONUNBUFFERED=1

echo "=== Environment configured ==="
echo "PYTHONPATH: $PYTHONPATH"
ros2 pkg list | grep json_mqtt_bridge || echo "Package not found"
echo "=============================="

# Check if the configuration file is provided via the environment variable CONFIG_FILE
if [ -z "$CONFIG_FILE" ]; then
    echo "❌ No configuration file provided. Usage: /ros_ws/src/json_mqtt_bridge/config/config.yaml"
    exit 1
fi

# Verify if the file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Configuration file not found: $CONFIG_FILE"
    exit 1
fi

echo "✅ Using configuration file: $CONFIG_FILE"

# Run with full error capture
while true; do
    echo "--- Starting node ---"
    python3 -u /ros_ws/src/json_mqtt_bridge/json_mqtt_bridge/bridge_mqtt_node.py "$CONFIG_FILE" 2>&1 | tee -a /ros_ws/bridge_log.txt
    EXIT_CODE=$?
    echo "The node exited with code $EXIT_CODE"
    
    # Show last log lines in case of an error
    if [ $EXIT_CODE -ne 0 ]; then
        echo "=== LAST LOG ENTRIES ==="
        tail -n 20 /ros_ws/bridge_log.txt
        echo "========================"
    fi
    
    sleep 5
done
