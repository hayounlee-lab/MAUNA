#!/bin/bash
export INTERBOTIX_ALOHA_IS_MOBILE=true # true for Mobile, false for Stationary
source /opt/ros/humble/setup.bash # configure ROS system install environment
source ~/interbotix_ws/install/setup.bash # configure ROS workspace environment
#cd ~/interbotix_ws/src/aloha/scripts/

# Function to display usage instructions
usage() {
    echo "Usage: $0 {0|0n|1b|1n|2|2f1|3}"
    echo "Example: $0 0"
    exit 1
}

# Check if the argument count is incorrect
if [ $# -ne 1 ]; then
    usage
fi

TAG_TYPE=$1

# 2. Branch processing based on tag type
case "$TAG_TYPE" in
    1)
	python3 ../internal/rotate.py --target -180
        ;;
    2)
	python3 ../internal/rotate.py
        ;;
    3)
	python3 ../internal/rotate.py --target 90
        ;;
    w)
	python3 ../internal/rotate.py 
        ;;
    *)
        echo "Error: Unsupported tag option."
        usage
        ;;
esac
