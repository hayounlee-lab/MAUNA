#!/bin/bash

set -e

# activate virtual environment
source new_venv/bin/activate

# Source ROS 2 Humble startup
source /opt/ros/humble/setup.bash

docker restart ollama

echo "Running main (Commands.py)"
python3 text/controller_local_llm.py

