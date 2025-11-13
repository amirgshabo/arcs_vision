#!/bin/bash

cd /ros2_ws

# Activate YOLO virtual environment
echo ">>> Activating YOLO virtual environment"
source yolo_env/bin/activate

# Add virtual environment to PYTHONPATH so ROS2 can find the packages
export PYTHONPATH="/ros2_ws/yolo_env/lib/python3.12/site-packages:$PYTHONPATH"

# Source ROS2 Jazzy
echo ">>> Sourcing ROS2 Jazzy"
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
echo ">>> Sourcing workspace (if exists)"
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo ">>> Environment ready!"
echo "You can now run:"
echo "   ros2 run arcs_yolo_detector yolo_onnx_node"
echo "   ros2 run arcs_yolo_test yolo_dummy_pub"

exec bash
