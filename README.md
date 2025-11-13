ARCS Vision – Object Detection System

Real-time YOLO-based object detection for the ARCS Watcher project using ZED stereo cameras and NVIDIA Jetson.

Overview

This package provides the Vision subsystem for the ARCS Watcher project. It runs YOLO object detection on ZED camera images and publishes bounding boxes, class IDs, and confidence scores to ROS 2 topics.
The detector connects to other ARCS subsystems including depth fusion, tracking, and mapping.

Pipeline:
ZED Camera → YOLO Detector → /yolo_detections → Downstream modules

System Architecture

Input: /zed/zed_node/rgb/image_rect_color (RGB frames from ZED)

Output: /yolo_detections (Detection2DArray)

Format: bounding boxes, class IDs, confidence scores

Frameworks: ROS 2 Jazzy, YOLO ONNX Runtime

Deployment: Docker container with NVIDIA runtime

Requirements
Hardware

NVIDIA Jetson (Orin, Xavier, Nano)

ZED2 / ZED2i camera

USB 3.0 connection

Software

Docker (with NVIDIA runtime)

Git

ZED SDK (installed on Jetson host)

Quick Start
1. Clone the repository
git clone https://github.com/amirgshabo/arcs_vision.git
cd arcs_vision

2. Build the Docker image
cd docker
docker build -t arcs_jazzy_ready .

3. Start the container (Jetson)
cd ~/arcs_vision

docker run -it \
  --privileged \
  --network host \
  --runtime nvidia \
  --device=/dev/video0 \
  --device=/dev/video1 \
  -v $(pwd)/ros2_jazzy_ws:/ros2_ws \
  --name arcs_ready \
  arcs_jazzy_ready bash

4. Initialize workspace inside the container
/ros2_ws/start.sh
touch yolo_env/COLCON_IGNORE      # prevent colcon from scanning venv
colcon build --packages-select arcs_yolo_detector arcs_yolo_test
source install/setup.bash

Running the System
A. Full System (with ZED + Jetson)
Terminal 1 – Start camera
docker start -ai arcs_ready
/ros2_ws/start.sh
ros2 launch zed_wrapper zed2.launch.py

Terminal 2 – Start YOLO detector
docker exec -it arcs_ready bash
/ros2_ws/start.sh
ros2 run arcs_yolo_detector yolo_onnx_node

Terminal 3 – Monitor detections
docker exec -it arcs_ready bash
/ros2_ws/start.sh
ros2 topic echo /yolo_detections

Option B – Practice at Home (No Jetson or ZED Required)
1. Start the container on Windows / Linux / Mac

Windows PowerShell

cd C:\ARCS_Vision\ros2_jazzy_ws
docker run -it --rm -v ${PWD}:/ros2_ws --name arcs_dev arcs_jazzy_ready bash


Linux / Mac

cd ~/arcs_vision/ros2_jazzy_ws
docker run -it --rm -v $(pwd):/ros2_ws --name arcs_dev arcs_jazzy_ready bash

2. Initialize workspace
/ros2_ws/start.sh
touch yolo_env/COLCON_IGNORE
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash

3. Run the dummy detection publisher (no hardware needed)
ros2 run arcs_yolo_test yolo_dummy_pub

4. Monitor the dummy detections
docker exec -it arcs_dev bash
/ros2_ws/start.sh
ros2 topic echo /fake_yolo_detections

5. Developer practice

Edit arcs_yolo_test/yolo_dummy_pub.py

Rebuild with:

colcon build --packages-select arcs_yolo_test
source install/setup.bash


Run again to test

ROS 2 Topics
Subscribed

/zed/zed_node/rgb/image_rect_color

Published

/yolo_detections (vision_msgs/Detection2DArray)
Includes:

Bounding boxes

Class IDs

Confidence

Troubleshooting
Camera not detected
lsusb | grep -i zed
ls /dev/video*

Build errors
touch yolo_env/COLCON_IGNORE
colcon build

Performance issues
sudo nvpmodel -m 0
sudo jetson_clocks

ONNX model missing
ls -lh /ros2_ws/onnx/model.onnx

Repository Structure
arcs_vision/
├── docker/
├── ros2_jazzy_ws/
│   ├── src/
│   │   ├── arcs_yolo_detector/
│   │   ├── arcs_yolo_test/
│   │   ├── zed-ros2-interfaces/
│   │   └── zed-ros2-wrapper/
│   ├── onnx/
│   ├── yolo_env/
│   └── start.sh
└── README.md

Restarting Later
docker start -ai arcs_ready
/ros2_ws/start.sh

Integration Details

Output from YOLO feeds Depth Sensing → adds 3D coordinates

Depth output feeds Tracking → tracks objects over time

Tracking feeds Mapping → builds global map

Your team (Vision) provides the /yolo_detections topic to the pipeline.

Testing Without Hardware

You can complete all of these at home:

Run dummy YOLO publisher

Echo detection topics

Modify detection messages

Practice colcon builds and debugging

Learn ROS2 tools (topic list, topic hz, node info)

Contributors

Vision Team Lead: Amir Shabo
Project: ARCS Watcher
Affiliation: ARCS Organization
