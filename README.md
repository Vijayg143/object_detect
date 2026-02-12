#This project implements real-time object detection in ROS2 using a webcam and the YOLOv8 deep learning model.
#It publishes camera frames as ROS topics and performs object detection using the Ultralytics YOLOv8 model.


# Clone Workspace

# Install Dependencies
pip install numpy==1.26.4
pip install opencv-python==4.8.1.78
pip install ultralytics --no-deps
pip install pyyaml pillow requests scipy matplotlib

# Build Package
cd ~/object_detect
colcon build
source install/setup.bash

# Running the Project
# Start Camera Publisher
ros2 run object_detect camera_publisher     
# Start Object Detector
ros2 run object_detect object_detector

# YOLO Model :- yolov8m.pt
# Why Medium Model?
 - Better accuracy than nano model
 - Balanced performance on laptops
 - Reduced false detections

# ROS Topics
# Topic	                Type    	Description
/camera/image_raw	Image	        Raw webcam frames
/detections	        Image	        Processed frames with bounding boxes
