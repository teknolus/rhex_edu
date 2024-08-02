# RHex YOLO DETECTOR
This package utilizes the YOLOv8 object detection model to detect the presence of humans within the field of view of the robot's camera. It integrates with ROS 2 to process image data from the camera, apply the YOLOv8 model for detection, and publish the detection results.


## Table of Contents

- [Installation](#installation)
- [Technologies Used](#technologies-used)
- [Features](#features)
- [Usage](#usage)
- [Contributors](#contributors)

## Features
- **Real-time Human Detection**: Continuously monitors the robot's camera feed to detect humans in real-time.
- **Integration with ROS2**: Subscribes to camera image topics and publishes detection results to relevant topics for further processing.
- **Efficient and Accurate**: Leverages the YOLOv8 model for high accuracy and fast detection.

## Technologies Used
- **Languages**: Python
- **Libraries**: rclpy, OpenCV, ultralytics(YOLOv8), datetime
- **Tools**: Gazebo, ROS2

## Usage
1. **Install required libraries**: Make sure ultralytics and OepnCV are installed.
    ```bash
    pip install ultralytics
    pip install opencv-python
1. **Ensure Correct Model Path**: An example model (`best.pt`) which is trained to detect humans is provided, but you can use any custom YOLOv8 model. You can change the path from within the `yolo_subscriber.py` node
    ```bash
    self.detector = YOLO('/home/rhex/mnt/rhex_ws/src/rhex_yolo_detector/rhex_yolo_detector/best.pt')
4. **Run the Gazebo Simulation**: Launch the Gazebo simulation environment.
    ```bash
    ros2 launch rhex_gazebo start_sim.launch.py
5. **Run the YOLO Detection Node**: In a new terminal, source the workspace and run the YOLO detection node within the rhex_yolo_detector package.
    ```bash
    ros2 run rhex_yolo_detector yolo_subscriber
## Contributors
- **Leen Said**
- **Hamzeh Awad**


