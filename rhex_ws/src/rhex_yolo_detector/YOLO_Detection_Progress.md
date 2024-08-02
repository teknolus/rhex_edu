# YOLOv8 Human Detection with RHex Simulation Project Progress Documentation

## Introduction

We initiated this project by taking a simple supervised machine learning course to refresh our knowledge of AI and machine learning principles. Following this, we delved into the YOLOv8 (You Only Look Once) algorithm to understand its distinct advantages over other computer vision algorithms like Convolutional Neural Networks (CNNs).

## Data Acquisition and Preparation

Understanding the type of data required was crucial. We obtained our dataset from [Roboflow Universe](https://universe.roboflow.com/titulacin/person-detection-9a6mk/dataset/16), which consisted of 4407 training images and 1071 validation images. This dataset was essential for training our YOLO model.

## Training the YOLO Model

Training the YOLO model was relatively straightforward. We ran the training for 100 epochs, leveraging the high-quality labeled data. The simplicity of the YOLO architecture facilitated an efficient training process, resulting in a well-trained model ready for deployment.

## Integration with ROS2

The challenging part of the project was implementing the trained YOLO model within a simulation environment using ROS2 (Robot Operating System 2). Here’s a step-by-step overview of our integration process:

1. **Learning ROS2 Basics**: We learned the basics of ROS2 to understand how to work with topics, nodes, and messages.

2. **Subscribing to the Depth Camera Topic**: We subscribed to the `depth_camera` topic, which provided the necessary image data.

3. **Processing Image Data**: Using the `cvBridge` library, we converted the ROS image messages to OpenCV format for processing.

4. **Running YOLO Model**: We ran the processed images through our trained YOLO model to detect objects.

5. **Retrieving Robot Position**: We subscribed to the `Odometry` topic to retrieve the robot's position.

6. **Outputting Information**: We printed essential information such as the number of humans detected, the time of detection, and the robot's position relative to the origin.

By following these steps, we successfully integrated our YOLO model with ROS2, enabling real-time object detection and localization in a simulated environment.

## Tools Used

- **Machine Learning Framework**: YOLO (You Only Look Once)
- **Dataset**: [Roboflow Universe](https://universe.roboflow.com/titulacin/person-detection-9a6mk/dataset/16)
- **ROS2**: Robot Operating System 2 for simulation
- **cvBridge**: For converting ROS image messages to OpenCV format
- **OpenCV**: For image processing tasks

## Conclusion

This project has provided valuable insights into the integration of machine learning models with robotic systems, particularly within simulation environments. The combination of YOLO and ROS2 has proven to be effective for real-time object detection and robot localization.


## Contributors
- Hamzeh Awad
- Leen Said
