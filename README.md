# Assignment
Keypoint Detection Using YOLOv8 and ROS2 in a Simulated Environment

This project demonstrates the integration of YOLOv8-Pose with ROS2 Humble to perform real-time human keypoint detection in a Gazebo-based simulated environment. A mobile two-wheeled robot equipped with an RGB camera observes a human actor, processes visual data using a deep learning model, and publishes detected pose keypoints as native ROS2 messages.

The system was developed and tested entirely within TheConstruct.ai ROS2 simulation environment, enabling safe, repeatable experimentation without physical hardware.

Objectives:-
- Design a ROS2-based perception pipeline for pose estimation
- Integrate a YOLOv8-Pose deep learning model into a ROS2 node
- Simulate a mobile robot equipped with an RGB camera in Gazebo
- Perform real-time human keypoint detection from camera data
- Publish detected keypoints using ROS2 messages (PoseArray)
- Visualise perception outputs using RViz2
