# Owl-tonomous Car 🚗🦉

This project implements a small autonomous vehicle capable of following a predefined track using computer vision and control techniques. The system was developed as part of the ELEC 553 course.

## Overview

The vehicle uses a camera and onboard processing (Raspberry Pi) to detect lane markings and navigate the track in real time. Steering and speed are controlled using PID controllers, enabling stable and smooth motion.

## Features

- Lane detection using OpenCV and HSV color segmentation  
- Polynomial fitting for trajectory estimation  
- Steering control using PID  
- Speed regulation using an optical encoder for feedback  
- Red signal detection for stop behavior  
- Object detection using YOLOv5 on Raspberry Pi  

## System Architecture

- **Camera**: captures real-time video of the track  
- **Raspberry Pi**: processes images and runs control algorithms  
- **Servo Motor**: controls steering  
- **DC Motor**: drives the vehicle forward  
- **Optical Encoder**: measures motor speed for closed-loop control  

## Lane Detection

The system detects blue tape marking the track using HSV color segmentation. A region of interest is applied to focus on relevant areas of the image. Detected lane pixels are used to fit second-order polynomials, and the center trajectory is computed from the lane boundaries. A look-ahead point is then used to estimate the steering angle.

## Control

Steering is controlled using a PID controller based on the difference between the desired heading (straight) and the estimated steering angle. Speed is regulated using a second PID loop, which uses feedback from an optical encoder to maintain a target RPM.

## Red Signal Detection

Red markers placed on the track are detected using color segmentation. The first detected marker causes the vehicle to stop briefly before continuing, while the second marker triggers a full stop.

## Object Detection

The system also integrates object detection using YOLOv5 running on the Raspberry Pi. The official Ultralytics repository was used, and images were resized to 320×320 for efficient inference. The system achieves approximately 3.8 FPS on test images, with performance varying during live streaming due to latency and hardware constraints.
