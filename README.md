# PincherX_PickPlace_Package

PincherX_PickPlace_Package is a pipeline designed to detect objects, pick them up, and place them in specified positions using a 4DOF Pincher X Manipulator arm. This package is entirely desigend and coded from scratch without using any kinematics / motion planning / control libraries or frameworks.

<img width="1370" height="796" alt="image" src="https://github.com/user-attachments/assets/23afbeb9-16ff-4a44-9d2f-416c9a739b3a" />

### State Flow
<img width="1375" height="869" alt="image" src="https://github.com/user-attachments/assets/a9b57628-c91b-47bd-be97-0dfdf62ed16f" />


This project is part of Robotics Lab Spring 2025.

## Usage

1. Open PickPlacePipeline folder
### Simple Pick Place Execution
1. Open PickPlace.mlx, input the pick, place points and hit run

### Perception based object identification (Requires Intel depth camera)
1. Open PickPlacePerception --> Input place location and run!

## Features

- Object detection and pose estimation
- Robot kinematics (forward and inverse)
- Rate control for smooth motion execution
- Collision avoidance and path planning (in progress)
- Language Grounding (TBA)

## Requirements

- Matlab
- Arbotix-M Software
- [Peter Corke&#39;s Interfacing Package](https://petercorke.com/matlab/interfacing-a-hobby-robot-arm-to-matlab/)

## Progress

- [X] Object Detection and Pose Estimation
- [X] Robot configuration
- [X] Forward Kinematics
- [X] Trajectory Following and Rate Control
- [X] Inverse Kinematics
- [ ] Collision Avoidance
- [X] Path Planning
- [X] PickPlace pipeline
- [X] Bin Place

## Tests

- [Rate Control line following Test 01]()
- [Pick Place Test 01](https://youtu.be/EYrFdB0laEY?feature=shared) 
- [Pick Place Test 02](https://youtu.be/jQurTOjDS5M?si=o-e3RmH0QStOSSTE)
- [Pipeline Execution](https://youtube.com/shorts/RyvSCiVWmow?si=RFqXScAhPhXi5GSY)
