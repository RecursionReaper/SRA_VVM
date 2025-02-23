# SRA_VVM
# Voice Video Manipulator
Design and implementing ROS2 and Gazebo for manipulating through voice and video command!

## Table of Contents

- [Voice Video Manipulator](#voice-video-manipulator)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
  - [Project Workflow](#project-workflow)
  - [Features](#features)
  - [Software Used](#software-used)
  - [Future Work](#future-work)
  - [Contributors](#contributors)
  - [Resources](#resources)
  - [Acknowledgements](#acknowledgements)


## About The Project
The project aims to develop a robotic manipulator system equipped with both video input capabilities and speech recognition for accepting spoken commands. This system will enable users to control the robotic manipulator intuitively through verbal instructions and visual feedback.

## Project Workflow  
#### Research

 1. ROS2 basics like creating a node, topic, publisher and subscriber
 2. Gazebo and spawing objects
 3. Neural Networks and Convolution Neural Networks for Object detection
 4. OpenCV 

#### Design and Development
We have taken OPEN-MANIPULATOR X as it was open source manipulator for this project. All the models and design are open source and take form ROBOTIS's OpenManipulatorX project

#### Testing and Optimization
This Project is still in testing phase. We have created basic codes for different functions now just we need to compile them all so they work together.

#### Deployment and Documentation
User need ROS2 Humble version as all these codes were written on this distribution.

## Features
We will be designing and developing the Intelligent Video Manipulator with Voice Commands, an innovative system that seamlessly integrates video processing and speech recognition technologies. Our goal is to enable users to interact with and manipulate video content using spoken commands. To achieve this, we will create a robust video input system capable of capturing and processing real-time video from various sources. We will implement an advanced speech recognition module to ensure accurate command interpretation and enable the system to perform specific video manipulation tasks based on these commands. 

## Software Used 

 - ROS2 Humble
 - Gazebo
 - OpenCV

## Future Work

 - Make grippers capable of lifting and holding items throughout the trajectory
 - Decrease the errors in distance given by camera
 - Add and implement audio detection

## Contributors

 -  [Kartikey Pathak](https://github.com/NoobMaster-version)
- [Aniket Desai](https://github.com/RecursionReaper) 
- [Yash Suthar](https://github.com/BlazinBull)

## Resources
• Nodes
• Discovery
• Interfaces
• Topics
• Services
• Actions
• Parameters
• Introspection with command line tools
• Launch
• Client libraries
All this can be found on official ROS2 Humble Website


## Acknowledgements 

 - SRA VJTI Eklavya 2024
 - Special thanks to out mentors Atharv Dubey , Abhinav Ananthu , Sarvesh Patki


