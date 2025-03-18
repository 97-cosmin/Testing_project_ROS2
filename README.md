# Testing_project_ROS2
1.Introduction
	This document aims to describe the testing plan for the software components responsible for managing and validating video streams from the Intel RealSense D455 camera. 
Since I do not have access to the physical device, I searched for a way to simulate the camera's real behavior and decided to use mocks. 

Why I decided to use mocks? Because firstly
I searched for:
	1.	A .bat file for ROS1:
	I initially searched for a .bat file, but I found that the files I came across were 	designed for ROS1. Since I was already advanced with setting up the environment 	for ROS2, this wasn’t a suitable solution for me.
    • Reaching out to friends:
	2.I also tried asking around among my friends to see if anyone had a camera similar to the Intel RealSense D455, but unfortunately, I couldn’t find anyone with access to the hardware I needed.
	3. Attempting to use Gazebo for simulation:
	I explored using Gazebo for simulating the camera, but I ran into a dead end because there wasn’t a plugin available for the Intel RealSense D455, and I couldn’t find anything useful on the internet to help me.

Using Mocks for Simulation:
	In the end, I decided to create mock objects to simulate the behavior of the Intel RealSense D455 camera. 
	Mocks are simulated objects or components used in testing to mimic the behavior of real objects. In this case, I created mock objects to replicate the behavior of the Intel RealSense D455 camera. 

Since the physical hardware is unavailable for testing, these mocks simulate the camera's responses, such as providing video streams (RGB, Infrared, and Depth). 
This approach allows me to test the software components, like the Camera Manager, under various conditions. 
By using mocks, I can ensure that the software behaves correctly in a controlled environment without needing the actual camera hardware.

 2.Scope

The tests will verify the proper functioning of the camera and the behavior of the Camera Manager under normal and edge conditions, using RGB, Infrared, and Depth streams.
Mock objects will simulate the camera’s behavior, allowing testing of the software components in various conditions.

    • Test Environment
    • ✅ Operating System:Ubuntu 22.04.5 LTS (Codename: Jammy) 
    • ✅ ROS Version:ROS 2 Humble 
    • ✅ Camera Model: Intel RealSense D400 Series (specifically, RealSense D455 model) 
    • ✅ Software Tools:
        ◦ librealsense: Version 2.55.1-0~realsense, a library for controlling and capturing data from Intel RealSense devices. 
            ▪ librealsense2-dkms 
            ▪ librealsense2-udev-rules 
        ◦ pytest: Version 8.3.5, a testing framework for running and managing Python tests 
        ◦ Python: Version 3.10.12 
        ◦ mock-camera-pkg: A ROS 2 package for mock camera data 
    • ✅ Kernel Version:
    • Linux Kernel 6.8.0-52-generic
**
Implementation of Automated Tests**

I chose to create a Git repository: https://github.com/97-cosmin/Testing_project_ROS2/tree/master
to demonstrate the implementation of automated tests and meet the required test examples.

(I won't add too much information; I have commented on the code for each test to explain what it does.)

Core Functional Files (mock_camera_pkg/):
https://github.com/97-cosmin/Testing_project_ROS2/tree/master/mock_camera_pkg

        ◦ Contains the main scripts for simulating camera streaming, input validation, and error handling.
        ◦  These are the files responsible for the core functionality of your package, which publishes and subscribes to different types of messages. 
            ▪ camera_stream_publisher.py: Contains the logic for publishing camera stream data. This file handles the publishing aspect of the camera stream. 
    • camera_stream_subscriber.py: Contains the logic for subscribing to camera stream data. This file listens to the stream and processes the received data.
    • publisher_for_errors.py: This file handles the publishing of error messages, likely related to system or data errors, during communication or processing. 
    • subscriber_for_errors.py: The corresponding subscriber for publisher_for_errors.py, this file processes incoming error messages.	
    • camera_config.yaml: Tech specs for D455 camera
    
 Test Files (test/):
https://github.com/97-cosmin/Testing_project_ROS2/tree/master/test

        ◦ Unit Tests: Validate individual components of the system. 
        ◦ Integration Tests: Test the interaction between different components and ensure that the whole system works together. 
        ◦ Reports: Generated HTML reports summarizing the results of the tests, including functional, integration, error handling, and performance tests. 

Configuration and Metadata Files:
https://github.com/97-cosmin/Testing_project_ROS2/tree/master

        ◦ setup.py: Defines the setup process for the package. 
        ◦ package.xml: ROS-specific metadata. 
        ◦ mock_camera_pkg.egg-info: Contains metadata used during packaging and distribution.
