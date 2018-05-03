This repository is for ME470 Spring 2018 at University of Illinois at Urbana-Champaign.
The code is for the automatically delivery cart project sponsored by John Deere.

# Test Platform 
For ROS code, it is developed and tested on Ubuntu 14.04 with ROS Indigo and Opencv 2.4.13.
The controller code is developed for Arduino Mega 2560 board.

# ROS Code Dependence
## ROS
Please follow this [link](http://wiki.ros.org/indigo/Installation/Ubuntu) for ROS Indigo installation 

## OpenCV
I built the OpenCV 2.4.13 from source with this [guide]( https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).  
`
sudo apt-get install libopencv-dev
`
should also work. If there is any issues with OpenCV, please refer to [CV_Bridge](http://wiki.ros.org/cv_bridge) package.

## ROS_SERIAL
To send ROS msg to Arduino, I used the [ROS_SERIAL](http://wiki.ros.org/rosserial) package. In short, you need to 
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

## USB_CAM
A ROS Driver for V4L USB Cameras. Forked from this [repository](https://github.com/ros-drivers/usb_cam). Detailed instruction please see this [doc](http://wiki.ros.org/usb_cam).

# Arduino Code Dependence
## VarSpeedServo
Please refer to this [repository](https://github.com/netlabtoolkit/VarSpeedServo)

## AccelStepper
Please refer to this [document](http://www.airspayce.com/mikem/arduino/AccelStepper/)

## ROS
Please refer to this [tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials)

# Installation
1. Place the ROS code in the `src` folder of your catkin workspace
2. In your catkin workspace
```
catkin_make
source devel/setup.bash
```
3. Send the Arduino code to your arduino
4. 
```
roslaunch usb_cam me470
```
Edit the launch file in the USB_CAM package according to your device.




