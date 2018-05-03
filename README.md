This repository is for ME470 Spring 2018 at University of Illinois at Urbana-Champaign.
The code is for the automatically delivery cart project sponsored by John Deere.

# Test Platform 
For ROS code, it is developed and tested on Ubuntu 14.04 with ROS Indigo and Opencv 2.4.13.
The controller code is developed for Arduino Mega 2560 board.

# Dependence
## ROS
Please follow this [link](http://wiki.ros.org/indigo/Installation/Ubuntu) for ROS Indigo installation 

## OpenCV
I built the OpenCV 2.4.13 from source with this [guide]( https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).  
```
sudo apt-get install libopencv-dev
```
should also work. If there is any issues with OpenCV, please refer to [CV_Bridge](http://wiki.ros.org/cv_bridge) package.

## ROS_SERIAL
To send ROS msg to Arduino, I used the [ROS_SERIAL](http://wiki.ros.org/rosserial) package. In short, you need to 
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```



