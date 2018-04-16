#ifndef IO_node
#define IO_node

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/String.h"
#include "../include/april_detector.h"
#include "../include/ImageConverter.h"
#include "../include/cart_state.h"

#include <sstream>




class node_io
{
public:
	ros::NodeHandle nh_;
	node_io();
	void init_status_sub_fun(const std_msgs::Bool& msg);  

	


private:

 	ros::Subscriber init_status_sub;

	april_detector detector;	
	cart_state cart;

	cv::Mat gray;

};
#endif