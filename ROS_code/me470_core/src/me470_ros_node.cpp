#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "../include/ImageConverter.h"
#include "../include/april_detector.h"
#include "../include/cart_state.h"

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>


using namespace std;

/* 
Global Variable
*/
ImageConverter * ic_ptr;
cart_state Cart;

/*
Following are publisher
*/
ros::Publisher linear_servo_pub;
ros::Publisher init_request_pub;


std_msgs::Bool msg_init_pub;


/*
Following are subscriber function
*/

void deliver_status_sub(const std_msgs::UInt16 & msg)
{
	/*
	DISCONNECTED=0,
	WAIT_FOR_INIT=1,
	LOADING=2,
	READY_FOR_DEL=3,
	DELIVERING=4,
	DELIVERED=5,
	FAILED=6
	*/
	switch(msg.data)
	{
		case 0: // fresh start
			Cart.set_init(false);
			msg_init_pub.data=1;
			init_request_pub.publish(msg_init_pub);
			break;

		case 1: // robot has not been initalized
			msg_init_pub.data=1;
			init_request_pub.publish(msg_init_pub);
			break;

		case 2: //robot is loading	
			Cart.set_loading(true);
			break;

		case 3: //ready for delivery
			if(!Cart.is_init())
				Cart.set_init(true);
			else if(!Cart.is_cart_ready())
				Cart.reset_delivery();
			break;

		case 4: //robot received the deliver message
			if(!Cart.is_deliver_start()){
				Cart.set_deliver_start(true);
			}
			break;

		case 5: //robot finished the delivery
			if(!Cart.is_delivered()){
				Cart.set_delivered(true);
			}	
			break;

		case 6: //deliver failure
			if(!Cart.is_failed()){
				Cart.failure(true);
			}
			break;

		default:
			break;
	}
	
}

void waypoint_reached_sub(const std_msgs::Bool& msg)
{
	if(Cart.is_cart_ready())//make sure that the cart is initialized
	{
		if(msg.data)//if the waypoint has been arrived
		{
			//float linear_distance = (ic_ptr->Detector).detection_distance(ic_ptr->gray,1);
			float linear_distance = 10.0*(ic_ptr->current_dis-48.0);
			if(linear_distance>=0){//target tag detected
				std_msgs::UInt16 linear2pub;
				if(linear_distance>300)
					linear2pub.data=300;
				else
					linear2pub.data=(int)linear_distance;
				linear_servo_pub.publish(linear2pub);
				std::cout<<"Detected "<<linear_distance<<" mm. Extend the linear servo "<<linear2pub.data<<" mm"<<std::endl;
		  	}
		  	else
		  	{
		  		std::cout<<"No target tag detected"<<std::endl;
		  	}
		}
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ME470_cart");
  ros::NodeHandle n;
  ImageConverter ic;
  ic_ptr=&ic;

  
  linear_servo_pub = n.advertise<std_msgs::UInt16>("/cart_msg/deliver_request", 1);
  init_request_pub = n.advertise<std_msgs::Bool>("/cart_msg/init_request",1);

  ros::Subscriber sub1 = n.subscribe("/cart_msg/cart_status",1,deliver_status_sub);
  ros::Subscriber sub2 = n.subscribe("/cart_bot_msg/waypoint_reached",1,waypoint_reached_sub);

  ros::spin();
  return 0;
}