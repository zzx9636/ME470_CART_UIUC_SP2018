#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "../include/ImageConverter.h"
#include "../include/april_detector.h"
#include "../include/node_io.h"

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>


using namespace std;
/* 
Global Variable
*/
enum Status{
	DISCONNECTED=0,
	WAIT_FOR_INIT=1,
	LOADING=2,
	READY_FOR_DEL=3,
	DELIVERING=4,
	DELIVERED=5,
	FAILED=6
};


Status System_status=DISCONNECTED;
ImageConverter * ic_ptr;
//april_detector Detector;
cart_state Cart;

/*
Following are publisher
*/
ros::Publisher status_pub;
ros::Publisher linear_servo_pub;
ros::Publisher error_pub;
ros::Publisher deliverd_pub;

/*
Following are subscriber function
*/

// subscriber for /cart_msg/init_done to determine if initializaion is finished

void status_publisher(const sensor_msgs::ImageConstPtr& msg) //if camera works
{
	std_msgs::UInt16 status2pub;
	status2pub.data=System_status;
	status_pub.publish(status2pub);  	
}

void init_status_sub(const std_msgs::Bool& msg)
{
	System_status=WAIT_FOR_INIT;
	if(~Cart.is_init())
	{
		Cart.set_init(msg.data);
		if(Cart.is_init())//if receve init done msg from cart
			System_status=READY_FOR_DEL;
	}
}

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
	if(Cart.is_init())
	{
		switch(msg.data)
		{
			case 2: //robot is loading
				Cart.set_loading(true);
				System_status=LOADING;
				break;

			case 3: //ready for delivery after loading
				if(!Cart.is_cart_ready())
					Cart.reset_deliverd();
				System_status=READY_FOR_DEL;
				break;

			case 4: //robot received the deliver message
				Cart.set_deliver_start(true);
				System_status=DELIVERING;
				break;

			case 5: //robot finished the delivery
				Cart.set_delivered(true);
				System_status=DELIVERED;
				std_msgs::Bool deliverd_msg;
				deliverd_msg.data=true;
				deliverd_pub.publish(deliverd_msg);
				break;

			case 6: //deliver failure
				System_status=FAILED;
				std_msgs::Bool error_msg;
				error_msg.data=true;
				error_pub.publish(error_msg);
				break;

			default:
				System_status=READY_FOR_DEL;
				break;
		}
	}
}

void waypoint_reached_sub(const std_msgs::Bool& msg)
{
	if(Cart.is_cart_ready())//make sure that the cart is initialized
	{
		if(msg.data)//if the waypoint has been arrived
		{
			float linear_distance = 100.0*(ic_ptr->Detector).detection_distance(ic_ptr->gray,0);
			if(linear_distance!=-1){//target tag detected
				std_msgs::UInt16 linear2pub;
				linear2pub.data=(int) linear_distance;
				linear_servo_pub.publish(linear2pub);
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

  status_pub = n.advertise<std_msgs::UInt16>("/cart_bot_msg/status_pub", 1);
  linear_servo_pub = n.advertise<std_msgs::UInt16>("/cart_msg/deliver_request", 1);
  error_pub = n.advertise<std_msgs::Bool>("/cart_bot_msg/error_msg",1);
  deliverd_pub = n.advertise<std_msgs::Bool>("/cart_bot_msg/deliverd_success",1)

  ros::Subscriber sub0 = n.subscribe("/usb_cam/image_raw",1,status_publisher);
  ros::Subscriber sub1 = n.subscribe("/cart_msg/init_done",1,init_status_sub);
  ros::Subscriber sub2 = n.subscribe("/cart_bot_msg/waypoint_reached",1,waypoint_reached_sub);

  

  ros::spin();
  return 0;
}