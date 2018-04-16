#include "../include/node_io.h"

node_io::node_io()
{

init_status_sub = nh_.subscribe("/cart_msg/init_done",1,init_status_sub_fun,this);

}

void node_io::init_status_sub_fun(const std_msgs::Bool& msg)
{
	if(~cart.is_init())
	{
		cart.set_init(msg.data);
		std::cout<<"sub test"<<cart.is_init()<<std::endl;
	}
}