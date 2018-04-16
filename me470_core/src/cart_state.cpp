#include "../include/cart_state.h"
using namespace std;
bool cart_state::is_init()
{
	return this->init;
}

bool cart_state::is_cart_ready()
{
	return this->ready_for_deliver;
}

void cart_state::set_init(bool input)
{
	this->init=input;
	if(input)
	{
		cout<<"Cart Initialization Done"<<endl;
		system_status=READY_FOR_DEL;
		this->reset_deliverd();
	}else
		system_status=WAIT_FOR_INIT;
}

bool cart_state::is_deliver_start()
{
	return this->deliver_start;
}

bool cart_state::is_delivered()
{
	return this->delivered;
}

void cart_state::set_deliver_start(bool input)
{
	if(input){
		this->deliver_start=true;
		this->delivered=false;
		this->ready_for_deliver=false;
		system_status=DELIVERING；
	}
}

void cart_state::set_delivered(bool input)
{
	this->delivered=input;
	system_status=DELIVERED;
}

void cart_state::reset_delivery()
{
	this->deliver_start=false;
	this->delivered=false;
	this->ready_for_deliver=true;
	system_status=READY_FOR_DEL;
}

void cart_state::set_loading(bool input)
{
	if(input){
		this->ready_for_deliver=false;
		system_status=LOADING;
	}

}

void cart_state::failure(bool input)
{
	if(input)
	{
		this->ready_for_deliver=false;
		system_status=FAILED;
	}
}

bool cart_state::is_failed()
{
	if(system_status==FAILED)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int cart_state::Get_current_status()
{
	return system_status;
}