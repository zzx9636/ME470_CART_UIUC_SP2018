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
		this->reset_deliverd();
	}
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
		this->ready_for_deliver=false;
	}
}

void cart_state::set_delivered(bool input)
{
	this->delivered=input;
}

void cart_state::reset_deliverd()
{
	this->deliver_start=false;
	this->delivered=false;
	this->ready_for_deliver=true;
}

void cart_state::set_loading(bool input)
{
	if(input)
		this->ready_for_deliver=false;
}