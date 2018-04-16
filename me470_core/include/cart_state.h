#ifndef CART
#define CART
#include <iostream>

class cart_state
{
public:
	bool is_init();
	bool is_cart_ready();
	void set_init(bool input);
	bool is_deliver_start();
	bool is_delivered();
	void set_deliver_start(bool input);
	void set_delivered(bool input);
	void reset_deliverd();
	void set_loading(bool input);
	
private:
	bool init=false;
	bool deliver_start=false;
	bool delivered=false;
	bool ready_for_deliver=false;
};

#endif