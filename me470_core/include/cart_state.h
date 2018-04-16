#ifndef CART
#define CART
#include <iostream>


enum Status{
	DISCONNECTED=0,
	WAIT_FOR_INIT=1,
	LOADING=2,
	READY_FOR_DEL=3,
	DELIVERING=4,
	DELIVERED=5,
	FAILED=6
};

class cart_state
{
public:
	bool is_init();
	void set_init(bool input);

	bool is_deliver_start();
	bool is_delivered();
	
	void set_deliver_start(bool input);
	void set_delivered(bool input);

	bool is_cart_ready();
	void reset_delivery();

	void set_loading(bool input);
	int Get_current_status();

	void failure(bool input);
	bool is_failed();

	
private:
	bool init=false;
	bool deliver_start=false;
	bool delivered=false;
	bool ready_for_deliver=false;
	Status system_status=DISCONNECTED;
};

#endif