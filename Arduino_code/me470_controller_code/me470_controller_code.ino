/*
rosrun rosserial_python serial_node.py /dev/ttyUSB0

rostopic pub /cart_bot_msg/waypoint_reached std_msgs/Bool '{data: 1}'

*/
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <VarSpeedServo.h> 
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

#define N_module 1 // number of mudule in the system

#define LINEAR_MIN      1040  //
#define LINEAR_MAX      2000 //

/*********************************
*********set up pin***************
**********************************/

#define Step_1_DIR 6 //vga 5                                    c
#define Step_1_PUL 7 //vga 12
#define Linear_1_PIN 5 //vga 10
#define Switch_1_PIN 52 //vga 13
#define Loading_1_PIN 50 //vga 14

#define gate_R_PIN 10//vga 2
#define gate_L_PIN 11//vga 3



/*********************************
*******set up global Variable*****
**********************************/


VarSpeedServo servo_1;
int servor_PIN_list[N_module]={Linear_1_PIN};
VarSpeedServo* servo_list[N_module]={&servo_1};


VarSpeedServo servo_R_1;
VarSpeedServo servo_L_1;

VarSpeedServo* Gate_R_list[N_module]={&servo_R_1};
VarSpeedServo* Gate_L_list[N_module]={&servo_L_1};

AccelStepper stepper_1(1, Step_1_PUL,Step_1_DIR);
AccelStepper* stepper_list[N_module]={&stepper_1};

int switch_PIN_list[N_module]={Switch_1_PIN};
int loading_PIN_list[N_module]={Loading_1_PIN};
//int last_loading_read[N_module];
int current_loading_module=-1;
bool loading_retrive=0;

int module_process=0;
int linearValue=0;
unsigned long time=0;
unsigned long time_count=0;

int time_to_wait=25000;


enum Status{
  DISCONNECTED=0,
  WAIT_FOR_INIT=1,
  LOADING=2,
  READY_FOR_DEL=3,
  DELIVERING=4,
  DELIVERED=5,
  FAILED=6
};

Status system_status=DISCONNECTED;



/*********************************
*************ROS Handles**********
**********************************/
ros::NodeHandle  nh;
std_msgs::Bool delivery_finished;
std_msgs::UInt16 status_msg;


/*********************************
**********ROS Subscriber**********
**********************************/

void messageInit( const std_msgs::Bool& init_msg){

  if(system_status==DISCONNECTED && init_msg.data)
  {
    system_status=WAIT_FOR_INIT;
  }
}

void servo_cb(const std_msgs::UInt16& servo){
  if(system_status==READY_FOR_DEL){
    linearValue = map(servo.data, 0, 300, 0, 180);
    if(linearValue>0)
    {
      delivery_finished.data=0;
      digitalWrite(LED_BUILTIN, LOW);
      system_status=DELIVERING;
      double temp=((double)linearValue)/180.0*25000.0;
      if(temp>25000)
        temp=25000;
      else if(temp<4000)
        temp=4000;

      time_to_wait=(int) (temp+2000.0);
    }
    else
    {
      linearValue=0;
    }
  }
}


ros::Subscriber<std_msgs::Bool> sub1("/cart_msg/init_request", &messageInit);
ros::Subscriber<std_msgs::UInt16> sub2("/cart_msg/deliver_request", servo_cb);


ros::Publisher delivery_done("/cart_msg/delivery_done", &delivery_finished);
ros::Publisher status_pub("/cart_msg/cart_status", &status_msg);


void setup()
{  
  // stepper motor setup
  stepper_1.setMaxSpeed(10000);//1100
  //stepper_1.setAcceleration(200);
  stepper_1.setSpeed(-2000);//should be more than 75 rpm=1.25 rps  

  pinMode(LED_BUILTIN, OUTPUT);

  servo_R_1.attach(gate_R_PIN);
  servo_R_1.write(170,0,false);//170 close gate 78 open gate 

  servo_L_1.attach(gate_L_PIN);
  servo_L_1.write(46,0,false);//46 close gate 135 open gate

  //ros init
  
  delivery_finished.data=0;
  status_msg.data=0;
  
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  nh.advertise(delivery_done);
  nh.advertise(status_pub);
}

// initialization stage

int pressSwitch = 0;
bool one_module_ready=0;

void module_init(AccelStepper *stepper, VarSpeedServo *servo, int switch_PIN,int Linear_pin)
{
  //linear servo pin set up
  if(!(servo->attached()))
  {
    servo->attach(Linear_pin,LINEAR_MIN,LINEAR_MAX);
  }
  //return the servo to 0
  if(servo->read())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    servo->write(0,0,0);
    time_count=time;
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }else if((time-time_count)>25000){
    pressSwitch = digitalRead(switch_PIN);
    if(pressSwitch == HIGH)
    { 
      one_module_ready=1;
      stepper->setCurrentPosition(0);
      delay(1000);
    }
    else{
      stepper->runSpeed();
    }
  }
}

// delivery stage state machine

bool deliver_pushed=0;
bool deliver_retrived=0;
int delivery_switch=1;


void module_delivery(AccelStepper *stepper, VarSpeedServo *servo, VarSpeedServo * Gate_L, VarSpeedServo* Gate_R)
{
  
  switch(delivery_switch)
  {
    case 1: //send linear servo out
      servo_R_1.detach();
      servo_L_1.detach();
      servo->write(linearValue,0,0);
      time_count=time;
      stepper->setSpeed(6000);
      delivery_switch=2;
      break;

    case 2: //wait linear servo reached its location, and drive pusher
      if((time-time_count)>time_to_wait)
      {
        servo_R_1.attach(gate_R_PIN);
        servo_L_1.attach(gate_L_PIN);

        Gate_L->write(135,0,0);
        Gate_R->write(78,0,0);
        stepper->moveTo(8600);
        delivery_switch=3;
        deliver_pushed=0;
        deliver_retrived=0;
      }
      break;

    case 3: // retrive pusher and linear servo
      if((stepper->currentPosition())==(stepper->targetPosition()) && deliver_pushed==0)
      {
        deliver_pushed=1;
        time_count=time;
      }else if(deliver_pushed==1 & deliver_retrived ==0 & (time-time_count)>3000)
      {
        stepper->moveTo(100);
        
        stepper->setSpeed(-6000);
        servo->write(0,0,0);
        deliver_retrived=1;
        time_count=time;
      }else if(deliver_retrived==1 && (stepper->currentPosition())==(stepper->targetPosition()) && (time-time_count)>time_to_wait)
      {
        Gate_R->write(170,0,false);
        Gate_L->write(46,0,false);
        one_module_ready=1;
      }else
      {
        stepper->run();
      }
      break;

    default:
      return;
  }  
}

void loop()
{
  time=millis(); 

  

  
  //initialization state
  if(system_status==WAIT_FOR_INIT)
  {
    if(module_process<N_module)
    {
      one_module_ready=0;
      module_init(stepper_list[module_process],servo_list[module_process],switch_PIN_list[module_process],servor_PIN_list[module_process]);
      if(one_module_ready==1)
      {
        module_process++;
      }
    }
    else{
      system_status=READY_FOR_DEL;
      module_process=0;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  //delivery_stage
  if(system_status==DELIVERING)
  {
    if(module_process<N_module)
    {
      one_module_ready=0;
      module_delivery(stepper_list[module_process],servo_list[module_process],Gate_L_list[module_process],Gate_R_list[module_process]);
      if(one_module_ready==1)
      {                
        delivery_switch=1;
        module_process++;
      }
    }
    else{
      delivery_finished.data=1;
      system_status=DELIVERED;
      time_to_wait=25000;
      time_count=time;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  //Send Delivery message for 5 seconds
  if(system_status==DELIVERED && (time-time_count)<3000)
  {
    delivery_done.publish(&delivery_finished);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if(system_status==DELIVERED)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    module_process=0;
    system_status=READY_FOR_DEL;
    delivery_finished.data=0;
    
  }

  if(system_status==READY_FOR_DEL)
  {
    for(int i=0; i<N_module; i++)
    {
     int current_read=digitalRead(loading_PIN_list[i]);
     if(current_read==1) //turn on loading
     {
      Gate_L_list[i]->write(135,0,0);
      Gate_R_list[i]->write(78,0,0);
      system_status=LOADING;
      (servo_list[i])->write(300,0,0);
      current_loading_module=i;
      break;
     } 
    }
  }

  if(system_status==LOADING)
  {
    int current_read=digitalRead(loading_PIN_list[current_loading_module]);
    if((!loading_retrive) && current_read==0)
    {
      loading_retrive=1;
      (servo_list[current_loading_module])->write(0,0,0);
      Gate_L_list[current_loading_module]->write(46,0,0);
      Gate_R_list[current_loading_module]->write(170,0,0);
      time_count=time;
    }else if(loading_retrive && (time-time_count)>25000)
    {
      loading_retrive=0;
      system_status=READY_FOR_DEL;
      current_loading_module=-1;
    }
  }

  //publish status state of the cart
  status_msg.data=system_status;
  status_pub.publish(&status_msg);

  nh.spinOnce();
}







