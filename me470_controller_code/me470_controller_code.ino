/*
rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub init_msg std_msgs/Bool '{data: 1}'
rostopic pub delivery_msg std_msgs/Bool '{data: 1}'
rostopic pub servo std_msgs/UInt16 30
*/
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <VarSpeedServo.h> 
//#include <Servo.h>
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

#define Step_1_DIR 9
#define Step_1_PUL 8
#define Linear_1_PIN 10
#define Switch_1_PIN 52



/*********************************
*******set up global Variable*****
**********************************/
ros::NodeHandle  nh;


VarSpeedServo servo_1;
int servor_PIN_list[N_module]={Linear_1_PIN};
VarSpeedServo* servo_list[N_module]={&servo_1};

AccelStepper stepper_1(1, Step_1_PUL,Step_1_DIR);
AccelStepper* stepper_list[N_module]={&stepper_1};

int switch_PIN_list[N_module]={Switch_1_PIN};


int pressSwitch = 0;
bool one_module_ready=0;
int wait_command=0;
int module_process=0;

int linearValue=0;
unsigned long time=0;
unsigned long time_count=0;

int delivery_switch=1;



/*********************************
*************ROS Handles**********
**********************************/

std_msgs::Bool init_finished;
bool init_start = 0;

std_msgs::Bool delivery_finished;
bool delivery_request=0;
bool delivery_start = 0;


void messageInit( const std_msgs::Bool& init_msg){
  if(init_start==0)
  {
    init_finished.data=0;
    init_start=init_msg.data;
  }
}

void messageDelivery( const std_msgs::Bool& delivery_msg){
  if( delivery_request==0)
  {        
    digitalWrite(LED_BUILTIN, HIGH);
    delivery_start=0;
    delivery_request=delivery_msg.data;
  }
}

void servo_cb( const std_msgs::UInt16& servo){
  if(delivery_request==1 && delivery_start==0){
    linearValue = map(servo.data, 0, 300, 0, 180);
    if(linearValue!=0)
    {
      delivery_finished.data=0;
      delivery_start=1;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  else if(delivery_request==0)
    linearValue = 0;
}


ros::Subscriber<std_msgs::Bool> sub1("init_msg", &messageInit);
ros::Subscriber<std_msgs::UInt16> sub2("servo", servo_cb);
ros::Subscriber<std_msgs::Bool> sub3("delivery_msg", &messageDelivery);


ros::Publisher init_done("init_done", &init_finished);
ros::Publisher delivery_done("delivery_done", &delivery_finished);

void setup()
{  
  // stepper motor setup
  stepper_1.setMaxSpeed(10000);//1100
  stepper_1.setAcceleration(200);
  stepper_1.setSpeed(1000);//should be more than 75 rpm=1.25 rps  

  //servo_1.attach(Linear_1_PIN,LINEAR_MIN,LINEAR_MAX);

  pinMode(LED_BUILTIN, OUTPUT);
  //ros init
  init_finished.data=0;
  delivery_finished.data=0;
  
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(init_done);
  nh.advertise(delivery_done);
}

void module_init(AccelStepper *stepper, VarSpeedServo *servo, int switch_PIN,int Liner_pin)
{
  //linear servo pin set up
  if(!(servo->attached()))
  {
    digitalWrite(LED_BUILTIN, HIGH);
    servo->attach(Liner_pin,LINEAR_MIN,LINEAR_MAX);
  }
  //return the servo to 0
  if(servo->read())
  {
    servo->write(0,0,0);
    time_count=time;
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

bool deliver_pushed=0;
bool deliver_retrived=0;

void module_delivery(AccelStepper *stepper, VarSpeedServo *servo)
{
  
  switch(delivery_switch)
  {
    case 1: //send linear servo out
      servo->write(linearValue,0,0);
      time_count=time;
      stepper->setSpeed(6000);
      delivery_switch=2;
      break;

    case 2: //wait linear servo reached its location, and drive pusher
      if((time-time_count)>30000)
      {
        stepper->moveTo(3000);
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
      }else if(deliver_pushed==1 & deliver_retrived ==0 & (time-time_count)>5000)
      {
        stepper->moveTo(0);
        stepper->setSpeed(-6000);
        servo->write(0,0,0);
        deliver_retrived=1;
        time_count=time;
      }else if(deliver_retrived==1 && (stepper->currentPosition())==(stepper->targetPosition()) && (time-time_count)>30000)
      {
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
  if(init_finished.data==0 && init_start==1)
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
      init_finished.data=1;
      module_process=0;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  init_done.publish(&init_finished);
  

  //delivery_stage
  if(init_finished.data==1 && delivery_request==1 && delivery_start==1 && delivery_finished.data==0)
  {
    if(module_process<N_module)
    {
      one_module_ready=0;
      module_delivery(stepper_list[module_process],servo_list[module_process]);
      if(one_module_ready==1)
      {                
        delivery_switch=1;
        module_process++;
      }
    }
    else{
      delivery_finished.data=1;
      delivery_request=0;
      delivery_start=0;
      module_process=0;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  delivery_done.publish(&delivery_finished);
  nh.spinOnce();

}







