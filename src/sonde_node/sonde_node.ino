#define USE_TEENSY_HW_SERIAL

#define MAX_NUM_MSGS 500

#include <ros.h>
#include <ros/time.h>
//#include <aro_ft/aro_ft.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "bar30.h"
#include <Wire.h>

ros::NodeHandle nh;

// Messages
std_msgs::Int32 sig;
//aro_refactored:: data_msg; // Need to add custom messages here TODO 
std_msgs::Empty data_msg;


int STATE;

void teensyStateCb (const std_msgs::Int32& newState)
{

  Serial.print("STATE was ");
  Serial.print(STATE);
  Serial.print("\t now is ");
  Serial.println(newState.data);
  STATE = newState.data;
  // Is that it?
}



// Publishers
ros::Publisher teensy_signal_pub("teensy_signal", &sig);
ros::Publisher teensy_data_pub("teensy_data", &data_msg);

// Subscribers
ros::Subscriber<std_msgs::Int32> teensy_state_sub("teensy_state", &teensyStateCb);



//int STATE; // So that its nonvolatile? Are callbacks interrupts? - they arent, entered when spin is called
int msg_count;

MS5837 bar30;

void setup() {
  
  nh.getHardware()->configureHardware(&Serial3, 9600);  //Using 9600 baud rate
  nh.initNode();

  // Publishers
  nh.advertise(teensy_signal_pub);
  nh.advertise(teensy_data_pub);

  // Subscribers
  nh.subscribe(teensy_state_sub);

  // Might want to make teensy signals more detailed
  sig.data = 0;


  msg_count = 0;
  STATE = 0;

  Wire2.begin(); // Depth sensor, I2C 2
  bar30.init();
  bar30.setFluidDensity(997);

  Serial2.begin(38400); // Sonde

  Serial.begin(9600); // Arduino serial monitor
  
}




void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce(); // This is where STATE will be updated

  switch (STATE)
  {
    case 0 : 
      // Idle state, put a non-busy wait here
      Serial.println("Chilling (state 0)");
      delay(1000);
      
      break;
    case 1 : 
      // Sending heartbeats for testing comms
      Serial.println("Sending hb");
      teensy_signal_pub.publish(&sig); // Change sig to something first? Value to indicate heartbeat? May help with debugging
      delay(500);

      break;
    case 2 : 
      // Start collecting data! Repeat until another signal arrives 
      Serial.println("Pretending to collect data brb");
      delay(250);
      
      break;
    case 3 : 

      break;
    case 4 : 

      break;
      
  }



}









