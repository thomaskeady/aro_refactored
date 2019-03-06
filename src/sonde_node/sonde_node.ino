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

void teensyStateCb (const std_msgs::Int32& sig)
{
  
}



// Publishers
ros::Publisher teensy_signal_pub("teensy_signal", &sig);
ros::Publisher teensy_data_pub("teensy_data", &data_msg);

// Subscribers
ros::Subscriber<std_msgs::Int32> teensy_state_sub("teensy_state", &teensyStateCb);



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

  msg_count = 0;

  Wire2.begin(); // Depth sensor, I2C 2
  bar30.init();
  bar30.setFluidDensity(997);

  Serial2.begin(38400); // Sonde

  Serial.begin(9600); // Arduino serial monitor
  
}




void loop() {
  // put your main code here, to run repeatedly:

}









