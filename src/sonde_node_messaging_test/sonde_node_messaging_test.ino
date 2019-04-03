#define USE_TEENSY_HW_SERIAL

#include <ros.h>
#include <ros/time.h>
//#include <aro_ft/aro_ft.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "bar30.h"
#include <Wire.h>

#include <aro_refactored/sample.h>

ros::NodeHandle nh;

// Messages
std_msgs::Int32 sig; 
//std_msgs::Empty data_msg;
aro_refactored::sample data_msg;

//aro_refactored:: data_msg; // Need to add custom messages here TODO
//aro_ft::aro_ft buffMsg;
//aro_ft::aro_ft msgArray[MAX_NUM_MSGS];

// Message sizing parameters
const int MAX_MSG_SIZE = 50; // Max # of messages that get sent at once, will make this bigger (currently for testing)
const int MAX_NUM_MSGS = 500; // Just so we can preallocate, can do this more smartly if necessary 

aro_refactored::sample samples[MAX_NUM_MSGS];
int currMsgSize = 0;
int numMsgs = 0;

aro_refactored::sample test;


ros::Time stampBuffer[MAX_MSG_SIZE];
float dissolvedOxygenBuffer[MAX_MSG_SIZE];
float waterTempBuffer[MAX_MSG_SIZE];
float depthBuffer[MAX_MSG_SIZE];


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

  delay(500);
  
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


  Serial.println("Before");
  //test.stamp = stampBuffer; // THIS WORKS
  //test.stamp = samples[0].stamp; // SO DOES THIS
  Serial.println("After");
}


String rawString;
ros::Time currTime;
double currTimeDouble;
float temp;
float DO;
float depth;
String tempString;
String doString;


void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce(); // This is where STATE will be updated

  //myFile.println("test");
  //myFile.flush();

          // Start collecting data! Repeat until another signal arrives 
          //Serial.println("(state 2) Pretending to collect data brb");
          //delay(250);
          
          // Tell Nuc that got signal
          //teensy_signal_pub.publish(&sig); // Would have to distinguish between hb and ack, may not be necessary
          
          Serial2.write("?01,PVAL,,\r\n");
          //nh.spinOnce();
          rawString = Serial2.readString();
          //rawString format: "=01,PVAL,24.173,271.62,"
          //nh.spinOnce(); 
          Serial.println(rawString);
      
          // For testing without Sonde:
          //rawString = "=01,PVAL,24.173,271.62,";
          //delay(250); // Change this number to simulate smapling delay
      
          bar30.read();
          tempString = rawString.substring(9,15);
          doString = rawString.substring(16,22);
          temp = tempString.toFloat();
          DO = doString.toFloat();
          depth = bar30.depth();
          
          //samples[numMsgs].stamp[currMsgSize++];
          //samples[numMsgs].stamp.push_back(nh.now());   // Not a std container *eye roll*
      
//          stampBuffer[currMsgSize] = nh.now();
//          dissolvedOxygenBuffer[currMsgSize] = DO;
//          waterTempBuffer[currMsgSize] = temp;
//          depthBuffer[currMsgSize] = depth;

          currTime = nh.now();
          currTimeDouble = nh.now().toSec();
          Serial.println(currTimeDouble);
          /*Serial.println("Before");
          //samples[numMsgs].stamp[currMsgSize] = currTime;
          Serial.println("Before");
          samples[numMsgs].dissolvedOxygen[currMsgSize]  = DO;
          Serial.println("Before");
          samples[numMsgs].waterTemp[currMsgSize]  = temp;
          Serial.println("Before");
          samples[numMsgs].depth[currMsgSize]  = depth;
          Serial.println("After");*/

          Serial.println("Before");
          //test.stamp[0] = currTime;
          Serial.println("Before");
          //test.dissolvedOxygen[0] = DO;
          Serial.println("Before");
          test.waterTemp[0] = temp;
          Serial.println("Before");
          test.depth[0] = depth;
          
          // Added to see what these data are
          //Serial.println(stampBuffer[currMsgSize]); Serial.println(dissolvedOxygenBuffer[currMsgSize]); Serial.println(waterTempBuffer[currMsgSize]); Serial.println(depthBuffer[currMsgSize]);    
          //Serial.println(nh.now());
          //Serial.print(DO);Serial.print("\t");Serial.print(temp);Serial.print("\t");Serial.println(depth);
          // End printing          

          //myFile.println((String)currTimeDouble + "\t" + (String)DO + "\t" + (String)temp + "\t" + (String)depth);
          //myFile.flush();
          
          ++currMsgSize;
    
          if (currMsgSize == MAX_MSG_SIZE) 
          {
            Serial.println("Copying to message array, creating new message");
            
            currMsgSize = 0;
            
            // Now add them to the array of sample messages for the Nuc
            //samples[numMsgs].stamp = stampBuffer;
            //samples[numMsgs].dissolvedOxygen = dissolvedOxygenBuffer;
            //samples[numMsgs].waterTemp = waterTempBuffer;
            //samples[numMsgs].depth = depthBuffer;
            // TODO will this add non-trivial delay to the sampling? Does that matter?
            // TODO will these need to be cleared now?
            
            ++numMsgs;
            
            if (numMsgs == MAX_NUM_MSGS) 
            {
              //numMsgs = 0; // Too many samples, stop collecting and add something smart to keep ROS messaging working
              Serial.println("************ \n\nUh oh! \n\n************");
            }
          
          }
          //Serial.println(numMsgs);


          // Send data
          Serial.println("(state 4) Sending data");
          Serial.println(numMsgs);
          //Serial.println(samples[0].dissolvedOxygen);
          // Dont forget to clear the data array!! TODO
          ++numMsgs; // Because the last one isn't full but isnt empty either
          for (int i = 0; i < numMsgs; ++i) 
          {
            Serial.print("Sending message ");
            teensy_data_pub.publish(&samples[i]);
            Serial.print("Sent message ");
            Serial.println(i);
            Serial.println("Printing contents");
            for (int j = 0; j < MAX_MSG_SIZE; ++j) 
            {
              Serial.print((String)samples[i].stamp[j].toSec() + "\t");
            }
            Serial.println();
          }
          Serial.println("Clearing data");
          //memset(samples, 0, sizeof(samples)); // This doesnt work, how overwrite old memory? Is necessary? TODO
          
          numMsgs = 0;
          currMsgSize = 0;

          //myFile.println("End sample");
          //myFile.flush();
    
          // When done sending data, inform nuc // Here safe to assume messages will go thru (?)
          Serial.println("(state 4) Informing Nuc data transmission complete");
          teensy_signal_pub.publish(&sig);
    
          // Change own state back to 0 i guess? // Actually nuc does it too, is redundant, choose one TODO
          Serial.println("(state 4) returning self to idle state");
          STATE = 0;
          //break;
      
  //}



}









