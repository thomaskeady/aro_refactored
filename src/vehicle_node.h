#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>
#include <stdlib.h>
//#include "aro_ft/aro_ft.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_msgs/JointState.h"


class DynamixelClient {

    //All speeds are in rad/s
    //Spool radius is ~1.5cm ~0.6in ~0.5ft
    private:
        ros::NodeHandle &nh_;

        float reel_speed;       // rad/s
        float desired_depth;    // ft
        float sample_time;      // seconds
        float current_torque;   // motor torque

        ros::ServiceClient dynamixel_client;
        ros::Subscriber dynamixel_torque;

    public:
        DynamixelClient(ros::NodeHandle &nh, float input_speed, float input_depth, float input_time):
                    nh_(nh),
                    reel_speed(input_speed),
                    desired_depth(input_depth),
                    sample_time(input_time),
                    current_torque(0),
                    dynamixel_client(nh_.serviceClient<dynamixel_controllers::SetSpeed>("/reel_controller/set_speed")),
                    dynamixel_torque(nh_.subscribe("/reel_controller/state", 100, &DynamixelClient::updateTorque, this))

        {}

        void actuateDynamixel() {
            dynamixel_controllers::SetSpeed temp_srv;
            temp_srv.request.speed = -reel_speed;   // Negative speed lets out line

	    if (desired_depth > 0) { // goto depth
                float duration = desired_depth / (reel_speed * 0.05);  // reel radius ~0.05ft

                ROS_INFO("Lowering Sonde to %f depth", desired_depth);
                dynamixel_client.call(temp_srv);
                ros::Duration(duration).sleep();        // Lower Sonde to desired depth

                temp_srv.request.speed = 0.0;
                dynamixel_client.call(temp_srv);        // Stop Sonde at desired Depth
	    }

	    if (desired_depth < 0) { // goto bottom
		ROS_INFO("Lowering Sonde to bottom.");
		dynamixel_client.call(temp_srv);
		ros::Duration(2).sleep();
	        while (current_torque < 0.15) { // 0.15 chosen arbitrarily
		    ROS_INFO("Torque: %f", current_torque);
    		    ros::spinOnce();
  		    ros::Duration(0.5).sleep();
		}
		temp_srv.request.speed = 0.0;
		dynamixel_client.call(temp_srv);
	    }

            ROS_INFO("Sonde lowered, sampling at specified depth.");
            ros::Duration(sample_time).sleep();     // Let Sonde sample at desired depth

            ROS_INFO("Raising Sonde.");
            temp_srv.request.speed = reel_speed;   // Positive speed reels line in

            //Raise for time:
            dynamixel_client.call(temp_srv);       // Raise Sonde
            //ros::Duration(duration).sleep();
	     // give it a moment to reverse torque sign

            //Raise until torque:
            while (current_torque > -0.18) {          // 0.8 chosen arbitrarily
                ROS_INFO("Torque: %f", current_torque);
                ros::spinOnce();
  		ros::Duration(0.5).sleep();
            }

	    //temp_srv.request.speed = 1;
	    //dynamixel_client.call(temp_srv);
	    //ros::Duration(3).sleep();

            temp_srv.request.speed = 0.0;
            dynamixel_client.call(temp_srv);
        }

        void updateTorque(const dynamixel_msgs::JointState& dynamixelState) {
            current_torque = dynamixelState.load;
        }
};



class CollectDataStateMachine 
{

private:
	ros::NodeHandle &nh_;
	rosbag::Bag bag;
	

	// Publishers
	ros::Publisher teensy_state_pub;

	// Subscribers
	ros::Subscriber teensy_signal_sub;
	ros::Subscriber teensy_data_sub;


	DynamixelClient dynamixel;


public:
	CollectDataStateMachine(ros::NodeHandle &nh) :
		nh_(nh),
		bag("collected_data.bag", rosbag::bagmode::Write),

		// Publishers
		teensy_state_pub(nh_.advertise<std_msgs::Int32>("teensy_state", 10)),

		// Subscribers
		teensy_signal_sub(nh_.subscribe("teensy_signal", 10, &CollectDataStateMachine::teensySignalCb, this)),
		teensy_data_sub(nh.subscribe("teensy_data", 10, &CollectDataStateMachine::teensyDataCb, this)),



		dynamixel(nh_, 5.0, 0.0, 5.0) // 3rd param "desired_depth" 0.0 by default for now

	{
		// pass for now
	}
	

	void teensySignalCb(const std_msgs::Int32& msg)
	{
		
	}

	// Need to change this to custom message type
	void teensyDataCb(const std_msgs::Empty& msg)
	{
		
	}



};





























