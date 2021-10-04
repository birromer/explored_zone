#include "ros/ros.h"
#include "wayfinder.h"

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

#include <unistd.h>
#include <string>
#include <math.h>
#include <thread>


Wayfinder *wayfinder;

float speed_of_sound;
float max_track_range;
bool software_trigger;

void callbackTrigger(const std_msgs::Empty& msg)
{
	wayfinder->sendSoftwareTrigger();
}

void callbackGetSetup(const std_msgs::Empty& msg)
{
	wayfinder->sendCommandGetSetup();
}

void callbackGetSystem(const std_msgs::Empty& msg)
{
	wayfinder->sendCommandGetSystem();
}

void setSetup()
{
	wayfinder->sendCommandSetSetup(software_trigger,speed_of_sound,max_track_range);
	wayfinder->sendCommandGetSetup();
}

void callbackSetSoftwareTrigger(const std_msgs::Bool& msg)
{
	software_trigger = msg.data;
	setSetup();
}

void callbackSetSpeedOfSound(const std_msgs::Float32& msg)
{
	speed_of_sound = msg.data;
	setSetup();
}

void callbackSetMaxTrackRange(const std_msgs::Float32& msg)
{
	max_track_range = msg.data;
	setSetup();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Wayfinder_node");
	ros::NodeHandle nh("~");
    std::string port =  nh.param<std::string>("port","/dev/ttyUSB0");
    speed_of_sound = nh.param<float>("speed_of_sound",1500);
    max_track_range = nh.param<float>("max_track_range",10);
    software_trigger = nh.param<bool>("software_trigger",true);
   	ros::NodeHandle n;
	
	wayfinder = new Wayfinder(port,115200);

	std::thread thread_wayfinder(&Wayfinder::openSerial,wayfinder);



	ros::Publisher chatter_input_voltage = n.advertise<std_msgs::Float32>("wayfinder/info/input_voltage", 1000);
	ros::Publisher chatter_transmit_voltage = n.advertise<std_msgs::Float32>("wayfinder/info/transmit_voltage", 1000);
	ros::Publisher chatter_transmit_current = n.advertise<std_msgs::Float32>("wayfinder/info/transmit_current", 1000);
	ros::Publisher chatter_velocity = n.advertise<geometry_msgs::Vector3>("wayfinder/data/velocity", 1000);
	ros::Publisher chatter_velocity_error = n.advertise<std_msgs::Float32>("wayfinder/data/velocity_error", 1000);
	ros::Publisher chatter_depth = n.advertise<std_msgs::Float32MultiArray>("wayfinder/data/depth", 1000);
	ros::Publisher chatter_mean_depth = n.advertise<std_msgs::Float32>("wayfinder/data/mean_depth", 1000);

	ros::Subscriber sub_trigger = n.subscribe("wayfinder/command/trigger", 1000, callbackTrigger);
	ros::Subscriber sub_getsetup = n.subscribe("wayfinder/command/getsetup", 1000, callbackGetSetup);
	ros::Subscriber sub_getsystem = n.subscribe("wayfinder/command/getsystem", 1000, callbackGetSystem);

	ros::Subscriber sub_settrigger = n.subscribe("wayfinder/configure/software_trigger", 1000, callbackSetSoftwareTrigger);
	ros::Subscriber sub_setsos = n.subscribe("wayfinder/configure/speed_of_sound", 1000, callbackSetSpeedOfSound);
	ros::Subscriber sub_setmtr = n.subscribe("wayfinder/configure/max_track_range", 1000, callbackSetMaxTrackRange);


	std_msgs::Float32 msg_input_voltage;
	std_msgs::Float32 msg_transmit_voltage;
	std_msgs::Float32 msg_transmit_current;
	std_msgs::Float32 msg_velocity_error;
	std_msgs::Float32 msg_mean_depth;

	std_msgs::Float32MultiArray msg_depth;
	msg_depth.data.resize(4);

	geometry_msgs::Vector3 msg_velocity;

	wayfinder->sendCommandSetSetup(software_trigger,speed_of_sound,max_track_range);
	wayfinder->sendCommandGetSetup();

	while(ros::ok())
	{
		if(wayfinder->checkForNewData())
		{
			MessageDataOutput data = wayfinder->getData();
			
			msg_transmit_voltage.data = data.transmit_voltage;
			msg_transmit_current.data = data.transmit_current;
			msg_input_voltage.data = data.input_voltage;

			msg_velocity.x = data.bt_vel_x;
			msg_velocity.y = data.bt_vel_y;
			msg_velocity.z = data.bt_vel_z;

			msg_velocity_error.data = data.bt_vel_e;
			msg_depth.data[0] = data.range_to_bottom_1;
			msg_depth.data[1] = data.range_to_bottom_2;
			msg_depth.data[2] = data.range_to_bottom_3;
			msg_depth.data[3] = data.range_to_bottom_4;
			msg_mean_depth.data = data.mean_range_to_bottom;

			chatter_transmit_current.publish(msg_transmit_current);
			chatter_transmit_voltage.publish(msg_transmit_voltage);
			chatter_input_voltage.publish(msg_input_voltage);
			chatter_velocity.publish(msg_velocity);
			chatter_velocity_error.publish(msg_velocity_error);
			chatter_depth.publish(msg_depth);
			chatter_mean_depth.publish(msg_mean_depth);
		}
		usleep(1);
		ros::spinOnce();
	}


	thread_wayfinder.join();

	delete wayfinder;

	return 0;
}