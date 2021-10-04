#include "ros/ros.h"
#include <cmath>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"


ros::Publisher chatter_custom;

ros::Time last_msg_custom0;
ros::Time last_msg_custom1;
ros::Time last_msg_custom2;
ros::Time last_msg_custom3;

double data0 = 0.;
double data1 = 0.;
double data2 = 0.;
double data3 = 0.;

bool timeOutMsg(ros::Time last_msg, double timeout = 0.5)
{
	if((ros::Time::now()-last_msg).toSec() < timeout)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int getNumberValid()
{
	int n = 0;
	if(timeOutMsg(last_msg_custom0))
	{
		n += 1;
	}
	if(timeOutMsg(last_msg_custom1))
	{
		n += 10;
	}
	if(timeOutMsg(last_msg_custom2))
	{
		n += 100;
	}
	if(timeOutMsg(last_msg_custom3))
	{
		n += 1000;
	}
	return n;
}

void callbackCustom0(const std_msgs::Bool& msg)
{
	data0 = (double)msg.data;
	last_msg_custom0 = ros::Time::now();
}

void callbackCustom1(const std_msgs::Float64& msg)
{
	data1 = msg.data;
	last_msg_custom1 = ros::Time::now();
}

void callbackCustom2(const std_msgs::Float64& msg)
{
	data2 = msg.data;
	last_msg_custom2 = ros::Time::now();
}

void callbackCustom3(const std_msgs::Float64& msg)
{
	data3 = msg.data;
	last_msg_custom3 = ros::Time::now();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "CustomMessages_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");



	std::string customTopic0 = nh.param<std::string>("customTopic0","");
	std::string customTopic1 = nh.param<std::string>("customTopic1","");
	std::string customTopic2 = nh.param<std::string>("customTopic2","");
	std::string customTopic3 = nh.param<std::string>("customTopic3","");
	ros::Subscriber sub_custom0;
	ros::Subscriber sub_custom1;
	ros::Subscriber sub_custom2;
	ros::Subscriber sub_custom3;

	if(customTopic0 != "")
	{
		sub_custom0 = n.subscribe(customTopic0, 1000, callbackCustom0);
	}
	if(customTopic1 != "")
	{
		sub_custom1 = n.subscribe(customTopic1, 1000, callbackCustom1);
	}
	if(customTopic2 != "")
	{
		sub_custom2 = n.subscribe(customTopic2, 1000, callbackCustom2);
	}
	if(customTopic3 != "")
	{
		sub_custom3 = n.subscribe(customTopic3, 1000, callbackCustom3);
	}

 	chatter_custom = n.advertise<std_msgs::Float64MultiArray>("/comms/out/sendCustom", 1000);

 	ros::Rate loop_rate(5);
 	std_msgs::Float64MultiArray msg;
 	msg.data.resize(5);
	
	while(ros::ok())
	{
		msg.data[0] = getNumberValid();
		msg.data[1] = data0;
		msg.data[2] = data1;
		msg.data[3] = data2;
		msg.data[4] = data3;

		chatter_custom.publish(msg);

		loop_rate.sleep();
		ros::spinOnce();
	}

  
	return 0;
}
