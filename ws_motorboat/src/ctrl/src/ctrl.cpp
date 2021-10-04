#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>


double heading;
double gyro_z;
ros::Time last_msg_imu;

double auto_heading = 0.;
double auto_speed= 0.;
ros::Time last_msg_cmd;

double speed = 0.;

const int freq = 30;

double previous_obj = 0.;
double error_integral = 0.;
double kp;
double kd;
double ki;

bool rc_assist = false;
bool rc_enabled = false;
double rc_forward = 0.;
double rc_left_right= 0.;

bool rc_assist_enabled = false;
double rc_assist_heading;

ros::Time last_msg_rc_assist;
ros::Time last_msg_rc_enabled;
ros::Time last_msg_rc_cmd;

void callbackImu(const sensor_msgs::Imu& msg)
{
	heading = tf::getYaw(msg.orientation);
	gyro_z = msg.angular_velocity.z;

	last_msg_imu = ros::Time::now();
}

void callbackRcEnabled(const std_msgs::Bool& msg)
{
	rc_enabled = msg.data;
	last_msg_rc_enabled = ros::Time::now();
}

void callbackRcAssist(const std_msgs::Bool& msg)
{
	rc_assist = msg.data;
	last_msg_rc_assist = ros::Time::now();
}

void callbackRcCmd(const geometry_msgs::Twist& msg)
{
	rc_forward = msg.linear.x;
	rc_left_right = msg.angular.z;
	last_msg_rc_cmd = ros::Time::now();
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
	auto_speed = msg.linear.x;
	auto_heading = msg.angular.z;
	last_msg_cmd = ros::Time::now();
}

double sawtooth(double x)
{
	return 2.*atan(tan(x/2.));
}

bool timeOutMsg(ros::Time last_msg, double timeout = 0.2)
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

double PID(double mes, double obj, double mes_dot)
{
	double output;
	double error_proportionnal = sawtooth(obj - mes);
	double error_derivative = 0. - mes_dot;

	if(previous_obj == obj)
	{
		error_integral += error_proportionnal*1./(double)freq;
	
	}
	else
	{
		error_integral = error_proportionnal*1./(double)freq;
	}

	output = kp*error_proportionnal + kd*error_derivative + ki*error_integral;

	previous_obj = obj;

	return output;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Ctrl_node");

	ros::NodeHandle n;
	ros::Subscriber sub_heading = n.subscribe("/imu/data", 1000, callbackImu);

	ros::Subscriber sub_rc_cmd= n.subscribe("/rc/cmd", 1000, callbackRcCmd);
	ros::Subscriber sub_rc_enabled = n.subscribe("/rc/enabled", 1000, callbackRcEnabled);
	ros::Subscriber sub_rc_assist = n.subscribe("/rc/assist", 1000, callbackRcAssist);

	ros::Subscriber sub_cmd = n.subscribe("/cmdVelHeading", 1000, callbackCmd);
 	ros::Publisher chatter_cmd = n.advertise<geometry_msgs::Twist>("/auto/cmd", 1000);
 	ros::Publisher chatter_assist_enabled = n.advertise<std_msgs::Bool>("/assist/enabled", 1000);
 	ros::Publisher chatter_assist_heading = n.advertise<std_msgs::Float64>("/assist/heading", 1000);

	ros::NodeHandle nh("~");
 	kp = nh.param<double>("kp",1.2);
	kd = nh.param<double>("kd",0.1);
	ki = nh.param<double>("ki",0.);

  	ros::Rate loop_rate(freq);

  	ROS_INFO("kp: %f   kd: %f   ki: %f",kp,kd,ki);
 
    while (ros::ok())
    {
    	geometry_msgs::Twist msg_cmd;
    	std_msgs::Bool msg_assist_enabled;
    	std_msgs::Float64 msg_assist_heading;

    	if(rc_enabled and rc_assist and timeOutMsg(last_msg_imu) and timeOutMsg(last_msg_rc_cmd) and timeOutMsg(last_msg_rc_enabled) and timeOutMsg(last_msg_rc_assist))
    	{
    		if(!rc_assist_enabled)
    		{
				rc_assist_enabled = true;
				rc_assist_heading = heading;
    		}
    		else if(std::fabs(rc_left_right) > 0.1)
			{
				rc_assist_heading = sawtooth(rc_assist_heading + 1./(double)freq*rc_left_right*M_PI/4.);
			}

    		msg_cmd.linear.x = rc_forward;
	    	msg_cmd.angular.z = PID(heading,rc_assist_heading,gyro_z);
	    	msg_assist_heading.data = rc_assist_heading;
	    	chatter_assist_heading.publish(msg_assist_heading);

    	}
    	else if(timeOutMsg(last_msg_imu) and timeOutMsg(last_msg_cmd))
    	{
    		rc_assist_enabled = false;
    		msg_cmd.linear.x = auto_speed;
	    	msg_cmd.angular.z = PID(heading,auto_heading,gyro_z);
    	}
    	else
    	{
    		rc_assist_enabled = false;
    		msg_cmd.linear.x = 0.;
    		msg_cmd.angular.z = 0.;
    	}
    	
    	//std::cout << msg_cmd.angular.z << std::endl;
    	msg_assist_enabled.data = rc_assist_enabled;
    	chatter_assist_enabled.publish(msg_assist_enabled);
		chatter_cmd.publish(msg_cmd);

        loop_rate.sleep();
	    ros::spinOnce();
    }

  
	return 0;
}