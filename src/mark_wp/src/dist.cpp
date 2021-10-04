#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "UTM.h"
#include <fstream>

bool assistOn = false;

float lat,lon,previous_lat,previous_lon;
float pos_x,pos_y;

float previous_pos_x,previous_pos_y;
bool previous_pos = false;

std::ofstream log_file;

float dist(float x1, float y1, float x2, float y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void calcDist(float x1, float y1)
{
	if(previous_pos)
	{
		float d = dist(x1,y1,previous_pos_x,previous_pos_y);
		
		std::cout << std::setprecision(10) << previous_lat << " " << std::setprecision(10) << previous_lon << " " << std::setprecision(10) << lat << " " << std::setprecision(10)<< lon << " " << std::setprecision(10)<< d << std::endl;
		log_file << std::setprecision(10) << previous_lat << " " << std::setprecision(10) << previous_lon << " " << std::setprecision(10) << lat << " " << std::setprecision(10)<< lon << " " << std::setprecision(10)<< d << std::endl;
	}
}

void callbackRcAssist(const std_msgs::Bool& msg)
{
	if(assistOn == true and msg.data == false)
	{
		assistOn = false;
	}
	else if(assistOn == false and msg.data == true)
	{
		assistOn = true;
		calcDist(pos_x,pos_y);
		previous_pos_x = pos_x;
		previous_pos_y = pos_y;
		previous_lat = lat;
		previous_lon = lon;
		previous_pos = true;
	}
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	LatLonToUTMXY(msg.latitude,msg.longitude,0,pos_y,pos_x);
	lat = msg.latitude;
	lon = msg.longitude;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Dist_node");

	ros::NodeHandle n;

	ros::Subscriber sub_pos = n.subscribe("/fix", 1000, callbackFix);
	ros::Subscriber sub_rc_enabled = n.subscribe("/assist/enabled", 1000, callbackRcAssist);
    std::string name = "log_points_"+ std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())+".txt";
  	log_file.open(name, std::ios::out);
	ros::spin();
	log_file.close();
  
	return 0;
}