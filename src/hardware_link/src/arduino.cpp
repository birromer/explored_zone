#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

ros::Publisher chatter_arduino_cmd;
ros::Publisher chatter_arduino_camera;
ros::Publisher chatter_arduino_rc_enabled;
ros::Publisher chatter_arduino_assist;

ros::Time last_msg_cmd;
ros::Time last_msg_camera;
ros::Time last_send;

double cmd_forward;
double cmd_left_right;
double cmd_camera;

serial::Serial arduino;

const int send_freq = 25;


std::vector<std::string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

int checksumNMEA(std::string data)
{
	int checksum = 0;
	for(int i = 0; i < data.length(); i++)
	{
		if(data.at(i)!= '$')
		{
			if(data.at(i) == '*')
			{
				return checksum;
			}
			else
			{
				checksum ^= (char)data.at(i);
			}

		}

	}

	return checksum;
}

std::string addChecksum(std::string data, bool addStar = true)
{
	std::stringstream stream; 
	stream << data;
	if(addStar)
	{
		stream << "*";
	}
	stream << std::hex << std::setw(2) << std::uppercase << std::setfill('0') << checksumNMEA(data);
	stream << '\r' << '\n';
	return stream.str();
}

double normalize(double x)
{
	return std::max(-1.,std::min(1.,x));
}

void callbackCmd(const geometry_msgs::Twist& msg)
{
	cmd_forward = normalize(msg.linear.x);
	cmd_left_right = normalize(msg.angular.z);
  	last_msg_cmd = ros::Time::now();
}

void callbackCamera(const std_msgs::Float64& msg)
{
	cmd_camera = msg.data;
  	last_msg_camera = ros::Time::now();
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

bool parseNMEA(std::string data)
{
	int found_comma1,found_comma2,found_comma3,found_star;
	found_comma1 = data.find(",");
	found_star = data.find("*");
	std::string type_of_data;
  	if (found_comma1 == std::string::npos or found_star == std::string::npos)
  	{
  		return false;
  	}
  	try
  	{
	  	int checksum = std::stoi(data.substr(found_star+1,2),0,16);
	  	std::string p1 = data.substr(1,found_comma1-1);
	  	if(checksum != checksumNMEA(data))
	  	{
	  		ROS_WARN("INVALID CHECKSUM");
	  		return false;
	  	}
	  	if(p1 != "PARDN")
	  	{
	  		return false;
	  	}
	  	data = data.substr(found_comma1+1,data.length());
	  	found_comma2 = data.find(",");
		type_of_data = data.substr(0,found_comma2);
		if(type_of_data == "CMD_RC")
		{
			std::vector<std::string> info_data = split(data.substr(found_comma2+1,found_star-(found_comma1+found_comma2+2)),',');
			if(info_data.size() != 5)
			{
				return false;
			}
			else
			{	

				double FORWARD = std::stod(info_data[0]);
				double LEFT_RIGHT = std::stod(info_data[1]);
				int MODE = std::stoi(info_data[2]);
				int ASSIST = std::stoi(info_data[3]);
				double CAMERA = std::stod(info_data[4]);

				geometry_msgs::Twist msg_cmd;
				std_msgs::Bool msg_rc_enabled;
				std_msgs::Bool msg_assist;
				std_msgs::Float64 msg_camera;
				msg_cmd.linear.x = FORWARD;
				msg_cmd.angular.z = LEFT_RIGHT;
				msg_rc_enabled.data = (MODE == 1);
				msg_assist.data = (ASSIST == 1);
				msg_camera.data = CAMERA;

				chatter_arduino_cmd.publish(msg_cmd);
				chatter_arduino_rc_enabled.publish(msg_rc_enabled);
				chatter_arduino_assist.publish(msg_assist);
				chatter_arduino_camera.publish(msg_camera);

				return true;
			}
	  	}
	  	else
	  	{
	  		return false;
	  	}
  	}
  	catch(...)
  	{
  		ROS_ERROR("Invalid data");
  	}
  	return false;


}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Arduino_link_node");
    std::string port = "/dev/ARDUINO";
   
	arduino.setPort(port);
	arduino.setBaudrate(115200);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	arduino.setTimeout(timeout);
	arduino.open();
	if(arduino.isOpen())
	{
		ros::NodeHandle n;
		ros::Subscriber sub_cmd = n.subscribe("auto/cmd", 1000, callbackCmd);
		ros::Subscriber sub_camera = n.subscribe("auto/camera", 1000, callbackCamera);
		chatter_arduino_cmd = n.advertise<geometry_msgs::Twist>("/rc/cmd", 1000);
		chatter_arduino_rc_enabled = n.advertise<std_msgs::Bool>("/rc/enabled", 1000);
		chatter_arduino_assist = n.advertise<std_msgs::Bool>("/rc/assist", 1000);
		chatter_arduino_camera = n.advertise<std_msgs::Float64>("/rc/camera", 1000);
		std::string buffer = "";
		last_send = ros::Time::now();
		
		while (ros::ok())
		{
			while(arduino.available())
			{
			    std::string ch;
				if(arduino.read(ch, 1) == 1)
				{
					if(ch == "$")
					{
						parseNMEA(buffer);
						buffer = ch;				
					}
					else
					{
						buffer = buffer + ch;
					}
					//std::cout << ch;

				}
				else
				{
					ROS_WARN("Failure to read");
				}
			}



		if(!timeOutMsg(last_send,1./(double)send_freq))
		{

			if(timeOutMsg(last_msg_cmd) and timeOutMsg(last_msg_camera))
			{
				std::string str_cmd = "$PARDN,CMD_AUTO,"+std::to_string(cmd_forward)+","+std::to_string(cmd_left_right)+","+std::to_string(cmd_camera);
				str_cmd = addChecksum(str_cmd);
				arduino.write(str_cmd);
				last_send = ros::Time::now();
			}
			else if(timeOutMsg(last_msg_cmd))
			{
				std::string str_cmd = "$PARDN,CMD_AUTO,"+std::to_string(cmd_forward)+","+std::to_string(cmd_left_right)+","+std::to_string(0.);
				str_cmd = addChecksum(str_cmd);
				arduino.write(str_cmd);
				last_send = ros::Time::now();
			}
			else if(timeOutMsg(last_msg_camera))
			{
				std::string str_cmd = "$PARDN,CMD_AUTO,"+std::to_string(0.)+","+std::to_string(0.)+","+std::to_string(cmd_camera);
				str_cmd = addChecksum(str_cmd);
				arduino.write(str_cmd);
				last_send = ros::Time::now();
			}
		}

			usleep(1);
			ros::spinOnce();
		}
		arduino.close();
	}
	else
	{
		ROS_ERROR("FAILURE TO OPEN %s",port.c_str());
	}
	return 0;

}