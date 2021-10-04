#include "serial/serial.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"


serial::Serial uart_gnss;
std::string port;
int baudrate;

ros::Publisher chatter_fix;
ros::Publisher chatter_str;
ros::Publisher chatter_mode_mag;
ros::Publisher chatter_heading;
ros::Publisher chatter_imu;
ros::Publisher chatter_vel;

double sawtooth(double x)
{
  return 2.*atan(tan(x/2.));
}

std::vector<std::string> split(std::string& str, char c)
{
	std::istringstream stream(str);
	std::string elem;
	std::vector<std::string> elems;
	while(std::getline(stream, elem, c))
	{
		elems.push_back(elem);
	}
	return elems;
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

void parseNMEA(std::string data)
{
	//ROS_INFO("%s",data.c_str());
	int checksum = checksumNMEA(data);
	std::vector<std::string> split_nmea_checksum = split(data,'*');
	if(split_nmea_checksum.size() >= 2)
	{
		split_nmea_checksum[1] = split_nmea_checksum[1].substr(0,2);
		std::vector<std::string> split_nmea = split(split_nmea_checksum[0],',');
		if(split_nmea.size() >= 2)
		{	
			try
			{
				if(std::stoi(split_nmea_checksum[1],0,16) == checksum)
				{
					if(split_nmea[0].substr(1,5) == "PCGFD")
					{
						sensor_msgs::NavSatFix msg_navsatfix;
						msg_navsatfix.header.stamp = ros::Time::now();
						msg_navsatfix.latitude = std::stod(split_nmea[1]);
						msg_navsatfix.longitude = std::stod(split_nmea[2]);
						msg_navsatfix.altitude = std::stod(split_nmea[3]);
						msg_navsatfix.position_covariance[0] = std::stod(split_nmea[6]);
						msg_navsatfix.position_covariance[4] = std::stod(split_nmea[6]);
						msg_navsatfix.position_covariance[8] = std::stod(split_nmea[7]);
						msg_navsatfix.position_covariance_type = (uint8_t)2;

						double gyro_x = std::stod(split_nmea[10]);
						double gyro_y = std::stod(split_nmea[11]);
						double gyro_z = std::stod(split_nmea[12]);
						double acc_x = std::stod(split_nmea[13]);
						double acc_y = std::stod(split_nmea[14]);
						double acc_z = std::stod(split_nmea[15]);
						double pitch = std::stod(split_nmea[16]);
						double roll = std::stod(split_nmea[17]);
						double mag_heading = sawtooth(std::stod(split_nmea[18]));
						double compass_heading = std::stod(split_nmea[20]);
						double accuracy_heading = std::stod(split_nmea[21]);

						double vel = std::stod(split_nmea[23]);
						double heading_gnss = sawtooth(std::stod(split_nmea[24])*M_PI/180.);
						double acc_vel = std::stod(split_nmea[25]);
						double acc_heading_gnss = sawtooth(std::stod(split_nmea[26])*M_PI/180.);

						double heading;
						bool using_mag;

						if(accuracy_heading < 0.5) //35
						{
							using_mag = false;
							heading = compass_heading;
						}
						else
						{
							using_mag = true;
							heading = mag_heading;
						}
						tf2::Quaternion quaternion_tf2;
  						quaternion_tf2.setRPY(roll, pitch, heading);

						sensor_msgs::Imu msg_imu;
						msg_imu.header.frame_id = "imu";
						msg_imu.header.stamp = ros::Time::now();
						msg_imu.orientation = tf2::toMsg(quaternion_tf2);
						msg_imu.orientation_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};
						msg_imu.angular_velocity.x = gyro_x;
						msg_imu.angular_velocity.y = gyro_y;
						msg_imu.angular_velocity.z = gyro_z;
						msg_imu.angular_velocity_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};
						msg_imu.linear_acceleration.x = acc_x;
						msg_imu.linear_acceleration.y = acc_y;
						msg_imu.linear_acceleration.z = acc_z;
						msg_imu.linear_acceleration_covariance = {-1,0.,0.,0.,0.,0.,0.,0.,0.};

						std_msgs::Float64 msg_heading;
						msg_heading.data = heading;

						std_msgs::Bool msg_mode_mag;
						msg_mode_mag.data = using_mag;

						geometry_msgs::TwistWithCovariance msg_vel;
						msg_vel.twist.linear.x = vel;
						msg_vel.twist.angular.z = heading_gnss;
						msg_vel.covariance[0] = acc_vel;	
						msg_vel.covariance[35] = acc_heading_gnss;										
						std_msgs::String str_;
						str_.data = data;

						chatter_fix.publish(msg_navsatfix);
						chatter_imu.publish(msg_imu);
						chatter_heading.publish(msg_heading);
						chatter_mode_mag.publish(msg_mode_mag);
						chatter_vel.publish(msg_vel);
						chatter_str.publish(str_);
					}
				}
				else
				{
					ROS_WARN("Invalid NMEA checksum");
				}
			}
			catch(...)
			{
				ROS_WARN("Invalid data");
			}
		}
		else
		{
			ROS_WARN("Invalid data: not enought data");
		}
	}
	else
	{
		ROS_WARN("Invalid data: no checksum");
	}
	
}

bool openSerial(int argc, char **argv)
{
	ROS_INFO("Trying to access %s",port.c_str());
	try
	{
		uart_gnss.open();
		if(uart_gnss.isOpen())
		{
			
			ROS_INFO("Success");
			ROS_INFO("Ready to receive data...");
			std::string buffer = "";
			std::string msg_intro = "";
					
			while (ros::ok())
			{
				if(uart_gnss.available())
				{
				    std::string ch;
					if(uart_gnss.read(ch, 1) == 1)
					{
						if(ch == "$")
						{	
							if(buffer.length() != 0)
							{
								parseNMEA(buffer);
							}
							buffer = ch;
						}
						else
						{
							buffer = buffer + ch;
						}
					}
					else
					{
						ROS_WARN("Failure to read");
					}
				}
				usleep(1);
			}
			return true;
		}
		else
		{
			uart_gnss.close();
			ROS_ERROR("Failure to open. It should have triggered an error, not this.");
			return false;
		}	
	}
	catch (const serial::IOException& e)
	{
		uart_gnss.close();
		ROS_ERROR("Failure to access %s",port.c_str());
		return false;
	}
	catch (const serial::SerialException& e)
	{
		uart_gnss.close();
		ROS_ERROR("Unexpected exception catched");
		return false;
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Simple_GNSS_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
    port =  nh.param<std::string>("port","/dev/GNSS");
	baudrate =  nh.param<int>("baudrate",115200);
	uart_gnss.setPort(port);
	uart_gnss.setBaudrate(baudrate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	uart_gnss.setTimeout(timeout);
	chatter_fix = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);
	chatter_imu = n.advertise<sensor_msgs::Imu>("/imu/data", 1000);
	chatter_str = n.advertise<std_msgs::String>("/str_data", 1000);
	chatter_mode_mag = n.advertise<std_msgs::Bool>("/using_mag", 1000);
	chatter_heading = n.advertise<std_msgs::Float64>("/heading", 1000);
	chatter_vel = n.advertise<geometry_msgs::TwistWithCovariance>("/overGround", 1000);

	while(openSerial(argc,argv) == false and ros::ok())
	{
		usleep(1000000);
	}
	uart_gnss.close();

	return 0;

}
