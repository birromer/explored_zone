#include "serial/serial.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "protocol.h"

ros::Publisher chatter_mission;
ros::Publisher chatter_speed;
ros::Publisher chatter_start;
ros::Publisher chatter_stop;
ros::Publisher chatter_setHeading;
serial::Serial serialPort;
Protocol *p;

ros::Time last_msg_imu;
ros::Time last_msg_vel;
ros::Time last_msg_fix;
ros::Time last_msg_heading;
ros::Time last_msg_custom;
ros::Time last_msg_currentmode;
ros::Time last_msg_currentwp;
ros::Time last_msg_disttoline;

std::vector<uint8_t> buffer;
int bytesRemaining = -1;
uint8_t messageType;

double currentSpeed = 0;

std::vector<objectiveWpOrLine> listObjectivesReceive;
std::vector<objectiveWpOrLine> currentListObjectives;
std::vector<objectiveWpOrLine> objectivesToSend;
int numberOfObjectivesToReceive = -1;
int objectivesToSendId = 0;
bool sendingObjectives = false;
ros::Time last_send_mission;

double timeOutValue = 0.5;

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

void send(std::vector<uint8_t> data)
{
	int written = serialPort.write(data);
	if(written != data.size())
	{
		ROS_ERROR("Unable to send data");
	}	
}

void callbackVel(const geometry_msgs::TwistWithCovariance& msg)
{
	if(!timeOutMsg(last_msg_vel,timeOutValue))
	{
		std::vector<uint8_t> data = p->packOverGroundMsg(msg.twist.angular.z,msg.twist.linear.x,msg.covariance[35],msg.covariance[0]);
		send(data);
		last_msg_vel = ros::Time::now();
	}
}

void callbackFix(const sensor_msgs::NavSatFix& msg)
{
	if(!timeOutMsg(last_msg_fix,timeOutValue))
	{
		std::vector<uint8_t> data = p->packPosMsg(msg.latitude, msg.longitude, msg.altitude, msg.position_covariance[0], msg.position_covariance[8]);
		send(data);
		last_msg_fix = ros::Time::now();
	}
}

void callbackImu(const sensor_msgs::Imu& msg)
{
	if(!timeOutMsg(last_msg_imu,timeOutValue))
	{
		tf::Quaternion quat;
	    tf::quaternionMsgToTF(msg.orientation, quat);
	    double roll, pitch, yaw;
	    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	    std::vector<uint8_t> data = p->packAttitudeMsg(yaw, roll, pitch);
		send(data);
		last_msg_imu = ros::Time::now();
	}
}

void callbackSendHeadingObj(const std_msgs::Float64 &msg)
{
	if(!timeOutMsg(last_msg_heading,timeOutValue))
	{
		std::vector<uint8_t> dataToSend = p->packHeadingMsg(msg.data);
		send(dataToSend);
		last_msg_heading = ros::Time::now();
	}
}

void callbackSendCurrentMode(const std_msgs::Int8 &msg)
{
	if(!timeOutMsg(last_msg_currentmode,timeOutValue))
	{
		std::vector<uint8_t> dataToSend = p->packModeMsg((int8_t)msg.data);
		send(dataToSend);
		last_msg_currentmode = ros::Time::now();
	}
}

void callbackSendCurrentWp(const std_msgs::Float64MultiArray &msg)
{
	if(!timeOutMsg(last_msg_currentwp,timeOutValue))
	{
		std::vector<uint8_t> dataToSend = p->packCurrentWpMsg(msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4]);
		send(dataToSend);
		last_msg_currentwp = ros::Time::now();
	}
}

void callbackSendDistToLine(const std_msgs::Float64 &msg)
{
	if(!timeOutMsg(last_msg_disttoline,timeOutValue))
	{
		std::vector<uint8_t> dataToSend = p->packDistToLineMsg(msg.data);
		send(dataToSend);
		last_msg_disttoline = ros::Time::now();
	}
}

void callbackCustom(const std_msgs::Float64MultiArray &msg)
{
	if(!timeOutMsg(last_msg_custom,timeOutValue))
	{
		int validNumber = msg.data[0];
		uint8_t valid[4];
		for(int i = 0; i < 4; i++)
		{
			if(validNumber % 10 == 1)
			{
				valid[i] = 0x01;
			}
			else
			{
				valid[i] = 0x00;
			}
			validNumber /= 10;
		}
		double data[4] = {msg.data[1],msg.data[2],msg.data[3],msg.data[4]};
		std::vector<uint8_t> dataToSend = p->packCustomMsg(valid,data);
		send(dataToSend);
		last_msg_custom = ros::Time::now();
	}
}

void sendObjectives()
{
	sendingObjectives = true;
	std::vector<objectiveWpOrLine> toSend;
	uint8_t override;
	if(objectivesToSendId == 0)
	{
		override = 0x01;
	}
	else
	{
		override = 0x00;
	}
	int s = std::min((int)objectivesToSend.size(),objectivesToSendId+20);
	while(objectivesToSendId < s)
	{
		toSend.push_back(objectivesToSend[objectivesToSendId]);
		objectivesToSendId++;
	}
	std::vector<uint8_t> dataToSend = p->packMissionMsg(toSend,objectivesToSend.size(),override);
	send(dataToSend);
	if(objectivesToSendId == objectivesToSend.size())
	{
		objectivesToSendId = 0;
		objectivesToSend.clear();
		sendingObjectives = false;
	}
	else
	{
		last_send_mission = ros::Time::now();
	}
}

void unpackMessage(std::vector<uint8_t> data)
{
	std::string messageName = "";
	bool parsed = false;
	missionMsg missionmsg;
	speedMsg speedmsg;
	orderMsg ordermsg;
	headingMsg headingmsg;
	if(messageType == 0x04)
	{
		parsed = p->unpackMissionMsg(data,missionmsg);
		messageName = "mission";
	}
	else if(messageType == 0x05)
	{
		parsed = p->unpackSpeedMsg(data,speedmsg);
		messageName = "speed";
	}
	else if(messageType == 0x06)
	{
		parsed = p->unpackOrderMsg(data,ordermsg);
		messageName = "order";
	}
	else if(messageType == 0x09)
	{
		parsed = p->unpackHeadingMsg(data,headingmsg);
		messageName = "heading";
	}
	if(parsed)
	{
		if(messageName == "mission")
		{
			if(missionmsg.override == 0x01)
			{
				ROS_INFO("New waypoints");
				listObjectivesReceive.clear();
				numberOfObjectivesToReceive = missionmsg.totalObjectives;

			}
			for(int i = 0; i < missionmsg.objectives.size(); i++)
			{
				listObjectivesReceive.push_back(missionmsg.objectives[i]);
			}
			if(listObjectivesReceive.size() == numberOfObjectivesToReceive)
			{
				ROS_INFO("All waypoints received");
				std_msgs::Float64MultiArray msg;
				msg.data.resize(numberOfObjectivesToReceive*3+1);
				msg.data[0] = numberOfObjectivesToReceive;
				for(int i = 0; i < numberOfObjectivesToReceive; i++)
				{
					msg.data[1 + 3*i] =  listObjectivesReceive[i].lat;
					msg.data[2 + 3*i] =  listObjectivesReceive[i].lon;
					msg.data[3 + 3*i] =  (double)((int)listObjectivesReceive[i].isLine);
				}
				chatter_mission.publish(msg);
				numberOfObjectivesToReceive = -1;
				currentListObjectives = listObjectivesReceive;
				listObjectivesReceive.clear();
				std::vector<uint8_t> dataToSend = p->packOrderMsg(0x05);
				send(dataToSend);
			}

		}
		else if(messageName == "speed")
		{
			std_msgs::Float64 msg;
			msg.data = speedmsg.speed;
			chatter_speed.publish(msg);
			currentSpeed = speedmsg.speed;
		}
		else if(messageName == "heading")
		{
			std_msgs::Float64 msg;
			msg.data = headingmsg.heading;
			chatter_setHeading.publish(msg);
		}
		else if(messageName == "order")
		{
			std_msgs::Empty msg;
			if(ordermsg.order == 0x03)
			{
				chatter_start.publish(msg);
			}
			else if(ordermsg.order == 0x04)
			{
				chatter_stop.publish(msg);
			}
			else if(ordermsg.order == 0x01)
			{
				std::vector<uint8_t> dataToSend = p->packSpeedMsg(currentSpeed);
				send(dataToSend);
			}
			else if(ordermsg.order == 0x02 and !sendingObjectives)
			{
				objectivesToSend = currentListObjectives;
				objectivesToSendId = 0;
				sendObjectives();
			}
			else
			{
				ROS_WARN("Unknown order received");	
			}
		}
	}
	else if(messageName == "")
	{
		ROS_WARN("Unknown message received");
	}
	else
	{
		ROS_WARN("Bad message %s received",messageName.c_str());
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "serialPort_link_node");
	ros::NodeHandle nh("~");
	std::string port;
    int baudrate;
 	port = nh.param<std::string>("port","/dev/ttyUSB0");
	baudrate = nh.param<int>("baudrate",115200);
	timeOutValue = nh.param<double>("timeout",0.5);
	serialPort.setPort(port);
	serialPort.setBaudrate(baudrate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	serialPort.setTimeout(timeout);
	serialPort.open();
	p = new Protocol;

	if(serialPort.isOpen())
	{
		ros::NodeHandle n;
		ros::Subscriber sub_fix = n.subscribe("/fix", 1000, callbackFix);
		ros::Subscriber sub_vel = n.subscribe("/overGround", 1000, callbackVel);
		ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, callbackImu);
		ros::Subscriber sub_currentWp = n.subscribe("/comms/out/sendCurrentWp", 1000, callbackSendCurrentWp);
		ros::Subscriber sub_distToLine = n.subscribe("/comms/out/sendDistToLine", 1000, callbackSendDistToLine);
		ros::Subscriber sub_mode = n.subscribe("/comms/out/sendCurrentMode", 1000, callbackSendCurrentMode);
		ros::Subscriber sub_heading = n.subscribe("/comms/out/sendHeadingObj", 1000, callbackSendHeadingObj);
		ros::Subscriber sub_custom = n.subscribe("/comms/out/sendCustom", 1000, callbackCustom);

		chatter_mission = n.advertise<std_msgs::Float64MultiArray>("/comms/in/mission", 1000);
		chatter_speed = n.advertise<std_msgs::Float64>("/comms/in/speed", 1000);
		chatter_setHeading = n.advertise<std_msgs::Float64>("/comms/in/setHeading", 1000);
		chatter_stop = n.advertise<std_msgs::Empty>("/comms/in/stop", 1000);
		chatter_start = n.advertise<std_msgs::Empty>("/comms/in/start", 1000);

		//ROS_INFO("Ready to receive data...");
		
		while (ros::ok())
		{
			if(serialPort.isOpen() and serialPort.available())
			{
				std::string ch;
				int r = serialPort.read(ch,1);
				if(r == 1)
				{
					buffer.push_back((uint8_t)ch[0]);
					int s = buffer.size();
					if(bytesRemaining > 0)
					{
						bytesRemaining--;
						if(bytesRemaining == 0)
						{
							unpackMessage(buffer);
							buffer.clear();
							bytesRemaining = -1;
							messageType = 0x0;
						}
					}
					else if(s >= 6)
					{
						if(buffer[s-6] == 0xab and buffer[s-5] == 0xcd and buffer[s-4] == 0xef)
						{
							messageType = buffer[s-3];
							bytesRemaining =  buffer[s-2]+buffer[s-1]*256 - 6;
							buffer.erase(buffer.begin(),buffer.end()-6);
						}
					}
				}
			}
			if(sendingObjectives and !timeOutMsg(last_send_mission,0.4))
			{
				sendObjectives();
			}

			usleep(1);
			ros::spinOnce();
		}
		serialPort.close();
	}
	else
	{
		ROS_ERROR("FAILURE TO OPEN %s",port.c_str());
	}
	delete p;
	return 0;

}