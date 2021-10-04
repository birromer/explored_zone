#include "wayfinder.h"

uint8_t Wayfinder::parseU8(std::string data,int pos)
{
	return *(uint8_t*)data.substr(pos,sizeof(uint8_t)).c_str();
}
uint16_t Wayfinder::parseU16(std::string data,int pos)
{
	return *(uint16_t*)data.substr(pos,sizeof(uint16_t)).c_str();
}
uint32_t Wayfinder::parseU32(std::string data,int pos)
{
	return *(uint32_t*)data.substr(pos,sizeof(uint32_t)).c_str();
}
uint64_t Wayfinder::parseU64(std::string data,int pos)
{
	return *(uint64_t*)data.substr(pos,sizeof(uint64_t)).c_str();
}
float Wayfinder::parseFloat(std::string data,int pos)
{
	return *(float*)data.substr(pos,sizeof(float)).c_str();
}

std::vector<uint8_t> Wayfinder::addChecksum(std::vector<uint8_t> data)
{
	uint16_t checksum_calc = 0;
	for(int i = 0; i < data.size()-2 ;i++)
	{
		checksum_calc += data[i];
	}
	data[data.size()-2] = (checksum_calc & 0xFF);
	data[data.size()-1] = (checksum_calc >> 8);
	return data;
}

uint16_t Wayfinder::getChecksum(std::string data,int start, int end)
{
	uint16_t checksum_calc = 0;
	for(int i = start; i < end ;i++)
	{
		checksum_calc += parseU8(data,i);
	}
	return checksum_calc;
}

std::string Wayfinder::findResponseType(uint8_t response_id[7])
{
	bool valid = true;
	std::cout << std::endl;
	for(int i = 0; i < responses_list.size(); i++)
	{
		valid = true;
		for(int j = 0; j < 7; j++)
		{
			if(response_id[j] != responses_list[i].sequence[j])
			{
				valid = false;
			}
		}
		if(valid)
		{
			return responses_list[i].type;
		}
	}
	return "unknown";
}

void Wayfinder::parse(std::string data,std::string message_type)
{
	if(message_type == "data")
	{
		MessageDataOutput msg;
		for(int i = 0; i < 6; i++)
		{
			msg.sop_id[i] = parseU8(data,0+i);
		}
		for(int i = 0; i < 9; i++)
		{
			msg.data_id[i] = parseU8(data,6+i);
		}

		msg.system_id_type = parseU8(data,15);
		msg.system_sub_type = parseU8(data,16);
		msg.fw_version_major = parseU8(data,17);
		msg.fw_version_minor = parseU8(data,18);
		msg.fw_version_patch = parseU8(data,19);
		msg.fw_version_build = parseU8(data,20);
		msg.year = parseU8(data,21);
		msg.month = parseU8(data,22);
		msg.day = parseU8(data,23);
		msg.hour = parseU8(data,24);
		msg.minute = parseU8(data,25);
		msg.second = parseU8(data,26);
		msg.milliseconds = parseU16(data,27);
		msg.coordinate_system = parseU8(data,29);
		msg.bt_vel_x = parseFloat(data,30);
		msg.bt_vel_y = parseFloat(data,34);
		msg.bt_vel_z = parseFloat(data,38);
		msg.bt_vel_e = parseFloat(data,42);
		msg.range_to_bottom_1 = parseFloat(data,46);
		msg.range_to_bottom_2 = parseFloat(data,50);
		msg.range_to_bottom_3 = parseFloat(data,54);
		msg.range_to_bottom_4 = parseFloat(data,58);
		msg.mean_range_to_bottom = parseFloat(data,62);
		msg.speed_of_sound = parseFloat(data,66);
		msg.bt_status = parseU16(data,70);
		msg.bit = parseU16(data,72);
		msg.input_voltage  = parseFloat(data,74);
		msg.transmit_voltage = parseFloat(data,78);
		msg.transmit_current = parseFloat(data,82);
		for(int i = 0; i < 6; i++)
		{
			msg.system_serial_no[i] = parseU8(data,86+i);
		}
		for(int i = 0; i < 20; i++)
		{
			msg.reserved[i] = parseU8(data,92+i);
		}
		msg.checksum_data = parseU16(data,112);
		msg.checksum = parseU16(data,114);
		
		uint16_t checksum_calc = getChecksum(data,0,114);
		uint16_t checksum_data_calc = getChecksum(data,8,112);

		if(msg.checksum == checksum_calc && msg.checksum_data == checksum_data_calc)
		{
			//ROS_INFO("Message data with valid checksum");		
			data_valid = msg;
			newData = true;
		}
		else
		{
			ROS_WARN("Message data with invalid checksum");
		}
	}
	else if(message_type == "getsystem")
	{

		ResponseGetSystem msg;
		for(int i = 0; i < 6; i++)
		{
			msg.sop_id[i] = parseU8(data,0+i);
		}
		for(int i = 0; i < 7; i++)
		{
			msg.response_id[i] = parseU8(data,6+i);
		}

		msg.status_major = parseU8(data,13);
		msg.status_minor = parseU8(data,14);
		for(int i = 0; i < 6; i++)
		{
			msg.payload_header[i] = parseU8(data,15+i);
		}
		msg.frequency = parseFloat(data,21);
		msg.firmware = parseU32(data,25);
		msg.fpga_version = parseU32(data,29);
		msg.unique_system_id = parseU64(data,33);
		msg.xdcr_type = parseU8(data,41);
		msg.beam_angle = parseFloat(data,42);
		msg.vertical_beam = parseU8(data,46);
		for(int i = 0; i < 101; i++)
		{
			msg.reserved[i] = parseU8(data,47+i);
		}
		msg.system_type = parseU8(data,148);
		msg.system_sub_type = parseU8(data,149);
		msg.checksum = parseU16(data,150);

		uint16_t checksum_calc = getChecksum(data,0,150);

		if(msg.checksum == checksum_calc)
		{
			if((int)msg.vertical_beam == 1)
			{
				ROS_INFO("Current system\nBeam angle: %f\nVertical beam: true \n",msg.beam_angle);
			}
			else
			{
				ROS_INFO("Current system\nBeam angle: %f\nVertical beam: false \n",msg.beam_angle);
			}
		}
		else
		{
			ROS_WARN("Message system with invalid checksum");
		}
		readyToSendCommand = true;
	}
	else if(message_type == "getsetup")
	{
		ResponseGetSetup msg;
		for(int i = 0; i < 6; i++)
		{
			msg.sop_id[i] = parseU8(data,0+i);
		}
		for(int i = 0; i < 7; i++)
		{
			msg.response_id[i] = parseU8(data,6+i);
		}

		msg.status_major = parseU8(data,13);
		msg.status_minor = parseU8(data,14);
		for(int i = 0; i < 6; i++)
		{
			msg.payload_header[i] = parseU8(data,15+i);
		}
		msg.software_trigger = parseU8(data,21);
		msg.baudrate = parseU8(data,22);
		msg.speed_of_sound = parseFloat(data,23);
		msg.max_track_range = parseFloat(data,27);
		msg.reserved = parseFloat(data,31);
		msg.checksum = parseU16(data,35);

		uint16_t checksum_calc = getChecksum(data,0,35);

		if(msg.checksum == checksum_calc)
		{
			//std::cout << message_type << " valid checksum " << msg.max_track_range <<" "<< msg.speed_of_sound <<std::endl;
			
			if(msg.software_trigger == 1)
			{
				ROS_INFO("Current setup\nSpeed of sound: %f\nMax track range: %f \nSoftware trigger: true",msg.speed_of_sound,msg.max_track_range);
			}
			else
			{
				ROS_INFO("Current setup\nSpeed of sound: %f\nMax track range: %f \nSoftware trigger: false",msg.speed_of_sound,msg.max_track_range);
			}
		}
		else
		{
			ROS_WARN("Message getsetup with invalid checksum");
		}
		readyToSendCommand = true;
	}
	else if(message_type == "gettime")
	{
		ResponseGetTime msg;
		for(int i = 0; i < 6; i++)
		{
			msg.sop_id[i] = parseU8(data,0+i);
		}
		for(int i = 0; i < 7; i++)
		{
			msg.response_id[i] = parseU8(data,6+i);
		}

		msg.status_major = parseU8(data,13);
		msg.status_minor = parseU8(data,14);
		for(int i = 0; i < 6; i++)
		{
			msg.payload_header[i] = parseU8(data,15+i);
		}

		msg.year = parseU8(data,21);
		msg.month = parseU8(data,22);
		msg.day = parseU8(data,23);
		msg.hour = parseU8(data,24);
		msg.minute = parseU8(data,25);
		msg.second = parseU8(data,26);
		msg.checksum = parseU16(data,27);

		uint16_t checksum_calc = getChecksum(data,0,27);

		if(msg.checksum == checksum_calc)
		{
			//std::cout <<std::dec << message_type << " valid checksum " << (int)msg.day << " "<<(int)msg.month << " "<< (int)msg.year<< " "<< (int)msg.hour<< " "<< (int)msg.minute << " " << (int)msg.second << std::endl;
		}
		else
		{
			ROS_WARN("Message getTime with invalid checksum");
		}
		readyToSendCommand = true;
	}
	else if(message_type == "setortrigger")
	{
		ResponseSetOrTrigger msg;
		for(int i = 0; i < 6; i++)
		{
			msg.sop_id[i] = parseU8(data,0+i);
		}
		for(int i = 0; i < 7; i++)
		{
			msg.response_id[i] = parseU8(data,6+i);
		}

		std::string responseType = findResponseType(msg.response_id);

		msg.status_major = parseU8(data,13);
		msg.status_minor = parseU8(data,14);
		msg.checksum = parseU16(data,15);

		uint16_t checksum_calc = getChecksum(data,0,15);

		if(msg.checksum == checksum_calc)
		{
			if((int)msg.status_major == 1 && (int)msg.status_minor == 0)
			{
				ROS_INFO("Message setortrigger / %s - received",responseType.c_str());
			}
			else
			{
				ROS_WARN("Message setortrigger / %s - error code %i %i ",responseType.c_str(),(int)msg.status_major,(int)msg.status_minor);
			}
		}
		else
		{
			ROS_WARN("Message setortrigger / %s - Invalid checksum",responseType.c_str());
		}

		readyToSendCommand = true;
	}
}

bool Wayfinder::checkSequence(std::string buffer,int &remaining_bytes, std::string &incomingMessageType)
{
	bool valid = true;
	for(int i = 0; i < sop_list.size();i++)
	{
		if(buffer.length() >= sop_list[i].sequence.size())
		{
			valid = true;
			int k = buffer.length() - sop_list[i].sequence.size();
			for(int j = 0; j < sop_list[i].sequence.size();j++)
			{
				if((uint8_t)buffer[k+j] != sop_list[i].sequence[j])
				{
					valid = false;
				}
			}
			if(valid == true)
			{
				remaining_bytes = sop_list[i].lengthOfMessage - sop_list[i].sequence.size();
				incomingMessageType = sop_list[i].type;
				return true;
			}
		}
	}
	return false;	
}

void Wayfinder::sendCommand()
{
	if(readyToSendCommand)
	{
		if(commandsToSend.size() != 0)
		{
			wayfinder_device.write(commandsToSend[0]);
			commandsToSend.erase(commandsToSend.begin());
			readyToSendCommand = false;
			wayfinder_device.flush();
		}
	}
}

void Wayfinder::sendCommandGetSystem()
{
	CommandGetSystem cmd;

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[15]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendCommandGetSetup()
{
	CommandGetSetup cmd;

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[15]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendCommandGetTime()
{	
	CommandGetTime cmd;

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[15]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendSoftwareTrigger()
{
	CommandTrigger cmd;

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[15]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendCommandSpeedOfSound(float speed_of_sound)
{
	CommandSpeedOfSound cmd;

	memcpy(cmd.speed_of_sound, (uint8_t *)(&speed_of_sound), sizeof(cmd.speed_of_sound));
	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[19]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendCommandSetSetup(bool software_trigger,float speed_of_sound, float max_track_range)
{
	CommandSetSetup cmd;

	cmd.software_trigger = (uint8_t)(software_trigger);
	cmd.baudrate = (uint8_t)7;
	memcpy(cmd.speed_of_sound, (uint8_t *)(&speed_of_sound), sizeof(cmd.speed_of_sound));
	memcpy(cmd.max_track_range, (uint8_t *)(&max_track_range), sizeof(cmd.max_track_range));

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[35]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

void Wayfinder::sendCommandSetTime(int year,int month,int day, int hour, int minute, int second)
{
	CommandSetTime cmd;

	cmd.year = (uint8_t)year;
	cmd.month = (uint8_t)month;
	cmd.day = (uint8_t)day;
	cmd.hour = (uint8_t)hour;
	cmd.minute = (uint8_t)minute;
	cmd.second = (uint8_t)second;

	uint8_t* cmd_bytes = (uint8_t *)(&cmd);
	std::vector<uint8_t> cmd_bytes_vector(&cmd_bytes[0], &cmd_bytes[27]);
	commandsToSend.push_back(addChecksum(cmd_bytes_vector));
}

bool Wayfinder::checkForNewData()
{
	return newData;
}

MessageDataOutput Wayfinder::getData()
{
	newData = false;
	return data_valid;
}

void Wayfinder::openSerial()
{
	std::cout << "Trying to access " + port << std::endl;
	try
	{
		wayfinder_device.open();
		if(wayfinder_device.isOpen())
		{
			ROS_INFO("Success");
			ROS_INFO("Ready to receive data...");

			std::string buffer = "";
			int remaining_bytes = 0;
			std::string incomingMessageType = "";

			//sendCommandGetSystem();
			//sendCommandGetSetup();
			//sendCommandGetTime();
			//sendSoftwareTrigger();
			//sendCommandSetTime(21,6,3,19,40,1);
			//sendCommandSpeedOfSound(1550);
			//sendCommandSetSetup(false,1500,100);
			//sendCommandGetSetup();
			//sendCommandGetTime();
			while(ros::ok())
			{
				if(wayfinder_device.available())
				{
				    std::string ch;
					if(wayfinder_device.read(ch, 1) == 1)
					{
						buffer += ch;
						remaining_bytes -= 1;
						if(checkSequence(buffer,remaining_bytes,incomingMessageType))
						{
							buffer = buffer.substr(buffer.length()-6,buffer.length());
						}
						if(remaining_bytes == 0 && incomingMessageType != "")
						{
							parse(buffer,incomingMessageType);
							incomingMessageType = "";
						}
					}
				}
				sendCommand();
	
				usleep(1);
			}
		}
		else
		{
			wayfinder_device.close();
			ROS_ERROR("Failure to open. It should have triggered an error, not this");
		}	
	}
	catch (const serial::IOException& e)
	{
		wayfinder_device.close();
		ROS_ERROR("Failure to access %s",port.c_str());
	}
	catch (const serial::SerialException& e)
	{
		wayfinder_device.close();
		std::cout << "" << std::endl;
		ROS_ERROR("Unexpected exception catched");
	}

	
}

Wayfinder::Wayfinder(const std::string port_,const int baudrate_) : port(port_),baudrate(baudrate_)
{	
	wayfinder_device.setPort(port);
	wayfinder_device.setBaudrate(baudrate);
	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
	wayfinder_device.setTimeout(timeout);

	SOP sop_data;
	SOP sop_getsystem;
	SOP sop_getsetup;
	SOP sop_gettime;
	SOP sop_set;

	sop_data.type = "data";
	sop_getsystem.type = "getsystem";
	sop_getsetup.type = "getsetup";
	sop_gettime.type = "gettime";
	sop_set.type = "setortrigger";

	sop_data.lengthOfMessage = 116;
	sop_getsystem.lengthOfMessage = 152;
	sop_getsetup.lengthOfMessage = 37;
	sop_gettime.lengthOfMessage = 29;
	sop_set.lengthOfMessage = 17;

	sop_data.sequence = {0xAA,0x10,0x01,0x74,0x00,0x10};
	sop_getsystem.sequence = {0xAA,0x10,0x01,0x98,0x00,0x10};
	sop_getsetup.sequence = {0xAA,0x10,0x01,0x25,0x00,0x10};
	sop_gettime.sequence = {0xAA,0x10,0x01,0x1D,0x00,0x10};
	sop_set.sequence = {0xAA,0x10,0x01,0x11,0x00,0x10};

	sop_list.push_back(sop_data);
	sop_list.push_back(sop_getsystem);
	sop_list.push_back(sop_getsetup);
	sop_list.push_back(sop_gettime);
	sop_list.push_back(sop_set);


	Response response_trigger;
	Response response_settime;
	Response response_setsetup;
	Response response_setspeedofsound;

	response_trigger.type = "trigger";
	response_settime.type = "settime";
	response_setsetup.type = "setsetup";
	response_setspeedofsound.type = "setspeedofsound";

	response_trigger.sequence = {0x04,0x0A,0x00,0x11,0x00,0x00,0x00};
	response_settime.sequence = {0x04,0x0A,0x00,0x02,0x00,0x00,0x1F};
	response_setsetup.sequence = {0x04,0x0A,0x00,0x02,0x00,0x00,0x87};
	response_setspeedofsound.sequence = {0x04,0x0A,0x00,0x03,0x00,0x00,0x86};


	responses_list.push_back(response_trigger);
	responses_list.push_back(response_setsetup);
	responses_list.push_back(response_setspeedofsound);
	responses_list.push_back(response_settime);

	readyToSendCommand = true;
	newData = false;

}

Wayfinder::~Wayfinder()
{
	wayfinder_device.close();
}