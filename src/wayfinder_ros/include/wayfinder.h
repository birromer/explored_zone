#ifndef __ZEDP9P_H__
#define __ZEDP9P_H__
#include <stdio.h>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <vector>
#include <sstream>
#include <cstdio>
#include <chrono>
#include "serial/serial.h"
#include "ros/ros.h"

struct SOP
{
	std::string type;
	int lengthOfMessage;
	std::vector<uint8_t> sequence;
};

struct Response
{
	std::string type;
	std::vector<uint8_t> sequence;
};

struct MessageDataOutput
{
	uint8_t sop_id[6];
	uint8_t data_id[9];
	uint8_t system_id_type;
	uint8_t system_sub_type;
	uint8_t fw_version_major;
	uint8_t fw_version_minor;
	uint8_t fw_version_patch;
	uint8_t fw_version_build;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t milliseconds;
	uint8_t coordinate_system;
	float bt_vel_x;
	float bt_vel_y;
	float bt_vel_z;
	float bt_vel_e;
	float range_to_bottom_1;
	float range_to_bottom_2;
	float range_to_bottom_3;
	float range_to_bottom_4;
	float mean_range_to_bottom;
	float speed_of_sound;
	uint16_t bt_status;
	uint16_t bit;
	float input_voltage;
	float transmit_voltage;
	float transmit_current;
	uint8_t	system_serial_no[6];
	uint8_t	reserved[20];
	uint16_t checksum_data;
	uint16_t checksum;
};

struct CommandGetSystem //Ok
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x0F,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x08,0x00,0x01,0x00,0x00,0x81};
	uint8_t checksum[2]; //= {0x67,0x01};
};

struct CommandGetSetup //Ok
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x0F,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x08,0x00,0x01,0x00,0x00,0x85};
	uint8_t checksum[2];// = {0x6B,0x01};
};

struct CommandGetTime //Ok
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x0F,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x08,0x00,0x01,0x00,0x00,0x1D};
	uint8_t checksum[2];// = {0x03,0x01};
};

struct CommandTrigger //Ok
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x0F,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x08,0x00,0x11,0x00,0x00,0x00};
	uint8_t checksum[2];// = {0xF6,00};
};

struct CommandSpeedOfSound //Ok
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x13,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x0C,0x00,0x03,0x00,0x00,0x86};
	uint8_t speed_of_sound[4]; //float
	uint8_t checksum[2];
};

struct CommandSetSetup
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x23,0x00,0x10}; //Ok
	uint8_t command_id[7] = {0x03,0x1c,0x00,0x02,0x00,0x00,0x87};
	uint8_t structure_id[6] = {0x22,0x12,0x14,0x00,0x00,0x00};
	uint8_t software_trigger;
	uint8_t baudrate;
	uint8_t speed_of_sound[4]; //float
	uint8_t max_track_range[4]; //float
	uint8_t reserved[4] = {0x00,0x00,0x00,0x00}; //float
	uint8_t checksum[2];
};

struct CommandSetTime
{
	uint8_t sop_id[6] = {0xAA,0x10,0x01,0x1B,0x00,0x10};
	uint8_t command_id[7] = {0x03,0x14,0x00,0x02,0x00,0x00,0x1F};
	uint8_t structure_id[6] = {0x23,0x10,0x0C,0x00,0x00,0x00};
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t checksum[2];
};

struct ResponseGetSystem
{
	uint8_t sop_id[6];
	uint8_t response_id[7];
	uint8_t status_major;
	uint8_t status_minor;
	uint8_t payload_header[6];
	float frequency;
	uint32_t firmware;
	uint32_t fpga_version;
	uint64_t unique_system_id;
	uint8_t xdcr_type;
	float beam_angle;
	uint8_t vertical_beam;
	uint8_t reserved[101];
	uint8_t system_type;
	uint8_t system_sub_type;
	uint16_t checksum;

};

struct ResponseGetSetup
{
	uint8_t sop_id[6];
	uint8_t response_id[7];
	uint8_t status_major;
	uint8_t status_minor;
	uint8_t payload_header[6];
	uint8_t software_trigger;
	uint8_t baudrate;
	float speed_of_sound;
	float max_track_range;
	float reserved;
	uint16_t checksum;
};

struct ResponseGetTime
{
	uint8_t sop_id[6];
	uint8_t response_id[7];
	uint8_t status_major;
	uint8_t status_minor;
	uint8_t payload_header[6];
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint16_t checksum;
};

struct ResponseSetOrTrigger
{
	uint8_t sop_id[6];
	uint8_t response_id[7];
	uint8_t status_major;
	uint8_t status_minor;
	uint16_t checksum;
};

class Wayfinder
{

public:
	
	Wayfinder(const std::string port_,const int baudrate_);
	~Wayfinder();
	void openSerial();

	void sendCommandGetSystem();
	void sendCommandGetSetup();
	void sendCommandGetTime();
	void sendSoftwareTrigger();
	void sendCommandSpeedOfSound(float speed_of_sound);
	void sendCommandSetSetup(bool software_trigger,float speed_of_sound, float max_track_range);
	void sendCommandSetTime(int year,int month,int day, int hour, int minute, int second);
	bool checkForNewData();
	MessageDataOutput getData();

private:

	uint8_t parseU8(std::string data,int pos);
	uint16_t parseU16(std::string data,int pos);
	uint32_t parseU32(std::string data,int pos);
	uint64_t parseU64(std::string data,int pos);
	float parseFloat(std::string data,int pos);

	uint16_t getChecksum(std::string data,int start, int end);
	std::vector<uint8_t> addChecksum(std::vector<uint8_t> data);

	void parse(std::string buffer,std::string message_type);

	bool checkSequence(std::string buffer,int &remaining_bytes, std::string &incomingMessageType);
	std::string findResponseType(uint8_t response_id[7]);
	void sendCommand();

	std::string device_name;
	std::string port;
	int baudrate;
	serial::Serial wayfinder_device;

	std::vector<SOP> sop_list;
	std::vector<Response> responses_list;

	std::vector<std::vector<uint8_t>> commandsToSend;
	bool readyToSendCommand;

	bool newData;
	MessageDataOutput data_valid;


};
#endif