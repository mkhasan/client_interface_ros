


#include "client_interface/parser.h"
#include "client_interface/client_interface.h"

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <memory>
#include <assert.h>
#include <boost/algorithm/string.hpp>


#include <unistd.h>


#define KNNOT2MS  0.514444444

using namespace std;







namespace client_interface {

inline bool condition(int ch) {
	bool flag;
	flag = (ch == '#' || ch == '$' || ch == 'M');
	return flag;
}


void parse(DeviceGroup & devices, char *buffer, int index) {

	static int maxLen = 0;

	static char temp[1024];

	if(index < MIN_INDEX || index > MAX_INDEX)
		return;

	if(condition(buffer[0]) == false)
		return;

	buffer[index] = 0;

	if(buffer[0] == '#')
		ParseDevice(devices, buffer, "IMU");
	else if (buffer[0] == '$')
		ParseDevice(devices, buffer, "GPS");
	else if (buffer[0] == 'M') {
		int i=0;
		for(i=0; i<index; i++)
			temp[i] = buffer[i];
		temp[i] = 0;

		ROS_DEBUG("temp is: %s", temp);
		ParseDevice(devices, buffer, "MotorInfo");

	}

}


void ParseIMU(DeviceGroup & devices, const vector<string> results) {
	try {
		devices.dr_imu.seq = stoi(results[0]);
		int size = results.size();
		int i=1;

		if(i+14<size) {

			devices.dr_imu.estyaw = stof(results[i+1]);
			i+=2;
			for (int k=0; k<3; k++) {
				devices.dr_imu.gyro[k] = stoi(results[i+1+k]);
			}
			i += 1+3;

			for (int k=0; k<3; k++) {
				devices.dr_imu.accel[k] = stoi(results[i+1+k]);
			}
			i += 1+3;

			for (int k=0; k<3; k++) {
				devices.dr_imu.compass[k] = stoi(results[i+1+k]);
			}
			i += 1+3;

		}
	}
	catch (const exception & e) {

		THROW(RobotException, string("Nested error " + string(e.what())).c_str());
	}



}

void ParseGPS(DeviceGroup & devices, const vector<string> results) {

	int i=0;
	int size = results.size();

	if(8<results.size()) {
		i++;
		if(results[i].length() < sizeof(devices.dr_gps.timestamp))
			strcpy(devices.dr_gps.timestamp, results[i].c_str());
		else
			strcpy(devices.dr_gps.timestamp, "None");
		i++;
		if(results[i] == "A")
			strcpy(devices.dr_gps.state, "Valid");
		else if(results[i] == "V")
			strcpy(devices.dr_gps.state, "Invalid");
		else
			strcpy(devices.dr_gps.state, "None");

		i++;
		string temp = results[i+1] + string(":") + results[i];
		if(temp.length() < sizeof(devices.dr_gps.latitude))
			strcpy(devices.dr_gps.latitude, temp.c_str());
		else
			strcpy(devices.dr_gps.latitude, "None");

		i+=2;
		temp = results[i+1] + string(":") + results[i];
		if(temp.length() < sizeof(devices.dr_gps.longuitude))
			strcpy(devices.dr_gps.longuitude, temp.c_str());
		else
			strcpy(devices.dr_gps.longuitude, "None");

		i+=2;
		if(results[i].length() > 0)
			devices.dr_gps.vog = stof(results[i])*KNNOT2MS;
		else
			devices.dr_gps.vog = -1.0;
		i++;

		if(results[i].length() > 0)
			devices.dr_gps.cog = stof(results[i]);
		else
			devices.dr_gps.cog = -1.0;
		i++;




	}


}

void ParseMotorInfo(DeviceGroup & devices, char * buffer) {


	if(!(buffer[0] == 'M' && buffer[1] == 'M' && (buffer[2] == '0' || buffer[2] == '1') && buffer[3] == ' '))
		THROW(RobotException, "abnormal start of msg");

	int index = buffer[2]-'0';
	char * p = &buffer[4];
	if(strlen(p) == 0)
		THROW(RobotException, "buffer too short");

	int i;
	for(i=0; p[i] != '\r'; i++)
 		if(i == strlen(p)) {
 			THROW(RobotException, "carriage return not found");
 		}
	p[i] = NULL;
	vector<string> results, division;
	boost::split(results, p, [](char c){return c == '=';});

	if(results.size() < 2) {
		stringstream ss;
		ss << "primary split failed with src=" << p;
		THROW(RobotException, ss.str().c_str());
	}

	boost::split(division, results[1], [](char c) {return c == ':';});
	if(division.size() == 0) {
		stringstream ss;
		ss << "secondary split failed with src=" << results[1];
		THROW(RobotException, ss.str().c_str());

	}

	try {
		if (results[0] == "A") {
			devices.dr_motor.current[2*index+0] = stof(division[0])/10.0;
			devices.dr_motor.current[2*index+1] = stof(division[1])/10.0
					;
		}
		else if (results[0] == "AI") {
			devices.dr_motor.temperature[2*index+0] = ad2Temperature(stoi(division[2]));
			devices.dr_motor.temperature[2*index+1] = ad2Temperature(stoi(division[3]));
		}
		else if (results[0] == "C") {
			devices.dr_motor.position[2*index+0] = stoi(division[0]);
			devices.dr_motor.position[2*index+1] = stoi(division[1]);
		}
		else if (results[0] == "P") {
			devices.dr_motor.pwm[2*index+0] = stoi(division[0]);
			devices.dr_motor.pwm[2*index+1] = stoi(division[1]);
		}
		else if (results[0] == "S") {
			devices.dr_motor.speed[2*index+0] = stoi(division[0]);
			devices.dr_motor.speed[2*index+1] = stoi(division[1]);
		}
		else if (results[0] == "V") {
			devices.dr_motor.voltage = stof(division[1])/10.0;

		}
		else if (results[0] == "FF") {

			//ROS_INFO("buffer is %s", buffer);
			string Drv;
			int driverState[4];
			driverState[index] = stoi(results[1]);


			if ((driverState[index] & 0x1) != 0){
				Drv = "OH";
			}
			if ((driverState[index] & 0x2) != 0){
				Drv += "OV";
			}
			if ((driverState[index] & 0x4) != 0){
				Drv += "UV";
			}
			if ((driverState[index] & 0x8) != 0){
				Drv += "SHT";
			}
			if ((driverState[index] & 0x10) != 0){
				Drv += "ESTOP";
			}
			if ((driverState[index] & 0x20) != 0){
				Drv += "SEPF";
			}
			if ((driverState[index] & 0x40) != 0){
				Drv += "PromF";
			}
			if ((driverState[index] & 0x80) != 0){
				Drv += "ConfF";
			}

			if(Drv.length() < sizeof(devices.dr_motor.driverstate[index]))
				strcpy(devices.dr_motor.driverstate[index], Drv.c_str());
			else
				THROW(RobotException, "driverstate size is too short");

		}
	}
	catch (const exception & e) {
		THROW(RobotException, string("Nested error " + string(e.what())).c_str());
	}









}
void ParseDevice(DeviceGroup & devices, char * buffer, const char *deviceName) {

	static bool first = true;
	if(first)
		cout << buffer << endl;


	if(strcmp(deviceName, "MotorInfo") == 0) {
		try {
			ParseMotorInfo(devices, buffer);

		}
		catch(const exception& e) {
			ROS_ERROR("Error: %s", e.what());

		}
		return;
	}

	const char *p = &buffer[1];
	vector<std::string> results;
	try {


		boost::split(results, p, [](char c){return c == ',';});

		if(first) {
			cout << "size is " << results.size() << endl;
			cout << results[1] << endl;
		}

		if(strcmp(deviceName, "IMU") == 0)
			ParseIMU(devices, results);
		else if(strcmp(deviceName, "GPS") == 0)
			ParseGPS(devices, results);


 	}
	catch (const exception & e) {
		ROS_ERROR("Error: %s", e.what());

	}

	first = false;

}


float ad2Temperature(int adValue)
{

	//for new temperature sensor

	const double FULLAD = 4095;
	const double resTable[25] = { 114660, 84510, 62927, 47077, 35563, 27119, 20860, 16204, 12683, 10000, 7942, 6327, 5074, 4103, 3336, 2724, 2237, 1846, 1530, 1275, 1068, 899.3, 760.7, 645.2, 549.4 };
	const double tempTable[25] = { -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };


	double tempM = 0;
	double k = (adValue / FULLAD);

	double resValue = 0;
	if (k != 1)
	{
		resValue = 10000 * k / (1 - k);      //AD value to resistor
	}
	else
	{
		resValue = resTable[0];
	}

	int index = -1;
	if (resValue >= resTable[0])       //too lower
	{
		tempM = -20;
	}
	else if (resValue <= resTable[24])
	{
		tempM = 100;
	}
	else
	{
		for (int i = 0; i < 24; i++)
		{
			if ((resValue <= resTable[i]) && (resValue >= resTable[i + 1]))
			{
				index = i;
				break;
			}
		}
		if (index >= 0)
		{
			tempM = tempTable[index] + (resValue - resTable[index]) / (resTable[index + 1] - resTable[index]) * (tempTable[index + 1] - tempTable[index]);
		}
		else
		{
			tempM = 0;
		}
	}


	return tempM;

}

}
