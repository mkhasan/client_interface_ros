/*
 * client_interface.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: kict
 */




#include "client_interface/client_interface.h"
#include "client_interface/dr_robot.h"

#include "ros/ros.h"

#include <boost/algorithm/string.hpp>

#include <string.h>
#include <iostream>
#include <sstream>

using namespace std;
using namespace mosqpp;




client_interface::ClientInterface::ClientInterface(DB_Handler &_db_handler, const char *id, const char *host, int port)
	: db_handler(_db_handler)
	, publishAngle(false)
	, publishIMU(false)
	, publishGPS(false)
	, pingReceived(false)
	, speedoConnected(false)
	, mosquittopp(id)
{
	int keepalive = 60;

	/* Connect immediately. This could also be done by calling
	 * mqtt_tempconv->connect(). */
	connect(host, port, keepalive);


}



client_interface::ClientInterface::~ClientInterface()
{
	disconnect();
}


void client_interface::ClientInterface::on_connect(int rc)
{
	cout << "Connected with code " << rc << endl;
	if(rc == 0){

		subscribe(NULL, SPECTATOR_GET_ANGLES_START);
		subscribe(NULL, SPECTATOR_GET_ANGLES_STOP);
		subscribe(NULL, IMU_START);
		subscribe(NULL, IMU_STOP);
		subscribe(NULL, GPS_START);
		subscribe(NULL, GPS_STOP);
		subscribe(NULL, SPECTATOR_HOMING);
		subscribe(NULL, SPECTATOR_GOTO_ABS);
		subscribe(NULL, DB_MANUAL);
		subscribe(NULL, DB_AUTO_START);
		subscribe(NULL, DB_AUTO_STOP);
		subscribe(NULL, DB_AUTO_VIDEO_START);
		subscribe(NULL, DB_AUTO_VIDEO_STOP);
		subscribe(NULL, DB_SEMI_AUTO_START);
		subscribe(NULL, DB_SEMI_AUTO_STOP);
		subscribe(NULL, DB_SEMI_AUTO_SNAP);
		subscribe(NULL, SYS_REBOOT);
		subscribe(NULL, SYS_POWEROFF);
		subscribe(NULL, LIGHTS_OFF);
		subscribe(NULL, LIGHTS_ON);


		// for Speedo //
		subscribe(NULL, SPEEDO_INFO);
		//subscribe(NULL, SPEEDO_IS_CONNECTED);

	}
}


void client_interface::ClientInterface::on_message(const struct mosquitto_message *message) {

	if(!strcmp(message->topic, SPECTATOR_GET_ANGLES_START)) {
		this->publishAngle = true;
	}
	else if (!strcmp(message->topic, SPECTATOR_GET_ANGLES_STOP)) {
		this->publishAngle = false;
	}
	else if (!strcmp(message->topic, IMU_START)) {
		this->publishIMU = true;
	}
	else if (!strcmp(message->topic, IMU_STOP)) {
		this->publishIMU = false;
	}
	else if (!strcmp(message->topic, GPS_START)) {
		this->publishGPS = true;
	}
	else if (!strcmp(message->topic, GPS_STOP)) {
		this->publishGPS = false;
	}
	else if (!strcmp(message->topic, DB_MANUAL)) {
		int ret;
		DB_Handler::StoreMode storeMode(DB_Handler::StoreMode::MANUAL);
		if ((ret =db_handler.StoreDataNow(storeMode)) != 0)
			ROS_ERROR("Error in %s with code %d", DB_MANUAL, ret);
	}
	else if (!strcmp(message->topic, DB_AUTO_START)) {

		if(message->payload == NULL) {
			db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING);
			ROS_ERROR("got it");
		}
		else{

			try {
				double freq = std::stod((char *)message->payload);
				db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING, freq);
			}
			catch (const exception &e) {
				ROS_ERROR("Error %s in handling msg %s", e.what(), DB_AUTO_START);
			}
		}

	}
	else if (!strcmp(message->topic, DB_AUTO_START)) {

		if(message->payload == NULL) {
			db_handler.ChangeCaptureMode(DB_Handler::IMAGE_MODE);
			db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING);
			ROS_ERROR("got it");
		}
		else{

			try {
				double freq = std::stod((char *)message->payload);
				db_handler.ChangeCaptureMode(DB_Handler::IMAGE_MODE);
				db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING, freq);
			}
			catch (const exception &e) {
				ROS_ERROR("Error %s in handling msg %s", e.what(), DB_AUTO_START);
			}
		}

	}
	else if (!strcmp(message->topic, DB_AUTO_VIDEO_START)) {


		if(message->payload == NULL) {
			db_handler.ChangeCaptureMode(DB_Handler::VIDEO_MODE);
			db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING);
			ROS_ERROR("got it");
		}
		else{

			try {
				double freq = std::stod((char *)message->payload);
				db_handler.ChangeCaptureMode(DB_Handler::VIDEO_MODE);
				db_handler.ChangeRunningMode(DB_Handler::AUTO_RUNNING, freq);
			}
			catch (const exception &e) {
				ROS_ERROR("Error %s in handling msg %s", e.what(), DB_AUTO_VIDEO_START);
			}
		}

	}


	else if (!strcmp(message->topic, DB_AUTO_STOP) || !strcmp(message->topic, DB_AUTO_VIDEO_STOP)) {
		db_handler.ChangeRunningMode(DB_Handler::STOPPED);
	}
	else if (!strcmp(message->topic, DB_SEMI_AUTO_START)) {

		ROS_DEBUG("GOT it");
		if(message->payload == NULL) {
			db_handler.ChangeCaptureMode(DB_Handler::IMAGE_MODE);
			db_handler.ChangeRunningMode(DB_Handler::SEMI_AUTO_RUNNING);

		}
		else{

			try {
				double freq = std::stod((char *)message->payload);
				db_handler.ChangeCaptureMode(DB_Handler::IMAGE_MODE);
				db_handler.ChangeRunningMode(DB_Handler::SEMI_AUTO_RUNNING, freq);
			}
			catch (const exception &e) {
				ROS_ERROR("Error %s in handling msg %s", e.what(), DB_SEMI_AUTO_START);
			}
		}

	}
	else if (!strcmp(message->topic, DB_SEMI_AUTO_STOP)) {
		db_handler.ChangeRunningMode(DB_Handler::STOPPED);
	}
	else if (!strcmp(message->topic, DB_SEMI_AUTO_SNAP)) {
		int ret;
		DB_Handler::StoreMode storeMode(DB_Handler::StoreMode::SNAP);
		if ((ret=db_handler.StoreDataNow(storeMode)) != 0)
			ROS_ERROR("Error in %s with code %d", DB_SEMI_AUTO_SNAP, ret);
	}


	else if(!strcmp(message->topic, SYS_REBOOT)) {
		ROS_WARN("Going to reboot the system");
		system("reboot");
	}
	else if(!strcmp(message->topic, SYS_POWEROFF)) {
		ROS_WARN("Going to turn off the system");
		system("poweroff");
	}
	else if(!strcmp(message->topic, LIGHTS_OFF)) {


		if(message->payload == NULL) {
			ROS_INFO("Going to turn off LIGHTS");
			system("/home/kict/KICT_MP/usb-relay-hid-master/commandline/makemake/hidusb-relay-cmd off all");
			ROS_INFO("Done");
		}
		else {
			int i=0;
			try {
				i = std::stoi((const char *)message->payload);
			}
			catch(const exception & e) {
				i = 0;
			}

			ROS_INFO("Going to turn off LIGHTS %d", i);
			if(i>=1 && i<=4) {
				stringstream ss;
				ss << "/home/kict/KICT_MP/usb-relay-hid-master/commandline/makemake/hidusb-relay-cmd off " << i;
				system(ss.str().c_str());
			}
			ROS_INFO("Done");
		}



	}
	else if(!strcmp(message->topic, LIGHTS_ON)) {


		if(message->payload == NULL) {
			ROS_INFO("Going to turn ont LIGHTS");
			system("/home/kict/KICT_MP/usb-relay-hid-master/commandline/makemake/hidusb-relay-cmd on all");
			ROS_INFO("Done");
		}
		else {
			int i=0;
			try {
				i = std::stoi((const char *)message->payload);
			}
			catch(const exception & e) {
				i = 0;
			}

			ROS_INFO("Going to turn on LIGHTS %d", i);
			if(i>=1 && i<=4) {
				stringstream ss;
				ss << "/home/kict/KICT_MP/usb-relay-hid-master/commandline/makemake/hidusb-relay-cmd on " << i;
				system(ss.str().c_str());
			}
			ROS_INFO("Done");
		}




	}

	else if(!strcmp(message->topic, SPEEDO_INFO) && message->payloadlen > 0) {
		string msg((const char *)message->payload);
		struct speedo_info speedoInfo;

		vector<string> results;
		boost::split(results, msg, [](char c){return c == DeviceGroup::SEPARATOR;});

		try {
			if(results.size() < 8) {
				THROW(RobotException, "msg format error");
			}

			int *p = &speedoInfo.position[0];
			for(int i=0; i<8; ++i, ++p) {
				*p = stoi(results[i]);
			}

			db_handler.SetSpeedoData(speedoInfo);
		}
		catch (const exception & e){
			ROS_ERROR("Error: ", e.what());
		}



	}









}


