/*
 * dr_robot.h
 *
 *  Created on: Feb 21, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DR_ROBOT_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DR_ROBOT_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include <memory>
#include <assert.h>
#include <cstring>
#include <iomanip>
#include <string.h>


#define USE_NONE_FOR_EMPTY_STR(X) (strlen(X) == 0 ? "None":X )
namespace client_interface {

class Dr_Robot_Component {
public:


	inline void ZeroMemory(void *ptr, size_t num) {
		memset(ptr, 0, num);
	}



};

class Dr_motor: Dr_Robot_Component{
public:
	int position[4]; //LFM, RFM, LRM, RRM
	int speed[4];
	float temperature[4];
	float pwm[4];
	float current[4];
	char driverstate[2][64];
	float voltage;

public:
	Dr_motor(){
		ZeroMemory(position, sizeof(position));
		ZeroMemory(speed, sizeof(speed));
		ZeroMemory(temperature, sizeof(temperature));
		ZeroMemory(pwm, sizeof(pwm));
		ZeroMemory(current, sizeof(current));
		ZeroMemory(driverstate, sizeof(driverstate));
		voltage = 99.0;
	}
	inline char * GetDriverState(int no){ return driverstate[no]; }
	inline int GetPosition(int no){ return position[no]; }
	inline int GetSpeed(int no){ return speed[no]; }
	inline float GetTemperature(int no){ return temperature[no]; }
	inline float GetPWM(int no){ return pwm[no]; }
	inline float GetCurrent(int no){ return current[no]; }
	inline float GetVoltage(){ return voltage; }
};

class Dr_gps : Dr_Robot_Component{
public:
	char state[16];
	char timestamp[16];
	char latitude[16];
	char longuitude[16];
	float cog;
	float vog;

public:
	Dr_gps(){
		ZeroMemory(state, sizeof(state));
		ZeroMemory(timestamp, sizeof(timestamp));
		ZeroMemory(latitude, sizeof(latitude));
		ZeroMemory(longuitude, sizeof(longuitude));
		cog = 0.0;
		vog = 0.0;
	}

	inline char *GetGpsState(){ return state; }
	inline char *GetTimeStamp(){ return timestamp; }
	inline char *GetLatitude(){ return latitude; }
	inline char *GetLonguitude(){ return longuitude; }
	inline char GetCog(){ return cog; }
	inline char GetVog(){ return vog; }
};

class Dr_imu : Dr_Robot_Component{
public:
	int seq;
	float estyaw;
	int gyro[3];
	int accel[3];
	int compass[3];

public:
	Dr_imu(){
		seq = 0;
		estyaw = 0;
		ZeroMemory(gyro, sizeof(gyro));
		ZeroMemory(accel, sizeof(accel));
		ZeroMemory(compass, sizeof(compass));
	}

	inline int GetSeq(){ return seq; }
	inline float GetEstYaw(){ return estyaw; }
	inline int GetGyro(int no){ return gyro[no]; }
	inline int GetAccel(int no){ return accel[no]; }
	inline int GetCompass(int no){ return compass[no]; }
};

class DeviceGroup {
public:
	static std::string deviceGrpStr;
	static void Get(std::string & deviceGrpStr);
	static void Set(const DeviceGroup & deviceGroup);
	static pthread_mutex_t lock;

private:
	const int FLOAT_PRECISION = 4;
public:
	static const char SEPARATOR = '|';


public:

	Dr_motor dr_motor;
	Dr_gps dr_gps;
	Dr_imu dr_imu;

	inline std::string ToString() const{
		std::stringstream ss;


		for (int i=0; i<4; i++) {
			ss << dr_motor.position[i] << SEPARATOR;
		}

		for (int i=0; i<4; i++) {
			ss << dr_motor.speed[i] << SEPARATOR;

		}

		for (int i=0; i<4; i++) {
			ss << std::setprecision(FLOAT_PRECISION) << dr_motor.temperature[i] << SEPARATOR;

		}

		for (int i=0; i<4; i++) {
			ss << std::setprecision(FLOAT_PRECISION) << dr_motor.pwm[i] << SEPARATOR;

		}

		for (int i=0; i<4; i++) {
			ss << std::setprecision(FLOAT_PRECISION) << dr_motor.current[i] << SEPARATOR;

		}

		ss << USE_NONE_FOR_EMPTY_STR(dr_motor.driverstate[0])  << SEPARATOR
				<< USE_NONE_FOR_EMPTY_STR(dr_motor.driverstate[1])
				<< SEPARATOR << std::setprecision(FLOAT_PRECISION) << dr_motor.voltage << SEPARATOR;

		ss << USE_NONE_FOR_EMPTY_STR(dr_gps.state) << SEPARATOR
				<< USE_NONE_FOR_EMPTY_STR(dr_gps.timestamp)<< SEPARATOR
				<< USE_NONE_FOR_EMPTY_STR(dr_gps.latitude) << SEPARATOR
				<< USE_NONE_FOR_EMPTY_STR(dr_gps.longuitude) << SEPARATOR;

		ss << std::setprecision(FLOAT_PRECISION) << dr_gps.cog << SEPARATOR << dr_gps.vog << SEPARATOR;



		ss << dr_imu.seq << SEPARATOR << std::setprecision(FLOAT_PRECISION) << dr_imu.estyaw << SEPARATOR;

		for (int i=0; i<3; i++) {
			ss << dr_imu.gyro[i] << SEPARATOR;

		}

		for (int i=0; i<3; i++) {
			ss << dr_imu.accel[i] << SEPARATOR;

		}

		for (int i=0; i<3; i++) {
			ss << dr_imu.compass[i];
			if(i < 2)
				ss << SEPARATOR;
		}

		return ss.str();


	}




	DeviceGroup() {
		DeviceGroup::Set(*this);

	}


};


}

#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DR_ROBOT_H_ */
