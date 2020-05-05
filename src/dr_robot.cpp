/*
 * dr_robot.cpp
 *
 *  Created on: Feb 26, 2019
 *      Author: kict
 */


#include "client_interface/dr_robot.h"

using namespace std;

namespace client_interface {


pthread_mutex_t DeviceGroup::lock = PTHREAD_MUTEX_INITIALIZER;
string DeviceGroup::deviceGrpStr;


void DeviceGroup::Get(string & _deviceGrpStr) {
	pthread_mutex_lock(&lock);

	_deviceGrpStr = DeviceGroup::deviceGrpStr;

	pthread_mutex_unlock(&lock);

}

void DeviceGroup::Set(const DeviceGroup & _deviceGroup) {


	pthread_mutex_lock(&lock);

	DeviceGroup::deviceGrpStr = _deviceGroup.ToString();

	pthread_mutex_unlock(&lock);

}


}

