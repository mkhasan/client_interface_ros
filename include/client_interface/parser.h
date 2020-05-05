/*
 * parser.h
 *
 *  Created on: Feb 25, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_PARSER_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_PARSER_H_

#include "client_interface/dr_robot.h"

namespace client_interface {

const int MAX_INDEX = 512;
const int MIN_INDEX = 3;


void CheckData(char selection);
void parse(DeviceGroup & devices, char * buffer, int index);
void ParseDevice(DeviceGroup & devices, char * buffer, const char *);
float ad2Temperature(int adValue);

void *watcher(void *);

}

#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_PARSER_H_ */
