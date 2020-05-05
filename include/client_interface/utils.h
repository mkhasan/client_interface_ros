/*
 * utils.h
 *
 *  Created on: Feb 17, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_UTILS_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_UTILS_H_


#include <time.h>
#include <sys/time.h>
#include <stdio.h>



#ifdef __cplusplus
extern "C" {
#endif

void print_time(char *str);

#ifdef __cplusplus
}
#endif



int StrToTime(const char *str, struct tm * tm);
int Gap(const char * str1, const char * str2);



#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_UTILS_H_ */
