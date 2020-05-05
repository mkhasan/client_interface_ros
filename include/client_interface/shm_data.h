/*
 * shm_data.h
 *
 *  Created on: Dec 6, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_DATA_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_DATA_H_


#define MAX_DATA_SIZE (1936*1464*3 + 1024)
#define MAX_LEN 255
//#define MUTEX_PREFIX "kict_mp_camera13_"

//#define _FILE_TEST
#define _DEBUG


struct shm_data{
	char url[MAX_LEN];
	char timestamp[MAX_LEN];
	int len;
	int width;
	int height;
	char data[MAX_DATA_SIZE];
};



#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_DATA_H_ */
