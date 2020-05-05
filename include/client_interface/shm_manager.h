/*
 * shm_manager.h
 *
 *  Created on: Dec 5, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_MANAGER_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_MANAGER_H_

#include "shm_data.h"
//#include "client_interface/db_handler.h"
#include <ace/SV_Semaphore_Complex.h>
#include <ace/Shared_Memory_SV.h>

#include <string>
#include <vector>

#include <semaphore.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>

#define DEFAULT_KEY 4500
#define BASE 10


namespace client_interface {


class ShmManager {

public:

	enum{MAX_NUM_CAMERA=5};
	enum {ERROR_CAMERA_NUM_OUT_OF_RANGE = -1, ERROR_SYNCH_NOT_INITIALIZED = -2, ERROR_MUTEX_NO_INITIALIZED = -3,
			ERROR_SHM_NOT_INITIALIZED = -4, ERROR_IN_ACQUIRING_MUTEX =-5, ERROR_IN_RELEASING_MUTEX = -6 };



	static const int SHM_KEY_START = DEFAULT_KEY;
	static const int SYNC_KEY_START = DEFAULT_KEY+10*BASE;
	//static const std::string MUTEX_PREFIX;


private:
	bool initialized;

	std::vector<std::string> camUrl;
	int numCam;
	std::string mutexPrefix;

	//ACE_SV_Semaphore_Complex synch[MAX_NUM_CAMERA];
	ACE_Shared_Memory_SV shm_server[MAX_NUM_CAMERA];
	sem_t *mutex[MAX_NUM_CAMERA];
	sem_t *debugMutex;

	struct shm_data *pData[MAX_NUM_CAMERA];
	ShmManager();
	~ShmManager();
	ShmManager(ShmManager & copy);
	ShmManager & operator=(const ShmManager & copy);


public:
	static ShmManager * Instance();
	int Initialize(const std::vector<std::string> & camUrl, const std::string & mutexPrefix);
	int Finalize();
	//int GetData(int index, shm_data &data) const;
	int GetData(shm_data* data) const;
	int NumCam() const;
	int SetUrl(int index, const char *p);
	bool IsInitialized() {
		return initialized;
	}


};


}		// end of namespace


#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_SHM_MANAGER_H_ */
