/*
 * shm_manager.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: usrc
 */



#include "client_interface/shm_manager.h"

#include "ros/ros.h"

#include <sstream>
#include <semaphore.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>


using namespace std;

#define SEM_PERMS (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)
#define INITIAL_VALUE 1


/*
 * Each cam_viewer gets a slot of info based on it's id
 * launcher sets the id appropriately so that each cam_viewer process gets it's camera url and each FLIR camera gets it
 * device serial number in place of the url
 * ShmManger sets url[0] = 0 to tell cam_viewers to terminate
 * cam_viewers after terminating sets len = MAX_DATA_SIZE+1 in order to indicate that it terminated
 * After getting len > MAX_DATA_SIZE ShmManager be sure that cam_viewer left
 */


client_interface::ShmManager::ShmManager() : initialized(false), numCam(0){

	for(int i=0; i<MAX_NUM_CAMERA; i++)
		mutex[i] = NULL;
}

client_interface::ShmManager::~ShmManager() {

	ROS_ERROR("Destructor %d", initialized);
	if(initialized)
		Finalize();
}

client_interface::ShmManager * client_interface::ShmManager::Instance() {
	static ShmManager manager;
	return &manager;
}

int client_interface::ShmManager::Initialize(const vector<string> & _camUrl, const string & _mutexPrefix) {

	if(initialized)
		return 0;

	mutexPrefix = _mutexPrefix;


	// we may not need synch
	/*
	for(int i=0; i<numCam; i++) {
		if(synch[i].open (SYNC_KEY_START+i,
							  ACE_SV_Semaphore_Complex::ACE_CREATE,
							  1) == -1) {

			return ERROR_SYNCH_NOT_INITIALIZED;


	}

	*/


	camUrl = _camUrl;
	numCam = camUrl.size();

	if(numCam <= 0 || numCam > MAX_NUM_CAMERA)
		return ERROR_CAMERA_NUM_OUT_OF_RANGE;



	//vector<string> mutexName;


	string mutexName[MAX_NUM_CAMERA];
	for(int i=0; i<numCam; i++) {
		stringstream ss;
		ss << i;
		mutexName[i] = mutexPrefix+ss.str();
	    mutex[i] = sem_open(mutexName[i].c_str(), O_CREAT | O_EXCL, SEM_PERMS, INITIAL_VALUE);

	    if (mutex[i] == SEM_FAILED) {

	    	/*
	    	for(int k=0; k<numCam; k++)
	    		synch[k].remove();		// ignore the error
	    		*/

	    	ROS_ERROR("Error in initialiaing mutex %s", mutexName[i].c_str());
	    	for(int k=0; k<i; k++) {
	    		sem_close(mutex[i]);
	    		sem_unlink(mutexName[i].c_str());
	    	}

	    	return ERROR_MUTEX_NO_INITIALIZED;

	    }
	}

	debugMutex = sem_open(mutexPrefix.c_str(), O_CREAT | O_EXCL, SEM_PERMS, INITIAL_VALUE);
	if(debugMutex == SEM_FAILED) {
		ROS_ERROR("Error in initialiaing debug mutex");
		return ERROR_MUTEX_NO_INITIALIZED;
	}

	//ROS_DEBUG("I am here 1");


	char *shm[MAX_NUM_CAMERA];
	for(int i=0; i<numCam; i++) {
		shm_server[i] = ACE_Shared_Memory_SV(SHM_KEY_START+i,
									   sizeof (shm_data),
									   ACE_Shared_Memory_SV::ACE_CREATE);

		shm[i] = (char *) shm_server[i].malloc();

		if(shm[i] == NULL) {

			/*
	    	for(int k=0; k<numCam; k++)
	    		synch[k].remove();		// ignore the error
	    	*/

	    	for(int k=0; k<numCam; k++) {
	    		sem_close(mutex[i]);
	    		sem_unlink(mutexName[i].c_str());
	    	}

	    	for(int k=0; k<i; k++) {
	    		shm_server[i].remove();		// again ignore the error
	    	}
			return ERROR_SHM_NOT_INITIALIZED;
		}

	}

	for(int i=0; i<numCam; i++)
		pData[i] = new (shm[i]) shm_data;


	for(int i=0; i<numCam; i++) {
		if(sem_wait(mutex[i]) < 0)
			return ERROR_IN_ACQUIRING_MUTEX;

		//ROS_DEBUG("index %d len %d max len %d", i, camUrl[i].length(), MAX_LEN);
		if(camUrl[i].length() < MAX_LEN-1)
			strcpy(pData[i]->url, camUrl[i].c_str());
		else
			pData[i]->url[0] = 0;

		pData[i]->len = MAX_DATA_SIZE+1;		// initial value of len


		if(sem_post(mutex[i]) < 0)
			return ERROR_IN_RELEASING_MUTEX;
	}

	initialized = true;
	return 0;

}

int client_interface::ShmManager::SetUrl(int index, const char *p) {

	if(initialized == false)
		return ERROR_SHM_NOT_INITIALIZED;

	int flag = false;
	if(sem_wait(mutex[index]) < 0)
		return ERROR_IN_ACQUIRING_MUTEX;

	if (pData[index]->url[0] != NULL) {
		strncpy(pData[index]->url, p, MAX_LEN);
		flag = true;
	}

	if(sem_post(mutex[index]) < 0)
		return ERROR_IN_RELEASING_MUTEX;

	return flag ? 0 : 1;

}
int client_interface::ShmManager::Finalize() {


	ROS_DEBUG("Finalizing");
	if(initialized == false)
		return 0;

	for(int i=0; i<numCam; i++) {
		int len=0;
		while(1) {
			if(sem_wait(mutex[i]) < 0)
				return ERROR_IN_ACQUIRING_MUTEX;

			pData[i]->url[0] = NULL;
			len = pData[i]->len;

			if(sem_post(mutex[i]) < 0)
				return ERROR_IN_RELEASING_MUTEX;

			ROS_DEBUG("len is %d ", len);
			if(len > MAX_DATA_SIZE)
				break;

			//break;			// should not be here
			ros::Duration(0.5).sleep();
		}

    	shm_server[i].remove();		// again ignore the error


	}


	string mutexName[MAX_NUM_CAMERA];


	for(int i=0; i<numCam; i++) {
		stringstream ss;
		ss << i;
		mutexName[i] = mutexPrefix+ss.str();
		sem_close(mutex[i]);
		sem_unlink(mutexName[i].c_str());
	}

	sem_close(debugMutex);
	sem_unlink(mutexPrefix.c_str());
	ROS_DEBUG("Finalizing done ...");
	initialized = false;
	return 0;

}

int client_interface::ShmManager::GetData(shm_data * data) const {
	if(!initialized)
		return ERROR_SHM_NOT_INITIALIZED;

	for (int i=0; i < numCam; i++) {
		if(sem_wait(mutex[i]) < 0) {
			for (int k=0; k<i; k++)
				sem_post(mutex[k]);
			return ERROR_IN_ACQUIRING_MUTEX;
		}
	}


	for (int i=0; i<numCam; i++) {
		data[i] = *pData[i];
	}

	bool flag = false;
	for (int i=0; i<numCam; i++) {
		if(sem_post(mutex[i]) < 0)
			flag = true;
	}

	if(flag)
		return ERROR_IN_RELEASING_MUTEX;

	return 0;

}

int client_interface::ShmManager::NumCam() const {
	return numCam;
}


