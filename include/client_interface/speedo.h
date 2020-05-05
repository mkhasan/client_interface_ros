/*
 * speedo.h
 *
 *  Created on: Feb 10, 2019
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_SPEEDO_INCLUDE_SPEEDO_SPEEDO_H_
#define CATKIN_WS_SRC_SPEEDO_INCLUDE_SPEEDO_SPEEDO_H_


#include "client_interface/client_acceptor.h"

#include <boost/scoped_ptr.hpp>
#include "client_interface/client_interface.h"
#include "client_interface/queue.h"


#include <ros/ros.h>

#include <mosquittopp.h>

#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#include <string>

#define SPEEDO_IP "192.168.0.60"
#define SPEEDO_PORT 10001
#define KNNOT2MS  0.514444444

namespace client_interface {
class CmdProcessor;
class Connector;
}

struct my_info {
	bool *quit;
	bool *send_data;
	int *sockfd;
	bool *recv_data;
	bool *start;
	ACE_Reactor* reactor;
	client_interface::queue *q;
	//client_interface::CmdProcessor *cmdProcessor;
	client_interface::queue *cmd_q;
	//std::unique_ptr<Connector> test;
	client_interface::Connector * acceptor;

};


namespace client_interface {

class CmdProcessor {
	pthread_mutex_t lock;
	std::string currCmd;
public:
	static const std::string PING;
	CmdProcessor() {
		currCmd = "";
		pthread_mutex_init(&lock, NULL);
	}
	~CmdProcessor() {
		pthread_mutex_destroy(&lock);
	}
	int GetCmd(std::string &cmd) {
		int ret=0;
		pthread_mutex_lock(&lock);
		if(currCmd.length() == 0)
			ret = -1;
		else
			cmd = currCmd;
		pthread_mutex_unlock(&lock);

		return ret;
	}
	int SetCmd(const std::string & cmd) {
		int ret=0;
		pthread_mutex_lock(&lock);
		if(currCmd.length() == 0)
			currCmd = cmd;
		else
			ret = -1;
		pthread_mutex_unlock(&lock);

		return ret;
	}
	void ClearCmd() {
		pthread_mutex_lock(&lock);
		currCmd = "";
		pthread_mutex_unlock(&lock);

	}

};




class Speedo
{
public:



	class Socket {
	public:
		int sockfd;
		Socket(int _sockfd);

		inline void Receive(char * buffer, int length) {
			int count=0;
			do {
				int size = read(sockfd, &buffer[count], 1);
				if(size <= 0)
					THROW(RobotException, "Error in receiving data");	// assume blocking call
				if(buffer[count] == NULL)
					size = 0;

				count += size;
			}
			while(count < length);
		}
	};

private:
	my_info myInfo;
	CmdProcessor cmdProcessor;
	struct sockaddr_in serverAddr;
public:

	enum{SPEEDO_MAX_DATA_SIZE=1024};
	enum{MAX_NO_PING_PERIOD_IN_SEC=5};


	ros::Publisher &pub;
	bool quit;
	pthread_t send_thread, recv_thread, connector_thread;
	bool connected;
	bool reqConnect;
	bool s;
	int sockfd;
	bool recv_data;
	bool send_data;
	bool start;

public:


	Speedo(ros::Publisher &pub);

	int Connect();
	int Disconnect();

	~Speedo();

	static bool IsSocketReady(int sockfd);
	static void GetData(char * p);
	static void SetData(const char *p);
	queue q;
	queue cmd_q;

private:
	static char globalBuffer[SPEEDO_MAX_DATA_SIZE];
public:
	std::unique_ptr<mosqpp::mosquittopp> mqttHandler;


	int CreateSocket();

public:
	void SendMsg();
	int GetRemoteClientCount() const;



};



}





#endif /* CATKIN_WS_SRC_SPEEDO_INCLUDE_SPEEDO_SPEEDO_H_ */
