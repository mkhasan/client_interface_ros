#include "ros/ros.h"
#include "std_msgs/String.h"


#include <sys/types.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>




#include "client_interface/client_interface.h"
#include "client_interface/speedo.h"
#include "client_interface/parser.h"

//#include "client_interface/client_handler.h"

#include <string>

using namespace std;






static pthread_mutex_t mutex;




namespace client_interface {

void *receive_location_data( void *ptr );
void *send_to_speedo(void *ptr);
void *client_connector(void * ptr);

char Speedo::globalBuffer[SPEEDO_MAX_DATA_SIZE];

const string CmdProcessor::PING = "PING\r\n";
Speedo::Speedo(ros::Publisher &_pub)
	: pub(_pub)
	, quit(false)
	, send_thread(NULL)
	, connector_thread(NULL)
	, connected(false)
	, reqConnect(false)
	, send_data(false)
	, sockfd(-1)
	, recv_data(false)

{


	//static char addr[MAX_LEN];


	//client_interface_node", "127.0.0.1", 1883)"

	mosqpp::lib_init();

	mqttHandler = make_unique<mosqpp::mosquittopp>("speedo");

	mqttHandler->connect("127.0.0.1", 1883, 60);

	myInfo.quit = &quit;
	myInfo.send_data = &send_data;
	myInfo.sockfd = &sockfd;
	myInfo.recv_data = &recv_data;
	myInfo.start = &start;
	myInfo.reactor = NULL;
	myInfo.q = &q;
	myInfo.cmd_q = &cmd_q;
	myInfo.acceptor = NULL;





    int  iret;

    pthread_mutex_init(&mutex, NULL);

    quit = false;

    char error[MAX_LEN];

    iret = pthread_create( &connector_thread, NULL, client_connector, (void *) &myInfo);

	if(iret) {


		 sprintf(error,  "Error - pthread_create() for ping return code: %d",iret);

		 THROW(RobotException, error);
	}

    iret = pthread_create( &send_thread, NULL, send_to_speedo, (void *) &myInfo);

	if(iret) {


		 sprintf(error,  "Error - pthread_create() for ping return code: %d",iret);

		 THROW(RobotException, error);
	}

	iret = pthread_create( &recv_thread, NULL, receive_location_data, (void *) &myInfo);

	if(iret) {
		 sprintf(error, "Error - pthread_create() for recv return code: %d",iret);

		 THROW(RobotException, error);

	}


	//Connect();
	return;

}


Speedo::~Speedo() {


	Disconnect();

	quit = true;
	if(myInfo.reactor != NULL)
		myInfo.reactor->close();

	pthread_join(send_thread, NULL);
	pthread_join(recv_thread, NULL);
	pthread_join(connector_thread, NULL);



	mqttHandler->disconnect();
	mosqpp::lib_cleanup();
	if(sockfd != -1)
		close(sockfd);



	pthread_mutex_destroy(&mutex);
}

bool Speedo::IsSocketReady(int sockfd)
{

    /// Got here because iSelectReturn > 0 thus data available on at least one descriptor
    // Is our socket in the return list of readable sockets
    bool             res;
    fd_set          sready;
    struct timeval  nowait;

    FD_ZERO(&sready);
    FD_SET((unsigned int)sockfd,&sready);
    //bzero((char *)&nowait,sizeof(nowait));
    memset((char *)&nowait,0,sizeof(nowait));

    res = select(sockfd+1,&sready,NULL,NULL,&nowait);
    if( FD_ISSET(sockfd,&sready) )
        res = true;
    else
        res = false;


    return res;

}

int Speedo::CreateSocket(){
	struct hostent * server;
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		THROW(RobotException, "Error in creating socket");
	}

	server = gethostbyname(SPEEDO_IP);

	if (server == NULL) {
		THROW(RobotException, "Server not found");
	}



	bzero((char *) &serverAddr, sizeof(serverAddr));

	serverAddr.sin_family = AF_INET;
	bcopy((char *)server->h_addr,
		  (char *)&serverAddr.sin_addr.s_addr, server->h_length);
	serverAddr.sin_port = htons(SPEEDO_PORT);

	return sockfd;

}

int Speedo::Connect() {

	if(connected == false) {
		try {
			sockfd = CreateSocket();
		}
		catch (const exception& e) {
			ROS_ERROR("%s", e.what());
			return -1;
		}
		if (connect(sockfd, (const struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
			ROS_ERROR("Speedo::Connect: Failed to connect");
			return -1;
		}

		q.initialize();
		cmd_q.initialize();
		send_data = true;
		recv_data = true;
		start = true;
		connected = true;
		if(myInfo.acceptor != NULL)
			myInfo.acceptor->speedo_connected = true;
		//cmdProcessor.ClearCmd();
	}

	//std_msgs::String msg;
	//msg.data = string(SPEEDO_CONNECTED_MSG);
	//pub.publish(msg);

	//pub.publish(string(SPEEDO_CONNECTED_MSG));
	ROS_INFO("Connected");
	return 0;
}

int Speedo::Disconnect() {

	if(connected == true) {
		send_data = false;
		recv_data = false;
		q.finalize();
		cmd_q.finalize();
		usleep(100000);
		close(sockfd);
		sockfd = -1;
		connected = false;
		if(myInfo.acceptor != NULL)
			myInfo.acceptor->speedo_connected = false;

	}

	//std_msgs::String msg;
	//msg.data = string(SPEEDO_DISCONNECTED_MSG);
	//pub.publish(msg);

	//ROS_INFO("Disconnected");

	return 0;
}

void Speedo::SendMsg() {
	static string msg;

	/*
	if(myInfo.acceptor != NULL) {
		const vector<string> list = myInfo.acceptor->GetRemoteList();
		for (vector<string>::const_iterator it = list.begin(); it !=list.end(); ++it ) {
			if(it == list.begin())
				msg += *it;
			else {
				msg += " | ";
				msg += *it;
			}

		}
	}
	*/
	//mqttHandler->publish(NULL, SPEEDO_TEST, msg.length(), msg.c_str());

	DeviceGroup::Get(msg);
	mqttHandler->publish(NULL, SPEEDO_INFO, msg.length(), msg.c_str());

}

int Speedo::GetRemoteClientCount() const {
	if(myInfo.acceptor == NULL)
		return -1;
	return myInfo.acceptor->RemoteConnectionCount();
}

void *send_to_speedo( void *ptr ) {


	pthread_t tID;

	my_info * myInfo = (my_info *) ptr;

	const char *pingMsg = CmdProcessor::PING.c_str();

	string currMsg;
	int ret;

	const int SIZE = (myInfo->cmd_q)->get_size();
	static vector<char> data;
	data.resize(SIZE);

	queue *cmd_q = myInfo->cmd_q;
	static char msg[255];
	while(*myInfo->quit == false) {

	    if(*myInfo->send_data) {

			int len = cmd_q->remove(&data[0]);
			int k;


	    	if(len > 0) {

	    		/*
				vector<char> temp;

				for(k=0; k<len; k++)
					temp.push_back(data[k]);

				temp.push_back(0);

				if(len != 6) {

					ROS_ERROR("CMD is %s size is %d", &temp[0], len);
				}

				*/
	    		int sent = write(*(myInfo->sockfd), &data[0], len);
				if(sent != len)
					ROS_ERROR("send_ping: Error in sending data to speedo");
	    	}

	    	/*

	    	else {
				int n = write(*(myInfo->sockfd), pingMsg, strlen(pingMsg));

				if (n != strlen(pingMsg)) {
					ROS_ERROR("receive_location_data: error in send ping msg");
				}
	    	}
	    	*/

	    }

		//ROS_DEBUG("wOR");
		usleep(200000);



	}

	return NULL;
}



Speedo::Socket::Socket(int _sockfd) : sockfd(_sockfd) {
	int flags = fcntl(sockfd, F_GETFL, 0);
	flags &= ~O_NONBLOCK;
	fcntl(sockfd, F_SETFL, flags);// | O_NONBLOCK);

}



void * receive_location_data(void *ptr) {


    my_info * myInfo = (my_info *) ptr;

    int written, i, length, flags;

    const int sleepUs = 5000;

    char buffer[Speedo::SPEEDO_MAX_DATA_SIZE];
    memset(buffer, 0, sizeof(buffer));


	char recvbuf[Speedo::SPEEDO_MAX_DATA_SIZE];
	memset(recvbuf, 0, sizeof(recvbuf));

	flags = fcntl(*(myInfo->sockfd), F_GETFL, 0);
	flags &= ~O_NONBLOCK;
	fcntl(*(myInfo->sockfd), F_SETFL);//, flags | O_NONBLOCK);

	const int LEN = 32;

	bool *start = myInfo->start;
	char prev = 0;
	char curr = 0;
	int index = 0;

	char temp[1];

	static DeviceGroup devices;

	flags = fcntl(*(myInfo->sockfd), F_GETFL, 0);
	flags &= ~O_NONBLOCK;
	fcntl(*(myInfo->sockfd), F_SETFL);//, flags | O_NONBLOCK);


	static int count=0;
	char test1='x', test2='x', test3='x', test4 = 'x';
    while (*(myInfo->quit) == false ) {

    	if(*(myInfo->recv_data)) {

    		try {



    			if(*start == false) {
    				prev = curr = 0;
    				index = 0;
    			}

				length = read(*(myInfo->sockfd), (char *)temp, 1);

				/*
				if(length) {
					if(test1 == 'F' && test2 == 'F' && test3 == '=' ) {
						ROS_ERROR("(%c%c)", test4, temp[0]);
					}
					test1 = test2;
					test2 = test3;
					test3 = test4;
					test4 = temp[0];
				}
				*/

				//usleep(1000);

				/*
				if(length < 0) {
					ROS_ERROR("receive_location_data: Error in reading data from speedo");
				}
				else if(length) {
					;//myInfo->q->add(buffer, length);
				}
				*/

				if(*start == false && temp[0] != '#')
					continue;
				*start = true;
				if(temp[0] == NULL)
					continue;
				buffer[index] = curr = temp[0];

				if(prev == '\r' && curr == '\n') {
					parse(devices, buffer, index);
					index = 0;
				}
				else
					index++;

				prev = curr;
				if(index > MAX_INDEX) {
					buffer[index] = 0;
					ROS_ERROR("receive_location_data: Getting abnornal input from speedo %s", buffer);
					index = 0;
				}

    		}
    		catch(const exception & e) {
    			ROS_ERROR("Error: %s", e.what());
    		}



    		/*
			if (1) {//client_interface::Speedo::IsSocketReady(*(myInfo->sockfd))) {
				//written = pBuffer->Receive(sockfd);

				flags = fcntl(*(myInfo->sockfd), F_GETFL, 0);
				flags &= ~O_NONBLOCK;
				fcntl(*(myInfo->sockfd), F_SETFL, flags);// | O_NONBLOCK);



				length = read(*(myInfo->sockfd), (char*)buffer, 1);//Speedo::SPEEDO_MAX_DATA_SIZE-1);

				Speedo::SetData(buffer);
				ROS_DEBUG("Data len is %d data first byte is %c", length, buffer[0]);
				usleep(1000);

			}
			*/

    		count ++;
    		const int skip = 500;
    		if (count % skip == skip-1) {
    			ROS_WARN("%s - %s", devices.dr_motor.driverstate[0], devices.dr_motor.driverstate[1] );
    			DeviceGroup::Set(devices);

    		}


    	}
    	else
    		usleep(10000);

    }

    return NULL;


}

void * client_connector(void * ptr) {

    my_info * myInfo = (my_info *) ptr;

    const int PORT = 5000;
	//ACE_Reactor* reactor = myInfo->reactor;

    ACE_Reactor reactor;

	static Connector peer_acceptor(myInfo->q, myInfo->cmd_q);

	myInfo->acceptor = &peer_acceptor;

	if (peer_acceptor.open (ACE_INET_Addr (PORT),
						  &reactor) == -1)
	ACE_ERROR_RETURN ((LM_ERROR,
					   "%p\n",
					   "open"),
					  NULL);

	//ACE_Sig_Action sa ((ACE_SignalHandler) handler, SIGINT);


	ACE_DEBUG ((LM_DEBUG,
			  "(%P|%t) starting up server daemon on port %d\n", PORT));

    assert(myInfo->reactor == NULL);
    myInfo->reactor = &reactor;

	while (!(*myInfo->quit))
		reactor.handle_events ();

	ACE_DEBUG ((LM_DEBUG,
			  "(%P|%t) shutting down server daemon\n"));

	return NULL;


}

void Speedo::GetData(char *p) {
	pthread_mutex_lock(&mutex);
	strcpy(p, globalBuffer);
	pthread_mutex_unlock(&mutex);
}

void Speedo::SetData(const char *p) {
	pthread_mutex_lock(&mutex);
	strcpy(globalBuffer, p);
	pthread_mutex_unlock(&mutex);
}



}

