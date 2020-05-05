
#include "client_interface/speedo.h"

#include "spectator/get_str.h"
#include "spectator/empty.h"

#include "std_msgs/String.h"

#include <signal.h>

#include <iostream>
#include <sstream>



using namespace std;
using namespace client_interface;

boost::shared_ptr<Speedo> _speedo;

bool connect(spectator::get_str::Request &req, spectator::get_str::Response &resp) {


	ROS_DEBUG("Got a call");
	if(_speedo->connected) {
	    resp.str = "Already connected";
	    return true;
    }

	int ret = _speedo->Connect();
	if (ret == 0) {
		resp.str = "Connected";
		return true;
	}
	else {
		stringstream ss;
		ss << "Error with code " << ret << "\n";
		resp.str = ss.str();
		return false;
	}
}


bool disconnect(spectator::empty::Request &req, spectator::empty::Response &resp) {
	_speedo->Disconnect();
	return true;
}

/*
bool is_connected(spectator::get_str::Request &req, spectator::get_str::Response &resp) {

	stringstream ss;
	ss << (_speedo->connected ? 1 : 0);
	resp.str = ss.str();

	return true;


}
*/


void mySigintHandler(int sig)
{


	_speedo->Disconnect();
	ROS_INFO("going to terminate");

	ros::shutdown();
}


int main(int argc, char * argv[]) {
	ros::init(argc, argv, "speedo_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	           ros::console::notifyLoggerLevelsChanged();
	}

	signal(SIGINT, mySigintHandler);
	signal(SIGTERM, mySigintHandler);



	ros::ServiceServer service1 = nh.advertiseService("speedo/connect", connect);
	ros::ServiceServer service2 = nh.advertiseService("speedo/disconnect", disconnect);
	//ros::ServiceServer service3 = nh.advertiseService("speedo/is_connect", is_connected);

	ros::Publisher info_pub = nh.advertise<std_msgs::String>("speedo/info", 1000);

	int rc;
	try {

		_speedo.reset(new Speedo(info_pub));


		ros::Rate loop_rate(2);

		//char buffer[Speedo::SPEEDO_MAX_DATA_SIZE];
		while(ros::ok()) {

			if(_speedo->connected) {
				_speedo->SendMsg();
			}

			int remoteCount = _speedo->GetRemoteClientCount();
			if(remoteCount == -1) {
				ROS_WARN("Error in getting remote conn info");

			}
			if(remoteCount > 0 && _speedo->connected == false)
				_speedo->Connect();
			else if (remoteCount == 0 && _speedo->connected == true)
				_speedo->Disconnect();


			rc = _speedo->mqttHandler->loop();
			if(rc)
				_speedo->mqttHandler->reconnect();

			ros::spinOnce();
			loop_rate.sleep();
		}

	}
	catch (const exception &e) {
		ROS_ERROR("Erorror is %s", e.what());
		return -1;
	}


	return 0;
}
