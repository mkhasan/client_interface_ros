/*
 * client_interface_node.cpp
 *
 *  Created on: Nov 27, 2018
 *      Author: kict
 */



#include "client_interface/shm_manager.h"
#include "client_interface/client_interface.h"
#include "client_interface/db_handler.h"
#include "client_interface/speedo.h"

#include "xsens_msgs/orientationEstimate.h"
#include "my_location/gps_data.h"
#include "spectator/set_val.h"
#include "spectator/get_str.h"
#include "spectator/empty.h"
#include "spectator/get_str_list.h"

#include "spectator/info.h"



#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <XmlRpcValue.h>

#include <sstream>
#include <signal.h>
#include <string>


using namespace std;
using namespace client_interface;


boost::shared_ptr<ClientInterface> clientInterface;
boost::shared_ptr<DB_Handler> db_handler;
vector<string> urlList;
#include <boost/algorithm/string.hpp>

static const int FREQUENCY = 20.0;


void SpectatorCallback(const spectator::info::ConstPtr & msg) {

	spectator_info_t spectatorInfo;
	static bool snapperWasActive = false;

	spectatorInfo.pan_angle = msg->panAngle;
	spectatorInfo.tilt_angle = msg->tiltAngle;
	spectatorInfo.snapWaitingMode = snapper_waiting_mode_t(msg->snapperWaitingMode);

	db_handler->SetSpectatorData(spectatorInfo);

	bool snapperIsActive = (msg->snapperIsActive == 1);

	if (snapperWasActive && !snapperIsActive)
		clientInterface->publish(NULL, SPECTATOR_SNAPPING_DONE, 0, NULL);

	snapperWasActive = snapperIsActive;
	//ROS_INFO("pan Angle is %f tilt angle is %f", spectatorInfo.pan_angle, spectatorInfo.tilt_angle);

	//string angles =
	char angles[256];
	sprintf(angles, "%.4f %.4f", msg->panAngle, msg->tiltAngle);
	if(clientInterface->publishAngle)
		clientInterface->publish(NULL, SPECTATOR_POSITION, strlen(angles), angles);

}


void IMU_FilterCallback(const xsens_msgs::orientationEstimate::ConstPtr & msg) {
	//ROS_INFO("I heard : [%f %f %f]", msg->roll, msg->pitch, msg->yaw);


	/*
	db_handler->SetIMU_Data(msg);
	static string mqttMsg;
	if(clientInterface->publishIMU) {
		stringstream ss;
		ss << msg->roll << " " << msg->pitch << " " << msg->yaw;
		mqttMsg = ss.str();
		clientInterface->publish(NULL, IMU_VALUES, mqttMsg.length(), mqttMsg.c_str());
	}
	*/
}


void IMU_Callback(const sensor_msgs::Imu::ConstPtr & msg) {


	//ROS_DEBUG("IN IMU_Callback");
	ClientInterface::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	double roll, pitch, yaw;
	auto radToDeg ([](double r) {return r*180.0/M_PI;});

	q.toEulerAngle(roll, pitch, yaw);
	roll = radToDeg(roll);
	pitch = radToDeg(pitch);
	yaw = radToDeg(yaw);

	//ROS_INFO("IMU values [%f, %f, %f]:", roll, pitch, yaw);

	ros::Time stamp = msg->header.stamp;
	stringstream ss;
	ss << stamp.sec << ":" << stamp.nsec;
	imu_info_t imuInfo;
	imuInfo.time_stamp = ss.str();
	imuInfo.roll = roll;
	imuInfo.pitch = pitch;
	imuInfo.yaw = yaw;
	imuInfo.roll_accel = msg->linear_acceleration.x;
	imuInfo.pitch_accel = msg->linear_acceleration.y;
	imuInfo.yaw_accel = msg->linear_acceleration.z;

	db_handler->SetIMU_Data(imuInfo);

	static string mqttMsg;
	if(clientInterface->publishIMU) {
		stringstream ss;
		ss << imuInfo.time_stamp << " " << imuInfo.roll << " " << imuInfo.pitch << " " << imuInfo.yaw << " "
				<< imuInfo.roll_accel << " " << imuInfo.pitch_accel << " " << imuInfo.yaw_accel;
		mqttMsg = ss.str();
		clientInterface->publish(NULL, IMU_VALUES, mqttMsg.length(), mqttMsg.c_str());
	}



	/*
	Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	double roll, pitch, yaw;
	toEulerAngle(q, roll, pitch, yaw);

	auto radToDeg ([](double r) {return r*180.0/M_PI;});


	std_msgs::Header test = msg->header;
	ros::Time t=test.stamp;

	ROS_INFO("(%d:%d) using toEulerAngle : [%f, %f, %f]", t.sec, t.nsec, radToDeg(roll), radToDeg(pitch), radToDeg(yaw) );
*/
}


void GPS_Callback(const my_location::gps_data::ConstPtr & msg) {
	//ROS_INFO("I heard : [%f %f %f]", msg->roll, msg->pitch, msg->yaw);
	db_handler->SetGPS_Data(msg);
	static string mqttMsg;
	if(clientInterface->publishGPS) {
		stringstream ss;
		//ss << msg->roll << " " << msg->pitch << " " << msg->yaw;
		ss << msg->time_stamp << " " << msg->satellite_count << " "
				<< msg->latitude << " " << msg->latitude_dir << " "
				<< msg->longitude << " " << msg->longitude_dir << " " << msg->quality << " "
				<< msg->vog << " " << msg->cog;

		mqttMsg = ss.str();


		clientInterface->publish(NULL, GPS_VALUES, mqttMsg.length(), mqttMsg.c_str());
	}

}

void SpeedoInfoCallback(const std_msgs::String::ConstPtr & msg) {
	//ROS_INFO("I heard : [%s]", msg->data.c_str());

	if(msg->data == string(SPEEDO_CONNECTED_MSG)) {
		ROS_INFO("Speedo connected");
		clientInterface->speedoConnected = true;
		clientInterface->publish(NULL, SPEEDO_CONNECTED, 0, NULL);
	}
	else if(msg->data == string(SPEEDO_DISCONNECTED_MSG)) {
		ROS_INFO("Speedo disconnected");
		clientInterface->speedoConnected = false;

	}
	else
		clientInterface->publish(NULL, SPEEDO_INFO, msg->data.length(), msg->data.c_str());


}

bool pause(spectator::set_val::Request &req, spectator::set_val::Response &resp) {

	//ROS_DEBUG("Got a call");
	//resp.str = "connected";
	ShmManager * pManager = ShmManager::Instance();
	try {
		int index = stoi(req.name);
		if(index >=0 && index < urlList.size()) {
			if(req.val == 1) {
				const char str[2] = {0x10, NULL};
				//db_handler->

				if (pManager->SetUrl(index, str) == 0)
					return true;
			}
			else
				pManager->SetUrl(index, urlList[index].c_str());

		}
		else
			return false;
	}
	catch (const exception & e) {
		return false;
	}
	return true;
}


bool GetCameraNames(spectator::get_str_list::Request &req, spectator::get_str_list::Response &resp) {


	//ROS_DEBUG("%s %f", req.joint_name.c_str(), req.target_value);

	vector<string> names;
	//names.push_back("hello");
	//names.push_back("hi");

	int camNum = DB_Handler::NUM_CAM;
	for (int i=0; i<camNum; i++ )
		names.push_back(DB_Handler::camNames[i]);

	resp.list = names;

	return true;

}

bool IsShmInitialized(spectator::get_str::Request & req, spectator::get_str::Response & resp) {

	if (ShmManager::Instance()->IsInitialized())
		resp.str = string("true");
	else
		resp.str = string("false");
	return true;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // All the default sigint handler does is call shutdown()

	ShmManager::Instance()->Finalize();
	ROS_INFO("going to terminate");
	db_handler->Finalize();



	mosqpp::lib_cleanup();


	ros::shutdown();
	//exit(1);
}

/*
void ParseParams(ros::NodeHandle n) {
  XmlRpc::XmlRpcValue filters;
  n.getParam("filter", filters);
  ROS_ASSERT(filters.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = filters.begin(); it != filters.end(); ++it) {
    ROS_INFO_STREAM("Found filter: " << (std::string)(it->first) << " ==> " << filters[it->first]);
    // Do stuff with filters:
    ROS_INFO_STREAM("filter_cascade_1: in: " << filters[it->first]["in"]);
    ROS_INFO_STREAM("filter_cascade_1: filter: first_applied_filter: config_for_filter_1: " << filters[it->first]["filter"]["first_applied_filter"][0]);
  }
}

*/

/*
 * list:
 - id: 0
   name: john
   pos: [0,0,0]

 - id: 1
   name: anna
   pos: [0,0,0]

   ---

   for (int i = 0; i < list.size(); i++) {
      XmlRpc::XmlRpcValue sublist = list[i];
      id=sublist["id"];
}


void ParseParams(ros::NodeHandle n) {

	//list<XmlRpc::XmlRpcValue> camera_info;
	//vector<XmlRpc::XmlRpcValue> test;
	XmlRpc::XmlRpcValue camera_info;
	n.getParam("camera_info", camera_info);
	ROS_DEBUG("%d", camera_info.size());
	//n.getParam("camera_info", camera_info);
	for (int i = 0; i < camera_info.size(); i++) {
	      XmlRpc::XmlRpcValue sublist = camera_info[i];
	      ROS_DEBUG_STREAM("Found info: " << sublist["name"]);
	      //id=sublist["id"];
	}
}
*/


int main(int argc, char **argv) {
	ros::init(argc, argv, "client_interface_node", ros::init_options::NoSigintHandler);

	signal(SIGTERM, mySigintHandler);
	signal(SIGINT, mySigintHandler);          // caught in a different way fo$
	signal(SIGHUP, mySigintHandler);
	signal(SIGKILL, mySigintHandler);
	signal(SIGTSTP, mySigintHandler);


	ros::NodeHandle nh;


	// must be inserted in the same order as appeared in the Camera Names in the DB_Handler

	urlList.push_back("rtsp://192.168.0.65/axis-media/media.amp?videocodec=h264");
	urlList.push_back("rtsp://admin:1234@192.168.0.200:554/video1");
	urlList.push_back("rtsp://admin:1234@192.168.0.100:554/video1");

	urlList.push_back("19504536");
	urlList.push_back("19504535");


	/*
		vector<string> urlList;
		urlList.push_back("rtsp://192.168.0.65/axis-media/media.amp?videocodec=h264");

		urlList.push_back("rtsp://admin:1234@192.168.0.100:554/video1s1");
		urlList.push_back("rtsp://admin:1234@192.168.0.101:554/video1s1");

		urlList.push_back("rtsp://192.168.0.10:554/user=admin_password=_channel=1_stream=1.sdp?real_stream");
		urlList.push_back("rtsp://192.168.0.11:554/user=admin_password=_channel=1_stream=1.sdp?real_stream");

		urlList.push_back("rtsp://192.168.0.31:554/user=admin_password=admin/Streaming/Channels/2");
		urlList.push_back("rtsp://192.168.0.41:554/user=admin_password=admin/Streaming/Channels/2");

		urlList.push_back("rtsp://192.168.0.30:554/user=admin_password=admin/Streaming/Channels/2");
		urlList.push_back("rtsp://192.168.0.40:554/user=admin_password=admin/Streaming/Channels/2");

	*/



	ros::Rate loop_rate(FREQUENCY);
	//ros::Subscriber cur_angle_sub = nh.subscribe("spectator/info", 1000, SpectatorCallback);
	ros::Subscriber cur_angle_sub = nh.subscribe("spectator/info", 1000, SpectatorCallback);
	//ros::Subscriber imu_filter_sub = nh.subscribe("/mti/filter/orientation", 1000, IMU_FilterCallback);
	ros::Subscriber imu = nh.subscribe("/mti/sensor/imu", 200, IMU_Callback);
	ros::Subscriber gps_sub = nh.subscribe("/my_location", 1000, GPS_Callback);
	//ros::Subscriber speedo_info_sub = nh.subscribe("/speedo/info", 1000, SpeedoInfoCallback);
	ros::ServiceServer service1 = nh.advertiseService("client_interface/pause", pause);
	ros::ServiceServer service2 = nh.advertiseService("client_interface/get_camera_names", GetCameraNames);

	ros::ServiceServer shm_initialized = nh.advertiseService("client_interface/is_shm_initialized", IsShmInitialized);


	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	mosqpp::lib_init();

	string mutexPrefix;
	nh.param("/ROBOT/MUTEX_PREFIX", mutexPrefix, string("kict_mp_camera00_"));

	ROS_ERROR("MUTEX PREFIX IS %s", mutexPrefix.c_str());


	//ParseParams(nh);

	int rc;








	ShmManager *pShm = ShmManager::Instance();

	if ((rc=pShm->Initialize(urlList, mutexPrefix)) != 0) {
		ROS_ERROR("Error in initializing shared mem (error code %d)", rc);
		exit(1);
	}

	db_handler.reset(new DB_Handler);
	rc = db_handler->Initialize();

	if(rc != 0) {
		ROS_ERROR("Error in initializing DB handler (error code %d)", rc);
	}


	clientInterface.reset(new ClientInterface(*db_handler, "client_interface_node", "127.0.0.1", 1883));



	const int MAX_PING_SKIP_COUNT = Speedo::MAX_NO_PING_PERIOD_IN_SEC*FREQUENCY;

	ROS_DEBUG("Max ping skip is %d", MAX_PING_SKIP_COUNT);

	int pingSkipCount = 0;
	ros::ServiceClient speedoConnect = nh.serviceClient<spectator::get_str>("speedo/connect");
	ros::ServiceClient speedoDisconnect = nh.serviceClient<spectator::empty>("speedo/disconnect");

	while (ros::ok()) {

		rc = clientInterface->loop();
		if(rc)
			clientInterface->reconnect();

		if(clientInterface->speedoConnected) {

			if (clientInterface->pingReceived == true) {
				pingSkipCount = 0;
				clientInterface->ResetPingReceived();
			}
			else {
				pingSkipCount++;
			}

			if(pingSkipCount > MAX_PING_SKIP_COUNT) {
				/////////////// calling a service to disconnect Speedo ///////////



				//////////////////
				pingSkipCount = 0;
			}
		}
		else
			pingSkipCount = 0;

		ros::spinOnce();
		loop_rate.sleep();


	}

	ROS_INFO("going to terminate");

	mosqpp::lib_cleanup();
	return 0;
}


