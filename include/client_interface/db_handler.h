/*
 * db_handler.h
 *
 *  Created on: Dec 7, 2018
 *      Author: usrc
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DB_HANDLER_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DB_HANDLER_H_

#include "client_interface/shm_manager.h"
#include "xsens_msgs/orientationEstimate.h"
#include "my_location/gps_data.h"



#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

#include <string>
#include <vector>


namespace client_interface{

typedef enum {SNAPPER_NOT_WATING, SNAPPER_WAITING_FOR_IMAGE, SNAPPER_WAITING_FOR_VIDEO} snapper_waiting_mode_t;

}

namespace client_interface {



typedef struct speedo_info {
	int position[4];
	int speed[4];
} __attribute__((packed)) speedo_info_t;

typedef struct spectator_info {
	double pan_angle;
	double tilt_angle;
	snapper_waiting_mode_t snapWaitingMode;


} spectator_info_t;

typedef struct imu_info {
	std::string time_stamp;
	double roll;
	double pitch;
	double yaw;
	double roll_accel;
	double pitch_accel;
	double yaw_accel;

} imu_info_t;


class DB_Handler {

	const int TIME_THRESH_IN_SEC = 5;
	bool initialized;
	double freq;
	bool autoStore;

	sql::Driver *driver;
	sql::Connection *con;
	sql::Statement *stmt;
	sql::ResultSet *res;


	//xsens_msgs::orientationEstimate imuData;
	//my_location::gps_data gpsData;


	imu_info_t imuInfo;
	my_location::gps_data gpsInfo;
	spectator_info_t spectatorInfo;
	speedo_info_t speedoInfo;



	static shm_data localAuto[ShmManager::MAX_NUM_CAMERA];
	static shm_data localManual[ShmManager::MAX_NUM_CAMERA];
	static shm_data localSnap[ShmManager::MAX_NUM_CAMERA];




public:
	enum{NUM_SENSOR_ATTR = 16};
	enum{NUM_SPECTATOR_ATTR = 2};
	enum{NUM_SPEEDO_ATTR = 8};
	enum{NUM_CAM = ShmManager::MAX_NUM_CAMERA};

	typedef enum{START, STOP, RESTART} state_t;

	state_t state;
	static std::string DB_NAME;
	static std::string sensorAttrib[NUM_SENSOR_ATTR];
	static std::string spectatorAttrib[NUM_SPECTATOR_ATTR];
	static std::string speedoAttrib[NUM_SPEEDO_ATTR];
	static std::string camNames[NUM_CAM];

	static const double MAX_FREQ;	// in Hz
	enum {ERROR_FREQ_OUT_OF_RANGE = -1, ERROR_IN_CONNECTING_MYSQL = -2, ERROR_IN_FINALIZING = -3,
		ERROR_DB_NOT_INITIALIZED = -4, ERROR_IN_JPEG_CONVERSION = -5, ERROR_IN_CREATING_THREAD = -6};

	class StoreMode {
	public:
		typedef enum {MANUAL=0, AUTO=1, SEMI_AUTO=2, SNAP=3, SNAP_AROUND=4, VIDEO_AUTO, VIDEO_SNAP_AROUND, NONE} mode_t;
		mode_t mode;
		StoreMode(mode_t _mode) : mode(_mode) {
		}
	};

	typedef enum{AUTO_RUNNING=0, SEMI_AUTO_RUNNING=1, SNAP_AROUND_WAITING=2, STOPPED=3} running_mode_t;
	typedef enum{IMAGE_MODE, VIDEO_MODE} capture_mode_t;



	DB_Handler();
	~DB_Handler();
	int Initialize();
	int Finalize();



	void ChangeRunningMode(running_mode_t targetMode, double freq = -1.0);
	//void ChangeRunningMode1(running_mode_t targetMode, double freq = -1.0);
	running_mode_t GetRunningMode() const;

	void ChangeCaptureMode(capture_mode_t captureMode);
	capture_mode_t GetCaptureMode() const;

	int SetFrequency(double freq);
	double GetFrequency() const;
	void SetIMU_Data(const imu_info_t & imuInfo);
	void SetGPS_Data(const my_location::gps_data::ConstPtr & gpsInfo);
	void SetSpectatorData(const spectator_info_t & spectatorInfo);
	void SetSpeedoData(const speedo_info_t & speedoInfo);

private:
	//void GetSensorData(xsens_msgs::orientationEstimate & imuData, my_location::gps_data & gpsData) const;

	void GetData(imu_info_t & imuInfo, my_location::gps_data & gpsInfo
			, spectator_info_t & spectatorInfo, speedo_info_t & speedoInfo ) const;

	int PrepareStmtNonImageData(int _auto_or_manual, const imu_info_t & imuInfo
			, const my_location::gps_data & gpsInfo, const spectator_info_t & spectatorInfo
			, const speedo_info_t & speedoInfo, std::unique_ptr<sql::PreparedStatement> & prep_stmt) const;

public:
	int StoreDataNow(const StoreMode & storeMode);	// to remove confustion between DB_Handler::mode vs Store mode where formar indicates if
													// mainly running mode like just once or continuously and
													// and the later will appear in the db
private:
	int StoreNonVisualData(const StoreMode & storeMode);

	int StoreData(const StoreMode & storeMode);

	char * ImageData (const std::string & filename, int &length);

private:
	running_mode_t runningMode;
	capture_mode_t captureMode;
	static void * auto_save_func( void *ptr );


};


}	// end of namespace


#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_DB_HANDLER_H_ */
