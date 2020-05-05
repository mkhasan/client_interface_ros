/*
 * db_handler.cpp
 *
 *  Created on: Dec 7, 2018
 *      Author: usrc
 */

#include "client_interface/db_handler.h"
#include "client_interface/shm_manager.h"
#include "client_interface/utils.h"

#include "client_interface/client_interface.h"

#include "ros/ros.h"

#include <Magick++.h>
#include <Magick++/Blob.h>

#include <cppconn/prepared_statement.h>

#include <pthread.h>
#include <exception>

#define INVALID "invalid"
using namespace std;
using namespace Magick;


static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t thread;
static bool quit = false;




string client_interface::DB_Handler::DB_NAME = "KICT_MP";
string client_interface::DB_Handler::sensorAttrib[client_interface::DB_Handler::NUM_SENSOR_ATTR] = {"imu_time_stamp"
		, "roll", "pitch", "yaw", "roll_accel", "pitch_accel", "yaw_accel"
		, "gps_time_stamp", "satellite_count", "latitude", "latitude_dir", "longitude", "longitude_dir"
		, "quality", "VOG", "COG"};
string client_interface::DB_Handler::spectatorAttrib[client_interface::DB_Handler::NUM_SPECTATOR_ATTR] = {"pan_angle", "tilt_angle"};

string client_interface::DB_Handler::speedoAttrib[client_interface::DB_Handler::NUM_SPEEDO_ATTR]
												  = {"position_0", "position_1", "position_2", "position_3"
													, "speed_0", "speed_1", "speed_2", "speed_3"};

string client_interface::DB_Handler::camNames[client_interface::DB_Handler::NUM_CAM] = {"DR_CAM", "NC_LEFT", "NC_RIGHT", "FLIR_LEFT", "FLIR_RIGHT"};

static const double EPSILON = 0.001;
const double client_interface::DB_Handler::MAX_FREQ = 1.0;

shm_data client_interface::DB_Handler::localAuto[ShmManager::MAX_NUM_CAMERA];
shm_data client_interface::DB_Handler::localManual[ShmManager::MAX_NUM_CAMERA];
shm_data client_interface::DB_Handler::localSnap[ShmManager::MAX_NUM_CAMERA];


class DataBuf : public streambuf
{
public:
   DataBuf(char * d, size_t s) {
      setg(d, d, d + s);
   }
};




client_interface::DB_Handler::DB_Handler()
	: initialized(false)
	, autoStore(false)
	, driver(NULL)
	, con(NULL)
	, res(NULL)
	, freq(MAX_FREQ)
	, state(STOP)
	, runningMode(STOPPED)
	, captureMode(IMAGE_MODE)

{

	//freq = MAX_FREQ;
	ROS_DEBUG("FREW is %lf", freq);
	gpsInfo.time_stamp = INVALID;

	spectatorInfo.pan_angle = spectatorInfo.tilt_angle = 0.0;
	int *ptr = &speedoInfo.position[0];
	for (int i=0; i<8; ++i, ++ptr)
		*ptr = 0;

	quit = false;

}

client_interface::DB_Handler::~DB_Handler() {
	Finalize();
}

int client_interface::DB_Handler::Initialize() {

	if(initialized == true)
		return 0;


//	cout << endl;
//	cout << "Running 'SELECT 'Hello World! AS _message'..." << endl;

	try {

	  /* Create a connection */
	  driver = get_driver_instance();
	  con = driver->connect("tcp://localhost:3306", "root", "Test123!");


	} catch (sql::SQLException &e) {

		stringstream ss;
		ss << "# ERR: SQLException in " << __FILE__;
	  	ss << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
	  	ss << "# ERR: " << e.what();
	  	ss << " (MySQL error code: " << e.getErrorCode();
	  	ss << ", SQLState: " << e.getSQLState() << " )" << endl;

		ROS_ERROR(ss.str().c_str());

	  return ERROR_IN_CONNECTING_MYSQL;
	}

	assert(con != NULL);
	quit = false;
    int iret = pthread_create( &thread, NULL, auto_save_func, (void *) this);

    if(iret)
    	return ERROR_IN_CREATING_THREAD;


	initialized = true;

	return EXIT_SUCCESS;

	return 0;
}


int client_interface::DB_Handler::Finalize() {

	if(initialized == false)
		return 0;

	ROS_DEBUG("In finalize");
	quit = true;
	pthread_join( thread, NULL);

	ROS_DEBUG("After Joint");
	try {


		delete res;
		delete stmt;
		delete con;

	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;

	  return ERROR_IN_FINALIZING;
	}

	cout << "finalized " << endl;

	initialized = false;

	return EXIT_SUCCESS;

}


//void client_interface::DB_Handler::ChangeRunningMode1(client_interface::DB_Handler::running_mode_t targetMode, double freq) {
//
//}
void client_interface::DB_Handler::ChangeRunningMode(client_interface::DB_Handler::running_mode_t targetMode, double _freq) {

	if(runningMode == targetMode)
		return;

	if(targetMode == STOPPED) {
		state = STOP;
		runningMode = targetMode;
		ChangeCaptureMode(IMAGE_MODE);
		return;
	}

	if(runningMode != STOPPED) {
		ROS_ERROR("DB_Handler::ChangeRunningMode: mode can be changed from stop mode only (curMode = %d, targetMode = %d)", runningMode, targetMode);
		return;
	}


	runningMode = targetMode;



	if(_freq > 0.0 && _freq <= MAX_FREQ)
		freq = _freq;


	state = START;


}

client_interface::DB_Handler::running_mode_t client_interface::DB_Handler::GetRunningMode() const {

	return runningMode;
}

void client_interface::DB_Handler::ChangeCaptureMode(capture_mode_t _captureMode) {
	captureMode = _captureMode;
}

client_interface::DB_Handler::capture_mode_t client_interface::DB_Handler::GetCaptureMode() const{
	return captureMode;
}

int client_interface::DB_Handler::SetFrequency(double _freq) {
	if (_freq < EPSILON || _freq > MAX_FREQ)
		return ERROR_FREQ_OUT_OF_RANGE;
	freq = _freq;
	return 0;
}

double client_interface::DB_Handler::GetFrequency() const {
	return freq;
}

void client_interface::DB_Handler::SetIMU_Data(const imu_info_t & _imuInfo) {
	pthread_mutex_lock(&lock);

	imuInfo = _imuInfo;

	pthread_mutex_unlock(&lock);
}

void client_interface::DB_Handler::SetGPS_Data(const my_location::gps_data::ConstPtr & _gpsInfo) {
	pthread_mutex_lock(&lock);

	gpsInfo = *_gpsInfo;

	pthread_mutex_unlock(&lock);
}

void client_interface::DB_Handler::SetSpectatorData(const spectator_info_t & _spectatorInfo) {
	pthread_mutex_lock(&lock);
	spectatorInfo = _spectatorInfo;
	pthread_mutex_unlock(&lock);


	if(spectatorInfo.snapWaitingMode == client_interface::SNAPPER_WAITING_FOR_IMAGE) {
		ChangeCaptureMode(client_interface::DB_Handler::IMAGE_MODE);
		ChangeRunningMode(client_interface::DB_Handler::SNAP_AROUND_WAITING);
	}
	else if(spectatorInfo.snapWaitingMode == client_interface::SNAPPER_WAITING_FOR_VIDEO) {
		ChangeCaptureMode(client_interface::DB_Handler::VIDEO_MODE);
		ChangeRunningMode(client_interface::DB_Handler::SNAP_AROUND_WAITING);
	}
	else {
		if(GetRunningMode() == client_interface::DB_Handler::SNAP_AROUND_WAITING) {
			ChangeRunningMode(client_interface::DB_Handler::STOPPED);
		}
	}





}

void client_interface::DB_Handler::SetSpeedoData(const speedo_info_t & _speedoInfo) {
	pthread_mutex_lock(&lock);
	speedoInfo = _speedoInfo;
	pthread_mutex_unlock(&lock);
}

/*
void client_interface::DB_Handler::GetSensorData(xsens_msgs::orientationEstimate & _imuData, my_location::gps_data & _gpsData) const {
	pthread_mutex_lock(&lock);

	_imuData = imuData;
	_gpsData = gpsData;



	pthread_mutex_unlock(&lock);
}

*/

void client_interface::DB_Handler::GetData(imu_info_t & _imuInfo, my_location::gps_data & _gpsInfo
		, spectator_info_t & _spectatorInfo, speedo_info_t & _speedoInfo ) const {
	pthread_mutex_lock(&lock);

	_imuInfo = imuInfo;
	_gpsInfo = gpsInfo;
	_spectatorInfo = spectatorInfo;
	_speedoInfo = speedoInfo;


	pthread_mutex_unlock(&lock);
}

int client_interface::DB_Handler::StoreDataNow(const StoreMode & storeMode) {
	ROS_DEBUG("MY cur mode %d", runningMode);


	if(runningMode != SEMI_AUTO_RUNNING && storeMode.mode == StoreMode::MANUAL)
		return StoreData(storeMode);

	if(runningMode == SEMI_AUTO_RUNNING && storeMode.mode == StoreMode::SNAP)
		return StoreData(storeMode);

	ROS_ERROR("DB_Handler::StoreDataNow: mode not set properly target operation is %d", storeMode.mode);

	return -1;
}

int client_interface::DB_Handler::StoreNonVisualData(const StoreMode & storeMode) {
	if(initialized == false)
		return ERROR_DB_NOT_INITIALIZED;

	//ROS_ERROR("storeMode.mode is %d", storeMode.mode);
	int ret=EXIT_SUCCESS;

	imu_info_t _imuInfo;
	my_location::gps_data _gpsInfo;
	spectator_info_t _spectatorInfo;
	speedo_info_t _speedoInfo;

	GetData(_imuInfo, _gpsInfo, _spectatorInfo, _speedoInfo);


	string ins = "INSERT INTO real_time_data (auto_or_manual" ;

	for(int i=0; i<NUM_SENSOR_ATTR; i++) {
		ins += ",";
		ins += sensorAttrib[i];
	}

	for(int i=0; i<NUM_SPECTATOR_ATTR; i++) {
		ins += ",";
		ins += spectatorAttrib[i];
	}

	for(int i=0; i<NUM_SPEEDO_ATTR; i++) {
		ins += ",";
		ins += speedoAttrib[i];
	}

	ins += ") VALUES (?";

	for(int i=0; i<NUM_SENSOR_ATTR; i++) {
		ins += ",";
		ins += "?";
	}

	for(int i=0; i<NUM_SPECTATOR_ATTR; i++) {
		ins += ",";
		ins += "?";
	}

	for(int i=0; i<NUM_SPEEDO_ATTR; i++) {
		ins += ",";
		ins += "?";
	}

	ins += ")";

	ROS_DEBUG("ins is %s", ins.c_str());

	try {



		con->setSchema(DB_NAME);

		auto prep_stmt = (std::unique_ptr<sql::PreparedStatement>) con->prepareStatement(ins);

		PrepareStmtNonImageData(storeMode.mode, _imuInfo, _gpsInfo, _spectatorInfo, _speedoInfo, prep_stmt);

		prep_stmt->execute();


	} catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;

		ret = EXIT_FAILURE;
	}


	return ret;



}




int client_interface::DB_Handler::StoreData(const StoreMode & storeMode) {

	if(initialized == false)
		return ERROR_DB_NOT_INITIALIZED;



	Blob jpegBlob[ShmManager::MAX_NUM_CAMERA];
	string debugInfo[ShmManager::MAX_NUM_CAMERA];
	shm_data *localData;

	switch(storeMode.mode) {
		case StoreMode::AUTO:
		case StoreMode::SNAP_AROUND:		// AUTO_RUNNING AND SNAP_AROUND_WAITING are mutually exclusive
			localData = &localAuto[0];
			break;
		case StoreMode::MANUAL:
			localData = &localManual[0];
			break;
		case StoreMode::SNAP:
			localData = &localSnap[0];
			break;
		default:
			localData = NULL;
	}

	if (localData == NULL) {
		ROS_ERROR("client_interface::DB_Handler::StoreData: storeMode (%d) not selected properly", storeMode);
		return -1;
	}

	//static istream str1[ShmManager::MAX_NUM_CAMERA];
	//std::unique_ptr<istream> p;
	vector<istream*> streamList;
	vector<DataBuf *> bufferList;
	vector<int> camList;


	imu_info_t _imuInfo;
	my_location::gps_data _gpsInfo;
	spectator_info_t _spectatorInfo;
	speedo_info_t _speedoInfo;

	GetData(_imuInfo, _gpsInfo, _spectatorInfo, _speedoInfo);

	ShmManager * pManager = ShmManager::Instance();
	int numCam = pManager->NumCam();




	///////////
	//int imageLen;
	//char* imageData = ImageData("/home/hasan/KICT_MP/cam_viewer/build_linux/10:50:27_112581.ppm", imageLen);

	//ROS_DEBUG("imageLen is %d", imageLen);

	////////////
	int i;
	int ret;
	char curTime[MAX_LEN];

	try {

		if((ret=pManager->GetData(localData)) != 0)
			return ret;

		for (i=0; i<numCam; i++) {

			if(localData[i].len < MAX_DATA_SIZE) {
				Blob blob(localData[i].data, localData[i].len);
				//Blob blob(imageData, imageLen);
				ROS_DEBUG("cam %d imageLen is %d", i, localData[i].len);



				if(localData[i].len) {
					Image image(blob);
					image.write( &jpegBlob[i], "jpg" );
					ROS_DEBUG("image %d: widht %d height %d", i, localData[i].width, localData[i].height);
				}
				debugInfo[i] = localData[i].timestamp;
				if(i == 0)
					ROS_DEBUG("DB_Handler::StoreData: jpeg len is %d debug info %s", jpegBlob[0].length(), debugInfo[0].c_str());




				print_time(curTime);

				ROS_DEBUG("curtime is %s timestamp is %s", curTime, localData[i].timestamp);

				if(strlen(curTime) == 0 || strlen(localData[i].timestamp) == 0) {
					if (strlen(curTime) == 0)
						ROS_WARN("DB_Handler::StoreData: curTime is not set for camera %d", i);
					else {
						ROS_WARN("DB_Handler::StoreData: timestamp is not set for camera %d", i);	// camera i not working

					}

					continue;
				}

				ret = Gap(curTime, localData[i].timestamp);
				if (ret < 0) {
					ROS_WARN("DB_Handler::StoreData: Time Gap is negative");
					ret += 24*3600;
				}

				if( ret < TIME_THRESH_IN_SEC) {
					camList.push_back(i);
				}
				else {
					ROS_WARN("DB_Handler::StoreData: Time Gap is too large");

				}




			}

			//DataBuf buffer((char*)jpegBlob[i].data(), jpegBlob[i].length());

			//boost::shared_ptr<istream> p;
			//p.reset(new istream(&buffer));
			//streamList.push_back(p);

			//streamList.push_back(make_unique<istream>(&buffer));
			//p = make_unique<istream>(&buffer);
			//static istream stream(&buffer);

			/*
			ROS_DEBUG("localData %d length is %d", i, localData.len);
			*/

		}
	}
	catch( const exception &error_ )
	{
	  //cout << "Caught exception: " << error_.what() << endl;
		ROS_ERROR("DB_Handler::StoreData: %s", error_.what());

		return ERROR_IN_JPEG_CONVERSION;
	}


	string ins = "INSERT INTO real_time_data (auto_or_manual" ;
	for(int i=0; i<NUM_SENSOR_ATTR; i++) {
		ins += ",";
		ins += sensorAttrib[i];
	}

	for(int i=0; i<NUM_SPECTATOR_ATTR; i++) {
		ins += ",";
		ins += spectatorAttrib[i];
	}

	for(int i=0; i<NUM_SPEEDO_ATTR; i++) {
		ins += ",";
		ins += speedoAttrib[i];
	}


	for(vector<int>::const_iterator it = camList.begin(); it != camList.end(); ++it) {
		ins += ",";
		ins += camNames[*it];
	}


	for(vector<int>::const_iterator it = camList.begin(); it != camList.end(); ++it) {
		ins += ",";
		ins += (camNames[*it]+"_DEBUG_INFO");
	}

	ins += ") VALUES (?";
	for(int i=0; i<NUM_SENSOR_ATTR; i++) {
		ins += ",";
		ins += "?";
	}

	for(int i=0; i<NUM_SPECTATOR_ATTR; i++) {
		ins += ",";
		ins += "?";
	}

	for(int i=0; i<NUM_SPEEDO_ATTR; i++) {
		ins += ",";
		ins += "?";
	}


	for(int i=0; i<camList.size(); i++) {
		ins += ",";
		ins += "?";
	}


	for(int i=0; i<camList.size(); i++) {		// for debug info
		ins += ",";
		ins += "?";
	}




	ins += ")";

	ROS_DEBUG("ins is %s", ins.c_str());

	try {



		con->setSchema(DB_NAME);

		auto prep_stmt = (std::unique_ptr<sql::PreparedStatement>) con->prepareStatement(ins);

		auto index = PrepareStmtNonImageData(storeMode.mode, _imuInfo, _gpsInfo, _spectatorInfo, _speedoInfo, prep_stmt);


		int k=index;
		//const int NUM_PRE_IMAGE_ATTR = (NUM_SENSOR_ATTR+NUM_SPECTATOR_ATTR+NUM_SPEEDO_ATTR);
		for(vector<int>::const_iterator it = camList.begin() ; it != camList.end(); ++it) {

			DataBuf *pBuf = new DataBuf((char*)jpegBlob[*it].data(), jpegBlob[*it].length());
			istream * p = new istream(pBuf);

			streamList.push_back(p);
			bufferList.push_back(pBuf);

			// Worning: the position must follow the relative position in ins string (starting from 1) not the positon in the actual table //
			prep_stmt->setBlob(k++, p);


			ROS_DEBUG("length is %d ", jpegBlob[*it].length());


		}


		for(vector<int>::const_iterator it = camList.begin(); it != camList.end(); ++it) {
			//string str = "test";
			prep_stmt->setString(k++, debugInfo[*it]);	// one for entry_no one for auto_or_manual

			ROS_DEBUG("(camList.size() %d)debug info is %s", camList.size(), debugInfo[*it].c_str());


		}



		prep_stmt->execute();


	} catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;

		ret = EXIT_FAILURE;
	}

	//for(int i=0; i<streamList.size(); i++)
		//delete streamList[i];

	//for(auto& p: streamList)
		//delete *p;

	for (auto& p:streamList) {
		//ROS_DEBUG("deleting stream");
		delete p;
	}

	for (auto& p:bufferList) {
		//ROS_DEBUG("deleting stream");
		delete p;
	}


	return 0;

	//return ShmManager::Instance()->GetData()

}

int client_interface::DB_Handler::PrepareStmtNonImageData(int _auto_or_manual, const imu_info_t & _imuInfo
		, const my_location::gps_data & _gpsInfo, const spectator_info_t & _spectatorInfo
		, const speedo_info_t & _speedoInfo, std::unique_ptr<sql::PreparedStatement> & prep_stmt) const {


	int k=1;
	prep_stmt->setInt(k++, _auto_or_manual);		// start from 1 not 0

	prep_stmt->setString(k++, imuInfo.time_stamp);
	prep_stmt->setDouble(k++, _imuInfo.roll);
	prep_stmt->setDouble(k++, _imuInfo.pitch);
	prep_stmt->setDouble(k++, _imuInfo.yaw);
	prep_stmt->setDouble(k++, _imuInfo.roll_accel);
	prep_stmt->setDouble(k++, _imuInfo.pitch_accel);
	prep_stmt->setDouble(k++, _imuInfo.yaw_accel);


	prep_stmt->setString(k++, _gpsInfo.time_stamp);
	if(string(_gpsInfo.time_stamp) != INVALID)	{
		prep_stmt->setInt(k++, _gpsInfo.satellite_count);
		prep_stmt->setString(k++, _gpsInfo.latitude);
		prep_stmt->setString(k++, string(1,_gpsInfo.latitude_dir));		// string(size_t n, char c) fill the string with n char each of which is 'c'
		prep_stmt->setString(k++,_gpsInfo.longitude);
		prep_stmt->setString(k++, string(1,_gpsInfo.longitude_dir));
		prep_stmt->setInt(k++, _gpsInfo.quality);
		prep_stmt->setDouble(k++, _gpsInfo.vog);
		prep_stmt->setDouble(k++, _gpsInfo.cog);
	}
	else {
		prep_stmt->setInt(k++, 0);
		prep_stmt->setString(k++, "0");
		prep_stmt->setString(k++, "X");
		prep_stmt->setString(k++, "0");
		prep_stmt->setString(k++, "X");
		prep_stmt->setInt(k++, 0);
		prep_stmt->setDouble(k++, 0.0);
		prep_stmt->setDouble(k++, 0.0);
	}

	prep_stmt->setDouble(k++, _spectatorInfo.pan_angle);
	prep_stmt->setDouble(k++, _spectatorInfo.tilt_angle);


	const int *ptr = &_speedoInfo.position[0];

	for (int i=0; i< NUM_SPEEDO_ATTR; ++i, ++ptr) {
		prep_stmt->setInt(k++, *ptr);
	}

	return k;



}


#include <fstream>

char * client_interface::DB_Handler::ImageData (const string & filename, int &length) {
	std::ifstream is (filename, std::ifstream::binary);
	if (is) {
		// get length of file:
		is.seekg (0, is.end);
		length = is.tellg();
		is.seekg (0, is.beg);

		// allocate memory:
		char * buffer = new char [length];

		// read data as a block:
		is.read (buffer,length);

		is.close();

		// print content:
		//std::cout.write (buffer,length);

		return buffer;
		//delete[] buffer;
	}

	return NULL;


}

void * client_interface::DB_Handler::auto_save_func( void *ptr ) {


	client_interface::DB_Handler *pDB = (client_interface::DB_Handler *) ptr;

	unsigned int periodUs = (unsigned int) ((1000000.0/pDB->freq) + EPSILON);
	ROS_DEBUG("period is %d freq is %f", periodUs, pDB->freq);
	ACE_Time_Value twakeup, tinc, tsleep, t1;

	while(quit == false) {
		//ROS_DEBUG("In loop");
		if(pDB->state == START) {
			periodUs = (unsigned int) ((1000000.0/pDB->freq) + EPSILON);

		    ROS_DEBUG("period is %d", periodUs);
			tinc.set(0, periodUs);
		    twakeup = ACE_OS::gettimeofday();
		}

		while(quit == false && pDB->state == START) {
			twakeup += tinc;

			//ROS_ERROR("captre mode is %d", pDB->captureMode);
			try {
				if((pDB->runningMode == AUTO_RUNNING || pDB->runningMode == SNAP_AROUND_WAITING) && pDB->captureMode == IMAGE_MODE) {
					StoreMode storeMode(pDB->runningMode == AUTO_RUNNING ? StoreMode::AUTO : StoreMode::SNAP_AROUND);
					pDB->StoreData(storeMode);

				}
				else if (pDB->runningMode == SEMI_AUTO_RUNNING || (pDB->captureMode == VIDEO_MODE)) {

					StoreMode storeMode(StoreMode::NONE);
					switch (pDB->runningMode) {
						case SEMI_AUTO_RUNNING:
							storeMode = StoreMode::SEMI_AUTO;
							break;
						case AUTO_RUNNING:
							storeMode = StoreMode::VIDEO_AUTO;
							break;
						case SNAP_AROUND_WAITING:
							storeMode = StoreMode::VIDEO_SNAP_AROUND;
							break;
						default:
							storeMode = StoreMode::NONE;
					}
					//ROS_ERROR("Store mode is %d", storeMode);
					pDB->StoreNonVisualData(storeMode);
				}
				else {
					THROW(RobotException, "mode not set properly");
				}
			}
			catch(const exception & e) {
				ROS_ERROR("Error: %s", e.what());
			}

			t1 = ACE_OS::gettimeofday();
			tsleep= twakeup - t1;
			//  cout<< "tsleep : "<<tsleep<<" twakeup : "<<twakeup<<" t1 :"<<t1<<endl;
			if (tsleep > ACE_Time_Value::zero) {
				  ACE_OS::sleep(tsleep);
				  //ROS_INFO("sleept");
			}
			else
				ROS_DEBUG("NoSleep");

		}

		ACE_OS::sleep(ACE_Time_Value(0,200000));	// 200 ms


	}

}

