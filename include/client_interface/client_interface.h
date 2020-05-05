/*
 * client_interface.h
 *
 *  Created on: Nov 27, 2018
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_INTERFACE_H_
#define CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_INTERFACE_H_


//#include <spectator/cmd_handler.h>


#include "client_interface/db_handler.h"
#include "client_interface/robot_exception.h"

#include <mosquittopp.h>


#define RIGHT_STICK "KICT_MP/CLIENT/RIGHT_STICK"
#define RIGHT_BUTTON_GROUP "KICT_MP/CLIENT/RIGHT_BUTTON_GROUP"
#define SPECTATOR_POSITION "KICT_MP/SPECTATOR/POSITION"

#define SPECTATOR_GET_ANGLES "KICT_MP/CLIENT/GET_ANGLES"
#define SPECTATOR_GET_ANGLES_START "KICT_MP/CLIENT/GET_ANGLES_START"
#define SPECTATOR_GET_ANGLES_STOP "KICT_MP/CLIENT/GET_ANGLES_STOP"

#define SPECTATOR_HOMING "KICT_MP/CLIENT/HOMING"
#define SPECTATOR_GOTO_ABS "KICT_MP/CLIENT/GOTO_ABS"
#define SPECTATOR_GOTO_PAN_ABS "KICT_MP/CLIENT/GOTO_PAN_ABS"
#define SPECTATOR_GOTO_TILT_ABS "KICT_MP/CLIENT/GOTO_TILT_ABS"

#define DB_MANUAL "KICT_MP/CLIENT/MANUAL_SAVE"

#define DB_AUTO_START "KICT_MP/CLIENT/AUTO_SAVE_START"
#define DB_AUTO_STOP "KICT_MP/CLIENT/AUTO_SAVE_STOP"

#define DB_AUTO_VIDEO_START "KICT_MP/CLIENT/AUTO_VIDEO_START"
#define DB_AUTO_VIDEO_STOP "KICT_MP/CLIENT/AUTO_VIDEO_STOP"


#define DB_SEMI_AUTO_START "KICT_MP/CLIENT/SEMI_AUTO_START"
#define DB_SEMI_AUTO_STOP "KICT_MP/CLIENT/SEMI_AUTO_STOP"
#define DB_SEMI_AUTO_SNAP "KICT_MP/CLIENT/SEMI_AUTO_SNAP"



#define IMU_START "KICT_MP/CLIENT/IMU_START"
#define IMU_STOP "KICT_MP/CLIENT/IMU_STOP"
#define IMU_VALUES "KICT_MP/IMU"

#define GPS_START "KICT_MP/CLIENT/GPS_START"
#define GPS_STOP "KICT_MP/CLIENT/GPS_STOP"
#define GPS_VALUES "KICT_MP/GPS"

#define SYS_REBOOT "KICT_MP/CLIENT/REBOOT"
#define SYS_POWEROFF "KICT_MP/CLIENT/POWEROFF"

#define LIGHTS_ON "KICT_MP/CLIENT/LIGHTS_ON"
#define LIGHTS_OFF "KICT_MP/CLIENT/LIGHTS_OFF"

#define SPEEDO_INFO "KICT_MP/SPEEDO_INFO"
#define SPEEDO_TEST "KICT_MP/SPEEDO_TEST"
#define SPEEDO_PING "KICT_MP/CLIENT/PING"
#define SPEEDO_CONNECTED "KICT_MP/SPEEOD_CONNECTED"
#define SPEEDO_DISCONNECTED "KICT_MP/SPEEDO_DISCONNECTED"
#define SPEEDO_IS_CONNECTED "KICT_MP/CLIENT/SPEEDO_ISCONNECTED"

#define SPECTATOR_SNAP_AROUND "KICT_MP/CLIENT/SPECTATOR_SNAP_AROUND"
#define SPECTATOR_SNAPPING_ABORT "KICT_MP/CLIENT/SPECTATOR_SNAPPING_ABORT" // will also abort videoing around (see next)

#define SPECTATOR_SNAPPING_DONE "KICT_MP/SPECTATOR/SNAPPING_DONE"	// will also indicate that videoing around is done

#define SPECTATOR_VIDEO_AROUND "KICT_MP/CLIENT/SPECTATOR_VIDEO_AROUND"





#define SPEEDO_CONNECTED_MSG "Speedo connected"
#define SPEEDO_DISCONNECTED_MSG "Speedo disconnected"




namespace client_interface {


class ClientInterface : public mosqpp::mosquittopp
{

public:

	class Quaterniond {

		double w, x, y, z;
	public:
		Quaterniond(double _w, double _x, double _y, double _z) {
			w = _w;
			x = _x;
			y = _y;
			z = _z;
		}

		double W() const{
			return w;
		}

		double X() const {
			return x;
		}

		double Y() const {
			return y;

		}

		double Z() const {
			return z;
		}

		static inline void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw) {
			double sinr_cosp = +2.0 * (q.W() * q.X() + q.Y() * q.Z());
			double cosr_cosp = +1.0 - 2.0 * (q.X() * q.X() + q.Y() * q.Y());
			roll = atan2(sinr_cosp, cosr_cosp);

			// pitch (y-axis rotation)
			double sinp = +2.0 * (q.W() * q.Y() - q.Z() * q.X());
			if (fabs(sinp) >= 1) {
				auto copysing([](double val, double s) {return (s > 0.0 ?  val : -1*val);});
				pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			}
			else
				pitch = asin(sinp);

			// yaw (z-axis rotation)
			double siny_cosp = +2.0 * (q.W() * q.Z() + q.X() * q.Y());
			double cosy_cosp = +1.0 - 2.0 * (q.Y() * q.Y() + q.Z() * q.Z());
			yaw = atan2(siny_cosp, cosy_cosp);
		}
		void toEulerAngle(double& roll, double& pitch, double& yaw) {
			toEulerAngle(*this, roll, pitch, yaw);
		}

	};


private:
	DB_Handler &db_handler;

public:
	bool publishAngle;
	bool publishIMU;
	bool publishGPS;
	bool pingReceived;
	bool speedoConnected;
public:
	ClientInterface(DB_Handler & db_handler, const char *id, const char *host, int port);
	~ClientInterface();




	void on_connect(int rc);
	void on_message(const struct mosquitto_message *message);

private:
	void SetPingReceived() {
		pingReceived = true;
	}

public:
	void ResetPingReceived(){
		pingReceived = false;
	}




};


}




#endif /* CATKIN_WS_SRC_CLIENT_INTERFACE_INCLUDE_CLIENT_INTERFACE_CLIENT_INTERFACE_H_ */
