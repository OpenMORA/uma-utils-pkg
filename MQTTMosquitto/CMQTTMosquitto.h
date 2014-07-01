/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef CMQTTMosquitto_H
#define CMQTTMosquitto_H

#include "Constants.h"
#include <COpenMORAMOOSApp.h>
#include <mosquitto.h>
#include <mosquittopp.h>

/*#include <mrpt/hwdrivers/CActivMediaRobotBase.h>

#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/base.h>

#include "CGiraffCommunication.h"
#include "CGiraffMotorsCom.h"*/
using namespace std;
using namespace mosqpp;

class CMQTTMosquitto :public COpenMORAApp,mosquittopp
{
		/** Class for storing an Hokuyo observation. */
	
public:
	CMQTTMosquitto();
    virtual ~CMQTTMosquitto();
	/////////////////// CHokuyoObservation ///////////////////
	bool saving_data;
	double lin_vel,ang_vel;	
protected:
	//const char *client_id;
	//const char *broker_host;
	std::string broker_host;
	int broker_port;
	std::string broker_username, broker_password;
    /** called at startup */
    virtual bool OnStartUp();
    /** called when new mail arrives */
    virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
    /** called when work is to be done */
    virtual bool Iterate();
    /** called when app connects to DB */
    virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

    /** performs the registration for mail */
    bool DoRegistrations();

private:
	void tlsOptionsSet();
	void on_connect(int rc);
	void on_message(const struct mosquitto_message *message);
	void on_subscribe(uint16_t mid, int qos_count, const uint8_t *granted_qos);
	void on_publish(int *mid, const char *topic, int payloadlen, const void *payload);

	mrpt::system::TTimeStamp timeStamp_last_loc, timeStamp_last_laser;
	bool sendLaserScans, enable_TLS_security;
	float laserScanResolution, laserScanRate, localizationRate;
	std::string TLS_path, TLS_CAfile_pem, TLS_CERTfile_pem, TLS_KEYfile_pem;
	bool is_motion_command_set;
	mrpt::system::TTimeStamp time_last_motion_command;
};

#endif
