/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2011  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Emil Jatib Khatib  <emilkhatib@uma.es>			               |
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

#ifndef CWifiAlert_H
#define CWifiAlert_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <COpenMORAMOOSApp.h>
#include <mrpt/poses/CPose3D.h>
#include <vector>
#include <mrpt/base.h>
#include <mrpt/slam/CObservationWirelessPower.h>

//#define M_RECV "SKYPE_RECV"
//#define M_SEND "SKYPE_SEND"
#define MIN_STRENGTH 20 // Default minimum acceptable signal strength (0-100) (overwritten by the value given in the mission file)
#define LIST_LENGTH 20 // Default length of past positions (and Wifi Strength values saved) (overwritten by the value given in the mission file)

class WifiAlert : public COpenMORAApp
{
public:
    WifiAlert();
    virtual ~WifiAlert();

protected:

	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);
	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );
	/** state our interest in variables from other modules (registration for mail)*/
	bool DoRegistrations();
	
	bool goingBack;
	int list_length,min_strength;
	std::string localization;
	std::vector<std::string> pastPositions;
	std::vector<int> pastWifiPwr;
};
#endif
