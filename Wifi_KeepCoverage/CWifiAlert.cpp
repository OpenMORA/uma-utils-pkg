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

/**  @moos_module Module to avoid a mobile robot to lost wifi connectivity.
  *  This module registers the latest values of the wifi signal strength (see module Wifi_Monitor) and in the case that it falls under a certain value, 
  *  it returns to the place that has the higher wifi strength in its list.
  *
  *  The objective is to avoid the robot to lost connectivity permanently, recovering it by commanding the robot to move to a know location with wifi coverage.
*/


/**  @moos_ToDo
  *  
  */

#include "CWifiAlert.h"
#include <string.h>
#include <fstream>
#include <sstream>
#include <mrpt/obs/CObservationWirelessPower.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;

WifiAlert::WifiAlert()
{

	goingBack = false; // true if the signal is below threshold and the robot is searching for a good signal again
}

WifiAlert::~WifiAlert()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool WifiAlert::OnStartUp()
{
	//!  @moos_param   min_strength   Power threshold of the wifi to consider coverage lost
	if(!(m_MissionReader.GetConfigurationParam("min_strength",min_strength))){
		std::cout << "Using default value for min_strength: " << MIN_STRENGTH << std::endl;
		min_strength = MIN_STRENGTH;
	}
	//!  @moos_param   list_length   Mumber of pairs <location-WifiPower> that will be kept in record
	if(!m_MissionReader.GetConfigurationParam( "list_length", list_length )){
		std::cout << "Using default value for list_length: " << LIST_LENGTH << std::endl;
		list_length = LIST_LENGTH;
	}

	// fill with zeros the list of locations and power readings
	int i;
	std::string dummy;
	for (i=0;i<list_length;i++){
		pastPositions.push_back(dummy);
		pastWifiPwr.push_back(0);
	}
	std::cout << "min_strength = " << min_strength << "\nlist_length = " << list_length << std::endl;
	return DoRegistrations();
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool WifiAlert::OnCommandMsg( CMOOSMsg Msg )
{
	return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool WifiAlert::Iterate()
{
	//std::cout << "Iterate()" << std::endl;
	//int power;
	//power = wifi.GetPower();
//	std::cout << "Iterate() end" << std::endl;
	//m_Comms.Notify("WIFI_POWER",(double)power);
	/*mrpt::poses::CPose3D robotPose3D;
	if (!goingBack){
		// If the robot is not moving back
		if ( power >= min_strength ){
			// The power is above threshold. Save the information
			// Get actual position



			CMOOSVariable * pVarLoc = GetMOOSVar( "LOCALIZATION" );
			if(pVarLoc && pVarLoc->IsFresh()){
				robotPose3D.fromString( pVarLoc->GetStringVal() );
			} else {
				MOOSTrace("CPose3D Not available \n");
			}

			// Save the value, and delete the oldest:
			pastPositions.pop_back();
			pastPositions.insert(pastPositions.begin(),robotPose3D);
			pastWifiPwr.pop_back();
			pastWifiPwr.insert(pastWifiPwr.begin(),power);
			std::vector<int>::iterator iter;
			for (iter = pastWifiPwr.begin();iter != pastWifiPwr.end();iter++){
				std::cout << *iter;
			}
			std::cout << std::endl;
		} else {
			// The strength is below the threshold. An action will be taken (by the moment, verbally notify the situation and go back to the last place where the signal was good enough
			goingBack = true;
			m_Comms.Notify("SAY","NO TENGO BUENA COBERTURA WIFI, VUELVO A UN PUNTO MEJOR");


			robotPose3D = *(pastPositions.begin());
			m_Comms.Notify("NAVIGATE_TARGET", mrpt::utils::format("[%.03f %.03f]", robotPose3D.x(),  robotPose3D.y()) );
		}
	} else {
		// The robot was moving back, but it is already in a zone with good strength
		if(power > min_strength) goingBack = false;
	}*/
	return true;
}




//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool WifiAlert::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool WifiAlert::DoRegistrations()
{
	//! @moos_subscribe WIFI_POWER_SER
	//! @moos_var WIFI_POWER_SER The wifi signal strenght as a CObservationWirelessPower observation
    this->m_Comms.Register("WIFI_POWER_SER",0);

	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN",0);

	//! @moos_subscribe LOCALIZATION
	//! @moos_var LOCALIZATION The Robot localization as (x y phi) triplet.
	AddMOOSVariable("LOCALIZATION","LOCALIZATION","LOCALIZATION",0.1);

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool WifiAlert::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	std::cout << "OnNewMail()" << std::endl;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		//MOOSTrace(format("New msg received: %s \n",i->GetKey()));
		if( i->GetName() == "WIFI_POWER_SER" )
		{
			// read the object
			CSerializablePtr obj;
			StringToObject(i->GetString(),obj);
			if (obj && IS_CLASS(obj,CObservationWirelessPower))
			{
				CObservationWirelessPowerPtr obj_wifi = CObservationWirelessPowerPtr(obj);
							
				// Get the reading
				int power;
				power = floor(obj_wifi->power);
				std::string robotPose3D;
				if (!goingBack){
					// If the robot is not moving back
					if ( power >= min_strength ){
						// The power is above threshold. Save the information

						// Save the position, and delete the oldest:
						pastPositions.pop_back();
						pastPositions.insert(pastPositions.begin(),localization);

						// Save the power reading
						pastWifiPwr.pop_back();
						pastWifiPwr.insert(pastWifiPwr.begin(),power);

						// Show the last pairs
						std::vector<int>::iterator iter;
						std::vector<std::string>::iterator iter2;
						iter2 = pastPositions.begin();
						for (iter = pastWifiPwr.begin();iter != pastWifiPwr.end();iter++){
							std::cout << *iter << "\t" << *iter2 << std::endl;
							iter2++;
						}
					} else {
						// The strength is below the threshold. An action will be taken (by the moment, verbally notify the situation and go back to the saved place with the greatest power
						goingBack = true;
						
						// Alert by voice when there is no good wifi coverage
						//!  @moos_publish  SAY  Variable containing a text phrase for voice reproduction
						m_Comms.Notify("SAY","NO TENGO BUENA COBERTURA WIFI, VUELVO A UN PUNTO MEJOR");
						
						// Search for the location with the maximum strength
						std::vector<int>::iterator iter;
						float max_pwr = 0;
						int pos = 0;
						int k = 0;
						for (iter = pastWifiPwr.begin();iter != pastWifiPwr.end();iter++){
							if(*iter >= max_pwr)
								pos = k;
							k++;
							
						}

						robotPose3D = pastPositions[pos];
						std::cout << "NO TENGO BUENA COBERTURA WIFI, VUELVO A UN PUNTO MEJOR: " << robotPose3D << std::endl;

						// Move to the place with the highest wifi strength using the reactive navigator
						char pose3d[128];
						strcpy(pose3d,robotPose3D.c_str());
						char *cx, *cy; // *pr
						//pr = strtok(pose3d,"[");
						cx = strtok(pose3d,"[ ");

						cy = strtok(NULL," ");

						float x,y;
						x = atof(cx);
						y = atoi(cy);
						
						//Set target to navigate for a better wifi signal
						//!  @moos_publish <NAVIGATE_TARGET> Reactive target position to navigate in the form [x y]
						m_Comms.Notify("NAVIGATE_TARGET", mrpt::format("[%.03f %.03f]", x,  y) );
					}
				} else {
					// The robot was moving back, but it is already in a zone with good strength
					if(power > min_strength) goingBack = false;
				}
			}
		}

		if( (i->GetName()=="LOCALIZATION")){
			std::cout << "LOCALIZATION: " << i->GetAsString() << std::endl;
			localization = i->GetAsString();
		}

        if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}
 
	//UpdateMOOSVariables(NewMail);
	return true;
}
