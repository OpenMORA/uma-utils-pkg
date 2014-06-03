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

/**  @moos_module Module to monitor the signal strength of an specified wireless network.
  *  This module uses the MRPT class CWirelessPower to be OS independent.
  *  This module just monitor the signal strenght and publish it as MOOS variables.
*/

#include "CWifiWatch.h"
#include <string.h>
#include <fstream>
#include <mrpt/system.h>
#include <sstream>
#include <mrpt/slam/CObservationWirelessPower.h>

WifiWatch::WifiWatch()
{
}

WifiWatch::~WifiWatch()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool WifiWatch::OnStartUp()
{
	// Load GenericSensor paremeters
	//! @moos_parameter <GenericSensor> The MRPT Generic sensor parameters
	wifi.loadConfig( m_ini, "pWifiWatch" );

	//! @moos_parameter SSID The SSID of the Wireless network to monitor
	//! @moos_parameter GUID The GUID of the Wireless network to monitor
	if(!(m_MissionReader.GetConfigurationParam( "GUID", guid ) && m_MissionReader.GetConfigurationParam( "SSID", ssid ))){
		std::cout << "Please specify both SSID and GUID of the wifi network to monitor" << std::endl;
	}
		
	return DoRegistrations();
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool WifiWatch::OnCommandMsg( CMOOSMsg Msg )
{
	return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool WifiWatch::Iterate()
{
	//std::cout << "Iterate()" << std::endl;
/*	int power;
	power = wifi.GetPower();*/
	mrpt::slam::CObservationWirelessPower power;
	bool obs_ok;
	obs_ok = wifi.getObservation(power);
	if (!obs_ok )
	{
		std::cout << "- Could not retrieve an observation from the Wifi dongle..." << std::endl;
		return true;
	}else{
	//	std::cout << "Potencia: " << power.power << std::endl;
		
		//!  @moos_publish   WIFI_POWER   Strength of the wifi signal (0 to 100) monitored as a double value.
		m_Comms.Notify("WIFI_POWER",power.power);
		
		mrpt::slam::CObservationWirelessPowerPtr powerOutput = mrpt::slam::CObservationWirelessPower::Create();
		powerOutput->timestamp = power.timestamp;
		powerOutput->power = power.power;
		powerOutput->sensorLabel = power.sensorLabel;
		std::string sObs = ObjectToString( powerOutput.pointer() );

		
		//!  @moos_publish   WIFI_POWER_SER   The last Wifi measurements as a MRPT CObservationWirelessPower observation
		m_Comms.Notify("WIFI_POWER_SER", sObs );
	}

	return true;
}



//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool WifiWatch::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool WifiWatch::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN",0);
	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool WifiWatch::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	std::cout << "OnNewMail()" << std::endl;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
	
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
