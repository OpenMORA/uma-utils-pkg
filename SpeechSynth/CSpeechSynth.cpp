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

/**  @moos_module A module to keep the high-level tasks commanded to the robot.
  *
  */

#include "CSpeechSynth.hpp"

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;
//using namespace ....


CSpeechSynthApp::CSpeechSynthApp()
//	: var (init_val), ...
{
}

CSpeechSynthApp::~CSpeechSynthApp()
{
}

bool CSpeechSynthApp::OnStartUp()
{
	// Read parameters (if any) from the mission configuration file.
	//! @moos_param PARAM_NAME  PARAM DESCRIPTION IN ONE LINE
	//m_MissionReader.GetConfigurationParam("my_param",m_myvar);

	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);
	return DoRegistrations();
}

bool CSpeechSynthApp::Iterate()
{
	// Are there


	return true;
}

bool CSpeechSynthApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

bool CSpeechSynthApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".
	return true;
}

bool CSpeechSynthApp::DoRegistrations()
{
	//! @moos_subscribe	NEW_TASK
	AddMOOSVariable("SAY","SAY","SAY",0);
	
	RegisterMOOSVariables();
	return true;
}


bool CSpeechSynthApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{

		if (i->GetName()=="SAY")
		{
		
		MOOSTrace("Received talk: %s\n",i->GetString().c_str());
			
			const size_t len = strlen(i->GetString().c_str()) + 1;
			HGLOBAL hMem =  GlobalAlloc(GMEM_MOVEABLE, len);
			memcpy(GlobalLock(hMem), i->GetString().c_str(), len);
			GlobalUnlock(hMem);
		    OpenClipboard(0);
			EmptyClipboard();
			SetClipboardData(CF_TEXT, hMem);
			CloseClipboard();
			
			// Is it necessary to free the memory allocated????
			// GlobalFree(hMem); 
					

		}
		
		if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:			
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

	}

	UpdateMOOSVariables(NewMail);
	return true;
}
