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

/**  @moos_module Module to enable communications with a Twitter agent.
  *  This module enables sending text incomming from a TWEET MOOS variable via Twitter.
  */

/**  @moos_ToDo
  *  Improve documentation of the module.
  *  Include paramteres to define the user and password that is going to connect to Twitter for publishing the messages.
  */

#include "CTwitterAgent.hpp"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <Windows.h>
using namespace std;

//using namespace ....


CTwitterAgentApp::CTwitterAgentApp()
//	: var (init_val), ...
{
	
}

CTwitterAgentApp::~CTwitterAgentApp()
{
}

bool CTwitterAgentApp::OnStartUp()
{
	// Read parameters (if any) from the mission configuration file.
	//! @moos_param PARAM_NAME  PARAM DESCRIPTION IN ONE LINE
	//m_MissionReader.GetConfigurationParam("my_param",m_myvar);

	// There is also a MRPT-like object (this->m_ini) that is a wrapper
	//  to read from the module config block of the current MOOS mission file.
	// m_ini.read_int(...);

	printf("On Startup\n");
	std::string user="sancho_robot";
	std::string pass="mapir236";
	//if (user=="" || pass=="")	
		//#pragma message("Twitter user and password must be setup before execution.")

	

	if (tw.Connect(user,pass))
			printf("\n[TwitterClient]: User %s connected\n\n",user.c_str());
	else
		printf("\n[TwitterClient]: Error connecting user [%s]\n\n",user.c_str());

	return DoRegistrations();
}

bool CTwitterAgentApp::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CTwitterAgentApp::Iterate()
{
	// Are there
/*

	Check here for tweets from my followers...

	*/
	char tst[80];
	static int c=0;
	sprintf(tst,"Test %d\n",c);
	printf("%s\n",tst);
			const size_t len = strlen(tst) + 1;
			HGLOBAL hMem =  GlobalAlloc(GMEM_MOVEABLE, len);
			memcpy(GlobalLock(hMem), tst, len);
			GlobalUnlock(hMem);
		    OpenClipboard(0);
			EmptyClipboard();
			SetClipboardData(CF_TEXT, hMem);
			CloseClipboard();
			Sleep(10000);
			c++;
	return true;
}

bool CTwitterAgentApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CTwitterAgentApp::DoRegistrations()
{
	//! @moos_subscribe	TWEET
	AddMOOSVariable("TWEET","TWEET","TWEET",0);
	
	//! @moos_subscribe	TALK
	AddMOOSVariable("TALK","TALK","TALK",0);
	
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}


bool CTwitterAgentApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{

		if (i->GetName()=="TWEET")
		{
			MOOSTrace("Received tweet: %s\n",i->GetString().c_str());
			tw.Tweet(i->GetAsString());

		}
		if (i->GetName()=="TALK")
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
			
			// Is it necessary to free the memory allocated
			// GlobalFree(hMem); 
		}
		{

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
