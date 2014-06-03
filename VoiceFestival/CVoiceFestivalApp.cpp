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


/**  @moos_module Speech synthesis (TTS), in different languages, using the free Festival program.
  *  This module requires installing the "festival" Text To Speech package to work at run-time. 
  *  However, it can be compiled without problems even without festival installed in the system.
  *  The "festival" App is designed to work under linux systems, so in windows use Cygwin.
  *  For more info on Festival TTS see: http://www.cstr.ed.ac.uk/projects/festival
  */

#include "CVoiceFestivalApp.h"

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;


CVoiceFestivalApp::CVoiceFestivalApp()
{
}

CVoiceFestivalApp::~CVoiceFestivalApp()
{
}


bool CVoiceFestivalApp::OnStartUp()
{
	EnableCommandMessageFiltering(true);

	try
	{

		DoRegistrations();
		return true;
    }
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}

bool CVoiceFestivalApp::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("This module only accepts string command messages\n");

    std::string sCmd = Msg.GetString();
//    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CVoiceFestivalApp::Iterate()
{
	try
	{
		// Nothing to do here...
		return true;
	}
	catch (std::exception &e)
	{
		cerr << "**ERROR** " << e.what() << endl;
		return MOOSFail( "Closing due to an exception." );
	}
}


bool CVoiceFestivalApp::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CVoiceFestivalApp::DoRegistrations()
{
	//! @moos_subscribe SAY
	m_Comms.Register("SAY", 0 );

	//! @moos_subscribe SHUTDOWN
	this->m_Comms.Register("SHUTDOWN",0);

    RegisterMOOSVariables();
    return true;
}


bool CVoiceFestivalApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
    UpdateMOOSVariables(NewMail);

    for (MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
    	try
    	{
			if (MOOSStrCmp(i->GetName(),"SAY"))
			{
				const std::string val = i->GetString();
				MOOSTrace("[VoiceFestival] Saying: '%s'\n", val.c_str());

				std::string cmd;
#ifdef MRPT_OS_LINUX
				cmd = format("sh -c \"echo '%s' | esddsp festival --tts\"",val.c_str());
#else
				cmd = format("sh -c \"echo '%s' | esddsp festival --tts\"",val.c_str());
#endif
				int ret = ::system(cmd.c_str());
				if (ret!=0)
				{
					MOOSTrace("[VoiceFestival] ERROR %i executing: '%s'\n",ret, cmd.c_str());

				}

				// Send event of Voice DONE:
				//! @moos_publish   VOICE_EVENT_DONE   Event to indicate that current text has been reproduced
				m_Comms.Notify("VOICE_EVENT_DONE","");
			}
			if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
			{
				// Disconnect comms:			
				MOOSTrace("Closing Module \n");
				this->RequestQuit();
			}
    	}
    	catch (std::exception &e)
    	{
    		cerr << "**ERROR** processing mail: " << i->GetName() << endl << e.what() << endl;
    	}
	}

    return true;
}
