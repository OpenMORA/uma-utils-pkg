/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                |
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

/**  @moos_module Speech synthesis (TTS), in Spanish using the comercial program Verbio.
  *  This module requires installing the Verbio license USB stick to work at run-time. 
  *  However, it can be compiled without problems without the license.
  *  For more info on Verbio TTS see: http://www.verbio.com/webverbio3/es/tecnologia/verbio-tts.html
  */

/**  @moos_ToDo
  *  Improve documentation
  *  Specify how to install the license or configure it.
  */


#include "CVoiceApp.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>


#ifndef _WIN32
#include <values.h>  // MAXINT
#endif

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;


CVoiceApp::CVoiceApp()
//	: var (init_val), ...
{
}

CVoiceApp::~CVoiceApp()
{
}

bool CVoiceApp::OnStartUp()
{

	m_ini.enableSectionNames();
	bool notfound=false;
	//! @moos_param   num_actions
	num_actions   = m_ini.read_uint64_t("pCommon","num_actions",0,notfound);
	//! @moos_param   num_elements
	num_elements  = m_ini.read_uint64_t("pCommon","num_elements",0,notfound);
	//! @moos_param   actions_names
	actions_names = m_ini.read_uint64_t("pCommon","actions_names",0,notfound);
	m_ini.disableSectionNames();

	std::vector<std::string> action_list;
	std::vector<std::string> element_list;

	printf("Num_actions %d\n",num_actions);
	for (size_t i=0;i<num_actions;i++)
	{
		std::string name;
		name=format("%s%u","action",(unsigned int)i);
		std::string value;
		if (!m_MissionReader.GetConfigurationParam(name,value)) printf("Error while retrieving %s\n",name.c_str());
		action_list.push_back(value);
	}

	for (size_t i=0;i<num_elements;i++)
	{
		std::string name;
		name=format("%s%u","element",(unsigned int)i);
		std::string value;
		if (!m_MissionReader.GetConfigurationParam(name,value)) printf("Error while retrieving %s\n",name.c_str());
		printf("------------------------>>>>>%s\n",value.c_str());
		element_list.push_back(value);
	}

	//! @moos_param say_welcome
	if (!m_MissionReader.GetConfigurationParam("say_welcome",welcome)) printf("Error while retrieving say_welcome\n");
	//! @moos_param say_welcome1
	if (!m_MissionReader.GetConfigurationParam("say_welcome1",welcome1)) printf("Error while retrieving say_welcome\n");




	voice_id=0;
	voice=new VoiceManager(0,0);
	VoiceEngineHandler::VoiceEngineType engineType; //Type of the engine
	engineType = VoiceEngineHandler::VERBIO;

	printf("Loading actions\n");
	voice->LoadActions(action_list);
	printf("Loading elements\n");
	voice->LoadWorldElements(element_list);
	printf("Initializing \n");
	voice->InitVoiceInteraction(engineType);

	return DoRegistrations();

}

bool CVoiceApp::OnCommandMsg( CMOOSMsg Msg )
{

	return true;
}

bool CVoiceApp::Iterate()
{
	// Reproduce pending TTS strings:
	if (voice->ReproduceQueuedStrings())
	{
		// At least one string was reproduced:
		//! @moos_publish   VOICE_EVENT_DONE   Event to indicate that current text has been reproduced
		m_Comms.Notify("VOICE_EVENT_DONE","");
	}

	// Process recognized phrases:
	VoiceManager::TRecognizedPhraseList lstRecog;
	voice->GetRecognizedPhrases(lstRecog);

	if (!lstRecog.empty())  cout << "[pVoiceVerbio] Recognized strings: " << lstRecog.size() << endl;

	for (VoiceManager::TRecognizedPhraseList::const_iterator it=lstRecog.begin();it!=lstRecog.end();++it)
	{
		// it->timestamp ...
		const std::string requestCode = it->requestedCode;
		const std::string goal = it->goal;

		printf("I heard %s %s\n",requestCode.c_str(),goal.c_str());

		if (requestCode=="HELLO")
		{
			voice->QueueString2Reproduce(welcome,1,100);
			//m_Comms.Notify("NEW_TASK", format("VOICE %d SAY %s",voice_id,welcome.c_str()));
		}
		else if (requestCode=="HELLO1")
		{
			voice->QueueString2Reproduce(welcome1,1,100);
			//m_Comms.Notify("NEW_TASK", format("VOICE %d SAY %s",voice_id,welcome1.c_str()));
		}
		else
		{
			//! @moos_publish   NEW_TASK   
			m_Comms.Notify("NEW_TASK", format("VOICE %d %s %s",voice_id,requestCode.c_str(),goal.c_str()));
		}
		voice_id++;

	} // end for it


	return true;
}

bool CVoiceApp::OnConnectToServer()
{
	DoRegistrations();
	return true;
}


bool CVoiceApp::DoRegistrations()
{
	//! @moos_subscribe	SAY
	AddMOOSVariable("SAY","SAY","SAY",0);

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}


bool CVoiceApp::OnNewMail(MOOSMSG_LIST &NewMail)
{

	std::string cad;
	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
		printf("%s\n",i->GetString().c_str());
		if (i->GetName()=="SAY")
		{
			printf("Text to SAY %s\n",i->GetString().c_str());
			voice->QueueString2Reproduce(i->GetString(),1,100);
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
