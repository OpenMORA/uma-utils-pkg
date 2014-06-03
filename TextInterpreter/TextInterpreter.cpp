/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2013  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Manuel Lopez Antequera  <mlopezantequera@gmail.com>           |
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

/**  @moos_module A generic text input/output module. This module parses any text written into a moos variable.
	* The variable name to listen to must be set as a parameter when invoking this module (see required parameters).
	*
	* After parsing, new tasks are generated through the "NEW_TASK" moos variable.
	* It's intended to parse high level commands like 'say to <person> <message>'
	* sent via Skype or interpreted through voice.
	*/

/** @moos_ToDo
  *  Improve documentation of module. Define parameters and published variables.
  *  Is variable SKYPE_CAM in use? If not, remove it.
  */

#include "TextInterpreter.h"
#include <mrpt/utils.h>

using namespace std;
using namespace mrpt;


TextInterpreterApp::TextInterpreterApp()
{
}

TextInterpreterApp::~TextInterpreterApp()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool TextInterpreterApp::OnStartUp()
{
  EnableCommandMessageFiltering(true);

  try
  {

    //! @moos_param listening_var The variable this module will listen to to obtain the text to parse.
    listening_variable = m_ini.read_string("pTextInterpreter","listening_var","",true);
    cout << "listening_variable = " << listening_variable << std::endl;

    //! @moos_param output_var The variable this module will write to for replies.
    output_variable = m_ini.read_string("pTextInterpreter","output_var","",true);
    cout << "listening_variable = " << listening_variable << std::endl;

    std::string read_mission;
    if(m_MissionReader.GetConfigurationParam("mission1",read_mission))
      mission_file_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission2",read_mission))
      mission_file_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission3",read_mission))
      mission_file_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission4",read_mission))
      mission_file_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission5",read_mission))
      mission_file_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission6",read_mission))
      mission_file_list.push_back(read_mission);


    if(m_MissionReader.GetConfigurationParam("mission1_title",read_mission))
      mission_name_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission2_title",read_mission))
      mission_name_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission3_title",read_mission))
      mission_name_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission4_title",read_mission))
      mission_name_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission5_title",read_mission))
      mission_name_list.push_back(read_mission);

    if(m_MissionReader.GetConfigurationParam("mission6_title",read_mission))
      mission_name_list.push_back(read_mission);

    if (mission_name_list.size() != mission_file_list.size())
      {
        MOOSTrace("Error reading mission names and files.\n");
        mission_name_list.clear();
        mission_file_list.clear();
      }

    cout << "Finish OnStartUp" << endl;
    DoRegistrations();
    return true;
  }
  catch (std::exception &e)
  {
    cerr << "**ERROR** " << e.what() << endl;
    return MOOSFail( "Closing due to an exception." );
  }
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool TextInterpreterApp::OnCommandMsg( CMOOSMsg Msg )
{
  if(Msg.IsSkewed(MOOSTime())) return true;
  if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
  const std::string sCmd = Msg.GetString();
  //MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
  // Process the command "sCmd".

  return true;
}


//-------------------------------------
// Personal Procedures
//-------------------------------------
bool TextInterpreterApp::InterpretString(std::string s)
{
  int uid = 0;
  std::vector<std::string> tokens;
  std::string s_aux;

  std::string help_string = "\nAvailable commands:\n\
      go to <location>:I will navigate to a location.\n\
      go recharge: I'll go to the docking station and recharge my battery. \n\
      say <message>: I'll speak on your behalf.\n\
      say to <person> <message>: I'll tell somebody something on your behalf.\n\
      look at <X>: I'll look at a person or location \n\
      show map: I'll show you my navigation map.\n\
      show camera: I'll show you video from the webcam on my shoulder.\n\
      play <song>: I'll play some music through my speakers.\n\
      stop: I'll stop moving around.\n\
      soon: you are near <location>: Helps me find where I am.\n\
      soon: switch <ON/OFF> <appliance>: I'll remotely turn on or off an appliance.\n\
      ";

      std::transform(s.begin(), s.end(), s.begin(), ::tolower);

  std::string command_origin = listening_variable;
  cout << "Interpreting:" << s << ". Sent by " << command_origin <<endl;

  try
  {
    mrpt::system::tokenize(s," ",tokens);


    if (tokens.at(0) == "echo")
      {
		//! @moos_publish "output_variable" The variable specified in the parameters to replay after parsing
        m_Comms.Notify(output_variable,s);
      }

    else if (tokens.at(0) == "show")
      {
        if (tokens.size() > 1)
          {
            if (tokens.at(1) == "map")
              {
                m_Comms.Notify(output_variable,"ok");
				//! @moos_publish SKYPE_CAM
                m_Comms.Notify("SKYPE_CAM",0.0);
              }
            else if (tokens.at(1) == "camera")
              {
                m_Comms.Notify(output_variable,"ok");
                m_Comms.Notify("SKYPE_CAM",0.0);
              }
          }
      }

    else if (tokens.at(0) == "go")
      {
        if (tokens.size() > 1)
          {
            if (tokens.at(1) == "to")
              {
                s_aux = "MOVE " + tokens[2];
				//! @moos_publish NEW_TASK The new tast generated after parsing the incomming text.
                m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
				//! @moos_publish NECKMSG The Neck position to achieve
                m_Comms.Notify("NECKMSG","SERVO=0,ANG=0,SPEED=60,FILTER=1");
                m_Comms.Notify(output_variable,"ok");
              }
            else if (tokens.at(1) == "recharge")
              {
                m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,"GO_TO_RECHARGE 90"));
                m_Comms.Notify("NECKMSG","SERVO=0,ANG=0,SPEED=60,FILTER=1");
                m_Comms.Notify(output_variable,"ok");
              }
          }
      }

    else if (tokens.at(0) == "say")
      {
        if (tokens.size() > 1)
          {
            if (tokens.at(1) == "to")
              {
                s_aux = "SAY_TO " + s.substr(7,s.npos);
                m_Comms.Notify("NECKMSG","SERVO=0,ANG=0,SPEED=60,FILTER=1");
                m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
                m_Comms.Notify(output_variable,"ok");
              }
            else
              {
                s_aux = "SAY "+s.substr(4,s.npos);
                m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
                m_Comms.Notify(output_variable,"ok");
              }
          }
      }

    else if (tokens.at(0) == "look")
      {
        if (tokens.size() > 1)
          {
            if (tokens.at(1) == "at")
              {
                s_aux = "LOOK_AT " + s.substr(8,s.npos);
                m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
                m_Comms.Notify(output_variable,"ok");
              }
          }
      }

    else if (tokens.at(0) == "play")
      {
        if (tokens.size() > 1)
          {
            s_aux = "PLAY_ITEM " + s.substr(5,s.npos);
            m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
            m_Comms.Notify(output_variable,"ok");
          }
      }

    else if (tokens.size() > 3){
        if (tokens.at(0) == "you" && tokens.at(1) == "are" && tokens.at(2) == "near")
          {
            s_aux = "LOCALIZATION_HINT "+ tokens.at(3);
            m_Comms.Notify("NEW_TASK",  format("%s %i %s",command_origin.c_str(),uid,s_aux.c_str()));
            m_Comms.Notify(output_variable,"ok");
          }
      }

    else if (tokens.at(0) == "stop")
      {
        m_Comms.Notify("STOP_STATECHART","");
        m_Comms.Notify("PNAVIGATORREACTIVEPTG_CMD","CANCEL");
        m_Comms.Notify("PNAVIGATORREACTIVEPTG3D_CMD","CANCEL");
        m_Comms.Notify(output_variable,"ok");
      }

    else if (tokens.at(0) == "missions")
      {
        if (mission_name_list.size() == 0)
          {
            s_aux ="There are no missions configured for remote use (Check the TextInterpreter section in the .moos file)";
          }
        else
          {
            s_aux ="Available missions:";
            for (unsigned int i = 0; i < mission_name_list.size() ; i++)
              {
                s_aux.append("\n"+format("%i:",i+1)+mission_name_list.at(i));
              }
            s_aux.append(("\n Choose a mission with the 'mission x' command"));
          }
        m_Comms.Notify(output_variable,s_aux.c_str());
      }

    else if (tokens.at(0) == "mission")
      {
        if (tokens.size() > 1)
          {
            int i = atoi(tokens[1].c_str());
            if ((i > 0) && (i <= mission_name_list.size()))
              {
                s_aux = format("Launching mission %i: %s",i,mission_name_list[i-1].c_str());
                m_Comms.Notify(output_variable,s_aux.c_str());
                m_Comms.Notify("OBJECTIVE_FILE",mission_file_list[i-1]);
              }
            else
              {
                s_aux = format("Mission %i is not on the list, try command 'missions' to see the list",i);
                m_Comms.Notify(output_variable,s_aux.c_str());
              }
          }
      }

    else m_Comms.Notify(output_variable,help_string);

  }
  catch(const std::exception& ex)
  {
    std::cout << ex.what() << endl;
    m_Comms.Notify(output_variable,help_string);
  }

  return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------

bool TextInterpreterApp::Iterate()
{
  try
  {

    return true;
  }

  catch(exception& e)
  {
    cerr << "**ERROR** " << e.what() << endl;
    return MOOSFail( "Closing due to an exception." );
  }
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool TextInterpreterApp::OnConnectToServer()
{
  DoRegistrations();
  return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool TextInterpreterApp::DoRegistrations()
{
  //! @moos_subscribe SHUTDOWN
  AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

  //Not a fixed name. The module will subscribe to whatever is inside listening_variable.
  //! @moos_subscribe <listening_variable>
  AddMOOSVariable( listening_variable, listening_variable, listening_variable, 0 );
  RegisterMOOSVariables();
  return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool TextInterpreterApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
  std::string cad;
  for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
    {
      if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
        {
          // Disconnect comms:
          MOOSTrace("Closing Module \n");
          this->RequestQuit();
        }
      if(i->GetName()==listening_variable)
        {
          InterpretString(i->GetString());
        }

    }
  UpdateMOOSVariables(NewMail);
  return true;
}
