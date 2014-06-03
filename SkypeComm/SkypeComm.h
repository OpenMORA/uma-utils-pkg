/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Emil Jatib Khatib  <emilkhatib@uma.es>		                   |
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

#ifndef SkypeComm_H
#define SkypeComm_H

#include <Python.h>  // Include BEFORE MRPT headers to avoid #define redefinition warnings
#include <MOOS/libMOOS/App/MOOSApp.h>
#include <CMapirMOOSApp.h>

#define M_RECV "SKYPE_RECV"
#define M_SEND "SKYPE_SEND"
#define M_RECV_USER "SKYPE_CONTACT"
#define M_SEND_USER "SKYPE_CONTACT"

//#define REMOTE_USER_DEF "rhodon.mapir"
//#define CLIENT_START_WAIT 3

class SkypeComm : public CMapirMOOSApp
{
public:
    SkypeComm();
    virtual ~SkypeComm();

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

	/** send a message through the Skype chat */
	void SendMessage(std::string);

	/** check if a new message was received */
    bool CheckNewMessage();

	/** read the last received message */
    std::string GetReceivedMessage();

	/** publish last received message */
    void PublishReceivedMessage();

	/** process last received message */
    void ProcessReceivedMessage();

	/** run a command */
	void runCommand(std::string command);

	/** get the last received message */
    PyObject* PopMessage();


	/** subscribe to MOOS variables */
	void AddVars();

	/** open help file (not used; see TODO list) */
    std::string openHelp(const char* filename);


	/** Task number */
	int taskLaunch;
	
	/** Skype object used by Python */
	PyObject *skype;

	/** List of received messages used by Python */
	PyObject *msgList;

	/** ID of the last received message */
	long msgcount;

	/** Remote user name */
	std::string remoteUser;

	/** list of MOOS variables */
	std::map<std::string,std::string> MOOSVars;
	
	/** list of configured sentences that the module recognizes */
	std::map<std::string,std::string> sentences;

	
	std::vector<std::string> transitiveActions;
	std::vector<std::string> sVariables;
};
#endif
