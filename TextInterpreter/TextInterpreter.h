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

#ifndef TextInterpreterApp_H
#define TextInterpreterApp_H

#include <MOOS/libMOOS/App/MOOSApp.h>
#include <mrpt/utils.h>
#include <COpenMORAMOOSApp.h>

using namespace mrpt::utils;

class TextInterpreterApp : public COpenMORAApp
{
public:
	/** Constructor */
    TextInterpreterApp();
	/** Destructor */
    virtual ~TextInterpreterApp();

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
	
	// state our interest in variables
	bool DoRegistrations();

	bool InterpretString(std::string s);

	// DATA. Your local variables here...	
	std::string listening_variable; //Variable where text is accepted.
	std::string output_variable; //Variable where replies are sent.
	std::vector<std::string> mission_file_list;
	std::vector<std::string> mission_name_list;
};
#endif
