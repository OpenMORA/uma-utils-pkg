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

#include "TextInterpreter.h"

int main(int argc ,char * argv[])
{
	try
	{
		const char * sMissionFile = "Mission.moos";
		const char * sMOOSName = "pTextInterpreterApp";


		switch(argc)
		{
		case 3:
			sMOOSName = argv[2];
		case 2:
			sMissionFile = argv[1];
		}

		TextInterpreterApp TheApp;
		TheApp.Run(sMOOSName,sMissionFile);

		return 0;
    }
	catch (std::exception &e)
	{
		std::cerr << "**ERROR** " << e.what() << std::endl;
		return MOOSFail( "Closing due to an exception" );
	}
	catch (...)
	{
		return MOOSFail( "Closing due to an untyped exception." );
	}
}
