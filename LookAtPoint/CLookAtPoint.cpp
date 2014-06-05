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

/**  @moos_module Module to make the robot turn to look at a given absolute (x,y) coordinates, including optionally turn its neck. */

#include "CLookAtPoint.h"

#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

using namespace mrpt;
using namespace mrpt::utils;

CLookAtPoint::CLookAtPoint() :
	m_maxRotationSpeed(40),
	m_maxAngleNeck(15),
	m_currentLookingAt_turning(false)
{
}

CLookAtPoint::~CLookAtPoint()
{
}

bool CLookAtPoint::OnStartUp()
{
	//! @moos_param maxRotationSpeed  The maximum rotation speed in deg/sec.
	m_MissionReader.GetConfigurationParam( "maxRotationSpeed", m_maxRotationSpeed );

	//! @moos_param maxAngleNeck  The maximum rotation of the eNeck, in deg.
	m_MissionReader.GetConfigurationParam( "maxAngleNeck", m_maxAngleNeck );

	return DoRegistrations();
}

bool CLookAtPoint::OnCommandMsg( CMOOSMsg Msg )
{
	if(Msg.IsSkewed(MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();
	//MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());
	// Process the command "sCmd".

	return true;
}

bool CLookAtPoint::Iterate()
{
	try
	{
		CMOOSVariable *pVarLoc = GetMOOSVar( "LOCALIZATION" );

		if(pVarLoc && pVarLoc->IsFresh())
			m_curPose.fromString( pVarLoc->GetStringVal() );


        static bool isFirstIter = false;

		CMOOSVariable *cmdMsg = GetMOOSVar("LOOK_AT_POINT");				// Get the msg to the neck
		if (cmdMsg && cmdMsg->IsFresh())
		{
			cmdMsg->SetFresh(false);

			m_currentLookingAt_turning = true;
			cout << "[LookAt] " << cmdMsg->GetStringVal()<<endl;
			isFirstIter = true;
			m_currentLookingAt.fromString(cmdMsg->GetStringVal());
		} // end-if-isfresh

		// Are we turning now:
		if (m_currentLookingAt_turning)
		{
			// Only go on if we have a proper, "recent" localization:
			if (pVarLoc && pVarLoc->GetAge( MOOSTime() )<15)
			{
				mrpt::poses::CPoint2D  relTarget = m_currentLookingAt - m_curPose;

				double trg_ang = atan2(relTarget.y(),relTarget.x());

				MOOSTrace("[LookAt] Relative target vector: (%.02f,%.02f)\n",relTarget.x(),relTarget.y());
				MOOSTrace("[LookAt] Relative target angle: %.02f\n",RAD2DEG(trg_ang));

				// Reduce trg_ang so:  (new)trg_ang + eNeck_ang = (old)trg_ang
				double eNeck_ang = 0;
				if (std::abs(trg_ang)>std::abs(DEG2RAD(m_maxAngleNeck)))
				{
					if (trg_ang>0)
					{
						trg_ang-=DEG2RAD(m_maxAngleNeck);
						eNeck_ang=DEG2RAD(m_maxAngleNeck);
					}
					else
					{
						trg_ang+=DEG2RAD(m_maxAngleNeck);
						eNeck_ang=-DEG2RAD(m_maxAngleNeck);
					}
				}
				else
				{
					// Moving the eNeck is enough, don't turn the robot itself:
					trg_ang=0;
					eNeck_ang=DEG2RAD(trg_ang);
				}

				if (isFirstIter)
				{
				    isFirstIter = false;
                    MOOSTrace("[pLookAt] Sending to neck:  ANG=%.1f\n", RAD2DEG(eNeck_ang));
                    // Send eNeck command:
					//! @moos_publish   NeckMSG   The required Neck position, in the format: "SERVO=S,ANG=A,SPEED=SP,FILTER=FI"
					//! Using an unsorted list notation:
					//! <UL>
					//! <li>S = index of the servo (0 or 1) </li>
					//! <li>A = angle in degrees </li>
					//! <li>SP = speed of the servo (from 15º/s to 250º/s approx) </li>
					//! <li>FI = 1:enable filtering, 0:no filter of subsequent angle commands.</li>
					//! </UL>
                    m_Comms.Notify("NECKMSG", format("SERVO=0,ANG=%.1f,SPEED=50,FILTER=0", RAD2DEG(eNeck_ang) ) );
				}

				// Are we close enough to the target orientation? If so, stop turning:
				if (std::abs(trg_ang)>DEG2RAD(3))
				{
					double rot_vel = DEG2RAD(m_maxRotationSpeed) * (std::min(std::abs(trg_ang),DEG2RAD(40.0))/DEG2RAD(40.0)) * mrpt::utils::sign(trg_ang);
					//! @moos_publish   MOTION_CMD_V  The requested robot linear speed
					m_Comms.Notify("MOTION_CMD_V", 0.0);
					//! @moos_publish   MOTION_CMD_W  The requested robot angular speed
					m_Comms.Notify("MOTION_CMD_W", rot_vel);
				}
				else
				{
					// End of turning:
					//! @moos_publish NAV_EVENT_END Indicates the current navigation is close to end
                    m_Comms.Notify("NAV_EVENT_END", "" );
					//! @moos_publish   MOTION_CMD_V  The requested robot linear speed
                    m_Comms.Notify("MOTION_CMD_V", 0.0);
					//! @moos_publish   MOTION_CMD_W  The requested robot angular speed
                    m_Comms.Notify("MOTION_CMD_W", 0.0);
					m_currentLookingAt_turning=false;
					MOOSTrace("Finished turning in pLookAtPoint.\n");
				}
			}
			else
			{
				//! @moos_publish NAV_EVENT_END Indicates the current navigation is close to end
				m_Comms.Notify("NAV_EVENT_END", "" );
				//! @moos_publish   MOTION_CMD_V  The requested robot linear speed
                m_Comms.Notify("MOTION_CMD_V", 0.0);
				//! @moos_publish   MOTION_CMD_W  The requested robot angular speed
                m_Comms.Notify("MOTION_CMD_W", 0.0);
				m_currentLookingAt_turning=false;
				MOOSTrace("Stopping turning in pLookAtPoint due to old localization data.\n");
			}
		}
		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail("Exception in Iterate: %s\n",e.what());
	}
}

bool CLookAtPoint::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

bool CLookAtPoint::DoRegistrations()
{
	//! @moos_subscribe	LOOK_AT_POINT
	//! @moos_var LOOK_AT_POINT The 2D absolute coordinates of the point to look at, in the format "[x y]"
	AddMOOSVariable(
		"LOOK_AT_POINT", "LOOK_AT_POINT", "LOOK_AT_POINT",
		0 /* Minimum time in seconds between updates */ );

	//! @moos_subscribe	LOCALIZATION
	AddMOOSVariable("LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0.05);
	
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	RegisterMOOSVariables();
	return true;
}

bool CLookAtPoint::OnNewMail(MOOSMSG_LIST &NewMail)
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

	}


    UpdateMOOSVariables(NewMail);
    return true;
}
