/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2012  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |    Robotics (MAPIR) Lab, University of Malaga (Spain).                    |
   |    Contact: Carlos Sánchez  <carlossanchez@uma.es>                        |
   |                                                                           |
   |   This file is part of the MORA project.                                  |
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

#ifndef CBatteryManager_H
#define CBatteryManager_H

#include <CMapirMOOSApp.h>
#include <sstream>
#include <iomanip>
#include <iostream>


class CBatteryManager : public CMapirMOOSApp
{
public:
    CBatteryManager();
    virtual ~CBatteryManager();

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

	bool DoRegistrations();

	// DATA. Your local variables here...

	std::string					Battery_V; //Battery Voltage Variable
	std::string					Battery_I; //Battery Intensity Variable
	std::string					Charger_I; //Charger Intensity Variable
	float						Level_0; //Voltage Level 0, 0% Charged	 	(Units = Volts) Discharged
	float						Level_1; //Voltage Level 1, 10% Charged 	(Units = Volts)	Obligatory recharge level
	float						Level_2; //Voltage Level 2, 20% Charged 	(Units = Volts) Necessary recharge level
	float						Level_3; //Voltage Level 3, 30% Charged 	(Units = Volts)
	float						Level_4; //Voltage Level 4, 40% Charged 	(Units = Volts)
	float						Level_5; //Voltage Level 5, 50% Charged 	(Units = Volts)
	float						Level_6; //Voltage Level 6, 60% Charged 	(Units = Volts)
	float						Level_7; //Voltage Level 7, 70% Charged 	(Units = Volts)
	float						Level_8; //Voltage Level 8, 80% Charged 	(Units = Volts)
	float						Level_9; //Voltage Level 9, 90% Charged 	(Units = Volts)
	float						Level_10; //Voltage Level 10, 100% Charged 	(Units = Volts) Full Charged
	float						Conv_Battery_V; //Battery Voltage Conversion Factor 
	float						Conv_Battery_I;	//Battery Intensity Conversion Factor 
	float						Conv_Charger_I; //Charger Intensity Conversion Factor 
	float						Conv_Battery_Life; //Duration of a 10% charge of the battery with 1Ah of consumption (Units = Minutes) 
	double						Is_Charging; // Detection of Battery Recharging, 0 = Not being recharged, 1 = Is being recharged
	double						Battery_Level; // Level indicator of battery charge (Value*10%)  
	double						Battery_Voltage; // Voltage at battery terminals 
	double						Battery_Intensity; // Intensity at battery terminals
	double						Charger_Intensity; // Intensity at charger terminals
	float						Energy_Consumption; // Robot's power consumption
	float						Min_I_Charge; //Minimum level of intensity for detecting the charging (Units = Amps) 
	float						Battery_Life; // Battery Life (Units=Minutes)
	float						Complete_Recharge; // 0 Recharging not finished , 1 Recharging finished
	float						Holding_Current; // Charger Holding Current

	// auxiliary class variables
	double						Old_Is_Charging; // previous value of Is_Charging that was published
	double						Old_Complete_Recharge; // previous value of Complete_Recharge that was published
	double						Old_Battery_Life; // previous value of Battery_Life that was published
	double						Old_Battery_Level; // previous value of Battery_Level that was published
	


};
#endif