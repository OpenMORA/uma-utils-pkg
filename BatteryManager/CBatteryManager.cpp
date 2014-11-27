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

/**  @moos_module Module to monitoring the status of a battery.
  *  It reads the battery votltage and intensity from MOOS variables (provided by the corresponding hardware module, robot base, DAQ, etc.), 
  *  and provides information related to the percentage of battery charge, estimated life, charging status, etc.
  */

#include "CBatteryManager.h"
#include <mrpt/slam/CObservationBatteryState.h>

using namespace std;

// Constructor
CBatteryManager::CBatteryManager() 
{
	Old_Is_Charging=-1;
	Old_Complete_Recharge=-1;
	Old_Battery_Life=-1;
	Old_Battery_Level=-1;
	
}

// Destructor
CBatteryManager::~CBatteryManager()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CBatteryManager::OnStartUp()
{
	//! @moos_param Battery_V Name of the MOOS variable containing the voltage of the battery to monitor.
	Battery_V = m_ini.read_string("","Battery_V","",true);

	//! @moos_param Battery_I Name of the MOOS variable containing the intensity of the battery to monitor.
	Battery_I = m_ini.read_string("","Battery_I","",true);

	//! @moos_param Charger_I Name of the MOOS variable containing the charge intensity of the battery to monitor.
	Charger_I = m_ini.read_string("","Charger_I","",true);

	//! @moos_param Offset_Battery_V An offset to apply to the readed voltage
	Offset_Battery_V = m_ini.read_float("","Offset_Battery_V",0.0,false);

	//! @moos_param Conv_Conv_Battery_V Battery Voltage Conversion Factor
	Conv_Battery_V = m_ini.read_float("","Conv_Battery_V",0,true);

	//! @moos_param Conv_Battery_I Battery Intensity Conversion Factor
	Conv_Battery_I = m_ini.read_float("","Conv_Battery_I",0,true);

	//! @moos_param Conv_Charger_I Charger Intensity Conversion Factor
	Conv_Charger_I = m_ini.read_float("","Conv_Charger_I",0,true);

	//! @moos_param Conv_Battery_Life Duration of a 10% charge of the battery with 1Ah of consumption (Units = Minutes)
	Conv_Battery_Life = m_ini.read_float("","Conv_Battery_Life",0,true);

	//! @moos_param Holding_Current Charger Holding Current (Units = Amps)
	Holding_Current = m_ini.read_float("","Holding_Current",0,true);

	//! @moos_param Level_0 Voltage Level 0, 0% Charged	 	(Units = Volts) Discharged
	Level_0 = m_ini.read_float("","Level_0",0,true);

	//! @moos_param Level_1 Voltage Level 1, 10% Charged 	(Units = Volts)	Obligatory recharge level
	Level_1 = m_ini.read_float("","Level_1",0,true);

	//! @moos_param Level_2 Voltage Level 2, 20% Charged 	(Units = Volts) Necessary recharge level
	Level_2 = m_ini.read_float("","Level_2",0,true);

	//! @moos_param Level_3 Voltage Level 3, 30% Charged 	(Units = Volts)
	Level_3 = m_ini.read_float("","Level_3",0,true);

	//! @moos_param Level_4 Voltage Level 4, 40% Charged 	(Units = Volts)
	Level_4 = m_ini.read_float("","Level_4",0,true);

	//! @moos_param Level_5 Voltage Level 5, 50% Charged 	(Units = Volts)
	Level_5 = m_ini.read_float("","Level_5",0,true);

	//! @moos_param Level_6 Voltage Level 6, 60% Charged 	(Units = Volts)
	Level_6 = m_ini.read_float("","Level_6",0,true);

	//! @moos_param Level_7 Voltage Level 7, 70% Charged 	(Units = Volts)
	Level_7 = m_ini.read_float("","Level_7",0,true);
		
	//! @moos_param Level_8 Voltage Level 8, 80% Charged 	(Units = Volts)
	Level_8 = m_ini.read_float("","Level_8",0,true);

	//! @moos_param Level_9 Voltage Level 9, 90% Charged 	(Units = Volts)
	Level_9 = m_ini.read_float("","Level_9",0,true);

	//! @moos_param Level_10 Voltage Level 10, 100% Charged (Units = Volts) Full Charged
	Level_10 = m_ini.read_float("","Level_10",0,true);

	//! @moos_param Min_I_Charge Minimum level of intensity for detecting the charging
	Min_I_Charge = m_ini.read_float("","Min_I_Charge",0,true);

	return DoRegistrations();
}
//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CBatteryManager::OnCommandMsg( CMOOSMsg Msg ) 
{
	if(Msg.IsSkewed( MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------
bool CBatteryManager::Iterate()
{
	static bool first_is_charging = true;
	static bool first_notify = true;
	bool hasV = false;
	bool hasI = false;
	bool hasC = false;
	
	try
	{
		printf("\n[BatteryManager]: Readigng OpenMORA var %s\n", Battery_V.c_str());
		//------------------------------------------
		// (1) Checking the Battery Voltage & Level
		//------------------------------------------
		CMOOSVariable *pVarBatteryV = GetMOOSVar( Battery_V );
		if( pVarBatteryV && pVarBatteryV->IsFresh() )
		{
			printf("\[BatteryManager]: Got V\n");
			pVarBatteryV->SetFresh(false);		
			Battery_Voltage = pVarBatteryV->GetDoubleVal();
			Battery_Voltage = (Battery_Voltage * Conv_Battery_V)+ Offset_Battery_V;
			hasV = true;
			printf("\[BatteryManager]: Got V=%0.2f\n",Battery_Voltage);

			if (Battery_Voltage>Level_10) //Determining the battery level
				Battery_Level = 100;
			else if (Battery_Voltage>Level_9)
				Battery_Level = 90;
			else if (Battery_Voltage>Level_8)
				Battery_Level = 80;
			else if (Battery_Voltage>Level_7)
				Battery_Level = 70;
			else if (Battery_Voltage>Level_6)
				Battery_Level = 60;
			else if (Battery_Voltage>Level_5)
				Battery_Level = 50;
			else if (Battery_Voltage>Level_4)
				Battery_Level = 40;
			else if (Battery_Voltage>Level_3)
				Battery_Level = 30;
			else if (Battery_Voltage>Level_2)
				Battery_Level = 20;
			else if (Battery_Voltage>Level_1)
				Battery_Level = 10;
			else
				Battery_Level = 0;
		}
		else
		{
			printf("[BatteryManager]: ERROR - Variable (%s) not found. Unable to monitor Voltage.", Battery_V);
			hasV = false;
		}

		printf("[BatteryManager]: Iterate has V\n");

		//-------------------------------------
		// (2) Checking Battery Intensity
		//------------------------------------
		CMOOSVariable *pVarBatteryI = GetMOOSVar( Battery_I );
		if( pVarBatteryI && pVarBatteryI->IsFresh() )
		{
			pVarBatteryI->SetFresh(false);
			Battery_Intensity = pVarBatteryI->GetDoubleVal();
			Battery_Intensity = (Battery_Intensity-2.5)*Conv_Battery_I;
			hasI = true;
		}
		else
		{
			printf("[BatteryManager]: ERROR - Variable (%s) not found. Unable to monitor Voltage.", Battery_I);
			hasI = false;
		}

		printf("[BatteryManager]: Iterate has V and I\n");
		//-------------------------------------
		// (2.1) Energy Compsumption
		//------------------------------------
		if(hasV && hasI)
		{
			Energy_Consumption = Battery_Voltage*Battery_Intensity;				//Determining the Energy Consumption
			if( Battery_Intensity < 0.1 )
				Battery_Intensity += 0.1;
			Battery_Life = (Battery_Level/Battery_Intensity)*Conv_Battery_Life; //Determining the Battery Life			
		}

		//-------------------------------------
		// (3) Checking Charge Intensity
		//------------------------------------
		CMOOSVariable *pVarChargerI = GetMOOSVar( Charger_I );
		if( pVarChargerI && pVarChargerI->IsFresh() )
		{
			pVarChargerI->SetFresh(false);
			Charger_Intensity = pVarChargerI->GetDoubleVal();
			Charger_Intensity = (Charger_Intensity-2.51)*Conv_Charger_I;
			hasC = true;

			if (Charger_Intensity>Min_I_Charge) //Determining the charge status
				Is_Charging = 1.0;			
			else
				Is_Charging = 0.0;
		}
		else
		{
			printf("[BatteryManager]: ERROR - Variable (%s) not found. Unable to monitor Voltage.", Charger_I);
			hasC = false;
		}


		//-------------------------------------
		// (3.1) Complete Recharge
		//------------------------------------
		if( hasI && hasC )
		{		
			if( (Charger_Intensity-Battery_Intensity)<Holding_Current )
				Complete_Recharge = 1.0;
			else
				Complete_Recharge = 0.0;
		}


		printf("[BatteryManager]: Iterate starts publishing\n");
		//-------------------------------------
		// (4) Moos Publish
		//------------------------------------
		if( hasV )
		{
			//!  @moos_publish   BATTERY_MANAGER_V_FLOAT	Voltage (V) of the monitored battery as a float
			m_Comms.Notify("BATTERY_MANAGER_V_FLOAT", Battery_Voltage);

			mrpt::slam::CObservationBatteryStatePtr battery_obs = mrpt::slam::CObservationBatteryState::Create();
			battery_obs->timestamp = mrpt::system::now();
			battery_obs->voltageMainRobotBattery = Battery_Voltage;
			battery_obs->voltageMainRobotBatteryIsValid = true;
			mrpt::vector_byte vec_bat;
			mrpt::utils::ObjectToOctetVector(battery_obs.pointer(), vec_bat);
			//!  @moos_publish   BATTERY_MANAGER_V   Voltage(V) of the monitored battery as an mrpt::CObservationBatteryState
			m_Comms.Notify("BATTERY_MANAGER_V", vec_bat );
			printf("[BatteryManager] Battery_Voltage: %.2f\n",Battery_Voltage);		
			

			//!  @moos_publish   BATTERY_MANAGER_LEVEL  Charge Level (Value*10%) of the monitored battery
			if( Old_Battery_Level!=Battery_Level )
			{
				m_Comms.Notify("BATTERY_MANAGER_LEVEL", Battery_Level);	
				Old_Battery_Level=Battery_Level;
				printf("[BatteryManager]: Battery_Level: %f\n",Battery_Level);
			}
		}
	

		if (hasI)
		{
			//!  @moos_publish   BATTERY_MANAGER_I Intensity (A) consumed by the monitored battery
			m_Comms.Notify("BATTERY_MANAGER_I", Battery_Intensity);
			printf("[BatteryManager] Battery_Intensity: %.2f\n",Battery_Intensity);			
		}


		if( hasV && hasI )
		{
			//!  @moos_publish BATTERY_MANAGER_ENERGY_COMPSUMPTION (Units=Watts)
			m_Comms.Notify("BATTERY_MANAGER_ENERGY_COMPSUMPTION", Energy_Consumption);

			//!  @moos_publish   BATTERY_MANAGER_LIFE	(Units=Minutes) 
			if( Old_Battery_Life!=Battery_Life )
			{
				m_Comms.Notify("BATTERY_MANAGER_LIFE", Battery_Life);
				Old_Battery_Life=Battery_Life;
				//	printf("Battery_Life:%f\n",Battery_Life);
			}
		}


		if( hasC )
		{
			//!  @moos_publish   BATTERY_MANAGER_CHARGER_I Intensity (A) of the charger
			m_Comms.Notify("BATTERY_MANAGER_CHARGER_I", Charger_Intensity);
			printf("[BatteryManager] Charger_Intensity: %.2f\n",Charger_Intensity);

			//!  @moos_publish   BATTERY_MANAGER_IS_CHARGING  0 The battery is not charging, 1 The battery is charging
			//It only publishes if there is a change in the status of the IS_CHARGING value
			if( Old_Is_Charging!=Is_Charging && !first_is_charging )
			{
				printf("[BatteryManager]: Is_Charging: %f\n",Is_Charging);
				m_Comms.Notify("BATTERY_MANAGER_IS_CHARGING", Is_Charging);
		
				if( Is_Charging==1 )
					m_Comms.Notify("NEW_TASK","BatteryManager 0 PLAY_ITEM docked.wav");
				else 
					m_Comms.Notify("NEW_TASK","BatteryManager 0 PLAY_ITEM undocked.wav");
		
				Old_Is_Charging=Is_Charging;	
			}
			else if (first_is_charging)
			{
				first_is_charging = false;
				Old_Is_Charging = Is_Charging;
				m_Comms.Notify("BATTERY_MANAGER_IS_CHARGING", Is_Charging);
			}
		}


		if( hasI && hasC)
		{
			//!  @moos_publish   BATTERY_MANAGER_COMPLETE_RECHARGE	0 Recharging not finished , 1 Recharging finished  
			if( Old_Complete_Recharge!=Complete_Recharge )
			{
				m_Comms.Notify("BATTERY_MANAGER_COMPLETE_RECHARGE", Complete_Recharge);
				Old_Complete_Recharge=Complete_Recharge;
				printf("[BatteryManager]: Complete_Recharge: %f\n",Complete_Recharge);
			}
		}
	

		//-------------------------------------
		// (5) Additional actions
		//------------------------------------
		if (first_notify)
		{
			char cad[80];
			sprintf(cad,"[BatteryManager]: Duración estimada de funcionamiento %d minutos",int(Battery_Life));
			printf("%s(%f)\n",cad,Battery_Life);
			if( Battery_Life>0.0 && Battery_Level>10 )
			{
				m_Comms.Notify("SAY",cad);
				printf("%s(%f)\n",cad,Battery_Life);
				first_notify=false;
			}
		}

	}
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}
	catch (...)
	{
		cout << "Program finished for an untyped exception!!" << endl;
		return -1;
	}

	return true; 
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CBatteryManager::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CBatteryManager::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe Battery_V	
	AddMOOSVariable( Battery_V, Battery_V, Battery_V, 0 );

	//! @moos_subscribe Battery_I
	AddMOOSVariable( Battery_I, Battery_I, Battery_I, 0 );

	//! @moos_subscribe Charger_I
	AddMOOSVariable( Charger_I, Charger_I, Charger_I, 0 );

	
	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CBatteryManager::OnNewMail(MOOSMSG_LIST &NewMail)
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

