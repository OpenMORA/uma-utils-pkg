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


/**  @moos_module Module to enable the communication between OpenMORA and a MQTT client.
  *  MQTT is a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, designed as an extremely lightweight publish/subscribe messaging transport.
  *  This module is used to communicate the robot (running the OpenMORA architecture), with an external MQTT client in the user side (Pilot, WebRTC, etc.).
  */


/*
	    MQTT Topic						<->          OpenMORA variable
  --------------------------------------------------------------
 
  Robot_Name/ClientACK								-->PILOT_MQTT_ACK

  Robot_Name/TopologyCommand
			  -GetTopology							-->  (Send Variable GRAPH in Topic Topology)
			  -ADD_NODE	label type x y				-->	ADD_NODE 
			  -ADD_ARC labelA labelB ArcType		-->	ADD ARC 
			  -REMOVE_NODE label					-->	REMOVE_NODE
			  -REMOVE_ARCC labelA labelB ArcType	-->	REMOVE_AR
			  -MOVE_NODE label x y					-->	MOVE_NODE
			  -CHANGE_NODE_LABEL labelOld labelNew	-->	CHANGE_NODE_LABEL
			  -LOAD_GRAPH filename					-->	LOAD_GRAPH
			  -SAVE_GRAPH filename					-->	SAVE_GRAPH

  Robot_Name/NavigationCommand
			  -StopGiraff							-->	CANCEL_NAVIGATION
			  -GoToNode	label						-->	GET_PATH 
			  -GoToPoint x y						-->	NAVIGATE_TARGET
			  -Relocalize x y						-->	RELOCALIZE_IN_AREA
			  -Motion v w							-->	MOTION_CMD_V + MOTION CMD_W
			  -RandomNavigator 0/1					--> RANDOM_NAVIGATOR

  Robot_Name/MapBuildingCommand
			 -Rawlog Start/Stop						-->	SAVE_RAWLOG
  
  ----------------------------------------------------------------------------------------
  Robot_Name/Localization							<--	LOCALIZATION
  Robot_Name/Status								<--	LOCALIZATION
  Robot_Name/ManualMode							<--	CANCEL_NAVIGATION
  Robot_Name/Topology								<--	GRAPH
  Robot_Name/ErrorMsg								<--	ERROR_MSG
  Robot_Name/LaserScan								<--	LASER1
  Robot_Name/NavigationMode manual|ninguno			<-- Navigation mode and destiny
							 auto|destino
							 recovery|destino

  Robot_Name/Question msg							<--
  Robot_Name/Answer msg							-->

  */

#include "CMQTTmosquitto.h"
#include "mosquitto.h"
#include <mosquittopp.h>
#include "Constants.h"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <mrpt/slam/CObservation2DRangeScan.h>

using namespace std;
using namespace mosqpp;
using namespace mrpt;
using namespace mrpt::utils;

CMQTTMosquitto::CMQTTMosquitto(): mosquittopp( ("Giraff_"+mrpt::system::dateTimeToString(mrpt::system::now())).c_str() ,false)
{	
}

CMQTTMosquitto::~CMQTTMosquitto()
{	
}

bool CMQTTMosquitto::OnStartUp()
{
	cout << "Configuring MQTT connection..." << endl;

	// Read Module Parameters
	//------------------------
	//! @moos_param localizationRate The rate (Hz) of sendign the localization to the Pilot client
	localizationRate = m_ini.read_float("","localizationRate",10.0,false);

	//! @moos_param sendLaserScans Yes/No send laser scans to Pilot client
	sendLaserScans = m_ini.read_bool("","sendLaserScans",true,false);
		
	//! @moos_param laserScanRate If sendLaserScans=yes, the rate (Hz) of sendign the laser scans
	laserScanRate = m_ini.read_float("","laserScanRate",10.0,false);

	//! @moos_param laserScanResolution If sendLaserScans=yes, the resolution (%) of samples from the laser scan
	laserScanResolution = m_ini.read_float("","laserScanResolution",100,false);

	cout << "READ: localizationRate = " << localizationRate << endl;
	cout << "READ: sendLaserScans = " << sendLaserScans << endl;
	cout << "READ: laserScanRate = " << laserScanRate << endl;
	cout << "READ: laserScanResolution = " << laserScanResolution << endl << endl;


	cout << "Setting Username and Password..." << endl;	
	// Read Connection Params
	//-------------------------	
	//! @moos_param user The username to connect to the MQTT broker (new sesion). It is the asigned name to the Robot, e.g: Robot_ES4
	broker_username = m_ini.read_string("","user","NO_USER_SET",true);	
	cout << "READ: user= " << broker_username << endl;

	//! @moos_param password The password to connect to the MQTT broker
	broker_password = m_ini.read_string("","password","NO_PASSWORD_SET",false);
	cout << "READ: password= " << broker_password << endl;
	
	cout << "Setting broker Host and Port..." << endl;
	
	std::string str_port;
	//! @moos_param IP The IP address of the MQTT broker
	if(m_MissionReader.GetConfigurationParam("IP",broker_host))
	{
		//broker_host = broker_host_string.c_str();
		cout << "LOCAL MQTT Broker set on IP = " << broker_host << endl;
	}
	else
	{		
		broker_host = Constants::host;
		std::cout  << "MQTT-IP default: " << broker_host << endl;
	}

	//! @moos_param PORT The PORT of the MQTT broker
    if(m_MissionReader.GetConfigurationParam("PORT", str_port))
	{
		broker_port = atoi( str_port.c_str());
		std::cout  << "LOCAL MQTT Broker set on PORT= " << broker_port << endl;
	}
	else
	{
		broker_port = Constants::port;
		std::cout  << "MQTT-PORT default: " << broker_port << endl;
	}

	cout << "Setting security parameters..." << endl;
	//Read Security Params
	//---------------------
	//! @moos_param enable_TLS_security Use TLS security layer to connect to MQTT broker
	enable_TLS_security = m_ini.read_bool("","enable_TLS_security",true,false);

	if( enable_TLS_security)
	{
		//! @moos_param TLS_path The path to the TLS certificates
		TLS_path = m_ini.read_string("","TLS_path","./",  /*Force existence:*/ false);
	
		//! @moos_param TLS_CAfile_pem Name of the CAfile
		TLS_CAfile_pem = m_ini.read_string("","TLS_CAfile_pem","",  /*Force existence:*/ true);
	
		//! @moos_param TLS_CERTfile_pem Name of the Certificate file
		TLS_CERTfile_pem = m_ini.read_string("","TLS_CERTfile_pem","",  /*Force existence:*/ true);
	
		//! @moos_param TLS_KEYfile_pem Name of the key file
		TLS_KEYfile_pem = m_ini.read_string("","TLS_KEYfile_pem","",  /*Force existence:*/ true);
	}



	// Connect to MQTT broker
	//-----------------------
	//mosquittopp(broker_username.c_str(),false);	//Initialize MQTT

	cout << "Establishing MQTT connection..." << endl;
	int g = lib_init();	
	if( username_pw_set(broker_username.c_str(), broker_password.c_str()) )
        std::cerr << "Error setting username and password.\n";
    		
	if (enable_TLS_security)
	{
		tlsOptionsSet();		//Set Security options and Data: tls-version, certificates... 
		cout << "Connecting to MQTT Broker with enabled TLS security" << endl;
	}
	else
	{
		cout << "Connecting to MQTT Broker with disabled TLS security" << endl;
	}

	// Connect the Mosquitto Client
	on_connect(connect(broker_host.c_str(), broker_port, Constants::keepalive_secs));
	//Subscribe to a list of topics 
	on_subscribe(NULL,2,NULL);


	//Init timeStamps
	timeStamp_last_loc = mrpt::system::now();
	timeStamp_last_laser = mrpt::system::now();

	
	is_motion_command_set = false;		//indicates (true/false) if a motion command is active (To control robot from client)
	return DoRegistrations();

}

bool CMQTTMosquitto::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("pMQTTMosquitto only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CMQTTMosquitto::Iterate()
{
	try
	{
		int rc = loop(-1);
		if(rc!=MOSQ_ERR_SUCCESS)
		{
			//Connection lost with Broker!
			std::cerr << "mosq_err " << rc << " - Reconnecting..." << endl;
			int go = disconnect();
			int g = lib_cleanup();
			int gogo = lib_init();
			
			if ( username_pw_set(broker_username.c_str(), broker_password.c_str()) ) 
				std::cerr << "Error setting username and password.\n";			

			if (enable_TLS_security)
			{
				tlsOptionsSet();		//Set Security options and Data: tls-version, certificates... 
				cout << "Connecting to MQTT Broker with enabled TLS security" << endl;
			}
			else
			{
				cout << "Connecting to MQTT Broker with disabled TLS security" << endl;
			}

			/* Check Broker configuration
				string str_ip;
				if(m_MissionReader.GetConfigurationParam("IP",str_ip))
				{
					broker_host = str_ip.c_str();
					cout << "LOCAL MQTT Broker set on IP = " << broker_host << endl;
				}
				else
				{		
					broker_host = Constants::host;
					std::cout  << "MQTT-IP default: " << broker_host << endl;
				}
			*/
			cout << "Connecting to MQTT Broker at IP: " << broker_host << ":"<< broker_port << endl;
			// Connect the Mosquitto Client
			on_connect(connect(broker_host.c_str(), broker_port, Constants::keepalive_secs));
			//Subscribe to a list of topics 
			on_subscribe(NULL,2,NULL);

		}

		//If a motion command (V,W) is active, check timeout
		if (is_motion_command_set)
		{
			//compute time difference from last motion command
			double seconds = mrpt::system::timeDifference(time_last_motion_command,mrpt::system::now());
			if (seconds > 0.2)
			{
				//Cancel motion
				is_motion_command_set = false;
				//! @moos_publish CANCEL_NAVIGATION Cancel the current navigation and return control to client (Pilot)				
				m_Comms.Notify("CANCEL_NAVIGATION", '1');
			}
		}

		return true;
    }
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}

}


bool CMQTTMosquitto::OnConnectToServer()
{
   // DoRegistrations();
    return true;
}


bool CMQTTMosquitto::DoRegistrations()
{
	//! @moos_subscribe LOCALIZATION
	AddMOOSVariable( "LOCALIZATION", "LOCALIZATION", "LOCALIZATION", 0);
			
	//! @moos_subscribe GRAPH
	AddMOOSVariable( "GRAPH", "GRAPH", "GRAPH", 0);			

	//! @moos_subscribe ERROR_MSG
	AddMOOSVariable( "ERROR_MSG", "ERROR_MSG", "ERROR_MSG", 0);

	//! @moos_subscribe CANCEL_NAVIGATION
	AddMOOSVariable( "CANCEL_NAVIGATION", "CANCEL_NAVIGATION", "CANCEL_NAVIGATION", 0);

	//! @moos_subscribe LASER1
	AddMOOSVariable( "LASER1", "LASER1", "LASER1", 0);
	
	//! @moos_subscribe ROBOT_TOPOLOGICAL_PLACE
	AddMOOSVariable( "ROBOT_TOPOLOGICAL_PLACE", "ROBOT_TOPOLOGICAL_PLACE", "ROBOT_TOPOLOGICAL_PLACE", 0);

	//! @moos_subscribe ROBOT_CONTROL_MODE
	AddMOOSVariable( "ROBOT_CONTROL_MODE", "ROBOT_CONTROL_MODE", "ROBOT_CONTROL_MODE", 0);

	//! @moos_subscribe GET_PATH
	AddMOOSVariable( "GET_PATH", "GET_PATH", "GET_PATH", 0);	

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0);

    RegisterMOOSVariables();
    return true;
}


bool CMQTTMosquitto::OnNewMail(MOOSMSG_LIST &NewMail)
{
	string message="";
	for (MOOSMSG_LIST::iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;		

		//Send localization to Client via MQTT topic
		// LOCALIZATION x y phi
		if( it->GetName()=="LOCALIZATION")
		{
			if( mrpt::system::timeDifference(timeStamp_last_loc, mrpt::system::now()) > (1/localizationRate) )
			{
				message=m.GetString(); // localization (x y phi)

				//Get aditionaly the Node Location (topological localization)
				CMOOSVariable * pVar = GetMOOSVar("ROBOT_TOPOLOGICAL_PLACE");
				if(pVar)
					message += "|" + pVar->GetStringVal();
				else
					message += "|NULL";
				on_publish(NULL, (broker_username + "/" + "Localization").c_str(), strlen(message.c_str()), message.c_str());


				//Get aditionaly the Working Mode (ROBOT_CONTROL_MODE: 0=Manual=Pilot, 2=Autonomous=OpenMORA)
				CMOOSVariable * pVar2 = GetMOOSVar("ROBOT_CONTROL_MODE");
				if(pVar2)
				{
					double current_mode = pVar2->GetDoubleVal();
					if( current_mode == 0.0)
						message += "|Manual";
					else if(current_mode == 2.0)
						message += "|Autonomous";
					else
						message += "|Undefined";
				}
				else
					message += "|NULL";

				on_publish(NULL, (broker_username + "/" + "Status").c_str(), strlen(message.c_str()), message.c_str());
				timeStamp_last_loc = mrpt::system::now();
			}
		}

		//Inform the Client that the GRAPH has been updated
		if( it->GetName()=="GRAPH")
		{			
			cout << "Sending New GRAPH to Client" << endl;			
			std::string graph = "Graph";
			graph.append(it->GetString().c_str());
			on_publish(NULL,(broker_username + "/" + "Topology").c_str(),strlen(graph.c_str()),graph.c_str());					
		}

		//Inform the Client of Errors
		if( it->GetName()=="ERROR_MSG")
		{
			std::string graph = it->GetString().c_str();
			on_publish(NULL,(broker_username + "/" + "ErrorMsg").c_str(),strlen(graph.c_str()),graph.c_str());
		}

		//Inform Client about the current Navigation MODE		
		if( it->GetName()=="ROBOT_CONTROL_MODE")
		{			
			cout << "Sending Current Navigation Mode to Client: " << it->GetString() << endl;
			size_t mode = (size_t)(it->GetDouble());
			
			switch(mode)
			{
				case 0:	//Manual mode
					{
					message = "manual|noDestiny";
					break;
					}
				case 2:	//Autonomous mode
					{
					CMOOSVariable * pVar = GetMOOSVar("GET_PATH");
					std::string destiny = "NULL";
					if(pVar)
						destiny = pVar->GetStringVal();
					message = "auto|" + destiny;
					break;
					}
				default:
					cout << "ERROR: Incorrect NavigationMode!!!!" << endl;
			}
			
			on_publish(NULL, (broker_username + "/" + "NavigationMode").c_str(), strlen(message.c_str()), message.c_str());
			cout << endl << "NavigationMode is " << message.c_str() << endl;
		}

		//Send Laser scans to client - 4Hz
		if( it->GetName()=="LASER1")
		{
			if (sendLaserScans)
			{
				if( mrpt::system::timeDifference(timeStamp_last_laser,mrpt::system::now()) > (1/laserScanRate) )
				{
					
					CSerializablePtr obj;					
					mrpt::utils::RawStringToObject(it->GetString(),obj);
					if (obj && IS_CLASS(obj,CObservation2DRangeScan))
					{
						CObservation2DRangeScanPtr laser_obj = CObservation2DRangeScanPtr(obj);
						size_t num_samples = laser_obj->scan.size();
						std::vector<float> laser_readings;
						
						//Change resolution						
						size_t iter_inc = (size_t) (100/laserScanResolution);
						
						for (size_t i=0; i<num_samples; i+=iter_inc )
						{
							laser_readings.push_back( laser_obj->scan[i] );
						}
						num_samples = laser_readings.size();
						
						message = format( "Aperture %.2f#",laser_obj->aperture );
						message.append( format("Nsamples %u#",num_samples) );
						message.append("Values");
						for (size_t i=0; i<num_samples; i++ )
							message.append(format(" %.2f",laser_readings[i]) );
					
						on_publish(NULL, (broker_username + "/" + "LaserScan").c_str(), strlen(message.c_str()), message.c_str());
						timeStamp_last_laser = mrpt::system::now();						
					}
				}
			}
		}

		//shutdown
		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
			this->RequestQuit();		
	}

	UpdateMOOSVariables(NewMail);
    return true;
}


/*---------------------------------------------------------------------------
					MQTT CallBack Methods
------------------------------------------------------------------------------*/

/*Additional method responsible for the csecurity configuration*/ 
void CMQTTMosquitto::tlsOptionsSet()
{
	tls_insecure_set(true);
	int r=tls_opts_set(1,"tlsv1");
	std::cout <<"TLS options:"<<r<<"\n";
	//r=tls_set("cacert.pem","I:/proyectos/proyectoMosquitto/MqttMosquitto/Debug","clientcert.pem","clientkey.pem",NULL);
	r=tls_set(TLS_CAfile_pem.c_str(),TLS_path.c_str(),TLS_CERTfile_pem.c_str(),TLS_KEYfile_pem.c_str(),NULL);
	std::cout <<"tls:"<<r<<"\n";
}

void CMQTTMosquitto::on_connect (int rc)
{    
    if (rc)
	{
		//! @moos_publish MQTT_CONNECT_STATUS Variable contaning the connection status of MQTT
		m_Comms.Notify("MQTT_CONNECT_STATUS", "OFFLINE");
        switch(rc)
		{
            case 1:
                std::cerr << "Connection Refused: unacceptable protocol version\n";
                break;
            case 2:
                std::cerr << "Connection Refused: identifier rejected\n";
                break;
            case 3:
                std::cerr << "Connection Refused: broker unavailable\n";
                break;
            case 4:
                std::cerr << "Connection Refused: bad user name or password\n";
                break;
            case 5:
                std::cerr << "Connection Refused: not authorised\n";
                break;
            default:
                std::cerr << "Connection Refused: unknown reason\n";
                break;
        }
    }
	else
	{
		//! @moos_publish MQTT_CONNECT_STATUS Variable contaning the connection status of MQTT
		m_Comms.Notify("MQTT_CONNECT_STATUS", "ONLINE");
		std::cout << "Connected successfully to " << broker_host << ":" << broker_port << endl;
	}
}

void CMQTTMosquitto::on_message(const mosquitto_message *message)
{
	std::cout << "Got message to topic " << message->topic;
    if (message->payloadlen)
        std::cout << ", payload text is " << message->payload << "\n";
    else
        std::cout << ", payload is (null)\n";

	
    char buf[51];	
	memset(buf, 0, 51*sizeof(char));
    /* Copy N-1 bytes to ensure always 0 terminated. */
    memcpy(buf, message->payload, 50*sizeof(char));
    printf(buf, 50);
	printf("\n");
	const char *aux;
	aux=buf;
    
		
	// TOPOLOGY-COMMANDS
	if(!strcmp(message->topic, (broker_username + "/" +"TopologyCommand").c_str() ))
	{
		string topologyMessage = string(aux);

		//First element is the name of the action to perform in the topology graph
		size_t pos = topologyMessage.find(" ");
		std::string action = topologyMessage.substr(0, pos);
		topologyMessage.erase(0, pos+1);
		
		if (action=="GetTopology")
		{
			CMOOSVariable *GRAPH = GetMOOSVar("GRAPH");
			std::string graph = "Graph";
			graph.append(GRAPH->GetStringVal().c_str());
			on_publish(NULL, (broker_username + "/" + "Topology").c_str(),strlen(graph.c_str()),graph.c_str());
		}
		//ADD_NODE label type x y
		else if(action == "ADD_NODE")
		{
			//! @moos_publish ADD_NODE Variable contaning the info of the new node to be added to the Graph (label type x y)
			m_Comms.Notify(action, topologyMessage);
		}
		//REMOVE_NODE nodeLabel
		else if(action == "REMOVE_NODE")
		{
			//! @moos_publish REMOVE_NODE Variable contaning the info of the node to be removed from the Graph (nodeLabel)
			m_Comms.Notify(action, topologyMessage);
		}
		//ADD_ARC label_nodeA label_nodeB type
		else if(action == "ADD_ARC")
		{
			//! @moos_publish ADD_ARC Variable contaning the info of the new arc to be added to the Graph (label_nodeA label_nodeB type)
			m_Comms.Notify(action, topologyMessage);
		}		
		//REMOVE_ARC nodeFrom nodeTo Arc_type
		else if(action == "REMOVE_ARC")
		{
			//! @moos_publish REMOVE_ARC Variable contaning the info of the arc to be removed from the Graph (nodeFrom nodeTo Arc_type)
			m_Comms.Notify(action, topologyMessage);
		}
		//MOVE_NODE label x y
		else if(action == "MOVE_NODE")
		{
			//! @moos_publish MOVE_NODE Variable contaning the info of the new node position on the Graph (label x y)
			m_Comms.Notify(action, topologyMessage);
		}
		//CHANGE_NODE_LABEL old_label new_label
		else if(action == "CHANGE_NODE_LABEL")
		{
			//! @moos_publish CHANGE_NODE_LABEL Variable contaning the info to change the name of an existing node (old_label new_label)
			m_Comms.Notify(action, topologyMessage);
		}
		//GET_NODE_POSITION node_label
		else if(action == "GET_NODE_POSITION")
		{
			//! @moos_publish GET_NODE_POSITION Variable to request the location (x,y) of an existing node (node_label)
			m_Comms.Notify(action, topologyMessage);
		}
		//LOAD_GRAPH file
		else if(action == "LOAD_GRAPH")
		{
			//! @moos_publish LOAD_GRAPH Variable to request loading the topological graph from file (file)
			m_Comms.Notify(action, topologyMessage);
		}
		//SAVE_GRAPH file
		else if(action == "SAVE_GRAPH")
		{
			//! @moos_publish SAVE_GRAPH Variable to request saving the current topology to file (file)
			m_Comms.Notify(action, topologyMessage);
		}
	}

	
	// NAVIGATION-COMMANDS
	else if(!strcmp(message->topic, (broker_username + "/" +"NavigationCommand").c_str() ))
	{
		string pluginCommand = string(aux);

		//First element is the name of the action to perform
		size_t pos = pluginCommand.find(" ");
		std::string action = pluginCommand.substr(0, pos);
		pluginCommand.erase(0, pos+1);

		if(action == "StopGiraff")
		{
			//! @moos_publish CANCEL_NAVIGATION Cancel the current navigation and return control to client (Manual Mode)
			cout << "Request received to cancel navigation" << endl;
			m_Comms.Notify("CANCEL_NAVIGATION", '1');
		}

		// GoToPoint x y
		if(action == "GoToPoint")
		{
			string str = format("Command for GoToPoint received at: %f",MOOSTime());
			MOOSDebugWrite(str);

			std::deque<std::string> list;
			mrpt::system::tokenize(pluginCommand," ",list);
			if( list.size() >=2 )
			{
				std::string gotox = list.at(0);
				std::string gotoy = list.at(1);				
				const string target = format("[%.03f %.03f]", atof(gotox.c_str()),atof(gotoy.c_str()));
				//! @moos_publish NAVIGATE_TARGET Set destination ([x y]) for robot to go autonomously
				m_Comms.Notify("NAVIGATE_TARGET",target);
				printf("%s\n",target.c_str());
			}
			else
				cout << "[MQTT Error]: GotoPoint command incorrect" << endl;
		}

		// GoToNode label
		else if(action == "GoToNode")
		{
			printf("Navegación solicitada a Nodo: %s\n",pluginCommand.c_str());
			
			//! @moos_publish GO_TO_NODE Request the RobotController to start a reactive navigation to selected node.
			m_Comms.Notify("GO_TO_NODE", pluginCommand );
		}

		// Relocalize x y
		else if(action == "Relocalize")
		{
			//! @moos_publish RELOCALIZE_IN_AREA Request a robot relocalization arround designed point (x y)
			m_Comms.Notify("RELOCALIZE_IN_AREA",pluginCommand);
		}

		// Motion v w
		else if(action == "Motion")
		{
			std::deque<std::string> list;
			mrpt::system::tokenize(pluginCommand," ",list);
			if( list.size() >=2 )
			{
				std::string setV = list.at(0);
				std::string setW = list.at(1);
				
				//Get timeStamp
				time_last_motion_command = mrpt::system::now();
				is_motion_command_set = true;

				//! @moos_publish ROBOT_CONTROL_MODE The robot working mode: 0=Manual=Pilot, 2=Autonomous=OpenMORA		
				m_Comms.Notify("ROBOT_CONTROL_MODE", 2);
				//! @moos_publish MOTION_CMD_V Set robot linear speed
				m_Comms.Notify("MOTION_CMD_V",atof(setV.c_str()));
				//! @moos_publish MOTION_CMD_W Set robot angular speed
				m_Comms.Notify("MOTION_CMD_W",atof(setW.c_str()));
			}
			else
				cout << "[MQTT Error]: Motion command incorrect" << endl;
		}

		// RandomNavigator 0/1
		else if(action == "RandomNavigator")
		{
			//! @moos_publish RANDOM_NAVIGATOR Activate/Deactivate the module to generate random navigations
			m_Comms.Notify("RANDOM_NAVIGATOR",atof(pluginCommand.c_str()) );
		}
	}
	
	// MAPBUILDING-COMMANDS
	else if(!strcmp(message->topic, (broker_username + "/" +"MapBuildingCommand").c_str() ))
	{
		string command = string(aux);

		//First element is the name of the action to perform
		size_t pos = command.find(" ");
		std::string action = command.substr(0, pos);
		command.erase(0, pos+1);

		// Rawlog Start/Stop
		if(action == "Rawlog")		
		{
			if( command == "Start")			
			{
				saving_data = true;
				command = "true";
			}
			else if( command == "Stop")
			{
				saving_data=false;
				command = "false";			
			}
			else
				return;

			//! @moos_publish SAVE_RAWLOG Commands module Rawlog-grabber to start/stop saving data to rawlog file
			m_Comms.Notify("SAVE_RAWLOG",command);
		}
	}

	// ClientACK
	else if(!strcmp(message->topic, (broker_username + "/" +"ClientACK").c_str() ))
	{
		//! @moos_publish PILOT_MQTT_ACK Flag to indicate that communication with client is alive.
		m_Comms.Notify("PILOT_MQTT_ACK","1");
	}
}


void CMQTTMosquitto::on_subscribe(uint16_t mid, int qos_count, const uint8_t *granted_qos) 
{
        for (int i = 0; i < Constants::topic_count; ++i) 
		{           
			std::string GirafName_topic = broker_username + "/" + Constants::topics[i];
			 std::cerr << "Subscribing to " << GirafName_topic << "\n";
			int rc=subscribe(NULL, GirafName_topic.c_str(), qos_count);
            if (rc)
                std::cerr << "Error: failed to subscribe to " << GirafName_topic << ", subscribe returned " << rc << "\n";            
			else		
				std::cerr << "Subscribed to " << GirafName_topic << ", OK!\n";
		}
        time_t rawtime;
        time(&rawtime);
        std::stringstream strm;
        strm << "MQTT topics subscription done: " << ctime(&rawtime) << endl;
}

void CMQTTMosquitto::on_publish(int *mid, const char *topic, int payloadlen, const void *payload)
{
	int n= publish(mid,topic,payloadlen,payload);	
}