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

/**  @moos_module ICP-based SLAM application that taking as input a rawlog file with odommetry and laser scans, generates the resulting map (image, simplemap, etc).
  *  The path to the rawlog-file to be processed can be specifyed by parameter or via a MOOS variable.
  *  This module is a slightly modifiction of the MRPT based ICP SLAM application (icp-slam_main.cpp) from Jose Luis Blanco Claraco
  *  For more information see http://www.mrpt.org/Application:icp-slam
  */

#include "CMapCreator.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/threads.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

// Constructor
CMapCreator::CMapCreator() 
{		
}

// Destructor
CMapCreator::~CMapCreator()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool CMapCreator::OnStartUp()
{
	printf("Setting configuration file parameters\n");
	//! @moos_param   configuration_file   Path to the configuration (.ini) file
	INI_FILENAME	=       m_ini.read_string("","configuration_file","MapCreator_config.ini",true);
	ASSERT_FILE_EXISTS_(INI_FILENAME)
	
	//! @moos_param   rawlog_file   (optional) Path to the rawlog file to process
	rawlog_file	=       m_ini.read_string("","rawlog_file","",false);
	ASSERT_FILE_EXISTS_(rawlog_file)

	//! @moos_param   output_path   (optional) Path to the directory where the output map will be created.
	output_path	=       m_ini.read_string("","output_path","",false);

	cout << "[MapCreator]: Parameters configuration Done " << endl;
	return DoRegistrations();
}
//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool CMapCreator::OnCommandMsg( CMOOSMsg Msg ) 
{
	if(Msg.IsSkewed( MOOSTime())) return true;
	if(!Msg.IsString()) return MOOSFail("This module only accepts string command messages\n");
	const std::string sCmd = Msg.GetString();

	return true;
}

//-------------------------------------
// Iterate()
//-------------------------------------
bool CMapCreator::Iterate()
{
	if( strcmp(rawlog_file.c_str(),"") != 0 )
	{
		try
		{
			printf("Rawlog-file provided. Starting ICP-based SLAM\n");
			// Run:
			MapBuilding_ICP(INI_FILENAME, rawlog_file);

		}
		catch (exception &e)
		{
			setConsoleColor(CONCOL_RED,true);
			cerr << "Program finished for an exception!!  Map not generated" << endl;
			setConsoleColor(CONCOL_NORMAL,true);

			cerr << e.what() << endl;

			mrpt::system::pause();
			return -1;
		}
		catch (...)
		{
			setConsoleColor(CONCOL_RED,true);
			cerr << "Program finished for an untyped exception!!  Map not generated" << endl;
			setConsoleColor(CONCOL_NORMAL,true);

			mrpt::system::pause();
			return -1;
		}
	}
		
	return true; 
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool CMapCreator::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool CMapCreator::DoRegistrations()
{
	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );

	//! @moos_subscribe MAPCREATOR_RAWLOGFILE
	//! @moos_variable MAPCREATOR_RAWLOGFILE The path to a rawlog file to be proecessed by the MapCrator App
	AddMOOSVariable( "MAPCREATOR_RAWLOGFILE", "MAPCREATOR_RAWLOGFILE", "MAPCREATOR_RAWLOGFILE", 0 );

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool CMapCreator::OnNewMail(MOOSMSG_LIST &NewMail)
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


// ------------------------------------------------------
//				MapBuilding_ICP
//  override_rawlog_file: If not empty, use that rawlog
//  instead of that in the config file.
// ------------------------------------------------------
void CMapCreator::MapBuilding_ICP(const string &INI_FILENAME, const string &override_rawlog_file)
{
	MRPT_START

	CConfigFile				iniFile(INI_FILENAME);

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	const string RAWLOG_FILE		 = !override_rawlog_file.empty() ? override_rawlog_file : iniFile.read_string("MappingApplication","rawlog_file","",  /*Force existence:*/ true);
	const unsigned int rawlog_offset = iniFile.read_int("MappingApplication","rawlog_offset",0,  /*Force existence:*/ true);
	const string OUT_DIR_STD		 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  /*Force existence:*/ true);
	const int LOG_FREQUENCY			 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  /*Force existence:*/ true);
	const bool  SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false,  /*Force existence:*/ true);
	const bool  SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false,  /*Force existence:*/ true);
	const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*Force existence:*/ true);
	const float grid_resolution		 = iniFile.read_float("MappingApplication","grid_resolution", 0.05, true);

	bool 	SHOW_PROGRESS_3D_REAL_TIME = false;
	int		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
	bool 	SHOW_LASER_SCANS_3D = true;

	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME, bool,  iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR( SHOW_LASER_SCANS_3D , bool,  iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");

	//output directory
	std::string OD = format( "%s\\%s",output_path.c_str(),OUT_DIR_STD.c_str() );
	cout << OD << endl;
	const char* OUT_DIR = OD.c_str();

	// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
	CMetricMapBuilderICP mapBuilder;

	mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");

	// Construct the maps with the loaded configuration.
	mapBuilder.initialize();

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.options.verbose = true;
    mapBuilder.options.alwaysInsertByClass.fromString( iniFile.read_string("MappingApplication","alwaysInsertByClass","") );


	// Print params:
	printf("Running with the following parameters:\n");
	printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
	printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
	printf(" matchAgainstTheGrid:\t\t\t%c\n", mapBuilder.ICP_options.matchAgainstTheGrid ? 'Y':'N');
	printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);
	printf("  SAVE_3D_SCENE:\t\t\t%c\n", SAVE_3D_SCENE ? 'Y':'N');
	printf("  SAVE_POSE_LOG:\t\t\t%c\n", SAVE_POSE_LOG ? 'Y':'N');
	printf("  CAMERA_3DSCENE_FOLLOWS_ROBOT:\t%c\n",CAMERA_3DSCENE_FOLLOWS_ROBOT ? 'Y':'N');

	printf("\n");

	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();


	// Checks:
	ASSERT_(RAWLOG_FILE.size()>0)
	ASSERT_FILE_EXISTS_(RAWLOG_FILE)

	CTicTac								tictac,tictacGlobal,tictac_JH;
	int									step = 0;
	string								str;
	CSimpleMap							finalMap;
	float								t_exec;
	COccupancyGridMap2D::TEntropyInfo	entropy;

	size_t								rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE.c_str() );


	// Prepare output directory:
	// --------------------------------
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Open log files:
	// ----------------------------------
	CFileOutputStream  f_log(format("%s/log_times.txt",OUT_DIR));
	CFileOutputStream  f_path(format("%s/log_estimated_path.txt",OUT_DIR));
	CFileOutputStream  f_pathOdo(format("%s/log_odometry_path.txt",OUT_DIR));


	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif

	// ----------------------------------------------------------
	//						Map Building - LOOP
	// ----------------------------------------------------------
	CPose2D					odoPose(0,0,0);

	tictacGlobal.Tic();
	for (;;)
	{
		CActionCollectionPtr	action;
		CSensoryFramePtr		observations;
		CObservationPtr			observation;

		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) )
			break; // file EOF

		const bool isObsBasedRawlog = observation.present();
		std::vector<mrpt::slam::CObservation2DRangeScanPtr> lst_current_laser_scans;   // Just for drawing in 3D views

		if (rawlogEntry>=rawlog_offset)
		{
			// Update odometry:
			if (isObsBasedRawlog)
			{
				static CPose2D lastOdo;
				static bool firstOdo = true;
				if (IS_CLASS(observation,CObservationOdometry))
				{
					CObservationOdometryPtr o = CObservationOdometryPtr(observation);
					if (!firstOdo)
						odoPose = odoPose + (o->odometry - lastOdo);

					lastOdo=o->odometry;
					firstOdo=false;
				}
			}
			else
			{
				CActionRobotMovement2DPtr act= action->getBestMovementEstimation();
				if (act)
					odoPose = odoPose + act->poseChange->getMeanVal();
			}

			// Build list of scans:
			if (SHOW_LASER_SCANS_3D)
			{
				// Rawlog in "Observation-only" format:
				if (isObsBasedRawlog)
				{
					if (IS_CLASS(observation,CObservation2DRangeScan))
					{
						lst_current_laser_scans.push_back( CObservation2DRangeScanPtr(observation) );
					}
				}
				else
				{
					// Rawlog in the Actions-SF format:
					for (size_t i=0; ; i++)
					{
						CObservation2DRangeScanPtr new_obs = observations->getObservationByClass<CObservation2DRangeScan>(i);
						if (!new_obs)
						     break; // There're no more scans
						else lst_current_laser_scans.push_back( new_obs );
					}
				}
			}


			// Execute:
			// ----------------------------------------
			tictac.Tic();
			if (isObsBasedRawlog)
					mapBuilder.processObservation( observation );
			else	mapBuilder.processActionObservation( *action, *observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );


			// Info log:
			// -----------
			f_log.printf("%f %i\n",1000.0f*t_exec,mapBuilder.getCurrentlyBuiltMapSize() );

			const CMultiMetricMap* mostLikMap =  mapBuilder.getCurrentlyBuiltMetricMap();

			if (0==(step % LOG_FREQUENCY))
			{
				// Pose log:
				// -------------
				if (SAVE_POSE_LOG)
				{
					printf("Saving pose log information...");
					mapBuilder.getCurrentPoseEstimation()->saveToTextFile( format("%s/mapbuild_posepdf_%03u.txt",OUT_DIR,step) );
					printf("Ok\n");
				}
			}

			// Save a 3D scene view of the mapping process:
			if (0==(step % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3D.present()))
			{
                CPose3D robotPose;
				mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

				COpenGLScenePtr		scene = COpenGLScene::Create();

                COpenGLViewportPtr view=scene->getViewport("main");
                ASSERT_(view);

                COpenGLViewportPtr view_map = scene->createViewport("mini-map");
                view_map->setBorderSize(2);
                view_map->setViewportPosition(0.01,0.01,0.35,0.35);
                view_map->setTransparent(false);

				{
					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-90);
					cam.setElevationDegrees(90);
					cam.setPointingAt(robotPose);
					cam.setZoomDistance(20);
					cam.setOrthogonal();
				}

				// The ground:
				mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
				groundPlane->setColor(0.4,0.4,0.4);
				view->insert( groundPlane );
				view_map->insert( CRenderizablePtr( groundPlane) ); // A copy

				// The camera pointing to the current robot pose:
				if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
				{
				    scene->enableFollowCamera(true);

					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-45);
					cam.setElevationDegrees(45);
					cam.setPointingAt(robotPose);
				}

				// The maps:
				{
					opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
					mostLikMap->getAs3DObject( obj );
					view->insert(obj);

					// Only the point map:
					opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
					if (mostLikMap->m_pointsMaps.size())
					{
                        mostLikMap->m_pointsMaps[0]->getAs3DObject(ptsMap);
                        view_map->insert( ptsMap );
					}
				}

				// Draw the robot:
				CPose3DPDFPtr posePDF =  mapBuilder.getCurrentPoseEstimation();
				CPose3D  curRobotPose;
				posePDF->getMean(curRobotPose);
				{
					opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
					obj->setPose( curRobotPose );
					view->insert(obj);
				}
				{
					opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
					obj->setPose( curRobotPose );
					view_map->insert( obj );
				}

				// Draw laser scanners in 3D:
				if (SHOW_LASER_SCANS_3D)
				{
					for (size_t i=0;i<lst_current_laser_scans.size();i++)
					{
						// Create opengl object and load scan data from the scan observation:
						opengl::CPlanarLaserScanPtr obj = opengl::CPlanarLaserScan::Create();
						obj->setScan(*lst_current_laser_scans[i]);
						obj->setPose( curRobotPose );
						obj->setSurfaceColor(1.0f,0.0f,0.0f, 0.5f);
						// inser into the scene:
						view->insert(obj);
					}
				}

				// Save as file:
				if (0==(step % LOG_FREQUENCY) && SAVE_3D_SCENE)
				{
					// As 3D scene
					CFileGZOutputStream	f( format("%s/buildingmap_%05u.3Dscene",OUT_DIR,step) );
					f << *scene;
					
					//Save MetricMap
					try
					{
						CMultiMetricMap  *TempMetricMap = mapBuilder.getCurrentlyBuiltMetricMap();
						str = format("%s/%05u_MetricMap_",OUT_DIR,step);
						printf("Dumping metric maps to %s_XXX\n", str.c_str() );
						TempMetricMap->saveMetricMapRepresentationToFile( str );
					}
					catch( exception e)
					{
						cout << "Exception saving MetricMap" << e.what() << endl;
					}
				}

				// Show 3D?
				if (win3D)
				{
					opengl::COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
					ptrScene = scene;

					win3D->unlockAccess3DScene();

					// Move camera:
					win3D->setCameraPointingToPoint( curRobotPose.x(),curRobotPose.y(),curRobotPose.z() );

					// Update:
					win3D->forceRepaint();

					sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
				}

				
			}//end-save3DScene


			// Save the memory usage:
			// ------------------------------------------------------------------
			{
				printf("Saving memory usage...");
				unsigned long	memUsage = getMemoryUsage();
				FILE		*f=os::fopen( format("%s/log_MemoryUsage.txt",OUT_DIR).c_str() ,"at");
				if (f)
				{
					os::fprintf(f,"%u\t%lu\n",step,memUsage);
					os::fclose(f);
				}
				printf("Ok! (%.04fMb)\n", ((float)memUsage)/(1024*1024) );
			}

			// Save the robot estimated pose for each step:
			f_path.printf("%i %f %f %f\n",
				step,
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().x(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().y(),
				mapBuilder.getCurrentPoseEstimation()->getMeanVal().yaw() );


			f_pathOdo.printf("%i %f %f %f\n",step,odoPose.x(),odoPose.y(),odoPose.phi());

		} // end of if "rawlog_offset"...

		step++;
		printf("\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",step, (unsigned)rawlogEntry);
	};


	//*************************************************************************************************
	printf("\n---------------- END!! (total time: %.03f sec) ----------------\n",tictacGlobal.Tac());
	//*************************************************************************************************

	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);

	// As SimpleMap
		str = format("%s/_finalmap_.simplemap",OUT_DIR);
		printf("Dumping final map in binary format to: %s\n", str.c_str() );
		mapBuilder.saveCurrentMapToFile(str);
	//As txt,png,limits,etc
		CMultiMetricMap  *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
		str = format("%s/_finalmaps_.txt",OUT_DIR);
		printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
		finalPointsMap->saveMetricMapRepresentationToFile( str );

	//As Image from pointsMap
		SaveAsImage(&mapBuilder,finalMap,grid_resolution,"final");	
	
	if (win3D)
		win3D->waitForKey();

	MRPT_END

	printf("Clearing current rawlog file. Waiting for new rawlog.\n" );
	rawlog_file = "";
}


//-----------------------------
// Save current map as Image
//-----------------------------
void CMapCreator::SaveAsImage(CMetricMapBuilderICP* mapBuilder_p, mrpt::slam::CSimpleMap theMap, const float gridRes, std::string prefix)
{	
	vector<float> x,y;
	float xmin=100,xmax=-100,ymin=100,ymax=-100;
	mapBuilder_p ->getCurrentMapPoints(x,y);
	for (unsigned int i=0; i<x.size(); i++)
	{
		if (x[i] < xmin)
			xmin = x[i];
		if (x[i] > xmax)
			xmax = x[i];
		if (y[i] < ymin)
			ymin = y[i];
		if (y[i] > ymax)
			ymax = y[i];
	}
	
	mrpt::slam::COccupancyGridMap2D grid(xmin, xmax, ymin, ymax, gridRes);
	grid.loadFromSimpleMap(theMap);
	grid.saveAsBitmapFile( format("%s_geometricMap.bmp",prefix.c_str()) );
}