/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose Raul Ruiz Sarmiento <jotaraul@uma.es>                     |
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

/**  @moos_module This module endows the robot the capability of autonomous docking.
  *  When activated, this module will search for a specific pattern in the incoming RGB image.
  *  When the pattern is detected, the robot will move towards it till the "is_charging" event is detected.
  */


#include "CAutoDocking.h"

#include <mrpt/gui.h>
#include <mrpt/system.h>
#include <mrpt/slam/CObservationImage.h>
#include <opencv2/core/core.hpp>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/utils/utils_defs.h>

using namespace std;
using namespace cv;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::slam;
//using namespace mrpt::hwdrivers;


RNG rng(12345);		//Initialize random generator


//-----------------------------------------------------------
//						CAutoDockingCameraCamera
//-----------------------------------------------------------

CAutoDockingCamera::CAutoDockingCamera() 	
{	
}


//-----------------------------------------------------------
//						~CAutoDockingCameraCamera
//-----------------------------------------------------------

CAutoDockingCamera::~CAutoDockingCamera()
{
	cout << "[CAutoDockingCameraCamera] Destroying Autodocking object..." << endl;
		
	// Save statistics to file
	if ( m_save_executions_time )
	{
		ofstream file;

		file.open( "stats.txt", ofstream::app );

		if ( file.is_open() )
		{
			file << m_timeLog.getStatsAsText();
		}

		file.close();	
	}

	cout << "[INFO] Autodocking object destroyed" << endl;
}


//-----------------------------------------------------------
// 						   OnStartUp
//-----------------------------------------------------------
bool CAutoDockingCamera::OnStartUp()
{
	try
	{	
		std::string section = "";
	
		// 1. Get images from:
		input_image_method = m_ini.read_string(section,"input_image_method","openCV",true);
		
		if( input_image_method == "openCV" )
		{
			// Load the camera config block.
			MOOSTrace("Loading camera config block...\n");
			m_camera.loadConfig(m_ini,"");

			// Init device:
			MOOSTrace("Initializing camera...\n");
			m_camera.initialize();
		}
		else if( input_image_method == "openMORA" )
		{
			input_image_variable = m_ini.read_string(section,"input_image_variable","CAMERA1",true);
			cout << "Getting Images from OpenMORA variable: " << input_image_variable << endl;
		}
		else
		{
			cout << "[CAutodockingCamera] ERROR Image source not recognized!" << endl;
			return false;
		}


		//Start as inactive and not initialized
		module_active = false;
		init = false;
		
		return DoRegistrations();
	}
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR**") + string(e.what())).c_str() );
	}
}


//-----------------------------------------------------------
// 						   get_new_image
//-----------------------------------------------------------
bool CAutoDockingCamera::get_new_image(cv::Mat &frame)
{
	if( input_image_method == "openCV" )
	{
		//Access the camera directly using OpenCV
		m_camera.doProcess();
	
		mrpt::hwdrivers::CGenericSensor::TListObservations lstObjs;
		m_camera.getObservations(lstObjs);

		if (lstObjs.empty())
		{
			cout << "NORRRRRRRRRRRR" << endl;
			return false;
		}

		//process last image observation
		mrpt::hwdrivers::CGenericSensor::TListObservations::iterator it = lstObjs.begin();
		
		CSerializablePtr obj = it->second;
		if( IS_DERIVED(obj,CObservationImage) )
		{
			CObservationImagePtr obs = CObservationImagePtr(obj);
			//obs->image.grayscaleInPlace();
			//obs->image.resize(640,480,3,true);
			cv::Mat frame_temp = obs->image.getAs<IplImage>();
			cv::Size s(640,480);
			cv::resize(frame_temp,frame,s);
			
			//Display image (debug)
			//cv::Mat rgb(obs->image.getAs<IplImage>() );
			//cv::namedWindow( "GETimg", cv::WINDOW_NORMAL );// Create a window for display. //WINDOW_AUTOSIZE
			//cv::imshow( "GETimg",frame );	                  // Show our image inside it.
			//cv::waitKey(0); // waits to display frame
			return true;
		}
		else
			cout << "[MonoCamera] Observation is not CObservationImage" << endl;

		return false;
	}
	else if( input_image_method == "openMORA" )
	{
		//Read a new image using the OpenMORA variables
		mrpt::slam::CObservationImagePtr image_obs;
		CMOOSVariable *pVar;

		pVar = GetMOOSVar("CAMERA_H");
		if(pVar)
			cout << "CAMERA_H = " << pVar->GetDoubleVal() << endl;
		if(pVar->IsFresh())
			cout << "CAMERA_H IS FRESH!"  << endl << endl;

		pVar = GetMOOSVar("CAMERA_W");
		if(pVar)
			cout << "CAMERA_W = " << pVar->GetDoubleVal() << endl;
		if(pVar->IsFresh())
			cout << "CAMERA_W IS FRESH!"  << endl << endl;

								
		pVar = GetMOOSVar(input_image_variable);
		if( (pVar) && (pVar->IsFresh()) )
		{
			pVar->SetFresh(false);
			cout << "Variable readed!" << endl;
			CSerializablePtr obj;
			mrpt::utils::RawStringToObject(pVar->GetStringVal(),obj);
			
			if( IS_CLASS(obj, CObservationImage) )
			{
				image_obs = mrpt::slam::CObservationImagePtr(obj);				
				frame = image_obs->image.getAs<IplImage>();
				return true;				
			}
			else
				cout << "[CAutoDockingCameraCamera]: ERROR: variable is not CObservationImage'" << endl;
		}
		else if( !pVar )
		{
			cout << "[CAutoDockingCameraCamera]: ERROR: Cannot read variable '" << input_image_variable << "'" << endl;
			mrpt::system::sleep(1000);
		}
		else
		{
			cout << "[CAutoDockingCameraCamera]: ERROR: Variable '" << input_image_variable << "' is NOT fresh." << endl;
			mrpt::system::sleep(1000);
		}
		return false;
	}
	else if( input_image_method == "socket")
	{
		//To be done, read image from socket.
		return false;
	}
}
//-----------------------------------------------------------
// 						   Init
//-----------------------------------------------------------
bool CAutoDockingCamera::Init()
{
	try
	{
		cout << "[CAutoDockingCamera]: Configuring Module..." << endl << endl;
		std::string section = "";
		cv::Mat frame;
		
		while( !get_new_image(frame) )
			mrpt::system::sleep(100);

		// 2. Retrieve camera resolution
		cout << "config. Img Resolution" << endl;
		m_frame_features.height = frame.rows;
		m_frame_features.width = frame.cols;
	
		cout << "config. Display" << endl;
		// 3.1 Display windows configuration(useful for debug)
		m_display.BGR							= m_ini.read_bool(section,"display_BGR",false,true);
		m_display.hsv_final_masks				= m_ini.read_bool(section,"display_hsv_final_masks",false,true);
		m_display.hsv_partial_masks				= m_ini.read_bool(section,"display_hsv_partial_masks",false,true);
		m_display.hsv_contours					= m_ini.read_bool(section,"display_hsv_contours",false,true);
		m_display.hough_circles_segmentation	= m_ini.read_bool(section,"display_hough_circle_segmentation",false,true); 
		m_display.partial_candidates			= m_ini.read_bool(section,"display_partial_candidates",false,true);
		m_display.pattern_detection				= m_ini.read_bool(section,"display_pattern_detection",false,true);
		m_display.canny_results					= m_ini.read_bool(section,"display_canny_results",false,true);

		cout << "config. Robot Movements" << endl;
		// 3.2 Movements configuration
		
		movement_mode							= m_ini.read_int(section,"movement_mode",1,true);
		m_max_linear_speed						= m_ini.read_double(section,"max_linear_speed",0.4,true);
		m_max_angular_speed						= m_ini.read_double(section,"max_angular_speed",0.4,true);

		m_degrees_turning_per_step				= m_ini.read_double(section,"degrees_turning_per_step",0,true);
		m_meters_moving_per_step				= m_ini.read_double(section,"meters_moving_per_step",0,true);

		m_kp_move_in_straight_line					= m_ini.read_double(section,"kp_move_in_straight_line",0,true);
		m_kp_consider_as_center_distance_relaxation = m_ini.read_double(section,"kp_consider_as_center_distance_relaxation",0,true);
		m_kp_turn									= m_ini.read_double(section,"kp_turn",0,true);

		m_angleFarMode							= m_ini.read_double(section,"angle_far_mode",0,true);
		m_angleNearMode							= m_ini.read_double(section,"angle_near_mode",0,true);

		// 3.3 General thresholds
		m_threshold_considered_as_center		= m_ini.read_double(section,"threshold_considered_as_center",0,true);	

		// 3.4 Colours to detect configuration
		m_colours_to_detect						= m_ini.read_uint64_t(section,"colours_to_detect",0,true);	

		m_hsv_segmentation_parameters.m_hsv_thresholds.resize( m_colours_to_detect );

		for ( size_t i = 0; i < m_colours_to_detect; i++ )
			m_ini.read_vector( section, mrpt::format("hsv_thresholds_%d",i),vector<int>(),m_hsv_segmentation_parameters.m_hsv_thresholds[i],true );
		
		cout << "config. AutoDocking Pattern" << endl;
		// 3.5 Pattern configuration
		m_ini.read_vector( section, "pattern_order", vector_size_t(), m_pattern_conf.m_pattern_order, true );	
		m_ini.read_vector( section, "max_inclination_fitted_rect",vector<float>(0),m_pattern_conf.m_max_inclination_fitted_rect,true );
		m_ini.read_vector( section, "max_radius_difference_ratio",vector<float>(0),m_pattern_conf.m_max_radius_difference_ratio,true );
		m_ini.read_vector( section, "min_circle_radius",vector<float>(0),m_pattern_conf.m_min_circle_radius,true );
		m_ini.read_vector( section, "max_circle_radius",vector<float>(0),m_pattern_conf.m_max_circle_radius,true );
		m_ini.read_vector( section, "max_error_adjusting_a_line",vector<float>(0),m_pattern_conf.m_max_error_adjusting_a_line,true );
	
		vector<float> aux_thresholds_size_relation;
		m_ini.read_vector( section, "thresholds_size_relation",vector<float>(0),aux_thresholds_size_relation,true );
		m_pattern_conf.m_thresholds_size_relation.resize(2);
		m_pattern_conf.m_thresholds_size_relation[FAR_MODE].push_back( aux_thresholds_size_relation[0] );
		m_pattern_conf.m_thresholds_size_relation[FAR_MODE].push_back( aux_thresholds_size_relation[1] );
		m_pattern_conf.m_thresholds_size_relation[NEAR_MODE].push_back( aux_thresholds_size_relation[2] );
		m_pattern_conf.m_thresholds_size_relation[NEAR_MODE].push_back( aux_thresholds_size_relation[3] );
		m_ini.read_vector( section, "final_pattern_row",vector<size_t>(0),m_pattern_conf.m_final_pattern_row,true );
		m_ini.read_vector( section, "final_pattern_col",vector<size_t>(0),m_pattern_conf.m_final_pattern_col,true );

		if ( m_pattern_conf.m_thresholds_size_relation.size() != 2 )
			throw std::logic_error("The size of the thresholds_size_relation vector must be 2");

		// 3.6 General configuration
		m_segmentation_algorithm		= m_ini.read_uint64_t(section,"segmentation_algorithm",0,true);	
		m_tries_detecting_pattern		= m_ini.read_uint64_t(section,"tries_detecting_pattern",0,true);	
		m_save_histogram_information	= m_ini.read_bool(section,"save_histogram_information",false,true);
		m_save_executions_time			= m_ini.read_bool(section,"save_executions_time",false,true);
		m_save_pattern_constrains		= m_ini.read_bool(section,"save_pattern_constrains",false,true);
		m_distance_for_changing_mode	= m_ini.read_uint64_t(section,"distance_for_changing_mode",0,true);

		// 3.7 Calculate rows and cols corresponding to the initial ROI
		float initial_ROI_row_upper_limit_percentaje =	m_ini.read_float(section,"initial_ROI_row_upper_limit",0,true);	
		float initial_ROI_row_lower_limit_percentaje =	m_ini.read_float(section,"initial_ROI_row_lower_limit",0,true);	
		float initial_ROI_col_left_limit_percentaje =	m_ini.read_float(section,"initial_ROI_col_left_limit",0,true);	
		float initial_ROI_col_right_limit_percentaje =	m_ini.read_float(section,"initial_ROI_col_right_limit",0,true);	
	
		m_frame_features.initial_ROI_row_upper_limit =	floor( (m_frame_features.height-1) * ( initial_ROI_row_upper_limit_percentaje / 100 ) );
		m_frame_features.initial_ROI_row_lower_limit =	floor( (m_frame_features.height-1) * ( initial_ROI_row_lower_limit_percentaje / 100 ) );
		m_frame_features.initial_ROI_col_left_limit	=	floor( (m_frame_features.width-1) * ( initial_ROI_col_left_limit_percentaje / 100 ) );
		m_frame_features.initial_ROI_col_right_limit =	floor( (m_frame_features.width-1) * ( initial_ROI_col_right_limit_percentaje / 100 ) );

		m_frame_features.ROI_width = m_frame_features.initial_ROI_col_right_limit - m_frame_features.initial_ROI_col_left_limit + 1;
		m_frame_features.ROI_height = m_frame_features.initial_ROI_row_lower_limit - m_frame_features.initial_ROI_row_upper_limit + 1;

		// 3.8 Hough segmentation paramters
		hough_segmentation_parameters &hough_params = m_hough_segmentation_parameters;
		hough_params.inverse_ratio_accumulator_resolution	= m_ini.read_float(section,"inverse_ratio_accumulator_resolution",0,true);	
		hough_params.min_dist								= m_ini.read_float(section,"min_dist",0,true);	
		hough_params.higher_canny_threshold					= m_ini.read_float(section,"higher_canny_threshold",0,true);	
		hough_params.accumulator_threshold					= m_ini.read_float(section,"accumulator_threshold",0,true);	
		hough_params.min_radius								= m_ini.read_float(section,"min_radius",0,true);	
		hough_params.max_radius								= m_ini.read_float(section,"max_radius",0,true);	
		hough_params.gaussian_blur_size						= m_ini.read_uint64_t(section,"gaussian_blur_size",0,true);
		hough_params.gaussian_blur_sigma_1					= m_ini.read_uint64_t(section,"gaussian_blur_sigma_1",0,true);
		hough_params.gaussian_blur_sigma_2					= m_ini.read_uint64_t(section,"gaussian_blur_sigma_2",0,true);

		// 3.9 Canny closed contours segmentation
		canny_segmentation_parameters &canny_params = m_canny_segmentation_parameters;

		canny_params.gaussian_blur_size		= m_ini.read_uint64_t(section,"gaussian_blur_size",0,true);
		canny_params.gaussian_blur_sigma_1	= m_ini.read_uint64_t(section,"gaussian_blur_sigma_1",0,true);
		canny_params.gaussian_blur_sigma_2	= m_ini.read_uint64_t(section,"gaussian_blur_sigma_2",0,true);
		canny_params.higher_threshold		= m_ini.read_float(section,"higher_canny_threshold",0,true);

		// 3.10 Tracking window
		m_tracking_window.window_size	= m_ini.read_float(section,"window_size",0,true);
		m_ini.read_vector( section, "window_adjustment_straight_line",vector<float>(0),m_tracking_window.v_window_adjustment_straight_line,true );
		m_ini.read_vector( section, "window_adjustment_turn_left",vector<float>(0),m_tracking_window.v_window_adjustment_turn_left,true );
		m_ini.read_vector( section, "window_adjustment_turn_right",vector<float>(0),m_tracking_window.v_window_adjustment_turn_right,true );
		
	
		// 5. Establish the initial state
		m_last_movement.last_movement	= TURN_PATTERN_WAS_NOT_DETECTED;
		m_last_movement.last_turn_mov_q	= 0;
		m_last_movement.last_straight_mov_q	= 0;

		m_detected_pattern.was_detected = false;

		m_turn_complete_tester.reset();	// Reset the turn complete without detect the pattern tester

		m_mode = FAR_MODE;

		cout << "[CAutoDockingCameraCamera]: Initialization process succesfully completed" << endl;

		//mrpt::system::sleep(1000);
		init = true;
		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR Iterate**") + string(e.what())).c_str() );
	}
}


bool CAutoDockingCamera::OnCommandMsg( CMOOSMsg Msg )
{
    if(Msg.IsSkewed(MOOSTime()))
        return true;

    if(!Msg.IsString())
        return MOOSFail("Robot_Control_Manager only accepts string command messages\n");

    std::string sCmd = Msg.GetString();

    MOOSTrace("COMMAND RECEIVED: %s\n",sCmd.c_str());

    return true;
}

bool CAutoDockingCamera::Iterate()
{
	try
	{
		if( !init )
			Init();
			
		if( module_active )
		{
			// Go Docking Starts!
			changeMode( FAR_MODE );

			//-------------------------------------------
			// Turn or MoveStright! Only one at a time.
			//-------------------------------------------
			if( movement_mode == 0 )
			{				
				if ( performGoodOrientation() )	// Check if we can find the docking station and it is reachable!
				{
					while( module_active )
					{	
						moveInStraightLineABit(); 

						if( checkGoalAchieved() )
						{
							cout << "Goal achieved! =)" << endl;
							break; // We have reached the goal!
						}

						if ( !performGoodOrientation() ) // If not, check if well oriented
							break; // Launch dock station missed!					
					}
				}
			}
			//-------------------------------------------
			// Linear and Angular speed simultaneously
			//-------------------------------------------
			else if( movement_mode == 1 )
			{
				while( module_active && !checkGoalAchieved() )
				{	
					moveABit();
				}
			}

			// Goal Achieved!
			// Stop robot and deactivate the module
			m_Comms.Notify("CANCEL_NAVIGATION", 1.0);
			module_active = false;
		}
		else
			printf(".");

		return true;
	}
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR Iterate**") + string(e.what())).c_str() );
	}
}


bool CAutoDockingCamera::OnConnectToServer()
{
    DoRegistrations();
    return true;
}


bool CAutoDockingCamera::DoRegistrations()
{
	//! @moos_subscribe input_image_variable
	AddMOOSVariable( input_image_variable, input_image_variable, input_image_variable, 0);
	//AddMOOSVariable( "MONOCAMERA_1", "MONOCAMERA_1", "MONOCAMERA_1", 0);

	//! @moos_subscribe CAMERA_H
	AddMOOSVariable( "CAMERA_H", "CAMERA_H", "CAMERA_H", 0);
	AddMOOSVariable( "CAMERA_W", "CAMERA_W", "CAMERA_W", 0);

	//! @moos_subscribe TILT_CURRENT_ANGLE_FROM_HOME
	AddMOOSVariable( "TILT_CURRENT_ANGLE_FROM_HOME", "TILT_CURRENT_ANGLE_FROM_HOME", "TILT_CURRENT_ANGLE_FROM_HOME", 0);
	
	//! @moos_subscribe BATTERY_IS_CHARGING
	AddMOOSVariable( "BATTERY_IS_CHARGING", "BATTERY_IS_CHARGING", "BATTERY_IS_CHARGING", 0);

	//! @moos_subscribe ODOMETRY_OBS
	AddMOOSVariable( "ODOMETRY_OBS", "ODOMETRY_OBS", "ODOMETRY_OBS", 0);
	
	//! @moos_subscribe PARKING
	//! @moos_var PARKING Variable that indicates when the robot should start the Docking process.
	AddMOOSVariable( "PARKING", "PARKING", "PARKING", 0 );

	//! @moos_subscribe SHUTDOWN
	AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0);

	RegisterMOOSVariables();
    return true;
}

bool CAutoDockingCamera::OnNewMail(MOOSMSG_LIST &NewMail)
{
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
	    const CMOOSMsg &m = *it;

		if( MOOSStrCmp(m.GetKey(),"PARKING") )
		{
			if( m.GetDouble() == 1.0 )
			{
				printf("[AutoDockingCamera] Module is now ACTIVE\n");
				module_active = true;
			}
			else
			{
				printf("[AutoDockingCamera] Module is now INACTIVE\n");
				module_active = false;
			}
		}

		if( MOOSStrCmp(m.GetKey(),input_image_variable) && module_active )
		{			
			CSerializablePtr obj;
			mrpt::utils::RawStringToObject(m.GetString() ,obj);
			
			if( IS_CLASS(obj, CObservationImage) )
			{
				mrpt::slam::CObservationImagePtr image_obs = mrpt::slam::CObservationImagePtr(obj);
				last_image_obs = image_obs->image.getAs<IplImage>();
				
				if( !last_image_obs.empty() )
				{
					cout << "Got New Image! " << last_image_obs.dims << "dims. " << last_image_obs.cols << " x " << last_image_obs.rows << endl;
					
					cv::namedWindow( "MORAimg", cv::WINDOW_AUTOSIZE );// Create a window for display.
					imshow( "MORAimg", last_image_obs );                   // Show our image inside it.
					cv::waitKey(1); // waits to display frame
				}
				else
					cout << "[CAutoDockingCameraCamera]: ERROR: Image EMPTY'" << endl;
			}
			else
				cout << "[CAutoDockingCameraCamera]: ERROR: variable is not CObservationImage'" << endl;
		}


		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{
			this->RequestQuit();			
		}
	}

	UpdateMOOSVariables(NewMail);
    return true;
}




//-----------------------------------------------------------
//					constrain_radius
//-----------------------------------------------------------
bool CAutoDockingCamera::constrain_radius( const float &radius ) const
{
	if ( ( radius < m_pattern_conf.m_max_circle_radius[m_mode] ) && ( radius > m_pattern_conf.m_min_circle_radius[m_mode] ) ) 
		return true;
	else
		return false;
}

//-----------------------------------------------------------
//					 obtainMaskMat
//-----------------------------------------------------------

void CAutoDockingCamera::obtainMaskMat( const Mat &channel_mat, const size_t &channel_num, int inf_threshold, int sup_threshold, Mat &mask, const size_t &colour_index )
{
	int threshold_value;
	int max_BINARY_value = 255;

	Mat mask_inf;

	if ( inf_threshold != -1 )
	{
		threshold_value = inf_threshold;
		threshold( channel_mat, mask_inf, 
			threshold_value, max_BINARY_value,3 );
	}

	Mat mask_sup;

	if ( sup_threshold != -1 )
	{
		threshold_value = sup_threshold;
		threshold( (inf_threshold != -1) ? mask_inf : channel_mat, 
			mask_sup, threshold_value, max_BINARY_value,4 );
	}

	const int from_to[] = { channel_num, 0 };
	mixChannels( (sup_threshold != -1 ) ? &mask_sup : &mask_inf, 1, &mask, 1, from_to, 1 );

	if ( m_display.hsv_partial_masks )
	{
		imshow( mrpt::format("Channel %d mask of colour %d", channel_num, colour_index), mask);
		cvWaitKey( 5 );	
	}
}


//-----------------------------------------------------------
//					obtainSegmentedMat
//-----------------------------------------------------------

void CAutoDockingCamera::obtainSegmentedMat( const Mat &frame, Mat &segmented, const size_t &colour_index )
{
	//int threshold_value; // threshold value for the threshold function
	//int max_BINARY_value = 255; // value for setting the pixels which fullfil the threshold

	vector<bool> mask_used(3,false); // Position 0: hue, 1: saturation, 2: value

	Mat hue_mask(frame.rows, frame.cols, CV_8UC1 ), 
		sat_mask(frame.rows, frame.cols, CV_8UC1 ), 
		value_mask(frame.rows, frame.cols, CV_8UC1 );

	hsv_segmentation_parameters &hsv_params = m_hsv_segmentation_parameters;

	if ( hsv_params.m_hsv_thresholds[colour_index][0] != -1 || hsv_params.m_hsv_thresholds[colour_index][1] != -1 ) mask_used[0] = true;
	if ( hsv_params.m_hsv_thresholds[colour_index][2] != -1 || hsv_params.m_hsv_thresholds[colour_index][3] != -1 ) mask_used[1] = true;
	if ( hsv_params.m_hsv_thresholds[colour_index][4] != -1 || hsv_params.m_hsv_thresholds[colour_index][5] != -1 ) mask_used[2] = true;

	if ( !mask_used[0] && !mask_used[1] && !mask_used[2] )
		throw std::logic_error("While segmenting: no threshodls specified");
	
	// Convert the frame from BGR to HSV
	Mat hsv_mat;	
	cvtColor(frame, hsv_mat, CV_BGR2HSV);

	//imshow("HSV",hsv_mat);

	// Obtain masks

	vector<Mat> masks(3);
	for ( size_t n_channel = 0; n_channel < 3; n_channel++ )
	{
		if ( mask_used[n_channel] )
		{
			masks[n_channel].create(hsv_mat.rows, hsv_mat.cols, CV_8UC1);
			
			Mat channel( hsv_mat.rows, hsv_mat.cols, CV_8UC3 );
			int from_to[] = { n_channel, n_channel }; // Channel n of hsv_mat to channel 0 of channel
			mixChannels( &hsv_mat, 1, &channel, 1, from_to, 1 );

			obtainMaskMat( channel, n_channel, 
							hsv_params.m_hsv_thresholds[colour_index][n_channel*2], 
							hsv_params.m_hsv_thresholds[colour_index][n_channel*2+1], 
							masks[n_channel], colour_index );
		}
		else
		{
			masks[n_channel] = Mat::ones(hsv_mat.rows, hsv_mat.cols, CV_8UC1);
		}
	}
	
	Mat intermediate_mask;
	Mat final_mask;

	masks[0].copyTo(intermediate_mask,masks[1]);
	intermediate_mask.copyTo(final_mask,masks[2]);
	threshold( final_mask, final_mask, 0, 255, 0);
	
	if ( m_display.hsv_partial_masks )
		imshow(mrpt::format("mask for colour %d",colour_index), final_mask);
			
	Mat aux_segmented;
	frame.copyTo( aux_segmented, final_mask );
	
	if ( m_display.hsv_final_masks )
		imshow(mrpt::format("Final for colour %d",colour_index), aux_segmented);
	

	final_mask.copyTo( segmented );				
	

	if ( m_display.hsv_partial_masks || m_display.hsv_final_masks )
		cvWaitKey( 5 );
}


//-----------------------------------------------------------
//					debug_pattern_detection
//-----------------------------------------------------------

void CAutoDockingCamera::debug_pattern_detection( vector<vector<TCircle>> &candidates, const size_t &i_colour )
{
	Mat drawing = Mat::zeros( m_frame_features.ROI_height, m_frame_features.ROI_width, CV_8UC3 );

	size_t N = candidates.size();

	for( unsigned int i = 0; i < N; i++ )
	{
		for( unsigned int j = 0; j < candidates[i].size(); j++ )
		{
			Scalar colour = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//drawContours( drawing, contours_poly[i], j, colour, 1, 8, vector<Vec4i>(), 0, Point() );
			//rectangle( drawing, boundRect[i][j].tl(), boundRect[i][j].br(), colour, 2, 8, 0 );
			circle( drawing, candidates[i][j].center, (int)candidates[i][j].radius, colour, 2, 8, 0 );
		}
	}

	// Show in a window
	imshow( mrpt::format("Candidates step: %d", i_colour), drawing );
	cvWaitKey(5);	
}


//-----------------------------------------------------------
//				computeErrorAdjustingALine
//-----------------------------------------------------------

float CAutoDockingCamera::computeErrorAdjustingALine( const TNCircles &v_final_candidates, TLine2D &adjusted_line )
{
	vector<TPoint2D>	v_candidates_centers;

	v_final_candidates.getCentersAsTPoint2DVector( v_candidates_centers );
	mrpt::math::getRegressionLine( v_candidates_centers, adjusted_line );

	float mean_distance = 0;

	size_t N = v_candidates_centers.size();

	for ( size_t i = 0; i < N ; i++ )
		mean_distance += adjusted_line.distance( v_candidates_centers[i] );

	mean_distance = mean_distance / N;

	return mean_distance;
}


//-----------------------------------------------------------
//					  searchPattern_afterCanny
//-----------------------------------------------------------

void CAutoDockingCamera::searchPattern_afterCanny( const vector<vector<Point2f>> &v_center, 
										     const vector<vector<float>> v_radius )
{
	// Currently not used
}


//-----------------------------------------------------------
//					  searchPattern_afterHsvOrHough
//-----------------------------------------------------------
void CAutoDockingCamera::searchPattern_afterHsvOrHough( vector<vector<Point2f>> &v_center, 
												  vector<vector<float>> &v_radius )
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Search pattern");

		// Useful vables
		pattern_conf &pattern = m_pattern_conf;

		size_t n_pattern_components = m_pattern_conf.m_pattern_order.size(); // Total of components to search
		size_t &n_colours			= m_colours_to_detect;

		vector<vector<TCircle>> candidates; // Vector of vectors of candidates regions for each component in the pattern		
		vector<vector<TNCircles>> v_pair_of_candidates;

		// Resize vectors for efficieny

		candidates.resize( n_pattern_components ); // One vector of circles for each pattern component
		v_pair_of_candidates.resize( n_pattern_components-1 );
	
		// Here we go!!!! Pattern detection is a piece of cake!

		// 1. First, delete candidates that don't fulfill a certain radius size constrain
		//    To do this here save much computational time

		for ( size_t i_colour = 0; i_colour < n_colours; i_colour++ )
		{
			for ( int i_circle = v_center[i_colour].size()-1; i_circle >= 0; i_circle-- ) 
			{
				float radius = v_radius[i_colour][i_circle];

				if ( !constrain_radius( radius ) )
				{
					v_center[i_colour].erase( v_center[i_colour].begin() + i_circle );
					v_radius[i_colour].erase( v_radius[i_colour].begin() + i_circle );
				}
			}
		}

		// 2. Now apply some extra constrains to candidates, obtaining a set of them in the candidates object

		for ( size_t i_pattern_component = 0; i_pattern_component < n_pattern_components; i_pattern_component++ )
		{		
			size_t i_colour = pattern.m_pattern_order[i_pattern_component];

			if ( i_pattern_component == 0 ) // The first pattern component for analyzing
			{
				candidates[0].resize( v_center[i_colour].size() );

				for ( size_t i_circle = 0; i_circle < v_center[i_colour].size(); i_circle++ ) 
				{
					float	&radius = v_radius[i_colour][i_circle];
					Point2f &center = v_center[i_colour][i_circle];

					candidates[0][i_circle] = TCircle( center, radius );					
				}
	
				if ( m_display.partial_candidates )
					debug_pattern_detection( candidates, i_pattern_component );
			}
			else // The second colour, third...
			{			
				// For each region of this color, check if its a candidate region ...
				for ( size_t i_circle = 0; i_circle < v_center[i_colour].size(); i_circle++ ) 
				{
					float radius = v_radius[i_colour][i_circle];

					CPoint2D center( v_center[i_colour][i_circle].x, v_center[i_colour][i_circle].y );	

					// ... analyzing its relation with the regions of the previous color
					for ( size_t i_previous_circle = 0; i_previous_circle < candidates[i_pattern_component-1].size(); i_previous_circle++ ) 
					{
						// Obtain distance between both regions
						float distance = center.distance2DTo( candidates[i_pattern_component-1][i_previous_circle].center.x, candidates[i_pattern_component-1][i_previous_circle].center.y );
						float previus_candidate_radius = candidates[i_pattern_component-1][i_previous_circle].radius;
						float radius_difference_ratio = previus_candidate_radius / radius; 

						// This region must be at the right of the previous one
						if ( ( v_center[i_colour][i_circle].x > candidates[i_pattern_component-1][i_previous_circle].center.x ) 
							// Check the difference ratio between both radius
							&& ( ( abs( radius_difference_ratio -1) < pattern.m_max_radius_difference_ratio[m_mode] ) ) 
							// Check the size/distance ratio
							&& ( ( pattern.m_thresholds_size_relation[m_mode][0] < distance/radius ) && ( distance/radius < pattern.m_thresholds_size_relation[m_mode][1] ) ) )												
						{							
							TCircle &c1 = candidates[i_pattern_component-1][i_previous_circle];
							TCircle &c2 = TCircle(v_center[i_colour][i_circle], radius );
								candidates[i_pattern_component].push_back( c2 );
								v_pair_of_candidates[i_pattern_component-1].push_back( TNCircles(c1,c2) );
								break;		
						}						
					}
				} // end for

				if ( m_display.partial_candidates )
					debug_pattern_detection( candidates, i_pattern_component );
			}
			if ( ! candidates[i_pattern_component].size() ) // If any candidate region found for this colour, finish!!!!
			break;		
		}

		//cout << "Obtaining final candidates..." << endl;

		/*for ( size_t i = 0; i < v_pair_of_candidates.size(); i++ )
		{
			cout << "Pattern component " << i << " : " << endl;
			for ( size_t j = 0; j < v_pair_of_candidates[i].size(); j++ )	
			{
				v_pair_of_candidates[i][j].print();
				cout << endl;
			}
			cout << endl;
		}

		if ( candidates[2].size() )
			mrpt::system::pause();*/

		// Construct a vector with all the possible combinations of candidates
		vector<TNCircles> v_final_candidates;
	
		// Check if we have real candidates!
		if ( candidates[n_pattern_components-1].size() )
		{
			if ( n_pattern_components == 2 ) // If there is only two pattern components, the final vector is the same that the first of the vector of pair of candidates
			{
				v_final_candidates = v_pair_of_candidates[0];
			}
			else
			{
				// 1. Build an initial final vector with al the pairs of the last candidates pairs vector, putting the first circle at the first position
				size_t N = v_pair_of_candidates[n_pattern_components-2].size(); 

				for( size_t i = 0; i < N; i++ )
					v_final_candidates.push_back( TNCircles( v_pair_of_candidates[n_pattern_components-2][i][1], v_pair_of_candidates[n_pattern_components-2][i][0] ) );

				/*for ( size_t i = 0; i < v_final_candidates.size(); i++ )
				{
					cout << v_final_candidates[i] << endl;
				}*/

				// 2. For each candidates vector apart of the last, check if the last circle in the final candidates vector exists on it,
				// and add the correspondant pair to the final candidates vector.
				for( int i_pairs = n_pattern_components-3; i_pairs >= 0; i_pairs-- )
				{
					size_t N_of_partial_candidates = v_final_candidates.size();
					size_t N_circles_in_partial_candidate = v_final_candidates[0].size();

					for ( size_t i_partial_candidate = 0; i_partial_candidate < N; i_partial_candidate++ )
					{
						// Num of candidates in this candidates pairs vector
						size_t N_candidates_pair = v_pair_of_candidates[i_pairs].size();
						// Last circle of this candidate in the final vector
						TCircle &last_circle_candidate = v_final_candidates[i_partial_candidate][N_circles_in_partial_candidate-1];
					
						// 3. Check the candidates pairs of this candidates pairs vector that have the second candidate circle
						// equal to the last_circle_candidate
						for ( size_t i_pair_of_candidates = 0; i_pair_of_candidates < N_candidates_pair; i_pair_of_candidates++ )
						{
							TCircle &second_circle_pair_of_candidates = v_pair_of_candidates[i_pairs][i_pair_of_candidates][1];
							TCircle &first_circle_pair_of_candidates = v_pair_of_candidates[i_pairs][i_pair_of_candidates][0];
							//cout << "[comparing]" << last_circle_candidate << " with " << second_circle_pair_of_candidates << endl;

							if ( ( last_circle_candidate == second_circle_pair_of_candidates ) 
								// If we have two or more pattern components of the same colour, the circle could be already 
								// in the partial candidate vector, so check it!
								&& ( !v_final_candidates[i_partial_candidate].contains( first_circle_pair_of_candidates ) ) ) 
							{
								//4. If it is the same, push it in the final candidates vector
								v_final_candidates.push_back( 
									v_final_candidates[i_partial_candidate].push_back( first_circle_pair_of_candidates ) );
							}
						}
					}

					// 5.Delete processed partial candidates
					v_final_candidates.erase( v_final_candidates.begin(), v_final_candidates.begin() + N_of_partial_candidates ); 				
				}
			}
		}

		ofstream fileErrorAdjustingLines;
		ofstream fileDifferenceRatio;
		ofstream fileRadiusDistanceRatio;
		ofstream fileRadius;
		ofstream fileInclination;
		ofstream fileDistanceToPattern;

		if ( m_save_pattern_constrains )
		{
			fileErrorAdjustingLines.open( "Errors adjusting lines.txt", std::ofstream::app );
			fileDifferenceRatio.open( "Difference ratio.txt", std::ofstream::app );	
			fileRadiusDistanceRatio.open( "Radius distance ratio.txt", std::ofstream::app );	
			fileRadius.open( "Radius.txt", std::ofstream::app );
			fileInclination.open( "Inclination.txt", std::ofstream::app );
			fileDistanceToPattern.open( "Position.txt", std::ofstream::app );
		}
	

		// Finally, if we have candidates, apply the straight line condition

		if ( v_final_candidates.size() )
		{
			for ( int i = v_final_candidates.size()-1; i >= 0; i-- )
			{			
				TLine2D line;
				float error = computeErrorAdjustingALine( v_final_candidates[i], line );

				if ( error > pattern.m_max_error_adjusting_a_line[m_mode] )
					v_final_candidates.erase( v_final_candidates.begin()+i );			
				else if ( m_save_pattern_constrains )
				{
					fileErrorAdjustingLines << error << endl;				
				
					double director_vector[2];
					line.getDirectorVector( director_vector );
	
					fileInclination <<  -director_vector[0]/director_vector[1] << endl;
				}
			}
		}

		if ( v_final_candidates.size() )
		{
			m_detected_pattern.last_was_detected = true;
			m_detected_pattern.last_location.clear();
			m_detected_pattern.last_location = v_final_candidates[0];

			if ( !m_detected_pattern.was_detected )
				m_detected_pattern.was_detected = true;

			if ( m_save_pattern_constrains )
			{
				fileDifferenceRatio << v_final_candidates[0][0].radius / v_final_candidates[0][1].radius << "  ";
				fileDifferenceRatio << v_final_candidates[0][1].radius / v_final_candidates[0][2].radius << endl;

				float distance1 = CPoint2D(v_final_candidates[0][0].center.x, v_final_candidates[0][0].center.y).distance2DTo( v_final_candidates[0][1].center.x, v_final_candidates[0][1].center.y );
				float distance2 = CPoint2D(v_final_candidates[0][1].center.x, v_final_candidates[0][1].center.y).distance2DTo( v_final_candidates[0][2].center.x, v_final_candidates[0][2].center.y );
				fileRadiusDistanceRatio << distance1 / v_final_candidates[0][1].radius << " ";
				fileRadiusDistanceRatio << distance2 / v_final_candidates[0][2].radius << endl;

				fileRadius << v_final_candidates[0][0].radius << " " <<  v_final_candidates[0][1].radius << " " <<  v_final_candidates[0][2].radius << endl;

				CPoint2D pattern_position(m_detected_pattern.last_location.obtainCenterCol(),
								  		  m_detected_pattern.last_location.obtainCenterRow() );

				fileDistanceToPattern << pattern_position.distance2DTo( m_pattern_conf.m_final_pattern_col[m_mode],
					m_pattern_conf.m_final_pattern_row[m_mode]) << endl;
			}
		}

		if ( m_save_pattern_constrains )
		{
			fileErrorAdjustingLines.close();
			fileDifferenceRatio.close();
			fileRadiusDistanceRatio.close();
			fileRadius.close();
			fileInclination.close();
			fileDistanceToPattern.close();
		}

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In search pattern method **") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Search pattern");
}


//-----------------------------------------------------------
//					  segmentation_hsv
//-----------------------------------------------------------

void CAutoDockingCamera::segmentation_hsv( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius )
{
	// Useful variables declaration
	Mat segmented;

	vector<vector<vector<Point>>>	contours;
	vector<vector<Vec4i>>			hierarchy;

	contours.resize( m_colours_to_detect );	
	hierarchy.resize( m_colours_to_detect );

	vector<vector<vector<Point>>>	contours_poly;
	vector<vector<Rect>>			boundRect;

	contours_poly.resize( m_colours_to_detect );
	boundRect.resize( m_colours_to_detect );
	center.resize( m_colours_to_detect );
	radius.resize( m_colours_to_detect );

	vector<Mat> segmented_colours(m_colours_to_detect);
	
	for ( size_t i = 0; i < m_colours_to_detect; i++ )
	{
		// Segmentation for each color
		obtainSegmentedMat( frame, segmented_colours[i], i );

		Mat &img = segmented_colours[i];

		// Find contours
		findContours( img, contours[i], hierarchy[i], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());
				
		/// Approximate contours to polygons + get bounding rects and circles
		contours_poly[i].resize( contours[i].size() );
		boundRect[i].resize( contours[i].size() );
		center[i].resize( contours[i].size() );
		radius[i].resize( contours[i].size() );

		for( unsigned int j = 0; j < contours[i].size(); j++ )
		{ 
			approxPolyDP( Mat(contours[i][j]), contours_poly[i][j], 3, true );
			boundRect[i][j] = boundingRect( Mat(contours_poly[i][j]) );
			Mat point(contours_poly[i][j]);
			minEnclosingCircle( point, center[i][j], radius[i][j] );
		}


		/// Draw polygonal contour + bonding rects + circles
		Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
		for( unsigned int j = 0; j< contours[i].size(); j++ )
		{
			Scalar colour = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			//drawContours( drawing, contours_poly[i], j, colour, 1, 8, vector<Vec4i>(), 0, Point() );
			//rectangle( drawing, boundRect[i][j].tl(), boundRect[i][j].br(), colour, 2, 8, 0 );
			circle( drawing, center[i][j], (int)radius[i][j], colour, 2, 8, 0 );
		}

		/// Show in a window
		if ( m_display.hsv_contours )
		{
			imshow( mrpt::format("Contours colour %d",i), drawing );
			cvWaitKey(5);
		}
	}

}


//-----------------------------------------------------------
//				segmentation_cannyFilledPolygons
//-----------------------------------------------------------
void CAutoDockingCamera::segmentation_cannyFilledPolygons( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius )
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Canny segmentation");

		// Declare/Define useful variables 
		canny_segmentation_parameters &canny_params = m_canny_segmentation_parameters;

		Mat gray_frame;
		Mat edges_img;

		// 1. Apply a gaussian filter for smoothing the noise

		cvtColor( frame, gray_frame, CV_BGR2GRAY );

		equalizeHist( gray_frame, gray_frame );

		GaussianBlur(  gray_frame, gray_frame, 
						Size(canny_params.gaussian_blur_size, canny_params.gaussian_blur_size), 
						canny_params.gaussian_blur_sigma_1, 
						canny_params.gaussian_blur_sigma_2 );

		// 2. Detect edges using Canny

		Canny(gray_frame, edges_img, canny_params.higher_threshold, canny_params.higher_threshold/2 );

		if ( m_display.canny_results )
		{
			imshow( "Canny edges detection", edges_img );
		}

		// 3. Find contours and draw it into two images, one drawing filled contours and another one without filling

		vector<vector<Point>>	contours_canny;
		vector<Vec4i>			hierarchy_canny;
	
		findContours( edges_img, contours_canny, hierarchy_canny, CV_RETR_LIST /*CV_RETR_EXTERNAL*/, CV_CHAIN_APPROX_SIMPLE, Point());

		Mat img_all_contours_and_filled = Mat::zeros( edges_img.size(), CV_8UC1 );
		Mat img_all_contours = Mat::zeros( edges_img.size(), CV_8UC1 );
		for( unsigned int j = 0; j< contours_canny.size(); j++ )		
		{
			Scalar color( 255, 255, 255 );
			drawContours( img_all_contours_and_filled, contours_canny, j, color, CV_FILLED, 8, hierarchy_canny );
			drawContours( img_all_contours, contours_canny, j, color, 1, 8, hierarchy_canny );
		}

		if ( m_display.canny_results )
		{
			imshow( "All contorus with closed", img_all_contours_and_filled );
		}

		// 4. Substract both images

		Mat img_only_closed_contours;
		absdiff( img_all_contours_and_filled, img_all_contours, img_only_closed_contours );

		if ( m_display.canny_results )
		{
			imshow( "Closed contours only", img_only_closed_contours );
		}

		// 5. Find contours again and fit circles for filling output vectors with candidate regions.

		vector<vector<Point>>	contours;
		vector<Vec4i>			hierarchy;

		vector<vector<Point>>	contours_poly;
		vector<Rect>			boundRect;
		vector<Point2f>			contours_center;
		vector<float>			contours_radius;	

		findContours( img_only_closed_contours, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point());

		size_t N_contours = contours.size();

		contours_poly.resize( N_contours );
		boundRect.resize( N_contours );
		contours_center.resize( N_contours );
		contours_radius.resize( N_contours );

		for( unsigned int i_contour = 0; i_contour < N_contours; i_contour++ )
		{ 
			approxPolyDP( Mat(contours[i_contour]), contours_poly[i_contour], 3, true );
			boundRect[i_contour] = boundingRect( Mat(contours_poly[i_contour]) );
			Mat point(contours_poly[i_contour]);
			minEnclosingCircle( point, contours_center[i_contour], contours_radius[i_contour] );
		}

		// 6. Fill output vectors

		size_t N = contours_center.size();

		for ( size_t i_colour = 0; i_colour < m_colours_to_detect; i_colour++ )
		{
			center[i_colour].resize( N );
			radius[i_colour].resize( N );

			for ( size_t i_circle = 0; i_circle < N; i_circle++ )
			{
				center[i_colour][i_circle] = contours_center[i_circle];	
				radius[i_colour][i_circle] = contours_radius[i_circle];
			}
		}

		// 7. Show graphical results

		if ( m_display.canny_results )
		{

			Mat drawing = Mat::zeros( img_only_closed_contours.size(), CV_8UC3 );
			for( unsigned int i_contour = 0; i_contour < N_contours; i_contour++ )
			{
				Scalar colour = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				circle( drawing, contours_center[i_contour], (int)contours_radius[i_contour], colour, 2, 8, 0 );
			}	
		
			imshow( "Canny closed contours", drawing );

			cvWaitKey(5);
		}

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In segmentation_cannyFilledPolygons method **") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Canny segmentation");

}


//-----------------------------------------------------------
//					segmentation_houghCircles
//-----------------------------------------------------------

void CAutoDockingCamera::segmentation_houghCircles( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius )
{
	try
	{
		// Useful alias
		hough_segmentation_parameters &hough_params = m_hough_segmentation_parameters;

		// smooth it, otherwise a lot of false circles may be detected
		Mat hough_img;
		cvtColor( frame, hough_img, CV_BGR2GRAY );
		GaussianBlur(  hough_img, hough_img, 
						Size(hough_params.gaussian_blur_size, hough_params.gaussian_blur_size), 
						hough_params.gaussian_blur_sigma_1, 
						hough_params.gaussian_blur_sigma_2 );

		vector<Vec3f> hough_circles;
		HoughCircles(hough_img, hough_circles, CV_HOUGH_GRADIENT,
						hough_params.inverse_ratio_accumulator_resolution, 
						hough_params.min_dist, 
						hough_params.higher_canny_threshold,
						hough_params.accumulator_threshold,
						hough_params.min_radius,
						hough_params.max_radius );


		// Fill output vectors

		size_t N = hough_circles.size();

		for ( size_t i_colour = 0; i_colour < m_colours_to_detect; i_colour++ )
		{
			center[i_colour].resize( N );
			radius[i_colour].resize( N );

			for ( size_t i_circle = 0; i_circle < N; i_circle++ )
			{
				center[i_colour][i_circle] = Point2f(hough_circles[i_circle][0],hough_circles[i_circle][1]);	
				radius[i_colour][i_circle] = hough_circles[i_circle][2];
			}
		}

		if ( m_display.hough_circles_segmentation )
		{
			Mat hough_img_color;
			Mat edges_img;

			Canny(hough_img, edges_img, hough_params.higher_canny_threshold, hough_params.higher_canny_threshold/2 );

			imshow("Canny edges", edges_img);

			cvtColor( hough_img, hough_img_color, CV_GRAY2BGR );

			for( size_t i = 0; i < hough_circles.size(); i++ )
			{
				 Point center(cvRound(hough_circles[i][0]), cvRound(hough_circles[i][1]));
				 int hough_radius = cvRound(hough_circles[i][2]);
				 // draw the circle center
				circle( hough_img_color, center, 3, Scalar(0,255,0), -1, 8, 0 );
				// draw the circle outline
				circle( hough_img_color, center, hough_radius, Scalar(0,0,255), 3, 8, 0 );
			}
			
			imshow( "Hough circles detection", hough_img_color );
			cvWaitKey(5);

			mrpt::system::pause();
		}

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In segmentation_houghCircles method **") + string(e.what())).c_str() );
	}	
}

void CAutoDockingCamera::obtainHistogramInformation( const Mat &frame )
{
	try
	{
		static size_t save_count = 0;

		if ( m_detected_pattern.last_was_detected )
		{
			for( unsigned int i_contour = 0; i_contour < 3; i_contour++ )
			{
				Mat drawing = Mat::zeros( frame.size(), CV_8UC1 );
				circle( drawing, 
						m_detected_pattern.last_location[i_contour].center, 
						(int)m_detected_pattern.last_location[i_contour].radius, 
						Scalar(255,255,255), 2, 8, 0 );

				Mat test = drawing;

				Rect rect;
				floodFill(test, m_detected_pattern.last_location[i_contour].center, 
							Scalar(255,255,255), &rect, Scalar(10,10,10), Scalar(10,10,10) );

				imshow( mrpt::format("Test %d", i_contour), test );
		
				Mat hsv;

				cvtColor(frame, hsv, CV_BGR2HSV);

				// Quantize the hue to 30 levels
				// and the saturation to 32 levels
				int hbins = 180;
				int histSize[] = {hbins};
				// hue varies from 0 to 179, see cvtColor
				float hranges[] = { 0, 180 };
				const float* ranges[] = { hranges };
				MatND hist;
				int channels[] = {0};

				calcHist( &hsv, 1, channels, test, // do not use mask
						 hist, 1, histSize, ranges,
						 true, // the histogram is uniform
						 false );
				double maxVal=0;
				minMaxLoc(hist, 0, &maxVal, 0, 0);

				int v_scale = 40;
				int h_scale = 8;

				Mat histImg = Mat::zeros(v_scale, (hbins+1)*h_scale, CV_8UC3);
				//int scale = 2;
				//Mat histImg = Mat::zeros(sbins*scale, hbins*scale, CV_8UC3);

				ofstream file;
				file.open( mrpt::format("Histogram %d.txt", i_contour ).c_str(), ofstream::app );

				for( int h = 0; h < hbins; h++ )					
				{
					float binVal = hist.at<float>( h );

					if (!( save_count % 10 ))
						file << binVal << " ";

					int elevation = cvRound((binVal/maxVal)*v_scale);
					//cout << elevation << endl;
					rectangle( histImg, Point(h*h_scale,v_scale-1), Point((h+1)*h_scale,v_scale-elevation-1) , Scalar(255,255,255) );
				}

				if (!( save_count % 10 ))
				{
					file << endl;
					save_count = 0;
				}

				file.close();

				imshow( mrpt::format("H-S Histogram %d",i_contour), histImg );
			}
		}

		save_count++;

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In obtainHistogramInformation method **") + string(e.what())).c_str() );
	}
}


//-----------------------------------------------------------
//				fixPossibleImageLimitsViolations
//-----------------------------------------------------------
void CAutoDockingCamera::fixPossibleImageLimitsViolations( double &top_left, double &top_right, double &top_up , double &top_down )
{
	top_left = ( top_left < m_frame_features.initial_ROI_col_left_limit ) ? m_frame_features.initial_ROI_col_left_limit : top_left;
	top_right = ( top_right > m_frame_features.initial_ROI_col_right_limit ) ? m_frame_features.initial_ROI_col_right_limit : top_right;
	top_up = ( top_up < m_frame_features.initial_ROI_row_upper_limit ) ? m_frame_features.initial_ROI_row_upper_limit : top_up;
	top_down = ( top_down > m_frame_features.initial_ROI_row_lower_limit ) ? m_frame_features.initial_ROI_row_lower_limit : top_down;	
}


//-----------------------------------------------------------
//					obtainROI
//-----------------------------------------------------------

void CAutoDockingCamera::obtainROI( const Mat &fullFrame, Mat &frame )
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Obtain ROI");

		TNCircles &pattern_location = m_detected_pattern.last_location;
		double pattern_top_left, pattern_top_right, pattern_top_up, pattern_top_down;

		TLast_movement_command	&last_movement = m_last_movement.last_movement;

		if ( ( last_movement == STRAIGHT_LINE ) 
			|| ( last_movement == TURN_LEFT_PATTERN_WAS_DETECTED )
			|| ( last_movement == TURN_RIGHT_PATTERN_WAS_DETECTED ) )
		{		
			pattern_location.getTopPixels( pattern_top_left, pattern_top_right, pattern_top_up, pattern_top_down );		
			fixPossibleImageLimitsViolations( pattern_top_left, pattern_top_right, pattern_top_up, pattern_top_down );		
			//writeDebugLine( mrpt::utils::format("t_l:%f t_r:%f t_u:%f t_d:%f",pattern_top_left, pattern_top_right, pattern_top_up, pattern_top_down ), AUTODOCKING);
		}

		double left_col, right_col, up_row, down_row;

		switch (last_movement)
		{
			case STRAIGHT_LINE:

				left_col	= pattern_top_left  - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_straight_line[0];
				right_col	= pattern_top_right + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_straight_line[1];
				up_row		= pattern_top_up    - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_straight_line[2];
				down_row	= pattern_top_down  + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_straight_line[3];

				fixPossibleImageLimitsViolations( left_col, right_col, up_row, down_row );

				break;

			case TURN_LEFT_PATTERN_WAS_DETECTED:

				left_col	= pattern_top_left  - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_left[0];
				right_col	= pattern_top_right + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_left[1];
				up_row		= pattern_top_up    - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_left[2];
				down_row	= pattern_top_down  + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_left[3];

				fixPossibleImageLimitsViolations( left_col, right_col, up_row, down_row );

				break;

			case TURN_RIGHT_PATTERN_WAS_DETECTED:

				left_col	= pattern_top_left  - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_right[0];
				right_col	= pattern_top_right + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_right[1];
				up_row		= pattern_top_up    - m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_right[2];
				down_row	= pattern_top_down  + m_tracking_window.window_size / m_tracking_window.v_window_adjustment_turn_right[3];

				fixPossibleImageLimitsViolations( left_col, right_col, up_row, down_row );

				break;

			case TURN_PATTERN_WAS_NOT_DETECTED:

				left_col	= m_frame_features.initial_ROI_col_left_limit;
				right_col	= m_frame_features.initial_ROI_col_right_limit;			
				up_row		= m_frame_features.initial_ROI_row_upper_limit;
				down_row	= m_frame_features.initial_ROI_row_lower_limit;		
			
				break;

			default:
				throw std::logic_error("Incorrect last movement, it doesn't exist!!!");
		}	

		frame = fullFrame( Range( up_row, down_row ), Range( left_col, right_col ) );

		// Save the up,left corner for adding its coordinates to the final detected pattern
		m_tracking_window.up_left_corner_col = left_col;
		m_tracking_window.up_left_corner_row = up_row;

		static int frame_tracking_count = 0;

		imwrite(mrpt::format("./tracking/img%d.jpg",frame_tracking_count), frame );

		frame_tracking_count++;	
		
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In obtainROI method **") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Obtain ROI");
}
//-----------------------------------------------------------
//					patternLocalization
//-----------------------------------------------------------
void CAutoDockingCamera::patternLocalization()
{
	if( m_save_executions_time )
		m_timeLog.enter("Pattern localization");

	try
	{
		// Useful variables declaration
		Mat fullFrame;
		Mat frame;

		vector<vector<Point2f>>			center;
		vector<vector<float>>			radius;

		center.resize( m_colours_to_detect );
		radius.resize( m_colours_to_detect );

		// Here we go!!!! 
		
		// 1. Obtaining a new frame...
		while( !get_new_image(fullFrame) )
			mrpt::system::sleep(100);

		
		//Display image (debug)
		//cv::namedWindow( "CAMERAimg", cv::WINDOW_NORMAL );// Create a window for display. //WINDOW_AUTOSIZE
		//cv::imshow( "CAMERAimg", fullFrame );	                  // Show our image inside it.
		//cv::waitKey(0); // waits to display frame
				
		//Obtain ROI
		obtainROI( fullFrame, frame );


		//Display image (debug)
		//cv::namedWindow( "ROI frame", cv::WINDOW_NORMAL );// Create a window for display. //WINDOW_AUTOSIZE
		//cv::imshow( "ROI frame", frame );	                  // Show our image inside it.
		//cv::waitKey(1); // waits to display frame
		
		// 2. Launch the chosen segmentation algorithm
		if ( m_segmentation_algorithm == 0 )
			segmentation_hsv( frame, center, radius );
		else if ( m_segmentation_algorithm == 1 )
			segmentation_houghCircles( frame, center, radius );
		else if ( m_segmentation_algorithm == 2 )
			segmentation_cannyFilledPolygons( frame, center, radius );

		// 3. Pattern detection
		searchPattern_afterHsvOrHough( center, radius );

		// 4. Display visual information (useful for debugging)
		if ( m_display.BGR )
		{
			imshow("BGR", fullFrame);	
			imshow("BGR-tracking", frame);	
		}

		// Save histogram information to file
		if ( m_save_histogram_information )
			obtainHistogramInformation( frame );

		if ( m_display.pattern_detection )
		{
			if ( m_detected_pattern.last_was_detected )
			{
				for( unsigned int i_pattern_component = 0; i_pattern_component < m_detected_pattern.last_location.size(); i_pattern_component++ )
				{
					Scalar colour = Scalar( 255, 255, 255 );
					//drawContours( drawing, contours_poly[i], j, colour, 1, 8, vector<Vec4i>(), 0, Point() );
					//rectangle( drawing, boundRect[i][j].tl(), boundRect[i][j].br(), colour, 2, 8, 0 );
					circle( frame, m_detected_pattern.last_location[i_pattern_component].center, 
						(int)m_detected_pattern.last_location[i_pattern_component].radius, colour, 2, 8, 0 );
				}
			
				imshow( "Detected pattern", frame );	
			}
		}

		if ( m_display.BGR || m_display.pattern_detection )
			cvWaitKey(5);

		// 5. Undo ROI coordinates (obtain the coordinates of the pattern w.r.t the full image
		if ( m_detected_pattern.last_was_detected )
		{
			m_detected_pattern.last_location.applyCenterOffset( m_tracking_window.up_left_corner_col, m_tracking_window.up_left_corner_row );
		}

		// 5. Save image to file
		static int frame_count = 0;

		if ( m_detected_pattern.last_was_detected )
		{
			for( unsigned int i_pattern_component = 0; i_pattern_component < m_detected_pattern.last_location.size(); i_pattern_component++ )
			{
				Scalar colour = Scalar( 0, 255, 0 );
				//drawContours( drawing, contours_poly[i], j, colour, 1, 8, vector<Vec4i>(), 0, Point() );
				//rectangle( drawing, boundRect[i][j].tl(), boundRect[i][j].br(), colour, 2, 8, 0 );
				circle( fullFrame, m_detected_pattern.last_location[i_pattern_component].center, 
					(int)m_detected_pattern.last_location[i_pattern_component].radius, colour, 2, 8, 0 );
			}
		}

		imwrite(mrpt::format("./frames/img%d.jpg",frame_count), fullFrame );

		frame_count++;	

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In obtainPatternLocation method **") + string(e.what())).c_str() );
	}
	
	if ( m_save_executions_time )
		m_timeLog.leave("Pattern localization");
}

//-----------------------------------------------------------
//				 patternLocalizationProcess
//-----------------------------------------------------------
void CAutoDockingCamera::patternLocalizationProcess()
{
	m_detected_pattern.last_was_detected = false; // Reset last detection to false
		
	for( size_t i_try = 0; i_try < m_tries_detecting_pattern; i_try++ )
	{
		patternLocalization();

		if ( m_detected_pattern.last_was_detected )
			break;		
	}
}

//-----------------------------------------------------------
//					  checkGoodOriented
//-----------------------------------------------------------
void CAutoDockingCamera::checkGoodOriented()
{
	try
	{
		patternLocalizationProcess();

		if ( m_detected_pattern.last_was_detected ) // Pattern found!
		{
			// Is the pattern placed "more or less" in the center of the image?
			float row = m_detected_pattern.last_location.obtainCenterRow();

			// The distance relaxation has a value between [1..m_kp_consider_as_center_distance_relaxation]
			float distance_relaxation = 1 + (abs(( m_pattern_conf.m_final_pattern_row[m_mode] - row )) / m_pattern_conf.m_final_pattern_row[m_mode])
												*m_kp_consider_as_center_distance_relaxation;

			float threshold = m_threshold_considered_as_center * distance_relaxation;

			//if ( abs ( m_detected_pattern.last_location.obtainCenterCol() - m_frame_features.ROI_width / 2 ) < threshold )
			if ( abs ( m_detected_pattern.last_location.obtainCenterCol() - m_pattern_conf.m_final_pattern_col[m_mode] ) < threshold )
			{
				m_good_oriented = true;
			}
		}
		else // Pattern not found!
		{
			//Do nothing!
		}
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In checkGoodOriented method **") + string(e.what())).c_str() );
	}
}


//-----------------------------------------------------------
//					updateLastMovement
//-----------------------------------------------------------

void CAutoDockingCamera::updateLastMovement( const TLast_movement_command &movement, const double &move_or_degrees_quantity )
{
	try
	{
		printf("[AutoDocking_Camera]Updating last movement\n");
		// m_last_movement structure to update:
		//
		// {
		//		double				last_turn_mov_q;	// Store the last quantity of turn movement (degrees)
		//		double				last_straight_mov_q;	// Store the last quantity of turn movement (meters)	
		//		TLast_movement_command last_movement;    // Of what type was the last movement?
		//		mrpt::utils::TTimeStamp	time;			// When was ordered?
		// }	

		// If the movement is different to TURN_PATTERN_WAS_NOT_DETECTED and we was turning without detecting
		// the pattern, the m_turn_complete_tester variable must be reset

		// We need to declare this varbiales here, a switch problem ¬¬
		CPose2D currPose;
		TTimeStamp time;
		CMOOSVariable * pVar;

		switch ( movement )
		{
			case STRAIGHT_LINE :
				m_last_movement.last_straight_mov_q = move_or_degrees_quantity;
				if ( m_turn_complete_tester.isTurning() ) m_turn_complete_tester.reset();
				break;

			case TURN_RIGHT_PATTERN_WAS_DETECTED : 
				m_last_movement.last_turn_mov_q	= move_or_degrees_quantity;
				if ( m_turn_complete_tester.isTurning() ) m_turn_complete_tester.reset();
				break;
		
			case TURN_LEFT_PATTERN_WAS_DETECTED :
				m_last_movement.last_turn_mov_q	= move_or_degrees_quantity;
				if ( m_turn_complete_tester.isTurning() ) m_turn_complete_tester.reset();
				break;

			case TURN_PATTERN_WAS_NOT_DETECTED :
				//Get Odometry
				pVar = GetMOOSVar("ODOMETRY_OBS");
				if( pVar /*&& pVar->IsFresh()*/ )
				{
					pVar->SetFresh(false);
					CSerializablePtr obj;
					mrpt::utils::RawStringToObject(pVar->GetStringVal(),obj);
					
					if( IS_CLASS(obj, CObservationOdometry) )
					{
						mrpt::slam::CObservationOdometryPtr odo_obs = mrpt::slam::CObservationOdometryPtr(obj);
						currPose = odo_obs->odometry;

						mrpt::math::CVectorDouble v_pose;
						currPose.getAsVector( v_pose );
						m_turn_complete_tester.setLastOrientation( v_pose[2] );
					
						if ( !m_turn_complete_tester.isTurning() )
							m_turn_complete_tester.setTurning( true );		
					}
					else
						printf("[AutoDocking_Camera] ODOMETRY_OBS is not CObservationOdometry\n");
				}
				else
					printf("[AutoDocking_Camera] ODOMETRY_OBS not available\n");
				
				m_last_movement.last_turn_mov_q	= move_or_degrees_quantity;			
				break;

			default:
				throw std::logic_error("Incorrect last movement, it doesn't exist!!!");
		}

		m_last_movement.time = mrpt::system::now();
		m_last_movement.last_movement	= movement;

	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**[AutoDocking_Camera] ERROR In updateLastMovement method**") + string(e.what())).c_str() );
	}	
}

//-----------------------------------------------------------
//						turnABit
//-----------------------------------------------------------

void CAutoDockingCamera::turnABit()
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Turn a bit");

		cout << "Turning a bit!" << endl;

		
		// 2. Check if the pattern has been seen, for rotating in the correct direction!
		
		if ( m_detected_pattern.last_was_detected )
		{
			double direction = m_detected_pattern.last_location.obtainCenterCol() - m_pattern_conf.m_final_pattern_col[m_mode];

			float current_col = m_detected_pattern.last_location.obtainCenterCol();

			double control_action = 1 + 
				(abs(( m_pattern_conf.m_final_pattern_col[m_mode] - current_col )) / m_pattern_conf.m_final_pattern_col[m_mode] )*m_kp_turn;

			double degrees;		// Degrees to turn
			double motion_w;	// Angular speed

			if ( direction > 0 ) // Turn right
			{
				degrees = m_degrees_turning_per_step*control_action;
				motion_w = -0.2*degrees;

				//Update last movement
				updateLastMovement( TURN_RIGHT_PATTERN_WAS_DETECTED, degrees );
			}
			else // Turn left
			{
				degrees = -(double)m_degrees_turning_per_step*control_action;
				motion_w = -0.2*degrees;

				//Update last movement
				updateLastMovement( TURN_LEFT_PATTERN_WAS_DETECTED, degrees );
			}			

			// Turn!
			//------------------------
			//! @moos_publish MOTION_CMD_V
			m_Comms.Notify("MOTION_CMD_V", 0.0);
			//! @moos_publish MOTION_CMD_W
			m_Comms.Notify("MOTION_CMD_W", motion_w);

			
			printf("DockStation detected but not in front of the robot (%f,%f), performing rotative movement: %f degrees"
				, m_detected_pattern.last_location.obtainCenterCol()
				, m_detected_pattern.last_location.obtainCenterRow()
				, degrees);
		}
		else // Pattern not saw, rotate in the usual direction
		{
			double degrees_to_turn;
			double motion_w;	// Angular speed

			if ( !m_detected_pattern.was_detected )	// The pattern wasn't detected any time!
			{
				//linea de autodocking original
				degrees_to_turn = m_degrees_turning_per_step*m_kp_turn;
				motion_w = -0.3*degrees_to_turn;

				//test
				/*
				if(degrees_to_turn>0)
					wdeg2=-15;
				else if(degrees_to_turn<0)
					wdeg2=+15;
				else if(degrees_to_turn==0)
					wdeg2=0;				
				*/
			}
			else	// If the pattern was detected, turn in the same direction that the last time
			{
				//linea de autodocking original
				degrees_to_turn = m_last_movement.last_turn_mov_q;
				motion_w = -0.3*degrees_to_turn;

				//test
				/*
				if(degrees_to_turn>0)
					wdeg2=15;
				else if(degrees_to_turn<0)
					wdeg2=-15;
				else if(degrees_to_turn==0)
					wdeg2=0;
				*/
			}

			// Turn!
			//------------------------
			//! @moos_publish MOTION_CMD_V
			m_Comms.Notify("MOTION_CMD_V", 0.0);
			//! @moos_publish MOTION_CMD_W
			m_Comms.Notify("MOTION_CMD_W", motion_w);

			// Update last movement
			updateLastMovement( TURN_PATTERN_WAS_NOT_DETECTED, degrees_to_turn );

			printf("DockStation not detected, performing rotative movement: %f degrees\n",degrees_to_turn);
		}		
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In turnABit method**") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Turn a bit");
}

//-----------------------------------------------------------
//					performGoodOrientation
//-----------------------------------------------------------
bool CAutoDockingCamera::performGoodOrientation()
{
	try
	{
		m_good_oriented = false;
	
		checkGoodOriented();

		while( !m_good_oriented && module_active )
		{
			turnABit();
		
			checkGoodOriented();

			if( m_turn_complete_tester.check() ) // We have completed a turn without finding the docking station!
			{
				if( m_mode == FAR_MODE )
				{
					changeMode( NEAR_MODE );
					cout << "[CAutoDockingCamera: performGoodOrientation] A turn completed without finding the docking station in FAR_MODE!... changing to NEAR_MODE" << endl;
					m_turn_complete_tester.reset();
				}
				else
				{
					cout << "[CAutoDockingCamera: performGoodOrientation] A turn completed without finding the docking station!" << endl;
					break;
				}
			}
		}

		return m_good_oriented;

	}
	catch (std::exception &e)
	{
		return MOOSFail( (string("**ERROR In performGoodOrientation method**") + string(e.what())).c_str() );
	}

	return false;
}

//-----------------------------------------------------------
//					  changeMode
//-----------------------------------------------------------
void CAutoDockingCamera::changeMode( const TMode &new_mode )
{
	try
	{
		double angleFarMode = m_angleFarMode; // 0.71273;
		double angleNearMode = m_angleNearMode; //0.4;
		double currTilt;
		CMOOSVariable * pVar;

		switch( new_mode )
		{
		case FAR_MODE:			
			//! @moos_publish TILT_SET_ANGLE_FROM_HOME: The new tilt angle (degress) to set the robot's screen
			m_Comms.Notify("TILT_SET_ANGLE_FROM_HOME", angleFarMode);
			m_mode = FAR_MODE;
			
			mrpt::system::sleep(2500);

			//Get new tilt angle
			pVar = GetMOOSVar("TILT_CURRENT_ANGLE_FROM_HOME");
			if( pVar && pVar->IsFresh() )
			{
				pVar->SetFresh(false);
				cout << "[CAutoDockingCameraCamera: changeMode] Mode changed to FAR_MODE, current tilt: " << pVar->GetDoubleVal() << endl;
			}
			break;

		case NEAR_MODE:
			//! @moos_publish TILT_SET_ANGLE_FROM_HOME: The new tilt angle (degress) to set the robot's screen
			m_Comms.Notify("TILT_SET_ANGLE_FROM_HOME", angleNearMode);
			m_mode = NEAR_MODE;

			mrpt::system::sleep(2500);

			//Get new tilt angle
			pVar = GetMOOSVar("TILT_CURRENT_ANGLE_FROM_HOME");
			if( pVar && pVar->IsFresh() )
			{
				pVar->SetFresh(false);
				cout << "[CAutoDockingCameraCamera: changeMode] Mode changed to NEAR_MODE, current tilt: " << pVar->GetDoubleVal() << endl;
			}
			break;

		default:
			throw std::logic_error("[CAutoDockingCameraCamera: changeMode] Incorrect performance mode, it doesn't exist!!!");
		}
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In changeMode method!**") + string(e.what())).c_str() );
	}
}

//-----------------------------------------------------------
//					moveInStraightLineABit
//-----------------------------------------------------------
void CAutoDockingCamera::moveInStraightLineABit()
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Move in straight line a bit");

		// Obtain the num of meters to move
		float row = m_detected_pattern.last_location.obtainCenterRow();
		// The control action has a value between [1..m_kp_move_in_straight_line]
		float control_action = 1 + (abs(( m_pattern_conf.m_final_pattern_row[m_mode] - row )) / m_pattern_conf.m_final_pattern_row[m_mode])*m_kp_move_in_straight_line;
		float move = m_meters_moving_per_step * control_action;

		float motion_v = move*2;
		// Move Stright at 0.1m/s
		//------------------------
		//! @moos_publish MOTION_CMD_V
		m_Comms.Notify("MOTION_CMD_V", motion_v);
		//! @moos_publish MOTION_CMD_W
		m_Comms.Notify("MOTION_CMD_W", 0.0);

		
		//Update last movement
		updateLastMovement( STRAIGHT_LINE, move );

		printf("DockStation detected in front of the robot (%f,%f), performing linear movement: %f meters\n"
					, m_detected_pattern.last_location.obtainCenterCol()
					, m_detected_pattern.last_location.obtainCenterRow()
					, move);
		
		// Check if we have to change the mode
		if ( m_mode == FAR_MODE )
		{
			mrpt::poses::CPoint2D detected_pattern_location( m_detected_pattern.last_location.obtainCenterCol(), m_detected_pattern.last_location.obtainCenterRow() );

			float distance = detected_pattern_location.distance2DTo( m_pattern_conf.m_final_pattern_col[m_mode],
																	 m_pattern_conf.m_final_pattern_col[m_mode] );

			printf("Distance to the final pattern position: %f pixels\n", distance );

			if ( distance < m_distance_for_changing_mode )
			{
				changeMode( NEAR_MODE );
				cout << "Changing to NEAR_MODE!" << endl;
			}
		}
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In moveInStraightLineABit method!**") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Move in straight line a bit");
}


//-----------------------------------------------------------
//					checkGoalAchieved
//-----------------------------------------------------------

bool CAutoDockingCamera::checkGoalAchieved()
{
	try
	{
		if ( m_save_executions_time )
			m_timeLog.enter("Check goal achieved");

		//Check if Is_Charging
		CMOOSVariable * pVar = GetMOOSVar("BATTERY_IS_CHARGING");
		if( pVar )
		{
			double is_charging = pVar->GetDoubleVal();

			if( is_charging!=0.0 )
			{
				cout << "Docking complete!" << endl;

				if ( m_save_executions_time )
					m_timeLog.leave("Check goal achieved");

				return true;	// Docked
			}
			else
			{
				if ( m_save_executions_time )
					m_timeLog.leave("Check goal achieved");

				return false;	// Not docked
			}
		}
		else
			return false;
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In checkGoalAchieved method!**") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Check goal achieved");

	return false;	

}


void CAutoDockingCamera::moveABit()
{
	/*
	try
	{
		// Check if the pattern has been seen, for rotating in the correct direction!		
		if ( m_detected_pattern.last_was_detected )
		{
			double direction = m_detected_pattern.last_location.obtainCenterCol() - m_pattern_conf.m_final_pattern_col[m_mode];

			float current_col = m_detected_pattern.last_location.obtainCenterCol();

			double control_action = 1 + 
				(abs(( m_pattern_conf.m_final_pattern_col[m_mode] - current_col )) / m_pattern_conf.m_final_pattern_col[m_mode] )*m_kp_turn;

			double degrees;		// Degrees to turn
			double motion_w;	// Angular speed

			if ( direction > 0 ) // Turn right
			{
				degrees = m_degrees_turning_per_step*control_action;
				motion_w = -0.2*degrees;

				//Update last movement
				updateLastMovement( TURN_RIGHT_PATTERN_WAS_DETECTED, degrees );
			}
			else // Turn left
			{
				degrees = -(double)m_degrees_turning_per_step*control_action;
				motion_w = -0.2*degrees;

				//Update last movement
				updateLastMovement( TURN_LEFT_PATTERN_WAS_DETECTED, degrees );
			}			

			// Turn!
			//------------------------
			//! @moos_publish MOTION_CMD_V
			m_Comms.Notify("MOTION_CMD_V", 0.0);
			//! @moos_publish MOTION_CMD_W
			m_Comms.Notify("MOTION_CMD_W", motion_w);

			
			printf("DockStation detected but not in front of the robot (%f,%f), performing rotative movement: %f degrees"
				, m_detected_pattern.last_location.obtainCenterCol()
				, m_detected_pattern.last_location.obtainCenterRow()
				, degrees);
		}
		else // Pattern not seen, rotate in the usual direction
		{
			double motion_w;	// Angular speed

			if( !m_detected_pattern.was_detected )	// The pattern wasn't detected any time!
			{
				//Set the max speed for turning!
				motion_w = m_max_angular_speed;				
			}
			else	// If the pattern was detected, turn in the same direction and speed as the last time
			{
				motion_w = m_last_movement.last_turn_mov_q;
			}

			// Turn!
			//------------------------
			//! @moos_publish MOTION_CMD_V
			m_Comms.Notify("MOTION_CMD_V", 0.0);
			//! @moos_publish MOTION_CMD_W
			m_Comms.Notify("MOTION_CMD_W", motion_w);

			// Update last movement
			updateLastMovement( TURN_PATTERN_WAS_NOT_DETECTED, motion_w );

			printf("DockStation not detected, performing rotative movement at: %f rad/s",motion_w);
		}		
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In turnABit method**") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Turn a bit");



	try
	{
		// Set a linear speed as a function to the distance to the docking station
		float row = m_detected_pattern.last_location.obtainCenterRow();

		// The control action has a value between [1..m_kp_move_in_straight_line]
		float control_action = 1 + (abs(( m_pattern_conf.m_final_pattern_row[m_mode] - row )) / m_pattern_conf.m_final_pattern_row[m_mode])*m_kp_move_in_straight_line;
		
		float motion_v = m_meters_moving_per_step * control_action;

		
		//------------------------
		//! @moos_publish MOTION_CMD_V
		m_Comms.Notify("MOTION_CMD_V", motion_v);
		//! @moos_publish MOTION_CMD_W
		m_Comms.Notify("MOTION_CMD_W", 0.0);

		
		//Update last movement
		updateLastMovement( STRAIGHT_LINE, move );

		printf("DockStation detected in front of the robot (%f,%f), performing linear movement: %f meters\n"
					, m_detected_pattern.last_location.obtainCenterCol()
					, m_detected_pattern.last_location.obtainCenterRow()
					, move);
		
		// Check if we have to change the mode
		if ( m_mode == FAR_MODE )
		{
			mrpt::poses::CPoint2D detected_pattern_location( m_detected_pattern.last_location.obtainCenterCol(), m_detected_pattern.last_location.obtainCenterRow() );

			float distance = detected_pattern_location.distance2DTo( m_pattern_conf.m_final_pattern_col[m_mode],
																	 m_pattern_conf.m_final_pattern_col[m_mode] );

			printf("Distance to the final pattern position: %f pixels\n", distance );

			if ( distance < m_distance_for_changing_mode )
			{
				changeMode( NEAR_MODE );
				cout << "Changing to NEAR_MODE!" << endl;
			}
		}
	}
	catch (std::exception &e)
	{
		MOOSFail( (string("**ERROR In moveInStraightLineABit method!**") + string(e.what())).c_str() );
	}

	if ( m_save_executions_time )
		m_timeLog.leave("Move in straight line a bit");
*/
}