	
	/*---------------------------------------------------------------
	|					NAAS Control Architecture					|
	|																|
	|			J.R. Ruiz-Sarmiento(jotaraul@uma.es)				|
	|		Department of Computer Engineering and Automatics.		|
	|			   MAPIR Group. University of Málaga				|
	|																|
	|							License:							|
	|	Creative Commons Attribution-NonCommercial-ShareAlike		|
	|	2.0 Generic (CC BY-NC-SA 2.0). Further information about	|
	|	this license here:											|
	|	http://creativecommons.org/licenses/by-nc-sa/2.0/			|
	|																|
	---------------------------------------------------------------*/


#include "CAutoDocking.h"

#include <mrpt/gui.h>
#include <mrpt/hwdrivers.h>

#include <opencv2/core/core.hpp>

using namespace NAAS;

using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::hwdrivers;

using namespace cv;

RNG rng(12345);


//-----------------------------------------------------------
//						CAutoDocking
//-----------------------------------------------------------

CAutoDocking::CAutoDocking(): m_started(false), 
								m_run(false), 
								m_stop(false), 
								m_motors(NULL),
								CLoggeable( "AUTODOCKING" )								
{
	init();
}


//-----------------------------------------------------------
//						~CAutoDocking
//-----------------------------------------------------------

CAutoDocking::~CAutoDocking()
{
	cout << "[INFO] Destroying Autodocking object..." << endl;

	m_stop = true;

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

	// Wait for thread termination

	if ( m_started )
		joinThread( m_thread_autoDocking );

	cout << "[INFO] Autodocking object destroyed" << endl;
}


//-----------------------------------------------------------
// 						   init
//-----------------------------------------------------------

void CAutoDocking::init()
{
	NAV_TRY

	// 1. Open the camera

	// Moved to DataSharer!

	// 2. Retrieve camera resolution

	writeDebugLine("Obtaining a first frame from the camera...",AUTODOCKING);

	Mat frame;
	TTimeStamp first_time = mrpt::system::now();
	bool obtained = false;

	while( !obtained && ( timeDifference( first_time, mrpt::system::now() ) < 15 ) )
	{
		g_cameraImage.working();

		if ( g_cameraImage.isFresh("AUTODOCKING") )
		{
			g_cameraImage.setFresh("AUTODOCKING",false);
			g_cameraImage.getImage( frame );
			obtained = true;
		}

		g_cameraImage.endWorking();
	}
	
	if ( !obtained )
		throw std::runtime_error("Unable to obtain the frame");	

	writeDebugLine("First frame obtained succesfully",AUTODOCKING);
			
	m_frame_features.height = frame.rows;
	m_frame_features.width = frame.cols;
	
	// 3. Obtain configuration parameters from config file

	// 3.1 Display windows configuration(useful for debug)
	m_display.BGR				= g_configFile.read_bool("AUTODOCKING","display_BGR",false,true);
	m_display.hsv_final_masks	= g_configFile.read_bool("AUTODOCKING","display_hsv_final_masks",false,true);
	m_display.hsv_partial_masks	= g_configFile.read_bool("AUTODOCKING","display_hsv_partial_masks",false,true);
	m_display.hsv_contours		= g_configFile.read_bool("AUTODOCKING","display_hsv_contours",false,true);
	m_display.hough_circles_segmentation = g_configFile.read_bool("AUTODOCKING","display_hough_circle_segmentation",false,true); 
	m_display.partial_candidates	= g_configFile.read_bool("AUTODOCKING","display_partial_candidates",false,true);
	m_display.pattern_detection	= g_configFile.read_bool("AUTODOCKING","display_pattern_detection",false,true);
	m_display.canny_results		= g_configFile.read_bool("AUTODOCKING","display_canny_results",false,true);

	// 3.2 Movements configuration
	m_degrees_turning_per_step	= g_configFile.read_double("AUTODOCKING","degrees_turning_per_step",0,true);
	m_meters_moving_per_step	= g_configFile.read_double("AUTODOCKING","meters_moving_per_step",0,true);	
	
	m_kp_move_in_straight_line	= g_configFile.read_double("AUTODOCKING","kp_move_in_straight_line",0,true);
	m_kp_consider_as_center_distance_relaxation = 
								  g_configFile.read_double("AUTODOCKING","kp_consider_as_center_distance_relaxation",0,true);
	m_kp_turn					= g_configFile.read_double("AUTODOCKING","kp_turn",0,true);

	m_angleFarMode				= g_configFile.read_double("AUTODOCKING","angle_far_mode",0,true);
	m_angleNearMode				= g_configFile.read_double("AUTODOCKING","angle_near_mode",0,true);

	// 3.3 General thresholds
	m_threshold_considered_as_center = g_configFile.read_double("AUTODOCKING","threshold_considered_as_center",0,true);	

	// 3.4 Colours to detect configuration
	m_colours_to_detect			= g_configFile.read_uint64_t("AUTODOCKING","colours_to_detect",0,true);	

	m_hsv_segmentation_parameters.m_hsv_thresholds.resize( m_colours_to_detect );

	for ( size_t i = 0; i < m_colours_to_detect; i++ )
		g_configFile.read_vector( "AUTODOCKING", mrpt::utils::format("hsv_thresholds_%d",i),vector_int(),m_hsv_segmentation_parameters.m_hsv_thresholds[i],true );

	// 3.5 Pattern configuration
	g_configFile.read_vector( "AUTODOCKING", "pattern_order", vector_size_t(), m_pattern_conf.m_pattern_order, true );
	
	g_configFile.read_vector( "AUTODOCKING", "max_inclination_fitted_rect",vector<float>(0),m_pattern_conf.m_max_inclination_fitted_rect,true );
	g_configFile.read_vector( "AUTODOCKING", "max_radius_difference_ratio",vector<float>(0),m_pattern_conf.m_max_radius_difference_ratio,true );
	g_configFile.read_vector( "AUTODOCKING", "min_circle_radius",vector<float>(0),m_pattern_conf.m_min_circle_radius,true );
	g_configFile.read_vector( "AUTODOCKING", "max_circle_radius",vector<float>(0),m_pattern_conf.m_max_circle_radius,true );
	g_configFile.read_vector( "AUTODOCKING", "max_error_adjusting_a_line",vector<float>(0),m_pattern_conf.m_max_error_adjusting_a_line,true );
	//m_pattern_conf.m_final_pattern_row				= g_configFile.read_uint64_t("AUTODOCKING","final_pattern_row",0,true);
	//m_pattern_conf.m_final_pattern_col				= g_configFile.read_uint64_t("AUTODOCKING","final_pattern_col",0,true);
	vector<float> aux_thresholds_size_relation;
	g_configFile.read_vector( "AUTODOCKING", "thresholds_size_relation",vector<float>(0),aux_thresholds_size_relation,true );
	m_pattern_conf.m_thresholds_size_relation.resize(2);
	m_pattern_conf.m_thresholds_size_relation[FAR_MODE].push_back( aux_thresholds_size_relation[0] );
	m_pattern_conf.m_thresholds_size_relation[FAR_MODE].push_back( aux_thresholds_size_relation[1] );
	m_pattern_conf.m_thresholds_size_relation[NEAR_MODE].push_back( aux_thresholds_size_relation[2] );
	m_pattern_conf.m_thresholds_size_relation[NEAR_MODE].push_back( aux_thresholds_size_relation[3] );
	g_configFile.read_vector( "AUTODOCKING", "final_pattern_row",vector<size_t>(0),m_pattern_conf.m_final_pattern_row,true );
	g_configFile.read_vector( "AUTODOCKING", "final_pattern_col",vector<size_t>(0),m_pattern_conf.m_final_pattern_col,true );

	if ( m_pattern_conf.m_thresholds_size_relation.size() != 2 )
		throw std::logic_error("The size of the thresholds_size_relation vector must be 2");

	// 3.6 General configuration
	m_segmentation_algorithm		= g_configFile.read_uint64_t("AUTODOCKING","segmentation_algorithm",0,true);
	m_no_motors_mode				= g_configFile.read_bool("AUTODOCKING","no_motors_mode",false,true);
	m_reactive_navigation_algorithm = g_configFile.read_uint64_t("AUTODOCKING","reactive_navigation_algorithm",0,true);
	m_tries_detecting_pattern		= g_configFile.read_uint64_t("AUTODOCKING","tries_detecting_pattern",0,true);
	m_robot_speed					= g_configFile.read_float("AUTODOCKING","robot_speed",0,true);	
	m_save_histogram_information	= g_configFile.read_bool("AUTODOCKING","save_histogram_information",false,true);
	m_save_executions_time			= g_configFile.read_bool("AUTODOCKING","save_executions_time",false,true);
	m_save_pattern_constrains		= g_configFile.read_bool("AUTODOCKING","save_pattern_constrains",false,true);
	m_distance_for_changing_mode	= g_configFile.read_uint64_t("AUTODOCKING","distance_for_changing_mode",0,true);

	// 3.7 Calculate rows and cols corresponding to the initial ROI
	float initial_ROI_row_upper_limit_percentaje = g_configFile.read_float("AUTODOCKING","initial_ROI_row_upper_limit",0,true);	
	float initial_ROI_row_lower_limit_percentaje = g_configFile.read_float("AUTODOCKING","initial_ROI_row_lower_limit",0,true);	
	float initial_ROI_col_left_limit_percentaje = g_configFile.read_float("AUTODOCKING","initial_ROI_col_left_limit",0,true);	
	float initial_ROI_col_right_limit_percentaje = g_configFile.read_float("AUTODOCKING","initial_ROI_col_right_limit",0,true);	
	
	m_frame_features.initial_ROI_row_upper_limit = floor( (m_frame_features.height-1) * ( initial_ROI_row_upper_limit_percentaje / 100 ) );
	m_frame_features.initial_ROI_row_lower_limit = floor( (m_frame_features.height-1) * ( initial_ROI_row_lower_limit_percentaje / 100 ) );
	m_frame_features.initial_ROI_col_left_limit	= floor( (m_frame_features.width-1) * ( initial_ROI_col_left_limit_percentaje / 100 ) );
	m_frame_features.initial_ROI_col_right_limit = floor( (m_frame_features.width-1) * ( initial_ROI_col_right_limit_percentaje / 100 ) );

	m_frame_features.ROI_width = m_frame_features.initial_ROI_col_right_limit - m_frame_features.initial_ROI_col_left_limit + 1;
	m_frame_features.ROI_height = m_frame_features.initial_ROI_row_lower_limit - m_frame_features.initial_ROI_row_upper_limit + 1;

	// 3.8 Hough segmentation paramters
	hough_segmentation_parameters &hough_params = m_hough_segmentation_parameters;

	hough_params.inverse_ratio_accumulator_resolution = g_configFile.read_float("AUTODOCKING","inverse_ratio_accumulator_resolution",0,true);	
	hough_params.min_dist = g_configFile.read_float("AUTODOCKING","min_dist",0,true);	
	hough_params.higher_canny_threshold = g_configFile.read_float("AUTODOCKING","higher_canny_threshold",0,true);	
	hough_params.accumulator_threshold	= g_configFile.read_float("AUTODOCKING","accumulator_threshold",0,true);	
	hough_params.min_radius	= g_configFile.read_float("AUTODOCKING","min_radius",0,true);	
	hough_params.max_radius = g_configFile.read_float("AUTODOCKING","max_radius",0,true);	
	hough_params.gaussian_blur_size		= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_size",0,true);
	hough_params.gaussian_blur_sigma_1	= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_sigma_1",0,true);
	hough_params.gaussian_blur_sigma_2	= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_sigma_2",0,true);

	// 3.9 Canny closed contours segmentation
	canny_segmentation_parameters &canny_params = m_canny_segmentation_parameters;

	canny_params.gaussian_blur_size		= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_size",0,true);
	canny_params.gaussian_blur_sigma_1	= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_sigma_1",0,true);
	canny_params.gaussian_blur_sigma_2	= g_configFile.read_uint64_t("AUTODOCKING","gaussian_blur_sigma_2",0,true);
	canny_params.higher_threshold		= g_configFile.read_float("AUTODOCKING","higher_canny_threshold",0,true);

	// 3.10 Tracking window
	m_tracking_window.window_size	= g_configFile.read_float("AUTODOCKING","window_size",0,true);
	g_configFile.read_vector( "AUTODOCKING", "window_adjustment_straight_line",vector<float>(0),m_tracking_window.v_window_adjustment_straight_line,true );
	g_configFile.read_vector( "AUTODOCKING", "window_adjustment_turn_left",vector<float>(0),m_tracking_window.v_window_adjustment_turn_left,true );
	g_configFile.read_vector( "AUTODOCKING", "window_adjustment_turn_right",vector<float>(0),m_tracking_window.v_window_adjustment_turn_right,true );

	
	// 4. Retrieve a pointer to the motors communication object (if needed)

	if ( !m_no_motors_mode )
	{
		if ( commonData.modulesUsed.motorsCom )
		{
			m_motors = static_cast<CMotorsCom*>( commonData.modules_pointers["MOTORS"] );
			double angle = 0.4;
			m_motors->execute("setTiltAngleFromHome", &angle );
		}
		else
			throw std::logic_error("A reference to a motors communication object is needed");
	}

	// 5. Establish the initial state

	m_last_movement.last_movement	= TURN_PATTERN_WAS_NOT_DETECTED;
	m_last_movement.last_turn_mov_q	= 0;
	m_last_movement.last_straight_mov_q	= 0;

	m_detected_pattern.was_detected = false;

	m_turn_complete_tester.reset();	// Reset the turn complete without detect the pattern tester

	m_mode = FAR_MODE;

	writeDebugLine("Initialization process succesfully completed",AUTODOCKING);

	NAV_CATCH_MODULE("While initializing")

}


//-----------------------------------------------------------
//							run
//-----------------------------------------------------------

void CAutoDocking::run()
{
	m_thread_autoDocking = mrpt::system::createThread( dummyAutoDocking, this );
}


//-----------------------------------------------------------
//					dummyAutoDocking
//-----------------------------------------------------------

void CAutoDocking::dummyAutoDocking( CAutoDocking *obj )
{
	dynamic_cast<CAutoDocking*>(obj)->autoDocking();
}


//-----------------------------------------------------------
//					changeNoMotorsMode
//-----------------------------------------------------------

void CAutoDocking::changeNoMotorsMode( const bool &noMotors )
{
	if ( noMotors )
	{
		m_no_motors_mode = true;
		m_motors = NULL;
	}
	else
	{
		m_motors = static_cast<CMotorsCom*>( commonData.modules_pointers["MOTORS"] );
		//autodocking original
		/*size_t mode = 1;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setVel", &m_robot_speed, &mode );*/
		
		//Test
		size_t mode = 2;
		double vel=0.1,wdeg=0,pos=1,poscero=0;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
		m_motors->execute("setVel", &vel, &mode );
		m_motors->execute("setPos", &pos);
		//----

		double angle = 0.4;
		m_motors->execute("setTiltAngleFromHome", &angle );

		m_no_motors_mode = false;
	}
}

//-----------------------------------------------------------
//					constrain_radius
//-----------------------------------------------------------

bool CAutoDocking::constrain_radius( const float &radius ) const
{
	/*****

		y = ones(161,1);
		p1 =   -0.000116;
		p2 =    0.006971;
		p3 =       4.779;

		for x=0:1:160
		    y(x+1) = p1*x^2 + p2*x + p3;
		end
		x=0:1:160;

	*****/

	if ( ( radius < m_pattern_conf.m_max_circle_radius[m_mode] ) && ( radius > m_pattern_conf.m_min_circle_radius[m_mode] ) ) 
		return true;
	else
		return false;
}

//-----------------------------------------------------------
//					 obtainMaskMat
//-----------------------------------------------------------

void CAutoDocking::obtainMaskMat( const Mat &channel_mat, const size_t &channel_num, int inf_threshold, int sup_threshold, Mat &mask, const size_t &colour_index )
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
		imshow( mrpt::utils::format("Channel %d mask of colour %d", channel_num, colour_index), mask);
		cvWaitKey( 5 );	
	}
}


//-----------------------------------------------------------
//					obtainSegmentedMat
//-----------------------------------------------------------

void CAutoDocking::obtainSegmentedMat( const Mat &frame, Mat &segmented, const size_t &colour_index )
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
		imshow(mrpt::utils::format("mask for colour %d",colour_index), final_mask);
			
	Mat aux_segmented;
	frame.copyTo( aux_segmented, final_mask );
	
	if ( m_display.hsv_final_masks )
		imshow(mrpt::utils::format("Final for colour %d",colour_index), aux_segmented);
	

	final_mask.copyTo( segmented );				
	

	if ( m_display.hsv_partial_masks || m_display.hsv_final_masks )
		cvWaitKey( 5 );
}


//-----------------------------------------------------------
//					debug_pattern_detection
//-----------------------------------------------------------

void CAutoDocking::debug_pattern_detection( vector<vector<TCircle>> &candidates, const size_t &i_colour )
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
	imshow( mrpt::utils::format("Candidates step: %d", i_colour), drawing );
	cvWaitKey(5);	
}


//-----------------------------------------------------------
//				computeErrorAdjustingALine
//-----------------------------------------------------------

float CAutoDocking::computeErrorAdjustingALine( const TNCircles &v_final_candidates, TLine2D &adjusted_line )
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

void CAutoDocking::searchPattern_afterCanny( const vector<vector<Point2f>> &v_center, 
										     const vector<vector<float>> v_radius )
{
	// Currently not used
}


//-----------------------------------------------------------
//					  searchPattern_afterHsvOrHough
//-----------------------------------------------------------

void CAutoDocking::searchPattern_afterHsvOrHough( vector<vector<Point2f>> &v_center, 
												  vector<vector<float>> &v_radius )
{
	NAV_TRY

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

	NAV_CATCH_MODULE("In search pattern method");

	if ( m_save_executions_time )
		m_timeLog.leave("Search pattern");
}


//-----------------------------------------------------------
//					  segmentation_hsv
//-----------------------------------------------------------

void CAutoDocking::segmentation_hsv( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius )
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
			imshow( mrpt::utils::format("Contours colour %d",i), drawing );
			cvWaitKey(5);
		}
	}

}


//-----------------------------------------------------------
//				segmentation_cannyFilledPolygons
//-----------------------------------------------------------

void CAutoDocking::segmentation_cannyFilledPolygons( Mat &frame, 
												   vector<vector<Point2f>> &center, 
												   vector<vector<float>> &radius )
{
	NAV_TRY

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

	NAV_CATCH_MODULE("In segmentation_cannyFilledPolygons method");

	if ( m_save_executions_time )
		m_timeLog.leave("Canny segmentation");

}


//-----------------------------------------------------------
//					segmentation_houghCircles
//-----------------------------------------------------------

void CAutoDocking::segmentation_houghCircles( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius )
{
	NAV_TRY

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

	NAV_CATCH_MODULE("In segmentation_houghCircles method");
}

void CAutoDocking::obtainHistogramInformation( const Mat &frame )
{
	NAV_TRY

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

			imshow( mrpt::utils::format("Test %d", i_contour), test );
		
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
			file.open( mrpt::utils::format("Histogram %d.txt", i_contour ).c_str(), ofstream::app );

			for( int h = 0; h < hbins; h++ )
				/*for( int s = 0; s < sbins; s++ )
				{
					float binVal = hist.at<float>(h, s);
					int intensity = cvRound(binVal*255/maxVal);
					rectangle( histImg, Point(h*scale, s*scale),
								Point( (h+1)*scale - 1, (s+1)*scale - 1),
								Scalar::all(intensity),
								CV_FILLED );
				}*/
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

			imshow( mrpt::utils::format("H-S Histogram %d",i_contour), histImg );
		}
	}

	save_count++;

	NAV_CATCH_MODULE("In obtainHistogramInformation method");
}


//-----------------------------------------------------------
//				fixPossibleImageLimitsViolations
//-----------------------------------------------------------

void CAutoDocking::fixPossibleImageLimitsViolations( double &top_left, double &top_right, double &top_up , double &top_down )
{
	top_left = ( top_left < m_frame_features.initial_ROI_col_left_limit ) ? m_frame_features.initial_ROI_col_left_limit : top_left;
	top_right = ( top_right > m_frame_features.initial_ROI_col_right_limit ) ? m_frame_features.initial_ROI_col_right_limit : top_right;
	top_up = ( top_up < m_frame_features.initial_ROI_row_upper_limit ) ? m_frame_features.initial_ROI_row_upper_limit : top_up;
	top_down = ( top_down > m_frame_features.initial_ROI_row_lower_limit ) ? m_frame_features.initial_ROI_row_lower_limit : top_down;	
}


//-----------------------------------------------------------
//					patternLocalization
//-----------------------------------------------------------

void CAutoDocking::obtainROI( const Mat &fullFrame, Mat &frame )
{
	NAV_TRY

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

	imwrite(mrpt::utils::format("./tracking/img%d.jpg",frame_tracking_count), frame );

	frame_tracking_count++;	
		
	NAV_CATCH_MODULE("In obtainROI method");

	if ( m_save_executions_time )
		m_timeLog.leave("Obtain ROI");
}

//-----------------------------------------------------------
//					patternLocalization
//-----------------------------------------------------------

void CAutoDocking::patternLocalization()
{
	if ( m_save_executions_time )
		m_timeLog.enter("Pattern localization");

	NAV_TRY

	// Useful variables declaration
	Mat fullFrame;
	Mat frame;

	vector<vector<Point2f>>			center;
	vector<vector<float>>			radius;

	center.resize( m_colours_to_detect );
	radius.resize( m_colours_to_detect );

	// Here we go!!!! 
	
	// 1. Obtaining a new frame...

	bool obtained = false;

	while( !obtained )
	{
		g_cameraImage.working();

		if ( g_cameraImage.isFresh("AUTODOCKING") )
		{
			g_cameraImage.setFresh("AUTODOCKING",false);
			g_cameraImage.getImage( fullFrame );
			obtained = true;
		}

		g_cameraImage.endWorking();

		mrpt::system::sleep(1);
	}

	obtainROI( fullFrame, frame );	

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

	imwrite(mrpt::utils::format("./frames/img%d.jpg",frame_count), fullFrame );

	frame_count++;	

	NAV_CATCH_MODULE("In obtainPatternLocation method");
	
	if ( m_save_executions_time )
		m_timeLog.leave("Pattern localization");
}


//-----------------------------------------------------------
//				 patternLocalizationProcess
//-----------------------------------------------------------

void CAutoDocking::patternLocalizationProcess()
{
	m_detected_pattern.last_was_detected = false; // Reset last detection to false
		
	//CTicTac clock;
	//ofstream	file;
	//file.open( "Process time.txt", ofstream::app );

	for ( size_t i_try = 0; i_try < m_tries_detecting_pattern; i_try++ )
	{
		//clock.Tic();
		patternLocalization();

		//file << clock.Tac() << endl;

		if ( m_detected_pattern.last_was_detected )
			break;		
	}

	//file.close();
}

//-----------------------------------------------------------
//					  checkGoodOriented
//-----------------------------------------------------------

void CAutoDocking::changeMode( const TMode &new_mode )
{
	NAV_TRY

	//double angleFarMode = 0.4;
	//double angleNearMode = 0.08727;
	double angleFarMode = m_angleFarMode; // 0.71273;
	double angleNearMode = m_angleNearMode; //0.4;
	double currTilt;

	switch( new_mode )
	{
	case FAR_MODE:
		m_motors->execute("setTiltAngleFromHome", &angleFarMode );
		m_mode = FAR_MODE;		
		m_motors->execute("getTiltAngleFromHome", &currTilt );
		writeDebugLine(mrpt::utils::format("Mode changed to FAR_MODE, current tilt: %f",currTilt),AUTODOCKING);
		break;

	case NEAR_MODE:
		m_motors->execute("setTiltAngleFromHome", &angleNearMode );
		m_mode = NEAR_MODE;
		m_motors->execute("getTiltAngleFromHome", &currTilt );
		writeDebugLine(mrpt::utils::format("Mode changed to NEAR_MODE, current tilt: %f",currTilt),AUTODOCKING);		
		break;

	default:
		throw std::logic_error("Incorrect performance mode, it doesn't exist!!!");
	}

	NAV_CATCH_MODULE("In changeMode method");
}

//-----------------------------------------------------------
//					  checkGoodOriented
//-----------------------------------------------------------

void CAutoDocking::checkGoodOriented()
{
	NAV_TRY

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
		
	}

	NAV_CATCH_MODULE("In checkGoodOriented method");
}


//-----------------------------------------------------------
//						turnABit
//-----------------------------------------------------------

void CAutoDocking::turnABit()
{
	NAV_TRY

	if ( m_save_executions_time )
		m_timeLog.enter("Turn a bit");

	writeDebugLine("Turning a bit!",AUTODOCKING);

	// Check if we are using the motors
	if ( !m_no_motors_mode )
	{
		// 1. Establish turn movement	

		//Autodocking original
		/*double radius = 0;
		m_motors->execute("setRadius", &radius );*/ 
		
		//test
		/*
		size_t mode = 2;
		double vel=0,wdeg=15,pos=1,poscero=0;
		m_motors->execute("setMode",&mode);//traza motors mode
		m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
		m_motors->execute("setVel", &vel, &mode );
		m_motors->execute("setVel", &pos, &mode );
		*/
		//----


		// 2. Check if the pattern has been seen, for rotating in the correct direction!
		
			//test
			double wdeg2;
			//----

		if ( m_detected_pattern.last_was_detected )
		{
			double direction = m_detected_pattern.last_location.obtainCenterCol() - m_pattern_conf.m_final_pattern_col[m_mode];

			float current_col = m_detected_pattern.last_location.obtainCenterCol();

			double control_action = 1 + 
				(abs(( m_pattern_conf.m_final_pattern_col[m_mode] - current_col )) / m_pattern_conf.m_final_pattern_col[m_mode] )*m_kp_turn;

			double degrees; // Degrees to turn

			if ( direction > 0 ) // Turn right
			{
				//autodocking original
				//degrees = m_degrees_turning_per_step*control_action;

				//test
				degrees = -5;
				//----

				//Update last movement
				updateLastMovement( TURN_RIGHT_PATTERN_WAS_DETECTED, degrees );
			}
			else // Turn left
			{
				//autodocking original
				//degrees = -(double)m_degrees_turning_per_step*control_action;
				
				
				//test
				degrees = 5;
				//----

				//Update last movement
				updateLastMovement( TURN_LEFT_PATTERN_WAS_DETECTED, degrees );
			}			

			//Autodocking original
			//m_motors->execute("setPos", &degrees ); // Turn

			
			//test
			size_t mode = 2;
			double vel=0,wdeg=degrees,pos=1,poscero=0;
			wdeg=degrees;
			m_motors->execute("setMode",&mode);//traza motors mode
			m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
			m_motors->execute("setVel", &vel, &mode );
			m_motors->execute("setPos", &pos);
			//----
	

			writeDebugLine(mrpt::utils::format("DockStation detected but not in front of the robot (%f,%f), performing rotative movement: %f degrees"
				, m_detected_pattern.last_location.obtainCenterCol()
				, m_detected_pattern.last_location.obtainCenterRow()
				, degrees)
				,AUTODOCKING);
		}
		else // Pattern not saw, rotate in the usual direction
		{
			double degrees_to_turn;

			if ( !m_detected_pattern.was_detected )	// The pattern wasn't detected any time!
			{
				//linea de autodocking original
				degrees_to_turn = m_degrees_turning_per_step*m_kp_turn;

				//test
				if(degrees_to_turn>0)
					wdeg2=-15;
				else if(degrees_to_turn<0)
					wdeg2=+15;
				else if(degrees_to_turn==0)
					wdeg2=0;
				//
			}
			else	// If the pattern was detected, turn in the same direction that the last time
			{
				//linea de autodocking original
				degrees_to_turn = m_last_movement.last_turn_mov_q;

				//test
				if(degrees_to_turn>0)
					wdeg2=15;
				else if(degrees_to_turn<0)
					wdeg2=-15;
				else if(degrees_to_turn==0)
					wdeg2=0;
				//

			}

			//autodocking original
			//m_motors->execute("setPos", &degrees_to_turn ); // Turn

			//test
			size_t mode = 2;
			double vel=0,wdeg=wdeg2,pos=1,poscero=0;
			wdeg=degrees_to_turn;
			m_motors->execute("setMode",&mode);//traza motors mode
			m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
			m_motors->execute("setVel", &vel, &mode );
			m_motors->execute("setPos", &pos);
			//----

			// Update last movement
			updateLastMovement( TURN_PATTERN_WAS_NOT_DETECTED, degrees_to_turn );

			writeDebugLine(mrpt::utils::format("DockStation not detected, performing rotative movement: %f degrees",degrees_to_turn),AUTODOCKING);
		}
	}

	NAV_CATCH_MODULE("In turnABit method");

	if ( m_save_executions_time )
		m_timeLog.leave("Turn a bit");
}

//-----------------------------------------------------------
//					updateLastMovement
//-----------------------------------------------------------

void CAutoDocking::updateLastMovement( const TLast_movement_command &movement, 
									  const double &move_or_degrees_quantity )
{
	NAV_TRY 

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

			g_odometry.working();			
					
			if ( g_odometry.isFresh("AUTODOCKING") )
			{
				g_odometry.setFresh("AUTODOCKING", false);
				g_odometry.getOdometry( currPose, time );

				vector_double v_pose;
				currPose.getAsVector( v_pose );
				m_turn_complete_tester.setLastOrientation( v_pose[2] );
					
				if ( !m_turn_complete_tester.isTurning() )
					m_turn_complete_tester.setTurning( true );										
			}

			g_odometry.endWorking();
				
			m_last_movement.last_turn_mov_q	= move_or_degrees_quantity;			
			break;

		default:
			throw std::logic_error("Incorrect last movement, it doesn't exist!!!");
	}

	m_last_movement.time = mrpt::system::now();
	m_last_movement.last_movement	= movement;

	NAV_CATCH_MODULE("In updateLastMovement method");
	
}

//-----------------------------------------------------------
//					performGoodOrientation
//-----------------------------------------------------------

bool CAutoDocking::performGoodOrientation()
{
	NAV_TRY

	m_good_oriented = false;
	
	checkGoodOriented();

	while ( !m_good_oriented && g_state.equalTo( CState::NAV_AUTODOCKING ) )
	{
		turnABit();
		
		checkGoodOriented();

		if ( m_turn_complete_tester.check() ) // We have completed a turn without find the docking station!
		{
			if ( m_mode == FAR_MODE )
			{
				changeMode( NEAR_MODE );
				writeDebugLine("A turn completed without find the docking station in FAR_MODE!... changing to NEAR_MODE",AUTODOCKING);
				m_turn_complete_tester.reset();
			}
			else
			{
				writeDebugLine("A turn completed without find the docking station!",AUTODOCKING);
				break;
			}
		}
	}

	return m_good_oriented;

	NAV_CATCH_MODULE("In performGoodOrientation method");

	return false;
}


//-----------------------------------------------------------
//					moveInStraightLineABit
//-----------------------------------------------------------

void CAutoDocking::moveInStraightLineABit()
{
	NAV_TRY

	if ( m_save_executions_time )
		m_timeLog.enter("Move in straight line a bit");

	// Check if we are using the motors
	if ( !m_no_motors_mode )
	{
		// Obtain the num of meters to move
		float row = m_detected_pattern.last_location.obtainCenterRow();
		// The control action has a value between [1..m_kp_move_in_straight_line]
		float control_action = 1 + (abs(( m_pattern_conf.m_final_pattern_row[m_mode] - row )) / m_pattern_conf.m_final_pattern_row[m_mode])*m_kp_move_in_straight_line;
		float move = m_meters_moving_per_step * control_action;

		//autodocking original
		/*
		double radius = 60;
		m_motors->execute("setRadius", &radius );
		m_motors->execute("setPos", &move );	*/

		//test
			size_t mode = 2;
			double vel=0.1,wdeg=0,pos=1,poscero=0;
			m_motors->execute("setMode",&mode);//traza motors mode
			m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
			m_motors->execute("setVel", &vel, &mode );
			m_motors->execute("setPos", &pos);
			//----
		
		//Update last movement
		updateLastMovement( STRAIGHT_LINE, move );

		writeDebugLine(mrpt::utils::format("DockStation detected in front of the robot (%f,%f), performing linear movement: %f meters"
					, m_detected_pattern.last_location.obtainCenterCol()
					, m_detected_pattern.last_location.obtainCenterRow()
					, move)
					,AUTODOCKING);
	}

	// Check if we have to change the mode
	if ( m_mode == FAR_MODE )
	{
		CPoint2D detected_pattern_location( m_detected_pattern.last_location.obtainCenterCol(),
											m_detected_pattern.last_location.obtainCenterRow() );

		float distance = detected_pattern_location.distance2DTo( m_pattern_conf.m_final_pattern_col[m_mode],
																 m_pattern_conf.m_final_pattern_col[m_mode] );

		writeDebugLine(mrpt::utils::format("Distance to the final pattern position: %f pixels", distance ),AUTODOCKING);

		if ( distance < m_distance_for_changing_mode )
		{
			changeMode( NEAR_MODE );
			writeDebugLine( "Changing to NEAR_MODE!", AUTODOCKING );
		}
	}

	NAV_CATCH_MODULE("In moveInStraightLineABit method");

	if ( m_save_executions_time )
		m_timeLog.leave("Move in straight line a bit");
}


//-----------------------------------------------------------
//					checkGoalAchieved
//-----------------------------------------------------------

bool CAutoDocking::checkGoalAchieved()
{
	NAV_TRY

	if ( m_save_executions_time )
		m_timeLog.enter("Check goal achieved");

	m_timeLog.enter("Pattern localization");

	if ( !m_no_motors_mode )
	{ 

		string info;
		m_motors->execute("getChargerStatus", &info );
		
		// The getChargerStatus returns a string with the format:
		// S:S#XXXX,......
		//
		// Where: The first two hexadecimal values is the fast charging
		// termination, the third is warnings and the forth is the status of the charger.
		//
		// The status of the charger has five states:
		//	Fast charging = 0
		//	Trickle charging = 1
		//	Charger in error state = 2
		//	Init state = 3
		//	Not docked = 4
		//
		// So we can analyze the 7th position of the returned string. If it's a 4, then
		// we haven't completed the docking task.

		if ( info.size() > 7 )
		{		
			if ( (char)(info[7]) != '4' )
			{
				writeDebugLine("Docking complete!",AUTODOCKING);

				if ( m_save_executions_time )
					m_timeLog.leave("Check goal achieved");

				return true;	// Docked
			}
			else
			{
				writeDebugLine(mrpt::utils::format("Not docked yet, value: %c",(char)info[7]), AUTODOCKING);
				if ( m_save_executions_time )
					m_timeLog.leave("Check goal achieved");

				return false;	// Not docked
			}
		}
	}
	
	NAV_CATCH_MODULE("In checkGoalAchieved method");

	if ( m_save_executions_time )
		m_timeLog.leave("Check goal achieved");

	return false;	

}


//-----------------------------------------------------------
//				  simpleReactiveNavigation
//-----------------------------------------------------------

size_t CAutoDocking::simpleReactiveNavigation()
{
	NAV_TRY

	changeMode( FAR_MODE );

	// Sleeping time for giving time to a correct head tilt initialization
	mrpt::system::sleep(2500); // 3 seconds

	double original_robot_speed;

	// Store original robot velocity and set the autodocking velocity
	{
		m_motors->execute("getVel", &original_robot_speed);
		
		//autodocking original
		/*
		size_t mode = 1;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setVel", &m_robot_speed, &mode );
		*/

		//Test
		size_t mode = 2;
		double vel=0.1,wdeg=0,pos=1,poscero=0;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
		m_motors->execute("setVel", &vel, &mode );
		m_motors->execute("setPos", &pos);
		//----

	}

	if ( performGoodOrientation() )	// Check if we can find the docking station and it is reachable!
	{
		while ( m_run && !m_stop && !g_state.equalTo( CState::NAV_TELEOPERATED ) )
		{	
			moveInStraightLineABit(); 

			if (checkGoalAchieved())
			{
				writeDebugLine("Goal achieved! =)",AUTODOCKING);
				break; // We have reached the goal!
			}

			if ( !performGoodOrientation() ) // If not, check if well oriented
				break; // Launch dock station missed!					
		}
	}

	// Restore original robot velocity
	{
		//autodocking original
		/*
		size_t mode = 1;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setVel", &original_robot_speed, &mode );*/

		//Test
		size_t mode = 2;
		double vel=original_robot_speed,wdeg=0,pos=1,poscero=0;
		m_motors->execute("setMode",&mode);//traza motors mode 
		m_motors->execute("setMaximumVirtualGearRatio", &wdeg);
		m_motors->execute("setVel", &vel, &mode );
		m_motors->execute("setPos", &pos);
		//----
	}

	return 0;

	NAV_CATCH_MODULE("In simpleReactiveNavigation method");

	return 0;
}

//-----------------------------------------------------------
// 					 saveMeasuresToFile
//-----------------------------------------------------------

void CAutoDocking::saveMeasuresToFile()
{
	if ( m_save_executions_time )
	{
		ofstream file;

		file.open( "stats.txt", ofstream::app );

		if ( file.is_open() )
		{
			file << m_timeLog.getStatsAsText();
		}

		file.close();	

		cout << "Autodocking measures saved to file" << endl;
	}
	else
	{
		cout << "Take measures option does not launched" << endl;
	}
}

//-----------------------------------------------------------
//						autoDocking
//-----------------------------------------------------------

void CAutoDocking::autoDocking()
{
	NAV_TRY

	m_run = true;

	while ( !m_stop )
	{
		while ( !g_state.equalTo( CState::NAV_AUTODOCKING ) )
			mrpt::system::sleep(200);

		{
			//autodocking original
			/*
			size_t navigationMode=1;
			m_motors->execute("setMode",&navigationMode);
			m_motors->execute("getMode",&navigationMode);*/

				//Test
				size_t mode = 2;
				m_motors->execute("setMode",&mode);//traza motors mode 
				writeDebugLine(mrpt::utils::format("Starting autodocking with navigation mode: %d",mode));
				//----
			//writeDebugLine(mrpt::utils::format("Starting autodocking with navigation mode: %d",navigationMode));
		}

		if ( m_reactive_navigation_algorithm == 0 )
			simpleReactiveNavigation();		

		// Stop all possible remaining movement commands
		double pos = 0;
		size_t restoreModeTwo=2;

		m_motors->execute("setMode",&restoreModeTwo);
		m_motors->execute("setMaximumVirtualGearRatio", &pos);
		m_motors->execute("setVel", &pos, &restoreModeTwo );
		m_motors->execute("setPos", &pos);

		g_state.change( CState::NAV_TELEOPERATED );

			

	}

	NAV_CATCH_MODULE("In autoDocking method");
}