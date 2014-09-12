	
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


#ifndef CAutoDockingCamera_H
#define CAutoDockingCamera_H

#include <COpenMORAMOOSApp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <mrpt/system/datetime.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/hwdrivers/CCameraSensor.h>

#include <mrpt/math.h>

using namespace cv;
using namespace std;
using namespace mrpt::math;

class CAutoDockingCamera : public COpenMORAApp
{
public:
    CAutoDockingCamera();
    virtual ~CAutoDockingCamera();

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

	void saveMeasuresToFile();

private:

	bool Init();
	bool init;
	bool get_new_image(cv::Mat &frame);
	cv::Mat last_image_obs;
	mrpt::hwdrivers::CCameraSensor  m_camera;	//!< The camera object. See MRPT docs.
	////////////////////////////////////////// VARIABLES //////////////////////////////////////////
	std::string input_image_variable;			//!< Name of the OpenMORA variable from where to get the input images
	std::string input_image_method;
	int movement_mode;
	bool module_active;

	size_t	m_segmentation_algorithm;			//!< the segmentation algorithm used. 0: HSV, 1: Hough circles, 2: Canny filled polygons

	size_t	m_tries_detecting_pattern;			//!< Number of times trying to detect the pattern in each patternLocalizationProcess

	size_t	m_degrees_turning_per_step;			//!< Degrees for turning
	double	m_meters_moving_per_step;			//!< Meters for advancing in straight line

	double	m_threshold_considered_as_center;	//!< Threshold for considering that a pattern is in the center of the image

	double	m_kp_move_in_straight_line;			//!< Proportional constant for multiplying the quantity of movement in straight line
	double	m_kp_turn;							//!< Proportional constant for multiplying the quantity of movement while turning
	double	m_kp_consider_as_center_distance_relaxation;	//!< Proportional constant for multiplying the threshold for considering that a pattern is in the center of the area

	double	m_robot_speed;						//!< Robot speed while performing autodocking
	double m_max_linear_speed, m_max_angular_speed;		//!< Robot speed while performing autodocking
		
	bool	m_save_histogram_information;		//!< Specify if we want to save information relative to the histogram of the pattern

	bool	m_save_executions_time;				//!< Save the execution times of different process

	bool	m_save_pattern_constrains;			//!< Useful when we are obtaining the contrains of a certain pattern

	size_t	m_colours_to_detect;				//!< Number of different colours to detect


		
	/** State variables */

		mrpt::system::TThreadHandle	m_thread_autoDocking;	//!< Autodocking thread handler

		bool m_good_oriented; //!< I'm good oriented?		

		enum TLast_movement_command{ STRAIGHT_LINE, 
							TURN_RIGHT_PATTERN_WAS_DETECTED, 
							TURN_LEFT_PATTERN_WAS_DETECTED, 
							TURN_PATTERN_WAS_NOT_DETECTED }; //!< Of what type was the last movement?

		enum TMode{ FAR_MODE=0, NEAR_MODE }m_mode;	//!< Whe are in the FAR o NEAR mode?

		size_t m_distance_for_changing_mode; //!< distance where we chage between FAR_MODE and NEAR_MODE

		double	m_angleFarMode;		//!< Tilt angle in far mode
		double	m_angleNearMode;	//!< Tilt angle in near mode

		struct movement
		{
			double	last_turn_mov_q;		//!< Store the last quantity of turn movement (degrees)
			double	last_straight_mov_q;	//!< Store the last quantity of turn movement (meters)	
			TLast_movement_command	last_movement;	//!< Of what type was the last movement?
			mrpt::system::TTimeStamp	time;			//!< When was it ordered?
		}m_last_movement; //!< Information relative to the las movement performed
		
		mrpt::utils::CTimeLogger	m_timeLog;	//!< To take measures about execution times
		

		////////////////////////////////////////// STRUCTURES //////////////////////////////////////////

		/** Soft type for storing circles
		  */
		struct TCircle
		{
			Point2f center;	//!< Center of the circle
			float	radius;	//!< Radius of the circle

			/** Default constructor.
			  */
			TCircle(){};

			/** Constructor.
			  * \param c point correspondant to the center of the circle.
			  * \param r radius of the circle.
			  */
			TCircle( const Point2f &c, const float &r) : center(c), radius(r) {};

			/** Copy constructor 
			  */
			TCircle operator=(const TCircle &c2){ this->center=c2.center; this->radius = c2.radius; return *this; };
	
			/** Overloaded operator ==
			  */
			bool operator==(const TCircle &c1)
			{ 
				if (( c1.center == this->center )&&( c1.radius == this->radius ) ) 
					return true; 
				else 
					return false; 
			}

			/** Print method for showing the center and the radius of a circle 
			  */
			void print(){ cout << "[" << center << "," << radius; }

			/** Overloaded operator <<
			  */
			friend ostream &operator<<(ostream &output, const TCircle &c)
			{ 
				output << "[" << c.center << "," << c.radius << "]"; 
				return output; 
			}
			
		}; //end-struc TCircle


		/** Soft type for storing a vector of N circles.
		  */
		struct TNCircles
		{
			vector<TCircle> v_circles;	//!< Vector of circles.

			/** Default constructor.
			  */
			TNCircles(){};

			/** Constructor from two TCircles.
			  */
			TNCircles( const TCircle &c1, const TCircle &c2)
			{ 
				v_circles.resize(2); 
				v_circles[0] = c1; 
				v_circles[1] = c2; 
			}

			/** Method inspired on the vector std type.
			  * \param c1 new circle to add.
			  *	\return this object.
			  */
			TNCircles push_back( const TCircle &c1 ) 
			{ 
				v_circles.push_back( c1 ); 
				return *this; 
			}

			/** Method inspired on the vector std type.
			  * \param i index of the circle to retrieve
			  * \return	the circle placed at the i position in the vector
			  */
			TCircle operator[]( const size_t &i )
			{ 
				return this->v_circles[i]; 
			}

			/** Method inspired on the vector std type.
			  * \return the size of the vector of circles.	
			  */
			size_t size() const 
			{ 
				return v_circles.size(); 
			}

			/** Method inspired on the vector std type.
			  */
			void clear()
			{ 
				v_circles.clear(); 
			}

			/*TNCircles operator=( const TNCircles &v_c1 )
			{	size_t N = v_c1.v_circles.size(); 
				this->v_circles.resize( N ); 
				for ( size_t i = 0; i < N; i++ )
					this->v_circles[i] = v_c1.v_circles[i];
			};*/			

			/** Print the content of the vector
			  */
			void print ()
			{ 
				for( size_t i = 0; i < v_circles.size(); i++ ) 
					cout << v_circles[i]; 
			}

			/** Overloaded operator <<
			  */
			friend ostream& operator<<( ostream &out, const TNCircles &v_c ) 
			{ 
				for( size_t i = 0; i < v_c.v_circles.size(); i++ ) 
					cout << v_c.v_circles[i]; 

				return cout;
			}

			/** Checks if a circle is already in the vector.
			  * \param cirlce circle to check.
			  * \return true if it is in the vector, otherwise false.
			  */
			bool contains( const TCircle &circle ) 
			{ 
				for ( size_t i=0; i < v_circles.size(); i++)
				{ 
					if ( v_circles[i]==circle ) return true; 
				} 
				return false;
			}

			/** Converts the centers of the stored circles to a vector of MRPT points.
			  * \centers object for storing the conversion.
			  */
			void getCentersAsTPoint2DVector( vector<TPoint2D> &centers ) const
			{
				size_t N = v_circles.size();
				centers.resize( N );
				for ( size_t i = 0; i < N; i++ )
					centers[i] = TPoint2D( v_circles[i].center.x, v_circles[i].center.y );
			}

			/** Calculate the center col of the centers a set of circles.
			  * \return the computed center col.
			  */
			double obtainCenterCol() const
			{
				double center_col = 0;
				size_t N = v_circles.size();
				for ( size_t i = 0; i < N; i++ )
					center_col += v_circles[i].center.x;

				return center_col/N;

			}

			/** Calculate the center row of the centers a set of circles.
			  * \return the computed center row.
			  */
			double obtainCenterRow() const
			{
				double center_row = 0;
				size_t N = v_circles.size();
				for ( size_t i = 0; i < N; i++ )
					center_row += v_circles[i].center.y;

				return center_row/N;

			}

			/** Obtain the left, right, up and down pixel limits.
			  * \param top_left top left limit.
			  * \param top_right top right limit.
			  * \param top_up top up limit.
			  * \param top_down top down limit.
			  */
			void getTopPixels( double &top_left, double &top_right, double &top_up , double &top_down )
			{
				_ASSERT( v_circles.size() > 0 );

				size_t i_circle_top_left = SIZE_MAX, i_circle_top_right = 0, i_circle_top_up = SIZE_MAX, i_circle_top_down = 0;
				double circle_top_left = SIZE_MAX, circle_top_right = 0, circle_top_up = SIZE_MAX, circle_top_down = 0;

				for ( size_t i_circle=0; i_circle < v_circles.size(); i_circle++ )
				{
					TCircle circle_to_test = v_circles[i_circle];

					if ( circle_to_test.center.x < circle_top_left )
					{
						circle_top_left = circle_to_test.center.x;
						i_circle_top_left = i_circle;
					}
					if ( circle_to_test.center.x > circle_top_right )
					{
						circle_top_right = circle_to_test.center.x;
						i_circle_top_right = i_circle;
					}
					if ( circle_to_test.center.y < circle_top_up )
					{
						circle_top_up =  circle_to_test.center.y;
						i_circle_top_up = i_circle;
					}
					if ( circle_to_test.center.y > circle_top_down )
					{
						circle_top_down =  circle_to_test.center.y;
						i_circle_top_down = i_circle;
					}
				}

				//cout << endl << "Index: " << i_circle_top_left << " " << i_circle_top_right << " " << i_circle_top_up << " " << i_circle_top_down << endl;

				top_left = circle_top_left - v_circles[i_circle_top_left].radius;
				top_right = circle_top_right + v_circles[i_circle_top_right].radius;
				top_up = circle_top_up - v_circles[i_circle_top_up].radius;
				top_down = circle_top_down + v_circles[i_circle_top_down].radius;

				//writeDebugLine( mrpt::utils::format("t_l:%f t_r:%f t_u:%f t_d:%f",pattern_top_left, pattern_top_right, pattern_top_up, top_down ), AUTODOCKING);
			}

			/** Apply a certain offset to the center of all the circles.
			  * \param colOffset number of pixels for the columns offset.
			  * \param rowOffset number of pixels for the row offset.
			  */
			void applyCenterOffset( const float &colOffset, const float &rowOffset )
			{
				for ( size_t i_circle = 0; i_circle < v_circles.size(); i_circle++ )
				{
					v_circles[i_circle].center.x += colOffset;
					v_circles[i_circle].center.y += rowOffset;
				}
			}

		}; // End-struct TNCircles


		/** Struct for storing information about the tracking window 
		  */
		struct tracking_window
		{
			float			window_size;	//!< Size of the window
			size_t			up_left_corner_row;	//!< Pixel of the top left corner row
			size_t			up_left_corner_col; //!< Pixel of the top left corner col
			vector<float>	v_window_adjustment_straight_line; //!< Vector loaded from file with how to adjust when we have performed a stright line movement
			vector<float>	v_window_adjustment_turn_left;	//!< Vector loaded from file with how to adjust when we have performed a turn left movement
			vector<float>	v_window_adjustment_turn_right; //!< Vector loaded from file with how to adjust when we have performed a turn right
		}m_tracking_window; //!< Tracking pattern window


		/** Unused struct 
		  */
		struct simple_nav_params
		{
			
		}m_simple_nav_params;	//!< Unused variable


		/** Structure of parameters of the HSV segmentation
		  */
		struct hsv_segmentation_parameters
		{
			std::vector< std::vector<int> >	m_hsv_thresholds;	//!< Thresholds for the HSV segmentation
		}m_hsv_segmentation_parameters; //!< Parameters of the HSV segmentation


		/** Structure of parameters for canny segmentation
		  */
		struct canny_segmentation_parameters
		{
			size_t	gaussian_blur_size;		//!< Size of the gaussian blur
			size_t	gaussian_blur_sigma_1;	//!< Sigma 1 of the gaussian blur
			size_t	gaussian_blur_sigma_2;	//!< Sigma 2 of the gaussian blur
			size_t	higher_threshold;		//!< Higher threshold of the canny edges detection
		}m_canny_segmentation_parameters;	//!< Parameters of the canny segmentation


		/** Structure of parameters for the hough circles segmentation
		  */
		struct hough_segmentation_parameters
		{
			float inverse_ratio_accumulator_resolution ; //!< Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
			float min_dist; //!< Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
			float higher_canny_threshold; //!< The higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
			float accumulator_threshold; //!< The accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
			float min_radius; //!< Minimum circle radius.
			float max_radius; //!< Maximum circle radius.
			size_t gaussian_blur_size;		//!< Size of the gaussian blur
			size_t gaussian_blur_sigma_1;	//!< Sigma 1 of the gaussian blur
			size_t gaussian_blur_sigma_2;	//!< Sigma 2 of the gaussian blur
		}m_hough_segmentation_parameters;	//!< Parameters of the hough circles segmentation


		/** Struct for storing the features of the frames to analyze
		  */
		struct frame_features
		{
			size_t height;	//!< Height of the frame
			size_t width;	//!< Width of the frame
			float initial_ROI_row_upper_limit;	//!< row upper limit of the ROI (Region Of Interest)
			float initial_ROI_row_lower_limit;	//!< row lower limit of the ROI
			float initial_ROI_col_left_limit;	//!< col left limit of the ROI
			float initial_ROI_col_right_limit;	//!< col right limit of the ROI
			size_t ROI_height;	//!< Height of the ROI
			size_t ROI_width;	//!< Width of the ROI
		}m_frame_features;	//!< features fo the frames to be analyzed


		/** Information which we want to see in a separate window 
		  */ 
		struct display_conf
		{
			bool	BGR;	//!< Show original BGR image
			bool	hsv_final_masks;	//!< Show final masks of HSV segmentation
			bool	hsv_partial_masks;	//!< Show partial masks of HSV segmentation
			bool	hsv_contours;		//!< Show hsv contours segmented
			bool	partial_candidates;	//!< Show partial candidates
			bool	hough_circles_segmentation;	//!< Show hough circles segmentation
			bool	pattern_detection;	//!< Show pattern detection
			bool	canny_results;		//!< Show canny edges detection results
		} m_display;	//!< Information to be shown in a separate window


		/** Struct for storing the constrains for pattern detection
		  */
		struct pattern_conf
		{
			vector<size_t>	m_pattern_order;	//!< Order of the colours of the circles of the pattern. Useful with HSV segmentation.
			vector<float>			m_max_inclination_fitted_rect;	//!< Max inclination of the fitted straight line 
			vector<float>			m_max_radius_difference_ratio;	//!< Max radius difference ratio between the circles
			vector<float>			m_min_circle_radius;	//!< Min radius of the circles
			vector<float>			m_max_circle_radius;	//!< Max radius of the circles
			vector<float>			m_max_error_adjusting_a_line;	//!< Max error adjusting a straight line
			vector<vector<float>>	m_thresholds_size_relation;		//!< Size relation between the circles
			vector<size_t>	m_final_pattern_row;	//!< Final row of the pattern, i.e., the row of the pattern in the image when the docking have been completed
			vector<size_t>	m_final_pattern_col;	//!< Final col of the pattern, i.e., the col of the pattern in the image when the docking have been completed
		} m_pattern_conf;	//!< Constrains for pattern detection


		/** Struct for storing information related to the pattern detection
		  */
		struct detected_pattern
		{
			bool		was_detected;	//!< It's initial value is false, set to true the first time that the pattern is detected
			bool		last_was_detected; //!< True if in the last obtainPatternLocationProcess the pattern was detected, false in other case
			TNCircles	last_location;	//!< Last localization of the detected pattern
		}m_detected_pattern;	//!< Information about the detected pattern


		/** Ideated for checking if the robot has done a complete turn without detecting the pattern
		  */
		struct turnCompleteTester 
		{
			float	last_orientation;		//!< Last robot orientation, in degrees
			float	orientation_increment;	//!< Increment in the robot orientation since the first turn command
			bool	turning;				//!< Are we turning without find the pattern?

			/** Constructor
			  */
			turnCompleteTester() : 	last_orientation(0),
									orientation_increment(0),
									turning(false)
			{}

			/** Check if the robot has done a complete turn
			  */
			bool check() const
			{ 
				if ( orientation_increment > 360 ) 
					return true; 
				else
				{
					printf("[AutoDocking_Camera] Cumulative orientation_increment is %.2f\n", orientation_increment);
					return false;
				}
			}

			/** Reset local variables
			  */
			void reset()
			{ 
				last_orientation = 0; 
				orientation_increment = 0; 
				turning = false; 
			}

			/** Set if the robot is turning or not
			  */
			void setTurning( const bool &t )
			{ 
				turning = t; 
			}

			/** Check if the robot is turning 
			  */
			bool isTurning()
			{ 
				return turning; 
			}

			/** Set the last robot orientation 
			  */
			void setLastOrientation( const float &orientation ) 
			{ 
				float new_orientation = abs(mrpt::utils::RAD2DEG( orientation ));
				// Only calculate the orientation increment if we are turning! if not, it's
				// the first time that this function is executed, so doesn't exist an increment!				
				if ( turning )
				{
					orientation_increment += abs( new_orientation - last_orientation );
					printf("[AutoDocking_Camera] Last angle increment is: %.3f\n", abs( new_orientation - last_orientation ));
				}
									
				last_orientation = new_orientation; 
			}

		}m_turn_complete_tester; //!< For testing if the robot has accomplished a complete turn without detecting the pattern

		
		////////////////////////////////////////// METHODS //////////////////////////////////////////


		/** Moves a bit the robot towards the docking. */
		void moveABit();

		/** Turns a bit the robot. */
		void turnABit();

		/** Check if the robot is good oriented. */
		void checkGoodOriented();

		/** Rotate the robot until to have a good orientation. */
		bool performGoodOrientation();

		/** Move in straight line the robot. */
		void moveInStraightLineABit();

		/** Check if the robot has achieved the goal */
		bool checkGoalAchieved();

		/** Dummy method for launching autoDocking one */

		static void dummyAutoDocking( CAutoDockingCamera *obj );

		/** Permanent loop which performs the reactive navigation */

		void autoDocking();


		/** Image segmentation methods */

		/** Segments a image using threshold in HSV channels
		  * \param frame frame where to perform the segmentation
		  * \param center vector with the centers of the candidate regions
		  * \param radius vector with the radius of the candidate regions
		  */
		void segmentation_hsv( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius );

		/** Segments a image using Hough circles
		  * \param frame frame where to perform the segmentation
		  * \param center vector with the centers of the candidate regions
		  * \param radius vector with the radius of the candidate regions
		  */
		void segmentation_houghCircles( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius );

		/** Segments a image canny filled polygons
		  * \param frame frame where to perform the segmentation
		  * \param center vector with the centers of the candidate regions
		  * \param radius vector with the radius of the candidate regions
		  */
		void segmentation_cannyFilledPolygons( Mat &frame, vector<vector<Point2f>> &center, vector<vector<float>> &radius );
		
		/** Method used by segmentation_hsv for segmenting only a colour 
		  */
		void obtainSegmentedMat( const cv::Mat &frame, 
								cv::Mat &segmented, 
								const size_t &colour_index );

		/** Method called by obtainSegmentedMat for obtaining a mask where the candidates are 
		  */
		void obtainMaskMat( const cv::Mat &channel_mat, 
							const size_t &channel_num, 
							int inf_threshold, 
							int sup_threshold, 
							cv::Mat &mask, 
							const size_t &colour_index );

		/** Obtain the ROI of a frame (if we've detected previously a frame, don't search the pattern in the whole frame!) 
		  */
		void obtainROI( const Mat &fullFrame, Mat &frame );

		/** Check if the coordinates are right (into the image) and otherwise fix them. 
		  */
		void fixPossibleImageLimitsViolations( double &top_left, double &top_right, double &top_up , double &top_down );


		/** Pattern detection methods */

		/** Method for trying to detect the pattern a certain number of times 
		  */
		void patternLocalizationProcess();

		/** Computes the error while adjusting a straight line to candidates circles.
          * \param v_final_candidates vector with final candidates.
		  * \param adjusted_line adjusted straight line.
		  * \return estimated error adjusting such a line.
		  */
		float computeErrorAdjustingALine( const TNCircles &v_final_candidates, TLine2D &adjusted_line );		

		/** Go go go go! Try to detect the pattern 
		  */
		void patternLocalization();

		/** Search the pattern once the segmentation is completed.
		  * \param center vector of candidates centers.
		  * \param radius vector of candidates radius.
		  */
		void searchPattern_afterHsvOrHough( vector<vector<Point2f>> &center, vector<vector<float>> &radius );
		
		/** Not used. Will disapear in posteriors refactorizing 
		  */
		void searchPattern_afterCanny( const vector<vector<Point2f>> &center, const vector<vector<float>> radius );


		/** Debug methods */

		/** When using HSV segmentation, this method shows the candidates which overcome each step.
		  * \param candidates candidates to show
		  * \param i_colour index of the colour of that step
		  */
		void debug_pattern_detection( vector<vector<TCircle>> &candidates, const size_t &i_colour );

		/** Retrieve histogram information from a frame.
		  */
		void obtainHistogramInformation( const Mat &frame );


		/** Cosntrains */

		/** Check if a certain radious fulfill the radius constrains.
		  * \param radius radius to check.
		  * \return true if fulfill the constrains, false in otherwise.
		  */
		bool constrain_radius( const float &radius) const;

		//void constrain_errorAdjustingALine() const;


		/** General methods */

		/** Update the last movement performed by the robot
		  * \param movement	last movement performed
		  * \move_or_turn_quantity quantity of movement ordered
		  */
		void updateLastMovement( const TLast_movement_command &movement, const double &move_or_turn_quantity );

		/** Change the Autodocking mode between NEAR and FAR
		 * \param new_mode new Autodocking mode
		 */
		void changeMode( const TMode &new_mode );

};
#endif