PROJECT(Wifi_KeepCoverage)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  hwdrivers ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CWifiAlert.h CWifiAlert.cpp
	#CWinPlot.h CWinPlot.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS})