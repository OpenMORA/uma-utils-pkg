PROJECT(AutoDocking_Camera)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui hwdrivers ) # hmtslam hwdrivers ... any other sub-library

FIND_PACKAGE(OpenCV REQUIRED )
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
     CAutoDocking.h CAutoDocking.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBS})
