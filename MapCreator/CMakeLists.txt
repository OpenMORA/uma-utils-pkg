PROJECT(MapCreator)

FIND_PACKAGE( MRPT REQUIRED base gui opengl slam maps reactivenav)

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Declare the target (an executable)
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp	  
	CMapCreator.h 
	CMapCreator.cpp
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} )
