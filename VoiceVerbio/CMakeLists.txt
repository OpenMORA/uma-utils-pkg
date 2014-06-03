PROJECT(VoiceVerbio)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  ) # hmtslam hwdrivers ... any other sub-library

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

# Find Verbio:
SET(VERBIO_PRESENT 0)

IF (WIN32)
	# Win   ---------------------------------------- 
	# Ask the user for verbio lib location...
	SET(VERBIO_PRESENT 1)
	SET(VERBIO_LIBS verbiolib.lib voxlib.lib) #  VoiceInteractionLibraryVC6.lib 

	# JL: TODO: Remove embedded copy of verbio libs and set thru CMake vars:
	INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/libverbio_win )
	LINK_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/libverbio_win )

ELSE(WIN32)
	# Linux ---------------------------------------- 
	# Look for voxlib.so, voxlib.h,...
	FIND_FILE(VOXLIB_HEADER_FILE voxlib.h)
	IF(VOXLIB_HEADER_FILE)
		FIND_LIBRARY(VOXLIB_LIB_FILE voxlib)
		IF(VOXLIB_LIB_FILE)
			SET(VERBIO_LIBS voxlib verbiolib) # xerces-c)
			

			# Verbio 8.0 needs: 
			#  libxalan-c.so.110
			#  libxerces-c.so.27

#			FIND_LIBRARY(XERCES_LIB xerces-c)  
#			IF(XERCES_LIB)
#				MESSAGE(STATUS "libxerces: ${XERCES_LIB}")			
				SET(XERCES_FOUND 1)
#			ELSE(XERCES_LIB)
#				MESSAGE(SEND_ERROR "libxerces-c not found.")
#				SET(XERCES_FOUND 0)
#			ENDIF(XERCES_LIB)
			
			#PKG_CHECK_MODULES(XERCES xerces-c)	# Verbio also needs xerces-c
			PKG_CHECK_MODULES(PULSE libpulse-simple)	# And we'll use libpulse
			IF(XERCES_FOUND AND PULSE_FOUND)
				INCLUDE_DIRECTORIES(${PULSE_INCLUDEDIR})
				LINK_DIRECTORIES(${PULSE_LIBDIR})

#				MESSAGE(STATUS "[Pulse lib] include: '${PULSE_INCLUDEDIRS}' libs: '${PULSE_LIBRARIES}' version: '${PULSE_VERSION}'")
#				MESSAGE(STATUS "[Xerces-c lib] include: '${XERCES_INCLUDEDIRS}' libs: '${XERCES_LIBRARIES}' version: '${XERCES_VERSION}'")

				INCLUDE_DIRECTORIES(${PULSE_INCLUDE_DIRS})

			 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PULSE_CFLAGS}")

				# All found OK
				SET(VERBIO_PRESENT 1)
			ELSE(XERCES_FOUND AND PULSE_FOUND)
				MESSAGE(SEND_ERROR "libpulse (-simple) or xerces-c not found. Install them or disable BUILD_MODULE_VoiceVerbio")
			ENDIF(XERCES_FOUND AND PULSE_FOUND)
	
		ELSE(VOXLIB_LIB_FILE)
			MESSAGE(SEND_ERROR "voxlib.so not found. Fix it or disable BUILD_MODULE_VoiceVerbiop")
		ENDIF(VOXLIB_LIB_FILE)

	ELSE(VOXLIB_HEADER_FILE)
		MESSAGE(SEND_ERROR "voxlib.h not found. Fix it or disable BUILD_MODULE_VoiceVerbio")
	ENDIF(VOXLIB_HEADER_FILE)
	

ENDIF(WIN32)

if(NOT VERBIO_PRESENT)
	MESSAGE(SEND_ERROR "Verbio was not found. Not building VoiceVerbio module. Fix it or disable BUILD_MODULE_VoiceVerbio")
endif(NOT VERBIO_PRESENT)

if(VERBIO_PRESENT)


FILE(GLOB SRCS_LST ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) 
FILE(GLOB HDRS_LST ${CMAKE_CURRENT_SOURCE_DIR}/*.h) 
FILE(GLOB HDRS2_LST ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp) 

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	${SRCS_LST} ${HDRS_LST} ${HDRS2_LST}
	)

# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} ${VERBIO_LIBS} ${PULSE_LIBRARIES})

endif(VERBIO_PRESENT)

