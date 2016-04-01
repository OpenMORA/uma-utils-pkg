PROJECT(MQTTMosquitto)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui hwdrivers)

# Get MOOS
INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )

#Use Mosquitto libraries
IF(WIN32)
	find_path(MQTTmosquitto_LIB NAMES mosquitto.lib mosquittopp.lib DOC "Path to the folder containing the MQTTmosquitto mosquitto.lib and mosquittopp.lib libraries")	
ENDIF(WIN32)
IF(UNIX)
	find_path(MQTTmosquitto_LIB NAMES libmosquitto libmosquittopp DOC "Path to the folder containing the libmosquitto.so and libmosquittopp.so libraries")
ENDIF(UNIX)

IF (${MQTTmosquitto_LIB} STREQUAL  "MQTTmosquitto_LIB-NOTFOUND")
	message ("MQTTmosquitto 'mosquitto' or 'mosquittopp' libs not Found!. \nPlease, provide the path to the MQTTmosquitto libraries.")
ENDIF (${MQTTmosquitto_LIB} STREQUAL  "MQTTmosquitto_LIB-NOTFOUND")

INCLUDE_DIRECTORIES(${MQTTmosquitto_LIB})


# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CMQTTMosquitto.cpp CMQTTMosquitto.h
	Constants.cpp Constants.h
	)
	
# Link the executable
IF(WIN32)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} "${MQTTmosquitto_LIB}/mosquitto.lib" "${MQTTmosquitto_LIB}/mosquittopp.lib" )
ENDIF(WIN32)
IF(UNIX)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} "${MQTTmosquitto_LIB}/libmosquitto.so" "${MQTTmosquitto_LIB}/libmosquittopp.so" )
ENDIF(UNIX)
