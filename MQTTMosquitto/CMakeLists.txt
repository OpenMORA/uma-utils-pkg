PROJECT(MQTTMosquitto)

# Use MRPT:
FIND_PACKAGE( MRPT REQUIRED slam gui  hwdrivers)

INCLUDE_DIRECTORIES( ${MOOS_INCLUDE_DIRS}  )
LINK_DIRECTORIES( ${MOOS_LIBRARY_DIRS} )


#Use Mosquitto libraries
find_path(MQTTmosquitto_LIB NAMES mosquitto.lib mosquittopp.lib DOC "Path to the folder containing the MQTTmosquitto mosquitto.lib and mosquittopp.lib libraries")
if (${MQTTmosquitto_LIB} STREQUAL  "MQTTmosquitto_LIB-NOTFOUND")
	message ("MQTTmosquitto 'mosquitto.lib' or 'mosquittopp.lib' not Found!. \nPlease, provide the path to the MQTTmosquitto libraries.")
endif (${MQTTmosquitto_LIB} STREQUAL  "MQTTmosquitto_LIB-NOTFOUND")

INCLUDE_DIRECTORIES(${MQTTmosquitto_LIB})

# Add executable
ADD_EXECUTABLE( ${PROJECT_NAME}
	main.cpp
	CMQTTMosquitto.cpp CMQTTMosquitto.h
	Constants.cpp Constants.h
	)
	
# Link the executable
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MOOS_LIBRARIES} ${MRPT_LIBS} "${MQTTmosquitto_LIB}/mosquitto.lib" "${MQTTmosquitto_LIB}/mosquittopp.lib" )
