#include "Constants.h"

// see constants.h for info
const char *Constants::host           = "giraffplus.xlab.si";
const int   Constants::port           = 8883;
const int   Constants::keepalive_secs = 30;
const char *Constants::topics[] = { "TopologyCommand", "NavigationCommand", "MapBuildingCommand", "SessionCommand", "ClientACK", "ServerACK", "Debug", "Debugf" };
const int   Constants::topic_count    = (sizeof(Constants::topics) / sizeof(char*));
const char *Constants::publish_topic  = "hello_topic";