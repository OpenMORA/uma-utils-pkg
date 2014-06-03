#include <cstdio>
#include <iostream>
#include <fstream>
#include "includes/twitcurl.h"

class CTwitterClient
{
public:

	CTwitterClient();
	~CTwitterClient();

	bool Connect(std::string user="",std::string pass="");
	bool Tweet(std::string msg);
	bool IsConnected();

private:
	  twitCurl twitterObj;
	  bool connected;
  
};
