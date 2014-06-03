#include <cstdio>
#include <iostream>
#include <fstream>
//#include "includes/twitcurl.h"

public class CTwitterClient
{

	CTwitterClient();
	~CTwitterClient();

	bool Connect(std::string user="",std::string pass="");
	bool Tweet(std::string msg);

//private:
//	  twitCurl twitterObj;
  
};
