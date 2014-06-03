/*---------------------------*/
/*	FILE STRING READER CLASS */
/*                           */
/*	ECM, 2006                */
/*---------------------------*/

/** \file FileStringReader.cpp
 *\brief Implementation of FileStringReader.hpp
 */

#include "FileStringReader.hpp"
#include "Jafma_Iostream"
#include <fstream>


using namespace std;
using namespace JAFMA;

/*------------------------*
 * FileStringReader class *
 *------------------------*/

FileStringReader::FileStringReader()
{
}

FileStringReader::~FileStringReader()
{
}

string FileStringReader::readFile(const std::string &fileName)
{
	string readedStr;
	ifstream fileReader(fileName.c_str());

	if(fileReader.good())
	{
		readedStr = ReadEverything(fileReader);
	}
	else
	{
		printf("Sorry, the file '%s' can not be readed\n",fileName.c_str());
	}
	fileReader.close();
	return(readedStr);
}


unsigned FileStringReader::getNumberOfRepetitions(string source,string pattern)
{
	unsigned count = 0;
	unsigned index = 0;
	string::size_type findRes;

	while (index < source.size())
	{
		findRes = source.find(pattern,index);
		if( findRes != string::npos)
		{
			count++;
			index = (unsigned)findRes + pattern.size();
		}
		else
		{
			return(count);
		}
	}
	return(count);
}

int FileStringReader::GetNumberOfWords(string wordsChain)
{
	string::iterator sIter;
	int number = 0;
	sIter = wordsChain.begin();
	while(sIter != wordsChain.end())
	{
		if(*sIter == '<')
		{
			number++;
		}
			sIter++;
		}
	return(number);
}
/* end FileStringReader class */
