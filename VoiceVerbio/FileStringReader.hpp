/*---------------------------*/
/*	FILE STRING READER CLASS */
/*                           */
/*	ECM, 2006                */
/*---------------------------*/

/** \file FileStringReader.hpp
 *\brief File reader classes.
 */

#ifndef FILESTRINGREADER
	#define FILESTRINGREADER

#include <string>

/*!\brief Reader for files.
 *
 * It reads the content of a file and stores it into a string.
 */
class FileStringReader
{
	public:
		/*! Default constructor */
		FileStringReader();

		/*! Class destructor */
		~FileStringReader();

		/*! Opens the source file and store its contents into a string. If any error occurs,
		 * returns empty string.
		 *\param fileName file to open and read.
		 *\return string with the content of the file
		 */
		std::string readFile(const std::string &fileName);

		/*! Returns the number of repetitions of a pattern string into a string
		 *\param source string to look for patterns
		 *\param pattern pattern to look for
		 *\return number of times that the pattern is repeated in the string
		 */
		 static unsigned getNumberOfRepetitions(std::string source,std::string pattern);

		 /*! Get the number of words in a pattern substring.
		  *\param wordsChain string to look for words
		  *\return number of words in the string
		  */
		 static int GetNumberOfWords(std::string wordsChain);
}; /* end FileStringReader */
#endif
