/********************************************************************************
*
*			Extension of the IOSTREAM class
*
*			(C) Juan Antonio Fernández Madrigal, 2000
*
********************************************************************************/

#include "Jafma_Iostream"

#include <fstream>

namespace JAFMA
{

/******************************************************************
*
*				Functions and Operators
*
*******************************************************************/

std::string rep(unsigned t, char x)
{
	if (t==0) return("");
	return(std::string(t,x));
}

std::string ReadLine(std::istream &is, const char delim)
{std::string res;
 char c;
 bool fin=false;

// std::cout << "\t****Reading line at " << is.tellg() << std::endl;
	res="";
	do
	{
		c=is.get();
		if (is.eof()) fin=true;
		else
		{
			if (c==delim) fin=true;
			else res=res+c;
		}
	} while (!fin);
//	std::cout << "\t****Read line [" << res << "]" << std::endl;
	return(res);
}

std::string PeekLine(std::istream &is, const char delim)
{std::streampos p;
 std::string s;

	p=is.tellg();
	s=ReadLine(is,delim);
	is.seekg(p);
	return(s);
}

bool SearchLineSubstr(std::istream &i, const std::string substr, std::string &line)
{std::string n,b;
 bool enc=false;

	do
	{
		n=ReadLine(i);
		if (n=="") return(false);
		if (find_substr(n,substr)!=n.npos) enc=true;
	} while (!enc);
	line=n;
	return(true);
}

std::string ReadEverything(std::istream &is, const unsigned maxlen, const unsigned lenbuf)
{char *buf;
 unsigned lbuf,ml=maxlen;
 std::string res;
 bool fin;
 
	if (lenbuf==0) lbuf=1;
	else lbuf=lenbuf;
	buf=new char[lbuf+1];
	if (buf==0) return(res);
	fin=false;
	while (!fin)
	{
		is.read(buf,lbuf);
		buf[is.gcount()]=0;
		if ((maxlen>0)&&(ml<(unsigned)is.gcount())) 
		{
			buf[ml]=0;
			fin=true;
		}
		else 
		{
			if (maxlen>0) ml-=is.gcount();
			if (is.eof()) fin=true;
		}
		res+=buf;
	}
	delete[] buf;
	return(res);
}

bool SearchCharacter(std::istream &is, char ch)
{char c;

	do c=is.get(); while ((!is.eof())&&(c!=ch));
	return(!is.eof());
}


} // End JAFMA namespace
