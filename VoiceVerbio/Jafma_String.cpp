/********************************************************************************
*
*			Extension of the STRING class
*
*			(C) Juan Antonio Fernández Madrigal, 2000
*
********************************************************************************/

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "Jafma_String"

#ifdef _MSC_VER
    #if _MSC_VER >= 1300 //VC++ .NET
    	#define sprintf sprintf_s
    #endif
#endif

namespace JAFMA
{

/******************************************************************
*
*				class: JAFMA_string
*
*******************************************************************/

unsigned find_substr(const std::string s, const std::string subs, unsigned pos0)
{unsigned est=0,posub,posant=0;

	if (subs.empty()) return((unsigned)s.npos);
	posub=0;
	while (pos0<s.size())
	{
		switch (est)
		{
			case 0: if (s[pos0]==subs[posub])
					{
						est=1;
						posub++;
						if (posub==subs.size()) return(pos0);
						posant=pos0;
					}
					break;
			case 1: if (s[pos0]!=subs[posub])
					{
						est=0;
						posub=0;
						pos0=posant;
						break;
					}
					posub++;
					if (posub==subs.size()) return(posant);
					break;
		}
		pos0++;
	}
	return((unsigned)s.npos);
}

bool bracketed_substr(const std::string &is, std::string &os,
					  const char delimo, const char delimc,
					  unsigned *posfound)
{unsigned posb,pose;

	posb=(unsigned)is.find(delimo);
	if (posb>=is.size()) return(false);
	if (posfound!=0) *posfound=posb;
	pose=(unsigned)is.find(delimc,posb+1);
	if (pose>=is.size()) return(false);
	os=is.substr(posb+1,pose-posb-1);
	return(true);
}

std::string noleadingspaces_substr(const std::string s, const bool noascii)
{unsigned f,l=(unsigned)s.size();

	for (f=0; f<l; f++)
		if (((noascii)&&(s[f]>' '))||
			((!noascii)&&(s[f]!=' ')&&(s[f]!='\t'))) return(s.substr(f,l-f));
	return("");
}

std::string notailingspaces_substr(const std::string s, const bool noascii)
{unsigned f,l=(unsigned)s.size();

	if (l==0) return("");
	for (f=l-1; f>=0; f--)
		if (((noascii)&&(s[f]>' '))||
			((!noascii)&&(s[f]!=' ')&&(s[f]!='\t'))) return(s.substr(0,f+1));
	return("");
}

std::string nolateralspaces_substr(const std::string s, const bool noascii)
{
	return(noleadingspaces_substr(notailingspaces_substr(s,noascii),noascii));
}

/* ---------------------- Compactación de datos ---------------------- */

static std::string byte_to_hexstring(unsigned v)
/* Return a 2 character string with the hexadecimal representation of the lower
byte of V */
{char res[3];
 unsigned h,l;

	v=v%256;
	h=v/16;
	l=v%16;
	res[0]=(h>9)?(h-10+'a'):(h+'0');
	res[1]=(l>9)?(l-10+'a'):(l+'0');
	res[2]=0;
	return(res);
}

static std::string character_to_hexstring(char c)
/* Return a string containing the hexadecimal representation of C. If C is an extended
character, return it in the format "*HHLL" */
{unsigned v=(unsigned)c,v1,v2;
 std::string kk;

	v=v%65536;
	if (v>255)
	{
		kk="*";
		v1=v/256;
		v2=v%256;
		kk+=byte_to_hexstring(v1)+byte_to_hexstring(v2);
		return(kk);
	}
	return(byte_to_hexstring(v));
}

std::string compact_str(const std::string s, const unsigned tabs, const unsigned tamline)
{std::string res,buf;
 unsigned tam,t,l,f;

	tam=(unsigned)s.length();
	res="\n";
	for (f=0; f<tabs; f++) res+="\t";
	res+="CompactStr(";
	number_to_string(tam,buf);
	res+=buf;
	res+=")";
	t=0;
	while (t<tam)
	{
		buf=s.substr(t,tamline);
		l=(unsigned)buf.length();
		if (l>0)
		{
			res+="\n";
			for (f=0; f<tabs+1; f++) res+="\t";
			for (unsigned g=0; g<l; g++) res+=character_to_hexstring(buf[g]);
			t+=l;
		}
		else t=tam;
	}
	res+="\n";
	for (f=0; f<tabs; f++) res+="\t";
	res+="EndCompactStr";
	return(res);
}

std::string compact_data(void *d, unsigned tam, const unsigned tabs, const unsigned tamline)
{std::string res,buf;
 unsigned t,l,f;

	if (d==0) tam=0;
	res="\n";
	for (f=0; f<tabs; f++) res+="\t";
	res+="CompactData(";
	number_to_string(tam,buf);
	res+=buf;
	res+=")";
	t=0;
	while (t<tam)
	{
		l=tamline;
		if (t+l>tam) l=tam-t;
		if (l>0)
		{
			res+="\n";
			for (f=0; f<tabs+1; f++) res+="\t";
			for (unsigned g=0; g<l; g++) res+=byte_to_hexstring(((char *)d)[t+g]);
			t+=l;
		}
		else t=tam;
	}
	res+="\n";
	for (f=0; f<tabs; f++) res+="\t";
	res+="EndCompactData";
	return(res);
}

static char read_hexbyte(char c1, char c2)
{unsigned v1,v2;

	v1=1000;
	if ((c1>='A')&&(c1<='F')) v1=(unsigned)c1-(unsigned)'A'+10;
	if ((c1>='a')&&(c1<='f')) v1=(unsigned)c1-(unsigned)'a'+10;
	if ((c1>='0')&&(c1<='9')) v1=(unsigned)c1-(unsigned)'0';
	if (v1==1000) return(' ');
	v2=1000;
	if ((c2>='A')&&(c2<='F')) v2=(unsigned)c2-(unsigned)'A'+10;
	if ((c2>='a')&&(c2<='f')) v2=(unsigned)c2-(unsigned)'a'+10;
	if ((c2>='0')&&(c2<='9')) v2=(unsigned)c2-(unsigned)'0';
	if (v2==1000) return(' ');
	return((char)(v1*16+v2));
}

static char read_hexword(char c1, char c2, char c3, char c4)
{unsigned v,w;

	v=read_hexbyte(c1,c2);
	w=read_hexbyte(c3,c4);
	return(v*256+w);
}

std::string expand_str(const std::string &s)
{std::string res,buf;
 unsigned tam,t,f,g,len,sl,r=0;
 bool err;
 char c[2];

	if (!bracketed_substr(s,buf,'(',')')) return(res);
	string_to_number(buf,tam);
	len=(unsigned)s.length();
	for (f=0; f<len; f++) if (s[f]=='\n') break;
	if (f>=len) return(res);
	f+=2;
	t=0;
	c[1]=0;
	err=false;
	while ((r<tam)&&(f<len))
	{
		while ((f<len)&&(s[f]=='\t')) f++;
		if (f<len)
		{
			g=f;
			while ((g<len)&&(s[g]!='\n')) g++;
			if (g<len)
			{
				buf=s.substr(f,g-f);
				sl=(unsigned)buf.length();
				for (unsigned h=0; h<sl; h+=2)
				{
					if (buf[h]!='*') c[0]=read_hexbyte(buf[h],buf[h+1]);
					else
					{
						if (h+4>=sl) err=true;
						else
						{
							c[0]=read_hexword(buf[h+1],buf[h+2],buf[h+3],buf[h+4]);
							h+=3;
						}
					}
					if (!err)
					{
						r++;
						res+=c;
					}
				}
				if (!err) f=g+1;
			}
			else
			{
				f=len;
				err=true;
			}
		}
		else err=true;
	}
	if (err) return("");
	return(res);
}

bool expand_data(const std::string s, void **d, unsigned *siz)
{std::string buf;
 unsigned tam,t,f,g,len,sl,r=0,pos;
 bool err;
 char c[2];

	if (!bracketed_substr(s,buf,'(',')' ,&g)) return(false);
	string_to_number(buf,tam);
	if (siz!=0) *siz=tam;
	if (tam==0)
	{
		*d=0;
		return(true);
	}
	*d=new char[tam];
	pos=0;
	len=(unsigned)s.length();
	for (f=g; f<len; f++) if (s[f]=='\n') break;
	if (f>=len) return(false);
	f+=2;
	t=0;
	c[1]=0;
	err=false;
	while ((r<tam)&&(f<len))
	{
		while ((f<len)&&(s[f]=='\t')) f++;
		if (f<len)
		{
			g=f;
			while ((g<len)&&(s[g]!='\n')) g++;
			if (g<len)
			{
				buf=s.substr(f,g-f);
				sl=(unsigned)buf.length();
				for (unsigned h=0; h<sl; h+=2)
				{
					c[0]=read_hexbyte(buf[h],buf[h+1]);
					if (!err)
					{
						r++;
						((unsigned char *)(*d))[pos]=c[0];
						pos++;
					}
				}
				if (!err) f=g+1;
			}
			else
			{
				f=len;
				err=true;
			}
		}
		else err=true;
	}
	if (err) return(false);
	return(true);
}

std::string read_compact_str(std::istream &is)
{std::string res,lin;
 unsigned f,g;
 bool endcs=false;

	lin=ReadLine(is);
//	std::cout << "-->[" << lin << "]" << std::endl;
	for (f=0; f<lin.length(); f++) if (lin[f]=='C') break;
	if (f>=lin.length()) return("");
	for (g=f; g<lin.length(); g++) if (lin[g]=='(') break;
	if (g>=lin.length()) return("");
	if (lin.substr(f,g-f)!="CompactStr") return("");
	res=lin+"\n";

	do
	{
		lin=ReadLine(is);
		for (f=0; f<lin.length(); f++) if (lin[f]=='E') break;
		if (f<lin.length())
		{
			for (g=f; g<lin.length(); g++) if (lin[g]=='r') break;
			if (g<lin.length())
			{
				if (lin.substr(f,g-f+1)=="EndCompactStr") endcs=true;
			}
		}
		res+=lin+"\n";
	} while (!endcs);

	return(res);
}

std::string read_compact_data(std::istream &is)
{std::string res,lin;
 unsigned f,g;
 bool endcs=false;

	lin=ReadLine(is);
	for (f=0; f<lin.length(); f++) if (lin[f]=='C') break;
	if (f>=lin.length()) return("");
	for (g=f; g<lin.length(); g++) if (lin[g]=='(') break;
	if (g>=lin.length()) return("");
	if (lin.substr(f,g-f)!="CompactData") return("");
	res=lin+"\n";

	do
	{
		lin=ReadLine(is);
		for (f=0; f<lin.length(); f++) if (lin[f]=='E') break;
		if (f<lin.length())
		{
			for (g=f; g<lin.length(); g++) if (lin[g]=='r') break;
			if (g<lin.length())
			{
				if (lin.substr(f,g-f+1)=="EndCompactData") endcs=true;
			}
		}
		res+=lin+"\n";
	} while (!endcs);

	return(res);
}

/* ---------------------------- Fin de compactación de datos ---------------------------- */

void indent_str(std::string &s, const unsigned numtabs, const char ic)
{unsigned l=(unsigned)s.size(),f;
 bool addtab=true;
 std::string tabs;
 char buf[2];

	buf[0]=ic;
	buf[1]=0;
	for (f=0; f<numtabs; f++) tabs+=buf;
	f=0;
	while (f<l)
	{
		if (addtab)
		{
			s.insert(f,tabs);
			l=(unsigned)s.size();
			addtab=false;
			f+=(unsigned)tabs.size();
		}
		else f++;
		if (s[f]=='\n')
		{
			addtab=true;
			f++;
		}
	}
}

std::string make_identifier_str(const std::string s)
{std::string s2;
 unsigned f,l;

	s2=nolateralspaces_substr(s);
	l=(unsigned)s2.size();
	for (f=0; f<l; f++)
		if ((s2[f]==' ')||(s2[f]=='\t')||
			(! (((s2[f]>='0')&&(s2[f]<='9'))||
				((s2[f]>='a')&&(s2[f]<='z'))||
				((s2[f]>='A')&&(s2[f]<='Z'))) )
			) s2[f]='_';
	if ((s2.size()>0)&&(s2[0]>='0')&&(s2[0]<='9')) s2="x_"+s2;
	return(s2);
}

std::string upper_str(const std::string s)
{std::string ss;
 unsigned f,l;

	ss=s;
	l=(unsigned)ss.size();
	for (f=0; f<l; f++)
		if ((ss[f]>='a')&&(ss[f]<='z')) ss[f]=ss[f]-'a'+'A';
	return(ss);
}

std::string truncate_str(const std::string s, unsigned maxlen)
{std::string ss;

	if (maxlen==0) ss=s;
	else
	{
		if (maxlen>=s.size()) ss=s;
		else ss=s.substr(0,maxlen);
	}
	return(ss);
}

std::string extract_directory(const std::string &path)
{int f;

	for (f=(int)path.size()-1; f>=0; f--) if (path[f]=='\\') return(path.substr(0,f));
	return(".");
}

std::string extract_filename(const std::string &path)
{int f;

	for (f=(int)path.size()-1; f>=0; f--) if (path[f]=='\\') return(path.substr(f+1,path.size()-f-1));
	return(path);
}

std::string extract_fileextension(const std::string &fname)
{int f;

	f=(int)fname.size()-1;
	while ((f>=0)&&(fname[f]!='.')) f--;
	if ((f<0)||(f==(int)fname.size()-1)) return("");
	return(fname.substr(f+1,fname.size()-f-1));
}


/* ----------------- Routines to convert from strings to numbers --------------- */

void string_to_number(const std::string s, int &n)
{
	n=(int)atof(s.c_str());
}

void string_to_number(const std::string s, unsigned &n)
{
	n=(unsigned)atof(s.c_str());
}

void string_to_number(const std::string s, unsigned long &n)
{
	n=atol(s.c_str());
}

void string_to_number(const std::string s, float &n)
{
	n=(float)atof(s.c_str());
}

void string_to_number(const std::string s, double &n)
{
	n=atof(s.c_str());
}

void number_to_string(const int n, std::string &s)
{char buf[80];

	sprintf(buf,"%i",n);
	s=std::string(buf);
}

void number_to_string(const long n, std::string &s)
{char buf[80];

	sprintf(buf,"%li",n);
	s=std::string(buf);
}

void number_to_string(const unsigned n, std::string &s)
{char buf[80];

	sprintf(buf,"%u",n);
	s=std::string(buf);
}

void number_to_string(const unsigned long n, std::string &s)
{char buf[80];

	sprintf(buf,"%lu",n);
	s=std::string(buf);
}

void number_to_string(const float n, std::string &s)
{char buf[80];

	sprintf(buf,"%f",n);
	s=std::string(buf);
}

void number_to_string(const double n, std::string &s)
{char buf[80];

	sprintf(buf,"%f",n);
	s=std::string(buf);
}


} // End JAFMA namespace
