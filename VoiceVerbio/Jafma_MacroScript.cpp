/*
	Implementation of the Jafma's Macro-Script files.
	(C) Juan Antonio Fernández Madrigal, 2000
*/

#include "Jafma_MacroScript"
#include <list>


/* ************************************************************************************
*
*						CLASS: JMSKeyword
*						(A JMS keyword)
*
***************************************************************************************/
namespace JAFMA	// For making definition of class JMSKeyword refer to its declaration in .h
{

class JMSKeyword
{
 public:

		/* ------------ Constants and types ----------- */

	 typedef enum {
					JMS_K_MACRO,		// #JMS-MACRO
					JMS_K_ZONE,			// #JMS-ZONE
					JMS_K_REPAREA,		// #JMS-REPAREA
					JMS_K_REPMACRO,		// #JMS-REPMACRO
					JMS_K_CONDITIONAL,	// #JMS-IF
					JMS_K_CONDITIONAL_REP,	// #JMS-IFREP
					JMS_K_CONDITIONAL_EQ,	// #JMS-IFEQ
					JMS_K_CONDITIONAL_REPEQ,	// #JMS-IFREPEQ
					JMS_K_INCLUDE,		// #JMS-INCLUDE
					JMS_K_REPINDEX,		// #JMS-REPINDEX#
					JMS_K_LIT,			// #JMS-LIT#
					JMS_K_invalid	// No valid JMS keyword
					} JMSKeywordType;

		/* ------------ Constructors ------------- */

	JMSKeyword(void);
	/* Default constructor: constructs an object representing an invalid keyword */

	~JMSKeyword(void);
	/* Destroy the object */

		/* ------------ Methods ------------- */

	void Clear(void);
	/* Clear the internal representation of the keyword, becoming an invalid one */

	JMSKeywordType Type(void) const;
	/* Return the type of the keyword */

	JMSKeywordType StringToType(const string txt, unsigned pos=0) const;
	/* Given a text in TXT that contains a keyword exactly from position POS, return the
	type of the keyword, or JMS_K_invalid if it does not correspond to any type */

	bool TypeToString(JMSKeywordType t, string &txt) const;
	/* Given a type of keyword in T, store in TXT its identifier text (i.e.: "#JMS-REPINDEX"
	for T=JMS_K_REPINDEX). Return FALSE if T is an invalid type */

	bool Locate(const string txt, JMSKeywordType &t, unsigned &posloc, const string firstarg="") const;
	/* Explore the string TXT from position POSLOC and stop when the first keyword is found,
	storing its initial position in POSLOC and its type in T. 
	If no keyword is found, return FALSE.
	If FIRSTARG!="", only locate a keyword whose first argument coincides with FIRSTARG.
	This method does not read the keyword or change the current represented keyword. */

	bool LocateGivenKeyword(const string txt, JMSKeywordType t, unsigned &posloc, const string firstarg="") const;
	/* The same as "Locate" but find only the next keyword of type T */

	bool Read(const string txt, unsigned posin=0, bool readbodies=true);
	/* Read the first keyword from the position POSIN of TXT and replace the current keyword 
	with that. Return FALSE if there is no keyword (in that case, the current keyword becomes
	invalid). If READBODIES==FALSE, read the keyword but keep the bodies empty */

	bool ReadGivenKeyword(const string txt, JMSKeywordType t, unsigned posin=0, bool readbodies=true);
	/* Read the first keyword of type T from position POSIN in TXT, and replace the current
	keyword with that. Return FALSE if there is no such keyword (in that case, the current
	keyword becomes invalid). If READBODIES==FALSE, read the keyword but keep the bodies empty */

	void OriginalPosition(unsigned &posb, unsigned &pose) const;
	/* Store in POSB and POSE the initial and final character occupied by the keyword
	in the original text */

	void CopyBody(unsigned bod, string &txt) const;
	/* Store in TXT the body number BOD (0 or 1) of the keyword if it exists. Store "" if
	it does not exist. For example, #JMS-ZONE()#<body>#JMS-ENDZONE# has a 
	body number 0, but #JMS-REPINDEX# has no bodies */

	unsigned NumberOfArguments(void) const;
	/* Return the number of arguments of the keyword. For example, #JMS-MACRO()# has 2
	arguments, while #JMS-REPINDEX# has none */

	bool CopyArgument(int pos, string &arg) const;
	/* Store in ARG a copy of the POS-th argument of the keyword. For example, the 
	1-st argument of a #JMS-MACRO()# is the default value of the macro. If the argument
	is enclosed between a given character, return the argument and the enclosing characters */

	void Print(ostream &os) const;
	/* Print through OS the content of the keyword */

 private:

	 JMSKeywordType ktype;	// Type of the keyword represented by this object
	 unsigned pos0,pos1;	// Position of the keyword inside the original text
	 string body0,body1;	// Bodies of the keyword, if any (up to 2, i.e. IF/ELSE)
	 list<string> args;		// List of arguments, if any
};

} // End namespace JAFMA

using namespace JAFMA;



/* ------------ Constructors ------------- */

JMSKeyword::JMSKeyword(void)
{
	Clear();
}

JMSKeyword::~JMSKeyword(void)
{
}


/* ------------ Methods ------------- */

void JMSKeyword::Clear(void)
{
	ktype=JMS_K_invalid;
	pos0=0;
	pos1=0;
	body0="";
	body1="";
	args.clear();
}

JMSKeyword::JMSKeywordType JMSKeyword::Type(void) const
{
	return(ktype);
}

JMSKeyword::JMSKeywordType JMSKeyword::StringToType(const string keyword, unsigned pos) const
{
		// In the case of sharing the beginning of the macro text,
		// first, longer macros, since shorter ones are subsets of them
	if		(keyword.substr(pos,10)=="#JMS-MACRO") return(JMS_K_MACRO);
	else if (keyword.substr(pos,9) =="#JMS-ZONE") return(JMS_K_ZONE);
	else if (keyword.substr(pos,12)=="#JMS-REPAREA") return(JMS_K_REPAREA);
	else if (keyword.substr(pos,13)=="#JMS-REPMACRO") return(JMS_K_REPMACRO);
	else if (keyword.substr(pos,12) =="#JMS-IFREPEQ") return(JMS_K_CONDITIONAL_REPEQ);
	else if (keyword.substr(pos,10) =="#JMS-IFREP") return(JMS_K_CONDITIONAL_REP);
	else if (keyword.substr(pos,9) =="#JMS-IFEQ") return(JMS_K_CONDITIONAL_EQ);
	else if (keyword.substr(pos,7) =="#JMS-IF") return(JMS_K_CONDITIONAL);
	else if (keyword.substr(pos,12)=="#JMS-INCLUDE") return(JMS_K_INCLUDE);
	else if (keyword.substr(pos,13)=="#JMS-REPINDEX") return(JMS_K_REPINDEX);
	else if (keyword.substr(pos,8) =="#JMS-LIT") return(JMS_K_LIT);

	return(JMS_K_invalid);
}

bool JMSKeyword::TypeToString(JMSKeywordType t, string &txt) const
{
	switch (t)
	{
		case JMS_K_MACRO:	txt="#JMS-MACRO"; break;
		case JMS_K_ZONE:	txt="#JMS-ZONE"; break;
		case JMS_K_REPAREA:	txt="#JMS-REPAREA"; break;
		case JMS_K_REPMACRO:txt="#JMS-REPMACRO"; break;
		case JMS_K_CONDITIONAL:txt="#JMS-IF"; break;
		case JMS_K_CONDITIONAL_REP:txt="#JMS-IFREP"; break;
		case JMS_K_CONDITIONAL_EQ:txt="#JMS-IFEQ"; break;
		case JMS_K_CONDITIONAL_REPEQ:txt="#JMS-IFREPEQ"; break;
		case JMS_K_INCLUDE:	txt="#JMS-INCLUDE"; break;
		case JMS_K_REPINDEX:txt="#JMS-REPINDEX"; break;
		case JMS_K_LIT:		txt="#JMS-LIT"; break;
		default:			txt=""; return(false);
	}
	return(true);
}

bool JMSKeyword::Locate(const string txt, JMSKeywordType &t, unsigned &posloc, const string firstarg) const
{bool fin;
 unsigned pos,pose;

	do
	{
		fin=true;
		posloc=find_substr(txt,"#JMS-",posloc);
		if (posloc==txt.npos) 
		{
//			cout << "locate: JMS not found any more" << endl;
			return(false);
		}
//cout << "locate: found [" << txt.substr(posloc,30) << "] at " << posloc << endl;
		t=StringToType(txt,posloc);
//		cout << "locate: type: " << t << " (" << endl;
		if (!firstarg.empty())
		{
			pose=find_substr(txt,"#",posloc+1);
//			cout << "locate: endpos of JMS keyword at " << pose << endl;
			if (pose!=txt.npos)
			{
				pos=find_substr(txt,"("+firstarg,posloc);
				if ((pos==txt.npos)||(pos>=pose)||
					((txt[pos+firstarg.size()+1]!=',')&&	// debe terminar ahi el argumento
					 (txt[pos+firstarg.size()+1]!=')')) )
				{
//					cout << "locate: first argument not found" << endl;
					posloc++;
					fin=false;
				}
//				else cout << "locate: fist argument found at " << pos << endl;
			}
		}
	} while (!fin);
	return(true);
}

bool JMSKeyword::LocateGivenKeyword(const string txt, JMSKeywordType t, unsigned &posloc, const string firstarg) const
{JMSKeywordType kt;

	do
	{
		if (!Locate(txt,kt,posloc,firstarg)) return(false);
		if (t==kt) return(true);
		posloc++;
	} while (posloc<txt.size());
	return(false);
}

static bool readnextargument(const string txt, string &arg, unsigned &pose)
/* Read from position POSE in TXT an argument (POSE must point to the first previous argument
delimiting character: comma, opening parenthesis, ...). Store in POSE the position of the 
delimiting character of the read argument. Return FALSE if there is no more arguments */
{unsigned pos0,posend;
 char c;

	posend=find_substr(txt,")#",pose);
	if (posend==txt.npos) return(false);
	if (pose>=posend) return(false);
	if ((txt[pose]!='(')&&(txt[pose]!=',')) return(false);
	arg="";

	pose++;
	pos0=pose;
	switch (txt[pose])
	{
		case '"': case '\'':
					c=txt[pose];
					do pose++; while ((pose<posend)&&(txt[pose]!=c));
					if (pose==posend) return(false);
					pose++;
					if ((txt[pose]!=',')&&(pose!=posend)) return(false);
					arg=txt.substr(pos0+1,pose-1-(pos0+1));
					break;
		case ',':	break;
		case ')':	break;
		default:	while ((pose<posend)&&(txt[pose]!=',')) pose++;
					arg=txt.substr(pos0,pose-pos0);
					break;
	}
	return(true);
}

static bool readNarguments_firstmandatory(const string txt, list<string> &args, 
										  unsigned pos=0, unsigned n=0)
/* Store in ARGS up to N arguments read from position POS in TXT. If N is 0, read an undefined
number of arguments. The first argument is mandatory, but the others may not appear. Return
FALSE if there is any error */
{unsigned posp;
 string value;
 bool nolim=(n==0),fin,prim;

	args.clear();
	posp=find_substr(txt,"(",pos);
	if (posp==txt.npos) return(false);
	fin=false;
	prim=true;
	while (!fin)
	{
		if (!readnextargument(txt,value,posp)) 
		{
			if (prim) return(false);
			fin=true;
		}
		else
		{
			args.push_back(value);
			n--;
			if ((!nolim)&&(n==0)) fin=true;
			prim=false;
		}
	}
	return(true);
}

static bool readbodies(const string txt, string &b0, string &b1, const unsigned pos0, 
					   bool readtwo, const string endword, const string interword,
					   unsigned &lastpos, bool actuallyread=true)
/* Read up to two bodies from TXT, storing them in B0 and B1. POS0 points to the first
character of the first body. READTWO must be TRUE for reading two bodies. ENDWORD must
be the ending word of the last body, and INTERWORD the word that separates both bodies.
If the INTERWORD is not found, only one body is recovered.
Store in LASTPOS the position in TXT of the last character of the ending word.
If ACTUALLYREAD==FALSE, the bodies are not actually read (the other variables are still
updated) */
{unsigned posend,posi;

	posend=find_substr(txt,endword,pos0);
	// cout << "--> leyendo cuerpos de (" << endword << ")" << endl;
	if (posend==txt.npos) return(false);
	b0="";
	b1="";
	lastpos=posend+endword.size()-1;
	if (readtwo)
	{
		// cout << "LEYENDO DOS CUERPOS (interword=[" << interword << "],endword=[" << endword << "], posend=" << posend << endl;

		posi=find_substr(txt,interword,pos0);
		if (posi<posend)
		{
			if (actuallyread)
			{
				b0=txt.substr(pos0,posi-pos0);
				b1=txt.substr(posi+interword.size(),posend-(posi+interword.size()));
				
		//		cout << "EYENDO DOS CUERPOS EN (" << pos0 << " a " << posi << " y (" << posi+interword.size() << " a " << posend << endl;
			}
			return(true);
		}
	}
	if (actuallyread) b0=txt.substr(pos0,posend-pos0);
	return(true);
}

bool JMSKeyword::Read(const string txt, unsigned posin, bool readthebodies)
{unsigned pos,posb,posend;
 JMSKeywordType t;
 string cualif;
	
	Clear();

	pos=posin;
	if (!Locate(txt,t,pos)) return(false);
	posb=find_substr(txt,"#",pos+1);
	if (posb==txt.npos) return(false);

	t=StringToType(txt,pos);
//cout << "type= " << t << endl;
	switch (t)
	{
		case JMS_K_MACRO:	if (!readNarguments_firstmandatory(txt,args,pos,2)) return(false);
							posend=posb;
							break;
		case JMS_K_ZONE:	if (!readNarguments_firstmandatory(txt,args,pos)) return(false);
							cualif=*args.begin();
							if (!readbodies(txt,body0,body1,posb+1,false,"#JMS-ENDZONE("+cualif+")#","",posend,readthebodies)) return(false);
							break;
		case JMS_K_REPAREA:	if (!readNarguments_firstmandatory(txt,args,pos,2)) return(false);
							cualif=*args.begin();
							//cout << "Leida reparea (" << cualif << ")" << endl;
							if (!readbodies(txt,body0,body1,posb+1,false,"#JMS-ENDREPAREA("+cualif+")#","",posend,readthebodies)) return(false);
							break;
		case JMS_K_REPMACRO:if (!readNarguments_firstmandatory(txt,args,pos,3)) return(false);
							if (args.size()<2) return(false);
							posend=posb;
							break;
		case JMS_K_CONDITIONAL:if (!readNarguments_firstmandatory(txt,args,pos,2)) return(false);
							cualif=*args.begin();
							if (!readbodies(txt,body0,body1,posb+1,true,"#JMS-ENDIF("+cualif+")#","#JMS-ELSE("+cualif+")#",posend,readthebodies)) return(false);
							break;
		case JMS_K_CONDITIONAL_REP:if (!readNarguments_firstmandatory(txt,args,pos,3)) return(false);
							if (args.size()<2) return(false);
							cualif=*(++args.begin());
							if (!readbodies(txt,body0,body1,posb+1,true,"#JMS-ENDIF("+cualif+")#","#JMS-ELSE("+cualif+")#",posend,readthebodies)) return(false);
							break;
		case JMS_K_CONDITIONAL_EQ:
							//cout << "reading ifeq" << endl;
							if (!readNarguments_firstmandatory(txt,args,pos,2)) return(false);
							//cout << "Leidos args de ifeq" << endl;
							if (args.size()<2) return(false);
							//cout << "Leidos 2 o mas args" << endl;
							cualif=*args.begin();
							if (!readbodies(txt,body0,body1,posb+1,true,"#JMS-ENDIF("+cualif+")#","#JMS-ELSE("+cualif+")#",posend,readthebodies)) return(false);
							break;
		case JMS_K_CONDITIONAL_REPEQ:if (!readNarguments_firstmandatory(txt,args,pos,3)) return(false);
							if (args.size()<3) return(false);
							cualif=*(++args.begin());
							if (!readbodies(txt,body0,body1,posb+1,true,"#JMS-ENDIF("+cualif+")#","#JMS-ELSE("+cualif+")#",posend,readthebodies)) return(false);
							break;
		case JMS_K_INCLUDE:	if (!readNarguments_firstmandatory(txt,args,pos)) return(false);
							posend=posb;
							break;
		case JMS_K_REPINDEX:if (!readNarguments_firstmandatory(txt,args,pos)) return(false);
							posend=posb;
							break;
		case JMS_K_LIT:		if (!readbodies(txt,body0,body1,posb+1,false,"#JMS-ENDLIT#","",posend,readthebodies)) return(false);
							break;
		default: return(false);
	}

	ktype=t;
	pos0=pos;
	pos1=posend;

	//cout << "Leida keyword: " << endl;
	//Print(cout);

	return(true);
}

bool JMSKeyword::ReadGivenKeyword(const string txt, JMSKeywordType t, unsigned posin, bool readbodies)
{unsigned pos;
	
	pos=posin;
	if (!LocateGivenKeyword(txt,t,pos)) return(false);
	return(Read(txt,pos,readbodies));
}

void JMSKeyword::OriginalPosition(unsigned &posb, unsigned &pose) const
{
	posb=pos0;
	pose=pos1;
}

void JMSKeyword::CopyBody(unsigned bod, string &txt) const
{
	switch (bod)
	{
		case 0: txt=body0; break;
		case 1: txt=body1; break;
		default: txt=""; break;
	}
}

unsigned JMSKeyword::NumberOfArguments(void) const
{
	return((unsigned)args.size());
}

bool JMSKeyword::CopyArgument(int pos, string &arg) const
{list<string>::const_iterator it=args.begin();

	if (pos>=(int)args.size()) return(false);
	while (pos>0)
	{
		pos--;
		it++;
	}
	arg=*it;
	return(true);
}

void JMSKeyword::Print(ostream &os) const
{string ts;
 unsigned f,p0,p1;

	os << "JMS-Keyword type (" << Type() << "): ";
	TypeToString(Type(),ts);
	OriginalPosition(p0,p1);
	os << ts << " Read from positions " << p0 << " to " << p1 << endl;
	os << "Number of arguments: " << NumberOfArguments() << endl;
	for (f=0; f<NumberOfArguments(); f++)
	{
		if (!CopyArgument(f,ts)) ts="<error reading argument>";
		os << "\tArgument " << f << ": (" << ts << ")" << endl;
	}
	CopyBody(0,ts);
	os << "Body 0: [" << ts << "]" << endl;
	CopyBody(1,ts);
	os << "Body 1: [" << ts << "]" << endl;
}


/* ************************************************************************************
*
*						CLASS: MacroScript
*						(A text with JMS keywords)
*
***************************************************************************************/

/* ---------------- Auxiliary Routines for the IndexesRepetition type ------------------- */

static int FindRepAreaIndex(const MacroScript::IndexesRepetitions &idxs, const string &repareaname)
/* Return the index into IDXS of an reparea of name REPAREANAME, or -1 if no such area is in IDXS */
{unsigned f,n;
	n=(unsigned)idxs.size();
	//cout << "find on " << n << endl;
	for (f=0; f<n; f++)
		if (idxs[f].repareaname==repareaname) return(f);
	return(-1);
}

static unsigned AddRepAreaIndex(MacroScript::IndexesRepetitions &idxs, const MacroScript::RepAreaIndex &rai)
/* Add to the end of IDXS a new index, a copy of RAI. Returns the index in IDXS of the added
element */
{
	idxs.push_back(rai);
	return((unsigned)(idxs.size()-1));
}

static bool DeleteRepAreaIndex(MacroScript::IndexesRepetitions &idxs, unsigned idx)
/* Erase the IDX-th element of IDXS. Return FALSE if such element does not exist */
{MacroScript::IndexesRepetitions::iterator ii;

	if (idx>=idxs.size()) return(false);
	ii=idxs.begin();
	while (idx>0)
	{
		ii++;
		idx--;
	}
	idxs.erase(ii);
	return(true);
}

static void PrintIndexes(const MacroScript::IndexesRepetitions &repindexes)
/* Prints in console the indexes in REPINDEXES */
{unsigned f,n;
	n=(unsigned)repindexes.size();
	cout << "Current Repetition Indexes (" << n << ")" << endl;
	for (f=0; f<n; f++)
		cout << "\tIndex " << f << "th in " << repindexes[f].repareaname << "#" << repindexes[f].index << endl;
}

/* ---------------- Auxiliary Routines for the RepMacroSubstitutions type ------------------- */

static unsigned AddRepMacroSubstitutions(MacroScript::RepMacroSubstitutions &rms, const MacroScript::RepMacroRegistration &rmr)
/* Add a new repmacro registration to RMS, returning the index in RMS where it is added */
{
	rms.push_back(rmr);
	return((unsigned)(rms.size()-1));
}

/* ---------------- Auxiliary Routines for the RepAreaRepetitions type ------------------- */

static int FindRepAreaNumberOfRepetitions(const MacroScript::RepAreaRepetitions &rars, const string &zonename, const string &repareaname)
/* Return the index into RARS of a reparea of name REPAREANAME and zone ZONENAME, or -1 if no such area is in RARS */
{unsigned n,f;
	n=(unsigned)rars.size();
	for (f=0; f<n; f++)
		if ((rars[f].zone==zonename)&&(rars[f].name==repareaname)) return(f);
	return(-1);
}

static unsigned AddRepAreaNumberOfRepetitions(MacroScript::RepAreaRepetitions &rars, const MacroScript::RepAreaNumberOfRepetitions &ranr)
/* Add a new repmacro registration to RMS, returning the index in RMS where it is added */
{
	rars.push_back(ranr);
	return((unsigned)(rars.size()-1));
}

/* ---------------- Auxiliary Routines for the repmacro registration ------------------- */

static bool DeconstructIndexesRepMacro(const string &id, MacroScript::IndexesRepetitions &indexesmap)
/* Store into INDEXESMAP the vector of indexes registered in the text ID */
{unsigned f,g,i;
 string a,b;
 MacroScript::RepAreaIndex rai;

	g=find_substr(id,"[",0);
	if (g==id.npos) return(false);
	indexesmap.clear();
	while (g!=id.npos)
	{
		f=find_substr(id,"#",g);
		if (f==id.npos) return(false);
		a=id.substr(g+1,f-g-1);
		g=find_substr(id,"]",f);
		if (g==id.npos) return(false);
		b=id.substr(f+1,g-f-1);
		string_to_number(b,i);
		rai.repareaname=a;
		rai.index=i;
		AddRepAreaIndex(indexesmap,rai);
		g=find_substr(id,"[",g);
	}
	return(true);
}

/* ---------------- Auxiliary Routines for Algorithms ------------------- */

static int AreWeInRepArea(const MacroScript::IndexesRepetitions &indexes, const string &repareaname)
/* If the program is running the script currently within reparea REPAREANAME, return the current
index for that area. Otherwise, return -1 */
{int f;

	//cout << "arewe in area " << repareaname << endl;
	//PrintIndexes(indexes);
	f=FindRepAreaIndex(indexes,repareaname); 
	//cout << "despues de findrepareaindex " << f << endl;
	if (f<0) return(-1);
	return(indexes[f].index);
}

static bool IsRepMacroActive(const MacroScript::IndexesRepetitions &indexes,
							 const MacroScript::RepMacroSubstitutions &repsubsts,
							 const string repmacro, const string repareaname, const string zonename,
							 string &value)
/* Return TRUE if there is any registration for the repmacro (ZONENAME::REPAREANAME::REPMACRO)
given that we are currently applying macros and the indexes of repareas are INDEXES.
If it returns TRUE, store into VALUE the value that substitutes the repmacro in that situation */
{unsigned imsize,imf,rsf,rssize;
 int indexesf;
 bool coinc;

	// given the current indexes, search for the value of the repmacro for those indexes
	rssize=(unsigned)repsubsts.size();
	for (rsf=0; rsf<rssize; rsf++)	// For each registration of any repmacro
	{
		if ((repsubsts[rsf].zone==zonename)&&
			(repsubsts[rsf].reparea==repareaname)&&
			(repsubsts[rsf].name==repmacro))	// If the registration is for the same repmacro
		{	// We have found a registration of the same repmacro
			coinc=true;
			imsize=(unsigned)repsubsts[rsf].indexes.size();	
			for (imf=0; imf<imsize; imf++)
			{
				indexesf=AreWeInRepArea(indexes,repsubsts[rsf].indexes[imf].repareaname);	// Are we currently into that area?
				if ((indexesf<0)||(indexesf!=(int)repsubsts[rsf].indexes[imf].index)) coinc=false;	// We are not or not with its current index
			}
			if (coinc) // All the indexes of the MSI registration coincides with the current indexes
			{
				value=repsubsts[rsf].value;
				return(true);
			}
		}
	}
	return(false);
}

static unsigned NumberOfRepetitionsOfRepArea(const string zonename, const string repareaname,
											 unsigned numberspecified, unsigned numberoverriden,
											 const MacroScript::RepMacroSubstitutions &repmacros,
											 const MacroScript::IndexesRepetitions &indexes)
/* Calculate the number of repetitions of the given rep-area, or 0 if such area does not
exist or does not have any repetition registered. NUMBERSPECIFIED is the number that has
been written as second parameter of the reparea, or 0 if no number is written there.
NUMBEROVERRIDEN is the number specified by the user through "SetNumberOfRepetitions", or 0
if the user has not done that.
REPMACROS is the set of registered repmacros by the user,
INDEXES is the current status (during applymacros) of the indexes within the repareas where we are.

  NOTE: This function should only be called when running the script within the given area */
{unsigned nreps,nrepsbymacro;
 unsigned indexesmapsize,indexesmapf,rsf,rssize;
 int whereweare;
 bool included;
 string v;

 
	//cout << "NumberOfRepetitionsOfRepArea(" << zonename << ", " << repareaname << ", " << numberspecified << ", " << numberoverriden << ")" << endl;

		// Except for the registering of repmacros, the number of repetitions is
		// the number overriden or, if none, the number specified
	if (numberoverriden==0) nreps=numberspecified;
	else nreps=numberoverriden;
//	cout << "Nreps=" << nreps << endl;
	
		// The registered repmacros have the highest priority

	rssize=(unsigned)repmacros.size();
	for (rsf=0; rsf<rssize; rsf++)	// For each registration of a repmacro
	{	
		indexesmapsize=(unsigned)repmacros[rsf].indexes.size(); 
		included=false;
		nrepsbymacro=0;
		for (indexesmapf=0; indexesmapf<indexesmapsize; indexesmapf++)	// For each index in the registration
		{
			if (repmacros[rsf].indexes[indexesmapf].repareaname==repareaname)	// The registration includes the reparea
			{
				// If we have reach the area in the indexes of the macro, take the index assigned to it
				nrepsbymacro=repmacros[rsf].indexes[indexesmapf].index;
				included=true;
				break;
			}
			else	// Take into account the number given by the repmacro
			{
				whereweare=AreWeInRepArea(indexes,repmacros[rsf].indexes[indexesmapf].repareaname);
				if (!((whereweare>=0)&&(whereweare==(int)repmacros[rsf].indexes[indexesmapf].index)))
					// We are either outside that area or with other index
				{
					included=false;
					break;
				}
			}
		}
		if (included)
		{
			//cout << "Taking repamcro!" << nrepsbymacro << endl;
			if (nrepsbymacro+1>nreps) nreps=nrepsbymacro+1;
		}
	}

	//cout << "Final nreps: " << nreps << endl;
	return(nreps);	
}

static void adderrortoresult(string &result, const string &errortxt)
{
	result=result+"\n**** PARSING ERROR **** : "+errortxt;
}

/* ---------------- Constructors ------------------- */

MacroScript::MacroScript(const string s)
{
	script=s;
	CleanRegister();
	CleanBackup();
}

MacroScript::MacroScript(ifstream &ifs, unsigned lb)
{
	script=ReadEverything(ifs,0,lb);
	CleanRegister();
	CleanBackup();
}

MacroScript::~MacroScript(void)
{
	CleanRegister();
	CleanBackup();
}

/* ---------------- Private Methods ------------------- */

void MacroScript::ClearStatus(RegisteringStatus &stat)
{
	stat.substitutions.clear();
	stat.numberreps.clear();
	stat.repsubsts.clear();
}

bool MacroScript::FindZone(const string zonename, unsigned pos0, JMSKeyword &zone, bool readbody)
/* Find the zone with name ZONENAME in the macroscript (zonename must not be empty) from the
position POS0. If it is found, read it into ZONE. If READBODY==FALSE, the body is not read */
{JMSKeyword k;

	if (zonename.empty()) return(false);
	zone.Clear();
	if (!zone.LocateGivenKeyword(script,JMSKeyword::JMS_K_ZONE,pos0,zonename)) return(false);
	return(zone.Read(script,pos0,readbody));
}

bool MacroScript::SubstituteEverything(string &result, const string zonename)
/* Given a text in RESULT, change it by substituting its macros and keywords. 
ZONENAME is the name of the zone where the text has been extracted from */
{string macroname,macrodef,macrocomp,substit,idrepmacro,repname;
 unsigned p0,p1,pos;
 JMSKeyword k;
 bool fin,hayerror;
 JMSKeyword::JMSKeywordType t;

	if (result.empty()) return(true);

//	cout << "--> entrando en subtitute everything" << endl;

	pos=0;
	fin=false;
	while (!fin)
	{
		if (!k.Locate(result,t,pos)) fin=true;
		else
		{
//cout << "************* LEYENDO KEYWORD **************** " << endl;

			if ((!k.Read(result,pos))||
				(k.Type()!=t)) 
			{
//cout << "Leyendo :[" << endl << result.substr(pos) << "]" << endl;
//cout << "tipo= " << t << endl;
				adderrortoresult(result,"Error reading a previous located keyword");
				return(false);
			}
//k.Print(cout);
			hayerror=false;
			switch (k.Type())
			{
				case JMSKeyword::JMS_K_LIT:
								{	
									k.CopyBody(0,substit); 
									break;
								}	
				case JMSKeyword::JMS_K_MACRO:
								{	
									if ((!k.CopyArgument(0,macroname))||
										(!k.CopyArgument(1,macrodef))) 
									{
										adderrortoresult(result,"Error reading arguments from a macro");
										hayerror=true;
										break;
									}
									MacroSubstitutions::iterator mss=status.substitutions.find(macroname);
									if (mss!=status.substitutions.end()) substit=(*mss).second;
									else substit=macrodef;
									break;
								}
				case JMSKeyword::JMS_K_REPMACRO:
								{
									if (!SubstituteRepMacro(zonename,k,substit))
									{
										adderrortoresult(result,"Error dealing with repmacro");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_REPINDEX:
								{
									if (!k.CopyArgument(0,repname))
									{
										adderrortoresult(result,"Error reading reparea name for a repindex");
										hayerror=true;
										break;
									}
									int f=FindRepAreaIndex(repindexes,repname);
									if (f<0) substit="";
									else number_to_string(repindexes[f].index,substit);
									break;
								}
				case JMSKeyword::JMS_K_REPAREA:
								{
									if (!SubstituteRepArea(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with a reparea");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_CONDITIONAL:
								{
									if (!SubstituteIfArea(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with ifarea");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_CONDITIONAL_EQ:
								{
									if (!SubstituteIfEQArea(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with ifeqarea");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_CONDITIONAL_REP:
								{
									if ((!k.CopyArgument(0,repname))||
										(!k.CopyArgument(1,macroname))||
										(!k.CopyArgument(2,macrodef))) 
									{
										adderrortoresult(result,"Error copying arguments from an ifrep");
										hayerror=true;
										break;
									}
									int f=FindRepAreaIndex(repindexes,repname);
									if (f<0) 
									{
										adderrortoresult(result,"Error finding a repareaindex");
										hayerror=true;
										break;
									}
									if (!SubstituteIfRepArea(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with ifreparea");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_CONDITIONAL_REPEQ:
								{
									if ((!k.CopyArgument(0,repname))||
										(!k.CopyArgument(1,macroname))||
										(!k.CopyArgument(2,macrocomp))) 
									{
										adderrortoresult(result,"Error copying arguments for a ifrepeq");
										hayerror=true;
										break;
									}
									int f=FindRepAreaIndex(repindexes,repname);
									if (f<0) 
									{
										adderrortoresult(result,"Error finding the index of a ifrepeq");
										hayerror=true;
										break;
									}
									if (!SubstituteIfRepEQArea(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with an ifrepeq");
										hayerror=true;
										break;
									}
									break;
								}
				case JMSKeyword::JMS_K_INCLUDE:	
								{
									if (!SubstituteInclude(zonename,k,substit)) 
									{
										adderrortoresult(result,"Error dealing with include");
										hayerror=true;
										break;
									}
									break;
								}
				default:		adderrortoresult(result,"Unknown keyword case found");	
								return(false);
			}	// switch
			if (hayerror) 
			{
			//	cout << "Error en la siguiente keyword:" << endl;
			//	k.Print(cout);
				k.OriginalPosition(p0,p1); // For copying error messages written by subroutines
				result.erase(p0,p1+1-p0);
				result.insert(p0,substit);
				return(false);
			}
			k.OriginalPosition(p0,p1);
			result.erase(p0,p1+1-p0);
			result.insert(p0,substit);
			pos=p0+substit.size();
		} // else -locate-
	}	// while (!fin)

	return(true);
}

bool MacroScript::SubstituteRepMacro(const string zonename, JMSKeyword &repmacro, string &result)
/* Given a recently read repmacro in REPMACRO into zone ZONENAME; substitute it by its value and
return TRUE if no error */
{string repname,macroname,macrodef;

	// retrieve keyword data
	if ((!repmacro.CopyArgument(0,repname))||	// The reparea to which the repmacro belongs
		(!repmacro.CopyArgument(1,macroname))||	// The name of the repmacro
		(!repmacro.CopyArgument(2,macrodef)))	// Its default value
	{
		adderrortoresult(result,"Error copying arguments for a repmacro");
		return(false);
	}

	// given the current indexes, search for the value of the repmacro for those indexes
	if (!IsRepMacroActive(repindexes,status.repsubsts,macroname,repname,zonename,result)) result=macrodef;
	return(true);
}

bool MacroScript::SubstituteRepArea(const string zonename, JMSKeyword &reparea, string &result)
/* Given in REPAREA a keyword JMS-REPAREA substitute that area for the resulting text, 
storing it in RESULT */
{unsigned nreps;
 int f;
 string repareaname,nrepstxt,sarea;

		// Read rep-area name and default numbers of repetitions

	if (!reparea.CopyArgument(0,repareaname)) 
	{
		adderrortoresult(result,"Error copying arguments for a reparea");
		return(false);
	}
	if (repareaname.empty()) 
	{
		adderrortoresult(result,"Reparea name empty");
		return(false);
	}

	if (!reparea.CopyArgument(1,nrepstxt)) nreps=0;
	else string_to_number(nrepstxt,nreps);

	f=FindRepAreaNumberOfRepetitions(status.numberreps,zonename,repareaname);
	if (f<0) f=0;
	else f=status.numberreps[f].repetitions;

	//cout << "Entrando en reparea [" << repareaname << "]" << endl;
	nreps=NumberOfRepetitionsOfRepArea(zonename,repareaname,
									   nreps,f,
									   status.repsubsts,repindexes);


		// Repeats the area

	result="";
	for (f=0; f<(int)nreps; f++)
	{
		//cout << "repeating for index " << f << endl;

		// Set the current index for the reparea
		int idx=FindRepAreaIndex(repindexes,repareaname);
		if (idx<0) 
		{RepAreaIndex rai;

			rai.repareaname=repareaname;
			rai.index=f;
			AddRepAreaIndex(repindexes,rai);
		}
		else repindexes[idx].index=f;

		// Substitute the area
		reparea.CopyBody(0,sarea);
		if (!SubstituteEverything(sarea,zonename)) 
		{
			result+=sarea;		
			adderrortoresult(result,"Error dealing with substituting everything into body of reparea");
			return(false);
		}
		result+=sarea;		
	}
	// Once terminated, the index is cleared
	int idx=FindRepAreaIndex(repindexes,repareaname);
	if (idx>=0) DeleteRepAreaIndex(repindexes,idx);

	return(true);
}

bool MacroScript::SubstituteIfArea(const string zonename, JMSKeyword &ifarea, string &result)
/* Given in IFAREA an IF area, substitute that area for the resulting text, storing it in 
RESULT. */
{string ifmacroname,ifdefvalue;
 bool r;

		// Read the ifmacro

	if (!ifarea.CopyArgument(0,ifmacroname)) 
	{
		adderrortoresult(result,"Error copying arguments for an ifarea");
		return(false);
	}
	if (ifmacroname.empty()) 
	{
		adderrortoresult(result,"Ifarea name empty");
		return(false);
	}
	ifarea.CopyArgument(1,ifdefvalue);

		// Get the actual truth value of the area

	MacroSubstitutions::iterator mss=status.substitutions.find(ifmacroname);
	if (mss!=status.substitutions.end()) ifdefvalue=(*mss).second;
	ifdefvalue=upper_str(ifdefvalue);

		// Interpret the appropriate subarea

	ifarea.CopyBody((ifdefvalue=="TRUE")?0:1,result);
	r=SubstituteEverything(result,zonename);
	if (!r) adderrortoresult(result,"Error substituting everything into body of an ifarea");
	return(r);
}

bool MacroScript::SubstituteIfEQArea(const string zonename, JMSKeyword &ifarea, string &result)
/* Given in IFAREA an IF-EQ area, substitute that area for the resulting text, storing it in 
RESULT. */
{string ifmacroname,ifcompvalue,ifmacrovalue;
 bool r;

		// Read the ifmacro

	if (!ifarea.CopyArgument(0,ifmacroname)) 
	{
		adderrortoresult(result,"Error copying arguments for an ifeq");
		return(false);
	}
	if (ifmacroname.empty()) 
	{
		adderrortoresult(result,"Ifeq name empty");
		return(false);
	}
	ifarea.CopyArgument(1,ifcompvalue);

		// Get the actual value of the macro

	MacroSubstitutions::iterator mss=status.substitutions.find(ifmacroname);
	if (mss!=status.substitutions.end()) ifmacrovalue=(*mss).second;
	else ifmacrovalue="";

		// Interpret the appropriate subarea

	ifarea.CopyBody((ifcompvalue==ifmacrovalue)?0:1,result);
	r=SubstituteEverything(result,zonename);
	if (!r) adderrortoresult(result,"Error substituting everything into body of an ifeq");
	return(r);
}

bool MacroScript::SubstituteIfRepArea(const string zonename, JMSKeyword &ifreparea, string &result)
/* Given in IFREPAREA an IFREP area, substitute that area for the resulting text, storing it in 
RESULT */
{string macroname,reparea,idrepmacro,ifdefvalue;
 bool r;

		// Read the ifmacro

	if ((!ifreparea.CopyArgument(0,reparea))||
		(!ifreparea.CopyArgument(1,macroname))||
		(!ifreparea.CopyArgument(2,ifdefvalue))) 
	{
		adderrortoresult(result,"Error copying arguments for an ifreparea");
		return(false);
	}
	if ((macroname=="")||(reparea==""))
	{
		adderrortoresult(result,"Macroname or repareaname empty in an ifrep");
		return(false);
	}

		// Get the actual truth value of the area and substitute

	if (!IsRepMacroActive(repindexes,status.repsubsts,macroname,reparea,zonename,ifdefvalue)) 
		ifreparea.CopyBody(1,result);
	else
	{
		ifdefvalue=upper_str(ifdefvalue);
		ifreparea.CopyBody((ifdefvalue=="TRUE")?0:1,result);
	}
	r=SubstituteEverything(result,zonename);
	if (!r) adderrortoresult(result,"Error substituting everything into body of an ifreparea");
	return(r);
}

bool MacroScript::SubstituteIfRepEQArea(const string zonename, JMSKeyword &ifreparea, string &result)
/* Given in IFREPAREA an IFREPEQ area, substitute that area for the resulting text, storing it in 
RESULT */
{string macroname,reparea,idrepmacro,ifcompvalue,ifmacrovalue;
 bool r;

		// Read the ifmacro

	if ((!ifreparea.CopyArgument(0,reparea))||
		(!ifreparea.CopyArgument(1,macroname))||
		(!ifreparea.CopyArgument(2,ifcompvalue)))
	{
		adderrortoresult(result,"Error copying arguments for an ifrepeq");
		return(false);
	}
	if ((macroname=="")||(reparea=="")) 
	{
		adderrortoresult(result,"Macroname or repareaname empty in an ifrepeq");
		return(false);
	}

		// Get the actual truth value of the area and substitute

	if (!IsRepMacroActive(repindexes,status.repsubsts,macroname,reparea,zonename,ifmacrovalue)) ifmacrovalue="";
	ifreparea.CopyBody((ifmacrovalue==ifcompvalue)?0:1,result);
	r=SubstituteEverything(result,zonename);
	if (!r) adderrortoresult(result,"Error substituting everything into body of an ifrepeq");
	return(r);
}

bool MacroScript::SubstituteInclude(const string zonename, JMSKeyword &includekeyword, string &result)
/* Given in INCLUDEKEYWORD an INCLUDE keyword substitute that keyword for the resulting text, 
storing it in RESULT */
{JMSKeyword zone;
 string macroname,macrovalue,inczonename;
 bool res;
 unsigned f;
	
		// Search the included zone

	if (!includekeyword.CopyArgument(0,inczonename)) 
	{
		adderrortoresult(result,"Error copying arguments for an include");
		return(false);
	}
//	cout << "Include: inczonename=" << inczonename << endl;
	if (!FindZone(inczonename,0,zone,true)) 
	{
		adderrortoresult(result,"Name of included zone empty");
		return(false);
	}

		// Reserve the current status

	BackupRegister();

		// Register the values for the included zone

	for (f=1; f<zone.NumberOfArguments(); f++)
	{
		if (!zone.CopyArgument(f,macroname)) 
		{
			RestoreRegister();
			adderrortoresult(result,"Error copying argument of an including macro");
			return(false);
		}
		if ((includekeyword.CopyArgument(f,macrovalue))&&(!macrovalue.empty()))
		{
			if (!RegisterMacro(macroname,macrovalue)) 
			{
				RestoreRegister();
				adderrortoresult(result,"Error registering macro into an include");
				return(false);
			}
//			cout << "Macro [" << macroname << "] registered with value [" << macrovalue << "]" << endl;
		}
	}

		// Translate the included zone

//	cout << "Substituting into:" << endl;
//	zone.Print(cout);
	zone.CopyBody(0,result);
	res=SubstituteEverything(result,inczonename);

		// Restore the status

	RestoreRegister();
	CleanBackup();
	if (!res) adderrortoresult(result,"Error substituting everything into body of an include");
	return(res);
}


/* ---------------- Public Methods ------------------- */

void MacroScript::PrintStatus(const string &s) const
{MacroSubstitutions::const_iterator msi;
 unsigned rsf,rssize,nrsize,nrf;

	cout << endl << "Internal Status of Macroscript #" << this << " [" << s << "]" << endl << endl;
	cout << "Internal substitutions for normal macros: " << endl;
	msi=status.substitutions.begin();
	while (msi!=status.substitutions.end())
	{
		cout << "\tMacro: [" << (*msi).first << "] into (" << (*msi).second << ")" << endl;
		msi++;
	}
	cout << "Internal substitutions for rep-macros: " << endl;
	rssize=(unsigned)status.repsubsts.size();
	for (rsf=0; rsf<rssize; rsf++)
		cout << "\t" << status.repsubsts[rsf].zone+"::"+status.repsubsts[rsf].reparea+"::"+status.repsubsts[rsf].name << " into (" << status.repsubsts[rsf].value << ")" << endl;
	cout << "Number of repetitions for rep-areas: " << endl;
	nrsize=(unsigned)status.numberreps.size(); 
	for (nrf=0; nrf<nrsize; nrf++)
		cout << "\tRepArea: [" << status.numberreps[nrf].zone+"::"+status.numberreps[nrf].name << "] repeats " << status.numberreps[nrf].repetitions << " times" << endl;
	PrintIndexes(repindexes);
	cout << "-- End Internal Status " << endl;
}

bool MacroScript::FindNextZone(string &zonename, unsigned &pos0, unsigned &pinit) const
{JMSKeyword k;
 JMSKeyword zone;

	zone.Clear();
	if (!zone.LocateGivenKeyword(script,JMSKeyword::JMS_K_ZONE,pos0)) return(false);
	if (!zone.Read(script,pos0,true)) return(false);
	zone.OriginalPosition(pinit,pos0);
	pos0++;
	if (!zone.CopyArgument(0,zonename)) return(false);
	return(true);
}

bool MacroScript::RegisterMacro(const string macroname, const string macrovalue)
{
	if (macroname.empty()) return(false);
	status.substitutions[macroname]=macrovalue;
	return(true);
}

void MacroScript::CleanRegister(void)
{
	ClearStatus(status);
}

void MacroScript::BackupRegister(bool clearstat)
{
	statbackup.substitutions=status.substitutions;
	statbackup.numberreps=status.numberreps;
	statbackup.repsubsts=status.repsubsts;
	if (clearstat) ClearStatus(status);
}

void MacroScript::RestoreRegister(void)
{
	status.substitutions=statbackup.substitutions;
	status.numberreps=statbackup.numberreps;
	status.repsubsts=statbackup.repsubsts;
}

void MacroScript::CleanBackup(void)
{
	ClearStatus(statbackup);
}

int MacroScript::NumberOfMacros(const string zonename)
{JMSKeyword zone;

	if (zonename.empty()) return(-1);
	if (!FindZone(zonename,0,zone,false)) return(-1);
	return((int)zone.NumberOfArguments()-1);
}

bool MacroScript::IndexedMacroName(string &macroname, const unsigned im, const string zonename)
{JMSKeyword zone;

	if (zonename.empty()) return(false);
	if (!FindZone(zonename,0,zone,false)) return(false);
	return(zone.CopyArgument(im+1,macroname));
}

bool MacroScript::SetNumberOfRepetitions(const string zonename, const string repareaname,
										 unsigned numreps)
{RepAreaNumberOfRepetitions ranor;

	ranor.zone=zonename;
	ranor.name=repareaname;
	ranor.repetitions=numreps;
	AddRepAreaNumberOfRepetitions(status.numberreps,ranor);
	return(true);
}

bool MacroScript::RegisterRepetitionMacroValue(const string zonename, const string repareaname,
											   const string repmacroname, 
											   const string indexes, 
											   const string value)
{RepMacroRegistration rmr;

		// Register the substitution

	rmr.zone=zonename;
	rmr.reparea=repareaname;
	rmr.name=repmacroname;
	rmr.value=value;
	if (!DeconstructIndexesRepMacro(indexes,rmr.indexes)) return(false);
	AddRepMacroSubstitutions(status.repsubsts,rmr);
	return(true);
}

bool MacroScript::RegisterRepetitionMacroValue(const string zonename, const string repareaname,
											  const string repmacroname, 
											  const unsigned index,
											  const string value)
{string indexes,n;

	number_to_string(index,n);
	indexes="["+repareaname+"#"+n+"]";
	return(RegisterRepetitionMacroValue(zonename,repareaname,repmacroname,indexes,value));
}

bool MacroScript::ApplyMacros(const string zonename, string &result, unsigned numtabs, char tabch)
{JMSKeyword zone;

	if (!FindZone(zonename,0,zone,true)) 
	{
		adderrortoresult(result,"zone not found");
		return(false);
	}

//	unsigned p0,p1;
//	zone.Print(cout);
//	zone.OriginalPosition(p0,p1);
//	cout << "[" << script.substr(p0,p1+1-p0) << "]" << endl;

	zone.CopyBody(0,result);
	repindexes.clear();	// There is no index for repareas yet
//cout << "Entrando en substitute main" << endl;
	if (!SubstituteEverything(result,zonename)) 
	{
		adderrortoresult(result,"failed to susbstitute everything in zone");
		return(false);
	}
	indent_str(result,numtabs,tabch);
	return(true);
}
