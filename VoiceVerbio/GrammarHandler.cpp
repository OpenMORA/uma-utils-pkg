/*-----------------------------------------------------------*/
/*	GRAMMAR HANDLER LIBRARY FOR BABEL MODULES ARCHITECTURE   */
/*                                                           */
/*	ECM, 2006                                                */
/*-----------------------------------------------------------*/

/** \file GrammarHandler.cpp
 *\brief Implementation of GrammarHandler.hpp
 */
#include <cstdio>
#include <cstdlib>
#include "GrammarHandler.hpp"

using namespace std;

/*--------------*
 * Object class *
 *--------------*/

GrammarHandler::Object::Object()
{
}

GrammarHandler::Object::Object(const string & label, const ArchitectureGrammar::ObjectNouns &nouns,const string &typeValue)
{
	objLabel = label;
	objNouns = nouns;
	objType = typeValue;
}

GrammarHandler::Object::Object(const GrammarHandler::Object & originObject)
{
	copyObject(originObject);
}

GrammarHandler::Object::~Object()
{
}

GrammarHandler::Object & GrammarHandler::Object::operator =(const GrammarHandler::Object & originObject)
{
	copyObject(originObject);
	return(*this);
}
void GrammarHandler::Object::setLabel(const std::string &label)
{
	objLabel = label;
}

string GrammarHandler::Object::getLabel()
{
	return(objLabel);
}

void GrammarHandler::Object::setNouns(const ArchitectureGrammar::ObjectNouns & nouns)
{
	objNouns = nouns;
}

ArchitectureGrammar::ObjectNouns GrammarHandler::Object::getNouns()
{
	return(objNouns);
}

void GrammarHandler::Object::setType(const std::string &typeValue)
{
	objType = typeValue;
}

string GrammarHandler::Object::getType()
{
	return(objType);
}

void GrammarHandler::Object::copyObject(const GrammarHandler::Object &originObject)
{
	objLabel = originObject.objLabel;
	objNouns = originObject.objNouns;
	objType = originObject.objType;
}
/* end Object class */

/*--------------*
 * Action class *
 *--------------*/

GrammarHandler::Action::Action()
{
}

GrammarHandler::Action::Action(const std::string &label, const ArchitectureGrammar::WordsList &words, const ArchitectureGrammar::WordsList &params, NonTerminal::NonTerminalTransitivity transitivity)
{
	actLabel = label;
	actWords = words;
	actParams = params;
	actTransitivity = transitivity;
}

GrammarHandler::Action::Action(const GrammarHandler::Action & originAction)
{
	copyAction(originAction);
}

GrammarHandler::Action::~Action()
{
}

GrammarHandler::Action & GrammarHandler::Action::operator =(const GrammarHandler::Action & originAction)
{
	copyAction(originAction);
	return(*this);
}

void GrammarHandler::Action::setLabel(const string &label)
{
	actLabel = label;
}

string GrammarHandler::Action::getLabel()
{
	return(actLabel);
}

void GrammarHandler::Action::setWords(const ArchitectureGrammar::WordsList & words)
{
	actWords = words;
}

ArchitectureGrammar::WordsList GrammarHandler::Action::getWords()
{
	return(actWords);
}

void GrammarHandler::Action::setParams(const ArchitectureGrammar::WordsList & params)
{
	actParams = params;
}

ArchitectureGrammar::WordsList GrammarHandler::Action::getParams()
{
	return(actParams);
}

void GrammarHandler::Action::setTransitivity(NonTerminal::NonTerminalTransitivity transitivity)
{
	actTransitivity = transitivity;
}

NonTerminal::NonTerminalTransitivity GrammarHandler::Action::getTransitivity()
{
	return(actTransitivity);
}

void GrammarHandler::Action::copyAction(const GrammarHandler::Action &originAction)
{
	actLabel = originAction.actLabel;
	actWords = originAction.actWords;
	actParams = originAction.actParams;
	actTransitivity = originAction.actTransitivity;
}
/* end Action class */

/*----------------------*
 * GrammarHandler class *
 *----------------------*/
GrammarHandler::GrammarHandler()
{
}

GrammarHandler::GrammarHandler(void * reqMethods)
{
	eiMethods = (neededMethods *)reqMethods;
}

GrammarHandler::~GrammarHandler()
{
}

unsigned GrammarHandler::retrieveObjects()
{
	GrammarHandler::Object currentObject;
	unsigned noAddObjects = 0;
	//First step: Retrieval of the objects from the WorldModel
	unsigned objsNoInWM = eiMethods->getWMNumberOfObjects();
	printf("GrammarHandlerObj: Number of objects to retrieve from the WorldModel: %u\n",objsNoInWM);
	for(int i = 0;i < (int)objsNoInWM;i++)
	{
		currentObject = eiMethods->getWMObjectAt(i);
		associatedObjects.push_back(currentObject);
		noAddObjects++;
	}
	//Second step: Retrieval of the objects from the TaskVoice
	unsigned objsNoInTP = eiMethods->getTPNumberOfObjects();
	printf("GrammarHandlerObj: Number of objects to retrieve from the TaskVoice: %u\n",objsNoInTP);
	for(int h = 0;h < (int)objsNoInTP;h++)
	{
		currentObject = eiMethods->getTPObjectAt(h);
		associatedObjects.push_back(currentObject);
		noAddObjects++;
	}

	//Print status message
	printf("GrammarHandlerObj: Retrieval of the objects finished: %u objects retrieved\n",noAddObjects);
	return(noAddObjects);
}

unsigned GrammarHandler::retrieveActions()
{
	GrammarHandler::Action currentAction;
	unsigned noAddActions = 0;

	unsigned actsNoInTP = eiMethods->getTPNumberOfActions();
	for(int i = 0;i < (int)actsNoInTP;i++)
	{
		currentAction = eiMethods->getTPActionAt(i);
		associatedActions.push_back(currentAction);
		noAddActions++;
	}

	//Print status message
	printf("Retrieval of the actions finished: %u actions retrieved\n",noAddActions);
	return(noAddActions);
}

bool GrammarHandler::convert2Rules()
{
	bool retVal = true;

	//First step: Insertion of all the objects
	list<GrammarHandler::Object>::iterator objIter;

	objIter = associatedObjects.begin();
	while(objIter != associatedObjects.end())
	{
		if(!associatedAGrammar.addNewObject((*objIter).getLabel(),(*objIter).getNouns(),(*objIter).getType()))
		{
			if(retVal)
			{
				retVal = false;
			}
		}
		objIter++;
	}

	//Second step: Insertion of all the actions
	list<GrammarHandler::Action>::iterator actIter;

	actIter = associatedActions.begin();
	while(actIter != associatedActions.end())
	{
		if(!associatedAGrammar.addNewAction((*actIter).getLabel(),(*actIter).getWords(),(*actIter).getParams(),(*actIter).getTransitivity()))
		{
			if(retVal)
			{
				retVal = false;
			}
		}
		actIter++;
	}
	return(retVal);
}

string GrammarHandler::getGrammarString(const string & outputFormat)
{
	return(associatedAGrammar.getGrammarAsString(outputFormat));
}

string GrammarHandler::getAssociatedAction(const list<string> actionRules)
{
	list<string>::const_iterator rulesIter;
	list<string> auxList;

	auxList = actionRules;
	rulesIter =actionRules.begin();
	return(associatedAGrammar.getAssociatedAction(atoi((*rulesIter).c_str()),auxList));
}
/* end GrammarHandler class */
