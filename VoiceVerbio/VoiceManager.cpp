/*---------------------------------------------------------*/
/*	VOICE MANAGER LIBRARY FOR BABEL MODULES ARCHITECTURE   */
/*                                                         */
/*	ECM, 2007                                              */
/*---------------------------------------------------------*/

/** \file VoiceManager.cpp
 *\brief Implementation of VoiceManager.hpp
 */
#include "VoiceManager.hpp"
#include "FileStringReader.hpp"
#include "ArchitectureGrammar.hpp"
#include <mrpt/system.h>
using namespace std;

/*--------------*
 * Object class *
 *--------------*/
VoiceManager::Object::Object()
{
}

VoiceManager::Object::Object(const string & label, const ArchitectureGrammar::ObjectNouns &nouns,const string &typeValue)
{
	objLabel = label;
	objNouns = nouns;
	objType = typeValue;
}

VoiceManager::Object::Object(const VoiceManager::Object & originObject)
{
	copyObject(originObject);
}

VoiceManager::Object::~Object()
{
}

VoiceManager::Object & VoiceManager::Object::operator =(const VoiceManager::Object & originObject)
{
	copyObject(originObject);
	return(*this);
}
void VoiceManager::Object::setLabel(const std::string &label)
{
	objLabel = label;
}

string VoiceManager::Object::getLabel()
{
	return(objLabel);
}

void VoiceManager::Object::setNouns(const ArchitectureGrammar::ObjectNouns & nouns)
{
	objNouns = nouns;
}

ArchitectureGrammar::ObjectNouns VoiceManager::Object::getNouns()
{
	return(objNouns);
}

void VoiceManager::Object::setType(const std::string &typeValue)
{
	objType = typeValue;
}

string VoiceManager::Object::getType()
{
	return(objType);
}

void VoiceManager::Object::copyObject(const VoiceManager::Object &originObject)
{
	objLabel = originObject.objLabel;
	objNouns = originObject.objNouns;
	objType = originObject.objType;
}
/* end Object class */

/*--------------*
 * Action class *
 *--------------*/

VoiceManager::Action::Action()
{
}

VoiceManager::Action::Action(const std::string &label, const ArchitectureGrammar::WordsList &words, const ArchitectureGrammar::WordsList &params, NonTerminal::NonTerminalTransitivity transitivity)
{
	actLabel = label;
	actWords = words;
	actParams = params;
	actTransitivity = transitivity;
}

VoiceManager::Action::Action(const VoiceManager::Action & originAction)
{
	copyAction(originAction);
}

VoiceManager::Action::~Action()
{
}

VoiceManager::Action & VoiceManager::Action::operator =(const VoiceManager::Action & originAction)
{
	copyAction(originAction);
	return(*this);
}

void VoiceManager::Action::setLabel(const string &label)
{
	actLabel = label;
}

string VoiceManager::Action::getLabel()
{
	return(actLabel);
}

void VoiceManager::Action::setWords(const ArchitectureGrammar::WordsList & words)
{
	actWords = words;
}

ArchitectureGrammar::WordsList VoiceManager::Action::getWords()
{
	return(actWords);
}

void VoiceManager::Action::setParams(const ArchitectureGrammar::WordsList & params)
{
	actParams = params;
}

ArchitectureGrammar::WordsList VoiceManager::Action::getParams()
{
	return(actParams);
}

void VoiceManager::Action::setTransitivity(NonTerminal::NonTerminalTransitivity transitivity)
{
	actTransitivity = transitivity;
}

NonTerminal::NonTerminalTransitivity VoiceManager::Action::getTransitivity()
{
	return(actTransitivity);
}

void VoiceManager::Action::copyAction(const VoiceManager::Action &originAction)
{
	actLabel = originAction.actLabel;
	actWords = originAction.actWords;
	actParams = originAction.actParams;
	actTransitivity = originAction.actTransitivity;
}
/* end Action class */

/*--------------------*
 * VoiceManager class *
 *--------------------*/
VoiceManager::VoiceManager(void * reqMethods, void * cMethods)
	: m_recog_thread_must_exit(false)
{
	//eiMethods = (neededRequestMethods *)reqMethods;
	//ecMethods = (concurrencyMethods *)cMethods;
	engineH = NULL;
	initStatus = false;

}

// A global variable to signal the shutdown of the class:
volatile bool verbio_has_to_stop_recording = false;

VoiceManager::~VoiceManager()
{
	// Stop recognition:
	if (!m_handle_recog_thread.isClear())
	{
		verbio_has_to_stop_recording = true;
		m_recog_thread_must_exit = true;
		mrpt::system::joinThread(m_handle_recog_thread);
	}

	switch(ourEngineType)
	{
		case VoiceEngineHandler::DEFAULT:   // JL: DEFAULT no estaba... esto es lo que se quiere no?
		case VoiceEngineHandler::VERBIO:
			VerbioHandler * verbioH = (VerbioHandler *)engineH;
			delete(verbioH);
			break;
	}
}

bool VoiceManager::InitVoiceInteraction(VoiceEngineHandler::VoiceEngineType engineType)
{
	bool retVal = true; //Returning value: true if all worked fine, false if error
	FileStringReader reader;
	string prefix;	//Prefix to build the naming elements
	string outputFormat; //Format of the output grammar string
	string formatFile;	//File to read the output format for the grammar
	string grammarName; //Name to identify the grammar
	string grammarStr;	//String that contains the grammar

	ourEngineType = engineType;

	//------------ENGINE INITIALIZATION ------------

	mrpt::synch::CCriticalSectionLocker lock(&sem);
	{
		if(!initStatus)
		{
			//Get the engine type:
			switch(ourEngineType)
			{
				case VoiceEngineHandler::VERBIO:
					engineH = new VerbioHandler();
					prefix = "VERBIO_";
					break;
				default:
					printf("VoiceManager: Sorry, unrecognized voice engine\n");
					retVal = false;
					break;
			}

			//------------GRAMMAR INITIALIZATION -----------
			list<Object> associatedObjects; //Objects that the modules understand
			list<Action> associatedActions; //Actions that the modules understand

			//-----Retrieval of the objects and actions-----
			VoiceManager::Object currentObject;
			//First step: Retrieval of the objects from the WorldModel

			for(size_t i = 0;i < elements_vec.size();i++)
			{
				//currentObject = eiMethods->getWMObjectAt(i);
				associatedObjects.push_back(elements_vec[i]);
			}
			printf("VoiceManager: Retrieval of the objects finished: %u objects retrieved\n",elements_vec.size());


			//Third step: Retrieval of actions

			for(size_t j = 0;j < actions_vec.size();j++)
			{
				associatedActions.push_back(actions_vec[j]);

			}

			//Print status message
			printf("VoiceManager: Retrieval of the actions finished: %u actions retrieved\n",actions_vec.size());

			//-----Convert the objects and actions to rules----
			//First step: Insertion of all the objects
			list<VoiceManager::Object>::iterator objIter;

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
			list<VoiceManager::Action>::iterator actIter;

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

			//------------ INITIALIZATION OF THE ENGINE -----------
			if(retVal) //(the grammar has been successfully built)
			{
				printf("Generating the grammar\n");
				formatFile = prefix + "architecture_template.jms";
				grammarName = "Generated_grammar";

				outputFormat = reader.readFile(formatFile.c_str());

				grammarStr = associatedAGrammar.getGrammarAsString(outputFormat);

				printf("%s\n",grammarStr.c_str());


				if( (actions_vec.size()!= 0) || (elements_vec.size()!= 0))

				{

					if(!engineH->Init(grammarName,grammarStr))
					{
						retVal = false;
					}
					else
					{
						initStatus = true;
					}
				}
				else
				{
					printf("VoiceManager: The grammar is empty.\n");
					retVal = false;
				}

			}

			//Check if there has been any error (if so, shutdown)
			if(!retVal)
			{
				//ecMethods->shutdown();
			}
		}
		//ecMethods->leaveCriticalZone();
	}
	/*else
	{
		retVal = false;
	}*/

	// If it's all OK, launch recognition thread:
	if (retVal)
	{
		std::cout << "[pVoiceVerbio] Launching recognition thread..." << std::endl;
		m_handle_recog_thread = mrpt::system::createThreadFromObjectMethod(this,&VoiceManager::recog_thread);
	}


return(retVal);
}

void VoiceManager::LoadWorldElements(std::vector<std::string> elements)
{

	std::deque<std::string> lista;
	std::deque<std::string> word_list;
	std::deque<std::string> param_list;





	for (size_t i=0;i<elements.size();i++)
	{
		mrpt::system::tokenize(elements[i],"#",lista);

		ArchitectureGrammar::ObjectNouns::nounInformation currentNoun;
		ArchitectureGrammar::ObjectNouns nouns;

		for (size_t j=2;j<lista.size();j++)
		{
			mrpt::system::tokenize(lista[j],",",word_list);


			currentNoun.associatedWord.append(word_list[0]);

			if (word_list[1]=="M") currentNoun.genre=NonTerminal::MASC;
			else currentNoun.genre=NonTerminal::FEM;

			if (word_list[2]=="S") currentNoun.number=NonTerminal::SINGULAR;
			else currentNoun.number=NonTerminal::PLURAL;

			nouns.addNoun2Object(currentNoun);

		}
		elements_vec.push_back(Object(lista[0],nouns,lista[1]));
		printf("Object %s\n",lista[0].c_str());
	}
}
void VoiceManager::LoadActions(std::vector<std::string> actions)
{
	std::deque<std::string> lista;
	std::deque<std::string> word_list;
	std::deque<std::string> param_list;

	ArchitectureGrammar::WordsList command_wl;
	ArchitectureGrammar::WordsList params_wl;
	NonTerminal::NonTerminalTransitivity transitivity;

	lista.clear();


	for (size_t i=0;i<actions.size();i++)
	{
		word_list.clear();
		param_list.clear();

		printf("Action %d:%s\n",i,actions[i].c_str());

		mrpt::system::tokenize(actions[i],"#",lista);

		mrpt::system::tokenize(lista[2],",",word_list);

		//printf("--->WordList %s\n",lista[2].c_str());
		if (lista.size()==4)
		{
		//	printf("Params: %s\n",lista[3].c_str());
			mrpt::system::tokenize(lista[3],",",param_list);  //Puede que no lleve parametros
		}


		if (lista[1]=="T") transitivity=NonTerminal::TRANSITIVE;
		else transitivity=NonTerminal::INTRANSITIVE;

		command_wl.clear();
		params_wl.clear();

		for (size_t j=0;j<word_list.size();j++)
		{
		//	printf("[%s]\n",word_list[j].c_str());
			command_wl.addWord(word_list[j]);
		}

		for (size_t k=0;k<param_list.size();k++)
		{
			printf("[%s]\n",param_list[k].c_str());
			params_wl.addWord(param_list[k]);
		}

		actions_vec.push_back(Action(lista[0],command_wl,params_wl,transitivity));
	}

}

bool VoiceManager::RecognitionProcess_private(string & recStr)
{
	VoiceEngineHandler::RecognitionStatus recStatus;
	list<string> rulesInfo; //Stores all the recognized rules antecedents
	list<string>::const_iterator rulesIter; //Rules iterator
	int antecedentTag;	//Tag of the current antecedent
	bool recResult = false;			//True if there is no error in the recognition process

	recStr = "";
	{
		//mrpt::synch::CCriticalSectionLocker lock(&sem);
		if(initStatus)
		{
			recStatus = engineH->RecognitionProcess(rulesInfo);
			if(recStatus == VoiceEngineHandler::REC_OK)
			{
				rulesIter = rulesInfo.begin();
				antecedentTag = atoi((*rulesIter).c_str());//Get the tag of the first antecedent
				recStr = associatedAGrammar.getAssociatedAction(antecedentTag,rulesInfo);
				recResult = true;
			}
		}
		else
		{
			recResult = false;
		}
	} // end lock sem

	return(recResult);
}

bool VoiceManager::RecognitionProcess_private(std::string &requestCode, std::string &goal)
{
	VoiceEngineHandler::RecognitionStatus recStatus;
	list<string> rulesInfo; //Stores all the recognized rules antecedents
	list<string>::const_iterator rulesIter; //Rules iterator
	string recStr; //Complete recognized string
	unsigned strIndex = 0; //Index to extract the goal list
	//bool reachFinal = false; // Final of the goal list reached or not
	int antecedentTag;	//Tag of the current antecedent
	bool recResult = false;			//True if there is no error in the recognition process

	//Cleaning of the parameters:
	requestCode = "";
	goal = "";

	recStr = "";

	{
		//mrpt::synch::CCriticalSectionLocker lock(&sem);

		if(initStatus)
		{
			recStatus = engineH->RecognitionProcess(rulesInfo);

			if(recStatus == VoiceEngineHandler::REC_OK)
			{
				rulesIter = rulesInfo.begin();
				antecedentTag = atoi((*rulesIter).c_str());//Get the tag of the first antecedent
				recStr = associatedAGrammar.getAssociatedAction(antecedentTag,rulesInfo);

				//Separation of the result
				// First step: get the Request Code:
				requestCode = recStr.substr(strIndex,recStr.find(" ",strIndex));
				strIndex = (unsigned)recStr.find(" ",strIndex)+1;
				//Second step: get the goal (if there is one)
				if(recStr.find(" ",0) != string::npos)
				{
					goal = recStr.substr(strIndex);
				}
				recResult = true;
			}
		}
		else
		{
			recResult = false;
		}
	} // end lock sem

	return(recResult);
}

void VoiceManager::QueueString2Reproduce(string str2Reproduce,unsigned prio,unsigned strTimeout)
{
	{
		mrpt::synch::CCriticalSectionLocker lock(&sem);
		prioQueue.push(str2Reproduce,prio,strTimeout);
	}

	//ReproduceQueuedStrings();
}

void VoiceManager::QueueString2Reproduce(string str2Reproduce,unsigned prio,unsigned strTimeout,unsigned groupId,double minTimeBtw)
{
	{
		mrpt::synch::CCriticalSectionLocker lock(&sem);
		prioQueue.push(str2Reproduce,prio,strTimeout,groupId,minTimeBtw);
	}
	//ReproduceQueuedStrings();
}

bool VoiceManager::ReproduceQueuedStrings(void)
{
	bool reproAny = false;
	{
		mrpt::synch::CCriticalSectionLocker lock(&sem);

		if(initStatus)
		{
			//Deactivate the active grammars to avoid feedback
			//engineH->PauseRecognitionProcess();
			while(!prioQueue.empty())
			{
				reproAny = true;
				engineH->ReproduceString(prioQueue.pop());
			}
			//Reactivate the active grammar
			//engineH->RestoreRecognitionProcess();
		}
		else
		{
			printf("VoiceManager: The voice engine must be initialized before the strings can be reproduced.\n");
		}
	}
	return reproAny;
}


/** Return a list of recognized phrases (recorded from an independent thread), ordered by time (the latest the most recent).
  *  The internal buffer of recognized strings is emptied with each call, so only new phrases will be returned upon each call.
  */
void VoiceManager::GetRecognizedPhrases(TRecognizedPhraseList &outList)
{
	mrpt::synch::CCriticalSectionLocker lock(&m_recog_buf_cs);
	outList = m_recog_buf;
	m_recog_buf.clear();
}

// Recognition thread:
void VoiceManager::recog_thread()
{
	cout << "[VoiceVerbio] Recognition thread is up." << endl;

	while (!m_recog_thread_must_exit)
	{
		string goal, requestedCode;
		if (RecognitionProcess_private(requestedCode, goal))
		{
			TRecognizedPhrase recog;
			recog.timestamp = mrpt::system::now();
			recog.requestedCode = requestedCode;
			recog.goal = goal;
			{
				mrpt::synch::CCriticalSectionLocker lock( &m_recog_buf_cs);
				m_recog_buf.push_back(recog);
			}
		}
	} // end while

	cout << "[VoiceVerbio] Recognition thread closed." << endl;
}

/* end VoiceManager class */
