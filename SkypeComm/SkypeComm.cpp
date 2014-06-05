/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |              http://sourceforge.net/p/openmora/home/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Emil Jatib Khatib  <emilkhatib@uma.es>                        |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/** @moos_module This module manages a Skype client to enable communication with a remote skype user    
  */

/** @moss_ToDo
  *  Adapt to work with Python 3 (PyString_AsString not found)
  */

#include "SkypeComm.h"
#include <string>
#include <fstream>
#include <mrpt/system.h>
#include <map>

SkypeComm::SkypeComm()
{
	std::cout << "NOTE: Do not forget to install the Skype4Py package in your default Python installation, otherwise this module will fail" << std::endl;
    // Constructor
    //remoteUser = REMOTE_USER_DEF;

    // Start embedded Python environment
    Py_Initialize();

    // Import the library and get a Skype object
    PyObject *libSkype;
    libSkype = PyImport_ImportModule("Skype4Py");   // import Skype4Py

    PyObject *GetSkypeObject;
    GetSkypeObject = Py_BuildValue("s","Skype");
    skype = PyObject_CallMethodObjArgs(libSkype, GetSkypeObject, NULL);     // skype = Skype4Py.Skype()

    // If skype is not started, start the client
        // first, get the client
    PyObject *client,*GetClient;
    GetClient = Py_BuildValue("s","Client");
    client = PyObject_GetAttr(skype,GetClient);     // client = skype.Client


	if (client == NULL)
		MOOSTrace("Error: could not obtain Client object\n");

        // check if skype is started
	/* COMMENTED FOR DEBUG: Better leave out, these lines are more problematic than helpful. They may (or may not) work fine under linux, but will fail under Windows for some reason. It is much easier to directly instruct the user to start Skype manually before running this program
    PyObject *isStarted, *GetStarted;
    long boolStarted;
    GetStarted = Py_BuildValue("s","IsRunning");
    isStarted = PyObject_GetAttr(client,GetStarted);    // isStarted = client.IsRunning
	    boolStarted = PyLong_AsLong(isStarted);
    if (boolStarted == 0) { // if not started, start skype
        PyObject *pyOs,*pyExeclp,*firstArg;
        pyOs = PyImport_ImportModule("subprocess");
        pyExeclp = Py_BuildValue("s","Popen");
        firstArg = Py_BuildValue("s","skype");
        PyObject_CallMethodObjArgs(pyOs, pyExeclp, firstArg, NULL);     // os.execlp('skype','')
		mrpt::system::sleep(1000);
        MOOSTrace("Waiting %i seconds for the skype client to start... ",CLIENT_START_WAIT);
		mrpt::system::sleep(CLIENT_START_WAIT*1000); // Sleep so the skype client is started
        MOOSTrace("Done\n");
    }
	*/
    // Attach to a running Skype client
    PyObject *Attach;
    Attach = Py_BuildValue("s","Attach");
    PyObject_CallMethodObjArgs(skype, Attach, NULL);  // skype.Attach()
	//mrpt::system::sleep(5000);

    // Set counter to last received message
	msgList = PyObject_CallMethod(skype, "Messages", NULL);  // msgList = skype.Messages()
    msgcount = 0;

    CheckNewMessage(); // this sets the counter to the last received message
	Py_DECREF(libSkype);
	Py_DECREF(GetSkypeObject);
	Py_DECREF(client);
	Py_DECREF(GetClient);
}

SkypeComm::~SkypeComm()
{
}

//-------------------------------------
// OnStartUp()
//-------------------------------------
bool SkypeComm::OnStartUp()
{
	// read the sentences from the file and add them to a sentence map (that matches each sentence with it's action)
	std::string sentencesFile;
	//std::map<std::string,std::string> sentences;
	char nameAction[10],*command,line[1024],*sentence,*allActions;
	int numAction;
	bool transitive;
	numAction = 0;
	taskLaunch = 0;
	sprintf(nameAction, "action%i",numAction);
	//std::cout << "Action " << nameAction << ":";
	//numAction++;
	while(m_MissionReader.GetConfigurationParam("pVoiceVerbio",nameAction,sentencesFile)){
		// once read the action definition, divide it into separate sentences
		strcpy(line,sentencesFile.c_str());
		//std::cout << std::string(line);
		command = strtok(line,"#");	// get the command
		transitive = !(strcmp(strtok(NULL,"#"),"T"));	// see if the action is transitive
		allActions = strtok(NULL,"#");

		sentence = strtok(allActions,",");
		while(sentence){
			std::cout << "\tSentence: " << sentence << std::endl;
			sentences.insert(std::pair<std::string, std::string>(std::string(sentence),std::string(command)));
			//sentences[std::string(sentence)] = std::string(command); // this matches each unique sentence with the command that it represents.
			if(transitive) transitiveActions.push_back(std::string(sentence));
			sentence = strtok(NULL,",");
		}
		numAction++;
		sprintf(nameAction, "action%i",numAction);
		//std::cout << "\nAction " << nameAction << ":";
		//m_MissionReader.GetConfigurationParam("pVoiceVerbio",nameAction,sentences);

	}

	std::string readVars;

	//!  @moos_param SubscribeTo Variables that will be available from the Skype chat
	m_MissionReader.GetConfigurationParam("SubscribeTo",readVars);
	char *var,configLine[1024];
	strcpy(configLine,readVars.c_str());
	var = strtok(configLine,",");
	while(var){
		sVariables.push_back(std::string(var));

		var = strtok(NULL,",");
	}

	// subscribe to all the variables
	AddVars();

	return DoRegistrations();
}


//-------------------------------------
// OnCommandMsg()
//-------------------------------------
bool SkypeComm::OnCommandMsg( CMOOSMsg Msg )
{
	return true;
}


//-------------------------------------
// Iterate()
//-------------------------------------
bool SkypeComm::Iterate()
{
    // Check for a new message. If it was received, then publish a MOOS variable
    if (CheckNewMessage()){
		ProcessReceivedMessage();
		PublishReceivedMessage();
    }
	return true;
}

//-------------------------------------
// CheckNewMessage(): Checks the message queue to see if there is a new message
//-------------------------------------

bool SkypeComm::CheckNewMessage()
{
    bool newMessage = false;
    PyObject *PopMsg=NULL,*IdCmd=NULL,*Id=NULL;
    int IdInt;
    // Get the last received message
    ///\todo: handle the case where there is no message. Could segfault otherwise
    PopMsg = PopMessage();

    if (PopMsg != NULL){
        // Check for the ID of the message
        IdCmd = Py_BuildValue("s","Id");
        Id = PyObject_GetAttr(PopMsg, IdCmd);   // id = popMsg.Id
        IdInt = PyLong_AsLong(Id);

        // If the ID of the message is greater than the saved ID, then the message is new
        if (IdInt>msgcount)
        {
            // save the ID of the last read message
            msgcount = IdInt;
            newMessage = true;
			//MOOSTrace("New message!\n");
        }
    }
	else{
		MOOSTrace("Error: could not retrieve last message\n");
	}
	Py_DECREF( IdCmd);
	Py_DECREF(  Id);
	Py_DECREF(  PopMsg);
    return newMessage;

}


//-------------------------------------
// GetReceivedMessage(): Get the body of the message
//-------------------------------------

std::string SkypeComm::GetReceivedMessage()
{
    PyObject *PopMsg,*BodyCmd,*Body,*UserCmd,*User;
    // Get the last received message
    ///\todo: handle the case where there is no message. Could segfault otherwise
    PopMsg = PopMessage();
    // Get the body of the message
    BodyCmd = Py_BuildValue("s","Body");
    Body = PyObject_GetAttr(PopMsg, BodyCmd);   // Body = popMsg.Body
    std::string msgString(PyString_AsString(Body));

    // Set the last received user, so any response is sent to him
    UserCmd = Py_BuildValue("s","FromHandle");
    User = PyObject_GetAttr(PopMsg, UserCmd);   // User = popMsg.User
    std::string userString(PyString_AsString(User));
    remoteUser = userString;
	Py_DECREF( PopMsg);
	Py_DECREF( BodyCmd);
	Py_DECREF( Body);
	Py_DECREF( UserCmd);
	Py_DECREF( User);
    return msgString;
}


//-------------------------------------
// GetReceivedMessage(): Publish the received message in a MOOS variable (defined as M_RECV in SkypeComm.h, by the moment)
//-------------------------------------

void SkypeComm::PublishReceivedMessage()
{
    // Get the message
    std::string msg;
    msg = GetReceivedMessage();
    // Publish it
    //MOOSTrace(msg);
	//!  @moos_publish <SKYPE_RECV> Raw text received from the skype chat
    m_Comms.Notify(M_RECV,msg);

	// Publish the user that the message is coming from (set in remoteUser)
	//!  @moos_publish <SKYPE_CONTACT> Username from who the last message was received
	m_Comms.Notify(M_RECV_USER,remoteUser);
}


//-------------------------------------
// TokenizeReceivedMessage(): Divide the received message in the expressions that make it up
//-------------------------------------

std::vector<std::string> TokenizeReceivedMessage(char message[1024])
{
	char *tk,*tk2,temp[1024];
	std::vector<std::string> tokens;


	// first, chack if the message is in the format CMD1 CMD2 ... "PARAM1 PARAM2 PARAM3 ..."
	tk = strtok(message,"\"");
	strcpy(temp,tk);
	// now tk contains the CMD1 CMD2 ... part
	tk2 = strtok(NULL, "\"");

	tk = strtok(temp," ");
	// now process the rest of the message (CMD1 CMD2 ... or the full message if the format doesn't match that pattern)
	while (tk){
		tokens.push_back(std::string(tk));
		tk = strtok(NULL," ");
	}
	if (tk2) { // If there is any "PARAM1 PARAM2 ... " section in the command
		tokens.push_back(std::string(tk2));
		std::cout << "ARGUMENTO: " << std::string(tk2) << std::endl;
	}
	for(size_t tki = 0;tki<tokens.size();tki++)
		std::cout << "TOKEN: " << tokens[tki] << std::endl;

	return tokens;
}



//-------------------------------------
// ProcessReceivedMessage(): Process the command contained in the received message
//-------------------------------------

void SkypeComm::ProcessReceivedMessage()
{
    // Get the message
    std::string msg;
    msg = GetReceivedMessage();
	// read the command
    char msg2[1024], *cmd;
    //std::string cmdStr;
    strcpy(msg2,msg.c_str());
    cmd = strtok(msg2," ");     // get the first token the parameters remain stored in strtok, in case they are needed
    //cmdStr = std::string(cmd);
    // process the command
    if (cmd && (strcmp(cmd,"PUBLISH") == 0))
    {
        // get the variable to be published
        char *variable, *parameter;
        variable = strtok(NULL," ");

        // get the values of the parameters
        char parameterBuffer[1024];
        bool first = true;
        strcpy(parameterBuffer,"");
            // take the parameters one by one and reassemble them in a string
        parameter = strtok(NULL," ");
        while (parameter){
            if (!first){ // to avoid adding a white space before the value
                strcat(parameterBuffer," ");
            }
			first = false;
            strcat(parameterBuffer,parameter);
            parameter = strtok(NULL," ");
        }

        // publish the variable
        m_Comms.Notify(std::string(variable),std::string(parameterBuffer));
    } else if (cmd && (strcmp(cmd,"GETVAL") == 0))
    {
        // get the variable to be published
        char *variable;
        std::string value;
		//CMOOSVariable *value;
        variable = strtok(NULL," ");

        // get the value of the variable from MOOS
        if (variable){
			// set the value of the requestedVar string, so next time a message is received, the value will be returned
			//varStr = std::string(variable);


            value = MOOSVars[std::string(variable)];
			//std::cout << std::string(variable) << " = " << value << std::endl;
			SendMessage(value);

/*
			value = GetMOOSVar(std::string(variable));
			if(value)
				SendMessage(value->GetName());// GetStringVal());*/
        }

    } else if (cmd && (strcmp(cmd,"HELP") == 0)){
        // return a message with a description of the basic commands
        // the help will be returned from a file
        char *command;
        //std::stringstream reply;
        command = strtok(NULL, " ");
        if (!command){
            // user typed HELP. Return a list with the basic commands
            SendMessage(std::string("HELP\nThis is the OpenMora Skype Module help.\nThe commands available through the Skype console are:\n\n\tPUBLISH variable value1\n\n\tGETVAL variable\nOther help topics:\n\n\tVIDEO\n\n\tFILE"));
        }
        else if (strcmp(command,"PUBLISH")){
            SendMessage(std::string("HELP\nPUBLISH:\n\tPUBLISH variable value\n\t\tThis command publishes a variable in the MOOS DB. The variable name is given by the first parameter, and the value by the second parameter."));
        }
        else if (strcmp(command,"VIDEO")){
            SendMessage(std::string("HELP\nVIDEO:\n\tTo see the view from the camera of the robot, simply make a video call. The Skype program on the robot must be configured so it autoaccepts incoming videocalls"));
        }
        else if (strcmp(command,"FILE")){
            SendMessage(std::string("HELP\nFILE:\n\tFile transfers can be done only in one direction: from the user to the robot"));
        }
        else if (strcmp(command,"GETVAL")){
            SendMessage(std::string("HELP\nGETVAL:\n\tGETVAL variable\n\t\tThis command shows the value of a variable in the MOOS DB. The variable to be read is passed as the argument"));
        } else {
            SendMessage(std::string("HELP\nNo help available for this command or command not available\nThe commands available through the Skype console are:\n\n\tPUBLISH variable value1"));
        }

	} else if (cmd){
		// Compare all the words with the saved sentences. If more than half of the words are present, the matching command will be executed.


		char message_[1024];
		int matches_found,sentence_length,chat_length;
		float match_idx, max_idx;
		bool transitive;
		size_t sentence_pos;
		bool found = false;
		std::map<std::string,std::string>::iterator sIter;
		std::string sentence,command,parameter;
		// tokenize the received message
		std::vector<std::string> tokens;
		strcpy(message_,msg.c_str());

		/*
		tk = strtok(message_," ");
		while (tk){
			tokens.push_back(std::string(tk));
			tk = strtok(NULL," ");
		}*/

		tokens = TokenizeReceivedMessage(message_);


		std::cout << std::endl;
		max_idx = 0;
		transitive = false;
		if (!tokens.empty())
			{
			// saved sentence loop: test each available sentence
			for (sIter = sentences.begin();(sIter!=sentences.end());sIter++){

				sentence = (*sIter).first;
				std::cout << "Sentence: " << sentence << std::endl;
				std::vector<std::string>::iterator wIter;
				matches_found = 0;
				sentence_length = 0;
				chat_length = 0;
				match_idx = 0;
				// word loop: find the word in the sentence. In found, increase counter
				for (wIter = tokens.begin();wIter != tokens.end(); wIter++){
					//std::cout << "\tToken: " << *wIter;
					if(sentence.find(*wIter) != std::string::npos){
						matches_found++;
						//std::cout << "\t" << matches_found;
					}
					//std::cout << std::endl;
					chat_length++;
				}
				// find the length of the original sentence
				sentence_pos = sentence.find(' ');
				while (sentence_pos != std::string::npos){
					sentence_length++;
					sentence_pos = sentence.find(' ',sentence_pos+1);
				}
				if (sentence_length == 0){
					sentence_length = 1;
				}
				// if the number of coincidences is bigger than half the length of the sentence, consider the received message and the sentence equal
				match_idx = ((float)matches_found / (float)sentence_length)*((float)matches_found/(float)chat_length);
				std::cout << "\tMatch index: " << match_idx << "\n\tMatches: " << matches_found << "\t Sentence length: " << sentence_length << "\tChat length: " << chat_length << "\n" << std::endl;

				if ((match_idx >= 0.5) && (match_idx > max_idx)&& !(std::find(transitiveActions.begin(), transitiveActions.end(), sentence) != transitiveActions.end())){
					// assign the command
					command = (*sIter).second;
					found = true;
					max_idx = match_idx;
					//std::cout << "FOUND MATCH!!!" << std::endl;

				}
			}
			std::cout << "Al finalizar los no transitivos, max_idx=" << max_idx << std::endl;


			// now repeat the search without the last word received from skype (in case it is a parameter of a transitive command)
			for (sIter = sentences.begin();(sIter!=sentences.end());sIter++){
				sentence = (*sIter).first;
				std::cout << "Sentence: " << sentence << std::endl;
				std::vector<std::string>::iterator wIter;
				matches_found = 0;
				sentence_length = 1;
				chat_length = 0;
				match_idx = 0;
				// word loop: find the word in the sentence. In found, increase counter

				for (wIter = tokens.begin();wIter != tokens.end()-1; wIter++){
					//std::cout << "\tToken: " << *wIter;
					if(sentence.find(*wIter) != std::string::npos){
						matches_found++;
						//std::cout << "\t" << matches_found;
					}
					//std::cout << std::endl;
					chat_length++;
				}

				// find the length of the original sentence
				sentence_pos = sentence.find(' ');
				while (sentence_pos != std::string::npos){
					sentence_length++;
					sentence_pos = sentence.find(' ',sentence_pos+1);
				}

				// if the number of coincidences is bigger than half the length of the sentence, consider the received message and the sentence equal
				match_idx = 0.5*(((float)matches_found / (float)sentence_length)+((float)matches_found/(float)chat_length));
				float max_idx_old = max_idx;

				if ((match_idx >= 0.5) && (match_idx > max_idx) && (std::find(transitiveActions.begin(), transitiveActions.end(), sentence) != transitiveActions.end())){
					// assign the command
					command = (*sIter).second;
					found = true;
					max_idx = match_idx;
					//std::cout << "FOUND MATCH!!!" << std::endl;
					transitive = true;
					parameter = *(tokens.end()-1);
					std::cout << "\tMatch index: " << match_idx << ">" << max_idx_old << "\n\tMatches: " << matches_found << "\t Sentence length: " << sentence_length << "\tChat length: " << chat_length << "\n" << std::endl;
				}
			}
		}
		if (transitive){
			command.append(" ");
			command.append(parameter);
		}
		if(found){
			std::cout << "Command found: " << command << std::endl;
			runCommand(command);
		}
	}
}


//-------------------------------------
// runCommand(): runs a command
//-------------------------------------
void SkypeComm::runCommand(std::string command)
{
	std::stringstream commandline;
	commandline << "SKYPECOMM " << taskLaunch << " " << command.c_str();
	
	//!  @moos_publish <NEW_TASK> Launch a task from the chat
	m_Comms.Notify("NEW_TASK", commandline.str().c_str());
	taskLaunch++;
}

//-------------------------------------
// openHelp(): opens a help file and returns its contents as a string
//-------------------------------------

std::string SkypeComm::openHelp(const char *filename){
    std::string line;
    std::stringstream output;
    std::ifstream helpFile (filename);
    if (helpFile.is_open())
    {
        while ( helpFile.good() )
        {
            getline (helpFile,line);
            output << line << "\n";
        }
        helpFile.close();
    }
    else printf("Unable to open help file");
    return output.str();
}



//-------------------------------------
// PopMessage(): Gets the last received message
//-------------------------------------

PyObject* SkypeComm::PopMessage()
{

    PyObject *PopMsgCmd,*PopMsg; // JL: Commented out, since there're not used: *MsgList, *RevListCmd,*RevList
	Py_DECREF(msgList);
    // Get the list of messages
    msgList = PyObject_CallMethod(skype, "Messages", NULL);  // msgList = skype.Messages()

	if (msgList == NULL)
		MOOSTrace("Error: Could not retrieve message list\n");

    // Pop the last message
    PopMsgCmd = Py_BuildValue("s","pop");
    PopMsg = PyObject_CallMethodObjArgs(msgList, PopMsgCmd, NULL);  // popMsg = msgList.pop()

	if (PopMsg == NULL)
		MOOSTrace("Error: Could not retrieve received message\n");


    // return the message (as PyObject*)

	Py_DECREF (PopMsgCmd);
    return PopMsg;
}

//-------------------------------------
// OnConnectToServer()
//-------------------------------------
bool SkypeComm::OnConnectToServer()
{
	DoRegistrations();
	return true;
}

//-------------------------------------
// DoRegistrations()
//-------------------------------------
bool SkypeComm::DoRegistrations()
{
	//! @moos_subscribe SKYPE_SEND 
	AddMOOSVariable( M_SEND, M_SEND, M_SEND, 0 );

	//! @moos_subscribe SKYPE_CONTACT
	AddMOOSVariable( M_SEND_USER, M_SEND_USER, M_SEND_USER, 0 );

	

	RegisterMOOSVariables();
	return true;
}

//-------------------------------------
// OnNewMail()
//-------------------------------------
bool SkypeComm::OnNewMail(MOOSMSG_LIST &NewMail)
{
	std::string cad;

	for(MOOSMSG_LIST::iterator i=NewMail.begin();i!=NewMail.end();++i)
	{
	//	MOOSTrace(format("New msg received: %s \n",i->GetKey()));
		MOOSVars[i->GetName()] = i->GetAsString();	// save the values in the map
		std::cout << i->GetName() << "=" << i->GetAsString() << std::endl;
		if (i->GetName() == M_SEND)
		{
			MOOSTrace("\n ----------------------------------------------------\n Someone wants to send a message through Skype.... \n ----------------------------------------------------\n");
			SendMessage(i->GetString());
		}
		if (i->GetName() == M_SEND_USER)
		{
			MOOSTrace("\n ----------------------------------------------------\n Someone changed the default Skype receiver.... \n ----------------------------------------------------\n");
			remoteUser = i->GetAsString();
		}


        if( (i->GetName()=="SHUTDOWN") && (MOOSStrCmp(i->GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}
	}

	//UpdateMOOSVariables(NewMail);
	return true;
}


//-------------------------------------
// SendMessage(): sends a string through Skype
//-------------------------------------
void SkypeComm::SendMessage(std::string message)
{

    PyObject *SendMsg,*Msg,*User,*ReturnVal,*ReturnVal2,*Id;


    // Send message
    SendMsg = Py_BuildValue("s","SendMessage");
    User = Py_BuildValue("s",remoteUser.c_str());         /// \todo: change this value. Either a global value containing the last user who contacted the account, or a variable passed to the function, containing the user
    Msg = Py_BuildValue("s",message.c_str());
    ReturnVal = PyObject_CallMethodObjArgs(skype, SendMsg, User, Msg, NULL);    // skype.SendMessage('username',message)

    // Get the ID of the message and save it
    Id = Py_BuildValue("s","Id");
    ReturnVal2 = PyObject_GetAttr(ReturnVal, Id);   // ReturnVal2 = ReturnVal.Id


    msgcount = PyLong_AsLong(ReturnVal2);
	Py_DECREF(SendMsg);
	Py_DECREF(Msg);
	Py_DECREF(User);
	Py_DECREF(ReturnVal);
	Py_DECREF(ReturnVal2);
	Py_DECREF(Id);

}

void SkypeComm::AddVars()
{
	// Add all the variables that the module is subscribed to according to the configuration file

	std::vector<std::string>::iterator vIter;
	std::cout << "AddVars()" << std::endl;
	for (vIter = sVariables.begin(); vIter != sVariables.end(); vIter++)
	{
		AddMOOSVariable(*vIter,*vIter,*vIter,0);
		std::cout << "Subscribed to: " << *vIter << std::endl;
	}

	RegisterMOOSVariables();
	//RegisterMOOSVariables_MAPIR();

}

