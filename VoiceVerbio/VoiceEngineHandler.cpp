/*--------------------------------*/
/*	VOICE ENGINE HANDLER OBJECTS  */
/*                                */
/*	ECM, 2006                     */
/*--------------------------------*/

/** \file VoiceEngineHandler.cpp
 *\brief Implementation of VoiceEngineHandler.hpp
 */

#include "VoiceEngineHandler.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>

#ifndef HIWORD
#define LOWORD(l) ((unsigned short)((unsigned long)(l) & 0xffff))
#define HIWORD(l) ((unsigned short)((unsigned long)(l) >> 16))
#endif

using namespace std;

VerbioHandler::VerbioHandler(float thresValue) :
	m_samplingFreq(0),
	m_asrObject(NULL),
	m_ttsObject(NULL),
	m_ASRResource(NULL),
	m_TTSResource(NULL)
#ifndef _WIN32
	,m_pulse_session_play(NULL),
	m_pulse_session_rec(NULL)
#endif
{
	myType = VoiceEngineHandler::VERBIO;
	confidenceThreshold = thresValue;
}


VerbioHandler::~VerbioHandler()
{
	DeinitializeAudioChannels();

	if(m_asrObject)
	{
		delete m_asrObject;
		m_asrObject = NULL;
	}

	if(m_ttsObject != NULL)
	{
		delete m_ttsObject;
		m_ttsObject = NULL;
	}

	/* Deletion of all the files associated with the active grammars */
	grammarsInfo::iterator grammarsIter; /* iterator to scan the list */
	grammarsIter = loadedGrammars.begin();

	while(grammarsIter != loadedGrammars.end())
	{
		string fileNameWithExt = (*grammarsIter).fileName + ".bnf";
		remove(fileNameWithExt.c_str());
		fileNameWithExt = (*grammarsIter).fileName + ".grm";
		remove(fileNameWithExt.c_str());
		fileNameWithExt = (*grammarsIter).fileName + ".trx";
		remove(fileNameWithExt.c_str());
		fileNameWithExt = (*grammarsIter).fileName + ".trc";
		remove(fileNameWithExt.c_str());
		grammarsIter++;
	}
}

VoiceEngineHandler::VoiceEngineType VerbioHandler::getType()
{
	return(myType);
}

bool VerbioHandler::Init()
{
	//VerbioASR * m_asrObject; /* Aux pointer to handle the ASR object */
	const VerbioASRLicense * licenseInfo; /* License information object */
	const VerbioTTSLicense * ttsLicenseInfo; /* Synthesis license information object */
	unsigned long versionInfo;	/* Version information */
	const char * serverIPset; /* Address of the voice server we want to connect to */
	const char * defaultLangset; /* Default language we want to set for the recognition */
	bool setError = false;		/* True if there is an error in the initialization of the server*/
	const char * serverIPget; /* Server IP version information */
	short availableConfsNum; /* Number of available configurations for the ASR server */
	short availableLangsNum; /* Number of available languages for the ASR server */
	int availableTTSResources; /* Number of TTS resources available for the default language */
	short availableSpksNum; /* Nunmber of available speakers for the TTS */
	VerbioSpeakerInfo * ttsSpeakerInfo; /* Information about the TTS speaker */
	const char * defaultSpk;	/* Default speaker for the TTS */
	const float * samplingFreq;	/* ASR sampling frequency */
	const char * currentLang; /* Current language in the available languages list */
	const char * defaultLangget; /* Default recognition language of the server */
	float ttssamplingFreq; /* Frequency of the tts sampling */

	/* Get the ASR object */

	printf("Getting ASR object\n");
	m_asrObject = getVerbioASR();



	/* Get the TTS object */
	printf("Getting TTS object\n");
	VerbioTTS * m_ttsObject = getVerbioTTS();


	/* Initialize the connection */
	serverIPset = "127.0.0.1";
	if((m_asrObject->SetDefaultServerIP(serverIPset) != 0) || (m_ttsObject->SetDefaultServerIP(serverIPset)))
	{
		printf("VerbioHandler: Unable to connect to the server at %s: Recognizer error : %s; Synthesizer error: %s\n",serverIPset,m_asrObject->GetError(),m_ttsObject->GetError());
		setError = true;
	}

	defaultLangset = VERBIO_LANG_SPANISH;
	defaultSpk = "carlos";

	if((m_asrObject->SetDefaultLanguage(defaultLangset) == -1) || (m_ttsObject->SetDefaultLanguage(defaultLangset) == -1))
	{
		printf("VerbioHandler: Unable to set %s as default language: Recognizer error: %s; Synthesizer error: %s\n",defaultLangset,m_asrObject->GetError(),m_ttsObject->GetError());
		setError = true;
	}

	if(!setError)
	{
	//	if((m_asrObject->Open() != 0) || (m_ttsObject->Open() != 0))
			if(m_ttsObject->Open() != 0)
		{
			printf("VerbioHandler: Unable to open a connection with the server at IP %s with default languge %s. Recognizer error: %s; Synthesizer error: %s\n",serverIPset,defaultLangset,m_asrObject->GetError(),m_ttsObject->GetError());
			return(false);
		}
		else
		{
			/* Get the information about the product */
			licenseInfo = m_asrObject->GetLicenseInfo();
			ttsLicenseInfo = m_ttsObject->GetLicenseInfo();
			if (!ttsLicenseInfo)
			{
				printf ("ERROR while getting Verbio License\n");
			}
/*			printf("*---------------------------------------------------------*\n");
			printf("*--     VERBIO HANDLER OBJECT GENERAL INFORMATION	--*\n");
			printf("*---------------------------------------------------------*\n");
			printf("* RECOGNITION ENGINE:\n");
			printf("\n - Licensing information:\n");
			if(ttsLicenseInfo->IsEvaluation())
			{
				printf("VerbioHandler: The available license for recognition is for evaluation.\n");
			}
			else
			{
				printf("VerbioHandler: The available license for recognition is definitive.\n");
			}

			if(ttsLicenseInfo->IsLite())
			{
				printf("VerbioHandler: The recongnizer has limited capabilities.\n");
			}
			else
			{
				printf("VerbioHandler: The recognizer has full capabilities.\n");
			}

		*/
		/*	printf("\n - Version information:\n");

			if(m_asrObject->GetVersion(&versionInfo) != 0)
			{
				printf("VerbioHandler: Unable to recover the version information: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Version %d.%02d\n",HIWORD(versionInfo),LOWORD(versionInfo));
			}*/

		//	printf("\n - Server IP information:\n");
/*			serverIPget = m_asrObject->GetServerIP();
			if( serverIPget == NULL)
			{
				printf("VerbioHandler: Unable to recover server IP information: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Server IP: %s\n",serverIPget);
			}
*/
			printf("\n - Available configurations:\n");
			availableConfsNum = m_asrObject->GetNumberOfAvailableConfs();
			if(availableConfsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available configurations: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available configurations:\n");
				for(short i = 0; i < availableConfsNum;i++)
				{
					printf("- %s\n",m_asrObject->GetConfiguration(i));
				}
			}

			printf("\n - Available languages:\n");
			availableLangsNum = m_asrObject->GetNumberOfAvailableLngs();
			if(availableLangsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available languages for the recognizer: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available languages:\n");
				defaultLangget = m_asrObject->GetDefaultLanguage();
				for(short j = 0; j < availableLangsNum;j++)
				{
					currentLang = m_asrObject->GetLanguage(j);
					if(!strcmp(defaultLangget, currentLang))
					{
						printf("- %s -> SET AS DEFAULT LANGUAGE\n",currentLang);
					}
					else
					{
						printf("- %s\n",currentLang);
					}
				}
			}

			printf("\n - Sampling frequency:\n");
			samplingFreq = m_asrObject->GetDefaultSamplingFrequency();
			if(samplingFreq == NULL)
			{
				printf("VerbioHandler: Unable to recover the sampling frequency information: %s\n",m_asrObject->GetError());
			}
			else
			{
				m_samplingFreq = *samplingFreq;
				printf("VerbioHandler: Sampling frequency value: %.2f (samples/second)\n", *samplingFreq);
			}

			//Synthesis engine information
			printf("\n\n* SYNTHESIS ENGINE:\n");
			printf("\n - Licensing information:\n");
			availableTTSResources = ttsLicenseInfo->GetAvailableResources(defaultLangset);
			printf("VerbioHandler: There are %u available licenses for voice synthesis\n",availableTTSResources);
			if(ttsLicenseInfo->IsEvaluation())
			{
				printf("VerbioHandler: The available license for synthesis is for evaluation.\n");
			}
			else
			{
				printf("VerbioHandler: The available license for synthesis is definitive.\n");
			}

			if(ttsLicenseInfo->IsLite())
			{
				printf("VerbioHandler: The synthesizer has limited capabilities (lower quality speakers).\n");
			}
			else
			{
				printf("VerbioHandler: The synthesizer has full capabilities.\n");
			}

			printf("\n - Version information:\n");
			if(m_ttsObject->GetVersion(&versionInfo) != 0)
			{
				printf("VerbioHandler: Unable to recover the version information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Version %d.%02d\n",HIWORD(versionInfo),LOWORD(versionInfo));
			}

			printf("\n - Server IP information:\n");
			serverIPget = m_ttsObject->GetServerIP();
			if( serverIPget == NULL)
			{
				printf("VerbioHandler: Unable to recover server IP information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Server IP: %s\n",serverIPget);
			}

			printf("\n - Available languages:\n");
			availableLangsNum = m_ttsObject->GetNumberOfAvailableLngs();
			if(availableLangsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available languages for the synthesizer: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available languages:\n");
				defaultLangget = m_ttsObject->GetDefaultLanguage();
				for(short j = 0; j < availableLangsNum;j++)
				{
					currentLang = m_ttsObject->GetLanguage(j);
					if(!strcmp(defaultLangget, currentLang))
					{
						printf("- %s -> SET AS DEFAULT LANGUAGE\n",currentLang);
					}
					else
					{
						printf("- %s\n",currentLang);
					}
				}
			}

			printf("\n - Available speakers information:\n");
			availableSpksNum = m_ttsObject->GetNumberOfAvailableSpks();
			for(short k = 0; k < availableSpksNum;k++)
			{
				ttsSpeakerInfo = m_ttsObject->GetSpeakerInfo(k);
				printf("- Speaker id : %s; Name: %s; Gender: %s; Age: %s, Language: %s",ttsSpeakerInfo->GetIdent(),ttsSpeakerInfo->GetName(),ttsSpeakerInfo->GetGender(),ttsSpeakerInfo->GetAge(),ttsSpeakerInfo->GetLanguage());
				if(!strcmp(defaultSpk,ttsSpeakerInfo->GetName()))
				{
					printf(" -> WILL BE SET AS DEFAULT SPEAKER");
				}
				printf("\n");
			}

			printf("\n - Sampling frequency:\n");
			ttssamplingFreq = m_ttsObject->GetSamplingFrequency();
			if(ttssamplingFreq == 0)
			{
				printf("VerbioHandler: Unable to recover the sampling frequency information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Sampling frequency value: %.2f (samples/second)\n", ttssamplingFreq);
			}
		}

		/* Get valid Verbio Resources */
		m_ASRResource = m_asrObject->GetVerbioASRResource();
		if(m_ASRResource == NULL)
		{
			printf("VerbioHandler: Error, the associated ASR resource for the recognizer could not be obtained: %s\n",m_asrObject->GetError());
			return(false);
		}
		else
		{
		 m_TTSResource = m_ttsObject->GetVerbioTTSResource();
			if(	m_TTSResource == NULL)
			{
				printf("VerbioHandler: Error, the associated TTS resource for the synthesizer could not be obtained: %s\n",m_ttsObject->GetError());
				return(false);
			}
			else
			{
				printf("VerbioHandler: Confidence threshold value for the recognition results: %2.2f\n",confidenceThreshold);

				if(!InitializeAudioChannels())
				{
					printf("VerbioHandler: Error, the sound card could not be assigned as audio device for the voice engine: Recognizer error:%s; Synthesizer error:%s\n",m_ASRResource->GetError(),m_TTSResource->GetError());
					return(false);
				}
				else
				{
					/* Set the number of hypothesis available for the recognizer (1 as no verification process
					 *is available) */
				/*if(m_ASRResource->SetNBest(1))
					{
						printf("VerbioHandler: Error, the number of hypotheis could not been set: %s\n",m_ASRResource->GetError());
						return(false);
					}
					*/
					// Set the name of the speaker */



					if(m_TTSResource->SetSpeakerName(defaultSpk))
					{
						printf("VerbioHandler: Error, unable to set %s as default speaker: %s\n",defaultSpk,m_TTSResource->GetError());
						return(false);
					}
				}
			}
		}

		
		int vol=m_TTSResource->GetVolume();
		m_TTSResource->SetVolume(vol*2);
		ReproduceString("Sistema de voz inicializado");
		vol=m_TTSResource->GetVolume();
		printf("Volumen is %d\n",vol);

		
		return(true);
	}
	else
	{
		return(false);
	}
}

bool VerbioHandler::Init(string grammar2LoadName, string grammar2Load)
{
	const VerbioASRLicense * licenseInfo; /* License information object */
	const VerbioTTSLicense * ttsLicenseInfo; /* Synthesis license information object */
	unsigned long versionInfo;	/* Version information */
	const char * serverIPset; /* Address of the voice server we want to connect to */
	const char * defaultLangset; /* Default language we want to set for the recognition */
	bool setError = false;		/* True if there is an error in the initialization of the server*/
	const char * serverIPget; /* Server IP version information */
	short availableConfsNum; /* Number of available configurations for the ASR server */
	short availableLangsNum; /* Number of available languages for the ASR server */
	int availableTTSResources; /* Number of TTS resources available for the default language */
	short availableSpksNum; /* Nunmber of available speakers for the TTS */
	VerbioSpeakerInfo * ttsSpeakerInfo; /* Information about the TTS speaker */
	const char * defaultSpk;	/* Default speaker for the TTS */
	//const float * samplingFreq;	/* ASR sampling frequency */
	const char * currentLang; /* Current language in the available languages list */
	const char * defaultLangget; /* Default recognition language of the server */
	float ttssamplingFreq; /* Frequency of the tts sampling */


	/* Get the ASR object */
	m_asrObject = getVerbioASR();

	/* Get the TTS object */
	m_ttsObject = getVerbioTTS();

	/* Initialize the connection */
	serverIPset = "127.0.0.1";
	if((m_asrObject->SetDefaultServerIP(serverIPset) != 0) || (m_ttsObject->SetDefaultServerIP(serverIPset)))
	{
		printf("VerbioHandler: Unable to connect to the server at %s: Recognizer error : %s; Synthesizer error: %s\n",serverIPset,m_asrObject->GetError(),m_ttsObject->GetError());
		setError = true;
	}

	defaultLangset = VERBIO_LANG_SPANISH;
	defaultSpk = "carlos";

	if((m_asrObject->SetDefaultLanguage(defaultLangset) == -1) || (m_ttsObject->SetDefaultLanguage(defaultLangset) == -1))
	{
		printf("VerbioHandler: Unable to set %s as default language: Recognizer error: %s; Synthesizer error: %s\n",defaultLangset,m_asrObject->GetError(),m_ttsObject->GetError());
		setError = true;
	}

	if(!setError)
	{
		if((m_asrObject->Open() != 0) || (m_ttsObject->Open() != 0))
		{
			printf("VerbioHandler: Unable to open a connection with the server at IP %s with default languge %s. Recognizer error: %s; Synthesizer error: %s\n",serverIPset,defaultLangset,m_asrObject->GetError(),m_ttsObject->GetError());
			return(false);
		}
		else
		{
			/* Get the information about the product */
			licenseInfo = m_asrObject->GetLicenseInfo();
			ttsLicenseInfo = m_ttsObject->GetLicenseInfo();
			printf("*---------------------------------------------------------*\n");
			printf("*--     VERBIO HANDLER OBJECT GENERAL INFORMATION	--*\n");
			printf("*---------------------------------------------------------*\n");
			printf("* RECOGNITION ENGINE:\n");
			printf("\n - Licensing information:\n");
			if(licenseInfo->IsEvaluation())
			{
				printf("VerbioHandler: The available license for recognition is for evaluation.\n");
			}
			else
			{
				printf("VerbioHandler: The available license for recognition is definitive.\n");
			}

			if(licenseInfo->IsLite())
			{
				printf("VerbioHandler: The recongnizer has limited capabilities.\n");
			}
			else
			{
				printf("VerbioHandler: The recognizer has full capabilities.\n");
			}


			printf("\n - Version information:\n");
			if(m_asrObject->GetVersion(&versionInfo) != 0)
			{
				printf("VerbioHandler: Unable to recover the version information: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Version %d.%02d\n",HIWORD(versionInfo),LOWORD(versionInfo));
			}

			printf("\n - Server IP information:\n");
			serverIPget = m_asrObject->GetServerIP();
			if( serverIPget == NULL)
			{
				printf("VerbioHandler: Unable to recover server IP information: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Server IP: %s\n",serverIPget);
			}

			printf("\n - Available configurations:\n");
			availableConfsNum = m_asrObject->GetNumberOfAvailableConfs();
			if(availableConfsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available configurations: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available configurations:\n");
				for(short i = 0; i < availableConfsNum;i++)
				{
					printf("- %s\n",m_asrObject->GetConfiguration(i));
				}
			}

			printf("\n - Available languages:\n");
			availableLangsNum = m_asrObject->GetNumberOfAvailableLngs();
			if(availableLangsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available languages for the recognizer: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available languages:\n");
				defaultLangget = m_asrObject->GetDefaultLanguage();
				for(short j = 0; j < availableLangsNum;j++)
				{
					currentLang = m_asrObject->GetLanguage(j);
					if(!strcmp(defaultLangget, currentLang))
					{
						printf("- %s -> SET AS DEFAULT LANGUAGE\n",currentLang);
					}
					else
					{
						printf("- %s\n",currentLang);
					}
				}
			}

		/*	printf("\n - Sampling frequency:\n");
			samplingFreq = m_asrObject->GetDefaultSamplingFrequency();
			if(samplingFreq == NULL)
			{
				printf("VerbioHandler: Unable to recover the sampling frequency information: %s\n",m_asrObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Sampling frequency value: %.2f (samples/second)\n", *samplingFreq);
			}
*/
			//Synthesis engine information
			printf("\n\n* SYNTHESIS ENGINE:\n");
			printf("\n - Licensing information:\n");
			availableTTSResources = ttsLicenseInfo->GetAvailableResources(defaultLangset);
			printf("VerbioHandler: There are %u available licenses for voice synthesis\n",availableTTSResources);
			if(ttsLicenseInfo->IsEvaluation())
			{
				printf("VerbioHandler: The available license for synthesis is for evaluation.\n");
			}
			else
			{
				printf("VerbioHandler: The available license for synthesis is definitive.\n");
			}

			if(ttsLicenseInfo->IsLite())
			{
				printf("VerbioHandler: The synthesizer has limited capabilities (lower quality speakers).\n");
			}
			else
			{
				printf("VerbioHandler: The synthesizer has full capabilities.\n");
			}

			printf("\n - Version information:\n");
			if(m_ttsObject->GetVersion(&versionInfo) != 0)
			{
				printf("VerbioHandler: Unable to recover the version information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Version %d.%02d\n",HIWORD(versionInfo),LOWORD(versionInfo));
			}

			printf("\n - Server IP information:\n");
			serverIPget = m_ttsObject->GetServerIP();
			if( serverIPget == NULL)
			{
				printf("VerbioHandler: Unable to recover server IP information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Server IP: %s\n",serverIPget);
			}

			printf("\n - Available languages:\n");
			availableLangsNum = m_ttsObject->GetNumberOfAvailableLngs();
			
			if(availableLangsNum == -1)
			{
				printf("VerbioHandler: Unable to retrieve the number of available languages for the synthesizer: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Available languages:\n");
				defaultLangget = m_ttsObject->GetDefaultLanguage();
				for(short j = 0; j < availableLangsNum;j++)
				{
					currentLang = m_ttsObject->GetLanguage(j);
					if(!strcmp(defaultLangget, currentLang))
					{
						printf("- %s -> SET AS DEFAULT LANGUAGE\n",currentLang);
					}
					else
					{
						printf("- %s\n",currentLang);
					}
				}
			}

			printf("\n - Available speakers information:\n");
			availableSpksNum = m_ttsObject->GetNumberOfAvailableSpks();
			for(short k = 0; k < availableSpksNum;k++)
			{
				ttsSpeakerInfo = m_ttsObject->GetSpeakerInfo(k);
				printf("- Speaker id : %s; Name: %s; Gender: %s; Age: %s, Language: %s",ttsSpeakerInfo->GetIdent(),ttsSpeakerInfo->GetName(),ttsSpeakerInfo->GetGender(),ttsSpeakerInfo->GetAge(),ttsSpeakerInfo->GetLanguage());
				if(!strcmp(defaultSpk,ttsSpeakerInfo->GetName()))
				{
					printf(" -> WILL BE SET AS DEFAULT SPEAKER");
				}
				printf("\n");
			}

			printf("\n - Sampling frequency:\n");
			ttssamplingFreq = m_ttsObject->GetSamplingFrequency();
			if(ttssamplingFreq == 0)
			{
				printf("VerbioHandler: Unable to recover the sampling frequency information: %s\n",m_ttsObject->GetError());
			}
			else
			{
				printf("VerbioHandler: Sampling frequency value: %.2f (samples/second)\n", ttssamplingFreq);
			}
		}

		/* Get valid Verbio Resources */
		m_ASRResource = m_asrObject->GetVerbioASRResource();
		if(m_ASRResource == NULL)
		{
			printf("VerbioHandler: Error, the associated ASR resource for the recognizer could not be obtained: %s\n",m_asrObject->GetError());
			return(false);
		}
		else
		{
			m_TTSResource = m_ttsObject->GetVerbioTTSResource();
			
			if(	m_TTSResource == NULL)
			{
				printf("VerbioHandler: Error, the associated TTS resource for the synthesizer could not be obtained: %s\n",m_ttsObject->GetError());
				return(false);
			}
			else
			{
				printf("VerbioHandler: Confidence threshold value for the recognition results: %2.2f\n",confidenceThreshold);

				if(!InitializeAudioChannels())
				{
					printf("VerbioHandler: Error, the sound card could not be assigned as audio device for the voice engine: Recognizer error:%s; Synthesizer error:%s\n",m_ASRResource->GetError(),m_TTSResource->GetError());
					return(false);
				}
				else
				{
					/* Set the number of hypothesis available for the recognizer (1 as no verification process
					 *is available) */
					if(m_ASRResource->SetNBest(1))
					{
						printf("VerbioHandler: Error, the number of hypotheis could not been set: %s\n",m_ASRResource->GetError());
						return(false);
					}
					else // Loading of the grammar
					{
						if(!LoadGrammar(grammar2LoadName,grammar2Load))
						{
							return(false);
						}
						else
						{
							if(!ActivateGrammar(grammar2LoadName))
							{
								return(false);
							}
							else
							{
								/* Set the name of the speaker */
								if(m_TTSResource->SetSpeakerName(defaultSpk))
								{
									printf("VerbioHandler: Error, unable to set %s as default speaker:  %s\n",defaultSpk,m_TTSResource->GetError());
									return(false);
								}
							}
						}
					}
					/*if(m_TTSResource->SetSpeakerName(defaultSpk))
								{
									printf("VerbioHandler: Error, unable to set %s as default speaker\n",defaultSpk,m_TTSResource->GetError());
									return(false);
								}
								*/
				}
			}
		}
		int vol=m_TTSResource->GetVolume();
		m_TTSResource->SetVolume(vol*2);
		ReproduceString("Sistema de voz inicializado");
		vol=m_TTSResource->GetVolume();
		printf("Volumen is %d\n",vol);
		return(true);
	}
	else
	{
		return(false);
	}
}

void VerbioHandler::Stop()
{
	m_asrObject->Close();
	m_ttsObject->Close();
}

bool VerbioHandler::LoadGrammar(string grammarName,string grammarStr)
{
	string filePrefix; /* Prefix of all the files associated to a grammar */
	string fileName; /* elements to store the grammar string into a file */
	ofstream outputFile;
	int grammarId;	/* Identifier for the grammar */
	grammarAssociation grammarInfo;

	/* Check if there is yet another loaded grammar with the same name */
	if(GetGrammarIdByName(grammarName) != -1)
	{
		printf("VerbioHandler: Sorry, there is another grammar loaded with the same name.\n");
		return(false);
	}

	/* Store the Grammar String into a file */
	filePrefix = "VERBIO_" + grammarName;
	fileName = filePrefix + ".bnf";

	outputFile.open(fileName.c_str(),ofstream::out);
	if(outputFile.fail())
	{
		printf("VerbioHandler: The file associated with the grammar could not be created. The Grammar will not be loaded\n");
		return(false);
	}
	outputFile << grammarStr.c_str();
	outputFile.close();

	/* Prepare the grammar file so it can be loaded by the recognizer */
	int errorLine; /* line where is the error (if there is any)*/
	if(m_asrObject->PrepareGrammar(fileName.c_str(),VERBIO_GRAMMAR_ABNF, &errorLine)==-1)
	{
		printf("VerbioHandler: Error preparing the grammar in line %d: %s. The file will be left undeleted so the user can check the line.\n",errorLine,m_asrObject->GetError());
		return(false);
	}

	/* Load the Grammar into the Recognizer */
	grammarId = m_ASRResource->LoadGrammar(fileName.c_str(),VERBIO_GRAMMAR_ABNF);
	if(grammarId == -1)
	{
		//Delete the grammar file:
		remove(fileName.c_str());
		printf("VerbioHandler: There has been an error loading the grammar file: %s\n",m_ASRResource->GetError());
		return(false);
	}
	else /* Insert the new element into the loaded grammars list */
	{
		grammarInfo.name = grammarName;
		grammarInfo.fileName = filePrefix;
		grammarInfo.id = grammarId;
		loadedGrammars.push_back(grammarInfo);
	}

	return(true);
}

bool VerbioHandler::FreeGrammar(string grammarName)
{
	int grammarId = GetGrammarIdByName(grammarName);
	if(grammarId != -1)
	{
		if(m_ASRResource->FreeGrammar(grammarId)==0)
		{
			//Delete the associated files and remove the input from the grammars list
			/* Deletion of all the files associated with the active grammars */
			grammarsInfo::iterator grammarsIter; /* iterator to scan the list */
			grammarsIter = loadedGrammars.begin();
			while(grammarsIter != loadedGrammars.end())
			{
				if((*grammarsIter).name == grammarName)
				{ //We have found the grammar entry
					string fileNameWithExt = (*grammarsIter).fileName + ".bnf";
					remove(fileNameWithExt.c_str());
					fileNameWithExt = (*grammarsIter).fileName + ".grm";
					remove(fileNameWithExt.c_str());
					fileNameWithExt = (*grammarsIter).fileName + ".trx";
					remove(fileNameWithExt.c_str());
					fileNameWithExt = (*grammarsIter).fileName + ".trc";
					remove(fileNameWithExt.c_str());
					loadedGrammars.erase(grammarsIter);
					return(true);
				}
				grammarsIter++;
			}
		}
		else
		{
			printf("VerbioHandler: There has been an error freeing the resources associated to the grammar named %s: %s\n",grammarName.c_str(),m_ASRResource->GetError());
		}
	}
	return(false);
}

int VerbioHandler::GetGrammarIdByName(string grammarName)
{
	grammarsInfo::iterator grammarsIter;
	grammarsIter = loadedGrammars.begin();
	while(grammarsIter != loadedGrammars.end())
	{
		if(grammarName == (*grammarsIter).name)
		{
			return((*grammarsIter).id);

		}
		grammarsIter++;
	}
	return(-1);
}

bool VerbioHandler::ActivateGrammar(string grammarName)
{
	int grammarId = GetGrammarIdByName(grammarName);
	if(grammarId != -1)
	{
		if(m_ASRResource->ActivateGrammar(grammarId) == 0)
		{
			return(true);
		}
		else
		{
			printf("VerbioHandler: There has been an error activating the grammar named %s: %s\n",grammarName.c_str(),m_ASRResource->GetError());
		}
	}
	return(false);
}

bool VerbioHandler::DeactivateGrammar(string grammarName)
{
	int grammarId = GetGrammarIdByName(grammarName);
	if(grammarId != -1)
	{
		if(m_ASRResource->DeactivateGrammar(grammarId) == 0)
		{
			return(true);
		}
		else
		{
			printf("VerbioHandler: There has been an error deactivating the grammar named %s: %s\n",grammarName.c_str(),m_ASRResource->GetError());
		}
	}
	return(false);
}

bool VerbioHandler::Wait4License(int time2Wait)
{
	if(m_ASRResource == NULL)
	{
		return(false);
	}

	if(m_ASRResource->WaitForLicense(time2Wait) == 0)
	{
		return(true);
	}
	else
	{
		printf("There is no available license after %u ms: %s\n",time2Wait,m_ASRResource->GetError());
		return(false);
	}
}

VerbioHandler::RecognitionStatus VerbioHandler::RecognitionProcess(list<string>& rulesInfo)
{
	VerbioResult * recognitionRes; /* Result of the recognition process */
	int numberOfRules;				/* Number of recognized rules for a result */
	int numberOfUnits;				/* Number of units for a recognition result */
	string ruleName;				/* Name of the rule associated with the recognized result */
	string hitName;					/* Name of the recognized rule */
	bool isNoise;					/* true if no recognized slot has enough confidence */
	list<string> listOfResults;		/* List to store the resulting units for the recognition */

	//Definition of the times for the recognition (milliseconds):
	int maxProcessTime = 5000;//10000;
	int maxInitSilence = 100;//5000;
	int maxFinalSilence = 50;//1000;

	if(RecStr(maxProcessTime, maxInitSilence, maxFinalSilence))
	{
		if((recognitionRes = m_ASRResource->GetResult(10)))
		{
			//printf("*-------------------------*\n");
			//printf("* RECOGNITION INFORMATION *\n\n");
			//Get the result information
			numberOfRules = recognitionRes->GetNumberOfSlots();
			isNoise = true;
			for(int i = 0;i < numberOfRules;i++)
			{
				ruleName = recognitionRes->GetName(i);
				if(recognitionRes->GetConfidence(ruleName.c_str()) > confidenceThreshold)
				{
					isNoise = false;
					//printf("- Recognized rule:\nName: %s; ",ruleName.c_str());
					//printf("Value: %s\n",recognitionRes->GetValue(ruleName.c_str()));
					printf(" Confidence level: %.2f\n",recognitionRes->GetConfidence(ruleName.c_str()));
				}
			}

			if(!isNoise)
			{
				//Units information
				numberOfUnits = recognitionRes->GetNumberOfUnits();
				//printf("- Number of units of the recognized pattern: %u\n",numberOfUnits);
				hitName = recognitionRes->GetName(0);
				//ReproduceString("Etiqueta de la regla reconocida: "+hitName); //ECM: prueba temporal
				//string temp = recognitionRes->GetValue(hitName.c_str());
				//ReproduceString("Valor de la regla: "+temp);
				listOfResults.push_back(hitName);
				for(int j = 0;j < numberOfUnits;j++)
				{
					if(hitName != recognitionRes->GetRule(j))
					{
						listOfResults.push_back(recognitionRes->GetRule(j));
					}
				}
				sortVerbioListOfResults(listOfResults);
				//Get the antecedent of each rule:
				list<string>::iterator resultListIter;
				string::size_type position;
				resultListIter = listOfResults.begin();
				rulesInfo.clear(); //Clear all the previous elements in rulesInfo
				while(resultListIter != listOfResults.end())
				{
					position = (*resultListIter).find_last_of('.');
					if( position != string::npos)
					{
						rulesInfo.push_back((*resultListIter).substr(position+1));
					}
					else
					{
						rulesInfo.push_back((*resultListIter));
					}
					resultListIter++;
				}
				//printf("*-------------------------*\n");
				return(REC_OK);
			}

			if(isNoise)
			{
				//printf("-NOISE-\n");
				//printf("*-------------------------*\n");
			}

		}
		else
		{
			if(strcmp(m_ASRResource->GetError(),"NOVOICE") == 0)
			{
				return(NO_INPUT);
			}
			else
			{
				printf("VerbioHandler: Error in the recognition process: %s\n",m_ASRResource->GetError());
			}
		}
	}
	return(REC_ERROR);
}

VerbioHandler::RecognitionStatus VerbioHandler::RecognitionProcess(list<string>& rulesInfo,int maxProcessTime,int maxInitSilence,int maxFinalSilence)
{
	VerbioResult * recognitionRes; /* Result of the recognition process */
	int numberOfRules;				/* Number of recognized rules for a result */
	int numberOfUnits;				/* Number of units for a recognition result */
	string ruleName;				/* Name of the rule associated with the recognized result */
	string hitName;					/* Name of the recognized rule */
	bool isNoise;					/* true if no recognized slot has enough confidence */


	list<string> listOfResults;		/* List to store the resulting units for the recognition */

	if(RecStr(maxProcessTime, maxInitSilence, maxFinalSilence))
	{
		if((recognitionRes = m_ASRResource->GetResult(10)))
		{
			//printf("*-------------------------*\n");
			//printf("* RECOGNITION INFORMATION *\n\n");
			//Get the result information
			numberOfRules = recognitionRes->GetNumberOfSlots();
			isNoise = true;
			for(int i = 0;i < numberOfRules;i++)
			{
				ruleName = recognitionRes->GetName(i);
				if(recognitionRes->GetConfidence(ruleName.c_str()) > confidenceThreshold)
				{
					isNoise = false;
					//printf("- Recognized rule:\nName: %s; ",ruleName.c_str());
					//printf("Value: %s\n",recognitionRes->GetValue(ruleName.c_str()));
					printf(" Confidence level: %.2f\n",recognitionRes->GetConfidence(ruleName.c_str()));
				}
			}

			if(!isNoise)
			{
				//Units information
				numberOfUnits = recognitionRes->GetNumberOfUnits();
				//printf("- Number of units of the recognized pattern: %u\n",numberOfUnits);
				hitName = recognitionRes->GetName(0);
				//ReproduceString("Etiqueta de la regla reconocida: "+hitName); //ECM: prueba temporal
				//string temp = recognitionRes->GetValue(hitName.c_str());
				//ReproduceString("Valor de la regla: "+temp);
				listOfResults.push_back(hitName);
				for(int j = 0;j < numberOfUnits;j++)
				{
					if(hitName != recognitionRes->GetRule(j))
					{
						listOfResults.push_back(recognitionRes->GetRule(j));
					}
				}
				sortVerbioListOfResults(listOfResults);
				//Get the antecedent of each rule:
				list<string>::iterator resultListIter;
				string::size_type position;
				resultListIter = listOfResults.begin();
				rulesInfo.clear(); //Clear all the previous elements in rulesInfo
				while(resultListIter != listOfResults.end())
				{
					position = (*resultListIter).find_last_of('.');
					if( position != string::npos)
					{
						rulesInfo.push_back((*resultListIter).substr(position+1));
					}
					else
					{
						rulesInfo.push_back((*resultListIter));
					}
					resultListIter++;
				}
				//printf("*-------------------------*\n");
				return(REC_OK);
			}

			if(isNoise)
			{
				//printf("-NOISE-\n");
				//printf("*-------------------------*\n");
			}

		}
		else
		{
			if(strcmp(m_ASRResource->GetError(),"NOVOICE") == 0)
			{
				return(NO_INPUT);
			}
			else
			{
				printf("VerbioHandler: Error in the recognition process: %s\n",m_ASRResource->GetError());
			}
		}
	}
	return(REC_ERROR);
}

bool VerbioHandler::PauseRecognitionProcess(void)
{
	list<grammarAssociation>::iterator grammarsIter;
	bool retVal = true;

	grammarsIter = loadedGrammars.begin();
	while(grammarsIter != loadedGrammars.end())
	{
		if(m_ASRResource->DeactivateGrammar((* grammarsIter).id) < 0)
		{
			printf("VerbioHandler: Error deactivating grammar named %s: %s\n",(* grammarsIter).name.c_str(),m_ASRResource->GetError());
			retVal = false;
		}
		grammarsIter++;
	}
	return(retVal);

}

bool VerbioHandler::RestoreRecognitionProcess(void)
{
	list<grammarAssociation>::iterator grammarsIter;
	bool retVal = true;

	grammarsIter = loadedGrammars.begin();
	while(grammarsIter != loadedGrammars.end())
	{
		if(m_ASRResource->ActivateGrammar((* grammarsIter).id) < 0)
		{
			printf("VerbioHandler: Error activating grammar named %s: %s\n",(* grammarsIter).name.c_str(),m_ASRResource->GetError());
			retVal = false;
		}
		grammarsIter++;
	}
	return(retVal);
}


void VerbioHandler::SetConfidenceThreshold(float thresValue)
{
	confidenceThreshold = thresValue;
}

float VerbioHandler::GetConfidenceThreshold()
{
	return(confidenceThreshold);
}

unsigned VerbioHandler::getNumberOfRepetitions(std::string str2Work,char pattern)
{
	unsigned repNum = 0; //Number of repetitions
	std::string::iterator strIter; //Iterator to scan the string
	strIter = str2Work.begin();
	while(strIter != str2Work.end())
	{
		if((char)(*strIter) == pattern)
		{
			repNum++;
		}
		strIter++;
	}
	return(repNum);
}

unsigned VerbioHandler::getVerbioMaxNumberOfFields(std::list<std::string> list2Sort)
{
	std::list<std::string>::iterator listIter;
	unsigned maxNumOfFields = 1;   //Maximum number of fields in the elements of the list
   	unsigned currentNumOfFields;

	//Get the maximum number of fields
   	listIter = list2Sort.begin();
   	while(listIter != list2Sort.end())
   	{
		currentNumOfFields = getNumberOfRepetitions((*listIter),'.')+1;
		if( currentNumOfFields> maxNumOfFields)
		{
			maxNumOfFields = currentNumOfFields;
		}
		listIter++;
   	}
	return(maxNumOfFields);
}

std::string VerbioHandler::getVerbioFieldAt(std::string unit, unsigned index)
{
	std::string::size_type strIter = 0;
	unsigned count = 0; //Count of scanned fields

	while((strIter < unit.size()) && (count < index))
	{
		if(unit.at(strIter) == '.')
		{
			count++;
		}
		strIter++;
	}
	return(unit.substr(strIter,unit.find('.',strIter)-strIter));
}

void VerbioHandler::sortVerbioListOfResults(std::list<std::string> & list2Sort)
{
	std::list<std::string>::iterator listIter;
	std::list<std::string>::iterator auxListIter;
	std::string auxElement; // Auxiliary element to order the list
	unsigned numOfElements; //Number of elements in the list
	unsigned maxNumOfFields; //Maximum number of fields for the rules
	bool noChanges = false; // Boolean to control the ordering of the list

	numOfElements = (unsigned)list2Sort.size();
	maxNumOfFields = getVerbioMaxNumberOfFields(list2Sort);
	for(int i = 0; i < (int)maxNumOfFields;i++)
	{
		noChanges = false;
		while(!noChanges)
		{
			noChanges = true;
  			listIter = list2Sort.begin();
			while(listIter != list2Sort.end())
			{
				auxListIter = listIter;
				auxListIter++;
				if(auxListIter != list2Sort.end())
				{
					if((atoi(getVerbioFieldAt((*auxListIter),i).c_str()) != 0) &&(atoi(getVerbioFieldAt((*listIter),i).c_str()) > atoi(getVerbioFieldAt((*auxListIter),i).c_str())))
					{
					  //The elements must be swapped
					  if(noChanges)
					  {
						noChanges = false;
					  }
					  auxElement = (*listIter); //Copy the element to remove
					  listIter = list2Sort.erase(listIter); //Erase the element from the list
					  auxListIter = listIter;
					  auxListIter ++;
					  list2Sort.insert(auxListIter,auxElement);//Push the element in the right position
					}
				}
				else
				{
					noChanges = true;
				}
				listIter++;
			}
		}
	}
}


/* end VerbioHandler class */
