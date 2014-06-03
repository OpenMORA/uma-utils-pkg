/*--------------------------------*/
/*	VOICE ENGINE HANDLER OBJECTS  */
/*                                */
/*	ECM, 2006                     */
/*--------------------------------*/

/** \file VoiceEngineHandler.hpp
 *\brief Voice handler objects for the BABEL modules library
 *
 * This file declares the object used to control the voice interaction with the
 * BABEL robotic modules. These objects provides Automatic Speech Recognition
 * (ASR) and Synthesis capabilities to the modules with different voice engines.
 */

#ifndef	VOICEENGINE
#define VOICEENGINE

#ifdef _WIN32
#define NOMINMAX	//to prevent windows.h from defining min/max symbols
#endif

#include <string>
#include <list>

#include "VerbioASR.h"
#include "VerbioTTS.h"

#ifndef _WIN32
	// non-Win32: Pulse library
	#include <pulse/simple.h>
#endif


/*! \brief VoiceEngineHandler handling objects.
 *
 *  VoiceEngineHandler objects are used to handle the interaction between the BABEL modules
 * and a voice engine. They give voice interaction capabilities to the robotic modules. This
 * is an abstract class from which derive all the handlers for the particular voice engines.
 */
class VoiceEngineHandler
{
	public:
		/*!\enum VoiceEngineType
		 *\brief Returns the type of the current VoiceEngineHandler object.
		 *
		 *That is: each voice engine (Verbio, SAPI...) has an associated object,
		 *this method allows to differentiate the type of the engine we are
		 *working with.
		 */
		typedef enum{DEFAULT = 0, /*!< Default */
					 VERBIO		/*!< Verbio Speech Engine */
					}VoiceEngineType;

		/*!\enum RecognitionStatus
		 *\brief This enum describes the different status outputs.
		 *
		 * That is: if there has been an error in the recgonition process,
		 * if the recognition process has finshed successfully or if there has been a
		 * timeout without spoken input.
		 */
		typedef enum{NO_INPUT = 0,	/*!< There has been a timeout without spoken input */
					 REC_OK,		/*!< The recognition process has a valid sample */
					 REC_ERROR		/*!< There has been any error in the recognition process */
					}RecognitionStatus;

		/*! Returns the type of the voice engine
		 *\return type of the current VoiceEngineHandler object.
		 */
		virtual VoiceEngineType getType(void)= 0;

		/*! Initializes the ASR object. The method gives information about the initialization
		 * process, and about the parameters set
		 *\return true if the object was initializated succesfully, false otherwise
		 */
		virtual bool Init(void)=0;

		/*! (Overloaded method)Initializes the ASR object with a particular grammar .
		 * The method gives information about the initialization process, and about
		 * the parameters set.
		 *\param grammar2LoadName name of the grammar to be loaded into the voice recognizer
		 *\param grammar2Load grammar to be loaded into the voice recognizer
		 *\return true if the object was initializated succesfully, false otherwise
		 */
		virtual bool Init(std::string grammar2LoadName, std::string grammar2Load)=0;

		/*! Stops the voice engine (ASR and synthesis)
		 */
		virtual void Stop(void)=0;

		/*! This function handles the recognition process. It obtain samples that are analysed
		 * by the recognition server in order to check if they satisfy one of the active grammars.
		 * If there is a match, it stores in rulesInfo an ordered list of the antecedents of the
		 * different rules that compose the main rule. The first position of the list is occupied by
		 * the main rule.The time to wait for a sample, the silences and the processing time are
		 * values set by default internally.
		 * If there is any error in the recognition process, the rulesInfo list values
		 * will be empty
		 *\param rulesInfo list of the rules the recognized pattern satisfies.
		 *\return status of the recognition process
		 */
		virtual RecognitionStatus RecognitionProcess(std::list<std::string>& rulesInfo)=0;

		/*! This function pauses the recognition process in order to avoid microphone feedback
		 * when the engine is synthesizing
		 *\return true if the recognition process was stopped succesfully, false otherwise
		 */
		virtual bool PauseRecognitionProcess(void)=0;

		/*! This function restores the recognition process when it has been paused.
		 *\return true if the recognition process was restored successfully, false otherwise
		 */
		virtual bool RestoreRecognitionProcess(void)=0;

		/*! This function allows to reproduce a string  with the voice engine.
		 *\param str2Reproduce string to reproduce
		 *\return true if the string was succesfully reproduced, false otherwise
		 */
		virtual bool ReproduceString(const std::string &str2Reproduce)=0;


}; /* end VoiceEngineHandler */

/*! \brief VerbioHandler handling objects.
 *
 *  VerbioHandler objects are used to handle the interaction between the BABEL modules and
 * the Verbio Speech Server. It gives voice interaction capabilities to the robotic modules.
 */
class VerbioHandler: public VoiceEngineHandler
{
	public:

		/*! Class constructor. If no other value is specified, the confidence threshold will
		 * be set at 90.0 by default.<I>(The confidence threshold is used to accept or
		 * discard rules in the recognition process)</I>
		 *\param thresValue value to be given to the confidence threshold
		 */
		VerbioHandler(float thresValue = 70.0);

		/*! Class destructor
		 */
		~VerbioHandler();

		/*! Returns the type of the voice engine
		 *\return type of the current VoiceEngineHandler object.
		 */
		 VoiceEngineType getType(void);

		/*! Initializes the ASR object. The method gives information about the initialization
		 * process, and about the parameters set
		 *\return true if the object was initializated succesfully, false otherwise
		 */
		bool Init(void);

		/*! (Overloaded method)Initializes the ASR object with a particular grammar .
		 * The method gives information about the initialization process, and about
		 * the parameters set.
		 *\param grammar2LoadName name of the grammar to be loaded into the voice recognizer
		 *\param grammar2Load grammar to be loaded into the voice recognizer
		 *\return true if the object was initializated succesfully, false otherwise
		 */
		bool Init(std::string grammar2LoadName, std::string grammar2Load);

		/*! Stops the voice engine (ASR and synthesis)
		 */
		void Stop(void);

		/*! Loads a grammar for the speech recognizer. The format of the Grammar must be
		 * ABNF, with the restrictions imposed by Verbio Grammars (for more info, refer to Verbio
		 * grammar specification). This method only loads the grammar in memory, but does not ac
		 * tivate it. To activate or free the grammar, the user must obtain an identifier for the
		 * grammar using the method GetGrammarByName.
		 *\param grammarName name of the grammar that is being loaded. This name is used to
		 *recognize the grammar for handling functions
		 *\param grammarStr string containing the grammar in a Verbio supported format
		 *\return true if the grammar was set succesfully, false otherwise
		 */
		bool LoadGrammar(std::string grammarName,std::string grammarStr);

		/*! Freeds all the resources consumed by a grammar loaded in the speech recognizer.
		 *\param grammarName name of the grammar the user wants to freed
		 *\return true if the grammar resources are successfully freed, false otherwise
		 */
		bool FreeGrammar(std::string grammarName);

		/*! Gets the identifier for a grammar name. This identifier is used to activate, deactivate
		 * or freed a lodaded grammar.
		 *\param grammarName string containing the name of the grammar to look for
		 *\return identifier for the grammar or -1 if the grammar could not be found.
		 */
		int GetGrammarIdByName(std::string grammarName);

		/*! Activates a grammar loaded in the recognizer.
		 *\param grammarName name of the grammar to activate
		 *\return true if the grammar was succesfully activated, false otherwise
		 */
		bool ActivateGrammar(std::string grammarName);

		/*! Deactivates a grammar loaded in the recognizer. This means that it is not used
		 * for the recognition process, but its resources are still reserved.
		 *\param grammarName name of the grammar to deactivate
		 *\return true if the grammar was succesfully deactivated, false otherwise
		 */
		bool DeactivateGrammar(std::string grammarName);

		/*! Waits a given time to assure that there is an available license. This allows to
		 * guarantee that the resources are available for the recognition process.
		 *\param time2Wait number of milliseconds to wait
		 *\return true if there is an available license, false otherwise
		 */
		bool Wait4License(int time2Wait);

		/*! This function handles the recognition process. It obtain samples that are analysed
		 * by the recognition server in order to check if they satisfy one of the active grammars.
		 * If there is a match, it stores in rulesInfo an ordered list of the antecedents of the
		 * different rules that compose the main rule. The first position of the list is occupied by
		 * the main rule.The time to wait for a sample, the silences and the processing time are
		 * values set by default internally.
		 * If there is any error in the recognition process, the rulesInfo list values
		 * will be empty
		 *\param rulesInfo list of the rules the recognized pattern satisfies.
		 *\return status of the recognition process
		 */
		RecognitionStatus RecognitionProcess(std::list<std::string>& rulesInfo);

		/*! (Overloaded method) This method handles the recognition process. It obtain samples that
		 * are analysed by the recognition server in order to check if they satisfy one of the active
		 * grammars. If there is a match, it stores in rulesInfo an ordered list of the antecedents of
		 * the different rules that compose the main rule. The first position of the list is occupied
		 * by the main rule.
		 * If there is any error in the recognition process, the rulesInfo list values
		 * will be empty
		 *\param rulesInfo list of the rules the recognized pattern satisfies.
		 *\param maxProcessTime maximum time (in milliseconds) to wait for the whole recognition
		 * process. If this time is exceeded, the recognition process will stop.
		 *\param maxInitSilence maximum wait time (in milliseconds) for the server to receive audio
		 * samples. If this time is exceeded, the recognition process will stop.
		 *\param maxFinalSilence Once a voice signal is detected, the recognition process will be
		 * stopped if there is a silence longer than maxFinalSilence (milliseconds).
		 *\return status of the recognition process
		 */
		RecognitionStatus RecognitionProcess(std::list<std::string>& rulesInfo,int maxProcessTime,int maxInitSilence,int maxFinalSilence);

		/*! This function pauses the recognition process in order to avoid microphone feedback
		 * when the engine is synthesizing
		 *\return true if the recognition process was stopped succesfully, false otherwise
		 */
		bool PauseRecognitionProcess(void);

		/*! This function restores the recognition process when it has been paused.
		 *\return true if the recognition process was restored successfully, false otherwise
		 */
		bool RestoreRecognitionProcess(void);

		/*! This function allows to reproduce a string  with the voice engine.
		 *\param str2Reproduce string to reproduce
		 *\return true if the string was succesfully reproduced, false otherwise
		 */
		bool ReproduceString(const std::string &str2Reproduce);

		/** A common method to initialize the audio system in win/lin.
		  * \return false on error
		  */
		bool InitializeAudioChannels();

		/** A common method to clean-up the audio system in win/lin.
		  */
		void DeinitializeAudioChannels();

		/*! Sets the confidence threshold (value used to accept or reject a rule in a
		 * recognition process) to the new value.
		 *\param thresValue new value for the confidence threshold
		 */
		void SetConfidenceThreshold(float thresValue);

		/*! Gets the current confidence threshold (value used to accept or reject a
		 * rule in a recognition process) value.
		 *\return current confidence threshold value
		 */
		float GetConfidenceThreshold(void);

	private:
		/*! Returns the number of repetitions of a given character in a string
		 *\param str2Work string to extract the number of repetitions
		 *\param pattern character to look for
		 *\return number of repetitions of the pattern param
		 */
		unsigned getNumberOfRepetitions(std::string str2Work,char pattern);

		/*! Gets the maximum number of fields of the units that compose a Verbio result
		 *\param list2Sort list to get the number of fields
		 *\return maximum number of fields in the list
		 */
		unsigned getVerbioMaxNumberOfFields(std::list<std::string> list2Sort);

		/*! Gets the field numbered by index in a unit of a Verbio result.Index goes
		 * from 0 to (Number of fields)-1.
		 *\param unit element of the list to get the field
		 *\param index index of the field to get
		 *\return selected field
		 */
		std::string getVerbioFieldAt(std::string unit, unsigned index);

		/*! Sorts the list of results provided by Verbio engines alphabetically
		 *\param list2Sort list of results to sort
		 */
		void sortVerbioListOfResults(std::list<std::string> & list2Sort);

		/*!\struct grammarAssociation
		 *\brief Grammar Identifier Association
		 *
		 * It stores the information about which identifiers are associated with the grammar names,
		 * the file in which the grammar will be stored and the status of each grammar
		 */
		struct grammarAssociation
		{
			std::string name; /*! Name of the grammar */
			std::string fileName; /*! Name of the file in which the grammar is stored */
			int id;		 /*! Identifier associated to the grammar name */
		};

		/** A cross platform version of Win32 "RecStr":
		  * \param maxtime In millisecs.
		  * \param initsil In millisecs.
		  * \param maxsil In millisecs.
		  * \return True: Speech recogniced OK, false: Error grabbing or no speech detected.
		  */
		bool RecStr(int maxtime, int initsil, int maxsil);

		/*!\brief Information about the grammars loaded in the ASR.
		 *
		 * It stores information about the names and identifiers for the loaded grammars, as well
		 * as the status of those grammars (active or deactivated)
		 */
		typedef std::list<grammarAssociation> grammarsInfo;

		VoiceEngineType myType; /* Type of the voice Engine object */
		float	m_samplingFreq;  //!< As returned by the verbio engine
		VerbioASR*	m_asrObject;
		VerbioTTS*	m_ttsObject;
		VerbioASRResource * m_ASRResource;
		VerbioTTSResource * m_TTSResource;

		grammarsInfo loadedGrammars; /* Grammars loaded for the recognizer */
		float confidenceThreshold; /* Threshold to consider a recognition result as valid */

#ifndef _WIN32
		// Data specific to Pulse audio library in non-WIN32:
		pa_simple	*m_pulse_session_play;
		pa_simple	*m_pulse_session_rec;
#endif

}; /* end VerbioHandler */
#endif
