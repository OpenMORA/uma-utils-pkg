/*---------------------------------------------------------*/
/*	VOICE MANAGER LIBRARY FOR BABEL MODULES ARCHITECTURE   */
/*                                                         */
/*	ECM, 2007                                              */
/*---------------------------------------------------------*/

/** \file VoiceManager.hpp
 *\brief Voice Manager object for Babel modules architecture
 *
 * This file declares the class VoiceManager designed to provide
 * voice interaction capabilities to the BABEL modules (or any 
 * object that follows the same architecture).
 */

#ifndef VOICEMANAGERCLASS
	#define VOICEMANAGERCLASS

#include <mrpt/system/threads.h>
#include <mrpt/system/datetime.h>
#include "mrpt/synch.h"
#include "ArchitectureGrammar.hpp"
#include "VoiceEngineHandler.hpp"
#include "TimedStringsQueue.hpp"

/*! \brief VoiceManager objects.
 *
 *  VoiceManager objects provide voice interaction capabilities to the BABEL
 * modules. They handle both the recognition and synthesis process, as well
 * as the initialization of the different voice engines (Verbio, SAPI...)
 */
class VoiceManager
{
	public:
		/******************************
		 * Nested classes definitions *
		 ******************************/

		/*!\brief Class to be used as definition of objects for BABEL grammars.
		 *
		 * The objects are characterized by a label, a set of associated words
		 * to the object and a type.		 
		 */
		class Object
		{
			public:
				/*! Default constructor
				*/
				Object();
		
				/*! Class constructor. Allows to fill the new object with the given
				 *values
				 *\param label value of the label to set for the object
				 *\param nouns list of nouns for the object 
				 *\param typeValue value of the type to set for the object
				 */
				Object(const std::string & label, const ArchitectureGrammar::ObjectNouns & nouns,const std::string & typeValue);
		
				/*! Copy constructor
				 */
				Object(const Object & originObject);
		
				/*! Default destructor 
				 */
				~Object();

				/*! Overloaded equal operator to avoid malfunction when copying 
				 *\param originObject object to replicate in this object.
				 *\return reference to a Object object
				 */ 
				Object & operator = (const Object &originObject);
			
				/*! Sets the label of the object 
				 *\param label value of the label to set for the object
				 */
				void setLabel(const std::string & label);

				/*! Gets the label of the object 
				 *\return label of the object
				 */
				std::string getLabel(void);

				/*! Set the nouns associated to the object.
		 		 *\param nouns list of nouns for the object
				 */
				void setNouns(const ArchitectureGrammar::ObjectNouns & nouns);

				/*! Gets the nouns associated to the object.
				 *\returns list of nouns for the object
				 */
				ArchitectureGrammar::ObjectNouns getNouns(void);

				/*! Sets the type of the object 
				*\param typeValue value of the type to set for the object
				*/
				void setType(const std::string & typeValue);

				/*! Gets the type of the object 
				 *\return type of the object
				 */
				 std::string getType(void);

			private:
				/*! Copy method to properly replicate objects when using the copy
				 *  constructor and the = operator.
				 *\param originObject Object object to replicate in this object.
				 */
				void copyObject(const Object &originObject);

				std::string objLabel; //Label of the object
				ArchitectureGrammar::ObjectNouns objNouns; //Nouns for the object
				std::string objType; //Type of the object
		
		}; /* end Object class */

		/*!\brief Class to be used as definition of actions for BABEL grammars.
		 *
		 * The actions are characterized by a label, a set of words to describe
		 * the action, a set of associated parameters needed to carry out the
		 * action and the transitivity property.
		 */
		class Action
		{
			public:
				/*! Default constructor
				 */
				Action();

				/*! Class constructor. Allows to fill the new object with the given
				 * values
				 *\param label value of the label to set for the action
				 *\param words list of words for the action
				 *\param params list of words for the action
				 *\param transitivity value of the transitivity to set for the action
				 */
				Action(const std::string & label, const ArchitectureGrammar::WordsList & words,const ArchitectureGrammar::WordsList & params,NonTerminal::NonTerminalTransitivity transitivity = NonTerminal::T_NOT_USED);
		
				/*! Copy constructor
				 */
				Action(const Action & originAction);
		
				/*! Default destructor 
				 */
				~Action();

				/*! Overloaded equal operator to avoid malfunction when copying 
				 *\param originAction object to replicate in this object.
				 *\return reference to an Action object
				 */ 
				Action & operator = (const Action &originAction);

				/*! Sets the label of the action
				 *\param label value of the label to set for the action
				 */
				void setLabel(const std::string & label);

				/*! Gets the label of the action
				 *\return label of the action
				 */
				std::string getLabel(void);

				/*! Set the words associated to the action.
		 		 *\param words list of words for the object
				 */
				void setWords(const ArchitectureGrammar::WordsList & words);

				/*! Gets the words associated to the object.
				 *\returns list of words for the object
				 */
				ArchitectureGrammar::WordsList getWords(void);

				/*! Set the parameters associated to the action.
		 		 *\param params list of words for the action
				 */
				void setParams(const ArchitectureGrammar::WordsList & params);

				/*! Gets the parameters associated to the action.
				 *\returns list of parameters for the action
				 */
				ArchitectureGrammar::WordsList getParams(void);

				/*! Set the transitivity value for the action.
		 		 *\param transitivity value of the transitivity for the action
				 */
				void setTransitivity(NonTerminal::NonTerminalTransitivity transitivity);

				/*! Gets the transitivity of the action.
				 *\returns transitivity of the action
				 */
				NonTerminal::NonTerminalTransitivity getTransitivity(void);

			private:
				/*! Copy method to properly replicate objects when using the copy
				 *  constructor and the = operator.
				 *\param originAction Action object to replicate in this object.
				 */
				void copyAction(const Action &originAction);

				std::string actLabel; //Label of the action
				ArchitectureGrammar::WordsList actWords; //Words for the action
				ArchitectureGrammar::WordsList actParams; //Parameters for the action
				NonTerminal::NonTerminalTransitivity actTransitivity; //Transitivity of the action
		}; /* end Action class */

		/*********************
		 * Types definitions *
		 *********************/

		/*!\struct neededRequestMethods
		 * \brief Pointers to the methods that are needed by VoiceManager class to work
		 * out its request of information.
		 * These methods are implemented by objects that are ouside this class.		 
		 */
		typedef struct
		{				
			/*! Gets the number of objects in the WorldModel 
			 *\return number of objects
			 */
			unsigned(* getWMNumberOfObjects)(void);

			/*! Gets an object from the WorldModel (index starts at 0).
			 *\param index number of the object to return
			 *\return object at index
			 */			
			Object(* getWMObjectAt)(unsigned index);

			/*! Gets the number of objects in the TaskVoice
			 *\return number of objects
			 */
			unsigned(* getTPNumberOfObjects)(void);

			/*! Gets an object from the TaskVoice (index starts at 0).
			 *\param index number of the object to return
			 *\return object at index
			 */
			Object(* getTPObjectAt)(unsigned index);

			/*! Gets the number of actions in the TaskVoice
			 *\return number of actions
			 */
			unsigned(* getTPNumberOfActions)(void);

			/*! Gets an object from the TaskVoice (index starts at 0).
			 *\param index number of the action to return
			 *\return action at index
			 */
			Action(* getTPActionAt)(unsigned index);															
		}neededRequestMethods;

		/*!\struct concurrencyMethods
		 * \brief Pointers to the methods that are needed by VoiceManager class to 
		 *
		 * manage concurrent access,but are implemented by objects that are ouside 
		 * this class.
		 */
		typedef struct
		{
			/*! Enter critical zone
			 *\return true if error
			 */
			bool (* enterCriticalZone)(void);

			/*! Leave critical zone
			 *\return true if error
			 */
			bool (* leaveCriticalZone)(void);

			/*! Shutdown routine
			 *\return true if error
			 */
			bool (* shutdown)(void);
			
		}concurrencyMethods;

		/**************************
		 * Main class definitions *
		 **************************/

		/*! Class constructor
		 *\param reqMethods pointer to a set of request methods that has to be defined 
		 * outside the method
		 *\param cMethods pointer to a set of concurrency methods that has to be defined
		 * outside the method
		 */
		VoiceManager(void * reqMethods,void * cMethods);

		/*! Class destructor
		 */
		~VoiceManager();

		/*! Initializes the Voice object. It carries out all the operations needed to
		 * build the grammar and load it into the voice engine and to start the voice 
		 * engine itself. The method gives information about the initialization process,
		 * as well as about the parameters set. This method is thread safe with the other
		 * methods of the class, but not with itself.
		 *\param engineType type of the engine the user wants to work with
		 *\return true if the object was initializated succesfully, false otherwise
		 */		 
		bool InitVoiceInteraction(VoiceEngineHandler::VoiceEngineType engineType);

		struct TRecognizedPhrase
		{
			mrpt::system::TTimeStamp timestamp; //!< Time of end of recognition
			std::string  requestedCode;
			std::string  goal;
		};

		typedef std::list<TRecognizedPhrase> TRecognizedPhraseList;

		/** Return a list of recognized phrases (recorded from an independent thread), ordered by time (the latest the most recent).
		  *  The internal buffer of recognized strings is emptied with each call, so only new phrases will be returned upon each call.
		  */
		void GetRecognizedPhrases(TRecognizedPhraseList &outList);



		/*! Receives a string to reproduce and puts it in the reproduction queues according
		 * to the priority level of the messange. The strings will be reproduced in turn by
		 * the voice engine. If the string is queued more time than strTimeout, it will be 
		 * removed from the queue. This method is thread safe.
		 *\param str2Reproduce string to reproduce
		 *\param prio priority of the string to queue (0->lowest priority)
		 *\param strTimeout time (in secs) to wait into its queue for the string 	 
		 */
		void QueueString2Reproduce(std::string str2Reproduce,unsigned prio,unsigned strTimeout);

		/*! (Overloaded method) Receives a string to reproduce and puts it in the reproduction
		 * queues according to the priority level of the messange. The strings will be reproduced
		 * in turn by the voice engine. If the string is queued more time than strTimeout, it 
		 * will be removed from the queue. This method is thread safe.
		 *\param str2Reproduce string to reproduce
		 *\param prio priority of the string to queue (0->lowest priority)
		 *\param strTimeout time (in secs) to wait into its queue for the string
		 *\param groupId identifier of the group the string belongs to
		 *\param minTimeBtw minimum elapsed time (in seconds) between two messages of the 
		 * same group to be reproduced
		 */
		void QueueString2Reproduce(std::string str2Reproduce,unsigned prio,unsigned strTimeout,unsigned groupId,double minTimeBtw);

		/*! This method sends to the voice engine all the available strings to reproduce
		 * in the strings queues. This method is thread-safe.
		 * \return True if any string has been reproduced.
		 */
		bool ReproduceQueuedStrings(void);

		void LoadActions(std::vector<std::string> actions);
		void LoadWorldElements(std::vector<std::string> elements);
	
	private:		
		neededRequestMethods * eiMethods; //Structure with the external request Methods
		concurrencyMethods * ecMethods; //Structure with the external concurrency Methods
		ArchitectureGrammar	associatedAGrammar; //Grammar associated to the voice interaction
		VoiceEngineHandler * engineH; //Pointer to the Voice Engine Handler
		bool initStatus; //True if the engine is initializated, false otherwise
		VoiceEngineHandler::VoiceEngineType ourEngineType; //Type of our engine	
		TimedStringsQueue prioQueue; //Priority queue of strings to reproduce

		std::vector<Action> actions_vec;
		std::vector<Object> elements_vec;

		mrpt::synch::CCriticalSection  sem;

		
		/** Internal buffer between the recog. thread and GetRecognizedPhrases */
		TRecognizedPhraseList          m_recog_buf;
		mrpt::synch::CCriticalSection  m_recog_buf_cs;
		bool	m_recog_thread_must_exit;
		void recog_thread();  //!< recognition thread
		mrpt::system::TThreadHandle  m_handle_recog_thread;

		/*! Carries out the recognition process and returns a string containing the
		 * result provided by the recognition engine. If any error occurs, it returns
		 * false and stores in recStr an empty string. This method is thread-safe.
		 *\param recStr string to store the result
		 *\return true if there is no error, false otherwise
		 */
		bool RecognitionProcess_private(std::string & recStr);

		/*! (Overloaded method) Carries out the recognition process and returns the 
		 * recognition result in a way that can be easily understood by the BABEL Task 
		 * Voice. If any error occurs, the output parameters will be left empty.
		 *\param requestCode Code of the recognized action
		 *\param goal string containing all the parameters needed for the action
		 *\return true if there is no error, false otherwise.
		 */
		bool RecognitionProcess_private(std::string &requestCode, std::string &goal);


};
/* End VoiceManager class */
#endif
