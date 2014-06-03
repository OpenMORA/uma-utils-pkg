/*-----------------------------------------------------------*/
/*	GRAMMAR HANDLER LIBRARY FOR BABEL MODULES ARCHITECTURE   */
/*                                                           */
/*	ECM, 2006                                                */
/*-----------------------------------------------------------*/

/** \file GrammarHandler.hpp
 *\brief Grammar Handler Library for Babel modules architecture
 *
 * This file declares the class GrammarHandler designed to handle the interaction
 * between the Babel Modules (or any external module that follows the same 
 * architecture) and the Grammar objects.
 */

#ifndef GRAMMARHANDLERCLASS
	#define GRAMMARHANDLERCLASS

#include "ArchitectureGrammar.hpp"
#include <list>

/*!\brief Class to handle the interaction between Grammars and BABEL Modules
 *
 *(or any user that follows the same architecture). The class provides objects 
 * to store the information relative to objects and actions that are recognized 
 * by the WorldModel and the TaskVoice modules. These classes will be used to 
 * build the associated rules in the Grammar.
 * In order to obtain a well-formed grammar, the methods of this class should
 * be executed sequentially, that is:
 * <br>1.- Retrieval of the objects and actions information
 * <br>2.- Convert the objects and actions information to rules
 * <br>3.- Produce the string associated to the grammar rules
 * \warning This class is a simple example of the integration between grammars
 * and BABEL, for a complete and correct voice interaction, it is strongly recommended
 * the usage of VoiceManager objects.
 */
class GrammarHandler
{
	public:
		/*!\brief Class to be used as definition of objects for BABEL grammars.
		 *
		 * The objects are characterized by a label, a set of associated words
		 * to the object and a type.		 
		 */

		/******************************
		 * Nested classes definitions *
		 ******************************/

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
				 *\param words list of words for the object
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

		/*!\struct neededMethods
		 * \brief Pointers to the methods that are needed by GrammarHandler class to work
		 *
		 * ,but are implemented by objects that are ouside this class.
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
		}neededMethods;

		/**************************
		 * Main class definitions *
		 **************************/

		/*! Default constructor
		 */
		GrammarHandler();

		/*! Class constructor
		 *\param reqMethods pointer to a set of methods that has to be defined 
		 * outside the method
		 */
		GrammarHandler(void * reqMethods);

		/*! Class destructor
		 */
		~GrammarHandler();

		/*! Retrieves all the exisiting objects and store their information
		 * in the GrammarHandler object. The method will show by console 
		 * status messages as the retrieving process is being carried out	
		 *\return number of retrieved objects
		 */
		unsigned retrieveObjects(void);

		/*! Retrieves all the existing actions and store their information
		 * in the GrammarHandler object. The method will show by console 
		 * status messages as the retrieving process is being carried out	
		 *\return number of retrieved actions
		 */
		unsigned retrieveActions(void);

		/*! Converts all the information about objects and actions into grammar
		 * rules.
		 *\return true if the conversion was succesfull, false if any error occurs
		 */
		bool convert2Rules(void);

		/*! Generates a string that contains all the rules generated for the grammar. 
		 * The string is generated following the format specified by the input parameter.
		 * Returns an empty string if the associated Grammar has no rules or in case of 
		 * error. The output format supplied as parameter must be generated following 
		 * the format (.jms) described in Jafma_MacroScript.h and has to include a 
		 * repetition macro for the rules and a repetition macro for the components of 
		 * the rule. An example of an appropiate outputFormat string is found in 
		 * "ArchitectureTemplateExample.jms", provided with this library. The zone of 
		 * code used is named ARCHITECTURE_GRAMMAR and the name of the used areas and 
		 * macros are:
		 * <br>ObjectType_rules: Repetition area used to iterate rules related to the 
		 * different types of objects.
		 * <br>Type_name: Macro for the name of each type of object.
		 * <br>Object_rules: Repetition area to iterate the rules of a same type
		 * <br>Object_label: Macro for the label of each object
		 * <br>Object_relative_rules: Repetition macro to iterate the rules of a 
		 * particular object
		 * <br>Antecedent_name: Macro for the name of the antecedent of a rule.
		 * <br>Obj_RelRule_consequents_list: Repetition area for the consecuents of a rule
		 * of a particular object.
		 * <br>is_NonTerminal: Condititional repetition macro to indicate if a Consequent is 
		 * a NonTerminal or not.
		 * <br>Consequent_name: Repetition macro indicating the name of a Consequent in a rule		 
		 * <br>is_MascSing: Conditional repetition macro, it indicates, for NonTerminal nouns
		 * that is masculine and singular.
		 * <br>is_MascPlural: Conditional repetition macro, it indicates, for NonTerminal nouns
		 * that is masculine and plural.
		 * <br>is_FemSing: Conditional repetition macro, it indicates, for NonTerminal nouns
		 * that is fememnine and singular.
		 * <br>is_FemPlural: Conditional repetition macro, it indicates, for NonTerminal nouns
		 * that is femenine and singular.
		 * <br>OR_Rule: Conditional repetition macro, it indicates whether the rule is an OR
		 * rule or a sequence rule.
		 * <br>ObjectType_rules: Macro for the name of each type of objects
		 * <br>Type_relative_rules: Repetition area for the types of object rules
		 * <br>Type_RelRule_consequents_list: Repetitiona area for the consecuents of the
		 * type of objects rules.
		 * <br>Actions_list: Repetition area for the list of actions
		 * <br>Action_name: Macro to indicate the name of an action
		 * <br>Action_rules: Repetition area for the rules of an action
		 * <br>is_Publishable: Macro to indicate if a rule is publishable or not
		 * <br>is_Transitive: Conditional repetition macro, it indicates, for NonTerminals if it
		 * is (the verb) transitive or not.
		 *\param outputFormat indicates the format for the output string.
		 *\result string associated with the Grammar.
		 */ 
		std::string getGrammarString(const std::string & outputFormat);

		 /*! Gets the expression that corresponds with the recognized actions in order to 
		  * give it to the TaskVoice. The Method accepts as parameter a set of antecedents
		  * and check out which action rule they represent
		  *\param actionRules list of antecedents of the terminals that compose the rule		
		  *\return string containing the translated action
		  */
		std::string getAssociatedAction(const std::list<std::string> actionRules);

	private:			
		neededMethods * eiMethods; //Structure with the external implemented Methods
		std::list<Object> associatedObjects;
		std::list<Action> associatedActions;
		ArchitectureGrammar	associatedAGrammar;
};
#endif