/*---------------------------------------------------*/
/*	ARCHITECTURE GRAMMAR LIBRARY FOR BABEL MODULES   */
/*                                                   */
/*	ECM, 2006                                        */
/*---------------------------------------------------*/

/** \file ArchitectureGrammar.hpp
 *\brief Architecture grammar library for Babel modules definition file.
 *
 * This file declares the class ArchitectureGrammar, designed to mantain a grammar
 * associated with a particular architecture.
 */


#ifndef ARCHITECTUREGRAMMARCLASS
	#define ARCHITECTUREGRAMMARCLASS

#include "Grammar.hpp"
#include <list>

/*! Definition of the label used to represent a free string.
 */
#define FREE_STRING "FREE_STRING"
/*! Definition of the Id of the antecedent of the associated rule for a free
 * string.
 */
#define FREE_STRING_ID 1

/*! Auxiliary function needed to know if a given rule is in a set
 * set of rules
 *\param antecedentName antecedent of the rule to look for
 *\param rulesList list of rules to look for the antecedent
 *\return true if the rule is on the set, false otherwise
 */
bool isOnSet(const std::string antecedentName, const std::list<std::string>rulesList);

/*!\brief ArchitectureGrammar Objects.
 *
 *  ArchitectureGrammar objects contain grammars related to a particular architecture
 * based in the existence of objects and actions, and generate information to represent
 * those grammars (i.e. provide .bnf files to be used with a voice recognition software).
 */
class ArchitectureGrammar
{
	public:
		/*!\brief Class to store and handle a list of words (strings).
		 *
		 * The lists can be used as parameters to some methods of the
		 * ArchitectureGrammar class or to store information inside the class.
		 */
		class WordsList
		{
			public:
				/*! List to store the words
				 */
				typedef std::list<std::string> words;

				/*! Default constructor
				 */
				WordsList();

				/*! Copy constructor
				 *\param originWordList WordList object to replicate in this object.
				 */
				WordsList(const WordsList & originWordList);

				/*! Class destructor
				 */
				~WordsList();

				/*! Overloaded equal operator to avoid malfunction when copying
				 *\param originWordList WordList object to replicate in this object.
				 *\return reference to a WordsList object
				 */
				WordsList & operator = (const WordsList & originWordList);

				/*!Adds a new word to the list of words.
				 *\param word word to add
				 */
				void addWord(const std::string & word);

				/*! Gets the word stored at the specified position. This
				 * method provides a copy of the information, but not
				 * removes the object from the list.
				 *\param position pose of the word to return
				 *\return word information at position
				 */
				std::string getWordAt(const words::iterator & position);

				/*! Removes from the list of words the element at the specified
				 * position.
				 *\param position pose of the word to erase
				 */
				void eraseWordAt(const words::iterator & position);

				/*! Removes all the elements from the list
				 */
				void clear(void);

				/*! Get an iterator to the beginning of the list of words.
				 *\return iterator to the beginning of the list
				 */
				words::const_iterator begin(void) const;

				/*! Get an iterator to the end of the list of words.
				 *\return iterator to the end of the list
				 */
				words::const_iterator end(void) const;

				/*! Check if the list of words is empty
				 *\return true if the list is empty, false if not
				 */
				bool empty(void) const;

			private:
				/*! Copy method to properly replicate objects when using the copy
				 *  constructor and the = operator.
				 *\param originWordList WordList object to replicate in this object.
				 */
				void copyWordsList(const WordsList &c);

				words listOfWords;

		}; /* end WordsList */

		/*!\brief Stores the nouns used to name an object in ArchitectureGrammar.
		 *
		 * The information is stored in a list. This information will be used to
		 * build the rules that compose the associated grammar for the current
		 * ArchitectureGrammar
		 */
		class ObjectNouns
		{
			public:
				/*!\struct nounInformation
				* \brief Information about a word
				*
				* Contains the information about a noun(word, genre and number).
				*/
				struct nounInformation
				{
					std::string associatedWord;
					NonTerminal::NonTerminalGenre genre;
					NonTerminal::NonTerminalMultiplicity number;
				};

				/*! List storing words and its characteristics information
				 */
				typedef std::list<nounInformation> nounInformationList;

				/*! Default constructor
				 */
				ObjectNouns();

				/*! Copy constructor
				 *\param originObjectNouns object to replicate in this object.
				 */
				ObjectNouns(const ObjectNouns & originObjectNouns);

				/*! Class destructor
				*/
				~ObjectNouns();

				/*! Overloaded equal operator to avoid malfunction when copying
				 *\param originObjectNouns object to replicate in this object.
				 *\return reference to a ObjectNouns object
				 */
				ObjectNouns & operator = (const ObjectNouns &originObjectNouns);

				/*!Adds a new noun to the list of nouns of the object
				 *\param nounInfo information about the new noun to add
				 */
				void addNoun2Object(const nounInformation & nounInfo);

				/*! Gets the noun stored at the specified position. This
				 * method provides a copy of the information, but not
				 * removes the object from the list.
				 *\param position pose of the noun to return
				 *\return noun information at position
				 */
				nounInformation getNounAt(const nounInformationList::iterator & position);

				/*! Removes from the list of nouns the element at the specified
				 * position.
				 *\param position pose of the noun to erase
				 */
				void eraseNounAt(const nounInformationList::iterator & position);

				/*! Removes all the elements from the list
				 */
				void clear(void);

				/*! Get an iterator to the beginning of the list of nouns
				 *\return iterator to the beginning of the list
				 */
				nounInformationList::const_iterator begin(void) const;

				/*! Get an iterator to the end of the list of nouns
				 *\return iterator to the end of the list
				 */
				nounInformationList::const_iterator end(void) const;

			private:
				/*! Copy method to properly replicate objects when using the copy
				 *  constructor and the = operator.
				 *\param originObjectNouns ObjectNouns object to replicate in this object.
				 */
				void copyObjectNouns(const ObjectNouns &originObjectNouns);

				nounInformationList nounsList;

		};/* end ObjectNouns */

		/*! Default Constructor.
		 */
		ArchitectureGrammar();

		/*! Class Destructor.
		*/
		~ArchitectureGrammar();

		/*! Shows (by console) the internal state of the object
		*/
		void showState(void);

		/*! Adds a new object to the associated grammar. Returs true if the object was added
		 * succesfully, false otherwise.
		 *\param newObjectLabel label of the new object(unique name)
		 *\param newObjectInfo list array containing all the words used to name the object, as well
		 * as information about the words.
		 *\param associatedType string containing a parameter indicating the associated type
		 * for a given object
		 *\return true if the object was added succesfully, false otherwise.
		 */
		bool addNewObject(const std::string &newObjectLabel,const ObjectNouns & newObjectInfo,const std::string & associatedType);

		/*! Adds a new action to the associated grammar. Returns true if the action was added
		 * succesfully, false otherwise. If the type of the parameters is not found in the list
		 * of objects, the action won't be added to the list, so the user must add all the
		 * objects to the grammar before the insertion of the actions. Some actions can have
		 * a free string as parameter, in this case, the type declared will be "FREE_STRING"
		 * and it is not necessary before the insertion of the actions in the
		 * ArchitectureGrammar. The free string rule has the NonTerminal antecedent 1.
		 *\param newActionLabel label of the new action (unique name)
		 *\param newActionWords list array containing all the words(verbs)used to represent the action.
		 *\param associatedParams list array containing the names of the types of the associated
		 * parameters for the action.
		 *\param transitivity Indicates if the action is transitive or intransitive. By default,
		 * the transitivity is declared as not specified.
		 *\return true if the action was added succesfully, false otherwise.
		 */
		bool addNewAction(const std::string & newActionLabel,const WordsList &newActionWords,const WordsList &associatedParams,NonTerminal::NonTerminalTransitivity transitivity = NonTerminal::T_NOT_USED);

		/*! Generates a string that contains all the rules of the ArchitectureGrammar.
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
		 * that is femenine and singular.
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
		std::string getGrammarAsString(const std::string & outputFormat);

		/*! Returns a pointer to a copy of the Grammar object associated with this handler
		 *\return pointer to the copy of the associated Grammar.
		 */
		Grammar * getGrammar(void);

		/*! Gets the action request associated to an action rule. If any error occurs, it
		 * returns an empty string.
		 *\param mainActionRule antecedent of the action rule
		 *\param rulesList list of rules that compose the action
		 *\return string containing the translated action
		 */
		std::string getAssociatedAction(const int mainActionRule, std::list<std::string> &rulesList);

	private:
		/*!\struct actionAssociation
		 *\brief Provides information about the rules associated to a given action
		 */
		 struct actionAssociation
		 {
			 std::string elementName; /*! Name of the actions */
			 std::list<int> associatedNtIds; /*! Identifiers of the associated non-terminals */
		 };

		/*!\struct objectAssociation
		 *\brief Provides information about the rules associated to a given object
		 */
		 struct objectAssociation
		 {
			 std::string elementName; /*! Name of the object */
			 std::string typeName;	  /*! Associated type */
			 std::list<int> associatedNtIds; /*! Identifiers of the associated non-terminals */
		 };

		/*!\struct typeAssociation
		 *\brief Provides information about the final rule compounded by all the objects of
		 * a given type.
		 */
		struct typeAssociation
		{
			std::string typeName; /*! Name of an existing type */
			int ntId;		 /*! Identifier of the non-terminal of the associated rule */
		};

		/*! Finds in the list of associated actions the given element name, if the element
		 * exists, points elementIter to it and returns true, otherwise, it returns false,
		 * and elementIter will point to the end of the list.
		 *\param elementName name of the element to look for
		 *\param elementIter iterator to return
		 *\return true if the element exists, false otherwise
		 */
		bool findActionInList(const std::string & elementName,std::list<actionAssociation>::iterator &elementIter);

		/*! Finds in the list of associated objects the given element name, if the element
		 * exists, points elementIter to it and returns true, otherwise, it returns false,
		 * and elementIter will point to the end of the list.
		 *\param elementName name of the element to look for
		 *\param elementIter iterator to return
		 *\return true if the element exists, false otherwise
		 */
		 bool findObjectInList(const std::string & elementName,std::list<objectAssociation>::iterator &elementIter);

		/*! Finds in the list of associated types the given identifier and returns the NonTerminal
		 * Id that corresponds to the rule. If the identifier is not in the list, returns 0.
		 *\param look4typeName name of the type to look for
		 *\return Id of the associated NonTerminal
		 */
		 int findTypeIdentifier(const std::string & look4typeName);

		/*! Finds in the list of associated actions the label of a action, given the antecedent
		 * name. If there is any error, it returns empty string.
		 *\param antecedentName antecedent to look for in the list of actions associations
		 *\return action label, or empty string if there is any error.
		 */
		 std::string findActionByAntecedent(const int antecedentName);

		/*! Finds in the list of associated objects the label of an action, given the antecedent
		 * name. If there is any error, it returns empty string.
		 *\param antecedentName antecedent to look for in the list of objects associations
		 *\return object label, or empty string if there is any error.
		 */
		 std::string findObjectByAntecedent(const int antecedentName);

		Grammar associatedGrammar;
		std::list<actionAssociation> actionsAssociations; /* List of non-terminals associated
													          with the actions.*/
		std::list<objectAssociation> objectsAssociations; /* List of non-terminals associated
													     with the objects.*/
		std::list<typeAssociation> grammarAssociations; /* List of the associations for types*/
		int currentNonTerminalIndex; /* Identifier for the next new NonTerminal introduced in
									  the grammar */
}; /* end ArchitectureGrammar */
#endif
