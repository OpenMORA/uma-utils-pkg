/*--------------------------------------*/
/*	GRAMMAR LIBRARY FOR BABEL MODULES   */
/*                                      */
/*	ECM, 2006                           */
/*--------------------------------------*/

/** \file Grammar.hpp
 *\brief Grammar library to store all the information related to a particular grammar.
 *
 * This file declares the class Grammar, designed to mantain a generic grammar that can
 * be converted to any particular representation format.
 */

#ifndef GRAMMARCLASS
	#define GRAMMARCLASS

#include "GrammarRules.hpp"
#include <string>

/*! \brief Grammar objects.
 *
 *  These objects are a set of rules that define the behaviour of a language
 */
class Grammar
{
	public:		
		/*!\struct GrammarRule
		 *\brief Rule objects for the Grammar.
		 * It also stores information indicating whether the rule is publishable or not
		 */
		struct GrammarRule
		{
			Rule * ruleObjectPtr;
			bool isPublishable;
		};

		/*! \brief List of Rule objects that form a Grammar.
		 * The list is composed by a set of pointers to GrammarRule structs.
		 */
		typedef std::list<GrammarRule> GrammarRules;

		/*! Grammar Class Constructor.
		 */
		Grammar();

		/*! Copy constructor. Replicates the object passed as parameter
		 *\param originGrammar Grammar object to replicate in this object.
		 */
		Grammar(const Grammar & originGrammar);

		/*! Class destructor.
		 */
		~Grammar();

		/*! Overloaded equal operator to avoid malfunction when copying 
		 *\param originGrammar Grammar object to replicate in this object.
		 */
		Grammar & operator = (const Grammar &originGrammar);

		/*! Allows the used to make a dynamic copy of the object, that is the method creates a copy of the object and returns a pointer to that copy.
		 \return pointer to the replicated object.*/
		Grammar * dynamicCopy(void);

		/*! Shows (by console) the internal state of the object 
		*/
		void showState(void);

		/*! Adds a new rule to the Grammar. It creates an object copy of the Rule passed as 
		 * parameter (and it is its owner). If isPublishable is not specified, it assumes the 
		 * rule is private.
		 *\param newRule pointer to the rule to be added in the Grammar.
		 *\param isPublishable true if the Rule is publishable, false if not.
		 */
		void addRule(Rule * newRule,bool isPublishable=false);

		/*! Looks in the Grammar for the Rule whose antecedent has the identifier antecedentId.
		 * If no rule exists with that antecedent, returns NULL.
		 *\param antecedentId antecedent identifier to look for.
		 *\return Pointer to the matching rule, or NULL if no matching rule exists.
		 */
		Rule * FindRuleFromAntecedentId(int antecedentId);

		/*! Checks if the Rule pointed by rulePtr is declared in the Grammar as publishable
		 *\param rulePtr Pointer to the rule to check
		 *\return true if the rule is publishable, false if not
		 */
		bool isPublishableRule(Rule * rulePtr);

		/*! Generates a string that contains all the rules of the Grammar. The string is 
		 * generated following the format specified by the input parameter. Returns an 
		 * empty string if the Grammar has no rules or in case of error. The output for
		 * mat supplied as parameter must be generated following the format (.jms) des
		 * cribed in Jafma_MacroScript.h and has to include a repetition macro for the
		 * rules and a repetition macro for the components of the rule. An example of
		 * an appropiate outputFormat string is found in "TemplateExample.jms", provided
		 * with this library. The zone of code used is named GRAMMAR_COMPOSITION and the 
		 * name of the used areas and macros are:
		 * <br>Rules_list: Repetition area used to iterate the rules.
		 * <br>Consequents_list: Repetition area used to iterate the Consequents into a rule.
		 * <br>Antecedent_name: Macro for the name of the antecedent of a rule.
		 * <br>is_NonTerminal: Condititional repetition macro to indicate if a Consequent is 
		 * a NonTerminal or not.
		 * <br>Consequent_name: Repetition macro indicating the name of a Consequent in a rule
		 * <br>is_Publishable: Macro to indicate if a rule is publishable or not
		 * <br>is_Transitive: Conditional repetition macro, it indicates, for NonTerminal if it
		 * is (the verb) transitive or not.
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
		 *
		 *\param outputFormat indicates the format for the output string.
		 *\result string associated with the Grammar.
		 */
		std::string getGrammarAsString(const std::string & outputFormat);

	private:
		/*! Copy method to properly replicate objects when using the copy constructor
		 * and the = operator.
		 *\param originGrammar Grammar object to replicate in this object.
		 */
		void copyGrammar(const Grammar &originGrammar);

		GrammarRules rules; /* list of rules for this Grammar */

}; /* end Grammar */

#endif