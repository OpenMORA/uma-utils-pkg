/*--------------------------------------------*/
/*	GRAMMAR RULES LIBRARY FOR BABEL MODULES   */
/*                                            */
/*	ECM, 2006                                 */
/*--------------------------------------------*/

/** \file GrammarRules.hpp
 *\brief Library for the rules of a generic grammar. 
 */

#ifndef GRAMMARRULES
	#define GRAMMARRULES

#include "GrammarComponents.hpp"
#include <list>

/*! \brief Rules that compose a grammar.
 *
 *  Rules are the components of grammars. They are built by the combination of Symbols: 
 * an Antecedent(a NonTerminal) and a set of Consequents( that can be Terminal or NonTerminal).
 * The set of Consequents can be a sequence or an OR expression.It the user wants the rule to
 * be an or Rule, it must be specified with the appropiate constructor, otherwise, the rule is
 * assumed to be a sequence expression. The Rule can be declared as publishable, which means 
 * that is visible outside the grammar and can be used to recognition or as unpublishable, that
 * is, visible only in the scope of the grammar.
 */
class Rule
{
	public:
		/*! \brief Rule antecedent.
		 * Only NonTerminal symbols allowed.
		 */
		typedef NonTerminal * Antecedent;

		/*! \brief List of Symbol objects that form the Consequents of a rule.
		 *
		 * Both Terminal and NonTerminal symbols allowed. The list is composed by a set of pointers to Symbol objects.
		 */
		typedef std::list<Symbol *> Consequents;
		
		/*! Class constructor. If no value its specified for newisOr, creates a sequential rule.	
		 *\param newisOr true for conditional rules, o for sequence
		 */
		Rule(bool newisOr=false);
		
		/*! Copy constructor. Replicates the object passed as parameter
		 \param originRule Rule object to replicate in this object. It also replicates components of the rule.
		 */
		Rule(const Rule & originRule);

		/*! Class destructor
		 */
		~Rule();

		/*! Overloaded equal operator to avoid malfunction when copying 
		 *\param originRule Rule object to replicate in this object.
		 */
		Rule & operator = (const Rule &originRule);

		/*! Allows the used to make a dynamic copy of the object, that is the method creates a copy of the object and returns a pointer to that copy.
		 \return pointer to the replicated object.*/
		Rule * dynamicCopy(void);

		/*! Shows (by console) the internal state of the object 
		*/
		void showState(void);
		
		/*! Shows if the rule is declared as sequence or as Or rule.
		 *\return false for secuence rules, true for or rules
		 */
		bool isOR(void);

		/*! Adds an Antecedent to the current rule.
		 *  The method creates a copy of the Antecedent object(and will be his owner). If the Antecedent was already set, it overwrites it.
		 *\param newAntecedent Antecedent to insert.
		 */
		void setAntecedent(const Antecedent & newAntecedent);

		/*! Gets a pointer to the Antecedent of the current rule. 
		 *\return antecedentPtr pointer to the Antecedent
		 */
		Antecedent getAntecedent(void);

		/*! Adds a Consequent to the current rule. The method creates a copy of the Consequent object(and will be his owner).
		 * Returns TRUE if the Consequent was added succesfully, FALSE otherwise.
		 *\param newConsequent
		 */
		bool addConsequent(Symbol * newConsequent);

		/*! Gets the number of Consequents in the rule. The indexes go from 0 to n-1.
		 *\return number of current Consequents stored in the rule.
		 */
		unsigned getNumberOfConsequents(void);

		/*! Get the Consequent stored at position n. The indexes go from 0 to n-1, being n the number of Consequents.
		If any error occurs, returns NULL
		 *\param n position to return the Consequent.
		 *\return Pointer to the Consequent at position n.
		 */
		Symbol * ConsequentAt(unsigned n);

	private:
		/*! Copy method to properly replicate objects when using the copy constructor
		 * and the = operator.
		 *\param originRule Rule object to replicate in this object.
		 */
		void copyRule(const Rule &originRule);

		Antecedent ruleAntecedent; 
		Consequents ruleConsequents;
		bool isOr; 
}; /* end Rule */
#endif