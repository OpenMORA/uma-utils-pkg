/*-------------------------------------------------*/
/*	GRAMMAR COMPONENTS LIBRARY FOR BABEL MODULES   */
/*                                                 */
/*	ECM, 2006                                      */
/*-------------------------------------------------*/

/** \file GrammarComponents.hpp
 *\brief Library with all the basic components of a generic grammar.
 */


#ifndef GRAMMARCOMPONENTS
	#define GRAMMARCOMPONENTS

#include <cstdio>  // printf,...
#include <cstdlib> // atoi,...
#include <string>

/*! \brief Basic elements to generate a grammar.
 *
 *  Symbols are the elements used to construct production rules for a grammar.
 */
class Symbol
{
	public:
		/*!\enum SymbolType
		 *\brief Defines the types of Symbol elements.
		 */
		typedef enum {TERMINAL = 0, /*!< Terminal symbol */
					  NONTERMINAL	/*!< NonTerminal symbol */
					 } SymbolType;

		/*! Allows the used to make a dynamic copy of the object, that is the method creates a copy of the object and returns a pointer to that copy.
		 \return pointer to the replicated object.*/
		virtual Symbol * dynamicCopy(void)=0;

		/*! Shows (by console) the internal state of the object
		*/
		virtual void showState(void)=0;

		/*! Returns the type of the Symbol.
		 *\return type of the current Symbol object.
		 */
		virtual SymbolType getType(void)= 0;
}; /* end Symbol */

/*! \brief Symbols (grammar elements) that can not be decomposed in more simple elements.
 *
 * Terminal objects are the simplest units of a grammar. They can't be decomposed and are used to construct more complex elements.
 */
class Terminal: public Symbol
{
	public:
		/*! Class constructor. Initializes the value of the terminal.
		 \param initValue initialization value for the terminal.
		*/
		Terminal(std::string initValue);

		/*! Copy constructor. Replicates the object passed as parameter
		 \param originTerminal Terminal object to replicate in this object.
		 */
		Terminal(const Terminal & originTerminal);

		/*! Class destructor
		*/
		~Terminal();

		/*! Overloaded equal operator to avoid malfunction when copying
		 *\param originTerminal Terminal object to replicate in this object.
		 */
		Terminal & operator = (const Terminal &originTerminal);

		/*! Allows the used to make a dynamic copy of the object, that is the method creates a copy of the object and returns a pointer to that copy.
		 \return pointer to the replicated object.*/
		Symbol * dynamicCopy(void);

		/*! Shows (by console) the internal state of the object
		*/
		void showState(void);

		/*! Returns the type of the Symbol.
		 *\return type of the current Symbol object.
		 */
		Symbol::SymbolType getType(void);

		/*! Returns a string containing the value of the terminal
		 \result value of the terminal
		*/
		std::string getValue(void);

	private:
		/*! Copy method to properly replicate objects when using the copy constructor
		 * and the = operator.
		 *\param originTerminal Terminal object to replicate in this object.
		 */
		void copyTerminal(const Terminal &originTerminal);

		std::string value; /* Terminal element represented by this class */

}; /* end Terminal */

/*!\brief Symbols (grammar elements) that can be decomposed in terminal elements.
 *
 * NonTerminal objects are simple elements of the grammar used to contruct more complex elements, but, opposite to Terminal objects, they can be decomposed in smaller units (Terminals).
 * The NonTerminal can be defined as Noun, Verb or as a NonTerminal used as Antecedent to complex Rule objects. A Noun is characterized by its genre and multiplicity, and a Verb by its
 * transitivity.
 */
class NonTerminal: public Symbol
{
	public:
		/*!\enum NonTerminalType
		 *\brief It indicates the type of NonTerminal it is (VERB, NOUN or OTHER)
		 */
		typedef enum{VERB, /*!< Verb NonTerminal */
					 NOUN, /*!< Noun NonTerminal */
					 OTHER /*!< Not specified */}NonTerminalType;

		/*!\enum NonTerminalGenre
		 *\brief It indicates the genre of the NonTerminal(masculine, femenine or
		 * not-specified).
		 * It's used for NonTerminals with NOUN modifier, for other cases it's set
		 * to G_NOT_USED.
		 */
		typedef enum{MASC, /*!< Genre = masculine */
					 FEM,  /*!< Genre = femenine */
					 G_NOT_USED}NonTerminalGenre;
		/*!\enum NonTerminalMultiplicity
		 *\brief It indicates the multiplicity of the NonTerminal(singular, plurar or
		 * not-specified).
		 * It's used for NonTerminals with NOUN modifier, for other cases it's set to
		 * M_NOT_USED.
		 */
		typedef enum{SINGULAR, /*!< Number = singular */
					 PLURAL,	/*!< Number = plural */
					 M_NOT_USED /*!< Number = not specified */}NonTerminalMultiplicity;

		/*!\enum NonTerminalTransitivity
		 *\brief It indicates the transitivity of the NonTerminal(trasitive, intransitive
		 * or not-specified).
		 * It's used for NonTerminals with VERB modifier, for other cases it's  set to
		 * T_NOT_USED.
		 */
		typedef enum{TRANSITIVE, /*!< Transitive verb */
					 INTRANSITIVE, /*!< Intransitive verb */
					 T_NOT_USED /*!< Transitivity not specified */}NonTerminalTransitivity;

		/*! Class constructor. Initializes the id of the NonTerminal. As NonTerminal no
		 * modifier is provided, the Object created is defined of type OTHER, that is
		 * as a NonTerminal used as antecedent to build rules.
		 *\param initId initalization value for the NonTerminal identifier
		 */
		NonTerminal(int initId);

		/*! Class constructor. Initializes the id of the NonTerminal. As the second
		 * is of type NonTerminalTransitivity, the NonTerminal will be created as VERB.
		 *\param initId initalization value for the NonTerminal identifier
		 *\param newTransitivity indicates the NonTerminal(verb) transitivity
		 */
		NonTerminal(int initId,NonTerminalTransitivity newTransitivity);

		/*! Class constructor. Initializes the id of the NonTerminal. As the second
		 * is of type NonTerminalGenre and the third of type NonTerminalMultiplicity,
		 * the NonTerminal will be created as NOUN.
		 *\param initId initalization value for the NonTerminal identifier
		 *\param newGenre indicates the NonTerminal(noun) genre
		 *\param newMultiplicity indicates the NonTerminal(noun) multiplicity
		 */
		NonTerminal(int initId,NonTerminalGenre newGenre,NonTerminalMultiplicity newMultiplicity);


		/*! Copy constructor. Replicates the object passed as parameter
		 \param originNonTerminal NonTerminal object to replicate in this object.
		 */
		NonTerminal(const NonTerminal & originNonTerminal);

		/*! Class destructor
		*/
		~NonTerminal();

		/*! Overloaded equal operator to avoid malfunction when copying
		 *\param originNonTerminal NonTerminal object to replicate in this object.
		 */
		NonTerminal & operator = (const NonTerminal &originNonTerminal);

		/*! Allows the used to make a dynamic copy of the object, that is the method creates a copy of the object and returns a pointer to that copy.
		 \return pointer to the replicated object.*/
		Symbol * dynamicCopy(void);

		/*! Shows (by console) the internal state of the object
		*/
		void showState(void);

		/*! Returns the type of the Symbol.
		 *\return type of the current Symbol object.
		 */
		Symbol::SymbolType getType(void);

		/*! Returns an integer containing the value of the non-terminal identifier.
		 \result value of the identifier
		*/
		int getId(void);

		/*! Returns the type of non-terminal stored in the NonTerminal object.
		 *\result type of the NonTerminal(noun, verb...)
		 */
		NonTerminalType getNTType(void);

		/*! Returns the genre of non-terminal stored in the NonTerminal object.
		 *\result genre of the NonTerminal(masculine,...)
		 */
		NonTerminalGenre getNTGenre(void);

		/*! Returns the multiplicity of non-terminal stored in the NonTerminal object.
		 *\result multiplicity of the NonTerminal(singular,...)
		 */
		NonTerminalMultiplicity getNTMultiplicity(void);

		/*! Returns the transitivity of non-terminal stored in the NonTerminal object.
		 *\result transitivity of the NonTerminal(transitive,...)
		 */
		NonTerminalTransitivity getNTTransitivity(void);

	private:
		/*! Copy method to properly replicate objects when using the copy constructor
		 * and the = operator.
		 *\param originNonTerminal NonTerminal object to replicate in this object.
		 */
		void copyNonTerminal(const NonTerminal &originNonTerminal);

		int id; /* NonTerminal identifier */
		NonTerminalType nt_type; /* NonTerminal type (noun, verb...)*/
		NonTerminalGenre genre;  /* NonTerminal genre (masculine...)*/
		NonTerminalMultiplicity multiplicity; /* NonTerminal multiplicity (singular...)*/
		NonTerminalTransitivity transitivity; /* NonTerminal transitivity (transitive...)*/
}; /* end NonTerminal */
#endif
