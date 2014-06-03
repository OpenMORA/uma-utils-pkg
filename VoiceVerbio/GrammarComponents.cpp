/*-------------------------------------------------*/
/*	GRAMMAR COMPONENTS LIBRARY FOR BABEL MODULES   */
/*                                                 */
/*	ECM, 2006                                      */
/*-------------------------------------------------*/

/** \file GrammarComponents.cpp
 *\brief Implementation of GrammarComponents.hpp
 */

#include "GrammarComponents.hpp"
using namespace std;

/*----------------*
 * Terminal class *
 *----------------*/
Terminal::Terminal(string initValue)
{
	value = initValue;
}

Terminal::Terminal(const Terminal & originTerminal)
{
	copyTerminal(originTerminal);
}

Terminal::~Terminal()
{
}

Terminal & Terminal::operator =(const Terminal &originTerminal)
{
	copyTerminal(originTerminal);
	return(*this);
}



Symbol * Terminal::dynamicCopy()
{
	Terminal * copiedTerminal = new Terminal(*this); //Replica Object
	return(copiedTerminal);
}

void Terminal::showState()
{
	printf("Terminal state: Terminal value = %s\n",value.c_str());
}

Symbol::SymbolType Terminal::getType()
{
	return(TERMINAL);
}

string Terminal::getValue()
{
	return(value);
}

void Terminal::copyTerminal(const Terminal &originTerminal)
{
	value = originTerminal.value;
}
/* end Terminal class */

/*-------------------*
 * NonTerminal class *
 *-------------------*/
NonTerminal::NonTerminal(int initId)
{
	id = initId;
	nt_type = NonTerminal::OTHER;
	genre = NonTerminal::G_NOT_USED;
	multiplicity = NonTerminal::M_NOT_USED;
	transitivity = NonTerminal::T_NOT_USED;
}

NonTerminal::NonTerminal(int initId, NonTerminal::NonTerminalTransitivity newTransitivity)
{
	id = initId;
	nt_type = NonTerminal::VERB;
	genre = NonTerminal::G_NOT_USED;
	multiplicity = NonTerminal::M_NOT_USED;
	transitivity = newTransitivity;
}

NonTerminal::NonTerminal(int initId, NonTerminal::NonTerminalGenre newGenre, NonTerminal::NonTerminalMultiplicity newMultiplicity)
{
	id = initId;
	nt_type = NonTerminal::NOUN;
	genre = newGenre;
	multiplicity = newMultiplicity;
	transitivity = NonTerminal::T_NOT_USED;
}
NonTerminal::NonTerminal(const NonTerminal & originNonTerminal)
{
	copyNonTerminal(originNonTerminal);
}

NonTerminal::~NonTerminal()
{
}

NonTerminal & NonTerminal::operator =(const NonTerminal &originNonTerminal)
{
	copyNonTerminal(originNonTerminal);
	return(*this);
}

Symbol * NonTerminal::dynamicCopy()
{
	NonTerminal * copiedNonTerminal = new NonTerminal(*this); //Replica Object	
	return(copiedNonTerminal);
}
		
void NonTerminal::showState()
{
	const char * TypeNames[] ={"VERB","NOUN","OTHER"};
	const char * GenreNames[] ={"MASC", "FEM", "G_NOT_USED"};
	const char * MultiplicityNames[] ={"SINGULAR", "PLURAL", "M_NOT_USED"};
	const char * TransitivityNames[] ={"TRANSITIVE", "INTRANSITIVE", "T_NOT_USED"};
	printf("NonTerminal state: NonTerminal id = %u; type = %s; genre = %s; multiplicity = %s; transitivity = %s\n",id,TypeNames[nt_type], GenreNames[genre],MultiplicityNames[multiplicity],TransitivityNames[transitivity]);
}

Symbol::SymbolType NonTerminal::getType()
{
	return(NONTERMINAL);
}

int NonTerminal::getId()
{
	return(id);
}

NonTerminal::NonTerminalType NonTerminal::getNTType()
{
	return(nt_type);
}

NonTerminal::NonTerminalGenre NonTerminal::getNTGenre()
{
	return(genre);
}

NonTerminal::NonTerminalMultiplicity NonTerminal::getNTMultiplicity()
{
	return(multiplicity);
}

NonTerminal::NonTerminalTransitivity NonTerminal::getNTTransitivity()
{
	return(transitivity);
}

void NonTerminal::copyNonTerminal(const NonTerminal &originNonTerminal)
{
	id = originNonTerminal.id;
	nt_type = originNonTerminal.nt_type;
	genre = originNonTerminal.genre;
	multiplicity = originNonTerminal.multiplicity;
	transitivity = originNonTerminal.transitivity;
}
/* end NonTerminal */


