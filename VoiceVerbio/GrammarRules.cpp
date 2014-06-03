/*--------------------------------------------*/
/*	GRAMMAR RULES LIBRARY FOR BABEL MODULES   */
/*                                            */
/*	ECM, 2006                                 */
/*--------------------------------------------*/

/** \file GrammarRules.cpp
 *\brief Implementation of GrammarRules.hpp
 */

#include <cstdio>
#include "GrammarRules.hpp"

/*------------*
 * Rule class *
 *------------*/

Rule::Rule(bool newisOr)
{
	ruleAntecedent = NULL;
	isOr = newisOr;
}

Rule::Rule(const Rule & originRule)
{
	copyRule(originRule);
}

Rule::~Rule()
{
}

Rule & Rule::operator =(const Rule &originRule)
{
	copyRule(originRule);
	return(*this);
}

Rule * Rule::dynamicCopy()
{
	Rule * copiedRule = new Rule(*this); //Replica Object
	return(copiedRule);
}

void Rule::showState()
{
	printf("Rule state:\nAntecedent:\n");
	if(ruleAntecedent != NULL)
	{
		ruleAntecedent->showState();
	}
	else
	{
		printf("The Antecedent is empty.\n");
	}
	Consequents::iterator ConsequentsIter;
	ConsequentsIter = ruleConsequents.begin();
	printf("Consequents:\n");
	while(ConsequentsIter != ruleConsequents.end())
	{
		(*ConsequentsIter)->showState();
		ConsequentsIter++;
	}

	if(isOr)
	{
		printf("OR Rule\n");
	}
	else
	{
		printf("Sequence Rule\n");
	}
}

bool Rule::isOR()
{
	return(isOr);
}

void Rule::setAntecedent(const Antecedent & newAntecedent)
{
	ruleAntecedent = (NonTerminal *)newAntecedent->dynamicCopy();
}

Rule::Antecedent Rule::getAntecedent()
{
	return(ruleAntecedent);
}

bool Rule::addConsequent(Symbol *newConsequent)
{
	try
	{
		ruleConsequents.push_back(newConsequent->dynamicCopy());
		return(true);
	}
	catch(...)
	{
		return(false);
	}
}

unsigned Rule::getNumberOfConsequents()
{
	return((unsigned)ruleConsequents.size());
}

Symbol * Rule::ConsequentAt(unsigned n)
{
	Consequents::iterator ConsequentsIter;
	if((n < 0)||(n>= ruleConsequents.size()))
	{
		return(NULL);
	}
	else
	{
		try
		{
			ConsequentsIter = ruleConsequents.begin();
			for(unsigned i = 1;i <= n;i++)
			{
				ConsequentsIter++;
			}
			return(*ConsequentsIter);
		}
		catch(...)
		{
			return(NULL);
		}
	}

}

void Rule::copyRule(const Rule &originRule)
{
	//Replication of the antecedent
	ruleAntecedent = (NonTerminal*)originRule.ruleAntecedent->dynamicCopy();
	//Replication of the list of Consequent
	Consequents::const_iterator originConsequentsIter; //Iterator of the object being copied
	originConsequentsIter = originRule.ruleConsequents.begin();
	while(originConsequentsIter != originRule.ruleConsequents.end())
	{
		ruleConsequents.push_back((*originConsequentsIter)->dynamicCopy());
		originConsequentsIter++;
	}
	isOr = originRule.isOr;
}
/* end Rule class*/
