/*--------------------------------------*/
/*	GRAMMAR LIBRARY FOR BABEL MODULES   */
/*                                      */
/*	ECM, 2006                           */
/*--------------------------------------*/

/** \file Grammar.cpp
 *\brief Implementation of Grammar.hpp
 */

#include "Grammar.hpp"
#include "Jafma_MacroScript"

using namespace std;
using namespace JAFMA;


/*---------------*
 * Grammar class *
 *---------------*/

Grammar::Grammar()
{
}

Grammar::Grammar(const Grammar &originGrammar)
{
	copyGrammar(originGrammar);
}

Grammar::~Grammar()
{
}

Grammar & Grammar::operator =(const Grammar &originGrammar)
{
	copyGrammar(originGrammar);
	return(*this);
}

Grammar * Grammar::dynamicCopy()
{
	Grammar * copiedGrammar = new Grammar(*this); //Replica Object
	return(copiedGrammar);
}

void Grammar::showState()
{
	//Indicate the status of each rule object
	GrammarRules::iterator rulesIter;
	rulesIter = rules.begin();
	while(rulesIter != rules.end())
	{
		(*rulesIter).ruleObjectPtr->showState();
		if((*rulesIter).isPublishable)
		{
			printf("Publishable rule\n");
		}
		else
		{
			printf("Unpublishable rule\n");
		}
		rulesIter++;		
	}
}

void Grammar::addRule(Rule * newRule,bool isPublishable)
{
	GrammarRule currentRule;
	currentRule.ruleObjectPtr = newRule->dynamicCopy();
	currentRule.isPublishable = isPublishable;
	rules.push_back(currentRule);
}

Rule * Grammar::FindRuleFromAntecedentId(int antecedentId)
{
	GrammarRules::iterator rulesIter;
	rulesIter = rules.begin();
	while(rulesIter != rules.end())
	{
		if((*rulesIter).ruleObjectPtr->getAntecedent()->getId() == antecedentId)
		{
			return((*rulesIter).ruleObjectPtr);
		}
		rulesIter++;
	}
	return(NULL);
}

bool Grammar::isPublishableRule(Rule * rulePtr)
{
	GrammarRules::iterator rulesIter;
	rulesIter = rules.begin();
	while(rulesIter != rules.end())
	{		
		if((*rulesIter).ruleObjectPtr == rulePtr)
		{			
			return((*rulesIter).isPublishable);
		}
		rulesIter++;
	}
	return(false);
}
string Grammar::getGrammarAsString(const string & outputFormat)
{
	string outputString;
	MacroScript outputTemplate(outputFormat);	

	//Reading of the rules and registration of the macros
	GrammarRules::iterator rulesIter;
	rulesIter = rules.begin();
	unsigned ruleConsequentsNumber;
	int antecedentId;
	string antecedentString;
	string rulesIndexString;
	string nonTerminalString;	
	string ConsequentIndexString;
	NonTerminal * ntPtr; //Auxiliary pointers to pick up NonTerminal and Terminal Symbols
	Terminal * tPtr;	 //from the Rule.	
	unsigned rulesIndex = 0;
	while(rulesIter != rules.end())
	{				
		//Get the Antecedent of the rule
		antecedentId = (*rulesIter).ruleObjectPtr->getAntecedent()->getId();	
		number_to_string(antecedentId,antecedentString);
		number_to_string(rulesIndex,rulesIndexString);
		outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Rules_list","Antecedent_name","[Rules_list#"+rulesIndexString+"]",antecedentString);
		if((*rulesIter).isPublishable)
		{
			outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Rules_list","is_Publishable","[Rules_list#"+rulesIndexString+"]","true");
		}	
		ruleConsequentsNumber = (*rulesIter).ruleObjectPtr->getNumberOfConsequents();						
		for(unsigned i = 0;i<ruleConsequentsNumber;i++)
		{					
			number_to_string(i,ConsequentIndexString);						
			switch((*rulesIter).ruleObjectPtr->ConsequentAt(i)->getType())
			{
				case Symbol::NONTERMINAL: ntPtr=dynamic_cast<NonTerminal *>((*rulesIter).ruleObjectPtr->ConsequentAt(i));	
										  number_to_string(ntPtr->getId(),nonTerminalString);																					  											
										  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","Consequent_name","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]",nonTerminalString);								
										  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_NonTerminal","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
										  if(ntPtr->getNTTransitivity() == NonTerminal::TRANSITIVE)
										  {
											outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_Transitive","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");										  
										  }
										  if((ntPtr->getNTGenre()== NonTerminal::MASC))
										  {
											  if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
											  {
												  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_MascSing","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
											  }
											  else
											  {
												  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_MascPlural","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
											  }
										  }
										  if((ntPtr->getNTGenre()== NonTerminal::FEM))
										  {
											  if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
											  {
												  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_FemSing","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
											  }
											  else
											  {
												  outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","is_FemPlural","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
											  }
										  }
										  break;
				case Symbol::TERMINAL: tPtr = dynamic_cast<Terminal *>((*rulesIter).ruleObjectPtr->ConsequentAt(i));	
									   outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","Consequent_name","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]",tPtr->getValue());
									   break;						
			} //end SWITCH		
			if((*rulesIter).ruleObjectPtr->isOR() && (i < (ruleConsequentsNumber-1)))
			{
				outputTemplate.RegisterRepetitionMacroValue("GRAMMAR_COMPOSITION","Consequents_list","OR_Rule","[Rules_list#"+rulesIndexString+"][Consequents_list#"+ConsequentIndexString+"]","true");
			}		
		} //end FOR				
		rulesIter++;
		rulesIndex++;					
	}			
	if(!outputTemplate.ApplyMacros("GRAMMAR_COMPOSITION",outputString))	
	{		
		printf("Error aplying the macros:%s\n",outputString.c_str());
		outputString = "";		
	}
	return(outputString);
}

void Grammar::copyGrammar(const Grammar &originGrammar)
{
	//Generate a copy of each Rule object
	GrammarRule currentRule;
	GrammarRules::const_iterator originRulesIter;
	originRulesIter = originGrammar.rules.begin();
	while(originRulesIter != originGrammar.rules.end())
	{
		currentRule.ruleObjectPtr = (*originRulesIter).ruleObjectPtr->dynamicCopy();
		currentRule.isPublishable = (*originRulesIter).isPublishable;
		rules.push_back(currentRule);		
		originRulesIter++;		
	}
}
/*end Grammar class */

