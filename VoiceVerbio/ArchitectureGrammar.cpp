/*---------------------------------------------------*/
/*	ARCHITECTURE GRAMMAR LIBRARY FOR BABEL MODULES   */
/*                                                   */
/*	ECM, 2006                                        */
/*---------------------------------------------------*/

/** \file ArchitectureGrammar.cpp
 *\brief Implementation of ArchitectureGrammar.hpp
 */

#include "ArchitectureGrammar.hpp"
#include "Jafma_MacroScript"
#include <sstream>

#ifdef MSC_VER
#pragma warning(disable:4786)
#endif

using namespace std;
using namespace JAFMA;

bool isOnSet(const string antecedentName, const list<string>rulesList)
{
	list<string>::const_iterator rulesIter;

	rulesIter =rulesList.begin();
	while(rulesIter != rulesList.end())
	{
		if((*rulesIter) == antecedentName)
		{
			return(true);
		}
		rulesIter++;
	}
	return(false);
}

/*-----------------*
 * WordsList class *
 *-----------------*/
ArchitectureGrammar::WordsList::WordsList()
{
}

ArchitectureGrammar::WordsList::WordsList(const ArchitectureGrammar::WordsList & originWordsList)
{
	copyWordsList(originWordsList);
}

ArchitectureGrammar::WordsList::~WordsList()
{
}

ArchitectureGrammar::WordsList & ArchitectureGrammar::WordsList::operator =(const ArchitectureGrammar::WordsList &originWordsList)
{
	copyWordsList(originWordsList);
	return(*this);
}

void ArchitectureGrammar::WordsList::addWord(const string &word)
{
	listOfWords.push_back(word);
}

string ArchitectureGrammar::WordsList::getWordAt(const words::iterator &position)
{
	return((*position));
}

void ArchitectureGrammar::WordsList::eraseWordAt(const words::iterator & position)
{
	listOfWords.erase(position);
}

void ArchitectureGrammar::WordsList::clear()
{
	listOfWords.clear();
}

ArchitectureGrammar::WordsList::words::const_iterator ArchitectureGrammar::WordsList::begin() const
{
	return(listOfWords.begin());
}

ArchitectureGrammar::WordsList::words::const_iterator ArchitectureGrammar::WordsList::end() const
{
	return(listOfWords.end());
}

bool ArchitectureGrammar::WordsList::empty() const
{
	return(listOfWords.empty());
}

void ArchitectureGrammar::WordsList::copyWordsList(const ArchitectureGrammar::WordsList & originWordsList)
{
	listOfWords = originWordsList.listOfWords;
}
/* end WordsList class */

/*-------------------*
 * ObjectNouns class *
 *-------------------*/
ArchitectureGrammar::ObjectNouns::ObjectNouns()
{
}

ArchitectureGrammar::ObjectNouns::ObjectNouns(const ArchitectureGrammar::ObjectNouns & originObjectNouns)
{
	copyObjectNouns(originObjectNouns);
}
ArchitectureGrammar::ObjectNouns::~ObjectNouns()
{
}

ArchitectureGrammar::ObjectNouns & ArchitectureGrammar::ObjectNouns::operator =(const ArchitectureGrammar::ObjectNouns & originObjectNouns)
{
	copyObjectNouns(originObjectNouns);
	return(*this);
}

void ArchitectureGrammar::ObjectNouns::addNoun2Object(const ArchitectureGrammar::ObjectNouns::nounInformation &nounInfo)
{
	nounsList.push_back(nounInfo);
}

ArchitectureGrammar::ObjectNouns::nounInformation ArchitectureGrammar::ObjectNouns::getNounAt(const list<nounInformation>::iterator &position)
{
	return((*position));
}

void ArchitectureGrammar::ObjectNouns::eraseNounAt(const nounInformationList::iterator & position)
{
	nounsList.erase(position);
}

void ArchitectureGrammar::ObjectNouns::clear()
{
	nounsList.clear();
}

ArchitectureGrammar::ObjectNouns::nounInformationList::const_iterator ArchitectureGrammar::ObjectNouns::begin() const
{
	return(nounsList.begin());
}

ArchitectureGrammar::ObjectNouns::nounInformationList::const_iterator ArchitectureGrammar::ObjectNouns::end() const
{
	return(nounsList.end());
}

void ArchitectureGrammar::ObjectNouns::copyObjectNouns(const ArchitectureGrammar::ObjectNouns & originObjectNouns)
{
	nounsList = originObjectNouns.nounsList;
}
/* end ObjectNouns class */

/*---------------------------*
 * ArchitectureGrammar class *
 *---------------------------*/
ArchitectureGrammar::ArchitectureGrammar()
{
	associatedGrammar = Grammar();
	/* The first NonTerminal index is associated to the free string rule */
	currentNonTerminalIndex = FREE_STRING_ID +1;
}

ArchitectureGrammar::~ArchitectureGrammar()
{
}

void ArchitectureGrammar::showState()
{
	associatedGrammar.showState();
}

bool ArchitectureGrammar::addNewObject(const string & newObjectLabel,const ObjectNouns & newObjectInfo,const string & associatedType)
{
	ArchitectureGrammar::ObjectNouns::nounInformationList::const_iterator objectIter;
	list<objectAssociation>::iterator objectNameIter;
	list<int> objectAntecedentsIds; //Identifiers of the antecedents associated with this object
	//Check if an object with the same label is included in the list of objects
	if(findObjectInList(newObjectLabel,objectNameIter))
	{
		printf("Error: There is another object named %s. The new object won't be included in the grammar.\n",newObjectLabel.c_str());
		return(false);
	}
	Rule globalObjectRule(true);
	NonTerminal globalObjectAntecedent(currentNonTerminalIndex);
	currentNonTerminalIndex++;
	globalObjectRule.setAntecedent(&globalObjectAntecedent);
	//Include the antecedent of the rule in the list of objects associated rules:
	objectAntecedentsIds.push_back(globalObjectAntecedent.getId());
	objectAssociation objectNameInfo;
	objectNameInfo.elementName = newObjectLabel;
	objectNameInfo.typeName = associatedType;
	objectNameInfo.associatedNtIds = objectAntecedentsIds;
	objectsAssociations.push_back(objectNameInfo);
	try
	{
		//Get the iterator to include the object information in the list of associations
		findObjectInList(newObjectLabel,objectNameIter);
		objectIter = newObjectInfo.begin();
		//Building of the rule
		while(objectIter != newObjectInfo.end())
		{
			Terminal newConsecuent(objectIter->associatedWord.c_str());
			NonTerminal newAntecedent(currentNonTerminalIndex,objectIter->genre,objectIter->number);
			Rule newRule;
			newRule.setAntecedent(&newAntecedent);
			newRule.addConsequent(&newConsecuent);
			associatedGrammar.addRule(&newRule);
			(*objectNameIter).associatedNtIds.push_back(newAntecedent.getId());
			currentNonTerminalIndex++;
			globalObjectRule.addConsequent(&newAntecedent);
			objectIter++;
		}
		associatedGrammar.addRule(&globalObjectRule);
		//Search for the type
		int typeRuleId = findTypeIdentifier(associatedType);
		if(typeRuleId != 0)
		{
			associatedGrammar.FindRuleFromAntecedentId(typeRuleId)->addConsequent(&globalObjectAntecedent);
		}
		else //Type not found in the list, create new rule
		{
			Rule newTypeRule(true); //We declare the Rule as OR rule.
			NonTerminal newTypeRuleAntecedent(currentNonTerminalIndex);
			currentNonTerminalIndex++;
			newTypeRule.setAntecedent(&newTypeRuleAntecedent);
			newTypeRule.addConsequent(&globalObjectAntecedent);
			associatedGrammar.addRule(&newTypeRule);
			//Add the element to the list of types
			typeAssociation newAssociation;
			newAssociation.typeName = associatedType;
			newAssociation.ntId = newTypeRuleAntecedent.getId();
			grammarAssociations.push_back(newAssociation);
		}
		return(true);
	}
	catch(...)
	{
		return(false);
	}
}

bool ArchitectureGrammar::addNewAction(const std::string & newActionLabel, const WordsList & newActionWords,const WordsList & associatedParams, NonTerminal::NonTerminalTransitivity transitivity)
{
	list<string>::const_iterator actionIter;
	list<string>::const_iterator paramIter;
	int typeRuleId;
	list<actionAssociation>::iterator actionNameIter;
	list<int> actionAntecedentsIds; //Identifiers of the antecedents associated with this action

	//Check if an action with the same label is included in the list of actions
	if(findActionInList(newActionLabel,actionNameIter))
	{
		printf("Error: There is another action with the tag %s. The new action won't be included in the grammar.\n",newActionLabel.c_str());
		return(false);
	}

	//Check if the parameters of the rule ar included in the grammar
	if(!associatedParams.empty())
	{
		paramIter = associatedParams.begin();
		while(paramIter != associatedParams.end())
		{
			if(!findTypeIdentifier((*paramIter)))
			{
				//Add the free string rule if it is not defined yet
				if(!(*paramIter).compare(FREE_STRING))
				{
					Rule freeStringRule;
					NonTerminal freeStringAntecedent(FREE_STRING_ID);
					//ECM: PROVISONAL!! The handling of free strings is not yet designed
					Terminal freeStringTerminal(FREE_STRING);
					freeStringRule.setAntecedent(&freeStringAntecedent);
					freeStringRule.addConsequent(&freeStringTerminal);
					associatedGrammar.addRule(&freeStringRule);
					//Add the type association to the list of type associations
					typeAssociation newAssociation;
					newAssociation.typeName = FREE_STRING;
					newAssociation.ntId = FREE_STRING_ID;
					grammarAssociations.push_back(newAssociation);
				}
				else
				{
					printf("Sorry, one of the parameters needed to execute %s action are unrecognized. The action won't be included in the grammar\n",newActionLabel.c_str());
					printf("%s\n",(*paramIter).c_str());
					return(false);
				}
			}
			paramIter++;
		}
	}
	Rule globalActionRule(true);
	NonTerminal globalActionAntecedent(currentNonTerminalIndex,transitivity);
	currentNonTerminalIndex++;
	globalActionRule.setAntecedent(&globalActionAntecedent);
	//Include the antecedent of the rule in the list of actions associated rules:
	actionAntecedentsIds.push_back(globalActionAntecedent.getId());
	actionAssociation actionNameInfo;
	actionNameInfo.elementName = newActionLabel;
	actionNameInfo.associatedNtIds = actionAntecedentsIds;
	actionsAssociations.push_back(actionNameInfo);
	try
	{
		//Get the iterator to include the action information in the list of associations
		findActionInList(newActionLabel,actionNameIter);
		actionIter = newActionWords.begin();
		//Building of the rule
		while(actionIter != newActionWords.end())
		{
			Terminal newConsecuent((*actionIter).c_str());
			NonTerminal newAntecedent(currentNonTerminalIndex);
			Rule newRule;
			newRule.setAntecedent(&newAntecedent);
			newRule.addConsequent(&newConsecuent);
			associatedGrammar.addRule(&newRule);
			(*actionNameIter).associatedNtIds.push_back(newAntecedent.getId());
			currentNonTerminalIndex++;
			globalActionRule.addConsequent(&newAntecedent);
			actionIter++;
		}
		associatedGrammar.addRule(&globalActionRule,associatedParams.empty());
		if(!associatedParams.empty())
		{
			//Compose the rule with the parameters
			//Rule creation
			Rule actionParamsRule;
			NonTerminal actionParamsAntecedent(currentNonTerminalIndex);
			currentNonTerminalIndex++;
			actionParamsRule.setAntecedent(&actionParamsAntecedent);
			actionParamsRule.addConsequent(&globalActionAntecedent);
			paramIter = associatedParams.begin();
			while(paramIter != associatedParams.end())
			{
				//Search for the type
				typeRuleId = findTypeIdentifier((*paramIter).c_str());
				actionParamsRule.addConsequent(associatedGrammar.FindRuleFromAntecedentId(typeRuleId)->getAntecedent());
				paramIter++;
			}
			//Insert the rule as publishable
			associatedGrammar.addRule(&actionParamsRule,true);
			(*actionNameIter).associatedNtIds.push_back(actionParamsAntecedent.getId());
		}
		return(true);
	}
	catch(...)
	{
		return(false);
	}
}

string ArchitectureGrammar::getGrammarAsString(const string & outputFormat)
{
	string outputString;
	MacroScript outputTemplate(outputFormat);

	//Objects insertion:
	//Types:
	unsigned objectTypesIndex = 0;
	string objectTypesnIndexString;
	list<typeAssociation>::iterator typesAssociationsIter;
	string currentTypeName;
	unsigned objectNamesIndex = 0;
	string objectNamesIndexString;
	list<objectAssociation>::iterator objectsAssociationsIter;
	string currentObjectName;
	list<int>::iterator objAntecedentsIdsIter;
	int currentAntecedentId;
	Rule * currentRule;
	string antecedentString;
	string rulesIndexString;
	unsigned objRulesIndex = 0;
	string objRulesIndexString;
	unsigned ruleConsequentsNumber;
	NonTerminal * ntPtr; //Auxiliary pointers to pick up NonTerminal and Terminal Symbols
	Terminal * tPtr;	 //from the Rule.
	string nonTerminalString;
	string consequentIndexString;

	typesAssociationsIter = grammarAssociations.begin();
	while(typesAssociationsIter != grammarAssociations.end())
	{
		//Get the name of the type:
		currentTypeName = (*typesAssociationsIter).typeName;
		number_to_string(objectTypesIndex,objectTypesnIndexString);
		outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","ObjectType_rules","Type_name","[ObjectType_rules#"+objectTypesnIndexString+"]",currentTypeName);
		typesAssociationsIter++;
		objectTypesIndex++;
		//Get the objects of the given type:
		objectNamesIndex = 0;
		objectsAssociationsIter = objectsAssociations.begin();
		while(objectsAssociationsIter != objectsAssociations.end())
		{
			//Check if the object is of the current type
			if((*objectsAssociationsIter).typeName.compare(currentTypeName) == 0)
			{
				currentObjectName = (*objectsAssociationsIter).elementName;
				number_to_string(objectNamesIndex,objectNamesIndexString);
				outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Object_rules","Object_label","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"]",currentObjectName);
				//Registration of the rules associated with the object:
				objAntecedentsIdsIter = (*objectsAssociationsIter).associatedNtIds.begin();
				objRulesIndex = 0;
				while(objAntecedentsIdsIter !=(*objectsAssociationsIter).associatedNtIds.end())
				{
					currentAntecedentId = *objAntecedentsIdsIter;
					currentRule = associatedGrammar.FindRuleFromAntecedentId(currentAntecedentId);
					//Analyse the rule and insert its components into the string:
					//Insert the antecedent of the rule into the string
					number_to_string(currentAntecedentId,antecedentString);
					number_to_string(objRulesIndex,objRulesIndexString);
					outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Object_relative_rules","Antecedent_name","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"]",antecedentString);
					//Get the consecuents of the rule and insert them into the string
					ruleConsequentsNumber = currentRule->getNumberOfConsequents();
					for(unsigned i = 0;i<ruleConsequentsNumber;i++)
					{
						number_to_string(i,consequentIndexString);
						switch(currentRule->ConsequentAt(i)->getType())
						{
							case Symbol::NONTERMINAL: ntPtr=dynamic_cast<NonTerminal *>(currentRule->ConsequentAt(i));
													  number_to_string(ntPtr->getId(),nonTerminalString);
													  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","Consequent_name","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]",nonTerminalString);
													  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","is_NonTerminal","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
													  //Genre & number
													  if((ntPtr->getNTGenre()== NonTerminal::MASC))
													  {
														if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
														{
															outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","is_MascSing","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
														}
														else
														{
															outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","is_MascPlural","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
														}
													}
													if((ntPtr->getNTGenre()== NonTerminal::FEM))
													{
														if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
														{
															outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","is_FemSing","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
														}
														else
														{
															outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","is_FemPlural","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
														}
													}
													break;
							case Symbol::TERMINAL: tPtr = dynamic_cast<Terminal *>(currentRule->ConsequentAt(i));
												   outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","Consequent_name","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]",tPtr->getValue());
												   break;
						} //end SWITCH
						if(currentRule->isOR() && (i < (ruleConsequentsNumber-1)))
						{
							outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Obj_RelRule_consequents_list","OR_Rule","[ObjectType_rules#"+objectTypesnIndexString+"][Object_rules#"+objectNamesIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Obj_RelRule_consequents_list#"+consequentIndexString+"]","true");
						}
					} //end FOR
					objAntecedentsIdsIter++;
					objRulesIndex++;
				}
				objectNamesIndex++;
			}
			objectsAssociationsIter++;
		}
		//Registration of the macros for the global rule:
		currentRule = associatedGrammar.FindRuleFromAntecedentId(findTypeIdentifier(currentTypeName));
		number_to_string(findTypeIdentifier(currentTypeName),antecedentString);
		outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_relative_rules","Antecedent_name","[ObjectType_rules#"+objectTypesnIndexString+"]",antecedentString);
		//Get the consecuents of the rule and insert them into the string
		ruleConsequentsNumber = currentRule->getNumberOfConsequents();
		for(unsigned i = 0;i<ruleConsequentsNumber;i++)
		{
			number_to_string(i,consequentIndexString);
			switch(currentRule->ConsequentAt(i)->getType())
			{
				case Symbol::NONTERMINAL: ntPtr=dynamic_cast<NonTerminal *>(currentRule->ConsequentAt(i));
										  number_to_string(ntPtr->getId(),nonTerminalString);
										  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","Consequent_name","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]",nonTerminalString);
										  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","is_NonTerminal","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
										  //Genre & number
										  if((ntPtr->getNTGenre()== NonTerminal::MASC))
										  {
											if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
											{
												outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","is_MascSing","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
											}
											else
											{
												outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","is_MascPlural","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
											}
										}
										if((ntPtr->getNTGenre()== NonTerminal::FEM))
										{
											if(ntPtr->getNTMultiplicity() == NonTerminal::SINGULAR)
											{
												outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","is_FemSing","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
											}
											else
											{
												outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","is_FemPlural","[ObjectType_rules#"+objectTypesnIndexString+"][Object_relative_rules#"+objRulesIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
											}
										}
										break;
				case Symbol::TERMINAL: tPtr = dynamic_cast<Terminal *>(currentRule->ConsequentAt(i));
									   outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","Consequent_name","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]",tPtr->getValue());
									   break;
			} //end SWITCH
			if(currentRule->isOR() && (i < (ruleConsequentsNumber-1)))
			{
				outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Type_RelRule_consequents_list","OR_Rule","[ObjectType_rules#"+objectTypesnIndexString+"][Type_RelRule_consequents_list#"+consequentIndexString+"]","true");
			}
		} //end FOR
	}

	//Actions insertion:
	string currentActionName;
	unsigned actionNamesIndex = 0;
	string actionNamesIndexString;
	list<actionAssociation>::iterator actionsIter;
	list<int>::iterator actAntecedentsIdsIter;
	unsigned actRulesIndex = 0;
	string actRulesIndexString;
	actionsIter = actionsAssociations.begin();
	while(actionsIter != actionsAssociations.end())
	{
		currentActionName = (*actionsIter).elementName;
		number_to_string(actionNamesIndex,actionNamesIndexString);
		outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Actions_list","Action_name","[Actions_list#"+actionNamesIndexString+"]",currentActionName);
		//Registration of the rules associated with the action:
		actAntecedentsIdsIter = (*actionsIter).associatedNtIds.begin();
		actRulesIndex = 0;
		while(actAntecedentsIdsIter != (*actionsIter).associatedNtIds.end())
		{
			currentAntecedentId = *actAntecedentsIdsIter;
			currentRule = associatedGrammar.FindRuleFromAntecedentId(currentAntecedentId);
			//Analyse the rule and insert its components into the string:
			number_to_string(currentAntecedentId,antecedentString);
			number_to_string(actRulesIndex,actRulesIndexString);
			//Check if it is publishable
			if(associatedGrammar.isPublishableRule(currentRule))
			{
				outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules","is_Publishable","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"]","true");
			}
			//Insert the antecedent of the rule into the string
			outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules","Antecedent_name","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"]",antecedentString);
			//Get the consecuents of the rule and insert them into the string
			ruleConsequentsNumber = currentRule->getNumberOfConsequents();
			for(unsigned i = 0;i<ruleConsequentsNumber;i++)
			{
				number_to_string(i,consequentIndexString);
				switch(currentRule->ConsequentAt(i)->getType())
				{
					case Symbol::NONTERMINAL: ntPtr=dynamic_cast<NonTerminal *>(currentRule->ConsequentAt(i));
											  number_to_string(ntPtr->getId(),nonTerminalString);
											  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules_consequents_list","Consequent_name","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"][Action_rules_consequents_list#"+consequentIndexString+"]",nonTerminalString);
											  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules_consequents_list","is_NonTerminal","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"][Action_rules_consequents_list#"+consequentIndexString+"]","true");
											  //Transitivity
											  if((ntPtr->getNTTransitivity()== NonTerminal::TRANSITIVE))
											  {
												  outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules_consequents_list","is_Transitive","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"][Action_rules_consequents_list#"+consequentIndexString+"]","true");
											  }
											  break;
					case Symbol::TERMINAL: tPtr = dynamic_cast<Terminal *>(currentRule->ConsequentAt(i));
										   outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules_consequents_list","Consequent_name","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"][Action_rules_consequents_list#"+consequentIndexString+"]",tPtr->getValue());
										   break;
				} //end SWITCH
				if(currentRule->isOR() && (i < (ruleConsequentsNumber-1)))
				{
					outputTemplate.RegisterRepetitionMacroValue("ARCHITECTURE_GRAMMAR","Action_rules_consequents_list","OR_Rule","[Actions_list#"+actionNamesIndexString+"][Action_rules#"+actRulesIndexString+"][Action_rules_consequents_list#"+consequentIndexString+"]","true");
				}
			}//End FOR
			actAntecedentsIdsIter++;
			actRulesIndex++;
		}
		actionsIter++;
		actionNamesIndex++;
	}
	//Macros substitution
	if(!outputTemplate.ApplyMacros("ARCHITECTURE_GRAMMAR",outputString))
	{
		printf("Error aplying the macros:%s\n",outputString.c_str());
		outputString = "";
	}
	return(outputString);
}

Grammar * ArchitectureGrammar::getGrammar()
{
	Grammar * newGrammar;
	newGrammar = new Grammar(associatedGrammar);
	return(newGrammar);
}

std::string ArchitectureGrammar::getAssociatedAction(const int actionRule, list<string>& rulesList)
{
	Rule * mainRule;	//Main rule to be working with(action rule)
	Symbol * currentConsequent; //Current consequent to work with
	Symbol::SymbolType currentType; //Type of the current consequent
	int currentAntecedentId;	//Identifier of the antecedent we are working with
	std::stringstream nameStream;
	string associatedAction; //Associated action (including the parameters)
	string actionLabel;	//Label of the associated action
	string objectName;	//Name of the associated object
	unsigned mainNumConsequents; //Number of consecuents for the main rule
	list<string>::iterator eraseIter; //Iterator to erase visited objects
	bool erased; //boolean to know if an object has been erased

	//Get the action associated to the given rule
	actionLabel = findActionByAntecedent(actionRule);
	//associatedAction = actionLabel;
	mainRule = associatedGrammar.FindRuleFromAntecedentId(actionRule);
	mainNumConsequents = mainRule->getNumberOfConsequents();
	for(int i = 0;i < (int)mainNumConsequents;i++)
	{
		currentConsequent = mainRule->ConsequentAt(i);
		currentType = currentConsequent->getType();
		switch(currentType)
		{
			case Symbol::NONTERMINAL: //Call to the recursive function
									  currentAntecedentId = dynamic_cast<NonTerminal *>(currentConsequent)->getId();
									  associatedAction = associatedAction + getAssociatedAction(currentAntecedentId,rulesList);
									  break;
			case Symbol::TERMINAL:
			{
				//Check if the rule is on the set of provided rules
				currentAntecedentId = mainRule->getAntecedent()->getId();
				nameStream << currentAntecedentId;
				if(isOnSet(nameStream.str().c_str(),rulesList))
				{
					//Erase the element to avoid repetitions
					erased = false;
					eraseIter = rulesList.begin();
					while((eraseIter != rulesList.end()) && (!erased))
					{
						if(*eraseIter == nameStream.str())
						{
							rulesList.erase(eraseIter);
							erased = true;
						}
						eraseIter++;
					}
					//Find the action label or the object label:
					actionLabel = findActionByAntecedent(currentAntecedentId);
					if(actionLabel.empty()) //It is an object
					{
						//Find the object label
						objectName = findObjectByAntecedent(currentAntecedentId);
						associatedAction = associatedAction + " " + objectName;
					}
					else
					{
						associatedAction = associatedAction + "" + actionLabel;
					}
				}
			}
			break;
		}
	}
	return(associatedAction);
}

bool ArchitectureGrammar::findActionInList(const string & elementName,list<actionAssociation>::iterator &elementIter)
{
	elementIter = actionsAssociations.begin();
	while(elementIter != actionsAssociations.end())
	{
		if((*elementIter).elementName == elementName)
		{
			return(true);
		}
		elementIter++;
	}
	return(false);
}

bool ArchitectureGrammar::findObjectInList(const string & elementName,list<objectAssociation>::iterator &elementIter)
{
	elementIter = objectsAssociations.begin();
	while(elementIter != objectsAssociations.end())
	{
		if((*elementIter).elementName == elementName)
		{
			return(true);
		}
		elementIter++;
	}
	return(false);
}

int ArchitectureGrammar::findTypeIdentifier(const string & look4typeName)
{
	list<typeAssociation>::iterator typesIterator;
	//bool found = false;
	typesIterator = grammarAssociations.begin();
	while(typesIterator != grammarAssociations.end())
	{
		if((*typesIterator).typeName == look4typeName)
		{
			return((*typesIterator).ntId);
		}
		typesIterator++;
	}
	return(0);
}

string ArchitectureGrammar::findActionByAntecedent(const int antecedentName)
{
	list<actionAssociation>::iterator actionsIterator;
	actionAssociation currentAssociation;
	list<int>::iterator ntIter; // Iterator for the associated non-terminals

	actionsIterator = actionsAssociations.begin();
	while(actionsIterator != actionsAssociations.end())
	{
		//Check if the Antecedent belong to the list of associated rules
		currentAssociation = (*actionsIterator);
		ntIter = currentAssociation.associatedNtIds.begin();
		while(ntIter != currentAssociation.associatedNtIds.end())
		{
			if(antecedentName == (*ntIter))
			{
				return(currentAssociation.elementName);
			}
			ntIter++;
		}
		actionsIterator++;
	}
	return("");
}

string ArchitectureGrammar::findObjectByAntecedent(const int antecedentName)
{
	list<objectAssociation>::iterator objectsIterator;
	objectAssociation currentAssociation;
	list<int>::iterator ntIter; // Iterator for the associated non-terminals

	objectsIterator = objectsAssociations.begin();
	while(objectsIterator != objectsAssociations.end())
	{
		//Check if the Antecedent belong to the list of associated rules
		currentAssociation = (*objectsIterator);
		ntIter = currentAssociation.associatedNtIds.begin();
		while(ntIter != currentAssociation.associatedNtIds.end())
		{
			if(antecedentName == (*ntIter))
			{
				return(currentAssociation.elementName);
			}
			ntIter++;
		}
		objectsIterator++;
	}
	return("");
}
/* end ArchitectureGrammar class */
