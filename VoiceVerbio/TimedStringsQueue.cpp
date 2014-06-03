/*----------------------------------------*/
/*	TIMED STRINGS QUEUE OBJECTS LIBRARY   */
/*                                        */
/*	ECM, 2007                             */
/*----------------------------------------*/

/** \file TimedStringsQueue.cpp
 *\brief Implementation of TimedStringsQueue.hpp 
 */

#include "TimedStringsQueue.hpp"
#include <string>
#include <time.h>

using namespace std;

/*-------------------*
 * TimedString class *
 *-------------------*/
TimedStringsQueue::TimedString::TimedString(string newStrValue,unsigned newPrio,double newTimeout,
					     unsigned newStrGroup,double newMinTimeBtw, long newArrivalTime)
{
	strValue = newStrValue;
	prio = newPrio;
	timeout = newTimeout;
	strGroup = newStrGroup;
	minTimeBtw = newMinTimeBtw;
	arrivalTime = newArrivalTime;
}


TimedStringsQueue::TimedString::TimedString(const TimedStringsQueue::TimedString &originString)
{
	copyTimedString(originString);
}

TimedStringsQueue::TimedString::~TimedString()
{
}

TimedStringsQueue::TimedString & TimedStringsQueue::TimedString::operator =(const TimedStringsQueue::TimedString &originString)
{
	copyTimedString(originString);
	return(*this);
}

string TimedStringsQueue::TimedString::GetStrValue()
{
	return(strValue);
}

unsigned TimedStringsQueue::TimedString::GetPrio()
{
	return(prio);
}

double TimedStringsQueue::TimedString::GetTimeout()
{
	return(timeout);
}

unsigned TimedStringsQueue::TimedString::GetStrGroup()
{
	return(strGroup);
}

double TimedStringsQueue::TimedString::GetMinTimeBtw()
{
	return(minTimeBtw);
}

long TimedStringsQueue::TimedString::GetArrivalTime()
{
	return(arrivalTime);
}

void TimedStringsQueue::TimedString::copyTimedString(const TimedStringsQueue::TimedString &originString)
{
	strValue = originString.strValue;
	prio = originString.prio;
	timeout = originString.timeout;
	strGroup = originString.strGroup;
	minTimeBtw = originString.minTimeBtw;
	arrivalTime = originString.arrivalTime;
}

/*-------------------------*
 * TimedStringsQueue class *
 *-------------------------*/
TimedStringsQueue::TimedStringsQueue()
{	
}

TimedStringsQueue::~TimedStringsQueue()
{	
}

void TimedStringsQueue::push(string str2Push,unsigned prio,double strTimeout)
{
	TimedStringsQueue::TimedString timedstr2Push(str2Push,prio,strTimeout,0,0,(long)time(NULL));	
	stringsQ.push(timedstr2Push);
}

void TimedStringsQueue::push(string str2Push,unsigned prio,double strTimeout,unsigned strGroup,double minTimeBtw)
{
	TimedStringsQueue::TimedString timedstr2Push(str2Push,prio,strTimeout,strGroup,minTimeBtw,(long)time(NULL));	
	stringsQ.push(timedstr2Push);
}

string TimedStringsQueue::pop(void)
{
	TimedStringsQueue::TimedString * timedstr2Pop = NULL;
	unsigned currentGroupId;
	list<timeRestriction>::iterator timesIter;
	bool notFound; //false if the restriction is found in the queue
	timeRestriction newTimeRestriction;
	while(!stringsQ.empty())
	{
		timedstr2Pop = new TimedStringsQueue::TimedString(stringsQ.top());
		stringsQ.pop();		

		if(timedstr2Pop->GetTimeout() >= 0) //There is a timeout
		{			
			if((difftime(time(NULL),(time_t)timedstr2Pop->GetArrivalTime())) <= timedstr2Pop->GetTimeout())
			{
				//Time restrictions handling				
				currentGroupId = timedstr2Pop->GetStrGroup();
				//printf("ECM: There is a timeout restriction for the group: %u\n",currentGroupId);
				if(currentGroupId != 0) //The string belongs to a group
				{
					//Check if there is an active timeout
					timesIter = activeTimeRestrictions.begin();
					notFound = true;
					while((timesIter != activeTimeRestrictions.end()) && notFound)
					{
						if((*timesIter).groupId == currentGroupId)
						{
							notFound = false;
						}
						else
						{
							timesIter++;
						}
					}
					if(notFound)
					{
						//printf("ECM: There is no restriction for the group %u, a new restriction will be set\n",currentGroupId);
						newTimeRestriction.groupId = currentGroupId;
						newTimeRestriction.startTime = (long)time(NULL);
						newTimeRestriction.minTimeBtw = timedstr2Pop->GetMinTimeBtw();
						activeTimeRestrictions.push_back(newTimeRestriction);
					}
					else
					{
						if((difftime(time(NULL),(time_t)(*timesIter).startTime)) <= (*timesIter).minTimeBtw)
						{						
							//printf("ECM: The string is under a group restriction, it will be inserted again into the queue\n");
							stringsQ.push(*timedstr2Pop);
							return("");
						}
						else
						{
							//Refresh the starting time of the restriction
							//printf("ECM: The restriction has expired, refreshing the starting time of the restriction for group: %u\n",currentGroupId);
							(*timesIter).minTimeBtw = timedstr2Pop->GetMinTimeBtw();
							(*timesIter).startTime = (long)time(NULL);

						}
					}
				}
				/*else
				{
					printf("ECM: The string doesn't belong to a group, no restriction check\n");
				}*/

				//printf("ECM: Sending string from group: %u \n",currentGroupId);
				return(timedstr2Pop->GetStrValue());
			}
			/*else
			{
				printf("ECM: Rejected message %s of group %u because of timeout %4.2f\n",timedstr2Pop->GetStrValue().c_str(),timedstr2Pop->GetStrGroup(),timedstr2Pop->GetTimeout());
			}*/
		}
		else //No timeout
		{
			return(timedstr2Pop->GetStrValue());
		}
	}
	return("");
}

bool TimedStringsQueue::empty(void)
{
	return(stringsQ.empty());
}
/* end TimedStringsQueue class */