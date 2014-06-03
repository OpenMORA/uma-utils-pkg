/*----------------------------------------*/
/*	TIMED STRINGS QUEUE OBJECTS LIBRARY   */
/*                                        */
/*	ECM, 2007                             */
/*----------------------------------------*/

/** \file TimedStringsQueue.hpp
 *\brief Implementation of a queue of timed-out strings
 *
 * This file declares the class TimedStringsQueue, which implements a queue
 * of string objects with time outs, that is, the elements in the queue
 * can have a maximum time of permanence in the queue, after that time the
 * objects will be removed from the queue. If no timeout is specified, the
 * object will remain in the queue until it is affected by a pop operation
 */

#ifndef TIMEDSTRINGSQUEUECLASS
	#define  TIMEDSTRINGSQUEUECLASS

#include <queue>
#include <list>
#include <string>

/*! \brief TimedStringsQueue objects.
 *
 *  TimedStringsQueue objects implements lists of strings that has a finite
 * remain time into the queue, that is, if the object is not affected by a
 * pop operation in a time less than its timeout, the object will be removed
 * from the queue. There can be objects with no associated timeout, this is
 * specified by setting the timeout value at -1 in the push operation.
 */
class TimedStringsQueue
{
	public:
		/******************************
		 * Nested classes definitions *
		 ******************************/

		/*!\brief Elements to be stored in the priority queue.
		 *
		 * This class holds information about the strings and their times
		 * of arrival into the queue, their prioriy, the group they belong
		 * to and the minimum time that must be elapsed between one message
		 * and the next message of its same group are popped-out from the
		 * queue.
		 */
		class TimedString
		{
			public:
				/*! Class constructor
				 *\param newStrValue value of the string in the new object
				 *\param newPrio priority of the new object
				 *\param newTimeout maximum time to wait for the object to be delivered
				 *\param newStrGroup group of the string identifier(for future use)
				 *\param newMinTimeBtw minimum time between messages of the same group
				 *\param newArrivalTime time of arrival of the element to the queue
				 */
				 TimedString(std::string newStrValue,unsigned newPrio,double newTimeout,unsigned newStrGroup,double newMinTimeBtw, long newArrivalTime);

				/*! Copy constructor. Replicates the object passed as parameter
				 *\param originString TimedString object to replicate in this object.
				 */
				 TimedString(const TimedString & originString);

				 /*! Class destructor
				 */
				~TimedString();

				/*! Overloaded equal operator to avoid malfunction when copying
				 *\param originString TimedString object to replicate in this object.
				 */
				TimedString & operator = (const TimedString &originString);


				/*! Gets the value of the string
				 *\return string value
				 */
				 std::string GetStrValue(void);

				/*! Gets the priority of the element
				 *\return priority value
				 */
				 unsigned GetPrio(void);

				 /*! Gets the timeout value of the element
				  *\return timeout value
				  */
				 double GetTimeout(void);

				 /*! Gets the group identifier of the element
				  *\return group identifier
				  */
				 unsigned GetStrGroup(void);

				 /*! Gets the minimum time between messages of the same group
				  *\return minimum time between messages of the same group
				  */
				 double GetMinTimeBtw(void);

				 /*! Gets the arrival time of the message
				  *\return arrival time of the message
				  */
				 long GetArrivalTime(void);

			private:
				/*! Copy method to properly replicate objects when using the copy constructor
				 * and the = operator.
				 *\param originString TimedString object to replicate in this object.
				 */
				void copyTimedString(const TimedString &originString);

				std::string strValue; // Value of the string
				unsigned prio; // Priority of the element
				double timeout; // expiral time of the string in the queue
				unsigned strGroup; // Group of the string message
				double minTimeBtw; //minimum time between two messages of the same group
				long arrivalTime; //insertion time of the string in the queue
		};
		/*! Default constructor
		 */
		TimedStringsQueue();

		/*! Class destructor
		 */
		~TimedStringsQueue();

		/*! Inserts the string element into the queue and starts its timeout
		 * expiry counting(if there is one). If no timeout is requested, the
		 * strTimeout parameter must be set to -1.
		 *\param str2Push string to push
		 *\param prio priority of the string to queue (0->lowest priority)
		 *\param strTimeout timeout(in seconds) of the string to push
		 */
		void push(std::string str2Push,unsigned prio, double strTimeout);

		/*! Overloaded method Inserts the string element into the queue and starts its
		 * timeout expiry counting(if there is one). If no timeout is requested, the
		 * strTimeout parameter must be set to -1. it also sets a type value for the
		 * string (for future use) and a minimum time value that must pass before a
		 * message of the same type can be consumed from the queue.
		 *\param str2Push string to push
		 *\param prio priority of the string into the queue
		 *\param strTimeout timeout(in seconds) of the string to push
		 *\param strGroup identifier of the group the string belongs to (>=1)
		 *\param minTimeBtw minimum elapsed time between two messages from the same group
		 */
		void push(std::string str2Push,unsigned prio, double strTimeout,unsigned strGroup, double minTimeBtw);

		/*! Returns the top element on the queue
		 *\return string in the top of the queue
		 */
		std::string pop(void);

		/*! Returs true if the queue is empty
		 *\return true if the queue is empty, false otherwise
		 */
		bool empty(void);

		/*!\struct ComparePrios
		 *\brief Compares the priority of two TimedString
		 */
		struct ComparePrios
		{
			bool operator()(TimedString a,TimedString b)
			{
				return(a.GetPrio() < b.GetPrio());
			}
		};
	private:
		/*!\struct timeRestriction
		 *\brief Stores the time restriction for the groups of the queue
		 */
		typedef struct
		{
			unsigned groupId; //Identifier of the group affected by the restriction
			long startTime; //Starting time of the restriction
			double minTimeBtw; //Time that must pass to wake the restriction
		}timeRestriction;

		std::priority_queue<TimedString,std::vector<TimedString>,ComparePrios> stringsQ; //Queue of strings
		std::list<timeRestriction> activeTimeRestrictions; //Time restrictions that are active for the queue
};
#endif
