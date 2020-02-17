////////////////////////////////////////////////////////////////////
// EventQueue.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_EVENTQUEUE_H__
#define __SEACAVE_EVENTQUEUE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Types.h"
#include "CriticalSection.h"
#include "Semaphore.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class Event
{
public:
	Event(uint32_t _id) : id(_id) {}
	virtual ~Event() {}

	uint32_t GetID() const { return id; }

	virtual bool Run(void* /*pArgs*/ = NULL) { return true; }

protected:
	const uint32_t id;
};
typedef cQueue<Event*,Event*,0> EVENTQUEUE;


/**************************************************************************************
 * Events Queue
 * --------------
 * basic eventing mechanism
 * multi-thread safe
 **************************************************************************************/

class GENERAL_API EventQueue  
{
public:
	EventQueue() {}
	~EventQueue() {}

	void Clear(); // reset the state of the locks and empty the queue

	void AddEvent(Event*); //add a new event to the end of the queue
	void AddEventFirst(Event*); //add a new event to the beginning of the queue
	Event* GetEvent(); //block until an event arrives and get the first event pending in the queue
	Event* GetEvent(uint32_t millis); //block until an event arrives or time expires and get the first event pending in the queue
	Event* GetEventLast(); //block until an event arrives and get the last event pending in the queue
	Event* GetEventLast(uint32_t millis); //block until an event arrives or time expires and get the last event pending in the queue

	bool IsEmpty() const; //are there any events in the queue?
	uint_t GetSize() const; //number of events in the queue

protected:
	Semaphore m_sem;
	mutable CriticalSection m_cs;
	EVENTQUEUE m_events;
};
/*----------------------------------------------------------------*/


// basic event and thread pool
class GENERAL_API EventThreadPool : public ThreadPool, public EventQueue
{
public:
	inline EventThreadPool() {}
	inline EventThreadPool(size_type nThreads) : ThreadPool(nThreads) {}
	inline EventThreadPool(size_type nThreads, Thread::FncStart pfnStarter, void* pData=NULL) : ThreadPool(nThreads, pfnStarter, pData) {}
	inline ~EventThreadPool() {}

	void stop(); //stop threads, reset locks state and empty event queue
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_EVENTQUEUE_H__
