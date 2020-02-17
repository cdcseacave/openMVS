////////////////////////////////////////////////////////////////////
// EventQueue.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "EventQueue.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

/*-----------------------------------------------------------*
 * EventQueue class implementation                           *
 *-----------------------------------------------------------*/

void EventQueue::Clear()
{
	m_cs.Clear();
	m_sem.Clear();
	m_events.Empty();
}

void EventQueue::AddEvent(Event* evt)
{
	Lock l(m_cs);
	m_events.AddTail(evt);
	m_sem.Signal();
}
void EventQueue::AddEventFirst(Event* evt)
{
	Lock l(m_cs);
	m_events.AddHead(evt);
	m_sem.Signal();
}

Event* EventQueue::GetEvent()
{
	m_sem.Wait();
	Lock l(m_cs);
	ASSERT(!m_events.IsEmpty());
	return m_events.RemoveHead();
}
Event* EventQueue::GetEvent(uint32_t millis)
{
	if (!m_sem.Wait(millis))
		return NULL;
	Lock l(m_cs);
	if (m_events.IsEmpty())
		return NULL;
	return m_events.RemoveHead();
}
Event* EventQueue::GetEventLast()
{
	m_sem.Wait();
	Lock l(m_cs);
	ASSERT(!m_events.IsEmpty());
	return m_events.RemoveTail();
}
Event* EventQueue::GetEventLast(uint32_t millis)
{
	if (!m_sem.Wait(millis))
		return NULL;
	Lock l(m_cs);
	if (m_events.IsEmpty())
		return NULL;
	return m_events.RemoveTail();
}
/*----------------------------------------------------------------*/

bool EventQueue::IsEmpty() const
{
	Lock l(m_cs);
	return m_events.IsEmpty();
}

uint_t EventQueue::GetSize() const
{
	Lock l(m_cs);
	return m_events.GetSize();
}
/*----------------------------------------------------------------*/



/*-----------------------------------------------------------*
* EventThreadPool class implementation                      *
*-----------------------------------------------------------*/

void EventThreadPool::stop()
{
	ThreadPool::stop();
	EventQueue::Clear();
}
/*----------------------------------------------------------------*/
