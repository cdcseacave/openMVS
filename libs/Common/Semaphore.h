////////////////////////////////////////////////////////////////////
// Semaphore.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_SEMAPHORE_H__
#define __SEACAVE_SEMAPHORE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Thread.h"

#ifndef _MSC_VER
#include "CriticalSection.h"
#include <sys/time.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class Semaphore  
{
#ifdef _MSC_VER
public:
	Semaphore(unsigned count=0) {
		h = CreateSemaphore(NULL, count, MAXLONG, NULL);
	};
	~Semaphore() {
		CloseHandle(h);
	};

	void Clear(unsigned count=0) {
		CloseHandle(h);
		h = CreateSemaphore(NULL, count, MAXLONG, NULL);
	}

	void Signal() {
		ReleaseSemaphore(h, 1, NULL);
	}
	void Signal(unsigned count) {
		ASSERT(count > 0);
		ReleaseSemaphore(h, count, NULL);
	}

	bool Wait() {
		return WaitForSingleObject(h, INFINITE) == WAIT_OBJECT_0;
	}
	bool Wait(uint32_t millis) {
		return WaitForSingleObject(h, millis) == WAIT_OBJECT_0;
	}

protected:
	HANDLE h;
#else
public:
	Semaphore(unsigned c=0) : count(c) { pthread_cond_init(&cond, NULL); }
	~Semaphore() { pthread_cond_destroy(&cond); }

	void Clear(unsigned c=0) {
		pthread_cond_destroy(&cond);
		pthread_cond_init(&cond, NULL);
		count = c;
	}

	void Signal() { 
		Lock l(cs);
		count++;
		pthread_cond_signal(&cond);
	}
	void Signal(unsigned c) { 
		ASSERT(c > 0);
		Lock l(cs);
		count += c;
		pthread_cond_signal(&cond);
	}

	bool Wait() { 
		Lock l(cs);
		if (count == 0)
			pthread_cond_wait(&cond, &cs.getMutex());
		count--;
		return true;
	}
	bool Wait(uint32_t millis) { 
		Lock l(cs);
		if (count == 0) {
			timeval timev;
			timespec t;
			gettimeofday(&timev, NULL);
			millis+=timev.tv_usec/1000;
			t.tv_sec = timev.tv_sec + (millis/1000);
			t.tv_nsec = (millis%1000)*1000*1000;
			if (pthread_cond_timedwait(&cond, &cs.getMutex(), &t) != 0)
				return false;
		}
		count--;
		return true;
	}

protected:
	pthread_cond_t cond;
	CriticalSection cs;
	int count;
#endif

private:
	Semaphore(const Semaphore&);
	Semaphore& operator=(const Semaphore&);
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_SEMAPHORE_H__
