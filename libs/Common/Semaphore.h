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
#ifdef _SUPPORT_CPP11
#include <mutex>
#include <condition_variable>
#else
#include "CriticalSection.h"
#include <sys/time.h>
#endif
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

	void Wait() {
		WaitForSingleObject(h, INFINITE);
	}
	bool Wait(uint32_t millis) {
		return WaitForSingleObject(h, millis) == WAIT_OBJECT_0;
	}

protected:
	HANDLE h;

#elif !defined(_SUPPORT_CPP11)
// pthread implementation
public:
	Semaphore(unsigned c=0) : count(c) { pthread_cond_init(&cond, NULL); }
	~Semaphore() { pthread_cond_destroy(&cond); }

	void Clear(unsigned c=0) {
		cs.Clear();
		pthread_cond_destroy(&cond);
		pthread_cond_init(&cond, NULL);
		count = c;
	}

	void Signal() {
		Lock l(cs);
		++count;
		pthread_cond_signal(&cond);
	}
	void Signal(unsigned c) {
		ASSERT(c > 0);
		for (unsigned i=0; i<c; ++i)
			Signal();
	}

	void Wait() {
		Lock l(cs);
		while (!count)
			pthread_cond_wait(&cond, &cs.getMutex());
		--count;
	}
	bool Wait(uint32_t millis) {
		Lock l(cs);
		if (count == 0) {
			timeval timev;
			gettimeofday(&timev, NULL);
			millis += timev.tv_usec/1000;
			timespec t = {
				timev.tv_sec + (millis/1000),
				(millis%1000)*1000*1000
			};
			pthread_cond_timedwait(&cond, &cs.getMutex(), &t);
			if (count == 0)
				return false;
		}
		--count;
		return true;
	}

protected:
	pthread_cond_t cond;
	CriticalSection cs;
	unsigned count;

#else
// C++11 implementation
public:
	Semaphore(unsigned c=0) : count(c) {}
	~Semaphore() {}

	void Clear(unsigned c=0) {
		std::lock_guard<std::mutex> lock{mtx};
		count = c;
	}

	void Signal() {
		std::lock_guard<std::mutex> lock{mtx};
		++count;
		cv.notify_one();
	}
	void Signal(unsigned c) {
		ASSERT(c > 0);
		for (unsigned i=0; i<c; ++i)
			Signal();
	}

	void Wait() {
		std::unique_lock<std::mutex> lock{mtx};
		cv.wait(lock, [&] { return count > 0; });
		--count;
	}
	bool Wait(uint32_t millis) {
		std::unique_lock<std::mutex> lock{mtx};
		if (!cv.wait_for(lock, std::chrono::milliseconds(millis), [&] { return count > 0; }))
			return false;
		--count;
		return true;
	}

protected:
	std::condition_variable cv;
	std::mutex mtx;
	unsigned count;
#endif

private:
	Semaphore(const Semaphore&);
	Semaphore& operator=(const Semaphore&);
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_SEMAPHORE_H__
