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
#include <mutex>
#include <condition_variable>
#endif


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class GENERAL_API Semaphore
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

#else
// standard library implementation
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
