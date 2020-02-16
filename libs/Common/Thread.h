////////////////////////////////////////////////////////////////////
// Thread.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_THREAD_H__
#define __SEACAVE_THREAD_H__


// I N C L U D E S /////////////////////////////////////////////////

#ifdef _MSC_VER
#include <windows.h>
#ifdef _SUPPORT_CPP11
#include <cstdint>
#else
#include <stdint.h>
#endif
#else
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/**************************************************************************************
 * Thread
 * --------------
 * basic thread control
 **************************************************************************************/

class GENERAL_API Thread
{
public:
	#ifdef  _ENVIRONMENT64
	typedef int64_t safe_t;
	#else
	typedef int32_t safe_t;
	#endif

	enum Priority {
		IDLE = -2,
		LOW = -1,
		NORMAL = 0,
		HIGH = 1
	};

	#ifdef _MSC_VER

	typedef HANDLE thread_t;
	typedef void* (STCALL *FncStart)(void*);

	typedef struct STARTER_DATA {
		FncStart fnStarter;
		void* pData;
		STARTER_DATA(FncStart _fnStarter, void* _pData) : fnStarter(_fnStarter), pData(_pData) {}
	} StarterData;

	Thread() : threadHandle(NULL), threadId(0) {}
	virtual ~Thread() { stop(); }

	inline bool isRunning() const { return (threadHandle != NULL); }

	bool start(FncStart pfnStarter, void* pData=NULL) {
		join();
		return ((threadHandle = CreateThread(NULL, 0, &starterConvertor, new StarterData(pfnStarter, pData), 0, &threadId)) != NULL);
	}
	inline bool start() {
		return start(&starter, this);
	}
	void stop() {
		if (threadHandle) {
			TerminateThread(threadHandle, -1);
			CloseHandle(threadHandle);
			threadHandle = NULL;
			threadId = 0;
		}
	}
	void join() {
		if (threadHandle == NULL)
			return;
		WaitForSingleObject(threadHandle, INFINITE);
		CloseHandle(threadHandle);
		threadHandle = NULL;
		threadId = 0;
	}

	inline void setThreadPriority(Priority p) const { ::SetThreadPriority(threadHandle, convertPriority(p)); }
	inline Priority getThreadPriority() const { return convertPriority((PriorityOS)::GetThreadPriority(threadHandle)); }
	static inline void setThreadPriority(thread_t th, Priority p) { ::SetThreadPriority(th, convertPriority(p)); }
	static inline Priority getThreadPriority(thread_t th) { return convertPriority((PriorityOS)::GetThreadPriority(th)); }

	static inline void sleep(uint32_t millis)	{ ::Sleep(millis); }
	static inline void yield()					{ ::Sleep(0); }
	static inline thread_t currentThread()		{ return ::GetCurrentThread(); }
	static uint32_t hardwareConcurrency() {
		SYSTEM_INFO info={{0}};
		GetSystemInfo(&info);
		return info.dwNumberOfProcessors;
	}

	STATIC_ASSERT(sizeof(int32_t)==sizeof(LONG));
	static inline int32_t safeInc(volatile int32_t& v) { return InterlockedIncrement((volatile LONG*)&v); };
	static inline int32_t safeDec(volatile int32_t& v) { return InterlockedDecrement((volatile LONG*)&v); };
	static inline int32_t safeExchange(volatile int32_t& target, int32_t value) { return InterlockedExchange((volatile LONG*)&target, value); };
	static inline int32_t safeCompareExchange(volatile int32_t& target, int32_t comp, int32_t value) { return InterlockedCompareExchange((volatile LONG*)&target, value, comp); };

	STATIC_ASSERT(sizeof(int64_t)==sizeof(LONGLONG));
	static inline int64_t safeInc(volatile int64_t& v) { return InterlockedIncrement64((volatile LONGLONG*)&v); };
	static inline int64_t safeDec(volatile int64_t& v) { return InterlockedDecrement64((volatile LONGLONG*)&v); };
	static inline int64_t safeExchange(volatile int64_t& target, int64_t value) { return InterlockedExchange64((volatile LONGLONG*)&target, value); };
	static inline int64_t safeCompareExchange(volatile int64_t& target, int64_t comp, int64_t value) { return InterlockedCompareExchange64((volatile LONGLONG*)&target, value, comp); };

	#else //_MSC_VER

	typedef pthread_t thread_t;
	typedef void* (STCALL *FncStart)(void*);

	Thread() : threadHandle(0) {}
	virtual ~Thread() { stop(); }

	inline bool isRunning() const { return (threadHandle != 0); }

	bool start(FncStart pfnStarter, void* pData=NULL) {
		join();
		return (pthread_create(&threadHandle, NULL, pfnStarter, pData) == 0);
	}
	bool start() {
		return start(&starter, this);
	}
	void stop() {
		if (threadHandle != 0) {
			pthread_detach(threadHandle);
			threadHandle = 0;
		}
	}
	void join() {
		if (threadHandle) {
			pthread_join(threadHandle, 0);
			threadHandle = 0;
		}
	}

	inline void setThreadPriority(Priority p) const { setThreadPriority(threadHandle, p); }
	inline Priority getThreadPriority() const { return getThreadPriority(threadHandle); }
	static void setThreadPriority(thread_t th, Priority p) {
		struct sched_param param;
		param.sched_priority = convertPriority(p);
		pthread_setschedparam(th, SCHED_OTHER, &param);
	}
	static Priority getThreadPriority(thread_t th) {
		struct sched_param param;
		int                policy;
		pthread_getschedparam(th, &policy, &param);
		return convertPriority((PriorityOS)param.sched_priority);
	}

	static void sleep(uint32_t millis)	{ ::usleep(millis*1000); }
	static void yield()					{ ::sched_yield(); }
	static inline thread_t currentThread(){ return pthread_self(); }
	static uint32_t hardwareConcurrency() { return sysconf(_SC_NPROCESSORS_ONLN); }

	static inline int32_t safeInc(volatile int32_t& v) { return __sync_add_and_fetch(&v, 1); }
	static inline int32_t safeDec(volatile int32_t& v) { return __sync_sub_and_fetch(&v, 1); }
	static inline int32_t safeExchange(volatile int32_t& target, int32_t value) { return __sync_val_compare_and_swap(&target, target, value); }
	static inline int32_t safeCompareExchange(volatile int32_t& target, int32_t comp, int32_t value) { return __sync_val_compare_and_swap(&target, comp, value); }

	static inline int64_t safeInc(volatile int64_t& v) { return __sync_add_and_fetch(&v, 1); }
	static inline int64_t safeDec(volatile int64_t& v) { return __sync_sub_and_fetch(&v, 1); }
	static inline int64_t safeExchange(volatile int64_t& target, int64_t value) { return __sync_val_compare_and_swap(&target, target, value); }
	static inline int64_t safeCompareExchange(volatile int64_t& target, int64_t comp, int64_t value) { return __sync_val_compare_and_swap(&target, comp, value); }

	#endif //_MSC_VER

	static unsigned getMaxThreads(unsigned threads) {
		if (threads == 1)
			return 1;
		const unsigned maxThreads = hardwareConcurrency();
		if (threads > 0 && threads < maxThreads)
			return threads;
		return maxThreads;
	}

protected:
	virtual void run() {}

protected:
	#ifdef _MSC_VER

	enum PriorityOS {
		OS_IDLE = THREAD_PRIORITY_IDLE,
		OS_LOW = THREAD_PRIORITY_BELOW_NORMAL,
		OS_NORMAL = THREAD_PRIORITY_NORMAL,
		OS_HIGH = THREAD_PRIORITY_ABOVE_NORMAL
	};

	static DWORD WINAPI starterConvertor(void* p)
	{
		CAutoPtr<StarterData> pData((StarterData*)p);
		return (DWORD)reinterpret_cast<size_t>(pData->fnStarter(pData->pData));
	}

	#else //_MSC_VER

	enum PriorityOS {
		OS_IDLE = 19,
		OS_LOW = 10,
		OS_NORMAL = 0,
		OS_HIGH = -10
	};

	#endif //_MSC_VER

	static inline PriorityOS convertPriority(Priority p) {
		switch (p) {
		case IDLE:		return OS_IDLE;
		case LOW:		return OS_LOW;
		case NORMAL:	return OS_NORMAL;
		case HIGH:		return OS_HIGH;
		}
		return OS_NORMAL;
	}
	static inline Priority convertPriority(PriorityOS p) {
		switch (p) {
		case OS_IDLE:	return IDLE;
		case OS_LOW:	return LOW;
		case OS_NORMAL:	return NORMAL;
		case OS_HIGH:	return HIGH;
		}
		return NORMAL;
	}

	static void* starter(void* p)
	{
		((Thread*)p)->run();
		return NULL;
	}

protected:
	thread_t threadHandle;
	#ifdef _MSC_VER
	DWORD threadId;
	#endif
};
/*----------------------------------------------------------------*/



/**************************************************************************************
 * ThreadPool
 * --------------
 * basic thread pool
 **************************************************************************************/

class GENERAL_API ThreadPool
{
public:
	typedef uint32_t size_type;
	typedef Thread value_type;
	typedef value_type* iterator;
	typedef const value_type* const_iterator;
	typedef value_type& reference;
	typedef const value_type& const_reference;
	typedef CLISTDEFIDX(Thread,size_type) Threads;

public:
	inline ThreadPool() {}
	inline ThreadPool(size_type nThreads) : _threads(nThreads>0?nThreads:Thread::hardwareConcurrency()) {}
	inline ThreadPool(size_type nThreads, Thread::FncStart pfnStarter, void* pData=NULL) : _threads(nThreads>0?nThreads:Thread::hardwareConcurrency()) { start(pfnStarter, pData); }
	inline ~ThreadPool() { join(); }

	#ifdef _SUPPORT_CPP11
	inline ThreadPool(ThreadPool&& rhs) : _threads(std::forward<Threads>(rhs._threads)) {
	}

	inline ThreadPool& operator=(ThreadPool&& rhs) {
		_threads.Swap(rhs._threads);
		return *this;
	}
	#endif

	// wait for all running threads to finish and resize threads array
	void resize(size_type nThreads) {
		join();
		_threads.resize(nThreads);
	}
	// start all threads with the given function
	bool start(Thread::FncStart pfnStarter, void* pData=NULL) {
		for (Thread& thread: _threads)
			if (!thread.start(pfnStarter, pData))
				return false;
		return true;
	}
	// wait for all running threads to finish
	void join() {
		for (Thread& thread: _threads)
			thread.join();
	}
	// stop all threads
	void stop() {
		for (Thread& thread: _threads)
			thread.stop();
	}
	// wait for all running threads to finish and release threads array
	void Release() {
		join();
		_threads.Release();
	}

	inline bool empty() const { return _threads.empty(); }
	inline size_type size() const { return _threads.size(); }
	inline const_iterator cbegin() const { return _threads.cbegin(); }
	inline const_iterator cend() const { return _threads.cend(); }
	inline const_iterator begin() const { return _threads.begin(); }
	inline const_iterator end() const { return _threads.end(); }
	inline iterator begin() { return _threads.begin(); }
	inline iterator end() { return _threads.end(); }
	inline const_reference operator[](size_type index) const { return _threads[index]; }
	inline reference operator[](size_type index) { return _threads[index]; }

protected:
	Threads _threads;
};
/*----------------------------------------------------------------*/



/**************************************************************************************
 * Process
 * --------------
 * basic process control
 **************************************************************************************/

class GENERAL_API Process
{
public:
	enum Priority {
		IDLE = -3,
		LOW = -2,
		BELOWNORMAL = -1,
		NORMAL = 0,
		ABOVENORMAL = 1,
		HIGH = 2,
		REALTIME = 3
	};

	#ifdef _MSC_VER

	typedef HANDLE process_t;

	static inline process_t getCurrentProcessID() { return ::GetCurrentProcess(); }
	static inline void setProcessPriority(process_t id, Priority p) { ::SetPriorityClass(id, convertPriority(p)); }
	static inline Priority getProcessPriority(process_t id) { return convertPriority((PriorityOS)::GetPriorityClass(id)); }

	#else //_MSC_VER

	typedef id_t process_t;

	static inline process_t getCurrentProcessID() { return ::getpid(); }
	static inline void setProcessPriority(process_t id, Priority p) { ::setpriority(PRIO_PROCESS, id, convertPriority(p)); }
	static inline Priority getProcessPriority(process_t id) { return convertPriority((PriorityOS)::getpriority(PRIO_PROCESS, id)); }

	#endif //_MSC_VER

	static inline void setCurrentProcessPriority(Priority p) { setProcessPriority(getCurrentProcessID(), p); }
	static inline Priority getCurrentProcessPriority() { return getProcessPriority(getCurrentProcessID()); }

protected:

	#ifdef _MSC_VER

	enum PriorityOS {
		OS_IDLE = IDLE_PRIORITY_CLASS,
		OS_LOW = PROCESS_MODE_BACKGROUND_BEGIN,
		OS_BELOWNORMAL = BELOW_NORMAL_PRIORITY_CLASS,
		OS_NORMAL = NORMAL_PRIORITY_CLASS,
		OS_ABOVENORMAL = ABOVE_NORMAL_PRIORITY_CLASS,
		OS_HIGH = HIGH_PRIORITY_CLASS,
		OS_REALTIME = REALTIME_PRIORITY_CLASS
	};

	#else //_MSC_VER

	enum PriorityOS {
		OS_IDLE = 19,
		OS_LOW = 15,
		OS_BELOWNORMAL = 10,
		OS_NORMAL = 0,
		OS_ABOVENORMAL = -10,
		OS_HIGH = -15,
		OS_REALTIME = -20
	};

	#endif //_MSC_VER

	static inline PriorityOS convertPriority(Priority p) {
		switch (p) {
		case IDLE:				return OS_IDLE;
		case LOW:				return OS_LOW;
		case BELOWNORMAL:		return OS_BELOWNORMAL;
		case NORMAL:			return OS_NORMAL;
		case ABOVENORMAL:		return OS_ABOVENORMAL;
		case HIGH:				return OS_HIGH;
		case REALTIME:			return OS_REALTIME;
		}
		return OS_NORMAL;
	}
	static inline Priority convertPriority(PriorityOS p) {
		switch (p) {
		case OS_IDLE:			return IDLE;
		case OS_LOW:			return LOW;
		case OS_BELOWNORMAL:	return BELOWNORMAL;
		case OS_NORMAL:			return NORMAL;
		case OS_ABOVENORMAL:	return ABOVENORMAL;
		case OS_HIGH:			return HIGH;
		case OS_REALTIME:		return REALTIME;
		}
		return NORMAL;
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_THREAD_H__
