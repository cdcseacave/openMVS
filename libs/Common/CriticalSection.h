////////////////////////////////////////////////////////////////////
// CriticalSection.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_CRITCALSECTION_H__
#define __SEACAVE_CRITCALSECTION_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Thread.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class CriticalSection  
{
#ifdef _MSC_VER

public:
	CriticalSection() {
		InitializeCriticalSection(&cs);
	}
	~CriticalSection() {
		DeleteCriticalSection(&cs);
	}
	void Clear() {
		DeleteCriticalSection(&cs);
		InitializeCriticalSection(&cs);
	}
	void Enter() {
		EnterCriticalSection(&cs);
	}
	bool TryEnter() {
		return (TryEnterCriticalSection(&cs) != 0);
	}
	void Leave() {
		LeaveCriticalSection(&cs);
	}

protected:
	CRITICAL_SECTION cs;

#else

public:
	CriticalSection() {
		#ifdef __APPLE__
		pthread_mutex_init(&mtx, NULL);
		#else
		mtx = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
		#endif
	}
	~CriticalSection() { pthread_mutex_destroy(&mtx); }
	void Clear() {
		pthread_mutex_destroy(&mtx);
		#ifdef __APPLE__
		pthread_mutex_init(&mtx, NULL);
		#else
		mtx = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
		#endif
	}
	void Enter() { pthread_mutex_lock(&mtx); }
	bool TryEnter() { return (pthread_mutex_trylock(&mtx) == 0); }
	void Leave() { pthread_mutex_unlock(&mtx); }
	pthread_mutex_t& getMutex() { return mtx; }

protected:
	pthread_mutex_t mtx;

#endif

private:
	CriticalSection(const CriticalSection&);
	CriticalSection& operator=(const CriticalSection&);
};

/**
* A fast, non-recursive and unfair implementation of the Critical Section.
* It is meant to be used in situations where the risk for lock conflict is very low, 
* i e locks that are held for a very short time. The lock is _not_ recursive, i e if 
* the same thread will try to grab the lock it'll hang in a never-ending loop. The lock
* is not fair, i e the first to try to enter a locked lock is not guaranteed to be the
* first to get it when it's freed...
*/
class FastCriticalSection {
public:
	FastCriticalSection() : state(0) {}

	void Clear() {
		Thread::safeExchange(state, 0);
	}

	void Enter() {
		while (Thread::safeCompareExchange(state, 0, 1) != 0)
			Thread::yield();
	}
	bool TryEnter() {
		return (Thread::safeCompareExchange(state, 0, 1) == 0);
	}
	void Leave() {
		Thread::safeDec(state);
	}

protected:
	volatile Thread::safe_t state;
};

template<class T>
class SimpleLock {
public:
	SimpleLock(T& aCs) : cs(aCs)	{ cs.Enter(); }
	~SimpleLock()					{ cs.Leave(); }
protected:
	T& cs;
};

template<class T>
class SimpleLockTry {
public:
	SimpleLockTry(T& aCs) : cs(aCs)	{ bLocked = cs.TryEnter(); }
	~SimpleLockTry()				{ if (bLocked) cs.Leave(); }
	bool IsLocked() const			{ return bLocked; }
protected:
	T& cs;
	bool bLocked;
};

typedef SimpleLock<CriticalSection> Lock;
typedef SimpleLock<FastCriticalSection> FastLock;
typedef SimpleLockTry<CriticalSection> LockTry;
typedef SimpleLockTry<FastCriticalSection> FastLockTry;

class RWLock
{
public:
	RWLock() : cs(), readers(0) {}
	~RWLock() { ASSERT(readers==0); }
	void Clear() { cs.Clear(); readers = 0; }

	// Read
	void EnterRead() {
		Lock l(cs);
		++readers;
	}

	bool TryEnterRead() {
		LockTry l(cs);
		if (!l.IsLocked())
			return false;
		++readers;
		return true;
	}

	void LeaveRead() {
		Lock l(cs);
		ASSERT(readers > 0);
		--readers;
	}

	bool TryLeaveRead() {
		LockTry l(cs);
		if (!l.IsLocked())
			return false;
		ASSERT(readers > 0);
		--readers;
		return true;
	}

	// Write
	void EnterWrite() {
		cs.Enter();
		while (readers) {
			cs.Leave();
			Thread::yield();
			cs.Enter();
		}
	}

	bool TryEnterWrite() {
		if (cs.TryEnter()) {
			if (readers == 0)
				return true;
			cs.Leave();
		}
		return false;
	}

	void LeaveWrite() {
		cs.Leave();
	}

private:
	RWLock(const RWLock&);
	RWLock& operator=(const RWLock&);

protected:
	CriticalSection cs;
	unsigned readers;
};

class RLock {
public:
	RLock(RWLock& aCs) : cs(aCs)	{ cs.EnterRead(); }
	~RLock()						{ cs.LeaveRead(); }
private:
	RLock(const RLock&);
	RLock& operator=(const RLock&);
protected:
	RWLock& cs;
};

class RLockTry {
public:
	RLockTry(RWLock& aCs) : cs(aCs)	{ bLocked = cs.TryEnterRead(); }
	~RLockTry()						{ if (bLocked) cs.LeaveRead(); }
	bool IsLocked() const			{ return bLocked; }
	bool TryEnter()					{ return (bLocked = cs.TryEnterRead()); }
	bool TryLeave()					{ return !(bLocked = !cs.TryLeaveRead()); }
private:
	RLockTry(const RLockTry&);
	RLockTry& operator=(const RLockTry&);
protected:
	RWLock& cs;
	bool bLocked;
};

class WLock {
public:
	WLock(RWLock& aCs) : cs(aCs)	{ cs.EnterWrite(); }
	~WLock()						{ cs.LeaveWrite(); }
private:
	WLock(const WLock&);
	WLock& operator=(const WLock&);
protected:
	RWLock& cs;
};

class WLockTry {
public:
	WLockTry(RWLock& aCs) : cs(aCs)	{ bLocked = cs.TryEnterWrite(); }
	~WLockTry()						{ if (bLocked) cs.LeaveWrite(); }
	bool IsLocked() const			{ return bLocked; }
	bool TryEnter()					{ return (bLocked = cs.TryEnterWrite()); }
private:
	WLockTry(const WLockTry&);
	WLockTry& operator=(const WLockTry&);
protected:
	RWLock& cs;
	bool bLocked;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_CRITCALSECTION_H__
