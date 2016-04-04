////////////////////////////////////////////////////////////////////
// Timer.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_TIMER_H__
#define  __SEACAVE_TIMER_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#ifdef _MSC_VER
//#define TIMER_OLDSUPPORT
#endif
#define FIX_FPS


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class GENERAL_API Timer
{
public:
	typedef int64_t SysType;
	typedef float Type;

public:
	Timer(uint8_t nHH=0, uint8_t nMM=0);
	~Timer();

	void	Update();

	void	SetClock(uint8_t nHH, uint8_t nMM);
	LPTSTR	GetClock(uint8_t* nHH, uint8_t* nMM, LPTSTR szChar) const;

	#ifdef FIX_FPS
	inline uint32_t	GetFPS() const			{ return m_nLastSecFrames; }
	#else
	inline uint32_t	GetFPS() const			{ return (uint32_t)(1.f / m_fElapsedTime); }
	#endif // FIX_FPS

	inline Type		GetRealCurrent() const	{ return SysTime2Time(m_nCrntTime); }
	inline Type		GetCurrent() const		{ return m_fTimeScale * SysTime2Time(m_nCrntTime); }
	inline Type		GetRealElapsed() const	{ return m_fElapsedTime; }
	inline Type		GetElapsed() const		{ return m_fTimeScale * m_fElapsedTime; }
	inline Type		GetDelay(Type d) const	{ return m_fTimeScale * d; }
	inline Type		GetScale() const		{ return m_fTimeScale; }
	inline void		SetScale(Type fFactor)	{ m_fTimeScale = fFactor; }

private:
	SysType		m_nCrntTime;			// current time
	Type		m_fElapsedTime;			// time elapsed since previous frame
	Type		m_fTimeScale;			// slowdown or speedup
	uint8_t		m_nHH;					// clock time hours
	uint8_t		m_nMM;					// clock time minutes
	uint8_t		m_nSS;					// clock time seconds
	Type		m_fClock;				// sum up milliseconds
	#ifdef FIX_FPS
	Type		m_fCurrentFramesTime;	// time elapsed for the current frames
	uint32_t	m_nCurrentFrames;		// counter of the current frames
	uint32_t	m_nLastSecFrames;		// counter of the frames in the last second
	#endif // FIX_FPS

public:
	// get current time in system units
	static inline SysType GetSysTime() {
		#ifdef _MSC_VER
		#ifdef TIMER_OLDSUPPORT
		if (!ms_bPerfFlag)
			return GetTickCount();
		#endif
		SysType nTime;
		QueryPerformanceCounter((LARGE_INTEGER*)&nTime);
		return nTime;
		#else
		timeval tv;
		gettimeofday(&tv, NULL);
		return (tv.tv_sec * 1000000) + tv.tv_usec;
		#endif // _MSC_VER
	}
	// get milliseconds scaling factor for time
	static Type GetTimeFactor();
	// get current time in milliseconds
	static inline Type GetTimeMs() {
		return GetTimeFactor() * GetSysTime();
	}
	// get current time in seconds
	static inline Type GetTime() {
		return 0.001f * GetTimeFactor() * GetSysTime();
	}
	// convert given time to milliseconds
	static inline Type SysTime2TimeMs(SysType t) {
		return GetTimeFactor() * t;
	}
	// convert given time to seconds
	static inline Type SysTime2Time(SysType t) {
		return 0.001f * GetTimeFactor() * t;
	}

protected:
	#ifdef TIMER_OLDSUPPORT
	static const bool	ms_bPerfFlag;	// flag for timer to use
	#endif
	static const Type	ms_fTimeFactor;	// milliseconds scaling factor for time
};
/*----------------------------------------------------------------*/


class GENERAL_API AutoTimer
{
public:
	typedef Timer::Type Type;
public:
	AutoTimer(Type& duration) : m_duration(duration) { m_duration = Timer::GetTime(); }
	~AutoTimer() { m_duration = Timer::GetTime() - m_duration; }
protected:
	Type&	m_duration;
};

class GENERAL_API AutoAddTimer
{
public:
	typedef Timer::Type Type;
public:
	AutoAddTimer(Type& duration) : m_duration(duration) { m_lastTime = Timer::GetTime(); }
	~AutoAddTimer() { m_duration += Timer::GetTime() - m_lastTime; }
protected:
	Type&	m_duration;
	Type	m_lastTime;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_TIMER_H__
