////////////////////////////////////////////////////////////////////
// Timer.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Timer.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

/**
 * Constructor
 */
Timer::Timer(uint8_t nHH, uint8_t nMM)
{
	m_fElapsedTime = 1.f;
	m_fTimeScale = 1.f;
	#ifdef FIX_FPS
	m_fCurrentFramesTime = 0.f;
	m_nCurrentFrames = 0;
	m_nLastSecFrames = 0;
	#endif // FIX_FPS

	// set clock
	m_fClock = 0.f;
	if (nHH > 23)
		nHH = 0;
	if (nMM > 59)
		nMM = 0;
	m_nHH = nHH;
	m_nMM = nMM;
	m_nSS = 0;

	m_nCrntTime = GetSysTime();
} // constructor
/*----------------------------------------------------------------*/


Timer::~Timer()
{
}
/*----------------------------------------------------------------*/


/**
 * Counts the actual time and adjusts clock.
 */
void Timer::Update()
{
	const SysType timeNow = GetSysTime();
	m_fElapsedTime = SysTime2Time(timeNow - m_nCrntTime);
	m_nCrntTime = timeNow;

	#ifdef FIX_FPS
	// compute FPS
	++m_nCurrentFrames;
	if ((m_fCurrentFramesTime += m_fElapsedTime) >= 1.f) {
		m_nLastSecFrames = (uint32_t)((Type)m_nCurrentFrames/m_fCurrentFramesTime);
		m_fCurrentFramesTime = 0.f;
		m_nCurrentFrames = 0;
	}
	#endif // FIX_FPS

	// adjust clock by seconds passed
	m_fClock += (m_fElapsedTime * m_fTimeScale);
	if (m_fClock >= 1.f) {
		m_nSS++;
		m_fClock = 0.f;
	}
	if (m_nSS >= 60) {
		m_nMM++;
		m_nSS = 0;
	}
	if (m_nMM >= 60) {
		m_nHH++;
		m_nMM = 0;
	}
	if (m_nHH >= 24)
		m_nHH = 0;
} // Update
/*----------------------------------------------------------------*/


/**
 * Sets the clock to any given starting time.
 */
void Timer::SetClock(uint8_t nHH, uint8_t nMM)
{
	// set clock
	if (nHH > 23)
		nHH = 0;
	if (nMM > 59)
		nMM = 0;

	m_nHH = nHH;
	m_nMM = nMM;
} // SetClock
/*----------------------------------------------------------------*/


/**
 * Gives you a time string with hours, minutes and seconds as return
 * and hours and/or minutes as reference parameters.
 */
LPTSTR Timer::GetClock(uint8_t* nHH, uint8_t* nMM, LPTSTR szChar) const
{
	if (nHH != NULL)
		*nHH = m_nHH;
	if (nMM != NULL)
		*nMM = m_nMM;

	if (szChar == NULL)
		return NULL;

	_stprintf(szChar, "%.2d:%.2d:%.2d", m_nHH, m_nMM, m_nSS);
	return szChar;
} // GetClock
/*----------------------------------------------------------------*/


#ifdef TIMER_OLDSUPPORT
// Check performance counter
bool HasPerfCounter()
{
	SysType nPerfCnt;
	return QueryPerformanceFrequency((LARGE_INTEGER*)&nPerfCnt) == TRUE;
}
const bool Timer::ms_bPerfFlag = HasPerfCounter();
/*----------------------------------------------------------------*/
#endif


// Get system time factor used to convert to milliseconds
Timer::Type GetSysTimeFactor()
{
	#ifdef _MSC_VER
	Timer::SysType nPerfCnt;
	const BOOL bPerfFlag = QueryPerformanceFrequency((LARGE_INTEGER*)&nPerfCnt);
	#ifdef TIMER_OLDSUPPORT
	// either QueryPerformanceCounter or GetTickCount
	return (bPerfFlag ? 1000.f / nPerfCnt : 1.f);
	#else
	ASSERT(bPerfFlag);
	return 1000.f / nPerfCnt;
	#endif
	#else // _MSC_VER
	return 0.001f;
	#endif // _MSC_VER
}
const Timer::Type Timer::ms_fTimeFactor = GetSysTimeFactor();
/*----------------------------------------------------------------*/

Timer::Type Timer::GetTimeFactor()
{
	return ms_fTimeFactor;
}
/*----------------------------------------------------------------*/
