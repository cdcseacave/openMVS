////////////////////////////////////////////////////////////////////
// Log.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_LOG_H__
#define __SEACAVE_LOG_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

// use file handles to access console
#define _USE_COSOLEFILEHANDLES

//#define LOG_DATE // add date info for every log
#define LOG_TIME // add time info for every log
#define LOG_THREAD // make log multi-thread safe
#define LOG_STREAM // add stream support (operator <<)
#define LOGTYPE_SIZE	8
#define DEFAULT_LOGTYPE	_T("App     ")

#define DECLARE_LOG() \
	protected: static const Log::Idx ms_nLogType;
#define DEFINE_LOG(classname, log) \
	const Log::Idx classname::ms_nLogType(REGISTER_LOG(log));

#ifdef LOG_THREAD
#include "CriticalSection.h"
#endif


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class GENERAL_API Log
{
	DECLARE_SINGLETON(Log);

public:
	typedef uint32_t Idx;
	typedef DELEGATE<void (const String&)> ClbkRecordMsg;
	typedef cList<ClbkRecordMsg> ClbkRecordMsgArray;
	typedef CSharedPtr<ClbkRecordMsgArray> ClbkRecordMsgArrayPtr;

public:
	// log methods
	void		Open() { m_arrRecordClbk = new ClbkRecordMsgArray; }
	void		Close() { m_arrRecordClbk = NULL; }
	void		Join(Log& log) { m_arrRecordClbk = log.m_arrRecordClbk; }
	void		RegisterListener(ClbkRecordMsg);
	void		UnregisterListener(ClbkRecordMsg);
	Idx 		RegisterType(LPCTSTR);
	void		ResetTypes();
	void		Write(LPCTSTR, ...);
	void		Write(Idx, LPCTSTR, ...);

	#ifdef LOG_STREAM
	template<class T> inline Log& operator<<(const T& val) {
		#ifdef LOG_THREAD
		Lock l(m_cs);
		std::ostringstream& ostr = m_streams[__THREAD__];
		#else
		std::ostringstream& ostr = m_stream;
		#endif
		ostr << val;
		const std::string& line = ostr.str();
		if (!line.empty() && *(line.end()-1) == _T('\n')) {
			Write(line.substr(0, line.size()-1).c_str());
			ostr.str(_T(""));
		}
		return *this;
	}
	// the type of std::cout
	typedef std::basic_ostream<char, std::char_traits<char> > CoutType;
	// the function signature of std::endl
	typedef CoutType& (*StandardEndLine)(CoutType&);
	// define an operator<< to take in std::endl
	inline Log& operator<<(StandardEndLine) {
		#ifdef LOG_THREAD
		Lock l(m_cs);
		std::ostringstream& ostr = m_streams[__THREAD__];
		#else
		std::ostringstream& ostr = m_stream;
		#endif
		Write(ostr.str().c_str());
		ostr.str(_T(""));
		return *this;
	}
	#endif

protected:
	// write a message of a certain type to the log
	void		_Record(Idx, LPCTSTR, va_list); 

protected:
	struct LogType {
		TCHAR szName[LOGTYPE_SIZE+1];
		inline operator LPCTSTR () const { return szName; }
		inline operator LPTSTR () { return szName; }
	};
	typedef cList<LogType, const LogType&, 0, 8> LogTypeArr;

	// log members
	String					m_message;		// last recorded message
	ClbkRecordMsgArrayPtr	m_arrRecordClbk;// the array with all registered listeners
	LogTypeArr				m_arrLogTypes;	// the array with all the registered log types

	#ifdef LOG_THREAD
	// threading
	RWLock					m_lock;			// mutex used to ensure multi-thread safety
	#endif

	#ifdef LOG_STREAM
	// streaming
	#ifdef LOG_THREAD
	typedef std::unordered_map<unsigned,std::ostringstream> StreamMap;
	StreamMap				m_streams;		// stream object used to handle one log with operator << (one for each thread)
	CriticalSection			m_cs;			// mutex used to ensure multi-thread safety for accessing m_streams
	#else
	std::ostringstream		m_stream;		// stream object used to handle one log with operator <<
	#endif
	#endif

	// static
	#ifndef DEFAULT_LOGTYPE
	static LogType			g_appType;
	#endif
};
#define GET_LOG()			SEACAVE::Log::GetInstance()
#define OPEN_LOG()			GET_LOG().Open()
#define CLOSE_LOG()			GET_LOG().Close()
#define JOIN_LOG(log)		GET_LOG().Join(log)
#define REGISTER_LOG(lt)	GET_LOG().RegisterType(lt)
#define LOG					GET_LOG().Write
#define SLOG(msg)			GET_LOG() << msg
#ifndef _RELEASE
#define LOGV				LOG // include extra details in the log
#else
#define LOGV(...)
#endif
/*----------------------------------------------------------------*/


class GENERAL_API LogFile
{
	DECLARE_SINGLETON(LogFile);

public:
	~LogFile() { Close(); }

	// log methods
	bool			Open(LPCTSTR);
	void			Close();
	void			Pause();
	void			Play();
	void			Record(const String&); 

protected:
	FilePtr			m_ptrFile;		// the log file
};
#define GET_LOGFILE()		LogFile::GetInstance()
#define OPEN_LOGFILE(log)	GET_LOGFILE().Open(log)
#define CLOSE_LOGFILE()		GET_LOGFILE().Close()
/*----------------------------------------------------------------*/


class GENERAL_API LogConsole
{
	DECLARE_SINGLETON(LogConsole);

public:
	~LogConsole() { Close(); }

	bool			IsOpen() const;

	// log methods
	void			Open();
	void			Close();
	void			Pause();
	void			Play();
	void			Record(const String&);

protected:
	#ifdef _USE_COSOLEFILEHANDLES
	typedef FILE* StreamHandle;
	#else
	typedef int StreamHandle;
	#endif
	StreamHandle	m_fileIn;		// the log file in
	StreamHandle	m_fileOut;		// the log file out
	StreamHandle	m_fileErr;		// the log file error
	std::streambuf*	m_cout;			// the redirected cout stream
	std::streambuf*	m_coutOld;		// the original cout stream
	std::streambuf*	m_cerr;			// the redirected cerr stream
	std::streambuf*	m_cerrOld;		// the original cout stream
	bool			bManageConsole;	// remember if the console is created here or is an existing console
};
#define GET_LOGCONSOLE()	LogConsole::GetInstance()
#define OPEN_LOGCONSOLE()	GET_LOGCONSOLE().Open()
#define CLOSE_LOGCONSOLE()	GET_LOGCONSOLE().Close()
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LOG_H__
