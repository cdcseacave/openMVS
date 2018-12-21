////////////////////////////////////////////////////////////////////
// Log.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Log.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

/*-----------------------------------------------------------*
 * Log class implementation                                  *
 *-----------------------------------------------------------*/

#ifndef DEFAULT_LOGTYPE
Log::LogType Log::g_appType;
#endif

/**
 * Constructor
 */
Log::Log()
{
	// generate default log type
	#ifndef DEFAULT_LOGTYPE
	String appName = Util::getAppName();
	appName = (Util::getFileExt(appName) == _T(".exe") ? Util::getFileName(appName) : Util::getFileNameExt(appName));
	Idx n = MINF((Idx)appName.length(), (Idx)LOGTYPE_SIZE);
	_tcsncpy(g_appType.szName, appName, n);
	while (n < LOGTYPE_SIZE)
		g_appType.szName[n++] = _T(' ');
	g_appType.szName[LOGTYPE_SIZE] = _T('\0');
	#endif
	ResetTypes();
}

void Log::RegisterListener(ClbkRecordMsg clbk)
{
	ASSERT(m_arrRecordClbk != NULL);
	if (m_arrRecordClbk == NULL)
		return;
	m_arrRecordClbk->Insert(clbk);
}
void Log::UnregisterListener(ClbkRecordMsg clbk)
{
	ASSERT(m_arrRecordClbk != NULL);
	if (m_arrRecordClbk == NULL)
		return;
	m_arrRecordClbk->Remove(clbk);
}

//Register a new type of log messages (LOGTYPE_SIZE chars)
Log::Idx Log::RegisterType(LPCTSTR lt)
{
	ASSERT(strlen(lt) == LOGTYPE_SIZE);
	const Idx idx = (Idx)m_arrLogTypes.GetSize();
	LogType& logType = m_arrLogTypes.AddEmpty();
	Idx n = MINF((Idx)strlen(lt), (Idx)LOGTYPE_SIZE);
	_tcsncpy(logType.szName, lt, n);
	while (n < LOGTYPE_SIZE)
		logType.szName[n++] = _T(' ');
	logType.szName[LOGTYPE_SIZE] = _T('\0');
	return idx;
}

/**
 * Empty the array with registered log types
 */
void Log::ResetTypes()
{
	m_arrLogTypes.Empty();
}

void Log::Write(LPCTSTR szFormat, ...)
{
	if (m_arrRecordClbk == NULL)
		return;
	va_list args;
	va_start(args, szFormat);
	_Record(NO_ID, szFormat, args);
	va_end(args);
}
void Log::Write(Idx lt, LPCTSTR szFormat, ...)
{
	if (m_arrRecordClbk == NULL)
		return;
	va_list args;
	va_start(args, szFormat);
	_Record(lt, szFormat, args);
	va_end(args);
}

/**
 * Write message to the log if this exists
 * -> IN: Idx - log type
 *        LPCTSTR - format message
 *        ...  - values
 */
void Log::_Record(Idx lt, LPCTSTR szFormat, va_list args)
{
	ASSERT(m_arrRecordClbk != NULL);
	if (m_arrRecordClbk->IsEmpty())
		return;

	// Format a message by adding the date (auto adds new line)
	TCHAR szTime[256];
	TCHAR szBuffer[2048];
	#if defined(LOG_DATE) || defined(LOG_TIME)
	TCHAR* szPtrTime = szTime;
	#ifdef _MSC_VER
	SYSTEMTIME st;
	GetLocalTime(&st);
	#ifdef LOG_THREAD
	WLock l(m_lock);
	#endif
	#ifdef LOG_DATE
	szPtrTime += GetDateFormat(LOCALE_USER_DEFAULT,0,&st,_T("dd'.'MM'.'yy"),szPtrTime,80)-1;
	#endif
	#if defined(LOG_DATE) && defined(LOG_TIME)
	szPtrTime[0] = _T('-'); ++szPtrTime;
	#endif
	#ifdef LOG_TIME
	szPtrTime += GetTimeFormat(LOCALE_USER_DEFAULT,0,&st,_T("HH':'mm':'ss"),szPtrTime,80)-1;
	#endif
	#else // _MSC_VER
	const time_t t = time(NULL);
	const struct tm *tmp = localtime(&t);
	#ifdef LOG_DATE
	szPtrTime += strftime(szPtrTime, 80, "%y.%m.%d", tmp);
	#endif
	#if defined(LOG_DATE) && defined(LOG_TIME)
	szPtrTime[0] = _T('-'); ++szPtrTime;
	#endif
	#ifdef LOG_TIME
	szPtrTime += strftime(szPtrTime, 80, "%H:%M:%S", tmp);
	#endif
	#endif // _MSC_VER
	#endif // LOG_DATE || LOG_TIME
	#ifdef DEFAULT_LOGTYPE
	LPCTSTR const logType(lt<m_arrLogTypes.GetSize() ? m_arrLogTypes[lt] : (LPCSTR)DEFAULT_LOGTYPE);
	#else
	LPCTSTR const logType(lt<m_arrLogTypes.GetSize() ? m_arrLogTypes[lt] : g_appType);
	#endif
	if ((size_t)_vsntprintf(szBuffer, 2048, szFormat, args) > 2048) {
		// not enough space for the full string, reprint dynamically
		m_message.FormatSafe("%s [%s] %s" LINE_SEPARATOR_STR, szTime, logType, String::FormatStringSafe(szFormat, args).c_str());
	} else {
		// enough space for all the string, print directly
		m_message.Format("%s [%s] %s" LINE_SEPARATOR_STR, szTime, logType, szBuffer);
	}
	TRACE(m_message);

	// signal listeners
	FOREACHPTR(pClbk, *m_arrRecordClbk)
		(*pClbk)(m_message);
}
/*----------------------------------------------------------------*/


/*-----------------------------------------------------------*
 * LogFile class implementation                              *
 *-----------------------------------------------------------*/

/**
 * Constructor
 */
LogFile::LogFile()
{
}

bool LogFile::Open(LPCTSTR logName)
{
	Util::ensureFolder(logName);
	m_ptrFile = new File(logName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!m_ptrFile->isOpen()) {
		m_ptrFile = NULL;
		return false;
	}
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogFile::Record, this));
	return true;
}
void LogFile::Close()
{
	if (m_ptrFile == NULL)
		return;
	GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogFile::Record, this));
	m_ptrFile = NULL;
}

void LogFile::Pause()
{
	if (m_ptrFile == NULL)
		return;
	GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogFile::Record, this));
}
void LogFile::Play()
{
	if (m_ptrFile == NULL)
		return;
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogFile::Record, this));
}

void LogFile::Record(const String& msg)
{
	ASSERT(m_ptrFile != NULL);
	m_ptrFile->print(msg);
	m_ptrFile->flush();
}
/*----------------------------------------------------------------*/


/*-----------------------------------------------------------*
 * LogConsole class implementation                           *
 *-----------------------------------------------------------*/

#define MAX_CONSOLE_WIDTH 100
#define MAX_CONSOLE_LINES 2000

#ifdef _MSC_VER
#include <io.h>
#define STDIN_FILENO       0    /* file descriptor for stdin */
#define STDOUT_FILENO      1    /* file descriptor for stdout */
#define STDERR_FILENO      2    /* file descriptor for stderr */
#else
#include <unistd.h>
#endif
#include <fcntl.h>

class LogConsoleOutbuf : public std::streambuf {
public:
	LogConsoleOutbuf() {
		setp(0, 0);
	}

	virtual int_type overflow(int_type c = traits_type::eof()) {
		return fputc(c, stdout) == EOF ? traits_type::eof() : c;
	}
};

/**
 * Constructor
 */
LogConsole::LogConsole()
	:
	#ifdef _USE_COSOLEFILEHANDLES
	m_fileIn(NULL), m_fileOut(NULL), m_fileErr(NULL),
	#else
	m_fileIn(-1), m_fileOut(-1), m_fileErr(-1),
	#endif
	m_cout(NULL), m_coutOld(NULL),
	m_cerr(NULL), m_cerrOld(NULL),
	bManageConsole(false)
{
}

bool LogConsole::IsOpen() const
{
	#ifdef _USE_COSOLEFILEHANDLES
	return (m_fileOut != NULL || m_fileIn != NULL || m_fileErr != NULL);
	#else
	return (m_fileOut != -1 || m_fileIn != -1 || m_fileErr != -1);
	#endif
}

#ifdef _MSC_VER

void LogConsole::Open()
{
	if (IsOpen())
		return;

	// allocate a console for this app
	bManageConsole = (AllocConsole()!=FALSE?true:false);

	// capture std::cout and std::cerr
	if (bManageConsole && !m_cout) {
		// set std::cout to use our custom streambuf
		m_cout = new LogConsoleOutbuf;
		m_coutOld = std::cout.rdbuf(m_cout);
		// use same buffer for std::cerr as well
		m_cerr = NULL;
		m_cerrOld = std::cerr.rdbuf(m_cout);
	}

	// set the screen buffer to be big enough to let us scroll text
	CONSOLE_SCREEN_BUFFER_INFO coninfo;
	GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &coninfo);
	coninfo.dwSize.X = MAX_CONSOLE_WIDTH; // does not resize the console window
	coninfo.dwSize.Y = MAX_CONSOLE_LINES;
	SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), coninfo.dwSize);

	#if 1 && defined(_USE_COSOLEFILEHANDLES)
	// redirect standard stream to the console
	m_fileIn = freopen("CONIN$", "r", stdin);
	m_fileOut = freopen("CONOUT$", "w", stdout);
	// there isn't any CONERR$, so that we merge stderr into CONOUT$
	// http://msdn.microsoft.com/en-us/library/windows/desktop/ms683231%28v=vs.85%29.aspx
	m_fileErr = freopen("CONOUT$", "w", stderr);
	#elif 1 && defined(_USE_COSOLEFILEHANDLES)
	// Fix up the allocated console so that output works in all cases.
	// Change std handles to refer to new console handles. Before doing so, ensure
	// that stdout/stderr haven't been redirected to a valid file
	// See http://support.microsoft.com/kb/105305/en-us
	// stdout
	if (_fileno(stdout) == -1 || _get_osfhandle(fileno(stdout)) == -1) {
		int hCrt = ::_open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
		if (hCrt != -1) {
			m_fileOut = ::_fdopen(hCrt, "w");
			if (m_fileOut)
				*stdout = *m_fileOut;
		}
	}
	// stderr
	if (_fileno(stderr) == -1 || _get_osfhandle(fileno(stderr)) == -1) {
		int hCrt = ::_open_osfhandle((intptr_t)::GetStdHandle(STD_ERROR_HANDLE), _O_TEXT);
		if (hCrt != -1) {
			m_fileErr = ::_fdopen(hCrt, "w");
			if (m_fileErr)
				*stderr = *m_fileErr;
		}
	}
	// stdin
	if (_fileno(stdin) == -1 || _get_osfhandle(fileno(stdin)) == -1) {
		int hCrt = ::_open_osfhandle((intptr_t)::GetStdHandle(STD_INPUT_HANDLE), _O_TEXT);
		if (hCrt != -1) {
			m_fileIn = ::_fdopen(hCrt, "r");
			if (m_fileIn)
				*stdin = *m_fileIn;
		}
	}
	#else
	int hConHandle;
	// redirect unbuffered STDIN to the console
	hConHandle = _open_osfhandle((intptr_t)GetStdHandle(STD_INPUT_HANDLE), _O_TEXT);
	m_fileIn = _dup(STDIN_FILENO);
	_dup2(hConHandle, STDIN_FILENO);
	_close(hConHandle);
	setvbuf(stdin, NULL, _IONBF, 0);
	// redirect unbuffered STDOUT to the console
	hConHandle = _open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	m_fileOut = _dup(STDOUT_FILENO);
	_dup2(hConHandle, STDOUT_FILENO);
	_close(hConHandle);
	setvbuf(stdout, NULL, _IONBF, 0);
	// redirect unbuffered STDERR to the console
	hConHandle = _open_osfhandle((intptr_t)GetStdHandle(STD_ERROR_HANDLE), _O_TEXT);
	m_fileErr = _dup(STDERR_FILENO);
	_dup2(hConHandle, STDERR_FILENO);
	_close(hConHandle);
	setvbuf(stderr, NULL, _IONBF, 0);
	// make cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
	// point to console as well
	std::ios::sync_with_stdio();
	#endif

	// register with our log system
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
}

void LogConsole::Close()
{
	if (!IsOpen())
		return;
	GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
	#ifdef _USE_COSOLEFILEHANDLES
	// close console stream handles
	fclose(m_fileIn); m_fileIn = NULL;
	fclose(m_fileOut); m_fileOut = NULL;
	fclose(m_fileErr); m_fileErr = NULL;
	#else
	// restore STDIN
	_dup2(m_fileIn, STDIN_FILENO);
	_close(m_fileIn);
	m_fileIn = -1;
	// restore STDOUT
	_dup2(m_fileOut, STDOUT_FILENO);
	_close(m_fileOut);
	m_fileOut = -1;
	// restore STDERR
	_dup2(m_fileErr, STDERR_FILENO);
	_close(m_fileErr);
	m_fileErr = -1;
	#endif
	// close console
	if (bManageConsole) {
		if (m_cout) {
			// set std::cout to the original streambuf
			std::cout.rdbuf(m_coutOld);
			std::cerr.rdbuf(m_cerrOld);
			delete m_cout; m_cout = NULL;
		}
		FreeConsole();
	}
	#ifndef _USE_COSOLEFILEHANDLES
	std::ios::sync_with_stdio();
	#endif
}

void LogConsole::Record(const String& msg)
{
	ASSERT(IsOpen());
	printf(msg);
	fflush(stdout);
}

#else

void LogConsole::Open()
{
	if (IsOpen())
		return;
	++m_fileIn;
	// register with our log system
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
}

void LogConsole::Close()
{
	if (!IsOpen())
		return;
	// unregister with our log system
	GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
	--m_fileIn;
}

void LogConsole::Record(const String& msg)
{
	ASSERT(IsOpen());
	printf(_T("%s"), msg.c_str());
	fflush(stdout);
}

#endif // _MSC_VER

void LogConsole::Pause()
{
	if (IsOpen())
		GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
}
void LogConsole::Play()
{
	if (IsOpen())
		GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogConsole::Record, this));
}
/*----------------------------------------------------------------*/
