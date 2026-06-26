////////////////////////////////////////////////////////////////////
// Util.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Util.h"
#ifdef _MSC_VER
#include <Shlobj.h>
#if defined(NTDDI_VERSION) && NTDDI_VERSION >= NTDDI_VISTA
#include <KnownFolders.h>
#endif
#ifndef _USE_WINSDKOS
#define _USE_WINSDKOS
#include <VersionHelpers.h>
#endif
#else
#include <sys/utsname.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#else
#include <sys/sysinfo.h>
#endif
#include <pwd.h>
#endif

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

typedef struct CPUINFO_TYP {
	bool bSSE;        // Streaming SIMD Extensions
	bool bSSE2;       // Streaming SIMD Extensions 2
	bool bSSE3;       // Streaming SIMD Extensions 3
	bool bSSE41;      // Streaming SIMD Extensions 4.1
	bool bSSE42;      // Streaming SIMD Extensions 4.2
	bool bAVX;        // Advanced Vector Extensions
	bool bFMA;        // Fused Multiply�Add
	bool b3DNOW;      // 3DNow! (vendor independent)
	bool b3DNOWEX;    // 3DNow! (AMD specific extensions)
	bool bMMX;        // MMX support
	bool bMMXEX;      // MMX (AMD specific extensions)
	bool bEXT;        // extended features available
	char vendor[13];  // vendor name
	char name[49];    // CPU name
} CPUINFO;


// F U N C T I O N S ///////////////////////////////////////////////

Flags InitCPU();
CPUINFO GetCPUInfo();
bool OSSupportsSSE();
bool OSSupportsAVX();


// G L O B A L S ///////////////////////////////////////////////////

const Flags Util::ms_CPUFNC(InitCPU());


// F U N C T I O N S ///////////////////////////////////////////////

String Util::getHomeFolder()
{
	#ifdef _MSC_VER
	// Use SHGetKnownFolderPath for Windows Vista+ (more modern and reliable)
	#if defined(NTDDI_VERSION) && NTDDI_VERSION >= NTDDI_VISTA
	PWSTR pszPath = NULL;
	if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Profile, 0, NULL, &pszPath))) {
		String dir(toString(pszPath) + PATH_SEPARATOR);
		CoTaskMemFree(pszPath);
		return ensureUnifySlash(dir);
	}
	#endif
	// Fallback to legacy API for older Windows versions
	TCHAR homedir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, homedir, CSIDL_PROFILE, TRUE) == TRUE) {
		String dir(String(homedir) + PATH_SEPARATOR);
		return ensureUnifySlash(dir);
	}
	// Final fallback: try environment variable
	TCHAR* userProfile = _tgetenv(_T("USERPROFILE"));
	if (userProfile != NULL) {
		String dir(String(userProfile) + PATH_SEPARATOR);
		return ensureUnifySlash(dir);
	}
	#else
	// Unix-like systems (Linux, macOS, etc.)
	// First try environment variable
	const char *homedir = getenv("HOME");
	if (homedir != NULL && homedir[0] != '\0') {
		String dir(String(homedir) + PATH_SEPARATOR);
		return ensureUnifySlash(dir);
	}
	// Fallback to getpwuid() with error checking
	struct passwd *pw = getpwuid(getuid());
	if (pw != NULL && pw->pw_dir != NULL && pw->pw_dir[0] != '\0') {
		String dir(String(pw->pw_dir) + PATH_SEPARATOR);
		return ensureUnifySlash(dir);
	}
	// Last resort fallbacks
	#ifdef __APPLE__
	// On macOS, try /Users/<username>
	const char* username = getenv("USER");
	if (username != NULL && username[0] != '\0') {
		String dir(String("/Users/") + String(username) + PATH_SEPARATOR);
		return ensureUnifySlash(dir);
	}
	#endif
	#endif // _MSC_VER
	// If all else fails, return empty string
	return String();
}

String Util::getApplicationFolder()
{
	const auto AddAppName = [](const String& dir) -> String {
		// Append application name to the directory
		String path = dir + PATH_SEPARATOR + _T("OpenMVS");
		ensureValidFolderPath(path);
		ensureFolder(path);
		return path;
	};
	#ifdef _MSC_VER
	// Use SHGetKnownFolderPath for Windows Vista+ (more modern and reliable)
	#if defined(NTDDI_VERSION) && NTDDI_VERSION >= NTDDI_VISTA
	PWSTR pszPath = NULL;
	if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_RoamingAppData, 0, NULL, &pszPath))) {
		String dir(toString(pszPath));
		CoTaskMemFree(pszPath);
		return AddAppName(dir);
	}
	#endif
	// Fallback to legacy API for older Windows versions
	TCHAR appdir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, appdir, CSIDL_APPDATA, TRUE) == TRUE)
		return AddAppName(appdir);
	// Final fallback: try environment variable
	TCHAR* appData = _tgetenv(_T("APPDATA"));
	if (appData != NULL)
		return AddAppName(appData);
	#else
	// Unix-like systems (Linux, macOS, etc.)
	String homeDir = getHomeFolder();
	if (!homeDir.empty()) {
		#ifdef __APPLE__
		// macOS: use ~/Library/Application Support/
		return AddAppName(homeDir + String(_T("Library")) + PATH_SEPARATOR + String(_T("Application Support")));
		#else
		// Linux and other Unix: use ~/.config/ (XDG Base Directory Specification)
		return AddAppName(homeDir + String(_T(".config")));
		#endif
	}
	#endif // _MSC_VER
	// Fallback: return home directory if specific app folder detection fails
	return getHomeFolder();
}

String Util::getCurrentFolder()
{
	TCHAR pathname[MAX_PATH+1];
	#ifdef _MSC_VER
	if (!GetCurrentDirectory(MAX_PATH, pathname))
	#else // _MSC_VER
	if (!getcwd(pathname, MAX_PATH))
	#endif // _MSC_VER
		return String();
	String dir(String(pathname) + PATH_SEPARATOR);
	return ensureUnifySlash(dir);
}
/*----------------------------------------------------------------*/


String Util::GetCPUInfo()
{
	const CPUINFO info(::GetCPUInfo());
	String cpu(info.name[0] == 0 ? info.vendor : info.name);
	#if 0
	if (info.bFMA)
		cpu += _T(" FMA");
	else if (info.bAVX)
		cpu += _T(" AVX");
	else if (info.bSSE42)
		cpu += _T(" SSE4.2");
	else if (info.bSSE41)
		cpu += _T(" SSE4.1");
	else if (info.bSSE3)
		cpu += _T(" SSE3");
	else if (info.bSSE2)
		cpu += _T(" SSE2");
	else if (info.bSSE)
		cpu += _T(" SSE");
	if (info.b3DNOWEX)
		cpu += _T(" 3DNOWEX");
	else if (info.b3DNOW)
		cpu += _T(" 3DNOW");
	#endif
	return cpu;
}

String Util::GetRAMInfo()
{
	const MemoryInfo memInfo(GetMemoryInfo());
	return formatBytes(memInfo.totalPhysical) + _T(" Physical Memory ") + formatBytes(memInfo.totalVirtual) + _T(" Virtual Memory");
}

String Util::GetOSInfo()
{
	#ifdef _MSC_VER

	String os;
	#ifdef _USE_WINSDKOS
	#ifndef _WIN32_WINNT_WIN10
	#define _WIN32_WINNT_WIN10 0x0A00
	if (IsWindowsVersionOrGreater(HIBYTE(_WIN32_WINNT_WIN10), LOBYTE(_WIN32_WINNT_WIN10), 0))
		os = _T("Windows 10+");
	#else
	// helper function to check for Windows 11+
	const auto IsWindows11OrGreater = []() -> bool {
		OSVERSIONINFOEXW osvi { sizeof(OSVERSIONINFOEXW) };
		DWORDLONG dwlConditionMask = 0;
		// Windows 11 starts at build 22000
		osvi.dwMajorVersion = 10;
		osvi.dwMinorVersion = 0;
		osvi.dwBuildNumber = 22000;
		VER_SET_CONDITION(dwlConditionMask, VER_MAJORVERSION, VER_GREATER_EQUAL);
		VER_SET_CONDITION(dwlConditionMask, VER_MINORVERSION, VER_GREATER_EQUAL);
		VER_SET_CONDITION(dwlConditionMask, VER_BUILDNUMBER, VER_GREATER_EQUAL);
		return VerifyVersionInfoW(&osvi,
			VER_MAJORVERSION | VER_MINORVERSION | VER_BUILDNUMBER,
			dwlConditionMask) != FALSE;
	};
	if (IsWindows11OrGreater())
		os = _T("Windows 11+");
	else if (IsWindows10OrGreater())
		os = _T("Windows 10");
	#endif
	else if (IsWindows8Point1OrGreater())
		os = _T("Windows 8.1");
	else if (IsWindows8OrGreater())
		os = _T("Windows 8");
	else if (IsWindows7SP1OrGreater())
		os = _T("Windows 7 (SP1)");
	else if (IsWindows7OrGreater())
		os = _T("Windows 7");
	else if (IsWindowsVistaSP2OrGreater())
		os = _T("Windows Vista (SP2)");
	else if (IsWindowsVistaSP1OrGreater())
		os = _T("Windows Vista (SP1)");
	else if (IsWindowsVistaOrGreater())
		os = _T("Windows Vista");
	else if (IsWindowsXPSP3OrGreater())
		os = _T("Windows XP (SP3)");
	else if (IsWindowsXPSP2OrGreater())
		os = _T("Windows XP (SP2)");
	else if (IsWindowsXPSP1OrGreater())
		os = _T("Windows XP (SP1)");
	else if (IsWindowsXPOrGreater())
		os = _T("Windows XP");
	else
		os = _T("Windows (unknown version)");
	#else
	OSVERSIONINFOEX ver;
	memset(&ver, 0, sizeof(OSVERSIONINFOEX));
	ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);

	if (!GetVersionEx((OSVERSIONINFO*)&ver)) {
		ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
		if (!GetVersionEx((OSVERSIONINFO*)&ver)) {
			return "Windows (unknown version)";
		}
	}

	if (ver.dwPlatformId != VER_PLATFORM_WIN32_NT) {
		os = "Win9x/ME";
	} else {
		switch (ver.dwMajorVersion)
		{
		case 4:
			os = "WinNT4";
			break;

		case 5:
			switch (ver.dwMinorVersion)
			{
			case 0: os = "Win2000"; break;
			case 1: os = "WinXP"; break;
			case 2: os = "Win2003"; break;
			default:os = "Unknown WinNT5";
			}
			break;

		case 6:
			switch (ver.dwMinorVersion)
			{
			case 0: os = (ver.wProductType == VER_NT_WORKSTATION ? "WinVista" : "Win2008"); break;
			case 1: os = (ver.wProductType == VER_NT_WORKSTATION ? "Win7" : "Win2008R2"); break;
			case 2: os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8" : "Win2012"); break;
			case 3: os = (ver.wProductType == VER_NT_WORKSTATION ? "Win8.1" : "Win2012R2"); break;
			case 4: os = "Win10"; break;
			default:os = "Unknown WinNT6";
			}
			break;

		default:
			os = "Windows (version unknown)";
		}
		if (ver.wProductType & VER_NT_WORKSTATION)
			os += " Pro";
		else if (ver.wProductType & VER_NT_SERVER)
			os += " Server";
		else if (ver.wProductType & VER_NT_DOMAIN_CONTROLLER)
			os += " DC";
	}

	if (ver.wServicePackMajor != 0) {
		os += " (SP";
		os += String::ToString(ver.wServicePackMajor);
		if (ver.wServicePackMinor != 0) {
			os += '.';
			os += String::ToString(ver.wServicePackMinor);
		}
		os += ")";
	}
	#endif

	#ifdef _WIN64
	os += " x64";
	#else
	typedef BOOL (WINAPI *LPFN_ISWOW64PROCESS) (HANDLE, PBOOL);
	const LPFN_ISWOW64PROCESS fnIsWow64Process = (LPFN_ISWOW64PROCESS)GetProcAddress(GetModuleHandle("kernel32"),"IsWow64Process");
	BOOL bIsWow64 = FALSE;
	if (fnIsWow64Process && fnIsWow64Process(GetCurrentProcess(),&bIsWow64) && bIsWow64)
		os += " x64";
	#endif

	return os;

	#elif defined(__APPLE__) // _MSC_VER

	// macOS: Get product version name and kernel version
	String osName(_T("macOS"));

	// Get product version (e.g., "26.2")
	char productVersion[256] = {0};
	size_t len = sizeof(productVersion) - 1;
	if (sysctlbyname("kern.osproductversion", productVersion, &len, NULL, 0) == 0 && len > 0) {
		osName += _T(" ") + String(productVersion);
		// Map major version to marketing name (e.g., 14 -> Sonoma, 15 -> Sequoia, 26 -> Tahoe)
		// Starting with 2025, Apple uses year-based versioning (26 = 2025, 27 = 2026, 28 = 2027, etc.)
		const char* marketingName = NULL;
		switch (std::atoi(productVersion)) {
			case 10: case 11: case 12: case 13: marketingName = NULL; break;
			case 14: marketingName = "Sonoma"; break;
			case 15: marketingName = "Sequoia"; break;
			case 26: marketingName = "Tahoe"; break;
			default: marketingName = NULL;  // future versions not yet named
		}
		if (marketingName)
			osName += _T(" (") + String(marketingName) + _T(")");
	}

	// Get architecture
	char machine[256] = {0};
	len = sizeof(machine) - 1;
	if (sysctlbyname("hw.machine", machine, &len, NULL, 0) == 0 && len > 0)
		osName += _T(" (") + String(machine) + _T(")");
	return osName;

	#else // __APPLE__

	// Linux: Get distribution name and kernel version
	String distroName;
	String distroVersion;

	// Try to read /etc/os-release (systemd standard, used by most distributions)
	std::ifstream osRelease("/etc/os-release");
	if (osRelease.is_open()) {
		std::string line;
		while (std::getline(osRelease, line) && (distroName.empty() || distroVersion.empty())) {
			if (line.find("PRETTY_NAME=") == 0) {
				// Extract value between quotes: PRETTY_NAME="Ubuntu 24.04 LTS"
				size_t start = line.find('"');
				size_t end = line.rfind('"');
				if (start != std::string::npos && end != std::string::npos && start < end) {
					distroName = line.substr(start + 1, end - start - 1);
					distroVersion.clear(); // version is already included in PRETTY_NAME
					break; // PRETTY_NAME is the most user-friendly, so we can stop here
				}
			} else if (line.find("NAME=") == 0 && distroName.empty()) {
				size_t start = line.find('"');
				size_t end = line.rfind('"');
				if (start != std::string::npos && end != std::string::npos && start < end)
					distroName = line.substr(start + 1, end - start - 1);
			} else if (line.find("VERSION_ID=") == 0 && distroVersion.empty()) {
				size_t start = line.find('"');
				size_t end = line.rfind('"');
				if (start != std::string::npos && end != std::string::npos && start < end)
					distroVersion = line.substr(start + 1, end - start - 1);
			}
		}
		// Combine name and version if PRETTY_NAME wasn't found
		if (!distroName.empty() && !distroVersion.empty())
			distroName += _T(" ") + distroVersion;
	}

	// Fallback: try /etc/lsb-release (Ubuntu and derivatives)
	if (distroName.empty()) {
		std::ifstream lsbRelease("/etc/lsb-release");
		if (lsbRelease.is_open()) {
			std::string line;
			while (std::getline(lsbRelease, line)) {
				if (line.find("DISTRIB_DESCRIPTION=") == 0) {
					size_t start = line.find('"');
					size_t end = line.rfind('"');
					if (start != std::string::npos && end != std::string::npos && start < end)
						distroName = line.substr(start + 1, end - start - 1);
					else
						distroName = line.substr(20);  // remove "DISTRIB_DESCRIPTION="
				}
				if (line.find("DISTRIB_RELEASE=") == 0)
					distroVersion = line.substr(16);  // remove "DISTRIB_RELEASE="
			}
		}
	}

	// Get kernel information
	utsname n;
	if (uname(&n) != 0)
		return (distroName.empty() ? String(_T("Linux (unknown)")) : distroName);

	// Build final string
	String osInfo;
	if (!distroName.empty())
		osInfo = distroName + _T(" Kernel ") + String(n.release);
	else
		osInfo = String(n.sysname) + _T(" ") + String(n.release);
	osInfo += _T(" (") + String(n.machine) + _T(")");
	return osInfo;

	#endif // _MSC_VER __APPLE__
}

String Util::GetDiskInfo(const String& path)
{
	#if defined(_SUPPORT_CPP17) && (defined(__APPLE__) || !defined(__GNUC__) || (__GNUC__ > 7))

	const std::filesystem::space_info si = std::filesystem::space(path.c_str());
	return String::FormatString("%s (%s) space", formatBytes(si.available).c_str(), formatBytes(si.capacity).c_str());

	#else

	return String();

	#endif // _SUPPORT_CPP17
}
/*----------------------------------------------------------------*/


// Initialize various global variables (ex: random-number-generator state).
void Util::Init()
{
	#ifdef _RELEASE
	const time_t t(Util::getTime());
	std::srand((unsigned)t);
	#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 4)
	cv::setRNGSeed((int)t);
	#endif
	#else
	std::srand((unsigned)0);
	#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 4)
	cv::setRNGSeed((int)0);
	#endif
	#endif
}
/*----------------------------------------------------------------*/


/**
 * Set global variable for availability of SSE instructions.
 */
Flags InitCPU()
{
	const CPUINFO info(GetCPUInfo());
	Flags cpufncs(0);
	if (info.bSSE) {
		#if defined(_MSC_VER) && !defined(_WIN64)
		_set_SSE2_enable(1);
		#endif
		if (OSSupportsSSE())
			cpufncs.set(Util::SSE);
	}
	if (info.bAVX && OSSupportsAVX())
		cpufncs.set(Util::AVX);
	return (cpufncs);
}
/*----------------------------------------------------------------*/


#if _PLATFORM_X86
#ifdef _MSC_VER
#include <intrin.h>
inline void CPUID(int CPUInfo[4], int level) {
	__cpuid(CPUInfo, level);
}
#else
#include <cpuid.h>
inline void CPUID(int CPUInfo[4], int level) {
	unsigned* p((unsigned*)CPUInfo);
	__get_cpuid((unsigned&)level, p+0, p+1, p+2, p+3);
}
#endif
#else // _PLATFORM_X86
inline void CPUID(int CPUInfo[4], int level) {
	memset(CPUInfo, 0, sizeof(int)*4);
}
#endif // _PLATFORM_X86

/**
 * Function to detect SSE availability in CPU.
 */
CPUINFO GetCPUInfo()
{
	CPUINFO info;
	// set all values to 0 (false)
	memset(&info, 0, sizeof(CPUINFO));

	#ifndef __APPLE__
	int CPUInfo[4];

	// CPUID with an InfoType argument of 0 returns the number of
	// valid Ids in CPUInfo[0] and the CPU identification string in
	// the other three array elements. The CPU identification string is
	// not in linear order. The code below arranges the information
	// in a human readable form.
	CPUID(CPUInfo, 0);
	*((int*)info.vendor) = CPUInfo[1];
	*((int*)(info.vendor+4)) = CPUInfo[3];
	*((int*)(info.vendor+8)) = CPUInfo[2];

	// Interpret CPU feature information.
	CPUID(CPUInfo, 1);
	info.bMMX = (CPUInfo[3] & 0x800000) != 0; // test bit 23 for MMX
	info.bSSE = (CPUInfo[3] & 0x2000000) != 0; // test bit 25 for SSE
	info.bSSE2 = (CPUInfo[3] & 0x4000000) != 0; // test bit 26 for SSE2
	info.bSSE3 = (CPUInfo[2] & 0x1) != 0; // test bit 0 for SSE3
	info.bSSE41 = (CPUInfo[2] & 0x80000) != 0; // test bit 19 for SSE4.1
	info.bSSE42 = (CPUInfo[2] & 0x100000) != 0; // test bit 20 for SSE4.2
	info.bAVX = (CPUInfo[2] & 0x18000000) == 0x18000000; // test bits 28,27 for AVX
	info.bFMA = (CPUInfo[2] & 0x18001000) == 0x18001000; // test bits 28,27,12 for FMA

	// EAX=0x80000000 => CPUID returns extended features
	CPUID(CPUInfo, 0x80000000);
	const unsigned nExIds = CPUInfo[0];
	info.bEXT = (nExIds >= 0x80000000);

	// must be greater than 0x80000004 to support CPU name
	if (nExIds > 0x80000004) {
		size_t idx(0);
		CPUID(CPUInfo, 0x80000002); // CPUID returns CPU name part1
		while (((uint8_t*)CPUInfo)[idx] == ' ')
			++idx;
		memcpy(info.name, (uint8_t*)CPUInfo + idx, sizeof(CPUInfo) - idx);
		idx = sizeof(CPUInfo) - idx;

		CPUID(CPUInfo, 0x80000003); // CPUID returns CPU name part2
		memcpy(info.name+idx, CPUInfo, sizeof(CPUInfo));
		idx += 16;

		CPUID(CPUInfo, 0x80000004); // CPUID returns CPU name part3
		memcpy(info.name+idx, CPUInfo, sizeof(CPUInfo));
	}

	if ((strncmp(info.vendor, "AuthenticAMD", 12)==0) && info.bEXT) {  // AMD
		CPUID(CPUInfo, 0x80000001); // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
		info.b3DNOWEX = (CPUInfo[3] & 0x40000000) != 0;	// indicates AMD extended 3DNow+!
		info.bMMXEX = (CPUInfo[3] & 0x400000) != 0; // indicates AMD extended MMX
	}

	#else

    size_t size = sizeof(info.name);
    sysctlbyname("machdep.cpu.brand_string", &info.name, &size, nullptr, 0);

	#endif // __APPLE__

	return info;
}
/*----------------------------------------------------------------*/

#if _PLATFORM_X86
#ifdef _MSC_VER
// Function to detect SSE availability in operating system.
bool OSSupportsSSE()
{
	#ifndef _WIN64
	// try SSE instruction and look for crash
	__try
	{
		_asm xorps xmm0, xmm0
	}
	__except(EXCEPTION_EXECUTE_HANDLER) {
		if (_exception_code() == STATUS_ILLEGAL_INSTRUCTION)
			return false; // sse not supported by os
		return false;     // unknown exception occurred
	}
	#endif // _WIN64

	return true;
}
// Function to detect AVX availability in operating system.
bool OSSupportsAVX()
{
	#ifndef _WIN64
	// try AVX instruction
	unsigned flag;
	_asm {
		mov ecx, 0; //specify 0 for XFEATURE_ENABLED_MASK register
		XGETBV; //result in EDX:EAX
		and eax, 06H;
		cmp eax, 06H; // check OS has enabled both XMM and YMM state support
		jne not_supported
		mov eax, 1; // mark as supported
		jmp done
		not_supported:
		mov eax, 0; // mark as not supported
		done:
		mov esi, flag
		mov [esi], eax
	}
	return flag != 0;
	#else
	// check if the OS will save the YMM registers
	unsigned long long xcrFeatureMask(_xgetbv(_XCR_XFEATURE_ENABLED_MASK));
	return (xcrFeatureMask & 0x6) == 0x6;
	#endif // _WIN64
}
/*----------------------------------------------------------------*/

#else // _MSC_VER

#ifndef _ENVIRONMENT64
// 32-bit x86 only: probe SSE OS-support via a SIGILL handler (see OSSupportsSSE).
#include <csetjmp>
#include <csignal>
namespace {
sigjmp_buf g_sigIllJmp;
void OnSigIll(int) { siglongjmp(g_sigIllJmp, 1); }
}
#endif

// Function to detect SSE availability in operating system.
bool OSSupportsSSE()
{
	#ifdef _ENVIRONMENT64
	// SSE/SSE2 are part of the mandatory x86-64 baseline and are always enabled
	// by any 64-bit OS, so on x86-64 they are guaranteed available here.
	return true;
	#else
	// 32-bit x86: SSE is optional and the OS must enable it (it has to preserve
	// the XMM state across context switches). Probe by executing an SSE
	// instruction under a temporary SIGILL handler: a disabled/unsupported
	// instruction raises a signal — not a C++ exception, which is why the old
	// try/catch could never catch it — recovered via sigsetjmp/siglongjmp.
	struct sigaction sa, old;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &OnSigIll;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIGILL, &sa, &old) != 0)
		return true; // cannot install handler; assume enabled on a modern OS
	volatile bool supported = true;
	if (sigsetjmp(g_sigIllJmp, 1) == 0)
		asm volatile ("xorps %xmm0, %xmm0");
	else
		supported = false;
	sigaction(SIGILL, &old, NULL);
	return supported;
	#endif
}
// Function to detect AVX availability in operating system.
bool OSSupportsAVX()
{
	// check if the OS will save the YMM registers
	unsigned int index(0); //specify 0 for XFEATURE_ENABLED_MASK register
	unsigned int eax, edx;
	__asm__ __volatile__("xgetbv" : "=a"(eax), "=d"(edx) : "c"(index));
	unsigned long long xcrFeatureMask(((unsigned long long)edx << 32) | eax);
	return (xcrFeatureMask & 0x6) == 0x6;
}
/*----------------------------------------------------------------*/
#endif // _MSC_VER

#else // _PLATFORM_X86

// Function to detect SSE availability in operating system.
bool OSSupportsSSE()
{
	return false;
}
// Function to detect AVX availability in operating system.
bool OSSupportsAVX()
{
	return false;
}
/*----------------------------------------------------------------*/
#endif // _PLATFORM_X86


// print details about the current build and PC
void Util::LogBuild()
{
	LOG(_T("OpenMVS ")
		#ifdef _ENVIRONMENT64
		_T("x64")
		#else
		_T("x32")
		#endif
		_T(" v" OpenMVS_VERSION)
		#ifndef _RELEASE
		_T(" (debug)")
		#endif
		_T("%s") OpenMVS_GIT_COMMIT _T("%s"),
		OpenMVS_GIT_COMMIT[0] ? _T(" (") : _T(""),
		OpenMVS_GIT_COMMIT[0] ? (OpenMVS_GIT_MODIFIED ? _T("*)") : _T(")")) : _T("")
	);
	#if TD_VERBOSE == TD_VERBOSE_OFF
	LOG(_T("Build date: ") __DATE__);
	#else
	LOG(_T("Build date: ") __DATE__ _T(", ") __TIME__);
	#endif
	LOG(_T("CPU: %s (%u cores)"), Util::GetCPUInfo().c_str(), Thread::hardwareConcurrency());
	LOG((_T("RAM: ") + Util::GetRAMInfo()).c_str());
	LOG((_T("OS: ") + Util::GetOSInfo()).c_str());
	#ifdef _SUPPORT_CPP17
	LOG((_T("Disk: ") + Util::GetDiskInfo(WORKING_FOLDER_FULL)).c_str());
	#endif
	#ifdef _USE_SSE
	if (!SIMD_ENABLED.isSet(Util::SSE)) LOG(_T("warning: no SSE compatible CPU or OS detected"));
	else if (!SIMD_ENABLED.isSet(Util::AVX)) LOG(_T("warning: no AVX compatible CPU or OS detected"));
	else LOG(_T("SSE & AVX compatible CPU & OS detected"));
	#endif
}

// print information about the memory usage
#ifdef _MSC_VER
#include <Psapi.h>
#pragma comment(lib, "Psapi.lib")
void Util::LogMemoryInfo()
{
	PROCESS_MEMORY_COUNTERS pmc;
	if (!GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
		return;
	LOG(_T("MEMORYINFO: {"));
	LOG(_T("\tPageFaultCount %d"), pmc.PageFaultCount);
	LOG(_T("\tPeakWorkingSetSize %s"), SEACAVE::Util::formatBytes(pmc.PeakWorkingSetSize).c_str());
	LOG(_T("\tWorkingSetSize %s"), SEACAVE::Util::formatBytes(pmc.WorkingSetSize).c_str());
	LOG(_T("\tQuotaPeakPagedPoolUsage %s"), SEACAVE::Util::formatBytes(pmc.QuotaPeakPagedPoolUsage).c_str());
	LOG(_T("\tQuotaPagedPoolUsage %s"), SEACAVE::Util::formatBytes(pmc.QuotaPagedPoolUsage).c_str());
	LOG(_T("\tQuotaPeakNonPagedPoolUsage %s"), SEACAVE::Util::formatBytes(pmc.QuotaPeakNonPagedPoolUsage).c_str());
	LOG(_T("\tQuotaNonPagedPoolUsage %s"), SEACAVE::Util::formatBytes(pmc.QuotaNonPagedPoolUsage).c_str());
	LOG(_T("\tPagefileUsage %s"), SEACAVE::Util::formatBytes(pmc.PagefileUsage).c_str());
	LOG(_T("\tPeakPagefileUsage %s"), SEACAVE::Util::formatBytes(pmc.PeakPagefileUsage).c_str());
	LOG(_T("} ENDINFO"));
}
#elif defined(__APPLE__) // _MSC_VER
#include <mach/mach.h>
void Util::LogMemoryInfo()
{
	// macOS (Intel and Apple Silicon) has no procfs; ask the Mach kernel for
	// the current task's resident/virtual footprint instead.
	mach_task_basic_info_data_t info;
	mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
	if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &count) != KERN_SUCCESS)
		return;
	LOG(_T("MEMORYINFO: {"));
	LOG(_T("\tPeakResidentSize %s"), SEACAVE::Util::formatBytes((int64_t)info.resident_size_max).c_str());
	LOG(_T("\tResidentSize %s"), SEACAVE::Util::formatBytes((int64_t)info.resident_size).c_str());
	LOG(_T("\tVirtualSize %s"), SEACAVE::Util::formatBytes((int64_t)info.virtual_size).c_str());
	LOG(_T("} ENDINFO"));
}
#else // _MSC_VER
void Util::LogMemoryInfo()
{
	std::ifstream proc("/proc/self/status");
	if (!proc.is_open())
		return;
	String s;
	LOG(_T("MEMORYINFO: {"));
	while (std::getline(proc, s), !proc.fail()) {
		if (s.substr(0, 6) == "VmPeak" || s.substr(0, 6) == "VmSize")
			LOG(_T("\t%s"), s.c_str());
	}
	LOG(_T("} ENDINFO"));
}
#endif // _MSC_VER



// get the total & free physical & virtual memory (in bytes)
Util::MemoryInfo Util::GetMemoryInfo()
{
	#if defined(_MSC_VER)                   // windows

	#ifdef _WIN64
	MEMORYSTATUSEX status;
	status.dwLength = sizeof(MEMORYSTATUSEX);
	if (::GlobalMemoryStatusEx(&status) == FALSE) {
		ASSERT(false);
		return MemoryInfo();
	}
	return MemoryInfo(status.ullTotalPhys, status.ullAvailPhys, status.ullTotalVirtual, status.ullAvailVirtual);
	#else
	MEMORYSTATUS status;
	status.dwLength = sizeof(MEMORYSTATUS);
	if (::GlobalMemoryStatus(&status) == FALSE) {
		ASSERT(false);
		return MemoryInfo();
	}
	return MemoryInfo(status.dwTotalPhys, status.dwAvailPhys, status.dwTotalVirtual, status.dwAvailVirtual);
	#endif


	#elif defined(__APPLE__)                // mac

	int mib[2] ={CTL_HW, HW_MEMSIZE};
	size_t len = sizeof(size_t);
	size_t totalMemory;
	if (sysctl(mib, 2, &totalMemory, &len, NULL, 0) < 0) {
		ASSERT(false);
		return MemoryInfo();
	}
	mib[1] = HW_USERMEM;
    size_t freeMemory;
    if (sysctl(mib, 2, &freeMemory, &len, NULL, 0) == -1) {
		ASSERT(false);
        return MemoryInfo();
    }
	return MemoryInfo(totalMemory, freeMemory);

	#else // __GNUC__                       // linux

	struct sysinfo info;
	if (sysinfo(&info) != 0) {
		ASSERT(false);
		return MemoryInfo();
	}
	return MemoryInfo(
		(size_t)info.totalram*(size_t)info.mem_unit,
		(size_t)info.freeram*(size_t)info.mem_unit,
		(size_t)info.totalswap*(size_t)info.mem_unit,
		(size_t)info.freeswap*(size_t)info.mem_unit
	);

	#endif
}
/*----------------------------------------------------------------*/


// Parses a ASCII command line string and returns an array of pointers to the command line arguments,
// along with a count of such arguments, in a way that is similar to the standard C run-time
// argv and argc values.
LPSTR* Util::CommandLineToArgvA(LPCSTR CmdLine, size_t& _argc)
{
	bool   in_QM(false);
	bool   in_TEXT(false);
	bool   in_SPACE(true);

	size_t argc(0);
	size_t len = strlen(CmdLine);
	size_t i = ((len+2)/2)*sizeof(void*) + sizeof(void*);
	LPSTR* argv = (LPSTR*)(new uint8_t[i + (len+2)*sizeof(CHAR)]);
	LPSTR _argv = (LPSTR)(((CHAR*)argv)+i);
	argv[argc] = _argv;
	size_t j(0); i = 0;

	CHAR a;
	while ((a = CmdLine[i]) != 0) {
		if (in_QM) {
			if (a == '\"') {
				in_QM = false;
			} else {
				_argv[j] = a;
				j++;
			}
		} else {
			switch (a) {
				case '\"':
					in_QM = true;
					in_TEXT = true;
					if (in_SPACE) {
						argv[argc] = _argv+j;
						argc++;
					}
					in_SPACE = false;
					break;
				case ' ':
				case '\t':
				case '\n':
				case '\r':
					if (in_TEXT) {
						_argv[j] = '\0';
						j++;
					}
					in_TEXT = false;
					in_SPACE = true;
					break;
				default:
					in_TEXT = true;
					if (in_SPACE) {
						argv[argc] = _argv+j;
						argc++;
					}
					_argv[j] = a;
					j++;
					in_SPACE = false;
					break;
			}
		}
		i++;
	}
	_argv[j] = '\0';
	argv[argc] = NULL;

	_argc = argc;
	return argv;
}
/*----------------------------------------------------------------*/
