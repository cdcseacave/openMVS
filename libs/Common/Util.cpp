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

/**
 * Lookup table (precomputed CRC64 values for each 8 bit string) computation
 * takes into account the fact that the reverse polynom has zeros in lower 8 bits:
 *
 * @code
 *    for (i = 0; i < 256; i++)
 *    {
 *        shiftRegister = i;
 *        for (j = 0; j < 8; j++)
 *        {
 *            if (shiftRegister & 1)
 *                shiftRegister = (shiftRegister >> 1) ^ Reverse_polynom;
 *            else
 *                shiftRegister >>= 1;
 *        }
 *        CRCTable[i] = shiftRegister;
 *    }
 * @endcode
 *
 * Generic code would look as follows:
 *
 * @code
 *    for (i = 0; i < 256; i++)
 *    {
 *        shiftRegister = 0;
 *        bitString = i;
 *        for (j = 0; j < 8; j++)
 *        {
 *            if ((shiftRegister ^ (bitString >> j)) & 1)
 *                shiftRegister = (shiftRegister >> 1) ^ Reverse_polynom;
 *            else
 *                shiftRegister >>= 1;
 *        }
 *        CRCTable[i] = shiftRegister;
 *    }
 * @endcode
 *
 * @remark  Since the lookup table elements have 0 in the lower 32 bit word,
 *          the 32 bit assembler implementation of CRC64Process can be optimized,
 *          avoiding at least one 'xor' operation.
 */
static const uint64_t gs_au64CRC64[256] =
{
	0x0000000000000000ULL, 0x01B0000000000000ULL, 0x0360000000000000ULL, 0x02D0000000000000ULL,
	0x06C0000000000000ULL, 0x0770000000000000ULL, 0x05A0000000000000ULL, 0x0410000000000000ULL,
	0x0D80000000000000ULL, 0x0C30000000000000ULL, 0x0EE0000000000000ULL, 0x0F50000000000000ULL,
	0x0B40000000000000ULL, 0x0AF0000000000000ULL, 0x0820000000000000ULL, 0x0990000000000000ULL,
	0x1B00000000000000ULL, 0x1AB0000000000000ULL, 0x1860000000000000ULL, 0x19D0000000000000ULL,
	0x1DC0000000000000ULL, 0x1C70000000000000ULL, 0x1EA0000000000000ULL, 0x1F10000000000000ULL,
	0x1680000000000000ULL, 0x1730000000000000ULL, 0x15E0000000000000ULL, 0x1450000000000000ULL,
	0x1040000000000000ULL, 0x11F0000000000000ULL, 0x1320000000000000ULL, 0x1290000000000000ULL,
	0x3600000000000000ULL, 0x37B0000000000000ULL, 0x3560000000000000ULL, 0x34D0000000000000ULL,
	0x30C0000000000000ULL, 0x3170000000000000ULL, 0x33A0000000000000ULL, 0x3210000000000000ULL,
	0x3B80000000000000ULL, 0x3A30000000000000ULL, 0x38E0000000000000ULL, 0x3950000000000000ULL,
	0x3D40000000000000ULL, 0x3CF0000000000000ULL, 0x3E20000000000000ULL, 0x3F90000000000000ULL,
	0x2D00000000000000ULL, 0x2CB0000000000000ULL, 0x2E60000000000000ULL, 0x2FD0000000000000ULL,
	0x2BC0000000000000ULL, 0x2A70000000000000ULL, 0x28A0000000000000ULL, 0x2910000000000000ULL,
	0x2080000000000000ULL, 0x2130000000000000ULL, 0x23E0000000000000ULL, 0x2250000000000000ULL,
	0x2640000000000000ULL, 0x27F0000000000000ULL, 0x2520000000000000ULL, 0x2490000000000000ULL,
	0x6C00000000000000ULL, 0x6DB0000000000000ULL, 0x6F60000000000000ULL, 0x6ED0000000000000ULL,
	0x6AC0000000000000ULL, 0x6B70000000000000ULL, 0x69A0000000000000ULL, 0x6810000000000000ULL,
	0x6180000000000000ULL, 0x6030000000000000ULL, 0x62E0000000000000ULL, 0x6350000000000000ULL,
	0x6740000000000000ULL, 0x66F0000000000000ULL, 0x6420000000000000ULL, 0x6590000000000000ULL,
	0x7700000000000000ULL, 0x76B0000000000000ULL, 0x7460000000000000ULL, 0x75D0000000000000ULL,
	0x71C0000000000000ULL, 0x7070000000000000ULL, 0x72A0000000000000ULL, 0x7310000000000000ULL,
	0x7A80000000000000ULL, 0x7B30000000000000ULL, 0x79E0000000000000ULL, 0x7850000000000000ULL,
	0x7C40000000000000ULL, 0x7DF0000000000000ULL, 0x7F20000000000000ULL, 0x7E90000000000000ULL,
	0x5A00000000000000ULL, 0x5BB0000000000000ULL, 0x5960000000000000ULL, 0x58D0000000000000ULL,
	0x5CC0000000000000ULL, 0x5D70000000000000ULL, 0x5FA0000000000000ULL, 0x5E10000000000000ULL,
	0x5780000000000000ULL, 0x5630000000000000ULL, 0x54E0000000000000ULL, 0x5550000000000000ULL,
	0x5140000000000000ULL, 0x50F0000000000000ULL, 0x5220000000000000ULL, 0x5390000000000000ULL,
	0x4100000000000000ULL, 0x40B0000000000000ULL, 0x4260000000000000ULL, 0x43D0000000000000ULL,
	0x47C0000000000000ULL, 0x4670000000000000ULL, 0x44A0000000000000ULL, 0x4510000000000000ULL,
	0x4C80000000000000ULL, 0x4D30000000000000ULL, 0x4FE0000000000000ULL, 0x4E50000000000000ULL,
	0x4A40000000000000ULL, 0x4BF0000000000000ULL, 0x4920000000000000ULL, 0x4890000000000000ULL,
	0xD800000000000000ULL, 0xD9B0000000000000ULL, 0xDB60000000000000ULL, 0xDAD0000000000000ULL,
	0xDEC0000000000000ULL, 0xDF70000000000000ULL, 0xDDA0000000000000ULL, 0xDC10000000000000ULL,
	0xD580000000000000ULL, 0xD430000000000000ULL, 0xD6E0000000000000ULL, 0xD750000000000000ULL,
	0xD340000000000000ULL, 0xD2F0000000000000ULL, 0xD020000000000000ULL, 0xD190000000000000ULL,
	0xC300000000000000ULL, 0xC2B0000000000000ULL, 0xC060000000000000ULL, 0xC1D0000000000000ULL,
	0xC5C0000000000000ULL, 0xC470000000000000ULL, 0xC6A0000000000000ULL, 0xC710000000000000ULL,
	0xCE80000000000000ULL, 0xCF30000000000000ULL, 0xCDE0000000000000ULL, 0xCC50000000000000ULL,
	0xC840000000000000ULL, 0xC9F0000000000000ULL, 0xCB20000000000000ULL, 0xCA90000000000000ULL,
	0xEE00000000000000ULL, 0xEFB0000000000000ULL, 0xED60000000000000ULL, 0xECD0000000000000ULL,
	0xE8C0000000000000ULL, 0xE970000000000000ULL, 0xEBA0000000000000ULL, 0xEA10000000000000ULL,
	0xE380000000000000ULL, 0xE230000000000000ULL, 0xE0E0000000000000ULL, 0xE150000000000000ULL,
	0xE540000000000000ULL, 0xE4F0000000000000ULL, 0xE620000000000000ULL, 0xE790000000000000ULL,
	0xF500000000000000ULL, 0xF4B0000000000000ULL, 0xF660000000000000ULL, 0xF7D0000000000000ULL,
	0xF3C0000000000000ULL, 0xF270000000000000ULL, 0xF0A0000000000000ULL, 0xF110000000000000ULL,
	0xF880000000000000ULL, 0xF930000000000000ULL, 0xFBE0000000000000ULL, 0xFA50000000000000ULL,
	0xFE40000000000000ULL, 0xFFF0000000000000ULL, 0xFD20000000000000ULL, 0xFC90000000000000ULL,
	0xB400000000000000ULL, 0xB5B0000000000000ULL, 0xB760000000000000ULL, 0xB6D0000000000000ULL,
	0xB2C0000000000000ULL, 0xB370000000000000ULL, 0xB1A0000000000000ULL, 0xB010000000000000ULL,
	0xB980000000000000ULL, 0xB830000000000000ULL, 0xBAE0000000000000ULL, 0xBB50000000000000ULL,
	0xBF40000000000000ULL, 0xBEF0000000000000ULL, 0xBC20000000000000ULL, 0xBD90000000000000ULL,
	0xAF00000000000000ULL, 0xAEB0000000000000ULL, 0xAC60000000000000ULL, 0xADD0000000000000ULL,
	0xA9C0000000000000ULL, 0xA870000000000000ULL, 0xAAA0000000000000ULL, 0xAB10000000000000ULL,
	0xA280000000000000ULL, 0xA330000000000000ULL, 0xA1E0000000000000ULL, 0xA050000000000000ULL,
	0xA440000000000000ULL, 0xA5F0000000000000ULL, 0xA720000000000000ULL, 0xA690000000000000ULL,
	0x8200000000000000ULL, 0x83B0000000000000ULL, 0x8160000000000000ULL, 0x80D0000000000000ULL,
	0x84C0000000000000ULL, 0x8570000000000000ULL, 0x87A0000000000000ULL, 0x8610000000000000ULL,
	0x8F80000000000000ULL, 0x8E30000000000000ULL, 0x8CE0000000000000ULL, 0x8D50000000000000ULL,
	0x8940000000000000ULL, 0x88F0000000000000ULL, 0x8A20000000000000ULL, 0x8B90000000000000ULL,
	0x9900000000000000ULL, 0x98B0000000000000ULL, 0x9A60000000000000ULL, 0x9BD0000000000000ULL,
	0x9FC0000000000000ULL, 0x9E70000000000000ULL, 0x9CA0000000000000ULL, 0x9D10000000000000ULL,
	0x9480000000000000ULL, 0x9530000000000000ULL, 0x97E0000000000000ULL, 0x9650000000000000ULL,
	0x9240000000000000ULL, 0x93F0000000000000ULL, 0x9120000000000000ULL, 0x9090000000000000ULL
};


// F U N C T I O N S ///////////////////////////////////////////////

String Util::getHomeFolder()
{
	#ifdef _MSC_VER
	TCHAR homedir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, homedir, CSIDL_PROFILE, TRUE) != TRUE)
		return String();
	#else
	const char *homedir;
	if ((homedir = getenv("HOME")) == NULL)
		homedir = getpwuid(getuid())->pw_dir;
	#endif // _MSC_VER
	String dir(String(homedir) + PATH_SEPARATOR);
	return ensureUnifySlash(dir);
}

String Util::getApplicationFolder()
{
	#ifdef _MSC_VER
	TCHAR appdir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, appdir, CSIDL_APPDATA, TRUE) != TRUE)
		return String();
	String dir(String(appdir) + PATH_SEPARATOR);
	#else
	const char *homedir;
	if ((homedir = getenv("HOME")) == NULL)
		homedir = getpwuid(getuid())->pw_dir;
	String dir(String(homedir) + PATH_SEPARATOR + String(_T(".config")) + PATH_SEPARATOR);
	#endif // _MSC_VER
	return ensureUnifySlash(dir);
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


uint64_t Util::CRC64(const void *pv, size_t cb)
{
	const uint8_t* pu8 = (const uint8_t *)pv;
	uint64_t       uCRC64 = 0ULL;
	while (cb--)
		uCRC64 = gs_au64CRC64[(uCRC64 ^ *pu8++) & 0xff] ^ (uCRC64 >> 8);
	return uCRC64;
}

uint64_t Util::CRC64Process(uint64_t uCRC64, const void *pv, size_t cb)
{
	const uint8_t *pu8 = (const uint8_t *)pv;
	while (cb--)
		uCRC64 = gs_au64CRC64[(uCRC64 ^ *pu8++) & 0xff] ^ (uCRC64 >> 8);
	return uCRC64;
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
/*----------------------------------------------------------------*/

String Util::GetRAMInfo()
{
	#if defined(_MSC_VER)

	#ifdef _WIN64
	MEMORYSTATUSEX memoryStatus;
	memset(&memoryStatus, sizeof(MEMORYSTATUSEX), 0);
	memoryStatus.dwLength = sizeof(memoryStatus);
	::GlobalMemoryStatusEx(&memoryStatus);
	const size_t nTotalPhys((size_t)memoryStatus.ullTotalPhys);
	const size_t nTotalVirtual((size_t)memoryStatus.ullTotalVirtual);
	#else
	MEMORYSTATUS memoryStatus;
	memset(&memoryStatus, sizeof(MEMORYSTATUS), 0);
	memoryStatus.dwLength = sizeof(MEMORYSTATUS);
	::GlobalMemoryStatus(&memoryStatus);
	const size_t nTotalPhys((size_t)memoryStatus.dwTotalPhys);
	const size_t nTotalVirtual((size_t)memoryStatus.dwTotalVirtual);
	#endif

	#elif defined(__APPLE__)

	int mib[2] = {CTL_HW, HW_MEMSIZE};
	const unsigned namelen = sizeof(mib) / sizeof(mib[0]);
	size_t len = sizeof(size_t);
	size_t nTotalPhys;
	sysctl(mib, namelen, &nTotalPhys, &len, NULL, 0);
	const size_t nTotalVirtual(nTotalPhys);

	#else // __GNUC__

	struct sysinfo info;
	sysinfo(&info);
	const size_t nTotalPhys((size_t)info.totalram);
	const size_t nTotalVirtual((size_t)info.totalswap);

	#endif // _MSC_VER
	return formatBytes(nTotalPhys) + _T(" Physical Memory ") + formatBytes(nTotalVirtual) + _T(" Virtual Memory");
}
/*----------------------------------------------------------------*/

String Util::GetOSInfo()
{
	#ifdef _MSC_VER

	String os;
	#ifdef _USE_WINSDKOS
	#ifndef _WIN32_WINNT_WIN10
	#define _WIN32_WINNT_WIN10 0x0A00
	if (IsWindowsVersionOrGreater(HIBYTE(_WIN32_WINNT_WIN10), LOBYTE(_WIN32_WINNT_WIN10), 0))
	#else
	if (IsWindows10OrGreater())
	#endif
		os = _T("Windows 10+");
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

	#else // _MSC_VER

	utsname n;
	if (uname(&n) != 0)
		return "linux (unknown version)";
	return String(n.sysname) + " " + String(n.release) + " (" + String(n.machine) + ")";

	#endif // _MSC_VER
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

/**
 * Function to detect SSE availability in CPU.
 */
CPUINFO GetCPUInfo()
{
	CPUINFO info;

	// set all values to 0 (false)
	memset(&info, 0, sizeof(CPUINFO));

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

	return info;
}
/*----------------------------------------------------------------*/

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

// Function to detect SSE availability in operating system.
bool OSSupportsSSE()
{
	// try SSE instruction and look for crash
	try {
		asm("xorps %xmm0, %xmm0");
	}
	catch(int e) {
		return false;     // unknown exception occurred
	}
	return true;
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


// print details about the current build and PC
void Util::LogBuild()
{
	#if TD_VERBOSE == TD_VERBOSE_OFF
	LOG(_T("Build date: ") __DATE__);
	#else
	LOG(_T("Build date: ") __DATE__ _T(", ") __TIME__);
	#endif
	LOG((_T("CPU: ") + Util::GetCPUInfo()).c_str());
	LOG((_T("RAM: ") + Util::GetRAMInfo()).c_str());
	LOG((_T("OS: ") + Util::GetOSInfo()).c_str());
	if (!SIMD_ENABLED.isSet(Util::SSE)) LOG(_T("warning: no SSE compatible CPU or OS detected"));
	else if (!SIMD_ENABLED.isSet(Util::AVX)) LOG(_T("warning: no AVX compatible CPU or OS detected"));
	else LOG(_T("SSE & AVX compatible CPU & OS detected"));
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
