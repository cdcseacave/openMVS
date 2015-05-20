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
#else
#include <sys/utsname.h>
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
	bool bFMA;        // Fused Multiply–Add
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

String Util::emptyString;
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
const uint64_t Util::ms_au64CRC64[256] =
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

String Util::getHomeDirectory()
{
	#ifdef _MSC_VER
	TCHAR homedir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, homedir, CSIDL_PROFILE, TRUE) != TRUE)
		return emptyString;
	#else
	const char *homedir;
	if ((homedir = getenv("HOME")) == NULL)
		homedir = getpwuid(getuid())->pw_dir;
	#endif // _MSC_VER
	String dir(String(homedir) + PATH_SEPARATOR);
	return ensureUnifySlash(dir);
}

String Util::getAppplicationsDirectory()
{
	#ifdef _MSC_VER
	TCHAR appdir[MAX_PATH];
	if (SHGetSpecialFolderPath(0, appdir, CSIDL_APPDATA, TRUE) != TRUE)
		return emptyString;
	String dir(String(appdir) + PATH_SEPARATOR);
	#else
	const char *homedir;
	if ((homedir = getenv("HOME")) == NULL)
		homedir = getpwuid(getuid())->pw_dir;
	String dir(String(homedir) + PATH_SEPARATOR + String(_T(".config")) + PATH_SEPARATOR);
	#endif // _MSC_VER
	return ensureUnifySlash(dir);
}

String Util::getCurrentDirectory()
{
	#ifdef _MSC_VER
	TCHAR buf[MAX_PATH+1];
	GetCurrentDirectory(MAX_PATH, buf);
	return ensureUnifySlash(String(buf)) + PATH_SEPARATOR;
	#else // _MSC_VER
	char* home = getenv("PATH");
	if (!home)
		return emptyString;
	return ensureUnifySlash(String(home) + PATH_SEPARATOR);
	#endif // _MSC_VER
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
	#ifdef _MSC_VER

	MEMORYSTATUS memoryStatus;
	memset(&memoryStatus, sizeof (MEMORYSTATUS), 0);
	memoryStatus.dwLength = sizeof (MEMORYSTATUS);

	// This function doesn't return a value.
	::GlobalMemoryStatus(&memoryStatus);

	return formatBytes(memoryStatus.dwTotalPhys) + _T(" Physical Memory ") + formatBytes(memoryStatus.dwTotalVirtual) + _T(" Virtual Memory");

	#else // _MSC_VER
	return emptyString;
	#endif // _MSC_VER
}
/*----------------------------------------------------------------*/

String Util::GetOSInfo()
{
	#ifdef _MSC_VER

	OSVERSIONINFOEX ver;
	memset(&ver, 0, sizeof(OSVERSIONINFOEX));
	ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFOEX);

	if (!GetVersionEx((OSVERSIONINFO*)&ver)) {
		ver.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
		if (!GetVersionEx((OSVERSIONINFO*)&ver)) {
			return "Windows (version unknown)";
		}
	}

	String os;
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

	#ifdef _WIN64
	os += " x64";
	#else
	typedef BOOL (WINAPI *LPFN_ISWOW64PROCESS) (HANDLE, PBOOL);
	const LPFN_ISWOW64PROCESS fnIsWow64Process = (LPFN_ISWOW64PROCESS)GetProcAddress(GetModuleHandle("kernel32"),"IsWow64Process");
	BOOL bIsWow64 = FALSE;
	if (fnIsWow64Process && fnIsWow64Process(GetCurrentProcess(),&bIsWow64) && bIsWow64)
		os += " x64";
	#endif

	if (ver.wServicePackMajor != 0) {
		os += " SP";
		os += String::ToString(ver.wServicePackMajor);
		if (ver.wServicePackMinor != 0) {
			os += '.';
			os += String::ToString(ver.wServicePackMinor);
		}
	}

	return os;

	#else // _MSC_VER
	utsname n;

	if (uname(&n) != 0) {
		return "unix (unknown version)";
	}

	return String(n.sysname) + " " + String(n.release) + " (" + String(n.machine) + ")";
	#endif // _MSC_VER
}
/*----------------------------------------------------------------*/

uint64_t Util::GetHardwareFingerprint()
{
	#ifdef _MSC_VER
	QueryWMI query;
	if (!query.IsOpen())
		return 0;
	const String hfp("CPU: " + query.cpuId() + "\nBIOS: " + query.biosId() + "\nBASE: " + query.baseId()
		//+ "\nDISK: "+ query.diskId() + "\nVIDEO: " + query.videoId() + "\nMAC: "+ query.macId()
	);
	return CRC64(hfp.c_str(), hfp.length());
	#else // _MSC_VER
	//TODO: implement
	exit(-1);
	return 0;
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

/**
 * Function to detect SSE availability in CPU.
 */
CPUINFO GetCPUInfo()
{
	CPUINFO info;

	// set all values to 0 (false)
	memset(&info, 0, sizeof(CPUINFO));

	#ifdef _WIN64

	int CPUInfo[4];

	// __cpuid with an InfoType argument of 0 returns the number of
	// valid Ids in CPUInfo[0] and the CPU identification string in
	// the other three array elements. The CPU identification string is
	// not in linear order. The code below arranges the information
	// in a human readable form.
	__cpuid(CPUInfo, 0);
	*((int*)info.vendor) = CPUInfo[1];
	*((int*)(info.vendor+4)) = CPUInfo[3];
	*((int*)(info.vendor+8)) = CPUInfo[2];

	// Interpret CPU feature information.
	__cpuid(CPUInfo, 1);
	info.bMMX = (CPUInfo[3] & 0x800000) != 0; // test bit 23 for MMX
	info.bSSE = (CPUInfo[3] & 0x2000000) != 0; // test bit 25 for SSE
	info.bSSE2 = (CPUInfo[3] & 0x4000000) != 0; // test bit 26 for SSE2
	info.bSSE3 = (CPUInfo[2] & 0x1) != 0; // test bit 0 for SSE3
	info.bSSE41 = (CPUInfo[2] & 0x80000) != 0; // test bit 19 for SSE4.1
	info.bSSE42 = (CPUInfo[2] & 0x100000) != 0; // test bit 20 for SSE4.2
	info.bAVX = (CPUInfo[2] & 0x18000000) == 0x18000000; // test bits 28,27 for AVX
	info.bFMA = (CPUInfo[2] & 0x18001000) == 0x18001000; // test bits 28,27,12 for FMA

	// EAX=0x80000000 => CPUID returns extended features
	__cpuid(CPUInfo, 0x80000000);
	const unsigned nExIds = CPUInfo[0];
	info.bEXT = (nExIds >= 0x80000000);

	// must be greater than 0x80000004 to support CPU name
	if (nExIds > 0x80000004) {
		size_t idx(0);
		__cpuid(CPUInfo, 0x80000002); // CPUID returns CPU name part1
		while (((uint8_t*)CPUInfo)[idx] == ' ')
			++idx;
		memcpy(info.name, (uint8_t*)CPUInfo + idx, sizeof(CPUInfo) - idx);
		idx = sizeof(CPUInfo) - idx;

		__cpuid(CPUInfo, 0x80000003); // CPUID returns CPU name part2
		memcpy(info.name+idx, CPUInfo, sizeof(CPUInfo));
		idx += 16;

		__cpuid(CPUInfo, 0x80000004); // CPUID returns CPU name part3
		memcpy(info.name+idx, CPUInfo, sizeof(CPUInfo));
	}

	if ((strncmp(info.vendor, "AuthenticAMD", 12)==0) && info.bEXT) {  // AMD
		__cpuid(CPUInfo, 0x80000001); // CPUID will copy ext. feat. bits to EDX and cpu type to EAX
		info.b3DNOWEX = (CPUInfo[3] & 0x40000000) != 0;	// indicates AMD extended 3DNow+!
		info.bMMXEX = (CPUInfo[3] & 0x400000) != 0; // indicates AMD extended MMX
	}

	#else //_WIN64

	UINT n=1;
	UINT *pn = &n;
	char* pStr;

	// 1: See if we can get CPUID and vendor 's name then
	//    check for SSE and MMX Support (vendor independant)
	pStr = info.vendor;
	__try {
		__asm {
			mov  eax, 0			// eax=0 => CPUID returns vendor name
			CPUID				// perform CPUID function

			mov  esi,     pStr
			mov  [esi],   ebx	// first 4 chars
			mov  [esi+4], edx	// next for chars
			mov  [esi+8], ecx	// last 4 chars

			mov  eax, 1			// EAX=1 => CPUID returns feature bits
			CPUID				// perform CPUID (puts feature info to EDX)

			test ecx, 00100000h	// test bit 20 for SSE4.2
			jz   _NOSSE42		// if test failed jump
			mov  [info.bSSE42],1// set to true

_NOSSE42:	test ecx, 00080000h	// test bit 19 for SSE4.1
			jz   _NOSSE41		// if test failed jump
			mov  [info.bSSE41],1// set to true

_NOSSE41:	test ecx, 00000001h	// test bit 0 for SSE3
			jz   _NOSSE3		// if test failed jump
			mov  [info.bSSE3], 1// set to true

_NOSSE3:	test edx, 04000000h	// test bit 26 for SSE2
			jz   _NOSSE2		// if test failed jump
			mov  [info.bSSE2], 1// set to true

_NOSSE2:	test edx, 02000000h	// test bit 25 for SSE
			jz   _NOSSE			// if test failed jump
			mov  [info.bSSE], 1	// set to true

_NOSSE:		test edx, 00800000h	// test bit 23 for MMX
			jz   _EXIT1			// if test failed jump
			mov  [info.bMMX], 1	// set to true
_EXIT1:  // nothing to do anymore
		}
	}
	__except(EXCEPTION_EXECUTE_HANDLER) {
		if (_exception_code() == STATUS_ILLEGAL_INSTRUCTION)
			return info;	// cpu inactive
		return info;			// unexpected exception occurred
	}

	// 2: See if we can get extended info (vendor independent)
	pStr = info.name;
	_asm {
		mov  eax, 80000000h		// EAX=0x80000000 => CPUID returns extended features
		CPUID

		cmp  eax, 80000000h		// must be greater than 0x80000000
		jbe  _EXIT2				// below or equal 0x80000000 then jump away
		mov [info.bEXT], 1		// set to true

		cmp  eax, 80000004h		// must be greater than 0x80000004 to support CPU name
		jb   _EXT1				// below 0x80000004 then jump away

		mov  esi, pStr

		mov  eax, 80000002h		// EAX=0x80000002 => CPUID returns CPU name part1
		CPUID
		mov  [esi],    eax		// first 4 chars
		mov  [esi+4],  ebx		// next 4 chars
		mov  [esi+8],  ecx		// next 4 chars
		mov  [esi+12], edx		// last 4 chars

		mov  eax, 80000003h		// EAX=0x80000003 => CPUID returns CPU name part2
		CPUID
		mov  [esi+16], eax		// first 4 chars
		mov  [esi+20], ebx		// next 4 chars
		mov  [esi+24], ecx		// next 4 chars
		mov  [esi+28], edx		// last 4 chars

		mov  eax, 80000004h		// EAX=0x80000004 => CPUID returns CPU name part3
		CPUID
		mov  [esi+32], eax		// first 4 chars
		mov  [esi+36], ebx		// next 4 chars
		mov  [esi+40], ecx		// next 4 chars
		mov  [esi+44], edx		// last 4 chars

_EXT1:	mov  eax, 80000001h		// EAX=0x80000001 => CPUID will copy ext. feat. bits to EDX
		CPUID
		test edx, 80000000h		// 0x80000000 indicates 3DNow!support
		jz   _EXIT2				// if failed jump away
		mov  [info.b3DNOW], 1	// set to true
_EXIT2:
	}

	// 3: Get vendor specific stuff
	//    INTEL: CPU id
	//    AMD:   CPU id, 3dnow_ex, mmx_ex
	//if ((strncmp(info.vendor, "GenuineIntel", 12)==0) && info.bEXT) {  // INTEL
	//	_asm {
	//		mov  eax, 1			// EAX=1 => CPUID will copy ext. feat. info
	//		CPUID				//          to EDX and brand id to EBX
	//		mov  esi,   pn		// get brand id which is only supported
	//		mov  [esi], ebx		// by processors > PIII/Celeron
	//	}
	//	n &= 0xff;				// id only needs lowest 8 bits
	//} else
	if ((strncmp(info.vendor, "AuthenticAMD", 12)==0) && info.bEXT) {  // AMD
		_asm {
			mov  eax, 1			// EAX=1 => CPUID will copy ext. feat. info
			CPUID				//          to EDX and brand id to EAX
			mov  esi,   pn		// get cpu type
			mov  [esi], eax

			mov  eax, 0x80000001// EAX=0x80000001 => CPUID will copy ext. feat. bits
			CPUID				//                   to EDX and cpu type to EAX

			test edx, 0x40000000	// 0x40000000 indicates AMD extended 3DNow+!
			jz   _AMD1				// if failed jump away
			mov  [info.b3DNOWEX], 1 // set to true
_AMD1:		test edx, 0x00400000	// 0x00400000 indicates AMD extended MMX
			jz   _AMD2				// if fail jump away
			mov  [info.bMMXEX], 1	// set to true
_AMD2:
		}
	}

	#endif // _WIN64

	return info;
}
/*----------------------------------------------------------------*/

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
	UINT flag;
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
	unsigned long long xcrFeatureMask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
	return (xcrFeatureMask & 0x6) == 0x6;
	#endif // _WIN64
}
/*----------------------------------------------------------------*/

#else // _MSC_VER

void cpuid(int *a, int *b, int *c, int *d)
{
	int keeper = 0;
	__asm__ __volatile__ (" mov %5, %%eax;"
						" mov %6, %%ecx;"
						" mov %%ebx, %0;"
						" cpuid;"
						" mov %%eax, %1;"
						" mov %%ebx, %2;"
						" mov %%ecx, %3;"
						" mov %%edx, %4;"
						" mov %7, %%ebx"
		 /* Output */ : "=r" (keeper), "=a" (*a), "=r" (*b), "=c" (*c), "=d" (*d)
		 /* Input  */ : "a" (*a), "c" (*c), "r" (keeper) );
}

bool bit_check(int x, int bit)
{
	return (((x >> bit) & 0x01) != 0);
}

int read_val(int reg, int startbit, int endbit)
{
	int i,val=0;
	for (i=0; i<=(endbit-startbit); i++)
	{
		if (bit_check(reg,startbit+i))
			val=val+pow(2,i);
	}
	return val;
}

/**
 * Function to detect SSE availability in CPU.
 */
CPUINFO GetCPUInfo()
{
	int a=0,b=0,c=0,d=0;
	CPUINFO info;

	// set all values to 0 (false)
	memset(&info, 0, sizeof(CPUINFO));

	// 1: See if we can get CPUID and vendor 's name then
	//    check for SSE and MMX Support (vendor independent)
	// eax=0 => CPUID returns vendor name
	a=0; cpuid(&a,(int*)info.vendor+0,(int*)info.vendor+2,(int*)info.vendor+1);

	// EAX=1 => CPUID returns feature bits
	// ecx test bit 20 for SSE4.2
	// ecx test bit 19 for SSE4.1
	// ecx test bit 0 for SSE3
	// edx test bit 26 for SSE2
	// edx test bit 25 for SSE
	// edx test bit 23 for MMX
	a=1; cpuid(&a,&b,&c,&d);
	info.bSSE42 = bit_check(c, 20);
	info.bSSE41 = bit_check(c, 19);
	info.bSSE3  = bit_check(c, 0);
	info.bSSE2  = bit_check(d, 26);
	info.bSSE   = bit_check(d, 25);
	info.bMMX   = bit_check(d, 23);
	info.bAVX   = bit_check(c, 28);
	info.bFMA   = bit_check(c, 12);

	//TODO: finish

	return info;
}
/*----------------------------------------------------------------*/

// Function to detect SSE availability in operating system.
bool OSSupportsSSE()
{
	// try SSE instruction and look for crash
	try
	{
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
	// try AVX instruction
	UINT flag;
	__asm__(
		"mov ecx, 0; //specify 0 for XFEATURE_ENABLED_MASK register"
		"XGETBV; //result in EDX:EAX"
		"and eax, 06H;"
		"cmp eax, 06H; // check OS has enabled both XMM and YMM state support"
		"jne not_supported"
		"mov eax, 1; // mark as supported"
		"jmp done"
		"not_supported:"
		"mov eax, 0; // mark as not supported"
		"done:"
		"mov esi, flag"
		"mov [esi], eax"
	);
	return flag != 0;
}
/*----------------------------------------------------------------*/
#endif // _MSC_VER


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
	std::ostringstream mem;
	std::ifstream proc("/proc/self/status");
	String s;
	while (std::getline(proc, s), !proc.fail()) {
		if(s.substr(0, 6) == "VmSize") {
			mem << s;
			break;
		}
	}
	LOG(_T("MEMORYINFO: {"));
	LOG(mem.str().c_str());
	LOG(_T("} ENDINFO"));
}
#endif // _MSC_VER


// Parses a ASCII command line string and returns an array of pointers to the command line arguments,
// along with a count of such arguments, in a way that is similar to the standard C run-time
// argv and argc values.
LPSTR* Util::CommandLineToArgvA(LPCSTR CmdLine, size_t& _argc)
{
	LPSTR* argv;
	LPSTR  _argv;
	size_t len;
	size_t argc;
	CHAR   a;
	size_t i, j;

	bool   in_QM;
	bool   in_TEXT;
	bool   in_SPACE;

	len = strlen(CmdLine);
	i = ((len+2)/2)*sizeof(void*) + sizeof(void*);
	argv = (LPSTR*)(new uint8_t[i + (len+2)*sizeof(CHAR)]);
	_argv = (LPSTR)(((CHAR*)argv)+i);

	argc = 0;
	argv[argc] = _argv;
	in_QM = false;
	in_TEXT = false;
	in_SPACE = true;
	i = 0;
	j = 0;

	while (a = CmdLine[i]) {
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



// C L A S S ///////////////////////////////////////////////////////

#ifdef _MSC_VER

#include <comdef.h>
#include <Wbemidl.h>
#include <locale.h>

# pragma comment(lib, "wbemuuid.lib")

QueryWMI::QueryWMI()
	:
	vpLoc(NULL), vpSvc(NULL)
{
	HRESULT hres;
	typedef IWbemLocator* PIWbemLocator;
	PIWbemLocator& pLoc(reinterpret_cast<PIWbemLocator&>(vpLoc));
	typedef IWbemServices* PIWbemServices;
	PIWbemServices& pSvc(reinterpret_cast<PIWbemServices&>(vpSvc));

	// Step 1: --------------------------------------------------
	// Initialize COM. ------------------------------------------

	hres =  CoInitializeEx(0, COINIT_MULTITHREADED); 
	if (FAILED(hres)) {
		std::cout << "Failed to initialize COM library. Error code = 0x" << std::hex << hres << std::endl;
		return;                      // Program has failed.
	}

	// Step 2: --------------------------------------------------
	// Set general COM security levels --------------------------

	hres =  CoInitializeSecurity(
		NULL, 
		-1,                          // COM authentication
		NULL,                        // Authentication services
		NULL,                        // Reserved
		RPC_C_AUTHN_LEVEL_DEFAULT,   // Default authentication 
		RPC_C_IMP_LEVEL_IMPERSONATE, // Default Impersonation  
		NULL,                        // Authentication info
		EOAC_NONE,                   // Additional capabilities 
		NULL                         // Reserved
	);

	if (FAILED(hres)) {
		std::cout << "Failed to initialize security. Error code = 0x" << std::hex << hres << std::endl;
		Release();
		return;                      // Program has failed.
	}

	// Step 3: ---------------------------------------------------
	// Obtain the initial locator to WMI -------------------------

	hres = CoCreateInstance(
		CLSID_WbemLocator,             
		0, 
		CLSCTX_INPROC_SERVER, 
		IID_IWbemLocator, (LPVOID *) &pLoc);

	if (FAILED(hres)) {
		std::cout << "Failed to create IWbemLocator object." << " Err code = 0x" << std::hex << hres << std::endl;
		Release();
		return;                   // Program has failed.
	}

	// Step 4: -----------------------------------------------------
	// Connect to WMI through the IWbemLocator::ConnectServer method

	// Connect to the root\cimv2 namespace with
	// the current user and obtain pointer pSvc
	// to make IWbemServices calls.
	hres = pLoc->ConnectServer(
		_bstr_t(L"ROOT\\CIMV2"), // Object path of WMI namespace
		NULL,                    // User name. NULL = current user
		NULL,                    // User password. NULL = current
		0,                       // Locale. NULL indicates current
		NULL,                    // Security flags.
		0,                       // Authority (for example, Kerberos)
		0,                       // Context object 
		&pSvc                    // pointer to IWbemServices proxy
	);

	if (FAILED(hres)) {
		std::cout << "Could not connect. Error code = 0x" << std::hex << hres << std::endl;
		Release();
		return;                  // Program has failed.
	}

	//std::cout << "Connected to ROOT\\CIMV2 WMI namespace" << std::endl;


	// Step 5: --------------------------------------------------
	// Set security levels on the proxy -------------------------

	hres = CoSetProxyBlanket(
		pSvc,                        // Indicates the proxy to set
		RPC_C_AUTHN_WINNT,           // RPC_C_AUTHN_xxx
		RPC_C_AUTHZ_NONE,            // RPC_C_AUTHZ_xxx
		NULL,                        // Server principal name 
		RPC_C_AUTHN_LEVEL_CALL,      // RPC_C_AUTHN_LEVEL_xxx 
		RPC_C_IMP_LEVEL_IMPERSONATE, // RPC_C_IMP_LEVEL_xxx
		NULL,                        // client identity
		EOAC_NONE                    // proxy capabilities 
	);

	if (FAILED(hres)) {
		std::cout << "Could not set proxy blanket. Error code = 0x" << std::hex << hres << std::endl;
		Release();
		return;                      // Program has failed.
	}
}

void QueryWMI::Release()
{
	if (vpSvc) {
		((IWbemServices*)vpSvc)->Release();
		vpSvc = NULL;
	}
	if (vpLoc) {
		((IWbemLocator*)vpLoc)->Release();
		vpLoc = NULL;
	}
	CoUninitialize();
}

String QueryWMI::Query(String wmiClass, String wmiProperty, String wmiCondition)
{
	HRESULT hres;
	String query;

	typedef IWbemServices* PIWbemServices;
	PIWbemServices& pSvc(reinterpret_cast<PIWbemServices&>(vpSvc));

	// Step 6: --------------------------------------------------
	// Use the IWbemServices pointer to make requests of WMI ----

	// For example, get the name of the operating system
	IEnumWbemClassObject* pEnumerator(NULL);
	hres = pSvc->ExecQuery(
		bstr_t("WQL"), 
		bstr_t("SELECT * FROM ") + bstr_t(wmiClass.c_str()),
		WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY, 
		NULL,
		&pEnumerator);

	if (FAILED(hres)) {
		std::cout << "Query for operating system name failed." << " Error code = 0x" << std::hex << hres << std::endl;
		Release();
		return query;               // Program has failed.
	}

	// Step 7: -------------------------------------------------
	// Get the data from the query in step 6 -------------------

	VARIANT vtProp;
	while (pEnumerator) {
		ULONG uReturn = 0;
		IWbemClassObject* pclsObj;
		hres = pEnumerator->Next(WBEM_INFINITE, 1, &pclsObj, &uReturn);
		if (FAILED(hres) || 0 == uReturn)
			break;

		if (wmiCondition.empty()) {
			// Get the value of the Name property
			hres = pclsObj->Get(bstr_t(wmiProperty.c_str()), 0, &vtProp, NULL, NULL);
			if (SUCCEEDED(hres) && vtProp.vt == CIM_STRING) {
				query += Util::toString(vtProp.bstrVal);
				VariantClear(&vtProp);
			}
		} else {
			// Check condition value to be True
			hres = pclsObj->Get(bstr_t(wmiCondition.c_str()), 0, &vtProp, NULL, NULL);
			if (SUCCEEDED(hres) && vtProp.vt == CIM_BOOLEAN && vtProp.boolVal) {
				// Get the value of the Name property
				hres = pclsObj->Get(bstr_t(wmiProperty.c_str()), 0, &vtProp, NULL, NULL);
				if (SUCCEEDED(hres) && vtProp.vt == CIM_STRING) {
					query += Util::toString(vtProp.bstrVal);
					VariantClear(&vtProp);
				}
			}
		}

		pclsObj->Release();
	}

	// Cleanup
	// ========

	pEnumerator->Release();

	return query;
}

String QueryWMI::cpuId()
{
	//Uses first CPU identifier available in order of preference
	//Don't get all identifiers, as very time consuming
	String retVal(Query("Win32_Processor", "UniqueId"));
	if (retVal.empty()) { //If no UniqueID, use ProcessorID
		retVal = Query("Win32_Processor", "ProcessorId");
		if (retVal.empty()) { //If no ProcessorId, use Name
			retVal = Query("Win32_Processor", "Name");
			if (retVal.empty()) //If no Name, use Manufacturer
				retVal = Query("Win32_Processor", "Manufacturer");
			//Add clock speed for extra security
			retVal += Query("Win32_Processor", "MaxClockSpeed");
		}
	}
	return retVal;
}
//BIOS Identifier
String QueryWMI::biosId()
{
	return Query("Win32_BIOS", "Manufacturer")
		 + Query("Win32_BIOS", "SMBIOSBIOSVersion")
		 + Query("Win32_BIOS", "IdentificationCode")
		 + Query("Win32_BIOS", "SerialNumber")
		 + Query("Win32_BIOS", "ReleaseDate")
		 + Query("Win32_BIOS", "Version");
}
//Main physical hard drive ID
String QueryWMI::diskId()
{
	return Query("Win32_DiskDrive", "Model")
		 + Query("Win32_DiskDrive", "Manufacturer")
		 + Query("Win32_DiskDrive", "Signature")
		 + Query("Win32_DiskDrive", "TotalHeads");
}
//Motherboard ID
String QueryWMI::baseId()
{
	return Query("Win32_BaseBoard", "Model")
		 + Query("Win32_BaseBoard", "Manufacturer")
		 + Query("Win32_BaseBoard", "Name")
		 + Query("Win32_BaseBoard", "SerialNumber");
}
//Primary video controller ID
String QueryWMI::videoId()
{
	return Query("Win32_VideoController", "DriverVersion")
		 + Query("Win32_VideoController", "Name");
}
//First enabled network card ID
String QueryWMI::macId()
{
	return Query("Win32_NetworkAdapterConfiguration", "MACAddress", "IPEnabled");
}
/*----------------------------------------------------------------*/

#endif // _MSC_VER
