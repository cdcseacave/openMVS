/*
* Common.cpp
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

// Source file that includes just the standard includes
// Common.pch will be the pre-compiled header
// Common.obj will contain the pre-compiled type information

#include "Common.h"
#include "Mesh.h"

using namespace MVS;

void MVS::Initialize(LPCTSTR appname, unsigned nMaxThreads, int nProcessPriority) {
	// initialize thread options
	Process::setCurrentProcessPriority((Process::Priority)nProcessPriority);
	#ifdef _USE_OPENMP
	if (nMaxThreads != 0)
		omp_set_num_threads(nMaxThreads);
	#endif

	#ifdef _USE_BREAKPAD
	// initialize crash memory dumper
	MiniDumper::Create(appname, WORKING_FOLDER);
	#endif

	// initialize random number generator
	Util::Init();
}

void MVS::Finalize() {
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif

	#ifdef _USE_CUDA
	// release static CUDA kernels before CUDA context is destroyed
	Mesh::kernelComputeFaceNormal.Release();
	#endif
}
/*----------------------------------------------------------------*/
