/*
 * ReconstructMesh.cpp
 *
 * Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
 * Please read <http://foxel.ch/license> for more information.
 *
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This file is part of the FOXEL project <http://foxel.ch>.
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
 *
 *      You are required to attribute the work as explained in the "Usage and
 *      Attribution" section of <http://foxel.ch/license>.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("ReconstructMesh")

#define WORKING_FOLDER		OPT::strWorkingFolder // empty by default (current folder)
#define WORKING_FOLDER_FULL	OPT::strWorkingFolderFull // full path to current folder


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName; // file name of the input data
String strOutputFileName; // file name of the mesh data
float fDistInsert; // minimum distance between the projection of two points to consider them different
bool bUseFreeSpaceSupport; // exploit free-space support in order to try reconstruct weakly-represented surfaces
String strWorkingFolder; // empty by default (current folder)
String strWorkingFolderFull; // full path to current folder
} // namespace OPT


// ParseCmdLine to parse the command line parameters
#define PARAM_EQUAL(name) (0 == _tcsncicmp(param, _T("-" #name "="), sizeof(_T("-" #name "="))-sizeof(TCHAR)))
bool ParseCmdLine(size_t argc, LPCTSTR* argv)
{
	OPT::fDistInsert = 2.f;
	OPT::bUseFreeSpaceSupport = true;
	bool bPrintHelp(false);
	String strCmdLine;
	for (size_t i=1; i<argc; ++i) {
		LPCSTR param = argv[i];
		strCmdLine += _T(" ") + String(param);
		if (PARAM_EQUAL(input) || PARAM_EQUAL(i)) {
			param = strchr(param, '=')+1;
			OPT::strInputFileName = param;
		} else if (PARAM_EQUAL(output) || PARAM_EQUAL(o)) {
			param = strchr(param, '=')+1;
			OPT::strOutputFileName = param;
		} else if (PARAM_EQUAL(dist) || PARAM_EQUAL(d)) {
			param = strchr(param, '=')+1;
			OPT::fDistInsert = (float)atof(param);
		} else if (PARAM_EQUAL(fss)) {
			param = strchr(param, '=')+1;
			OPT::bUseFreeSpaceSupport = (atoi(param) != 0);
		} else if (PARAM_EQUAL(workdir) || PARAM_EQUAL(w)) {
			param = strchr(param, '=')+1;
			OPT::strWorkingFolder = param;
		} else if (0 == _tcsicmp(param, _T("-help")) || 0 == _tcsicmp(param, _T("-h")) || 0 == _tcsicmp(param, _T("-?"))) {
			bPrintHelp = true;
		}
	}
	LOG(_T("Command line:%s"), strCmdLine.c_str());
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (bPrintHelp || OPT::strInputFileName.IsEmpty())
		LOG("Available list of command-line parameters:\n"
			"\t-input= or -i=<input_filename> for setting the input filename containing camera poses and image list\n"
			"\t-output= or -o=<output_filename> for setting the output filename for storing the mesh and texture\n"
			"\t-dist= or -d=<distance_insert> for setting the minimum distance between the projection of two 3D points to consider them different (default 2 pixels)\n"
			"\t-fss=<free_space_support> for exploiting free-space support in order to try reconstruct weakly-represented surfaces (default on)\n"
			"\t-workdir= or -w=<folder_path> for setting the working directory (default current directory)\n"
			"\t-help or -h or -? for displaying this message"
		);
	if (OPT::strInputFileName.IsEmpty())
		return false;
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + _T(".ply");
	Util::ensureValidDirectoryPath(OPT::strWorkingFolder);
	OPT::strWorkingFolderFull = Util::getFullPath(OPT::strWorkingFolder);
	return true;
}

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	OPEN_LOG();
	OPEN_LOGCONSOLE();
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	Util::LogBuild();
	#ifdef _USE_BREAKPAD
	MiniDumper::Create(APPNAME, WORKING_FOLDER);
	#endif
	if (!ParseCmdLine(argc, argv))
		return -1;

	TD_TIMER_START();

	Scene scene;
	scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::strWorkingFolderFull);
	scene.ReconstructMesh(OPT::fDistInsert, OPT::bUseFreeSpaceSupport);
	
	VERBOSE("Mesh reconstruction completed: %u vertices & %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());

	scene.mesh.Save(MAKE_PATH_SAFE(OPT::strOutputFileName));

	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
