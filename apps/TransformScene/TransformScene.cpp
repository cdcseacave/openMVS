/*
 * TransformScene.cpp
 *
 * Copyright (c) 2014-2021 SEACAVE
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

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("TransformScene")
#define MVS_EXT _T(".mvs")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
	String strInputFileName;
	String strOutputFileName;
	String strAlignFileName;
	bool bComputeVolume;
	float fPlaneThreshold;
	float fSampleMesh;
	unsigned nUpAxis;
	unsigned nArchiveType;
	int nProcessPriority;
	unsigned nMaxThreads;
	String strConfigFileName;
	boost::program_options::variables_map vm;
} // namespace OPT

// initialize and parse the command line parameters
bool Initialize(size_t argc, LPCTSTR* argv)
{
	// initialize log and console
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	// group of options allowed only on command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_MVS), "project archive type: 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("max-threads", boost::program_options::value(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value(&g_nVerbosityLevel)->default_value(
			#if TD_VERBOSE == TD_VERBOSE_DEBUG
			3
			#else
			2
			#endif
		), "verbosity level")
		#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input scene filename")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the scene")
		("align-file,o", boost::program_options::value<std::string>(&OPT::strAlignFileName), "input scene filename to which the scene will be cameras aligned")
		("compute-volume", boost::program_options::value(&OPT::bComputeVolume)->default_value(false), "compute the volume of the given watertight mesh, or else try to estimate the ground plane and assume the mesh is bounded by it")
		("plane-threshold", boost::program_options::value(&OPT::fPlaneThreshold)->default_value(0.f), "threshold used to estimate the ground plane (<0 - disabled, 0 - auto, >0 - desired threshold)")
		("sample-mesh", boost::program_options::value(&OPT::fSampleMesh)->default_value(-300000.f), "uniformly samples points on a mesh (0 - disabled, <0 - number of points, >0 - sample density per square unit)")
		("up-axis", boost::program_options::value(&OPT::nUpAxis)->default_value(2), "scene axis considered to point upwards (0 - x, 1 - y, 2 - z)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config);

	boost::program_options::positional_options_description p;
	p.add("input-file", -1);

	try {
		// parse command line options
		boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
		boost::program_options::notify(OPT::vm);
		INIT_WORKING_FOLDER;
		// parse configuration file
		std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName));
		if (ifs) {
			boost::program_options::store(parse_config_file(ifs, config_file_options), OPT::vm);
			boost::program_options::notify(OPT::vm);
		}
	}
	catch (const std::exception& e) {
		LOG(e.what());
		return false;
	}

	// initialize the log file
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-") + Util::getUniqueName(0) + _T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureValidPath(OPT::strAlignFileName);
	const String strInputFileNameExt(Util::getFileExt(OPT::strInputFileName).ToLower());
	const bool bInvalidCommand(OPT::strInputFileName.empty() || (OPT::strAlignFileName.empty() && !OPT::bComputeVolume));
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileName(OPT::strInputFileName) + "_transformed" MVS_EXT;

	// initialize global options
	Process::setCurrentProcessPriority((Process::Priority)OPT::nProcessPriority);
	#ifdef _USE_OPENMP
	if (OPT::nMaxThreads != 0)
		omp_set_num_threads(OPT::nMaxThreads);
	#endif

	#ifdef _USE_BREAKPAD
	// start memory dumper
	MiniDumper::Create(APPNAME, WORKING_FOLDER);
	#endif
	return true;
}

// finalize application instance
void Finalize()
{
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // unnamed namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	Scene scene(OPT::nMaxThreads);

	// load given scene
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::bComputeVolume))
		return EXIT_FAILURE;

	if (!OPT::strAlignFileName.empty()) {
		// transform this scene such that it best aligns with the given scene based on the camera positions
		Scene sceneRef(OPT::nMaxThreads);
		if (!sceneRef.Load(MAKE_PATH_SAFE(OPT::strAlignFileName)))
			return EXIT_FAILURE;
		if (!scene.AlignTo(sceneRef))
			return EXIT_FAILURE;
		VERBOSE("Scene aligned to the given reference scene (%s)", TD_TIMER_GET_FMT().c_str());
	}

	if (OPT::bComputeVolume && !scene.mesh.IsEmpty()) {
		// compute the mesh volume
		const REAL volume(scene.ComputeLeveledVolume(OPT::fPlaneThreshold, OPT::fSampleMesh, OPT::nUpAxis));
		VERBOSE("Mesh volume: %g (%s)", volume, TD_TIMER_GET_FMT().c_str());
		scene.mesh.Save(Util::getFileFullName(MAKE_PATH_SAFE(OPT::strOutputFileName)) + _T(".ply"));
		if (scene.images.empty())
			return EXIT_SUCCESS;
		OPT::nArchiveType = ARCHIVE_DEFAULT;
	}

	// write transformed scene
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
