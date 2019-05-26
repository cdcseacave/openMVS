/*
 * RefineMesh.cpp
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

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("RefineMesh")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
unsigned nResolutionLevel;
unsigned nMinResolution;
unsigned nMaxViews;
float fDecimateMesh;
unsigned nCloseHoles;
unsigned nEnsureEdgeSize;
unsigned nScales;
float fScaleStep;
unsigned nReduceMemory;
unsigned nAlternatePair;
float fRegularityWeight;
float fRatioRigidityElasticity;
unsigned nMaxFaceArea;
float fPlanarVertexRatio;
float fGradientStep;
#ifdef _USE_CUDA
bool bUseCUDA;
#endif
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
String strExportType;
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
		("export-type", boost::program_options::value<std::string>(&OPT::strExportType)->default_value(_T("ply")), "file type used to export the 3D scene (ply or obj)")
		("archive-type", boost::program_options::value<unsigned>(&OPT::nArchiveType)->default_value(2), "project archive type: 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value<int>(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("max-threads", boost::program_options::value<unsigned>(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value<int>(&g_nVerbosityLevel)->default_value(
			#if TD_VERBOSE == TD_VERBOSE_DEBUG
			3
			#else
			2
			#endif
			), "verbosity level")
		#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Refine options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("resolution-level", boost::program_options::value<unsigned>(&OPT::nResolutionLevel)->default_value(0), "how many times to scale down the images before mesh refinement")
		("min-resolution", boost::program_options::value<unsigned>(&OPT::nMinResolution)->default_value(640), "do not scale images lower than this resolution")
		("max-views", boost::program_options::value<unsigned>(&OPT::nMaxViews)->default_value(8), "maximum number of neighbor images used to refine the mesh")
		("decimate", boost::program_options::value<float>(&OPT::fDecimateMesh)->default_value(0.f), "decimation factor in range [0..1] to be applied to the input surface before refinement (0 - auto, 1 - disabled)")
		("close-holes", boost::program_options::value<unsigned>(&OPT::nCloseHoles)->default_value(30), "try to close small holes in the input surface (0 - disabled)")
		("ensure-edge-size", boost::program_options::value<unsigned>(&OPT::nEnsureEdgeSize)->default_value(1), "ensure edge size and improve vertex valence of the input surface (0 - disabled, 1 - auto, 2 - force)")
		("max-face-area", boost::program_options::value<unsigned>(&OPT::nMaxFaceArea)->default_value(64), "maximum face area projected in any pair of images that is not subdivided (0 - disabled)")
		("scales", boost::program_options::value<unsigned>(&OPT::nScales)->default_value(3), "how many iterations to run mesh optimization on multi-scale images")
		("scale-step", boost::program_options::value<float>(&OPT::fScaleStep)->default_value(0.5f), "image scale factor used at each mesh optimization step")
		("reduce-memory", boost::program_options::value<unsigned>(&OPT::nReduceMemory)->default_value(1), "recompute some data in order to reduce memory requirements")
		("alternate-pair", boost::program_options::value<unsigned>(&OPT::nAlternatePair)->default_value(0), "refine mesh using an image pair alternatively as reference (0 - both, 1 - alternate, 2 - only left, 3 - only right)")
		("regularity-weight", boost::program_options::value<float>(&OPT::fRegularityWeight)->default_value(0.2f), "scalar regularity weight to balance between photo-consistency and regularization terms during mesh optimization")
		("rigidity-elasticity-ratio", boost::program_options::value<float>(&OPT::fRatioRigidityElasticity)->default_value(0.9f), "scalar ratio used to compute the regularity gradient as a combination of rigidity and elasticity")
		("gradient-step", boost::program_options::value<float>(&OPT::fGradientStep)->default_value(45.05f), "gradient step to be used instead (0 - auto)")
		("planar-vertex-ratio", boost::program_options::value<float>(&OPT::fPlanarVertexRatio)->default_value(0.f), "threshold used to remove vertices on planar patches (0 - disabled)")
		#ifdef _USE_CUDA
		("use-cuda", boost::program_options::value<bool>(&OPT::bUseCUDA)->default_value(true), "refine mesh using CUDA")
		#endif
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("mesh-file", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name to refine (overwrite the existing mesh)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config).add(hidden);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config).add(hidden);

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
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line:%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (OPT::vm.count("help") || OPT::strInputFileName.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (OPT::strInputFileName.IsEmpty())
		return false;
	OPT::strExportType = OPT::strExportType.ToLower() == _T("obj") ? _T(".obj") : _T(".ply");

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_refine.mvs");

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

	Util::Init();
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

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	Scene scene(OPT::nMaxThreads);
	// load and refine the coarse mesh
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
		return EXIT_FAILURE;
	if (!OPT::strMeshFileName.IsEmpty()) {
		// load given coarse mesh
		scene.mesh.Load(MAKE_PATH_SAFE(OPT::strMeshFileName));
	}
	if (scene.mesh.IsEmpty()) {
		VERBOSE("error: empty initial mesh");
		return EXIT_FAILURE;
	}
	TD_TIMER_START();
	#ifdef _USE_CUDA
	if (!OPT::bUseCUDA ||
		!scene.RefineMeshCUDA(OPT::nResolutionLevel, OPT::nMinResolution, OPT::nMaxViews,
							  OPT::fDecimateMesh, OPT::nCloseHoles, OPT::nEnsureEdgeSize,
							  OPT::nMaxFaceArea,
							  OPT::nScales, OPT::fScaleStep,
							  OPT::nAlternatePair>10 ? OPT::nAlternatePair%10 : 0,
							  OPT::fRegularityWeight,
							  OPT::fRatioRigidityElasticity,
							  OPT::fGradientStep))
	#endif
	if (!scene.RefineMesh(OPT::nResolutionLevel, OPT::nMinResolution, OPT::nMaxViews,
						  OPT::fDecimateMesh, OPT::nCloseHoles, OPT::nEnsureEdgeSize,
						  OPT::nMaxFaceArea,
						  OPT::nScales, OPT::fScaleStep,
						  OPT::nReduceMemory, OPT::nAlternatePair,
						  OPT::fRegularityWeight,
						  OPT::fRatioRigidityElasticity,
						  OPT::fPlanarVertexRatio,
						  OPT::fGradientStep))
		return EXIT_FAILURE;
	VERBOSE("Mesh refinement completed: %u vertices, %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());

	// save the final mesh
	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
	scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
	scene.mesh.Save(baseFileName+OPT::strExportType);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+OPT::strExportType);
	#endif

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
