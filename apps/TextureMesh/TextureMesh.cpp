/*
 * TextureMesh.cpp
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

#define APPNAME _T("TextureMesh")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
unsigned nResolutionLevel;
unsigned nMinResolution;
float fOutlierThreshold;
float fRatioDataSmoothness;
bool bGlobalSeamLeveling;
bool bLocalSeamLeveling;
unsigned nTextureSizeMultiple;
unsigned nRectPackingHeuristic;
uint32_t nColEmpty;
unsigned nOrthoMapResolution;
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
	boost::program_options::options_description config("Texture options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("resolution-level", boost::program_options::value<unsigned>(&OPT::nResolutionLevel)->default_value(0), "how many times to scale down the images before mesh refinement")
		("min-resolution", boost::program_options::value<unsigned>(&OPT::nMinResolution)->default_value(640), "do not scale images lower than this resolution")
		("outlier-threshold", boost::program_options::value<float>(&OPT::fOutlierThreshold)->default_value(6e-2f), "threshold used to find and remove outlier face textures (0 - disabled)")
		("cost-smoothness-ratio", boost::program_options::value<float>(&OPT::fRatioDataSmoothness)->default_value(0.1f), "ratio used to adjust the preference for more compact patches (1 - best quality/worst compactness, ~0 - worst quality/best compactness)")
		("global-seam-leveling", boost::program_options::value<bool>(&OPT::bGlobalSeamLeveling)->default_value(true), "generate uniform texture patches using global seam leveling")
		("local-seam-leveling", boost::program_options::value<bool>(&OPT::bLocalSeamLeveling)->default_value(true), "generate uniform texture patch borders using local seam leveling")
		("texture-size-multiple", boost::program_options::value<unsigned>(&OPT::nTextureSizeMultiple)->default_value(0), "texture size should be a multiple of this value (0 - power of two)")
		("patch-packing-heuristic", boost::program_options::value<unsigned>(&OPT::nRectPackingHeuristic)->default_value(3), "specify the heuristic used when deciding where to place a new patch (0 - best fit, 3 - good speed, 100 - best speed)")
		("empty-color", boost::program_options::value<uint32_t>(&OPT::nColEmpty)->default_value(0x00FF7F27), "color used for faces not covered by any image")
		("orthographic-image-resolution", boost::program_options::value<unsigned>(&OPT::nOrthoMapResolution)->default_value(0), "orthographic image resolution to be generated from the textured mesh - the mesh is expected to be already geo-referenced or at least properly oriented (0 - disabled)")
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("mesh-file", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name to texture (overwrite the existing mesh)")
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
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_texture.mvs");

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

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	Scene scene(OPT::nMaxThreads);
	// load and texture the mesh
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
		return EXIT_FAILURE;
	if (!OPT::strMeshFileName.IsEmpty()) {
		// load given mesh
		scene.mesh.Load(MAKE_PATH_SAFE(OPT::strMeshFileName));
	}
	if (scene.mesh.IsEmpty()) {
		VERBOSE("error: empty initial mesh");
		return EXIT_FAILURE;
	}
	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
	if (OPT::nOrthoMapResolution && !scene.mesh.textureDiffuse.empty()) {
		// the input mesh is already textured and an orthographic projection was requested
		goto ProjectOrtho;
	}

	{
	// compute mesh texture
	TD_TIMER_START();
	if (!scene.TextureMesh(OPT::nResolutionLevel, OPT::nMinResolution, OPT::fOutlierThreshold, OPT::fRatioDataSmoothness, OPT::bGlobalSeamLeveling, OPT::bLocalSeamLeveling, OPT::nTextureSizeMultiple, OPT::nRectPackingHeuristic, Pixel8U(OPT::nColEmpty)))
		return EXIT_FAILURE;
	VERBOSE("Mesh texturing completed: %u vertices, %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());

	// save the final mesh
	scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
	scene.mesh.Save(baseFileName+OPT::strExportType);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+OPT::strExportType);
	#endif
	}

	if (OPT::nOrthoMapResolution) {
		// project mesh as an orthographic image
		ProjectOrtho:
		Image8U3 imageRGB;
		Image8U imageRGBA[4];
		Point3 center;
		scene.mesh.ProjectOrthoTopDown(OPT::nOrthoMapResolution, imageRGB, imageRGBA[3], center);
		Image8U4 image;
		cv::split(imageRGB, imageRGBA);
		cv::merge(imageRGBA, 4, image);
		image.Save(baseFileName+_T("_orthomap.png"));
		SML sml(_T("OrthoMap"));
		sml[_T("Center")].val = String::FormatString(_T("%g %g %g"), center.x, center.y, center.z);
		sml.Save(baseFileName+_T("_orthomap.txt"));
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
