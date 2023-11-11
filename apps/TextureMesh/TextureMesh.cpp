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

namespace {

namespace OPT {
String strInputFileName;
String strMeshFileName;
String strOutputFileName;
String strViewsFileName;
float fDecimateMesh;
unsigned nCloseHoles;
unsigned nResolutionLevel;
unsigned nMinResolution;
unsigned minCommonCameras;
float fOutlierThreshold;
float fRatioDataSmoothness;
bool bGlobalSeamLeveling;
bool bLocalSeamLeveling;
unsigned nTextureSizeMultiple;
unsigned nRectPackingHeuristic;
uint32_t nColEmpty;
float fSharpnessWeight;
int nIgnoreMaskLabel;
unsigned nOrthoMapResolution;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
int nMaxTextureSize;
String strExportType;
String strConfigFileName;
boost::program_options::variables_map vm;
} // namespace OPT

class Application {
public:
	Application() {}
	~Application() { Finalize(); }

	bool Initialize(size_t argc, LPCTSTR* argv);
	void Finalize();
}; // Application

// initialize and parse the command line parameters
bool Application::Initialize(size_t argc, LPCTSTR* argv)
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
		("export-type", boost::program_options::value<std::string>(&OPT::strExportType)->default_value(_T("ply")), "file type used to export the 3D scene (ply, obj, glb or gltf)")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_MVS), "project archive type: -1-interface, 0-text, 1-binary, 2-compressed binary")
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
		#ifdef _USE_CUDA
		("cuda-device", boost::program_options::value(&CUDA::desiredDeviceID)->default_value(-1), "CUDA device number to be used to texture the mesh (-2 - CPU processing, -1 - best GPU, >=0 - device index)")
		#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Texture options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("mesh-file,m", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name to texture (overwrite existing mesh)")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("decimate", boost::program_options::value(&OPT::fDecimateMesh)->default_value(1.f), "decimation factor in range [0..1] to be applied to the input surface before refinement (0 - auto, 1 - disabled)")
		("close-holes", boost::program_options::value(&OPT::nCloseHoles)->default_value(30), "try to close small holes in the input surface (0 - disabled)")
		("resolution-level", boost::program_options::value(&OPT::nResolutionLevel)->default_value(0), "how many times to scale down the images before mesh refinement")
		("min-resolution", boost::program_options::value(&OPT::nMinResolution)->default_value(640), "do not scale images lower than this resolution")
		("outlier-threshold", boost::program_options::value(&OPT::fOutlierThreshold)->default_value(6e-2f), "threshold used to find and remove outlier face textures (0 - disabled)")
		("cost-smoothness-ratio", boost::program_options::value(&OPT::fRatioDataSmoothness)->default_value(0.1f), "ratio used to adjust the preference for more compact patches (1 - best quality/worst compactness, ~0 - worst quality/best compactness)")
		("virtual-face-images", boost::program_options::value(&OPT::minCommonCameras)->default_value(0), "generate texture patches using virtual faces composed of coplanar triangles sharing at least this number of views (0 - disabled, 3 - good value)")
		("global-seam-leveling", boost::program_options::value(&OPT::bGlobalSeamLeveling)->default_value(true), "generate uniform texture patches using global seam leveling")
		("local-seam-leveling", boost::program_options::value(&OPT::bLocalSeamLeveling)->default_value(true), "generate uniform texture patch borders using local seam leveling")
		("texture-size-multiple", boost::program_options::value(&OPT::nTextureSizeMultiple)->default_value(0), "texture size should be a multiple of this value (0 - power of two)")
		("patch-packing-heuristic", boost::program_options::value(&OPT::nRectPackingHeuristic)->default_value(3), "specify the heuristic used when deciding where to place a new patch (0 - best fit, 3 - good speed, 100 - best speed)")
		("empty-color", boost::program_options::value(&OPT::nColEmpty)->default_value(0x00FF7F27), "color used for faces not covered by any image")
		("sharpness-weight", boost::program_options::value(&OPT::fSharpnessWeight)->default_value(0.5f), "amount of sharpness to be applied on the texture (0 - disabled)")
		("orthographic-image-resolution", boost::program_options::value(&OPT::nOrthoMapResolution)->default_value(0), "orthographic image resolution to be generated from the textured mesh - the mesh is expected to be already geo-referenced or at least properly oriented (0 - disabled)")
		("ignore-mask-label", boost::program_options::value(&OPT::nIgnoreMaskLabel)->default_value(-1), "label value to ignore in the image mask, stored in the MVS scene or next to each image with '.mask.png' extension (-1 - auto estimate mask for lens distortion, -2 - disabled)")
		("max-texture-size", boost::program_options::value(&OPT::nMaxTextureSize)->default_value(8192), "maximum texture size, split it in multiple textures of this size if needed (0 - unbounded)")
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("views-file", boost::program_options::value<std::string>(&OPT::strViewsFileName), "file name containing the list of views to be used for texturing (optional)")
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
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	if (OPT::vm.count("help") || OPT::strInputFileName.empty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (OPT::strInputFileName.empty())
		return false;
	OPT::strExportType = OPT::strExportType.ToLower();
	if (OPT::strExportType == _T("obj"))
		OPT::strExportType =  _T(".obj");
	else
	if (OPT::strExportType == _T("glb"))
		OPT::strExportType =  _T(".glb");
	else
	if (OPT::strExportType == _T("gltf"))
		OPT::strExportType =  _T(".gltf");
	else
		OPT::strExportType =  _T(".ply");

	// initialize optional options
	Util::ensureValidPath(OPT::strMeshFileName);
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureValidPath(OPT::strViewsFileName);
	if (OPT::strMeshFileName.empty() && (ARCHIVE_TYPE)OPT::nArchiveType == ARCHIVE_MVS)
		OPT::strMeshFileName = Util::getFileFullName(OPT::strInputFileName) + _T(".ply");
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_texture.mvs");

	MVS::Initialize(APPNAME, OPT::nMaxThreads, OPT::nProcessPriority);
	return true;
}

// finalize application instance
void Application::Finalize()
{
	MVS::Finalize();

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // unnamed namespace

IIndexArr ParseViewsFile(const String& filename, const Scene& scene) {
	IIndexArr views;
	std::ifstream file(filename);
	if (!file.good()) {
		VERBOSE("error: unable to open views file '%s'", filename.c_str());
		return views;
	}
	while (true) {
		String imageName;
		std::getline(file, imageName);
		if (file.fail() || imageName.empty())
			break;
		LPTSTR endIdx;
		const IDX idx(strtoul(imageName, &endIdx, 10));
		const size_t szIndex(*endIdx == '\0' ? size_t(0) : imageName.size());
		FOREACH(idxImage, scene.images) {
			const Image& image = scene.images[idxImage];
			if (szIndex == 0) {
				// try to match by index
				if (image.ID != idx)
					continue;
			} else {
				// try to match by file name
				const String name(Util::getFileNameExt(image.name));
				if (name.size() < szIndex || _tcsnicmp(name, imageName, szIndex) != 0)
					continue;
			}
			views.emplace_back(idxImage);
			break;
		}
	}
	return views;
}

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	Scene scene(OPT::nMaxThreads);
	// load and texture the mesh
	const Scene::SCENE_TYPE sceneType(scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)));
	if (sceneType == Scene::SCENE_NA)
		return EXIT_FAILURE;
	if (!OPT::strMeshFileName.empty() && !scene.mesh.Load(MAKE_PATH_SAFE(OPT::strMeshFileName))) {
		VERBOSE("error: cannot load mesh file");
		return EXIT_FAILURE;
	}
	if (scene.mesh.IsEmpty()) {
		VERBOSE("error: empty initial mesh");
		return EXIT_FAILURE;
	}
	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
	if (OPT::nOrthoMapResolution && !scene.mesh.HasTexture()) {
		// the input mesh is already textured and an orthographic projection was requested
		goto ProjectOrtho;
	}

	{
	// decimate to the desired resolution
	if (OPT::fDecimateMesh < 1.f) {
		ASSERT(OPT::fDecimateMesh > 0.f);
		scene.mesh.Clean(OPT::fDecimateMesh, 0.f, false, OPT::nCloseHoles, 0u, 0.f, false);
		scene.mesh.Clean(1.f, 0.f, false, 0u, 0u, 0.f, true); // extra cleaning to remove non-manifold problems created by closing holes
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 3)
			scene.mesh.Save(baseFileName +_T("_decim")+OPT::strExportType);
		#endif
	}
	// fetch list of views to be used for texturing
	IIndexArr views;
	if (!OPT::strViewsFileName.empty())
		views = ParseViewsFile(MAKE_PATH_SAFE(OPT::strViewsFileName), scene);

	// compute mesh texture
	TD_TIMER_START();
	if (!scene.TextureMesh(OPT::nResolutionLevel, OPT::nMinResolution, OPT::minCommonCameras, OPT::fOutlierThreshold, OPT::fRatioDataSmoothness,
						   OPT::bGlobalSeamLeveling, OPT::bLocalSeamLeveling, OPT::nTextureSizeMultiple, OPT::nRectPackingHeuristic, Pixel8U(OPT::nColEmpty),
						   OPT::fSharpnessWeight, OPT::nIgnoreMaskLabel, OPT::nMaxTextureSize, views))
		return EXIT_FAILURE;
	VERBOSE("Mesh texturing completed: %u vertices, %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());

	// save the final mesh
	scene.mesh.Save(baseFileName+OPT::strExportType);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+OPT::strExportType);
	#endif
	if ((ARCHIVE_TYPE)OPT::nArchiveType != ARCHIVE_MVS || sceneType != Scene::SCENE_INTERFACE)
		scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
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

	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
