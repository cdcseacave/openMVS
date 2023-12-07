/*
 * DensifyPointCloud.cpp
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

#define APPNAME _T("DensifyPointCloud")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strPointCloudFileName;
String strOutputFileName;
String strViewNeighborsFileName;
String strOutputViewNeighborsFileName;
String strMeshFileName;
String strExportROIFileName;
String strImportROIFileName;
String strDenseConfigFileName;
String strExportDepthMapsName;
String strMaskPath;
float fMaxSubsceneArea;
float fSampleMesh;
float fBorderROI;
bool bCrop2ROI;
int nEstimateROI;
int	nTowerMode;
int nFusionMode;
float fEstimateScale;
int thFilterPointCloud;
int nExportNumViews;
int nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
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
		("cuda-device", boost::program_options::value(&CUDA::desiredDeviceID)->default_value(-1), "CUDA device number to be used for depth-map estimation (-2 - CPU processing, -1 - best GPU, >=0 - device index)")
		#endif
		;

	// group of options allowed both on command line and in config file
	#ifdef _USE_CUDA
	const unsigned nNumViewsDefault(8);
	const unsigned numIters(4);
	#else
	const unsigned nNumViewsDefault(5);
	const unsigned numIters(3);
	#endif
	unsigned nResolutionLevel;
	unsigned nMaxResolution;
	unsigned nMinResolution;
	unsigned nNumViews;
	unsigned nMinViewsFuse;
	unsigned nSubResolutionLevels;
	unsigned nEstimationIters;
	unsigned nEstimationGeometricIters;
	unsigned nEstimateColors;
	unsigned nEstimateNormals;
	unsigned nOptimize;
	int nIgnoreMaskLabel;
	bool bRemoveDmaps;
	boost::program_options::options_description config("Densify options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("pointcloud-file,p", boost::program_options::value<std::string>(&OPT::strPointCloudFileName), "sparse point-cloud with views file name to densify (overwrite existing point-cloud)")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the dense point-cloud (optional)")
		("view-neighbors-file", boost::program_options::value<std::string>(&OPT::strViewNeighborsFileName), "input filename containing the list of views and their neighbors (optional)")
		("output-view-neighbors-file", boost::program_options::value<std::string>(&OPT::strOutputViewNeighborsFileName), "output filename containing the generated list of views and their neighbors")
		("resolution-level", boost::program_options::value(&nResolutionLevel)->default_value(1), "how many times to scale down the images before point cloud computation")
		("max-resolution", boost::program_options::value(&nMaxResolution)->default_value(2560), "do not scale images higher than this resolution")
		("min-resolution", boost::program_options::value(&nMinResolution)->default_value(640), "do not scale images lower than this resolution")
		("sub-resolution-levels", boost::program_options::value(&nSubResolutionLevels)->default_value(2), "number of patch-match sub-resolution iterations (0 - disabled)")
		("number-views", boost::program_options::value(&nNumViews)->default_value(nNumViewsDefault), "number of views used for depth-map estimation (0 - all neighbor views available)")
		("number-views-fuse", boost::program_options::value(&nMinViewsFuse)->default_value(3), "minimum number of images that agrees with an estimate during fusion in order to consider it inlier (<2 - only merge depth-maps)")
		("ignore-mask-label", boost::program_options::value(&nIgnoreMaskLabel)->default_value(-1), "label value to ignore in the image mask, stored in the MVS scene or next to each image with '.mask.png' extension (<0 - disabled)")
		("mask-path", boost::program_options::value<std::string>(&OPT::strMaskPath), "path to folder containing mask images with '.mask.png' extension")
		("iters", boost::program_options::value(&nEstimationIters)->default_value(numIters), "number of patch-match iterations")
		("geometric-iters", boost::program_options::value(&nEstimationGeometricIters)->default_value(2), "number of geometric consistent patch-match iterations (0 - disabled)")
		("estimate-colors", boost::program_options::value(&nEstimateColors)->default_value(2), "estimate the colors for the dense point-cloud (0 - disabled, 1 - final, 2 - estimate)")
		("estimate-normals", boost::program_options::value(&nEstimateNormals)->default_value(2), "estimate the normals for the dense point-cloud (0 - disabled, 1 - final, 2 - estimate)")
		("estimate-scale", boost::program_options::value(&OPT::fEstimateScale)->default_value(0.f), "estimate the point-scale for the dense point-cloud (scale multiplier, 0 - disabled)")
		("sub-scene-area", boost::program_options::value(&OPT::fMaxSubsceneArea)->default_value(0.f), "split the scene in sub-scenes such that each sub-scene surface does not exceed the given maximum sampling area (0 - disabled)")
		("sample-mesh", boost::program_options::value(&OPT::fSampleMesh)->default_value(0.f), "uniformly samples points on a mesh (0 - disabled, <0 - number of points, >0 - sample density per square unit)")
		("fusion-mode", boost::program_options::value(&OPT::nFusionMode)->default_value(0), "depth-maps fusion mode (-2 - fuse disparity-maps, -1 - export disparity-maps only, 0 - depth-maps & fusion, 1 - export depth-maps only)")
		("postprocess-dmaps", boost::program_options::value(&nOptimize)->default_value(7), "flags used to filter the depth-maps after estimation (0 - disabled, 1 - remove-speckles, 2 - fill-gaps, 4 - adjust-filter)")
		("filter-point-cloud", boost::program_options::value(&OPT::thFilterPointCloud)->default_value(0), "filter dense point-cloud based on visibility (0 - disabled)")
		("export-number-views", boost::program_options::value(&OPT::nExportNumViews)->default_value(0), "export points with >= number of views (0 - disabled, <0 - save MVS project too)")
        ("roi-border", boost::program_options::value(&OPT::fBorderROI)->default_value(0), "add a border to the region-of-interest when cropping the scene (0 - disabled, >0 - percentage, <0 - absolute)")
        ("estimate-roi", boost::program_options::value(&OPT::nEstimateROI)->default_value(2), "estimate and set region-of-interest (0 - disabled, 1 - enabled, 2 - adaptive)")
        ("crop-to-roi", boost::program_options::value(&OPT::bCrop2ROI)->default_value(true), "crop scene using the region-of-interest")
        ("remove-dmaps", boost::program_options::value(&bRemoveDmaps)->default_value(false), "remove depth-maps after fusion")
		("tower-mode", boost::program_options::value(&OPT::nTowerMode)->default_value(4), "add a cylinder of points in the center of ROI; scene assume to be Z-up oriented (0 - disabled, 1 - replace, 2 - append, 3 - select neighbors, 4 - select neighbors & append, <0 - force tower mode)")
        ;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("mesh-file", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name used for image pair overlap estimation")
		("export-roi-file", boost::program_options::value<std::string>(&OPT::strExportROIFileName), "ROI file name to be exported form the scene")
		("import-roi-file", boost::program_options::value<std::string>(&OPT::strImportROIFileName), "ROI file name to be imported into the scene")
		("dense-config-file", boost::program_options::value<std::string>(&OPT::strDenseConfigFileName), "optional configuration file for the densifier (overwritten by the command line options)")
		("export-depth-maps-name", boost::program_options::value<std::string>(&OPT::strExportDepthMapsName), "render given mesh and save the depth-map for every image to this file name base (empty - disabled)")
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

	// initialize optional options
	Util::ensureValidPath(OPT::strPointCloudFileName);
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureValidPath(OPT::strViewNeighborsFileName);
	Util::ensureValidPath(OPT::strOutputViewNeighborsFileName);
	Util::ensureValidPath(OPT::strMeshFileName);
	Util::ensureValidPath(OPT::strExportROIFileName);
	Util::ensureValidPath(OPT::strImportROIFileName);
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_dense.mvs");

	// init dense options
	if (!OPT::strDenseConfigFileName.empty())
		OPT::strDenseConfigFileName = MAKE_PATH_SAFE(OPT::strDenseConfigFileName);
	OPTDENSE::init();
	const bool bValidConfig(OPTDENSE::oConfig.Load(OPT::strDenseConfigFileName));
	OPTDENSE::update();
	OPTDENSE::nResolutionLevel = nResolutionLevel;
	OPTDENSE::nMaxResolution = nMaxResolution;
	OPTDENSE::nMinResolution = nMinResolution;
	OPTDENSE::nSubResolutionLevels = nSubResolutionLevels;
	OPTDENSE::nNumViews = nNumViews;
	OPTDENSE::nMinViewsFuse = nMinViewsFuse;
	OPTDENSE::nEstimationIters = nEstimationIters;
	OPTDENSE::nEstimationGeometricIters = nEstimationGeometricIters;
	OPTDENSE::nEstimateColors = nEstimateColors;
	OPTDENSE::nEstimateNormals = nEstimateNormals;
	OPTDENSE::nOptimize = nOptimize;
	OPTDENSE::nIgnoreMaskLabel = nIgnoreMaskLabel;
	OPTDENSE::bRemoveDmaps = bRemoveDmaps;
	if (!bValidConfig && !OPT::strDenseConfigFileName.empty())
		OPTDENSE::oConfig.Save(OPT::strDenseConfigFileName);

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
	if (OPT::fSampleMesh != 0) {
		// sample input mesh and export the obtained point-cloud
		if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), true) || scene.mesh.IsEmpty())
			return EXIT_FAILURE;
		TD_TIMER_START();
		PointCloud pointcloud;
		if (OPT::fSampleMesh > 0)
			scene.mesh.SamplePoints(OPT::fSampleMesh, 0, pointcloud);
		else
			scene.mesh.SamplePoints(ROUND2INT<unsigned>(-OPT::fSampleMesh), pointcloud);
		VERBOSE("Sample mesh completed: %u points (%s)", pointcloud.GetSize(), TD_TIMER_GET_FMT().c_str());
		pointcloud.Save(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName))+_T(".ply"));
		return EXIT_SUCCESS;
	}
	// load and estimate a dense point-cloud
	const Scene::SCENE_TYPE sceneType(scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)));
	if (sceneType == Scene::SCENE_NA)
		return EXIT_FAILURE;
	if (!OPT::strPointCloudFileName.empty() && !scene.pointcloud.Load(MAKE_PATH_SAFE(OPT::strPointCloudFileName))) {
		VERBOSE("error: cannot load point-cloud file");
		return EXIT_FAILURE;
	}
	if (!OPT::strMaskPath.empty()) {
		Util::ensureValidFolderPath(OPT::strMaskPath);
		for (Image& image : scene.images) {
			if (!image.maskName.empty()) {
				VERBOSE("error: Image %s has non-empty maskName %s", image.name.c_str(), image.maskName.c_str());
				return EXIT_FAILURE;
			}
			image.maskName = OPT::strMaskPath + Util::getFileName(image.name) + ".mask.png";
			if (!File::access(image.maskName)) {
				VERBOSE("error: Mask image %s not found", image.maskName.c_str());
				return EXIT_FAILURE;
			}
		}
	}
	if (!OPT::strImportROIFileName.empty()) {
		std::ifstream fs(MAKE_PATH_SAFE(OPT::strImportROIFileName));
		if (!fs)
			return EXIT_FAILURE;
		fs >> scene.obb;
		scene.Save(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName))+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
		return EXIT_SUCCESS;
	}
	if (!scene.IsBounded())
		scene.EstimateROI(OPT::nEstimateROI, 1.1f);
	if (!OPT::strExportROIFileName.empty() && scene.IsBounded()) {
		std::ofstream fs(MAKE_PATH_SAFE(OPT::strExportROIFileName));
		if (!fs)
			return EXIT_FAILURE;
		fs << scene.obb;
		return EXIT_SUCCESS;
	}
	if (OPT::nTowerMode!=0)
		scene.InitTowerScene(OPT::nTowerMode);
	if (!OPT::strMeshFileName.empty())
		scene.mesh.Load(MAKE_PATH_SAFE(OPT::strMeshFileName));
	if (!OPT::strViewNeighborsFileName.empty())
		scene.LoadViewNeighbors(MAKE_PATH_SAFE(OPT::strViewNeighborsFileName));
	if (!OPT::strOutputViewNeighborsFileName.empty()) {
		if (!scene.ImagesHaveNeighbors()) {
			VERBOSE("error: neighbor views not computed yet");
			return EXIT_FAILURE;
		}
		scene.SaveViewNeighbors(MAKE_PATH_SAFE(OPT::strOutputViewNeighborsFileName));
		return EXIT_SUCCESS;
	}
	if (!OPT::strExportDepthMapsName.empty() && !scene.mesh.IsEmpty()) {
		// project mesh onto each image and save the resulted depth-maps
		TD_TIMER_START();
		if (!scene.ExportMeshToDepthMaps(MAKE_PATH_SAFE(OPT::strExportDepthMapsName)))
			return EXIT_FAILURE;
		VERBOSE("Mesh projection completed: %u depth-maps (%s)", scene.images.size(), TD_TIMER_GET_FMT().c_str());
		return EXIT_SUCCESS;
	}
	if (OPT::fMaxSubsceneArea > 0) {
		// split the scene in sub-scenes by maximum sampling area
		Scene::ImagesChunkArr chunks;
		scene.Split(chunks, OPT::fMaxSubsceneArea);
		scene.ExportChunks(chunks, GET_PATH_FULL(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);
		return EXIT_SUCCESS;
	}
	if (OPT::thFilterPointCloud < 0) {
		// filter point-cloud based on camera-point visibility intersections
		scene.PointCloudFilter(OPT::thFilterPointCloud);
		const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName))+_T("_filtered"));
		scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
		scene.pointcloud.Save(baseFileName+_T(".ply"));
		return EXIT_SUCCESS;
	}
	if (OPT::nExportNumViews && scene.pointcloud.IsValid()) {
		// export point-cloud containing only points with N+ views
		const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName))+
			String::FormatString(_T("_%dviews"), ABS(OPT::nExportNumViews)));
		if (OPT::nExportNumViews > 0) {
			// export point-cloud containing only points with N+ views
			scene.pointcloud.SaveNViews(baseFileName+_T(".ply"), (IIndex)OPT::nExportNumViews);
		} else {
			// save scene and export point-cloud containing only points with N+ views
			scene.pointcloud.RemoveMinViews((IIndex)-OPT::nExportNumViews);
			scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
			scene.pointcloud.Save(baseFileName+_T(".ply"));
		}
		return EXIT_SUCCESS;
	}
	if (OPT::fEstimateScale > 0 && !scene.pointcloud.IsEmpty() && !scene.images.empty()) {
		// simply export existing point-cloud with scale
		if (scene.pointcloud.normals.empty()) {
			if (!scene.pointcloud.IsValid()) {
				VERBOSE("error: can not estimate normals as the point-cloud is not valid");
				return EXIT_FAILURE;
			}
			EstimatePointNormals(scene.images, scene.pointcloud);
		}
		const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
		scene.pointcloud.SaveWithScale(baseFileName+_T("_scale.ply"), scene.images, OPT::fEstimateScale);
		return EXIT_SUCCESS;
	}
	PointCloud sparsePointCloud;
	if ((ARCHIVE_TYPE)OPT::nArchiveType != ARCHIVE_MVS || sceneType == Scene::SCENE_INTERFACE) {
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1 && !scene.pointcloud.IsEmpty())
			scene.pointcloud.PrintStatistics(scene.images.data(), &scene.obb);
		#endif
		if ((ARCHIVE_TYPE)OPT::nArchiveType == ARCHIVE_MVS)
			sparsePointCloud = scene.pointcloud;
		TD_TIMER_START();
		if (!scene.DenseReconstruction(OPT::nFusionMode, OPT::bCrop2ROI, OPT::fBorderROI)) {
			if (ABS(OPT::nFusionMode) != 1)
				return EXIT_FAILURE;
			VERBOSE("Depth-maps estimated (%s)", TD_TIMER_GET_FMT().c_str());
			return EXIT_SUCCESS;
		}
		VERBOSE("Densifying point-cloud completed: %u points (%s)", scene.pointcloud.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// save the final point-cloud
	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
	scene.pointcloud.Save(baseFileName+_T(".ply"), (ARCHIVE_TYPE)OPT::nArchiveType == ARCHIVE_MVS);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+_T(".ply"));
	#endif
	if ((ARCHIVE_TYPE)OPT::nArchiveType == ARCHIVE_MVS)
		scene.pointcloud.Swap(sparsePointCloud);
	scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
