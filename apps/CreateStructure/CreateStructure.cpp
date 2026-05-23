/*
 * CreateStructure.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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

#include "../../libs/SFM.h"
#include <boost/program_options.hpp>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("CreateStructure")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strSource;
String strOutputFileName;
String strOutputFileNameMVS;
String strDetectorType;
String strImportPosesCSV;
String strExportPosesCSV;
String strImportOpenMVGDir;
String strExportOpenMVGDir;
String strExportPairsCSV;
String strImportROMA2Path;
String strCompareMVS;
int matchMode;
unsigned importPosesMode;
unsigned matchSequenceOverlap;
unsigned maxPairsPerImage;
unsigned expandPairsTopK;
bool releaseDescriptors;
bool matchImagesOnly;
float defaultFocalRatio;
float focalLength;
float k1;
float k2;
String strImageIndices;
unsigned nMaxFeaturesPerCell;
unsigned nMinFeaturesPerCell;
unsigned maxViewsPerCluster;
bool bUseGlobalSolver;
bool bExtractColors;
float undistortAlpha;
float thAlignGPS;
unsigned nMaxThreads;
int nArchiveType;
int nProcessPriority;
String strConfigFileName;
boost::program_options::variables_map vm;
}

class Application {
public:
	Application() {}
	~Application() { Finalize(); }

	bool Initialize(size_t argc, LPCTSTR* argv);
	void Finalize();
};

bool Application::Initialize(size_t argc, LPCTSTR* argv)
{
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_DEFAULT), "project archive type: 0-text, 1-binary, 2-compressed binary")
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
		("cuda-device", boost::program_options::value<std::string>(&SEACAVE::CUDA::desiredDeviceIDs)->default_value("-1"), "CUDA device(s) for processing (-1 best GPU, -2/cpu/empty CPU/GLSL, >=0 comma-separated IDs)")
		#endif
		;

	boost::program_options::options_description config("Reconstruction options");
	config.add_options()
		("source,s", boost::program_options::value<std::string>(&OPT::strSource), "source folder or semicolon-separated list of images")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output scene file path")
		("export-mvs", boost::program_options::value<std::string>(&OPT::strOutputFileNameMVS), "output MVS file path (optional)")
		("detector-type,t", boost::program_options::value<std::string>(&OPT::strDetectorType)->default_value(FeatureTypeToString(FeatureType::DEFAULT)), "feature detector type: AKAZE, ORB, SIFT or SIFTGPU")
		("import-poses-csv", boost::program_options::value<std::string>(&OPT::strImportPosesCSV)->default_value("poses.csv"), "import camera poses from CSV file (optional)")
		("export-poses-csv", boost::program_options::value<std::string>(&OPT::strExportPosesCSV), "export camera poses to CSV file (optional)")
		("import-poses-mode", boost::program_options::value(&OPT::importPosesMode)->default_value(0), "mode for importing camera poses from CSV: 0=none, 1=all, 2=extrinsics only, 3=positions only")
		("import-openmvg-dir", boost::program_options::value<std::string>(&OPT::strImportOpenMVGDir), "import OpenMVG features from directory (optional)")
		("export-openmvg-dir", boost::program_options::value<std::string>(&OPT::strExportOpenMVGDir), "export OpenMVG features to directory (optional)")
		("export-pairs-csv", boost::program_options::value<std::string>(&OPT::strExportPairsCSV), "export image pairs to CSV file (optional)")
		("import-roma2", boost::program_options::value<std::string>(&OPT::strImportROMA2Path), "import ROMA2 reconstruction from .npz files (folder or semicolon-separated list)")
		("compare-mvs", boost::program_options::value<std::string>(&OPT::strCompareMVS), "compare reconstruction against ground-truth MVS file (optional)")
		("max-features-per-cell", boost::program_options::value(&OPT::nMaxFeaturesPerCell)->default_value(3000), "maximum features per grid cell (3x3 grid)")
		("min-features-per-cell", boost::program_options::value(&OPT::nMinFeaturesPerCell)->default_value(500), "minimum features per cell before adjusting sensitivity")
		("match-mode", boost::program_options::value(&OPT::matchMode)->default_value(1), "match mode: -1=SKIP,0=EXHAUSTIVE,1=VOCABULARY,2=SEQUENTIAL")
		("match-sequence-overlap", boost::program_options::value(&OPT::matchSequenceOverlap)->default_value(3), "sequence overlap for sequential matching")
		("vocab-max-pairs", boost::program_options::value(&OPT::maxPairsPerImage)->default_value(50), "maximum pairs per image for vocabulary matching")
		("expand-pairs-topk", boost::program_options::value(&OPT::expandPairsTopK)->default_value(5), "top-K per endpoint to expand vocabulary pairs (0 = disable)")
		("release-descriptors", boost::program_options::value(&OPT::releaseDescriptors)->default_value(true), "release descriptors after matching to save memory")
		("match-images-only", boost::program_options::value(&OPT::matchImagesOnly)->default_value(false), "match only the image pairs and save the scene without reconstruction (release descriptors)")
		("default-focal-ratio", boost::program_options::value(&OPT::defaultFocalRatio)->default_value(1.2f), "focal-length is set to ratio * max(width,height) for images with unknown focal-length")
		("focal-length,f", boost::program_options::value(&OPT::focalLength)->default_value(0.f), "force focal-length (in pixels) for specified images (0 = disabled)")
		("k1", boost::program_options::value(&OPT::k1)->default_value(0.f), "force k1 distortion coefficient for specified images (0 = not used)")
		("k2", boost::program_options::value(&OPT::k2)->default_value(0.f), "force k2 distortion coefficient for specified images (0 = not used)")
		("image-indices", boost::program_options::value<std::string>(&OPT::strImageIndices), "image indices to apply forced parameters (e.g., '0 5-10 15', empty = all images)")
		("max-views-per-cluster", boost::program_options::value(&OPT::maxViewsPerCluster)->default_value(200), "maximum images per cluster for hierarchical reconstruction (0 = disable clustering)")
		("use-global-solver", boost::program_options::value<bool>(&OPT::bUseGlobalSolver)->default_value(false), "use global solver for calibration, istead of hierarhical solver")
		("extract-colors", boost::program_options::value<bool>(&OPT::bExtractColors)->default_value(false), "extract colors for reconstructed points")
		("undistort-alpha", boost::program_options::value<float>(&OPT::undistortAlpha)->default_value(0.6f), "alpha parameter for undistortion (0=zoomed in, 1=all pixels retained)")
		("align-gps-threshold", boost::program_options::value<float>(&OPT::thAlignGPS)->default_value(5.f), "maximum distance in meters for aligning GPS positions to reconstruction poses (0 = disabled)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config);

	boost::program_options::positional_options_description p;
	p.add("source", -1);

	try {
		boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
		boost::program_options::notify(OPT::vm);
		INIT_WORKING_FOLDER;

		std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName).c_str());
		if (ifs) {
			boost::program_options::store(parse_config_file(ifs, config_file_options), OPT::vm);
			boost::program_options::notify(OPT::vm);
		}
	} catch (const std::exception& e) {
		LOG(e.what());
		return false;
	}

	// initialize the log file
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strSource);
	if (OPT::vm.count("help") || OPT::strSource.empty()) {
		GET_LOG() << cmdline_options;
		if (OPT::strSource.empty())
			LOG("error: source (folder or list) is required");
		return false;
	}
	Util::ensureValidPath(OPT::strOutputFileName);
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = _T("scene.sfm");
	Util::ensureValidPath(OPT::strOutputFileNameMVS);
	Util::ensureValidPath(OPT::strImportPosesCSV);
	Util::ensureValidPath(OPT::strExportPosesCSV);
	Util::ensureValidFolderPath(OPT::strImportOpenMVGDir);
	Util::ensureValidFolderPath(OPT::strExportOpenMVGDir);
	Util::ensureValidPath(OPT::strExportPairsCSV);
	Util::ensureValidPath(OPT::strImportROMA2Path);
	Util::ensureValidPath(OPT::strCompareMVS);

	// Use max threads option if provided
	SEACAVE::Initialize(APPNAME, OPT::nMaxThreads, OPT::nProcessPriority);
	return true;
}

void Application::Finalize()
{
	SEACAVE::Finalize();
	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index or use _CrtSetBreakAlloc() to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	// Prepare reconstruction config
	ReconstructionConfig cfg;
	cfg.importCfg.defaultFocalRatio = OPT::defaultFocalRatio;
	cfg.importCfg.focalLength = OPT::focalLength;
	cfg.importCfg.k1 = OPT::k1;
	cfg.importCfg.k2 = OPT::k2;
	cfg.importCfg.imageIndicesStr = OPT::strImageIndices;
	cfg.importCfg.importPosesCSV = OPT::importPosesMode ? OPT::strImportPosesCSV : String();
	cfg.importCfg.importPosesMode = OPT::importPosesMode ? OPT::importPosesMode - 1 : 0;
	cfg.importCfg.archiveType = (ARCHIVE_TYPE)OPT::nArchiveType;
	cfg.featuresCfg.detectorType = FeatureTypeFromString(OPT::strDetectorType);
	cfg.featuresCfg.maxFeaturesPerCell = OPT::nMaxFeaturesPerCell;
	cfg.featuresCfg.minFeaturesPerCell = OPT::nMinFeaturesPerCell;
	cfg.featuresCfg.importOpenMVGDir = OPT::strImportOpenMVGDir;
	cfg.featuresCfg.exportOpenMVGDir = OPT::strExportOpenMVGDir;
	cfg.roma2Cfg.importROMA2Path = OPT::strImportROMA2Path;
	cfg.matchCfg.DefaultsForFeatureType(cfg.featuresCfg.detectorType);
	cfg.matchCfg.mode = static_cast<MatchConfig::MatchMode>(OPT::matchMode);
	cfg.matchCfg.matchSequenceOverlap = OPT::matchSequenceOverlap;
	cfg.matchCfg.maxPairsPerImage = OPT::maxPairsPerImage;
	cfg.matchCfg.expandPairsTopK = OPT::expandPairsTopK;
	cfg.matchCfg.releaseDescriptors = OPT::releaseDescriptors;
	#ifdef _USE_CUDA
	cfg.matchCfg.useCUDA = cfg.featuresCfg.useCUDA = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	#endif
	cfg.matchImagesOnly = OPT::matchImagesOnly;
	cfg.viewgraphCfg.maxTwoViewError = 0; // disable pair filtering after ViewGraph calibration
	cfg.useGlobalSolver = OPT::bUseGlobalSolver;
	cfg.thAlignGPS = OPT::thAlignGPS;
	cfg.extractColors = OPT::bExtractColors;
	cfg.clusterCfg.maxViewsPerCluster = OPT::maxViewsPerCluster;

	// Run SfM reconstruction
	Scene scene(OPT::nMaxThreads);
	if (!scene.Reconstruct(OPT::strSource, cfg)) {
		if (!scene.status.nState.isSet(Scene::Status::STATE::CALIBRATED) &&
			!(cfg.matchImagesOnly && scene.status.nState.isSet(Scene::Status::STATE::MATCHED))) {
			VERBOSE("error: reconstruction failed");
			return EXIT_FAILURE;
		} else if (OPT::bExtractColors && scene.colors.empty() && !scene.SampleColors()) {
			VERBOSE("warning: color extraction failed");
		}
	} else if (!scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType)) {
		VERBOSE("error: failed to save reconstructed scene to %s", OPT::strOutputFileName.c_str());
		return EXIT_FAILURE;
	}
	// Compare against ground-truth MVS scene
	if (!OPT::strCompareMVS.empty()) {
		if (!CompareScenes(scene, MAKE_PATH_SAFE(OPT::strCompareMVS)))
			VERBOSE("warning: scene comparison against '%s' failed", OPT::strCompareMVS.c_str());
	}
	// Export camera poses to CSV file
	if (!OPT::strExportPosesCSV.empty() && !ExportPosesCSV(OPT::strExportPosesCSV, scene.images)) {
		VERBOSE("error: failed to export camera poses to CSV file %s", OPT::strExportPosesCSV.c_str());
		return EXIT_FAILURE;
	}
	// Export image pairs to CSV file
	if (!OPT::strExportPairsCSV.empty() && !PairsMatcher::ExportPairsCSV(scene, MAKE_PATH_SAFE(OPT::strExportPairsCSV), 3.f)) {
		VERBOSE("error: failed to export image pairs to CSV file %s", OPT::strExportPairsCSV.c_str());
		return EXIT_FAILURE;
	}
	// Export MVS scene
	if (!OPT::strOutputFileNameMVS.empty()) {
		SFM::ExportMVSConfig cfg;
		cfg.undistortImageDir = MAKE_PATH("undistorted");
		cfg.undistortAlpha    = OPT::undistortAlpha;
		if (!ExportMVS(MAKE_PATH_SAFE(OPT::strOutputFileNameMVS), scene, cfg)) {
			VERBOSE("error: failed to export MVS file to %s", OPT::strOutputFileNameMVS.c_str());
			return EXIT_FAILURE;
		}
	}
	// Generate depth-maps from ROMA2 NPZ files
	CLISTDEF2(String) depthMapFiles;
	if (!OPT::strImportROMA2Path.empty() && ImportROMA2DepthMaps(scene, cfg.roma2Cfg, &depthMapFiles) == 0) {
		VERBOSE("error: failed to generate depth-maps from '%s'", cfg.roma2Cfg.importROMA2Path.c_str());
		return EXIT_FAILURE;
	}
	if (!depthMapFiles.empty())
		UndistortDepthMaps(scene, depthMapFiles, OPT::undistortAlpha);
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
