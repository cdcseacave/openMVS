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
#include "../../libs/SFM/RoMa2Matcher.h" // RoMa2Onnx::IsAvailable()
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
String strImportPosesFile;
String strKnownPosesConvention;
FramesConvention knownPosesConvention;
String strExportPosesCSV;
String strExportPoseQuality;
String strImportOpenMVGDir;
String strExportOpenMVGDir;
String strExportPairsCSV;
String strExportRetrievalCSV;
String strCompareMVS;
int matchMode;
unsigned importPosesMode;
unsigned matchSequenceOverlap;
unsigned maxPairsPerImage;
bool matchVerificationFeedback;
bool releaseDescriptors;
bool matchImagesOnly;
bool bROMA2;
String strROMA2Model;
String strROMA2Setting;
bool bROMA2Retrieval;
bool bROMA2Match;
unsigned nROMA2Slots;
unsigned nROMA2SkipHealthy;
unsigned nROMA2MaxReplace;
String strROMA2RetrievalRecipe;
String strROMA2Provider;
float defaultFocalRatio;
float focalLength;
float k1;
float k2;
String strImageIndices;
unsigned nMaxFeaturesPerCell;
unsigned nMinFeaturesPerCell;
unsigned maxViewsPerCluster;
bool bClusterCommunities;
bool bUseGlobalSolver;
bool bExtractColors;
float undistortAlpha;
String strUndistortExt;
float thAlignGPS;
double gpsPositionWeight;
double gpsPositionWeightZ;
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
		("gpu-device", boost::program_options::value<std::string>(&SEACAVE::CUDA::desiredDeviceIDs)->default_value("-1"), "GPU device(s) for processing (-1 best GPU, -2/cpu/empty CPU/GLSL, >=0 comma-separated IDs)")
		#endif
		;

	boost::program_options::options_description config("Reconstruction options");
	config.add_options()
		("source,s", boost::program_options::value<std::string>(&OPT::strSource), "source folder or semicolon-separated list of images")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output scene file path")
		("export-mvs", boost::program_options::value<std::string>(&OPT::strOutputFileNameMVS), "output MVS file path (optional)")
		("detector-type,t", boost::program_options::value<std::string>(&OPT::strDetectorType)->default_value(FeatureTypeToString(FeatureType::DEFAULT)), "feature detector type: AKAZE, ORB, SIFT or SIFTGPU")
		("import-poses-file", boost::program_options::value<std::string>(&OPT::strImportPosesFile)->default_value("poses.csv"), "import camera poses from file: .csv (OpenMVS pose CSV) or .json (frames.json)")
		("export-poses-csv", boost::program_options::value<std::string>(&OPT::strExportPosesCSV), "export camera poses to CSV file (optional)")
		("export-pose-quality", boost::program_options::value<std::string>(&OPT::strExportPoseQuality), "estimate the pose covariance during the final bundle adjustment and export the per-image quality report to CSV file (optional)")
		("import-poses-mode", boost::program_options::value(&OPT::importPosesMode)->default_value(0), "mode for importing camera poses: 0=none, 1=poses+intrinsics, 2=poses only, 3=positions only")
		("known-poses-convention", boost::program_options::value<std::string>(&OPT::strKnownPosesConvention), "camera-axes convention of the poses in a frames.json: arkit|opencv (default: auto-detect)")
		("import-openmvg-dir", boost::program_options::value<std::string>(&OPT::strImportOpenMVGDir), "import OpenMVG features from directory (optional)")
		("export-openmvg-dir", boost::program_options::value<std::string>(&OPT::strExportOpenMVGDir), "export OpenMVG features to directory (optional)")
		("export-pairs-csv", boost::program_options::value<std::string>(&OPT::strExportPairsCSV), "export image pairs to CSV file (optional)")
		("export-retrieval-csv", boost::program_options::value<std::string>(&OPT::strExportRetrievalCSV), "export the per-image global-descriptor retrieval rankings to CSV file (optional)")
		("compare-mvs", boost::program_options::value<std::string>(&OPT::strCompareMVS), "compare reconstruction against ground-truth MVS file (optional)")
		("max-features-per-cell", boost::program_options::value(&OPT::nMaxFeaturesPerCell)->default_value(3000), "maximum features per grid cell (3x3 grid)")
		("min-features-per-cell", boost::program_options::value(&OPT::nMinFeaturesPerCell)->default_value(500), "minimum features per cell before adjusting sensitivity")
		("match-mode", boost::program_options::value(&OPT::matchMode)->default_value(1), "match mode: -1=SKIP,0=EXHAUSTIVE,1=VOCABULARY,2=SEQUENTIAL,3=KNOWN_POSES")
		("match-sequence-overlap", boost::program_options::value(&OPT::matchSequenceOverlap)->default_value(3), "sequence overlap for sequential matching")
		("vocab-max-pairs", boost::program_options::value(&OPT::maxPairsPerImage)->default_value(50), "target pairs per image for vocabulary and pose-guided matching")
		("match-verification-feedback", boost::program_options::value(&OPT::matchVerificationFeedback)->default_value(true), "hold back part of the matching budget and re-invest it in pairs suggested by the geometrically verified matches (vocabulary and pose-guided matching)")
		("release-descriptors", boost::program_options::value(&OPT::releaseDescriptors)->default_value(true), "release descriptors after matching to save memory")
		("match-images-only", boost::program_options::value(&OPT::matchImagesOnly)->default_value(false), "match only the image pairs and save the scene without reconstruction (release descriptors)")
		("roma2", boost::program_options::value<bool>(&OPT::bROMA2)->default_value(false), "enable the in-process RoMa v2 model: DINOv3/GeM pair retrieval and dense matching (needs an exported model, see --roma2-model)")
		("roma2-model", boost::program_options::value<std::string>(&OPT::strROMA2Model), "directory with the RoMa v2 ONNX graphs + manifest (default: $OPENMVS_ROMA2_MODEL_PATH)")
		("roma2-setting", boost::program_options::value<std::string>(&OPT::strROMA2Setting)->default_value("base"), "RoMa v2 export preset: turbo (320px), fast (512px) or base (640px)")
		("roma2-retrieval", boost::program_options::value<bool>(&OPT::bROMA2Retrieval)->default_value(true), "rank candidate pairs by the global descriptors instead of the vocabulary tree")
		("roma2-match", boost::program_options::value<bool>(&OPT::bROMA2Match)->default_value(true), "dense-match every candidate pair and replace weaker descriptor matches")
		("roma2-slots", boost::program_options::value(&OPT::nROMA2Slots)->default_value(64), "images kept resident on the device while dense matching (12.5 MB each at base)")
		("roma2-skip-healthy", boost::program_options::value(&OPT::nROMA2SkipHealthy)->default_value(0), "round-1 dense matching: skip pairs that already have at least this many inliers (0 = warp every pair)")
		("roma2-max-replace", boost::program_options::value(&OPT::nROMA2MaxReplace)->default_value(0), "round-1 dense matching: replace only pairs with fewer than this many inliers (0 = replace any weaker pair)")
		("roma2-retrieval-recipe", boost::program_options::value<std::string>(&OPT::strROMA2RetrievalRecipe)->default_value("facets"), "global descriptor pooling: facets (value projections of blocks 15+20, 2048-D) or layers (GeM on the matcher's deepest layer, 1024-D, legacy)")
		("roma2-provider", boost::program_options::value<std::string>(&OPT::strROMA2Provider)->default_value("auto"), "ONNX Runtime execution provider: auto (CUDA > CoreML > DirectML > CPU), cuda, coreml, dml or cpu")
		("default-focal-ratio", boost::program_options::value(&OPT::defaultFocalRatio)->default_value(1.2f), "focal-length is set to ratio * max(width,height) for images with unknown focal-length")
		("focal-length,f", boost::program_options::value(&OPT::focalLength)->default_value(0.f), "force focal-length (in pixels) for specified images (0 = disabled)")
		("k1", boost::program_options::value(&OPT::k1)->default_value(0.f), "force k1 distortion coefficient for specified images (0 = not used)")
		("k2", boost::program_options::value(&OPT::k2)->default_value(0.f), "force k2 distortion coefficient for specified images (0 = not used)")
		("image-indices", boost::program_options::value<std::string>(&OPT::strImageIndices), "image indices to apply forced parameters (e.g., '0 5-10 15', empty = all images)")
		("max-views-per-cluster", boost::program_options::value(&OPT::maxViewsPerCluster)->default_value(200), "maximum images per cluster for hierarchical reconstruction (0 = disable clustering)")
		("cluster-communities", boost::program_options::value<bool>(&OPT::bClusterCommunities)->default_value(false), "cluster by community detection + capacity packing instead of pure aggregative clustering")
		("use-global-solver", boost::program_options::value<bool>(&OPT::bUseGlobalSolver)->default_value(false), "use global solver for calibration instead of the hierarchical solver")
		("extract-colors", boost::program_options::value<bool>(&OPT::bExtractColors)->default_value(false), "extract colors for reconstructed points")
		("undistort-alpha", boost::program_options::value<float>(&OPT::undistortAlpha)->default_value(0.6f), "alpha parameter for undistortion (0=zoomed in, 1=all pixels retained)")
		("undistort-extension", boost::program_options::value<std::string>(&OPT::strUndistortExt)->default_value(".jxl"), "file extension/format for the exported undistorted images (e.g. .jpg, .png, .jxl)")
		("align-gps-threshold", boost::program_options::value<float>(&OPT::thAlignGPS)->default_value(5.f), "maximum distance in meters for aligning GPS positions to reconstruction poses (0 = disabled)")
		("gps-position-weight", boost::program_options::value(&OPT::gpsPositionWeight)->default_value(0.0), "horizontal weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled)")
		("gps-position-weight-z", boost::program_options::value(&OPT::gpsPositionWeightZ)->default_value(0.0), "vertical weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled)")
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
	if (OPT::importPosesMode > static_cast<unsigned>(PoseImportMode::POSITIONS)) {
		LOG("error: unknown import poses mode %u (accepted: 0, 1, 2, 3)", OPT::importPosesMode);
		return false;
	}
	if (OPT::matchMode < static_cast<int>(MatchConfig::SKIP) ||
		OPT::matchMode > static_cast<int>(MatchConfig::KNOWN_POSES)) {
		LOG("error: unknown match mode %d (accepted: -1, 0, 1, 2, 3)", OPT::matchMode);
		return false;
	}
	Util::ensureValidPath(OPT::strImportPosesFile);
	// Parse the camera-axes convention; empty means auto-detect.
	if (!FramesConventionFromString(OPT::strKnownPosesConvention, OPT::knownPosesConvention)) {
		LOG("error: unknown known-poses convention '%s' (accepted: auto, arkit, opencv)", OPT::strKnownPosesConvention.c_str());
		return false;
	}
	Util::ensureValidPath(OPT::strExportPosesCSV);
	Util::ensureValidPath(OPT::strExportPoseQuality);
	Util::ensureValidFolderPath(OPT::strImportOpenMVGDir);
	Util::ensureValidFolderPath(OPT::strExportOpenMVGDir);
	Util::ensureValidPath(OPT::strExportPairsCSV);
	Util::ensureValidPath(OPT::strExportRetrievalCSV);
	if (!OPT::strExportRetrievalCSV.empty() && (!OPT::bROMA2 || !OPT::bROMA2Retrieval)) {
		LOG("error: --export-retrieval-csv needs --roma2 true and --roma2-retrieval true");
		return false;
	}
	Util::ensureValidPath(OPT::strCompareMVS);
	Util::ensureValidFolderPath(OPT::strROMA2Model);
	if (OPT::bROMA2 && !RoMa2Onnx::IsAvailable()) {
		LOG("error: --roma2 needs a build with ONNX Runtime (-DOpenMVS_USE_ONNXRUNTIME=ON)");
		return false;
	}
	if (OPT::strROMA2Setting != "turbo" && OPT::strROMA2Setting != "fast" && OPT::strROMA2Setting != "base") {
		LOG("error: unknown ROMA2 export preset '%s' (accepted: turbo, fast, base)", OPT::strROMA2Setting.c_str());
		return false;
	}
	if (OPT::strROMA2RetrievalRecipe != "facets" && OPT::strROMA2RetrievalRecipe != "layers") {
		LOG("error: unknown ROMA2 retrieval recipe '%s' (accepted: facets, layers)", OPT::strROMA2RetrievalRecipe.c_str());
		return false;
	}
	if (OPT::strROMA2Provider != "auto" && OPT::strROMA2Provider != "cuda" && OPT::strROMA2Provider != "coreml" &&
		OPT::strROMA2Provider != "dml" && OPT::strROMA2Provider != "cpu") {
		LOG("error: unknown ROMA2 execution provider '%s' (accepted: auto, cuda, coreml, dml, cpu)", OPT::strROMA2Provider.c_str());
		return false;
	}
	if (OPT::bROMA2 && (OPT::bROMA2Retrieval || OPT::bROMA2Match)) {
		// the library refuses this same condition inside Scene::MatchPairs (design decision 10),
		// gated the same way (enabled && (useRetrieval || useMatching)), Scene.cpp:574; this early
		// check just gives the hint before any feature extraction runs
		ROMA2Config roma2Cfg;
		roma2Cfg.modelPath = OPT::strROMA2Model;
		if (roma2Cfg.ResolveModelPath().empty()) {
			LOG("error: --roma2 needs a model (set --roma2-model or $OPENMVS_ROMA2_MODEL_PATH)");
			return false;
		}
	}

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
	cfg.importCfg.importPosesFile = OPT::importPosesMode ? OPT::strImportPosesFile : String();
	cfg.importCfg.importPosesMode = static_cast<SFM::PoseImportMode>(OPT::importPosesMode);
	cfg.importCfg.framesConvention = OPT::knownPosesConvention;
	cfg.importCfg.archiveType = (ARCHIVE_TYPE)OPT::nArchiveType;
	cfg.featuresCfg.detectorType = FeatureTypeFromString(OPT::strDetectorType);
	cfg.featuresCfg.maxFeaturesPerCell = OPT::nMaxFeaturesPerCell;
	cfg.featuresCfg.minFeaturesPerCell = OPT::nMinFeaturesPerCell;
	cfg.featuresCfg.importOpenMVGDir = OPT::strImportOpenMVGDir;
	cfg.featuresCfg.exportOpenMVGDir = OPT::strExportOpenMVGDir;
	cfg.matchCfg.DefaultsForFeatureType(cfg.featuresCfg.detectorType);
	cfg.matchCfg.mode = static_cast<MatchConfig::MatchMode>(OPT::matchMode);
	cfg.matchCfg.matchSequenceOverlap = OPT::matchSequenceOverlap;
	cfg.matchCfg.maxPairsPerImage = OPT::maxPairsPerImage;
	cfg.matchCfg.verificationFeedback = OPT::matchVerificationFeedback;
	cfg.matchCfg.releaseDescriptors = OPT::releaseDescriptors;
	cfg.roma2Cfg.enabled = OPT::bROMA2;
	cfg.roma2Cfg.modelPath = OPT::strROMA2Model;
	cfg.roma2Cfg.setting = OPT::strROMA2Setting;
	cfg.roma2Cfg.useRetrieval = OPT::bROMA2Retrieval;
	cfg.roma2Cfg.useMatching = OPT::bROMA2Match;
	cfg.roma2Cfg.slotBudget = OPT::nROMA2Slots;
	cfg.roma2Cfg.skipHealthyInliers = OPT::nROMA2SkipHealthy;
	cfg.roma2Cfg.maxReplaceInliers = OPT::nROMA2MaxReplace;
	cfg.roma2Cfg.retrievalRecipe = (OPT::strROMA2RetrievalRecipe == "layers") ? RetrievalRecipe::LAYERS : RetrievalRecipe::FACETS;
	cfg.roma2Cfg.provider = OPT::strROMA2Provider;
	#ifdef _USE_CUDA
	cfg.matchCfg.useCUDA = cfg.featuresCfg.useCUDA = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	cfg.roma2Cfg.useGPU = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	#endif
	cfg.matchImagesOnly = OPT::matchImagesOnly;
	cfg.viewgraphCfg.maxTwoViewError = 0; // disable pair filtering after ViewGraph calibration
	cfg.useGlobalSolver = OPT::bUseGlobalSolver;
	cfg.thAlignGPS = OPT::thAlignGPS;
	cfg.baConfig.gpsPositionWeight = OPT::gpsPositionWeight;
	cfg.baConfig.gpsPositionWeightZ = OPT::gpsPositionWeightZ;
	cfg.estimatePoseUncertainty = !OPT::strExportPoseQuality.empty();
	cfg.extractColors = OPT::bExtractColors;
	cfg.clusterCfg.maxViewsPerCluster = OPT::maxViewsPerCluster;
	cfg.clusterCfg.useCommunityDetection = OPT::bClusterCommunities;

	// known-poses mode: pose-guided pair selection unless the user chose a mode explicitly
	// (bringing the result back to the imported pose frame, which takes precedence over the
	// GPS alignment, is handled inside Scene::Reconstruct so every caller gets it)
	if (cfg.HasKnownPoses() && OPT::vm["match-mode"].defaulted()) {
		cfg.matchCfg.mode = MatchConfig::KNOWN_POSES;
		VERBOSE("Known camera poses imported: pose-guided pair selection auto-selected (use --match-mode to override)");
	}

	// Run SfM reconstruction
	Scene scene(OPT::nMaxThreads);
	if (!scene.Reconstruct(OPT::strSource, cfg)) {
		// a scene calibrated before the failure is a usable partial result and is still
		// exported below; anything less (including a match-images-only run that failed to
		// resolve the pose convention) must propagate to the process exit status
		if (!scene.status.nState.isSet(Scene::Status::STATE::CALIBRATED)) {
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
	// Export per-image pose quality report to CSV file; this is an optional diagnostic, so a
	// failure to produce it (e.g. a rank-deficient covariance yielding no rows) must not abort
	// the run and lose the primary outputs (the scene and the MVS export below)
	if (!OPT::strExportPoseQuality.empty() && !ExportPoseUncertaintyCSV(MAKE_PATH_SAFE(OPT::strExportPoseQuality), scene))
		VERBOSE("warning: failed to export pose quality report to CSV file %s", OPT::strExportPoseQuality.c_str());
	// Export image pairs to CSV file
	if (!OPT::strExportPairsCSV.empty() && !PairsMatcher::ExportPairsCSV(scene, MAKE_PATH_SAFE(OPT::strExportPairsCSV), 3.f)) {
		VERBOSE("error: failed to export image pairs to CSV file %s", OPT::strExportPairsCSV.c_str());
		return EXIT_FAILURE;
	}
	// Export global-descriptor retrieval rankings to CSV file; this is an optional diagnostic (like
	// the pose quality report above), so a failure to produce it must not abort the run and lose
	// the primary outputs (the scene and the MVS export below)
	if (!OPT::strExportRetrievalCSV.empty() && !ExportRetrievalRankingsCSV(scene, MAKE_PATH_SAFE(OPT::strExportRetrievalCSV), 50))
		VERBOSE("warning: failed to export retrieval rankings to CSV file %s", OPT::strExportRetrievalCSV.c_str());
	// Export MVS scene
	if (!OPT::strOutputFileNameMVS.empty()) {
		SFM::ExportMVSConfig cfg;
		cfg.undistortImageDir = MAKE_PATH("undistorted");
		cfg.undistortAlpha    = OPT::undistortAlpha;
		if (!OPT::strUndistortExt.empty())
			cfg.extension = OPT::strUndistortExt;
		if (!ExportMVS(MAKE_PATH_SAFE(OPT::strOutputFileNameMVS), scene, cfg)) {
			VERBOSE("error: failed to export MVS file to %s", OPT::strOutputFileNameMVS.c_str());
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
