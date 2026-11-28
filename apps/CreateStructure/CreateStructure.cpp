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
#include "../../libs/SFM/DenseMatcher.h" // DenseMatcher::IsAvailable()
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
String strCompareMVS;
int matchMode;
unsigned importPosesMode;
unsigned matchSequenceOverlap;
bool imageSequence;
unsigned sequenceMaxGap;
unsigned maxPairsPerImage;
bool matchVerificationFeedback;
bool releaseDescriptors;
bool matchImagesOnly;
String strROMA2Model;
String strDenseMatcher;
String strFlexGlueModel;
float fDenseMinCovisibility;
bool bDenseMatch;
unsigned nDenseSlots;
float fDenseMinConfidence;
float fDenseMinOverlap;
unsigned nDenseMatches;
bool bFilterTriplets;
bool bTripletCut;
float fTripletMinScore;
float fTripletSecondFaceScore;
float fMaxPairDirectionResidual;
bool bRepairContradictedPairs;
float fMaxLinkDirectionError;
String strDenseProvider;
float defaultFocalRatio;
int nCameraType;
float focalLength;
float k1;
float k2;
String strImageIndices;
unsigned nMaxFeaturesPerCell;
unsigned nMinFeaturesPerCell;
bool bSiftGPUDarknessAdaptivity;
float fSphericalBottomBand;
bool bSiftGPUCUDA;
unsigned maxViewsPerCluster;
unsigned clusterAlignment;
bool bUseGlobalSolver;
bool bExtractColors;
float undistortAlpha;
String strUndistortExt;
String strSphericalExport;
float thAlignGPS;
double gpsPositionWeight;
double gpsPositionWeightZ;
double baDenseWeight;
unsigned baMaxObservations;
unsigned baCapMinObservations;
float baPairSigma;
unsigned refineIntrinsics;
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

// the --refine-intrinsics levels: the intrinsics flags of ReconstructionConfig each one stands for
constexpr unsigned REFINE_INTRINSICS_FLAGS[] = {
	ReconstructionConfig::INTRINSIC_NONE, ReconstructionConfig::INTRINSIC_MAIN, ReconstructionConfig::INTRINSIC_MAIN_EXTRA};

// the level the configuration's own default intrinsics flags correspond to
unsigned DefaultRefineIntrinsicsLevel(unsigned flags)
{
	for (unsigned level = 0; level < std::size(REFINE_INTRINSICS_FLAGS); ++level)
		if (REFINE_INTRINSICS_FLAGS[level] == flags)
			return level;
	ASSERT("the default intrinsics flags are not one of the --refine-intrinsics levels" == NULL);
	return 0;
}

bool Application::Initialize(size_t argc, LPCTSTR* argv)
{
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	// every default of an option below is the one of the configuration it fills
	const ReconstructionConfig defaults;

	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value((int)defaults.importCfg.archiveType), "project archive type: 0-text, 1-binary, 2-compressed binary")
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

	boost::program_options::options_description input("Input and camera options");
	input.add_options()
		("source,s", boost::program_options::value<std::string>(&OPT::strSource), "source folder or semicolon-separated list of images")
		("import-poses-file", boost::program_options::value<std::string>(&OPT::strImportPosesFile)->default_value("poses.csv"), "import camera poses from file: .csv (OpenMVS pose CSV) or .json (frames.json)")
		("import-poses-mode", boost::program_options::value(&OPT::importPosesMode)->default_value((unsigned)defaults.importCfg.importPosesMode), "mode for importing camera poses: 0=none, 1=poses+intrinsics, 2=poses only, 3=positions only")
		("known-poses-convention", boost::program_options::value<std::string>(&OPT::strKnownPosesConvention), "camera-axes convention of the poses in a frames.json: auto|arkit|opencv (default: auto, detect it from the poses)")
		("import-openmvg-dir", boost::program_options::value<std::string>(&OPT::strImportOpenMVGDir), "import OpenMVG features from directory (optional)")
		("camera-type", boost::program_options::value(&OPT::nCameraType)->default_value((int)defaults.importCfg.cameraType - 1), "camera model type: -1-from the image metadata (an equirectangular projection tag gives spherical, else pinhole), 0-pinhole, 1-spherical (equirectangular, width = 2 x height)")
		("default-focal-ratio", boost::program_options::value(&OPT::defaultFocalRatio)->default_value(defaults.importCfg.defaultFocalRatio), "focal-length is set to ratio * max(width,height) for images with unknown focal-length")
		("focal-length,f", boost::program_options::value(&OPT::focalLength)->default_value(defaults.importCfg.focalLength), "force focal-length (in pixels) for specified images (0 = disabled)")
		("k1", boost::program_options::value(&OPT::k1)->default_value(defaults.importCfg.k1), "force k1 distortion coefficient for specified images (0 = not used)")
		("k2", boost::program_options::value(&OPT::k2)->default_value(defaults.importCfg.k2), "force k2 distortion coefficient for specified images (0 = not used)")
		("image-indices", boost::program_options::value<std::string>(&OPT::strImageIndices), "image indices to apply forced parameters (e.g., '0 5-10 15', empty = all images)")
		;

	boost::program_options::options_description features("Feature extraction options");
	features.add_options()
		("detector-type,t", boost::program_options::value<std::string>(&OPT::strDetectorType)->default_value(FeatureTypeToString(FeatureType::DEFAULT)), "feature detector type: AKAZE, ORB, SIFT or SIFTGPU (any letter case)")
		("max-features-per-cell", boost::program_options::value(&OPT::nMaxFeaturesPerCell)->default_value(defaults.featuresCfg.maxFeaturesPerCell), "maximum features per grid cell (3x3 grid)")
		("min-features-per-cell", boost::program_options::value(&OPT::nMinFeaturesPerCell)->default_value(defaults.featuresCfg.minFeaturesPerCell), "minimum features per cell before adjusting sensitivity")
		("siftgpu-darkness-adaptivity", boost::program_options::value(&OPT::bSiftGPUDarknessAdaptivity)->default_value(defaults.featuresCfg.siftgpuDarknessAdaptivity), "SiftGPU: lower the DoG threshold and curvature floor in dark areas")
		("spherical-bottom-band", boost::program_options::value(&OPT::fSphericalBottomBand)->default_value(defaults.featuresCfg.sphericalBottomBand), "spherical images: the fraction of the image height, counted from the bottom, whose features and dense matches are discarded (the operator and the tripod are there), in [0,1); 0 = keep every feature; perspective images are not affected")
		("siftgpu-cuda", boost::program_options::value(&OPT::bSiftGPUCUDA)->default_value(defaults.featuresCfg.useCUDA), "SiftGPU feature extraction and matching backend: 1 = CUDA (if available), 0 = OpenGL/GLSL")
		;

	boost::program_options::options_description matching("Pair selection and matching options");
	matching.add_options()
		("match-mode", boost::program_options::value(&OPT::matchMode)->default_value((int)defaults.matchCfg.mode), "match mode: -1=SKIP (saved .sfm with matches), 0=EXHAUSTIVE, 1=VOCABULARY, 2=SEQUENTIAL, 3=KNOWN_POSES, 4=RETRIEVAL")
		("match-sequence-overlap", boost::program_options::value(&OPT::matchSequenceOverlap)->default_value(defaults.matchCfg.matchSequenceOverlap), "number of ahead neighbors to match sequentially (or add to vocabulary/retrieval; 0=disabled); for an image sequence, how many positions apart two images may be to be neighbours")
		("vocab-max-pairs", boost::program_options::value(&OPT::maxPairsPerImage)->default_value(defaults.matchCfg.maxPairsPerImage), "target pairs per image for vocabulary and pose-guided matching")
		("image-sequence", boost::program_options::value(&OPT::imageSequence)->default_value(defaults.matchCfg.imageSequence), "treat images in import order as an ordered sequence (always on for --match-mode 2): forces neighbor pairs and trusts the sequence chain in the contradiction cut (needs --max-pair-direction-residual above 0); a matched scene given back as the source needs the option again")
		("sequence-max-gap", boost::program_options::value(&OPT::sequenceMaxGap)->default_value(defaults.matchCfg.sequenceMaxGap), "max multiple of the median frame-number step before breaking an image sequence (0=never break on numeric jumps); a change of directory or name prefix always breaks it")
		("match-verification-feedback", boost::program_options::value(&OPT::matchVerificationFeedback)->default_value(defaults.matchCfg.verificationFeedback), "re-invest part of the matching budget in pairs suggested by geometrically verified matches")
		("release-descriptors", boost::program_options::value(&OPT::releaseDescriptors)->default_value(defaults.matchCfg.releaseDescriptors), "release descriptors after matching to save memory")
		("match-images-only", boost::program_options::value(&OPT::matchImagesOnly)->default_value(defaults.matchImagesOnly), "match pairs, filter the view graph, and save the scene without reconstruction")
		;

	boost::program_options::options_description dense("Dense matching options");
	dense.add_options()
		("dense-matcher", boost::program_options::value<std::string>(&OPT::strDenseMatcher)->default_value("none"), "in-process dense matcher: none, roma2, or flexglue")
		("roma2-model", boost::program_options::value<std::string>(&OPT::strROMA2Model), "directory with RoMa v2 ONNX graphs + manifest (default: $OPENMVS_ROMA2_MODEL_PATH or share/openMVS/roma2)")
		("flexglue-model", boost::program_options::value<std::string>(&OPT::strFlexGlueModel), "directory with FlexGlue ONNX graphs + manifest (default: $OPENMVS_FLEXGLUE_MODEL_PATH or share/openMVS/flexglue)")
		("dense-provider", boost::program_options::value<std::string>(&OPT::strDenseProvider)->default_value(defaults.denseCfg.provider), "ONNX Runtime execution provider: auto (CUDA > CoreML > DirectML > CPU), cuda, coreml, dml, or cpu")
		("dense-match", boost::program_options::value<bool>(&OPT::bDenseMatch)->default_value(defaults.denseCfg.useMatching), "one-pass dense pair matching: verify pairs from bidirectional warps, guide sparse matches, and fill remaining overlap densely")
		("dense-slots", boost::program_options::value(&OPT::nDenseSlots)->default_value(defaults.denseCfg.slotBudget), "images kept resident on the device while dense matching")
		("dense-min-confidence", boost::program_options::value(&OPT::fDenseMinConfidence)->default_value(defaults.denseCfg.minConfidence), "minimum warp-cell confidence for dense matching")
		("dense-min-overlap", boost::program_options::value(&OPT::fDenseMinOverlap)->default_value(defaults.denseCfg.minOverlap), "minimum inlier overlap fraction to accept a dense pair (0=accept all)")
		("dense-matches", boost::program_options::value(&OPT::nDenseMatches)->default_value(defaults.denseCfg.denseMatchesPerFrame), "target dense correspondences per full frame of uncovered overlap")
		("dense-min-covisibility", boost::program_options::value(&OPT::fDenseMinCovisibility)->default_value(defaults.denseCfg.minCovisibility), "minimum FlexGlue covisibility score to accept a pair (0=disabled)")
		;

	boost::program_options::options_description viewGraph("View graph filtering options");
	viewGraph.add_options()
		("filter-triplets", boost::program_options::value<bool>(&OPT::bFilterTriplets)->default_value(defaults.tripletFilterCfg.enabled), "score the view graph with the camera-triplet filter (Manam & Govindu, CVPR 2024)")
		("triplet-cut", boost::program_options::value<bool>(&OPT::bTripletCut)->default_value(defaults.tripletFilterCfg.cut), "remove pairs below the triplet-filter threshold (also splits symmetric two-face buildings)")
		("triplet-min-score", boost::program_options::value(&OPT::fTripletMinScore)->default_value(defaults.tripletFilterCfg.minScore), "minimum edge score m in [0,1] for the triplet filter")
		("triplet-second-face-score", boost::program_options::value(&OPT::fTripletSecondFaceScore)->default_value(defaults.tripletFilterCfg.secondFaceScore), "stricter minimum score for detecting and cutting symmetric second faces (with --triplet-cut)")
		("max-pair-direction-residual", boost::program_options::value(&OPT::fMaxPairDirectionResidual)->default_value(defaults.maxPairDirectionResidual), "contradiction-cut baseline direction residual threshold in degrees in [0,180] (0=rotation stage only)")
		("repair-contradicted-pairs", boost::program_options::value<bool>(&OPT::bRepairContradictedPairs)->default_value(defaults.repairContradictedPairs), "refit contradicted pairs at the averaged rotation when matches and centers agree")
		;

	boost::program_options::options_description solver("Solver, block and merge options");
	solver.add_options()
		("use-global-solver", boost::program_options::value<bool>(&OPT::bUseGlobalSolver)->default_value(defaults.useGlobalSolver), "reconstruct with the global solver instead of the hierarchical one")
		("max-views-per-cluster", boost::program_options::value(&OPT::maxViewsPerCluster)->default_value(defaults.clusterCfg.maxViewsPerCluster), "maximum images per cluster for hierarchical reconstruction (0=disable clustering)")
		("cluster-alignment", boost::program_options::value(&OPT::clusterAlignment)->default_value(defaults.globalAlignmentCfg.alignment), "sub-scene alignment method: 0=3D-3D similarity, 1=generalized-camera PnP")
		("max-link-direction-error", boost::program_options::value(&OPT::fMaxLinkDirectionError)->default_value(defaults.resectionCfg.maxLinkDirectionError), "resection direction check threshold in degrees in [0,90] (0=disabled)")
		;

	boost::program_options::options_description adjustment("Bundle adjustment options");
	adjustment.add_options()
		("refine-intrinsics", boost::program_options::value(&OPT::refineIntrinsics)->default_value(DefaultRefineIntrinsicsLevel(defaults.baIntrinsicFlags)), "intrinsics refinement level: 0=fixed, 1=focal+k1+k2, 2=focal+cx+cy+k1+k2+k3+p1+p2")
		("ba-dense-weight", boost::program_options::value(&OPT::baDenseWeight)->default_value(defaults.baConfig.denseObservationWeight), "relative loss weight for dense-keypoint residuals (<0=auto from robust sigmas, >=0=fixed)")
		("ba-max-obs", boost::program_options::value(&OPT::baMaxObservations)->default_value(defaults.baConfig.maxObservationsPerImage), "max observations per image in bundle adjustment (0=uncapped)")
		("ba-cap-min-obs", boost::program_options::value(&OPT::baCapMinObservations)->default_value(defaults.baConfig.minObservationsForCap), "scene observation count threshold above which --ba-max-obs activates (0=always)")
		("ba-pair-sigma", boost::program_options::value(&OPT::baPairSigma)->default_value(defaults.relativeRotationSigma), "relative-pose pair constraint rotation sigma in degrees, the baseline direction getting twice it (0=disabled)")
		;

	boost::program_options::options_description geo("Geo-alignment options");
	geo.add_options()
		("align-gps-threshold", boost::program_options::value<float>(&OPT::thAlignGPS)->default_value(defaults.thAlignGPS), "maximum distance in meters for aligning GPS positions to reconstruction poses (0 = disabled)")
		("gps-position-weight", boost::program_options::value(&OPT::gpsPositionWeight)->default_value(defaults.baConfig.gpsPositionWeight), "horizontal weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled; refused with panoramas)")
		("gps-position-weight-z", boost::program_options::value(&OPT::gpsPositionWeightZ)->default_value(defaults.baConfig.gpsPositionWeightZ), "vertical weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled; refused with panoramas)")
		;

	boost::program_options::options_description output("Output and export options");
	output.add_options()
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output scene file path")
		("extract-colors", boost::program_options::value<bool>(&OPT::bExtractColors)->default_value(defaults.extractColors), "extract colors for reconstructed points")
		("export-mvs", boost::program_options::value<std::string>(&OPT::strOutputFileNameMVS), "output MVS file path (optional)")
		("export-poses-csv", boost::program_options::value<std::string>(&OPT::strExportPosesCSV), "export camera poses to CSV file (optional; skipped by --match-images-only, which reconstructs no pose)")
		("export-pose-quality", boost::program_options::value<std::string>(&OPT::strExportPoseQuality), "estimate the pose covariance during the final bundle adjustment and export the per-image quality report to CSV file (optional)")
		("export-openmvg-dir", boost::program_options::value<std::string>(&OPT::strExportOpenMVGDir), "export OpenMVG features to directory (optional)")
		("export-pairs-csv", boost::program_options::value<std::string>(&OPT::strExportPairsCSV), "export image pairs to CSV file (written right after matching, before reconstruction) (optional)")
		("compare-mvs", boost::program_options::value<std::string>(&OPT::strCompareMVS), "compare reconstruction against ground-truth MVS file (optional)")
		("undistort-alpha", boost::program_options::value<float>(&OPT::undistortAlpha)->default_value(ExportMVSConfig().undistortAlpha), "alpha parameter for undistortion (0=zoomed in, 1=all pixels retained)")
		("undistort-extension", boost::program_options::value<std::string>(&OPT::strUndistortExt), "file extension/format of undistorted images written by the MVS export (e.g. .jpg, .png, .jxl); empty (default) exports the distorted cameras with the original images, undistorted when the MVS scene loads them")
		("spherical-export", boost::program_options::value<std::string>(&OPT::strSphericalExport)->default_value("native"), "MVS export of the spherical cameras: native (equirectangular cameras) or cubemap (rendered pinhole faces)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(input).add(features).add(matching).add(dense).add(viewGraph).add(solver).add(adjustment).add(geo).add(output);

	boost::program_options::options_description config_file_options;
	config_file_options.add(input).add(features).add(matching).add(dense).add(viewGraph).add(solver).add(adjustment).add(geo).add(output);

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
		OPT::matchMode > static_cast<int>(MatchConfig::RETRIEVAL)) {
		LOG("error: unknown match mode %d (accepted: -1, 0, 1, 2, 3, 4)", OPT::matchMode);
		return false;
	}
	if (OPT::matchMode == static_cast<int>(MatchConfig::SKIP) &&
		!(Util::getFileExt(OPT::strSource) == ".sfm" && File::isFile(MAKE_PATH_SAFE(OPT::strSource)))) {
		LOG("error: --match-mode -1 (SKIP) runs no matching, the source must be a saved scene (.sfm) that already holds matches (after ExtractKeyframes or a --match-images-only run)");
		return false;
	}
	if (OPT::matchMode == static_cast<int>(MatchConfig::SKIP) && OPT::strDenseMatcher != "none") {
		LOG("error: --dense-matcher %s with --match-mode -1 (SKIP): no matching runs, the dense matcher has nothing to do", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (FeatureTypeFromString(OPT::strDetectorType) == FeatureType::NONE) {
		LOG("error: unknown detector type '%s' (accepted: AKAZE, ORB, SIFT, SIFTGPU, in any letter case)", OPT::strDetectorType.c_str());
		return false;
	}
	if (OPT::nCameraType < -1 || OPT::nCameraType > 1) {
		LOG("error: unknown camera type %d (accepted: -1, 0, 1)", OPT::nCameraType);
		return false;
	}
	if (OPT::refineIntrinsics >= std::size(REFINE_INTRINSICS_FLAGS)) {
		LOG("error: unknown refine-intrinsics level %u (accepted: 0, 1, 2)", OPT::refineIntrinsics);
		return false;
	}
	if (OPT::clusterAlignment > GlobalAlignmentConfig::ALIGN_CAMERAS) {
		LOG("error: unknown cluster alignment %u (accepted: 0, 1)", OPT::clusterAlignment);
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
	Util::ensureValidPath(OPT::strCompareMVS);
	if (OPT::fTripletMinScore < 0.f || OPT::fTripletMinScore > 1.f) {
		LOG("error: --triplet-min-score is the paper's minimum edge score m, it must be in [0,1] (got %g)", OPT::fTripletMinScore);
		return false;
	}
	if (OPT::fTripletSecondFaceScore < 0.f || OPT::fTripletSecondFaceScore > 1.f) {
		LOG("error: --triplet-second-face-score must be in [0,1] (got %g)", OPT::fTripletSecondFaceScore);
		return false;
	}
	if (!(OPT::fMaxPairDirectionResidual >= 0.f && OPT::fMaxPairDirectionResidual <= 180.f)) {
		LOG("error: --max-pair-direction-residual is the translation stage's bar in degrees, it must be in [0,180] (got %g; 0 = no translation stage)", OPT::fMaxPairDirectionResidual);
		return false;
	}
	if (!(OPT::fMaxLinkDirectionError >= 0.f && OPT::fMaxLinkDirectionError <= 90.f)) {
		LOG("error: --max-link-direction-error is the resection's direction check bar in degrees, it must be in [0,90] (got %g; 0 = no direction check)", OPT::fMaxLinkDirectionError);
		return false;
	}
	if (!(OPT::fSphericalBottomBand >= 0.f && OPT::fSphericalBottomBand < 1.f)) {
		LOG("error: --spherical-bottom-band is a fraction of the image height, it must be in [0,1) (got %g; 0 = keep every feature)", OPT::fSphericalBottomBand);
		return false;
	}
	Util::ensureValidFolderPath(OPT::strROMA2Model);
	Util::ensureValidFolderPath(OPT::strFlexGlueModel);
	if (OPT::strDenseMatcher != "none" && OPT::strDenseMatcher != "roma2" && OPT::strDenseMatcher != "flexglue") {
		LOG("error: unknown dense matcher '%s' (accepted: none, roma2, flexglue)", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (OPT::strDenseMatcher != "flexglue" && !OPT::strFlexGlueModel.empty()) {
		LOG("error: --flexglue-model given with --dense-matcher %s (it belongs to --dense-matcher flexglue)", OPT::strDenseMatcher.c_str());
		return false;
	}
	// an option given on the command line (or the config file) that the chosen dense matcher does not read is
	// refused naming it: a defaulted option does not count as given
	const auto given = [](const char* name) { return OPT::vm.count(name) && !OPT::vm[name].defaulted(); };
	// the global solver neither clusters nor merges, and known poses take the known-poses solver before it is
	// reached: an option it ignores, given together with it, is refused naming it
	if (OPT::bUseGlobalSolver) {
		for (const char* name: {"max-views-per-cluster", "cluster-alignment"})
			if (given(name)) {
				LOG("error: --%s given with --use-global-solver 1 (the global solver never clusters or merges the images)", name);
				return false;
			}
		if (OPT::importPosesMode == static_cast<unsigned>(PoseImportMode::POSES_INTRINSICS) || OPT::importPosesMode == static_cast<unsigned>(PoseImportMode::POSES)) {
			LOG("error: --use-global-solver 1 with --import-poses-mode %u (known poses take the known-poses solver, never the global one)", OPT::importPosesMode);
			return false;
		}
	}
	if (OPT::strDenseMatcher == "flexglue" && given("roma2-model")) {
		LOG("error: --roma2-model given with --dense-matcher flexglue (it belongs to --dense-matcher roma2; the FlexGlue model is --flexglue-model)");
		return false;
	}
	if (OPT::strDenseMatcher != "flexglue" && given("dense-min-covisibility")) {
		LOG("error: --dense-min-covisibility given with --dense-matcher %s (it gates FlexGlue pairs, RoMa2 gives no covisibility score)", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (OPT::strDenseMatcher == "none") {
		for (const char* name: {"roma2-model", "dense-slots", "dense-min-confidence", "dense-min-overlap", "dense-matches", "dense-provider"})
			if (given(name)) {
				LOG("error: --%s given with --dense-matcher none (the dense matcher is off, nothing reads it)", name);
				return false;
			}
	}
	{
		MatchConfig matchCfg;
		matchCfg.mode = static_cast<MatchConfig::MatchMode>(OPT::matchMode);
		matchCfg.imageSequence = OPT::imageSequence;
		if (!matchCfg.IsImageSequence() && given("sequence-max-gap")) {
			LOG("error: --sequence-max-gap given without --image-sequence or --match-mode 2 (the images are not read as a sequence, nothing uses it)");
			return false;
		}
	}
	if (!(OPT::fDenseMinCovisibility >= 0.f && OPT::fDenseMinCovisibility <= 1.f)) {
		LOG("error: --dense-min-covisibility is a covisibility score, it must be in [0,1] (got %g)", OPT::fDenseMinCovisibility);
		return false;
	}
	if (OPT::strDenseMatcher != "none" && !DenseMatcher::IsAvailable()) {
		LOG("error: --dense-matcher %s needs a build with ONNX Runtime (-DOpenMVS_USE_ONNXRUNTIME=ON)", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (OPT::strDenseProvider != "auto" && OPT::strDenseProvider != "cuda" && OPT::strDenseProvider != "coreml" &&
		OPT::strDenseProvider != "dml" && OPT::strDenseProvider != "cpu") {
		LOG("error: unknown execution provider '%s' for --dense-provider (accepted: auto, cuda, coreml, dml, cpu)", OPT::strDenseProvider.c_str());
		return false;
	}
	if (OPT::fDenseMinConfidence < 0.f || OPT::fDenseMinConfidence > 1.f) {
		LOG("error: --dense-min-confidence is a confidence, it must be in [0,1] (got %g)", OPT::fDenseMinConfidence);
		return false;
	}
	if (OPT::fDenseMinOverlap < 0.f || OPT::fDenseMinOverlap > 1.f) {
		LOG("error: --dense-min-overlap is a fraction of the warp grid, it must be in [0,1] (got %g)", OPT::fDenseMinOverlap);
		return false;
	}
	if (OPT::nDenseSlots < 2) {
		LOG("error: --dense-slots must be at least the 2 slots a pair needs (got %u)", OPT::nDenseSlots);
		return false;
	}
	if (OPT::nDenseMatches < 1) {
		LOG("error: --dense-matches is the dense fill's draw per full frame of overlap and the frame the pair evidence is anchored to, it must be at least 1 (got %u)", OPT::nDenseMatches);
		return false;
	}
	if (!(OPT::baPairSigma >= 0.f)) {
		LOG("error: --ba-pair-sigma must be non-negative (got %g)", OPT::baPairSigma);
		return false;
	}
	if (OPT::strSphericalExport != "native" && OPT::strSphericalExport != "cubemap") {
		LOG("error: invalid --spherical-export '%s' (accepted: native, cubemap)", OPT::strSphericalExport.c_str());
		return false;
	}
	if (OPT::bDenseMatch && OPT::strDenseMatcher == "none") {
		// a requested-but-unavailable backend never silently degrades: the one
		// pass IS the matching round when it runs, and without --dense-matcher the enabled flag
		// Scene::MatchPairs gates on (enabled && (needsDescriptors || needsWarps)) is false regardless
		// of useMatching, so the descriptor round would quietly run the whole matching stage in its
		// place with nothing in the log naming ROMA2
		LOG("error: --dense-match needs --dense-matcher roma2 or flexglue (the one pass is the dense matcher's warp)");
		return false;
	}
	if (OPT::strDenseMatcher != "none" && !OPT::bDenseMatch && OPT::matchMode != static_cast<int>(MatchConfig::RETRIEVAL)) {
		// the match mode decides whether the scene needs describing (Scene::MatchPairs' own
		// wantsDescriptors), and --dense-match is the only other consumer of the model -- with
		// neither, --dense-matcher has nothing to load a model for. Deliberately not checked here: the
		// converse, --match-mode 4 without --dense-matcher -- a scene loaded from an .sfm may already carry global descriptors, and this parser cannot
		// see what the source will import, only the flags (GlobalDescriptors::Build fails loudly
		// and by name if it does not)
		LOG("error: --dense-matcher %s has nothing to do without --dense-match true or --match-mode 4 (RETRIEVAL)", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (OPT::bDenseMatch) {
		// --dense-match unconditionally needs the model to run the warp, unlike the descriptor-only
		// case above (--match-mode 4): whether that needs a model depends on
		// whether the scene already carries global descriptors, which this parser cannot see, so --
		// same precedent as the converse case in the check above -- it is deliberately left to
		// Scene::MatchPairs' own named error, which fires only when a model turns out to actually be
		// needed. This check just gives the hint before any feature extraction
		// runs, for the one case the parser can be sure about regardless of scene state.
		DenseMatchConfig denseCfg;
		denseCfg.matcher = OPT::strDenseMatcher;
		denseCfg.modelPath = OPT::strROMA2Model;
		denseCfg.flexglueModelPath = OPT::strFlexGlueModel;
		if (denseCfg.ResolveModelPath().empty()) {
			LOG("error: --dense-match needs a model (set %s, or install the model into the install prefix)", OPT::strDenseMatcher == "flexglue" ? "--flexglue-model or $OPENMVS_FLEXGLUE_MODEL_PATH" : "--roma2-model or $OPENMVS_ROMA2_MODEL_PATH");
			return false;
		}
	}

	// a saved scene is resumed from what it holds: say which of the given options it does not read
	if (Util::getFileExt(OPT::strSource) == ".sfm" && File::isFile(MAKE_PATH_SAFE(OPT::strSource))) {
		if (given("camera-type"))
			LOG("warning: --camera-type is ignored, the saved scene's cameras are kept");
		if (given("detector-type"))
			LOG("warning: --detector-type is ignored if the saved scene already holds features; its own detector then sets the matching thresholds");
		if (given("spherical-bottom-band"))
			LOG("warning: --spherical-bottom-band does not filter the features a saved scene already holds (it still keeps the dense matching out of the band)");
		for (const char* name: {"filter-triplets", "triplet-cut", "triplet-min-score", "triplet-second-face-score", "max-pair-direction-residual", "repair-contradicted-pairs"})
			if (given(name))
				LOG("warning: --%s is ignored if the saved scene's view graph is already filtered (a --match-images-only scene)", name);
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
	cfg.importCfg.cameraType = (CameraType)(OPT::nCameraType+1);
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
	cfg.featuresCfg.siftgpuDarknessAdaptivity = OPT::bSiftGPUDarknessAdaptivity;
	cfg.featuresCfg.sphericalBottomBand = OPT::fSphericalBottomBand;
	cfg.featuresCfg.importOpenMVGDir = OPT::strImportOpenMVGDir;
	cfg.featuresCfg.exportOpenMVGDir = OPT::strExportOpenMVGDir;
	cfg.matchCfg.DefaultsForFeatureType(cfg.featuresCfg.detectorType);
	cfg.matchCfg.mode = static_cast<MatchConfig::MatchMode>(OPT::matchMode);
	cfg.matchCfg.matchSequenceOverlap = OPT::matchSequenceOverlap;
	cfg.matchCfg.imageSequence = OPT::imageSequence;
	cfg.matchCfg.sequenceMaxGap = OPT::sequenceMaxGap;
	cfg.matchCfg.maxPairsPerImage = OPT::maxPairsPerImage;
	cfg.matchCfg.verificationFeedback = OPT::matchVerificationFeedback;
	cfg.matchCfg.releaseDescriptors = OPT::releaseDescriptors;
	cfg.denseCfg.enabled = OPT::strDenseMatcher != "none";
	cfg.denseCfg.modelPath = OPT::strROMA2Model;
	cfg.denseCfg.useMatching = OPT::bDenseMatch;
	cfg.denseCfg.minConfidence = OPT::fDenseMinConfidence;
	cfg.denseCfg.minOverlap = OPT::fDenseMinOverlap;
	cfg.denseCfg.denseMatchesPerFrame = OPT::nDenseMatches;
	cfg.matchCfg.weightingCfg.denseMatchesPerFrame = OPT::nDenseMatches; // the pair evidence is anchored to the same frame target the matcher draws
	cfg.denseCfg.slotBudget = OPT::nDenseSlots;
	cfg.denseCfg.provider = OPT::strDenseProvider;
	if (cfg.denseCfg.enabled)
		cfg.denseCfg.matcher = OPT::strDenseMatcher;
	cfg.denseCfg.flexglueModelPath = OPT::strFlexGlueModel;
	cfg.denseCfg.minCovisibility = OPT::fDenseMinCovisibility;
	#ifdef _USE_CUDA
	cfg.matchCfg.useCUDA = cfg.featuresCfg.useCUDA = OPT::bSiftGPUCUDA && !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	cfg.denseCfg.useGPU = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	#endif
	cfg.matchImagesOnly = OPT::matchImagesOnly;
	// written by Scene::Reconstruct() right after pair matching (see ReconstructionConfig)
	cfg.exportPairsCSV = OPT::strExportPairsCSV.empty() ? String() : MAKE_PATH_SAFE(OPT::strExportPairsCSV);
	// applied by Scene::Reconstruct() right after those exports, so they still list every matched pair
	cfg.tripletFilterCfg.enabled = OPT::bFilterTriplets;
	cfg.tripletFilterCfg.cut = OPT::bTripletCut;
	cfg.tripletFilterCfg.minScore = OPT::fTripletMinScore;
	cfg.tripletFilterCfg.secondFaceScore = OPT::fTripletSecondFaceScore;
	cfg.maxPairDirectionResidual = OPT::fMaxPairDirectionResidual;
	cfg.repairContradictedPairs = OPT::bRepairContradictedPairs;
	// every resection copies cfg.resectionCfg: the sub-scenes' and the whole-scene pass after the merge
	cfg.resectionCfg.maxLinkDirectionError = OPT::fMaxLinkDirectionError;
	cfg.viewgraphCfg.maxTwoViewError = 0; // disable pair filtering after ViewGraph calibration
	cfg.useGlobalSolver = OPT::bUseGlobalSolver;
	cfg.thAlignGPS = OPT::thAlignGPS;
	cfg.baConfig.gpsPositionWeight = OPT::gpsPositionWeight;
	cfg.baConfig.gpsPositionWeightZ = OPT::gpsPositionWeightZ;
	// bundle adjustment's own weight, an override on the per-solve measurement -- NOT the view
	// graph's dense evidence (PairsWeightingConfig::denseFrameInliers), a different quantity with
	// its own number
	cfg.baConfig.denseObservationWeight = OPT::baDenseWeight;
	// one budget over every keypoint kind, and the size from which a solve is subject to it; every
	// derived BA configuration copies cfg.baConfig, so both reach all of them
	cfg.baConfig.maxObservationsPerImage = OPT::baMaxObservations;
	cfg.baConfig.minObservationsForCap = OPT::baCapMinObservations;
	// the rotation sigma names the pair constraints; the baseline direction follows at
	// ReconstructionConfig::relativeTranslationSigmaFactor times it
	cfg.relativeRotationSigma = MAXF(OPT::baPairSigma, 0.f);
	cfg.baIntrinsicFlags = REFINE_INTRINSICS_FLAGS[OPT::refineIntrinsics];
	cfg.estimatePoseUncertainty = !OPT::strExportPoseQuality.empty();
	cfg.extractColors = OPT::bExtractColors;
	cfg.clusterCfg.SetMaxViews(OPT::maxViewsPerCluster);
	cfg.globalAlignmentCfg.alignment = OPT::clusterAlignment;

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
	// Export camera poses to CSV file; a match-images-only run leaves the scene uncalibrated, no image
	// has a pose and there is nothing to export
	if (!OPT::strExportPosesCSV.empty() && !scene.status.nState.isSet(Scene::Status::STATE::CALIBRATED))
		VERBOSE("--export-poses-csv skipped: no pose was reconstructed (--match-images-only)");
	else if (!OPT::strExportPosesCSV.empty() && !ExportPosesCSV(MAKE_PATH_SAFE(OPT::strExportPosesCSV), scene.images)) {
		VERBOSE("error: failed to export camera poses to CSV file %s", OPT::strExportPosesCSV.c_str());
		return EXIT_FAILURE;
	}
	// Export per-image pose quality report to CSV file; this is an optional diagnostic, so a
	// failure to produce it (e.g. a rank-deficient covariance yielding no rows) must not abort
	// the run and lose the primary outputs (the scene and the MVS export below)
	if (!OPT::strExportPoseQuality.empty() && !ExportPoseUncertaintyCSV(MAKE_PATH_SAFE(OPT::strExportPoseQuality), scene))
		VERBOSE("warning: failed to export pose quality report to CSV file %s", OPT::strExportPoseQuality.c_str());
	// note: --export-pairs-csv is written by Scene::Reconstruct()
	// right after pair matching (see ReconstructionConfig::exportPairsCSV),
	// not here, so they describe the matched scene rather than whatever reconstruction kept
	// Export MVS scene
	if (!OPT::strOutputFileNameMVS.empty()) {
		SFM::ExportMVSConfig exportCfg;
		if (!OPT::strUndistortExt.empty()) {
			// write undistorted image files instead of the Brown cameras of the original ones
			exportCfg.undistortImageDir = MAKE_PATH("undistorted");
			exportCfg.undistortAlpha    = OPT::undistortAlpha;
			exportCfg.extension         = OPT::strUndistortExt;
		}
		exportCfg.sphericalExport = OPT::strSphericalExport == "cubemap" ? SFM::ExportMVSConfig::Spherical::CUBEMAP : SFM::ExportMVSConfig::Spherical::NATIVE;
		if (!ExportMVS(MAKE_PATH_SAFE(OPT::strOutputFileNameMVS), scene, exportCfg)) {
			VERBOSE("error: failed to export MVS file to %s", OPT::strOutputFileNameMVS.c_str());
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
