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
unsigned maxPairsPerImage;
bool matchVerificationFeedback;
bool releaseDescriptors;
bool matchImagesOnly;
bool bROMA2;
String strROMA2Model;
String strDenseMatcher;
String strFlexGlueModel;
float fDenseMinCovisibility;
bool bROMA2Match;
unsigned nROMA2Slots;
float fROMA2MinConfidence;
float fROMA2MinOverlap;
unsigned nROMA2DenseMatches;
bool bFilterTriplets;
bool bTripletCut;
float fTripletMinScore;
float fTripletSecondFaceScore;
float fMaxPairDirectionResidual;
bool bRepairContradictedPairs;
float fMaxLinkDirectionError;
String strROMA2Provider;
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

	boost::program_options::options_description config("Reconstruction options");
	config.add_options()
		("source,s", boost::program_options::value<std::string>(&OPT::strSource), "source folder or semicolon-separated list of images")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output scene file path")
		("export-mvs", boost::program_options::value<std::string>(&OPT::strOutputFileNameMVS), "output MVS file path (optional)")
		("detector-type,t", boost::program_options::value<std::string>(&OPT::strDetectorType)->default_value(FeatureTypeToString(FeatureType::DEFAULT)), "feature detector type: AKAZE, ORB, SIFT or SIFTGPU (any letter case)")
		("import-poses-file", boost::program_options::value<std::string>(&OPT::strImportPosesFile)->default_value("poses.csv"), "import camera poses from file: .csv (OpenMVS pose CSV) or .json (frames.json)")
		("export-poses-csv", boost::program_options::value<std::string>(&OPT::strExportPosesCSV), "export camera poses to CSV file (optional; skipped by --match-images-only, which reconstructs no pose)")
		("export-pose-quality", boost::program_options::value<std::string>(&OPT::strExportPoseQuality), "estimate the pose covariance during the final bundle adjustment and export the per-image quality report to CSV file (optional)")
		("import-poses-mode", boost::program_options::value(&OPT::importPosesMode)->default_value((unsigned)defaults.importCfg.importPosesMode), "mode for importing camera poses: 0=none, 1=poses+intrinsics, 2=poses only, 3=positions only")
		("known-poses-convention", boost::program_options::value<std::string>(&OPT::strKnownPosesConvention), "camera-axes convention of the poses in a frames.json: auto|arkit|opencv (default: auto, detect it from the poses)")
		("import-openmvg-dir", boost::program_options::value<std::string>(&OPT::strImportOpenMVGDir), "import OpenMVG features from directory (optional)")
		("export-openmvg-dir", boost::program_options::value<std::string>(&OPT::strExportOpenMVGDir), "export OpenMVG features to directory (optional)")
		("export-pairs-csv", boost::program_options::value<std::string>(&OPT::strExportPairsCSV), "export image pairs to CSV file (written right after matching, before reconstruction) (optional)")
		("compare-mvs", boost::program_options::value<std::string>(&OPT::strCompareMVS), "compare reconstruction against ground-truth MVS file (optional)")
		("filter-triplets", boost::program_options::value<bool>(&OPT::bFilterTriplets)->default_value(defaults.tripletFilterCfg.enabled), "score the matched view graph with the camera-triplet filter (Manam & Govindu, CVPR 2024): a pair is scored by how its coverage-discounted inlier count compares with the strongest pair of every triangle it belongs to; by default nothing is removed, the scores weigh the contradiction cut (see --max-pair-direction-residual) and the reconstruction seeds in the largest piece the paper's threshold leaves; with --triplet-cut every pair below the threshold goes; with the filter off the contradiction cut still runs, unweighted")
		("triplet-cut", boost::program_options::value<bool>(&OPT::bTripletCut)->default_value(defaults.tripletFilterCfg.cut), "camera-triplet filter: the cutting rule -- every pair scoring below the threshold goes, the smaller face of a symmetric building is cut off (--triplet-second-face-score), otherwise below a threshold that shatters the graph the strictest one joining the pieces applies; unfolds a symmetric building, halves the registrations of an interior")
		("triplet-min-score", boost::program_options::value(&OPT::fTripletMinScore)->default_value(defaults.tripletFilterCfg.minScore), "camera-triplet filter: the paper's minimum edge score m in [0,1], from which the threshold tau = m(1-r)+r follows, r the maximum-degree ratio of the scored graph (0.3 medium and small ambiguous sets, 0.6 generic, 0.9 highly ambiguous)")
		("triplet-second-face-score", boost::program_options::value(&OPT::fTripletSecondFaceScore)->default_value(defaults.tripletFilterCfg.secondFaceScore), "camera-triplet filter, with --triplet-cut: a stricter minimum score whose threshold names the two faces of a symmetric building (a majority piece and a second one of at least a third of it); the smaller face is then cut off; at or below --triplet-min-score the faces are not looked for")
		("max-pair-direction-residual", boost::program_options::value(&OPT::fMaxPairDirectionResidual)->default_value(defaults.maxPairDirectionResidual), "the view graph's contradiction cut, after the camera-triplet filter: every pair whose rotation the global rotation averaging of the pairs contradicts goes; its translation stage also removes every pair whose baseline direction is more than this many degrees from the camera centres the pairs place, in [0,180]; 0 = no translation stage, the rotation stage alone")
		("repair-contradicted-pairs", boost::program_options::value<bool>(&OPT::bRepairContradictedPairs)->default_value(defaults.repairContradictedPairs), "repair instead of remove, in the contradiction cut (does not act with --max-pair-direction-residual 0): a pair the cut removes comes back with a relative pose refitted at the averaged rotation when at least 80 percent of its matches agree with that pose, the camera centres the cut places confirm its baseline direction within 20 degrees, and its two images are the same spot or a third spot holds a pair with each")
		("max-link-direction-error", boost::program_options::value(&OPT::fMaxLinkDirectionError)->default_value(defaults.resectionCfg.maxLinkDirectionError), "the direction check of every resection: a resected pose is refused when the verified pairs to registered images whose baseline direction it contradicts outweigh the ones that agree; each pair is held to this many degrees as seen from the structure, atan(tan(this) / a) with a the larger of the pair's ray angle and the model's parallax of the two cameras, a pair both angles put under 2 degrees agrees, one that only one of them puts that close abstains, and a pair of unknown ray angle is held to the model's parallax alone; in [0,90] (past 90 degrees the tangent turns negative and every pair would contradict); 0 = no direction check at all (the inlier-share and rotation checks stay)")
		("max-features-per-cell", boost::program_options::value(&OPT::nMaxFeaturesPerCell)->default_value(defaults.featuresCfg.maxFeaturesPerCell), "maximum features per grid cell (3x3 grid)")
		("min-features-per-cell", boost::program_options::value(&OPT::nMinFeaturesPerCell)->default_value(defaults.featuresCfg.minFeaturesPerCell), "minimum features per cell before adjusting sensitivity")
		("siftgpu-darkness-adaptivity", boost::program_options::value(&OPT::bSiftGPUDarknessAdaptivity)->default_value(defaults.featuresCfg.siftgpuDarknessAdaptivity), "SiftGPU: lower the DoG threshold and curvature floor in dark areas")
		("spherical-bottom-band", boost::program_options::value(&OPT::fSphericalBottomBand)->default_value(defaults.featuresCfg.sphericalBottomBand), "spherical images: the fraction of the image height, counted from the bottom, whose features are discarded (the operator and the tripod are there), in [0,1); 0 = keep every feature; perspective images are not affected")
		("siftgpu-cuda", boost::program_options::value(&OPT::bSiftGPUCUDA)->default_value(defaults.featuresCfg.useCUDA), "SiftGPU feature extraction and matching backend: 1 = CUDA (if available), 0 = OpenGL/GLSL")
		("match-mode", boost::program_options::value(&OPT::matchMode)->default_value((int)defaults.matchCfg.mode), "match mode: -1=SKIP (no matching: only for a saved .sfm scene that already holds matches),0=EXHAUSTIVE,1=VOCABULARY,2=SEQUENTIAL,3=KNOWN_POSES,4=RETRIEVAL")
		("match-sequence-overlap", boost::program_options::value(&OPT::matchSequenceOverlap)->default_value(defaults.matchCfg.matchSequenceOverlap), "consecutive images each image is matched with: the whole candidate set of the sequential mode, and a prior added to the vocabulary and retrieval candidates (0 = no prior)")
		("vocab-max-pairs", boost::program_options::value(&OPT::maxPairsPerImage)->default_value(defaults.matchCfg.maxPairsPerImage), "target pairs per image for vocabulary and pose-guided matching")
		("match-verification-feedback", boost::program_options::value(&OPT::matchVerificationFeedback)->default_value(defaults.matchCfg.verificationFeedback), "hold back part of the matching budget and re-invest it in pairs suggested by the geometrically verified matches (vocabulary and pose-guided matching)")
		("release-descriptors", boost::program_options::value(&OPT::releaseDescriptors)->default_value(defaults.matchCfg.releaseDescriptors), "release descriptors after matching to save memory")
		("match-images-only", boost::program_options::value(&OPT::matchImagesOnly)->default_value(defaults.matchImagesOnly), "match only the image pairs and save the scene without reconstruction")
		("roma2", boost::program_options::value<bool>(&OPT::bROMA2)->default_value(defaults.roma2Cfg.enabled), "enable the in-process RoMa v2 model: DINOv3/GeM pair retrieval and dense matching (needs an exported model, see --roma2-model)")
		("roma2-model", boost::program_options::value<std::string>(&OPT::strROMA2Model), "directory with the RoMa v2 ONNX graphs + manifest (default: $OPENMVS_ROMA2_MODEL_PATH, else the installed share/openMVS/roma2)")
		("roma2-match", boost::program_options::value<bool>(&OPT::bROMA2Match)->default_value(defaults.roma2Cfg.useMatching), "one-pass dense pair matching: judge every candidate pair on its bidirectional warp alone, then guide its sparse matching and fill the rest of the overlap densely (a rejected pair is dropped, never descriptor-matched)")
		("roma2-slots", boost::program_options::value(&OPT::nROMA2Slots)->default_value(defaults.roma2Cfg.slotBudget), "images kept resident on the device while dense matching (12.5 MB each at base)")
		("roma2-min-confidence", boost::program_options::value(&OPT::fROMA2MinConfidence)->default_value(defaults.roma2Cfg.minConfidence), "dense matching: confidence at which a warp cell takes part in the verdict, the keypoint tracking and the dense fill")
		("roma2-min-overlap", boost::program_options::value(&OPT::fROMA2MinOverlap)->default_value(defaults.roma2Cfg.minOverlap), "dense matching: the verdict, as the smaller of the two inlier areas one fitted geometry explains -- of the first image's confident cells and of the second's (0.10 is about 0.15-0.17 of true overlap, 0.15 about 0.25; 0 admits every pair)")
		("roma2-dense-matches", boost::program_options::value(&OPT::nROMA2DenseMatches)->default_value(defaults.roma2Cfg.denseMatchesPerFrame), "dense matching: correspondences the dense fill adds per FULL FRAME of overlap; a pair draws that density over the part of its overlap its guided sparse matches did not already cover, capped by the smaller of the two inlier areas the verdict measured, so the dense keypoint density is bounded in both images; each costs a keypoint in both images plus a track")
		("dense-matcher", boost::program_options::value<std::string>(&OPT::strDenseMatcher)->default_value(defaults.roma2Cfg.matcher), "the dense matcher behind --roma2 true (retrieval descriptors and --roma2-match warps): roma2 or flexglue (flexglue needs its model, see --flexglue-model; the --roma2-* matching options then apply to its warps)")
		("flexglue-model", boost::program_options::value<std::string>(&OPT::strFlexGlueModel), "directory of the FlexGlue model bundle: flexglue_<setting>.json and the two graphs it names (default: $OPENMVS_FLEXGLUE_MODEL_PATH, else the install prefix's share/openMVS/flexglue, as `--target flexglue-model` fetches it)")
		("dense-min-covisibility", boost::program_options::value(&OPT::fDenseMinCovisibility)->default_value(defaults.roma2Cfg.minCovisibility), "gate before the pair's verdict: a pair whose dense matcher gives a covisibility score (FlexGlue; RoMa2 gives none and is never gated) below this is dropped (0 = off)")
		("roma2-provider", boost::program_options::value<std::string>(&OPT::strROMA2Provider)->default_value(defaults.roma2Cfg.provider), "ONNX Runtime execution provider: auto (CUDA > CoreML > DirectML > CPU), cuda, coreml, dml or cpu")
		("camera-type", boost::program_options::value(&OPT::nCameraType)->default_value((int)defaults.importCfg.cameraType - 1), "camera model type: -1-from the image metadata (an equirectangular projection tag gives spherical, else pinhole), 0-pinhole, 1-spherical (equirectangular, width = 2 x height)")
		("default-focal-ratio", boost::program_options::value(&OPT::defaultFocalRatio)->default_value(defaults.importCfg.defaultFocalRatio), "focal-length is set to ratio * max(width,height) for images with unknown focal-length")
		("focal-length,f", boost::program_options::value(&OPT::focalLength)->default_value(defaults.importCfg.focalLength), "force focal-length (in pixels) for specified images (0 = disabled)")
		("k1", boost::program_options::value(&OPT::k1)->default_value(defaults.importCfg.k1), "force k1 distortion coefficient for specified images (0 = not used)")
		("k2", boost::program_options::value(&OPT::k2)->default_value(defaults.importCfg.k2), "force k2 distortion coefficient for specified images (0 = not used)")
		("image-indices", boost::program_options::value<std::string>(&OPT::strImageIndices), "image indices to apply forced parameters (e.g., '0 5-10 15', empty = all images)")
		("max-views-per-cluster", boost::program_options::value(&OPT::maxViewsPerCluster)->default_value(defaults.clusterCfg.maxViewsPerCluster), "maximum images per cluster for hierarchical reconstruction; clusters aim at two thirds of it (0 = disable clustering)")
		("cluster-alignment", boost::program_options::value(&OPT::clusterAlignment)->default_value(defaults.globalAlignmentCfg.alignment), "how sub-scenes are aligned before merging: 0 = similarity from 3D-3D track correspondences, 1 = generalized-camera PnP of one sub-scene's cameras against the other's tracks")
		("use-global-solver", boost::program_options::value<bool>(&OPT::bUseGlobalSolver)->default_value(defaults.useGlobalSolver), "reconstruct with the global solver (global rotation and position averaging) instead of the hierarchical one")
		("extract-colors", boost::program_options::value<bool>(&OPT::bExtractColors)->default_value(defaults.extractColors), "extract colors for reconstructed points")
		("undistort-alpha", boost::program_options::value<float>(&OPT::undistortAlpha)->default_value(ExportMVSConfig().undistortAlpha), "alpha parameter for undistortion (0=zoomed in, 1=all pixels retained)")
		("undistort-extension", boost::program_options::value<std::string>(&OPT::strUndistortExt)->default_value(ExportMVSConfig().extension), "file extension/format for the exported undistorted images (e.g. .jpg, .png, .jxl)")
		("align-gps-threshold", boost::program_options::value<float>(&OPT::thAlignGPS)->default_value(defaults.thAlignGPS), "maximum distance in meters for aligning GPS positions to reconstruction poses (0 = disabled)")
		("gps-position-weight", boost::program_options::value(&OPT::gpsPositionWeight)->default_value(defaults.baConfig.gpsPositionWeight), "horizontal weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled; refused with panoramas)")
		("gps-position-weight-z", boost::program_options::value(&OPT::gpsPositionWeightZ)->default_value(defaults.baConfig.gpsPositionWeightZ), "vertical weight of the GPS position priors used to refine the geo-aligned reconstruction (0 = disabled; refused with panoramas)")
		("ba-dense-weight", boost::program_options::value(&OPT::baDenseWeight)->default_value(defaults.baConfig.denseObservationWeight), "bundle adjustment: loss weight of a reprojection residual on a dense (warp-sampled) keypoint, relative to the 1.0 a described one carries; negative (default) measures it as (sigma described/sigma dense)^2 on the scene each solve is about to fit and falls back to 0.25 on a scene too small to give either sigma; any value >= 0 pins it as given, so use [0,1] (1 = no down-weighting)")
		("ba-max-obs", boost::program_options::value(&OPT::baMaxObservations)->default_value(defaults.baConfig.maxObservationsPerImage), "bundle adjustment: the most observations one image contributes to a solve, of any keypoint kind (0 = no cap); the budget goes to the described keypoints first and the warp-sampled ones fill what is left, each kind spread evenly over the image frame, the longer tracks before the shorter ones and the remaining ties drawn again for every solve; it acts only on a scene holding at least --ba-cap-min-obs observations")
		("ba-cap-min-obs", boost::program_options::value(&OPT::baCapMinObservations)->default_value(defaults.baConfig.minObservationsForCap), "bundle adjustment: observations the scene must hold before the per-image cap above applies to any solve (0 = always); a scene small enough to be solved whole is solved whole; the default is high enough that the cap acts only on the very largest scenes")
		("ba-pair-sigma", boost::program_options::value(&OPT::baPairSigma)->default_value(defaults.relativeRotationSigma), "bundle adjustment: standard deviation in degrees by which the model's relative rotation may differ from what a verified pair measured, the baseline direction being allowed twice as much; the pairs then hold the joints the tracks are too few to hold (0 = disabled, the solve fits reprojections alone)")
		("refine-intrinsics", boost::program_options::value(&OPT::refineIntrinsics)->default_value(DefaultRefineIntrinsicsLevel(defaults.baIntrinsicFlags)), "bundle adjustment intrinsics refinement: 0=fixed intrinsics, 1=focal length and the first two radial terms (k1, k2), 2=those plus the third radial term, the principal point and the tangential terms")
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
		OPT::matchMode > static_cast<int>(MatchConfig::RETRIEVAL)) {
		LOG("error: unknown match mode %d (accepted: -1, 0, 1, 2, 3, 4)", OPT::matchMode);
		return false;
	}
	if (OPT::matchMode == static_cast<int>(MatchConfig::SKIP) &&
		!(Util::getFileExt(OPT::strSource) == ".sfm" && File::isFile(MAKE_PATH_SAFE(OPT::strSource)))) {
		LOG("error: --match-mode -1 (SKIP) runs no matching, the source must be a saved scene (.sfm) that already holds matches (after ExtractKeyframes or a --match-images-only run)");
		return false;
	}
	if (OPT::matchMode == static_cast<int>(MatchConfig::SKIP) && OPT::bROMA2) {
		LOG("error: --roma2 true with --match-mode -1 (SKIP): no matching runs, the dense matcher has nothing to do");
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
	if (OPT::strDenseMatcher != "roma2" && OPT::strDenseMatcher != "flexglue") {
		LOG("error: unknown dense matcher '%s' (accepted: roma2, flexglue)", OPT::strDenseMatcher.c_str());
		return false;
	}
	if (OPT::strDenseMatcher == "flexglue" && !OPT::bROMA2) {
		LOG("error: --dense-matcher flexglue selects the model behind --roma2 true, it does nothing without it");
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
		LOG("error: --dense-min-covisibility given with %s (it gates FlexGlue pairs, RoMa2 gives no covisibility score)", OPT::bROMA2 ? "--dense-matcher roma2" : "--roma2 false");
		return false;
	}
	if (!OPT::bROMA2) {
		for (const char* name: {"roma2-model", "roma2-slots", "roma2-min-confidence", "roma2-min-overlap", "roma2-dense-matches", "roma2-provider"})
			if (given(name)) {
				LOG("error: --%s given with --roma2 false (the dense matcher is off, nothing reads it)", name);
				return false;
			}
	}
	if (!(OPT::fDenseMinCovisibility >= 0.f && OPT::fDenseMinCovisibility <= 1.f)) {
		LOG("error: --dense-min-covisibility is a covisibility score, it must be in [0,1] (got %g)", OPT::fDenseMinCovisibility);
		return false;
	}
	if (OPT::bROMA2 && !DenseMatcher::IsAvailable()) {
		LOG("error: --roma2 needs a build with ONNX Runtime (-DOpenMVS_USE_ONNXRUNTIME=ON)");
		return false;
	}
	if (OPT::strROMA2Provider != "auto" && OPT::strROMA2Provider != "cuda" && OPT::strROMA2Provider != "coreml" &&
		OPT::strROMA2Provider != "dml" && OPT::strROMA2Provider != "cpu") {
		LOG("error: unknown ROMA2 execution provider '%s' (accepted: auto, cuda, coreml, dml, cpu)", OPT::strROMA2Provider.c_str());
		return false;
	}
	if (OPT::fROMA2MinConfidence < 0.f || OPT::fROMA2MinConfidence > 1.f) {
		LOG("error: --roma2-min-confidence is a confidence, it must be in [0,1] (got %g)", OPT::fROMA2MinConfidence);
		return false;
	}
	if (OPT::fROMA2MinOverlap < 0.f || OPT::fROMA2MinOverlap > 1.f) {
		LOG("error: --roma2-min-overlap is a fraction of the warp grid, it must be in [0,1] (got %g)", OPT::fROMA2MinOverlap);
		return false;
	}
	if (OPT::nROMA2Slots < 2) {
		LOG("error: --roma2-slots must be at least the 2 slots a pair needs (got %u)", OPT::nROMA2Slots);
		return false;
	}
	if (OPT::nROMA2DenseMatches < 1) {
		LOG("error: --roma2-dense-matches is the dense fill's draw per full frame of overlap and the frame the pair evidence is anchored to, it must be at least 1 (got %u)", OPT::nROMA2DenseMatches);
		return false;
	}
	if (OPT::bROMA2Match && !OPT::bROMA2) {
		// a requested-but-unavailable backend never silently degrades: the one
		// pass IS the matching round when it runs, and without --roma2 the enabled flag
		// Scene::MatchPairs gates on (enabled && (needsDescriptors || needsWarps)) is false regardless
		// of useMatching, so the descriptor round would quietly run the whole matching stage in its
		// place with nothing in the log naming ROMA2
		LOG("error: --roma2-match needs --roma2 true (the one pass is the ROMAv2 warp)");
		return false;
	}
	if (OPT::bROMA2 && !OPT::bROMA2Match && OPT::matchMode != static_cast<int>(MatchConfig::RETRIEVAL)) {
		// the match mode decides whether the scene needs describing (Scene::MatchPairs' own
		// wantsDescriptors), and --roma2-match is the only other consumer of the model -- with
		// neither, --roma2 has nothing to load a model for. Deliberately not checked here: the
		// converse, --match-mode 4 without --roma2 -- a scene loaded from an .sfm may already carry global descriptors, and this parser cannot
		// see what the source will import, only the flags (GlobalDescriptors::Build fails loudly
		// and by name if it does not)
		LOG("error: --roma2 has nothing to do without --roma2-match true or --match-mode 4 (RETRIEVAL)");
		return false;
	}
	if (OPT::bROMA2Match) {
		// --roma2-match unconditionally needs the model to run the warp, unlike the descriptor-only
		// case above (--match-mode 4): whether that needs a model depends on
		// whether the scene already carries global descriptors, which this parser cannot see, so --
		// same precedent as the converse case in the check above -- it is deliberately left to
		// Scene::MatchPairs' own named error, which fires only when a model turns out to actually be
		// needed. This check just gives the hint before any feature extraction
		// runs, for the one case the parser can be sure about regardless of scene state.
		ROMA2Config roma2Cfg;
		roma2Cfg.matcher = OPT::strDenseMatcher;
		roma2Cfg.modelPath = OPT::strROMA2Model;
		roma2Cfg.flexglueModelPath = OPT::strFlexGlueModel;
		if (roma2Cfg.ResolveModelPath().empty()) {
			LOG("error: --roma2-match needs a model (set %s, or install the model into the install prefix)", OPT::strDenseMatcher == "flexglue" ? "--flexglue-model or $OPENMVS_FLEXGLUE_MODEL_PATH" : "--roma2-model or $OPENMVS_ROMA2_MODEL_PATH");
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
			LOG("warning: --spherical-bottom-band is ignored if the saved scene already holds features");
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
	cfg.matchCfg.maxPairsPerImage = OPT::maxPairsPerImage;
	cfg.matchCfg.verificationFeedback = OPT::matchVerificationFeedback;
	cfg.matchCfg.releaseDescriptors = OPT::releaseDescriptors;
	cfg.roma2Cfg.enabled = OPT::bROMA2;
	cfg.roma2Cfg.modelPath = OPT::strROMA2Model;
	cfg.roma2Cfg.useMatching = OPT::bROMA2Match;
	cfg.roma2Cfg.minConfidence = OPT::fROMA2MinConfidence;
	cfg.roma2Cfg.minOverlap = OPT::fROMA2MinOverlap;
	cfg.roma2Cfg.denseMatchesPerFrame = OPT::nROMA2DenseMatches;
	cfg.matchCfg.weightingCfg.denseMatchesPerFrame = OPT::nROMA2DenseMatches; // the pair evidence is anchored to the same frame target the matcher draws
	cfg.roma2Cfg.slotBudget = OPT::nROMA2Slots;
	cfg.roma2Cfg.provider = OPT::strROMA2Provider;
	cfg.roma2Cfg.matcher = OPT::strDenseMatcher;
	cfg.roma2Cfg.flexglueModelPath = OPT::strFlexGlueModel;
	cfg.roma2Cfg.minCovisibility = OPT::fDenseMinCovisibility;
	#ifdef _USE_CUDA
	cfg.matchCfg.useCUDA = cfg.featuresCfg.useCUDA = OPT::bSiftGPUCUDA && !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	cfg.roma2Cfg.useGPU = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
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
	else if (!OPT::strExportPosesCSV.empty() && !ExportPosesCSV(OPT::strExportPosesCSV, scene.images)) {
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
