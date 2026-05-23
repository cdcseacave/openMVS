/*
 * ExtractKeyframes.cpp
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

#include "../../libs/SFM/Common.h"
#include "../../libs/SFM.h"
#include <boost/program_options.hpp>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("ExtractKeyframes")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strOutputDirectory;
String strDetectorType;
float fOverlapThreshold;
float fFocalLength;
float fPPOffsetX;
float fPPOffsetY;
unsigned nCameraType;
unsigned nRefineCalibration;
unsigned nBlurSize;
unsigned nMaxFeaturesPerCell;
unsigned nMinFeaturesPerCell;
unsigned nCubemapFaces;
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

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Keyframe extraction options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input video file path")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output scene file path")
		("output-directory,d", boost::program_options::value<std::string>(&OPT::strOutputDirectory)->default_value("keyframes"), "output directory for keyframe images")
		("detector-type,t", boost::program_options::value<std::string>(&OPT::strDetectorType)->default_value(FeatureTypeToString(FeatureType::DEFAULT)), "feature detector type: AKAZE, ORB, SIFT or SIFTGPU")
		("overlap-threshold", boost::program_options::value(&OPT::fOverlapThreshold)->default_value(0.85f), "minimum overlap threshold between consecutive keyframes (0.0-1.0)")
		("focal-length,f", boost::program_options::value(&OPT::fFocalLength)->default_value(0.f), "known focal length in pixels (<=0 for auto-calibration from fundamental matrices)")
		("pp-offset-x", boost::program_options::value(&OPT::fPPOffsetX)->default_value(0.f), "principal point X offset from image center in pixels")
		("pp-offset-y", boost::program_options::value(&OPT::fPPOffsetY)->default_value(0.f), "principal point Y offset from image center in pixels")
		("camera-type", boost::program_options::value(&OPT::nCameraType)->default_value(0), "camera model type: 0-pinhole, 1-spherical")
		("refine-calibration", boost::program_options::value(&OPT::nRefineCalibration)->default_value(3), "enable intrinsic refinement (focal & distortion) during matching (0=disabled, 1=two-view, 2=three-view, 3=view-graph)")
		("blur-size", boost::program_options::value(&OPT::nBlurSize)->default_value(0), "Gaussian blur kernel size applied to images used for optical flow (0 = disabled)")
		("max-features-per-cell", boost::program_options::value(&OPT::nMaxFeaturesPerCell)->default_value(3000), "maximum features per grid cell (3x3 grid)")
		("min-features-per-cell", boost::program_options::value(&OPT::nMinFeaturesPerCell)->default_value(500), "minimum features per cell before adjusting sensitivity")
		("cubemap-faces", boost::program_options::value(&OPT::nCubemapFaces)->default_value(6), "number of tangent-pinhole faces used for spherical feature extraction (4, 6, 8, 12 or 20)")
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
		std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName).c_str());
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
		GET_LOG() << cmdline_options;
		if (OPT::strInputFileName.empty())
			LOG("error: input video file is required");
		return false;
	}

	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = _T("scene_tracked.sfm");
	else
		Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureValidFolderPath(OPT::strOutputDirectory);

	// validate detector type
	if (FeatureTypeFromString(OPT::strDetectorType) == FeatureType::NONE) {
		VERBOSE("error: invalid detector type '%s' (must be AKAZE, ORB, SIFT, or SIFTGPU)", OPT::strDetectorType.c_str());
		return false;
	}

	// validate overlap threshold
	if (OPT::fOverlapThreshold < 0.f || OPT::fOverlapThreshold > 1.f) {
		VERBOSE("error: overlap threshold must be between 0 and 1");
		return false;
	}

	SEACAVE::Initialize(APPNAME, OPT::nMaxThreads, OPT::nProcessPriority);
	return true;
}

// finalize application instance
void Application::Finalize()
{
	SEACAVE::Finalize();
	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // namespace


// Main function
int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index or use _CrtSetBreakAlloc() to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	// Configure keyframe extraction
	KeyframeConfig config;
	config.detectorType = FeatureTypeFromString(OPT::strDetectorType);
	config.overlapThreshold = OPT::fOverlapThreshold;
	config.maxFeaturesPerCell = OPT::nMaxFeaturesPerCell;
	config.minFeaturesPerCell = OPT::nMinFeaturesPerCell;
	config.cubemapFaces = OPT::nCubemapFaces;
	config.blurSize = OPT::nBlurSize;
	config.outputDirectory = MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputDirectory);
	config.focalLength = OPT::fFocalLength;
	config.ppOffsetX = OPT::fPPOffsetX;
	config.ppOffsetY = OPT::fPPOffsetY;
	config.cameraType = (CameraType)(OPT::nCameraType+1);
	config.refineCalibration = (KeyframeConfig::RefineCalibrationType)OPT::nRefineCalibration;
	#ifdef _USE_CUDA
	config.useCUDA = !SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs);
	#endif

	VERBOSE("Keyframe Extraction Configuration:");
	VERBOSE("  Input video: %s", OPT::strInputFileName.c_str());
	VERBOSE("  Output directory: %s", config.outputDirectory.c_str());
	VERBOSE("  Output scene: %s", OPT::strOutputFileName.c_str());
	VERBOSE("  Detector type: %s", OPT::strDetectorType.c_str());
	VERBOSE("  Overlap threshold: %.2f", config.overlapThreshold);
	VERBOSE("  Max features per cell: %u", config.maxFeaturesPerCell);
	VERBOSE("  Min features per cell: %u", config.minFeaturesPerCell);
	VERBOSE("  Blur size (optical flow kernel size): %u", config.blurSize);
	VERBOSE("  Calibration refinement: %s", config.refineCalibration ? "enabled" : "disabled");
	if (config.focalLength > 0) {
		VERBOSE("  Focal length: %.2f pixels [user-provided]", config.focalLength);
		VERBOSE("  Principal point offset: (%.2f, %.2f) pixels", config.ppOffsetX, config.ppOffsetY);
	} else {
		VERBOSE("  Focal length: auto-calibrate from fundamental matrices");
		VERBOSE("  Principal point: image center%s",
		        (config.ppOffsetX != 0 || config.ppOffsetY != 0) ?
		        String::FormatString(" + offset (%.2f, %.2f)", config.ppOffsetX, config.ppOffsetY).c_str() : "");
	}

	// Extract keyframes from video
	Scene scene(OPT::nMaxThreads);
	if (!KeyframeExtractor::ExtractFromVideo(MAKE_PATH_SAFE(OPT::strInputFileName), config, scene)) {
		VERBOSE("error: keyframe extraction failed");
		return EXIT_FAILURE;
	}

	// Print statistics
	VERBOSE("");
	VERBOSE("Keyframe Extraction Statistics:");
	VERBOSE("  Number of keyframes: %u", scene.images.size());
	VERBOSE("  Number of image pairs: %u", scene.pairs.size());
	VERBOSE("  Number of cameras: %u", scene.cameras.size());

	if (!scene.images.IsEmpty()) {
		// Compute total features
		MeanStdMinMax<size_t,double> featuresStats;
		for (const Image& image : scene.images)
			featuresStats.Update(image.keypoints.size());
		VERBOSE("  Features per keyframe: num %zu, min %u, mean %.0f, max %u",
			(unsigned)featuresStats.size, (unsigned)featuresStats.GetMin(), featuresStats.GetMean(), (unsigned)featuresStats.GetMax());
	}

	if (!scene.pairs.IsEmpty()) {
		// Compute pair statistics
		MeanStdMinMax<size_t,double> matchesStats;
		MeanStdMinMax<size_t,double> inlierMatchesStats;
		float avgOverlapRatio = 0.f, avgOverlapArea = 0.f;
		for (const ImagePair& pair : scene.pairs) {
			matchesStats.Update(pair.matches.size());
			inlierMatchesStats.Update(pair.GetNumFilteredInliers());
			avgOverlapRatio += pair.overlapRatio;
			avgOverlapArea += pair.overlapArea;
		}
		avgOverlapRatio /= scene.pairs.size();
		avgOverlapArea /= scene.pairs.size();
		VERBOSE("  Matches: num %zu, mean %.0f, min %u, max %u", matchesStats.size, matchesStats.GetMean(), (unsigned)matchesStats.GetMin(), (unsigned)matchesStats.GetMax());
		VERBOSE("  Inlier matches: num %zu, mean %.0f, min %u, max %u", inlierMatchesStats.size, inlierMatchesStats.GetMean(), (unsigned)inlierMatchesStats.GetMin(), (unsigned)inlierMatchesStats.GetMax());
		VERBOSE("  Average overlap: ratio %.2f%%, area %.2f%%", avgOverlapRatio * 100.f, avgOverlapArea * 100.f);
	}

	// Save scene to file
	VERBOSE("");
	if (!scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType)) {
		VERBOSE("error: failed to save scene");
		return EXIT_FAILURE;
	}
	VERBOSE("Keyframe extraction completed successfully in %s", TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

