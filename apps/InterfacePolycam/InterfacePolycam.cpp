/*
 * InterfacePolycam.cpp
 *
 * Copyright (c) 2014-2023 SEACAVE
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
#define JSON_NOEXCEPTION
#include "../../libs/IO/json.hpp"
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfacePolycam")
#define MVS_EXT _T(".mvs")
#define JSON_EXT _T(".json")
#define DEPTH_EXT _T(".png")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strOutputFileName;
unsigned nArchiveType;
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
		("help,h", "imports SfM scene stored Polycam format")
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
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input folder containing Polycam camera poses, images and depth-maps")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the scene")
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
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidFolderPath(OPT::strInputFileName);
	const bool bInvalidCommand(OPT::strInputFileName.empty());
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidFolderPath(OPT::strOutputFileName);
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = "scene" MVS_EXT;

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

// parse image containing calibration, pose, and depth-map information
bool ParseImage(Scene& scene, const String& imagePath, const String& cameraPath, const String& depthPath,
	const std::unordered_map<String, IIndex>& mapImageName)
{
	nlohmann::json data = nlohmann::json::parse(std::ifstream(cameraPath));
	if (data.empty())
		return false;
	const cv::Size resolution(data["width"].get<uint32_t>(), data["height"].get<uint32_t>());
	// set platform
	const IIndex platformID = scene.platforms.size();
	Platform& platform = scene.platforms.AddEmpty();
	Platform::Camera& camera = platform.cameras.AddEmpty();
	camera.K = KMatrix::IDENTITY;
	camera.R = RMatrix::IDENTITY;
	camera.C = CMatrix::ZERO;
	camera.K(0,0) = data["fx"].get<float>();
	camera.K(1,1) = data["fy"].get<float>();
	camera.K(0,2) = data["cx"].get<float>();
	camera.K(1,2) = data["cy"].get<float>();
	// set image
	const IIndex imageID = scene.images.size();
	Image& imageData = scene.images.AddEmpty();
	imageData.platformID = platformID;
	imageData.cameraID = 0; // only one camera per platform supported by this format
	imageData.poseID = NO_ID;
	imageData.ID = imageID;
	imageData.name = imagePath;
	ASSERT(Util::isFullPath(imageData.name));
	// set image resolution
	imageData.width = resolution.width;
	imageData.height = resolution.height;
	imageData.scale = 1;
	// set camera pose
	imageData.poseID = platform.poses.size();
	Platform::Pose& pose = platform.poses.AddEmpty();
	const Eigen::Matrix3d R_session_arkitcam{
		{data["t_00"].get<double>(), data["t_01"].get<double>(), data["t_02"].get<double>()},
		{data["t_10"].get<double>(), data["t_11"].get<double>(), data["t_12"].get<double>()},
		{data["t_20"].get<double>(), data["t_21"].get<double>(), data["t_22"].get<double>()}
	};
	const Eigen::Vector3d t_session_arkitcam{
		data["t_03"].get<double>(),
		data["t_13"].get<double>(),
		data["t_23"].get<double>()
	};
	const Eigen::Affine3d T_session_arkitcam{
		Eigen::Affine3d(Eigen::Translation3d(t_session_arkitcam)) * Eigen::Affine3d(Eigen::AngleAxisd(R_session_arkitcam))
	};
	const Eigen::Affine3d T_cam_arkitcam{
		Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
	};
	const Eigen::Matrix4d P{
		(T_cam_arkitcam * T_session_arkitcam.inverse()).matrix()
	};
	pose.R = P.topLeftCorner<3, 3>().eval();
	pose.R.EnforceOrthogonality();
	const Point3d t = P.topRightCorner<3, 1>().eval();
	pose.C = pose.R.t() * (-t);
	imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
	// set image neighbors if available
	nlohmann::json::const_iterator itNeighbors = data.find("neighbors");
	if (itNeighbors != data.end()) {
		const std::vector<uint64_t> neighborTimestamps = itNeighbors->get<std::vector<uint64_t>>();
		for (uint64_t timestamp: neighborTimestamps) {
			const String neighborName = std::to_string(timestamp);
			const IIndex neighborID = mapImageName.at(neighborName);
			if (neighborID != imageData.ID)
				imageData.neighbors.emplace_back(ViewScore{neighborID, 0, 1.f, FD2R(15.f), 0.5f, 3.f});
		}
	}
	// load and convert depth-map
	if (!depthPath.empty()) {
		DepthMap depthMap; {
			constexpr double depthScale{1000.0};
			const cv::Mat imgDepthMap = cv::imread(depthPath, cv::IMREAD_ANYDEPTH);
			if (imgDepthMap.empty())
				return false;
			imgDepthMap.convertTo(depthMap, CV_32FC1, 1.0/depthScale);
		}
		IIndexArr IDs = {imageData.ID};
		IDs.JoinFunctor(imageData.neighbors.size(), [&imageData](IIndex i) {
			return imageData.neighbors[i].ID;
		});
		double dMin, dMax;
		cv::minMaxIdx(depthMap, &dMin, &dMax, NULL, NULL, depthMap > 0);
		const NormalMap normalMap;
		const ConfidenceMap confMap;
		const ViewsMap viewsMap;
		if (!ExportDepthDataRaw(MAKE_PATH(String::FormatString("depth%04u.dmap", imageData.ID)),
			imageData.name, IDs, resolution,
			camera.K, pose.R, pose.C,
			(float)dMin, (float)dMax,
			depthMap, normalMap, confMap, viewsMap))
			return false;
	}
	return true;
}

// parse scene stored in Polycam format
bool ParseScene(Scene& scene, const String& scenePath)
{
	#if defined(_SUPPORT_CPP17) && (!defined(__GNUC__) || (__GNUC__ > 7))
	size_t numCorrectedFolders(0), numCorrectedDepthFolders(0), numFolders(0), numDepthFolders(0);
	for (const auto& file: std::filesystem::directory_iterator(scenePath.c_str())) {
		if (file.path().stem() == "corrected_cameras" ||
			file.path().stem() == "corrected_images")
			++numCorrectedFolders;
		else if (file.path().stem() == "corrected_depth")
			++numCorrectedDepthFolders;
		else if (file.path().stem() == "cameras" ||
			file.path().stem() == "images")
			++numFolders;
		else if (file.path().stem() == "depth")
			++numDepthFolders;
	}
	if (numFolders != 2) {
		VERBOSE("Invalid scene folder");
		return false;
	}
	if (numCorrectedFolders == 2) {
		// corrected data
		CLISTDEFIDX(String, IIndex) imagePaths;
		for (const auto& file: std::filesystem::directory_iterator((scenePath + "corrected_images").c_str()))
			imagePaths.emplace_back(file.path().string());
		VERBOSE("Parsing corrected data: %u...", imagePaths.size());
		std::unordered_map<String, IIndex> mapImageName;
		mapImageName.reserve(imagePaths.size());
		for (String& imagePath: imagePaths) {
			Util::ensureValidPath(imagePath);
			mapImageName.emplace(Util::getFileName(imagePath), static_cast<IIndex>(mapImageName.size()));
		}
		for (const String& imagePath: imagePaths) {
			const String imageName = Util::getFileName(imagePath);
			const String cameraPath(scenePath + "corrected_cameras" + PATH_SEPARATOR_STR + imageName + JSON_EXT);
			const String depthPath(numCorrectedDepthFolders ? scenePath + "corrected_depth" + PATH_SEPARATOR_STR + imageName + DEPTH_EXT : String());
			if (!ParseImage(scene, imagePath, cameraPath, depthPath, mapImageName))
				return false;
		}
	} else {
		// raw data
		CLISTDEFIDX(String, IIndex) imagePaths;
		for (const auto& file: std::filesystem::directory_iterator((scenePath + "images").c_str()))
			imagePaths.emplace_back(file.path().string());
		VERBOSE("Parsing raw data: %u...", imagePaths.size());
		std::unordered_map<String, IIndex> mapImageName;
		mapImageName.reserve(imagePaths.size());
		for (String& imagePath: imagePaths) {
			Util::ensureValidPath(imagePath);
			mapImageName.emplace(Util::getFileName(imagePath), static_cast<IIndex>(mapImageName.size()));
		}
		for (const String& imagePath: imagePaths) {
			const String imageName = Util::getFileName(imagePath);
			const String cameraPath(scenePath + "cameras" + PATH_SEPARATOR_STR + imageName + JSON_EXT);
			const String depthPath(numDepthFolders ? scenePath + "depth" + PATH_SEPARATOR_STR + imageName + DEPTH_EXT : String());
			if (!ParseImage(scene, imagePath, cameraPath, depthPath, mapImageName))
				return false;
		}
	}
	return true;
	#else
	return false;
	#endif
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

	TD_TIMER_START();

	Scene scene(OPT::nMaxThreads);

	// convert data from Polycam format to OpenMVS
	if (!ParseScene(scene, MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strInputFileName)))
		return EXIT_FAILURE;

	// write OpenMVS input data
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	VERBOSE("Exported data: %u platforms, %u cameras, %u poses, %u images (%s)",
			scene.platforms.size(), scene.images.size(), scene.images.size(), scene.images.size(),
			TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
