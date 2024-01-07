/*
 * InterfaceMVSNet.cpp
 *
 * Copyright (c) 2014-2021 SEACAVE
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

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define MVSNET_USE_OPENMP
#endif

#define APPNAME _T("InterfaceMVSNet")
#define MVS_EXT _T(".mvs")
#define MVSNET_IMAGES_FOLDER _T("images")
#define MVSNET_CAMERAS_FOLDER _T("cams")
#define MVSNET_IMAGES_EXT _T(".jpg")
#define MVSNET_CAMERAS_NAME _T("_cam.txt")
#define RTMV_CAMERAS_EXT _T(".json")
#define NERFSTUDIO_TRANSFORMS _T("transforms.json")


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
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
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
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-") + Util::getUniqueName(0) + _T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	const String strInputFileNameExt(Util::getFileExt(OPT::strInputFileName).ToLower());
	const bool bInvalidCommand(OPT::strInputFileName.empty());
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = Util::getFileName(OPT::strInputFileName) + "scene" MVS_EXT;

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

void ImageListParse(const LPSTR* argv, Point3& C)
{
	// read position vector
	C.x = String::FromString<REAL>(argv[0]);
	C.y = String::FromString<REAL>(argv[1]);
	C.z = String::FromString<REAL>(argv[2]);
}

void ImageListParse(const LPSTR* argv, Matrix3x3& R)
{
	// read rotation matrix
	R(0, 0) = String::FromString<REAL>(argv[0]);
	R(0, 1) = String::FromString<REAL>(argv[1]);
	R(0, 2) = String::FromString<REAL>(argv[2]);
	R(1, 0) = String::FromString<REAL>(argv[3]);
	R(1, 1) = String::FromString<REAL>(argv[4]);
	R(1, 2) = String::FromString<REAL>(argv[5]);
	R(2, 0) = String::FromString<REAL>(argv[6]);
	R(2, 1) = String::FromString<REAL>(argv[7]);
	R(2, 2) = String::FromString<REAL>(argv[8]);
}

void ImageListParse(const LPSTR* argv, Matrix3x4& P)
{
	// read projection matrix
	P(0, 0) = String::FromString<REAL>(argv[0]);
	P(0, 1) = String::FromString<REAL>(argv[1]);
	P(0, 2) = String::FromString<REAL>(argv[2]);
	P(0, 3) = String::FromString<REAL>(argv[3]);
	P(1, 0) = String::FromString<REAL>(argv[4]);
	P(1, 1) = String::FromString<REAL>(argv[5]);
	P(1, 2) = String::FromString<REAL>(argv[6]);
	P(1, 3) = String::FromString<REAL>(argv[7]);
	P(2, 0) = String::FromString<REAL>(argv[8]);
	P(2, 1) = String::FromString<REAL>(argv[9]);
	P(2, 2) = String::FromString<REAL>(argv[10]);
	P(2, 3) = String::FromString<REAL>(argv[11]);
}

// convert a range-map to depth-map
void RangeToDepthMap(const Image32F& rangeMap, const Camera& camera, DepthMap& depthMap)
{
	depthMap.create(rangeMap.size());
	for (int y = 0; y < depthMap.rows; ++y) {
		const float* const rangeRow = rangeMap.ptr<float>(y);
		float* const depthRow = depthMap.ptr<float>(y);
		for (int x = 0; x < depthMap.cols; ++x) {
			const float range = rangeRow[x];
			depthRow[x] = (Depth)(range <= 0 ? 0 : normalized(camera.TransformPointI2C(Point2(x,y))).z*range);
		}
	}
}

// parse scene stored in MVSNet format composed of undistorted images and camera poses
// for example see GigaVision benchmark (http://gigamvs.net):
// |--sceneX
//   |--images
//     |--xxx.jpg
//     |--xxx.jpg
//     ....
//   |--cams
//     |--xxx_cam.txt
//     |--xxx_cam.txt
//     ....
//   |--render_cams
//     |--xxx_cam.txt
//     |--xxx_cam.txt
//     ....
//
// where the camera parameter of one image stored in a cam.txt file contains the camera
// extrinsic E = [R|t], intrinsic K and the depth range:
//  extrinsic
//  E00 E01 E02 E03
//  E10 E11 E12 E13
//  E20 E21 E22 E23
//  E30 E31 E32 E33
//
//  intrinsic
//  K00 K01 K02
//  K10 K11 K12
//  K20 K21 K22
//
//  DEPTH_MIN DEPTH_INTERVAL (DEPTH_NUM DEPTH_MAX)
bool ParseSceneMVSNet(Scene& scene, const String& strPath)
{
	#if defined(_SUPPORT_CPP17) && (!defined(__GNUC__) || (__GNUC__ > 7))
	IIndex prevPlatformID = NO_ID;
	for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator((strPath + MVSNET_IMAGES_FOLDER).c_str())) {
		if (entry.path().extension() != MVSNET_IMAGES_EXT)
			continue;
		// parse camera
		const std::string strCamFileName(strPath + MVSNET_CAMERAS_FOLDER PATH_SEPARATOR_STR + entry.path().stem().string().c_str() + MVSNET_CAMERAS_NAME);
		std::ifstream fcam(strCamFileName);
		if (!fcam)
			continue;
		String line;
		do {
			std::getline(fcam, line);
		} while (line != "extrinsic" && fcam.good());
		String strP;
		for (int i = 0; i < 3; ++i) {
			std::getline(fcam, line);
			strP += ' ' + line;
		}
		if (!fcam.good())
			continue;
		size_t argc;
		CAutoPtrArr<LPSTR> argv;
		argv = Util::CommandLineToArgvA(strP, argc);
		if (argc != 12)
			continue;
		Matrix3x4 P;
		ImageListParse(argv, P);
		do {
			std::getline(fcam, line);
		} while (line != "intrinsic" && fcam.good());
		strP.clear();
		for (int i = 0; i < 3; ++i) {
			std::getline(fcam, line);
			strP += ' ' + line;
		}
		if (!fcam.good())
			continue;
		argv = Util::CommandLineToArgvA(strP, argc);
		if (argc != 9)
			continue;
		Matrix3x3 K;
		ImageListParse(argv, K);
		// setup camera
		IIndex platformID;
		if (prevPlatformID == NO_ID || !K.IsEqual(scene.platforms[prevPlatformID].cameras[0].K, 1e-3)) {
			prevPlatformID = platformID = scene.platforms.size();
			Platform& platform = scene.platforms.emplace_back();
			Platform::Camera& camera = platform.cameras.emplace_back();
			camera.K = K;
			camera.R = RMatrix::IDENTITY;
			camera.C = CMatrix::ZERO;
		} else {
			platformID = prevPlatformID;
		}
		Platform& platform = scene.platforms[platformID];
		// setup image
		const IIndex ID = scene.images.size();
		Image& imageData = scene.images.emplace_back();
		imageData.platformID = platformID;
		imageData.cameraID = 0; // only one camera per platform supported by this format
		imageData.poseID = NO_ID;
		imageData.ID = ID;
		imageData.name = entry.path().string();
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(strPath, imageData.name);
		// set image resolution
		IMAGEPTR pimage = Image::ReadImageHeader(imageData.name);
		imageData.width = pimage->GetWidth();
		imageData.height = pimage->GetHeight();
		imageData.scale = 1;
		// set camera pose
		imageData.poseID = platform.poses.size();
		Platform::Pose& pose = platform.poses.emplace_back();
		DecomposeProjectionMatrix(P, pose.R, pose.C);
		imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
	}
	if (scene.images.size() < 2)
		return false;
	scene.nCalibratedImages = (unsigned)scene.images.size();
	return true;
	#else
	VERBOSE("error: C++17 is required to parse MVSNet format");
	return false;
	#endif // _SUPPORT_CPP17
}

// RTMV scene format: http://www.cs.umd.edu/~mmeshry/projects/rtmv
// |--sceneX
//   |--images
//     |--xxx.jpg
//     |--xxx.jpg
//     ....
//   |--outputs (optional)
//     |--depthxxxx.exr
//     |--normalxxxx.exr
//     ....
//   |--transforms.json
bool ParseSceneNerfstudio(Scene& scene, const String& strPath)
{
	const nlohmann::json data = nlohmann::json::parse(std::ifstream(strPath + NERFSTUDIO_TRANSFORMS));
	if (data.empty())
		return false;
	// parse camera
	const cv::Size resolution(data["w"].get<uint32_t>(), data["h"].get<uint32_t>());
	const IIndex platformID = scene.platforms.size();
	Platform& platform = scene.platforms.emplace_back();
	Platform::Camera& camera = platform.cameras.emplace_back();
	camera.K = KMatrix::IDENTITY;
	camera.R = RMatrix::IDENTITY;
	camera.C = CMatrix::ZERO;
	camera.K(0,0) = data["fl_x"].get<REAL>();
	camera.K(1,1) = data["fl_y"].get<REAL>();
	camera.K(0,2) = data["cx"].get<REAL>();
	camera.K(1,2) = data["cy"].get<REAL>();
	const String cameraModel = data["camera_model"].get<std::string>();
	if (cameraModel == "SIMPLE_PINHOLE") {
	} else
	// check ZERO radial distortion for all "PERSPECTIVE" type cameras
	if (cameraModel == "PINHOLE" || cameraModel == "SIMPLE_RADIAL" || cameraModel == "RADIAL" || cameraModel == "OPENCV") {
		const REAL k1 = data["k1"].get<REAL>();
		const REAL k2 = data["k2"].get<REAL>();
		const REAL p1 = data["p1"].get<REAL>();
		const REAL p2 = data["p2"].get<REAL>();
		if (k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0) {
			VERBOSE("error: radial distortion not supported");
			return false;
		}
	} else {
		VERBOSE("error: camera model not supported");
		return false;
	}
	// parse images
	const nlohmann::json& frames = data["frames"];
	for (const nlohmann::json& frame: frames) {
		// set image
		// frames expected to be ordered in JSON
		const IIndex imageID = scene.images.size(); 
		const String strFileName(strPath + frame["file_path"].get<std::string>().c_str());
		Image& imageData = scene.images.emplace_back();
		imageData.platformID = platformID;
		imageData.cameraID = 0; // only one camera per platform supported by this format
		imageData.poseID = NO_ID;
		imageData.ID = imageID;
		imageData.name = strFileName;
		ASSERT(Util::isFullPath(imageData.name));
		// set image resolution
		imageData.width = resolution.width;
		imageData.height = resolution.height;
		imageData.scale = 1;
		// load camera pose
		imageData.poseID = platform.poses.size();
		Platform::Pose& pose = platform.poses.emplace_back();
		const auto Ps = frame["transform_matrix"].get<std::vector<std::vector<double>>>();
		Eigen::Matrix4d P{
			{Ps[0][0], Ps[0][1], Ps[0][2], Ps[0][3]},
			{Ps[1][0], Ps[1][1], Ps[1][2], Ps[1][3]},
			{Ps[2][0], Ps[2][1], Ps[2][2], Ps[2][3]},
			{Ps[3][0], Ps[3][1], Ps[3][2], Ps[3][3]}
		};
		// revert nerfstudio conversion:
		// convert from COLMAP's camera coordinate system (OpenCV) to ours (OpenGL)
		//   c2w[0:3, 1:3] *= -1
		//   c2w = c2w[np.array([1, 0, 2, 3]), :]
		//   c2w[2, :] *= -1
		P.row(2) *= -1;
		P.row(0).swap(P.row(1));
		P.col(2) *= -1;
		P.col(1) *= -1;
		// set camera pose
		pose.R = P.topLeftCorner<3, 3>().transpose().eval();
		pose.R.EnforceOrthogonality();
		pose.C = P.topRightCorner<3, 1>().eval();
		imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
		// try reading depth-map and normal-map
		DepthMap depthMap; {
			const String depthPath(strPath + String::FormatString("outputs/depth%04u.exr", imageID));
			const Image32F rangeMap = cv::imread(depthPath, cv::IMREAD_UNCHANGED);
			if (rangeMap.empty()) {
				VERBOSE("Unable to load depthmap %s.", depthPath.c_str());
				continue;
			}
			RangeToDepthMap(rangeMap, imageData.camera, depthMap);
		}
		NormalMap normalMap; {
			const String normalPath(strPath + String::FormatString("outputs/normal%04u.exr", imageID));
			normalMap = cv::imread(normalPath, cv::IMREAD_UNCHANGED);
			if (normalMap.empty()) {
				VERBOSE("Unable to load normalMap %s.", normalPath.c_str());
				continue;
			}
		}
		const ConfidenceMap confMap;
		const ViewsMap viewsMap;
		const IIndexArr IDs = {imageID};
		double dMin, dMax;
		cv::minMaxIdx(depthMap, &dMin, &dMax, NULL, NULL, depthMap > 0);
		const String dmapPath(strPath + String::FormatString("depth%04u.dmap", imageID));
		if (!ExportDepthDataRaw(dmapPath,
			imageData.name, IDs, resolution,
			camera.K, pose.R, pose.C,
			(float)dMin, (float)dMax,
			depthMap, normalMap, confMap, viewsMap))
		{
			VERBOSE("Unable to save dmap: %s", dmapPath.c_str());
			continue;
		}
	}
	if (scene.images.size() < 2)
		return false;
	scene.nCalibratedImages = (unsigned)scene.images.size();
	return true;
}

// RTMV scene format: http://www.cs.umd.edu/~mmeshry/projects/rtmv
// |--sceneX
//   |--xxx.exr
//   |--xxx.seg.exr
//   |--xxx.depth.exr
//   |--xxx.json
//   ....
bool ParseSceneRTMV(Scene& scene, const String& strPath)
{
	const String strImagePath(strPath + "images/");
	Util::ensureFolder(strImagePath);
	std::vector<String> strImageNames;
	#if defined(_SUPPORT_CPP17) && (!defined(__GNUC__) || (__GNUC__ > 7))
	for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(strPath.c_str())) {
		if (entry.path().extension() != RTMV_CAMERAS_EXT)
			continue;
		strImageNames.emplace_back(entry.path().stem().string());
	}
	#else
	VERBOSE("error: C++17 is required to parse RTMV format");
	return false;
	#endif // _SUPPORT_CPP17
	IIndex prevPlatformID = NO_ID;
	scene.images.resize((IIndex)strImageNames.size());
	scene.platforms.reserve((IIndex)strImageNames.size());
	#ifdef MVSNET_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	for (int_t i=0; i<(int_t)strImageNames.size(); ++i) {
	#else
	FOREACH(i, strImageNames) {
	#endif
		const IIndex imageID((IIndex)i);
		const String& strImageName(strImageNames[imageID]);
		// parse camera
		const String strFileName(strPath + strImageName);
		const nlohmann::json dataCamera = nlohmann::json::parse(std::ifstream(strFileName+RTMV_CAMERAS_EXT));
		if (dataCamera.empty())
			continue;
		const nlohmann::json& data = dataCamera["camera_data"];
		const cv::Size resolution(data["width"].get<uint32_t>(), data["height"].get<uint32_t>());
		// set platform
		Matrix3x3 K = Matrix3x3::IDENTITY;
		K(0,0) = data["intrinsics"]["fx"].get<REAL>();
		K(1,1) = data["intrinsics"]["fy"].get<REAL>();
		K(0,2) = data["intrinsics"]["cx"].get<REAL>();
		K(1,2) = data["intrinsics"]["cy"].get<REAL>();
		IIndex platformID;
		if (prevPlatformID == NO_ID || !K.IsEqual(scene.platforms[prevPlatformID].cameras[0].K, 1e-3)) {
			#ifdef MVSNET_USE_OPENMP
			#pragma omp critical
			#endif
			{
			prevPlatformID = platformID = scene.platforms.size();
			Platform& platform = scene.platforms.emplace_back();
			Platform::Camera& camera = platform.cameras.emplace_back();
			platform.poses.reserve((IIndex)strImageNames.size());
			camera.K = K;
			camera.R = RMatrix::IDENTITY;
			camera.C = CMatrix::ZERO;
			}
		} else {
			platformID = prevPlatformID;
		}
		Platform& platform = scene.platforms[platformID];
		// set image
		Image& imageData = scene.images[imageID];
		imageData.platformID = platformID;
		imageData.cameraID = 0; // only one camera per platform supported by this format
		imageData.poseID = NO_ID;
		imageData.ID = imageID;
		imageData.name = strImagePath+strImageName+".jpg";
		ASSERT(Util::isFullPath(imageData.name));
		{
			cv::Mat image = cv::imread(strFileName+".exr", cv::IMREAD_UNCHANGED);
			ASSERT(image.type() == CV_32FC4);
			std::vector<cv::Mat> channels;
			cv::split(image, channels);
			cv::merge(std::vector<cv::Mat>{channels[0], channels[1], channels[2]}, image);
			image.convertTo(imageData.image, CV_8UC3, 255);
		}
		ASSERT(resolution == imageData.image.size());
		if (imageData.image.empty()) {
			VERBOSE("Unable to load image %s.", (strFileName+".exr").c_str());
			continue;
		}
		cv::imwrite(imageData.name, imageData.image);
		imageData.ReleaseImage();
		// set image resolution
		imageData.width = resolution.width;
		imageData.height = resolution.height;
		imageData.scale = 1;
		// load camera pose
		#ifdef MVSNET_USE_OPENMP
		#pragma omp critical
		#endif
		{
			imageData.poseID = platform.poses.size();
			platform.poses.emplace_back();
		}
		Platform::Pose& pose = platform.poses[imageData.poseID];
		const auto Ps = data["cam2world"].get<std::vector<std::vector<double>>>();
		Eigen::Matrix4d P{
			{Ps[0][0], Ps[1][0], Ps[2][0], Ps[3][0]},
			{Ps[0][1], Ps[1][1], Ps[2][1], Ps[3][1]},
			{Ps[0][2], Ps[1][2], Ps[2][2], Ps[3][2]},
			{Ps[0][3], Ps[1][3], Ps[2][3], Ps[3][3]}
		};
		// apply the same transforms as nerfstudio converter
		P.row(2) *= -1;
		P.row(0).swap(P.row(1));
		P.col(2) *= -1;
		P.col(1) *= -1;
		// set camera pose
		pose.R = P.topLeftCorner<3, 3>().transpose().eval();
		pose.R.EnforceOrthogonality();
		pose.C = P.topRightCorner<3, 1>().eval();
		imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
		// try reading the segmentation mask
		{
			cv::Mat imgMask = cv::imread(strFileName+".seg.exr", cv::IMREAD_UNCHANGED);
			if (imgMask.empty()) {
				VERBOSE("Unable to load segmentation mask %s.", (strFileName+".seg.exr").c_str());
				continue;
			}
			ASSERT(imgMask.type() == CV_32FC4);
			ASSERT(resolution == imgMask.size());
			std::vector<cv::Mat> channels;
			cv::split(imgMask, channels);
			channels[0].convertTo(imgMask, CV_16U);
			imageData.maskName = strImagePath+strImageName+".mask.png";
			cv::imwrite(imageData.maskName, imgMask);
		}
		// try reading the depth-map
		DepthMap depthMap; {
			const cv::Mat imgDepthMap = cv::imread(strFileName+".depth.exr", cv::IMREAD_UNCHANGED);
			if (imgDepthMap.empty()) {
				VERBOSE("Unable to load depthmap %s.", (strFileName+".depth.exr").c_str());
				continue;
			}
			ASSERT(imgDepthMap.type() == CV_32FC4);
			ASSERT(resolution == imgDepthMap.size());
			std::vector<cv::Mat> channels;
			cv::split(imgDepthMap, channels);
			RangeToDepthMap(channels[0], imageData.camera, depthMap);
		}
		const NormalMap normalMap;
		const ConfidenceMap confMap;
		const ViewsMap viewsMap;
		const IIndexArr IDs = {imageID};
		double dMin, dMax;
		cv::minMaxIdx(depthMap, &dMin, &dMax, NULL, NULL, depthMap > 0);
		const String dmapPath(strPath + String::FormatString("depth%04u.dmap", imageID));
		if (!ExportDepthDataRaw(dmapPath,
			imageData.name, IDs, resolution,
			K, pose.R, pose.C,
			(float)dMin, (float)dMax,
			depthMap, normalMap, confMap, viewsMap))
		{
			VERBOSE("Unable to save dmap: %s", dmapPath.c_str());
			continue;
		}
	}
	if (scene.images.size() < 2)
		return false;
	scene.nCalibratedImages = (unsigned)scene.images.size();
	return true;
}

bool ParseScene(Scene& scene)
{
	#if defined(_SUPPORT_CPP17) && (!defined(__GNUC__) || (__GNUC__ > 7))
	String strPath(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strInputFileName));
	Util::ensureValidFolderPath(strPath);
	const std::filesystem::path path(static_cast<const std::string&>(strPath));
	enum Type {
		MVSNet = 0,
		NERFSTUDIO,
		RTMV,
	} sceneType = MVSNet;
	for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(path)) {
		if (entry.path().extension() == RTMV_CAMERAS_EXT) {
			if (entry.path().filename() == NERFSTUDIO_TRANSFORMS) {
				sceneType = NERFSTUDIO;
				break;
			}
			sceneType = RTMV;
		}
	}
	switch (sceneType) {
	case NERFSTUDIO: return ParseSceneNerfstudio(scene, strPath);
	case RTMV: return ParseSceneRTMV(scene, strPath);
	default: return ParseSceneMVSNet(scene, strPath);
	}
	#else
	VERBOSE("error: C++17 is required to parse MVSNet format");
	return false;
	#endif // _SUPPORT_CPP17
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

	// convert data from MVSNet format to OpenMVS
	if (!ParseScene(scene))
		return EXIT_FAILURE;

	// write OpenMVS input data
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	VERBOSE("Imported data: %u platforms, %u images, %u vertices (%s)",
		scene.platforms.size(), scene.images.size(), scene.pointcloud.GetSize(),
		TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
