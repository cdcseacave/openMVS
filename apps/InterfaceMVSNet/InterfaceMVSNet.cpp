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
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceMVSNet")
#define MVS_EXT _T(".mvs")
#define MVSNET_IMAGES_FOLDER _T("images")
#define MVSNET_CAMERAS_FOLDER _T("cams")
#define MVSNET_IMAGES_EXT _T(".jpg")
#define MVSNET_CAMERAS_NAME _T("_cam.txt")


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
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_MVS), "project archive type: 0-text, 1-binary, 2-compressed binary")
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
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileName(OPT::strInputFileName) + "scene" MVS_EXT;

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

// parse scene stored in MVSNet format composed of undistorted images and camera poses
// for example see GigaVision benchmark (http://gigamvs.net):
// |--sceneX
//   |--images
//     |--xxx.jpg
//     |--xxx.jpg
//     ....
//     |--xxx.jpg
//   |--cams
//     |--xxx_cam.txt
//     |--xxx_cam.txt
//     ....
//     |--xxx_cam.txt
//   |--render_cams
//     |--xxx_cam.txt
//     |--xxx_cam.txt
//     ....
//     |--xxx_cam.txt
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
bool ParseSceneMVSNet(Scene& scene)
{
	#if defined(_SUPPORT_CPP17) && (!defined(__GNUC__) || (__GNUC__ > 7))
	String strPath(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strInputFileName));
	Util::ensureValidFolderPath(strPath);
	const std::filesystem::path path(static_cast<std::string&>(strPath));
	IIndex prevPlatformID = NO_ID;
	for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(path / MVSNET_IMAGES_FOLDER)) {
		if (entry.path().extension() != MVSNET_IMAGES_EXT)
			continue;
		// parse camera
		const std::string strCamFileName((path / MVSNET_CAMERAS_FOLDER / entry.path().stem()).string() + MVSNET_CAMERAS_NAME);
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

} // unnamed namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	Scene scene(OPT::nMaxThreads);

	// convert data from MVSNet format to OpenMVS
	if (!ParseSceneMVSNet(scene))
		return EXIT_FAILURE;

	// write OpenMVS input data
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	VERBOSE("Imported data: %u platforms, %u images, %u vertices (%s)",
		scene.platforms.size(), scene.images.size(), scene.pointcloud.GetSize(),
		TD_TIMER_GET_FMT().c_str());

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
