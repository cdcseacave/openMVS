/*
 * InterfaceVisualSFM.cpp
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
#include "opencv2/ccalib/multicalib.hpp"
#include "opencv2/ccalib/randpattern.hpp"
#include <boost/program_options.hpp>


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("MultiCameraCalibration")
#define LST_EXT _T(".xml")
#define MVS_EXT _T(".mvs")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strPatternFileName;
float patternWidth;
float patternHeight;
int numCameras;
int minMatches;
int cameraType;
bool fixAspectRatio;
bool fixPrincipalPoint;
bool fixRadialDistortion12;
bool fixRadialDistortion3;
bool fixRadialDistortion46;
bool fixTangentDistortion;
int showFeatureExtraction;
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
		("archive-type", boost::program_options::value<unsigned>(&OPT::nArchiveType)->default_value(2), "project archive type: 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value<int>(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("max-threads", boost::program_options::value<unsigned>(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value<int>(&g_nVerbosityLevel)->default_value(
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
		("pattern-file,p", boost::program_options::value<std::string>(&OPT::strPatternFileName)->default_value("pattern.png"), "input/output filename containing pattern")
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("pattern-width,x", boost::program_options::value<float>(&OPT::patternWidth)->default_value(1920), "physical width of random pattern")
		("pattern-height,y", boost::program_options::value<float>(&OPT::patternHeight)->default_value(1080), "physical height of random pattern")
		("number-cameras,n", boost::program_options::value<int>(&OPT::numCameras)->default_value(0), "number of cameras to calibrate")
		("camera-type,t", boost::program_options::value<int>(&OPT::cameraType)->default_value(0), "camera type: 0 - pinhole, 1 - omnidirectional")
		("fix-aspect-ratio,a", boost::program_options::value<bool>(&OPT::fixAspectRatio)->default_value(true), "fix the focal length aspect ratio")
		("fix-principal-point,f", boost::program_options::value<bool>(&OPT::fixPrincipalPoint)->default_value(false), "fix the principal point at the center")
		("fix-radial-distortion-1-2", boost::program_options::value<bool>(&OPT::fixRadialDistortion12)->default_value(false), "fix the radial distortions k1 and k2")
		("fix-radial-distortion-3,k", boost::program_options::value<bool>(&OPT::fixRadialDistortion3)->default_value(false), "fix the radial distortion k3")
		("fix-radial-distortion-4-6", boost::program_options::value<bool>(&OPT::fixRadialDistortion46)->default_value(true), "fix the radial distortions k4, k5 and k6")
		("fix-tangential-distortion,d", boost::program_options::value<bool>(&OPT::fixTangentDistortion)->default_value(false), "fix the tangential distortions p1 and p2")
		("show-feature-extraction,e", boost::program_options::value<int>(&OPT::showFeatureExtraction)->default_value(0), "whether show extracted features and feature filtering")
		("min-matches,m", boost::program_options::value<int>(&OPT::minMatches)->default_value(25), "minimal number of matched features for a frame")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the result")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config);

	boost::program_options::positional_options_description p;
	p.add("pattern-file", -1);

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
	LOG(_T("Command line:%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	Util::ensureValidPath(OPT::strPatternFileName);
	Util::ensureUnifySlash(OPT::strPatternFileName);
	bool bInvalidInput(OPT::strPatternFileName.IsEmpty());
	if (OPT::patternWidth <= 0 || OPT::patternHeight <= 0)
		VERBOSE("error: invalid pattern width/height"), bInvalidInput = true;
	if (!OPT::strInputFileName.IsEmpty()) {
		if (OPT::numCameras <= 0)
			VERBOSE("error: invalid number of cameras"), bInvalidInput = true;
		if (OPT::cameraType != 0 && OPT::cameraType != 1)
			VERBOSE("error: invalid camera type, 0 for pinhole and 1 for omnidirectional"), bInvalidInput = true;
		if (OPT::minMatches <= 0)
			VERBOSE("error: invalid number of minimal matches"), bInvalidInput = true;
		if (OPT::showFeatureExtraction != 1 && OPT::showFeatureExtraction != 0)
			VERBOSE("error: not bool value, set to 0 or 1"), bInvalidInput = true;
	}
	if (OPT::vm.count("help") || bInvalidInput) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << _T("\n"
			"Implements multi-camera calibration based on a random pattern.\n"
			"The random pattern needs to be seen by one camera for intra-calibration\n"
			"and at least two cameras at once for inter-calibration.\n"
			"Pinhole and omnidirectional cameras are supported.\n"
			"\n"
			"Example command line:\n"
			"   " APPNAME " -x 800 -y 600 -n 5 -t 1 -m 25 -e 0 -v 3 -p pattern.png -i images/*.jpg\n"
			"\n"
			"Note the first filename in images.xml is the pattern, the rest are photo filenames.\n"
			"The photo filenames should be in form of cameraIdx-timeStamp.*, where cameraIdx starts from 0.\n"
			"\n")
			<< visible;
	}
	if (bInvalidInput)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = _T("calibration") MVS_EXT;

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

// save file-names list OpenCV
bool SaveFilenamesListCV(const String& strList, const std::vector<String>& fileNames)
{
	cv::FileStorage fs(strList, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;
	fs << "images" << "[";
	for (const String& fileName: fileNames)
		fs << fileName;
	fs << "]";
	return true;
}
// load file-names list OpenCV
bool LoadFilenamesListCV(const String& strList, std::vector<String>& fileNames)
{
	fileNames.resize(0);
	cv::FileStorage fs(strList, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
		return false;
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		fileNames.emplace_back(*it);
	return true;
}

bool SaveCameraParamsCV(const String& filename, cv::Size imageSize,
	float patternWidth, float patternHeight, int flags,
	const cv::Matx33d& cameraMatrix, const std::vector<double>& distCoeffs,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	double rms)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);
	fs << "calibration_time" << buf;

	if (!rvecs.empty())
		fs << "nframes" << (int)rvecs.size();

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "pattern_width" << patternWidth;
	fs << "pattern_height" << patternHeight;

	fs << "flags" << flags;

	fs << "camera_matrix" << cv::Mat(cameraMatrix);
	fs << "camera_distortion" << cv::Mat(distCoeffs);

	if (!rvecs.empty() && !tvecs.empty()) {
		ASSERT(rvecs[0].type() == tvecs[0].type());
		cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++) {
			cv::Mat r = bigmat(cv::Range(i, i + 1), cv::Range(0, 3));
			cv::Mat t = bigmat(cv::Range(i, i + 1), cv::Range(3, 6));
			ASSERT(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			ASSERT(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "camera_poses" << bigmat;
	}

	fs << "rms" << rms;
	return true;
}

// generate random pattern used for calibration
int GeneratePattern()
{
	TD_TIMER_START();

	#ifdef _RELEASE
	cv::setRNGSeed((int)Util::getTime());
	#endif
	cv::randpattern::RandomPatternGenerator generator(ROUND2INT(OPT::patternWidth), ROUND2INT(OPT::patternHeight));
	generator.generatePattern();
	cv::Mat pattern = generator.getPattern();
	if (!cv::imwrite(MAKE_PATH_SAFE(OPT::strPatternFileName), pattern))
		return EXIT_FAILURE;

	VERBOSE("Random pattern generated (%ux%u pixels): '%s' (%s)", pattern.cols, pattern.rows, OPT::strPatternFileName.c_str(), TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}

// calibrate intrinsics and extrinsics of a multi camera system
int CalibrateMultiCamera()
{
	TD_TIMER_START();

	// fetch all input calibration image filenames
	const String dir(Util::getFilePath(MAKE_PATH_SAFE(OPT::strInputFileName)));
	const String wld(Util::getFileNameExt(OPT::strInputFileName));
	std::vector<String> fileNames(Util::getFolderWildcard(dir, wld, true, false, true));

	// creates a yaml or xml list of files
	const String strImagesList(Util::getFileFullName(MAKE_PATH_SAFE(OPT::strOutputFileName)) + _T("_images") LST_EXT);
	for (String& fileName: fileNames)
		fileName = Util::getSimplifiedPath(dir+fileName);
	fileNames.insert(fileNames.begin(), MAKE_PATH_SAFE(OPT::strPatternFileName));
	if (!SaveFilenamesListCV(strImagesList, fileNames))
		return EXIT_FAILURE;

	// set calibration flags
	int flags(0);
	if (OPT::fixAspectRatio)
		flags |= cv::CALIB_FIX_ASPECT_RATIO;
	if (OPT::fixPrincipalPoint)
		flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
	if (OPT::fixRadialDistortion12)
		flags |= cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2;
	if (OPT::fixRadialDistortion3)
		flags |= cv::CALIB_FIX_K3;
	if (OPT::fixRadialDistortion46)
		flags |= cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
	else
		flags |= cv::CALIB_RATIONAL_MODEL;
	if (OPT::fixTangentDistortion)
		flags |= cv::CALIB_FIX_TANGENT_DIST;

	ASSERT(OPT::numCameras > 0);
	const String strCalibration(Util::getFileFullName(MAKE_PATH_SAFE(OPT::strOutputFileName)) + LST_EXT);
	if (OPT::numCameras == 1) {
		// the first image is the pattern
		const cv::Mat pattern(cv::imread(fileNames.front(), cv::IMREAD_GRAYSCALE));
		std::vector<cv::Mat> vecImg;
		for (size_t i=1; i<fileNames.size(); ++i) {
			vecImg.emplace_back(cv::imread(fileNames[i], cv::IMREAD_GRAYSCALE));
			if ((pattern.rows < pattern.cols) != (vecImg.back().rows < vecImg.back().cols))
				cv::flip(vecImg.back().t(), vecImg.back(), 1);
		}

		// do single-camera calibration
		cv::randpattern::RandomPatternCornerFinder finder(OPT::patternWidth, OPT::patternHeight, OPT::minMatches, CV_32F, VERBOSITY_LEVEL>2?1:0);
		finder.loadPattern(pattern);
		finder.computeObjectImagePoints(vecImg);
		cv::Matx33d K(
			1,0,double(vecImg.front().cols-1)*0.5,
			0,1,double(vecImg.front().rows-1)*0.5,
			0,0,1
		);
		std::vector<double> D(8, 0.0);
		std::vector<cv::Mat> rvec, tvec;
		double rms = cv::calibrateCamera(finder.getObjectPoints(), finder.getImagePoints(), vecImg[0].size(), K, D, rvec, tvec, flags);
		SaveCameraParamsCV(strCalibration, vecImg.front().size(), OPT::patternWidth, OPT::patternHeight, flags, K, D, rvec, tvec, rms);

		VERBOSE("Camera calibrated: %u images, %gRMS (%s)", fileNames.size()-1, rms, TD_TIMER_GET_FMT().c_str());
	} else {
		// do multi-camera calibration
		cv::multicalib::MultiCameraCalibration multiCalib(OPT::cameraType, OPT::numCameras, strImagesList, OPT::patternWidth, OPT::patternHeight, VERBOSITY_LEVEL>2?1:0, OPT::showFeatureExtraction, OPT::minMatches, flags);

		// next three lines can be replaced by multiCalib.run()
		multiCalib.loadImages();
		multiCalib.initialize();
		double rms = multiCalib.optimizeExtrinsics();

		multiCalib.writeParameters(strCalibration);

		VERBOSE("Multi-camera system calibrated: %u cameras, %u images, %gRMS (%s)", OPT::numCameras, fileNames.size()-1, rms, TD_TIMER_GET_FMT().c_str());
	}
	return EXIT_SUCCESS;
}

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	int ret;
	if (OPT::strInputFileName.IsEmpty()) {
		// generate pattern
		ret = GeneratePattern();
	} else {
		// calibrate multi-cameras system
		ret = CalibrateMultiCamera();
	}

	Finalize();
	return ret;
}
/*----------------------------------------------------------------*/
