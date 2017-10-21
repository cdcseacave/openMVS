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
		("pattern-width,x", boost::program_options::value<float>(&OPT::patternWidth)->default_value(800), "physical width of random pattern")
		("pattern-height,y", boost::program_options::value<float>(&OPT::patternHeight)->default_value(600), "physical height of random pattern")
		("number-cameras,n", boost::program_options::value<int>(&OPT::numCameras)->default_value(0), "number of cameras to calibrate")
		("camera-type,t", boost::program_options::value<int>(&OPT::cameraType)->default_value(0), "camera type: 0 - pinhole, 1 - omnidirectional")
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

// generate random pattern used for calibration
int GeneratePattern()
{
	TD_TIMER_START();

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
	const std::vector<String> files(Util::getFolderWildcard(dir, wld, true, false, true));

	// creates a yaml or xml list of files
	const String strImagesList(Util::getFileFullName(MAKE_PATH_SAFE(OPT::strOutputFileName)) + _T("_images") LST_EXT);
	{
		cv::FileStorage fs(strImagesList, cv::FileStorage::WRITE);
		if (!fs.isOpened())
			return EXIT_FAILURE;
		fs << "images" << "[";
		fs << MAKE_PATH_SAFE(OPT::strPatternFileName);
		for (const String& fileName: files)
			fs << Util::getSimplifiedPath(dir+fileName);
		fs << "]";
	}

	// do multi-camera calibration
	cv::multicalib::MultiCameraCalibration multiCalib(OPT::cameraType, OPT::numCameras, strImagesList, OPT::patternWidth, OPT::patternHeight, VERBOSITY_LEVEL>2?1:0, OPT::showFeatureExtraction, OPT::minMatches);

	// next three lines can be replaced by multiCalib.run()
	multiCalib.loadImages();
	multiCalib.initialize();
	multiCalib.optimizeExtrinsics();

	const String strCalibration(Util::getFileFullName(MAKE_PATH_SAFE(OPT::strOutputFileName)) + LST_EXT);
	multiCalib.writeParameters(strCalibration);

	VERBOSE("Multi-camera system calibrated: %u cameras, %u images (%s)", OPT::numCameras, files.size(), TD_TIMER_GET_FMT().c_str());
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
