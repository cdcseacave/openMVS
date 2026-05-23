/*
 * Viewer.cpp
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

#include "Common.h"
#include "Scene.h"
#include <boost/program_options.hpp>

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("Viewer")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strGeometryFileName;
String strOutputFileName;
String strScreenshotFileName;
String strViewFileName;
int nViewCamera;
String strShow;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
unsigned nMaxMemory;
String strExportType;
String strConfigFileName;
#if TD_VERBOSE != TD_VERBOSE_OFF
bool bLogFile;
#endif
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
	#ifndef _RELEASE
	OPEN_LOGCONSOLE();
	#endif

	// group of options allowed only on command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("export-type", boost::program_options::value<std::string>(&OPT::strExportType), "file type used to export the 3D scene (ply or obj)")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_MVS), "project archive type: -1-interface, 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value(&OPT::nProcessPriority)->default_value(0), "process priority (normal by default)")
		("max-threads", boost::program_options::value(&OPT::nMaxThreads)->default_value(0), "maximum number of threads that this process should use (0 - use all available cores)")
		("max-memory", boost::program_options::value(&OPT::nMaxMemory)->default_value(0), "maximum amount of memory in MB that this process should use (0 - use all available memory)")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("log-file", boost::program_options::value(&OPT::bLogFile)->default_value(false), "dump log to a file")
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
	boost::program_options::options_description config("Viewer options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input project filename containing camera poses and scene (point-cloud/mesh)")
		("geometry-file,g", boost::program_options::value<std::string>(&OPT::strGeometryFileName), "mesh or point-cloud with views file name (overwrite existing geometry)")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("screenshot-file,S", boost::program_options::value<std::string>(&OPT::strScreenshotFileName), "render the scene off-screen to this image file and exit (scriptable; extension selects the format, .png if omitted)")
		("view-file", boost::program_options::value<std::string>(&OPT::strViewFileName), "transform file controlling the screenshot viewpoint (12 or 16 whitespace-separated values, row-major camera-to-world); if omitted the default fitted view is used")
		("view-camera", boost::program_options::value(&OPT::nViewCamera)->default_value(-1), "set the screenshot viewpoint to this scene camera's pose for a natural upright framing (-1 disabled; out-of-range selects a central camera); overridden by --view-file")
		("screenshot-show", boost::program_options::value<std::string>(&OPT::strShow), "which layers to render in the screenshot, as a string of flags: p=point-cloud, m=mesh, t=textured, c=cameras, w=wireframe, b=bounding-box, u=UI overlay (e.g. 'p', 'm', 'mt', 'mu'); if omitted the interactive defaults are kept for the layers and the UI overlay is disabled")
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

	#if TD_VERBOSE != TD_VERBOSE_OFF
	// initialize the log file
	if (OPT::bLogFile)
		OPEN_LOGFILE((MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log"))).c_str());
	#endif

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	if (OPT::vm.count("help")) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << _T("\n"
			"Visualize any know point-cloud/mesh formats or MVS projects. Supply files through command line or Drag&Drop.\n")
			<< visible;
	}
	if (!OPT::strExportType.empty())
		OPT::strExportType = OPT::strExportType.ToLower() == _T("obj") ? _T(".obj") : _T(".ply");

	// initialize optional options
	Util::ensureValidPath(OPT::strGeometryFileName);
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureValidPath(OPT::strScreenshotFileName);
	Util::ensureValidPath(OPT::strViewFileName);

	MVS::Initialize(APPNAME, OPT::nMaxThreads, OPT::nProcessPriority);
	return true;
}

// finalize application instance
void Application::Finalize()
{
	MVS::Finalize();

	if (OPT::bLogFile)
		CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // unnamed namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index or use _CrtSetBreakAlloc() to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	// create viewer
	Scene viewer;
	if (!viewer.Initialize(cv::Size(1280, 720), APPNAME,
			OPT::strInputFileName.empty() ? OPT::strInputFileName : MAKE_PATH_SAFE(OPT::strInputFileName),
			OPT::strGeometryFileName.empty() ? OPT::strGeometryFileName : MAKE_PATH_SAFE(OPT::strGeometryFileName)))
		return EXIT_FAILURE;
	if (viewer.IsOpen() && !OPT::strOutputFileName.empty()) {
		// export the scene
		viewer.Export(MAKE_PATH_SAFE(OPT::strOutputFileName), OPT::strExportType.empty()?LPCTSTR(NULL):OPT::strExportType.c_str());
	}
	if (!OPT::strScreenshotFileName.empty()) {
		// scriptable mode: optionally set the viewpoint, capture one frame off-screen, then exit
		if (!viewer.IsOpen())
			return EXIT_FAILURE;
		bool includeUI = false;
		if (!OPT::strShow.empty()) {
			// select which render layers are visible in the screenshot
			Window& w = viewer.GetWindow();
			w.showPointCloud    = OPT::strShow.find('p') != std::string::npos;
			w.showMeshTextured  = OPT::strShow.find('t') != std::string::npos;
			// 't' is a modifier of mesh rendering: requesting textured implies mesh
			w.showMesh          = OPT::strShow.find('m') != std::string::npos || w.showMeshTextured;
			w.showCameras       = OPT::strShow.find('c') != std::string::npos;
			w.showMeshWireframe = OPT::strShow.find('w') != std::string::npos;
			w.showBounds        = OPT::strShow.find('b') != std::string::npos;
			includeUI           = OPT::strShow.find('u') != std::string::npos;
		}
		if (!OPT::strViewFileName.empty())
			viewer.SetViewFromFile(MAKE_PATH_SAFE(OPT::strViewFileName));
		else if (OPT::nViewCamera >= 0)
			viewer.SetViewFromCamera((unsigned)OPT::nViewCamera);
		viewer.GetWindow().RequestScreenshot(MAKE_PATH_SAFE(OPT::strScreenshotFileName), includeUI, true);
	}
	// enter viewer loop (returns immediately after the screenshot in scriptable mode)
	viewer.Run();
	return EXIT_SUCCESS;
}
#ifdef _WIN32
// bridge WinMain -> main()
int APIENTRY WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
	return main(__argc, const_cast<LPCTSTR*>(__argv));
}
#endif
/*----------------------------------------------------------------*/
