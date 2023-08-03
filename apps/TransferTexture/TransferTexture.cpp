#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include "../../libs/MVS/Mesh.h"

#include <boost/program_options.hpp>

#include <iostream>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("TransferTexture")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strSourceMeshFileName;
String strTargetMeshFileName;
String strOutputFileName;

//String strExportType;
String strConfigFileName;
unsigned nMaxThreads;

String hidden;

int nProcessPriority;


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
	boost::program_options::options_description config("Transfer texture options");
	config.add_options()
		("source,s", boost::program_options::value<std::string>(&OPT::strSourceMeshFileName), "Mesh to transfer the texture from")
		("target,t", boost::program_options::value<std::string>(&OPT::strTargetMeshFileName), "Mesh to transfer the texture to")
		("output,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "Output mesh, with texture")
		("process-priority", boost::program_options::value(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("max-threads", boost::program_options::value(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("hidden", boost::program_options::value<std::string>(&OPT::hidden), "")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config).add(hidden);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config).add(hidden);

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
	Util::ensureValidPath(OPT::strSourceMeshFileName);
	if (OPT::vm.count("help") || OPT::strSourceMeshFileName.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (OPT::strSourceMeshFileName.IsEmpty())
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strOutputFileName);

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

	Util::Init();
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

} // unnamed namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;


	std::cout << "Transferring texture" << std::endl;

	std::cout << "Initialization successful" << std::endl;
        std::cout << std::endl;

	std::cout << "Loading source mesh " << OPT::strSourceMeshFileName << std::endl;
	Mesh source;
	source.Load(OPT::strSourceMeshFileName);
	std::cout << "[OK] Loading source mesh " << std::endl;

	std::cout << "Loading target mesh " << OPT::strTargetMeshFileName << std::endl;
	Mesh target;
	target.Load(OPT::strTargetMeshFileName);
	std::cout << "[OK] Loading target mesh " << std::endl;

        std::cout << "Loading successful" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;

        std::cout << "Target has texture coords? " << target.HasTexture() << std::endl;
        std::cout << "Target texture empty? " << target.textureDiffuse.empty() << std::endl;
        std::cout << "Target texture size " << target.textureDiffuse.size() << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;


        source.Save("loaded_source.ply");
        target.Save("loaded_target.ply");


        source.TransferTexture(target, {});

        std::cout << "Target has texture coords? " << target.HasTexture() << std::endl;
        std::cout << "Target texture empty? " << target.textureDiffuse.empty() << std::endl;
        std::cout << "Target texture size " << target.textureDiffuse.size() << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;


        target.Save("transferred_target_to_source.ply");



	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
