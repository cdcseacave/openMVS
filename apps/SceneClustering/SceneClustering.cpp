/*
 * SceneClustering.cpp
 *
 * Copyright (c) 2019 SEACAVE
 *
 * Author(s):
 *      Romain Janvier <romain.janvier@lepatriscope.com>
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
#include "domset/domset.h"
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("SceneClustering")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strInputFileName;
String strOutputDirectory;
float fVoxelSize;
unsigned nMinClusterSize;
unsigned nMaxClusterSize;
unsigned nClusterOverlap;
bool bDoSVM;
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
	boost::program_options::options_description config("Texture options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "Input filename containing camera poses and image list")
		("voxel-size,x", boost::program_options::value<float>(&OPT::fVoxelSize)->default_value(5.f), "Size of a cell in the voxel grid: level of simplification of the original point cloud")
		("min-cluster-size,m", boost::program_options::value<unsigned>(&OPT::nMinClusterSize)->default_value(30), "Min number of camera in a cluster" )
		("max-cluster-size,M", boost::program_options::value<unsigned>(&OPT::nMaxClusterSize)->default_value(50), "Max number of camera in a cluster")
		("cluster-overlap,o", boost::program_options::value<unsigned>(&OPT::nClusterOverlap)->default_value(4), "Number of views in overlap [NOT implemented yet]" )
		//("svm-classification,s", boost::program_options::bool_switch(&OPT::bDoSVM), "Do the SVM classification [NOT Implemented yet]" )
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
	LOG(_T("Command line:%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (OPT::vm.count("help") || OPT::strInputFileName.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}

	if (OPT::strInputFileName.IsEmpty())
		return false;
	
	// initialize optional options
	Util::ensureValidPath(OPT::strOutputDirectory);
	Util::ensureUnifySlash(OPT::strOutputDirectory);
	if (OPT::strOutputDirectory.IsEmpty())
		OPT::strOutputDirectory = WORKING_FOLDER;

	if(OPT::nMaxClusterSize < OPT::nMinClusterSize)
	{
		LOG("max-cluster-size value must be greater than min-cluster-size value");
		return false;
	}
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

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	Scene scene(OPT::nMaxThreads);
	// load and texture the mesh
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
		return EXIT_FAILURE;
	
	if (scene.pointcloud.IsEmpty()) {
		LOG("error: empty initial point-cloud");
		return EXIT_FAILURE;
	}


	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strInputFileName)));

	{
	// compute clustering
	TD_TIMER_START();

	// set up data
	std::vector<nomoko::Camera> domset_cameras; // Camera is useless in domset...
	std::vector<nomoko::View> domset_views;
	std::vector<nomoko::Point> domset_points; // nomoko::Point is an Eigen::Vec3f...
	std::map<uint32_t, uint32_t> view_fwd_reindexing; // get continuous indexing
	std::map<uint32_t, uint32_t> view_bkwd_reindexing; // reverse continuous indexing

	uint32_t curr_ID = 0;
	FOREACH(IdxC, scene.images)
	{
		const auto & curr_image = scene.images[IdxC];
		if(curr_image.IsValid())
		{
			view_fwd_reindexing[IdxC] = curr_ID;
			view_bkwd_reindexing[curr_ID] = IdxC; 
			nomoko::View v;
			v.rot = Eigen::Matrix<double,3,3,1>(curr_image.camera.R).cast<float>();
			v.trans = Eigen::Vector3d(curr_image.camera.C).cast<float>();
			domset_views.push_back(v);
			++curr_ID;
		}
	}

	FOREACH(IdxP, scene.pointcloud.points)
	{
		nomoko::Point p;
		p.pos = Eigen::Vector3f(scene.pointcloud.points[IdxP]);
		FOREACH(IdxV, scene.pointcloud.pointViews[IdxP])
		{
			auto idx = view_fwd_reindexing[scene.pointcloud.pointViews[IdxP][IdxV]];
			p.viewList.push_back(idx);
		}
		domset_points.push_back(p);
	}

	nomoko::Domset domset_instance(domset_points, domset_views, domset_cameras, OPT::fVoxelSize);

	domset_instance.clusterViews(OPT::nMinClusterSize, OPT::nMaxClusterSize, OPT::nClusterOverlap);

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
	{
		domset_instance.printClusters();
		domset_instance.exportToPLY(baseFileName + _T("_clusters.ply"));
	}
	#endif

	const auto domset_clusters = domset_instance.getClusters();
	VERBOSE("Clustering completed : %u clusters (%s)", domset_clusters.size(), TD_TIMER_GET_FMT().c_str());

	for (int i = 0; i < domset_clusters.size(); ++i) 
	{
		const auto & cluster = domset_clusters[i];

		Scene scene_cluster;
		std::map<uint32_t, uint32_t> map_global_to_local;
		std::vector<uint32_t> global_IDs;
		scene_cluster.platforms = scene.platforms; // We copy all the plateforms for now, it's easier and harmless

		uint32_t local_ID = 0;
		for (const auto inCluster_ID : cluster)
		{
			const size_t global_ID = view_bkwd_reindexing[inCluster_ID];
			auto & image = scene.images[global_ID];
			image.ID = global_ID; 
			scene_cluster.images.Insert(scene.images[global_ID]);
			map_global_to_local[global_ID] = local_ID;
			global_IDs.push_back(global_ID);
			++local_ID;
		}

		FOREACH(IdxP, scene.pointcloud.points)
		{
			const auto & curr_viewArr = scene.pointcloud.pointViews[IdxP];
			PointCloud::ViewArr new_viewArr;
			
			for(const auto & idxVG: global_IDs)
			{
				if(curr_viewArr.FindFirst(idxVG) != PointCloud::ViewArr::NO_INDEX)
					new_viewArr.InsertSort(map_global_to_local.at(idxVG));
			}

			if(new_viewArr.GetSize() > 1)
			{
				scene_cluster.pointcloud.points.Insert(scene.pointcloud.points[IdxP]);
				scene_cluster.pointcloud.pointViews.Insert(new_viewArr);
				if(!scene.pointcloud.colors.IsEmpty())
					scene_cluster.pointcloud.colors.Insert(scene.pointcloud.colors[IdxP]);
				if(!scene.pointcloud.pointWeights.IsEmpty())
					scene_cluster.pointcloud.pointWeights.Insert(scene.pointcloud.pointWeights[IdxP]);
			}
		}
		LOG(String::FormatString("Saving cluster #%i", i));
		scene_cluster.Save(baseFileName + String::FormatString("_cluster_%i.mvs", i), (ARCHIVE_TYPE)OPT::nArchiveType);
		scene_cluster.pointcloud.Save(baseFileName + String::FormatString("_cluster_%i.ply", i));
	}
	
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
