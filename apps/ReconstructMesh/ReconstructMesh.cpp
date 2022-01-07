/*
 * ReconstructMesh.cpp
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
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("ReconstructMesh")

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define RECMESH_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
bool bMeshExport;
float fDistInsert;
bool bUseConstantWeight;
bool bUseFreeSpaceSupport;
float fThicknessFactor;
float fQualityFactor;
float fDecimateMesh;
unsigned nTargetFaceNum;
float fRemoveSpurious;
bool bRemoveSpikes;
unsigned nCloseHoles;
unsigned nSmoothMesh;
float fSplitMaxArea;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
String strImagePointsFileName;
String strExportType;
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
		("export-type", boost::program_options::value<std::string>(&OPT::strExportType)->default_value(_T("ply")), "file type used to export the 3D scene (ply or obj)")
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
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config_main("Reconstruct options");
	config_main.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("min-point-distance,d", boost::program_options::value(&OPT::fDistInsert)->default_value(2.5f), "minimum distance in pixels between the projection of two 3D points to consider them different while triangulating (0 - disabled)")
		("constant-weight", boost::program_options::value(&OPT::bUseConstantWeight)->default_value(true), "considers all view weights 1 instead of the available weight")
		("free-space-support,f", boost::program_options::value(&OPT::bUseFreeSpaceSupport)->default_value(false), "exploits the free-space support in order to reconstruct weakly-represented surfaces")
		("thickness-factor", boost::program_options::value(&OPT::fThicknessFactor)->default_value(1.f), "multiplier adjusting the minimum thickness considered during visibility weighting")
		("quality-factor", boost::program_options::value(&OPT::fQualityFactor)->default_value(1.f), "multiplier adjusting the quality weight considered during graph-cut")
		;
	boost::program_options::options_description config_clean("Clean options");
	config_clean.add_options()
		("decimate", boost::program_options::value(&OPT::fDecimateMesh)->default_value(1.f), "decimation factor in range (0..1] to be applied to the reconstructed surface (1 - disabled)")
		("target-face-num", boost::program_options::value(&OPT::nTargetFaceNum)->default_value(0), "target number of faces to be applied to the reconstructed surface. (0 - disabled)")
		("remove-spurious", boost::program_options::value(&OPT::fRemoveSpurious)->default_value(20.f), "spurious factor for removing faces with too long edges or isolated components (0 - disabled)")
		("remove-spikes", boost::program_options::value(&OPT::bRemoveSpikes)->default_value(true), "flag controlling the removal of spike faces")
		("close-holes", boost::program_options::value(&OPT::nCloseHoles)->default_value(30), "try to close small holes in the reconstructed surface (0 - disabled)")
		("smooth", boost::program_options::value(&OPT::nSmoothMesh)->default_value(2), "number of iterations to smooth the reconstructed surface (0 - disabled)")
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("mesh-file", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name to clean (skips the reconstruction step)")
		("mesh-export", boost::program_options::value(&OPT::bMeshExport)->default_value(false), "just export the mesh contained in loaded project")
		("split-max-area", boost::program_options::value(&OPT::fSplitMaxArea)->default_value(0.f), "maximum surface area that a sub-mesh can contain (0 - disabled)")
		("image-points-file", boost::program_options::value<std::string>(&OPT::strImagePointsFileName), "input filename containing the list of points from an image to project on the mesh (optional)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config_main).add(config_clean).add(hidden);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config_main).add(config_clean).add(hidden);

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
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (OPT::vm.count("help") || OPT::strInputFileName.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config_main).add(config_clean);
		GET_LOG() << visible;
	}
	if (OPT::strInputFileName.IsEmpty())
		return false;
	OPT::strExportType = OPT::strExportType.ToLower() == _T("obj") ? _T(".obj") : _T(".ply");

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	Util::ensureValidPath(OPT::strImagePointsFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_mesh.mvs");

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


// export 3D coordinates corresponding to 2D ccordinates provided by inputFileName:
// parse image point list; first line is the name of the image to project,
// each consequent line store the xy coordinates to project:
// <image-name> <number-of-points>
// <x-coord1> <y-coord1>
// <x-coord2> <y-coord2>
// ...
// 
// for example:
// N01.JPG 3
// 3090 2680
// 3600 2100
// 3640 2190
bool Export3DProjections(Scene& scene, const String& inputFileName) {
	SML smlPointList(_T("ImagePoints"));
	smlPointList.Load(inputFileName);
	const LPSMLARR& arrSmlChild = smlPointList.GetArrChildren();
	ASSERT(arrSmlChild.size() <= 1);
	IDX idx(0);

	// read image name
	size_t argc;
	CAutoPtrArr<LPSTR> argv;
	while (true) {
		argv = Util::CommandLineToArgvA(smlPointList.GetValue(idx).val, argc);
		if (argc > 0 && argv[0][0] != _T('#'))
			break;
		if (++idx == smlPointList.size())
			return false;
	}
	if (argc < 2)
		return false;
	String imgName(argv[0]);
	IIndex imgID(NO_ID);
	for (const Image& imageData : scene.images) {
		if (!imageData.IsValid())
			continue;
		if (imageData.name.substr(imageData.name.size() - imgName.size()) == imgName) {
			imgID = imageData.ID;
			break;
		}
	}
	if (imgID == NO_ID) {
		VERBOSE("Unable to find image named: %s", imgName.c_str());
		return false;
	}

	// read image points
	std::vector<Point2f> imagePoints;
	while (++idx != smlPointList.size()) {
		// parse image element
		const String& line(smlPointList.GetValue(idx).val);
		argv = Util::CommandLineToArgvA(line, argc);
		if (argc > 0 && argv[0][0] == _T('#'))
			continue;
		if (argc < 2) {
			VERBOSE("Invalid image coordonates: %s", line.c_str());
			continue;
		}
		const Point2f pt(
			String::FromString<float>(argv[0], -1),
			String::FromString<float>(argv[1], -1));
		if (pt.x > 0 && pt.y > 0)
			imagePoints.emplace_back(pt);
	}
	if (imagePoints.empty()) {
		VERBOSE("Unable to read image points from: %s", imgName.c_str());
		return false;
	}

	// prepare output file
	String outFileName(Util::insertBeforeFileExt(inputFileName, "_3D"));
	File oStream(outFileName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!oStream.isOpen()) {
		VERBOSE("Unable to open output file: %s", outFileName.c_str());
		return false;
	}

	// print image name
	oStream.print("%s %u\n", imgName.c_str(), imagePoints.size());

	// init mesh octree
	const Mesh::Octree octree(scene.mesh.vertices, [](Mesh::Octree::IDX_TYPE size, Mesh::Octree::Type /*radius*/) {
		return size > 256;
	});
	scene.mesh.ListIncidenteFaces();

	// save 3D coord in the output file
	const Image& imgToExport = scene.images[imgID];
	for (const Point2f& pt : imagePoints) {
		// define ray from camera center to each x,y image coord
		const Ray3 ray(imgToExport.camera.C, normalized(imgToExport.camera.RayPoint<REAL>(pt)));
		// find ray intersection with the mesh
		const IntersectRayMesh intRay(octree, ray, scene.mesh);
		if (intRay.pick.IsValid()) {
			const Point3d ptHit(ray.GetPoint(intRay.pick.dist));
			oStream.print("%.7f %.7f %.7f\n", ptHit.x, ptHit.y, ptHit.z);
		} else 
			oStream.print("NA\n");
	}
	return true;
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
	// load project
	if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::fSplitMaxArea > 0 || OPT::fDecimateMesh < 1 || OPT::nTargetFaceNum > 0))
		return EXIT_FAILURE;
	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));
	if (OPT::fSplitMaxArea > 0) {
		// split mesh using max-area constraint
		Mesh::FacesChunkArr chunks;
		if (scene.mesh.Split(chunks, OPT::fSplitMaxArea))
			scene.mesh.Save(chunks, baseFileName);
		Finalize();
		return EXIT_SUCCESS;
	}

	if (!OPT::strImagePointsFileName.empty() && !scene.mesh.IsEmpty()) {
		Export3DProjections(scene, MAKE_PATH_SAFE(OPT::strImagePointsFileName));
		return EXIT_SUCCESS;
	}

	if (OPT::bMeshExport) {
		// check there is a mesh to export
		if (scene.mesh.IsEmpty())
			return EXIT_FAILURE;
		// save mesh
		const String fileName(MAKE_PATH_SAFE(OPT::strOutputFileName));
		scene.mesh.Save(fileName);
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			scene.ExportCamerasMLP(baseFileName+_T(".mlp"), fileName);
		#endif
	} else {
		if (OPT::strMeshFileName.IsEmpty() && scene.mesh.IsEmpty()) {
			// reset image resolution to the original size and
			// make sure the image neighbors are initialized before deleting the point-cloud
			#ifdef RECMESH_USE_OPENMP
			bool bAbort(false);
			#pragma omp parallel for
			for (int_t idx=0; idx<(int_t)scene.images.GetSize(); ++idx) {
				#pragma omp flush (bAbort)
				if (bAbort)
					continue;
				const uint32_t idxImage((uint32_t)idx);
			#else
			FOREACH(idxImage, scene.images) {
			#endif
				Image& imageData = scene.images[idxImage];
				if (!imageData.IsValid())
					continue;
				// reset image resolution
				if (!imageData.ReloadImage(0, false)) {
					#ifdef RECMESH_USE_OPENMP
					bAbort = true;
					#pragma omp flush (bAbort)
					continue;
					#else
					return EXIT_FAILURE;
					#endif
				}
				imageData.UpdateCamera(scene.platforms);
				// select neighbor views
				if (imageData.neighbors.IsEmpty()) {
					IndexArr points;
					scene.SelectNeighborViews(idxImage, points);
				}
			}
			#ifdef RECMESH_USE_OPENMP
			if (bAbort)
				return EXIT_FAILURE;
			#endif
			// reconstruct a coarse mesh from the given point-cloud
			TD_TIMER_START();
			if (OPT::bUseConstantWeight)
				scene.pointcloud.pointWeights.Release();
			if (!scene.ReconstructMesh(OPT::fDistInsert, OPT::bUseFreeSpaceSupport, 4, OPT::fThicknessFactor, OPT::fQualityFactor))
				return EXIT_FAILURE;
			VERBOSE("Mesh reconstruction completed: %u vertices, %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
			#if TD_VERBOSE != TD_VERBOSE_OFF
			if (VERBOSITY_LEVEL > 2) {
				// dump raw mesh
				scene.mesh.Save(baseFileName+_T("_raw")+OPT::strExportType);
			}
			#endif
		} else if (!OPT::strMeshFileName.IsEmpty()) {
			// load existing mesh to clean
			scene.mesh.Load(MAKE_PATH_SAFE(OPT::strMeshFileName));
		}

		// clean the mesh
		const float fDecimate(OPT::nTargetFaceNum ? static_cast<float>(OPT::nTargetFaceNum) / scene.mesh.faces.size() : OPT::fDecimateMesh);
		scene.mesh.Clean(fDecimate, OPT::fRemoveSpurious, OPT::bRemoveSpikes, OPT::nCloseHoles, OPT::nSmoothMesh, false);
		scene.mesh.Clean(1.f, 0.f, OPT::bRemoveSpikes, OPT::nCloseHoles, 0, false); // extra cleaning trying to close more holes
		scene.mesh.Clean(1.f, 0.f, false, 0, 0, true); // extra cleaning to remove non-manifold problems created by closing holes

		// save the final mesh
		scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
		scene.mesh.Save(baseFileName+OPT::strExportType);
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+OPT::strExportType);
		#endif
	}

	if (!OPT::strImagePointsFileName.empty()) {
		Export3DProjections(scene, MAKE_PATH_SAFE(OPT::strImagePointsFileName));
		return EXIT_SUCCESS;
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
