/*
 * InterfaceOpenMVG.cpp
 *
 * Copyright (c) 2014-2015 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *      Pierre MOULON <p.moulon@foxel.ch>
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
#ifdef _USE_OPENMVG
#undef D2R
#undef R2D
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/image/image.hpp>
#endif


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceOpenMVG")
#define MVS_EXT _T(".mvs")
#define MVG_EXT _T(".baf")
#define MVG2_EXT _T(".json")
#define MVG3_EXT _T(".bin")


// S T R U C T S ///////////////////////////////////////////////////

namespace openMVS {
namespace MVS_IO {

typedef REAL RealT;
typedef Eigen::Matrix<RealT,3,3,Eigen::RowMajor> Mat33;
typedef Eigen::Matrix<RealT,3,1> Vec3;

// Structure to model the pinhole camera projection model
struct Camera
{
	Mat33 K; // camera's normalized intrinsics matrix
};
typedef std::vector<Camera> vec_Camera;

// structure describing a pose along the trajectory of a platform
struct Pose {
	Mat33 R;  // pose's rotation matrix
	Vec3 C;   // pose's translation vector
};
typedef std::vector<Pose> vec_Pose;

// structure describing an image
struct Image {
	uint32_t id_camera; // ID of the associated camera on the associated platform
	uint32_t id_pose;   // ID of the pose of the associated platform
	std::string name;   // image file name
};
typedef std::vector<Image> vec_Image;

// structure describing a 3D point
struct Vertex {

	typedef std::vector<uint32_t> vec_View;

	Vec3 X; // 3D point position
	vec_View views; // view visibility for this 3D feature
};
typedef std::vector<Vertex> vec_Vertex;

struct SfM_Scene
{
	vec_Pose poses;       // array of poses
	vec_Camera cameras;   // array of cameras
	vec_Image images;     // array of images
	vec_Vertex vertices;  // array of reconstructed 3D points
};


bool ImportScene(const std::string& sList_filename, const std::string& sBaf_filename, SfM_Scene& sceneBAF)
{
	LOG_OUT() << "Reading:\n"
		<< sList_filename << "\n"
		<< sBaf_filename << std::endl;

	// Read view list file (view filename, id_intrinsic, id_pose)
	// Must be read first, since it allow to establish the link between the ViewId and the camera/poses ids.
	std::map< std::pair<uint32_t, uint32_t>, uint32_t > map_cam_pose_toViewId;
	{
		std::ifstream file(sList_filename.c_str());
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", sList_filename.c_str());
			return false;
		}
		Image image;
		uint32_t count = 0;
		while (file >> image.name >> image.id_camera >> image.id_pose) {
			sceneBAF.images.push_back(image);
			map_cam_pose_toViewId[std::make_pair(image.id_camera, image.id_pose)] = count++;
			LOG_OUT() << image.name << ' ' << image.id_camera << ' ' << image.id_pose << std::endl;
		}
	}

	// Read BAF file
	{
		std::ifstream file(sBaf_filename.c_str());
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", sBaf_filename.c_str());
			return false;
		}

		uint32_t num_intrinsics, num_poses, num_points;

		// Read header
		file >> num_intrinsics;
		file >> num_poses;
		file >> num_points;

		LOG_OUT() << "Reading BAF file with:\n"
			<< " num_intrinsics: " << num_intrinsics << "\n"
			<< " num_poses: " << num_poses << "\n"
			<< " num_points: " << num_points << "\n";

		// Read the intrinsics (only support reading Pinhole Radial 3).
		{
			for (uint32_t i = 0; i < num_intrinsics; ++i) {
				double focal, ppx, ppy, k1, k2, k3;
				file >> focal >> ppx >> ppy >> k1 >> k2 >> k3;
				Camera cam;
				cam.K <<
					focal, 0, ppx,
					0, focal, ppy,
					0, 0, 1;
				LOG_OUT() << "\n" << cam.K << std::endl;
				sceneBAF.cameras.push_back(cam);
			}
		}

		// Read poses
		{
			for (uint32_t i = 0; i < num_poses; ++i) {
				Pose pose;
				for (int r = 0; r < 3; ++r) {
					for (int c = 0; c < 3; ++c) {
						file >> pose.R(r,c);
					}
				}
				file >> pose.C[0] >> pose.C[1] >> pose.C[2];
				#ifndef _RELEASE
				LOG_OUT() << "\n" << pose.R << "\n\n" << pose.C.transpose() << std::endl;
				#endif
				sceneBAF.poses.push_back(pose);
			}
		}

		// Read structure and visibility
		{
			#ifdef _RELEASE
			Util::Progress progress(_T("Processed points"), num_points);
			#endif
			for (uint32_t i = 0; i < num_points; ++i) {
				Vertex vertex;
				file >> vertex.X[0] >> vertex.X[1] >> vertex.X[2];
				uint32_t num_observations_for_point = 0;
				file >> num_observations_for_point;
				for (uint32_t j = 0; j < num_observations_for_point; ++j) {
					uint32_t id_intrinsics, id_pose;
					double x, y;
					file >> id_intrinsics >> id_pose >> x >> y;
					#ifndef _RELEASE
					LOG_OUT() << "observation:"
						<< " " <<  id_intrinsics
						<< " " <<  id_pose
						<< " " << x << " " << y << std::endl;
					#endif
					const auto itIntrPose(map_cam_pose_toViewId.find(std::make_pair(id_intrinsics, id_pose)));
					if (itIntrPose == map_cam_pose_toViewId.end()) {
						LOG_OUT() << "error: intrinsics-pose pair not existing" << std::endl;
						continue;
					}
					const uint32_t id_view(itIntrPose->second);
					vertex.views.push_back(id_view);
				}
				sceneBAF.vertices.push_back(vertex);
				#ifdef _RELEASE
				progress.display(i);
				#endif
			}
			#ifdef _RELEASE
			progress.close();
			#endif
		}
	}
	return true;
}

bool ExportScene(const std::string& sList_filename, const std::string& sBaf_filename, const SfM_Scene& sceneBAF)
{
	LOG_OUT() << "Writing:\n"
		<< sList_filename << "\n"
		<< sBaf_filename << std::endl;

	// Write view list file (view filename, id_intrinsic, id_pose)
	{
		std::ofstream file(sList_filename.c_str());
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", sList_filename.c_str());
			return false;
		}
		for (uint32_t i=0; i<sceneBAF.images.size(); ++i) {
			const Image& image = sceneBAF.images[i];
			file << image.name << ' ' << image.id_camera << ' ' << image.id_pose << std::endl;
			LOG_OUT() << image.name << ' ' << image.id_camera << ' ' << image.id_pose << std::endl;
		}
	}

	// Write BAF file
	{
		std::ofstream file(sBaf_filename.c_str());
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", sBaf_filename.c_str());
			return false;
		}

		const uint32_t num_intrinsics = (uint32_t)sceneBAF.cameras.size();
		const uint32_t num_poses = (uint32_t)sceneBAF.poses.size();
		const uint32_t num_points = (uint32_t)sceneBAF.vertices.size();

		LOG_OUT() << "Writing BAF file with:\n"
			<< " num_intrinsics: " << num_intrinsics << "\n"
			<< " num_poses: " << num_poses << "\n"
			<< " num_points: " << num_points << "\n";

		// Write header
		file << num_intrinsics << std::endl;
		file << num_poses << std::endl;
		file << num_points << std::endl;

		// Write the intrinsics (only support writing Pinhole Radial 3).
		{
			for (uint32_t i = 0; i < num_intrinsics; ++i) {
				const Camera& cam = sceneBAF.cameras[i];
				file << cam.K(0,0) << ' ' << cam.K(0,2) << ' ' << cam.K(1,2) << ' ' << 0 << ' ' << 0 << ' ' << 0 << std::endl;
				LOG_OUT() << "\n" << cam.K << std::endl;
			}
		}

		// Write poses
		{
			for (uint32_t i = 0; i < num_poses; ++i) {
				const Pose& pose = sceneBAF.poses[i];
				for (int r = 0; r < 3; ++r) {
					for (int c = 0; c < 3; ++c) {
						file << pose.R(r,c) << ' ';
					}
				}
				file << pose.C[0] << ' ' << pose.C[1] << ' ' << pose.C[2] << std::endl;
				#ifndef _RELEASE
				LOG_OUT() << "\n" << pose.R << "\n\n" << pose.C.transpose() << std::endl;
				#endif
			}
		}

		// Write structure and visibility
		{
			#ifdef _RELEASE
			Util::Progress progress(_T("Processed points"), num_points);
			#endif
			for (uint32_t i = 0; i < num_points; ++i) {
				const Vertex& vertex = sceneBAF.vertices[i];
				file << vertex.X[0] << ' ' << vertex.X[1] << ' ' << vertex.X[2] << std::endl;
				const uint32_t num_observations_for_point = (uint32_t)vertex.views.size();
				file << num_observations_for_point << std::endl;
				for (uint32_t j = 0; j < num_observations_for_point; ++j) {
					const uint32_t id_view = vertex.views[j];
					const Image& image = sceneBAF.images[id_view];
					file << image.id_camera << ' ' << image.id_pose << ' ' << 0 << ' ' << 0 << std::endl;
					#ifndef _RELEASE
					LOG_OUT() << "observation:"
						<< " " <<  image.id_camera
						<< " " <<  image.id_pose << std::endl;
					#endif
				}
				#ifdef _RELEASE
				progress.display(i);
				#endif
			}
			#ifdef _RELEASE
			progress.close();
			#endif
		}
	}
	return true;
}
} // MVS_IO
} // openMVS


namespace OPT {
#ifdef _USE_OPENMVG
bool bOpenMVGjson; // new import format
#endif
bool bOpenMVS2OpenMVG; // conversion direction
bool bNormalizeIntrinsics;
String strListFileName;
String strInputFileName;
String strOutputFileName;
String strOutputImageFolder;
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
		("images-list-file,l", boost::program_options::value<std::string>(&OPT::strListFileName), "input filename containing image list")
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("output-image-folder", boost::program_options::value<std::string>(&OPT::strOutputImageFolder)->default_value("undistorted_images"), "output folder to store undistorted images")
		("normalize,f", boost::program_options::value<bool>(&OPT::bNormalizeIntrinsics)->default_value(true), "normalize intrinsics while exporting to OpenMVS format")
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
	Util::ensureValidPath(OPT::strListFileName);
	Util::ensureUnifySlash(OPT::strListFileName);
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strOutputImageFolder);
	Util::ensureDirectorySlash(OPT::strOutputImageFolder);
	const String strInputFileNameExt(Util::getFileExt(OPT::strInputFileName).ToLower());
	OPT::bOpenMVS2OpenMVG = (strInputFileNameExt == MVS_EXT);
	#ifdef _USE_OPENMVG
	OPT::bOpenMVGjson = (strInputFileNameExt == MVG2_EXT || strInputFileNameExt == MVG3_EXT);
	const bool bInvalidCommand(OPT::strInputFileName.IsEmpty() || (OPT::strListFileName.IsEmpty() && !OPT::bOpenMVGjson && !OPT::bOpenMVS2OpenMVG));
	#else
	const bool bInvalidCommand(OPT::strInputFileName.IsEmpty() || (OPT::strListFileName.IsEmpty() && !OPT::bOpenMVS2OpenMVG));
	#endif
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
		#ifndef _USE_OPENMVG
		GET_LOG() << "\nWARNING: Only " MVG_EXT " files supported! In order to import " MVG2_EXT " or " MVG3_EXT " files use the converter included in OpenMVG, or link OpenMVS to the latest version of OpenMVG during compile time.\n";
		#endif
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::bOpenMVS2OpenMVG) {
		if (OPT::strOutputFileName.IsEmpty())
			OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName);
	} else {
		if (OPT::strOutputFileName.IsEmpty())
			OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + MVS_EXT;
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

	TD_TIMER_START();

	if (OPT::bOpenMVS2OpenMVG) {
		// read OpenMVS input data
		MVS::Scene scene(OPT::nMaxThreads);
		if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName)))
			return EXIT_FAILURE;

		// convert data from OpenMVS to OpenMVG
		openMVS::MVS_IO::SfM_Scene sceneBAF;
		FOREACH(p, scene.platforms) {
			const MVS::Platform& platform = scene.platforms[p];
			if (platform.cameras.GetSize() != 1) {
				LOG("error: unsupported scene structure");
				return EXIT_FAILURE;
			}
			const MVS::Platform::Camera& camera = platform.cameras[0];
			openMVS::MVS_IO::Camera cameraBAF;
			cameraBAF.K = camera.K;
			sceneBAF.cameras.push_back(cameraBAF);
		}
		FOREACH(i, scene.images) {
			const MVS::Image& image = scene.images[i];
			const MVS::Platform& platform = scene.platforms[image.platformID];
			const MVS::Platform::Pose& pose = platform.poses[image.poseID];
			openMVS::MVS_IO::Image imageBAF;
			imageBAF.name = image.name;
			imageBAF.name = MAKE_PATH_REL(WORKING_FOLDER_FULL, imageBAF.name);
			imageBAF.id_camera = image.platformID;
			imageBAF.id_pose = (uint32_t)sceneBAF.poses.size();
			sceneBAF.images.push_back(imageBAF);
			openMVS::MVS_IO::Pose poseBAF;
			poseBAF.R = pose.R;
			poseBAF.C = pose.C;
			sceneBAF.poses.push_back(poseBAF);
		}
		sceneBAF.vertices.reserve(scene.pointcloud.points.GetSize());
		FOREACH(p, scene.pointcloud.points) {
			const MVS::PointCloud::Point& point = scene.pointcloud.points[p];
			openMVS::MVS_IO::Vertex vertexBAF;
			vertexBAF.X = ((const MVS::PointCloud::Point::EVec)point).cast<REAL>();
			const MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[p];
			FOREACH(v, views) {
				unsigned viewBAF = views[(uint32_t)v];
				vertexBAF.views.push_back(viewBAF);
			}
			sceneBAF.vertices.push_back(vertexBAF);
		}

		// write OpenMVG input data
		const String strOutputFileNameMVG(OPT::strOutputFileName + MVG_EXT);
		openMVS::MVS_IO::ExportScene(MAKE_PATH_SAFE(OPT::strListFileName), MAKE_PATH_SAFE(strOutputFileNameMVG), sceneBAF);

		VERBOSE("Input data exported: %u cameras & %u poses & %u images & %u vertices (%s)", sceneBAF.cameras.size(), sceneBAF.poses.size(), sceneBAF.images.size(), sceneBAF.vertices.size(), TD_TIMER_GET_FMT().c_str());
	} else {
		// convert data from OpenMVG to OpenMVS
		MVS::Scene scene(OPT::nMaxThreads);
		size_t nCameras(0), nPoses(0);

	#ifdef _USE_OPENMVG
	if (OPT::bOpenMVGjson) {
		// read OpenMVG input data from a JSON file
		using namespace openMVG::sfm;
		using namespace openMVG::cameras;
		SfM_Data sfm_data;
		const String strSfM_Data_Filename(MAKE_PATH_SAFE(OPT::strInputFileName));
		if (!Load(sfm_data, strSfM_Data_Filename, ESfM_Data(ALL))) {
			VERBOSE("error: the input SfM_Data file '%s' cannot be read", strSfM_Data_Filename.c_str());
			return EXIT_FAILURE;
		}
		VERBOSE("Imported data: %u cameras, %u poses, %u images, %u vertices",
				sfm_data.GetIntrinsics().size(),
				sfm_data.GetPoses().size(),
				sfm_data.GetViews().size(),
				sfm_data.GetLandmarks().size());

		// OpenMVG can have not contiguous index, use a map to create the required OpenMVS contiguous ID index
		std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

		// define a platform with all the intrinsic group
		nCameras = sfm_data.GetIntrinsics().size();
		for (const auto& intrinsic: sfm_data.GetIntrinsics()) {
			if (isPinhole(intrinsic.second.get()->getType())) {
				const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());
				if (map_intrinsic.count(intrinsic.first) == 0)
					map_intrinsic.insert(std::make_pair(intrinsic.first, scene.platforms.GetSize()));
				MVS::Platform& platform = scene.platforms.AddEmpty();
				// add the camera
				MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
				camera.K = cam->K();
				// sub-pose
				camera.R = RMatrix::IDENTITY;
				camera.C = CMatrix::ZERO;
			}
		}

		// define images & poses
		const uint32_t nViews((uint32_t)sfm_data.GetViews().size());
		scene.images.Reserve(nViews);
		scene.nCalibratedImages = 0;
		Util::Progress progress(_T("Processed images"), nViews);
		GET_LOGCONSOLE().Pause();
		#ifdef _USE_OPENMP
		const std::vector<Views::value_type> views(sfm_data.GetViews().cbegin(), sfm_data.GetViews().cend());
		#pragma omp parallel for schedule(dynamic)
		for (int i=0; i<(int)nViews; ++i) {
			const Views::value_type& view = views[i];
			#pragma omp critical
			map_view[view.first] = scene.images.GetSize();
		#else
		for (const auto& view : sfm_data.GetViews()) {
			map_view[view.first] = scene.images.GetSize();
		#endif
			MVS::Image& image = scene.images.AddEmpty();
			image.name = view.second->s_Img_path;
			Util::ensureUnifySlash(image.name);
			Util::strTrim(image.name, PATH_SEPARATOR_STR);
			String pathRoot(sfm_data.s_root_path); Util::ensureDirectorySlash(pathRoot);
			const String srcImage(MAKE_PATH_FULL(WORKING_FOLDER_FULL, pathRoot+image.name));
			image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputImageFolder+image.name);
			Util::ensureDirectory(image.name);
			image.ID = static_cast<MVS::IIndex>(view.first);
			image.platformID = map_intrinsic.at(view.second->id_intrinsic);
			MVS::Platform& platform = scene.platforms[image.platformID];
			image.cameraID = 0;
			if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()) &&
				File::access(srcImage)) {
				MVS::Platform::Pose* pPose;
				#ifdef _USE_OPENMP
				#pragma omp critical
				#endif
				{
				image.poseID = platform.poses.GetSize();
				pPose = &platform.poses.AddEmpty();
				++scene.nCalibratedImages;
				}
				const openMVG::geometry::Pose3 poseMVG(sfm_data.GetPoseOrDie(view.second.get()));
				pPose->R = poseMVG.rotation();
				pPose->C = poseMVG.center();
				// export undistorted images
				const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view.second->id_intrinsic).get();
				if (cam->have_disto())  {
					// undistort and save the image
					openMVG::image::Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
					openMVG::image::ReadImage(srcImage, &imageRGB);
					openMVG::cameras::UndistortImage(imageRGB, cam, imageRGB_ud, openMVG::image::BLACK);
					openMVG::image::WriteImage(image.name, imageRGB_ud);
				} else  {
					// no distortion, copy the image
					File::copyFile(srcImage, image.name);
				}
				++nPoses;
			} else {
				// image have not valid pose, so set an undefined pose
				image.poseID = NO_ID;
				// just copy the image
				File::copyFile(srcImage, image.name);
				DEBUG_EXTRA("warning: uncalibrated image '%s'", view.second->s_Img_path.c_str());
			}
			++progress;
		}
		GET_LOGCONSOLE().Play();
		progress.close();

		// define structure
		scene.pointcloud.points.Reserve(sfm_data.GetLandmarks().size());
		scene.pointcloud.pointViews.Reserve(sfm_data.GetLandmarks().size());
		for (const auto& vertex: sfm_data.GetLandmarks()) {
			const Landmark & landmark = vertex.second;
			MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews.AddEmpty();
			for (const auto& observation: landmark.obs) {
				const auto it(map_view.find(observation.first));
				if (it != map_view.end())
					views.InsertSort(it->second);
			}
			if (views.GetSize() < 2) {
				scene.pointcloud.pointViews.RemoveLast();
				continue;
			}
			MVS::PointCloud::Point& point = scene.pointcloud.points.AddEmpty();
			point = landmark.X.cast<float>();
		}
	} else
	#endif
	{
		// read OpenMVG input data from BAF file
		openMVS::MVS_IO::SfM_Scene sceneBAF;
		if (!openMVS::MVS_IO::ImportScene(MAKE_PATH_SAFE(OPT::strListFileName), MAKE_PATH_SAFE(OPT::strInputFileName), sceneBAF))
			return EXIT_FAILURE;

		// convert data from OpenMVG to OpenMVS
		nCameras = sceneBAF.cameras.size();
		scene.platforms.Reserve((uint32_t)nCameras);
		for (const auto& cameraBAF: sceneBAF.cameras) {
			MVS::Platform& platform = scene.platforms.AddEmpty();
			MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
			camera.K = cameraBAF.K;
			camera.R = RMatrix::IDENTITY;
			camera.C = CMatrix::ZERO;
		}
		nPoses = sceneBAF.images.size();
		scene.images.Reserve((uint32_t)nPoses);
		for (const auto& imageBAF: sceneBAF.images) {
			openMVS::MVS_IO::Pose& poseBAF = sceneBAF.poses[imageBAF.id_pose];
			MVS::Image& image = scene.images.AddEmpty();
			image.name = imageBAF.name;
			Util::ensureUnifySlash(image.name);
			image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);
			image.ID = imageBAF.id_camera;
			image.platformID = imageBAF.id_camera;
			MVS::Platform& platform = scene.platforms[image.platformID];
			image.cameraID = 0;
			image.poseID = platform.poses.GetSize();
			MVS::Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = poseBAF.R;
			pose.C = poseBAF.C;
		}
		scene.pointcloud.points.Reserve(sceneBAF.vertices.size());
		scene.pointcloud.pointViews.Reserve(sceneBAF.vertices.size());
		for (const auto& vertexBAF: sceneBAF.vertices) {
			MVS::PointCloud::Point& point = scene.pointcloud.points.AddEmpty();
			point = vertexBAF.X.cast<float>();
			MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews.AddEmpty();
			for (const auto& viewBAF: vertexBAF.views)
				views.InsertSort(viewBAF);
		}
	}

		// read images meta-data
		FOREACHPTR(pImage, scene.images) {
			if (!pImage->ReloadImage(0, false))
				LOG("error: can not read image %s", pImage->name.c_str());
		}
		if (OPT::bNormalizeIntrinsics) {
			// normalize camera intrinsics
			FOREACH(p, scene.platforms) {
				MVS::Platform& platform = scene.platforms[p];
				FOREACH(c, platform.cameras) {
					MVS::Platform::Camera& camera = platform.cameras[c];
					// find one image using this camera
					MVS::Image* pImage(NULL);
					FOREACHPTR(pImg, scene.images) {
						if (pImg->platformID == p && pImg->cameraID == c && pImg->poseID != NO_ID) {
							pImage = pImg;
							break;
						}
					}
					if (pImage == NULL) {
						LOG("error: no image using camera %u of platform %u", c, p);
						continue;
					}
					const REAL fScale(REAL(1)/MVS::Camera::GetNormalizationScale(pImage->width, pImage->height));
					camera.K(0,0) *= fScale;
					camera.K(1,1) *= fScale;
					camera.K(0,2) *= fScale;
					camera.K(1,2) *= fScale;
				}
			}
		}

		// write OpenMVS input data
		scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

		VERBOSE("Exported data: %u platforms, %u cameras, %u poses, %u images, %u vertices (%s)",
				scene.platforms.GetSize(), nCameras, nPoses, scene.images.GetSize(), scene.pointcloud.GetSize(),
				TD_TIMER_GET_FMT().c_str());
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
