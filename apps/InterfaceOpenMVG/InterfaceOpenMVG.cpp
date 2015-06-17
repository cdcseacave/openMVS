/*
 * InterfaceOpenMVG.cpp
 *
 * Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
 * Please read <http://foxel.ch/license> for more information.
 *
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *      Pierre MOULON <p.moulon@foxel.ch>
 *
 *
 * This file is part of the FOXEL project <http://foxel.ch>.
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
 *
 *      You are required to attribute the work as explained in the "Usage and
 *      Attribution" section of <http://foxel.ch/license>.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceOpenMVG")
#define MVS_EXT _T(".mvs")
#define MVG_EXT _T(".baf")

#define WORKING_FOLDER		OPT::strWorkingFolder // empty by default (current folder)
#define WORKING_FOLDER_FULL	OPT::strWorkingFolderFull // full path to current folder


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
bool bOpenMVS2OpenMVG; // conversion direction
bool bNormalizeIntrinsics; // normalize K while exporting to OpenMVS format
String strListFileName; // image list file name
String strInputFileName; // BAF file name of the input data
String strOutputFileName; // file name of the mesh data
String strWorkingFolder; // empty by default (current folder)
String strWorkingFolderFull; // full path to current folder
} // namespace OPT


namespace openMVS {
namespace MVS_IO {

typedef REAL RealT;
typedef Eigen::Matrix<RealT, 3, 3> Mat33;
typedef Eigen::Matrix<RealT, 3, 1> Vec3;

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
			LOG_OUT() << "Unable to open file: " << sList_filename << std::endl;
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
			LOG_OUT() << "Unable to open file: " << sBaf_filename << std::endl;
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
				for (int j = 0; j < 9; ++j) {
					file >> pose.R.array()(j);
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
			LOG_OUT() << "Unable to open file: " << sList_filename << std::endl;
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
			LOG_OUT() << "Unable to open file: " << sBaf_filename << std::endl;
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
				for (int j = 0; j < 9; ++j) {
					file << pose.R.array()(j) << ' ';
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


// ParseCmdLine to parse the command line parameters
#define PARAM_EQUAL(name) (0 == _tcsncicmp(param, _T("-" #name "="), sizeof(_T("-" #name "="))-sizeof(TCHAR)))
bool ParseCmdLine(size_t argc, LPCTSTR* argv, String& strCmdLine)
{
	bool bPrintHelp(false);
	OPT::bNormalizeIntrinsics = true;
	for (size_t i=1; i<argc; ++i) {
		LPCSTR param = argv[i];
		strCmdLine += _T(" ") + String(param);
		if (PARAM_EQUAL(list) || PARAM_EQUAL(l)) {
			param = strchr(param, '=')+1;
			OPT::strListFileName = param;
		} else if (PARAM_EQUAL(input) || PARAM_EQUAL(i)) {
			param = strchr(param, '=')+1;
			OPT::strInputFileName = param;
		} else if (PARAM_EQUAL(output) || PARAM_EQUAL(o)) {
			param = strchr(param, '=')+1;
			OPT::strOutputFileName = param;
		} else if (PARAM_EQUAL(workdir) || PARAM_EQUAL(w)) {
			param = strchr(param, '=')+1;
			OPT::strWorkingFolder = param;
		} else if (PARAM_EQUAL(normalize) || PARAM_EQUAL(n)) {
			param = strchr(param, '=')+1;
			OPT::bNormalizeIntrinsics = (atoi(param) != 0);
		} else if (0 == _tcsicmp(param, _T("-help")) || 0 == _tcsicmp(param, _T("-h")) || 0 == _tcsicmp(param, _T("-?"))) {
			bPrintHelp = true;
		}
	}
	Util::ensureValidDirectoryPath(OPT::strWorkingFolder);
	OPT::strWorkingFolderFull = Util::getFullPath(OPT::strWorkingFolder);
	return bPrintHelp;
}

bool ValidateInput(bool bPrintHelp)
{
	Util::ensureValidPath(OPT::strListFileName);
	Util::ensureUnifySlash(OPT::strListFileName);
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (bPrintHelp || OPT::strListFileName.IsEmpty() || OPT::strInputFileName.IsEmpty())
		LOG("Available list of command-line parameters:\n"
			"\t-list= or -l=<list_filename> for setting the input filename containing image list\n"
			"\t-input= or -i=<input_filename> for setting the BAF input filename containing camera poses\n"
			"\t-output= or -o=<output_filename> for setting the output filename for storing the mesh and texture\n"
			"\t-workdir= or -w=<folder_path> for setting the working directory (default current directory)\n"
			"\t-normalize= or -n=<bool> normalize intrinsics while exporting to OpenMVS format (default true)\n"
			"\t-help or -h or -? for displaying this message"
		);
	if (OPT::strListFileName.IsEmpty() || OPT::strInputFileName.IsEmpty())
		return false;
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	OPT::bOpenMVS2OpenMVG = (Util::getFileExt(OPT::strInputFileName) == MVS_EXT);
	if (OPT::bOpenMVS2OpenMVG) {
		if (OPT::strOutputFileName.IsEmpty())
			OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + MVG_EXT;
	} else {
		if (OPT::strOutputFileName.IsEmpty())
			OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + MVS_EXT;
	}
	return true;
}


int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	{
	OPEN_LOG();
	OPEN_LOGCONSOLE();
	String strCmdLine;
	const bool bPrintHelp(ParseCmdLine(argc, argv, strCmdLine));
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	Util::LogBuild();
	LOG(_T("Command line:%s"), strCmdLine.c_str());
	if (!ValidateInput(bPrintHelp))
		return -1;
	#ifdef _USE_BREAKPAD
	MiniDumper::Create(APPNAME, WORKING_FOLDER);
	#endif
	}

	TD_TIMER_START();

	if (OPT::bOpenMVS2OpenMVG) {
		// read OpenMVS input data
		MVS::Scene scene;
		scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::strWorkingFolderFull);

		// convert data from OpenMVS to OpenMVG
		openMVS::MVS_IO::SfM_Scene sceneBAF;
		FOREACH(p, scene.platforms) {
			const MVS::Platform& platform = scene.platforms[p];
			if (platform.cameras.GetSize() != 1) {
				LOG("error: unsupported scene structure");
				return false;
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
			vertexBAF.X = ((const MVS::PointCloud::Position::EVec)point.X).cast<REAL>();
			FOREACH(v, point.views) {
				unsigned viewBAF = point.views[(uint32_t)v];
				vertexBAF.views.push_back(viewBAF);
			}
			sceneBAF.vertices.push_back(vertexBAF);
		}

		// write OpenMVG input data
		openMVS::MVS_IO::ExportScene(MAKE_PATH_SAFE(OPT::strListFileName), MAKE_PATH_SAFE(OPT::strOutputFileName), sceneBAF);

		VERBOSE("Input data exported: %u cameras & %u poses & %u images & %u vertices (%s)", sceneBAF.cameras.size(), sceneBAF.poses.size(), sceneBAF.images.size(), sceneBAF.vertices.size(), TD_TIMER_GET_FMT().c_str());
	} else {
		// read OpenMVG input data
		openMVS::MVS_IO::SfM_Scene sceneBAF;
		openMVS::MVS_IO::ImportScene(MAKE_PATH_SAFE(OPT::strListFileName), MAKE_PATH_SAFE(OPT::strInputFileName), sceneBAF);

		// convert data from OpenMVG to OpenMVS
		MVS::Scene scene;
		scene.platforms.Reserve(sceneBAF.cameras.size());
		for (const auto& cameraBAF: sceneBAF.cameras) {
			MVS::Platform& platform = scene.platforms.AddEmpty();
			MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
			camera.K = cameraBAF.K;
			camera.R = RMatrix::IDENTITY;
			camera.C = CMatrix::ZERO;
		}
		scene.images.Reserve(sceneBAF.images.size());
		for (const auto& imageBAF: sceneBAF.images) {
			openMVS::MVS_IO::Pose& poseBAF = sceneBAF.poses[imageBAF.id_pose];
			MVS::Image& image = scene.images.AddEmpty();
			image.name = imageBAF.name;
			Util::ensureUnifySlash(image.name);
			image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);
			image.platformID = imageBAF.id_camera;
			MVS::Platform& platform = scene.platforms[image.platformID];
			image.cameraID = 0;
			image.poseID = (uint32_t)platform.poses.GetSize();
			MVS::Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = poseBAF.R;
			pose.C = poseBAF.C;
		}
		scene.pointcloud.points.Reserve(sceneBAF.vertices.size());
		for (const auto& vertexBAF: sceneBAF.vertices) {
			MVS::PointCloud::Point& point = scene.pointcloud.points.AddEmpty();
			point.X = vertexBAF.X.cast<float>();
			for (const auto& viewBAF: vertexBAF.views) {
				point.views.Insert(viewBAF);
			}
		}
		if (OPT::bNormalizeIntrinsics) {
			FOREACH(p, scene.platforms) {
				MVS::Platform& platform = scene.platforms[p];
				FOREACH(c, platform.cameras) {
					MVS::Platform::Camera& camera = platform.cameras[c];
					// find one image using this camera
					MVS::Image* pImage(NULL);
					FOREACHPTR(pImg, scene.images) {
						if (pImg->platformID == p && pImg->cameraID == c) {
							pImage = pImg;
							break;
						}
					}
					if (pImage == NULL) {
						LOG("error: no image using camera %u of platform %u", c, p);
						continue;
					}
					if (!pImage->ReloadImage(0, false)) {
						LOG("error: can not read image %s", pImage->name.c_str());
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
		scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), OPT::strWorkingFolderFull);

		VERBOSE("Input data imported: %u cameras, %u poses, %u images, %u vertices (%s)", sceneBAF.cameras.size(), sceneBAF.poses.size(), sceneBAF.images.size(), sceneBAF.vertices.size(), TD_TIMER_GET_FMT().c_str());
	}

	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();

	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
