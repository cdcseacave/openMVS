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
#undef D2R
#undef R2D
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/image/image.hpp>
#include <openMVG/third_party/stlplus3/filesystemSimplified/file_system.hpp>
#include <openMVG/third_party/progress/progress.hpp>


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceOpenMVG2")
#define MVS_EXT _T(".mvs")
#define PLY_EXT _T(".ply")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
bool bOpenMVS2PLY; // conversion direction
bool bNormalizeIntrinsics;
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
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the OpenMVS scene")
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
	if (OPT::strInputFileName.IsEmpty())
		return false;
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	OPT::bOpenMVS2PLY = (Util::getFileExt(OPT::strInputFileName) == MVS_EXT);
	if (OPT::bOpenMVS2PLY) {
		if (OPT::strOutputFileName.IsEmpty())
			OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + PLY_EXT;
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

	if (OPT::bOpenMVS2PLY) {
		// read OpenMVS input data
		MVS::Scene scene(OPT::nMaxThreads);
		scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName));

		// Export the scene to a PLY file
		uint32_t nbcamera = 0;
		FOREACH(i, scene.images) {
			const MVS::Image& image = scene.images[i];
			if (image.poseID != NO_ID)
				++nbcamera;
	    }

		//Create the stream and check it is ok
		const std::string filename = MAKE_PATH_SAFE(OPT::strOutputFileName);
		std::ofstream stream(filename.c_str());
		if (!stream.is_open())
			return false;

		stream << "ply"
			<< '\n' << "format ascii 1.0"
			<< '\n' << "element vertex "
			<< (nbcamera + scene.pointcloud.GetSize())
			<< '\n' << "property float x"
			<< '\n' << "property float y"
			<< '\n' << "property float z"
			<< '\n' << "property uchar red"
			<< '\n' << "property uchar green"
			<< '\n' << "property uchar blue"
			<< '\n' << "end_header" << std::endl;

		FOREACH(i, scene.images) {
			const MVS::Image& image = scene.images[i];
			if (image.poseID != NO_ID)
				stream << image.camera.C[0] << " " << image.camera.C[1] << " " << image.camera.C[2] << " 0 255 0" << "\n";
		}
		FOREACH(p, scene.pointcloud.points) {
			const MVS::PointCloud::Point& point = scene.pointcloud.points[p];
			stream << point[0] << " "  << point[1] << " " << point[2] << " 255 255 255" << "\n";
		}
		stream.flush();
		stream.close();

		VERBOSE("Input data exported: %u poses & %u vertices", nbcamera, scene.pointcloud.GetSize());
		} else {
			// read OpenMVG input data
			using namespace openMVG::sfm;
			using namespace openMVG::cameras;
			SfM_Data sfm_data;
			std::string sSfM_Data_Filename = MAKE_PATH_SAFE(OPT::strInputFileName);
			if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL))) {
				VERBOSE("error: the input SfM_Data file '%s' cannot be read", sSfM_Data_Filename.c_str());
				return EXIT_FAILURE;
		}

		VERBOSE("Imported data: %u cameras, %u poses, %u images, %u vertices",
				sfm_data.GetIntrinsics().size(),
				sfm_data.GetPoses().size(),
				sfm_data.GetViews().size(),
				sfm_data.GetLandmarks().size());

		// convert data from OpenMVG to OpenMVS
		MVS::Scene scene(OPT::nMaxThreads);

		// OpenMVG can have not contigous index, use a map to create the required OpenMVS contiguous ID index
		std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

		// Define a platform with all the intrinsic group
		for (const auto& intrinsic: sfm_data.GetIntrinsics()) {
			if (isPinhole(intrinsic.second.get()->getType())) {
				const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());

				MVS::Platform& platform =
					(scene.platforms.GetSize() == 0) ? scene.platforms.AddEmpty() : scene.platforms[0];

				if (map_intrinsic.count(intrinsic.first) == 0)
					map_intrinsic.insert(std::make_pair(intrinsic.first, platform.cameras.GetSize()));

				// Add the camera
				MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
				camera.K = cam->K();
				// Sub-pose
				camera.R = RMatrix::IDENTITY;
				camera.C = CMatrix::ZERO;
			}
		}

		// Create output directory for the undistorted images
		if (!stlplus::is_folder(MAKE_PATH_FULL(WORKING_FOLDER_FULL, std::string("images")))) {
				stlplus::folder_create(MAKE_PATH_FULL(WORKING_FOLDER_FULL, std::string("images")));
		}

		// Define images & poses
		C_Progress_display progress_bar( sfm_data.GetViews().size()*1);
		scene.images.Reserve(sfm_data.GetViews().size());
		for (const auto& view : sfm_data.GetViews()) {
			++progress_bar;
			map_view[view.first] = scene.images.GetSize();
			MVS::Image& image = scene.images.AddEmpty();
			image.name = "images/" + view.second->s_Img_path;
			Util::ensureUnifySlash(image.name);
			image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);
			image.platformID = 0;
			MVS::Platform& platform = scene.platforms[image.platformID];
			image.cameraID = map_intrinsic.at(view.second->id_intrinsic);

			openMVG::image::Image<openMVG::image::RGBColor> imageRGB, imageRGB_ud;
			const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view.second->s_Img_path);
			const std::string dstImage = image.name;

			if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
				image.poseID = platform.poses.GetSize();
				MVS::Platform::Pose& pose = platform.poses.AddEmpty();
				const openMVG::geometry::Pose3 poseMVG = sfm_data.GetPoseOrDie(view.second.get());
				pose.R = poseMVG.rotation();
				pose.C = poseMVG.center();
				// export undistorted images
				const openMVG::cameras::IntrinsicBase * cam = sfm_data.GetIntrinsics().at(view.second->id_intrinsic).get();
				if (cam->have_disto())  {
					// Undistort and save the image
					openMVG::image::ReadImage(srcImage.c_str(), &imageRGB);
					openMVG::cameras::UndistortImage(imageRGB, cam, imageRGB_ud, openMVG::image::BLACK);
					openMVG::image::WriteImage(dstImage.c_str(), imageRGB_ud);
				} else  { // (no distortion)
					// If extensions match, copy the image
					if (stlplus::extension_part(srcImage) == "JPG" ||
					stlplus::extension_part(srcImage) == "jpg")
					{
						stlplus::file_copy(srcImage, dstImage);
					} else  {
					openMVG::image::ReadImage( srcImage.c_str(), &imageRGB);
					openMVG::image::WriteImage( dstImage.c_str(), imageRGB);
					}
				}
			} else {
				// Image have not valid pose, so set an undefined pose
				image.poseID = NO_ID;
				// export the corresponding image
				if (stlplus::extension_part(srcImage) == "JPG" ||
						stlplus::extension_part(srcImage) == "jpg")
				{
					stlplus::file_copy(srcImage, dstImage);
				} else  {
					openMVG::image::ReadImage( srcImage.c_str(), &imageRGB);
					openMVG::image::WriteImage( dstImage.c_str(), imageRGB);
				}
			}
		}

		// Define structure
		scene.pointcloud.points.Reserve(sfm_data.GetLandmarks().size());
		scene.pointcloud.pointViews.Reserve(sfm_data.GetLandmarks().size());
		for (const auto& vertex: sfm_data.GetLandmarks()) {
			const Landmark & landmark = vertex.second;
			MVS::PointCloud::Point& point = scene.pointcloud.points.AddEmpty();
			point = landmark.X.cast<float>();
			MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews.AddEmpty();
			for (const auto& observation: landmark.obs) {
				views.Insert(map_view.at(observation.first));
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
						if (pImg->platformID == p && pImg->cameraID == c) {
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
				scene.platforms.GetSize(),
				(scene.platforms.GetSize() == 0) ? 0 : scene.platforms[0].cameras.GetSize(),
				(scene.platforms.GetSize() == 0) ? 0 : scene.platforms[0].poses.GetSize(),
				scene.images.GetSize(),
				scene.pointcloud.GetSize(),
				TD_TIMER_GET_FMT().c_str());
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
