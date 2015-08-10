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
#include <boost/program_options.hpp>
#undef D2R
#undef R2D
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
//#include <openMVG/sfm/sfm.hpp>


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceOpenMVG_2")
#define MVS_EXT _T(".mvs")
#define PLY_EXT _T(".ply")

#define WORKING_FOLDER		OPT::strWorkingFolder // empty by default (current folder)
#define WORKING_FOLDER_FULL	OPT::strWorkingFolderFull // full path to current folder

// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
bool bOpenMVS2PLY; // conversion direction
bool bNormalizeIntrinsics;
String strInputFileName;
String strOutputFileName;
String strWorkingFolder; // empty by default (current folder)
String strWorkingFolderFull; // full path to current folder
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
		("help", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&OPT::strWorkingFolder), "the working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "the file name containing program options")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value<int>(&g_nVerbosityLevel)->default_value(
			#if TD_VERBOSE == TD_VERBOSE_DEBUG
			3
			#else
			2
			#endif
			), "the verbosity level")
		#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "the input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "the output filename for storing the mesh")
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

		// initialize working folder
		Util::ensureValidDirectoryPath(OPT::strWorkingFolder);
		OPT::strWorkingFolderFull = Util::getFullPath(OPT::strWorkingFolder);

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
		MVS::Scene scene;
		scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::strWorkingFolderFull);

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
      {
        stream << image.camera.C[0] << " " << image.camera.C[1] << " " << image.camera.C[2]
          << " 0 255 0" << "\n";
      }
    }
    FOREACH(p, scene.pointcloud.points) {
			const MVS::PointCloud::Point& point = scene.pointcloud.points[p];
      stream << point.X[0] << " "  << point.X[1] << " " << point.X[2] << " 255 255 255" << "\n";
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
      std::cerr << std::endl
        << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
      return EXIT_FAILURE;
    }

    VERBOSE("Imported data: %u cameras, %u poses, %u images, %u vertices",
      sfm_data.GetIntrinsics().size(),
      sfm_data.GetPoses().size(),
      sfm_data.GetViews().size(),
      sfm_data.GetLandmarks().size());

    // convert data from OpenMVG to OpenMVS
    MVS::Scene scene;

    // OpenMVG can have not contigous index, use a map to create the required OpenMVS contiguous ID index
    std::map<openMVG::IndexT, uint32_t> map_intrinsic, map_view;

    // Define a platform with all the intrinsic group
		for (const auto& intrinsic: sfm_data.GetIntrinsics())
		{
			if(isPinhole(intrinsic.second.get()->getType()))
      {
        const Pinhole_Intrinsic * cam = dynamic_cast<const Pinhole_Intrinsic*>(intrinsic.second.get());

        MVS::Platform& platform =
          (scene.platforms.GetSize() == 0) ? scene.platforms.AddEmpty() : scene.platforms[0];

        if (map_intrinsic.count(intrinsic.first) == 0)
          map_intrinsic.insert(std::make_pair(intrinsic.first, platform.cameras.GetSize()));

        // Add the camera
        MVS::Platform::Camera& camera = platform.cameras.AddEmpty();
        camera.K = cam->K();
        // Subpose
        camera.R = RMatrix::IDENTITY;
        camera.C = CMatrix::ZERO;
      }
		}

		// Define images & poses
    scene.images.Reserve(sfm_data.GetViews().size());
    for (const auto& view : sfm_data.GetViews())
    {
      map_view[view.first] = scene.images.GetSize();
			MVS::Image& image = scene.images.AddEmpty();
			image.name = "images/" + view.second->s_Img_path;
			Util::ensureUnifySlash(image.name);
			image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, image.name);
			image.platformID = 0;
			MVS::Platform& platform = scene.platforms[image.platformID];
			//image.cameraID = map_intrinsic[view.second->id_intrinsic];
			image.cameraID = view.second->id_intrinsic;
			if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
			{
        image.poseID = platform.poses.GetSize();
        MVS::Platform::Pose& pose = platform.poses.AddEmpty();
        const openMVG::geometry::Pose3 poseMVG = sfm_data.GetPoses().at(view.second->id_pose);
        pose.R = poseMVG.rotation();
        pose.C = poseMVG.center();
			}
			else // Image have not valid pose, so set an undefined pose
			{
        image.poseID = NO_ID;
			}
    }

    // Define structure
    scene.pointcloud.points.Reserve(sfm_data.GetLandmarks().size());
		for (const auto& vertex: sfm_data.GetLandmarks())
		{
      const Landmark & landmark = vertex.second;
			MVS::PointCloud::Point& point = scene.pointcloud.points.AddEmpty();
			point.X = landmark.X.cast<float>();
			for (const auto& observation: landmark.obs)
			{
				//point.views.Insert(map_view[observation.first]);
				point.views.Insert(observation.first);
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
					camera.K *= fScale;
				}
			}
		}

		// write OpenMVS input data
		scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), OPT::strWorkingFolderFull);

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