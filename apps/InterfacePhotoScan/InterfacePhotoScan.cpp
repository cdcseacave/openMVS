/*
 * InterfacePhotoScan.cpp
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


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfacePhotoScan")
#define MVS_EXT _T(".mvs")
#define XML_EXT _T(".xml")
#define PLY_EXT _T(".ply")


// S T R U C T S ///////////////////////////////////////////////////

namespace OPT {
String strPointsFileName;
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
		("points-file,p", boost::program_options::value<std::string>(&OPT::strPointsFileName), "input filename containing the 3D points")
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("output-image-folder", boost::program_options::value<std::string>(&OPT::strOutputImageFolder)->default_value("undistorted_images"), "output folder to store undistorted images")
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
	Util::ensureValidPath(OPT::strPointsFileName);
	Util::ensureUnifySlash(OPT::strPointsFileName);
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strOutputImageFolder);
	Util::ensureDirectorySlash(OPT::strOutputImageFolder);
	const String strInputFileNameExt(Util::getFileExt(OPT::strInputFileName).ToLower());
	const bool bInvalidCommand(OPT::strInputFileName.IsEmpty() || OPT::strPointsFileName.IsEmpty());
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFullFileName(OPT::strInputFileName) + MVS_EXT;

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

struct DistCoeff {
	REAL k1, k2, k3, p1, p2, k4, k5, k6;
	DistCoeff() : k1(0), k2(0), k3(0), p1(0), p2(0), k4(0), k5(0), k6(0) {}
};
typedef cList<DistCoeff> DistCoeffs;
typedef cList<DistCoeffs> PlatformDistCoeffs;

void ImageListParseP(const LPSTR* argv, Matrix3x4& P)
{
	// read projection matrix
	P(0, 0) = String::FromString<REAL>(argv[0]);
	P(0, 1) = String::FromString<REAL>(argv[1]);
	P(0, 2) = String::FromString<REAL>(argv[2]);
	P(0, 3) = String::FromString<REAL>(argv[3]);
	P(1, 0) = String::FromString<REAL>(argv[4]);
	P(1, 1) = String::FromString<REAL>(argv[5]);
	P(1, 2) = String::FromString<REAL>(argv[6]);
	P(1, 3) = String::FromString<REAL>(argv[7]);
	P(2, 0) = String::FromString<REAL>(argv[8]);
	P(2, 1) = String::FromString<REAL>(argv[9]);
	P(2, 2) = String::FromString<REAL>(argv[10]);
	P(2, 3) = String::FromString<REAL>(argv[11]);
}

// parse images list containing calibration and pose information
// and load the corresponding 3D point-cloud
bool ParseImageListXML(MVS::Scene& scene, PlatformDistCoeffs& pltDistCoeffs, size_t& nCameras, size_t& nPoses)
{
	using namespace MVS;

	// open image list
	nCameras = nPoses = 0;
	const String strInputFileName(MAKE_PATH_SAFE(OPT::strInputFileName));
	ISTREAMPTR pStream(new File(strInputFileName, File::READ, File::OPEN));
	if (!((File*)(ISTREAM*)pStream)->isOpen()) {
		LOG(_T("error: failed opening the input image list"));
		return false;
	}

	// parse camera list
	tinyxml2::XMLElement* elem;
	const size_t nLen(pStream->getSize());
	String strCameras; strCameras.resize(nLen);
	pStream->read(&strCameras[0], nLen);
	tinyxml2::XMLDocument doc;
	doc.Parse(strCameras.c_str(), nLen);
	if (doc.ErrorID() != tinyxml2::XML_SUCCESS)
		goto InvalidDocument;
	tinyxml2::XMLElement* document = doc.FirstChildElement(_T("document"))->FirstChildElement(_T("chunk"));
	if (document == NULL)
		goto InvalidDocument;
	bool bPhotoScanFile(false);

	// parse platform and camera models
	{
	tinyxml2::XMLElement* sensors = document->FirstChildElement(_T("sensors"));
	if (sensors == NULL)
		goto InvalidDocument;
	{
	for (tinyxml2::XMLElement* sensor=sensors->FirstChildElement(); sensor!=NULL; sensor=sensor->NextSiblingElement()) {
		unsigned ID;
		if (0 != _tcsicmp(sensor->Value(), _T("sensor")) || sensor->QueryUnsignedAttribute(_T("id"), &ID) != tinyxml2::XML_SUCCESS)
			goto InvalidDocument;
		{
		// add new camera
		enum CameraModel {PHOTOSCAN=0, VSFM};
		int model(PHOTOSCAN);
		sensor->QueryIntAttribute(_T("model"), &model);
		ASSERT(scene.platforms.GetSize() == ID);
		Platform& platform = scene.platforms.AddEmpty();
		LPCTSTR name;
		if ((name=sensor->Attribute(_T("label"))) != NULL)
			platform.name = name;
		// parse intrinsics
		tinyxml2::XMLElement* calibration = sensor->FirstChildElement(_T("calibration"));
		if (calibration == NULL)
			goto InvalidDocument;
		{
		REAL scale(1);
		if ((elem=calibration->FirstChildElement(_T("resolution"))) != NULL) {
			scale = REAL(1)/(REAL)Camera::GetNormalizationScale(elem->UnsignedAttribute(_T("width")), elem->UnsignedAttribute(_T("height")));
			ASSERT(model == PHOTOSCAN);
			bPhotoScanFile = true;
		}
		Platform::Camera& camera = platform.cameras.AddEmpty();
		camera.K = KMatrix::IDENTITY;
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;
		DistCoeff& dc = pltDistCoeffs.AddEmpty().AddEmpty();
		for (elem=calibration->FirstChildElement(); elem!=NULL; elem=elem->NextSiblingElement()) {
			if (0 == _tcsicmp(elem->Value(), _T("f"))) {
				camera.K(0,0) = camera.K(1,1) = String::FromString<REAL>(elem->GetText())*scale;
			} else
			if (0 == _tcsicmp(elem->Value(), _T("fx"))) {
				elem->QueryDoubleText(&camera.K(0,0));
				camera.K(0,0) *= scale;
			} else
			if (0 == _tcsicmp(elem->Value(), _T("fy"))) {
				elem->QueryDoubleText(&camera.K(1,1));
				camera.K(1,1) *= scale;
			} else
			if (0 == _tcsicmp(elem->Value(), _T("cx"))) {
				elem->QueryDoubleText(&camera.K(0,2));
				camera.K(0,2) *= scale;
			} else
			if (0 == _tcsicmp(elem->Value(), _T("cy"))) {
				elem->QueryDoubleText(&camera.K(1,2));
				camera.K(1,2) *= scale;
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k1"))) {
				elem->QueryDoubleText(&dc.k1);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k2"))) {
				elem->QueryDoubleText(&dc.k2);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k3"))) {
				elem->QueryDoubleText(&dc.k3);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("p1"))) {
				elem->QueryDoubleText(&dc.p1);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("p2"))) {
				elem->QueryDoubleText(&dc.p2);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k4"))) {
				elem->QueryDoubleText(&dc.k4);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k5"))) {
				elem->QueryDoubleText(&dc.k5);
			} else
			if (0 == _tcsicmp(elem->Value(), _T("k6"))) {
				elem->QueryDoubleText(&dc.k6);
			}
		}
		++nCameras;
		}
		}
	}
	}
	}

	// parse poses
	{
	tinyxml2::XMLElement* cameras = document->FirstChildElement(_T("cameras"));
	if (cameras == NULL)
		goto InvalidDocument;
	{
	PMatrix P;
	size_t argc;
	const String strPath(GET_PATH_FULL(strInputFileName));
	for (tinyxml2::XMLElement* camera=cameras->FirstChildElement(); camera!=NULL; camera=camera->NextSiblingElement()) {
		unsigned ID;
		if (0 != _tcsicmp(camera->Value(), _T("camera")) || camera->QueryUnsignedAttribute(_T("id"), &ID) != tinyxml2::XML_SUCCESS)
			goto InvalidDocument;
		{
		// add new image
		ASSERT(scene.images.GetSize() == ID);
		Image& imageData = scene.images.AddEmpty();
		LPCTSTR name;
		if ((name=camera->Attribute(_T("type"))) != NULL && _tcsicmp(name, _T("frame")) != 0) {
			DEBUG_EXTRA("warning: unsupported camera calibration '%s'", name);
			continue;
		}
		if ((name=camera->Attribute(_T("label"))) != NULL)
			imageData.name = name;
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(strPath, imageData.name);
		imageData.platformID = camera->UnsignedAttribute(_T("sensor_id"));
		imageData.cameraID = 0; // only one camera per platform supported by this format
		imageData.ID = ID;
		if (!camera->BoolAttribute(_T("enabled"))) {
			imageData.poseID = NO_ID;
			DEBUG_EXTRA("warning: uncalibrated image '%s'", name);
			continue;
		}
		// set pose
		CAutoPtrArr<LPSTR> argv;
		if ((elem=camera->FirstChildElement(_T("transform"))) == NULL ||
			(argv=Util::CommandLineToArgvA(elem->GetText(), argc)) == NULL ||
			(argc != (bPhotoScanFile ? 16 : 12)))
		{
			VERBOSE("Invalid image list camera: %u", ID);
			continue;
		}
		Platform& platform = scene.platforms[imageData.platformID];
		imageData.poseID = platform.poses.GetSize();
		Platform::Pose& pose = platform.poses.AddEmpty();
		ImageListParseP(argv, P);
		DecomposeProjectionMatrix(P, pose.R, pose.C);
		if (bPhotoScanFile) {
			pose.C = pose.R*(-pose.C);
			pose.R = pose.R.t();
		}
		imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
		++nPoses;
		}
	}
	}
	}

	return true;
	InvalidDocument:
	VERBOSE("Invalid camera list");
	return false;
}

// undistort image using Brown's model
bool UndistortBrown(MVS::Image& imageData, uint32_t ID, const DistCoeff& dc, const String& pathData)
{
	using namespace MVS;

	// load image pixels
	if (!imageData.ReloadImage())
		return false;

	// initialize intrinsics
	cv::Vec<double,8> distCoeffs;
	distCoeffs(0) = dc.k1;
	distCoeffs(1) = dc.k2;
	distCoeffs(2) = dc.p1;
	distCoeffs(3) = dc.p2;
	distCoeffs(4) = dc.k3;
	distCoeffs(5) = dc.k4;
	distCoeffs(6) = dc.k5;
	distCoeffs(7) = dc.k6;
	const KMatrix prevK(imageData.camera.GetK<REAL>(imageData.width, imageData.height));
	#if 1
	const KMatrix& K(prevK);
	#else
	const KMatrix K(cv::getOptimalNewCameraMatrix(prevK, distCoeffs, imageData.GetSize(), 0.0, cv::Size(), NULL, true));
	ASSERT(K(0,2) == Camera::ComposeK(prevK(0,0), prevK(1,1), imageData.width(), imageData.height())(0,2));
	ASSERT(K(1,2) == Camera::ComposeK(prevK(0,0), prevK(1,1), imageData.width(), imageData.height())(1,2));
	if (K.IsEqual(prevK)) {
		int i(0);
		while (distCoeffs(i++) == 0.0) {
			if (i == 8)
				return true; // nothing to do
		}
	}
	#endif

	// undistort image
	Image8U3 imgUndist;
	cv::undistort(imageData.image, imgUndist, prevK, distCoeffs, K);
	imageData.ReleaseImage();

	// save undistorted image
	imageData.image = imgUndist;
	imageData.name = pathData + String::FormatString(_T("%05u.png"), ID);
	Util::ensureDirectory(imageData.name);
	return imageData.image.Save(imageData.name);
}

// project all points in this image and keep those looking at the camera and are most in front
void AssignPoints(const MVS::Image& imageData, uint32_t ID, MVS::PointCloud& pointcloud)
{
	using namespace MVS;

	ASSERT(pointcloud.IsValid());
	const int CHalfSize(1);
	const int FHalfSize(5);
	const Depth thCloseDepth(0.1f);

	// sort points by depth
	IndexScoreArr points(0, pointcloud.points.GetSize());
	FOREACH(p, pointcloud.points) {
		const PointCloud::Point& X(pointcloud.points[p]);
		const float d((float)imageData.camera.PointDepth(X));
		if (d <= 0)
			continue;
		points.AddConstruct((uint32_t)p, d);
	}
	points.Sort();

	// project all points to this view
	DepthMap depthMap(imageData.GetSize());
	TImage<cuint32_t> pointMap(imageData.GetSize());
	depthMap.fill(FLT_MAX);
	pointMap.memset((uint8_t)NO_ID);
	RFOREACHPTR(pPD, points) {
		const Point3 X(pointcloud.points[pPD->idx]);
		const Point3f Xc(imageData.camera.TransformPointW2C(X));
		// (also the view to point vector cause the face is in camera view space)
		// point skip already in the previous step if the (cos) angle between
		// the view to point vector and the view direction is negative
		ASSERT(Xc.z > 0);
		// skip point if the (cos) angle between
		// its normal and the point to view vector is negative
		if (!pointcloud.normals.IsEmpty() && Xc.dot(pointcloud.normals[pPD->idx]) > 0)
			continue;
		const Point2f x(imageData.camera.TransformPointC2I(Xc));
		const ImageRef ir(ROUND2INT(x));
		if (!depthMap.isInside(ir))
			continue;
		// skip point if the there is a very near by point closer
		for (int i=-CHalfSize; i<=CHalfSize; ++i) {
			const int rw(ir.y+i);
			for (int j=-CHalfSize; j<=CHalfSize; ++j) {
				const int cw(ir.x+j);
				if (!depthMap.isInside(ImageRef(cw,rw)))
					continue;
				if (depthMap(rw,cw) < Xc.z)			
					goto NEXT_POINT;
			}
		}
		// skip the point if there is a near by point much closer
		for (int i=-FHalfSize; i<=FHalfSize; ++i) {
			const int rw(ir.y+i);
			for (int j=-FHalfSize; j<=FHalfSize; ++j) {
				const int cw(ir.x+j);
				if (!depthMap.isInside(ImageRef(cw,rw)))
					continue;
				const Depth depth(depthMap(rw,cw));
				if (depth < Xc.z && !IsDepthSimilar(depth, Xc.z, thCloseDepth))
					goto NEXT_POINT;
			}
		}
		// store this point
		depthMap(ir) = Xc.z;
		pointMap(ir) = pPD->idx;
		NEXT_POINT:;
	}

	// add all points viewed by this camera
	const int HalfSize(1);
	const int RowsEnd(pointMap.rows-HalfSize);
	const int ColsEnd(pointMap.cols-HalfSize);
	unsigned nNumPoints(0);
	#ifdef _USE_OPENMP
	#pragma omp critical
	#endif
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			const uint32_t idx(pointMap(r,c));
			if (idx == NO_ID)
				continue;
			#ifdef _USE_OPENMP
			pointcloud.pointViews[idx].InsertSort(ID);
			#else
			pointcloud.pointViews[idx].Insert(ID);
			ASSERT(pointcloud.pointViews[idx].IsSorted());
			#endif
			++nNumPoints;
		}
	}

	DEBUG_ULTIMATE("\tview %3u sees %u points", ID, nNumPoints);
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

	// read the 3D point-cloud
	MVS::Scene scene(OPT::nMaxThreads);
	if (!scene.pointcloud.Load(MAKE_PATH_SAFE(OPT::strPointsFileName)))
		return EXIT_FAILURE;
	ASSERT(!scene.pointcloud.IsValid());
	scene.pointcloud.pointViews.Resize(scene.pointcloud.points.GetSize());

	// convert data from PhotoScan format to OpenMVS
	PlatformDistCoeffs pltDistCoeffs;
	size_t nCameras, nPoses;
	if (!ParseImageListXML(scene, pltDistCoeffs, nCameras, nPoses))
		return EXIT_FAILURE;

	// undistort images
	const String pathData(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputImageFolder));
	Util::Progress progress(_T("Processed images"), scene.images.GetSize());
	GET_LOGCONSOLE().Pause();
	#ifdef _USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for shared(bAbort) schedule(dynamic)
	for (int ID=0; ID<(int)scene.images.GetSize(); ++ID) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
	#else
	FOREACH(ID, scene.images) {
	#endif
		++progress;
		MVS::Image& imageData = scene.images[ID];
		if (!UndistortBrown(imageData, ID, pltDistCoeffs[imageData.platformID][imageData.cameraID], pathData)) {
			#ifdef _USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
		imageData.UpdateCamera(scene.platforms);
		AssignPoints(imageData, ID, scene.pointcloud);
	}
	GET_LOGCONSOLE().Play();
	#ifdef _USE_OPENMP
	if (bAbort)
		return EXIT_SUCCESS;
	#endif
	progress.close();

	// write OpenMVS input data
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	VERBOSE("Exported data: %u platforms, %u cameras, %u poses, %u images, %u vertices (%s)",
			scene.platforms.GetSize(), nCameras, nPoses, scene.images.GetSize(), scene.pointcloud.GetSize(),
			TD_TIMER_GET_FMT().c_str());

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
