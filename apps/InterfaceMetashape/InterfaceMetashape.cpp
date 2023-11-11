/*
 * InterfaceMetashape.cpp
 *
 * Copyright (c) 2014-2021 SEACAVE
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
#include "../../libs/IO/TinyXML2.h"
#include <boost/program_options.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceMetashape") // previously PhotoScan
#define MVS_EXT _T(".mvs")
#define XML_EXT _T(".xml")
#define PLY_EXT _T(".ply")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strPointsFileName;
String strOutputFileName;
String strOutputImageFolder;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
String strConfigFileName;
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
	OPEN_LOGCONSOLE();

	// group of options allowed only on command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "imports SfM scene stored either in Metashape Agisoft/BlocksExchange or ContextCapture BlocksExchange XML format")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_MVS), "project archive type: -1-interface, 0-text, 1-binary, 2-compressed binary")
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
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("points-file,p", boost::program_options::value<std::string>(&OPT::strPointsFileName), "input filename containing the 3D points")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the scene")
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
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strPointsFileName);
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureValidFolderPath(OPT::strOutputImageFolder);
	const bool bInvalidCommand(OPT::strInputFileName.empty());
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	if (OPT::strOutputFileName.empty())
		OPT::strOutputFileName = Util::getFileName(OPT::strInputFileName) + MVS_EXT;

	MVS::Initialize(APPNAME, OPT::nMaxThreads, OPT::nProcessPriority);
	return true;
}

// finalize application instance
void Application::Finalize()
{
	MVS::Finalize();

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

struct DistCoeff {
	union {
		REAL coeff[8];
		struct {
			REAL k1, k2, p1, p2, k3, k4, k5, k6;
		};
	};
	DistCoeff() : k1(0), k2(0), p1(0), p2(0), k3(0), k4(0), k5(0), k6(0) {}
	bool HasDistortion() const { return k1 != 0 || k2 != 0 || k3 != 0 || k4 != 0 || k5 != 0 || k6 != 0; }
};
typedef cList<DistCoeff> DistCoeffs;
typedef cList<DistCoeffs> PlatformDistCoeffs;

void ImageListParseC(const LPSTR* argv, Point3& C)
{
	// read position vector
	C.x = String::FromString<REAL>(argv[0]);
	C.y = String::FromString<REAL>(argv[1]);
	C.z = String::FromString<REAL>(argv[2]);
}

void ImageListParseR(const LPSTR* argv, Matrix3x3& R)
{
	// read rotation matrix
	R(0, 0) = String::FromString<REAL>(argv[0]);
	R(0, 1) = String::FromString<REAL>(argv[1]);
	R(0, 2) = String::FromString<REAL>(argv[2]);
	R(1, 0) = String::FromString<REAL>(argv[3]);
	R(1, 1) = String::FromString<REAL>(argv[4]);
	R(1, 2) = String::FromString<REAL>(argv[5]);
	R(2, 0) = String::FromString<REAL>(argv[6]);
	R(2, 1) = String::FromString<REAL>(argv[7]);
	R(2, 2) = String::FromString<REAL>(argv[8]);
}

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
bool ParseImageListXML(tinyxml2::XMLDocument& doc, Scene& scene, PlatformDistCoeffs& pltDistCoeffs, size_t& nCameras, size_t& nPoses)
{
	String strInputFileName(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strInputFileName));
	Util::ensureValidPath(strInputFileName);
	tinyxml2::XMLElement* elem;
	if (doc.ErrorID() != tinyxml2::XML_SUCCESS)
		goto InvalidDocument;
	{
	tinyxml2::XMLElement* document = doc.FirstChildElement(_T("document"))->FirstChildElement(_T("chunk"));
	if (document == NULL)
		goto InvalidDocument;
	{
	bool bMetashapeFile(false);
	CLISTDEF0(cv::Size) resolutions;
	std::unordered_map<IIndex,IIndex> mapPlatformID;
	std::unordered_map<IIndex,IIndex> mapImageID;

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
		enum CameraModel {METASHAPE=0, VSFM};
		int model(METASHAPE);
		sensor->QueryIntAttribute(_T("model"), &model);
		mapPlatformID.emplace(ID, scene.platforms.size());
		Platform& platform = scene.platforms.AddEmpty();
		LPCTSTR name;
		if ((name=sensor->Attribute(_T("label"))) != NULL)
			platform.name = name;
		// parse intrinsics
		tinyxml2::XMLElement* calibration = sensor->FirstChildElement(_T("calibration"));
		if (calibration == NULL)
			goto InvalidDocument;
		{
		if ((elem=calibration->FirstChildElement(_T("resolution"))) != NULL) {
			resolutions.emplace_back(
				elem->UnsignedAttribute(_T("width")),
				elem->UnsignedAttribute(_T("height"))
			);
			ASSERT(model == METASHAPE);
			bMetashapeFile = true;
		}
		Platform::Camera& camera = platform.cameras.AddEmpty();
		camera.K = KMatrix::IDENTITY;
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;
		DistCoeff& dc = pltDistCoeffs.AddEmpty().AddEmpty();
		for (elem=calibration->FirstChildElement(); elem!=NULL; elem=elem->NextSiblingElement()) {
			if (0 == _tcsicmp(elem->Value(), _T("f"))) {
				camera.K(0,0) = camera.K(1,1) = String::FromString<REAL>(elem->GetText());
			} else
			if (0 == _tcsicmp(elem->Value(), _T("fx"))) {
				elem->QueryDoubleText(&camera.K(0,0));
			} else
			if (0 == _tcsicmp(elem->Value(), _T("fy"))) {
				elem->QueryDoubleText(&camera.K(1,1));
			} else
			if (0 == _tcsicmp(elem->Value(), _T("cx"))) {
				elem->QueryDoubleText(&camera.K(0,2));
			} else
			if (0 == _tcsicmp(elem->Value(), _T("cy"))) {
				elem->QueryDoubleText(&camera.K(1,2));
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
		if (bMetashapeFile) {
			const cv::Size& resolution = resolutions.back();
			camera.K(0,2) += resolution.width*REAL(0.5);
			camera.K(1,2) += resolution.height*REAL(0.5);
			camera.K = camera.GetScaledK(REAL(1)/Camera::GetNormalizationScale(resolution.width, resolution.height));
			std::swap(dc.p1, dc.p2);
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
	const String strPath(Util::getFilePath(strInputFileName));
	for (tinyxml2::XMLElement* camera=cameras->FirstChildElement(); camera!=NULL; camera=camera->NextSiblingElement()) {
		unsigned ID;
		if (0 != _tcsicmp(camera->Value(), _T("camera")) || camera->QueryUnsignedAttribute(_T("id"), &ID) != tinyxml2::XML_SUCCESS)
			goto InvalidDocument;
		{
		// add new image
		mapImageID.emplace(ID, scene.images.size());
		Image& imageData = scene.images.AddEmpty();
		LPCTSTR name;
		if ((name=camera->Attribute(_T("type"))) != NULL && _tcsicmp(name, _T("frame")) != 0) {
			DEBUG_EXTRA("warning: unsupported camera calibration '%s'", name);
			continue;
		}
		if ((name=camera->Attribute(_T("label"))) != NULL)
			imageData.name = name;
		Util::ensureUnifySlash(imageData.name);
		if (Util::getFileExt(imageData.name).empty())
			imageData.name += _T(".jpg");
		imageData.name = MAKE_PATH_FULL(strPath, imageData.name);
		imageData.platformID = mapPlatformID.at(camera->UnsignedAttribute(_T("sensor_id")));
		imageData.cameraID = 0; // only one camera per platform supported by this format
		imageData.ID = mapImageID.at(ID);
		const cv::Size& resolution = resolutions[imageData.platformID];
		imageData.width = resolution.width;
		imageData.height = resolution.height;
		imageData.scale = 1;
		if (!bMetashapeFile && !camera->BoolAttribute(_T("enabled"))) {
			imageData.poseID = NO_ID;
			DEBUG_EXTRA("warning: uncalibrated image '%s'", name);
			continue;
		}
		// set pose
		CAutoPtrArr<LPSTR> argv;
		if ((elem=camera->FirstChildElement(_T("transform"))) == NULL ||
			(argv=Util::CommandLineToArgvA(elem->GetText(), argc)) == NULL ||
			(argc != (bMetashapeFile ? 16 : 12)))
		{
			VERBOSE("Invalid image list camera: %u", ID);
			continue;
		}
		Platform& platform = scene.platforms[imageData.platformID];
		imageData.poseID = platform.poses.size();
		Platform::Pose& pose = platform.poses.AddEmpty();
		ImageListParseP(argv, P);
		DecomposeProjectionMatrix(P, pose.R, pose.C);
		if (bMetashapeFile) {
			pose.C = pose.R*(-pose.C);
			pose.R = pose.R.t();
		}
		imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
		++nPoses;
		}
	}
	scene.nCalibratedImages = (unsigned)nPoses;
	}

	// parse bounding-box
	{
	tinyxml2::XMLElement* region = document->FirstChildElement(_T("region"));
	if (region == NULL)
		goto InvalidDocument;
	{
	size_t argc;
	CAutoPtrArr<LPSTR> argv;
	Point3 C, E; Matrix3x3 R;
	if ((elem=region->FirstChildElement(_T("center"))) == NULL ||
		(argv=Util::CommandLineToArgvA(elem->GetText(), argc)) == NULL ||
		argc != 3)
	{
		VERBOSE("Invalid image list region: %s", elem->GetText());
		goto InvalidDocument;
	}
	ImageListParseC(argv, C);
	if ((elem=region->FirstChildElement(_T("size"))) == NULL ||
		(argv=Util::CommandLineToArgvA(elem->GetText(), argc)) == NULL ||
		argc != 3)
	{
		VERBOSE("Invalid image list region: %s", elem->GetText());
		goto InvalidDocument;
	}
	ImageListParseC(argv, E);
	E *= REAL(0.5);
	if ((elem=region->FirstChildElement(_T("R"))) == NULL ||
		(argv=Util::CommandLineToArgvA(elem->GetText(), argc)) == NULL ||
		argc != 9)
	{
		VERBOSE("Invalid image list region: %s", elem->GetText());
		goto InvalidDocument;
	}
	ImageListParseR(argv, R);
	scene.obb.m_rot = Cast<float>(R);
	scene.obb.m_pos = Cast<float>(C);
	scene.obb.m_ext = Cast<float>(E);
	}
	}
	}
	}
	}

	return true;
	InvalidDocument:
	VERBOSE("Invalid camera list");
	return false;
}

// parse scene stored in ContextCapture BlocksExchange format containing cameras, images and sparse point-cloud
bool ParseBlocksExchangeXML(tinyxml2::XMLDocument& doc, Scene& scene, PlatformDistCoeffs& pltDistCoeffs, size_t& nCameras, size_t& nPoses) {
	String strInputFileName(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strInputFileName));
	Util::ensureValidPath(strInputFileName);
	const tinyxml2::XMLElement* blocksExchange;
	const tinyxml2::XMLElement* document;
	const tinyxml2::XMLElement* photogroups;
	if (doc.ErrorID() != tinyxml2::XML_SUCCESS ||
		(blocksExchange=doc.FirstChildElement("BlocksExchange")) == NULL ||
		(document=blocksExchange->FirstChildElement("Block")) == NULL ||
		(photogroups=document->FirstChildElement("Photogroups")) == NULL) {
		VERBOSE("error: invalid scene file");
		return false;
	}
	CLISTDEF0(cv::Size) resolutions;
	std::unordered_map<IIndex,IIndex> mapImageID;
	const String strPath(Util::getFilePath(strInputFileName));
	const tinyxml2::XMLElement* elem;
	for (const tinyxml2::XMLElement* photogroup=photogroups->FirstChildElement(); photogroup!=NULL; photogroup=photogroup->NextSiblingElement()) {
		if ((elem=photogroup->FirstChildElement("CameraModelType")) == NULL ||
			std::strcmp(elem->GetText(), "Perspective") != 0)
			continue;
		if ((elem=photogroup->FirstChildElement("ImageDimensions")) == NULL)
			continue;
		const IIndex platformID = scene.platforms.size();
		Platform& platform = scene.platforms.AddEmpty();
		platform.name = photogroup->FirstChildElement("Name")->GetText();
		resolutions.emplace_back(
			elem->FirstChildElement("Width")->UnsignedText(),
			elem->FirstChildElement("Height")->UnsignedText()
		);
		// parse camera
		Platform::Camera& camera = platform.cameras.AddEmpty();
		camera.K = KMatrix::IDENTITY;
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;
		const float resolutionScale = Camera::GetNormalizationScale(resolutions.back().width, resolutions.back().height);
		if ((elem=photogroup->FirstChildElement("FocalLengthPixels")) != NULL) {
			camera.K(0,0) = camera.K(1,1) = photogroup->FirstChildElement("FocalLengthPixels")->DoubleText();
		} else {
			camera.K(0,0) = camera.K(1,1) = photogroup->FirstChildElement("FocalLength")->DoubleText() * resolutionScale / photogroup->FirstChildElement("SensorSize")->DoubleText();
		}
		if ((elem=photogroup->FirstChildElement("PrincipalPoint")) != NULL) {
			camera.K(0,2) = elem->FirstChildElement("x")->DoubleText();
			camera.K(1,2) = elem->FirstChildElement("y")->DoubleText();
		} else {
			camera.K(0,2) = resolutions.back().width*REAL(0.5);
			camera.K(1,2) = resolutions.back().height*REAL(0.5);
		}
		if ((elem=photogroup->FirstChildElement("AspectRatio")) != NULL)
			camera.K(1,1) *= elem->DoubleText();
		if ((elem=photogroup->FirstChildElement("Skew")) != NULL)
			camera.K(0,1) = elem->DoubleText();
		camera.K = camera.GetScaledK(REAL(1)/resolutionScale);
		// parse distortion parameters
		DistCoeff& dc = pltDistCoeffs.AddEmpty().AddEmpty(); {
			const tinyxml2::XMLElement* distortion=photogroup->FirstChildElement("Distortion");
			if (distortion) {
				if ((elem=distortion->FirstChildElement("K1")) != NULL)
					dc.k1 = elem->DoubleText();
				if ((elem=distortion->FirstChildElement("K2")) != NULL)
					dc.k2 = elem->DoubleText();
				if ((elem=distortion->FirstChildElement("K3")) != NULL)
					dc.k3 = elem->DoubleText();
				if ((elem=distortion->FirstChildElement("P1")) != NULL)
					dc.p2 = elem->DoubleText();
				if ((elem=distortion->FirstChildElement("P2")) != NULL)
					dc.p1 = elem->DoubleText();
			}
		}
		++nCameras;
		for (const tinyxml2::XMLElement* photo=photogroup->FirstChildElement("Photo"); photo!=NULL; photo=photo->NextSiblingElement()) {
			const IIndex idxImage = scene.images.size();
			Image& imageData = scene.images.AddEmpty();
			imageData.platformID = platformID;
			imageData.cameraID = 0; // only one camera per platform supported by this format
			imageData.poseID = NO_ID;
			imageData.ID = photo->FirstChildElement("Id")->UnsignedText();
			imageData.name = photo->FirstChildElement("ImagePath")->GetText();
			Util::ensureUnifySlash(imageData.name);
			imageData.name = MAKE_PATH_FULL(strPath, imageData.name);
			mapImageID.emplace(imageData.ID, idxImage);
			// set image resolution
			const cv::Size& resolution = resolutions[imageData.platformID];
			imageData.width = resolution.width;
			imageData.height = resolution.height;
			imageData.scale = 1;
			// set camera pose
			const tinyxml2::XMLElement* photoPose = photo->FirstChildElement("Pose");
			if (photoPose == NULL)
				continue;
			if ((elem=photoPose->FirstChildElement("Rotation")) == NULL)
				continue;
			imageData.poseID = platform.poses.size();
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = Matrix3x3(
				elem->FirstChildElement("M_00")->DoubleText(),
				elem->FirstChildElement("M_01")->DoubleText(),
				elem->FirstChildElement("M_02")->DoubleText(),
				elem->FirstChildElement("M_10")->DoubleText(),
				elem->FirstChildElement("M_11")->DoubleText(),
				elem->FirstChildElement("M_12")->DoubleText(),
				elem->FirstChildElement("M_20")->DoubleText(),
				elem->FirstChildElement("M_21")->DoubleText(),
				elem->FirstChildElement("M_22")->DoubleText());
			if ((elem=photoPose->FirstChildElement("Center")) == NULL)
				continue;
			pose.C = Point3(
				elem->FirstChildElement("x")->DoubleText(),
				elem->FirstChildElement("y")->DoubleText(),
				elem->FirstChildElement("z")->DoubleText());
			imageData.camera = platform.GetCamera(imageData.cameraID, imageData.poseID);
			// set depth stats
			if ((elem=photo->FirstChildElement("MedianDepth")) != NULL)
				imageData.avgDepth = (float)elem->DoubleText();
			else if (photo->FirstChildElement("NearDepth") != NULL && photo->FirstChildElement("FarDepth") != NULL)
				imageData.avgDepth = (float)((photo->FirstChildElement("NearDepth")->DoubleText() + photo->FirstChildElement("FarDepth")->DoubleText())/2);
			else
				imageData.avgDepth = 0;
			++nPoses;
		}
	}
	if (scene.images.size() < 2)
		return false;
	scene.nCalibratedImages = (unsigned)nPoses;
	// transform poses to a local coordinate system
	const bool bLocalCoords(document->FirstChildElement("SRSId") == NULL ||
		((elem=blocksExchange->FirstChildElement("SpatialReferenceSystems")) != NULL && (elem=elem->FirstChildElement("SRS")) != NULL && (elem=elem->FirstChildElement("Name")) != NULL && _tcsncmp(elem->GetText(), "Local Coordinates", 17) == 0));
	Point3 center = Point3::ZERO;
	if (!bLocalCoords) {
		for (const Image& imageData : scene.images)
			center += imageData.camera.C;
		center /= scene.images.size();
		for (Platform& platform : scene.platforms)
			for (Platform::Pose& pose : platform.poses)
				pose.C -= center;
	}
	// try to read also the sparse point-cloud
	const tinyxml2::XMLElement* tiepoints = document->FirstChildElement("TiePoints");
	if (tiepoints == NULL)
		return true;
	for (const tinyxml2::XMLElement* tiepoint=tiepoints->FirstChildElement(); tiepoint!=NULL; tiepoint=tiepoint->NextSiblingElement()) {
		if ((elem=tiepoint->FirstChildElement("Position")) == NULL)
			continue;
		scene.pointcloud.points.emplace_back(
			(float)elem->FirstChildElement("x")->DoubleText(),
			(float)elem->FirstChildElement("y")->DoubleText(),
			(float)elem->FirstChildElement("z")->DoubleText());
		if (!bLocalCoords)
			scene.pointcloud.points.back() -= Cast<float>(center);
		if ((elem=tiepoint->FirstChildElement("Color")) != NULL)
			scene.pointcloud.colors.emplace_back(
				(uint8_t)CLAMP(elem->FirstChildElement("Red")->DoubleText()*255, 0.0, 255.0),
				(uint8_t)CLAMP(elem->FirstChildElement("Green")->DoubleText()*255, 0.0, 255.0),
				(uint8_t)CLAMP(elem->FirstChildElement("Blue")->DoubleText()*255, 0.0, 255.0));
		PointCloud::ViewArr views;
		for (const tinyxml2::XMLElement* view=tiepoint->FirstChildElement("Measurement"); view!=NULL; view=view->NextSiblingElement())
			views.emplace_back(mapImageID.at(view->FirstChildElement("PhotoId")->UnsignedText()));
		scene.pointcloud.pointViews.emplace_back(std::move(views));
	}
	return true;
}

// parse scene stored either in Metashape images list format or ContextCapture BlocksExchange format
bool ParseSceneXML(Scene& scene, PlatformDistCoeffs& pltDistCoeffs, size_t& nCameras, size_t& nPoses)
{
	// parse XML file
	const String strInputFileName(MAKE_PATH_SAFE(OPT::strInputFileName));
	tinyxml2::XMLDocument doc; {
		ISTREAMPTR pStream(new File(strInputFileName, File::READ, File::OPEN));
		if (!((File*)(ISTREAM*)pStream)->isOpen()) {
			VERBOSE("error: failed opening the input scene file");
			return false;
		}
		const size_t nLen(pStream->getSize());
		String str; str.resize(nLen);
		pStream->read(&str[0], nLen);
		doc.Parse(str.c_str(), nLen);
	}
	if (doc.ErrorID() != tinyxml2::XML_SUCCESS) {
		VERBOSE("error: invalid XML file");
		return false;
	}
	// parse scene
	if (doc.FirstChildElement("BlocksExchange") == NULL)
		return ParseImageListXML(doc, scene, pltDistCoeffs, nCameras, nPoses);
	return ParseBlocksExchangeXML(doc, scene, pltDistCoeffs, nCameras, nPoses);
}

// undistort image using Brown's model
bool UndistortBrown(Image& imageData, uint32_t ID, const DistCoeff& dc, const String& pathData)
{
	// do we need to undistort?
	if (!dc.HasDistortion())
		return true;

	// load image pixels
	if (!imageData.ReloadImage())
		return false;

	// initialize intrinsics
	const cv::Vec<double,8>& distCoeffs = *reinterpret_cast<const cv::Vec<REAL,8>*>(dc.coeff);
	const KMatrix prevK(imageData.camera.GetK<REAL>(imageData.width, imageData.height));
	#if 1
	const KMatrix& K(prevK);
	#else
	const KMatrix K(cv::getOptimalNewCameraMatrix(prevK, distCoeffs, imageData.size(), 0.0, cv::Size(), NULL, true));
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
	imageData.name = pathData + String::FormatString(_T("%05u.jpg"), ID);
	Util::ensureFolder(imageData.name);
	return imageData.image.Save(imageData.name);
}

// project all points in this image and keep those looking at the camera and are most in front
void AssignPoints(const Image& imageData, uint32_t ID, PointCloud& pointcloud)
{
	ASSERT(pointcloud.IsValid());
	const int CHalfSize(1);
	const int FHalfSize(5);
	const Depth thCloseDepth(0.1f);

	// sort points by depth
	IndexScoreArr points(0, pointcloud.points.size());
	FOREACH(p, pointcloud.points) {
		const PointCloud::Point& X(pointcloud.points[p]);
		const float d((float)imageData.camera.PointDepth(X));
		if (d <= 0)
			continue;
		points.emplace_back((uint32_t)p, d);
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
		if (!pointcloud.normals.empty() && Xc.dot(pointcloud.normals[pPD->idx]) > 0)
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

} // unnamed namespace

int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	Scene scene(OPT::nMaxThreads);

	// convert data from Metashape format to OpenMVS
	PlatformDistCoeffs pltDistCoeffs;
	size_t nCameras(0), nPoses(0);
	if (!ParseSceneXML(scene, pltDistCoeffs, nCameras, nPoses))
		return EXIT_FAILURE;

	// read the 3D point-cloud if available
	if (!OPT::strPointsFileName.empty() && !scene.pointcloud.Load(MAKE_PATH_SAFE(OPT::strPointsFileName)))
		return EXIT_FAILURE;
	const bool bAssignPoints(!scene.pointcloud.IsEmpty() && !scene.pointcloud.IsValid());
	if (bAssignPoints)
		scene.pointcloud.pointViews.resize(scene.pointcloud.GetSize());

	// undistort images
	const String pathData(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputImageFolder));
	Util::Progress progress(_T("Processed images"), scene.images.size());
	GET_LOGCONSOLE().Pause();
	#ifdef _USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for shared(bAbort) schedule(dynamic)
	for (int ID=0; ID<(int)scene.images.size(); ++ID) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
	#else
	FOREACH(ID, scene.images) {
	#endif
		++progress;
		Image& imageData = scene.images[ID];
		if (!imageData.IsValid())
			continue;
		if (!UndistortBrown(imageData, ID, pltDistCoeffs[imageData.platformID][imageData.cameraID], pathData)) {
			#ifdef _USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return EXIT_FAILURE;
			#endif
		}
		imageData.UpdateCamera(scene.platforms);
		if (bAssignPoints)
			AssignPoints(imageData, ID, scene.pointcloud);
	}
	GET_LOGCONSOLE().Play();
	#ifdef _USE_OPENMP
	if (bAbort)
		return EXIT_FAILURE;
	#endif
	progress.close();

	if (scene.pointcloud.IsValid()) {
		// filter invalid points
		RFOREACH(i, scene.pointcloud.points)
			if (scene.pointcloud.pointViews[i].size() < 2)
				scene.pointcloud.RemovePoint(i);
		// compute average scene depth per image
		if (!std::any_of(scene.images.begin(), scene.images.end(), [](const Image& imageData) { return imageData.avgDepth > 0; })) {
			std::vector<float> avgDepths(scene.images.size(), 0.f);
			std::vector<uint32_t> numDepths(scene.images.size(), 0u);
			FOREACH(idxPoint, scene.pointcloud.points) {
				const Point3 X(scene.pointcloud.points[idxPoint]);
				for (const PointCloud::View& idxImage: scene.pointcloud.pointViews[idxPoint]) {
					const Image& imageData = scene.images[idxImage];
					const float depth((float)imageData.camera.PointDepth(X));
					if (depth > 0) {
						avgDepths[idxImage] += depth;
						++numDepths[idxImage];
					}
				}
			}
			FOREACH(idxImage, scene.images) {
				Image& imageData = scene.images[idxImage];
				if (numDepths[idxImage] > 0)
					imageData.avgDepth = avgDepths[idxImage] / numDepths[idxImage];
			}
		}
	}

	// print average scene depth per image stats
	MeanStdMinMax<float,double> acc;
	for (const Image& imageData: scene.images)
		if (imageData.avgDepth > 0)
			acc.Update(imageData.avgDepth);

	// write OpenMVS input data
	scene.Save(MAKE_PATH_SAFE(OPT::strOutputFileName), (ARCHIVE_TYPE)OPT::nArchiveType);

	VERBOSE("Exported data: %u platforms, %u cameras, %u poses, %u images, %u vertices, %g min / %g mean (%g std) / %g max average scene depth per image (%s)",
			scene.platforms.size(), nCameras, nPoses, scene.images.size(), scene.pointcloud.GetSize(),
			acc.minVal, acc.GetMean(), acc.GetStdDev(), acc.maxVal,
			TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
