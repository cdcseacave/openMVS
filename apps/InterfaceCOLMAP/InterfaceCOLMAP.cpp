/*
 * InterfaceCOLMAP.cpp
 *
 * Copyright (c) 2014-2018 SEACAVE
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
#define _USE_OPENCV
#include "../../libs/MVS/Interface.h"
#include <boost/program_options.hpp>
#include "endian.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("InterfaceCOLMAP")
#define MVS_EXT _T(".mvs")
#define COLMAP_IMAGES_FOLDER _T("images/")
#define COLMAP_SPARSE_FOLDER _T("sparse/")
#define COLMAP_CAMERAS_TXT COLMAP_SPARSE_FOLDER _T("cameras.txt")
#define COLMAP_IMAGES_TXT COLMAP_SPARSE_FOLDER _T("images.txt")
#define COLMAP_POINTS_TXT COLMAP_SPARSE_FOLDER _T("points3D.txt")
#define COLMAP_CAMERAS_BIN COLMAP_SPARSE_FOLDER _T("cameras.bin")
#define COLMAP_IMAGES_BIN COLMAP_SPARSE_FOLDER _T("images.bin")
#define COLMAP_POINTS_BIN COLMAP_SPARSE_FOLDER _T("points3D.bin")
#define COLMAP_DENSE_POINTS _T("fused.ply")
#define COLMAP_DENSE_POINTS_VISIBILITY _T("fused.ply.vis")
#define COLMAP_STEREO_FOLDER _T("stereo/")
#define COLMAP_FUSION COLMAP_STEREO_FOLDER _T("fusion.cfg")
#define COLMAP_PATCHMATCH COLMAP_STEREO_FOLDER _T("patch-match.cfg")
#define COLMAP_STEREO_CONSISTENCYGRAPHS_FOLDER COLMAP_STEREO_FOLDER _T("consistency_graphs/")
#define COLMAP_STEREO_DEPTHMAPS_FOLDER COLMAP_STEREO_FOLDER _T("depth_maps/")
#define COLMAP_STEREO_NORMALMAPS_FOLDER COLMAP_STEREO_FOLDER _T("normal_maps/")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
bool bFromOpenMVS; // conversion direction
bool bNormalizeIntrinsics;
String strInputFileName;
String strOutputFileName;
String strImageFolder;
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
	boost::program_options::options_description config("Main options");
	config.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input COLMAP folder containing cameras, images and points files OR input MVS project file")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the MVS project")
		("image-folder", boost::program_options::value<std::string>(&OPT::strImageFolder)->default_value(COLMAP_IMAGES_FOLDER), "folder to the undistorted images")
		("normalize,f", boost::program_options::value(&OPT::bNormalizeIntrinsics)->default_value(false), "normalize intrinsics while exporting to MVS format")
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
		Util::ensureValidPath(OPT::strInputFileName);
		WORKING_FOLDER = (File::isFolder(OPT::strInputFileName) ? OPT::strInputFileName : Util::getFilePath(OPT::strInputFileName));
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
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")));

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	const bool bInvalidCommand(OPT::strInputFileName.empty());
	if (OPT::vm.count("help") || bInvalidCommand) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config);
		GET_LOG() << _T("\n"
			"Import/export 3D reconstruction from COLMAP (TXT/BIN format) and to COLMAP (TXT format). \n"
			"In order to import a scene, run COLMAP SfM and next undistort the images (only PINHOLE\n"
			"camera model supported for the moment)."
			"\n")
			<< visible;
	}
	if (bInvalidCommand)
		return false;

	// initialize optional options
	Util::ensureValidFolderPath(OPT::strImageFolder);
	Util::ensureValidPath(OPT::strOutputFileName);
	const String strInputFileNameExt(Util::getFileExt(OPT::strInputFileName).ToLower());
	OPT::bFromOpenMVS = (strInputFileNameExt == MVS_EXT);
	if (OPT::bFromOpenMVS) {
		if (OPT::strOutputFileName.empty())
			OPT::strOutputFileName = Util::getFilePath(OPT::strInputFileName);
	} else {
		Util::ensureFolderSlash(OPT::strInputFileName);
		if (OPT::strOutputFileName.empty())
			OPT::strOutputFileName = OPT::strInputFileName + _T("scene") MVS_EXT;
		else
			OPT::strImageFolder = Util::getRelativePath(Util::getFilePath(OPT::strOutputFileName), OPT::strInputFileName+OPT::strImageFolder);
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

namespace COLMAP {

using namespace colmap;

// See colmap/src/util/types.h
typedef uint32_t camera_t;
typedef uint32_t image_t;
typedef uint64_t image_pair_t;
typedef uint32_t point2D_t;
typedef uint64_t point3D_t;

typedef std::unordered_map<camera_t,String> CameraModelMap;
CameraModelMap mapCameraModel;

void DefineCameraModels() {
	COLMAP::mapCameraModel.emplace(0, "SIMPLE_PINHOLE");
	COLMAP::mapCameraModel.emplace(1, "PINHOLE");
	COLMAP::mapCameraModel.emplace(2, "SIMPLE_RADIAL");
	COLMAP::mapCameraModel.emplace(3, "RADIAL");
	COLMAP::mapCameraModel.emplace(4, "OPENCV");
	COLMAP::mapCameraModel.emplace(5, "OPENCV_FISHEYE");
	COLMAP::mapCameraModel.emplace(6, "FULL_OPENCV");
	COLMAP::mapCameraModel.emplace(7, "FOV");
	COLMAP::mapCameraModel.emplace(8, "SIMPLE_RADIAL_FISHEYE");
	COLMAP::mapCameraModel.emplace(9, "RADIAL_FISHEYE");
	COLMAP::mapCameraModel.emplace(10, "THIN_PRISM_FISHEYE");
}

// tools
bool NextLine(std::istream& stream, std::istringstream& in, bool bIgnoreEmpty=true) {
	String line;
	do {
		std::getline(stream, line);
		Util::strTrim(line, _T(" "));
		if (stream.fail())
			return false;
	} while (((bIgnoreEmpty && line.empty()) || line[0u] == '#') && stream.good());
	in.clear();
	in.str(line);
	return true;
}
// structure describing a camera
struct Camera {
	uint32_t ID; // ID of the camera
	String model; // camera model name
	uint32_t width, height; // camera resolution
	std::vector<REAL> params; // camera parameters
	bool parsedNumCameras = false;

	Camera() {}
	Camera(uint32_t _ID) : ID(_ID) {}
	bool operator < (const Camera& rhs) const { return ID < rhs.ID; }

	struct CameraHash {
		size_t operator()(const Camera& camera) const {
			const size_t h1(std::hash<String>()(camera.model));
			const size_t h2(std::hash<uint32_t>()(camera.width));
			const size_t h3(std::hash<uint32_t>()(camera.height));
			size_t h(h1 ^ ((h2 ^ (h3 << 1)) << 1));
			for (REAL p: camera.params)
				h = std::hash<REAL>()(p) ^ (h << 1);
			return h;
		}
	};
	struct CameraEqualTo {
		bool operator()(const Camera& _Left, const Camera& _Right) const {
			return _Left.model == _Right.model &&
				_Left.width == _Right.width && _Left.height == _Right.height &&
				_Left.params == _Right.params;
		}
	};

	bool Read(std::istream& stream, bool binary) {
		if (binary)
			return ReadBIN(stream);
		return ReadTXT(stream);
	}	

	// Camera list with one line of data per camera:
	//   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
	bool ReadTXT(std::istream& stream) {
		std::istringstream in;
		if (!NextLine(stream, in))
			return false;
		in >> ID >> model >> width >> height;
		if (in.fail())
			return false;
		--ID;
		if (model != _T("PINHOLE"))
			return false;
		params.resize(4);
		in >> params[0] >> params[1] >> params[2] >> params[3];
		return !in.fail();
	}

	// See: colmap/src/base/reconstruction.cc
	// 		void Reconstruction::ReadCamerasBinary(const std::string& path)
	bool ReadBIN(std::istream& stream) {
		if (stream.peek() == EOF)
			return false;

		if (!parsedNumCameras) {
			// Read the first entry in the binary file
			ReadBinaryLittleEndian<uint64_t>(&stream);
			parsedNumCameras = true;
		}

		ID = ReadBinaryLittleEndian<camera_t>(&stream)-1;
		model = mapCameraModel.at(ReadBinaryLittleEndian<int>(&stream));
		width = (uint32_t)ReadBinaryLittleEndian<uint64_t>(&stream);
		height = (uint32_t)ReadBinaryLittleEndian<uint64_t>(&stream);
		if (model != _T("PINHOLE"))
			return false;
		params.resize(4);
		ReadBinaryLittleEndian<double>(&stream, &params);
		return true;
	}

	bool Write(std::ostream& out) const {
		out << ID+1 << _T(" ") << model << _T(" ") << width << _T(" ") << height;
		if (out.fail())
			return false;
		for (REAL param: params) {
			out << _T(" ") << param;
			if (out.fail())
				return false;
		}
		out << std::endl;
		return true;
	}
};
typedef std::vector<Camera> Cameras;
// structure describing an image
struct Image {
	struct Proj {
		Eigen::Vector2f p;
		uint32_t idPoint;
	};
	uint32_t ID; // ID of the image
	Eigen::Quaterniond q; // rotation
	Eigen::Vector3d t; // translation
	uint32_t idCamera; // ID of the associated camera
	String name; // image file name
	std::vector<Proj> projs; // known image projections
	bool parsedNumRegImages = false;

	Image() {}
	Image(uint32_t _ID) : ID(_ID) {}
	bool operator < (const Image& rhs) const { return ID < rhs.ID; }

	bool Read(std::istream& stream, bool binary) {
		if (binary)
			return ReadBIN(stream); 
		return ReadTXT(stream); 
	}

	// Image list with two lines of data per image:
	//   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
	//   POINTS2D[] as (X, Y, POINT3D_ID)
	bool ReadTXT(std::istream& stream) {
		std::istringstream in;
		if (!NextLine(stream, in))
			return false;
		in  >> ID
			>> q.w() >> q.x() >> q.y() >> q.z()
			>> t(0) >> t(1) >> t(2)
			>> idCamera >> name;
		if (in.fail())
			return false;
		--ID; --idCamera;
		Util::ensureValidPath(name);
		if (!NextLine(stream, in, false))
			return false;
		projs.clear();
		while (true) {
			Proj proj;
			in >> proj.p(0) >> proj.p(1) >> (int&)proj.idPoint;
			if (in.fail())
				break;
			projs.emplace_back(proj);
		}
		return true;
	}

	// See: colmap/src/base/reconstruction.cc
	// 		void Reconstruction::ReadImagesBinary(const std::string& path)
	bool ReadBIN(std::istream& stream) {
		if (stream.peek() == EOF)
			return false;

		if (!parsedNumRegImages) {
			// Read the first entry in the binary file
			ReadBinaryLittleEndian<uint64_t>(&stream);
			parsedNumRegImages = true;
		}

		ID = ReadBinaryLittleEndian<image_t>(&stream)-1;
		q.w() = ReadBinaryLittleEndian<double>(&stream);
		q.x() = ReadBinaryLittleEndian<double>(&stream);
		q.y() = ReadBinaryLittleEndian<double>(&stream);
		q.z() = ReadBinaryLittleEndian<double>(&stream);
		t(0) = ReadBinaryLittleEndian<double>(&stream);
		t(1) = ReadBinaryLittleEndian<double>(&stream);
		t(2) = ReadBinaryLittleEndian<double>(&stream);
		idCamera = ReadBinaryLittleEndian<camera_t>(&stream)-1;

		name = "";
		char nameChar;
		do {
			stream.read(&nameChar, 1);
			if (nameChar != '\0') {
				name += nameChar;
			}
		} while (nameChar != '\0');
		Util::ensureValidPath(name);

		const size_t numPoints2D = ReadBinaryLittleEndian<uint64_t>(&stream);
		projs.clear();
		for (size_t j = 0; j < numPoints2D; ++j) {
			Proj proj;
			proj.p(0) = (float)ReadBinaryLittleEndian<double>(&stream);
			proj.p(1) = (float)ReadBinaryLittleEndian<double>(&stream);
			proj.idPoint = (uint32_t)ReadBinaryLittleEndian<point3D_t>(&stream);
			projs.emplace_back(proj);
		}
		return true;
	}

	bool Write(std::ostream& out) const {
		out << ID+1 << _T(" ")
			<< q.w() << _T(" ") << q.x() << _T(" ") << q.y() << _T(" ") << q.z() << _T(" ")
			<< t(0) << _T(" ") << t(1) << _T(" ") << t(2) << _T(" ")
			<< idCamera+1 << _T(" ") << name
			<< std::endl;
		for (const Proj& proj: projs) {
			out << proj.p(0) << _T(" ") << proj.p(1) << _T(" ") << (int)proj.idPoint+1 << _T(" ");
			if (out.fail())
				return false;
		}
		out << std::endl;
		return !out.fail();
	}
};
typedef std::vector<Image> Images;
// structure describing a 3D point
struct Point {
	struct Track {
		uint32_t idImage;
		uint32_t idProj;
	};
	uint32_t ID; // ID of the point
	Interface::Pos3f p; // position
	Interface::Col3 c; // BGR color
	float e; // error
	std::vector<Track> tracks; // point track
	bool parsedNumPoints3D = false;

	Point() {}
	Point(uint32_t _ID) : ID(_ID) {}
	bool operator < (const Image& rhs) const { return ID < rhs.ID; }

	bool Read(std::istream& stream, bool binary) {
		if (binary)
			return ReadBIN(stream);
		return ReadTXT(stream);
	}

	// 3D point list with one line of data per point:
	//   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
	bool ReadTXT(std::istream& stream) {
		std::istringstream in;
		if (!NextLine(stream, in))
			return false;
		int r,g,b;
		in  >> ID
			>> p.x >> p.y >> p.z
			>> r >> g >> b
			>> e;
		c.x = CLAMP(b,0,255);
		c.y = CLAMP(g,0,255);
		c.z = CLAMP(r,0,255);
		if (in.fail())
			return false;
		--ID;
		tracks.clear();
		while (true) {
			Track track;
			in >> track.idImage >> track.idProj;
			if (in.fail())
				break;
			--track.idImage; --track.idProj;
			tracks.emplace_back(track);
		}
		return !tracks.empty();
	}

	// See: colmap/src/base/reconstruction.cc
	// 		void Reconstruction::ReadPoints3DBinary(const std::string& path)
	bool ReadBIN(std::istream& stream) {
		if (stream.peek() == EOF)
			return false;

		if (!parsedNumPoints3D) {
			// Read the first entry in the binary file
			ReadBinaryLittleEndian<uint64_t>(&stream);
			parsedNumPoints3D = true;
		}

		int r,g,b;
		ID = (uint32_t)ReadBinaryLittleEndian<point3D_t>(&stream)-1;
		p.x = (float)ReadBinaryLittleEndian<double>(&stream);
		p.y = (float)ReadBinaryLittleEndian<double>(&stream);
		p.z = (float)ReadBinaryLittleEndian<double>(&stream);
		r = ReadBinaryLittleEndian<uint8_t>(&stream);
		g = ReadBinaryLittleEndian<uint8_t>(&stream);
		b = ReadBinaryLittleEndian<uint8_t>(&stream);
		e = (float)ReadBinaryLittleEndian<double>(&stream);
		c.x = CLAMP(b,0,255);
		c.y = CLAMP(g,0,255);
		c.z = CLAMP(r,0,255);
		
		const size_t trackLength = ReadBinaryLittleEndian<uint64_t>(&stream);
		tracks.clear();
		for (size_t j = 0; j < trackLength; ++j) {
			Track track;
			track.idImage = ReadBinaryLittleEndian<image_t>(&stream)-1;
			track.idProj = ReadBinaryLittleEndian<point2D_t>(&stream)-1;
			tracks.emplace_back(track);
    	}
		return !tracks.empty();
	}

	bool Write(std::ostream& out) const {
		ASSERT(!tracks.empty());
		const int r(c.z),g(c.y),b(c.x);
		out << ID+1 << _T(" ")
			<< p.x << _T(" ") << p.y << _T(" ") << p.z << _T(" ")
			<< r << _T(" ") << g << _T(" ") << b << _T(" ")
			<< e << _T(" ");
		for (const Track& track: tracks) {
			out << track.idImage+1 << _T(" ") << track.idProj+1 << _T(" ");
			if (out.fail())
				return false;
		}
		out << std::endl;
		return !out.fail();
	}
};
typedef std::vector<Point> Points;
// structure describing an 2D dynamic matrix
template <typename T>
struct Mat {
	size_t width_ = 0;
	size_t height_ = 0;
	size_t depth_ = 0;
	std::vector<T> data_;

	size_t GetNumBytes() const {
		return data_.size() * sizeof(T);
	}
	const T* GetChannelPtr(size_t c) const {
		return data_.data()+width_*height_*c;
	}

	// See: colmap/src/mvs/mat.h
	void Read(const std::string& path) {
		std::streampos pos; {
			std::fstream text_file(path, std::ios::in | std::ios::binary);
			char unused_char;
			text_file >> width_ >> unused_char >> height_ >> unused_char >> depth_ >>
				unused_char;
			pos = text_file.tellg();
		}
		data_.resize(width_ * height_ * depth_);
		std::fstream binary_file(path, std::ios::in | std::ios::binary);
		binary_file.seekg(pos);
		ReadBinaryLittleEndian<T>(&binary_file, &data_);
	}
	void Write(const std::string& path) const {
		{
			std::fstream text_file(path, std::ios::out);
			text_file << width_ << "&" << height_ << "&" << depth_ << "&";
		}
		std::fstream binary_file(path, std::ios::out | std::ios::binary | std::ios::app);
		WriteBinaryLittleEndian<T>(&binary_file, data_);
	}
};
} // namespace COLMAP

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> EMat33d;
typedef Eigen::Matrix<double,3,1> EVec3d;


bool DetermineInputSource(const String& filenameTXT, const String& filenameBIN, std::ifstream& file, String& filenameCamera, bool& binary)
{
	file.open(filenameTXT);
	if (file.good()) {
		filenameCamera = filenameTXT;
		binary = false;
		return true;
	}
	file.open(filenameBIN, std::ios::binary);
	if (file.good()) {
		filenameCamera = filenameBIN;
		binary = true;
		return true;
	}
	VERBOSE("error: unable to open file '%s'", filenameTXT.c_str());
	VERBOSE("error: unable to open file '%s'", filenameBIN.c_str());
	return false;
}


bool ImportScene(const String& strFolder, const String& strOutFolder, Interface& scene)
{
	COLMAP::DefineCameraModels();
	// read camera list
	typedef std::unordered_map<uint32_t,uint32_t> CamerasMap;
	CamerasMap mapCameras;
	{
		const String filenameCamerasTXT(strFolder+COLMAP_CAMERAS_TXT);
		const String filenameCamerasBIN(strFolder+COLMAP_CAMERAS_BIN);
		std::ifstream file;
		bool binary;
		String filenameCamera;
		if (!DetermineInputSource(filenameCamerasTXT, filenameCamerasBIN, file, filenameCamera, binary)) {
			return false;
		}
		LOG_OUT() << "Reading cameras: " << filenameCamera << std::endl;
	
		typedef std::unordered_map<COLMAP::Camera,uint32_t,COLMAP::Camera::CameraHash,COLMAP::Camera::CameraEqualTo> CamerasSet;
		CamerasSet setCameras;
		COLMAP::Camera colmapCamera;
		while (file.good() && colmapCamera.Read(file, binary)) {
			const auto setIt(setCameras.emplace(colmapCamera, (uint32_t)scene.platforms.size()));
			mapCameras.emplace(colmapCamera.ID, setIt.first->second);
			if (!setIt.second) {
				// reuse existing platform
				continue;
			}
			// create new platform
			Interface::Platform platform;
			platform.name = String::FormatString(_T("platform%03u"), colmapCamera.ID); // only one camera per platform supported
			Interface::Platform::Camera camera;
			camera.name = colmapCamera.model;
			camera.K = Interface::Mat33d::eye();
			camera.K(0,0) = colmapCamera.params[0];
			camera.K(1,1) = colmapCamera.params[1];
			camera.K(0,2) = colmapCamera.params[2];
			camera.K(1,2) = colmapCamera.params[3];
			camera.R = Interface::Mat33d::eye();
			camera.C = Interface::Pos3d(0,0,0);
			if (OPT::bNormalizeIntrinsics) {
				// normalize camera intrinsics
				camera.K = Camera::ScaleK<double>(camera.K, 1.0/Camera::GetNormalizationScale(colmapCamera.width, colmapCamera.height));
			} else {
				camera.width = colmapCamera.width;
				camera.height = colmapCamera.height;
			}
			platform.cameras.emplace_back(camera);
			scene.platforms.emplace_back(platform);
		}
	}
	if (mapCameras.empty()) {
		VERBOSE("error: no valid cameras (make sure they are in PINHOLE model)");
		return false;
	}

	// read images list
	typedef std::map<COLMAP::Image, uint32_t> ImagesMap;
	ImagesMap mapImages;
	{
		const String filenameImagesTXT(strFolder+COLMAP_IMAGES_TXT);
		const String filenameImagesBIN(strFolder+COLMAP_IMAGES_BIN);
		std::ifstream file;
		bool binary;
		String filenameImages;
		if (!DetermineInputSource(filenameImagesTXT, filenameImagesBIN, file, filenameImages, binary)) {
			return false;
		}
		LOG_OUT() << "Reading images: " << filenameImages << std::endl;

		COLMAP::Image imageColmap;
		while (file.good() && imageColmap.Read(file, binary)) {
			mapImages.emplace(imageColmap, (uint32_t)scene.images.size());
			Interface::Platform::Pose pose;
			Eigen::Map<EMat33d>(pose.R.val) = imageColmap.q.toRotationMatrix();
			EnsureRotationMatrix((Matrix3x3d&)pose.R);
			Eigen::Map<EVec3d>(&pose.C.x) = -(imageColmap.q.inverse() * imageColmap.t);
			Interface::Image image;
			image.name = MAKE_PATH_REL(strOutFolder,OPT::strImageFolder+imageColmap.name);
			image.platformID = mapCameras.at(imageColmap.idCamera);
			image.cameraID = 0;
			image.ID = imageColmap.ID;
			Interface::Platform& platform = scene.platforms[image.platformID];
			image.poseID = (uint32_t)platform.poses.size();
			platform.poses.emplace_back(pose);
			scene.images.emplace_back(image);
		}
	}

	// read points list
	const String filenameDensePoints(strFolder+COLMAP_DENSE_POINTS);
	const String filenameDenseVisPoints(strFolder+COLMAP_DENSE_POINTS_VISIBILITY);
	if (!File::access(filenameDensePoints) || !File::access(filenameDenseVisPoints)) {
		// parse sparse point-cloud
		const String filenamePointsTXT(strFolder+COLMAP_POINTS_TXT);
		const String filenamePointsBIN(strFolder+COLMAP_POINTS_BIN);
		std::ifstream file;
		bool binary;
		String filenamePoints;
		if (!DetermineInputSource(filenamePointsTXT, filenamePointsBIN, file, filenamePoints, binary)) {
			return false;
		}
		LOG_OUT() << "Reading points: " << filenamePoints << std::endl;

		COLMAP::Point point;
		while (file.good() && point.Read(file, binary)) {
			Interface::Vertex vertex;
			vertex.X = point.p;
			for (const COLMAP::Point::Track& track: point.tracks) {
				Interface::Vertex::View view;
				view.imageID = mapImages.at(COLMAP::Image(track.idImage));
				view.confidence = 0;
				vertex.views.emplace_back(view);
			}
			std::sort(vertex.views.begin(), vertex.views.end(),
				[](const Interface::Vertex::View& view0, const Interface::Vertex::View& view1) { return view0.imageID < view1.imageID; });
			scene.vertices.emplace_back(std::move(vertex));
			scene.verticesColor.emplace_back(Interface::Color{point.c});
		}
	} else {
		// parse dense point-cloud
		LOG_OUT() << "Reading points: " << filenameDensePoints << " and " << filenameDenseVisPoints << std::endl;
		PointCloud pointcloud;
		if (!pointcloud.Load(filenameDensePoints)) {
			VERBOSE("error: unable to open file '%s'", filenameDensePoints.c_str());
			return false;
		}
		File file(filenameDenseVisPoints, File::READ, File::OPEN);
		if (!file.isOpen()) {
			VERBOSE("error: unable to open file '%s'", filenameDenseVisPoints.c_str());
			return false;
		}
		uint64_t numPoints(0);
		file.read(&numPoints, sizeof(uint64_t));
		if (pointcloud.GetSize() != numPoints) {
			VERBOSE("error: point-cloud and visibility have different size");
			return false;
		}
		for (size_t i=0; i<numPoints; ++i) {
			Interface::Vertex vertex;
			vertex.X = pointcloud.points[i];
			uint32_t numViews(0);
			file.read(&numViews, sizeof(uint32_t));
			for (uint32_t v=0; v<numViews; ++v) {
				Interface::Vertex::View view;
				file.read(&view.imageID, sizeof(uint32_t));
				view.confidence = 0;
				vertex.views.emplace_back(view);
			}
			std::sort(vertex.views.begin(), vertex.views.end(),
				[](const Interface::Vertex::View& view0, const Interface::Vertex::View& view1) { return view0.imageID < view1.imageID; });
			scene.vertices.emplace_back(std::move(vertex));
			scene.verticesNormal.emplace_back(Interface::Normal{pointcloud.normals[i]});
			scene.verticesColor.emplace_back(Interface::Color{pointcloud.colors[i]});
		}
	}

	// read depth-maps
	const String pathDepthMaps(strFolder+COLMAP_STEREO_DEPTHMAPS_FOLDER);
	const String pathNormalMaps(strFolder+COLMAP_STEREO_NORMALMAPS_FOLDER);
	if (File::isFolder(pathDepthMaps) && File::isFolder(pathNormalMaps)) {
		// read patch-match list
		CLISTDEF2IDX(IIndexArr,IIndex) imagesNeighbors((IIndex)scene.images.size());
		{
			const String filenameFusion(strFolder+COLMAP_PATCHMATCH);
			LOG_OUT() << "Reading patch-match configuration: " << filenameFusion << std::endl;
			std::ifstream file(filenameFusion);
			if (!file.good()) {
				VERBOSE("error: unable to open file '%s'", filenameFusion.c_str());
				return false;
			}
			while (true) {
				String imageName, neighbors;
				std::getline(file, imageName);
				std::getline(file, neighbors);
				if (file.fail() || imageName.empty() || neighbors.empty())
					break;
				const ImagesMap::const_iterator it_image = std::find_if(mapImages.begin(), mapImages.end(),
					[&imageName](const ImagesMap::value_type& image) {
						return image.first.name == imageName;
					});
				if (it_image == mapImages.end())
					continue;
				IIndexArr& imageNeighbors =  imagesNeighbors[it_image->second];
				CLISTDEF2(String) neighborNames;
				Util::strSplit(neighbors, _T(','), neighborNames);
				FOREACH(i, neighborNames) {
					String& neighborName = neighborNames[i];
					Util::strTrim(neighborName, _T(" "));
					const ImagesMap::const_iterator it_neighbor = std::find_if(mapImages.begin(), mapImages.end(),
						[&neighborName](const ImagesMap::value_type& image) {
							return image.first.name == neighborName;
						});
					if (it_neighbor == mapImages.end()) {
						if (i == 0)
							break;
						continue;
					}
					imageNeighbors.emplace_back(scene.images[it_neighbor->second].ID);
				}
			}
		}
		LOG_OUT() << "Reading depth-maps/normal-maps: " << pathDepthMaps << " and " << pathNormalMaps << std::endl;
		Util::ensureFolder(strOutFolder);
		const String strType[] = {".geometric.bin", ".photometric.bin"};
		FOREACH(idx, scene.images) {
			const Interface::Image& image = scene.images[idx];
			COLMAP::Mat<float> colDepthMap, colNormalMap;
			const String filenameImage(Util::getFileNameExt(image.name));
			for (int i=0; i<2; ++i) {
				const String filenameDepthMaps(pathDepthMaps+filenameImage+strType[i]);
				if (File::isFile(filenameDepthMaps)) {
					colDepthMap.Read(filenameDepthMaps);
					const String filenameNormalMaps(pathNormalMaps+filenameImage+strType[i]);
					if (File::isFile(filenameNormalMaps)) {
						colNormalMap.Read(filenameNormalMaps);
					}
					break;
				}
			}
			if (!colDepthMap.data_.empty()) {
				IIndexArr IDs = {image.ID};
				IDs.Join(imagesNeighbors[(IIndex)idx]);
				const Interface::Platform& platform = scene.platforms[image.platformID];
				const Interface::Platform::Pose pose(platform.GetPose(image.cameraID, image.poseID));
				const Interface::Mat33d K(platform.GetFullK(image.cameraID, (uint32_t)colDepthMap.width_, (uint32_t)colDepthMap.height_));
				MVS::DepthMap depthMap((int)colDepthMap.height_, (int)colDepthMap.width_);
				memcpy(depthMap.getData(), colDepthMap.data_.data(), colDepthMap.GetNumBytes());
				MVS::NormalMap normalMap;
				if (!colNormalMap.data_.empty()) {
					normalMap.create((int)colNormalMap.height_, (int)colNormalMap.width_);
					cv::merge(std::vector<cv::Mat>{
						cv::Mat((int)colNormalMap.height_, (int)colNormalMap.width_, CV_32F, (void*)colNormalMap.GetChannelPtr(0)),
						cv::Mat((int)colNormalMap.height_, (int)colNormalMap.width_, CV_32F, (void*)colNormalMap.GetChannelPtr(1)),
						cv::Mat((int)colNormalMap.height_, (int)colNormalMap.width_, CV_32F, (void*)colNormalMap.GetChannelPtr(2))
					}, normalMap);
				}
				MVS::ConfidenceMap confMap;
				const auto depthMM(std::minmax_element(colDepthMap.data_.cbegin(), colDepthMap.data_.cend()));
				const MVS::Depth dMin(*depthMM.first), dMax(*depthMM.second);
				if (!ExportDepthDataRaw(strOutFolder+String::FormatString("depth%04u.dmap", image.ID), MAKE_PATH_FULL(strOutFolder, image.name), IDs, depthMap.size(), K, pose.R, pose.C, dMin, dMax, depthMap, normalMap, confMap))
					return false;
			}
		}
	}
	return true;
}


bool ExportScene(const String& strFolder, const Interface& scene)
{
	Util::ensureFolder(strFolder+COLMAP_SPARSE_FOLDER);

	// write camera list
	CLISTDEF0IDX(KMatrix,uint32_t) Ks;
	CLISTDEF0IDX(COLMAP::Camera,uint32_t) cams;
	{
		const String filenameCameras(strFolder+COLMAP_CAMERAS_TXT);
		LOG_OUT() << "Writing cameras: " << filenameCameras << std::endl;
		std::ofstream file(filenameCameras);
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", filenameCameras.c_str());
			return false;
		}
		file << _T("# Camera list with one line of data per camera:") << std::endl;
		file << _T("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]") << std::endl;
		COLMAP::Camera cam;
		cam.model = _T("PINHOLE");
		cam.params.resize(4);
		for (uint32_t ID=0; ID<(uint32_t)scene.platforms.size(); ++ID) {
			const Interface::Platform& platform = scene.platforms[ID];
			ASSERT(platform.cameras.size() == 1); // only one camera per platform supported
			const Interface::Platform::Camera& camera = platform.cameras[0];
			cam.ID = ID;
			if (camera.width == 0 || camera.height == 0) {
				// find one image using this camera
				const Interface::Image* pImage(NULL);
				for (uint32_t i=0; i<(uint32_t)scene.images.size(); ++i) {
					const Interface::Image& image = scene.images[i];
					if (image.platformID == ID && image.cameraID == 0 && image.poseID != MVS::NO_ID) {
						pImage = &image;
						break;
					}
				}
				if (pImage == NULL) {
					LOG("error: no image using camera %u of platform %u", 0, ID);
					continue;
				}
				IMAGEPTR ptrImage(Image::ReadImageHeader(MAKE_PATH_SAFE(pImage->name.c_str())));
				if (ptrImage == NULL)
					return false;
				cam.width = ptrImage->GetWidth();
				cam.height = ptrImage->GetHeight();
				// unnormalize camera intrinsics
				const Interface::Mat33d K(platform.GetFullK(0, cam.width, cam.height));
				cam.params[0] = K(0,0);
				cam.params[1] = K(1,1);
				cam.params[2] = K(0,2);
				cam.params[3] = K(1,2);
			} else {
				cam.width = camera.width;
				cam.height = camera.height;
				cam.params[0] = camera.K(0,0);
				cam.params[1] = camera.K(1,1);
				cam.params[2] = camera.K(0,2);
				cam.params[3] = camera.K(1,2);
			}
			if (!cam.Write(file))
				return false;
			KMatrix& K = Ks.emplace_back(KMatrix::IDENTITY);
			K(0,0) = cam.params[0];
			K(1,1) = cam.params[1];
			K(0,2) = cam.params[2];
			K(1,2) = cam.params[3];
			cams.emplace_back(cam);
		}
	}

	// create images list
	COLMAP::Images images;
	CameraArr cameras;
	float maxNumPointsSparse(0);
	const float avgViewsPerPoint(3.f);
	const uint32_t avgResolutionSmallView(640*480), avgResolutionLargeView(6000*4000);
	const uint32_t avgPointsPerSmallView(3000), avgPointsPerLargeView(12000);
	{
		images.resize(scene.images.size());
		cameras.resize(scene.images.size());
		for (uint32_t ID=0; ID<(uint32_t)scene.images.size(); ++ID) {
			const Interface::Image& image = scene.images[ID];
			if (image.poseID == MVS::NO_ID)
				continue;
			const Interface::Platform& platform = scene.platforms[image.platformID];
			const Interface::Platform::Pose& pose = platform.poses[image.poseID];
			ASSERT(image.cameraID == 0);
			COLMAP::Image& img = images[ID];
			img.ID = ID;
			img.q = Eigen::Quaterniond(Eigen::Map<const EMat33d>(pose.R.val));
			img.t = -(img.q * Eigen::Map<const EVec3d>(&pose.C.x));
			img.idCamera = image.platformID;
			img.name = MAKE_PATH_REL(OPT::strImageFolder, image.name);
			Camera& camera = cameras[ID];
			camera.K = Ks[image.platformID];
			camera.R = pose.R;
			camera.C = pose.C;
			camera.ComposeP();
			const COLMAP::Camera& cam = cams[image.platformID];
			const uint32_t resolutionView(cam.width*cam.height);
			const float linearFactor(float(avgResolutionLargeView-resolutionView)/(avgResolutionLargeView-avgResolutionSmallView));
			maxNumPointsSparse += (avgPointsPerSmallView+(avgPointsPerLargeView-avgPointsPerSmallView)*linearFactor)/avgViewsPerPoint;
		}
	}

	// auto-select dense or sparse mode based on number of points
	const bool bSparsePointCloud(scene.vertices.size() < (size_t)maxNumPointsSparse);
	if (bSparsePointCloud) {
		// write points list
		{
			const String filenamePoints(strFolder+COLMAP_POINTS_TXT);
			LOG_OUT() << "Writing points: " << filenamePoints << std::endl;
			std::ofstream file(filenamePoints);
			if (!file.good()) {
				VERBOSE("error: unable to open file '%s'", filenamePoints.c_str());
				return false;
			}
			file << _T("# 3D point list with one line of data per point:") << std::endl;
			file << _T("#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)") << std::endl;
			for (uint32_t ID=0; ID<(uint32_t)scene.vertices.size(); ++ID) {
				const Interface::Vertex& vertex = scene.vertices[ID];
				COLMAP::Point point;
				point.ID = ID;
				point.p = vertex.X;
				for (const Interface::Vertex::View& view: vertex.views) {
					COLMAP::Image& img = images[view.imageID];
					point.tracks.emplace_back(COLMAP::Point::Track{view.imageID, (uint32_t)img.projs.size()});
					COLMAP::Image::Proj proj;
					proj.idPoint = ID;
					const Point3 X(vertex.X);
					ProjectVertex_3x4_3_2(cameras[view.imageID].P.val, X.ptr(), proj.p.data());
					img.projs.emplace_back(proj);
				}
				point.c = scene.verticesColor.empty() ? Interface::Col3(255,255,255) : scene.verticesColor[ID].c;
				point.e = 0;
				if (!point.Write(file))
					return false;
			}
		}

		Util::ensureFolder(strFolder+COLMAP_STEREO_FOLDER);

		// write fusion list
		{
			const String filenameFusion(strFolder+COLMAP_FUSION);
			LOG_OUT() << "Writing fusion configuration: " << filenameFusion << std::endl;
			std::ofstream file(filenameFusion);
			if (!file.good()) {
				VERBOSE("error: unable to open file '%s'", filenameFusion.c_str());
				return false;
			}
			for (const COLMAP::Image& img: images) {
				if (img.projs.empty())
					continue;
				file << img.name << std::endl;
				if (file.fail())
					return false;
			}
		}

		// write patch-match list
		{
			const String filenameFusion(strFolder+COLMAP_PATCHMATCH);
			LOG_OUT() << "Writing patch-match configuration: " << filenameFusion << std::endl;
			std::ofstream file(filenameFusion);
			if (!file.good()) {
				VERBOSE("error: unable to open file '%s'", filenameFusion.c_str());
				return false;
			}
			for (const COLMAP::Image& img: images) {
				if (img.projs.empty())
					continue;
				file << img.name << std::endl;
				if (file.fail())
					return false;
				file << _T("__auto__, 20") << std::endl;
				if (file.fail())
					return false;
			}
		}

		Util::ensureFolder(strFolder+COLMAP_STEREO_CONSISTENCYGRAPHS_FOLDER);
		Util::ensureFolder(strFolder+COLMAP_STEREO_DEPTHMAPS_FOLDER);
		Util::ensureFolder(strFolder+COLMAP_STEREO_NORMALMAPS_FOLDER);
	} else {
		// export dense point-cloud
		const String filenameDensePoints(strFolder+COLMAP_DENSE_POINTS);
		const String filenameDenseVisPoints(strFolder+COLMAP_DENSE_POINTS_VISIBILITY);
		LOG_OUT() << "Writing points: " << filenameDensePoints << " and " << filenameDenseVisPoints << std::endl;
		File file(filenameDenseVisPoints, File::WRITE, File::CREATE | File::TRUNCATE);
		if (!file.isOpen()) {
			VERBOSE("error: unable to write file '%s'", filenameDenseVisPoints.c_str());
			return false;
		}
		const uint64_t numPoints(scene.vertices.size());
		file.write(&numPoints, sizeof(uint64_t));
		PointCloud pointcloud;
		for (size_t i=0; i<numPoints; ++i) {
			const Interface::Vertex& vertex = scene.vertices[i];
			pointcloud.points.emplace_back(vertex.X);
			if (!scene.verticesNormal.empty())
				pointcloud.normals.emplace_back(scene.verticesNormal[i].n);
			if (!scene.verticesColor.empty())
				pointcloud.colors.emplace_back(scene.verticesColor[i].c);
			const uint32_t numViews((uint32_t)vertex.views.size());
			file.write(&numViews, sizeof(uint32_t));
			for (uint32_t v=0; v<numViews; ++v) {
				const Interface::Vertex::View& view = vertex.views[v];
				file.write(&view.imageID, sizeof(uint32_t));
			}
		}
		if (!pointcloud.Save(filenameDensePoints, true)) {
			VERBOSE("error: unable to write file '%s'", filenameDensePoints.c_str());
			return false;
		}
	}

	// write images list
	{
		const String filenameImages(strFolder+COLMAP_IMAGES_TXT);
		LOG_OUT() << "Writing images: " << filenameImages << std::endl;
		std::ofstream file(filenameImages);
		if (!file.good()) {
			VERBOSE("error: unable to open file '%s'", filenameImages.c_str());
			return false;
		}
		file << _T("# Image list with two lines of data per image:") << std::endl;
		file << _T("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME") << std::endl;
		file << _T("#   POINTS2D[] as (X, Y, POINT3D_ID)") << std::endl;
		for (const COLMAP::Image& img: images) {
			if (bSparsePointCloud && img.projs.empty())
				continue;
			if (!img.Write(file))
				return false;
		}
	}
	return true;
}


// export image poses in log format
// see: http://redwood-data.org/indoor/fileformat.html
// to support: https://www.tanksandtemples.org/tutorial
bool ExportImagesLog(const String& fileName, const Interface& scene)
{
	LOG_OUT() << "Writing poses: " << fileName << std::endl;
	Util::ensureFolder(fileName);
	std::ofstream out(fileName);
	if (!out.good()) {
		VERBOSE("error: unable to open file '%s'", fileName.c_str());
		return false;
	}
	out << std::setprecision(12);
	IIndexArr orderedImages((uint32_t)scene.images.size());
	std::iota(orderedImages.begin(), orderedImages.end(), 0u);
	orderedImages.Sort([&scene](IIndex i, IIndex j) {
		return scene.images[i].ID < scene.images[j].ID;
	});
	for (IIndex ID: orderedImages) {
		const Interface::Image& image = scene.images[ID];
		Eigen::Matrix3d R(Eigen::Matrix3d::Identity());
		Eigen::Vector3d t(Eigen::Vector3d::Zero());
		if (image.poseID != MVS::NO_ID) {
			const Interface::Platform& platform = scene.platforms[image.platformID];
			const Interface::Platform::Pose& pose = platform.poses[image.poseID];
			R = Eigen::Map<const EMat33d>(pose.R.val).transpose();
			t = Eigen::Map<const EVec3d>(&pose.C.x);
		}
		Eigen::Matrix4d T(Eigen::Matrix4d::Identity());
		T.topLeftCorner<3,3>() = R;
		T.topRightCorner<3,1>() = t;
		out << ID << _T(" ") << ID << _T(" ") << 0 << _T("\n");
		out << T(0,0) << _T(" ") << T(0,1) << _T(" ") << T(0,2) << _T(" ") << T(0,3) << _T("\n");
		out << T(1,0) << _T(" ") << T(1,1) << _T(" ") << T(1,2) << _T(" ") << T(1,3) << _T("\n");
		out << T(2,0) << _T(" ") << T(2,1) << _T(" ") << T(2,2) << _T(" ") << T(2,3) << _T("\n");
		out << T(3,0) << _T(" ") << T(3,1) << _T(" ") << T(3,2) << _T(" ") << T(3,3) << _T("\n");
	}
	return !out.fail();
}


// export poses in Strecha camera format:
// Strecha model is P = K[R^T|-R^T t]
// our model is P = K[R|t], t = -RC
bool ExportImagesCamera(const String& pathName, const Interface& scene)
{
	LOG_OUT() << "Writing poses: " << pathName << std::endl;
	Util::ensureFolder(pathName);
	for (uint32_t ID=0; ID<(uint32_t)scene.images.size(); ++ID) {
		const Interface::Image& image = scene.images[ID];
		String imageFileName(image.name);
		Util::ensureValidPath(imageFileName);
		const String fileName(pathName+Util::getFileNameExt(imageFileName)+".camera");
		std::ofstream out(fileName);
		if (!out.good()) {
			VERBOSE("error: unable to open file '%s'", fileName.c_str());
			return false;
		}
		out << std::setprecision(12);
		KMatrix K(KMatrix::IDENTITY);
		RMatrix R(RMatrix::IDENTITY);
		CMatrix t(CMatrix::ZERO);
		unsigned width(0), height(0);
		if (image.platformID != MVS::NO_ID && image.cameraID != MVS::NO_ID) {
			const Interface::Platform& platform = scene.platforms[image.platformID];
			const Interface::Platform::Camera& camera = platform.cameras[image.cameraID];
			if (camera.HasResolution()) {
				width = camera.width;
				height = camera.height;
				K = camera.K;
			} else {
				IMAGEPTR pImage = Image::ReadImageHeader(image.name);
				width = pImage->GetWidth();
				height = pImage->GetHeight();
				K = platform.GetFullK(image.cameraID, width, height);
			}
			if (image.poseID != MVS::NO_ID) {
				const Interface::Platform::Pose& pose = platform.poses[image.poseID];
				R = pose.R.t();
				t = pose.C;
			}
		}
		out << K(0,0) << _T(" ") << K(0,1) << _T(" ") << K(0,2) << _T("\n");
		out << K(1,0) << _T(" ") << K(1,1) << _T(" ") << K(1,2) << _T("\n");
		out << K(2,0) << _T(" ") << K(2,1) << _T(" ") << K(2,2) << _T("\n");
		out << _T("0 0 0") << _T("\n");
		out << R(0,0) << _T(" ") << R(0,1) << _T(" ") << R(0,2) << _T("\n");
		out << R(1,0) << _T(" ") << R(1,1) << _T(" ") << R(1,2) << _T("\n");
		out << R(2,0) << _T(" ") << R(2,1) << _T(" ") << R(2,2) << _T("\n");
		out << t.x << _T(" ") << t.y << _T(" ") << t.z << _T("\n");
		out << width << _T(" ") << height << _T("\n");
		if (out.fail()) {
			VERBOSE("error: unable to write file '%s'", fileName.c_str());
			return false;
		}
	}
	return true;
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

	TD_TIMER_START();

	if (OPT::bFromOpenMVS) {
		// read MVS input data
		Interface scene;
		if (!ARCHIVE::SerializeLoad(scene, MAKE_PATH_SAFE(OPT::strInputFileName)))
			return EXIT_FAILURE;
		if (Util::getFileExt(OPT::strOutputFileName) == _T(".log")) {
			// write poses in log format
			ExportImagesLog(MAKE_PATH_SAFE(OPT::strOutputFileName), scene);
		} else
		if (Util::getFileExt(OPT::strOutputFileName) == _T(".camera")) {
			// write poses in Strecha camera format
			ExportImagesCamera((OPT::strOutputFileName=Util::getFileFullName(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputFileName)))+PATH_SEPARATOR, scene);
		} else {
			// write COLMAP input data
			Util::ensureFolderSlash(OPT::strOutputFileName);
			ExportScene(MAKE_PATH_SAFE(OPT::strOutputFileName), scene);
		}
		VERBOSE("Input data exported: %u images & %u vertices (%s)", scene.images.size(), scene.vertices.size(), TD_TIMER_GET_FMT().c_str());
	} else {
		// read COLMAP input data
		Interface scene;
		const String strOutFolder(Util::getFilePath(MAKE_PATH_FULL(WORKING_FOLDER_FULL, OPT::strOutputFileName)));
		if (!ImportScene(MAKE_PATH_SAFE(OPT::strInputFileName), strOutFolder, scene))
			return EXIT_FAILURE;
		// write MVS input data
		Util::ensureFolder(strOutFolder);
		if (!ARCHIVE::SerializeSave(scene, MAKE_PATH_SAFE(OPT::strOutputFileName)))
			return EXIT_FAILURE;
		VERBOSE("Exported data: %u images & %u vertices (%s)", scene.images.size(), scene.vertices.size(), TD_TIMER_GET_FMT().c_str());
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
