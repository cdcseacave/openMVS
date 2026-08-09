#ifndef _INTERFACE_MVS_H_
#define _INTERFACE_MVS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <fstream>
#include <string>
#include <vector>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>


// D E F I N E S ///////////////////////////////////////////////////

#define MVSI_PROJECT_ID "MVSI" // identifies the project stream
#define MVSI_PROJECT_VER ((uint32_t)7) // identifies the version of a project stream

// set a default namespace name if none given
#ifndef _INTERFACE_NAMESPACE
#define _INTERFACE_NAMESPACE MVS
#endif

// uncomment to enable custom OpenCV data types
// (should be uncommented if OpenCV is not available)
#if !defined(_USE_OPENCV) && !defined(_USE_CUSTOM_CV)
#define _USE_CUSTOM_CV
#endif
#ifdef _USE_OPENCV
// the matrix, point and image types below, and the conversions the depth-data codec
// needs, are OpenCV's own; without it this header defines the little it uses of them
#include <opencv2/core.hpp>
#endif

// set to disable custom NO_ID declaration
#ifndef _DISABLE_NO_ID
#define _INTERFACE_NO_ID
#endif


// S T R U C T S ///////////////////////////////////////////////////

#ifdef _USE_CUSTOM_CV

// element types, same values as OpenCV
#define CV_8U   0
#define CV_8S   1
#define CV_16U  2
#define CV_16S  3
#define CV_32S  4
#define CV_32F  5
#define CV_64F  6
#define CV_MAKETYPE(depth,cn) ((depth) + (((cn)-1) << 3))
#define CV_8UC4  CV_MAKETYPE(CV_8U,4)
#define CV_32FC1 CV_MAKETYPE(CV_32F,1)
#define CV_32FC3 CV_MAKETYPE(CV_32F,3)

namespace cv {

// simple cv::Point3_
template<typename Type>
class Point3_
{
public:
	typedef Type value_type;

	inline Point3_() {}
	inline Point3_(Type _x, Type _y, Type _z) : x(_x), y(_y), z(_z) {}
	template<typename Type2>
	inline Point3_(const Point3_<Type2>& pt) : x(Type(pt.x)), y(Type(pt.y)), z(Type(pt.z)) {}
	#ifdef _USE_EIGEN
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Type,3)
	typedef Eigen::Matrix<Type,3,1> EVec;
	typedef Eigen::Map<EVec> EVecMap;
	template<typename Derived>
	inline Point3_(const Eigen::EigenBase<Derived>& rhs) { operator EVecMap () = rhs; }
	template<typename Derived>
	inline Point3_& operator = (const Eigen::EigenBase<Derived>& rhs) { operator EVecMap () = rhs; return *this; }
	inline operator const EVecMap () const { return EVecMap((Type*)this); }
	inline operator EVecMap () { return EVecMap((Type*)this); }
	#endif

	const Type* ptr() const { return &x; }
	Type* ptr() { return &x; }
	Type operator()(int r) const { return (&x)[r]; }
	Type& operator()(int r) { return (&x)[r]; }
	Point3_ operator - () const {
		return Point3_(
			-x,
			-y,
			-z
		);
	}
	Point3_ operator + (const Point3_& X) const {
		return Point3_(
			x+X.x,
			y+X.y,
			z+X.z
		);
	}
	Point3_ operator - (const Point3_& X) const {
		return Point3_(
			x-X.x,
			y-X.y,
			z-X.z
		);
	}
	Point3_ operator * (Type s) const {
		return Point3_(
			x*s,
			y*s,
			z*s
		);
	}

public:
	Type x, y, z;
};

// simple cv::Matx
template<typename Type, int m, int n>
class Matx
{
public:
	typedef Type value_type;
	enum {
		rows     = m,
		cols     = n,
		channels = rows*cols
	};

	inline Matx() {}
	#ifdef _USE_EIGEN
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Type,m*n)
	typedef Eigen::Matrix<Type,m,n,(n>1?Eigen::RowMajor:Eigen::Default)> EMat;
	typedef Eigen::Map<const EMat> CEMatMap;
	typedef Eigen::Map<EMat> EMatMap;
	template<typename Derived>
	inline Matx(const Eigen::EigenBase<Derived>& rhs) { operator EMatMap () = rhs; }
	template<typename Derived>
	inline Matx& operator = (const Eigen::EigenBase<Derived>& rhs) { operator EMatMap () = rhs; return *this; }
	inline operator CEMatMap() const { return CEMatMap((const Type*)val); }
	inline operator EMatMap () { return EMatMap((Type*)val); }
	#endif

	Type operator()(int r, int c) const { return val[r*n+c]; }
	Type& operator()(int r, int c) { return val[r*n+c]; }
	Point3_<Type> operator * (const Point3_<Type>& X) const {
		Point3_<Type> R;
		for (int r = 0; r < m; r++) {
			R(r) = Type(0);
			for (int c = 0; c < n; c++)
				R(r) += operator()(r,c)*X(c);
		}
		return R;
	}
	template<int k>
	Matx<Type,m,k> operator * (const Matx<Type,n,k>& M) const {
		Matx<Type,m,k> R;
		for (int r = 0; r < m; r++) {
			for (int l = 0; l < k; l++) {
				R(r,l) = Type(0);
				for (int c = 0; c < n; c++)
					R(r,l) += operator()(r,c)*M(c,l);
			}
		}
		return R;
	}
	Matx<Type,n,m> t() const {
		Matx<Type,n,m> M;
		for (int r = 0; r < m; r++)
			for (int c = 0; c < n; c++)
				M(c,r) = operator()(r,c);
		return M;
	}

	static Matx eye() {
		Matx M;
		memset(M.val, 0, sizeof(Type)*m*n);
		const int shortdim(m < n ? m : n);
		for (int i = 0; i < shortdim; i++)
			M(i,i) = 1;
		return M;
	}

public:
	Type val[m*n];
};

// simple cv::Mat: the subset of the interface the depth-data codec below needs, so
// that it is written once against the OpenCV types and works either way; the pixels
// are owned here, where the real one shares them reference-counted
class Mat
{
public:
	inline Mat() : rows(0), cols(0), flags(0), step(0) {}
	inline Mat(int _rows, int _cols, int _type) : rows(0), cols(0), flags(0), step(0) { create(_rows, _cols, _type); }

	inline bool empty() const { return buffer.empty(); }
	inline int type() const { return flags; }
	inline int depth() const { return flags & 7; }
	inline int channels() const { return (flags >> 3) + 1; }
	inline size_t elemSize() const { return (size_t)channels()*DepthSize(depth()); }

	inline void create(int _rows, int _cols, int _type) {
		if (rows == _rows && cols == _cols && flags == _type)
			return;
		rows = _rows; cols = _cols; flags = _type;
		step = (size_t)cols*elemSize();
		buffer.resize(step*(size_t)rows);
	}
	inline void release() { rows = cols = 0; step = 0; buffer.clear(); }

	inline uint8_t* ptr(int r=0) { return buffer.data()+(size_t)r*step; }
	inline const uint8_t* ptr(int r=0) const { return buffer.data()+(size_t)r*step; }
	template<typename Type> inline Type* ptr(int r=0) { return reinterpret_cast<Type*>(buffer.data()+(size_t)r*step); }
	template<typename Type> inline const Type* ptr(int r=0) const { return reinterpret_cast<const Type*>(buffer.data()+(size_t)r*step); }

	static inline size_t DepthSize(int d) {
		switch (d) {
		case CV_8U: case CV_8S: return 1;
		case CV_16U: case CV_16S: return 2;
		case CV_64F: return 8;
		default: return 4;
		}
	}

public:
	int rows, cols; // pixel resolution
	int flags; // element type
	size_t step; // bytes between two consecutive rows
	std::vector<uint8_t> buffer; // the pixels
};

} // namespace cv
#endif
/*----------------------------------------------------------------*/


namespace _INTERFACE_NAMESPACE {

// invalid index
#ifdef _INTERFACE_NO_ID
constexpr uint32_t NO_ID {std::numeric_limits<uint32_t>::max()};
#endif

// custom serialization
namespace ARCHIVE {

// Basic serialization types
struct ArchiveSave {
	std::ostream& stream;
	uint32_t version;
	ArchiveSave(std::ostream& _stream, uint32_t _version)
		: stream(_stream), version(_version) {}
	template<typename _Tp>
	ArchiveSave& operator & (const _Tp& obj);
};
struct ArchiveLoad {
	std::istream& stream;
	uint32_t version;
	ArchiveLoad(std::istream& _stream, uint32_t _version)
		: stream(_stream), version(_version) {}
	template<typename _Tp>
	ArchiveLoad& operator & (_Tp& obj);
};

template<typename _Tp>
bool Save(ArchiveSave& a, const _Tp& obj) {
	const_cast<_Tp&>(obj).serialize(a, a.version);
	return true;
}
template<typename _Tp>
bool Load(ArchiveLoad& a, _Tp& obj) {
	obj.serialize(a, a.version);
	return true;
}

template<typename _Tp>
ArchiveSave& ArchiveSave::operator & (const _Tp& obj) {
	Save(*this, obj);
	return *this;
}
template<typename _Tp>
ArchiveLoad& ArchiveLoad::operator & (_Tp& obj) {
	Load(*this, obj);
	return *this;
}

// Main exporter & importer
template<typename _Tp>
bool SerializeSave(const _Tp& obj, const std::string& fileName, uint32_t version=MVSI_PROJECT_VER) {
	// open the output stream
	std::ofstream stream(fileName, std::ofstream::binary);
	if (!stream.is_open())
		return false;
	// write header
	if (version > 0) {
		// save project ID
		stream.write(MVSI_PROJECT_ID, 4);
		// save project version
		stream.write((const char*)&version, sizeof(uint32_t));
		// reserve some bytes
		const uint32_t reserved(0);
		stream.write((const char*)&reserved, sizeof(uint32_t));
	}
	// serialize out the current state
	ARCHIVE::ArchiveSave serializer(stream, version);
	serializer & obj;
	// flush and verify the write actually reached disk: a failure here (e.g. the volume ran
	// out of space) otherwise leaves a silently truncated file that still parses its element
	// counts but is missing data, so report it as an error instead of a successful save
	return stream.flush().good();
}
template<typename _Tp>
bool SerializeLoad(_Tp& obj, const std::string& fileName, uint32_t* pVersion=NULL) {
	// open the input stream
	std::ifstream stream(fileName, std::ifstream::binary);
	if (!stream.is_open())
		return false;
	// read header
	uint32_t version(0);
	// load project header ID
	char szHeader[4];
	stream.read(szHeader, 4);
	if (!stream)
		return false;
	if (strncmp(szHeader, MVSI_PROJECT_ID, 4) != 0) {
		// try to load as the first version that didn't have a header
		const size_t size(fileName.size());
		if (size <= 4)
			return false;
		std::string ext(fileName.substr(size-4));
		std::transform(ext.begin(), ext.end(), ext.begin(), [](char c) { return (char)std::tolower(c); });
		if (ext != ".mvs")
			return false;
		stream.seekg(0, std::ifstream::beg);
	} else {
		// load project version
		stream.read((char*)&version, sizeof(uint32_t));
		if (!stream || version > MVSI_PROJECT_VER)
			return false;
		// skip reserved bytes
		uint32_t reserved;
		stream.read((char*)&reserved, sizeof(uint32_t));
	}
	// serialize in the current state
	ARCHIVE::ArchiveLoad serializer(stream, version);
	serializer & obj;
	if (pVersion)
		*pVersion = version;
	return true;
}


#define ARCHIVE_DEFINE_TYPE(TYPE) \
template<> \
inline bool Save<TYPE>(ArchiveSave& a, const TYPE& v) { \
	a.stream.write((const char*)&v, sizeof(TYPE)); \
	return true; \
} \
template<> \
inline bool Load<TYPE>(ArchiveLoad& a, TYPE& v) { \
	a.stream.read((char*)&v, sizeof(TYPE)); \
	return true; \
}

// Serialization support for basic types
ARCHIVE_DEFINE_TYPE(uint32_t)
ARCHIVE_DEFINE_TYPE(uint64_t)
ARCHIVE_DEFINE_TYPE(float)
ARCHIVE_DEFINE_TYPE(double)

// Serialization support for cv::Matx
template<typename _Tp, int m, int n>
inline bool Save(ArchiveSave& a, const cv::Matx<_Tp,m,n>& _m) {
	a.stream.write((const char*)_m.val, sizeof(_Tp)*m*n);
	return true;
}
template<typename _Tp, int m, int n>
inline bool Load(ArchiveLoad& a, cv::Matx<_Tp,m,n>& _m) {
	a.stream.read((char*)_m.val, sizeof(_Tp)*m*n);
	return true;
}

// Serialization support for cv::Point3_
template<typename _Tp>
inline bool Save(ArchiveSave& a, const cv::Point3_<_Tp>& pt) {
	a.stream.write((const char*)&pt.x, sizeof(_Tp)*3);
	return true;
}
template<typename _Tp>
inline bool Load(ArchiveLoad& a, cv::Point3_<_Tp>& pt) {
	a.stream.read((char*)&pt.x, sizeof(_Tp)*3);
	return true;
}

// Serialization support for std::string
template<>
inline bool Save<std::string>(ArchiveSave& a, const std::string& s) {
	const uint64_t size(s.size());
	Save(a, size);
	if (size > 0)
		a.stream.write(&s[0], sizeof(char)*size);
	return true;
}
template<>
inline bool Load<std::string>(ArchiveLoad& a, std::string& s) {
	uint64_t size;
	Load(a, size);
	if (size > 0) {
		s.resize(size);
		a.stream.read(&s[0], sizeof(char)*size);
	}
	return true;
}

// Serialization support for std::vector
template<typename _Tp>
inline bool Save(ArchiveSave& a, const std::vector<_Tp>& v) {
	const uint64_t size(v.size());
	Save(a, size);
	for (uint64_t i=0; i<size; ++i)
		Save(a, v[i]);
	return true;
}
template<typename _Tp>
inline bool Load(ArchiveLoad& a, std::vector<_Tp>& v) {
	uint64_t size;
	Load(a, size);
	if (size > 0) {
		v.resize(size);
		for (uint64_t i=0; i<size; ++i)
			Load(a, v[i]);
	}
	return true;
}

} // namespace ARCHIVE
/*----------------------------------------------------------------*/


// interface used to export/import MVS input data;
//  - MAX(width,height) is used for normalization
//  - row-major order is used for storing the matrices
struct Interface
{
	typedef cv::Point3_<float> Pos3f;
	typedef cv::Point3_<double> Pos3d;
	typedef cv::Matx<double,3,3> Mat33d;
	typedef cv::Matx<double,4,4> Mat44d;
	typedef cv::Point3_<uint8_t> Col3; // x=B, y=G, z=R
	/*----------------------------------------------------------------*/

	// structure describing a mobile platform with cameras attached to it
	struct Platform {
		// structure describing a camera mounted on a platform
		struct Camera {
			std::string name; // camera's name
			std::string bandName; // camera's band name, ex: RGB, BLUE, GREEN, RED, NIR, THERMAL, etc (optional)
			uint32_t width, height; // image resolution in pixels for all images sharing this camera (optional)
			Mat33d K; // camera's intrinsics matrix (normalized if image resolution not specified), where integer coordinates is by convention the pixel center
			Mat33d R; // camera's rotation matrix relative to the platform
			Pos3d C; // camera's translation vector relative to the platform

			Camera() : width(0), height(0) {}
			bool HasResolution() const { return width > 0 && height > 0; }
			bool IsNormalized() const { return !HasResolution(); }
			static uint32_t GetNormalizationScale(uint32_t width, uint32_t height) { return std::max(width, height); }
			uint32_t GetNormalizationScale() const { return GetNormalizationScale(width, height); }

			// project point: camera to image (homogeneous) coordinates
			inline Pos3d operator * (const Pos3d& X) const {
				return Pos3d(
					K(0,2)+K(0,0)*X.x/X.z,
					K(1,2)+K(1,1)*X.y/X.z,
					1.0);
			}
			// back-project point: image (z is the depth) to camera coordinates
			inline Pos3d operator / (const Pos3d& x) const {
				return Pos3d(
					(x.x-K(0,2))*x.z/K(0,0),
					(x.y-K(1,2))*x.z/K(1,1),
					1.0);
			}

			template <class Archive>
			void serialize(Archive& ar, const unsigned int version) {
				ar & name;
				if (version > 3) {
					ar & bandName;
				}
				if (version > 0) {
					ar & width;
					ar & height;
				}
				ar & K;
				ar & R;
				ar & C;
			}
		};
		typedef std::vector<Camera> CameraArr;

		// structure describing a pose along the trajectory of a platform
		struct Pose {
			Mat33d R; // platform's rotation matrix that rotates a point from world to camera coordinate system
			Pos3d C; // platform's translation vector (position) in world coordinate system

			Pose() {}
			template <typename MAT, typename POS>
			Pose(const MAT& _R, const POS& _C) : R(_R), C(_C) {}

			// translation vector t = -RC
			inline Pos3d GetTranslation() const { return R*(-C); }
			inline void SetTranslation(const Pos3d& T) { C = R.t()*(-T); }

			// combine poses
			inline Pose operator * (const Pose& P) const {
				return Pose(R*P.R, P.R.t()*C+P.C);
			}
			inline Pose& operator *= (const Pose& P) {
				R = R*P.R; C = P.R.t()*C+P.C; return *this;
			}

			// project point: world to local coordinates
			inline Pos3d operator * (const Pos3d& X) const {
				return R * (X - C);
			}
			// back-project point: local to world coordinates
			inline Pos3d operator / (const Pos3d& X) const {
				return R.t() * X + C;
			}

			template <class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & R;
				ar & C;
			}
		};
		typedef std::vector<Pose> PoseArr;

		std::string name; // platform's name
		CameraArr cameras; // cameras mounted on the platform
		PoseArr poses; // trajectory of the platform

		const Mat33d& GetK(uint32_t cameraID) const {
			return cameras[cameraID].K;
		}
		static Mat33d ScaleK(const Mat33d& _K, double scale) {
			Mat33d K(_K);
			const bool bNormalized(K(0,2) < 3 && K(1,2) < 3);
			K(0,0) *= scale;
			K(1,1) *= scale;
			K(0,2) = bNormalized ? K(0,2)*scale : (K(0,2)+0.5)*scale-0.5;
			K(1,2) = bNormalized ? K(1,2)*scale : (K(1,2)+0.5)*scale-0.5;
			K(0,1) *= scale;
			return K;
		}
		const Mat33d& SetFullK(uint32_t cameraID, const Mat33d& K, uint32_t width, uint32_t height, bool normalize=false) {
			Camera& camera = cameras[cameraID];
			if (normalize) {
				camera.width = camera.height = 0;
				camera.K = ScaleK(K, 1.0/(double)Camera::GetNormalizationScale(width, height));
			} else {
				camera.width = width; camera.height = height;
				camera.K = K;
			}
			return camera.K;
		}
		Mat33d GetFullK(uint32_t cameraID, uint32_t width, uint32_t height) const {
			const Camera& camera = cameras[cameraID];
			if (!camera.IsNormalized() && camera.width == width && camera.height == height)
				return camera.K;
			return ScaleK(camera.K, (double)Camera::GetNormalizationScale(width, height)/
				(camera.IsNormalized()?1.0:(double)camera.GetNormalizationScale()));
		}

		Pose GetPose(uint32_t cameraID, uint32_t poseID) const {
			const Camera& camera = cameras[cameraID];
			const Pose& pose = poses[poseID];
			// add the relative camera pose to the platform
			return Pose{
				camera.R*pose.R,
				pose.R.t()*camera.C+pose.C
			};
		}

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & name;
			ar & cameras;
			ar & poses;
		}
	};
	typedef std::vector<Platform> PlatformArr;
	/*----------------------------------------------------------------*/

	// structure describing an image
	struct Image {
		// structure describing how an other image relates to this image in terms of overlap,
		// i.e. how many 3D points are shared between the two images, base-line and common area,
		// useful for ex. when selecting the best images to densly match with
		struct ViewScore {
			uint32_t ID; // image local-ID, the index in this scene images list
			uint32_t points; // number of 3D points shared with the reference image
			float scale; // image scale relative to the reference image
			float angle; // image angle relative to the reference image (radians)
			float area; // common image area relative to the reference image (ratio)
			float score; // aggregated image score relative to the reference image (larger is better)

			template<class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & ID;
				ar & points;
				ar & scale;
				ar & angle;
				ar & area;
				ar & score;
			}
		};

		std::string name; // image file name
		std::string maskName; // segmentation file name (optional)
		uint32_t platformID; // ID of the associated platform
		uint32_t cameraID; // ID of the associated camera on the associated platform
		uint32_t poseID; // ID of the pose of the associated platform
		uint32_t ID; // image global-ID, ex. the ID given outside the current scene, like the index in the full list of image files (optional)
		float minDepth; // minimum depth of the points seen by this image (optional)
		float avgDepth; // average depth of the points seen by this image (optional)
		float maxDepth; // maximum depth of the points seen by this image (optional)
		std::vector<ViewScore> viewScores; // list of view scores for this image (optional)

		Image() : platformID(NO_ID), cameraID(NO_ID), poseID(NO_ID), ID(NO_ID), minDepth(0), avgDepth(0), maxDepth(0) {}

		bool IsValid() const { return poseID != NO_ID; }

		template <class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar & name;
			if (version > 4) {
				ar & maskName;
			}
			ar & platformID;
			ar & cameraID;
			ar & poseID;
			if (version > 2) {
				ar & ID;
			}
			if (version > 6) {
				ar & minDepth;
				ar & avgDepth;
				ar & maxDepth;
				ar & viewScores;
			}
		}
	};
	typedef std::vector<Image> ImageArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point
	struct Vertex {
		// structure describing one view for a given 3D feature
		struct View {
			uint32_t imageID; // image ID corresponding to this view
			float confidence; // view's confidence (0 - not available)

			template<class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & imageID;
				ar & confidence;
			}
		};
		typedef std::vector<View> ViewArr;

		Pos3f X; // 3D point position
		ViewArr views; // list of all available views for this 3D feature

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & X;
			ar & views;
		}
	};
	typedef std::vector<Vertex> VertexArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D line
	struct Line {
		// structure describing one view for a given 3D feature
		struct View {
			uint32_t imageID; // image ID corresponding to this view
			float confidence; // view's confidence (0 - not available)

			template<class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & imageID;
				ar & confidence;
			}
		};
		typedef std::vector<View> ViewArr;

		Pos3f pt1; // 3D line segment end-point
		Pos3f pt2; // 3D line segment end-point
		ViewArr views; // list of all available views for this 3D feature

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & pt1;
			ar & pt2;
			ar & views;
		}
	};
	typedef std::vector<Line> LineArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point's normal (optional)
	struct Normal {
		Pos3f n; // 3D feature normal

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & n;
		}
	};
	typedef std::vector<Normal> NormalArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point's color (optional)
	struct Color {
		Col3 c; // 3D feature color

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & c;
		}
	};
	typedef std::vector<Color> ColorArr;
	/*----------------------------------------------------------------*/

	// structure describing a Oriented Bounding-Box (optional)
	struct OBB {
		Mat33d rot; // rotation from scene to OBB coordinate system
		Pos3d ptMin; // minimal point represented in OBB coordinate system
		Pos3d ptMax; // maximal point represented in OBB coordinate system

		OBB() : rot(Mat33d::eye()), ptMin(0, 0, 0), ptMax(0, 0, 0) {}

		bool IsValid() const { return ptMin.x < ptMax.x && ptMin.y < ptMax.y && ptMin.z < ptMax.z; }

		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & rot;
			ar & ptMin;
			ar & ptMax;
		}
	};
	/*----------------------------------------------------------------*/

	PlatformArr platforms; // array of platforms
	ImageArr images; // array of images
	VertexArr vertices; // array of reconstructed 3D points
	NormalArr verticesNormal; // array of reconstructed 3D points' normal (optional)
	ColorArr verticesColor; // array of reconstructed 3D points' color (optional)
	LineArr lines; // array of reconstructed 3D lines (optional)
	NormalArr linesNormal; // array of reconstructed 3D lines' normal (optional)
	ColorArr linesColor; // array of reconstructed 3D lines' color (optional)
	Mat44d transform; // transformation used to convert from absolute to relative coordinate system (optional)
	OBB obb; // minimum oriented bounding box containing the scene (optional)

	Interface() : transform(Mat44d::eye()) {}

	const Mat33d& GetK(uint32_t imageID) const {
		const Image& image = images[imageID];
		return platforms[image.platformID].GetK(image.cameraID);
	}
	Mat33d GetFullK(uint32_t imageID, uint32_t width, uint32_t height) const {
		const Image& image = images[imageID];
		return platforms[image.platformID].GetFullK(image.cameraID, width, height);
	}

	const Platform::Camera& GetCamera(uint32_t imageID) const {
		const Image& image = images[imageID];
		return platforms[image.platformID].cameras[image.cameraID];
	}

	Platform::Pose GetPose(uint32_t imageID) const {
		const Image& image = images[imageID];
		return platforms[image.platformID].GetPose(image.cameraID, image.poseID);
	}

	// apply similarity transform
	void Transform(const Mat33d& rotation, const Pos3d& translation, const double scale) {
		for (Platform& platform : platforms) {
			for (Platform::Pose& pose : platform.poses) {
				pose.R = pose.R * rotation.t();
				pose.C = rotation * pose.C * scale + translation;
			}
		}
		for (Vertex& vertex : vertices) {
			vertex.X = rotation * Pos3d(vertex.X) * scale + translation;
		}
		for (Normal& normal : verticesNormal) {
			normal.n = rotation * Pos3d(normal.n);
		}
		for (Line& line : lines) {
			line.pt1 = rotation * Pos3d(line.pt1) * scale + translation;
			line.pt2 = rotation * Pos3d(line.pt2) * scale + translation;
		}
		for (Normal& normal : linesNormal) {
			normal.n = rotation * Pos3d(normal.n);
		}
	}

	template <class Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & platforms;
		ar & images;
		ar & vertices;
		ar & verticesNormal;
		ar & verticesColor;
		if (version > 0) {
			ar & lines;
			ar & linesNormal;
			ar & linesColor;
			if (version > 1) {
				ar & transform;
				if (version > 5) {
					ar & obb;
				}
			}
		}
	}
};
/*----------------------------------------------------------------*/


// interface used to export/import MVS depth-map data;
// see MVS::ExportDepthDataRaw() and MVS::ImportDepthDataRaw() for usage example:
//  - image-resolution at which the depth-map was estimated
//  - depth-map-resolution, for now only the same resolution as the image is supported
//  - min/max-depth of the values in the depth-map
//  - image-file-name is the path to the reference color image
//  - image-IDs are the reference view ID and neighbor view IDs used to estimate the depth-map (global ID)
//  - camera/rotation/position matrices (row-major) is the absolute pose corresponding to the reference view
//  - depth-map: the pixels' depth
//  - normal-map (optional): the 3D point normal in camera space; same resolution as the depth-map
//  - confidence-map (optional): the 3D point confidence (usually a value in [0,1]); same resolution as the depth-map
//  - views-map (optional): the pixels' views, indexing image-IDs starting after first view (up to 4); same resolution as the depth-map
// The maps are stored quantized, 11 bytes per pixel in total; every value is still
// float in memory, the packing happens only in ExportDepthDataRaw/ImportDepthDataRaw:
//  - depth: half, after scaling by 2^-depthExp so the values land in the well-conditioned
//    part of the half range whatever the scene scale; the scaling is exact (it only
//    shifts the exponent), so the sole error is the half rounding, 4.9e-4 relative
//  - normal: two int16 holding the octahedral projection of the unit vector; the
//    all-zero normal of an invalid pixel is kept exactly via a reserved sentinel
//  - confidence: uint8 over [0,confScale]; the scale is per depth-map because the
//    values are in [0,1] for the patch-match estimators but are raw matching costs
//    on the semi-global-matching fusion path
struct HeaderDepthDataRaw {
	enum {
		HAS_DEPTH = (1<<0),
		HAS_NORMAL = (1<<1),
		HAS_CONF = (1<<2),
		HAS_VIEWS = (1<<3),
	};
	uint16_t name; // file type
	uint8_t type; // content type
	int8_t depthExp; // power-of-two exponent the stored depths were scaled down by
	uint32_t imageWidth, imageHeight; // image resolution
	uint32_t depthWidth, depthHeight; // depth-map resolution
	float dMin, dMax; // depth range for this view
	float confScale; // confidence value the stored uint8 range maps onto
	// image file name length followed by the characters: uint16_t nFileNameSize; char* FileName
	// number of view IDs followed by view ID and neighbor view IDs: uint32_t nIDs; uint32_t* IDs
	// camera, rotation and position matrices (row-major) at image resolution: double K[3][3], R[3][3], C[3]
	// depth, normal, confidence maps: half depthMap[height][width], int16_t normalMap[height][width][2], uint8_t confMap[height][width]
	inline HeaderDepthDataRaw() : name(0), type(0), depthExp(0), confScale(1.f) {}
	static uint16_t HeaderDepthDataRawName() { return *reinterpret_cast<const uint16_t*>("D2"); }
};
/*----------------------------------------------------------------*/


// the meta-data of one depth-map, everything a DMAP file stores besides the maps
// themselves, which are passed separately as the caller's own matrices
struct DepthDataRaw {
	HeaderDepthDataRaw header; // resolutions, depth range and content type
	std::string imageFileName; // path to the reference color image, by convention relative to the DMAP file
	std::vector<uint32_t> IDs; // reference view ID followed by the neighbor view IDs
	cv::Matx<double,3,3> K; // reference view intrinsics, at image resolution
	cv::Matx<double,3,3> R; // reference view rotation, world to camera
	cv::Point3_<double> C; // reference view position, in world coordinates
};

// The quantization the DMAP format stores its maps with. Where OpenCV has the
// conversion it does it -- convertTo() reaches the F16C instructions for the halves and
// the SIMD saturating cast for the confidence -- and the plain C fallbacks below only
// stand in for a project that does not have OpenCV, or whose OpenCV predates CV_16F
// (before 3.4.4/4.0): they implement the very same IEEE-754 round-to-nearest-even
// rules, so both write the same bytes.
namespace DEPTHDATA {

#if defined(_USE_CUSTOM_CV) || !defined(CV_16F)

// IEEE-754 binary32 to binary16, round-to-nearest-even
inline uint16_t Float2Half(float value) {
	uint32_t f;
	memcpy(&f, &value, sizeof(uint32_t));
	const uint32_t sign(f & 0x80000000u);
	f ^= sign;
	uint16_t h;
	if (f >= 0x47800000u) {
		// too large for half: infinity, or NaN made quiet
		h = (uint16_t)(f > 0x7f800000u ? 0x7e00u : 0x7c00u);
	} else if (f < 0x38800000u) {
		// subnormal or zero: adding the magic value lines the mantissa up at the bottom
		// of the float, and the round-to-nearest-even addition does the rounding
		const uint32_t magicBits(((127-15)+(23-10)+1) << 23);
		float magic, m;
		memcpy(&magic, &magicBits, sizeof(uint32_t));
		memcpy(&m, &f, sizeof(uint32_t));
		m += magic;
		memcpy(&f, &m, sizeof(uint32_t));
		h = (uint16_t)(f - magicBits);
	} else {
		// normal: rebias the exponent and round the mantissa to nearest-even
		const uint32_t mantOdd((f >> 13) & 1);
		f += ((uint32_t)(15-127) << 23) + 0xfff + mantOdd;
		h = (uint16_t)(f >> 13);
	}
	return (uint16_t)(h | (uint16_t)(sign >> 16));
}
// IEEE-754 binary16 to binary32, exact
inline float Half2Float(uint16_t value) {
	const uint32_t shiftedExp(0x7c00 << 13); // exponent mask, after the shift below
	uint32_t o((uint32_t)(value & 0x7fff) << 13);
	const uint32_t exp(shiftedExp & o);
	o += (uint32_t)(127-15) << 23; // rebias the exponent
	if (exp == shiftedExp) {
		// infinity or NaN: rebias again, the half exponent being all ones
		o += (uint32_t)(128-16) << 23;
	} else if (exp == 0) {
		// subnormal or zero: renormalize by subtracting the implicit leading one back out
		const uint32_t magicBits(113 << 23);
		float magic, m;
		o += 1 << 23;
		memcpy(&m, &o, sizeof(uint32_t));
		memcpy(&magic, &magicBits, sizeof(uint32_t));
		m -= magic;
		memcpy(&o, &m, sizeof(uint32_t));
	}
	o |= (uint32_t)(value & 0x8000) << 16;
	float f;
	memcpy(&f, &o, sizeof(uint32_t));
	return f;
}

// float already scaled to [0,255] to uint8, round-to-nearest-even and clamped
inline uint8_t Float2Unorm8(float value) {
	if (!(value > 0.f)) // also catches NaN
		return 0;
	if (value >= 255.f)
		return 255;
	// adding 1.5*2^23 pushes the fraction out of the mantissa, so the addition itself
	// rounds to the nearest integer, ties to even, which then sits in the low bits
	const float t(value + 12582912.f);
	uint32_t bits;
	memcpy(&bits, &t, sizeof(uint32_t));
	return (uint8_t)(bits & 0xff);
}
#endif // _USE_CUSTOM_CV || !CV_16F

// run the given per-row work over [0,rows), on OpenCV's thread pool where there is one
// -- it is the one the rest of the process already uses, and it runs the body inline
// when called from inside another parallel region -- and on OpenMP otherwise
template <typename TROWS>
inline void ParallelForRows(int rows, const TROWS& ForRows) {
#ifdef _USE_CUSTOM_CV
	#ifdef _OPENMP
	#pragma omp parallel for
	#endif
	for (int r=0; r<rows; ++r)
		ForRows(r, r+1);
#else
	cv::parallel_for_(cv::Range(0, rows), [&ForRows](const cv::Range& range) {
		ForRows(range.start, range.end);
	});
#endif
}

// Octahedral direction quantized to two int16. Both directions are ~10 ALU operations
// and the error stays near-uniform over the sphere, which is what makes the pair worth
// storing at 16 bits. A pixel without an estimate carries an all-zero normal, which is
// not a unit vector and so cannot go through the octahedral projection at all; it gets
// the one code an encoded unit vector can never produce -- both components below the
// -32767 the encoder clamps to -- so that invalid pixels round-trip exactly instead of
// decoding to a NaN that would then propagate into fusion.
enum : int16_t { NORMAL_SENTINEL = -32768 };
inline void EncodeNormal(float x, float y, float z, int16_t& ox, int16_t& oy) {
	if (x == 0 && y == 0 && z == 0) {
		ox = oy = NORMAL_SENTINEL;
		return;
	}
	const float invL1(1.f/(std::fabs(x)+std::fabs(y)+std::fabs(z)));
	float px(x*invL1), py(y*invL1);
	if (z < 0) {
		const float nx((1.f-std::fabs(py)) * (px < 0 ? -1.f : 1.f));
		const float ny((1.f-std::fabs(px)) * (py < 0 ? -1.f : 1.f));
		px = nx; py = ny;
	}
	ox = (int16_t)(int)std::floor((px < -1.f ? -1.f : (px > 1.f ? 1.f : px))*32767.f + 0.5f);
	oy = (int16_t)(int)std::floor((py < -1.f ? -1.f : (py > 1.f ? 1.f : py))*32767.f + 0.5f);
}
inline void DecodeNormal(int16_t ox, int16_t oy, float& x, float& y, float& z) {
	if (ox == NORMAL_SENTINEL && oy == NORMAL_SENTINEL) {
		x = y = z = 0.f;
		return;
	}
	const float px(ox*(1.f/32767.f)), py(oy*(1.f/32767.f));
	x = px;
	y = py;
	z = 1.f-std::fabs(px)-std::fabs(py);
	if (z < 0) {
		x = (1.f-std::fabs(py)) * (px < 0 ? -1.f : 1.f);
		y = (1.f-std::fabs(px)) * (py < 0 ? -1.f : 1.f);
	}
	const float invNorm(1.f/std::sqrt(x*x + y*y + z*z));
	x *= invNorm; y *= invNorm; z *= invNorm;
}

// largest value of a single channel float map, NaNs skipped
inline float MaxValue(const cv::Mat& map) {
#ifdef _USE_CUSTOM_CV
	float maxValue(0);
	for (int r=0; r<map.rows; ++r) {
		const float* p = map.ptr<float>(r);
		for (int c=0; c<map.cols; ++c)
			if (maxValue < p[c])
				maxValue = p[c];
	}
	return maxValue;
#else
	double maxValue(0);
	cv::minMaxIdx(map, NULL, &maxValue);
	return (float)maxValue;
#endif
}

// depth-map to and from halves scaled by the given power of two
inline void PackDepth(const cv::Mat& depthMap, double scale, uint16_t* packed) {
#if defined(_USE_CUSTOM_CV) || !defined(CV_16F)
	const int cols(depthMap.cols);
	const float scaleF((float)scale);
	ParallelForRows(depthMap.rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const float* pD = depthMap.ptr<float>(r);
			uint16_t* pH = packed + (size_t)r*cols;
			for (int c=0; c<cols; ++c)
				pH[c] = Float2Half(pD[c]*scaleF);
		}
	});
#else
	cv::Mat depthMapH(depthMap.rows, depthMap.cols, CV_16F, packed);
	depthMap.convertTo(depthMapH, CV_16F, scale);
#endif
}
inline void UnpackDepth(const uint16_t* packed, int rows, int cols, double scale, cv::Mat& depthMap) {
#if defined(_USE_CUSTOM_CV) || !defined(CV_16F)
	depthMap.create(rows, cols, CV_32FC1);
	const float scaleF((float)scale);
	ParallelForRows(rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const uint16_t* pH = packed + (size_t)r*cols;
			float* pD = depthMap.ptr<float>(r);
			for (int c=0; c<cols; ++c)
				pD[c] = Half2Float(pH[c])*scaleF;
		}
	});
#else
	const cv::Mat depthMapH(rows, cols, CV_16F, const_cast<uint16_t*>(packed));
	depthMapH.convertTo(depthMap, CV_32F, scale);
#endif
}

// confidence-map to and from unorm8 over the given scale
inline void PackConf(const cv::Mat& confMap, double scale, uint8_t* packed) {
#ifdef _USE_CUSTOM_CV
	const int cols(confMap.cols);
	const float scaleF((float)scale);
	ParallelForRows(confMap.rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const float* pC = confMap.ptr<float>(r);
			uint8_t* pU = packed + (size_t)r*cols;
			for (int c=0; c<cols; ++c)
				pU[c] = Float2Unorm8(pC[c]*scaleF);
		}
	});
#else
	cv::Mat confMapU(confMap.rows, confMap.cols, CV_8U, packed);
	confMap.convertTo(confMapU, CV_8U, scale);
#endif
}
inline void UnpackConf(const uint8_t* packed, int rows, int cols, double scale, cv::Mat& confMap) {
#ifdef _USE_CUSTOM_CV
	confMap.create(rows, cols, CV_32FC1);
	const float scaleF((float)scale);
	ParallelForRows(rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const uint8_t* pU = packed + (size_t)r*cols;
			float* pC = confMap.ptr<float>(r);
			for (int c=0; c<cols; ++c)
				pC[c] = pU[c]*scaleF;
		}
	});
#else
	const cv::Mat confMapU(rows, cols, CV_8U, const_cast<uint8_t*>(packed));
	confMapU.convertTo(confMap, CV_32F, scale);
#endif
}

// normal-map to and from the octahedral pairs; OpenCV has no such projection, so this
// one is ours either way
inline void PackNormals(const cv::Mat& normalMap, int16_t* packed) {
	const int cols(normalMap.cols);
	ParallelForRows(normalMap.rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const float* pN = normalMap.ptr<float>(r);
			int16_t* pO = packed + (size_t)r*cols*2;
			for (int c=0; c<cols; ++c)
				EncodeNormal(pN[c*3], pN[c*3+1], pN[c*3+2], pO[c*2], pO[c*2+1]);
		}
	});
}
inline void UnpackNormals(const int16_t* packed, int rows, int cols, cv::Mat& normalMap) {
	normalMap.create(rows, cols, CV_32FC3);
	ParallelForRows(rows, [&](int rBegin, int rEnd) {
		for (int r=rBegin; r<rEnd; ++r) {
			const int16_t* pO = packed + (size_t)r*cols*2;
			float* pN = normalMap.ptr<float>(r);
			for (int c=0; c<cols; ++c)
				DecodeNormal(pO[c*2], pO[c*2+1], pN[c*3], pN[c*3+1], pN[c*3+2]);
		}
	});
}

} // namespace DEPTHDATA
/*----------------------------------------------------------------*/


// Export the depth-data of one view to the given stream, opened in binary mode.
// The caller fills in data the image resolution, the depth range, the image file name,
// the view IDs and the pose, while the map resolution, the content type and the
// quantization scales are derived from the maps themselves. Only the depth-map is
// mandatory; an empty map is simply not written. The maps are read where they are, no
// copy of them is made: depth and confidence are CV_32FC1, the normals CV_32FC3 in
// camera space, and the views CV_8UC4.
inline bool ExportDepthDataRaw(std::ostream& stream, const DepthDataRaw& data,
	const cv::Mat& depthMap, const cv::Mat& normalMap, const cv::Mat& confMap, const cv::Mat& viewsMap)
{
	if (depthMap.empty() || depthMap.type() != CV_32FC1 || data.IDs.empty() || data.IDs.size() >= 256)
		return false;
	if ((!normalMap.empty() && (normalMap.type() != CV_32FC3 || normalMap.rows != depthMap.rows || normalMap.cols != depthMap.cols)) ||
		(!confMap.empty() && (confMap.type() != CV_32FC1 || confMap.rows != depthMap.rows || confMap.cols != depthMap.cols)) ||
		(!viewsMap.empty() && (viewsMap.type() != CV_8UC4 || viewsMap.rows != depthMap.rows || viewsMap.cols != depthMap.cols)))
		return false;
	const int height(depthMap.rows), width(depthMap.cols);
	const size_t area((size_t)height*width);

	// write header
	HeaderDepthDataRaw header(data.header);
	header.name = HeaderDepthDataRaw::HeaderDepthDataRawName();
	header.type = HeaderDepthDataRaw::HAS_DEPTH;
	header.depthWidth = (uint32_t)width;
	header.depthHeight = (uint32_t)height;
	if (header.imageWidth < header.depthWidth || header.imageHeight < header.depthHeight)
		return false;
	// bring the depths into the well-conditioned part of the half range whatever the
	// scene scale: a power-of-two factor only shifts the exponent, so this is exact and
	// the sole error left is the half rounding. Take the exponent from the data and not
	// from dMax, which some callers set to FLT_MAX as an "unbounded" sentinel.
	const float maxDepth(DEPTHDATA::MaxValue(depthMap));
	if (maxDepth > 0 && std::isfinite(maxDepth)) {
		const int e(std::ilogb(maxDepth));
		header.depthExp = (int8_t)(e < -100 ? -100 : (e > 100 ? 100 : e));
	} else {
		header.depthExp = 0;
	}
	// a depth sitting on either end of the range can be rounded just past it by the
	// half quantization, so widen the recorded range by that bound (half carries an
	// 11-bit significand) and keep "every stored depth is inside [dMin,dMax]" true
	const float depthQuantRelErr(1.f/1024.f);
	if (std::isfinite(header.dMin))
		header.dMin *= 1.f-depthQuantRelErr;
	if (std::isfinite(header.dMax))
		header.dMax *= 1.f+depthQuantRelErr;
	if (!normalMap.empty())
		header.type |= HeaderDepthDataRaw::HAS_NORMAL;
	if (!confMap.empty()) {
		header.type |= HeaderDepthDataRaw::HAS_CONF;
		// the patch-match estimators normalize confidence to [0,1], but the
		// semi-global-matching fusion stores raw matching costs, so take the range
		// from the data rather than assuming it
		const float maxConf(DEPTHDATA::MaxValue(confMap));
		header.confScale = (maxConf > 0 && std::isfinite(maxConf)) ? maxConf : 1.f;
	}
	if (!viewsMap.empty())
		header.type |= HeaderDepthDataRaw::HAS_VIEWS;
	stream.write((const char*)&header, sizeof(HeaderDepthDataRaw));

	// write image file name
	const uint16_t nFileNameSize((uint16_t)data.imageFileName.length());
	stream.write((const char*)&nFileNameSize, sizeof(uint16_t));
	stream.write(data.imageFileName.c_str(), nFileNameSize);

	// write neighbor IDs
	const uint32_t nIDs((uint32_t)data.IDs.size());
	stream.write((const char*)&nIDs, sizeof(uint32_t));
	stream.write((const char*)data.IDs.data(), sizeof(uint32_t)*nIDs);

	// write pose
	stream.write((const char*)data.K.val, sizeof(double)*9);
	stream.write((const char*)data.R.val, sizeof(double)*9);
	stream.write((const char*)&data.C.x, sizeof(double)*3);

	// write depth-map, as half scaled by 2^-depthExp (zero stays exactly zero)
	{
		std::vector<uint16_t> depthMapH(area);
		DEPTHDATA::PackDepth(depthMap, std::ldexp(1.0, -header.depthExp), depthMapH.data());
		stream.write((const char*)depthMapH.data(), sizeof(uint16_t)*area);
	}

	// write normal-map, as the octahedral direction quantized to two int16
	if ((header.type & HeaderDepthDataRaw::HAS_NORMAL) != 0) {
		std::vector<int16_t> normalMapOct(area*2);
		DEPTHDATA::PackNormals(normalMap, normalMapOct.data());
		stream.write((const char*)normalMapOct.data(), sizeof(int16_t)*area*2);
	}

	// write confidence-map, as unorm8 over [0,confScale]
	if ((header.type & HeaderDepthDataRaw::HAS_CONF) != 0) {
		std::vector<uint8_t> confMapU(area);
		DEPTHDATA::PackConf(confMap, 255.0/header.confScale, confMapU.data());
		stream.write((const char*)confMapU.data(), sizeof(uint8_t)*area);
	}

	// write views-map, stored as it is
	if ((header.type & HeaderDepthDataRaw::HAS_VIEWS) != 0)
		for (int r=0; r<height; ++r)
			stream.write((const char*)viewsMap.ptr(r), (size_t)width*4);

	return (bool)stream;
}
// same, writing to the given file
inline bool ExportDepthDataRaw(const std::string& fileName, const DepthDataRaw& data,
	const cv::Mat& depthMap, const cv::Mat& normalMap, const cv::Mat& confMap, const cv::Mat& viewsMap)
{
	std::ofstream stream(fileName, std::ofstream::binary);
	if (!stream.is_open())
		return false;
	if (!ExportDepthDataRaw(stream, data, depthMap, normalMap, confMap, viewsMap))
		return false;
	// verify the write actually reached disk: a failure here (e.g. the volume ran out of
	// space) otherwise leaves a silently truncated file that still parses its header
	return stream.flush().good();
}

// Import the depth-data of one view from the given stream, opened in binary mode.
// Every map requested through flags and present in the file is created at the stored
// resolution and read into directly; the others are stepped over and left untouched.
// Passing no flag at all reads the meta-data alone, the maps being the bulk of the file.
inline bool ImportDepthDataRaw(std::istream& stream, DepthDataRaw& data,
	cv::Mat& depthMap, cv::Mat& normalMap, cv::Mat& confMap, cv::Mat& viewsMap,
	unsigned flags=HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL|HeaderDepthDataRaw::HAS_CONF|HeaderDepthDataRaw::HAS_VIEWS)
{
	// read header
	HeaderDepthDataRaw& header = data.header;
	stream.read((char*)&header, sizeof(HeaderDepthDataRaw));
	if (!stream ||
		header.name != HeaderDepthDataRaw::HeaderDepthDataRawName() ||
		(header.type & HeaderDepthDataRaw::HAS_DEPTH) == 0 ||
		header.depthWidth == 0 || header.depthHeight == 0 ||
		header.imageWidth < header.depthWidth || header.imageHeight < header.depthHeight)
		return false;

	// read image file name
	uint16_t nFileNameSize;
	stream.read((char*)&nFileNameSize, sizeof(uint16_t));
	data.imageFileName.resize(nFileNameSize);
	if (nFileNameSize > 0)
		stream.read(&data.imageFileName[0], nFileNameSize);

	// read neighbor IDs
	uint32_t nIDs;
	stream.read((char*)&nIDs, sizeof(uint32_t));
	if (!stream || nIDs == 0 || nIDs >= 256)
		return false;
	data.IDs.resize(nIDs);
	stream.read((char*)data.IDs.data(), sizeof(uint32_t)*nIDs);

	// read pose
	stream.read((char*)data.K.val, sizeof(double)*9);
	stream.read((char*)data.R.val, sizeof(double)*9);
	stream.read((char*)&data.C.x, sizeof(double)*3);
	if (!stream || flags == 0)
		return (bool)stream; // only the meta-data was requested
	// a map handed over already allocated has to be of the type it is stored as
	if ((!depthMap.empty() && depthMap.type() != CV_32FC1) ||
		(!normalMap.empty() && normalMap.type() != CV_32FC3) ||
		(!confMap.empty() && confMap.type() != CV_32FC1) ||
		(!viewsMap.empty() && viewsMap.type() != CV_8UC4))
		return false;

	const int height((int)header.depthHeight), width((int)header.depthWidth);
	const size_t area((size_t)height*width);

	// read depth-map, stored as half scaled by 2^-depthExp
	if ((flags & HeaderDepthDataRaw::HAS_DEPTH) != 0) {
		std::vector<uint16_t> depthMapH(area);
		stream.read((char*)depthMapH.data(), sizeof(uint16_t)*area);
		if (!stream)
			return false;
		DEPTHDATA::UnpackDepth(depthMapH.data(), height, width, std::ldexp(1.0, header.depthExp), depthMap);
	} else {
		stream.seekg(sizeof(uint16_t)*area, std::ios::cur);
	}

	// read normal-map, stored as the octahedral direction quantized to two int16
	if ((header.type & HeaderDepthDataRaw::HAS_NORMAL) != 0) {
		if ((flags & HeaderDepthDataRaw::HAS_NORMAL) != 0) {
			std::vector<int16_t> normalMapOct(area*2);
			stream.read((char*)normalMapOct.data(), sizeof(int16_t)*area*2);
			if (!stream)
				return false;
			DEPTHDATA::UnpackNormals(normalMapOct.data(), height, width, normalMap);
		} else {
			stream.seekg(sizeof(int16_t)*area*2, std::ios::cur);
		}
	}

	// read confidence-map, stored as unorm8 over [0,confScale]
	if ((header.type & HeaderDepthDataRaw::HAS_CONF) != 0) {
		if ((flags & HeaderDepthDataRaw::HAS_CONF) != 0) {
			std::vector<uint8_t> confMapU(area);
			stream.read((char*)confMapU.data(), sizeof(uint8_t)*area);
			if (!stream)
				return false;
			DEPTHDATA::UnpackConf(confMapU.data(), height, width, header.confScale/255.0, confMap);
		} else {
			stream.seekg(sizeof(uint8_t)*area, std::ios::cur);
		}
	}

	// read views-map, stored as it is
	if ((header.type & flags & HeaderDepthDataRaw::HAS_VIEWS) != 0) {
		viewsMap.create(height, width, CV_8UC4);
		for (int r=0; r<height; ++r)
			stream.read((char*)viewsMap.ptr(r), (size_t)width*4);
	}

	return (bool)stream;
}
// same, reading from the given file
inline bool ImportDepthDataRaw(const std::string& fileName, DepthDataRaw& data,
	cv::Mat& depthMap, cv::Mat& normalMap, cv::Mat& confMap, cv::Mat& viewsMap,
	unsigned flags=HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL|HeaderDepthDataRaw::HAS_CONF|HeaderDepthDataRaw::HAS_VIEWS)
{
	std::ifstream stream(fileName, std::ifstream::binary);
	if (!stream.is_open())
		return false;
	return ImportDepthDataRaw(stream, data, depthMap, normalMap, confMap, viewsMap, flags);
}
/*----------------------------------------------------------------*/

} // namespace _INTERFACE_NAMESPACE

#endif // _INTERFACE_MVS_H_
