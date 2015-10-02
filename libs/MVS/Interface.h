#ifndef _INTERFACE_H_
#define _INTERFACE_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable custom OpenCV data types
// (should be uncommented if OpenCV is not available)
#ifndef _USE_CUSTOM_CV
//#define _USE_CUSTOM_CV
#endif

// uncomment to enable BOOST serialization support
// (should be uncommented)
#ifndef _USE_BOOST
//#define _USE_BOOST
#endif

// uncomment to enable BOOST serialization type specializations
// (should be uncommented if serialization specialization not previously implemented)
#ifndef _USE_BOOST_SERIALIZATION
//#defined _USE_BOOST_SERIALIZATION
#endif


// S T R U C T S ///////////////////////////////////////////////////

#ifdef _USE_CUSTOM_CV

namespace cv {

// simple cv::Matx
template<typename _Tp, int m, int n>
class Matx
{
public:
	_Tp val[m*n];
};

// simple cv::Matx
template<typename _Tp>
class Point3_
{
public:
	_Tp x, y, z;
};

} // namespace cv
/*----------------------------------------------------------------*/
#endif


#if defined(_USE_BOOST) && defined(_USE_BOOST_SERIALIZATION)

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>

namespace boost {
namespace serialization {

// Serialization support for cv::Matx
template<class Archive, typename _Tp, int m, int n>
void serialize(Archive& ar, cv::Matx<_Tp, m, n>& _m, const unsigned int /*version*/) {
	ar & _m.val;
}

// Serialization support for cv::Point3_
template<class Archive, typename _Tp>
void serialize(Archive& ar, cv::Point3_<_Tp>& pt, const unsigned int /*version*/) {
	ar & pt.x & pt.y & pt.z;
}

} // namespace serialization
} // namespace boost
/*----------------------------------------------------------------*/
#endif


namespace MVS {

// interface used to export/import MVS input data;
// MAX(width,height) is used for normalization
struct Interface
{
	typedef float Real;
	typedef cv::Point3_<Real> Pos3;
	typedef cv::Matx<Real,3,3> Mat33;
	typedef uint8_t Color;
	typedef cv::Point3_<Color> Col3; // x=B, y=G, z=R
	/*----------------------------------------------------------------*/

	// structure describing a mobile platform with cameras attached to it
	struct Platform {
		// structure describing a camera mounted on a platform
		struct Camera {
			std::string name; // camera's name
			Mat33 K; // camera's normalized intrinsics matrix
			Mat33 R; // camera's rotation matrix relative to the platform
			Pos3 C; // camera's translation vector relative to the platform

			#ifdef _USE_BOOST
			template <class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & name;
				ar & K;
				ar & R;
				ar & C;
			}
			#endif
		};
		typedef std::vector<Camera> CameraArr;

		// structure describing a pose along the trajectory of a platform
		struct Pose {
			Mat33 R; // platform's rotation matrix
			Pos3 C; // platform's translation vector in the global coordinate system

			#ifdef _USE_BOOST
			template <class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & R;
				ar & C;
			}
			#endif
		};
		typedef std::vector<Pose> PoseArr;

		std::string name; // platform's name
		CameraArr cameras; // cameras mounted on the platform
		PoseArr poses; // trajectory of the platform

		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & name;
			ar & cameras;
			ar & poses;
		}
		#endif
	};
	typedef std::vector<Platform> PlatformArr;
	/*----------------------------------------------------------------*/

	// structure describing an image
	struct Image {
		std::string name; // image file name
		uint32_t platformID; // ID of the associated platform
		uint32_t cameraID; // ID of the associated camera on the associated platform
		uint32_t poseID; // ID of the pose of the associated platform

		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & name;
			ar & platformID;
			ar & cameraID;
			ar & poseID;
		}
		#endif
	};
	typedef std::vector<Image> ImageArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point
	struct Vertex {
		// structure describing one view for a given 3D feature
		struct View {
			uint32_t imageID; // image ID corresponding to this view
			Real confidence; // view's confidence (0 - not available)

			#ifdef _USE_BOOST
							   // implement BOOST serialization
			template<class Archive>
			void serialize(Archive& ar, const unsigned int /*version*/) {
				ar & imageID;
				ar & confidence;
			}
			#endif
		};
		typedef std::vector<View> ViewArr;

		Pos3 X; // 3D point position
		ViewArr views; // list of all available views for this 3D feature

		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & X;
			ar & views;
		}
		#endif
	};
	typedef std::vector<Vertex> VertexArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point's normal (optional)
	struct VertexNormal {
		Pos3 n; // 3D feature normal

		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & n;
		}
		#endif
	};
	typedef std::vector<VertexNormal> VertexNormalArr;
	/*----------------------------------------------------------------*/

	// structure describing a 3D point's color (optional)
	struct VertexColor {
		Col3 c; // 3D feature color

		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & c;
		}
		#endif
	};
	typedef std::vector<VertexColor> VertexColorArr;
	/*----------------------------------------------------------------*/

	PlatformArr platforms; // array of platforms
	ImageArr images; // array of images
	VertexArr vertices; // array of reconstructed 3D points
	VertexNormalArr verticesNormal; // array of reconstructed 3D points' normal (optional)
	VertexColorArr verticesColor; // array of reconstructed 3D points' color (optional)

	#ifdef _USE_BOOST
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & platforms;
		ar & images;
		ar & vertices;
		ar & verticesNormal;
		ar & verticesColor;
	}
	#endif
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _INTERFACE_H_

