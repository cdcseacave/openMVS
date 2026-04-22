////////////////////////////////////////////////////////////////////
// InterfaceMVS.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_INTERFACEMVS_H_
#define _SFM_INTERFACEMVS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;


// Depth-map undistortion using depth-aware interpolation
bool UndistortDMAP(const String& depthMapFile,
	const cv::Mat& map1, const cv::Mat& map2, const KMatrix& imageUndistortedK);

// Batch undistort depth-maps at their native resolution, matching image undistortion.
// alpha should match the alpha used for image undistortion.
bool UndistortDepthMaps(const Scene& scene,
	const CLISTDEF2(String)& depthMapFiles,
	float alpha=0.6f,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics=NULL);


// Depth-map import from MVS format (similar to MVS::ImportDepthDataRaw)
bool ImportDepthDataRaw(const String& fileName, String& imageFileName,
	IIndexArr& IDs, cv::Size& imageSize, cv::Size& depthSize,
	KMatrix& K, RMatrix& R, CMatrix& C,
	float& dMin, float& dMax,
	Image32F& depthMap, Image32F3& normalMap, Image32F& confMap, Image8U4& viewsMap, unsigned flags=15/*all*/);

// Depth-map export to MVS format (similar to MVS::ExportDepthDataRaw)
bool ExportDepthDataRaw(const String& fileName, const String& imageFileName,
	const IIndexArr& IDs, const cv::Size& imageSize,
	const KMatrix& K, const RMatrix& R, const Point3& C,
	float dMin, float dMax,
	const Image32F& depthMap, const Image32F& confMap, const Image8U4& viewsMap);


/**
 * @brief Import an MVS::Interface (.mvs) project file into the SfM scene
 * Populates cameras, images, poses, tracks and optional colors from the
 * serialized interface. Existing scene data is released prior to import.
 * @param fileName input .mvs file path
 * @param scene output SfM scene to populate
 * @param loadColors whether to import per-point colors when present
 * @return true on success
 */
bool ImportMVS(const String& fileName, Scene& scene, bool loadColors=true);

// Configuration bundle for ExportMVS. All fields have sane defaults so the
// common case is just `SFM::ExportMVS(path, scene)`. Individual knobs can
// be overridden with designated initializers, e.g.
//   SFM::ExportMVS(path, scene, { .undistortImageDir = "undist",
//                                 .sphericalNumFaces = 20 });
struct SFM_API ExportMVSConfig {
	// Optional directory to store undistorted images (created if missing).
	// If empty, no undistortion is performed (original image paths are exported as-is).
	// For scenes with spherical cameras, this directory doubles as the output
	// root for the rendered cube-map face files; when left empty the faces
	// are written next to the .mvs output file.
	String undistortImageDir;

	// Output image file extension (for undistorted images and spherical faces).
	String extension = _T(".jxl");

	// Alpha parameter for undistortion (0 = zoomed in, 1 = keep all pixels).
	// Only applies when undistortImageDir is set.
	float undistortAlpha = 0.6f;

	// When true, export only tracks with numInliers > 1 (otherwise any
	// track with ≥ 2 observations is exported).
	bool onlyInlierTracks = true;

	// Export per-point colors if available on the scene.
	bool includeColors = true;

	// Cube-map expansion parameters for spherical cameras. Ignored when the
	// scene has no spherical cameras. See SphereCubeMap::FaceRotations for the
	// supported face counts {4, 6, 8, 12, 20}.
	int    sphericalFaceSize  = 1024;       // square face resolution in pixels
	int    sphericalNumFaces  = 6;          // 4 | 6 | 8 | 12 | 20
};

/**
 * @brief Export current SfM scene to an MVS::Interface (.mvs) project file.
 *
 * Pinhole scenes are written as-is (one platform with a single mounted
 * camera per SFM camera, optionally undistorted).
 *
 * If the scene contains any spherical cameras, each spherical source image
 * is automatically expanded into a cube-map rig of N virtual pinhole faces
 * (config.sphericalNumFaces, default 6) that are rendered to disk alongside
 * the existing pinhole images. All faces of one source spherical image
 * share the source pose on the rig platform.
 *
 * @param fileName output .mvs file path
 * @param scene    input SfM scene to export
 * @param config   export configuration (undistortion, track-inlier filter,
 *                 color export, spherical cube-map options)
 * @return true on success
 */
bool ExportMVS(const String& fileName, const Scene& scene,
	ExportMVSConfig config = {});
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_INTERFACEMVS_H_
