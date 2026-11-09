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
#include "../Math/Camera/CameraUndistort.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;


// Depth-map undistortion using depth-aware interpolation
SFM_API bool UndistortDMAP(const String& depthMapFile,
	const UndistortMaps& maps, const KMatrix& imageUndistortedK);

// Batch undistort depth-maps at their native resolution, matching image undistortion:
// each depth-map lands in the frame of the undistorted camera UndistortedK(K, distortion, size, alpha)
// of its image (stored in the depth-map), scaled to the depth-map resolution; alpha is the one used for
// the undistorted image files, or UNDISTORT_ALPHA for a native export, which the MVS loader undistorts.
// undistortedIntrinsics receives that undistorted camera per distorted camera.
SFM_API bool UndistortDepthMaps(const Scene& scene,
	const CLISTDEF2(String)& depthMapFiles,
	float alpha=0.6f,
	std::unordered_map<const Camera*, KMatrix>* undistortedIntrinsics=NULL);


// Depth-map import from MVS format (similar to MVS::ImportDepthDataRaw).
// The DMAP header carries the recalibrated-confidence flag (CONF_ADJUSTED in
// Math/Interface.h): anything that rewrites a depth-map has to carry it over, or the
// stored confidence would be recalibrated a second time by a later dense run.
SFM_API bool ImportDepthDataRaw(const String& fileName, String& imageFileName,
	IIndexArr& IDs, cv::Size& imageSize, cv::Size& depthSize,
	KMatrix& K, RMatrix& R, CMatrix& C,
	float& dMin, float& dMax,
	Image32F& depthMap, Image32F3& normalMap, Image32F& confMap, Image8U4& viewsMap,
	unsigned flags=15/*maps to read: HAS_DEPTH|HAS_NORMAL|HAS_CONF|HAS_VIEWS, all of them*/,
	bool* pbConfAdjusted=NULL/*receives the stored CONF_ADJUSTED flag*/);

// Depth-map export to MVS format (similar to MVS::ExportDepthDataRaw)
SFM_API bool ExportDepthDataRaw(const String& fileName, const String& imageFileName,
	const IIndexArr& IDs, const cv::Size& imageSize,
	const KMatrix& K, const RMatrix& R, const Point3& C,
	float dMin, float dMax,
	const Image32F& depthMap, const Image32F& confMap, const Image8U4& viewsMap,
	bool bConfAdjusted=false/*mark the stored confMap as already recalibrated (CONF_ADJUSTED)*/);


/**
 * @brief Import an MVS::Interface (.mvs) project file into the SfM scene
 * Populates cameras, images, poses, tracks and optional colors from the
 * serialized interface. Existing scene data is released prior to import.
 * @param fileName input .mvs file path
 * @param scene output SfM scene to populate
 * @param loadColors whether to import per-point colors when present
 * @return true on success
 */
SFM_API bool ImportMVS(const String& fileName, Scene& scene, bool loadColors=true);

// Configuration bundle for ExportMVS. All fields have sane defaults so the
// common case is just `SFM::ExportMVS(path, scene)`. Individual knobs can
// be overridden with designated initializers, e.g.
//   SFM::ExportMVS(path, scene, { .undistortImageDir = "undist",
//                                 .sphericalNumFaces = 20 });
struct SFM_API ExportMVSConfig {
	// how the spherical cameras are exported
	enum class Spherical {
		NATIVE, // equirectangular cameras (Interface SPHERICAL model) of the original images
		CUBEMAP, // each image expanded into a rig of sphericalNumFaces rendered pinhole faces
	};

	// Optional directory to store undistorted images (created if missing).
	// If empty, the distorted cameras are exported with their Brown distortion
	// and the original image paths (the MVS loader undistorts on load).
	// In CUBEMAP mode this directory doubles as the output root for the
	// rendered face files; when left empty the faces are written next to
	// the .mvs output file.
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

	// Export the feature position of every view (Interface HAS_OBSERVATIONS):
	// the keypoint in the exported image file, so the scene must still hold
	// the keypoints of its tracks.
	bool includeObservations = false;

	Spherical sphericalExport = Spherical::NATIVE;

	// Cube-map expansion parameters for spherical cameras in CUBEMAP mode.
	// See SphereCubeMap::FaceRotations for the supported face counts {4, 6, 8, 12, 20}.
	int    sphericalFaceSize  = 1024;       // square face resolution in pixels
	int    sphericalNumFaces  = 6;          // 4 | 6 | 8 | 12 | 20
};

/**
 * @brief Export current SfM scene to an MVS::Interface (.mvs) project file.
 *
 * Pinhole cameras: one platform with a single mounted camera per SFM camera;
 * a distorted camera is written as an Interface BROWN camera of the original
 * images, or, if undistortImageDir is set, as the pinhole camera of the
 * undistorted images written there.
 *
 * Spherical cameras: NATIVE writes one SPHERICAL camera per SFM camera with
 * the angular K of the original images. CUBEMAP expands each spherical image
 * into a rig of N virtual pinhole faces (config.sphericalNumFaces, default 6)
 * rendered to disk; all faces of one source image share its pose on the rig
 * platform.
 *
 * @param fileName output .mvs file path
 * @param scene    input SfM scene to export
 * @param config   export configuration (undistortion, track-inlier filter,
 *                 color export, spherical cube-map options)
 * @return true on success
 */
SFM_API bool ExportMVS(const String& fileName, const Scene& scene,
	ExportMVSConfig config = {});
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_INTERFACEMVS_H_
