////////////////////////////////////////////////////////////////////
// ImportROMA2.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_IMPORTROMA2_H_
#define _SFM_IMPORTROMA2_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Foreward declarations
class SFM_API PairsMatcher;

// Configuration for ROMAv2 matches
struct SFM_API ROMA2Config {
	// Import matches settings
	String importROMA2Path;	               // optional path to import ROMA2 .npz files
	float minPairWeight = 3.f;             // minimum composite weight for image pairs to be included during matches import
	float epipolarThreshold = 2.f;         // maximum distance to epipolar line when filtering candidates
	int erodeBorder = 8;                   // border size (in pixels) to erode disparity-map (0 = disabled)
	// Import depth-maps settings
	String depthMapPath;                   // optional output folder where to save depth-map files
	float minConfidence = 0.3f;            // minimum confidence threshold for depth map correspondences
	float minErodeConfidence = 0.9f;       // minimum confidence threshold for depth map correspondences
	float minTriangulationAngle = 0.9f;    // minimum triangulation angle in degrees (0 = disabled)
	float maxReprojectionError = 2.f;      // maximum reprojection error for the triangulated depth estimate
	float depthSimilarityThreshold = 0.3f; // maximum depth similarity threshold for depth-map correspondences
	bool weightedDepthAverage = true;      // use weighted average when merging depth-map correspondences
};

// Import list of ROMAv2 NPZ files from a directory, single file, or semicolon-separated list.
CLISTDEF2(String) ImportROMA2Files(const String& importROMA2Path);

// Import ROMAv2 matches from NPZ files listed in npzFiles (absolute or relative paths).
// Returns the number of pairs created/updated with matches.
unsigned ImportROMA2Matches(
	PairsMatcher& pairsMatcher,
	const CLISTDEF2(String)& npzFiles,
	const ROMA2Config& config = {});
// Convenience overload: accept a directory (scans for .npz), a single file, or a semicolon-separated list.
unsigned ImportROMA2Matches(
	PairsMatcher& pairsMatcher,
	const ROMA2Config& config = {});

// Import ROMAv2 depths-maps from NPZ files listed in npzFiles (absolute or relative paths).
// Returns the number of images updated with depth maps, and optionally the paths to the saved depth-map files.
SFM_API unsigned ImportROMA2DepthMaps(
	class Scene& scene,
	const CLISTDEF2(String)& npzFiles,
	const ROMA2Config& config = {},
    CLISTDEF2(String)* outDepthMapFiles = NULL);
// Convenience overload: accept a directory (scans for .npz), a single file, or a semicolon-separated list.
SFM_API unsigned ImportROMA2DepthMaps(
	class Scene& scene,
    const ROMA2Config& config = {},
    CLISTDEF2(String)* outDepthMapFiles = NULL);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_IMPORTROMA2_H_
