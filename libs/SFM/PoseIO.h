////////////////////////////////////////////////////////////////////
// PoseIO.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_POSEIO_H_
#define _SFM_POSEIO_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;

// Mode for importing camera poses from an external file
enum class PoseImportMode {
	NONE = 0,              // no import
	POSES_INTRINSICS = 1,  // import intrinsics (when available) and rotation + camera center
	POSES = 2,             // import rotation + camera center only
	POSITIONS = 3          // import camera center only
};

/**
 * @brief Export/import image poses in the OpenMVS human-readable CSV format
 *
 * Schema per row: filename,fx,fy,cx,cy,qx,qy,qz,qw,Cx,Cy,Cz,score.
 * Rows are matched to images by file-name stem, case-insensitive; ambiguous stems are
 * rejected. POSES_INTRINSICS also marks applied intrinsics as trusted.
 * @return number of valid image poses exported/imported
 */
SFM_API unsigned ExportPosesCSV(const String& fileName, const ImageArr& images);
SFM_API unsigned ImportPosesCSV(const String& fileName, ImageArr& images,
	PoseImportMode mode = PoseImportMode::POSES_INTRINSICS);

// Camera-axes convention of a frames.json `transform` matrix.
// The file stores a camera-to-world transform, but does not declare the camera axes
// it is expressed in; the two options differ by a pi rotation about the camera X axis,
// so picking the wrong one reverses every optical axis and triangulation collapses.
enum class FramesConvention {
	AUTO = 0,   // unknown: decide after matching, from the verified relative poses
	ARKIT = 1,  // ARKit/OpenGL camera axes (X right, Y up, Z backward)
	OPENCV = 2  // OpenCV camera axes (X right, Y down, Z forward)
};

// Human readable name of the given convention (for logging)
SFM_API String FramesConventionToString(FramesConvention convention);

// Parse a convention name (the inverse of FramesConventionToString, case-insensitive);
// empty and "auto" map to AUTO; returns false for any other name
SFM_API bool FramesConventionFromString(const String& str, FramesConvention& convention);

/**
 * @brief Import camera poses (and optionally intrinsics) from a Polycam-style frames.json
 *
 * The file is a JSON array of `{name, transform[16], params?}` entries, where `transform`
 * is a column-major 4x4 camera-to-world matrix and the optional `params` holds an OPENCV
 * camera model declared for a possibly different image resolution.
 * Entries are matched to the already imported scene images by file name (full name first,
 * then stem, both case-insensitive); ambiguous names and duplicate entries are rejected,
 * while images without a matching entry are left unposed.
 * The intrinsics are only imported when the mode asks for them and `params` is present,
 * otherwise the EXIF-derived intrinsics computed by Image::LoadMetadata are kept.
 * Must be called before the images are assigned shared cameras, so that identical
 * per-frame intrinsics collapse into a single camera.
 * @param fileName input frames.json file path
 * @param scene scene holding the images to be posed
 * @param mode what to import from each entry (see PoseImportMode)
 * @param convention camera-axes convention of the `transform` matrices; AUTO is not
 *        accepted here (it can only be resolved after matching, see DetectFramesConvention)
 * @return number of images that received a pose; 0 on parse failure
 */
SFM_API unsigned ImportFramesJSON(const String& fileName, Scene& scene, PoseImportMode mode,
	FramesConvention convention = FramesConvention::ARKIT);

/**
 * @brief Import camera poses (and optionally intrinsics) from a file, dispatched by extension
 *
 * Supported formats: the OpenMVS CSV (.csv, see ImportPosesCSV) and the Polycam frames.json
 * (.json, see ImportFramesJSON). The convention only applies to a frames.json import; AUTO
 * optimistically imports as ARKit, to be fixed after matching by ResolveFramesConvention().
 * @return true when at least one image received a pose
 */
SFM_API bool ImportPoses(Scene& scene, const String& fileName, PoseImportMode mode,
	FramesConvention convention = FramesConvention::AUTO);

/**
 * @brief Flip every posed image between the two camera-axes conventions
 *
 * `img.R <- diag(1,-1,-1) * img.R`, camera centers unchanged
 * (derivation: the camera-to-world rotation changes as `R_c2w * D`, so its
 * transpose, which is what Pose3D stores, changes as `D * R_c2w^T`).
 * EXIF-rotated images store `Rz(90) * D * R_c2w^T` (see ImportFramesJSON), so their
 * flip conjugates to `Rz(90) * D * Rz(-90) = diag(-1,1,-1)` instead.
 */
SFM_API void FlipFramesConvention(Scene& scene);

/**
 * @brief Decide which camera-axes convention the imported poses use
 *
 * Primary signal: over the pairs carrying a match-verified relative pose, the imported
 * relative rotation is compared against the verified one under both hypotheses and the
 * lower median angular error wins.
 * Fallback (no pair was verified, e.g. all pairs are F-only, or the rotation signal is
 * ambiguous, e.g. a low-rotation forward walk): the matches of the highest-weighted pairs
 * are two-view triangulated with the imported poses under both hypotheses and the one
 * producing more cheirality-positive, low-reprojection inliers wins.
 * Both tests require a clear margin, so that a genuinely ambiguous scene reports AUTO
 * instead of guessing.
 * @param scene scene with imported poses and geometrically verified pairs (not modified)
 * @param appliedConvention convention the scene poses were imported with; AUTO is treated
 *        as ARKIT, matching what Scene::Import applies when the configured convention is AUTO
 * @return the detected convention, or AUTO when the evidence is inconclusive
 *         (the caller must fail loudly and ask for an explicit convention)
 */
SFM_API FramesConvention DetectFramesConvention(const Scene& scene,
	FramesConvention appliedConvention = FramesConvention::ARKIT);

/**
 * @brief Resolve the camera-axes convention of an AUTO frames.json pose import
 *
 * No-op unless the import was a frames.json with FramesConvention::AUTO. Runs
 * DetectFramesConvention() on the matched pairs and flips the imported poses when the
 * detected convention differs from the optimistically applied ARKit one (see ImportPoses);
 * must therefore run after matching and before the poses are used or persisted.
 * @param scene scene with imported poses and matched pairs
 * @param configuredConvention convention the import was configured with
 * @param importPosesFile the imported poses file (identifies a frames.json import)
 * @return false when the evidence is inconclusive (the caller must fail loudly)
 */
SFM_API bool ResolveFramesConvention(Scene& scene, FramesConvention configuredConvention,
	const String& importPosesFile);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_POSEIO_H_
