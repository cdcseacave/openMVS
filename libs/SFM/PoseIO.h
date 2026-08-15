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

// In-plane rotation, in quarter turns about the optical axis, taking the camera frame a
// frames.json pose is expressed in to the working raster OpenMVS reconstructs in.
// Only meaningful for an EXIF-rotated image, which OpenMVS rotates 90 degrees clockwise on load
// so that every working raster is landscape (see View::ToWorkingOrientation); it is always 0 for
// an unrotated image, where the two frames coincide. A pose file never declares this, and no
// single value is right for all producers:
//   1  the raster as stored on disk, i.e. what ImportFramesJSON composes
//   2  the sensor-native landscape orientation, which is what ARKit reports poses in -- it sits
//      180 degrees from the landscape reached by rotating a display-oriented portrait raster
//      90 degrees clockwise, so an ARKit capture needs one more quarter turn than the import
//      applies (measured on a real iPhone capture: 0.55 deg of residual vs 68 deg at 1 turn)
// Getting it wrong conjugates every rotation by a multiple of Rz(90), which leaves its angle
// intact but tilts its axis, so the poses stay plausible while relative rotations disagree by up
// to twice the baseline rotation -- and no camera-axes flip can repair it.
constexpr unsigned FRAMES_IN_PLANE_TURNS = 4;

// The frame an imported pose set turned out to be expressed in: the camera axes and, for
// EXIF-rotated images, the in-plane rotation. The two choices are independent.
struct FramesPoseFrame {
	FramesConvention convention = FramesConvention::AUTO;
	unsigned inPlaneTurns = 0;

	// AUTO marks an inconclusive detection
	bool IsValid() const { return convention != FramesConvention::AUTO; }
};

// Human readable name of the given convention (for logging)
SFM_API String FramesConventionToString(FramesConvention convention);
// Human readable name of a detected pose frame, "<convention>/<turns>q" (for logging)
SFM_API String FramesPoseFrameToString(FramesPoseFrame poseFrame);

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
 * @brief Re-express every posed image in the detected pose frame
 *
 * Rewrites `img.R` from the frame the import applied to the detected one, leaving the camera
 * centers untouched (neither an axes flip nor an in-plane rotation moves the camera):
 *  - a camera-axes flip is `img.R <- diag(1,-1,-1) * img.R` (derivation: the camera-to-world
 *    rotation changes as `R_c2w * D`, so its transpose, which is what Pose3D stores, changes
 *    as `D * R_c2w^T`);
 *  - the in-plane rotation replaces the single `Rz(90)` ImportFramesJSON composes for an
 *    EXIF-rotated image with `inPlaneTurns` quarter turns (see FRAMES_IN_PLANE_TURNS);
 *  - when both apply the flip is taken between the two in-plane rotations, so order matters.
 * Idempotent only in the sense that applying the frame the import already used is a no-op.
 */
SFM_API void ApplyFramesPoseFrame(Scene& scene, FramesConvention appliedConvention,
	FramesPoseFrame poseFrame);

/**
 * @brief Decide which frame the imported poses are expressed in
 *
 * Searches the camera-axes convention and, when the scene holds EXIF-rotated posed images, the
 * in-plane rotation as well -- up to eight hypotheses, since the two are independent and only
 * their combination can be scored (a wrong in-plane rotation cannot be repaired by an axes flip).
 * Primary signal: over the pairs carrying a match-verified relative pose, the imported
 * relative rotation is compared against the verified one under every hypothesis and the
 * lowest median angular error wins.
 * Fallback (no pair was verified, e.g. all pairs are F-only, or the rotation signal is
 * ambiguous, e.g. a low-rotation forward walk): the matches of the highest-weighted pairs
 * are two-view triangulated with the imported poses under every hypothesis and the one
 * producing the most cheirality-positive, low-reprojection inliers wins.
 * Both tests require a clear margin over the runner-up, so that a genuinely ambiguous scene
 * reports AUTO instead of guessing.
 * @param scene scene with imported poses and geometrically verified pairs (not modified)
 * @param appliedConvention convention the scene poses were imported with; AUTO is treated
 *        as ARKIT, matching what Scene::Import applies when the configured convention is AUTO
 * @param knownConvention when set, only the in-plane rotation is searched (the caller was
 *        given an explicit convention, so the axes are not in question)
 * @return the detected pose frame, invalid (AUTO) when the evidence is inconclusive
 *         (the caller must fail loudly and ask for an explicit convention)
 */
SFM_API FramesPoseFrame DetectFramesConvention(const Scene& scene,
	FramesConvention appliedConvention = FramesConvention::ARKIT,
	bool knownConvention = false);

/**
 * @brief Resolve the frame of a frames.json pose import
 *
 * No-op unless the import was a frames.json needing a search: an AUTO camera-axes convention,
 * or any EXIF-rotated posed image (whose in-plane rotation is never declared by the file, so
 * it is resolved from the data even when the convention was given explicitly). Runs
 * DetectFramesConvention() on the matched pairs and re-expresses the imported poses when the
 * result differs from what the import applied (see ImportPoses); must therefore run after
 * matching and before the poses are used or persisted.
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
