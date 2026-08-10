////////////////////////////////////////////////////////////////////
// ImportFramesJSON.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImportFramesJSON.h"
#include "Scene.h"
#include "Triangulation.h"
#include "../IO/json.hpp"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// Largest deviation from orthonormality (max |R^T*R - I| element and |det(R)-1|) tolerated
// in an imported camera-to-world rotation: within it the matrix is re-orthonormalized,
// outside it the entry is rejected (sanitation of an external file)
constexpr REAL orthonormalityTolerance = REAL(1e-3);
// Largest deviation tolerated in the last row of the 4x4 transform, which must be (0,0,0,1);
// a transform failing this is either not affine or not stored column-major
constexpr REAL affineRowTolerance = REAL(1e-6);
// Largest relative disagreement tolerated between the horizontal and vertical rescale
// factors implied by `params.w/h` versus the actual image resolution
constexpr REAL intrinsicsScaleTolerance = REAL(1e-3);
// Number of per-entry problems reported individually before only the totals are logged
constexpr unsigned maxLoggedWarnings = 10;

// Convention detection: the winning hypothesis must be this many times better than the other
constexpr REAL conventionMarginRatio = REAL(3);
// Convention detection: minimum number of match-verified pairs for the primary signal
constexpr unsigned conventionMinVerifiedPairs = 3;
// Convention detection fallback: number of highest-weighted pairs triangulated
constexpr unsigned conventionMaxTriangulatedPairs = 10;
// Convention detection fallback: matches sampled per pair
constexpr unsigned conventionMaxMatchesPerPair = 200;
// Convention detection fallback: minimum inliers the winning hypothesis must reach
constexpr unsigned conventionMinTriangulatedInliers = 20;
// Convention detection fallback: triangulation thresholds
constexpr float conventionReprojThreshold = 4.f;
constexpr float conventionMinAngle = 0.5f;

// Pi rotation about the camera X axis, converting between the ARKit/OpenGL and the OpenCV
// camera axes; it is symmetric and its own inverse, so the same matrix applies both ways
inline Matrix3x3 CameraAxesFlip() {
	return Matrix3x3(
		1, 0, 0,
		0, -1, 0,
		0, 0, -1);
}

// The convention on the other side of the axes flip
inline FramesConvention OppositeConvention(FramesConvention convention) {
	return convention == FramesConvention::ARKIT ? FramesConvention::OPENCV : FramesConvention::ARKIT;
}

// Case-insensitive file name key used to match a frames.json entry to a scene image
inline String NameKey(const String& path) {
	return Util::getFileNameExt(path).ToLower();
}
inline String StemKey(const String& path) {
	return Util::getFileName(path).ToLower();
}

// Check that the given matrix is a rotation, up to the given tolerance
bool IsRotationMatrix(const Matrix3x3& R, REAL tolerance)
{
	// R^T*R must be the identity
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			const REAL dot = R(0,i)*R(0,j) + R(1,i)*R(1,j) + R(2,i)*R(2,j);
			if (ABS(dot - (i == j ? REAL(1) : REAL(0))) > tolerance)
				return false;
		}
	}
	// and the determinant must be +1 (a mirroring matrix is not a valid pose)
	const REAL det =
		R(0,0) * (R(1,1)*R(2,2) - R(1,2)*R(2,1)) -
		R(0,1) * (R(1,0)*R(2,2) - R(1,2)*R(2,0)) +
		R(0,2) * (R(1,0)*R(2,1) - R(1,1)*R(2,0));
	return ABS(det - REAL(1)) <= tolerance;
}

// Apply the imported OPENCV intrinsics, declared for the original (on-disk) image
// orientation and resolution, to the camera built at the working resolution.
// Returns false and fills `error` when the parameters cannot be used.
bool ApplyImportedIntrinsics(const nlohmann::json& params, Image& img, String& error)
{
	PinholeCamera* const camera = dynamic_cast<PinholeCamera*>(img.pCamera);
	if (camera == NULL) {
		error = "only pinhole cameras support imported intrinsics";
		return false;
	}
	const auto itModel = params.find("camera_model");
	if (itModel == params.end() || !itModel->is_string()) {
		error = "missing 'camera_model'";
		return false;
	}
	const String model(itModel->get<std::string>());
	if (model != "OPENCV") {
		error = String::FormatString("unsupported camera model '%s' (supported: OPENCV)", model.c_str());
		return false;
	}
	// read the declared parameters
	REAL declaredWidth = 0, declaredHeight = 0;
	REAL fx = 0, fy = 0, cx = 0, cy = 0, k1 = 0, k2 = 0, p1 = 0, p2 = 0;
	const struct { const char* name; REAL* value; } fields[] = {
		{"w", &declaredWidth}, {"h", &declaredHeight},
		{"fx", &fx}, {"fy", &fy}, {"cx", &cx}, {"cy", &cy},
		{"k1", &k1}, {"k2", &k2}, {"p1", &p1}, {"p2", &p2}
	};
	for (const auto& field : fields) {
		const auto it = params.find(field.name);
		if (it == params.end() || !it->is_number()) {
			error = String::FormatString("missing or invalid '%s'", field.name);
			return false;
		}
		*field.value = it->get<REAL>();
	}
	if (declaredWidth <= 0 || declaredHeight <= 0 || fx <= 0 || fy <= 0) {
		error = String::FormatString("invalid resolution %gx%g or focal (%g, %g)",
			declaredWidth, declaredHeight, fx, fy);
		return false;
	}
	// the parameters describe the image as stored on disk, so they must be rescaled to the
	// original resolution (the camera is built at the working resolution, landscape)
	const cv::Size originalSize = img.GetOriginalSize();
	const REAL scale = (REAL)originalSize.width / declaredWidth;
	const REAL scaleHeight = (REAL)originalSize.height / declaredHeight;
	if (ABS(scaleHeight - scale) > intrinsicsScaleTolerance * scale) {
		error = String::FormatString("declared resolution %gx%g does not match image %dx%d (scale %g vs %g)",
			declaredWidth, declaredHeight, originalSize.width, originalSize.height, scale, scaleHeight);
		return false;
	}
	fx *= scale; fy *= scale;
	cx *= scale; cy *= scale;
	if (img.IsRotated()) {
		// the raster is rotated 90 degrees clockwise on load (View::ToWorkingOrientation),
		// so the intrinsics must follow: this is the inverse of the K branch of
		// View::RevertRotation, with the tangential coefficients rotated accordingly
		// (the radial ones are invariant to an in-plane rotation)
		camera->SetIntrinsics(fy, fx, (REAL)(originalSize.height - 1) - cy, cx);
		camera->SetDistortion(k1, k2, p2, -p1);
	} else {
		camera->SetIntrinsics(fx, fy, cx, cy);
		camera->SetDistortion(k1, k2, p1, p2);
	}
	camera->trustIntrinsics = true;
	return true;
}

// Count the two-view triangulation inliers of the given matches under one axes hypothesis:
// an inlier is a match triangulating in front of both cameras with a small reprojection error
unsigned CountTriangulationInliers(const Image& img1, const Image& img2,
	const std::vector<DMatch>& matches, bool flipAxes)
{
	// build a two-image scene holding the hypothesis poses; the shared cameras are borrowed,
	// so the camera IDs are kept valid to stop the view destructor from deleting them
	ImageArr images(2);
	const Matrix3x3 flip = CameraAxesFlip();
	for (IIndex k = 0; k < 2; ++k) {
		const Image& src = k == 0 ? img1 : img2;
		Image& dst = images[k];
		dst.ID = k;
		dst.cameraID = k;
		dst.pCamera = src.pCamera;
		dst.keypoints = src.keypoints;
		dst.R = flipAxes ? RMatrix(flip * src.R) : src.R;
		dst.C = src.C;
	}
	const unsigned numMatches = (unsigned)matches.size();
	const unsigned step = MAXF(1u, numMatches / conventionMaxMatchesPerPair);
	unsigned numInliers = 0;
	for (unsigned i = 0; i < numMatches; i += step) {
		const DMatch& match = matches[i];
		if (match.queryIdx >= images[0].keypoints.size() ||
			match.trainIdx >= images[1].keypoints.size())
			continue;
		Track track;
		track.observations.emplace_back(0u, match.queryIdx);
		track.observations.emplace_back(1u, match.trainIdx);
		if (TriangulateSkewLLS(track, images, conventionReprojThreshold, conventionMinAngle, 2) < 2)
			continue;
		if (images[0].Depth(track.position) <= 0 || images[1].Depth(track.position) <= 0)
			continue;
		++numInliers;
	}
	// release the borrowed cameras before the array is destroyed
	for (Image& img : images) {
		img.cameraID = NO_ID;
		img.pCamera = NULL;
	}
	return numInliers;
}

// Pair candidate used to select the strongest pairs for the detection fallback
struct PairScore {
	float score;
	IIndex idx;
};

} // unnamed namespace


String SFM::FramesConventionToString(FramesConvention convention)
{
	switch (convention) {
	case FramesConvention::ARKIT: return "arkit";
	case FramesConvention::OPENCV: return "opencv";
	default: return "auto";
	}
} // FramesConventionToString
/*----------------------------------------------------------------*/


unsigned SFM::ImportFramesJSON(const String& fileName, Scene& scene, PoseImportMode mode,
	FramesConvention convention)
{
	if (mode == PoseImportMode::NONE)
		return 0;
	if (convention == FramesConvention::AUTO) {
		VERBOSE("error: the camera-axes convention of '%s' can only be resolved after matching; "
			"import it as arkit or opencv", fileName.c_str());
		return 0;
	}
	std::ifstream stream(fileName.c_str());
	if (!stream.is_open()) {
		VERBOSE("error: failed to open frames file '%s'", fileName.c_str());
		return 0;
	}
	const nlohmann::json data = nlohmann::json::parse(stream, nullptr, false);
	if (data.is_discarded()) {
		VERBOSE("error: failed to parse frames file '%s'", fileName.c_str());
		return 0;
	}
	if (!data.is_array() || data.empty()) {
		VERBOSE("error: frames file '%s' is not a non-empty array of frames", fileName.c_str());
		return 0;
	}

	// index the images by full file name and by stem, both case-insensitive
	std::unordered_map<String, IIndex> imageByName, imageByStem;
	imageByName.reserve(scene.images.size());
	imageByStem.reserve(scene.images.size());
	FOREACH(i, scene.images) {
		const String& imgFileName = scene.images[i].fileName;
		imageByName.emplace(NameKey(imgFileName), i);
		imageByStem.emplace(StemKey(imgFileName), i);
	}

	const Matrix3x3 flip = CameraAxesFlip();
	unsigned numPosed = 0, numUnmatched = 0, numRejected = 0, numIntrinsics = 0, numWarnings = 0;
	bool intrinsicsRequestedButMissing = false;
	for (size_t e = 0; e < data.size(); ++e) {
		const nlohmann::json& entry = data[e];
		const auto itName = entry.find("name"); // returns end() for any non-object entry
		if (itName == entry.end() || !itName->is_string()) {
			if (++numWarnings <= maxLoggedWarnings)
				VERBOSE("error: frame %u of '%s' has no 'name' string; skipped",
					(unsigned)e, fileName.c_str());
			++numRejected;
			continue;
		}
		const String name(itName->get<std::string>());
		// find the matching image: full name first, then stem
		IIndex imageID = NO_ID;
		const auto itByName = imageByName.find(NameKey(name));
		if (itByName != imageByName.end()) {
			imageID = itByName->second;
		} else {
			const auto itByStem = imageByStem.find(StemKey(name));
			if (itByStem != imageByStem.end())
				imageID = itByStem->second;
		}
		if (imageID == NO_ID) {
			if (++numWarnings <= maxLoggedWarnings)
				VERBOSE("warning: frame '%s' of '%s' matches no input image; skipped",
					name.c_str(), fileName.c_str());
			++numUnmatched;
			continue;
		}
		Image& img = scene.images[imageID];
		// read the 4x4 column-major camera-to-world transform
		const auto itTransform = entry.find("transform");
		if (itTransform == entry.end() || !itTransform->is_array() || itTransform->size() != 16) {
			if (++numWarnings <= maxLoggedWarnings)
				VERBOSE("error: frame '%s' of '%s' has no 'transform' array of 16 numbers; skipped",
					name.c_str(), fileName.c_str());
			++numRejected;
			continue;
		}
		REAL transform[16] = {};
		bool validNumbers = true;
		for (unsigned k = 0; k < 16; ++k) {
			const nlohmann::json& value = (*itTransform)[k];
			if (!value.is_number()) {
				validNumbers = false;
				break;
			}
			transform[k] = value.get<REAL>();
		}
		if (!validNumbers) {
			if (++numWarnings <= maxLoggedWarnings)
				VERBOSE("error: frame '%s' of '%s' has a non-numeric 'transform'; skipped",
					name.c_str(), fileName.c_str());
			++numRejected;
			continue;
		}
		// the last row of a column-major affine transform must be (0,0,0,1);
		// a row-major file would carry its translation here instead
		if (ABS(transform[3]) > affineRowTolerance || ABS(transform[7]) > affineRowTolerance ||
			ABS(transform[11]) > affineRowTolerance || ABS(transform[15] - REAL(1)) > affineRowTolerance)
		{
			if (++numWarnings <= maxLoggedWarnings)
				VERBOSE("error: frame '%s' of '%s' has last row (%g, %g, %g, %g) instead of (0, 0, 0, 1); "
					"expected a column-major camera-to-world matrix; skipped", name.c_str(), fileName.c_str(),
					transform[3], transform[7], transform[11], transform[15]);
			++numRejected;
			continue;
		}
		const CMatrix center(transform[12], transform[13], transform[14]);
		if (mode != PoseImportMode::POSITIONS) {
			// camera-to-world rotation, stored column-major
			Matrix3x3 rotationC2W;
			for (int c = 0; c < 3; ++c)
				for (int r = 0; r < 3; ++r)
					rotationC2W(r, c) = transform[c*4 + r];
			if (!IsRotationMatrix(rotationC2W, orthonormalityTolerance)) {
				if (++numWarnings <= maxLoggedWarnings)
					VERBOSE("error: frame '%s' of '%s' has a non-orthonormal rotation; skipped",
						name.c_str(), fileName.c_str());
				++numRejected;
				continue;
			}
			if (convention == FramesConvention::ARKIT)
				rotationC2W = Matrix3x3(rotationC2W * flip);
			// Pose3D stores the world-to-camera rotation
			RMatrix rotation(rotationC2W.t());
			rotation.EnforceOrthogonality();
			if (img.IsRotated()) {
				// the imported pose describes the image as stored on disk, but the raster is
				// rotated 90 degrees clockwise on load (View::ToWorkingOrientation), so the
				// same in-plane rotation must be composed here; it is the inverse of the R
				// branch of View::RevertRotation, which undoes it again on export
				rotation = RMatrix(RMatrix(0, 0, REAL(M_PI_2)) * rotation);
			}
			img.R = rotation;
		}
		img.C = center;
		++numPosed;
		// import the intrinsics only when asked for and available
		if (mode == PoseImportMode::POSES_INTRINSICS) {
			const auto itParams = entry.find("params");
			if (itParams == entry.end() || !itParams->is_object()) {
				intrinsicsRequestedButMissing = true;
			} else if (!img.HasCamera()) {
				if (++numWarnings <= maxLoggedWarnings)
					VERBOSE("warning: frame '%s' of '%s' has intrinsics, but the image has no camera; ignored",
						name.c_str(), fileName.c_str());
			} else {
				String error;
				if (ApplyImportedIntrinsics(*itParams, img, error)) {
					++numIntrinsics;
				} else if (++numWarnings <= maxLoggedWarnings) {
					VERBOSE("warning: frame '%s' of '%s' has unusable intrinsics (%s); using the EXIF ones",
						name.c_str(), fileName.c_str(), error.c_str());
				}
			}
		}
	}
	if (numWarnings > maxLoggedWarnings)
		VERBOSE("warning: %u more problems in '%s' not listed", numWarnings - maxLoggedWarnings, fileName.c_str());
	if (intrinsicsRequestedButMissing)
		DEBUG("Frames file '%s' has no 'params' for some frames; their intrinsics stay as estimated from EXIF",
			Util::getFileNameExt(fileName).c_str());

	unsigned numUnposed = 0;
	for (const Image& img : scene.images)
		if (!img.HasPose())
			++numUnposed;
	VERBOSE("Imported %u/%u frames from '%s' as %s poses: %u with intrinsics, "
		"%u unmatched frames, %u invalid frames, %u images left unposed",
		numPosed, (unsigned)data.size(), Util::getFileNameExt(fileName).c_str(),
		FramesConventionToString(convention).c_str(),
		numIntrinsics, numUnmatched, numRejected, numUnposed);
	return numPosed;
} // ImportFramesJSON
/*----------------------------------------------------------------*/


void SFM::FlipFramesConvention(Scene& scene)
{
	const Matrix3x3 flip = CameraAxesFlip();
	unsigned numFlipped = 0;
	for (Image& img : scene.images) {
		if (!img.HasPose())
			continue;
		img.R = RMatrix(flip * img.R);
		++numFlipped;
	}
	DEBUG("Flipped the camera-axes convention of %u poses", numFlipped);
} // FlipFramesConvention
/*----------------------------------------------------------------*/


FramesConvention SFM::DetectFramesConvention(const Scene& scene, FramesConvention appliedConvention)
{
	if (appliedConvention == FramesConvention::AUTO)
		appliedConvention = FramesConvention::ARKIT;
	const FramesConvention flippedConvention = OppositeConvention(appliedConvention);
	const Matrix3x3 flip = CameraAxesFlip();

	// primary signal: compare the imported relative rotations against the verified ones
	REALArr errorsApplied(0, scene.pairs.size()), errorsFlipped(0, scene.pairs.size());
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasMatches())
			continue;
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		if (!img1.HasPose() || !img2.HasPose())
			continue;
		// relativePose maps the first image to the second one, same as R2 * R1^T
		const Matrix3x3& verified = pair.relativePose->R;
		const Matrix3x3 relative(img2.R * img1.R.t());
		// flipping both images conjugates their relative rotation by the axes flip
		const Matrix3x3 relativeFlipped(Matrix3x3(flip * relative) * flip);
		errorsApplied.emplace_back(ComputeAngleSO3<REAL>(relative, verified));
		errorsFlipped.emplace_back(ComputeAngleSO3<REAL>(relativeFlipped, verified));
	}
	const unsigned numVerifiedPairs = (unsigned)errorsApplied.size();
	if (numVerifiedPairs >= conventionMinVerifiedPairs) {
		const REAL medianApplied = errorsApplied.GetMedian();
		const REAL medianFlipped = errorsFlipped.GetMedian();
		DEBUG_EXTRA("Frames convention: %u verified pairs, median relative rotation error %.2f deg as %s, %.2f deg as %s",
			numVerifiedPairs, R2D(medianApplied), FramesConventionToString(appliedConvention).c_str(),
			R2D(medianFlipped), FramesConventionToString(flippedConvention).c_str());
		if (medianApplied * conventionMarginRatio < medianFlipped) {
			VERBOSE("Detected %s camera-axes convention (median relative rotation error %.2f deg vs %.2f deg)",
				FramesConventionToString(appliedConvention).c_str(), R2D(medianApplied), R2D(medianFlipped));
			return appliedConvention;
		}
		if (medianFlipped * conventionMarginRatio < medianApplied) {
			VERBOSE("Detected %s camera-axes convention (median relative rotation error %.2f deg vs %.2f deg)",
				FramesConventionToString(flippedConvention).c_str(), R2D(medianFlipped), R2D(medianApplied));
			return flippedConvention;
		}
		VERBOSE("error: the camera-axes convention is ambiguous: median relative rotation error "
			"%.2f deg as %s vs %.2f deg as %s over %u verified pairs",
			R2D(medianApplied), FramesConventionToString(appliedConvention).c_str(),
			R2D(medianFlipped), FramesConventionToString(flippedConvention).c_str(), numVerifiedPairs);
		return FramesConvention::AUTO;
	}

	// fallback: triangulate the strongest pairs under both hypotheses and compare the inliers
	CLISTDEF0(PairScore) candidates;
	candidates.reserve(scene.pairs.size());
	FOREACH(p, scene.pairs) {
		const ImagePair& pair = scene.pairs[p];
		if (!pair.HasMatches())
			continue;
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		if (!img1.IsValid() || !img2.IsValid())
			continue;
		const float weight = pair.GetCompositeWeight();
		candidates.emplace_back(PairScore{weight > 0.f ? weight : (float)pair.GetNumInliers(), p});
	}
	if (candidates.empty()) {
		VERBOSE("error: cannot detect the camera-axes convention: no matched pair between posed images");
		return FramesConvention::AUTO;
	}
	std::sort(candidates.begin(), candidates.end(),
		[](const PairScore& a, const PairScore& b) { return a.score > b.score; });
	const unsigned numPairs = MINF((unsigned)candidates.size(), conventionMaxTriangulatedPairs);
	unsigned inliersApplied = 0, inliersFlipped = 0;
	for (unsigned i = 0; i < numPairs; ++i) {
		const ImagePair& pair = scene.pairs[candidates[i].idx];
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		inliersApplied += CountTriangulationInliers(img1, img2, pair.matches, false);
		inliersFlipped += CountTriangulationInliers(img1, img2, pair.matches, true);
	}
	DEBUG_EXTRA("Frames convention: %u triangulated pairs, %u inliers as %s, %u inliers as %s",
		numPairs, inliersApplied, FramesConventionToString(appliedConvention).c_str(),
		inliersFlipped, FramesConventionToString(flippedConvention).c_str());
	if (inliersApplied >= conventionMinTriangulatedInliers &&
		(REAL)inliersApplied > conventionMarginRatio * inliersFlipped)
	{
		VERBOSE("Detected %s camera-axes convention (%u vs %u two-view triangulation inliers)",
			FramesConventionToString(appliedConvention).c_str(), inliersApplied, inliersFlipped);
		return appliedConvention;
	}
	if (inliersFlipped >= conventionMinTriangulatedInliers &&
		(REAL)inliersFlipped > conventionMarginRatio * inliersApplied)
	{
		VERBOSE("Detected %s camera-axes convention (%u vs %u two-view triangulation inliers)",
			FramesConventionToString(flippedConvention).c_str(), inliersFlipped, inliersApplied);
		return flippedConvention;
	}
	VERBOSE("error: the camera-axes convention is ambiguous: %u two-view triangulation inliers as %s "
		"vs %u as %s over %u pairs",
		inliersApplied, FramesConventionToString(appliedConvention).c_str(),
		inliersFlipped, FramesConventionToString(flippedConvention).c_str(), numPairs);
	return FramesConvention::AUTO;
} // DetectFramesConvention
/*----------------------------------------------------------------*/
