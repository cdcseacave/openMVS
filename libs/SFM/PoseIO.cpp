////////////////////////////////////////////////////////////////////
// PoseIO.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "PoseIO.h"
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
constexpr REAL ORTHONORMALITY_TOLERANCE = REAL(1e-3);
// Largest deviation tolerated in the last row of the 4x4 transform, which must be (0,0,0,1);
// a transform failing this is either not affine or not stored column-major
constexpr REAL AFFINE_ROW_TOLERANCE = REAL(1e-6);
// Largest relative disagreement tolerated between the horizontal and vertical rescale
// factors implied by `params.w/h` versus the actual image resolution
constexpr REAL INTRINSICS_SCALE_TOLERANCE = REAL(1e-3);
// Number of per-entry problems reported individually before only the totals are logged
constexpr unsigned MAX_LOGGED_WARNINGS = 10;

// Convention detection: the winning hypothesis must be this many times better than the other
constexpr REAL CONVENTION_MARGIN_RATIO = REAL(3);
// Convention detection: minimum number of match-verified pairs for the primary signal
constexpr unsigned CONVENTION_MIN_VERIFIED_PAIRS = 3;
// Convention detection fallback: number of highest-weighted pairs triangulated
constexpr unsigned CONVENTION_MAX_TRIANGULATED_PAIRS = 10;
// Convention detection fallback: matches sampled per pair
constexpr unsigned CONVENTION_MAX_MATCHES_PER_PAIR = 200;
// Convention detection fallback: minimum inliers the winning hypothesis must reach
constexpr unsigned CONVENTION_MIN_TRIANGULATED_INLIERS = 20;
// Convention detection fallback: triangulation thresholds
constexpr float CONVENTION_REPROJ_THRESHOLD = 4.f;
constexpr float CONVENTION_MIN_ANGLE = 0.5f;

// Pi rotation about the camera X axis, converting between the ARKit/OpenGL and the OpenCV
// camera axes; it is symmetric and its own inverse, so the same matrix applies both ways
inline Matrix3x3 CameraAxesFlip() {
	return Matrix3x3(
		1, 0, 0,
		0, -1, 0,
		0, 0, -1);
}

// The axes flip expressed in the working raster orientation of the given image: an
// EXIF-rotated image stores Rz(90)*R_file (see ImportFramesJSON), so undoing the file-side
// flip conjugates it by that in-plane rotation, Rz(90)*D*Rz(-90) = diag(-1,1,-1);
// both matrices are symmetric and involutive, so the same flip applies both ways
inline Matrix3x3 CameraAxesFlipFor(const Image& img) {
	return img.IsRotated() ?
		Matrix3x3(
			-1, 0, 0,
			 0, 1, 0,
			 0, 0, -1) :
		CameraAxesFlip();
}

// The convention on the other side of the axes flip
inline FramesConvention OppositeConvention(FramesConvention convention) {
	return convention == FramesConvention::ARKIT ? FramesConvention::OPENCV : FramesConvention::ARKIT;
}

// Case-insensitive file name key used to match a poses-file entry to a scene image
inline String NameKey(const String& path) {
	return Util::getFileNameExt(path).ToLower();
}
inline String StemKey(const String& path) {
	return Util::getFileName(path).ToLower();
}

// Insert an image key, rejecting ambiguous keys so they never silently select one image
void AddUniqueImageKey(std::unordered_map<String, IIndex>& imageMap, const String& key, IIndex imageID)
{
	const auto [it, inserted] = imageMap.emplace(key, imageID);
	if (!inserted)
		it->second = NO_ID;
}

// Read a finite floating-point value from external JSON.
bool ReadFiniteNumber(const nlohmann::json& value, REAL& number)
{
	if (!value.is_number())
		return false;
	try {
		number = value.get<REAL>();
	} catch (const std::exception&) {
		return false;
	}
	return ISFINITE(number);
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
		if (it == params.end() || !ReadFiniteNumber(*it, *field.value)) {
			error = String::FormatString("missing or invalid '%s'", field.name);
			return false;
		}
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
	if (ABS(scaleHeight - scale) > INTRINSICS_SCALE_TOLERANCE * scale) {
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

// Count the two-view triangulation inliers of the given matches under both axes hypotheses
// (first as imported, then flipped to the opposite convention): an inlier is a match
// triangulating in front of both cameras with a small reprojection error
std::pair<unsigned, unsigned> CountTriangulationInliers(const Image& img1, const Image& img2,
	const std::vector<DMatch>& matches)
{
	// build a two-image scene holding the hypothesis poses; the shared cameras are borrowed,
	// so the camera IDs are kept valid to stop the view destructor from deleting them
	ImageArr images(2);
	for (IIndex k = 0; k < 2; ++k) {
		const Image& src = k == 0 ? img1 : img2;
		Image& dst = images[k];
		dst.ID = k;
		dst.cameraID = k;
		dst.pCamera = src.pCamera;
		dst.keypoints = src.keypoints;
		dst.R = src.R;
		dst.C = src.C;
	}
	const auto Count = [&images, &matches]() {
		const unsigned numMatches = (unsigned)matches.size();
		const unsigned step = MAXF(1u, numMatches / CONVENTION_MAX_MATCHES_PER_PAIR);
		unsigned numInliers = 0;
		for (unsigned i = 0; i < numMatches; i += step) {
			const DMatch& match = matches[i];
			if (match.queryIdx >= images[0].keypoints.size() ||
				match.trainIdx >= images[1].keypoints.size())
				continue;
			Track track;
			track.observations.emplace_back(0u, match.queryIdx);
			track.observations.emplace_back(1u, match.trainIdx);
			if (TriangulateSkewLLS(track, images, CONVENTION_REPROJ_THRESHOLD, CONVENTION_MIN_ANGLE, 2) < 2)
				continue;
			if (images[0].Depth(track.position) <= 0 || images[1].Depth(track.position) <= 0)
				continue;
			++numInliers;
		}
		return numInliers;
	};
	std::pair<unsigned, unsigned> numInliers;
	numInliers.first = Count();
	images[0].R = RMatrix(CameraAxesFlipFor(img1) * img1.R);
	images[1].R = RMatrix(CameraAxesFlipFor(img2) * img2.R);
	numInliers.second = Count();
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


unsigned SFM::ExportPosesCSV(const String& fileName, const ImageArr& images)
{
	unsigned numValid = 0;
	std::ofstream os(fileName);
	if (!os.is_open())
		return numValid;

	os << "# columns: filename(stem, no ext), fx, fy, cx, cy, qx, qy, qz, qw (world->camera quaternion), Cx, Cy, Cz (camera center in world coords), score (0 invalid/unknown - 1 accurate)\n";
	os << "filename,fx,fy,cx,cy,qx,qy,qz,qw,Cx,Cy,Cz,score\n";
	os << std::setprecision(17);

	for (const Image& image : images) {
		const bool valid = image.IsValid();
		const float score = valid ? 1.f : 0.f;
		const std::string stem = Util::getFileName(image.fileName);

		double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
		Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
		double Cx = 0.0, Cy = 0.0, Cz = 0.0;
		if (valid) {
			// Intrinsics
			const KMatrix K = image.GetK();
			fx = K(0, 0);
			fy = K(1, 1);
			cx = K(0, 2);
			cy = K(1, 2);
			// Extrinsics
			const Eigen::Matrix3d R = image.R;
			q = Eigen::Quaterniond(R);
			q.normalize();
			Cx = image.C.x;
			Cy = image.C.y;
			Cz = image.C.z;
			++numValid;
		}

		os << stem << ','
		   << fx << ',' << fy << ',' << cx << ',' << cy << ','
		   << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
		   << Cx << ',' << Cy << ',' << Cz << ','
		   << score << '\n';
	}

	return numValid;
}

unsigned SFM::ImportPosesCSV(const String& fileName, ImageArr& images, PoseImportMode mode)
{
	unsigned numUpdated = 0;
	if (mode == PoseImportMode::NONE)
		return numUpdated;
	ASSERT(mode == PoseImportMode::POSES_INTRINSICS || mode == PoseImportMode::POSES ||
		mode == PoseImportMode::POSITIONS);
	std::ifstream is(fileName);
	if (!is.is_open())
		return numUpdated;

	// index the images by stem, case-insensitive, with ambiguous stems rejected
	std::unordered_map<String, IIndex> stemToIndex;
	stemToIndex.reserve(images.size());
	FOREACH(i, images)
		AddUniqueImageKey(stemToIndex, StemKey(images[i].fileName), i);

	String line;
	if (!std::getline(is, line))
		return numUpdated; // missing comment/header
	// Consume header line after comment if present
	if (!line.empty() && line[0] == '#' && !std::getline(is, line))
		return numUpdated; // missing header

	CLISTDEF2(String) fields;
	while (std::getline(is, line)) {
		if (line.empty())
			continue;
		// Parse CSV line
		Util::strSplit(line, ',', fields);
		if (fields.size() < 13)
			continue;
		// Find image by stem
		const auto it = stemToIndex.find(fields[0].ToLower());
		if (it == stemToIndex.end() || it->second == NO_ID)
			continue;
		Image& image = images[it->second];
		// Parse values
		try {
			const double fx = std::stod(fields[1]);
			const double fy = std::stod(fields[2]);
			const double cx = std::stod(fields[3]);
			const double cy = std::stod(fields[4]);
			const double qx = std::stod(fields[5]);
			const double qy = std::stod(fields[6]);
			const double qz = std::stod(fields[7]);
			const double qw = std::stod(fields[8]);
			const double Cx = std::stod(fields[9]);
			const double Cy = std::stod(fields[10]);
			const double Cz = std::stod(fields[11]);
			const float score = std::stof(fields[12]);
			if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
				!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw) ||
				!std::isfinite(Cx) || !std::isfinite(Cy) || !std::isfinite(Cz) || !std::isfinite(score))
				continue;

			if (score <= 0.f) {
				image.InvalidatePose();
				continue;
			}

			Eigen::Quaterniond q;
			if (mode != PoseImportMode::POSITIONS) {
				q = Eigen::Quaterniond(qw, qx, qy, qz);
				if (q.norm() <= std::numeric_limits<double>::epsilon())
					continue;
				q.normalize();
			}

			if (mode == PoseImportMode::POSES_INTRINSICS && image.HasCamera()) {
				// Set intrinsics, but only for camera models that expose them
				if (PinholeCamera* cam = dynamic_cast<PinholeCamera*>(image.pCamera)) {
					if (fx > 0.0 && fy > 0.0) {
						cam->SetIntrinsics((REAL)fx, (REAL)fy, (REAL)cx, (REAL)cy);
						cam->trustIntrinsics = true;
					} else {
						DEBUG("warning: image '%s' has invalid intrinsics in the poses file (fx %g, fy %g); importing the pose only",
							fields[0].c_str(), fx, fy);
					}
				}
			}
			if (mode != PoseImportMode::POSITIONS) {
				// Set rotation from quaternion
				image.R = q.toRotationMatrix();
			}
			// Set camera position
			image.C = CMatrix((REAL)Cx, (REAL)Cy, (REAL)Cz);
			++numUpdated;
		} catch (const std::exception&) {
			continue;
		}
	}

	return numUpdated;
}
/*----------------------------------------------------------------*/


String SFM::FramesConventionToString(FramesConvention convention)
{
	switch (convention) {
	case FramesConvention::ARKIT: return "arkit";
	case FramesConvention::OPENCV: return "opencv";
	default: return "auto";
	}
} // FramesConventionToString

bool SFM::FramesConventionFromString(const String& str, FramesConvention& convention)
{
	const String name(str.ToLower());
	if (name.empty() || name == "auto")
		convention = FramesConvention::AUTO;
	else if (name == "arkit")
		convention = FramesConvention::ARKIT;
	else if (name == "opencv")
		convention = FramesConvention::OPENCV;
	else
		return false;
	return true;
} // FramesConventionFromString
/*----------------------------------------------------------------*/


unsigned SFM::ImportFramesJSON(const String& fileName, Scene& scene, PoseImportMode mode,
	FramesConvention convention)
{
	if (mode == PoseImportMode::NONE)
		return 0;
	ASSERT(mode == PoseImportMode::POSES_INTRINSICS || mode == PoseImportMode::POSES ||
		mode == PoseImportMode::POSITIONS);
	if (convention == FramesConvention::AUTO) {
		VERBOSE("error: the camera-axes convention of '%s' can only be resolved after matching; "
			"import it as arkit or opencv", fileName.c_str());
		return 0;
	}
	ASSERT(convention == FramesConvention::ARKIT || convention == FramesConvention::OPENCV);
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
		AddUniqueImageKey(imageByName, NameKey(imgFileName), i);
		AddUniqueImageKey(imageByStem, StemKey(imgFileName), i);
	}

	const Matrix3x3 flip = CameraAxesFlip();
	unsigned numPosed = 0, numUnmatched = 0, numRejected = 0, numIntrinsics = 0, numWarnings = 0;
	bool intrinsicsRequestedButMissing = false;
	std::vector<uint8_t> importedImages(scene.images.size(), 0);
	for (size_t e = 0; e < data.size(); ++e) {
		const nlohmann::json& entry = data[e];
		const auto itName = entry.find("name"); // returns end() for any non-object entry
		if (itName == entry.end() || !itName->is_string()) {
			if (++numWarnings <= MAX_LOGGED_WARNINGS)
				VERBOSE("error: frame %u of '%s' has no 'name' string; skipped",
					(unsigned)e, fileName.c_str());
			++numRejected;
			continue;
		}
		const String name(itName->get<std::string>());
		// find the matching image: full name first, then stem
		IIndex imageID = NO_ID;
		bool ambiguousName = false;
		const auto itByName = imageByName.find(NameKey(name));
		if (itByName != imageByName.end()) {
			imageID = itByName->second;
			ambiguousName = imageID == NO_ID;
		} else {
			const auto itByStem = imageByStem.find(StemKey(name));
			if (itByStem != imageByStem.end()) {
				imageID = itByStem->second;
				ambiguousName = imageID == NO_ID;
			}
		}
		if (imageID == NO_ID) {
			if (++numWarnings <= MAX_LOGGED_WARNINGS) {
				if (ambiguousName)
					VERBOSE("error: frame '%s' of '%s' ambiguously matches multiple input images; skipped",
						name.c_str(), fileName.c_str());
				else
					VERBOSE("warning: frame '%s' of '%s' matches no input image; skipped",
						name.c_str(), fileName.c_str());
			}
			if (ambiguousName)
				++numRejected;
			else
				++numUnmatched;
			continue;
		}
		if (importedImages[imageID]) {
			if (++numWarnings <= MAX_LOGGED_WARNINGS)
				VERBOSE("error: more than one frame in '%s' matches image '%s'; duplicate skipped",
					fileName.c_str(), Util::getFileNameExt(scene.images[imageID].fileName).c_str());
			++numRejected;
			continue;
		}
		Image& img = scene.images[imageID];
		// read the 4x4 column-major camera-to-world transform
		const auto itTransform = entry.find("transform");
		if (itTransform == entry.end() || !itTransform->is_array() || itTransform->size() != 16) {
			if (++numWarnings <= MAX_LOGGED_WARNINGS)
				VERBOSE("error: frame '%s' of '%s' has no 'transform' array of 16 numbers; skipped",
					name.c_str(), fileName.c_str());
			++numRejected;
			continue;
		}
		REAL transform[16] = {};
		bool validNumbers = true;
		for (unsigned k = 0; k < 16; ++k) {
			const nlohmann::json& value = (*itTransform)[k];
			if (!ReadFiniteNumber(value, transform[k])) {
				validNumbers = false;
				break;
			}
		}
		if (!validNumbers) {
			if (++numWarnings <= MAX_LOGGED_WARNINGS)
				VERBOSE("error: frame '%s' of '%s' has a non-finite or non-numeric 'transform'; skipped",
					name.c_str(), fileName.c_str());
			++numRejected;
			continue;
		}
		// the last row of a column-major affine transform must be (0,0,0,1);
		// a row-major file would carry its translation here instead
		if (ABS(transform[3]) > AFFINE_ROW_TOLERANCE || ABS(transform[7]) > AFFINE_ROW_TOLERANCE ||
			ABS(transform[11]) > AFFINE_ROW_TOLERANCE || ABS(transform[15] - REAL(1)) > AFFINE_ROW_TOLERANCE)
		{
			if (++numWarnings <= MAX_LOGGED_WARNINGS)
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
			if (!IsRotationMatrix(rotationC2W, ORTHONORMALITY_TOLERANCE)) {
				if (++numWarnings <= MAX_LOGGED_WARNINGS)
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
		importedImages[imageID] = 1;
		++numPosed;
		// import the intrinsics only when asked for and available
		if (mode == PoseImportMode::POSES_INTRINSICS) {
			const auto itParams = entry.find("params");
			if (itParams == entry.end() || !itParams->is_object()) {
				intrinsicsRequestedButMissing = true;
			} else if (!img.HasCamera()) {
				if (++numWarnings <= MAX_LOGGED_WARNINGS)
					VERBOSE("warning: frame '%s' of '%s' has intrinsics, but the image has no camera; ignored",
						name.c_str(), fileName.c_str());
			} else {
				String error;
				if (ApplyImportedIntrinsics(*itParams, img, error)) {
					++numIntrinsics;
				} else if (++numWarnings <= MAX_LOGGED_WARNINGS) {
					VERBOSE("warning: frame '%s' of '%s' has unusable intrinsics (%s); using the EXIF ones",
						name.c_str(), fileName.c_str(), error.c_str());
				}
			}
		}
	}
	if (numWarnings > MAX_LOGGED_WARNINGS)
		VERBOSE("warning: %u more problems in '%s' not listed", numWarnings - MAX_LOGGED_WARNINGS, fileName.c_str());
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


bool SFM::ImportPoses(Scene& scene, const String& fileName, PoseImportMode mode, FramesConvention convention)
{
	const String ext = Util::getFileExt(fileName).ToLower();
	unsigned numPosesImported;
	if (ext == ".csv") {
		numPosesImported = ImportPosesCSV(fileName, scene.images, mode);
	} else if (ext == ".json") {
		// the convention can only be resolved after matching, so AUTO imports the poses
		// with the ARKit convention and ResolveFramesConvention() flips them if needed
		numPosesImported = ImportFramesJSON(fileName, scene, mode,
			convention == FramesConvention::AUTO ? FramesConvention::ARKIT : convention);
	} else {
		VERBOSE("error: unsupported poses file '%s' (supported extensions: .csv, .json)", fileName.c_str());
		return false;
	}
	if (numPosesImported == 0) {
		VERBOSE("error: failed to import poses from file '%s'", fileName.c_str());
		return false;
	}
	DEBUG("Imported poses for %u images from file '%s'", numPosesImported, fileName.c_str());
	return true;
} // ImportPoses
/*----------------------------------------------------------------*/


void SFM::FlipFramesConvention(Scene& scene)
{
	unsigned numFlipped = 0;
	for (Image& img : scene.images) {
		if (!img.HasPose())
			continue;
		// the flip differs for EXIF-rotated images, whose stored pose composes an in-plane
		// rotation with the file pose (see CameraAxesFlipFor)
		img.R = RMatrix(CameraAxesFlipFor(img) * img.R);
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
		// flipping both images conjugates their relative rotation by the per-image axes
		// flips (both symmetric): R2f * R1f^T = F2 * R2 * R1^T * F1
		const Matrix3x3 relativeFlipped(Matrix3x3(CameraAxesFlipFor(img2) * relative) * CameraAxesFlipFor(img1));
		errorsApplied.emplace_back(ComputeAngleSO3<REAL>(relative, verified));
		errorsFlipped.emplace_back(ComputeAngleSO3<REAL>(relativeFlipped, verified));
	}
	const unsigned numVerifiedPairs = (unsigned)errorsApplied.size();
	if (numVerifiedPairs >= CONVENTION_MIN_VERIFIED_PAIRS) {
		const REAL medianApplied = errorsApplied.GetMedian();
		const REAL medianFlipped = errorsFlipped.GetMedian();
		DEBUG_EXTRA("Frames convention: %u verified pairs, median relative rotation error %.2f deg as %s, %.2f deg as %s",
			numVerifiedPairs, R2D(medianApplied), FramesConventionToString(appliedConvention).c_str(),
			R2D(medianFlipped), FramesConventionToString(flippedConvention).c_str());
		if (medianApplied * CONVENTION_MARGIN_RATIO < medianFlipped) {
			VERBOSE("Detected %s camera-axes convention (median relative rotation error %.2f deg vs %.2f deg)",
				FramesConventionToString(appliedConvention).c_str(), R2D(medianApplied), R2D(medianFlipped));
			return appliedConvention;
		}
		if (medianFlipped * CONVENTION_MARGIN_RATIO < medianApplied) {
			VERBOSE("Detected %s camera-axes convention (median relative rotation error %.2f deg vs %.2f deg)",
				FramesConventionToString(flippedConvention).c_str(), R2D(medianFlipped), R2D(medianApplied));
			return flippedConvention;
		}
		// the rotation signal loses its discriminative power on low-rotation captures
		// (conjugating a small rotation barely changes it), so an ambiguous margin falls
		// through to the cheirality-based triangulation test instead of giving up
		DEBUG("Frames convention: ambiguous rotation signal (median relative rotation error "
			"%.2f deg as %s vs %.2f deg as %s over %u verified pairs); falling back to two-view triangulation",
			R2D(medianApplied), FramesConventionToString(appliedConvention).c_str(),
			R2D(medianFlipped), FramesConventionToString(flippedConvention).c_str(), numVerifiedPairs);
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
	const unsigned numPairs = MINF((unsigned)candidates.size(), CONVENTION_MAX_TRIANGULATED_PAIRS);
	std::partial_sort(candidates.begin(), candidates.begin() + numPairs, candidates.end(),
		[](const PairScore& a, const PairScore& b) { return a.score > b.score; });
	unsigned inliersApplied = 0, inliersFlipped = 0;
	for (unsigned i = 0; i < numPairs; ++i) {
		const ImagePair& pair = scene.pairs[candidates[i].idx];
		const auto [applied, flipped] = CountTriangulationInliers(
			scene.images[pair.ID1], scene.images[pair.ID2], pair.matches);
		inliersApplied += applied;
		inliersFlipped += flipped;
	}
	DEBUG_EXTRA("Frames convention: %u triangulated pairs, %u inliers as %s, %u inliers as %s",
		numPairs, inliersApplied, FramesConventionToString(appliedConvention).c_str(),
		inliersFlipped, FramesConventionToString(flippedConvention).c_str());
	if (inliersApplied >= CONVENTION_MIN_TRIANGULATED_INLIERS &&
		(REAL)inliersApplied > CONVENTION_MARGIN_RATIO * inliersFlipped)
	{
		VERBOSE("Detected %s camera-axes convention (%u vs %u two-view triangulation inliers)",
			FramesConventionToString(appliedConvention).c_str(), inliersApplied, inliersFlipped);
		return appliedConvention;
	}
	if (inliersFlipped >= CONVENTION_MIN_TRIANGULATED_INLIERS &&
		(REAL)inliersFlipped > CONVENTION_MARGIN_RATIO * inliersApplied)
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


bool SFM::ResolveFramesConvention(Scene& scene, FramesConvention configuredConvention, const String& importPosesFile)
{
	if (configuredConvention != FramesConvention::AUTO ||
		Util::getFileExt(importPosesFile).ToLower() != ".json")
		return true; // convention explicit, or not a frames.json import
	constexpr FramesConvention appliedConvention = FramesConvention::ARKIT; // what ImportPoses applied
	const FramesConvention detectedConvention = DetectFramesConvention(scene, appliedConvention);
	if (detectedConvention == FramesConvention::AUTO) {
		VERBOSE("error: could not decide the camera-axes convention of '%s' from the matched pairs; "
			"re-run passing the convention explicitly", importPosesFile.c_str());
		return false;
	}
	VERBOSE("Known poses use the %s camera-axes convention%s",
		FramesConventionToString(detectedConvention).c_str(),
		detectedConvention == appliedConvention ? "" : " (flipping the imported poses)");
	if (detectedConvention != appliedConvention)
		FlipFramesConvention(scene);
	return true;
} // ResolveFramesConvention
/*----------------------------------------------------------------*/
