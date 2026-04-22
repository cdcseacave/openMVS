////////////////////////////////////////////////////////////////////
// Image.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Image.h"
#include <TinyEXIF.h>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace {
// EXIFStream wrapper for SEACAVE::IOStream to enable TinyEXIF stream-based parsing
class IOStreamEXIFWrapper : public TinyEXIF::EXIFStream {
public:
	IOStreamEXIFWrapper(SEACAVE::IOSTREAMPTR& stream) : pStream(stream), buffer(4096), pos(0) {
		// Reset to beginning for EXIF parsing
		pStream->getInputStream()->setPos(0);
	}
	bool IsValid() const override {
		return pStream != nullptr;
	}
	const uint8_t* GetBuffer(unsigned desiredLength) override {
		if (desiredLength == 0)
			return NULL;
		// Ensure buffer is large enough
		if (buffer.size() < desiredLength)
			buffer.resize(desiredLength);
		// Read from stream
		const size_t bytesRead = pStream->getInputStream()->read(buffer.data(), desiredLength);
		if (bytesRead == STREAM_ERROR || bytesRead == 0)
			return NULL;
		pos += bytesRead;
		// If we read less than requested, we're at EOF or error
		if (bytesRead < desiredLength)
			return NULL;
		return buffer.data();
	}
	bool SkipBuffer(unsigned desiredLength) override {
		if (desiredLength == 0)
			return false;
		pos += desiredLength;
		return pStream->getInputStream()->setPos(pos);
	}
private:
	SEACAVE::IOSTREAMPTR pStream;
	std::vector<uint8_t> buffer;
	size_f_t pos;
};
} // namespace
/*----------------------------------------------------------------*/


bool Image::LoadPixels(bool gray)
{
	if (fileName.empty()) {
		VERBOSE("Image::LoadPixels: empty file name");
		return false;
	}
	if (!LoadImage(fileName, pixels, gray ? 1 : -1)) {
		VERBOSE("Image::LoadPixels: failed to load image '%s'", fileName.c_str());
		return false;
	}
	ASSERT(!pixels.empty());
	// Rotate 90 degrees clockwise if needed, so width > height
	View::metadata.rotated = pixels.cols < pixels.rows;
	ToWorkingOrientation(pixels);
	return true;
}

Image8U3 Image::GetImage8U3() const
{
	// Fast path: already BGR/8U, return a shared view (no copy).
	if (pixels.channels() == 3 && pixels.depth() == CV_8U)
		return Image8U3(pixels);
	// Else convert: grayscale -> BGR, BGRA -> BGR, float/etc -> CV_8UC3.
	cv::Mat converted;
	if (pixels.channels() == 1)
		cv::cvtColor(pixels, converted, cv::COLOR_GRAY2BGR);
	else if (pixels.channels() == 4)
		cv::cvtColor(pixels, converted, cv::COLOR_BGRA2BGR);
	else
		pixels.convertTo(converted, CV_8UC3);
	return Image8U3(converted);
}

bool Image::SavePixels() const
{
	if (!HasPixels()) {
		VERBOSE("Image::SavePixels: no pixels to save");
		return false;
	}
	// Determine format from extension, default to JXL
	String savePath = fileName;
	if (Util::getFileExt(savePath).empty())
		savePath += ".jxl";
	// Use OpenCV to save the image
	if (!SaveImage(pixels, savePath)) {
		VERBOSE("Image::SavePixels: failed to save image to '%s'", savePath.c_str());
		return false;
	}
	return true;
}

bool Image::LoadMetadata(float defaultFocalRatio)
{
	if (fileName.empty()) {
		VERBOSE("Image::LoadMetadata: empty file name");
		return false;
	}

	// Use CImage to read header for image dimensions only (no pixel decoding)
	IMAGEPTR pImage(CImage::Create(fileName, CImage::READ));
	if (!pImage) {
		VERBOSE("Image::LoadMetadata: failed to open '%s'", fileName.c_str());
		return false;
	}
	if (!pImage->ReadHeader()) {
		VERBOSE("Image::LoadMetadata: failed to read image header '%s'", fileName.c_str());
		return false;
	}

	// Extract image dimensions from header
	const int ow = pImage->GetWidth();
	const int oh = pImage->GetHeight();
	ASSERT(ow > 0 && oh > 0);
	// Determine if rotation is needed (width < height)
	// Note: CImage does not handle the EXIF orientation tag, in contrast to the default behaviour of cv::imread;
	// EXIF orientation is handled later when loading EXIF metadata in order to match cv::imread behaviour
	View::metadata.rotated = ow < oh;

	// Working dimensions after rotation
	const int w = View::metadata.rotated ? oh : ow;
	const int h = View::metadata.rotated ? ow : oh;
	REAL sensorWmm = 0.0, sensorHmm = 0.0;
	REAL fx = 0, fy = 0; // to be computed
	// Principal point defaults to image center under integer=pixel-center convention
	const REAL cx = (w - 1) * 0.5, cy = (h - 1) * 0.5;
	bool isSpherical = false;
	bool trustIntrinsics = false;

	// Parse EXIF from same buffer
	TinyEXIF::EXIFInfo exif;
	IOStreamEXIFWrapper exifStream(pImage->GetStream());
	if (exif.parseFrom(exifStream) == TinyEXIF::PARSE_SUCCESS) {
		// Basic camera/lens metadata
		metadata.dateTimeOriginal = exif.DateTimeOriginal;
		metadata.exposureTime = exif.ExposureTime;
		metadata.ISO = exif.ISOSpeedRatings;
		metadata.orientation = exif.Orientation;
		// Determine if the image will be swapped by cv::imread (orientations 5, 6, 7, 8)
		// and adjust roated flag accordingly to match it
		if (metadata.orientation >= 5 && metadata.orientation <= 8)
			View::metadata.rotated = oh < ow;
		// Geo + device orientation (WGS84) into View::metadata; flags set accordingly
		if (exif.GeoLocation.hasLatLon()) {
			View::metadata.latitude = exif.GeoLocation.Latitude;
			View::metadata.longitude = exif.GeoLocation.Longitude;
		}
		if (exif.GeoLocation.hasAltitude())
			View::metadata.altitude = exif.GeoLocation.Altitude;
		if (exif.GeoLocation.hasAccuracy()) {
			View::metadata.positionAccuracy = exif.GeoLocation.AccuracyXY;
			View::metadata.positionAccuracyZ = exif.GeoLocation.AccuracyZ;
		}
		if (exif.GeoLocation.hasOrientation()) {
			View::metadata.yawDeg = exif.GeoLocation.YawDegree;
			View::metadata.pitchDeg = exif.GeoLocation.PitchDegree;
			View::metadata.rollDeg = exif.GeoLocation.RollDegree;
		}
		// Projection type: 2 = equirectangular/spherical
		isSpherical = (exif.ProjectionType == 2);
		// Focal estimation priority
		// F1: FocalLength (mm) * FocalPlaneResolution (px per unit)
		double mmPerUnit = 0.0;
		switch (exif.LensInfo.FocalPlaneResolutionUnit) {
		case 2: mmPerUnit = 25.4; break; // inches to mm
		case 3: mmPerUnit = 10.0; break; // cm to mm
		case 4: mmPerUnit = 1.0; break;  // mm to mm
		case 5: mmPerUnit = 0.1; break;  // um to mm
		}
		bool setFromFocalAndSensor = false;
		if (exif.FocalLength > 0.0 && mmPerUnit > 0.0 && (exif.LensInfo.FocalPlaneXResolution > 0.0 || exif.LensInfo.FocalPlaneYResolution > 0.0)) {
			int ew = exif.ImageWidth > 0 ? (int)exif.ImageWidth : w;
			int eh = exif.ImageHeight > 0 ? (int)exif.ImageHeight : h;
			if (exif.LensInfo.FocalPlaneXResolution > 0.0) {
				fx = fy = (REAL)(exif.FocalLength * exif.LensInfo.FocalPlaneXResolution / mmPerUnit);
				sensorWmm = (REAL)ew / exif.LensInfo.FocalPlaneXResolution * mmPerUnit; // px per unit -> sensor size in mm
			}
			if (exif.LensInfo.FocalPlaneYResolution > 0.0) {
				fy = (REAL)(exif.FocalLength * exif.LensInfo.FocalPlaneYResolution / mmPerUnit);
				sensorHmm = (REAL)eh / exif.LensInfo.FocalPlaneYResolution * mmPerUnit; // px per unit -> sensor size in mm
			}
			if (fx > 0.f && fy > 0.f) {
				// Swap fx/fy if resolution not in landscape orientation
				if (ew < eh) {
					std::swap(ew, eh);
					std::swap(fx, fy);
					std::swap(sensorWmm, sensorHmm);
				}
				// Scale focal if image size differs from EXIF size
				if (ew != w)
					fx *= (REAL)w / (REAL)ew;
				if (eh != h)
					fy *= (REAL)h / (REAL)eh;
				setFromFocalAndSensor = true;
				trustIntrinsics = true;
			}
		}
		// F2: 35mm equivalent
		if (!setFromFocalAndSensor && exif.LensInfo.FocalLengthIn35mm > 0.0) {
			// according to CIPA guidelines, 35 mm equivalent focal length is to be calculated like this:
			// focal length in 35 mm camera = focal length of the lens of the DSC *
			//   (Diagonal distance of image area in the 35 mm camera (43.27 mm) /
			//    Diagonal distance of image area on the image sensor of the DSC)
			// see: https://en.wikipedia.org/wiki/35_mm_equivalent_focal_length
			const REAL diagonal = SQRT(SQUARE((REAL)w) + SQUARE((REAL)h));
			fx = fy = diagonal * exif.LensInfo.FocalLengthIn35mm / REAL(43.27);
			trustIntrinsics = true;
		}
		// F3: Calibration focal in pixels (if present)
		if ((fx <= 0.f || fy <= 0.f) && exif.Calibration.FocalLength > 0.0) {
			fx = fy = (REAL)exif.Calibration.FocalLength;
			trustIntrinsics = true;
		}
	}
	// F4: fallback
	if (fx <= 0.f || fy <= 0.f) {
		fx = fy = defaultFocalRatio * (REAL)MAXF(w, h);
	}

	// Instantiate per-image camera
	if (isSpherical) {
		if (w != 2 * h) {
			VERBOSE("warning: image '%s' is marked spherical but has %dx%d; equirectangular input requires width == 2 * height",
				Util::getFileName(fileName).c_str(), w, h);
		}
		SphericalCamera* cam = new SphericalCamera(cv::Size(w, h));
		pCamera = cam;
	} else {
		PinholeCamera* cam = new PinholeCamera(cv::Size(w, h), fx, fy, cx, cy);
		// If we had sensor size, store it
		if (sensorWmm > 0.0 || sensorHmm > 0.0)
			cam->SetSensorSize((REAL)sensorWmm, (REAL)sensorHmm);
		cam->trustIntrinsics = trustIntrinsics;
		pCamera = cam;
	}
	// Camera metadata from image EXIF
	pCamera->SetName(exif.Make);
	pCamera->SetModel(exif.Model);
	cameraID = NO_ID;
	DEBUG_ULTIMATE("Load metadata for image % 4u ('%s'): size %dx%d%s, focal-length %.2f%s%s, camera '%s'",
		ID, Util::getFileName(fileName).c_str(), w, h, View::metadata.rotated ? " (rotated)" : "",
		fx, fy!=fx ? String::FormatString("x%.2f", fy).c_str() : "", pCamera->TrustIntrinsics() ? "" : "*",
		exif.Make.empty() && exif.Model.empty() ? "unknown" : (exif.Make + " - " + exif.Model).c_str());
	return true;
}

UnsignedArr Image::SelectTopKeypoints(unsigned maxKeypoints) const
{
	// Select top keypoints using both spatial distribution (3x3 grid) and keypoint response and size quality.
	// Round-robin selection algorithm:
	// instead of taking all top N keypoints from each cell upfront, the function
	//  - first sorts keypoints in each cell by response * size (quality)
	//  - then iterates round-robin across all 9 cells
	//  - takes one keypoint at a time from each cell's sorted list
	//  - continues until reaching maxKeypoints or exhausting all cells
	// This ensures much better spatial distribution while still prioritizing quality within each cell.
	UnsignedArr indices;
	if (keypoints.empty())
		return indices;
	const unsigned numKeypoints = (unsigned)keypoints.size();
	if (numKeypoints <= maxKeypoints) {
		// Return all indices if we have fewer keypoints than requested
		indices.resize(numKeypoints);
		std::iota(indices.begin(), indices.end(), 0u);
		return indices;
	}

	// Use 3x3 grid similar to ExtractFeatures
	const int width = GetWidth();
	const int height = GetHeight();
	const float cellWidth = width / 3.f;
	const float cellHeight = height / 3.f;

	// Assign each keypoint to a grid cell
	std::vector<UnsignedArr> cellKeypoints(9);
	for (unsigned i = 0; i < numKeypoints; ++i) {
		const cv::Point2f& pt = keypoints[i].pt;
		const int col = MINF(2, (int)(pt.x / cellWidth));
		const int row = MINF(2, (int)(pt.y / cellHeight));
		const int cellIdx = row * 3 + col;
		cellKeypoints[cellIdx].push_back(i);
	}

	// Sort each cell's keypoints by response * size (descending)
	for (int cellIdx = 0; cellIdx < 9; ++cellIdx) {
		auto& cellIndices = cellKeypoints[cellIdx];
		if (cellIndices.empty())
			continue;
		std::sort(cellIndices.begin(), cellIndices.end(),
			[this](int a, int b) {
				return ComputeKeypointWeight(keypoints[a]) > ComputeKeypointWeight(keypoints[b]);
			});
	}

	// Round-robin selection: iterate over cells, taking one descriptor at a time from each;
	// continue until reaching maxKeypoints (guranteed to terminate since total keypoints > maxKeypoints)
	indices.reserve(maxKeypoints);
	UnsignedArr cellOffsets(9); // current index in each cell
	cellOffsets.Memset(0);
	for (unsigned currentCell = 0; indices.size() < maxKeypoints; currentCell = (currentCell + 1) % 9) {
		unsigned& cellIdx = cellOffsets[currentCell];
		const auto& cellIndices = cellKeypoints[currentCell];
		if (cellIdx < cellIndices.size()) {
			indices.push_back(cellIndices[cellIdx]);
			++cellIdx;
		}
	}
	return indices;
}

float Image::ComputeKeypointWeight(const cv::KeyPoint& kp, float minResponse)
{
	if (kp.response < minResponse)
		return 0.f;
	// Weight based on response (normalized)
	float responseWeight = (float)(kp.response / (kp.response + 0.03f));
	// Weight based on size: linear ramp from 0.5 (size=2px) to 1.5 (size=20px)
	float sizeWeight = 0.5f + (CLAMP(kp.size, 2.f, 20.f) - 2.f) / (20.f - 2.f);
	// Combined weight
	return responseWeight * sizeWeight;
}

float Image::ComputeKeypointPrecision(const cv::KeyPoint& kp, float minResponse)
{
	if (kp.response < minResponse)
		return 0.0f;
	// Response Component (Signal Strength / Reliability)
	// Saturated normalization: response / (response + 0.03) (heuristic based on typical Hessian responses)
	// This ensures we don't give high weight to weak features even if they are small.
	float responseWeight = kp.response / (kp.response + 0.03f);
	// Precision Component (Inverse Variance)
	// Uncertainty sigma is proportional to scale (size).
	// Weight W ~ 1 / sigma^2 ~ 1 / size^2.
	// Reference scale: size = 2px -> weight factor = 1.0.
	// For size = 20px -> weight factor = (2/20)^2 = 0.01.
	// This prioritizes small, sharp features (high precision) over large blobs (structurally stable but imprecise).
	constexpr float baseSize = 2.f;
	float sizeWeight = SQUARE(baseSize / MAXF(kp.size, 1.f));
	// Combined weight
	return responseWeight * sizeWeight;
}
/*----------------------------------------------------------------*/


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

unsigned SFM::ImportPosesCSV(const String& fileName, ImageArr& images, unsigned mode)
{
	unsigned numUpdated = 0;
	std::ifstream is(fileName);
	if (!is.is_open())
		return numUpdated;

	std::unordered_map<String, IIndex> stemToIndex;
	stemToIndex.reserve(images.size());
	FOREACH(i, images)
		stemToIndex.emplace(Util::getFileName(images[i].fileName), i);

	String line;
	if (!std::getline(is, line))
		return numUpdated; // missing comment/header
	// Consume header line after comment if present
	if (!line.empty() && line[0] == '#' && !std::getline(is, line))
		return numUpdated; // missing header

	while (std::getline(is, line)) {
		if (line.empty())
			continue;
		// Parse CSV line
		std::vector<String> fields;
		fields.reserve(13);
		std::stringstream ss(line);
		String token;
		while (std::getline(ss, token, ','))
			fields.push_back(token);
		if (fields.size() < 13)
			continue;
		// Find image by stem
		const auto it = stemToIndex.find(fields[0]);
		if (it == stemToIndex.end())
			continue;
		Image& image = images[it->second];
		// Parse values
		try {
			double fx = std::stod(fields[1]);
			double fy = std::stod(fields[2]);
			double cx = std::stod(fields[3]);
			double cy = std::stod(fields[4]);
			double qx = std::stod(fields[5]);
			double qy = std::stod(fields[6]);
			double qz = std::stod(fields[7]);
			double qw = std::stod(fields[8]);
			double Cx = std::stod(fields[9]);
			double Cy = std::stod(fields[10]);
			double Cz = std::stod(fields[11]);
			float score = std::stof(fields[12]);

			if (score <= 0.f) {
				image.InvalidatePose();
				continue;
			}

			if (image.HasCamera() && mode == 0 /*all*/) {
				// Set intrinsics
				if (PinholeCamera* cam = dynamic_cast<PinholeCamera*>(image.pCamera))
					cam->SetIntrinsics((REAL)fx, (REAL)fy, (REAL)cx, (REAL)cy);
			}
			if (mode == 1 /*extrinsics only*/) {
				// Set rotation from quaternion
				Eigen::Quaterniond q(qw, qx, qy, qz);
				if (q.norm() == 0.0)
					continue;
				q.normalize();
				image.R = q.toRotationMatrix();
			}
			if (mode == 1 /*extrinsics only*/ || mode == 2 /*positions only*/) {
				// Set camera position
				image.C = CMatrix((REAL)Cx, (REAL)Cy, (REAL)Cz);
			}
			++numUpdated;
		} catch (const std::exception&) {
			continue;
		}
	}

	return numUpdated;
}


float SFM::EstimateImageSharpness(const cv::Mat& pixels)
{
	if (pixels.empty())
		return 0.f;
	// Convert to grayscale
	cv::Mat gray;
	if (pixels.channels() == 1) {
		gray = pixels;
	} else if (pixels.channels() == 3) {
		cv::cvtColor(pixels, gray, cv::COLOR_BGR2GRAY);
	} else if (pixels.channels() == 4) {
		cv::cvtColor(pixels, gray, cv::COLOR_BGRA2GRAY);
	} else {
		cv::extractChannel(pixels, gray, 0);
	}
	// Normalize to float [0,1]
	cv::Mat gray32;
	switch (gray.depth()) {
	case CV_8U:
		gray.convertTo(gray32, CV_32F, 1.f/255.f);
		break;
	case CV_16U:
		gray.convertTo(gray32, CV_32F, 1.f/65535.f);
		break;
	case CV_32F:
		gray32 = gray;
		break;
	case CV_64F:
	default:
		gray.convertTo(gray32, CV_32F);
	}
	// Focus measure: multi-scale variance of Laplacian (robust, noise-suppressed)
	auto FocusAtScale = [](const cv::Mat& src) {
		cv::Mat lap;
		cv::Laplacian(src, lap, CV_32F, 3);
		cv::Scalar mu, sigma;
		cv::meanStdDev(lap, mu, sigma);
		return SQUARE(sigma.val[0]);
	};
	// Multi-scale: compute focus at multiple pyramid levels and average
	double focusAccum = 0;
	int usedLevels = 0;
	while (true) {
		focusAccum += FocusAtScale(gray32);
		if (++usedLevels >= 3 || gray32.cols < 64 || gray32.rows < 64)
			break;
		cv::pyrDown(gray32, gray32);
	}
	return static_cast<float>(focusAccum * 10.0 / (double)usedLevels);
}
/*----------------------------------------------------------------*/
