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

// Fallback pixel loader for formats OpenCV's imread cannot decode (e.g. HEIC): routes
// the file through the CImage reader chain instead (mirrors MVS::Image::ReadImage).
// Destination format is chosen to match what cv::imread would have produced (BGR/8U),
// since every downstream consumer of Image::pixels assumes imread's channel order.
// PF_* names list channels most- to least-significant bit (see PIXELFORMAT in IO/Image.h),
// so on little-endian PF_R8G8B8 is B,G,R in memory, i.e. imread's order; same request as
// MVS::Image::ReadImage. The readers themselves declare PF_B8G8R8, which is R,G,B in memory.
bool LoadPixelsViaCImage(const String& fileName, cv::Mat& pixels, bool gray)
{
	IMAGEPTR pImage(CImage::Create(fileName, CImage::READ));
	if (!pImage || !pImage->ReadHeader())
		return false;
	// Decode into a local buffer and publish it into 'pixels' only once fully populated:
	// callers treat a non-empty 'pixels' as "fully loaded" (Image::HasPixels()), and pixel
	// loading also runs as a detached prefetch task concurrently with the consumer, so
	// filling 'pixels' in place would expose a partially-decoded image. This mirrors the
	// cv::imread path, which likewise assigns the destination only after decoding.
	cv::Mat decoded((int)pImage->GetHeight(), (int)pImage->GetWidth(), gray ? CV_8UC1 : CV_8UC3);
	if (!pImage->ReadData(decoded.data, gray ? PF_GRAY8 : PF_R8G8B8, gray ? 1 : 3, (CImage::Size)decoded.step))
		return false;
	pixels = decoded;
	return true;
}
} // namespace
/*----------------------------------------------------------------*/


bool Image::LoadPixels(bool gray)
{
	if (fileName.empty()) {
		VERBOSE("Image::LoadPixels: empty file name");
		return false;
	}
	if (!LoadImage(fileName, pixels, gray ? 1 : -1)) {
		// fallback: route formats OpenCV cannot decode (e.g. HEIC) through the CImage readers
		if (!LoadPixelsViaCImage(fileName, pixels, gray)) {
			VERBOSE("Image::LoadPixels: failed to load image '%s'", fileName.c_str());
			return false;
		}
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

	// Parse EXIF metadata: prefer the container-native metadata blob (e.g. HEIF's "Exif"
	// item, exposed by GetMetadataEXIF()) and fall back to the classic stream-based scan
	// (JPEG/TIFF-style APP1 segment) used by every other format
	TinyEXIF::EXIFInfo exif;
	bool parsed = false, containerOriented = false;
	std::vector<uint8_t> blob;
	if (pImage->GetMetadataEXIF(blob)) {
		// blob is guaranteed by the GetMetadataEXIF contract to start with "Exif\0\0"
		if (exif.parseFromEXIFSegment(blob.data(), (unsigned)blob.size()) == TinyEXIF::PARSE_SUCCESS) {
			parsed = true;
			containerOriented = true; // decoder already applied the container irot/imir transforms
		}
	}
	if (!parsed) {
		// blob missing, or present but failed to parse: fall back to the raw stream scan;
		// clear() first so a failed blob-parse attempt cannot leave stale fields behind
		exif.clear();
		IOStreamEXIFWrapper exifStream(pImage->GetStream());
		parsed = exif.parseFrom(exifStream) == TinyEXIF::PARSE_SUCCESS;
	}
	if (parsed) {
		if (containerOriented) {
			// HEIF stores rotation in container-level irot/imir boxes, which libheif applies
			// during decode and reflects in the header/pixel dimensions already used above.
			// Many writers ALSO stamp the EXIF Orientation tag for the same rotation:
			// honoring it here would rotate a second time and desync View::metadata.rotated
			// from the actual pixel layout -- which feeds the known-poses rotated-image flip
			// (diag(-1,1,-1)), i.e. a silent pose-import breaker, not a cosmetic bug -- so
			// normalize it away whenever the blob parse (not just the stream fallback) succeeded.
			exif.Orientation = 1;
		}
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
