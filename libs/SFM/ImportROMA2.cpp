////////////////////////////////////////////////////////////////////
// ImportROMA2.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImportROMA2.h"
#include "Scene.h"
#include "MatchGeometric.h"
#include "InterfaceMVS.h"
#include <TinyNPY.h>
#include "../Common/ListFIFO.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

namespace {

inline Point2f CoordFromTo(const Point2f& coord, const cv::Size& sizeA, const cv::Size& sizeB) {
	return Point2f(
		coord.x * (float)(sizeB.width  - 1) / (float)(sizeA.width  - 1),
		coord.y * (float)(sizeB.height - 1) / (float)(sizeA.height - 1)
	);
}

inline Point2f DenormCoord(const Point2f& normCoord, const cv::Size& size) {
	// adjust for align_corners=False mapping (default PyTorch grid_sample),
	// which adds the 0.5-pixel offset (different from OpenMVS integer = pixel center convention)
	return Point2f(
		0.5f * (normCoord.x + 1.f) * (float)size.width - 0.5f,
		0.5f * (normCoord.y + 1.f) * (float)size.height - 0.5f
	);
}

// Rotate warp/overlap maps when reference image was rotated to landscape
void RotateMapsForReference(const Image& img, Image32F2& warp, Image32F& overlap, Image32F& precision) {
	if (!img.IsRotated())
		return;
	img.ToWorkingOrientation(warp);
	if (!overlap.empty())
		img.ToWorkingOrientation(overlap);
	if (!precision.empty())
		img.ToWorkingOrientation(precision);
	// Rotate warp target coordinates when target image was rotated to landscape;
	// since the warp values are centered (and normalized), the rotation is directly applied,
	// no translation needed
	for (int y = 0; y < warp.rows; ++y) {
		for (int x = 0; x < warp.cols; ++x) {
			Point2f& v = warp(y, x);
			// Apply 90° CCW rotation in normalized coordinates: (u', v') = (-v, u)
			std::swap(v.x, v.y);
			v.x = -v.x;
		}
	}
}

// Erode confidence map if requested (helps remove outliers near edges)
void ErodeConfidenceMap(Image32F& imgConfidence, int erodeBorder, float minConfidence, float minErodeConfidence) {
	ASSERT(erodeBorder > 0);
	// Create binary mask: 0 for invalid pixels (0.f values), 1 for valid
	Image8U mask(imgConfidence >= minConfidence);
	// Compute distance from each pixel to nearest 0 pixel
	Image32F distMap;
	cv::distanceTransform(mask, distMap, cv::DIST_L2, cv::DIST_MASK_PRECISE);
	// Zero out pixels closer than erodeBorder to invalid pixels, if confidence is below threshold
	for (int y = 0; y < imgConfidence.rows; ++y)
		for (int x = 0; x < imgConfidence.cols; ++x)
			if (distMap(y, x) < erodeBorder && imgConfidence(y, x) < minErodeConfidence)
				imgConfidence(y, x) = 0.f;
}


// Structure to hold NPZ pair data (metadata + large matrices, cached)
struct NPZPairData {
	// Metadata (lightweight, always loaded)
	String fileName;
	cv::Size warpSize;
	String suffix; // "AB" or "BA" to select which arrays to load from NPZ

	// Heavy data (loaded on-demand when warp.empty())
	Image32F2 warp;       // Point2f per pixel
	Image32F overlap;     // float per pixel
	Image32F precisionWeight; // float per pixel (currently unused)

	bool HasData() const {
		return !warp.empty();
	}
	void ClearHeavyData() {
		warp.release();
		overlap.release();
		precisionWeight.release();
	}
};

// LRU cache for warp data with configurable max memory
class NPZPairCache {
public:
	NPZPairCache(Scene& scene_, const ROMA2Config& config_)
		: scene(scene_), config(config_) {}
	static constexpr size_t MAX_CACHED_PAIRS = 10;  // Keep at least 5-10 warp matrices in memory

	// Get pair data from cache (might contain only the header if not yet loaded)
	NPZPairData& GetPairData(uint64_t pairKey) {
		return cache.at(pairKey);
	}

	// Get or load pair data from cache (manages eviction automatically)
	NPZPairData& GetOrLoadPairData(uint64_t pairKey) {
		// If already cached, move to front (most recently used)
		NPZPairData& pairData = cache.at(pairKey);
		const bool contains = fifo.Contains(pairKey);
		fifo.Put(pairKey);
		if (contains) {
			// Warp data is already loaded
			ASSERT(pairData.HasData());
			return pairData;
		}

		// Evict oldest entries if cache is full
		while (fifo.Size() >= MAX_CACHED_PAIRS) {
			const uint64_t oldestKey = fifo.Pop();
			cache.at(oldestKey).ClearHeavyData();
		}

		// Load warp data from NPZ file
		ASSERT(!pairData.HasData());
		LoadPairDataFromNPZ(PairIdx(pairKey), pairData);
		return pairData;
	}

	// Add metadata entry to cache (without loading matrices yet)
	void AddMetadata(uint64_t pairKey, const String& fileName, const cv::Size& warpSize, const String& suffix) {
		NPZPairData& pairData = cache[pairKey];
		pairData.fileName = fileName;
		pairData.warpSize = warpSize;
		pairData.suffix = suffix;
	}

private:
	// Load warp/overlap/precisionWeight from NPZ file into existing cache entry
	void LoadPairDataFromNPZ(const PairIdx& pairIdx, NPZPairData& pairData) {
		NpyArray::npz_t arrays;
		if (const char* err = NpyArray::LoadNPZ(pairData.fileName, arrays)) {
			DEBUG("error: failed to load '%s' (%s)", pairData.fileName.c_str(), err);
			return;
		}

		// Load warp matrix using appropriate suffix (_AB or _BA)
		const NpyArray& warp = arrays.at(String("warp_") + pairData.suffix);
		pairData.warp = Image32F2(pairData.warpSize, const_cast<Point2f*>(warp.Data<Point2f>())).clone();

		// Load overlap matrix
		const NpyArray& overlap = arrays.at(String("overlap_") + pairData.suffix);
		ASSERT(warp.Shape()[0] == overlap.Shape()[0] && warp.Shape()[1] == overlap.Shape()[1]);
		ASSERT(ABS(overlap.Type()) == NpyArray::GetTypeChar(typeid(float)));
		pairData.overlap = Image32F(pairData.warpSize, const_cast<float*>(overlap.Data<float>())).clone();

		// Load precision weight (optional)
		auto precisionWeightIt = arrays.find(String("precision_weight_") + pairData.suffix);
		if (precisionWeightIt != arrays.end()) {
			const NpyArray& precisionWeight = precisionWeightIt->second;
			ASSERT(warp.Shape()[0] == precisionWeight.Shape()[0] && warp.Shape()[1] == precisionWeight.Shape()[1]);
			ASSERT(ABS(precisionWeight.Type()) == NpyArray::GetTypeChar(typeid(float)));
			pairData.precisionWeight = Image32F(pairData.warpSize, const_cast<float*>(precisionWeight.Data<float>())).clone();
		}

		// Apply orientation fix and optional erosion using reference image context
		const Image& refImage = scene.images[pairIdx.i];
		RotateMapsForReference(refImage, pairData.warp, pairData.overlap, pairData.precisionWeight);
		if (config.erodeBorder > 0)
			ErodeConfidenceMap(pairData.overlap, config.erodeBorder, config.minConfidence, config.minErodeConfidence);
	}

	std::unordered_map<uint64_t, NPZPairData> cache;  // Fast lookup of cached data
	ListFIFO<uint64_t> fifo;                          // Track LRU order
	Scene& scene;
	const ROMA2Config& config;
};

} // namespace
/*----------------------------------------------------------------*/


CLISTDEF2(String) SFM::ImportROMA2Files(const String& importROMA2Path)
{
	CLISTDEF2(String) files;
	if (importROMA2Path.empty())
		return files;
	if (std::filesystem::is_directory(std::string(importROMA2Path))) {
		for (const auto& entry : std::filesystem::directory_iterator(std::string(importROMA2Path))) {
			if (!entry.is_regular_file())
				continue;
			const String ext = String(entry.path().extension().string()).ToLower();
			if (ext == ".npz" || ext == ".npy")
				files.push_back(entry.path().string());
		}
	} else {
		// If source contains semicolon, treat as list
		Util::strSplit(importROMA2Path, ';', files);
	}
	return files;
}

unsigned SFM::ImportROMA2Matches(
	PairsMatcher& pairsMatcher,
	const ROMA2Config& config)
{
	CLISTDEF2(String) files = ImportROMA2Files(config.importROMA2Path);
	if (files.empty()) {
		VERBOSE("error: no ROMA2 NPZ files found in '%s'", config.importROMA2Path.c_str());
		return 0;
	}
	return ImportROMA2Matches(pairsMatcher, files, config);
}

unsigned SFM::ImportROMA2Matches(
	PairsMatcher& pairsMatcher,
	const CLISTDEF2(String)& npzFiles,
	const ROMA2Config& config)
{
	if (npzFiles.empty()) {
		VERBOSE("error: no NPZ files provided");
		return 0;
	}
	TD_TIMER_STARTD();
	Scene& scene = pairsMatcher.GetScene();

	// Build basename -> image ID map
	std::unordered_map<String, IIndex> nameToID;
	nameToID.reserve(scene.images.size());
	for (const Image& img : scene.images)
		nameToID.emplace(Util::getFileName(img.fileName), img.ID);

	unsigned numImportedPairs = 0;
	for (const String& file : npzFiles) {
		NpyArray::npz_t arrays;
		if (const char* err = NpyArray::LoadNPZ(file, arrays)) {
			VERBOSE("error: failed to load '%s' (%s)", file.c_str(), err);
			continue;
		}

		const NpyArray& warp = arrays.at("warp_AB");
		if (warp.Shape().size() != 3 || warp.Shape()[2] != 2 || ABS(warp.Type()) != NpyArray::GetTypeChar(typeid(float))) {
			VERBOSE("error: invalid warp_AB shape/type in '%s'", file.c_str());
			continue;
		}
		const cv::Size warpSize((int)warp.Shape()[1], (int)warp.Shape()[0]);

		const NpyArray& overlap = arrays.at("overlap_AB");
		if (overlap.Shape().size() != 3 || overlap.Shape()[2] != 1 || ABS(overlap.Type()) != NpyArray::GetTypeChar(typeid(float))) {
			VERBOSE("error: invalid overlap_AB shape/type in '%s'", file.c_str());
			continue;
		}
		ASSERT(warp.Shape()[0] == overlap.Shape()[0] && warp.Shape()[1] == overlap.Shape()[1]);

		const NpyArray& pathAArr = arrays.at("image_A_path");
		const NpyArray& pathBArr = arrays.at("image_B_path");
		const String pathA = Util::getFileName(pathAArr.StringVector().front());
		const String pathB = Util::getFileName(pathBArr.StringVector().front());
		if (pathA.empty() || pathB.empty()) {
			VERBOSE("error: invalid image paths in '%s'", file.c_str());
			continue;
		}
		auto idaIt = nameToID.find(pathA);
		auto idbIt = nameToID.find(pathB);
		if (idaIt == nameToID.end() || idbIt == nameToID.end()) {
			VERBOSE("error: images not found for pair %s - %s", pathA.c_str(), pathB.c_str());
			continue;
		}

		const IIndex idA = idaIt->second;
		const IIndex idB = idbIt->second;
		Image& imgA = scene.images[idA];
		Image& imgB = scene.images[idB];
		if (!imgA.HasDescriptors() || !imgB.HasDescriptors()) {
			VERBOSE("error: missing descriptors for pair %s - %s", pathA.c_str(), pathB.c_str());
			continue;
		}
		if (!imgA.HasCamera() || !imgB.HasCamera()) {
			VERBOSE("error: missing camera for image %s", pathA.c_str());
			continue;
		}

		const PairIdx pairIdx = MakePairIdx(idaIt->second, idbIt->second);
		ImagePair* scenePair = scene.FindPair(pairIdx.i, pairIdx.j);
		if (scenePair && scenePair->GetCompositeWeight() < config.minPairWeight) {
			DEBUG_ULTIMATE("warning: pair (% 4u, % 4u) already exists with low weight (%.2f), skipping",
				pairIdx.i, pairIdx.j, scenePair->GetCompositeWeight());
			continue;
		}

		std::vector<Point2f> trackedA, trackedB;
		std::vector<uchar> trackStatus;
		if (0 && arrays.find("keypoints_A") != arrays.end() && arrays.find("keypoints_B") != arrays.end()) {
			const NpyArray& arrayKeypointsA = arrays.at("keypoints_A");
			const NpyArray& arrayKeypointsB = arrays.at("keypoints_B");
			trackedA = std::vector<Point2f>(arrayKeypointsA.Data<Point2f>(), arrayKeypointsA.Data<Point2f>() + arrayKeypointsA.Shape()[0]);
			trackedB = std::vector<Point2f>(arrayKeypointsB.Data<Point2f>(), arrayKeypointsB.Data<Point2f>() + arrayKeypointsB.Shape()[0]);
			trackStatus.resize(trackedA.size(), 1);
		} else {
			Image32F2 imgWarp(warpSize, const_cast<Point2f*>(warp.Data<Point2f>()));
			Image32F imgOverlap(warpSize, const_cast<float*>(overlap.Data<float>()));
			// Create precision image only if precision_weight array exists (optional)
			Image32F imgPrecision;
			auto precisionWeightIt = arrays.find("precision_weight_AB");
			if (precisionWeightIt != arrays.end()) {
				const NpyArray& precisionWeight = precisionWeightIt->second;
				ASSERT(warp.Shape()[0] == precisionWeight.Shape()[0] && warp.Shape()[1] == precisionWeight.Shape()[1]);
				ASSERT(ABS(precisionWeight.Type()) == NpyArray::GetTypeChar(typeid(float)));
				imgPrecision = Image32F(warpSize, const_cast<float*>(precisionWeight.Data<float>()));
			}
			RotateMapsForReference(imgA, imgWarp, imgOverlap, imgPrecision);
			// Erode confidence map if requested (helps remove outliers near edges)
			if (config.erodeBorder > 0)
				ErodeConfidenceMap(imgOverlap, config.erodeBorder, config.minConfidence, config.minErodeConfidence);
			// Track keypoints from A to B using warp and overlap maps
			const size_t numKp = imgA.keypoints.size();
			trackedA.resize(numKp);
			trackedB.resize(numKp);
			trackStatus.resize(numKp);
			for (size_t i = 0; i < numKp; ++i) {
				const cv::Point2f& kpA = imgA.keypoints[i].pt;
				trackedA[i] = kpA;
				const Point2f wkpA = CoordFromTo(kpA, imgA.GetSize(), imgWarp.size());
				const float ckpB = imgOverlap.sample(wkpA);
				if (ckpB < config.minConfidence) {
					trackStatus[i] = 0;
					continue;
				}
				const Point2f nwkpB = imgWarp.sample(wkpA);
				const Point2f kpB = DenormCoord(nwkpB, imgB.GetSize());
				if (!Image8U::isInside(kpB, imgB.GetSize())) {
					trackStatus[i] = 0;
					continue;
				}
				trackedB[i] = kpB;
				trackStatus[i] = 1;
			}
		}

		ImagePair pair(pairIdx.i, pairIdx.j);
		MatchFeaturesGeometric(
			pairsMatcher,
			imgA,
			imgB,
			trackedA, trackedB, trackStatus,
			pair, config.epipolarThreshold);
		if (pair.matches.empty()) {
			DEBUG("error: no matches for pair (% 4u, % 4u) (%s - %s)",
				pair.ID1, pair.ID2, pathA.c_str(), pathB.c_str());
			continue;
		}

		if (scenePair) {
			if (scenePair->GetNumFilteredInliers() > pair.GetNumFilteredInliers()) {
				DEBUG("warning: pair (% 4u, % 4u) already has matches (%u vs %u inliers), skipping",
					pair.ID1, pair.ID2, scenePair->GetNumFilteredInliers(), pair.GetNumFilteredInliers());
				continue;
			}
			*scenePair = std::move(pair);
		} else {
			pair.overlapRatio = 1.f;
			pair.overlapArea = 1.f;
			scenePair = &scene.pairs.emplace_back(std::move(pair));
		}
		++numImportedPairs;
		DEBUG_EXTRA("Imported pair (% 4u, % 4u) with %u matches",
			scenePair->ID1, scenePair->ID2, (unsigned)scenePair->GetNumFilteredInliers());
	}

	if (numImportedPairs == 0) {
		DEBUG("error: no pairs imported from %zu files", npzFiles.size());
		return 0;
	}
	DEBUG("Imported %u ROMA2 pairs from %zu files (%s)",
		numImportedPairs, npzFiles.size(), TD_TIMER_GET_FMT().c_str());
	return numImportedPairs;
}
/*----------------------------------------------------------------*/


unsigned SFM::ImportROMA2DepthMaps(
	Scene& scene,
	const ROMA2Config& config,
	CLISTDEF2(String)* outDepthMapFiles)
{
	CLISTDEF2(String) files = ImportROMA2Files(config.importROMA2Path);
	if (files.empty()) {
		VERBOSE("error: no ROMA2 NPZ files found in '%s'", config.importROMA2Path.c_str());
		return 0;
	}
	return ImportROMA2DepthMaps(scene, files, config, outDepthMapFiles);
}

unsigned SFM::ImportROMA2DepthMaps(
	Scene& scene,
	const CLISTDEF2(String)& npzFiles,
	const ROMA2Config& config,
	CLISTDEF2(String)* outDepthMapFiles)
{
	if (npzFiles.empty()) {
		VERBOSE("error: no NPZ files provided");
		return 0;
	}
	TD_TIMER_STARTD();

	// Create output directory for depth maps
	String depthMapPath = MAKE_PATH_SAFE(config.depthMapPath);
	Util::ensureValidFolderPath(depthMapPath);
	Util::ensureFolder(depthMapPath);

	// Build basename -> image ID map
	std::unordered_map<String, IIndex> nameToID;
	nameToID.reserve(scene.images.size());
	for (const Image& img : scene.images)
		nameToID.emplace(Util::getFileName(img.fileName), img.ID);

	// For each image, store neighbor IDs for pairs where this image is reference
	std::unordered_map<IIndex, IIndexArr> imageToPairs;
	if (outDepthMapFiles)
		outDepthMapFiles->clear();

	// Initialize cache for metadata/warp data
	NPZPairCache pairCache(scene, config);

	// First pass: load all NPZ files and index them by participating images
	for (const String& file : npzFiles) {
		NpyArray::npz_t arrays;
		if (const char* err = NpyArray::LoadNPZ(file, arrays)) {
			VERBOSE("error: failed to load '%s' (%s)", file.c_str(), err);
			continue;
		}

		const NpyArray& warp = arrays.at("warp_AB");
		if (warp.Shape().size() != 3 || warp.Shape()[2] != 2 || ABS(warp.Type()) != NpyArray::GetTypeChar(typeid(float))) {
			VERBOSE("error: invalid warp_AB shape/type in '%s'", file.c_str());
			continue;
		}
		const cv::Size warpSize((int)warp.Shape()[1], (int)warp.Shape()[0]);

		const NpyArray& overlap = arrays.at("overlap_AB");
		if (overlap.Shape().size() != 3 || overlap.Shape()[2] != 1 || ABS(overlap.Type()) != NpyArray::GetTypeChar(typeid(float))) {
			VERBOSE("error: invalid overlap_AB shape/type in '%s'", file.c_str());
			continue;
		}
		ASSERT(warp.Shape()[0] == overlap.Shape()[0] && warp.Shape()[1] == overlap.Shape()[1]);

		const NpyArray& pathAArr = arrays.at("image_A_path");
		const NpyArray& pathBArr = arrays.at("image_B_path");
		const String pathA = Util::getFileName(pathAArr.StringVector().front());
		const String pathB = Util::getFileName(pathBArr.StringVector().front());
		if (pathA.empty() || pathB.empty()) {
			VERBOSE("error: invalid image paths in '%s'", file.c_str());
			continue;
		}
		auto idaIt = nameToID.find(pathA);
		auto idbIt = nameToID.find(pathB);
		if (idaIt == nameToID.end() || idbIt == nameToID.end()) {
			VERBOSE("error: images not found for pair %s - %s", pathA.c_str(), pathB.c_str());
			continue;
		}

		const IIndex idA = idaIt->second;
		const IIndex idB = idbIt->second;
		Image& imgA = scene.images[idA];
		Image& imgB = scene.images[idB];
		if (!imgA.IsValid() || !imgB.IsValid()) {
			VERBOSE("error: missing calibrated camera for images %s - %s", pathA.c_str(), pathB.c_str());
			continue;
		}

		// Store AB direction: add neighbor B to image A's list and cache metadata
		const PairIdx pairIdxAB(idA, idB);
		pairCache.AddMetadata(pairIdxAB.idx, file, warpSize, "AB");
		imageToPairs[idA].push_back(idB);

		// Check if BA direction exists and store it separately (only for image B)
		auto warpBAIt = arrays.find("warp_BA");
		auto overlapBAIt = arrays.find("overlap_BA");
		if (warpBAIt != arrays.end() && overlapBAIt != arrays.end()) {
			const NpyArray& warpBA = warpBAIt->second;
			const NpyArray& overlapBA = overlapBAIt->second;

			// Validate BA arrays (precision weight is optional)
			if (warpBA.Shape().size() == 3 && warpBA.Shape()[2] == 2 &&
				ABS(warpBA.Type()) == NpyArray::GetTypeChar(typeid(float)) &&
				overlapBA.Shape().size() == 3 && overlapBA.Shape()[2] == 1 &&
				ABS(overlapBA.Type()) == NpyArray::GetTypeChar(typeid(float))) {

				const cv::Size warpSizeBA((int)warpBA.Shape()[1], (int)warpBA.Shape()[0]);

				// Store BA direction: add neighbor A to image B's list and cache metadata
				const PairIdx pairIdxBA(idB, idA);
				pairCache.AddMetadata(pairIdxBA.idx, file, warpSizeBA, "BA");
				imageToPairs[idB].push_back(idA);
			}
		}
	}
	if (imageToPairs.empty()) {
		DEBUG("error: no valid NPZ pairs found in %zu files", npzFiles.size());
		return 0;
	}

	// Second pass: compose depth-maps for each image using cache for warp data
	if (outDepthMapFiles)
		outDepthMapFiles->resize(scene.images.size());
	const REAL cosAngleThreshold = COS(D2R((REAL)config.minTriangulationAngle));
	const float maxReprojectionErrorSq = SQUARE(config.maxReprojectionError);
	unsigned numImagesUpdated = 0;
	for (auto& [imageID, neighborIDs] : imageToPairs) {
		const Image& image = scene.images[imageID];
		if (!image.IsValid())
			continue;

		// Compute depth map resolution from max(warpSize) at original image aspect ratio
		const float aspectRatio = (float)image.GetWidth() / (float)image.GetHeight();
		// Find max dimension of warp grids for this image by checking cache metadata
		int maxWarpDim = 0;
		for (const IIndex neighID : neighborIDs) {
			const PairIdx pairIdx(imageID, neighID);
			const NPZPairData& pairData = pairCache.GetPairData(pairIdx.idx);
			maxWarpDim = MAXF3(maxWarpDim, pairData.warpSize.width, pairData.warpSize.height);
		}
		maxWarpDim = MINF(maxWarpDim, MAXF(image.GetWidth(), image.GetHeight()));
		// Compute depth-map size preserving aspect ratio
		cv::Size depthSize;
		if (aspectRatio >= 1.f) {
			depthSize.width = maxWarpDim;
			depthSize.height = ROUND2INT(maxWarpDim / aspectRatio);
		} else {
			depthSize.height = maxWarpDim;
			depthSize.width = ROUND2INT(maxWarpDim * aspectRatio);
		}

		// Structure to track multiple depth candidates per pixel
		struct DepthCandidate {
			float depth;
			float confidence;
			IIndex neighborID;
		};
		typedef SEACAVE::cList<DepthCandidate, const DepthCandidate&, 0, 4, uint16_t> DepthCandidateArr;
		std::vector<DepthCandidateArr> pixelCandidates(depthSize.area());

		// Process each pair containing this image
		for (const IIndex neighID : neighborIDs) {
			const Image& refImage = image;
			const Image& neighImage = scene.images[neighID];

			// Precompute relative pose using Image's Pose3D base
			const Pose3D relPose = neighImage / refImage;

			// Load or retrieve pair data from cache (already rotated/eroded during load)
			const PairIdx pairIdx(imageID, neighID);
			NPZPairData& pairData = pairCache.GetOrLoadPairData(pairIdx.idx);
			ASSERT(pairData.HasData());

			// Iterate over depth map pixels
			for (int y = 0; y < depthSize.height; ++y) {
				for (int x = 0; x < depthSize.width; ++x) {
					// Convert depth map pixel to warp grid coordinates
					const Point2f pt((float)x, (float)y);
					const Point2f warpCoord = CoordFromTo(pt, depthSize, pairData.warp.size());
					if (!pairData.warp.isInsideWithBorder(warpCoord, 1))
						continue;

					// Sample normalized warp coordinates, overlap and precision weight
					const Point2f normWarp = pairData.warp.sample(warpCoord);
					const float overlap = pairData.overlap.sample(warpCoord);
					float conf = overlap;
					#if 0
					const float precisionWeight = pairData.precisionWeight.sample(warpCoord);
					conf *= precisionWeight; // combined confidence metric
					#endif
					if (conf < config.minConfidence)
						continue;

					// Denormalize to pixel coordinates in neighbor image
					const Point2f pixelNeigh = DenormCoord(normWarp, neighImage.GetSize());
					if (!Image8U::isInside(pixelNeigh, neighImage.GetSize()))
						continue;

					// Convert depth map pixel to reference image coordinates
					const Point2f pixelRef = CoordFromTo(pt, depthSize, refImage.GetSize());

					// Unproject pixel coordinates to 3D bearing-ray points in camera space
					const Point3 rayRef = refImage.pCamera->Unproject(Cast<REAL>(pixelRef));
					const Point3 rayNeigh = neighImage.pCamera->Unproject(Cast<REAL>(pixelNeigh));

					// Check ray's angle; ignore if near parallel as the depth can not be computed accurately
					const Point3 rayNeighRef = relPose.R.t() * rayNeigh;
					const REAL cosAngle = ComputeAngle(rayRef.ptr(), rayNeighRef.ptr());
					if (cosAngle > cosAngleThreshold)
						continue;

					// Triangulate 3D point using relative pose and normalized coordinates
					Point3 X;
					if (!TriangulatePoint3D(relPose.R, relPose.C, rayRef, rayNeigh, X))
						continue;

					// Check depth validity and reprojection error
					const auto [pixelProj, valid] = refImage.pCamera->Project(X);
					if (!valid)
						continue;
					const float reprojErrorSq = normSq(Cast<float>(pixelProj) - pixelRef);
					if (reprojErrorSq > maxReprojectionErrorSq)
						continue;

					// Store this candidate for later sorting
					const int idx = y * depthSize.width + x;
					pixelCandidates[idx].push_back({(float)X.z, conf, neighID});
				}
			}
		}

		// Order neighbor IDs for this image by pair weight
		neighborIDs.Sort([&scene, &imageID](IIndex a, IIndex b) {
			const PairIdx pairIdxA = MakePairIdx(imageID, a);
			const PairIdx pairIdxB = MakePairIdx(imageID, b);
			const ImagePair* pairA = scene.FindPair(pairIdxA.i, pairIdxA.j);
			const ImagePair* pairB = scene.FindPair(pairIdxB.i, pairIdxB.j);
			const float weightA = pairA ? pairA->GetCompositeWeight() : 0.f;
			const float weightB = pairB ? pairB->GetCompositeWeight() : 0.f;
			return weightA > weightB;
		});

		// Build mapping from neighbor ID to index in neighbor list
		std::unordered_map<IIndex, uint8_t> neighborIDToIndex;
		FOREACH(i, neighborIDs)
			neighborIDToIndex[neighborIDs[i]] = (uint8_t)i;

		// Build final depth map and views map by selecting best candidates per pixel
		Image8U4 viewsMap(depthSize, Color8U::BLACK);

		// Initialize depth accumulation maps
		Image32F bestDepthMap(depthSize, 0.f);
		Image32F bestConfidenceMap(depthSize, 0.f);

		// Process each pixel's candidates
		for (int y = 0; y < depthSize.height; ++y) {
			for (int x = 0; x < depthSize.width; ++x) {
				const int idx = y * depthSize.width + x;
				DepthCandidateArr& candidates = pixelCandidates[idx];
				if (candidates.empty())
					continue;
				// Sort candidates by confidence (descending)
				candidates.Sort([](const DepthCandidate& a, const DepthCandidate& b) {
					return a.confidence > b.confidence;
				});
				// Compute the weighted average of the similar depth candidates
				if (config.weightedDepthAverage) {
					TAccumulator<float> depthAcc(candidates[0].depth, candidates[0].confidence);
					// Consider only similar depth candidates
					for (unsigned i = 1; i < candidates.size(); ++i)
						if (IsDepthSimilar(candidates[0].depth, candidates[i].depth, config.depthSimilarityThreshold))
							depthAcc.Add(candidates[i].depth, candidates[i].confidence);
					bestDepthMap(y, x) = depthAcc.Normalized();
					bestConfidenceMap(y, x) = depthAcc.NormalizedWeight();
				} else {
					// Use best candidate directly
					bestDepthMap(y, x) = candidates[0].depth;
					bestConfidenceMap(y, x) = candidates[0].confidence;
				}
				// Store up to 4 neighbor indices in views map (ordered by confidence)
				uint8_t* views = viewsMap.ptr(y, x);
				const size_t numViews = MINF(candidates.size(), uint16_t(4));
				for (size_t i = 0; i < numViews; ++i) {
					const IIndex neighID = candidates[i].neighborID;
					views[i] = neighborIDToIndex.at(neighID);
				}
			}
		}

		// Check if we have any valid depth values
		double dMin, dMax;
		cv::minMaxIdx(bestDepthMap, &dMin, &dMax, NULL, NULL, bestDepthMap > 0);
		if (dMax <= 0) {
			DEBUG("warning: no valid depths for image %u (%s)", imageID, image.fileName.c_str());
			continue;
		}

		// Build view IDs array: reference image first, then neighbors
		IIndexArr IDs;
		IDs.push_back(imageID);
		IDs.Join(neighborIDs);

		// Export depth map to DMAP file (restore original orientation when needed)
		image.ToOriginalOrientation(bestDepthMap);
		image.ToOriginalOrientation(bestConfidenceMap);
		image.ToOriginalOrientation(viewsMap);
		KMatrix K = image.GetK();
		RMatrix R = image.R;
		const cv::Size imageSize = image.RevertRotation(&K, &R);
		const String dmapFileName = depthMapPath + String::FormatString("depth%04u.dmap", imageID);
		if (!ExportDepthDataRaw(
			dmapFileName,
			image.fileName,
			IDs,
			imageSize,
			K, R, image.C,
			(float)dMin, (float)dMax,
			bestDepthMap,
			bestConfidenceMap,
			viewsMap))
		{
			VERBOSE("error: failed to export depth map '%s'", dmapFileName.c_str());
			continue;
		}

		++numImagesUpdated;
		if (outDepthMapFiles)
			(*outDepthMapFiles)[imageID] = dmapFileName;
		DEBUG_EXTRA("Exported depth map for image %u (%s): %dx%d, depth [%.3f, %.3f], %u neighbors",
			imageID, image.fileName.c_str(), depthSize.width, depthSize.height,
			(float)dMin, (float)dMax, (unsigned)neighborIDs.size());
	}

	if (numImagesUpdated == 0) {
		DEBUG("error: no depth maps imported from %zu files", npzFiles.size());
		return 0;
	}
	DEBUG("Imported %u ROMA2 depth maps from %zu files (%s)",
		numImagesUpdated, npzFiles.size(), TD_TIMER_GET_FMT().c_str());
	return numImagesUpdated;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
