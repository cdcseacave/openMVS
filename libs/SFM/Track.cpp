/*
 * Track.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 */

#include "Common.h"
#include "Track.h"
#include "Scene.h"

using namespace SFM;

// S T R U C T S ///////////////////////////////////////////////////

float Track::ComputeMinAngleBetweenRays(const ImageArr& images) const
{
	// Minimum triangulation angle
	float minCosAngle = 1;
	for (uint32_t i=0; i+1<numInliers; ++i) {
		const Observation& obs1 = observations[i];
		const Image& img1 = images[obs1.imageID];
		// Compute ray from point to camera center
		const Point3 ray1 = img1.C - position;
		for (uint32_t j=i+1; j<numInliers; ++j) {
			const Observation& obs2 = observations[j];
			const Image& img2 = images[obs2.imageID];
			// Compute angle between rays
			const Point3 ray2 = img2.C - position;
			const float cosAngle = ComputeAngle(ray1.ptr(), ray2.ptr());
			if (minCosAngle > cosAngle)
				minCosAngle = cosAngle;
		}
	}
	return ACOS(minCosAngle);
}

void SFM::BuildTracks(Scene& scene, float minPairWeight)
{
	TD_TIMER_STARTD();
	scene.tracks.Release();

	// 1. Pre-compute feature offsets for O(1) global ID lookup
	// globalID = featureOffsets[imageID] + featureID
	Unsigned32Arr featureOffsets(0, scene.images.size() + 1);
	uint32_t globalID = 0;
	for (const Image& img : scene.images) {
		featureOffsets.push_back(globalID);
		globalID += (uint32_t)img.keypoints.size();
	}
	featureOffsets.push_back(globalID); // sentinel
	if (globalID == 0) {
		VERBOSE("error: no features found in images");
		return;
	}

	using ImageCount = std::unordered_map<IIndex, uint16_t>;
	std::vector<std::unique_ptr<ImageCount>> trackImages(globalID);
	std::vector<bool> featureCounted(globalID, false);
	DisjointSet<uint32_t> ds(globalID);

	// Ensure the root set has a map and that this feature's image is counted once
	auto AccumulateFeature = [&](uint32_t gid) {
		if (featureCounted[gid])
			return;
		const uint32_t root = ds.Find(gid);
		if (!trackImages[root])
			trackImages[root] = std::make_unique<ImageCount>();
		// Given a global feature ID, find its image ID via featureOffsets
		auto it = std::upper_bound(featureOffsets.begin(), featureOffsets.end(), gid);
		ASSERT(it != featureOffsets.begin());
		const IIndex imgID = static_cast<IIndex>(it - featureOffsets.begin() - 1);
		++((*trackImages[root])[imgID]);
		featureCounted[gid] = true;
	};

	// 2. Merge observations from image pairs
	// Ideally the pairs are pre-filtered to only include inlier matches
	// and sorted by weight (most reliable first) to maximize track quality.
	unsigned numPairsProcessed = 0;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches())
			continue;
		if (minPairWeight >= 0 && pair.GetCompositeWeight() <= minPairWeight)
			continue;
		// Only inlier matches contribute to tracks
		const uint32_t offset1 = featureOffsets[pair.ID1];
		const uint32_t offset2 = featureOffsets[pair.ID2];
		FOREACHRAW(i, pair.GetNumFilteredInliers()) {
			const DMatch& m = pair.matches[i];
			ASSERT(m.queryIdx < scene.images[pair.ID1].keypoints.size());
			ASSERT(m.trainIdx < scene.images[pair.ID2].keypoints.size());
			const uint32_t id1 = offset1 + m.queryIdx;
			const uint32_t id2 = offset2 + m.trainIdx;
			// Make sure current features are accounted in their roots before testing overlap
			AccumulateFeature(id1);
			AccumulateFeature(id2);
			// Attempt to union the two features
			ds.UnionIf(id1, id2,
				// Combined guard+merge: veto if same image repeats; otherwise merge metadata
				[&](uint32_t rootDst, uint32_t rootSrc) {
					auto& mapDst = trackImages[rootDst];
					auto& mapSrc = trackImages[rootSrc];
					ASSERT(mapDst && mapSrc);
					for (const auto& kv : *mapSrc)
						if (mapDst->find(kv.first) != mapDst->end())
							return false; // duplicate image, reject union
					for (const auto& kv : *mapSrc)
						(*mapDst)[kv.first] += kv.second;
					mapSrc.reset();
					return true;
				}
			);
		}
		++numPairsProcessed;
	}

	// 3. Group observations by track representative
	// Map: rootGlobalID -> list of observations
	std::map<uint32_t, ObservationArr> tracks;
	// Iterate all features to find their roots
	for (uint32_t imgID = 0; imgID < scene.images.size(); ++imgID) {
		const Image& img = scene.images[imgID];
		const uint32_t offset = featureOffsets[imgID];
		for (uint32_t fid = 0; fid < img.keypoints.size(); ++fid) {
			const uint32_t gid = offset + fid;
			const uint32_t root = ds.Find(gid);
			tracks[root].emplace_back(imgID, fid);
		}
	}

	// 4. Filter tracks (minimum 2 views) and add to scene
	scene.tracks.reserve(tracks.size() / 2); // heuristic reservation
	uint32_t numObservations = 0;
	for (auto& [root, observations] : tracks) {
		if (observations.size() < 2)
			continue;
		// Sort observations for consistent ordering
		observations.Sort();
		// Create track (position will be triangulated later)
		Track& track = scene.tracks.emplace_back();
		track.observations.reserve(observations.size());
		for (const Observation& obs : observations)
			track.observations.emplace_back(obs);
		numObservations += observations.size();
	}
	DEBUG("Built %u tracks from %u observations and %u pairs (avg %.2f views/track) in %s",
	    scene.tracks.size(), globalID, numPairsProcessed,
	    numObservations / (float)MAXF(scene.tracks.size(), 1u), TD_TIMER_GET_FMT().c_str());

	#ifndef _RELEASE
	VERBOSE("Performing additional track consistency checks...");
	// Temporary safety check: ensure match indices are within keypoints bounds
	FOREACH(pairIdx, scene.pairs) {
		const ImagePair& pair = scene.pairs[pairIdx];
		if (!pair.HasMatches())
			continue;
		if (pair.ID1 >= scene.images.size() || pair.ID2 >= scene.images.size()) {
			VERBOSE("BuildTracks: invalid pair image IDs (%u, %u) for %u images", pair.ID1, pair.ID2, (unsigned)scene.images.size());
			continue;
		}
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		FOREACHRAW(i, pair.GetNumFilteredInliers()) {
			const DMatch& m = pair.matches[i];
			if (static_cast<uint32_t>(m.queryIdx) >= img1.keypoints.size() ||
				static_cast<uint32_t>(m.trainIdx) >= img2.keypoints.size()) {
				VERBOSE("BuildTracks: out-of-range match index (q=%d/%u, t=%d/%u) in pair (%u, %u)",
					m.queryIdx, (unsigned)img1.keypoints.size(), m.trainIdx, (unsigned)img2.keypoints.size(), pair.ID1, pair.ID2);
			}
		}
	}
	// Temporary safety check: ensure track observations are valid
	FOREACH(trackIdx, scene.tracks) {
		const Track& track = scene.tracks[trackIdx];
		std::unordered_set<IIndex> seenImages;
		FOREACH(obsIdx, track.observations) {
			const Observation& obs = track.observations[obsIdx];
			if (obs.imageID >= scene.images.size()) {
				VERBOSE("BuildTracks: invalid observation imageID %u (tracks=%u images=%u)",
					obs.imageID, (unsigned)scene.tracks.size(), (unsigned)scene.images.size());
				continue;
			}
			if (obs.featureID >= scene.images[obs.imageID].keypoints.size()) {
				VERBOSE("BuildTracks: invalid observation featureID %u (image=%u, keypoints=%u)",
					obs.featureID, obs.imageID, (unsigned)scene.images[obs.imageID].keypoints.size());
			}
			// Check that each image appears at most once in the track
			if (!seenImages.emplace(obs.imageID).second) {
				VERBOSE("BuildTracks: duplicate image %u in track %u (observation %u)",
					obs.imageID, trackIdx, obsIdx);
			}
		}
	}
	#endif
}


std::pair<float, float> SFM::ComputeTracksMeanReprojectionError(Scene& scene)
{
	// Compute average reprojection errors
	double sumAngularError = 0.0, sumPixelError = 0.0;
	uint32_t numTracks = 0, numErrors = 0;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const Image& img = scene.images[obs.imageID];
			ASSERT(img.IsValid());
			ASSERT(obs.featureID < img.keypoints.size());
			const Point2 kppt = Cast<REAL>(img.keypoints[obs.featureID].pt);
			// Compute predicted projection
			const Point3 Xworld = track.position;
			const Point3 Xcam = img.TransformPointW2C(Xworld);
			// Pixel error
			const auto [projected, valid] = img.pCamera->Project(Xcam);
			if (!valid)
				continue;
			const double pixelError = norm(projected - kppt);
			sumPixelError += pixelError;
			// Angular error
			const Point3 observedRay = img.pCamera->UnprojectNormalized(kppt);
			const double cosAngularError = ComputeAngle(observedRay.ptr(), Xcam.ptr());
			sumAngularError += cosAngularError;
			++numErrors;
		}
		++numTracks;
	}
	double avgAngular = 0.0, avgPixel = 0.0;
	if (numErrors > 0) {
		avgAngular = R2D(ACOS(sumAngularError / numErrors));
		avgPixel = sumPixelError / numErrors;
	}
	DEBUG_EXTRA("Mean reprojection error: %.2f pixels (%.2f deg) from %u tracks (%.2f views/track)",
		avgPixel, avgAngular, numTracks, numErrors / (double)numTracks);
	return std::make_pair(avgPixel, avgAngular);
}

std::pair<float, float> SFM::FilterTracks(Scene& scene,
	float maxReprojErrorPixels, float minAngleDegrees,
	float multDepthNear, float multDepthFar)
{
	const float minAngleRadians = D2R(minAngleDegrees);

	// Process each track
	MeanStdMinMax<REAL> trackCompletenessStats;
	double sumAngularError = 0.0, sumPixelError = 0.0;
	uint32_t numInlierTracks = 0, numInlierErrors = 0;
	FloatArr dists(0, MAXF(scene.status.nTracks, 100u));
	for (Track& track : scene.tracks) {
		track.numInliers = 0;
		if (!track.IsValid())
			continue;

		// Partition observations into inliers and outliers
		double sumTrackAngularError = 0.0, sumTrackPixelError = 0.0, sumTrackDist = 0.0;
		FOREACH(obsIdx, track.observations) {
			const Observation& obs = track.observations[obsIdx];
			const Image& img = scene.images[obs.imageID];
			ASSERT(img.HasCamera());
			if (!img.IsValid())
				continue;
			// Angular reprojection error — unified gate that works for both pinhole and spherical
			// (equirectangular pixel distance doesn't correspond linearly to angular separation).
			// Pinhole cheirality is handled automatically: a back-facing Xcam yields a negative
			// dot product with the front-facing observedRay, so cos < 0 < minCosAngularError.
			const Point3 Xcam = img.TransformPointW2C(track.position);
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			const Point3 observedRay = img.pCamera->UnprojectNormalized(Cast<REAL>(kp.pt));
			const REAL cosAngularError = ComputeAngle(observedRay.ptr(), Xcam.ptr());
			const REAL minCosAngularError = COS(img.pCamera->PixelErrorToAngular(maxReprojErrorPixels));
			if (cosAngularError < minCosAngularError)
				continue; // outlier or behind the camera observation
			// Accepted — compute projection for pixel-error stats (well-defined now: cheirality passed above)
			const Point2 projected = img.pCamera->Project(Xcam).first;
			const float pixelError = norm(Cast<float>(projected) - kp.pt);
			// Move inlier to the front of the observation list
			if (track.numInliers < obsIdx)
				std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			sumTrackPixelError += pixelError;
			sumTrackAngularError += cosAngularError;
			// Euclidean distance from camera center — always non-negative and
			// well-defined for any central camera, including spherical
			sumTrackDist += (float)norm(Xcam);
			++track.numInliers;
		}

		// Track must have at least 2 inlier observations to be considered inlier
		if (!track.IsInlier())
			continue;

		// Check minimum angle between any two inlier observations
		const float minAngle = track.ComputeMinAngleBetweenRays(scene.images);
		if (minAngle < minAngleRadians) {
			track.numInliers = 0; // mark track as outlier
			continue;
		}

		// This is a valid inlier track, accumulate reprojection errors
		numInlierErrors += track.numInliers;
		sumPixelError += sumTrackPixelError;
		sumAngularError += sumTrackAngularError;
		dists.push_back(sumTrackDist / track.numInliers);
		trackCompletenessStats.Update((REAL)track.numInliers / track.observations.size());
		++numInlierTracks;
	}

	// Remove far tracks based on depth statistics
	uint32_t filteredTracksNear = 0, filteredTracksFar = 0;
	if (dists.size() > 1000 && (multDepthNear > 0.f || multDepthFar > 0.f)) {
		// Compute median distance
		const float medianDist = FloatArr(dists).GetMedian();
		// Define minimum/maximum allowed distance
		const float minAllowedDistNear = multDepthNear * medianDist;
		const float maxAllowedDistFar = multDepthFar > 0 ? multDepthFar * medianDist : FLT_MAX;
		// Filter tracks based on distance
		uint32_t idxInlier = 0;
		for (Track& track : scene.tracks) {
			if (!track.IsInlier())
				continue;
			const float avgDist = dists[idxInlier++];
			if (avgDist < minAllowedDistNear) {
				track.numInliers = 0; // mark track as outlier
				++filteredTracksNear;
			} else if (avgDist > maxAllowedDistFar) {
				track.numInliers = 0; // mark track as outlier
				++filteredTracksFar;
			}
		}
		if (filteredTracksNear > 0 || filteredTracksFar > 0) {
			numInlierTracks -= (filteredTracksNear + filteredTracksFar);
			DEBUG_EXTRA("Filtered %u tracks (%u near, %u far) based on distance threshold [%.2f near, %.2f far] (median %.2f)",
				filteredTracksNear + filteredTracksFar, filteredTracksNear, filteredTracksFar, minAllowedDistNear, maxAllowedDistFar, medianDist);
		}
	}
	scene.status.nTracks = numInlierTracks;

	// Compute mean errors
	REAL avgAngular = 0.0, avgPixel = 0.0;
	if (numInlierErrors > 0) {
		avgAngular = R2D(ACOS(sumAngularError / numInlierErrors));
		avgPixel = sumPixelError / numInlierErrors;
	}
	DEBUG_EXTRA("Tracks filtered: %u/%u inliers, mean reprojection error %.2f pixels (%.2f th), angular %.2g deg, %.2f views/track (completeness: %.2f mean, %.2f stddev)",
		numInlierTracks, scene.tracks.size(), avgPixel, maxReprojErrorPixels, avgAngular, numInlierErrors / (double)numInlierTracks, trackCompletenessStats.GetMean()*100, trackCompletenessStats.GetStdDev()*100);
	return std::make_pair(avgPixel, avgAngular);
}


// Prunes weakly connected images and clusters the remainder based on 3D point
// covisibility.
//
// Covisibility is the number of 3D points visible in both images. Images with
// high covisibility likely have reliable relative pose estimates, while weakly
// connected images may have less reliable geometry.
//
// Additionally, applies two pre-filters:
// Tier 1: Spatial Distribution Filter - invalidates images with clustered tracks
//         (Effective Inlier Count < threshold)
// Tier 2: Geometric Degeneracy Filter - invalidates images with small triangulation
//         angles (median angle < threshold), indicating insufficient baseline
//
// Algorithm:
//   1. Pre-filter 1: Check spatial distribution of inlier tracks in each image
//   2. Pre-filter 2: Check triangulation angles for geometric constraints
//   3. Build a covisibility graph where edges connect images sharing >= min points
//   4. Find the largest connected component and mark isolated images for removal
//   5. Compute an adaptive edge weight threshold using median minus maximum
//      absolute deviation (MAD)
//   6. Cluster images using union-find: first merge strongly connected images,
//      then iteratively merge clusters connected by weaker edges
// Prunes weakly connected images and clusters the remainder based on 3D point covisibility.
//
// ALGORITHM STAGES:
// =================
//
// 1. PRE-FILTER 1: Spatial Distribution Check (Effective Inlier Count)
//    - Problem: Images with clustered features have weak pose constraints
//    - Solution: Divide image into 10x10 grid, count occupied cells
//    - Threshold: Neff = occupied_cells/100; invalidate if Neff < 0.15 (default)
//    - Detects: Textureless regions, poor parallax, insufficient constraints
//
// 2. PRE-FILTER 2: Geometric Degeneracy Check (Triangulation Angles)
//    - Problem: Images too distant from structure have small triangulation angles
//    - Solution: Compute median angle between rays to all visible 3D points
//    - Threshold: Invalidate if median_angle < 1.5° (default)
//    - Detects: Insufficient depth resolution, high translation uncertainty
//
// 3. COVISIBILITY GRAPH CONSTRUCTION
//    - For each inlier track: increment edge weight for all image pairs that see it
//    - Keep edges with weight >= minCovisibilityCount (e.g., 5 shared points)
//    - Result: Undirected graph where weights = shared 3D points
//    - High covisibility → reliable relative pose; Low covisibility → weak geometry
//
// 4. LARGEST CONNECTED COMPONENT FILTERING (Boost Graph)
//    - Computes connected components of covisibility graph
//    - Keeps largest component (by image count)
//    - Removes isolated image groups not well-connected to main reconstruction
//    - Graceful fallback: skips this stage if Boost unavailable
//
// 5. ADAPTIVE THRESHOLDING (Median-MAD)
//    - Collect all edge weights in largest component
//    - Compute: median = median(weights)
//    - Compute: MAD = median(|weight - median|)
//    - Threshold: T = max(median - MAD, 5)
//    - Rationale: Robust to non-Gaussian weight distributions; prevents over-segmentation
//
// 6. HIERARCHICAL CLUSTERING (Union-Find)
//    - Phase 1: Merge image pairs with weight > threshold (strong clustering)
//    - Phase 2: Iteratively merge clusters with >= 2 weak edges (weight >= 0.75*T)
//    - Result: Final cluster assignments for each image
//    - Benefit: Avoids over-segmentation when weak connections are distributed
//
// RETURN VALUE:
// =============
// Array of cluster IDs (one per image):
//   - NO_ID: Image invalidated by pre-filters or not in largest component
//   - 0, 1, 2, ...: Cluster assignment for valid, well-connected images
//
// USAGE:
// ======
//   // After building and filtering tracks
//   BuildTracks(scene);
//   FilterTracks(scene);
//   IIndexArr clusterIds = FilterWeaklyConnectedImages(scene);
//
// PARAMETER GUIDANCE:
// ===================
// minCovisibilityCount (default 5):
//   - Minimum shared 3D points to link two images
//   - Lower (3-4): More edges, sparser clustering
//   - Higher (7-10): Fewer edges, denser clustering
//   - Typical: 5 (balances robustness vs connectivity)
//
// minObservationArea (default 0.15):
//   - Minimum fraction of 10x10 grid cells that must contain tracks
//   - Lower (0.10): More lenient, keeps images with clustered features
//   - Higher (0.20): Stricter, requires distributed features
//   - Detects spatial degeneracy: features in textureless regions or poor parallax
//
// minTriangulationAngle (default 1.5):
//   - Minimum median triangulation angle in degrees
//   - Lower (1.0): More lenient, accepts distant images
//   - Higher (2.5): Stricter, requires better baselines and parallax
//   - Detects geometric degeneracy: insufficient depth resolution or translation uncertainty
IIndexArr SFM::FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount,
	float minObservationArea,
	float minTriangulationAngle)
{
	TD_TIMER_STARTD();
	struct PairIdxCount {
		PairIdx pairIdx;
		unsigned count;
	};
	IIndexArr filteredIDs;

	// Tier 1: Spatial Distribution Filter (Effective Inlier Count)
	// Check if inlier tracks are spread across the image or clustered in one area
	constexpr int gridSize = 10; // 10x10 grid
	const unsigned minNumObservationsForGrid = ROUND2INT<unsigned>(SQUARE(gridSize) * minObservationArea);
	MeanStdMinMax<REAL> coverageStats;
	TMatrix<uint8_t, gridSize, gridSize> occupiedCells;
	// Tier 2: Geometric Degeneracy Filter (Triangulation Angle)
	// Check if the image has sufficient triangulation angles for reliable pose
	const float minAngleRadians = D2R(minTriangulationAngle);
	MeanStdMinMax<REAL> angleStats;
	FloatArr angles(0, MAXF(scene.tracks.size(), 100u));
	FOREACH(imgIdx, scene.images) {
		const Image& image = scene.images[imgIdx];
		if (!image.IsValid())
			continue;
		// Count occupied grid cells and
		// compute median triangulation angle for tracks visible in this image
		const float cellWidth = (float)image.pCamera->GetWidth() / gridSize;
		const float cellHeight = (float)image.pCamera->GetHeight() / gridSize;
		unsigned numOccupiedCells = 0;
		occupiedCells.memset(0);
		angles.clear();
		for (const Track& track : scene.tracks) {
			if (!track.IsInlier())
				continue;
			// Find observation in this image
			bool seesTrack = false;
			for (const Observation& obs : track) {
				if (obs.imageID == imgIdx) {
					// Project observation location to grid
					const cv::KeyPoint& kp = image.keypoints[obs.featureID];
					int cellX = static_cast<int>(kp.pt.x / cellWidth);
					int cellY = static_cast<int>(kp.pt.y / cellHeight);
					uint8_t& cell = occupiedCells(cellX, cellY);
					if (cell == 0) {
						cell = 1; // mark cell as occupied
						++numOccupiedCells;
					}
					seesTrack = true;
					break;
				}
			}
			if (!seesTrack)
				continue;
			// Compute triangulation angle for this track
			const Point3 ray = image.C - track.position;
			FloatArr obsAngles(0, track.numInliers-1);
			for (const Observation& obs : track) {
				if (obs.imageID == imgIdx)
					continue;
				const Image& otherImage = scene.images[obs.imageID];
				Point3 otherRay = otherImage.C - track.position;
				float cosAngle = ComputeAngle(ray.ptr(), otherRay.ptr());
				obsAngles.push_back(cosAngle);
			}
			ASSERT(!obsAngles.empty())
			angles.push_back(obsAngles.GetNth((obsAngles.size()-1) / 2)); // median angle
		}
		// Effective inlier count as fraction of occupied cells
		if (numOccupiedCells < minNumObservationsForGrid) {
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated for low spatial distribution (%.2f%% < %.2f%% cells occupied), %u visible tracks",
				imgIdx, Util::getFileName(image.fileName).c_str(), (float)numOccupiedCells / SQUARE(gridSize) * 100.f, (float)minObservationArea * 100.f, (unsigned)angles.size());
			if (scene.InvalidateImage(imgIdx))
				filteredIDs.push_back(imgIdx);
			continue;
		}
		// Check median triangulation angle
		if (angles.empty())
			continue;
		const float medianAngle = ACOS(angles.GetMedian());
		if (medianAngle < minAngleRadians) {
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated for low median triangulation angle (%.2f° < %.2f°), %.2f%% cells occupied, %u visible tracks",
				imgIdx, Util::getFileName(image.fileName).c_str(), R2D(medianAngle), minTriangulationAngle, (float)numOccupiedCells / SQUARE(gridSize) * 100.f, (unsigned)angles.size());
			if (scene.InvalidateImage(imgIdx))
				filteredIDs.push_back(imgIdx);
			continue;
		}
		coverageStats.Update((REAL)numOccupiedCells / SQUARE(gridSize));
		angleStats.Update(medianAngle);
	}
	DEBUG_EXTRA("Image coverage: mean %.2f stddev %.2f range [%.2f,%.2f] n %u",
		coverageStats.GetMean()*100, coverageStats.GetStdDev()*100, coverageStats.GetMin()*100, coverageStats.GetMax()*100, coverageStats.size);
	DEBUG_EXTRA("Triangulation angle: mean %.2f° stddev %.2f° range [%.2f°,%.2f°] n %u",
		R2D(angleStats.GetMean()), R2D(angleStats.GetStdDev()), R2D(angleStats.GetMin()), R2D(angleStats.GetMax()), angleStats.size);

	// Step 1: Compute covisibility counts between all image pairs
	// based on shared inlier tracks
	constexpr uint8_t minInliersPerTrack = 3; // only consider tracks with >= 3 inliers
	std::unordered_map<PairIdx, unsigned> imageCovisibilityCount;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier(minInliersPerTrack))
			continue;
		// Increment covisibility count for all image pairs
		for (uint8_t i = 0; i < track.numInliers; ++i) {
			for (uint8_t j = i + 1; j < track.numInliers; ++j) {
				IIndex img1 = track.observations[i].imageID;
				IIndex img2 = track.observations[j].imageID;
				PairIdx pairIdx = MakePairIdx(img1, img2);
				imageCovisibilityCount[pairIdx]++;
			}
		}
	}

	// Step 2: Filter edges to keep only reliable connections
	CLISTDEF0(PairIdxCount) edgeWeights;
	edgeWeights.reserve(imageCovisibilityCount.size());
	for (const auto& [pairIdx, count] : imageCovisibilityCount)
		if (count >= minCovisibilityCount)
			edgeWeights.push_back({ pairIdx, count });
	DEBUG_EXTRA("Established visibility graph with %u/%u images and %u/%u image pairs",
		scene.status.nCalibratedImages, scene.images.size(), (unsigned)edgeWeights.size(), (unsigned)imageCovisibilityCount.size());
	if (edgeWeights.empty()) {
		DEBUG("error: no valid image pairs found for clustering");
		return filteredIDs;
	}

	// Step 3: Keep only the largest connected component and invalidate the rest
	// Use disjoint-set (union-find) for connected component analysis
	DisjointSet<IIndex> ds(scene.images.size());
	// Union all image pairs connected by edges
	for (const PairIdxCount& edge : edgeWeights)
		ds.Union(edge.pairIdx.i, edge.pairIdx.j);
	const auto InvaidateImagesIfNotInLargestComponent = [&scene,&filteredIDs,&ds]() {
		std::unordered_map<IIndex, unsigned> componentSizes = ds.CompressAllPaths().GetComponentSizes();
		// Find the largest component root
		IIndex largestComponentRoot = NO_ID;
		unsigned maxSize = 0;
		for (const auto& [root, size] : componentSizes) {
			if (size > maxSize) {
				maxSize = size;
				largestComponentRoot = root;
			}
		}
		// Invalidate images not in largest component
		FOREACH(imgIdx, scene.images) {
			if (scene.images[imgIdx].IsValid() && largestComponentRoot != ds.Find(imgIdx)) {
				DEBUG_EXTRA("warning: image %u (`%s`) invalidated for not in largest connected component",
					imgIdx, Util::getFileName(scene.images[imgIdx].fileName).c_str());
				if (scene.InvalidateImage(imgIdx))
					filteredIDs.push_back(imgIdx);
			}
		}
		DEBUG_EXTRA("Kept %u images in largest connected component (from %u components)",
			scene.status.nCalibratedImages, (unsigned)componentSizes.size());
	};
	InvaidateImagesIfNotInLargestComponent();

	// Filter edge weights to keep only edges within largest component
	RFOREACH(i, edgeWeights) {
		const PairIdxCount& edge = edgeWeights[i];
		if (!scene.images[edge.pairIdx.i].IsValid() ||
		    !scene.images[edge.pairIdx.j].IsValid())
			edgeWeights.RemoveAt(i);
	}
	if (edgeWeights.empty()) {
		DEBUG("error: no edge weights available for clustering");
		return filteredIDs;
	}

	// Step 4: Compute adaptive threshold using median minus median absolute deviation (MAD)
	constexpr unsigned minEdgeWeightThreshold = 5; // minimum threshold for acceptable covisibility
	const auto [median, mad] = ComputeX84Threshold<unsigned,float>(edgeWeights.size(),
	[&edgeWeights](size_t i) { return edgeWeights[i].count; }, 1.f);
	const unsigned edgeWeightThreshold = MAXF((unsigned)(median - mad), minEdgeWeightThreshold);
	DEBUG_EXTRA("Image covisibility clustering: median=%u, MAD=%u, threshold=%u",
		(unsigned)median, (unsigned)mad, edgeWeightThreshold);

	// Step 5: Clusters images using union-find based on pair covisibility weights.
	// The iterative refinement helps avoid over-segmentation when the connection
	// between two groups of images is distributed across multiple weaker edges.
	constexpr float weakEdgeMultiplier = 0.75f; // weaker edges must have weight >= 75% of threshold to be considered for merging
	constexpr unsigned minWeakEdgesToMerge = 2; // clusters must share >= 2 weak edges to be merged
	constexpr unsigned maxClusteringIterations = 10; // limit iterations to prevent infinite loops

	// 5.1. Create initial clusters from strong edges (weight > threshold)
	ds.Reset(scene.images.size());
	for (const PairIdxCount& edge : edgeWeights)
		if (edge.count > edgeWeightThreshold)
			ds.Union(edge.pairIdx.i, edge.pairIdx.j);

	// 5.2. Iteratively merge clusters connected by multiple weaker edges
	// Two clusters are merged if they share >= minWeakEdgesToMerge edges with
	// weight >= weakEdgeMultiplier * threshold. This continues until no more
	// merges occur (or maxClusteringIterations iterations)
	bool changed = true;
	unsigned iteration = 0;
	while (changed && iteration++ < maxClusteringIterations) {
		changed = false;

		// Count edges between each pair of cluster roots
		std::unordered_map<IIndex, std::unordered_map<IIndex, unsigned>> numPairs;
		for (const PairIdxCount& edge : edgeWeights) {
			if (edge.count < weakEdgeMultiplier * edgeWeightThreshold)
				continue;
			const IIndex root1 = ds.Find(edge.pairIdx.i);
			const IIndex root2 = ds.Find(edge.pairIdx.j);
			if (root1 == root2)
				continue; // already in same cluster
			numPairs[root1][root2]++;
			numPairs[root2][root1]++;
		}

		// Merge clusters that share >= minWeakEdgesToMerge connecting edges
		for (const auto& [root1, counter] : numPairs) {
			for (const auto& [root2, count] : counter) {
				if (root1 <= root2)
					continue; // process each pair once
				if (count >= minWeakEdgesToMerge) {
					changed = true;
					ds.Union(root1, root2);
				}
			}
		}
	}

	// 5.3. Invalidate images not in largest component after clustering
	InvaidateImagesIfNotInLargestComponent();

	DEBUG("Filtered %u/%u weakly connected images in %s",
		filteredIDs.size(), scene.status.nCalibratedImages+filteredIDs.size(), TD_TIMER_GET_FMT().c_str());
	return filteredIDs;
}
/*----------------------------------------------------------------*/
