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
		avgPixel, avgAngular, numTracks, numErrors / (double)MAXF(numTracks, 1u));
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
		numInlierTracks, scene.tracks.size(), avgPixel, maxReprojErrorPixels, avgAngular, numInlierErrors / (double)MAXF(numInlierTracks, 1u), trackCompletenessStats.GetMean()*100, trackCompletenessStats.GetStdDev()*100);
	return std::make_pair(avgPixel, avgAngular);
}


namespace {

// Per-image inlier-observation index in CSR layout (avoids the O(images x tracks) membership
// scan the filter used to run per image). Built once over the inlier prefix of
// every track with >= 2 inliers: for image i, its observations live in
// [offset[i], offset[i+1]) as parallel (track index, featureID) pairs.
struct ImageObsCSR {
	std::vector<uint32_t> offset; // size numImages+1
	std::vector<uint32_t> track;  // size totObs (track index into scene.tracks)
	std::vector<uint32_t> feat;   // size totObs (featureID within the image)
};

// Build the CSR from the current scene state (counting pass -> prefix sum -> fill pass).
static void BuildImageObsCSR(const Scene& scene, ImageObsCSR& csr)
{
	const IIndex n = scene.images.size();
	csr.offset.assign(n + 1, 0);
	for (const Track& t : scene.tracks) {
		if (!t.IsInlier())
			continue;
		for (uint8_t k = 0; k < t.numInliers; ++k)
			++csr.offset[t.observations[k].imageID + 1];
	}
	for (IIndex i = 0; i < n; ++i)
		csr.offset[i + 1] += csr.offset[i];
	const uint32_t totObs = csr.offset[n];
	csr.track.resize(totObs);
	csr.feat.resize(totObs);
	std::vector<uint32_t> cursor(csr.offset.begin(), csr.offset.end() - 1);
	FOREACH(ti, scene.tracks) {
		const Track& t = scene.tracks[ti];
		if (!t.IsInlier())
			continue;
		for (uint8_t k = 0; k < t.numInliers; ++k) {
			const Observation& obs = t.observations[k];
			const uint32_t pos = cursor[obs.imageID]++;
			csr.track[pos] = ti;
			csr.feat[pos] = obs.featureID;
		}
	}
}

// Deterministic sort-based covisibility: every track with >= minInliersPerTrack inliers
// contributes one shared point to each unordered pair of its inlier images. Emits the pairs
// as packed (lo<<32|hi) keys, sorts, run-length-encodes, and keeps edges whose total count is
// >= minCovisibilityCount. Output is sorted by (i,j) ascending, so every downstream consumer
// (union-find, k-core) sees a fixed order (replaces the former unordered_map iteration order).
static void BuildCovisEdges(const Scene& scene, unsigned minCovisibilityCount,
	uint8_t minInliersPerTrack, std::vector<std::array<unsigned, 3>>& edges)
{
	std::vector<uint64_t> keys;
	size_t est = 0;
	for (const Track& t : scene.tracks)
		if (t.IsInlier(minInliersPerTrack))
			est += (size_t)t.numInliers * (t.numInliers - 1) / 2;
	keys.reserve(MINF(est, (size_t)64 * 1024 * 1024)); // cap the reservation; grow geometrically past it
	for (const Track& t : scene.tracks) {
		if (!t.IsInlier(minInliersPerTrack))
			continue;
		for (uint8_t a = 0; a < t.numInliers; ++a) {
			const uint32_t ia = t.observations[a].imageID;
			for (uint8_t b = a + 1; b < t.numInliers; ++b) {
				const uint32_t ib = t.observations[b].imageID;
				const uint32_t lo = MINF(ia, ib), hi = MAXF(ia, ib);
				keys.push_back(((uint64_t)lo << 32) | hi);
			}
		}
	}
	std::sort(keys.begin(), keys.end());
	edges.clear();
	for (size_t s = 0; s < keys.size(); ) {
		size_t e = s + 1;
		while (e < keys.size() && keys[e] == keys[s])
			++e;
		const unsigned count = (unsigned)(e - s);
		if (count >= minCovisibilityCount) {
			const uint64_t key = keys[s];
			edges.push_back({ (unsigned)(key >> 32), (unsigned)(key & 0xffffffffu), count });
		}
		s = e;
	}
}

} // namespace


// Prunes weakly connected and wrongly positioned images, clustering the remainder by 3D
// point covisibility. Covisibility is the number of 3D points visible in both images; high
// covisibility implies a reliable relative pose, low covisibility a weak link.
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
// 4. LARGEST CONNECTED COMPONENT FILTERING
//    - Computes connected components of the covisibility graph (edges >= minCovisibilityCount)
//    - Keeps largest component (by image count)
//    - Removes isolated image groups not connected to the main reconstruction
//
// 5. WEAK-ATTACHMENT REMOVAL (absolute k-core)
//    - Peel images whose covisibility degree (number of neighbors sharing >= minCovisibilityCount
//      inlier tracks) is < minCovisDegree (default 2), iterating until stable, then keep the
//      largest connected component of what remains
//    - Absolute, not scene-relative: densely-connected images always survive; only thinly-attached
//      images and the tails/segments left behind when they are peeled are removed
//    - Replaces a former median-MAD clustering that used a scene-relative threshold as a trust
//      criterion and was unstable (nondeterministically over-cut densely-connected scenes)
//
// POSE-CONSISTENCY (optional, off by default; maxPoseInconsistencyAngle > 0):
//    - The connectivity stages measure conditioning, not pose correctness: a wrongly positioned
//      view (bad resection on repetitive texture, mis-merged segment) can be well spread, well
//      triangulated, and share >= minCovisibilityCount tracks with its (co-wrong) neighbors.
//    - What betrays a wrong pose is disagreement with independent evidence. Between edge
//      construction and the largest-CC pass, each covisibility edge whose global relative
//      rotation (R2 * R1^T) disagrees with the stored two-view relativePose by more than
//      maxPoseInconsistencyAngle is cut. A lone wrong view loses its edges to the correct core
//      and is peeled by the k-core; a co-wrong clique keeps its internal edges but splits off
//      and is removed by the largest-CC. One mechanism, both cases, no new drop stage.
//    - Optional per-image backstops (maxReprojErrorPixels > 0) drop an image only when BOTH
//      absolute signals agree (low match-survival AND high robust reprojection error).
//      Never a single-signal or scene-relative drop.
//    - Blind spot: a wrong placement that is fully self-consistent (two-view poses, tracks, and
//      BA all agreeing because the same repetitive structure fooled all of them) is undetectable
//      from internal geometry; it needs external evidence (GPS, loop closure, semantics).
//
// RETURN VALUE:
// =============
// Array of invalidated image IDs (removed by the pre-filters or the connectivity stages).
//
// USAGE:
// ======
//   // After building and filtering tracks
//   BuildTracks(scene);
//   FilterTracks(scene);
//   IIndexArr removedIDs = FilterWeaklyConnectedImages(scene);
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
//
// minCovisDegree (default 2):
//   - Minimum number of independent covisibility neighbors an image must keep to survive the
//     k-core peel; sibling absolute threshold to minCovisibilityCount
//
// maxPoseInconsistencyAngle (default 0 = disabled):
//   - Cut covisibility edges whose global relative rotation disagrees with the stored two-view
//     relativePose by more than this angle (detects wrongly positioned views). 0 disables.
//   - Clean, well-converged scenes carry up to ~5 deg two-view-vs-BA rotation noise on correct
//     edges, so prefer 8-10 deg when enabling; at or below 5 deg correct edges start being cut.
//
// maxReprojErrorPixels (default 0 = disabled; pass config.maxFineReprojError to enable):
//   - Enables the agreement-gated per-image backstops (match-survival + robust reprojection).
//     0 leaves the backstops off. Both signals are absolute (never scene-relative).
IIndexArr SFM::FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount,
	float minObservationArea,
	float minTriangulationAngle,
	unsigned minCovisDegree,
	float maxPoseInconsistencyAngle,
	float maxReprojErrorPixels)
{
	TD_TIMER_STARTD();
	struct PairIdxCount {
		PairIdx pairIdx;
		unsigned count;
	};
	IIndexArr filteredIDs;

	// One shared, entry-state CSR of per-image inlier observations: kills the former
	// O(images x tracks) membership scan that the tier pre-filters ran per image.
	constexpr int gridSize = 10; // 10x10 grid
	constexpr uint8_t minInliersPerTrack = 3; // covisibility only counts tracks with >= 3 inliers
	ImageObsCSR csr;
	BuildImageObsCSR(scene, csr);

	// Tier 1: Spatial Distribution Filter (Effective Inlier Count) — clustered features give a
	//         weak pose constraint.
	// Tier 2: Geometric Degeneracy Filter (Triangulation Angle) — a small median angle means a
	//         degenerate baseline.
	// Both verdicts are computed on the entry state (below) and applied together afterwards, so
	// they are order-independent: invalidating one image never shifts another's angle medians or
	// covisibility mid-loop (the old inline InvalidateImage made verdicts index-order dependent).
	const unsigned minNumObservationsForGrid = ROUND2INT<unsigned>(SQUARE(gridSize) * minObservationArea);
	MeanStdMinMax<REAL> coverageStats;
	TMatrix<uint8_t, gridSize, gridSize> occupiedCells;
	const float minAngleRadians = D2R(minTriangulationAngle);
	MeanStdMinMax<REAL> angleStats;
	std::vector<uint8_t> tierDrop(scene.images.size(), 0); // 0=keep, 1=tier1, 2=tier2
	FloatArr obsAngles, medAngles; // hoisted out of the per-image loop (avoid per-track churn)
	FOREACH(imgIdx, scene.images) {
		const Image& image = scene.images[imgIdx];
		if (!image.IsValid())
			continue;
		// Count occupied grid cells and the median triangulation angle over this image's own
		// inlier observations (CSR slice), then record a tier verdict without mutating the scene.
		const float cellWidth = (float)image.pCamera->GetWidth() / gridSize;
		const float cellHeight = (float)image.pCamera->GetHeight() / gridSize;
		unsigned numOccupiedCells = 0;
		occupiedCells.memset(0);
		medAngles.clear();
		for (uint32_t p = csr.offset[imgIdx]; p < csr.offset[imgIdx + 1]; ++p) {
			const Track& track = scene.tracks[csr.track[p]];
			const cv::KeyPoint& kp = image.keypoints[csr.feat[p]];
			// Clamp cell indices: undistorted keypoints can land at/past the border, which would
			// otherwise write outside the fixed 10x10 grid (mirrors the diagnostics twin).
			const int cellX = MINF(MAXF((int)(kp.pt.x / cellWidth), 0), gridSize - 1);
			const int cellY = MINF(MAXF((int)(kp.pt.y / cellHeight), 0), gridSize - 1);
			uint8_t& cell = occupiedCells(cellX, cellY);
			if (cell == 0) { cell = 1; ++numOccupiedCells; }
			// Median triangulation angle: this image's ray vs every other inlier observation's ray
			const Point3 ray = image.C - track.position;
			obsAngles.clear();
			for (uint8_t k = 0; k < track.numInliers; ++k) {
				const IIndex other = track.observations[k].imageID;
				if (other == imgIdx)
					continue;
				const Point3 otherRay = scene.images[other].C - track.position;
				obsAngles.push_back(ComputeAngle(ray.ptr(), otherRay.ptr()));
			}
			if (!obsAngles.empty())
				medAngles.push_back(obsAngles.GetNth((obsAngles.size() - 1) / 2)); // per-track median angle
		}
		// Tier-1 verdict: effective inlier count as fraction of occupied cells
		if (numOccupiedCells < minNumObservationsForGrid) {
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated for low spatial distribution (%.2f%% < %.2f%% cells occupied), %u visible tracks",
				imgIdx, Util::getFileName(image.fileName).c_str(), (float)numOccupiedCells / SQUARE(gridSize) * 100.f, (float)minObservationArea * 100.f, (unsigned)medAngles.size());
			tierDrop[imgIdx] = 1;
			continue;
		}
		// Tier-2 verdict: median triangulation angle
		if (medAngles.empty())
			continue;
		const float medianAngle = ACOS(medAngles.GetMedian());
		if (medianAngle < minAngleRadians) {
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated for low median triangulation angle (%.2f° < %.2f°), %.2f%% cells occupied, %u visible tracks",
				imgIdx, Util::getFileName(image.fileName).c_str(), R2D(medianAngle), minTriangulationAngle, (float)numOccupiedCells / SQUARE(gridSize) * 100.f, (unsigned)medAngles.size());
			tierDrop[imgIdx] = 2;
			continue;
		}
		coverageStats.Update((REAL)numOccupiedCells / SQUARE(gridSize));
		angleStats.Update(medianAngle);
	}
	DEBUG_EXTRA("Image coverage: mean %.2f stddev %.2f range [%.2f,%.2f] n %u",
		coverageStats.GetMean()*100, coverageStats.GetStdDev()*100, coverageStats.GetMin()*100, coverageStats.GetMax()*100, coverageStats.size);
	DEBUG_EXTRA("Triangulation angle: mean %.2f° stddev %.2f° range [%.2f°,%.2f°] n %u",
		R2D(angleStats.GetMean()), R2D(angleStats.GetStdDev()), R2D(angleStats.GetMin()), R2D(angleStats.GetMax()), angleStats.size);

	// Apply the tier verdicts in a single batch sweep over the tracks (order-independent).
	{
		IIndexArr tierDropIDs;
		FOREACH(imgIdx, scene.images)
			if (tierDrop[imgIdx])
				tierDropIDs.push_back(imgIdx);
		scene.InvalidateImages(tierDropIDs);
		for (const IIndex id : tierDropIDs)
			filteredIDs.push_back(id);
	}

	// Step 1+2: covisibility graph over the surviving inlier tracks (sort-based, deterministic,
	// recomputed on the post-tier state so counts match the current scene). Output is sorted by
	// (i,j), so union-find and the k-core peel below see a fixed edge order.
	std::vector<std::array<unsigned, 3>> covisEdges;
	BuildCovisEdges(scene, minCovisibilityCount, minInliersPerTrack, covisEdges);
	CLISTDEF0(PairIdxCount) edgeWeights;
	edgeWeights.reserve(covisEdges.size());
	for (const auto& e : covisEdges)
		edgeWeights.push_back({ PairIdx(e[0], e[1]), e[2] });
	DEBUG_EXTRA("Established visibility graph with %u/%u images and %u image pairs",
		scene.status.nCalibratedImages, scene.images.size(), (unsigned)edgeWeights.size());
	if (edgeWeights.empty()) {
		DEBUG("error: no valid image pairs found for clustering");
		return filteredIDs;
	}

	// Pose-consistency edge filter (optional; off unless maxPoseInconsistencyAngle > 0).
	// Cut covisibility edges whose global relative rotation (R2 * R1^T; Pose3D::R is world->camera)
	// disagrees with the stored two-view relativePose (image1->image2, ID1 < ID2 == pidx.i < pidx.j,
	// so orientation always matches). An unknown or thin pair keeps its edge — absence of evidence
	// is not evidence of inconsistency. Cutting a wrong view's edges starves its degree so the
	// k-core peels it (lone view) or the largest-CC drops it (co-wrong clique). One mechanism, both
	// cases. Same consistency criterion as GlobalRotationEstimator::FilterRelativeRotations.
	if (maxPoseInconsistencyAngle > 0.f) {
		constexpr unsigned minPairInliersForCheck = 30;
		const REAL minCosAngle = COS(D2R(maxPoseInconsistencyAngle));
		unsigned numChecked = 0, numCut = 0;
		RFOREACH(ei, edgeWeights) {
			const PairIdx pidx = edgeWeights[ei].pairIdx;
			const ImagePair* p = scene.FindPair(pidx.i, pidx.j);
			if (!p || !p->relativePose || p->GetNumFilteredInliers() < minPairInliersForCheck)
				continue; // unknown != inconsistent: keep the edge
			const Matrix3x3 relCalcR = scene.images[pidx.j].R * scene.images[pidx.i].R.t();
			const REAL cosAngle = ComputeAngle(p->relativePose->R, relCalcR);
			++numChecked;
			if (cosAngle < minCosAngle) {
				DEBUG_EXTRA("warning: covisibility edge (%u,%u) cut for pose inconsistency (%.2f° > %.2f°, %u inliers)",
					pidx.i, pidx.j, R2D(ACOS(cosAngle)), maxPoseInconsistencyAngle, p->GetNumFilteredInliers());
				edgeWeights.RemoveAt(ei);
				++numCut;
			}
		}
		DEBUG("Pose-consistency: cut %u/%u checked covisibility edges (> %.2f°)", numCut, numChecked, maxPoseInconsistencyAngle);
	}

	// Step 3: Keep only the largest connected component and invalidate the rest
	// Use disjoint-set (union-find) for connected component analysis
	DisjointSet<IIndex> ds(scene.images.size());
	// Union all image pairs connected by edges
	for (const PairIdxCount& edge : edgeWeights)
		ds.Union(edge.pairIdx.i, edge.pairIdx.j);
	const auto InvalidateImagesIfNotInLargestComponent = [&scene, &filteredIDs, &ds]() {
		const std::unordered_map<IIndex, unsigned> componentSizes = ds.CompressAllPaths().GetComponentSizes();
		// Largest component root; tie-break to the smaller root ID so the choice is deterministic
		// regardless of the (unordered) map iteration order.
		IIndex largestComponentRoot = NO_ID;
		unsigned maxSize = 0;
		for (const auto& [root, size] : componentSizes)
			if (size > maxSize || (size == maxSize && root < largestComponentRoot)) {
				maxSize = size;
				largestComponentRoot = root;
			}
		// Invalidate images not in the largest component, in one batch sweep
		IIndexArr dropIDs;
		FOREACH(imgIdx, scene.images) {
			if (scene.images[imgIdx].IsValid() && largestComponentRoot != ds.Find(imgIdx)) {
				DEBUG_EXTRA("warning: image %u (`%s`) invalidated for not in largest connected component",
					imgIdx, Util::getFileName(scene.images[imgIdx].fileName).c_str());
				dropIDs.push_back(imgIdx);
			}
		}
		scene.InvalidateImages(dropIDs);
		for (const IIndex id : dropIDs)
			filteredIDs.push_back(id);
		DEBUG_EXTRA("Kept %u images in largest connected component (from %u components)",
			scene.status.nCalibratedImages, (unsigned)componentSizes.size());
	};
	InvalidateImagesIfNotInLargestComponent();

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

	// Step 4: Stable absolute weak-attachment removal (k-core), replacing a former median-MAD
	// clustering. That clustering used a scene-relative threshold (median minus MAD of the edge
	// weights) as a trust criterion, which is unstable: on densely-connected scenes it
	// nondeterministically split off and discarded well-connected, trustworthy images (e.g. ~200
	// on Tanks&Temples Courthouse, all immediately re-registered by the following resection).
	// Trust is absolute, not relative to how dense the rest of the scene is: an image is weakly
	// attached only when it shares enough covisibility with too few independent neighbors. So peel
	// images whose covisibility degree (number of neighbors sharing >= minCovisibilityCount inlier
	// tracks) is below minCovisDegree, iterating until stable, then keep the largest connected
	// component. The peel runs purely on the edge graph via a local alive[]/degree[] pair, so its
	// correctness never depends on when the scene is mutated; the peeled images are invalidated in
	// one batch at the end. (Whole sub-scenes that cannot be placed in a common frame are already
	// handled earlier, at merge time in GlobalAlignment, by keeping only the largest sub-scene.)
	std::vector<std::vector<IIndex>> adj(scene.images.size());
	for (const PairIdxCount& edge : edgeWeights) {
		adj[edge.pairIdx.i].push_back(edge.pairIdx.j);
		adj[edge.pairIdx.j].push_back(edge.pairIdx.i);
	}
	std::vector<uint8_t> alive(scene.images.size(), 0);
	FOREACH(imgIdx, scene.images)
		alive[imgIdx] = scene.images[imgIdx].IsValid() ? 1 : 0;
	std::vector<unsigned> degree(scene.images.size(), 0);
	FOREACH(imgIdx, scene.images)
		if (alive[imgIdx])
			for (const IIndex nb : adj[imgIdx])
				if (alive[nb])
					++degree[imgIdx];
	std::vector<IIndex> peelQueue;
	FOREACH(imgIdx, scene.images)
		if (alive[imgIdx] && degree[imgIdx] < minCovisDegree)
			peelQueue.push_back(imgIdx);
	IIndexArr peeledIDs;
	while (!peelQueue.empty()) {
		const IIndex imgIdx = peelQueue.back();
		peelQueue.pop_back();
		if (!alive[imgIdx] || degree[imgIdx] >= minCovisDegree)
			continue;
		DEBUG_EXTRA("warning: image %u (`%s`) invalidated for weak covisibility degree (%u < %u)",
			imgIdx, Util::getFileName(scene.images[imgIdx].fileName).c_str(), degree[imgIdx], minCovisDegree);
		alive[imgIdx] = 0;
		peeledIDs.push_back(imgIdx);
		for (const IIndex nb : adj[imgIdx])
			if (alive[nb] && degree[nb] > 0) {
				--degree[nb];
				if (degree[nb] < minCovisDegree)
					peelQueue.push_back(nb);
			}
	}
	scene.InvalidateImages(peeledIDs);
	for (const IIndex id : peeledIDs)
		filteredIDs.push_back(id);

	// Keep the largest connected component of the peeled graph (drops any segment that the
	// peeling severed from the main reconstruction).
	ds.Reset(scene.images.size());
	for (const PairIdxCount& edge : edgeWeights)
		if (scene.images[edge.pairIdx.i].IsValid() && scene.images[edge.pairIdx.j].IsValid())
			ds.Union(edge.pairIdx.i, edge.pairIdx.j);
	InvalidateImagesIfNotInLargestComponent();

	// Agreement-gated per-image backstops (optional; off unless maxReprojErrorPixels > 0).
	// Drop an image only when BOTH absolute signals agree — low match-survival AND high robust
	// reprojection error — then run one more largest-CC pass so a backstop drop cannot strand a
	// segment. Both signals are absolute; a single marginal signal never fires.
	if (maxReprojErrorPixels > 0.f) {
		constexpr float survivalFloor = 0.2f;
		constexpr unsigned survivalMinMatches = 100;

		// Signal A source: match-survival. Verified inlier matches whose two endpoints do not land
		// on one shared inlier track are "lost" (FilterTracks stripped a misregistered view's obs).
		std::unordered_map<uint64_t, uint32_t> featToTrack;
		FOREACH(t, scene.tracks) {
			const Track& track = scene.tracks[t];
			if (!track.IsInlier(minInliersPerTrack))
				continue;
			for (uint8_t k = 0; k < track.numInliers; ++k)
				featToTrack[((uint64_t)track.observations[k].imageID << 32) | track.observations[k].featureID] = (uint32_t)t;
		}
		std::vector<unsigned> lostCross(scene.images.size(), 0), totalVerified(scene.images.size(), 0);
		for (const ImagePair& pair : scene.pairs) {
			if (!scene.images[pair.ID1].IsValid() || !scene.images[pair.ID2].IsValid())
				continue;
			const unsigned nInl = MINF(pair.GetNumFilteredInliers(), (unsigned)pair.matches.size());
			for (unsigned m = 0; m < nInl; ++m) {
				const DMatch& dm = pair.matches[m];
				const auto a = featToTrack.find(((uint64_t)pair.ID1 << 32) | dm.queryIdx);
				const auto b = featToTrack.find(((uint64_t)pair.ID2 << 32) | dm.trainIdx);
				++totalVerified[pair.ID1]; ++totalVerified[pair.ID2];
				if (a == featToTrack.end() || b == featToTrack.end() || a->second != b->second) {
					++lostCross[pair.ID1]; ++lostCross[pair.ID2];
				}
			}
		}

		IIndexArr backstopIDs;
		FloatArr resid;
		FOREACH(imgIdx, scene.images) {
			const Image& image = scene.images[imgIdx];
			if (!image.IsValid())
				continue;
			// Signal A: match-survival ratio
			if (totalVerified[imgIdx] < survivalMinMatches)
				continue;
			const float survival = 1.f - (float)lostCross[imgIdx] / (float)totalVerified[imgIdx];
			if (survival >= survivalFloor)
				continue; // first signal did not fire -> cannot reach 2 signals
			// Signal B: robust (median) per-image reprojection error against current track positions
			resid.clear();
			for (uint32_t p = csr.offset[imgIdx]; p < csr.offset[imgIdx + 1]; ++p) {
				const Track& track = scene.tracks[csr.track[p]];
				if (!track.IsInlier())
					continue;
				const Point3 Xcam = image.TransformPointW2C(track.position);
				const auto [projected, valid] = image.pCamera->Project(Xcam);
				if (!valid)
					continue;
				const Point2 kppt = Cast<REAL>(image.keypoints[csr.feat[p]].pt);
				resid.push_back((float)norm(projected - kppt));
			}
			if (resid.empty() || resid.GetMedian() <= maxReprojErrorPixels)
				continue; // second signal did not fire
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated by backstops (survival %.2f < %.2f, reproj median %.2fpx > %.2fpx)",
				imgIdx, Util::getFileName(image.fileName).c_str(), survival, survivalFloor, resid.GetMedian(), maxReprojErrorPixels);
			backstopIDs.push_back(imgIdx);
		}
		if (!backstopIDs.empty()) {
			scene.InvalidateImages(backstopIDs);
			for (const IIndex id : backstopIDs)
				filteredIDs.push_back(id);
			ds.Reset(scene.images.size());
			for (const PairIdxCount& edge : edgeWeights)
				if (scene.images[edge.pairIdx.i].IsValid() && scene.images[edge.pairIdx.j].IsValid())
					ds.Union(edge.pairIdx.i, edge.pairIdx.j);
			InvalidateImagesIfNotInLargestComponent();
		}
	}

	DEBUG("Filtered %u/%u weakly connected images in %s",
		filteredIDs.size(), scene.status.nCalibratedImages+filteredIDs.size(), TD_TIMER_GET_FMT().c_str());
	return filteredIDs;
}
/*----------------------------------------------------------------*/
