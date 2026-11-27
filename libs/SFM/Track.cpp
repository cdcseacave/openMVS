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
#include "PoseLink.h"
#include "Scene.h"

using namespace SFM;

// S T R U C T S ///////////////////////////////////////////////////

unsigned Track::GetNumPosedObservations(const ImageArr& images) const
{
	unsigned numPosed = 0;
	for (const Observation& obs : observations)
		if (images[obs.imageID].IsValid())
			++numPosed;
	return numPosed;
}

float Track::ComputeMaxAngleBetweenRays(const ImageArr& images) const
{
	// the widest angle is the smallest cosine
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

namespace {

// A component of the plain union of the matches that holds one image twice, settled on its own
// links (see BuildTracks): the links are made local, the conflict is cut or vetoed, and the pieces
// left are the component's tracks.
class ConflictedComponent
{
public:
	struct Link {
		uint32_t a, b;  // the two keypoints: global feature IDs given, local indices once localized
		uint32_t order; // position among the track-forming matches, in pair order
		float weight;   // the pair's composite weight
	};

	// Take the links of one component; nodeInfo(globalID, image, position) describes a keypoint.
	template <typename Iterator, typename NodeInfo>
	void Set(Iterator first, Iterator last, NodeInfo nodeInfo) {
		links.clear();
		nodes.clear();
		for (Iterator it = first; it != last; ++it) {
			links.push_back({it->a, it->b, it->order, it->weight});
			nodes.push_back(it->a);
			nodes.push_back(it->b);
		}
		std::sort(nodes.begin(), nodes.end());
		nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
		for (Link& link : links) {
			link.a = Local(link.a);
			link.b = Local(link.b);
		}
		images.resize(nodes.size());
		positions.resize(nodes.size());
		FOREACH(n, nodes)
			nodeInfo(nodes[n], images[n], positions[n]);
		standing.assign(links.size(), true);
		adjacency.assign(nodes.size(), {});
		FOREACH(e, links) {
			adjacency[links[e].a].emplace_back(links[e].b, (uint32_t)e);
			adjacency[links[e].b].emplace_back(links[e].a, (uint32_t)e);
		}
	}

	size_t NumNodes() const { return nodes.size(); }

	// Cut, while two keypoints of one image share a piece without being one feature (within
	// mergeDist2; negative merges nothing), the least-supported link on the shortest path between
	// them: the fewest triangles through it, then the lighter pair, then the earlier one in pair order. The
	// support is counted once, on the component as it arrived: a triangle is evidence whether or
	// not one of its links has since been cut. Returns the number of links cut.
	unsigned Cut(float mergeDist2) {
		// the triangles through each link: the common neighbours of its two keypoints
		std::vector<unsigned> support(links.size(), 0);
		std::vector<uint32_t> mark(nodes.size(), NO_ID);
		FOREACH(e, links) {
			for (const auto& [n, l] : adjacency[links[e].a])
				mark[n] = (uint32_t)e;
			for (const auto& [n, l] : adjacency[links[e].b])
				if (mark[n] == e)
					++support[e];
		}
		unsigned numCut = 0;
		std::vector<uint32_t> via(nodes.size()); // the link a keypoint was reached through
		std::vector<uint32_t> queue;
		while (true) {
			LabelPieces();
			const auto [x1, x2] = FirstConflict(mergeDist2);
			if (x1 == NO_ID)
				break;
			// the shortest path from x1 to x2 over the standing links
			std::fill(via.begin(), via.end(), NO_ID);
			queue.assign(1, x1);
			via[x1] = (uint32_t)links.size(); // reached, through no link
			for (size_t i = 0; i < queue.size() && via[x2] == NO_ID; ++i)
				for (const auto& [n, l] : adjacency[queue[i]])
					if (standing[l] && via[n] == NO_ID) {
						via[n] = l;
						queue.push_back(n);
					}
			ASSERT(via[x2] != NO_ID); // the two are one piece
			// the least-supported link on it
			uint32_t cut = NO_ID;
			for (uint32_t n = x2; n != x1; ) {
				const uint32_t l = via[n];
				if (cut == NO_ID || support[l] < support[cut] || (support[l] == support[cut] &&
					(links[l].weight < links[cut].weight || (links[l].weight == links[cut].weight && links[l].order < links[cut].order))))
					cut = l;
				n = links[l].a == n ? links[l].b : links[l].a;
			}
			standing[cut] = false;
			++numCut;
		}
		return numCut;
	}

	// Union the links in order, refusing one that would bring an image in twice unless the two
	// keypoints are one feature (within mergeDist2; negative merges nothing): the rule of the
	// union veto. Returns the number of links vetoed.
	unsigned Veto(float mergeDist2) {
		DisjointSet<uint32_t> ds(nodes.size());
		// the keypoints of every set, sorted by image
		std::vector<std::vector<uint32_t>> members(nodes.size());
		FOREACH(n, nodes)
			members[n].assign(1, (uint32_t)n);
		const auto ByImage = [this](uint32_t n1, uint32_t n2) { return images[n1] < images[n2]; };
		unsigned numVetoed = 0;
		std::vector<uint32_t> merged;
		FOREACH(e, links) {
			const uint32_t r1 = ds.Find(links[e].a), r2 = ds.Find(links[e].b);
			if (r1 == r2)
				continue;
			// every image the two sets share has to be one feature across them: every keypoint of
			// it in one set within the merge distance of every one in the other (a set holds
			// several of one image only when they were merged as one feature before)
			bool oneFeature = true;
			const auto& m1 = members[r1];
			const auto& m2 = members[r2];
			for (size_t i1 = 0, i2 = 0; oneFeature && i1 < m1.size() && i2 < m2.size(); ) {
				if (images[m1[i1]] < images[m2[i2]]) { ++i1; continue; }
				if (images[m2[i2]] < images[m1[i1]]) { ++i2; continue; }
				size_t j1 = i1, j2 = i2;
				while (j1 < m1.size() && images[m1[j1]] == images[m1[i1]]) ++j1;
				while (j2 < m2.size() && images[m2[j2]] == images[m2[i2]]) ++j2;
				for (size_t k1 = i1; oneFeature && k1 < j1; ++k1)
					for (size_t k2 = i2; oneFeature && k2 < j2; ++k2)
						oneFeature = mergeDist2 >= 0.f && Distance2(m1[k1], m2[k2]) <= mergeDist2;
				i1 = j1;
				i2 = j2;
			}
			if (!oneFeature) {
				standing[e] = false;
				++numVetoed;
				continue;
			}
			ds.Union(r1, r2);
			const uint32_t r = ds.Find(r1);
			merged.resize(members[r1].size() + members[r2].size());
			std::merge(members[r1].begin(), members[r1].end(), members[r2].begin(), members[r2].end(), merged.begin(), ByImage);
			members[r == r1 ? r2 : r1].clear();
			members[r].swap(merged);
		}
		return numVetoed;
	}

	// The pieces the standing links leave are the tracks, numbered from nextTrackID; where a piece
	// still holds one image twice (one feature detected twice) the keypoint with more standing
	// links stands for the image, the first at a tie, and the others get no track. Returns the
	// number of keypoints so dropped.
	unsigned Assign(uint32_t& nextTrackID, std::vector<uint32_t>& trackOf) {
		const unsigned numPieces = LabelPieces();
		FOREACH(n, nodes)
			trackOf[nodes[n]] = nextTrackID + piece[n];
		nextTrackID += numPieces;
		unsigned numDropped = 0;
		SortByPieceAndImage();
		for (size_t i = 0, j; i < order.size(); i = j) {
			for (j = i + 1; j < order.size() && SamePieceAndImage(order[i], order[j]); ++j) ;
			uint32_t winner = order[i];
			for (size_t k = i + 1; k < j; ++k)
				if (degree[order[k]] > degree[winner] || (degree[order[k]] == degree[winner] && order[k] < winner))
					winner = order[k];
			for (size_t k = i; k < j; ++k)
				if (order[k] != winner) {
					trackOf[nodes[order[k]]] = NO_ID;
					++numDropped;
				}
		}
		return numDropped;
	}

private:
	uint32_t Local(uint32_t globalID) const {
		return (uint32_t)(std::lower_bound(nodes.begin(), nodes.end(), globalID) - nodes.begin());
	}
	float Distance2(uint32_t n1, uint32_t n2) const {
		const Point2f d = positions[n1] - positions[n2];
		return d.x*d.x + d.y*d.y;
	}
	// label the pieces the standing links leave, and count them on every keypoint
	unsigned LabelPieces() {
		piece.assign(nodes.size(), NO_ID);
		degree.assign(nodes.size(), 0);
		unsigned numPieces = 0;
		std::vector<uint32_t> stack;
		FOREACH(n, nodes) {
			if (piece[n] != NO_ID)
				continue;
			piece[n] = numPieces;
			stack.assign(1, (uint32_t)n);
			while (!stack.empty()) {
				const uint32_t x = stack.back();
				stack.pop_back();
				for (const auto& [y, l] : adjacency[x]) {
					if (!standing[l])
						continue;
					++degree[x];
					if (piece[y] == NO_ID) {
						piece[y] = numPieces;
						stack.push_back(y);
					}
				}
			}
			++numPieces;
		}
		return numPieces;
	}
	void SortByPieceAndImage() {
		order.resize(nodes.size());
		std::iota(order.begin(), order.end(), 0u);
		std::sort(order.begin(), order.end(), [this](uint32_t n1, uint32_t n2) {
			return piece[n1] < piece[n2] || (piece[n1] == piece[n2] && images[n1] < images[n2]);
		});
	}
	bool SamePieceAndImage(uint32_t n1, uint32_t n2) const {
		return piece[n1] == piece[n2] && images[n1] == images[n2];
	}
	// the two keypoints farthest apart of the first piece and image holding two that are not one
	// feature, or NO_ID when every piece is a track
	std::pair<uint32_t, uint32_t> FirstConflict(float mergeDist2) {
		SortByPieceAndImage();
		for (size_t i = 0, j; i < order.size(); i = j) {
			for (j = i + 1; j < order.size() && SamePieceAndImage(order[i], order[j]); ++j) ;
			std::pair<uint32_t, uint32_t> farthest(NO_ID, NO_ID);
			float farthestDist2 = mergeDist2;
			for (size_t k = i; k < j; ++k)
				for (size_t m = k + 1; m < j; ++m)
					if (const float dist2 = Distance2(order[k], order[m]); dist2 > farthestDist2) {
						farthestDist2 = dist2;
						farthest = {order[k], order[m]};
					}
			if (farthest.first != NO_ID)
				return farthest;
		}
		return {NO_ID, NO_ID};
	}

	std::vector<uint32_t> nodes;     // the keypoints, as global feature IDs, sorted
	std::vector<IIndex> images;      // their images
	std::vector<Point2f> positions;  // their positions
	std::vector<Link> links;         // the links, localized
	std::vector<bool> standing;      // the links not cut or vetoed
	std::vector<std::vector<std::pair<uint32_t, uint32_t>>> adjacency; // keypoint -> (neighbour, link)
	std::vector<uint32_t> piece;     // the piece of every keypoint over the standing links
	std::vector<unsigned> degree;    // the standing links of every keypoint
	std::vector<uint32_t> order;     // the keypoints sorted by piece and image
};

} // namespace

Unsigned32Arr SFM::BuildFeatureOffsets(const ImageArr& images)
{
	Unsigned32Arr featureOffsets(0, images.size() + 1);
	uint32_t globalID = 0;
	for (const Image& img : images) {
		featureOffsets.push_back(globalID);
		globalID += (uint32_t)img.keypoints.size();
	}
	featureOffsets.push_back(globalID); // sentinel
	return featureOffsets;
}

void SFM::BuildTracks(Scene& scene, float minPairWeight, const TrackConflictConfig& conflict)
{
	TD_TIMER_STARTD();
	scene.tracks.Release();

	// 1. Pre-compute feature offsets for O(1) global ID lookup: globalID = featureOffsets[imageID] + featureID
	const Unsigned32Arr featureOffsets = BuildFeatureOffsets(scene.images);
	const uint32_t globalID = featureOffsets.back();
	if (globalID == 0) {
		VERBOSE("error: no features found in images");
		return;
	}

	const auto NodeInfo = [&](uint32_t gid, IIndex& imgID, Point2f& position) {
		const auto it = std::upper_bound(featureOffsets.begin(), featureOffsets.end(), gid);
		ASSERT(it != featureOffsets.begin());
		imgID = static_cast<IIndex>(it - featureOffsets.begin() - 1);
		position = scene.images[imgID].keypoints[gid - featureOffsets[imgID]].pt;
	};

	// 2. Collect pairs above minPairWeight in decreasing composite weight order
	std::vector<uint32_t> usedPairs;
	unsigned numPairsSkippedWeightSupplemented = 0, numPairsSkippedWeightNotSupplemented = 0;
	FOREACH(p, scene.pairs) {
		const ImagePair& pair = scene.pairs[p];
		if (!pair.HasMatches())
			continue;
		if (!pair.ExceedsWeightBar(minPairWeight)) {
			++(pair.GetNumDenseInliers() > 0 ? numPairsSkippedWeightSupplemented : numPairsSkippedWeightNotSupplemented);
			continue;
		}
		usedPairs.push_back(p);
	}
	DEBUG("Pairs skipped below minPairWeight %g: %u supplemented, %u not supplemented",
		minPairWeight, numPairsSkippedWeightSupplemented, numPairsSkippedWeightNotSupplemented);
	const auto ForEachLink = [&](auto&& fn) {
		for (uint32_t p : usedPairs) {
			const ImagePair& pair = scene.pairs[p];
			const uint32_t offset1 = featureOffsets[pair.ID1];
			const uint32_t offset2 = featureOffsets[pair.ID2];
			FOREACHRAW(i, pair.GetNumTrackFormingMatches()) {
				const DMatch& m = pair.matches[i];
				ASSERT(m.queryIdx < scene.images[pair.ID1].keypoints.size());
				ASSERT(m.trainIdx < scene.images[pair.ID2].keypoints.size());
				fn(offset1 + m.queryIdx, offset2 + m.trainIdx, pair);
			}
		}
	};

	// 3. Union every track-forming link and detect components containing any image more than once
	DisjointSet<uint32_t> ds(globalID);
	ForEachLink([&](uint32_t id1, uint32_t id2, const ImagePair&) { ds.Union(id1, id2); });
	std::vector<uint32_t> trackOf(globalID);
	std::vector<bool> conflicted(globalID, false); // by the component's root
	{
		std::vector<uint32_t> rootImage(globalID, NO_ID);
		for (uint32_t imgID = 0; imgID < scene.images.size(); ++imgID) {
			for (uint32_t gid = featureOffsets[imgID]; gid < featureOffsets[imgID+1]; ++gid) {
				const uint32_t root = ds.Find(gid);
				trackOf[gid] = root;
				if (rootImage[root] == imgID)
					conflicted[root] = true;
				else
					rootImage[root] = imgID;
			}
		}
	}

	// 4. Settle conflicted components by cutting least-supported links or vetoing in pair order
	unsigned numConflicted = 0, numCut = 0, numVetoed = 0, numLarge = 0, numMerged = 0;
	uint32_t nextTrackID = globalID; // past every root, so the new tracks clash with none
	if (std::find(conflicted.begin(), conflicted.end(), true) != conflicted.end()) {
		struct Edge { uint32_t root, a, b, order; float weight; };
		std::vector<Edge> edges;
		uint32_t order = 0;
		ForEachLink([&](uint32_t id1, uint32_t id2, const ImagePair& pair) {
			const uint32_t root = ds.Find(id1);
			if (conflicted[root])
				edges.push_back({root, id1, id2, order, pair.GetCompositeWeight()});
			++order;
		});
		std::sort(edges.begin(), edges.end(), [](const Edge& e1, const Edge& e2) {
			return e1.root < e2.root || (e1.root == e2.root && e1.order < e2.order);
		});
		const float mergeDist2 = conflict.mergeDistance > 0.f ? SQUARE(conflict.mergeDistance) : -1.f;
		ConflictedComponent component;
		for (size_t i = 0, j; i < edges.size(); i = j) {
			for (j = i + 1; j < edges.size() && edges[j].root == edges[i].root; ++j) ;
			component.Set(edges.begin() + i, edges.begin() + j, NodeInfo);
			++numConflicted;
			if (conflict.cut && component.NumNodes() <= conflict.maxComponentSize) {
				numCut += component.Cut(mergeDist2);
			} else {
				numLarge += conflict.cut;
				numVetoed += component.Veto(mergeDist2);
			}
			numMerged += component.Assign(nextTrackID, trackOf);
		}
	}
	DEBUG("Components holding an image twice: %u of %u features; %u links cut, %u vetoed (%u components above %u keypoints), %u keypoints merged",
		numConflicted, globalID, numCut, numVetoed, numLarge, conflict.maxComponentSize, numMerged);

	// 5. Group observations by track ID using dense counting + index mapping (observations arrive
	//    sorted by (imageID, featureID) because we iterate images and features in ascending order)
	std::vector<uint32_t> counts(nextTrackID, 0);
	for (const uint32_t tid : trackOf)
		if (tid != NO_ID)
			++counts[tid];
	std::vector<uint32_t> trackIndex(nextTrackID, NO_ID);
	uint32_t numTracks = 0, numObservations = 0;
	for (uint32_t tid = 0; tid < nextTrackID; ++tid) {
		if (counts[tid] >= 2) {
			trackIndex[tid] = numTracks++;
			numObservations += counts[tid];
		}
	}
	scene.tracks.Resize(numTracks);
	for (uint32_t tid = 0; tid < nextTrackID; ++tid)
		if (trackIndex[tid] != NO_ID)
			scene.tracks[trackIndex[tid]].observations.Reserve(counts[tid]);
	for (uint32_t imgID = 0; imgID < scene.images.size(); ++imgID) {
		const uint32_t offset = featureOffsets[imgID];
		for (uint32_t gid = offset; gid < featureOffsets[imgID+1]; ++gid) {
			const uint32_t tid = trackOf[gid];
			if (tid == NO_ID)
				continue;
			const uint32_t idx = trackIndex[tid];
			if (idx != NO_ID)
				scene.tracks[idx].observations.emplace_back(imgID, gid - offset);
		}
	}
	DEBUG("Built %u tracks from %u observations and %u pairs (avg %.2f views/track) in %s",
	    scene.tracks.size(), globalID, (unsigned)usedPairs.size(),
	    numObservations / (float)MAXF(scene.tracks.size(), 1u), TD_TIMER_GET_FMT().c_str());
}


std::pair<float, bool> SFM::ComputeReprojectionErrorPixels(const Camera& camera, const Point3& Xcam, const Point2f& kpPt)
{
	const auto [projected, valid] = camera.Project(Xcam);
	if (!valid)
		return std::make_pair(0.f, false);
	return std::make_pair((float)norm(projected - Cast<REAL>(kpPt)), true);
}

std::pair<float, float> SFM::ComputeTracksMeanReprojectionError(const Scene& scene, const ObservationFilter& isUsed)
{
	double sumAngularError = 0.0, sumPixelError = 0.0;
	uint32_t numTracks = 0, numErrors = 0, numLeftOut = 0;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const Image& img = scene.images[obs.imageID];
			ASSERT(img.IsValid());
			ASSERT(obs.featureID < img.keypoints.size());
			const Point3 Xcam = img.TransformPointW2C(track.position);
			const auto [pixelError, valid] = ComputeReprojectionErrorPixels(*img.pCamera, Xcam, img.keypoints[obs.featureID].pt);
			if (!valid)
				continue;
			if (isUsed && !isUsed(obs.imageID, obs.featureID)) {
				++numLeftOut;
				continue;
			}
			sumPixelError += pixelError;
			const Point3 observedRay = img.pCamera->UnprojectNormalized(Cast<REAL>(img.keypoints[obs.featureID].pt));
			sumAngularError += ComputeAngle(observedRay.ptr(), Xcam.ptr());
			++numErrors;
		}
		++numTracks;
	}
	double avgAngular = 0.0, avgPixel = 0.0;
	if (numErrors > 0) {
		avgAngular = R2D(ACOS(sumAngularError / numErrors));
		avgPixel = sumPixelError / numErrors;
	}
	if (!isUsed) {
		DEBUG_EXTRA("Mean reprojection error: %.2f pixels (%.2f deg) from %u tracks (%.2f views/track)",
			avgPixel, avgAngular, numTracks, numErrors / (double)MAXF(numTracks, 1u));
	} else {
		DEBUG_EXTRA("Mean reprojection error: %.2f pixels (%.2f deg) from %u tracks (%.2f views/track), over the %u observations the solve used (the budget left out %u more of these tracks)",
			avgPixel, avgAngular, numTracks, numErrors / (double)MAXF(numTracks, 1u), numErrors, numLeftOut);
	}
	return std::make_pair(avgPixel, avgAngular);
}

void SFM::ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense)
{
	FloatArr errorsDescribed, errorsDense;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const Image& img = scene.images[obs.imageID];
			if (!img.IsValid())
				continue;
			const auto [pixelError, valid] = ComputeReprojectionErrorPixels(img, track.position, obs.featureID);
			if (!valid)
				continue;
			(img.IsDenseKeypoint(obs.featureID) ? errorsDense : errorsDescribed).push_back(pixelError);
		}
	}
	numDescribed = errorsDescribed.size();
	numDense = errorsDense.size();
	sigmaDescribed = errorsDescribed.empty() ? 0.0 : (double)errorsDescribed.GetNth(errorsDescribed.size() / 2);
	sigmaDense = errorsDense.empty() ? 0.0 : (double)errorsDense.GetNth(errorsDense.size() / 2);
}

std::pair<float, float> SFM::FilterTracks(Scene& scene,
	float maxReprojErrorPixels, float minAngleDegrees,
	float multDepthNear, float multDepthFar)
{
	const float minAngleRadians = D2R(minAngleDegrees);

	MeanStdMinMax<REAL> trackCompletenessStats;
	double sumAngularError = 0.0, sumPixelError = 0.0;
	uint32_t numInlierTracks = 0, numInlierErrors = 0;
	uint32_t numDenseKept = 0, numDenseDropped = 0, numDescribedKept = 0, numDescribedDropped = 0;
	FloatArr dists(0, MAXF(scene.status.nTracks, 100u));
	for (Track& track : scene.tracks) {
		track.numInliers = 0;
		if (!track.IsValid())
			continue;
		if (track.GetNumPosedObservations(scene.images) < Track::MIN_VIEWS)
			continue;

		double sumTrackAngularError = 0.0, sumTrackPixelError = 0.0, sumTrackDist = 0.0;
		FOREACH(obsIdx, track.observations) {
			const Observation& obs = track.observations[obsIdx];
			const Image& img = scene.images[obs.imageID];
			ASSERT(img.HasCamera());
			if (!img.IsValid())
				continue;
			const Point3 Xcam = img.TransformPointW2C(track.position);
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			const Point3 observedRay = img.pCamera->UnprojectNormalized(Cast<REAL>(kp.pt));
			const REAL cosAngularError = ComputeAngle(observedRay.ptr(), Xcam.ptr());
			const bool bDense = img.IsDenseKeypoint(obs.featureID);
			const REAL minCosAngularError = COS(img.pCamera->PixelErrorToAngular(maxReprojErrorPixels));
			if (cosAngularError < minCosAngularError) {
				++(bDense ? numDenseDropped : numDescribedDropped);
				continue;
			}
			++(bDense ? numDenseKept : numDescribedKept);
			const auto [pixelError, projValid] = ComputeReprojectionErrorPixels(*img.pCamera, Xcam, kp.pt);
			ASSERT(projValid);
			if (track.numInliers < obsIdx)
				std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			sumTrackPixelError += pixelError;
			sumTrackAngularError += cosAngularError;
			sumTrackDist += (float)norm(Xcam);
			++track.numInliers;
		}

		// Track must have at least 2 inlier observations to be considered inlier
		if (!track.IsInlier())
			continue;

		// the widest angle between two inlier rays must reach the minimum triangulation angle
		const float maxAngle = track.ComputeMaxAngleBetweenRays(scene.images);
		if (maxAngle < minAngleRadians) {
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
	// the dense/described split of what this bar dropped: the two rates are what says whether the
	// dense observations are being filtered harder than the sparse ones at the same threshold
	if (numDenseKept + numDenseDropped > 0) {
		DEBUG("Observations filtered: %u/%u dense dropped (%.1f%%), %u/%u described dropped (%.1f%%)",
			numDenseDropped, numDenseKept + numDenseDropped,
			100.0 * numDenseDropped / (double)(numDenseKept + numDenseDropped),
			numDescribedDropped, numDescribedKept + numDescribedDropped,
			numDescribedKept + numDescribedDropped > 0 ?
				100.0 * numDescribedDropped / (double)(numDescribedKept + numDescribedDropped) : 0.0);
	}
	return std::make_pair(avgPixel, avgAngular);
}


namespace {

// Per-image inlier-observation index in CSR layout (avoids the O(images x tracks) membership
// scan per image). Built once over the inlier prefix of
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

struct PairIdxCount {
	PairIdx pairIdx;
	unsigned count;
};

// Deterministic sort-based covisibility: every track with >= minInliersPerTrack inliers
// contributes one shared point to each unordered pair of its inlier images. Output is sorted by (i,j).
static void BuildCovisEdges(const Scene& scene, unsigned minCovisibilityCount,
	uint8_t minInliersPerTrack, CLISTDEF0(PairIdxCount)& edges)
{
	std::vector<uint64_t> keys;
	size_t est = 0;
	for (const Track& t : scene.tracks)
		if (t.IsInlier(minInliersPerTrack))
			est += (size_t)t.numInliers * (t.numInliers - 1) / 2;
	keys.reserve(MINF(est, (size_t)64 * 1024 * 1024));
	for (const Track& t : scene.tracks) {
		if (!t.IsInlier(minInliersPerTrack))
			continue;
		for (uint8_t a = 0; a < t.numInliers; ++a) {
			const uint32_t ia = t.observations[a].imageID;
			for (uint8_t b = a + 1; b < t.numInliers; ++b) {
				const uint32_t ib = t.observations[b].imageID;
				keys.push_back(MakePairIdx(ia, ib).idx);
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
		if (count >= minCovisibilityCount)
			edges.push_back({ PairIdx(keys[s]), count });
		s = e;
	}
}

// How far the pose the model gives an image lies from what one of its verified pairs measured, seen
// from the pair's other image (PoseLink.h)
static PairDisagreement MeasurePairAgainstModel(const Scene& scene, const ImagePair& pair, IIndex imageID, IIndex neighborID)
{
	return MeasurePairDisagreement(pair, scene.images[imageID], scene.images[neighborID], neighborID);
}

// The verified pairs joining a component the largest-component pass is about to cut to the
// component it keeps, with the model's agreement on each: the inlier weight of the pairs that agree
// with the pose the model gives the cut image and of those that put it elsewhere. Only pairs of at
// least minInliers weighted inliers are read: below it a pair says nothing either way.
struct CutJunction {
	float agreeWeight{0.f}, disagreeWeight{0.f}; // weighted inliers of the pairs on either side
	bool Contradicts() const { return disagreeWeight > agreeWeight; }
};
typedef std::unordered_map<IIndex, CutJunction> CutJunctionMap; // by component root

static CutJunctionMap MeasureCutJunctions(const Scene& scene, const std::vector<IIndex>& rootOf,
	IIndex largestRoot, float maxAgreementAngle, unsigned minInliers)
{
	CutJunctionMap junctions;
	for (const ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair) || pair.GetNumWeightedInliers() < minInliers)
			continue;
		const IIndex r1 = rootOf[pair.ID1], r2 = rootOf[pair.ID2];
		if (r1 == NO_ID || r2 == NO_ID || (r1 == largestRoot) == (r2 == largestRoot))
			continue; // both sides kept, or neither
		const IIndex imageID = r1 == largestRoot ? pair.ID2 : pair.ID1;
		const IIndex neighborID = r1 == largestRoot ? pair.ID1 : pair.ID2;
		CutJunction& j = junctions[rootOf[imageID]];
		const PairDisagreement d = MeasurePairAgainstModel(scene, pair, imageID, neighborID);
		(d.Within(maxAgreementAngle) ? j.agreeWeight : j.disagreeWeight) += (float)pair.GetNumWeightedInliers();
	}
	return junctions;
}


} // namespace


// Prunes weakly connected images using spatial coverage, median triangulation angle,
// largest connected covisibility component, and absolute k-core peeling.
RemovedImages SFM::FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount,
	float minObservationArea,
	float minTriangulationAngle,
	unsigned minCovisDegree)
{
	TD_TIMER_STARTD();
	RemovedImages removed;
	IIndexArr& filteredIDs = removed.all;
	IIndexArr& contradictingIDs = removed.contradicting;

	constexpr int gridSize = 10; // 10x10 grid
	constexpr uint8_t minInliersPerTrack = 3; // covisibility only counts tracks with >= 3 inliers
	constexpr unsigned minContradictionInliers = 15;
	constexpr float maxContradictionAngle = 5.f;
	ImageObsCSR csr;
	BuildImageObsCSR(scene, csr);

	// Tier 1 (spatial coverage) and Tier 2 (median triangulation angle) evaluated on the entry state
	const unsigned minNumObservationsForGrid = ROUND2INT<unsigned>(SQUARE(gridSize) * minObservationArea);
	MeanStdMinMax<REAL> coverageStats;
	TMatrix<uint8_t, gridSize, gridSize> occupiedCells;
	const float minAngleRadians = D2R(minTriangulationAngle);
	MeanStdMinMax<REAL> angleStats;
	std::vector<uint8_t> tierDrop(scene.images.size(), 0); // 0=keep, 1=tier1, 2=tier2
	FloatArr obsAngles, medAngles;
	FOREACH(imgIdx, scene.images) {
		const Image& image = scene.images[imgIdx];
		if (!image.IsValid())
			continue;
		const float cellWidth = (float)image.pCamera->GetWidth() / gridSize;
		const float cellHeight = (float)image.pCamera->GetHeight() / gridSize;
		unsigned numOccupiedCells = 0;
		occupiedCells.memset(0);
		medAngles.clear();
		for (uint32_t p = csr.offset[imgIdx]; p < csr.offset[imgIdx + 1]; ++p) {
			const Track& track = scene.tracks[csr.track[p]];
			const cv::KeyPoint& kp = image.keypoints[csr.feat[p]];
			const int cellX = CLAMP((int)(kp.pt.x / cellWidth), 0, gridSize - 1);
			const int cellY = CLAMP((int)(kp.pt.y / cellHeight), 0, gridSize - 1);
			uint8_t& cell = occupiedCells(cellX, cellY);
			if (cell == 0) { cell = 1; ++numOccupiedCells; }
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
				medAngles.push_back(obsAngles.GetNth((obsAngles.size() - 1) / 2));
		}
		if (numOccupiedCells < minNumObservationsForGrid) {
			DEBUG_EXTRA("warning: image %u (`%s`) invalidated for low spatial distribution (%.2f%% < %.2f%% cells occupied), %u visible tracks",
				imgIdx, Util::getFileName(image.fileName).c_str(), (float)numOccupiedCells / SQUARE(gridSize) * 100.f, (float)minObservationArea * 100.f, (unsigned)medAngles.size());
			tierDrop[imgIdx] = 1;
			continue;
		}
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

	{
		IIndexArr tierDropIDs;
		FOREACH(imgIdx, scene.images)
			if (tierDrop[imgIdx])
				tierDropIDs.push_back(imgIdx);
		scene.InvalidateImages(tierDropIDs);
		for (const IIndex id : tierDropIDs)
			filteredIDs.push_back(id);
	}

	// Build covisibility graph over surviving inlier tracks
	CLISTDEF0(PairIdxCount) edgeWeights;
	BuildCovisEdges(scene, minCovisibilityCount, minInliersPerTrack, edgeWeights);
	DEBUG_EXTRA("Established visibility graph with %u/%u images and %u image pairs",
		scene.status.nCalibratedImages, scene.images.size(), (unsigned)edgeWeights.size());
	if (edgeWeights.empty()) {
		DEBUG("error: no valid image pairs found for clustering");
		return removed;
	}

	// Step 3: Keep only the largest connected component and invalidate the rest
	DisjointSet<IIndex> ds(scene.images.size());
	for (const PairIdxCount& edge : edgeWeights)
		ds.Union(edge.pairIdx.i, edge.pairIdx.j);
	const auto InvalidateImagesIfNotInLargestComponent =
		[&scene, &filteredIDs, &contradictingIDs, &ds]() {
		const std::unordered_map<IIndex, unsigned> componentSizes = ds.CompressAllPaths().GetComponentSizes();
		IIndex largestComponentRoot = NO_ID;
		unsigned maxSize = 0;
		for (const auto& [root, size] : componentSizes)
			if (size > maxSize || (size == maxSize && root < largestComponentRoot)) {
				maxSize = size;
				largestComponentRoot = root;
			}
		std::vector<IIndex> rootOf(scene.images.size(), NO_ID);
		FOREACH(imgIdx, scene.images)
			if (scene.images[imgIdx].IsValid())
				rootOf[imgIdx] = ds.Find(imgIdx);
		const CutJunctionMap cutJunctions = MeasureCutJunctions(scene, rootOf, largestComponentRoot, maxContradictionAngle, minContradictionInliers);
		IIndexArr dropIDs;
		FOREACH(imgIdx, scene.images) {
			if (scene.images[imgIdx].IsValid() && largestComponentRoot != rootOf[imgIdx]) {
				DEBUG_EXTRA("warning: image %u (`%s`) invalidated for not in largest connected component",
					imgIdx, Util::getFileName(scene.images[imgIdx].fileName).c_str());
				dropIDs.push_back(imgIdx);
				const auto it = cutJunctions.find(rootOf[imgIdx]);
				if (it != cutJunctions.end() && it->second.Contradicts())
					contradictingIDs.push_back(imgIdx);
			}
		}
		scene.InvalidateImages(dropIDs);
		for (const IIndex id : dropIDs)
			filteredIDs.push_back(id);
		DEBUG_EXTRA("Kept %u images in largest connected component (from %u components)",
			scene.status.nCalibratedImages, (unsigned)componentSizes.size());
	};
	InvalidateImagesIfNotInLargestComponent();

	RFOREACH(i, edgeWeights) {
		const PairIdxCount& edge = edgeWeights[i];
		if (!scene.images[edge.pairIdx.i].IsValid() ||
		    !scene.images[edge.pairIdx.j].IsValid())
			edgeWeights.RemoveAt(i);
	}
	if (edgeWeights.empty()) {
		DEBUG("error: no edge weights available for clustering");
		return removed;
	}

	// Step 4: Absolute k-core peel + largest connected component
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

	ds.Reset(scene.images.size());
	for (const PairIdxCount& edge : edgeWeights)
		if (scene.images[edge.pairIdx.i].IsValid() && scene.images[edge.pairIdx.j].IsValid())
			ds.Union(edge.pairIdx.i, edge.pairIdx.j);
	InvalidateImagesIfNotInLargestComponent();

	DEBUG("Filtered %u/%u weakly connected images in %s",
		filteredIDs.size(), scene.status.nCalibratedImages+filteredIDs.size(), TD_TIMER_GET_FMT().c_str());
	return removed;
}
/*----------------------------------------------------------------*/
