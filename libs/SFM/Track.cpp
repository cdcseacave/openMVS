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
	// them: the fewest triangles through it, then the lighter pair, then the later link. The
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
			if (via[x2] == NO_ID)
				break;
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

void SFM::BuildTracks(Scene& scene, float minPairWeight, const TrackConflictConfig& conflict)
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

	// the image of a global feature ID, and the position of the keypoint
	const auto NodeInfo = [&](uint32_t gid, IIndex& imgID, Point2f& position) {
		const auto it = std::upper_bound(featureOffsets.begin(), featureOffsets.end(), gid);
		ASSERT(it != featureOffsets.begin());
		imgID = static_cast<IIndex>(it - featureOffsets.begin() - 1);
		position = scene.images[imgID].keypoints[gid - featureOffsets[imgID]].pt;
	};

	// 2. The pairs the tracks are built from: those with matches above the weight bar, in the order
	// they arrive, which is decreasing composite weight: ComputePairsWeights (PairsWeighting.cpp,
	// step 5) sorts scene.pairs at the end of every weighting, and matching, the triplet filter and
	// the dense supplement each end in one.
	// An infused pair is judged on the same composite weight as every other pair, its dense matches
	// counted at the dense observation weight (GetNumWeightedInliers) rather than at 1 or at 0: a
	// coverage fill-in is evidence, but not sub-pixel evidence. One that still falls under
	// minPairWeight is a correct drop -- but correct-and-uncounted is how the last silent defect
	// stayed silent, so split the skip count by supplemented vs not rather than reporting one number.
	std::vector<uint32_t> usedPairs;
	unsigned numPairsSkippedWeightSupplemented = 0, numPairsSkippedWeightNotSupplemented = 0;
	FOREACH(p, scene.pairs) {
		const ImagePair& pair = scene.pairs[p];
		if (!pair.HasMatches())
			continue;
		if (minPairWeight >= 0 && pair.GetCompositeWeight() <= minPairWeight) {
			++(pair.GetNumDenseInliers() > 0 ? numPairsSkippedWeightSupplemented : numPairsSkippedWeightNotSupplemented);
			continue;
		}
		usedPairs.push_back(p);
	}
	DEBUG("Pairs skipped below minPairWeight %g: %u supplemented, %u not supplemented",
		minPairWeight, numPairsSkippedWeightSupplemented, numPairsSkippedWeightNotSupplemented);
	// Their track-forming matches, as links between global feature IDs, in that order. Only the
	// track-forming matches contribute to tracks: the pair's verified sparse inliers plus its dense
	// supplement, and nothing past them -- what follows are RANSAC inliers the strict filter
	// deliberately rejected. Not GetNumFilteredInliers(), which is the sparse count alone: with the
	// supplement outside it a loop bounded by it would union no dense match at all and dense
	// supplementation would silently do nothing.
	const auto ForEachLink = [&](auto&& fn) {
		for (uint32_t p : usedPairs) {
			const ImagePair& pair = scene.pairs[p];
			const uint32_t offset1 = featureOffsets[pair.ID1];
			const uint32_t offset2 = featureOffsets[pair.ID2];
			// clamped by the array itself: the loop dereferences matches[i] and only inspects the
			// DMatch on the next line, so a count that outran `matches` would be read out of bounds
			// before either assertion below could look at it (ImagePair's accessors assert the
			// arithmetic in Debug; this keeps release builds in range too)
			FOREACHRAW(i, MINF(pair.GetNumTrackFormingMatches(), (unsigned)pair.matches.size())) {
				const DMatch& m = pair.matches[i];
				ASSERT(m.queryIdx < scene.images[pair.ID1].keypoints.size());
				ASSERT(m.trainIdx < scene.images[pair.ID2].keypoints.size());
				fn(offset1 + m.queryIdx, offset2 + m.trainIdx, pair);
			}
		}
	};

	// 3. Union every link, plain: a component is every keypoint the matches connect, right or wrong.
	// A component holding one image twice is not a track -- one of its links is wrong, or the two
	// keypoints are one feature detected twice -- and is settled on its own links in step 4; the
	// wrong link is found by what the other links say about it, not by which pair arrived first.
	DisjointSet<uint32_t> ds(globalID);
	ForEachLink([&](uint32_t id1, uint32_t id2, const ImagePair&) { ds.Union(id1, id2); });
	// the track of every feature: its component, until step 4 settles the conflicted ones
	std::vector<uint32_t> trackOf(globalID);
	std::vector<bool> conflicted(globalID, false); // by the component's root
	{
		// the features arrive image by image, so a root that sees the same image twice in a row
		// holds two of its keypoints
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

	// 4. Settle every conflicted component on its own links (ConflictedComponent): the cut of the
	// least-supported link on the path between the two keypoints, which keeps the side the
	// triangles corroborate, or the veto of the union in pair order, which keeps whichever link
	// arrived first, when the cut is off or the component is too large for it; either way two
	// keypoints of one image within the merge distance are one feature and stay one track.
	// On alameda (1734 images, 12.7M links) the veto keeps the wrong keypoint in 22% of the
	// conflicts a triangulation can judge, the cut with the merge in 17%.
	unsigned numConflicted = 0, numCut = 0, numVetoed = 0, numLarge = 0, numMerged = 0;
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
		uint32_t nextTrackID = globalID; // past every root, so the new tracks clash with none
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

	// 5. Group observations by track
	std::map<uint32_t, ObservationArr> tracks;
	for (uint32_t imgID = 0; imgID < scene.images.size(); ++imgID) {
		const uint32_t offset = featureOffsets[imgID];
		for (uint32_t gid = offset; gid < featureOffsets[imgID+1]; ++gid)
			if (trackOf[gid] != NO_ID)
				tracks[trackOf[gid]].emplace_back(imgID, gid - offset);
	}

	// 6. Filter tracks (minimum 2 views) and add to scene
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
	    scene.tracks.size(), globalID, (unsigned)usedPairs.size(),
	    numObservations / (float)MAXF(scene.tracks.size(), 1u), TD_TIMER_GET_FMT().c_str());

	// Dense track-length histogram, on a scene that was dense-supplemented. Reported as a
	// distribution rather than summarized as a mean in either direction: a length above 2 is the
	// product of the exact-position reuse PairsMatcher::FilterRedundantKeypoints performs between
	// pairs sharing an image, so a histogram sitting entirely at 2 means that reuse never fired,
	// which is itself a finding (and the expected output with --release-descriptors false, where the
	// filter does not run at all). A track is dense-only when every one of its observations is a
	// dense keypoint, mixed when it carries both kinds.
	if (std::any_of(scene.images.begin(), scene.images.end(), [](const Image& img) { return img.HasDenseKeypoints(); })) {
		unsigned hist[5] = {0, 0, 0, 0, 0}; // dense-only lengths 2, 3, 4, 5-9, 10+
		unsigned numDenseOnly = 0, numMixed = 0;
		size_t numDenseObservations = 0;
		for (const Track& track : scene.tracks) {
			unsigned numDense = 0;
			for (const Observation& obs : track.observations)
				numDense += scene.images[obs.imageID].IsDenseKeypoint(obs.featureID);
			numDenseObservations += numDense;
			if (numDense == 0)
				continue;
			if (numDense < track.observations.size()) {
				++numMixed;
				continue;
			}
			++numDenseOnly;
			const size_t length = track.observations.size();
			++hist[length <= 4 ? length - 2 : (length <= 9 ? 3 : 4)];
		}
		DEBUG("Dense track lengths: %u at 2, %u at 3, %u at 4, %u at 5-9, %u at 10+ "
			"(%u dense-only tracks, %u mixed, %zu dense observations of %u)",
			hist[0], hist[1], hist[2], hist[3], hist[4], numDenseOnly, numMixed,
			numDenseObservations, numObservations);
	}

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
		// exactly the matches ForEachLink above dereferences, so the bound must be the same one --
		// clamped by the array for the same reason it is there
		FOREACHRAW(i, MINF(pair.GetNumTrackFormingMatches(), (unsigned)pair.matches.size())) {
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


std::pair<float, bool> SFM::ComputeReprojectionErrorPixels(const Camera& camera, const Point3& Xcam, const Point2f& kpPt)
{
	const auto [projected, valid] = camera.Project(Xcam);
	if (!valid)
		return std::make_pair(0.f, false);
	return std::make_pair((float)norm(projected - Cast<REAL>(kpPt)), true);
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

void SFM::ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense)
{
	std::vector<float> errorsDescribed, errorsDense;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const Image& img = scene.images[obs.imageID];
			if (!img.IsValid())
				continue;
			ASSERT(obs.featureID < img.keypoints.size());
			const Point3 Xcam = img.TransformPointW2C(track.position);
			const auto [pixelError, valid] =
				ComputeReprojectionErrorPixels(*img.pCamera, Xcam, img.keypoints[obs.featureID].pt);
			if (!valid)
				continue;
			(img.IsDenseKeypoint(obs.featureID) ? errorsDense : errorsDescribed).push_back(pixelError);
		}
	}
	const auto Median = [](std::vector<float>& errors) {
		if (errors.empty())
			return 0.0;
		const size_t half = errors.size()/2;
		std::nth_element(errors.begin(), errors.begin() + half, errors.end());
		return (double)errors[half];
	};
	numDescribed = errorsDescribed.size();
	numDense = errorsDense.size();
	sigmaDescribed = Median(errorsDescribed);
	sigmaDense = Median(errorsDense);
}

std::pair<float, float> SFM::FilterTracks(Scene& scene,
	float maxReprojErrorPixels, float minAngleDegrees,
	float multDepthNear, float multDepthFar, float denseReprojErrorFactor)
{
	const float minAngleRadians = D2R(minAngleDegrees);
	const float maxDenseReprojErrorPixels = maxReprojErrorPixels * denseReprojErrorFactor;

	// Process each track
	MeanStdMinMax<REAL> trackCompletenessStats;
	double sumAngularError = 0.0, sumPixelError = 0.0;
	uint32_t numInlierTracks = 0, numInlierErrors = 0;
	// A down-weighted dense observation still faces this same reprojection bar, so it may be
	// filtered at a different rate than a described one. Counted here, and reported below when the
	// scene carries any, so that rate is visible rather than moving silently.
	uint32_t numDenseKept = 0, numDenseDropped = 0, numDescribedKept = 0, numDescribedDropped = 0;
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
			const bool bDense = img.IsDenseKeypoint(obs.featureID);
			const REAL minCosAngularError = COS(img.pCamera->PixelErrorToAngular(bDense ? maxDenseReprojErrorPixels : maxReprojErrorPixels));
			if (cosAngularError < minCosAngularError) {
				++(bDense ? numDenseDropped : numDescribedDropped);
				continue; // outlier or behind the camera observation
			}
			++(bDense ? numDenseKept : numDescribedKept);
			// Accepted — pixel-error stats, via the shared helper (well-defined now: cheirality passed above)
			const auto [pixelError, projValid] = ComputeReprojectionErrorPixels(*img.pCamera, Xcam, kp.pt);
			ASSERT(projValid);
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

// How far the pose the model gives an image lies from what one of its verified pairs measured, seen
// from the pair's other image (PoseLink.h)
static PairDisagreement MeasurePairAgainstModel(const Scene& scene, const ImagePair& pair, IIndex imageID, IIndex neighborID)
{
	return MeasurePairDisagreement(pair, scene.images[imageID], scene.images[neighborID], neighborID);
}

// The verified pairs joining a component the largest-component pass is about to cut to the
// component it keeps, with the model's agreement on each: how many agree with the pose the model
// gives the cut image and how many put it elsewhere, and the inlier weight on either side. Only
// pairs of at least minInliers weighted inliers are read, as the corroboration reads them.
struct CutJunction {
	unsigned numPairs{0}, numWeightedInliers{0}, numAgree{0}, numRotationOff{0}, numDirectionOff{0};
	float agreeWeight{0.f}, disagreeWeight{0.f}; // weighted inliers of the pairs on either side
	FloatArr rotations, directions; // disagreement of every pair across the cut, in degrees
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
		++j.numPairs;
		j.numWeightedInliers += pair.GetNumWeightedInliers();
		const PairDisagreement d = MeasurePairAgainstModel(scene, pair, imageID, neighborID);
		j.rotations.push_back((float)d.rotation);
		if (d.HasDirection())
			j.directions.push_back((float)d.direction);
		if (d.rotation > maxAgreementAngle)
			++j.numRotationOff;
		else if (d.HasDirection() && d.direction > d.DirectionTolerance(maxAgreementAngle))
			++j.numDirectionOff;
		else
			++j.numAgree;
		(d.Within(maxAgreementAngle) ? j.agreeWeight : j.disagreeWeight) += (float)pair.GetNumWeightedInliers();
	}
	return junctions;
}

// What still joins a component the largest-component pass is about to cut to the component it
// keeps: the tracks seen on both sides, the strongest image pair among them, and the verified
// pairs across the cut with the model's agreement on each. Reported for every component of at
// least minComponentImages images, so a whole block leaving the model is explained, not just
// counted.
static void ReportCutComponents(const Scene& scene, const std::vector<IIndex>& rootOf, IIndex largestRoot,
	CutJunctionMap& cutJunctions, uint8_t minInliersPerTrack, unsigned minComponentImages)
{
	struct Junction {
		unsigned numImages{0}, minID{NO_ID}, maxID{0};
		unsigned numTracks{0}, numInlierTracks{0};
		std::unordered_map<uint64_t, unsigned> sharedTracks; // per image pair across the cut
	};
	std::unordered_map<IIndex, Junction> junctions; // by component root
	unsigned numKept = 0;
	FOREACH(imgIdx, scene.images) {
		const IIndex root = rootOf[imgIdx];
		if (root == NO_ID)
			continue;
		if (root == largestRoot) {
			++numKept;
			continue;
		}
		Junction& j = junctions[root];
		++j.numImages;
		j.minID = MINF(j.minID, (unsigned)imgIdx);
		j.maxID = MAXF(j.maxID, (unsigned)imgIdx);
	}
	for (auto it = junctions.begin(); it != junctions.end(); )
		it = it->second.numImages < minComponentImages ? junctions.erase(it) : std::next(it);
	if (junctions.empty())
		return;
	// the tracks seen on both sides of a cut, inlier or not, and the image pairs the inlier ones join
	std::vector<IIndex> cutRoots;
	for (const Track& track : scene.tracks) {
		if (!track.IsValid())
			continue;
		bool keptSide = false;
		cutRoots.clear();
		for (const Observation& obs : track.observations) {
			const IIndex root = rootOf[obs.imageID];
			if (root == largestRoot)
				keptSide = true;
			else if (root != NO_ID && junctions.count(root) && std::find(cutRoots.begin(), cutRoots.end(), root) == cutRoots.end())
				cutRoots.push_back(root);
		}
		if (!keptSide)
			continue;
		for (const IIndex root : cutRoots) {
			Junction& j = junctions[root];
			++j.numTracks;
			if (!track.IsInlier(minInliersPerTrack))
				continue;
			bool inlierBothSides = false;
			for (uint8_t a = 0; a < track.numInliers && !inlierBothSides; ++a) {
				const IIndex ia = track.observations[a].imageID;
				if (rootOf[ia] != root)
					continue;
				for (uint8_t b = 0; b < track.numInliers; ++b) {
					const IIndex ib = track.observations[b].imageID;
					if (rootOf[ib] != largestRoot)
						continue;
					inlierBothSides = true;
					++j.sharedTracks[((uint64_t)MINF(ia, ib) << 32) | MAXF(ia, ib)];
				}
			}
			if (inlierBothSides)
				++j.numInlierTracks;
		}
	}
	std::vector<IIndex> roots;
	for (const auto& [root, j] : junctions)
		roots.push_back(root);
	std::sort(roots.begin(), roots.end());
	for (const IIndex root : roots) {
		const Junction& j = junctions[root];
		unsigned strongestPair = 0;
		for (const auto& [key, count] : j.sharedTracks)
			strongestPair = MAXF(strongestPair, count);
		CutJunction& c = cutJunctions[root]; // GetMedian sorts in place
		DEBUG_EXTRA("Component of %u images (%u..%u) cut from the kept %u: %u tracks seen on both sides, %u with inliers on both, "
			"the strongest image pair sharing %u; %u verified pairs across (%u weighted inliers): %u agree with the model, "
			"%u off in rotation, %u off in direction; median disagreement %.1f deg in rotation, %.1f deg in direction%s",
			j.numImages, j.minID, j.maxID, numKept, j.numTracks, j.numInlierTracks, strongestPair,
			c.numPairs, c.numWeightedInliers, c.numAgree, c.numRotationOff, c.numDirectionOff,
			c.rotations.empty() ? -1.f : c.rotations.GetMedian(), c.directions.empty() ? -1.f : c.directions.GetMedian(),
			c.Contradicts() ? "; contradicts the model" : "");
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
// CORROBORATION (maxCorroborationAngle > 0, on by default):
//    - Stages 1 to 5 all read an image's triangulated structure: its spread of points, its
//      triangulation angles, its shared tracks. An image that entered the model through two-view
//      geometry alone has none of that -- its tracks have two views, so the covisibility graph,
//      which counts tracks of three, does not even hold an edge for it -- and every stage therefore
//      drops it for want of evidence it was never going to have.
//    - The evidence that does exist for such an image is the verified pairs joining it to images
//      already settled. When two of them agree with the pose the model gives it, in the relative
//      rotation and in the direction of the baseline alike, the image is corroborated and exempt
//      from the tier verdicts, the largest-CC pass and the peel.
//    - Iterative: round 0's settled images are those that pass the tier verdicts and lie in the
//      largest component of the entry state on their own; a later round settles whatever two pairs
//      newly reach into that set, so a chain of two-view registrations is followed one verified link
//      at a time. No image is settled by another its own round is still deciding, so a round only
//      ever reaches back into what an earlier round already settled -- a chain cannot lift itself.
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
//
// maxCorroborationAngle (default 5 = enabled):
//   - Keep an image that two verified pairs to distinct settled images agree with, within this angle
//     in both the relative rotation and the direction of the baseline. Settled starts as the images
//     the filter keeps on their own merits and grows a round at a time as pairs reach further images,
//     so a chain of two-view registrations is rescued as far as it reaches back into the model. Every
//     stage above judges an image by its triangulated structure, which an image joined to the model
//     by two-view geometry alone does not have; this is the one rule that reads the two-view evidence
//     directly.
//   - 0 disables the rescue, leaving every image to the stages above.
RemovedImages SFM::FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount,
	float minObservationArea,
	float minTriangulationAngle,
	unsigned minCovisDegree,
	float maxPoseInconsistencyAngle,
	float maxReprojErrorPixels,
	float maxCorroborationAngle)
{
	TD_TIMER_STARTD();
	struct PairIdxCount {
		PairIdx pairIdx;
		unsigned count;
	};
	RemovedImages removed;
	IIndexArr& filteredIDs = removed.all;
	IIndexArr& contradictingIDs = removed.contradicting;

	// One shared, entry-state CSR of per-image inlier observations: kills the former
	// O(images x tracks) membership scan that the tier pre-filters ran per image.
	constexpr int gridSize = 10; // 10x10 grid
	constexpr uint8_t minInliersPerTrack = 3; // covisibility only counts tracks with >= 3 inliers
	// a verified pair's word about a pose is read only past this many weighted inliers: below it a
	// pair says nothing either way
	constexpr unsigned minCorroborationInliers = 15;
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

	// Images corroborated by two consistent pairs (optional; off when maxCorroborationAngle <= 0).
	// An image joined to the model by two-view geometry alone holds no track of three views once the
	// angle and reprojection filters have run, so it has no covisibility edge and often no spread of
	// triangulated points either: every stage of this filter judges an image by structure that such
	// an image simply does not have. Its pose is nevertheless vouched for by verified pairs, so it is
	// kept when at least two of them join it to distinct settled images and, for each of those pairs,
	// the model agrees with what the pair measured -- the relative rotation within
	// maxCorroborationAngle of the pair's, and the direction of the model's baseline within the same
	// angle of the one the pair's relative pose gives. A pair with no baseline of its own fixes no
	// direction, so it is judged on the rotation alone.
	// Settled starts, in round 0, as the images that pass the tier verdicts and lie in the largest
	// component of the covisibility graph of the state this call was entered with -- images this
	// filter keeps on their own merits, whatever the pairs say. Every later round settles whatever two
	// pairs newly reach into that set and stops when a round settles nothing, so a chain of two-view
	// registrations is followed one verified link at a time; an image is never settled by another its
	// own round is still deciding, so a round only reaches back into what an earlier round already
	// settled and a chain cannot lift itself.
	std::vector<uint8_t> corroborated(scene.images.size(), 0);
	std::vector<uint8_t> keptByComponent(scene.images.size(), 0); // the largest-component pass runs more than once
	unsigned numCorroborated = 0, numKeptTier = 0, numKeptComponent = 0, numKeptPeel = 0, numCorroborationRounds = 0;
	if (maxCorroborationAngle > 0.f) {
		// the pairs the resection registers from carry this many weighted inliers at the least,
		// and a pair of that strength whose relative pose agrees with the model is evidence
		std::vector<std::array<unsigned, 3>> entryEdges;
		BuildCovisEdges(scene, minCovisibilityCount, minInliersPerTrack, entryEdges);
		DisjointSet<IIndex> entryDS(scene.images.size());
		for (const auto& e : entryEdges)
			entryDS.Union(e[0], e[1]);
		const std::unordered_map<IIndex, unsigned> entrySizes = entryDS.CompressAllPaths().GetComponentSizes();
		IIndex largestRoot = NO_ID;
		unsigned maxSize = 0;
		for (const auto& [root, size] : entrySizes)
			if (size > maxSize || (size == maxSize && root < largestRoot)) {
				maxSize = size;
				largestRoot = root;
			}
		std::vector<uint8_t> settled(scene.images.size(), 0);
		FOREACH(imgIdx, scene.images)
			settled[imgIdx] = (scene.images[imgIdx].IsValid() && !tierDrop[imgIdx] &&
				entryDS.Find(imgIdx) == largestRoot) ? 1 : 0;
		for (;;) {
			// One pass over the pairs: count, per not-yet-settled image, the settled neighbours whose
			// pair agrees with the model about it. Both endpoints of a pair are tried, since either may
			// be the one in need of corroboration; a pair between two settled images, or two unsettled
			// ones, decides nothing this round.
			std::vector<unsigned> numWitnesses(scene.images.size(), 0);
			unsigned numThin = 0, numTried = 0, numRotationOff = 0, numDirectionOff = 0;
			for (const ImagePair& pair : scene.pairs) {
				if (!IsPoseLinkPair(pair))
					continue;
				if (pair.GetNumWeightedInliers() < minCorroborationInliers) {
					++numThin;
					continue;
				}
				for (unsigned side = 0; side < 2; ++side) {
					const IIndex imageID = side == 0 ? pair.ID1 : pair.ID2;
					const IIndex neighborID = side == 0 ? pair.ID2 : pair.ID1;
					if (settled[imageID] || !settled[neighborID] || !scene.images[imageID].IsValid())
						continue;
					++numTried;
					const PairDisagreement d = MeasurePairAgainstModel(scene, pair, imageID, neighborID);
					if (d.rotation > maxCorroborationAngle)
						++numRotationOff; // the pair puts the image at another orientation than the model does
					else if (d.HasDirection() && d.direction > d.DirectionTolerance(maxCorroborationAngle))
						++numDirectionOff; // ... or on another side of its neighbor
					else
						++numWitnesses[imageID];
				}
			}
			IIndexArr newlySettled;
			unsigned numSingle = 0;
			FOREACH(imgIdx, scene.images) {
				if (numWitnesses[imgIdx] >= 2)
					newlySettled.push_back(imgIdx);
				else if (numWitnesses[imgIdx] == 1)
					++numSingle;
			}
			DEBUG_EXTRA("Corroboration round %u: %u pairs to a settled neighbour tried, %u thin pairs in the graph, %u off in rotation, %u off in direction; %u images with one witness, %u with two or more",
				numCorroborationRounds + 1, numTried, numThin, numRotationOff, numDirectionOff, numSingle, (unsigned)newlySettled.size());
			if (newlySettled.empty())
				break;
			for (const IIndex imgIdx : newlySettled) {
				corroborated[imgIdx] = 1;
				settled[imgIdx] = 1;
			}
			numCorroborated += (unsigned)newlySettled.size();
			++numCorroborationRounds;
		}
	}

	// Apply the tier verdicts in a single batch sweep over the tracks (order-independent).
	{
		IIndexArr tierDropIDs;
		FOREACH(imgIdx, scene.images)
			if (tierDrop[imgIdx]) {
				if (corroborated[imgIdx]) {
					++numKeptTier;
					continue;
				}
				tierDropIDs.push_back(imgIdx);
			}
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
		return removed;
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
		const REAL minCosAngle = COS(D2R(REAL(maxPoseInconsistencyAngle)));
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
	const auto InvalidateImagesIfNotInLargestComponent =
		[&scene, &filteredIDs, &contradictingIDs, &ds, &corroborated, &keptByComponent, &numKeptComponent, maxCorroborationAngle]() {
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
		// what the verified pairs across each cut say about the component leaving: a component
		// they put elsewhere than the model does is cut for contradicting it, one they say nothing
		// about is cut for want of evidence
		std::vector<IIndex> rootOf(scene.images.size(), NO_ID);
		FOREACH(imgIdx, scene.images)
			if (scene.images[imgIdx].IsValid())
				rootOf[imgIdx] = ds.Find(imgIdx);
		CutJunctionMap cutJunctions;
		if (maxCorroborationAngle > 0.f)
			cutJunctions = MeasureCutJunctions(scene, rootOf, largestComponentRoot, maxCorroborationAngle, minCorroborationInliers);
		#if TD_VERBOSE != TD_VERBOSE_OFF
		// a component of a block's size leaving the model is explained, not just counted
		if (VERBOSITY_LEVEL > 1 && maxCorroborationAngle > 0.f) {
			constexpr unsigned minReportedComponentImages = 5;
			ReportCutComponents(scene, rootOf, largestComponentRoot, cutJunctions, minInliersPerTrack, minReportedComponentImages);
		}
		#endif
		// Invalidate images not in the largest component, in one batch sweep
		IIndexArr dropIDs;
		FOREACH(imgIdx, scene.images) {
			if (scene.images[imgIdx].IsValid() && largestComponentRoot != rootOf[imgIdx]) {
				if (corroborated[imgIdx]) {
					if (!keptByComponent[imgIdx]) {
						keptByComponent[imgIdx] = 1;
						++numKeptComponent;
					}
					continue;
				}
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

	// Filter edge weights to keep only edges within largest component
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
		if (alive[imgIdx] && !corroborated[imgIdx] && degree[imgIdx] < minCovisDegree)
			peelQueue.push_back(imgIdx);
	IIndexArr peeledIDs;
	while (!peelQueue.empty()) {
		const IIndex imgIdx = peelQueue.back();
		peelQueue.pop_back();
		if (!alive[imgIdx] || corroborated[imgIdx] || degree[imgIdx] >= minCovisDegree)
			continue;
		DEBUG_EXTRA("warning: image %u (`%s`) invalidated for weak covisibility degree (%u < %u)",
			imgIdx, Util::getFileName(scene.images[imgIdx].fileName).c_str(), degree[imgIdx], minCovisDegree);
		alive[imgIdx] = 0;
		peeledIDs.push_back(imgIdx);
		for (const IIndex nb : adj[imgIdx])
			if (alive[nb] && degree[nb] > 0) {
				--degree[nb];
				if (degree[nb] < minCovisDegree && !corroborated[nb])
					peelQueue.push_back(nb);
			}
	}
	FOREACH(imgIdx, scene.images)
		if (alive[imgIdx] && corroborated[imgIdx] && degree[imgIdx] < minCovisDegree)
			++numKeptPeel;
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

	if (numCorroborated > 0)
		DEBUG("Corroboration: %u images kept by two consistent pairs in %u rounds (%u of them the tier verdicts "
			"would have dropped, %u as not in the largest connected component, %u for a weak covisibility degree)",
			numCorroborated, numCorroborationRounds, numKeptTier, numKeptComponent, numKeptPeel);
	DEBUG("Filtered %u/%u weakly connected images in %s",
		filteredIDs.size(), scene.status.nCalibratedImages+filteredIDs.size(), TD_TIMER_GET_FMT().c_str());
	return removed;
}
/*----------------------------------------------------------------*/
