////////////////////////////////////////////////////////////////////
// PairsWeighting.cpp
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "PairsWeighting.h"
#include "Scene.h"
#include <algorithm>

#ifdef _USE_BOOST
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#endif

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define PAIRSWEIGHTING_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

float SFM::ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize)
{
	ASSERT(gridSize > 0);
	if (!pair.HasMatches())
		return 0.f;
	// The coverage runs over the TRACK-FORMING matches, dense supplement included: it measures
	// where this pair has correspondences, and a dense draw covers the frame it was drawn over
	// whether or not that counts as descriptor evidence.
	const auto [points1, points2] = pair.GetTrackFormingPoints(img1, img2);
	// Divide each view into gridSize x gridSize cells:
	//  - pinhole  : uniform pixel grid (each cell = equal pixel area)
	//  - spherical: equal-solid-angle bins on the unit sphere via (azimuth, sin(latitude));
	//               each cell covers 4*pi/gridSize^2 sr, and azimuth binning wraps
	//               across the equirectangular seam (u=0 ~ u=W)
	const auto binFeature = [gridSize](const Point2f& p, const Image& img) {
		int gx, gy;
		if (img.pCamera->GetType() == CameraType::SPHERICAL) {
			const Point3 b = img.pCamera->UnprojectNormalized(Cast<REAL>(p));
			const REAL azimuth = ATAN2(b.x, b.z); // [-pi, pi]
			gx = MINF((int)((azimuth + REAL(M_PI)) / (REAL(2) * REAL(M_PI)) * REAL(gridSize)), gridSize - 1);
			gy = MINF((int)((b.y + REAL(1)) * REAL(0.5) * REAL(gridSize)), gridSize - 1);
		} else {
			gx = (int)(p.x / (float)img.GetWidth() * gridSize);
			gy = (int)(p.y / (float)img.GetHeight() * gridSize);
		}
		return std::make_pair(gx, gy);
	};
	const auto occupied = [&](const std::vector<Point2f>& points, const Image& img) {
		std::vector<bool> grid(gridSize * gridSize, false);
		for (const Point2f& p : points) {
			const auto [gx, gy] = binFeature(p, img);
			if (gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize)
				grid[gy * gridSize + gx] = true;
		}
		return (int)std::count(grid.begin(), grid.end(), true);
	};
	return (float)MINF(occupied(points1, img1), occupied(points2, img2)) / (float)(gridSize * gridSize);
}

// Intrinsic weight of a pair: the grid coverage of its inliers (ComputePairCoverage) times the
// angle/baseline term, with a proxy from the homography overlap for pairs without stored matches.
float ComputeIntrinsicWeight(ImagePair& pair, const Image& img1, const Image& img2, int gridSize = 10, unsigned minInliers = 15) {
	if (!pair.HasMatches())
		return 0.f;

	// The VALIDITY FLOOR: does this pair carry enough verified correspondence to be considered at
	// all. The track-forming set is the right quantity -- a gate-validated dense supplement is
	// evidence about the pair's geometry -- and returning 0 here zeroes weightSpatial, hence
	// GetCompositeWeight(), hence BuildTracks' minPairWeight cut, so an infused pair whose sparse
	// segment dips below the floor would contribute NO tracks at all, sparse or dense, and the
	// infusion would go silently inert on exactly the weak pairs it exists to serve.
	if (pair.GetNumTrackFormingMatches() < minInliers)
		return 0.f; // minimal support needed
	// The ANGLE term reads meanRayAngle, accumulated over that same track-forming set (FilterMatches,
	// or ImagePair::ComputeMeanRayAngle when an append changed the set without re-filtering it), for
	// the same reason: a ray angle is a geometric quantity, not a sub-pixel one, and this is the one
	// term that can demote a degenerate baseline -- it has to be available on a pair whose evidence
	// is dense. Only a pair with no relative pose at all reads 0 here, which
	// ComputeAngleBaselineWeight scores at its MAXIMUM (see the note there): no baseline was
	// measurable, and no term in this product will demote such a pair.
	const float areaScore = ComputePairCoverage(pair, img1, img2, gridSize);
	if (pair.overlapArea <= 0.f)
		pair.overlapArea = areaScore; // no overlap, store area score as proxy

	// Apply angle baseline weighting
	const float angleScore = pair.ComputeAngleBaselineWeight();
	return areaScore * angleScore;
}


void SFM::ComputePairsWeights(Scene& scene, const PairsWeightingConfig& config, IIndexArr* pComponents) {
	TD_TIMER_STARTD();

	// 1. Compute Intrinsic Weights (Parallelizable)
	// This depends only on the pair itself
	#ifdef PAIRSWEIGHTING_USE_OPENMP
	#pragma omp parallel for
	for (int_t i = 0; i < (int_t)scene.pairs.size(); ++i) {
		ImagePair& pair = scene.pairs[i];
	#else
	for (ImagePair& pair : scene.pairs) {
	#endif
		// the pair's inlier evidence, which this pass is the one holder of the dense discount for:
		// written before any weight reads it, since the connectivity step below and every consumer
		// of GetCompositeWeight() downstream are exactly its readers. A pair with no matches at all
		// carries no partition to read a dense count out of (the weight below is 0 for it anyway),
		// so it keeps the "never computed" value and the accessor answers from its counts alone.
		pair.weightedInliers = pair.HasMatches() ?
			(float)pair.GetNumFilteredInliers() + config.denseObservationWeight*(float)pair.GetNumDenseInliers() : -1.f;
		pair.weightSpatial = ComputeIntrinsicWeight(pair, scene.images[pair.ID1], scene.images[pair.ID2], config.gridSize, config.minInliers);
		// A pair whose evidence ROUNDS AWAY carries none: with a small enough minInliers the floor
		// above admits a pair of two dense matches, whose discounted evidence is 0.25*2 = 0.5 -> 0,
		// and a zero magnitude is a zero composite weight however good the quality factors are. Give
		// it the same answer the floor gives instead, here, once: every later step of this pass and
		// every consumer downstream reads "no weight" off weightSpatial, and the connectivity step
		// below in particular divides by a per-node maximum this pair would otherwise be excluded
		// from while still being asked for its own share of it.
		if (pair.GetNumWeightedInliers() == 0)
			pair.weightSpatial = 0.f;
	}

	#ifdef _USE_BOOST
	// 2. Build Graph for Extrinsic Weights
	// Map pair index to graph edge
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, unsigned> Graph;
	Graph g(scene.images.size());
	FOREACH(i, scene.pairs) {
		ImagePair& pair = scene.pairs[i];
		// Only consider pairs that have some intrinsic weight (i.e. valid geometry)
		if (pair.weightSpatial > 1e-6f && pair.HasGeometricVerification())
			boost::add_edge(pair.ID1, pair.ID2, i, g);
	}
	ASSERT(boost::num_vertices(g) == scene.images.size(), "ComputePairsWeights: graph node count mismatch");

	// 3. Compute Triplet Support (Cycle Consistency)
	// Iterate valid edges and check triangles
	// Note: We could use specialized triangle counting algorithms, but simple iteration is fine for typical SfM graph density

	// Helper to get rotation error (cos of angle) for a triplet
	auto GetRotationError = [&](const ImagePair& p_ij, const ImagePair& p_jk, const ImagePair& p_ki, IIndex i, IIndex j, IIndex k) {
		// Pair stores R s.t. x2 = R*x1 + t so R_12 is pose of 2 relative to 1;
		// Get relative rotation from pair in the correct direction
		auto GetRelR = [](const ImagePair& p, IIndex u, IIndex v) -> Matrix3x3 {
			if (p.ID1 == u && p.ID2 == v) return p.relativePose->R; // R_uv
			if (p.ID1 == v && p.ID2 == u) return p.relativePose->R.t(); // R_vu = R_uv^T
			return Matrix3x3::IDENTITY;
		};
		if (!p_ij.relativePose.has_value() || !p_jk.relativePose.has_value() || !p_ki.relativePose.has_value())
			return -1.f; // invalid triplet
		// Compose: R_ij * R_jk * R_ki (should be Identity)
		Matrix3x3 R_ij = GetRelR(p_ij, i, j);
		Matrix3x3 R_jk = GetRelR(p_jk, j, k);
		Matrix3x3 R_ki = GetRelR(p_ki, k, i);
		// Compose: cycle k->i->j->k
		Matrix3x3 R_loop = R_jk * R_ij * R_ki;
		return (float)ComputeAngle(R_loop);
	};

	// Iterate all edges in the graph
	const float minCosAngleError = COS(D2R(config.maxAngleTripletDegrees));
	Graph::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei) {
		unsigned pairIdx = g[*ei];
		ImagePair& pair = scene.pairs[pairIdx];
		if (!pair.relativePose.has_value()) {
			pair.weightTriplet = 0.f;
			continue;
		}
		IIndex u = pair.ID1;
		IIndex v = pair.ID2;

		// Find common neighbors (triangles)
		Graph::adjacency_iterator u_nbr, u_nbr_end;

		// Simple intersection (can be optimized if degrees are high)
		// For typical view graphs, degree is manageable (20-100)
		unsigned numValidTriplets = 0, numInvalidTriplets = 0;
		for (boost::tie(u_nbr, u_nbr_end) = boost::adjacent_vertices(u, g); u_nbr != u_nbr_end; ++u_nbr) {
			IIndex k = (IIndex)*u_nbr;
			if (k == v)
				continue;
			// Check if k is neighbor of v
			auto edge_vk = boost::edge(v, k, g);
			if (!edge_vk.second)
				continue;
			// Found triangle u-v-k
			unsigned idx_uk = g[boost::edge(u, k, g).first];
			unsigned idx_vk = g[edge_vk.first];
			float cosAngle = GetRotationError(pair, scene.pairs[idx_vk], scene.pairs[idx_uk], u, v, k);
			if (cosAngle > minCosAngleError) {
				// Valid triplet: cycle closure error is within threshold
				++numValidTriplets;
			} else {
				// Invalid triplet: cycle closure error exceeds threshold
				++numInvalidTriplets;
			}
		}

		// Score accounts for both valid triplets and invalid triplets;
		// this penalizes pairs that are part of inconsistent triplets (e.g., due to mismatches or geometry errors)
		ASSERT(pair.weightSpatial > 0.f, "ComputePairsWeights: zero intrinsic weight in triplet computation");
		pair.weightTriplet = (float)numValidTriplets / ((float)(numValidTriplets + numInvalidTriplets) + config.tripletSaturation);
	}

	// 4. Compute Local Connectivity (Relative Density)
	// D_local = sqrt( (N_ij / Max_N_i) * (N_ij / Max_N_j) )
	// Where N_ij can be the raw count or the spatial weighted count. Let's use spatial weighted count for robustness.

	// Precompute max weight per node
	UnsignedArr maxNodeWeight(scene.images.size());
	maxNodeWeight.Memset(0);
	for (const ImagePair& pair : scene.pairs) {
		if (pair.weightSpatial <= 0.f)
			continue; // skip if no matches
		const unsigned w = pair.GetNumWeightedInliers();
		// guaranteed by step 1, which zeroes weightSpatial on a pair whose evidence rounds to 0 --
		// this assertion is therefore a statement about that invariant, not a bar a configuration
		// can trip
		ASSERT(w > 0, "ComputePairsWeights: non-positive intrinsic weight in connectivity computation");
		if (w > maxNodeWeight[pair.ID1]) maxNodeWeight[pair.ID1] = w;
		if (w > maxNodeWeight[pair.ID2]) maxNodeWeight[pair.ID2] = w;
	}

	const float ratioSigma = -1.f / (2.f * SQUARE(config.sigmaInlierPerMatches)); // Gaussian sigma for inliers ratio weighting
	#ifdef PAIRSWEIGHTING_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	for (int_t i = 0; i < (int_t)scene.pairs.size(); ++i) {
		ImagePair& pair = scene.pairs[i];
	#else
	for (ImagePair& pair : scene.pairs) {
	#endif
		pair.weightConnectivity = 0.f;
		if (pair.weightSpatial <= 0.f)
			continue;
		const float w = (float)pair.GetNumWeightedInliers();
		const float max1 = (float)maxNodeWeight[pair.ID1];
		const float max2 = (float)maxNodeWeight[pair.ID2];
		pair.weightConnectivity = MINF(SQRT((w * w) / (max1 * max2)), 1.f);
		// Boost by inlier ratio
		const float inliersRatio = w / (float)pair.GetNumMatches();
		const float wInliersRatio = MINF((1.f - EXP(SQUARE(inliersRatio) * ratioSigma)) * 2.f, 1.f);
		pair.weightConnectivity *= wInliersRatio;
	}

	// 5. Sort pairs by composite weight (decreasing)
	scene.pairs.Sort([](const ImagePair& a, const ImagePair& b) {
		return a.GetCompositeWeight() > b.GetCompositeWeight();
	});

	// 6. Compute connected components
	IIndexArr component(scene.images.size());
	const unsigned numComponents = boost::connected_components(g, component.data());
	ASSERT(numComponents > 0, "ComputePairsWeights: no connected components found");
	// Compute component size statistics
	UnsignedArr componentSizes(numComponents);
	componentSizes.Memset(0);
	for (IIndex comp : component)
		++componentSizes[comp];
	MeanStdMinMax<unsigned, double> stats(componentSizes.data(), componentSizes.size());
	DEBUG("Connected components: %u components, sizes: max %u, min %u, median %.1f, mean %.2f, std %.2f",
		componentSizes.size(), stats.maxVal, stats.minVal, componentSizes.GetMedian(), stats.GetMean(), stats.GetStdDev());
	if (pComponents)
		*pComponents = std::move(component);
	#else
	// Fallback if Boost Graph is not available (though OpenMVS requires Boost)
	// Just use intrinsic weights
	for (auto& pair : scene.pairs) {
		pair.weightConnectivity = 1.f;
		pair.weightTriplet = 0.f;
	}
	#endif

	// 7. Print weights stats (optional)
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		MeanStdMinMax<float,REAL> weightPerPair;
		unsigned numPairsWithMatches = 0;
		for (const ImagePair& pair : scene.pairs) {
			if (!pair.HasMatches()) {
				ASSERT(!pair.HasValidWeight());
				continue;
			}
			++numPairsWithMatches;
			if (!pair.HasValidWeight())
				continue;
			weightPerPair.Update(pair.GetCompositeWeight());
		}
		VERBOSE("Weight per pair (pairs %u with matches, %u with weight): mean %.2f, std %.2f, range [%.4g, %.4g]",
				numPairsWithMatches, weightPerPair.size,
				weightPerPair.GetMean(), weightPerPair.GetStdDev(),
				weightPerPair.GetMin(), weightPerPair.GetMax());
	}
	#endif

	DEBUG("Computed pairs weights (Intrinsic and Extrinsic): %u pairs (%s)",
		scene.pairs.size(), TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/
