////////////////////////////////////////////////////////////////////
// PairsWeighting.cpp
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "PairsWeighting.h"
#include "Scene.h"

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

// Compute spatial spread of inliers (Intrinsic Weight)
// Combines coverage (grid)
float ComputeIntrinsicWeight(ImagePair& pair, const Image& img1, const Image& img2, int gridSize = 10, unsigned minInliers = 15) {
	if (!pair.HasMatches())
		return 0.f;

	// Collect points, use filtered inliers if available
	if (pair.GetNumFilteredInliers() < minInliers)
		return 0.f; // minimal support needed
	const auto [points1, points2] = pair.GetMatchedPoints(img1, img2);

	// Grid Coverage Score (N_eff)
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
	std::vector<bool> grid1(gridSize * gridSize, false);
	std::vector<bool> grid2(gridSize * gridSize, false);
	for (const auto& p : points1) {
		auto [gx, gy] = binFeature(p, img1);
		if (gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize)
			grid1[gy * gridSize + gx] = true;
	}
	for (const auto& p : points2) {
		auto [gx, gy] = binFeature(p, img2);
		if (gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize)
			grid2[gy * gridSize + gx] = true;
	}
	int occupied1 = 0, occupied2 = 0;
	for (bool b : grid1) if (b) occupied1++;
	for (bool b : grid2) if (b) occupied2++;
	const float areaScore = (float)MINF(occupied1, occupied2) / (float)(gridSize * gridSize);
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
		pair.weightSpatial = ComputeIntrinsicWeight(pair, scene.images[pair.ID1], scene.images[pair.ID2], config.gridSize, config.minInliers);
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
		const unsigned w = pair.GetNumFilteredInliers();
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
		const float w = (float)pair.GetNumFilteredInliers();
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
