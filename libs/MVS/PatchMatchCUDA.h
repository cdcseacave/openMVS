/*
* PatchMatchCUDA.h
*
* Copyright (c) 2014-2021 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#ifndef _MVS_PATCHMATCHCUDA_H_
#define _MVS_PATCHMATCHCUDA_H_

#ifdef _USE_CUDA


// I N C L U D E S /////////////////////////////////////////////////

#include "SceneDensify.h"
#include "PatchMatchCUDA.inl"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

/**
 * @brief Propagate and refine the depth/normal estimate for a single pixel.
 *
 * This is the core PatchMatch iteration step, implementing the AMHMVS algorithm
 * (Asymmetric Multi-Hypothesis Multi-View Stereo). Each call processes one pixel
 * in a red-black checkerboard pattern, allowing parallel execution on GPU.
 *
 * The algorithm has four main phases:
 *
 * **Phase 1: Neighbor Hypothesis Gathering**
 * - Search 8 directional patterns (4 near + 4 far) to find best neighbor in each
 * - For each neighbor found, interpolate its plane to current pixel and compute
 *   per-view matching costs -> costArray[8][MAX_VIEWS]
 *
 * **Phase 2: Multi-Hypothesis Joint View Selection (AMHMVS)**
 * - Build view priors from 4-connected neighbors' selectedViews
 * - Compute sampling probability for each view based on:
 *   - Prior from neighbors (which views they found useful)
 *   - Cost performance across the 8 hypotheses (Gaussian-weighted)
 *   - Iteration-dependent threshold (stricter over time)
 * - Sample views via PDF->CDF to get viewWeights[]
 *
 * **Phase 3: Propagation**
 * - Aggregate each neighbor's costs using viewWeights
 * - Compare best neighbor against current estimate
 * - Adopt neighbor's plane if it gives lower cost
 *
 * **Phase 4: Refinement**
 * - Test 4 candidate planes to escape local minima:
 *   0. Perturbed depth + current normal
 *   1. Current depth + perturbed normal
 *   2. Current depth + random normal
 *   3. Current depth + surface normal (estimated from 4-connected neighbors)
 * - Adopt any candidate that improves the cost
 *
 * **Smoothness coupling** (phases 3 and 4)
 * - Every aggregated cost, the stored one included, is scaled by SmoothnessFactor:
 *   up to 20 % lower the closer the hypothesis lies, in depth and normal, to the
 *   pixel's smoothed plane, which the SmoothPlanes kernel fits to the 3x3
 *   neighborhood (gray-similarity x confidence weights) before every pass
 *   (docs/design/PatchMatchSmoothness.md)
 *
 * @see InitializePixelScore() for initial setup before iterations
 * @see ScorePlane() for the photometric matching cost computation
 * @see ProcessPixel() reads selectedViews from 4-connected neighbors processed in the
 *      previous kernel call (red reads black, black reads red). It writes selectedViews
 *      for the current pixel only when a better neighbor is adopted.
 *      The depth prior (lowDepths) does not directly influence propagation decisions.
 *      It only affects the cost computation inside ScorePlane() by blending NCC cost
 *      with depth-prior cost in textureless regions.
 * @see "Multi-View Stereo with Asymmetric Checkerboard Propagation and Multi-Hypothesis
 *       Joint View Selection" (https://arxiv.org/abs/1805.07920)
 */

} // namespace MVS

#endif // _USE_CUDA

#endif // _MVS_PATCHMATCHCUDA_H_
