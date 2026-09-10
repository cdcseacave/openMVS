/*
* SceneRefineCommon.cpp
*
* Copyright (c) 2014-2026 SEACAVE
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

#include "Common.h"
// Tell DEFVAR_OPTION/DEFOPT_SPACE (defined in libs/Common/Common.h) to tag
// the OPTREFINE namespace's data symbols and helpers with MVS_API so they
// are exported from MVS.dll instead of the default Common-side tag -- same
// trick DepthMap.cpp uses for OPTDENSE.
#undef OPTCONFIG_API
#define OPTCONFIG_API MVS_API
#include "SceneRefineCommon.h"
#include "Scene.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define DEFVAR_OPTREFINE_int32(name, title, desc, ...)  DEFVAR_int32(OPTREFINE, name, title, desc, __VA_ARGS__)
#define DEFVAR_OPTREFINE_float(name, title, desc, ...)  DEFVAR_float(OPTREFINE, name, title, desc, __VA_ARGS__)
#define DEFVAR_OPTREFINE_bool(name, title, desc, ...)   DEFVAR_bool(OPTREFINE, name, title, desc, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {
DEFOPT_SPACE(OPTREFINE, _T("Refine"))

DEFVAR_OPTREFINE_int32(nIgnoreMaskLabel, "Ignore Mask Label", "label id used during ignore mask filter (<0 - disabled)", "-1")
DEFVAR_OPTREFINE_int32(nImageGradient, "Image Gradient", "image derivative stencil (0 - 3x5 separable, 1 - central, 2 - Sobel, 3 - bilinear interpolant derivative)", "1")
DEFVAR_OPTREFINE_float(fGateMeanDiff, "Gate Mean Diff", "reject a pixel pair whose local mean differs by more than this (0 - disabled)", "0.4")
DEFVAR_OPTREFINE_float(fGateVarRatio, "Gate Var Ratio", "reject a pixel pair whose local variance ratio exceeds this (0 - disabled)", "8.0")
DEFVAR_OPTREFINE_int32(nMaxEvaluations, "Max Evaluations", "hard cap on the energy evaluations of every scale (0 - the convergence rules alone decide)", "0")
DEFVAR_OPTREFINE_bool(bAdaptiveFaceSize, "Adaptive Face Size", "grade the prepared faces per vertex, so every face projects to about the face cap in the pair that refines it, instead of one world-space density for the whole mesh", "1")
DEFVAR_OPTREFINE_float(fSimplifyTolerance, "Simplify Tolerance", "decimate the refined mesh within this reprojection error, in pixels of the working resolution in the pair that resolves each vertex best, once the refinement ends (0 - disabled)", "0.25")

} // namespace MVS


// the neighbor views one image contributes refinement pairs with; hoisted out of both backends'
// constructors (CPU MeshRefine::ThSelectNeighbors, SceneRefine.cpp; CUDA MeshRefineCUDA's
// constructor, SceneRefineCUDA.cpp) so the filter thresholds and the missing-neighbor recovery
// exist once -- see the doc comment in SceneRefineCommon.h
bool MVS::SelectRefineNeighbors(Scene& scene, uint32_t idxImage, unsigned nMaxViews, ViewScoreArr& neighbors)
{
	// keep only best neighbor views
	const float fMinArea(0.1f);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(D2R(2.5f)), fMaxAngle(D2R(45.f));
	Image& imageData = scene.images[idxImage];
	if (!imageData.IsValid())
		return false;
	if (imageData.neighbors.IsEmpty()) {
		IndexArr points;
		scene.SelectNeighborViews(idxImage, points);
	}
	neighbors = imageData.neighbors;
	Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
	return true;
}

// load, gray-convert, blur and resize one refine image at the given scale; the common part of
// CPU MeshRefine::ThInitImage (SceneRefine.cpp) and CUDA MeshRefineCUDA::InitImages
// (SceneRefineCUDA.cpp), see the doc comment in SceneRefineCommon.h
bool MVS::PrepareRefineImage(Image& imageData, const PlatformArr& platforms,
	unsigned nResolutionLevel, unsigned nMinResolution, float scale, float sigma, Image32F& gray)
{
	ASSERT(imageData.IsValid());
	// load and init image
	unsigned level(nResolutionLevel);
	const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
	if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && !imageData.ReloadImage(imageSize))
		return false;
	imageData.image.toGray(gray, cv::COLOR_BGR2GRAY, true);
	imageData.image.release();
	if (sigma > 0)
		cv::GaussianBlur(gray, gray, cv::Size(), sigma);
	if (scale < 1.0) {
		cv::resize(gray, gray, cv::Size(), scale, scale, cv::INTER_AREA);
		imageData.width = gray.width(); imageData.height = gray.height();
	}
	imageData.UpdateCamera(platforms);
	return true;
}

// per-view keep-mask, called right after PrepareRefineImage -- see the doc comment in
// SceneRefineCommon.h; leaves keepMask empty unless masking is enabled and the image has a mask
void MVS::PrepareRefineImageMask(const Image& imageData, const cv::Size& size, BitMatrix& keepMask)
{
	keepMask.release();
	if (OPTREFINE::nIgnoreMaskLabel < 0)
		return;
	DepthEstimator::ImportKeepMask(imageData, size, (uint8_t)OPTREFINE::nIgnoreMaskLabel, keepMask);
}

// the largest finite value of a per-vertex field, the stand-in for the vertices no view saw
static float MaxSeen(const FloatArr& values, const FloatArr& pixelFactors)
{
	float maxValue(0);
	FOREACH(v, values)
		if (pixelFactors[v] > 0 && values[v] > maxValue)
			maxValue = values[v];
	return maxValue;
}

void MVS::ListPairImages(const PairIdxArr& pairs, size_t numImages, Unsigned8Arr& used)
{
	used.Resize(numImages);
	used.Memset(0);
	for (const PairIdx& pair: pairs) {
		ASSERT(pair.i < numImages && pair.j < numImages);
		used[pair.i] = used[pair.j] = 1;
	}
}

void MVS::ReduceFaceAreasOverPairs(const ViewAreaArr& viewAreas, const PairIdxArr& pairs, Mesh::AreaArr& faceAreas)
{
	ASSERT(!faceAreas.IsEmpty());
	faceAreas.Memset(0);
	FOREACHPTR(pPair, pairs) {
		const Mesh::AreaArr& areasA = viewAreas[pPair->i];
		const Mesh::AreaArr& areasB = viewAreas[pPair->j];
		ASSERT(areasA.size() == faceAreas.size() && areasB.size() == faceAreas.size());
		FOREACH(f, faceAreas) {
			const uint16_t pairArea(MINF(areasA[f], areasB[f]));
			if (faceAreas[f] < pairArea)
				faceAreas[f] = pairArea;
		}
	}
}

void MVS::SeenAreasToPixelFactors(const Mesh& mesh, const Mesh::AreaArr& seenAreas, FloatArr& pixelFactors)
{
	ASSERT(seenAreas.size() == mesh.faces.size());
	pixelFactors.Resize(mesh.vertices.size());
	pixelFactors.Memset(0);
	Unsigned32Arr numSeen(mesh.vertices.size());
	numSeen.Memset(0);
	FOREACH(f, mesh.faces) {
		if (seenAreas[f] == 0)
			continue;
		const float world(mesh.ComputeArea(f));
		if (!(world > 0))
			continue;
		const float factor(SQRT((float)seenAreas[f]/world));
		const Mesh::Face& face = mesh.faces[f];
		for (int v=0; v<3; ++v) {
			pixelFactors[face[v]] += factor;
			++numSeen[face[v]];
		}
	}
	FOREACH(v, pixelFactors)
		if (numSeen[v] > 0)
			pixelFactors[v] /= (float)numSeen[v];
}

void MVS::SeenAreasToEdgeTargets(const Mesh& mesh, const Mesh::AreaArr& seenAreas, float targetArea, FloatArr& targets)
{
	ASSERT(seenAreas.size() == mesh.faces.size() && targetArea > 0);
	targets.Resize(mesh.vertices.size());
	targets.Memset(0);
	Unsigned32Arr numSeen(mesh.vertices.size());
	numSeen.Memset(0);
	FOREACH(f, mesh.faces) {
		if (seenAreas[f] == 0)
			continue; // no pair saw it: it states no scale
		const float world(mesh.ComputeArea(f)*targetArea/(float)seenAreas[f]);
		const Mesh::Face& face = mesh.faces[f];
		for (int v=0; v<3; ++v) {
			targets[face[v]] += world;
			++numSeen[face[v]];
		}
	}
	// an equilateral triangle of area A has edge sqrt(4/sqrt(3)*A)
	constexpr float squaredEdge(4.f/1.7320508f);
	float longest(0);
	FOREACH(v, targets) {
		if (numSeen[v] == 0)
			continue;
		targets[v] = SQRT(squaredEdge*targets[v]/(float)numSeen[v]);
		longest = MAXF(longest, targets[v]);
	}
	ASSERT(longest > 0);
	FOREACH(v, targets)
		if (numSeen[v] == 0)
			targets[v] = longest;
}

void MVS::PixelFactorsToErrorBounds(const FloatArr& pixelFactors, float tolerancePx, FloatArr& bounds)
{
	ASSERT(tolerancePx > 0);
	bounds.Resize(pixelFactors.size());
	FOREACH(v, pixelFactors)
		bounds[v] = pixelFactors[v] > 0 ? SQUARE(tolerancePx/pixelFactors[v]) : 0.f;
	const float unseen(MaxSeen(bounds, pixelFactors));
	ASSERT(unseen > 0);
	FOREACH(v, pixelFactors)
		if (pixelFactors[v] <= 0)
			bounds[v] = unseen;
}

// the tightest-pair areas of the faces some pair saw, in place of the caller's array
static void SeenFaceAreas(const Mesh::AreaArr& maxAreas, Mesh::AreaArr& seen)
{
	seen.Empty();
	seen.Reserve(maxAreas.size());
	for (uint16_t area: maxAreas)
		if (area > 0)
			seen.Insert(area);
}

void MVS::LogFaceAreas(const char* stage, const Mesh::AreaArr& maxAreas, float meanSeenArea, const String& elapsed)
{
	Mesh::AreaArr seen;
	SeenFaceAreas(maxAreas, seen);
	if (seen.IsEmpty()) {
		DEBUG_EXTRA("Mesh projected (%s): no face seen by any image pair (%s)", stage, elapsed.c_str());
		return;
	}
	const size_t n(seen.size());
	DEBUG_EXTRA("Mesh projected (%s): %u/%u faces seen by a pair, tightest-pair area px2 mean %.2f (sampled), rasterized p10/p50/p90/p99 %u/%u/%u/%u (%s)",
		stage, (unsigned)n, (unsigned)maxAreas.size(), meanSeenArea, (unsigned)seen.GetNth(n/10), (unsigned)seen.GetNth(n/2), (unsigned)seen.GetNth(n*9/10), (unsigned)seen.GetNth(n*99/100), elapsed.c_str());
}

void MVS::SimplifyMeshWithinTolerance(Mesh& mesh, const FloatArr& pixelFactors, float tolerancePx)
{
	ASSERT(pixelFactors.size() == mesh.vertices.size() && tolerancePx > 0);
	TD_TIMER_STARTD();
	const size_t numVertsOld(mesh.vertices.size());
	const size_t numFacesOld(mesh.faces.size());
	FloatArr bounds;
	PixelFactorsToErrorBounds(pixelFactors, tolerancePx, bounds);
	Mesh::CleanParams params;
	params.vertexMaxError = &bounds;
	mesh.Clean(params);
	DEBUG_EXTRA("Mesh simplified within %g px: %u/%u -> %u/%u vertices/faces (%s)", tolerancePx,
		(unsigned)numVertsOld, (unsigned)numFacesOld, (unsigned)mesh.vertices.size(), (unsigned)mesh.faces.size(), TD_TIMER_GET_FMT().c_str());
}

void MVS::ComputeRefineImageGradient(const Image32F& gray, Image32F& gradX, Image32F& gradY)
{
	ASSERT(!gray.empty());
	switch (OPTREFINE::nImageGradient) {
	case 1: {
		// central differences
		const cv::Matx13f kernel(-0.5f, 0.f, 0.5f);
		cv::filter2D(gray, gradX, CV_32F, kernel);
		cv::filter2D(gray, gradY, CV_32F, kernel.t());
		break; }
	case 2:
		cv::Sobel(gray, gradX, CV_32F, 1, 0, 3, 1.0/8.0);
		cv::Sobel(gray, gradY, CV_32F, 0, 1, 3, 1.0/8.0);
		break;
	default: {
		const TMatrix<float,3,5> kernel(CreateDerivativeKernel3x5());
		cv::filter2D(gray, gradX, CV_32F, kernel);
		cv::filter2D(gray, gradY, CV_32F, kernel.t());
		break; }
	}
}
/*----------------------------------------------------------------*/


// M E S H   R E F I N E   S T E P /////////////////////////////////

void MeshRefineStep::Reset(uint32_t numVertices, float stepInit)
{
	ASSERT(stepInit > 0 && stepInit <= StepMax);
	step = stepInit;
	median = -1.f; // derived at the scale's first evaluation
	scoreRef = scoreRefAlt = FLT_MAX; // nothing to compare the first evaluation against: accepted
	numAccepted = numRejected = numRejectedTotal = numStalled = numEvaluated = 0;
	stepPrev.Resize(numVertices);
	stepPrev.Memset(0);
	scratch.Reserve(numVertices);
	scratch.Empty();
} // Reset

void MeshRefineStep::TopologyChanged(uint32_t numVertices)
{
	// the vertex array was rebuilt: the undo buffer no longer indexes the same vertices, and the
	// next S of either parity is measured on a different surface, so neither reference survives
	scoreRef = scoreRefAlt = FLT_MAX;
	stepPrev.Resize(numVertices);
	stepPrev.Memset(0);
	scratch.Reserve(numVertices);
} // TopologyChanged

// |g_v|/s_v of the median seen vertex: the scale that turns the raw photometric gradient into a
// step in pixels, computed once per scale and then held (see the class comment)
float MeshRefineStep::ComputeMedianScale(const Terms& terms)
{
	scratch.Empty();
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		// a vertex a single pair-direction saw has no triangulation behind its gradient; it is
		// left out of the normalizer for the same reason it is not moved
		if (terms.photoCount[v] < 2)
			continue;
		ASSERT(terms.footprint[v] > 0); // guaranteed wherever photoCount > 0
		scratch.Insert(norm(terms.photoGrad[v]/terms.photoCount[v])/terms.footprint[v]);
	}
	return scratch.IsEmpty() ? 0.f : scratch.GetMedian();
} // ComputeMedianScale

MeshRefineStep::Action MeshRefineStep::Evaluate(const Terms& terms, Mesh::VertexArr& vertices, Stats& stats)
{
	ASSERT(terms.numVertices == vertices.GetSize() && stepPrev.GetSize() == terms.numVertices);
	ASSERT(terms.S >= 0 && terms.S <= 2);
	// explicit-flow stability of the regularization term: one step of StepMax px must not
	// amplify the Laplacian it is applied to
	ASSERT(StepMax*terms.regularityWeight <= 1);
	const unsigned index(numEvaluated++);

	// the S this one has to beat: when each evaluation sees only one direction of every pair,
	// only evaluations of the same parity measured the same pixels
	float& reference = (terms.alternating && (index&1)) ? scoreRefAlt : scoreRef;
	stats.S = terms.S;
	stats.relChange = reference < FLT_MAX && reference > 0 ? (terms.S-reference)/reference : 0.f;
	stats.medianPx = 0;
	stats.numMoved = 0;
	stats.accepted = !(terms.S > reference);

	if (!stats.accepted) {
		// REJECT: the step overshot. Undo half of it -- every vertex goes back to exactly
		// v_prev + stepPrev/2 -- and leave the halved step behind so a second rejection undoes
		// half of what is left. The reference, the accepted count and the stall count stay: a
		// rejected evaluation never becomes the thing later ones are compared against.
		for (uint32_t v=0; v<terms.numVertices; ++v) {
			Grad& delta = stepPrev[v];
			delta *= 0.5f;
			vertices[v] -= delta;
		}
		step *= StepShrink;
		++numRejected;
		++numRejectedTotal;
		stats.step = step;
		stats.numAccepted = numAccepted;
		stats.numRejected = numRejectedTotal;
		return numRejected >= MaxRejects ? STOP : REJECT;
	}

	// ACCEPT: this S becomes the reference, and one that did not improve on the previous one by
	// ProgressTol counts as a stall (the product form needs no guard against a perfect S of 0)
	if (reference < FLT_MAX)
		numStalled = reference-terms.S <= ProgressTol*reference ? numStalled+1 : 0;
	reference = terms.S;
	numRejected = 0;
	++numAccepted;
	step = MINF(step*StepGrow, StepMax);
	stats.step = step;
	stats.numAccepted = numAccepted;
	stats.numRejected = numRejectedTotal;
	// S has stopped moving: more evaluations of this scale buy nothing
	if (numAccepted >= MinIters && numStalled >= Patience)
		return STOP;

	// the per-scale normalizer, derived at the first evaluation and then held
	if (median < 0)
		median = ComputeMedianScale(terms);

	// move the vertices
	scratch.Empty();
	const float scale(Kappa*median);
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		// photometric step in scene units, PROPORTIONAL to the raw gradient (see the class
		// comment); a vertex a single pair-direction saw has no triangulation behind its gradient
		// and moves on smoothing alone
		const float footprint(terms.footprint[v]);
		Grad photoDelta(Grad::ZERO);
		if (terms.photoCount[v] >= 2 && scale > 0) {
			ASSERT(footprint > 0);
			photoDelta = terms.photoGrad[v]/(terms.photoCount[v]*scale);
		}
		const Grad regular(terms.bilap[v]*terms.rigidity - terms.lap[v]*(1.f-terms.rigidity));
		const Grad delta((photoDelta + regular*terms.regularityWeight)*-step);
		// contract on the producers (ScoreMesh on either backend): every term is finite; a
		// non-finite delta would be "not applied" by the len > 0 test below and yet be stored
		// for a later rejection's undo to subtract from a vertex that never moved
		ASSERT(ISFINITE(delta));
		stepPrev[v] = delta;
		const float len(norm(delta));
		if (len > 0) {
			vertices[v] += delta;
			++stats.numMoved;
			// the stop rule is in pixels: a vertex with no footprint (unseen, moved on smoothing
			// alone) contributes a position change but no pixel measurement
			if (footprint > 0)
				scratch.Insert(len/footprint);
		}
	}
	stats.medianPx = scratch.IsEmpty() ? 0.f : scratch.GetMedian();
	// Nobody is moving any more, measured in pixels: this scale has converged. The test is on
	// what the median vertex WOULD move at a full stride, not on what it just moved: medianPx is
	// proportional to eta, and eta is the bold driver's own state -- a rejection halves it while
	// an acceptance only grows it by 1.1, so an accept/reject oscillation around a plateau
	// ratchets eta down by 0.55 per cycle and would otherwise report convergence while the
	// direction field is still large
	if (numAccepted >= MinIters && stats.medianPx*(StepMax/step) < StepStop)
		return STOP;
	return APPLY;
} // Evaluate
/*----------------------------------------------------------------*/
