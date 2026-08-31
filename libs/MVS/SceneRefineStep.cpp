/*
* SceneRefineStep.cpp
*
* Copyright (c) 2014-2015 SEACAVE
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
#include "SceneRefineStep.h"

using namespace MVS;


// S T R U C T S ///////////////////////////////////////////////////

void MeshRefineStep::Reset(uint32_t numVertices, float stepInit, bool _boldDriver)
{
	ASSERT(stepInit >= 0 && stepInit <= StepMax);
	step = stepInit;
	median = -1.f; // "not derived yet"; the first evaluation of the scale sets it
	scoreRef = scoreRefAlt = FLT_MAX;
	numAccepted = numRejected = numRejectedTotal = numStalled = numEvaluated = 0;
	boldDriver = _boldDriver;
	// nothing has been applied yet, so the first evaluation has no predecessor to be worse than
	topologyChanged = true;
	stepPrev.Resize(numVertices);
	stepPrev.Memset(0);
	scratch.Reserve(numVertices);
	scratch.Empty();
} // Reset

void MeshRefineStep::TopologyChanged(uint32_t numVertices)
{
	// the vertex array was rebuilt, so the undo buffer no longer indexes the same vertices and the
	// next S is measured on a different surface: it cannot be rejected against the previous one
	topologyChanged = true;
	stepPrev.Resize(numVertices);
	stepPrev.Memset(0);
	scratch.Reserve(numVertices);
} // TopologyChanged


// |g_v|/s_v of the median seen vertex: the scale that turns the raw photometric gradient into a
// step in pixels. Computed once per scale, at its first evaluation, and then held -- see the
// header for why recomputing it every iteration would defeat the stop rule.
float MeshRefineStep::ComputeMedianScale(const Terms& terms)
{
	scratch.Empty();
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		// a vertex a single pair-direction saw has no triangulation behind its gradient; it is left
		// out of the normalizer for the same reason it is not moved
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
	// explicit-flow stability of the regularization term: one step of eta_max px must not amplify
	// the Laplacian it is applied to
	ASSERT(StepMax*terms.regularityWeight <= 1);
	const unsigned index(numEvaluated++);

	stats.S = terms.S;
	stats.medianPx = 0;
	stats.numMoved = 0;

	// which S this one has to beat: when each evaluation sees only one direction of every pair,
	// only evaluations of the same parity measured the same pixels
	float& reference = (terms.alternating && (index&1)) ? scoreRefAlt : scoreRef;

	// the fixed-step control arm never rejects: it exists to answer whether the bold driver's
	// REJECT/grow/shrink machinery is what matters, so it must never engage either
	if (boldDriver && !topologyChanged && terms.S > reference) {
		// REJECT: the step overshot. Undo half of it -- every vertex goes back to exactly
		// v_prev + stepPrev/2 -- and leave the halved step behind so a second rejection undoes half
		// of what is left. The reference, the accepted count and the stall count are deliberately
		// untouched: a rejected evaluation never becomes the thing later ones are compared against.
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

	// ACCEPT
	const float previous(reference);
	reference = terms.S;
	numRejected = 0;
	++numAccepted;
	if (previous < FLT_MAX) {
		ASSERT(previous > 0);
		numStalled = (previous-terms.S)/previous < ProgressTol ? numStalled+1 : 0;
	}
	if (boldDriver)
		step = MINF(step*StepGrow, StepMax);
	topologyChanged = false;
	stats.step = step;
	stats.numAccepted = numAccepted;
	stats.numRejected = numRejectedTotal;
	// S has stopped moving: more iterations of this scale buy nothing
	if (numAccepted >= MinIters && numStalled >= Patience)
		return STOP;

	// the per-scale normalizer, derived at the first evaluation and then held; the vote/tanh
	// formulations already deliver a bounded direction and must not be renormalized
	if (median < 0)
		median = terms.bounded ? 1.f : ComputeMedianScale(terms);

	// move the vertices
	scratch.Empty();
	const float scale(Kappa*median);
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		// photometric step in scene units, PROPORTIONAL to the raw gradient: kappa*m converts the
		// scale's median gradient-per-footprint to one pixel, so eta is a pixel length for the
		// median seen vertex while every vertex keeps its relative magnitude (see the header for
		// why the per-vertex normalize-and-clamp this replaces was a measured 0.021 F1
		// regression); a vertex a single pair-direction saw has no triangulation behind its
		// gradient and moves on smoothing alone
		const float footprint(terms.footprint[v]);
		Grad photoDelta(Grad::ZERO);
		if (terms.photoCount[v] >= 2 && scale > 0) {
			ASSERT(footprint > 0);
			photoDelta = terms.bounded ?
				terms.photoGrad[v]*footprint : // the vote/tanh arms deliver |photoGrad| <= 1 along N
				terms.photoGrad[v]/(terms.photoCount[v]*scale);
		}
		// regularization in scene units, then the combined step: the legacy combination
		// (photoGrad/count + w*(rho*bilap - (1-rho)*lap)) with eta*P_v in place of gstep*g_v, so
		// that eta is a length in pixels instead of an opaque scene-unit factor
		const Grad regular(terms.bilap[v]*terms.rigidity - terms.lap[v]*(1.f-terms.rigidity));
		const Grad delta((photoDelta + regular*terms.regularityWeight)*-step);
		stepPrev[v] = delta;
		const float len(norm(delta));
		if (len > 0) {
			vertices[v] += delta;
			++stats.numMoved;
			// the stop rule is in pixels, so a vertex with no footprint (no pair-direction saw it,
			// it moved on smoothing alone) contributes a position change but no pixel measurement
			if (footprint > 0)
				scratch.Insert(len/footprint);
		}
	}
	stats.medianPx = scratch.IsEmpty() ? 0.f : scratch.GetMedian();
	// Nobody is moving any more, measured in pixels: this scale has converged. The test is on what
	// the median vertex WOULD move at a full step, not on what it just moved: medianPx is
	// proportional to the current eta, and eta is the bold driver's own state -- a rejection halves
	// it while an acceptance only multiplies by 1.1, so an accept/reject oscillation around a
	// plateau ratchets eta down by 0.55 per cycle and would otherwise drive medianPx under the
	// threshold within a few iterations and report convergence while the direction field is still
	// large. Measured on Truck L1: eta collapsed 0.605 -> 0.101 px over five evaluations and the
	// scale stopped at medianPx 0.023 with S improved by only 0.4 %.
	const float medianAtFullStep(step > 0 ? stats.medianPx*(StepMax/step) : 0.f);
	if (numAccepted >= MinIters && medianAtFullStep < StepStop)
		return STOP;
	return APPLY;
} // Evaluate
/*----------------------------------------------------------------*/
