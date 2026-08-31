/*
* SceneRefineStep.h
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

#ifndef _MVS_SCENEREFINESTEP_H_
#define _MVS_SCENEREFINESTEP_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Mesh.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Vertex-position stepper shared by the CPU and CUDA mesh-refinement backends.
//
// The legacy loop ran a fixed iteration count with a step in *scene units* decayed by a constant
// 0.98 per iteration, so both how far a vertex moved and when the loop gave up depended on the
// scene's absolute scale, on the image resolution and on the number of image pairs. This stepper
// works entirely in **pixels** and in **ZNCC**:
//
//   * every step length is expressed in pixels of the current scale and converted through the
//     per-vertex footprint s_v (scene units per pixel, = depth_A(v)/f_A minimized over the
//     pair-directions that saw v): eta is the step of the median seen vertex, each vertex moving
//     in proportion to its own photometric gradient;
//   * convergence is judged on S, the reliability-weighted mean of (1 - ZNCC) over every pixel of
//     every pair, which lies in [0,2] regardless of scene scale, contrast, resolution or pair
//     count.
//
// A scene scaled by 100x therefore produces the identical sequence of decisions and the identical
// eta trajectory, and the per-scale iteration count becomes a property of the surface rather than
// of the units it was reconstructed in.
//
// Per iteration, with rho the rigidity-elasticity ratio and w the regularity weight (the
// formulas below are the Terms::bounded == false path; a bounded arm delivers |photoGrad| <= 1
// along the normal and substitutes photoGrad_v * s_v for P_v, with no kappa and no m):
//
//     gamma_v = |g_v| / s_v                        g_v = photoGrad_v / c_v
//     m       = median gamma_v over seen vertices  (computed ONCE per scale, then held)
//     P_v     = g_v / (kappa * m)                  zero if c_v < 2 or m == 0
//     R_v     = rho * bilap_v - (1 - rho) * lap_v
//     D_v     = -eta * (P_v + w * R_v)
//     delta_v = |D_v| / s_v                        the per-vertex step in pixels
//
// m is fixed at the scale's first evaluation on purpose: recomputing it every iteration would
// renormalize the median vertex back to the same step every time and so defeat the stop rule.
// And m is a GLOBAL conversion factor: the median seen vertex moves eta/kappa px at the scale's
// first iteration and every other vertex moves in PROPORTION to its own gradient, exactly as the
// legacy update did. The first formulation normalized per vertex instead -- d_v =
// g_v/(kappa*m*s_v) clamped to |d_v| <= 1 -- which flattened the gradient distribution (every
// vertex past the clamp moved the same eta px no matter how much larger its gradient was) and
// lost 0.021 mean F1 on the 4-scene T&T gate, unhelped by running longer and partly rescued by
// extra smoothing; see docs/design/MeshRefinement.md section 8, the four-controls table.
// Proportionality to g_v is load-bearing; s_v converts lengths to pixels for the eta cap and the
// stop rule, never rescales the direction.
//
// The caller owns the vertices and the energy evaluation; this class owns only the step state.
// It is deliberately not included by Scene.h -- only the two backend .cpp files use it.
class MVS_API MeshRefineStep
{
public:
	typedef TPoint3<float> Grad;
	typedef CLISTDEF0IDX(Grad,uint32_t) GradArr;

	// The single operating point. These are pixel/ZNCC quantities, so they are scene-independent
	// and there is no reason to expose them: a scene that needs different numbers is a scene whose
	// footprint or S is wrong, and that is the thing to fix.
	static constexpr float StepMax = 1.f; // eta_max, px
	static constexpr float StepGrow = 1.1f; // eta *= this after an accepted evaluation
	static constexpr float StepShrink = 0.5f; // eta *= this after a rejected one
	static constexpr float StepStop = 0.05f; // eps_stop: median per-vertex step, px
	static constexpr float ProgressTol = 1e-3f; // tau_progress: relative decrease of S below which an iteration counts as stalled
	static constexpr float Kappa = 2.f; // the median seen vertex moves eta/kappa px at the first iteration
	static constexpr unsigned Patience = 3; // consecutive stalled iterations that end the scale
	static constexpr unsigned MaxRejects = 4; // consecutive rejections that end the scale
	static constexpr unsigned MinIters = 3; // no stop rule fires before this many accepted iterations

	enum Action {
		APPLY, // the vertices were moved; evaluate the energy again
		REJECT, // half the previous step was undone (vertices sit at v_prev + stepPrev/2) and eta was halved; evaluate the energy again
		STOP // converged, or out of rejects/budget: leave this scale
	};

	// One energy evaluation, as the backend already has it. Pointers are borrowed for the duration
	// of the Evaluate() call only; the arrays stay the caller's.
	struct Terms {
		const Grad* photoGrad; // sum over pair-directions of the photometric gradient (NOT divided by count)
		const float* photoCount; // c_v: how many pair-directions saw v (0 = unseen)
		const float* footprint; // s_v: scene units per pixel; 0 exactly where c_v == 0
		const Grad* lap; // first-order smoothness term (smoothGrad1), already 0 on boundary vertices
		const Grad* bilap; // second-order smoothness term (smoothGrad2), already 0 on boundary vertices
		float S; // reliability-weighted mean of (1 - ZNCC) over all pairs, in [0,2]
		float rigidity; // rho
		float regularityWeight; // w
		uint32_t numVertices;
		// the photometric term is already a bounded direction (|photoDir| <= 1 per vertex) and must
		// NOT be renormalized by the median: the sign-vote/tanh formulations deliver this, the
		// legacy magnitude formulation does not
		bool bounded;
		// this evaluation saw only one direction of each image pair, alternating with the
		// evaluation index (nAlternatePair == 1): S is then only comparable with the S of an
		// evaluation of the same parity, so two references are carried instead of one
		bool alternating;
	};

	// What one Evaluate() decided, for the caller's log line.
	struct Stats {
		float S; // the S this evaluation reported
		float step; // eta after the decision, px
		float medianPx; // median per-vertex step actually applied, px (0 when not APPLY)
		uint32_t numMoved; // vertices that received a non-zero step
		unsigned numAccepted; // accepted evaluations so far this scale
		unsigned numRejected; // consecutive rejections
	};

public:
	MeshRefineStep() { Reset(0, 0.f, true); }

	// begin a scale: eta starts at stepInit px, the median normalizer is re-derived at the next
	// evaluation, and the first evaluation is accepted unconditionally (there is nothing to
	// compare it against yet); boldDriver selects the arm (OPTREFINE::nOptimizer): true runs the
	// bold driver (REJECT + grow/shrink step, the default), false runs the fixed-step control arm
	// (every evaluation is accepted, step never changes) so the two can be compared head to head
	void Reset(uint32_t numVertices, float stepInit, bool boldDriver);

	// judge one evaluation and, when it is accepted, move the vertices
	Action Evaluate(const Terms& terms, Mesh::VertexArr& vertices, Stats& stats);

	// the mesh changed under us (planar-vertex removal): the next evaluation cannot be compared
	// against the previous S, and the undo buffer no longer indexes the same vertices
	void TopologyChanged(uint32_t numVertices);

	// a caller running two phases back to back within the same scale (different rigidity, S and
	// step carried over) calls this between them so the second phase gets its own stall budget
	// instead of inheriting one already primed to stop by the first phase's tail
	void ResetStall() { numStalled = 0; }

	float GetStep() const { return step; }
	unsigned GetNumAccepted() const { return numAccepted; }
	// index of the evaluation about to be judged; the caller must feed this to the energy as its
	// iteration number so that the pair direction an alternating run picks matches the parity this
	// class compares S against
	unsigned GetNumEvaluated() const { return numEvaluated; }

protected:
	// |g_v|/s_v of the median seen vertex, computed once per scale; NaN until then
	float ComputeMedianScale(const Terms& terms);

protected:
	float step; // eta, px
	float median; // m, the fixed per-scale normalizer; negative until the first evaluation derives it
	float scoreRef; // S of the last accepted evaluation; FLT_MAX until there is one
	float scoreRefAlt; // the same for the odd evaluation of an alternating-pair run
	unsigned numAccepted;
	unsigned numRejected; // CONSECUTIVE rejections, reset by every accepted evaluation
	unsigned numRejectedTotal; // rejections this scale, for the log line
	unsigned numStalled;
	unsigned numEvaluated; // evaluations this scale, accepted or not (selects the alternating S reference)
	bool topologyChanged; // the next evaluation is accepted unconditionally
	bool boldDriver; // false selects the fixed-step control arm: never REJECT, never grow/shrink step
	GradArr stepPrev; // the last applied per-vertex step, for the halving undo
	FloatArr scratch; // median workspace, kept across iterations so no iteration allocates
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_SCENEREFINESTEP_H_
