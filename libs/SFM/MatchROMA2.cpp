/*
 * MatchROMA2.cpp
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
 */

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "MatchROMA2.h"
#include "Scene.h"
#include "RoMa2Matcher.h"
#include "PairsMatcher.h"
#include "MatchGeometric.h"
#include "ROMA2Warp.h"

#include <deque>
#include <future>
#include <limits>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

namespace {

typedef std::vector<float> PlanarImage;

#ifdef _USE_ONNXRUNTIME

// Pool task: load (if not already resident) and preprocess one image into the planar RGB
// buffer the descriptor graph expects. Uses the working-orientation pixels (Image::LoadPixels
// rotates portrait to landscape -- design decision 12), the same pixels the keypoints were
// extracted from. Each image is touched by exactly one pool task, and by the calling thread
// only after that task's future has resolved (PrefetchRing::Take, below), so no locking is
// needed on Image.
bool PrepareImageROMA2(Image& img, int size, PlanarImage& planar)
{
	const bool hadPixels = img.HasPixels();
	if (!hadPixels && !img.LoadPixels())
		return false;
	PreprocessImageRoMa2(img.GetImage8U3(), size, planar);
	if (!hadPixels)
		img.ReleasePixels();
	return true;
}

// Pipelines image load+preprocess on the thread pool while the calling thread runs the ONNX
// Runtime call for the previously prepared buffer (the SiftGPU bulk-driver pattern,
// FeaturesExtractor.cpp:113-153, applied to ROMA2). Fixed-size ring of `capacity` planar
// buffers keyed by *submission order*, not by image: Submit(image) schedules a pool task
// writing into buffer `numSubmitted++ % capacity`, and Take() blocks on the oldest
// still-outstanding submission and hands back the buffer it filled (NULL if it could not).
// Submission order is what lets both passes share the ring: the describe pass submits every
// image once, in order, but the dense-matching pass replays the slot plan's load sequence,
// which revisits images in an arbitrary order (a slot reload describes the same image again),
// so keying by `image % capacity` would give two in-flight loads of images congruent modulo
// capacity the same buffer.
//
// Ordering invariant callers must keep, restated as a count in Submit()'s ASSERT: at most
// `capacity` submissions may be outstanding, so the (capacity+1)-th may only be issued after
// Take() consumed the first. Both passes keep it by construction (each pass issues at most one
// submission per Take()). A submission for an image another outstanding submission is already
// preparing is refused as well: PrepareImageROMA2 loads and releases that Image's pixels, so
// two concurrent tasks on one image would race on them. Only the dense-matching pass can ask
// for that (an image evicted from its slot and reloaded a few steps later); CanSubmit() tells
// it to let the prefetch window run short until the earlier submission has been taken.
//
// The destructor waits on every outstanding future: without it, a throw unwinding through
// Take()'s future::get() (RoMa2Onnx::Describe or a pool task itself can throw) would let
// planars be destroyed while a pool task is still writing into it.
class PrefetchRing
{
public:
	PrefetchRing(Scene& _scene, unsigned capacity, int _size)
		: scene(_scene), size(_size), planars(capacity) {}
	PrefetchRing(const PrefetchRing&) = delete;
	PrefetchRing& operator=(const PrefetchRing&) = delete;

	~PrefetchRing() {
		while (!pending.empty()) {
			pending.front().future.wait(); // never get(): a destructor must not propagate an exception
			pending.pop_front();
		}
	}

	// true if a submission for the given image may be issued right now (see the class comment)
	bool CanSubmit(IIndex image) const {
		if (pending.size() >= planars.size())
			return false;
		for (const Submission& submission : pending)
			if (submission.image == image)
				return false;
		return true;
	}

	void Submit(IIndex image) {
		ASSERT(CanSubmit(image));
		const size_t idxPlanar = numSubmitted++ % planars.size();
		pending.push_back(Submission{image, scene.threadPool.submit_task([this, image, idxPlanar]() {
			return PrepareImageROMA2(scene.images[image], size, planars[idxPlanar]);
		})});
	}

	// The buffer the oldest outstanding submission prepared, or NULL if its image could not be
	// prepared; stays valid until `capacity` further submissions have been issued.
	const PlanarImage* Take() {
		ASSERT(!pending.empty());
		const bool prepared = pending.front().future.get();
		pending.pop_front();
		const PlanarImage& planar = planars[numTaken++ % planars.size()];
		return prepared ? &planar : NULL;
	}

private:
	struct Submission {
		IIndex image;             // image this submission prepares (CanSubmit rejects a second one)
		std::future<bool> future; // whether the pool task managed to prepare it
	};

	Scene& scene;
	int size;
	std::vector<PlanarImage> planars;
	std::deque<Submission> pending;
	size_t numSubmitted = 0, numTaken = 0; // both count buffers of the ring, modulo its capacity
};

// Restores a pass's OpenCV thread count, log-console pause, and progress bar on scope exit,
// including through an exception (they used to be plain statements reachable only on the
// non-throwing path). Finish() runs the restoration immediately and is what the normal path
// calls, right before the summary DEBUG line, so that line isn't swallowed by the paused
// console; the destructor calls it too (idempotent) as the safety net for any exceptional exit.
// Shared by the describe and the dense-matching passes, which differ only in the progress
// caption and in how many steps it counts.
struct ScopedPassState
{
	Scene& scene;
	Util::Progress progress;
	bool bFinished = false;

	ScopedPassState(Scene& _scene, LPCTSTR caption, size_t numSteps)
		: scene(_scene), progress(caption, numSteps)
	{
		cv::setNumThreads(1); // temporarily turn off multi-threading for OpenCV functions (the pool tasks are already parallel)
		GET_LOGCONSOLE().Pause();
	}
	ScopedPassState(const ScopedPassState&) = delete;
	ScopedPassState& operator=(const ScopedPassState&) = delete;
	~ScopedPassState() { Finish(); }

	void Finish() {
		if (bFinished)
			return;
		bFinished = true;
		GET_LOGCONSOLE().Play();
		progress.close();
		cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading
	}
};

// Waits for every task of the scene thread pool on scope exit. MatchPairsROMA2's consumers are
// detached (they hand nothing back but the result slot they fill), so this is what guarantees
// none of them is still running when the locals they capture by reference die -- on the normal
// path, where the explicit wait has already drained them, and on an exceptional one alike.
struct ScopedPoolDrain
{
	Scene& scene;

	explicit ScopedPoolDrain(Scene& _scene) : scene(_scene) {}
	ScopedPoolDrain(const ScopedPoolDrain&) = delete;
	ScopedPoolDrain& operator=(const ScopedPoolDrain&) = delete;
	~ScopedPoolDrain() { scene.threadPool.wait(); }
};

// Returns one permit of the in-flight semaphore on scope exit. MatchPairsROMA2's producer takes
// a permit before it hands a warp to a consumer, so a consumer that left without returning one
// (the verdict, the guided match and the assembly can all throw) would starve the producer
// permanently; this keeps the count balanced on every path out of the task.
struct ScopedSemaphore
{
	Semaphore& semaphore;

	explicit ScopedSemaphore(Semaphore& _semaphore) : semaphore(_semaphore) {}
	ScopedSemaphore(const ScopedSemaphore&) = delete;
	ScopedSemaphore& operator=(const ScopedSemaphore&) = delete;
	~ScopedSemaphore() { semaphore.Signal(); }
};

// Which device slot each image of each pair is described into, and in which order the slots
// are (re)loaded: one Step per pair, in the order MatchPairsROMA2 walks the pairs.
struct SlotPlan {
	struct Load {
		IIndex image;  // image to describe
		unsigned slot; // slot its description overwrites
	};
	struct Step {
		unsigned slotA = NO_ID, slotB = NO_ID; // slots holding the pair's two images once `loads` ran
		CLISTDEF0(Load) loads;                 // slots to (re)load before this pair can be matched
	};
	std::vector<Step> steps;
	unsigned numSlots = 0;               // slots the plan actually uses (at most the budget)
	size_t numLoads = 0, numReloads = 0; // describe calls the plan costs, and how many are reloads
};

// Plan the device slots of the given pairs: Belady's optimal replacement, the semantics of
// polycpp's plan.cpp with a single step in flight (ONNX Runtime's Run is synchronous, so the
// producer never has more than one pair's descriptors in preparation). The pairs come in
// (ID1,ID2) order, which by itself keeps every use of an image inside a short window
// (plan.hpp:38-45); once the pool is full, the resident image whose next use is furthest away
// -- or which is never needed again -- is the one overwritten. The pool only grows towards the
// budget when it has to, so a scene with fewer images than slots simply keeps them all resident
// and the plan degenerates to one load per image.
SlotPlan MakeSlotPlan(const PairIdxArr& pairs, unsigned slotBudget, IIndex nImages)
{
	SlotPlan plan;
	const unsigned budget = MAXF(slotBudget, 2u); // a pair always needs two slots
	// steps every image is used at, in increasing order, and how many of them are already past
	std::vector<std::vector<size_t>> uses(nImages);
	FOREACH(p, pairs) {
		uses[pairs[p].i].push_back(p);
		uses[pairs[p].j].push_back(p);
	}
	std::vector<size_t> cursor(nImages, 0);
	const auto NextUseAfter = [&](IIndex image, size_t position) {
		for (size_t u = cursor[image]; u < uses[image].size(); ++u)
			if (uses[image][u] > position)
				return uses[image][u];
		return std::numeric_limits<size_t>::max();
	};
	IIndexArr imageInSlot;                            // image currently resident in each slot
	std::vector<unsigned> slotOfImage(nImages, NO_ID); // slot each image is resident in, if any
	plan.steps.resize(pairs.size());
	FOREACH(p, pairs) {
		const PairIdx pair(pairs[p]);
		SlotPlan::Step& step = plan.steps[p];
		for (const IIndex image : {pair.i, pair.j}) {
			if (slotOfImage[image] != NO_ID)
				continue; // still resident from an earlier step
			unsigned slot = NO_ID;
			size_t furthestUse = 0;
			FOREACH(s, imageInSlot) {
				const IIndex resident = imageInSlot[s];
				if (resident == pair.i || resident == pair.j)
					continue; // this very pair needs it
				const size_t next = NextUseAfter(resident, p);
				if (next == std::numeric_limits<size_t>::max()) {
					slot = (unsigned)s; // never needed again: reuse it rather than grow the pool
					break;
				}
				if (imageInSlot.size() >= budget && (slot == NO_ID || next > furthestUse)) {
					slot = (unsigned)s;
					furthestUse = next;
				}
			}
			if (slot == NO_ID) {
				slot = (unsigned)imageInSlot.size();
				imageInSlot.push_back(image);
			} else {
				slotOfImage[imageInSlot[slot]] = NO_ID;
				imageInSlot[slot] = image;
			}
			slotOfImage[image] = slot;
			step.loads.push_back({image, slot});
			++plan.numLoads;
		}
		step.slotA = slotOfImage[pair.i];
		step.slotB = slotOfImage[pair.j];
		++cursor[pair.i];
		++cursor[pair.j];
	}
	plan.numSlots = (unsigned)imageInSlot.size();
	size_t numUsedImages = 0;
	for (const std::vector<size_t>& imageUses : uses)
		if (!imageUses.empty())
			++numUsedImages;
	ASSERT(plan.numLoads >= numUsedImages); // every used image is loaded at least once
	plan.numReloads = plan.numLoads - numUsedImages;
	return plan;
}

// What one warp pass cost, for its caller's summary line
struct WarpPassStats {
	unsigned numSlots = 0;               // device slots the plan used
	size_t numLoads = 0, numReloads = 0; // describe calls it cost, and how many were reloads
	unsigned numFailedLoads = 0;         // images that could not be loaded or described
	unsigned numFailedMatches = 0;       // pairs the coarse-match graph could not warp
};

// One pair's bidirectional warp, handed to a pool thread: the pair's index in the pass's list, its
// images, the warps themselves (the task owns them) and the index of the private descriptor
// matcher it may use.
typedef std::function<void(size_t idxPair, const PairIdx& pair, PairWarps& warps, unsigned threadIdx)> WarpConsumer;

// Drive the ROMAv2 coarse-match graph over the given pairs, handing each pair's bidirectional warp
// to the thread pool: plan the device slots (Belady over the pairs in (ID1,ID2) order), pipeline the
// image load+preprocess on the pool while this thread runs Describe and MatchCoarse, and detach one
// `consume` task per warped pair. Both directions come out of the SAME MatchCoarse call, so only the
// (A,B) pair with A < B is ever warped. The pairs must already be filtered to the ones worth a warp
// and sorted in (ID1,ID2) order -- the slot plan's locality, and the order results are stored in.
// The semaphore bounds how many warps are alive at once (~600 KB per pair at base). Every pool task
// is drained, and the OpenCV thread count, log console and progress bar restored, on every exit path
// including an exceptional one (Describe, MatchCoarse and the consumers can all throw).
// Returns false only when the device slot pool could not be allocated (nothing was warped).
bool ForEachWarpROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& pairs,
	unsigned slotBudget, LPCTSTR progressCaption, const WarpConsumer& consume, WarpPassStats& stats)
{
	ASSERT(!pairs.empty() && std::is_sorted(pairs.begin(), pairs.end()));
	Scene& scene = pairsMatcher.GetScene();
	const unsigned nThreads = (unsigned)scene.threadPool.get_thread_count();

	// device slots: every descriptor tensor is allocated up front, so a pool that does not fit is a
	// clean early return instead of a failure halfway through the pass. They are locals (never
	// members of the PairsMatcher): a tensor from the descriptor session's device arena must be
	// destroyed before the RoMa2Onnx that owns it (see RoMa2Onnx::MakeLayers).
	const SlotPlan plan(MakeSlotPlan(pairs, slotBudget, (IIndex)scene.images.size()));
	std::vector<OrtTensor> slotLayers(plan.numSlots);
	for (OrtTensor& layers : slotLayers) {
		layers = roma2.MakeLayers();
		if (!layers.IsValid()) {
			VERBOSE("error: ROMA2 slot pool: could not allocate the %u descriptor slots the plan needs (lower --roma2-slots)", plan.numSlots);
			return false;
		}
	}
	IIndexArr slotImage(plan.numSlots);
	slotImage.Memset(0xFF); // NO_ID: slot empty or its load failed; a pair reading it is dropped, never matched against a stale image
	stats.numSlots = plan.numSlots;
	stats.numLoads = plan.numLoads;
	stats.numReloads = plan.numReloads;
	DEBUG_EXTRA("ROMA2 slot plan: %u pairs, %u slots, %u loads (%u reloads)",
		pairs.size(), plan.numSlots, (unsigned)plan.numLoads, (unsigned)plan.numReloads);

	Semaphore inFlight(2*nThreads);
	// the plan's load sequence, flattened into the order the ring prefetches images in
	std::vector<SlotPlan::Load> loads;
	loads.reserve(plan.numLoads);
	for (const SlotPlan::Step& step : plan.steps)
		for (const SlotPlan::Load& load : step.loads)
			loads.push_back(load);
	ScopedPassState state(scene, progressCaption, pairs.size());
	PrefetchRing ring(scene, MINF(2u*nThreads, 8u), roma2.ImageSize());
	// the detached consumers capture every local above by reference, so the pool must be drained
	// before any of them dies -- on the exceptional path too. Declared last, so that its
	// destructor is the first one to run.
	const ScopedPoolDrain drain(scene);
	size_t nextLoad = 0, nextSubmit = 0;
	// Describe()'s retrieval readback is not optional, but the warp pass has no use for a
	// retrieval descriptor: written and discarded every load, reused so the loop does not churn
	std::vector<float> discardedRetrieval;
	FOREACH(p, pairs) {
		const PairIdx pair(pairs[p]);
		const SlotPlan::Step& step = plan.steps[p];
		for (const SlotPlan::Load& load : step.loads) {
			ASSERT(nextLoad < loads.size() && loads[nextLoad].image == load.image && loads[nextLoad].slot == load.slot);
			// keep the prefetch window as full as the ring allows; the load about to be taken is
			// always submittable, since when it is the next submission the ring holds nothing
			while (nextSubmit < loads.size() && ring.CanSubmit(loads[nextSubmit].image))
				ring.Submit(loads[nextSubmit++].image);
			ASSERT(nextSubmit > nextLoad);
			const PlanarImage* const planar = ring.Take();
			slotImage[load.slot] = NO_ID;
			// value_facets stays on the device: the warp pass never reads it back (design decision 4)
			if (planar && roma2.Describe(planar->data(), slotLayers[load.slot], NULL, discardedRetrieval)) {
				slotImage[load.slot] = load.image;
			} else {
				++stats.numFailedLoads;
				DEBUG_EXTRA("error: could not describe image %u '%s'",
					scene.images[load.image].ID, scene.images[load.image].fileName.c_str());
			}
			++nextLoad;
		}
		if (slotImage[step.slotA] != pair.i || slotImage[step.slotB] != pair.j) {
			++state.progress; // a load failed: drop the pair rather than match it against a stale slot
			continue;
		}
		PairWarps warps;
		if (!roma2.MatchCoarse(slotLayers[step.slotA], slotLayers[step.slotB],
			warps.ab.warp, warps.ab.confidence, warps.ba.warp, warps.ba.confidence)) {
			++stats.numFailedMatches;
			DEBUG_EXTRA("error: could not dense match pair (% 4u, % 4u)", pair.i, pair.j);
			++state.progress;
			continue;
		}
		ASSERT(warps.IsValid() && warps.ab.warp.rows == roma2.WarpSize());
		inFlight.Wait();
		scene.threadPool.detach_task([&, p, pair, warps = std::move(warps)]() mutable {
			const ScopedSemaphore released(inFlight); // returned however this task ends (see the struct)
			const std::optional<size_t> threadIdx = BS::this_thread::get_index();
			ASSERT(threadIdx && *threadIdx < pairsMatcher.GetNumMatchers());
			consume(p, pair, warps, (unsigned)*threadIdx);
			++state.progress;
		});
	}
	scene.threadPool.wait();
	state.Finish(); // unpause the log console before the caller's summary line (see the struct comment)
	return true;
}

#endif // _USE_ONNXRUNTIME

// Target size of the coverage-uniform sample the verdict fits its one geometry on. A target, not a
// cap (SampleWarpByCoverage): large enough that a spread sample of a genuine overlap conditions the
// estimator well, small enough that a RANSAC per candidate pair stays affordable. It is a cost, not
// a resolution -- the areas the verdict decides on are measured over ALL eligible cells, not over
// the sample.
constexpr unsigned VERDICT_SAMPLE = 4000;

// Every eligible cell of one warp direction, in raster order. Eligible means what it means
// everywhere else on a warp (CollectWarpCandidates, ROMA2Warp.cpp): confidence at or above
// minConfidence AND the warped point landing inside the target image. `ptsSrc` are the cell centres
// in the pixels of the working orientation of the source image, `ptsDst` the warped points in the
// target image, index-parallel with `confidences`.
// Returns the number of eligible cells.
size_t CollectEligibleCells(const WarpMaps& maps, const cv::Size& sizeSrc, const cv::Size& sizeDst,
	float minConfidence, std::vector<Point2f>& ptsSrc, std::vector<Point2f>& ptsDst, std::vector<float>& confidences)
{
	ASSERT(maps.IsValid());
	ptsSrc.clear();
	ptsDst.clear();
	confidences.clear();
	for (int y = 0; y < maps.confidence.rows; ++y) {
		for (int x = 0; x < maps.confidence.cols; ++x) {
			const float conf = maps.confidence(y, x);
			if (conf < minConfidence)
				continue;
			const Point2f ptDst(DenormCoord(maps.warp(y, x), sizeDst));
			if (!Image8U::isInside(ptDst, sizeDst))
				continue; // the warp sends this cell outside the other image
			ptsSrc.push_back(CoordFromTo(Point2f((float)x, (float)y), maps.confidence.size(), sizeSrc));
			ptsDst.push_back(ptDst);
			confidences.push_back(conf);
		}
	}
	return ptsSrc.size();
}

// One correspondence measured against the geometry a fit produced: the first-order (Sampson)
// distance, compared against a tolerance. Through F where the fit produced one -- in pixels, the
// units WarpTolerance and MatchConfig::maxEpipolarError are both expressed in -- and otherwise
// through E on calibrated bearings, with the pixel tolerance converted per camera exactly as
// PairsMatcher::GeometricFilter converts its own. The second path is not a fallback but the same
// test in the other space: F is not geometrically meaningful on a spherical or mixed pair
// (SphericalCamera::GetK is the identity, so GeometricFilter leaves F unset), and such a pair has
// to be judged rather than silently rejected for want of a matrix.
// The convention is the matcher's own throughout: F maps a point of the FIRST image to its epipolar
// line in the second (PairsMatcher::GeometricFilter composes it that way, MatchGeometric reads it
// that way), so a correspondence is always given as (A side, B side).
class SampsonTest
{
public:
	SampsonTest(const Image& imgA, const Image& imgB, const ImagePair& pair, float tolerance)
		: camA(imgA.pCamera), camB(imgB.pCamera), bFundamental(pair.F.has_value())
	{
		if (bFundamental) {
			M = pair.F.value();
			toleranceSq = (double)tolerance*(double)tolerance;
		} else if (pair.E.has_value()) {
			M = pair.E.value();
			// the symmetric averaging GeometricFilter uses to turn a pixel threshold into the
			// radians its bearing estimator scores in, so both spaces demand the same precision
			const double angle = 0.5*(double)(camA->PixelErrorToAngular((REAL)tolerance) +
				camB->PixelErrorToAngular((REAL)tolerance));
			toleranceSq = angle*angle;
		} else {
			toleranceSq = -1.0; // the fit produced no geometry at all: nothing can be scored
		}
	}

	inline bool IsValid() const { return toleranceSq >= 0.0; }

	// true if the correspondence lies within the tolerance of the geometry; compared squared, so
	// no square root runs per warp cell
	inline bool operator()(const Point2f& ptA, const Point2f& ptB) const {
		ASSERT(IsValid());
		Eigen::Vector3d a, b;
		if (bFundamental) {
			a = Eigen::Vector3d(ptA.x, ptA.y, 1.0);
			b = Eigen::Vector3d(ptB.x, ptB.y, 1.0);
		} else {
			a = camA->UnprojectNormalized(Cast<REAL>(ptA));
			b = camB->UnprojectNormalized(Cast<REAL>(ptB));
		}
		const Eigen::Vector3d Ma(M*a), Mtb(M.transpose()*b);
		const double den = Ma.x()*Ma.x() + Ma.y()*Ma.y() + Mtb.x()*Mtb.x() + Mtb.y()*Mtb.y();
		if (den < 1e-14)
			return false; // degenerate epipolar geometry here (the bearing lies along the baseline)
		const double num = b.dot(Ma);
		return num*num <= toleranceSq*den;
	}

private:
	CameraPtr camA, camB;   // only read for the bearings of the E path
	Eigen::Matrix3d M;      // F in pixels, or E on unit bearings
	double toleranceSq;     // squared, in the units of M's space
	bool bFundamental;
};

// Inverse of DenormCoord: a pixel position of an image back to the normalized (align_corners=false)
// coordinate a warp map stores.
inline Point2f NormCoord(const Point2f& coord, const cv::Size& size)
{
	return Point2f(
		2.f * (coord.x + 0.5f) / (float)size.width - 1.f,
		2.f * (coord.y + 0.5f) / (float)size.height - 1.f
	);
}

// Put the verdict's inlier cells back on the warp grid they were read off: a warpSize x warpSize
// confidence map holding each inlier cell's own confidence and 0 everywhere else, plus the matching
// normalized warp. This is what lets the dense fill run through the ONE warp draw
// (SampleWarpComplementary) instead of a second implementation of the same stratification -- same
// eligibility bar, same bucket sizing, same lattice winner rule, hence the same cross-pair keypoint
// identity FilterRedundantKeypoints chains tracks through.
// The cell of an inlier is recovered from its A position rather than carried through PairVerdict:
// the verdict wrote it as CoordFromTo(cell, grid, sizeA), a linear map with no half-pixel term, so
// the inverse lands back on the same integer cell.
void RebuildInlierWarp(const PairVerdict& verdict, const cv::Size& sizeA, const cv::Size& sizeB,
	int warpSize, Image32F2& warp, Image32F& confidence)
{
	ASSERT(verdict.inliersA.size() == verdict.inliersB.size() &&
		verdict.inliersA.size() == verdict.confidences.size());
	const cv::Size gridSize(warpSize, warpSize);
	warp.create(gridSize);
	confidence.create(gridSize);
	confidence.memset(0);
	// a normalized coordinate far outside [-1,1] in every cell the verdict did not keep: DenormCoord
	// sends it well outside imgB, so such a cell fails the draw's in-frame test whatever the
	// confidence floor is -- a zero floor included, where a zeroed confidence would admit it and a
	// zeroed warp would point every one of them at the centre of imgB
	for (int y = 0; y < warpSize; ++y)
		for (int x = 0; x < warpSize; ++x)
			warp(y, x) = Point2f(-3.f, -3.f);
	FOREACH(k, verdict.inliersA) {
		const Point2f cell(CoordFromTo(verdict.inliersA[k], sizeA, gridSize));
		const int x = MINF(MAXF(ROUND2INT(cell.x), 0), warpSize-1);
		const int y = MINF(MAXF(ROUND2INT(cell.y), 0), warpSize-1);
		confidence(y, x) = verdict.confidences[k];
		warp(y, x) = NormCoord(verdict.inliersB[k], sizeB);
	}
}

// Arm the temporary Image copies every fit in this file runs on: the given correspondences as their
// keypoints, and NO pose -- so that a scene which happens to hold a ground-truth solution cannot
// leak it into a fitted geometry however the estimator later changes. Enforced structurally rather
// than trusted, because judging a pair on its warp alone is the whole point.
void MakeFitImages(const std::vector<Point2f>& pointsA, const std::vector<Point2f>& pointsB,
	Image& imgACopy, Image& imgBCopy, ImagePair& fit)
{
	ASSERT(pointsA.size() == pointsB.size());
	imgACopy.InvalidatePose();
	imgBCopy.InvalidatePose();
	ASSERT(!imgACopy.HasPose() && !imgBCopy.HasPose());
	imgACopy.keypoints = ConvertToKeypoints(pointsA);
	imgBCopy.keypoints = ConvertToKeypoints(pointsB);
	fit.matches.reserve(pointsA.size());
	for (uint32_t i = 0; i < (uint32_t)pointsA.size(); ++i)
		fit.matches.emplace_back(i, i);
}

} // namespace


// D E S C R I B E   P A S S //////////////////////////////////////////

unsigned SFM::ComputeGlobalDescriptorsROMA2(Scene& scene, RoMa2Onnx& roma2)
{
#ifdef _USE_ONNXRUNTIME
	ASSERT(roma2.IsLoaded());
	TD_TIMER_STARTD();
	const IIndex nImages = (IIndex)scene.images.size();
	const int size = roma2.ImageSize();
	const unsigned nPrefetch = MINF(2u*(unsigned)scene.threadPool.get_thread_count(), 8u);

	// destruction order matters on every exit path (normal or exceptional): ring must destruct
	// before state, so it is declared after state (reverse declaration order == destruction
	// order) -- every pool task is drained before cv::setNumThreads/log console are restored
	ScopedPassState state(scene, _T("Describe images"), nImages);
	PrefetchRing ring(scene, nPrefetch, size);
	for (IIndex i = 0; i < MINF(nPrefetch, nImages); ++i)
		ring.Submit(i);

	unsigned numDescribed = 0;
	OrtTensor layers(roma2.MakeLayers()); // reused across every image (design decision 7: the describe pass keeps only the pooled vector, not the layers)
	if (!layers.IsValid()) {
		VERBOSE("error: could not allocate the ROMA2 layers tensor");
		return 0;
	}
	std::vector<float> descriptor;
	for (IIndex i = 0; i < nImages; ++i, ++state.progress) {
		// consume the buffer this image was prefetched into before ever reusing it below
		const PlanarImage* const planar = ring.Take();
		Image& img = scene.images[i];
		// the graph's own on-device retrieval pooling (RoMa2Onnx::Describe), read straight into
		// descriptor -- the CPU pooling this pass used to fall back to is gone, one path remains
		if (planar && roma2.Describe(planar->data(), layers, NULL, descriptor)) {
			img.globalDescriptor = cv::Mat(1, (int)descriptor.size(), CV_32F, descriptor.data()).clone();
			++numDescribed;
		} else {
			VERBOSE("error: could not describe image %u '%s'", img.ID, img.fileName.c_str());
		}
		// only now may the buffer this image just consumed be overwritten by a new submission
		if (i + nPrefetch < nImages)
			ring.Submit(i + nPrefetch);
	}
	state.Finish(); // unpause the log console before the summary line below (see struct comment)

	// descriptor may be empty (no image was ever successfully described): fall back to the
	// manifest's declared dimension so the summary line stays meaningful
	const unsigned descriptorDim = descriptor.empty() ? roma2.Manifest().facetsDim : (unsigned)descriptor.size();
	DEBUG("Global descriptors computed for %u/%u images (%u-D, %s provider, %s)",
		numDescribed, (unsigned)nImages, descriptorDim, roma2.ProviderName().c_str(), TD_TIMER_GET_FMT().c_str());
	return numDescribed;
#else // _USE_ONNXRUNTIME
	// unreachable: RoMa2Onnx::IsAvailable() is false in this build, so Scene::MatchPairs never
	// loads a model and never calls here
	ASSERT(false);
	return 0;
#endif // _USE_ONNXRUNTIME
}
/*----------------------------------------------------------------*/


// T H E   V E R D I C T //////////////////////////////////////////////

void SFM::JudgePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairWarps& warps, const ROMA2Config& config, ImagePair& pair, PairVerdict& verdict)
{
	ASSERT(warps.IsValid());
	ASSERT(imgA.HasCamera() && imgB.HasCamera());
	verdict = PairVerdict();
	pair.Reset();
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());
	const int warpSize = warps.ab.warp.rows;
	// areas are fractions of the WHOLE C x C grid, never of the eligible cells: a warp confident
	// about a corner of the frame has to read as a corner, which a fraction of the confident cells
	// would hide (it is 1 by construction)
	const float numCells = (float)(warps.ab.warp.rows*warps.ab.warp.cols);

	// the two populations the verdict is decided on: each direction's cells the model is confident
	// about and which land inside the other image. The B side is the precision of the whole rule --
	// it is what rejects the pairs that put the whole of A on a few pixels of B.
	std::vector<Point2f> cellsAinA, cellsAinB, cellsBinA, cellsBinB;
	std::vector<float> confidencesA, confidencesB;
	CollectEligibleCells(warps.ab, sizeA, sizeB, config.minConfidence, cellsAinA, cellsAinB, confidencesA);
	CollectEligibleCells(warps.ba, sizeB, sizeA, config.minConfidence, cellsBinB, cellsBinA, confidencesB);
	verdict.confidentAreaA = (float)cellsAinA.size()/numCells;
	verdict.confidentAreaB = (float)cellsBinB.size()/numCells;

	// ONE geometry, fitted on a coverage-uniform sample of A's cells alone. Nothing is pre-selected
	// along the epipolar lines of a geometry the warp itself supplied, which is what makes the
	// verdict independent of the warp's own claim: the sample is exactly what the warp asserts,
	// chosen for spread and confidence only, and one geometry either explains it or does not.
	std::vector<Point2f> sampledA, sampledB;
	float coverageA, coverageB;
	SampleWarpByCoverage(imgA, imgB, warps.ab.warp, warps.ab.confidence, config.minConfidence,
		VERDICT_SAMPLE, sampledA, sampledB, coverageA, coverageB);
	if (sampledA.size() < 8) {
		// the estimator needs 8 correspondences of its own; a warp that cannot offer that many
		// confident, spread-out cells is no evidence about the pair
		pair.Reset();
		return;
	}
	// the tolerance every test on warp cells uses, including this fit: half a warp cell, the
	// accuracy a coarse-warp correspondence can claim. Demanding the descriptor path's sub-pixel
	// precision of a 160-cell grid would reject the true pairs along with the false ones.
	const float tolerance = WarpTolerance(sizeA, sizeB, warpSize);
	Image imgACopy(imgA.ID, imgA.fileName, Pose3D(), imgA.cameraID, imgA.pCamera);
	Image imgBCopy(imgB.ID, imgB.fileName, Pose3D(), imgB.cameraID, imgB.pCamera);
	MakeFitImages(sampledA, sampledB, imgACopy, imgBCopy, pair);
	if (!pairsMatcher.GeometricFilter(imgACopy, imgBCopy, pair, tolerance)) {
		pair.Reset(); // no single geometry explained enough of the sample to survive the estimator
		return;
	}
	// the fit's matches index the temporary copies, not the two images: the pair keeps the geometry
	// the branch produced and nothing else
	pair.ResetMatches();
	const SampsonTest inlier(imgA, imgB, pair, tolerance);
	if (!inlier.IsValid()) {
		pair.Reset(); // the branch left neither F nor E, so its inlier area cannot be measured
		return;
	}

	// A's inlier area, and with it the population the dense fill draws from
	verdict.inliersA.reserve(cellsAinA.size());
	verdict.inliersB.reserve(cellsAinA.size());
	verdict.confidences.reserve(cellsAinA.size());
	FOREACH(k, cellsAinA) {
		if (!inlier(cellsAinA[k], cellsAinB[k]))
			continue;
		verdict.inliersA.push_back(cellsAinA[k]);
		verdict.inliersB.push_back(cellsAinB[k]);
		verdict.confidences.push_back(confidencesA[k]);
	}
	verdict.inlierAreaA = (float)verdict.inliersA.size()/numCells;
	// B's inlier area against the SAME geometry, each of B's cells taken as the correspondence
	// (where the B->A warp sends it in A, its own centre in B)
	size_t numInliersB = 0;
	FOREACH(k, cellsBinB)
		if (inlier(cellsBinA[k], cellsBinB[k]))
			++numInliersB;
	verdict.inlierAreaB = (float)numInliersB/numCells;

	// the whole rule: an inlier COUNT or RATIO on the warp's own cells cannot tell a hallucinated
	// warp from a true pair (a hallucination is locally a homography, and every homography is
	// explained exactly by a family of fundamental matrices), but the min-side inlier AREA can
	verdict.admitted = MINF(verdict.inlierAreaA, verdict.inlierAreaB) >= config.minOverlap;
	if (!verdict.admitted) {
		// a rejected pair is dropped: no descriptor matching, no second chance in this round, and
		// nothing downstream reads its cells, so they go back now
		pair.Reset();
		verdict.inliersA = std::vector<Point2f>();
		verdict.inliersB = std::vector<Point2f>();
		verdict.confidences = std::vector<float>();
	}
}
/*----------------------------------------------------------------*/


// P A I R   A S S E M B L Y   A N D   S T O R A G E //////////////////

bool SFM::AssemblePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairVerdict& verdict, const std::vector<DMatch>& guided, const ROMA2Config& config, int warpSize,
	ImagePair& pair, DenseMatches& dense)
{
	ASSERT(verdict.admitted);
	ASSERT(pair.matches.empty() && pair.GetNumDenseInliers() == 0);
	ASSERT(warpSize > 0);
	dense = DenseMatches();
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());

	// 1) the dense fill: the verdict's inlier cells where the guided candidates are NOT. Occupancy
	// is read in A's frame, the frame the warp grid lives in and the only one where a keypoint
	// position and a warp cell are directly comparable.
	std::vector<Point2f> occupiedA;
	occupiedA.reserve(guided.size());
	for (const DMatch& match : guided) {
		ASSERT((size_t)match.queryIdx < imgA.NumDescribedKeypoints());
		occupiedA.push_back(imgA.keypoints[match.queryIdx].pt);
	}
	{
		Image32F2 inlierWarp;
		Image32F inlierConfidence;
		RebuildInlierWarp(verdict, sizeA, sizeB, warpSize, inlierWarp, inlierConfidence);
		SampleWarpComplementary(imgA, imgB, inlierWarp, inlierConfidence, config.minConfidence,
			config.denseMatches, occupiedA, dense.pointsA, dense.pointsB, dense.confidences);
	}

	// 2) ONE geometry for the pair, fitted on guided u dense through the matcher's own estimator at
	// its own threshold: the sparse correspondences are sub-pixel where texture exists and the dense
	// ones carry the parts of the overlap they left empty, so the union is the best-conditioned
	// evidence the pair has. On failure the verdict's geometry stands, unchanged -- an admitted pair
	// is always stored, and a pair holding one fit's pose next to another fit's matrices would
	// describe two geometries as one, which is why all three members move together or none do.
	const size_t numTotal = guided.size() + dense.pointsA.size();
	if (numTotal >= 8) {
		std::vector<Point2f> pointsA, pointsB;
		pointsA.reserve(numTotal);
		pointsB.reserve(numTotal);
		for (const DMatch& match : guided) {
			pointsA.push_back(imgA.keypoints[match.queryIdx].pt);
			pointsB.push_back(imgB.keypoints[match.trainIdx].pt);
		}
		pointsA.insert(pointsA.end(), dense.pointsA.begin(), dense.pointsA.end());
		pointsB.insert(pointsB.end(), dense.pointsB.begin(), dense.pointsB.end());
		Image imgACopy(imgA.ID, imgA.fileName, Pose3D(), imgA.cameraID, imgA.pCamera);
		Image imgBCopy(imgB.ID, imgB.fileName, Pose3D(), imgB.cameraID, imgB.pCamera);
		ImagePair fit(pair.ID1, pair.ID2);
		MakeFitImages(pointsA, pointsB, imgACopy, imgBCopy, fit);
		if (pairsMatcher.GeometricFilter(imgACopy, imgBCopy, fit)) {
			// straight from `fit` rather than recomposed here: GeometricFilter ran the branch's own
			// estimator and left F/E in that branch's own convention (on SHARED_FOCAL it composes F
			// from the RANSAC-estimated focal, not from the camera's nominal K), so recomposing
			// would silently disagree with the pose it just fitted
			pair.F = fit.F;
			pair.E = fit.E;
			pair.relativePose = fit.relativePose;
		}
	}

	// 3) classify both kinds of correspondence against the kept geometry, each at its own precision:
	// a descriptor correspondence is sub-pixel and is held to the matcher's own epipolar error, a
	// warp cell can only claim half a cell
	const MatchConfig& cfg = pairsMatcher.GetConfig();
	// maxEpipolarError 0 turns the RANSAC verification off scene-wide (MatchConfig), and with it the
	// sparse epipolar test: in that configuration every guided match stands, exactly as the
	// descriptor path keeps its matches unverified
	const bool bVerifySparse = cfg.maxEpipolarError > 0.f;
	const SampsonTest sparseTest(imgA, imgB, pair, cfg.maxEpipolarError);
	const SampsonTest denseTest(imgA, imgB, pair, WarpTolerance(sizeA, sizeB, warpSize));
	ASSERT(sparseTest.IsValid() == denseTest.IsValid());
	pair.matches.clear();
	pair.matches.reserve(guided.size());
	pair.outlierMatches.clear();
	for (const DMatch& match : guided) {
		if (!bVerifySparse || !sparseTest.IsValid() ||
			sparseTest(imgA.keypoints[match.queryIdx].pt, imgB.keypoints[match.trainIdx].pt))
			pair.matches.push_back(match);
		else
			pair.outlierMatches.push_back(match);
	}
	// the sparse segment is materialised here rather than left at -1: the dense segment goes right
	// after it (ImagePair's partition), and StorePairROMA2 appends it as soon as the pair exists
	pair.numFilteredInliers = (int)pair.matches.size();
	pair.numDenseInliers = 0;
	pair.weightedInliers = -1.f;
	size_t numDenseKept = 0;
	for (size_t k = 0; k < dense.pointsA.size(); ++k) {
		if (denseTest.IsValid() && !denseTest(dense.pointsA[k], dense.pointsB[k]))
			continue;
		dense.pointsA[numDenseKept] = dense.pointsA[k];
		dense.pointsB[numDenseKept] = dense.pointsB[k];
		dense.confidences[numDenseKept] = dense.confidences[k];
		++numDenseKept;
	}
	dense.pointsA.resize(numDenseKept);
	dense.pointsB.resize(numDenseKept);
	dense.confidences.resize(numDenseKept);
	// the pseudo-baseline of the sparse segment, so that a pair stored with no dense fill still
	// carries the one weight term that can demote a degenerate baseline (AppendDenseMatches
	// recomputes it over both segments for a pair that does get one)
	if (pair.relativePose.has_value() && !pair.matches.empty())
		pair.meanRayAngle = pair.ComputeMeanRayAngle(imgA, imgB);
	return !pair.matches.empty() || numDenseKept > 0;
}

void SFM::StorePairROMA2(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
	ImagePair&& pair, const DenseMatches& dense, int warpSize)
{
	ASSERT(pair.ID1 < pair.ID2 && pair.ID2 < scene.images.size());
	ASSERT(warpSize > 0);
	const PairIdx::PairIndex key = PairIdx(pair.ID1, pair.ID2).idx;
	const auto it = pairIndexMap.find(key);
	IIndex idxPair;
	if (it != pairIndexMap.end()) {
		// a same-key pair a previous Match() left: this pass judged the pair again on its warp
		// alone, so its verdict replaces that pair whole rather than merging two sets of evidence.
		// Unreached today -- MatchPairsROMA2 skips every candidate already in pairIndexMap -- and
		// safe only as long as that holds: the incumbent must carry no dense segment of its own, or
		// this move would orphan the keypoints its dense fill already appended to both images.
		idxPair = it->second;
		scene.pairs[idxPair] = std::move(pair);
	} else {
		// overlapRatio/overlapArea stay at their reset value (0): a created pair is weighted exactly
		// like any other pair, ComputePairsWeights computing its own overlap proxy for it. Stamping
		// a full 1/1 overlap here (what the old NPZ import did) would survive PairsMatcher::Match --
		// nothing else writes overlapRatio -- and hand every dense pair a best-possible overlap
		// score it was never measured to have
		idxPair = (IIndex)scene.pairs.size();
		pairIndexMap.emplace(key, idxPair);
		scene.pairs.emplace_back(std::move(pair));
	}
	// only now, and serially: the keypoint indices the append hands out depend on what the two
	// images already carry, so a parallel or completion-ordered append would relabel them run to run
	AppendDenseMatches(scene, scene.pairs[idxPair], dense.pointsA, dense.pointsB, dense.confidences,
		cv::Size(warpSize, warpSize));
}
/*----------------------------------------------------------------*/


// T H E   O N E   P A S S ////////////////////////////////////////////

unsigned SFM::MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs, const ROMA2Config& config)
{
#ifdef _USE_ONNXRUNTIME
	ASSERT(roma2.IsLoaded());
	if (candidatePairs.empty())
		return 0;
	TD_TIMER_STARTD();
	Scene& scene = pairsMatcher.GetScene();
	// every candidate pair indexes scene.images directly (slot planning, loads, the pair keys), so
	// image IDs must be their own indices - the convention the whole matcher assumes
	ASSERT(std::all_of(scene.images.begin(), scene.images.end(),
		[&](const Image& img) { return img.ID == (IIndex)(&img - scene.images.begin()); }));

	// 1) the candidates to judge, in the one order everything downstream depends on: (ID1,ID2),
	// de-duplicated (the same unordered pair may reach here twice, and only the A < B direction is
	// ever warped), and without the pairs the scene already holds -- the feedback round proposes
	// only new pairs, and a re-run must not double-store
	std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
	pairIndexMap.reserve(scene.pairs.size() + candidatePairs.size());
	FOREACH(i, scene.pairs)
		pairIndexMap.emplace(PairIdx(scene.pairs[i].ID1, scene.pairs[i].ID2).idx, i);
	PairIdxArr sorted(candidatePairs);
	sorted.Sort();
	sorted.Resize((PairIdxArr::IDX)(std::unique(sorted.begin(), sorted.end()) - sorted.begin()));
	PairIdxArr pairs(0, sorted.size());
	unsigned numSkipped = 0;
	for (const PairIdx& p : sorted) {
		ASSERT(p.i < p.j); // MakePairIdx orders the two indices, so each unordered pair appears once
		const Image& imgA = scene.images[p.i];
		const Image& imgB = scene.images[p.j];
		// the verdict fits in the two cameras' bearings and the guided pass reads descriptors
		if (!imgA.HasCamera() || !imgB.HasCamera() || !imgA.HasDescriptors() || !imgB.HasDescriptors() ||
			pairIndexMap.find(p.idx) != pairIndexMap.end()) {
			++numSkipped;
			continue;
		}
		pairs.push_back(p);
	}
	if (pairs.empty()) {
		// same field list as the summary line below (the measurement tools parse both the same way):
		// nothing past this point ran, so every field the warp pass and the store would have filled is 0
		DEBUG("ROMA2 one pass: %u candidates, %u judged, %u admitted, %u stored, %u dense-only; "
			"%u slots, %u loads, %u reloads; %u skipped, %u failed loads, %u failed matches, %u dense matches (%s)",
			candidatePairs.size(), 0u, 0u, 0u, 0u,
			0u, 0u, 0u,
			numSkipped, 0u, 0u, 0u,
			TD_TIMER_GET_FMT().c_str());
		return 0;
	}

	// 2) the warp pass: this thread plans the device slots, pipelines the image loads and runs the
	// bidirectional coarse-match graph pair by pair, while the pool judges each pair, guides its
	// sparse matching and assembles it. One result slot per pair collects what the consumers produce.
	struct AssembledPair {
		ImagePair pair;
		DenseMatches dense;
		bool bAssembled = false;
	};
	std::vector<AssembledPair> results(pairs.size());
	std::atomic<unsigned> numJudged{0}, numAdmitted{0};
	const int warpSize = roma2.WarpSize();
	WarpPassStats stats;
	if (!ForEachWarpROMA2(pairsMatcher, roma2, pairs, config.slotBudget, _T("Dense match image pairs"),
		[&](size_t p, const PairIdx& pairIdx, PairWarps& warps, unsigned threadIdx) {
			// this pair's own clock, named apart from the pass's TD_TIMER_STARTD() one (a nested
			// TD_TIMER_START() would redeclare that timer's local) and wound up only when the record
			// below will actually print, so a run below verbosity 3 never pays for it
			const bool bTimePair = VERBOSITY_LEVEL > 2;
			const SEACAVE::Timer::SysType pairTimeStart = bTimePair ? SEACAVE::Timer::GetSysTime() : 0;
			const Image& imgA = scene.images[pairIdx.i];
			const Image& imgB = scene.images[pairIdx.j];
			ImagePair pair(pairIdx.i, pairIdx.j);
			PairVerdict verdict;
			JudgePairROMA2(pairsMatcher, imgA, imgB, warps, config, pair, verdict);
			++numJudged;
			if (!verdict.admitted) {
				DEBUG_ULTIMATE("ROMA2 pair %u-%u: conf %.4f %.4f inl %.4f %.4f REJECT",
					pairIdx.i, pairIdx.j, verdict.confidentAreaA, verdict.confidentAreaB,
					verdict.inlierAreaA, verdict.inlierAreaB);
				return;
			}
			++numAdmitted;
			// the guided sparse matching: the warp tracks A's described keypoints into B, and the
			// descriptor match is restricted to a disc of two warp cells around each prediction.
			// This is where appearance enters, and the only place it does.
			std::vector<Point2f> trackedA, trackedB;
			std::vector<uchar> trackStatus;
			TrackKeypointsByWarp(imgA, imgB, warps.ab.warp, warps.ab.confidence, config.minConfidence,
				trackedA, trackedB, trackStatus);
			const float discRadius = 2.f*(float)MAXF(imgB.GetWidth(), imgB.GetHeight())/(float)warpSize;
			std::vector<DMatch> guided;
			MatchFeaturesGuided(pairsMatcher, imgA, imgB, trackedB, trackStatus, discRadius, threadIdx, guided);
			AssembledPair& result = results[p];
			result.bAssembled = AssemblePairROMA2(pairsMatcher, imgA, imgB, verdict, guided, config,
				warpSize, pair, result.dense);
			const unsigned numSparse = result.bAssembled ? pair.GetNumFilteredInliers() : 0u;
			if (result.bAssembled)
				result.pair = std::move(pair);
			DEBUG_ULTIMATE("ROMA2 pair %u-%u: conf %.4f %.4f inl %.4f %.4f ADMIT guided %u sparse %u dense %u %ums",
				pairIdx.i, pairIdx.j, verdict.confidentAreaA, verdict.confidentAreaB,
				verdict.inlierAreaA, verdict.inlierAreaB, (unsigned)guided.size(), numSparse,
				(unsigned)result.dense.pointsA.size(), bTimePair ?
				(unsigned)SEACAVE::Timer::SysTime2TimeMs(SEACAVE::Timer::GetSysTime() - pairTimeStart) : 0u);
		}, stats))
		return 0;

	// 3) serial store, in (ID1,ID2) order: the keypoint indices the dense append hands out depend on
	// what the two images already carry, so the labelling must not depend on the pool's order
	unsigned numStored = 0, numDenseOnly = 0;
	size_t numDenseMatches = 0;
	FOREACH(p, results) {
		AssembledPair& result = results[p];
		if (!result.bAssembled)
			continue;
		if (result.pair.GetNumFilteredInliers() == 0)
			++numDenseOnly; // no sparse inlier at all: the dense segment is this pair's whole evidence
		numDenseMatches += result.dense.pointsA.size();
		StorePairROMA2(scene, pairIndexMap, std::move(result.pair), result.dense, warpSize);
		++numStored;
	}
	DEBUG("ROMA2 one pass: %u candidates, %u judged, %u admitted, %u stored, %u dense-only; "
		"%u slots, %u loads, %u reloads; %u skipped, %u failed loads, %u failed matches, %u dense matches (%s)",
		candidatePairs.size(), numJudged.load(), numAdmitted.load(), numStored, numDenseOnly,
		stats.numSlots, (unsigned)stats.numLoads, (unsigned)stats.numReloads,
		numSkipped, stats.numFailedLoads, stats.numFailedMatches, (unsigned)numDenseMatches,
		TD_TIMER_GET_FMT().c_str());
	return numStored;
#else // _USE_ONNXRUNTIME
	// unreachable: RoMa2Onnx::IsAvailable() is false in this build, so Scene::MatchPairs never
	// loads a model and PairsMatcher::Match never calls here
	ASSERT(false);
	return 0;
#endif // _USE_ONNXRUNTIME
}

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
