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
// (TrackKeypointsByWarp, MatchFeaturesGeometric and GeometricFilter can all throw) would starve
// the producer permanently; this keeps the count balanced on every path out of the task.
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

// One pair's warp, handed to a pool thread: the pair's index in the pass's list, its images, the
// warp itself (the task owns it) and the index of the private descriptor matcher it may use.
typedef std::function<void(size_t idxPair, const PairIdx& pair, WarpMaps& maps, unsigned threadIdx)> WarpConsumer;

// Drive the ROMAv2 coarse-match graph over the given pairs, handing each pair's warp to the thread
// pool: plan the device slots (Belady over the pairs in (ID1,ID2) order), pipeline the image
// load+preprocess on the pool while this thread runs Describe and MatchCoarse, and detach one
// `consume` task per warped pair. The pairs must already be filtered to the ones worth a warp and
// sorted in (ID1,ID2) order -- the slot plan's locality, and the order results are applied in.
// The semaphore bounds how many warps are alive at once (~300 KB each at base). Every pool task is
// drained, and the OpenCV thread count, log console and progress bar restored, on every exit path
// including an exceptional one (Describe, MatchCoarse and the consumers can all throw).
// Both passes share this because they differ only in what they do with a warp: the gate judges the
// pair, the dense matcher re-matches it.
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
	// Describe()'s retrieval readback is not optional, but neither warp pass has a use for a
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
			// value_facets stays on the device: neither warp pass reads it back (design decision 4)
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
		WarpMaps maps;
		if (!roma2.MatchCoarse(slotLayers[step.slotA], slotLayers[step.slotB], maps.warp, maps.overlap)) {
			++stats.numFailedMatches;
			DEBUG_EXTRA("error: could not dense match pair (% 4u, % 4u)", pair.i, pair.j);
			++state.progress;
			continue;
		}
		ASSERT(maps.IsValid() && maps.warp.rows == roma2.WarpSize());
		inFlight.Wait();
		scene.threadPool.detach_task([&, p, pair, maps = std::move(maps)]() mutable {
			const ScopedSemaphore released(inFlight); // returned however this task ends (see the struct)
			const std::optional<size_t> threadIdx = BS::this_thread::get_index();
			ASSERT(threadIdx && *threadIdx < pairsMatcher.GetNumMatchers());
			consume(p, pair, maps, (unsigned)*threadIdx);
			++state.progress;
		});
	}
	scene.threadPool.wait();
	state.Finish(); // unpause the log console before the caller's summary line (see the struct comment)
	return true;
}

#endif // _USE_ONNXRUNTIME

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


// D E N S E   M A T C H I N G   P A S S //////////////////////////////

unsigned SFM::MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs, const ROMA2Config& config, bool bFeedbackRound)
{
#ifdef _USE_ONNXRUNTIME
	ASSERT(roma2.IsLoaded());
	if (candidatePairs.empty())
		return 0;
	TD_TIMER_STARTD();
	Scene& scene = pairsMatcher.GetScene();
	// every candidate pair indexes scene.images directly (slot planning, loads, ApplyROMA2Pair's
	// keys), so image IDs must be their own indices - the convention the whole matcher assumes
	ASSERT(std::all_of(scene.images.begin(), scene.images.end(),
		[&](const Image& img) { return img.ID == (IIndex)(&img - scene.images.begin()); }));
	// per-round replace policy (design decision 6): the first round warps every candidate and
	// replaces whenever the guided set is larger, while the verification-feedback round spends
	// its warps only on the pairs that are still weak, and replaces only the weakest of them
	const unsigned skipHealthy = bFeedbackRound ? config.feedbackSkipHealthyInliers : config.skipHealthyInliers;
	const unsigned maxReplace = bFeedbackRound ? config.feedbackMaxReplaceInliers : config.maxReplaceInliers;

	// 1) the pairs worth a warp: both images usable, and no healthy descriptor-matched pair
	// already stored for them (compared by inlier count, not by composite weight: inside Match()
	// every weight is still zero, ComputePairsWeights only runs after both rounds)
	std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
	pairIndexMap.reserve(scene.pairs.size() + candidatePairs.size());
	FOREACH(i, scene.pairs)
		pairIndexMap.emplace(PairIdx(scene.pairs[i].ID1, scene.pairs[i].ID2).idx, i);
	PairIdxArr pairs(0, candidatePairs.size());
	unsigned numSkippedHealthy = 0;
	for (const PairIdx& p : candidatePairs) {
		const Image& imgA = scene.images[p.i];
		const Image& imgB = scene.images[p.j];
		if (!imgA.HasDescriptors() || !imgB.HasDescriptors() || !imgA.HasCamera() || !imgB.HasCamera())
			continue;
		if (skipHealthy > 0) {
			const auto it = pairIndexMap.find(p.idx);
			if (it != pairIndexMap.end() && scene.pairs[it->second].GetNumFilteredInliers() >= skipHealthy) {
				++numSkippedHealthy;
				continue;
			}
		}
		pairs.push_back(p);
	}
	if (pairs.empty())
		return 0;
	pairs.Sort(); // (ID1,ID2): the slot plan's locality, and the order the results are applied in

	// 2) the warp pass: the shared driver plans the device slots, pipelines the image loads and
	// coarse-matches the pairs on this thread, while the pool turns each warp into a guided sparse
	// re-match of its pair. One result slot per pair collects what the consumers produce.
	std::vector<ImagePair> results(pairs.size());
	std::atomic<unsigned> numGuided{0};
	std::atomic<unsigned> numGated{0};
	// whether the per-pair DEBUG_ULTIMATE line below will be emitted at all, and hence whether the
	// two quantities only it and the gate consume are worth computing (loop invariants, and the
	// verbosity does not change while a pass runs); declared here because the consumers read them
	#if TD_VERBOSE == TD_VERBOSE_OFF
	const bool bDiagnostic = false;
	#else
	const bool bDiagnostic = VERBOSITY_LEVEL > 2;
	#endif
	const bool bCountOverlap = bDiagnostic || config.minCreatedOverlap > 0;
	WarpPassStats stats;
	if (!ForEachWarpROMA2(pairsMatcher, roma2, pairs, config.slotBudget, _T("Dense match image pairs"),
		[&](size_t p, const PairIdx& pair, WarpMaps& maps, unsigned threadIdx) {
			const Image& imgA = scene.images[pair.i];
			const Image& imgB = scene.images[pair.j];
			if (config.erodeBorder > 0)
				ErodeConfidenceMap(maps.overlap, config.erodeBorder, config.minConfidence, config.minErodeConfidence);
			// confident-overlap gate: how much of the warp survived the erosion above, as a fraction
			// of the whole warp grid. A pair the descriptor matcher could not verify is created out
			// of nothing but this warp, so on a repetitive scene it is only as trustworthy as the
			// area the warp is actually confident about (design note, Limitations); a pair that
			// already exists is never gated, its replacement policy is unchanged. Existence is read
			// from pairIndexMap, captured by reference and read-only for the whole pass: it is built
			// in step 1 and only step 4 (after the pool is drained) inserts into it.
			// The gate and the per-pair diagnostic are the only readers of that fraction, so on the
			// default path (gate off, no DEBUG_ULTIMATE) it is not even counted -- which cannot
			// change any result, the condition below being false whenever minCreatedOverlap is 0.
			float overlapFraction = 0.f;
			if (bCountOverlap) {
				unsigned numConfident = 0;
				for (int y = 0; y < maps.overlap.rows; ++y)
					for (int x = 0; x < maps.overlap.cols; ++x)
						if (maps.overlap(y, x) >= config.minConfidence)
							++numConfident;
				overlapFraction = (float)numConfident / (float)(maps.overlap.rows*maps.overlap.cols);
			}
			const bool bExisting = pairIndexMap.find(pair.idx) != pairIndexMap.end();
			if (!bExisting && config.minCreatedOverlap > 0 && overlapFraction < config.minCreatedOverlap) {
				++numGated;
				DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u): new, overlap %.3f, 0 tracked, 0 guided, 0 shared-train, gated",
					pair.i, pair.j, overlapFraction);
				return;
			}
			std::vector<Point2f> trackedA, trackedB;
			std::vector<uchar> trackStatus;
			const unsigned numTracked = (unsigned)TrackKeypointsByWarp(imgA, imgB, maps.warp, maps.overlap, config.minConfidence, trackedA, trackedB, trackStatus);
			ImagePair guided(pair.i, pair.j);
			// only a fully guided result may be kept: when the warp yields no usable geometry
			// MatchFeaturesGeometric falls back to plain descriptor matching, which is exactly
			// what MatchPairsBatch already stored for this pair (and geometrically verified),
			// so such a fallback must never be offered as a replacement for it
			unsigned numSharedTrain = 0;
			const bool bGuided = MatchFeaturesGeometric(pairsMatcher, imgA, imgB, trackedA, trackedB, trackStatus,
				guided, config.epipolarThreshold, threadIdx, config.guidedCrossCheck,
				// only the diagnostic reads the count, so asking for it off the diagnostic path
				// would build the collision map for nothing when the cross-check is off too
				bDiagnostic ? &numSharedTrain : NULL);
			const unsigned numMatches = guided.GetNumMatches();
			if (bGuided && !guided.matches.empty() &&
				// and it has to be verified the way MatchPairsBatch verifies the pair it may
				// replace: MatchFeaturesGeometric leaves behind the geometry it estimated from the
				// coarse tracked points together with every match it then selected along those
				// epipolar lines, so without this its "inliers" are an unverified count that always
				// beats a RANSAC-verified one -- neither a fair replace decision (design decision 6
				// replaces only a strictly weaker pair) nor a pair the rest of the pipeline can
				// consume. This refits the geometry to the final match set and splits its outliers
				// off, exactly as MatchPair does, with the same fixed RANSAC seed. With the
				// verification switched off the guided pair is held to the same size bar the
				// descriptor path applies in that configuration (MatchPair, PairsMatcher.cpp:626).
				(pairsMatcher.GetConfig().maxEpipolarError > 0 ?
					pairsMatcher.GeometricFilter(imgA, imgB, guided) :
					guided.GetNumMatches() >= pairsMatcher.GetConfig().minMatches)) {
				ASSERT(!guided.matches.empty());
				results[p] = std::move(guided);
				++numGuided;
				DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u): %s, overlap %.3f, %u tracked, %u guided, %u shared-train, kept",
					pair.i, pair.j, bExisting ? "existing" : "new", overlapFraction, numTracked, numMatches, numSharedTrain);
			} else {
				DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u): %s, overlap %.3f, %u tracked, %u guided, %u shared-train, rejected",
					pair.i, pair.j, bExisting ? "existing" : "new", overlapFraction, numTracked, numMatches, numSharedTrain);
			}
		}, stats))
		return 0;

	// 3) serial, in-order apply: which of two near-tied match sets wins depends on what the
	// scene already holds, so the outcome must not depend on the order the pool finished in
	unsigned numCreated = 0, numReplaced = 0;
	FOREACH(p, results) {
		ImagePair& guided = results[p];
		if (guided.matches.empty())
			continue;
		bool bCreated;
		if (ApplyROMA2Pair(scene, pairIndexMap, std::move(guided), maxReplace, bCreated))
			++(bCreated ? numCreated : numReplaced);
	}
	DEBUG("ROMA2 dense matching (%s round): %u/%u pairs guided, %u created, %u replaced, %u gated, %u skipped healthy, %u failed loads, %u failed matches; %u slots, %u loads, %u reloads (%s)",
		bFeedbackRound ? "feedback" : "first", numGuided.load(), pairs.size(), numCreated, numReplaced, numGated.load(), numSkippedHealthy,
		stats.numFailedLoads, stats.numFailedMatches, stats.numSlots, (unsigned)stats.numLoads, (unsigned)stats.numReloads, TD_TIMER_GET_FMT().c_str());
	return numCreated + numReplaced;
#else // _USE_ONNXRUNTIME
	// unreachable: RoMa2Onnx::IsAvailable() is false in this build, so Scene::MatchPairs never
	// loads a model and PairsMatcher::Match never calls here
	ASSERT(false);
	return 0;
#endif // _USE_ONNXRUNTIME
}
/*----------------------------------------------------------------*/


// D E N S E   V A L I D A T I O N   P A S S //////////////////////////

#ifdef _USE_ONNXRUNTIME

namespace {

// One candidate image pair as the dense two-view gate judged it: the sample drawn from its warp,
// the subset of that sample one fitted geometry explains, and the spread the verdict rests on.
// Internal to the gate -- nothing downstream reads a judged pair, only the surviving pair list --
// so the arrays are released as soon as the verdict is in, on an accepted pair as much as a
// rejected one.
// A default-constructed record (ID1 == ID2 == NO_ID) means the pair was never warped at all, which
// is a different fact from being rejected.
struct DensePairValidation {
	IIndex ID1 = NO_ID, ID2 = NO_ID;         // the pair, ID1 < ID2 (indices into Scene::images)
	std::vector<Point2f> pointsA, pointsB;   // the dense sample: pixels of the working orientation of each image, index-parallel
	std::vector<uint32_t> inliers;           // ascending indices into pointsA/pointsB the fitted geometry explains
	// Spread, on the same DENSE_COVERAGE_GRID^2 grid over each image: first of the whole drawn
	// sample, then of its inlier subset alone. The pair is the diagnostic -- a sample spread over
	// the overlap whose *inliers* huddle in one corner is a wrong pair that an inlier count cannot see.
	float coverageA = 0.f, coverageB = 0.f;
	float coverageInlierA = 0.f, coverageInlierB = 0.f;
	unsigned numSampled = 0;                 // size of the drawn sample
	unsigned numInliers = 0;                 // RANSAC inlier set of the fitted geometry (ImagePair::GetNumInliers)
	// PairsMatcher::GeometryBranch the estimator actually took; unset (not some branch's own value)
	// when the sample was too small for GeometricFilter to even run, so an un-fitted pair cannot be
	// misread as having taken SHARED_FOCAL, which is also value 0
	std::optional<PairsMatcher::GeometryBranch> geometryBranch;
	bool bValidated = false;                 // min(coverageInlierA, coverageInlierB) >= ROMA2Config::minInlierCoverage
};

// Judge one candidate pair on its warp alone: draw the coverage-maximising sample, fit one geometry
// to the whole of it and record which of its correspondences that geometry explains. Fills
// everything of `val` but the verdict, which its caller derives.
// The sample is fitted through the same estimator the descriptor path uses, on temporary Image
// copies whose keypoints are the dense points -- the MatchFeaturesGeometric precedent, so no second
// estimator has to exist. What is deliberately *not* replayed from there is its epipolar
// pre-selection: MatchFeaturesGeometric estimates a geometry from the warp's own tracked points and
// then keeps the descriptor matches lying on those epipolar lines, which largely confirms whatever
// the warp asserted. Here the sample is exactly what the warp claims, chosen for spread and
// confidence only, and one geometry either explains it or does not.
// A sample too small for the estimator leaves the pair with no inliers at all (rejected): a warp
// that cannot even offer minMatches confident, spread-out correspondences is no evidence.
//
// What this reads of the two images: `pCamera` (the intrinsics, and TrustIntrinsics() through them),
// the image size, and the warp. It does NOT read their poses -- the temporary copies are built with
// an explicitly invalidated pose, so a scene that happens to carry a ground-truth solution cannot
// leak it into the gate's geometry however the estimator later changes. That is the whole point of
// the gate, so it is enforced structurally rather than trusted.
void ValidateOnePairROMA2(PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const WarpMaps& maps, const ROMA2Config& config, DensePairValidation& val)
{
	SampleWarpByCoverage(imgA, imgB, maps.warp, maps.overlap, config.minConfidence,
		config.denseSampleSize, val.pointsA, val.pointsB, val.coverageA, val.coverageB);
	val.numSampled = (unsigned)val.pointsA.size();
	// GeometricFilter needs 8 correspondences of its own, and a pair is not worth keeping below the
	// same match bar every descriptor pair clears
	const MatchConfig& cfg = pairsMatcher.GetConfig();
	if (val.numSampled < MAXF(cfg.minMatches, 8u))
		return;
	Image imgACopy(imgA.ID, imgA.fileName, Pose3D(), imgA.cameraID, imgA.pCamera);
	Image imgBCopy(imgB.ID, imgB.fileName, Pose3D(), imgB.cameraID, imgB.pCamera);
	imgACopy.InvalidatePose(); // see the note above: the gate never sees a pose, only intrinsics
	imgBCopy.InvalidatePose();
	ASSERT(!imgACopy.HasPose() && !imgBCopy.HasPose());
	imgACopy.keypoints = ConvertToKeypoints(val.pointsA);
	imgBCopy.keypoints = ConvertToKeypoints(val.pointsB);
	// recorded for the per-pair log only: the estimator makes the same call itself, from the same
	// one named decision, so the two can never disagree
	val.geometryBranch = PairsMatcher::SelectGeometryBranch(cfg, imgACopy, imgBCopy);
	ImagePair fit(val.ID1, val.ID2);
	fit.matches.reserve(val.numSampled);
	for (uint32_t i = 0; i < val.numSampled; ++i)
		fit.matches.emplace_back(i, i);
	if (!pairsMatcher.GeometricFilter(imgACopy, imgBCopy, fit))
		return; // no single geometry explained enough of the sample to survive the estimator
	// `fit.matches` is the RANSAC inlier set on every branch: PartitionMatchesByMask splits the
	// outliers off, and the strict cheirality/angle/reprojection filter that follows on a branch
	// with a relative pose only *reorders* matches (FilterMatches -> PartitionMatchesByMask with
	// reorderOnly=true). So GetNumInliers() means the same thing on the calibrated and the
	// uncalibrated branch, which is why the inlier subset is read from it.
	val.inliers.reserve(fit.GetNumInliers());
	for (const DMatch& match : fit.matches)
		val.inliers.push_back(match.queryIdx);
	std::sort(val.inliers.begin(), val.inliers.end()); // FilterMatches may have reordered them
	val.numInliers = fit.GetNumInliers();
	ASSERT(val.numInliers == (unsigned)val.inliers.size());
	// spread of the inlier subset alone, on the same grid as the sample's own coverage: a sample
	// spread over the overlap whose inliers huddle in one corner is what an inlier count cannot see
	ComputeSampleCoverage(val.pointsA, val.pointsB, imgA.GetSize(), imgB.GetSize(), val.inliers,
		val.coverageInlierA, val.coverageInlierB);
}

} // namespace

#endif // _USE_ONNXRUNTIME

unsigned SFM::ValidatePairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, PairIdxArr& pairs, const ROMA2Config& config)
{
#ifdef _USE_ONNXRUNTIME
	ASSERT(roma2.IsLoaded());
	if (pairs.empty())
		return 0;
	TD_TIMER_STARTD();
	Scene& scene = pairsMatcher.GetScene();
	// every candidate pair indexes scene.images directly (slot planning, loads), so image IDs must
	// be their own indices - the convention the whole matcher assumes
	ASSERT(std::all_of(scene.images.begin(), scene.images.end(),
		[&](const Image& img) { return img.ID == (IIndex)(&img - scene.images.begin()); }));

	// 1) the candidates the gate can judge. Unlike the dense matching pass this needs no
	// descriptors -- it runs before any descriptor matching and reads nothing but the warp -- but
	// it does need both cameras, since the geometry is fitted in their bearings
	PairIdxArr candidates(0, pairs.size());
	for (const PairIdx& p : pairs)
		if (scene.images[p.i].HasCamera() && scene.images[p.j].HasCamera())
			candidates.push_back(p);
	const unsigned numUnusable = pairs.size() - candidates.size();
	if (candidates.empty()) {
		pairs.Empty();
		VERBOSE("warning: ROMA2 dense pair validation: none of the %u candidate pairs has cameras on both images", numUnusable);
		return 0;
	}
	candidates.Sort(); // (ID1,ID2): the slot plan's locality

	// 2) the gate fits with the matcher's own configuration -- no threshold and no branch choice of
	// its own. Which geometry runs is SelectGeometryBranch's decision from what the images carry
	// (calibrated bearings where both trust their intrinsics, F otherwise), and the precision
	// demanded of the warp is MatchConfig::maxEpipolarError, the descriptor path's own. Measured on
	// Truck at 1.0 and 2.89 warp-native px -- the default 4 target px is between them there -- the
	// gate's recall moved 98.99% -> 99.31% (F) and 99.23% -> 99.48% (E) and its normal-false leak
	// 0.00% -> 0.04% and 0.00% -> 0.20%: the verdict is not sensitive to the threshold anywhere in
	// that band, on either branch, so there is nothing for a separate setting to buy
	DEBUG_EXTRA("ROMA2 gate: %g px epipolar error, %u-px warp frame, sample %u, inlier coverage >= %g",
		pairsMatcher.GetConfig().maxEpipolarError, roma2.ImageSize(), config.denseSampleSize,
		config.minInlierCoverage);

	// 3) the warp pass: one verdict per candidate, on the pool, over the same slot plan and
	// prefetch pipeline the dense matching pass uses
	std::vector<DensePairValidation> results(candidates.size());
	std::atomic<unsigned> numValidated{0};
	WarpPassStats stats;
	if (!ForEachWarpROMA2(pairsMatcher, roma2, candidates, config.slotBudget, _T("Validate image pairs"),
		[&](size_t p, const PairIdx& pair, WarpMaps& maps, unsigned /*threadIdx*/) {
			const Image& imgA = scene.images[pair.i];
			const Image& imgB = scene.images[pair.j];
			if (config.erodeBorder > 0)
				ErodeConfidenceMap(maps.overlap, config.erodeBorder, config.minConfidence, config.minErodeConfidence);
			DensePairValidation& val = results[p];
			val.ID1 = pair.i;
			val.ID2 = pair.j;
			ValidateOnePairROMA2(pairsMatcher, imgA, imgB, maps, config, val);
			// the whole rule: the inlier subset's coverage of the two images
			val.bValidated = MINF(val.coverageInlierA, val.coverageInlierB) >= config.minInlierCoverage;
			if (val.bValidated)
				++numValidated;
			// nothing downstream reads a judged pair's sample -- the gate hands over a pair list --
			// so the arrays go back as soon as the verdict is in
			val.pointsA = std::vector<Point2f>();
			val.pointsB = std::vector<Point2f>();
			val.inliers = std::vector<uint32_t>();
			DEBUG_ULTIMATE("ROMA2 gate (% 4u, % 4u): %s, %u sampled, %u inliers, "
				"coverage %.3f/%.3f, inlier coverage %.3f/%.3f, %s",
				pair.i, pair.j,
				val.geometryBranch ? PairsMatcher::GeometryBranchName(*val.geometryBranch) : _T("no branch (sample too small)"),
				val.numSampled, val.numInliers,
				val.coverageA, val.coverageB, val.coverageInlierA, val.coverageInlierB,
				val.bValidated ? "validated" : "rejected");
		}, stats))
		return 0;

	// 4) keep the pairs a single geometry explained; a rejected pair is dropped, never demoted to
	// ordinary descriptor matching -- a SIFT fallback would hide the gate's mistakes instead of
	// leaving them measurable
	PairIdxArr kept(0, candidates.size());
	unsigned numRejected = 0, numUnwarped = 0;
	FOREACH(p, results) {
		const DensePairValidation& val = results[p];
		// a candidate whose image could not be described, or whose warp the coarse-match graph could
		// not produce, was never judged at all: its record is still default constructed,
		// ID1 == ID2 == NO_ID. MatchPairsROMA2 keeps the same distinction by skipping an empty match
		// set; the gate has to state it, because a rejected verdict and an absent verdict are
		// different facts and the summary below reports them separately.
		if (val.ID1 == NO_ID) {
			ASSERT(val.ID2 == NO_ID && val.numSampled == 0 && !val.bValidated);
			++numUnwarped;
			continue;
		}
		if (val.bValidated)
			kept.push_back(candidates[p]);
		else
			++numRejected;
	}
	ASSERT(kept.size() == numValidated.load());
	ASSERT(numUnwarped == 0 || stats.numFailedLoads > 0 || stats.numFailedMatches > 0);
	pairs = std::move(kept);
	DEBUG("ROMA2 dense pair validation: %u/%u pairs validated, %u rejected, %u never warped, %u without cameras, %u failed loads, %u failed matches; %u slots, %u loads, %u reloads (%s)",
		pairs.size(), candidates.size(), numRejected, numUnwarped, numUnusable,
		stats.numFailedLoads, stats.numFailedMatches, stats.numSlots, (unsigned)stats.numLoads, (unsigned)stats.numReloads, TD_TIMER_GET_FMT().c_str());
	return pairs.size();
#else // _USE_ONNXRUNTIME
	// unreachable: RoMa2Onnx::IsAvailable() is false in this build, so Scene::MatchPairs never
	// loads a model and PairsMatcher::Match never calls here
	ASSERT(false);
	return 0;
#endif // _USE_ONNXRUNTIME
}

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
