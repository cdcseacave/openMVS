/*
 * MatchDense.cpp
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
#include "MatchDense.h"
#include "Scene.h"
#include "DenseMatcher.h"
#include "PairsMatcher.h"
#include "MatchGeometric.h"
#include "DenseWarp.h"

#include <deque>
#include <future>
#include <limits>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("DENSE   "));

namespace {

typedef std::vector<float> PlanarImage;

// Temporarily switches OpenCV to single-threaded execution (for pool-parallel passes) and restores on exit.
struct ScopedCvThreads
{
	int restoreThreads;
	explicit ScopedCvThreads(int nMaxThreads) : restoreThreads(nMaxThreads) { cv::setNumThreads(1); }
	ScopedCvThreads(const ScopedCvThreads&) = delete;
	ScopedCvThreads& operator=(const ScopedCvThreads&) = delete;
	~ScopedCvThreads() { cv::setNumThreads(restoreThreads); }
};

// Waits for every detached task in the scene thread pool on scope exit (including exceptional exits).
struct ScopedPoolDrain
{
	Scene& scene;
	explicit ScopedPoolDrain(Scene& _scene) : scene(_scene) {}
	ScopedPoolDrain(const ScopedPoolDrain&) = delete;
	ScopedPoolDrain& operator=(const ScopedPoolDrain&) = delete;
	~ScopedPoolDrain() { scene.threadPool.wait(); }
};

#ifdef _USE_ONNXRUNTIME

PairInput MakePairInput(const Image& imgA, const Image& imgB)
{
	return PairInput{ imgA.GetSize(), imgB.GetSize() };
}

bool PrepareImageDense(Image& img, const DenseMatcher& matcher, PlanarImage& planar)
{
	ASSERT(matcher.IsLoaded());
	const bool hadPixels = img.HasPixels();
	if (!hadPixels && !img.LoadPixels())
		return false;
	matcher.Preprocess(img.GetImage8U3(), planar);
	if (!hadPixels)
		img.ReleasePixels();
	return true;
}

// Pipelines image load+preprocess on the thread pool while the calling thread drives ONNX Runtime.
class PrefetchRing
{
public:
	PrefetchRing(Scene& _scene, unsigned capacity, const DenseMatcher& _matcher)
		: scene(_scene), matcher(_matcher), planars(capacity) { ASSERT(capacity > 0); }
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
			return PrepareImageDense(scene.images[image], matcher, planars[idxPlanar]);
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
	const DenseMatcher& matcher;
	std::vector<PlanarImage> planars;
	std::deque<Submission> pending;
	size_t numSubmitted = 0, numTaken = 0; // both count buffers of the ring, modulo its capacity
};

// Restores a pass's OpenCV thread count, log-console pause, and progress bar on scope exit,
// including through an exception. Finish() runs the restoration immediately and is what the normal path
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

// Returns one permit of the in-flight semaphore on scope exit.
struct ScopedSemaphore
{
	Semaphore& semaphore;

	explicit ScopedSemaphore(Semaphore& _semaphore) : semaphore(_semaphore) {}
	ScopedSemaphore(const ScopedSemaphore&) = delete;
	ScopedSemaphore& operator=(const ScopedSemaphore&) = delete;
	~ScopedSemaphore() { semaphore.Signal(); }
};

// Device slot plan for a sequence of pairs (Belady optimal replacement).
struct SlotPlan {
	struct Load {
		IIndex image;  // image to describe
		unsigned slot; // slot its description overwrites
	};
	struct Step {
		unsigned slotA = NO_ID, slotB = NO_ID;
		CLISTDEF0(Load) loads;
	};
	std::vector<Step> steps;
	unsigned numSlots = 0;
	size_t numLoads = 0, numReloads = 0;
};

SlotPlan MakeSlotPlan(const PairIdxArr& pairs, unsigned slotBudget, IIndex nImages)
{
	ASSERT(slotBudget >= 2u);
	SlotPlan plan;
	const unsigned budget = slotBudget;
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
typedef std::function<void(size_t idxPair, const PairIdx& pair, PairOutput& output, unsigned threadIdx)> WarpConsumer;

// Drive the dense matcher's coarse-match graph over the given pairs, handing each pair's bidirectional warp
// to the thread pool: plan the device slots (Belady over the pairs in (ID1,ID2) order), pipeline the
// image load+preprocess on the pool while this thread runs Describe and MatchCoarse, and detach one
// `consume` task per warped pair. Both directions come out of the SAME MatchCoarse call, so only the
// (A,B) pair with A < B is ever warped. The pairs must already be filtered to the ones worth a warp
// and sorted in (ID1,ID2) order -- the slot plan's locality, and the order results are stored in.
// The semaphore bounds how many warps are alive at once (~600 KB per pair at base). Every pool task
// is drained, and the OpenCV thread count, log console and progress bar restored, on every exit path
// including an exceptional one (Describe, MatchCoarse and the consumers can all throw).
// Returns false only when the device slot pool could not be allocated (nothing was warped).
bool ForEachDenseWarp(PairsMatcher& pairsMatcher, DenseMatcher& matcher, const PairIdxArr& pairs,
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
		layers = matcher.MakeCode();
		if (!layers.IsValid()) {
			VERBOSE("error: dense matcher slot pool: could not allocate the %u descriptor slots the plan needs (lower --dense-slots)", plan.numSlots);
			return false;
		}
	}
	IIndexArr slotImage(plan.numSlots);
	slotImage.Memset(0xFF); // NO_ID: slot empty or its load failed; a pair reading it is dropped, never matched against a stale image
	stats.numSlots = plan.numSlots;
	stats.numLoads = plan.numLoads;
	stats.numReloads = plan.numReloads;
	DEBUG_EXTRA("Dense slot plan: %u pairs, %u slots, %u loads (%u reloads)",
		pairs.size(), plan.numSlots, (unsigned)plan.numLoads, (unsigned)plan.numReloads);

	Semaphore inFlight(2*nThreads);
	// the plan's load sequence, flattened into the order the ring prefetches images in
	std::vector<SlotPlan::Load> loads;
	loads.reserve(plan.numLoads);
	for (const SlotPlan::Step& step : plan.steps)
		for (const SlotPlan::Load& load : step.loads)
			loads.push_back(load);
	ScopedPassState state(scene, progressCaption, pairs.size());
	PrefetchRing ring(scene, MINF(2u*nThreads, 8u), matcher);
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
			// value_facets stays on the device: the warp pass never reads it back
			if (planar && matcher.DescribeImage(planar->data(), slotLayers[load.slot], discardedRetrieval)) {
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
		PairOutput warps;
		if (!matcher.MatchPair(slotLayers[step.slotA], slotLayers[step.slotB], MakePairInput(scene.images[pair.i], scene.images[pair.j]), warps)) {
			++stats.numFailedMatches;
			DEBUG_EXTRA("error: could not dense match pair (% 4u, % 4u)", pair.i, pair.j);
			++state.progress;
			continue;
		}
		ASSERT(warps.IsValid() && warps.ab.warp.rows == matcher.GridSize());
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

constexpr unsigned VERDICT_SAMPLE = 4000;

// First-order (Sampson) distance test against pair geometry: uses F in pixels unless a calibrated
// pair has lens distortion (or non-pinhole camera), in which case it uses E on unit bearings.
class SampsonTest
{
public:
	SampsonTest(const Image& imgA, const Image& imgB, const ImagePair& pair, float tolerance)
		: camA(imgA.pCamera), camB(imgB.pCamera),
		  bFundamental(pair.F.has_value() &&
			!(pair.E.has_value() && (imgA.pCamera->HasDistortion() || imgB.pCamera->HasDistortion()) &&
			  imgA.TrustIntrinsics() && imgB.TrustIntrinsics()))
	{
		if (bFundamental) {
			M = pair.F.value();
			Mt = M.t();
			toleranceSq = (REAL)tolerance * (REAL)tolerance;
		} else if (pair.E.has_value()) {
			M = pair.E.value();
			Mt = M.t();
			const REAL angle = PairPixelErrorToAngular(*camA, *camB, (REAL)tolerance);
			toleranceSq = angle * angle;
		} else {
			toleranceSq = REAL(-1);
		}
	}

	inline bool IsValid() const { return toleranceSq >= REAL(0); }

	inline bool operator()(const Point2f& ptA, const Point2f& ptB) const {
		ASSERT(IsValid());
		if (bFundamental) {
			const Point3 a(ptA.x, ptA.y, REAL(1));
			const Point3 b(ptB.x, ptB.y, REAL(1));
			const Point3 Ma(M * a), Mtb(Mt * b);
			const REAL den = Ma.x * Ma.x + Ma.y * Ma.y + Mtb.x * Mtb.x + Mtb.y * Mtb.y;
			if (den < REAL(1e-14))
				return false;
			const REAL num = b.dot(Ma);
			return num * num <= toleranceSq * den;
		}
		const Point3 a(camA->UnprojectNormalized(Cast<REAL>(ptA)));
		const Point3 b(camB->UnprojectNormalized(Cast<REAL>(ptB)));
		const Point3 Ma(M * a), Mtb(Mt * b);
		// Sphere-tangent gradient norm squared: valid for both pinhole and spherical unit bearings
		const REAL den = normSq(b.cross(Ma)) + normSq(a.cross(Mtb));
		if (den < REAL(1e-14))
			return false;
		const REAL num = b.dot(Ma);
		return num * num <= toleranceSq * den;
	}

private:
	CameraPtr camA, camB;
	Matrix3x3 M, Mt;
	REAL toleranceSq;
	bool bFundamental;
};

// Build pose-free temporary Image copies holding pointsA/pointsB as keypoints and 1:1 matches in fit.
void MakeFitImages(const Image& imgA, const Image& imgB,
	const std::vector<Point2f>& pointsA, const std::vector<Point2f>& pointsB,
	Image& imgACopy, Image& imgBCopy, ImagePair& fit)
{
	ASSERT(pointsA.size() == pointsB.size());
	ASSERT(fit.matches.empty());
	imgACopy = Image(imgA.ID, imgA.fileName, Pose3D(), imgA.cameraID, imgA.pCamera);
	imgBCopy = Image(imgB.ID, imgB.fileName, Pose3D(), imgB.cameraID, imgB.pCamera);
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

unsigned SFM::ComputeGlobalDescriptorsDense(Scene& scene, DenseMatcher& matcher)
{
#ifdef _USE_ONNXRUNTIME
	ASSERT(matcher.IsLoaded());
	TD_TIMER_STARTD();
	const IIndex nImages = (IIndex)scene.images.size();
	const unsigned nPrefetch = MINF(2u*(unsigned)scene.threadPool.get_thread_count(), 8u);

	// destruction order matters on every exit path (normal or exceptional): ring must destruct
	// before state, so it is declared after state (reverse declaration order == destruction
	// order) -- every pool task is drained before cv::setNumThreads/log console are restored
	ScopedPassState state(scene, _T("Describe images"), nImages);
	PrefetchRing ring(scene, nPrefetch, matcher);
	for (IIndex i = 0; i < MINF(nPrefetch, nImages); ++i)
		ring.Submit(i);

	unsigned numDescribed = 0;
	OrtTensor layers(matcher.MakeCode()); // reused across every image: the describe pass keeps only the pooled vector, not the layers
	if (!layers.IsValid()) {
		VERBOSE("error: could not allocate the dense matcher's descriptor tensor");
		return 0;
	}
	std::vector<float> descriptor;
	for (IIndex i = 0; i < nImages; ++i, ++state.progress) {
		// consume the buffer this image was prefetched into before ever reusing it below
		const PlanarImage* const planar = ring.Take();
		Image& img = scene.images[i];
		// the graph's own on-device retrieval pooling (RoMa2Onnx::Describe), read straight into
		// descriptor
		if (planar && matcher.DescribeImage(planar->data(), layers, descriptor)) {
			ASSERT(descriptor.size() == matcher.RetrievalSize());
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

	DEBUG("Global descriptors computed for %u/%u images (%u-D, %s provider, %s)",
		numDescribed, (unsigned)nImages, matcher.RetrievalSize(), matcher.ProviderName().c_str(), TD_TIMER_GET_FMT().c_str());
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

void SFM::JudgePairDense(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairWarps& warps, const DenseMatchConfig& config, ImagePair& pair, PairVerdict& verdict)
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

	std::vector<Point2f> cellsAinA, cellsAinB, cellsBinA, cellsBinB;
	std::vector<float> confidencesA, confidencesB;
	std::vector<int> cellsA, cellsB;
	CollectEligibleWarpCells(warps.ab, imgA, imgB, config.minConfidence, config.sphericalBottomBand, cellsAinA, cellsAinB, confidencesA, cellsA);
	CollectEligibleWarpCells(warps.ba, imgB, imgA, config.minConfidence, config.sphericalBottomBand, cellsBinB, cellsBinA, confidencesB, cellsB);
	verdict.confidentAreaA = (float)cellsAinA.size() / numCells;
	verdict.confidentAreaB = (float)cellsBinB.size() / numCells;

	std::vector<Point2f> sampledA, sampledB;
	SampleWarpByCoverage(cellsAinA, cellsAinB, cellsA, warps.ab.confidence.size(), VERDICT_SAMPLE, sampledA, sampledB);
	if (sampledA.size() < 8)
		return;
	const float tolerance = WarpTolerance(sizeA, sizeB, warpSize);
	Image imgACopy, imgBCopy;
	MakeFitImages(imgA, imgB, sampledA, sampledB, imgACopy, imgBCopy, pair);
	if (!pairsMatcher.GeometricFilter(imgACopy, imgBCopy, pair, tolerance)) {
		pair.Reset();
		return;
	}
	pair.ResetMatches();
	const SampsonTest inlier(imgA, imgB, pair, tolerance);
	if (!inlier.IsValid()) {
		pair.Reset();
		return;
	}

	verdict.inliersA.reserve(cellsAinA.size());
	verdict.inliersB.reserve(cellsAinA.size());
	verdict.confidences.reserve(cellsAinA.size());
	verdict.inlierCells.reserve(cellsAinA.size());
	FOREACH(k, cellsAinA) {
		if (!inlier(cellsAinA[k], cellsAinB[k]))
			continue;
		verdict.inliersA.push_back(cellsAinA[k]);
		verdict.inliersB.push_back(cellsAinB[k]);
		verdict.confidences.push_back(confidencesA[k]);
		verdict.inlierCells.push_back(cellsA[k]);
	}
	verdict.inlierAreaA = (float)verdict.inliersA.size() / numCells;
	size_t numInliersB = 0;
	FOREACH(k, cellsBinB)
		if (inlier(cellsBinA[k], cellsBinB[k]))
			++numInliersB;
	verdict.inlierAreaB = (float)numInliersB / numCells;

	verdict.admitted = MINF(verdict.inlierAreaA, verdict.inlierAreaB) >= config.minOverlap;
	if (!verdict.admitted) {
		pair.Reset();
		verdict.inliersA = std::vector<Point2f>();
		verdict.inliersB = std::vector<Point2f>();
		verdict.confidences = std::vector<float>();
		verdict.inlierCells = std::vector<int>();
	}
}
/*----------------------------------------------------------------*/


// P A I R   A S S E M B L Y   A N D   S T O R A G E //////////////////

bool SFM::AssemblePairDense(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairVerdict& verdict, const std::vector<DMatch>& guided, const DenseMatchConfig& config, int warpSize,
	ImagePair& pair, DenseMatches& dense)
{
	ASSERT(verdict.admitted);
	ASSERT(pair.matches.empty() && pair.GetNumDenseInliers() == 0);
	ASSERT(pair.F.has_value() || pair.E.has_value());
	ASSERT(warpSize > 0);
	dense = DenseMatches();
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());

	std::vector<Point2f> occupiedA;
	occupiedA.reserve(guided.size());
	for (const DMatch& match : guided) {
		ASSERT((size_t)match.queryIdx < imgA.NumDescribedKeypoints());
		occupiedA.push_back(imgA.keypoints[match.queryIdx].pt);
	}
	const cv::Size gridSize(warpSize, warpSize);
	std::vector<int> derivedCells;
	const std::vector<int>* pCells = &verdict.inlierCells;
	if (verdict.inlierCells.size() != verdict.inliersA.size()) {
		// Support synthetic PairVerdict fixtures constructed by callers without populating inlierCells
		derivedCells.reserve(verdict.inliersA.size());
		for (const Point2f& pt : verdict.inliersA) {
			const Point2f c(PixelToCell(pt, sizeA, gridSize));
			const int x = CLAMP(ROUND2INT(c.x), 0, warpSize - 1);
			const int y = CLAMP(ROUND2INT(c.y), 0, warpSize - 1);
			derivedCells.push_back(y * warpSize + x);
		}
		pCells = &derivedCells;
	}
	SampleWarpComplementary(verdict.inliersA, verdict.inliersB, verdict.confidences, *pCells,
		sizeA, gridSize, DenseFillGridSide(config.denseMatchesPerFrame, warpSize), DenseFillCeiling(config, verdict),
		occupiedA, dense.pointsA, dense.pointsB, dense.confidences);

	// 2) the one geometry and the classification of both kinds of correspondence
	// (FitAndClassifyPairDense); the warp's half cell is the dense bar when the verification is off,
	// and when the union fit fails the classification ran against the verdict's geometry, which stands
	std::vector<uint8_t> denseKept;
	FitAndClassifyPairDense(pairsMatcher, imgA, imgB, guided, dense.pointsA, dense.pointsB, config,
		WarpTolerance(sizeA, sizeB, warpSize), pair, denseKept);
	size_t numDenseKept = 0;
	FOREACH(k, dense.pointsA) {
		if (!denseKept[k])
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

bool SFM::FitAndClassifyPairDense(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const std::vector<DMatch>& sparse, const std::vector<Point2f>& densePointsA, const std::vector<Point2f>& densePointsB,
	const DenseMatchConfig& config, float denseTolerance, ImagePair& pair, std::vector<uint8_t>& denseKept, bool bFit)
{
	ASSERT(pair.matches.empty() && pair.outlierMatches.empty());
	ASSERT(bFit || pair.relativePose.has_value());
	ASSERT(densePointsA.size() == densePointsB.size());
	// ONE geometry for the pair, fitted on sparse u dense through the matcher's own estimator at its
	// own threshold: the sparse correspondences are sub-pixel where texture exists and the dense ones
	// carry the parts of the overlap they left empty, so the union is the best-conditioned evidence
	// the pair has. On failure the geometry the pair arrived with stays, unchanged, and the caller
	// decides whether it may stand: at matching it is the verdict's and stands, an admitted pair is
	// always stored; at re-verification it does not, a pair no fit stands behind is invalidated. A
	// pair holding one fit's pose next to another fit's matrices would describe two geometries as
	// one, which is why all three members move together or none do. Without the fit (bFit false)
	// the geometry the pair arrives with is the one kept.
	bool bFitted = !bFit;
	const size_t numTotal = sparse.size() + densePointsA.size();
	if (bFit && numTotal >= 8) {
		std::vector<Point2f> pointsA, pointsB;
		pointsA.reserve(numTotal);
		pointsB.reserve(numTotal);
		for (const DMatch& match : sparse) {
			pointsA.push_back(imgA.keypoints[match.queryIdx].pt);
			pointsB.push_back(imgB.keypoints[match.trainIdx].pt);
		}
		pointsA.insert(pointsA.end(), densePointsA.begin(), densePointsA.end());
		pointsB.insert(pointsB.end(), densePointsB.begin(), densePointsB.end());
		Image imgACopy, imgBCopy;
		ImagePair fit(pair.ID1, pair.ID2);
		MakeFitImages(imgA, imgB, pointsA, pointsB, imgACopy, imgBCopy, fit);
		if (pairsMatcher.GeometricFilter(imgACopy, imgBCopy, fit)) {
			// the polish that closes the fit, again over the fit's inliers but with each dense
			// correspondence at DenseMatchConfig::denseFitWeight: the union found the pose, the sparse
			// matches set it wherever they are many (MakeFitImages numbers the sparse matches first)
			if (fit.relativePose.has_value()) {
				std::vector<double> weights;
				weights.reserve(fit.matches.size());
				for (const DMatch& match : fit.matches)
					weights.push_back(match.queryIdx < sparse.size() ? 1.0 : (double)config.denseFitWeight);
				pairsMatcher.RefineRelativePose(imgACopy, imgBCopy, fit, weights);
			}
			// straight from `fit` rather than recomposed here: GeometricFilter ran the branch's own
			// estimator and left F/E in that branch's own convention (on SHARED_FOCAL it composes F
			// from the RANSAC-estimated focal, not from the camera's nominal K), so recomposing
			// would silently disagree with the pose it just fitted
			pair.F = fit.F;
			pair.E = fit.E;
			pair.relativePose = fit.relativePose;
			bFitted = true;
		}
	}

	// classify both kinds of correspondence against the kept geometry: a descriptor correspondence
	// is sub-pixel and is held to the matcher's own epipolar error; a warp cell is held to a multiple
	// of that same bar (DenseMatchConfig::denseEpipolarErrorFactor), since a correspondence admitted at the
	// half cell the coarse warp can claim pollutes every track it enters downstream
	const MatchConfig& cfg = pairsMatcher.GetConfig();
	// maxEpipolarError 0 turns the RANSAC verification off scene-wide (MatchConfig), and with it the
	// sparse epipolar test: in that configuration every sparse match stands, exactly as the
	// descriptor path keeps its matches unverified
	const bool bVerifySparse = cfg.maxEpipolarError > 0.f;
	const SampsonTest sparseTest(imgA, imgB, pair, cfg.maxEpipolarError);
	const float denseBar = bVerifySparse ? cfg.maxEpipolarError*config.denseEpipolarErrorFactor : denseTolerance;
	const SampsonTest denseTest(imgA, imgB, pair, denseBar);
	ASSERT(sparseTest.IsValid() == denseTest.IsValid());
	pair.matches.reserve(sparse.size());
	for (const DMatch& match : sparse) {
		if (!bVerifySparse || !sparseTest.IsValid() ||
			sparseTest(imgA.keypoints[match.queryIdx].pt, imgB.keypoints[match.trainIdx].pt))
			pair.matches.push_back(match);
		else
			pair.outlierMatches.push_back(match);
	}
	// the sparse segment is materialised here rather than left at -1: the dense segment goes right
	// after it (ImagePair's partition), and StorePairDense appends it as soon as the pair exists
	pair.numFilteredInliers = (int)pair.matches.size();
	pair.numDenseInliers = 0;
	pair.weightedInliers = -1.f;
	// a dense correspondence stands unless the geometry measures it past its bar, which is the
	// verification's own whenever it is on (a factor of 0 included); with the verification off and no
	// tolerance given (re-verification, where the warp the matching's fallback bar comes from is
	// gone) there is no bar, and every one stands
	denseKept.assign(densePointsA.size(), 1);
	if ((bVerifySparse || denseTolerance > 0.f) && denseTest.IsValid())
		FOREACH(k, densePointsA)
			denseKept[k] = denseTest(densePointsA[k], densePointsB[k]) ? 1 : 0;
	return bFitted;
}

void SFM::ReverifyPairDense(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const DenseMatchConfig& config, ImagePair& pair, bool bRefit)
{
	ASSERT(pair.ID1 == imgA.ID && pair.ID2 == imgB.ID);
	const unsigned numSparse = pair.GetNumFilteredInliers(), numDense = pair.GetNumDenseInliers();
	// the dense candidates: the dense segment, as stored (a sample the matching dropped is gone), and
	// after it every other match with a dense end -- a pair invalidated after its fill was stored
	// (FilterRedundantKeypoints, left under the minimum of matches) holds its dense matches among its
	// outliers, and a dense correspondence answers to the dense bar wherever the pair keeps it
	std::vector<DMatch> denseMatches(pair.matches.begin() + numSparse, pair.matches.begin() + numSparse + numDense);
	// the sparse candidates: every other match, described at both ends -- the sparse inliers, any
	// RANSAC inlier a strict filter rejected, and the outliers, so that a match an earlier geometry
	// refused can come back -- in (A, B) keypoint order, the order the guided pass hands them out in
	std::vector<DMatch> sparse;
	sparse.reserve(pair.GetNumMatches() - numDense);
	const auto Route = [&](std::vector<DMatch>::const_iterator first, std::vector<DMatch>::const_iterator last) {
		for (; first != last; ++first)
			(imgA.IsDenseKeypoint(first->queryIdx) || imgB.IsDenseKeypoint(first->trainIdx) ? denseMatches : sparse).push_back(*first);
	};
	Route(pair.matches.cbegin(), pair.matches.cbegin() + numSparse);
	Route(pair.matches.cbegin() + numSparse + numDense, pair.matches.cend());
	Route(pair.outlierMatches.cbegin(), pair.outlierMatches.cend());
	std::sort(sparse.begin(), sparse.end(), [](const DMatch& a, const DMatch& b) {
		return a.queryIdx < b.queryIdx || (a.queryIdx == b.queryIdx && a.trainIdx < b.trainIdx);
	});
	std::vector<Point2f> densePointsA, densePointsB;
	densePointsA.reserve(denseMatches.size());
	densePointsB.reserve(denseMatches.size());
	for (const DMatch& m : denseMatches) {
		densePointsA.push_back(imgA.keypoints[m.queryIdx].pt);
		densePointsB.push_back(imgB.keypoints[m.trainIdx].pt);
	}
	pair.ResetMatches();
	std::vector<uint8_t> denseKept;
	if (!FitAndClassifyPairDense(pairsMatcher, imgA, imgB, sparse, densePointsA, densePointsB, config, 0.f, pair, denseKept, bRefit)) {
		// no geometry explains the candidates any more (too few of them, or too few inliers for the
		// estimator): the pair is invalidated, as GeometricFilter leaves a pair it cannot verify -- no
		// geometry, every candidate an outlier and no partition -- rather than classified against a
		// geometry no fit stands behind, the stored one or none at all, which would admit every one
		pair.ResetMatches();
		pair.ResetGeometry();
		pair.outlierMatches = std::move(sparse);
		pair.outlierMatches.insert(pair.outlierMatches.end(), denseMatches.begin(), denseMatches.end());
		return;
	}
	// the dense segment: the stored dense matches that passed, right after the sparse inliers; a dense
	// match that failed is dropped, its keypoints left in the images
	FOREACH(k, denseMatches)
		if (denseKept[k])
			pair.matches.push_back(denseMatches[k]);
	pair.numDenseInliers = (int)(pair.matches.size() - (size_t)pair.numFilteredInliers);
	pair.meanRayAngle = (pair.relativePose.has_value() && pair.GetNumTrackFormingMatches() > 0) ?
		pair.ComputeMeanRayAngle(imgA, imgB) : 0.f;
}

bool SFM::PlanarTwinDense(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const DenseMatchConfig& config, const ImagePair& pair, ImagePair& twin, float& share)
{
	ASSERT(pair.ID1 == imgA.ID && pair.ID2 == imgB.ID);
	ASSERT(pair.relativePose.has_value());
	share = 0.f;
	const Camera& camA = *imgA.pCamera;
	const Camera& camB = *imgB.pCamera;
	if (camA.GetType() != CameraType::PINHOLE || camB.GetType() != CameraType::PINHOLE)
		return false;
	const unsigned numSparse = pair.GetNumFilteredInliers(), numMatches = pair.GetNumTrackFormingMatches();
	if (numMatches < 8)
		return false;
	// Project track-forming matches onto each camera's normalized image plane
	std::vector<cv::Point2f> pointsA, pointsB;
	pointsA.reserve(numMatches);
	pointsB.reserve(numMatches);
	for (unsigned k = 0; k < numMatches; ++k) {
		const DMatch& match = pair.matches[k];
		const Point3 bearingA = camA.UnprojectNormalized(Cast<REAL>(imgA.keypoints[match.queryIdx].pt));
		const Point3 bearingB = camB.UnprojectNormalized(Cast<REAL>(imgB.keypoints[match.trainIdx].pt));
		if (bearingA.z <= 1e-6 || bearingB.z <= 1e-6)
			continue;
		pointsA.emplace_back((float)(bearingA.x/bearingA.z), (float)(bearingA.y/bearingA.z));
		pointsB.emplace_back((float)(bearingB.x/bearingB.z), (float)(bearingB.y/bearingB.z));
	}
	if (pointsA.size() < 8)
		return false;
	const double angle = (double)PairPixelErrorToAngular(camA, camB, (REAL)pairsMatcher.GetConfig().maxEpipolarError);
	cv::Mat mask;
	const cv::Mat H = cv::findHomography(pointsA, pointsB, cv::RANSAC, angle, mask, 2000, 0.999);
	if (H.empty())
		return false;
	share = (float)cv::countNonZero(mask)/(float)pointsA.size();
	if (share < config.twinMinPlaneShare)
		return false;
	// Keep physically valid decompositions (90% of H inliers in front of both cameras);
	// the one nearest the stored pose is the stored pose itself, the other is its planar twin.
	std::vector<cv::Mat> rotations, translations, normals;
	cv::decomposeHomographyMat(H, cv::Mat::eye(3, 3, CV_64F), rotations, translations, normals);
	const Matrix3x3 R0(pair.relativePose->R);
	const Point3 t0 = pair.relativePose->GetT();
	std::vector<Pose3D> valid;
	int nearest = -1;
	REAL nearestDistance = std::numeric_limits<REAL>::max();
	FOREACH(s, rotations) {
		const Matrix3x3 R(rotations[s]);
		const Point3 t = cv::Vec3d(translations[s]);
		const Point3 n = cv::Vec3d(normals[s]);
		const REAL normT = norm(t);
		if (normT < 1e-9)
			continue;
		unsigned numInFront = 0, numInliers = 0;
		FOREACH(i, pointsA) {
			if (!mask.at<uchar>((int)i))
				continue;
			const Point3 x(pointsA[i].x, pointsA[i].y, 1.0);
			const REAL depth = n.dot(x);
			++numInliers;
			if (depth > 0 && (R*(x/depth) + t).z > 0)
				++numInFront;
		}
		if (numInliers == 0 || numInFront < 0.9*numInliers)
			continue;
		Pose3D& pose = valid.emplace_back();
		pose.R = R;
		pose.SetT(t*(REAL(1)/normT));
		const REAL distance = R2D(ACOS(ComputeAngle(R, R0))) + DirectionAngleDeg(t, t0)/10;
		if (distance < nearestDistance) {
			nearestDistance = distance;
			nearest = (int)valid.size()-1;
		}
	}
	ASSERT(valid.size() <= 2);
	if (valid.size() < 2)
		return false;
	const int other = 1 - nearest;
	// Refine the twin on the same weighted union of matches
	twin = ImagePair(pair.ID1, pair.ID2);
	twin.matches.assign(pair.matches.begin(), pair.matches.begin() + numMatches);
	twin.relativePose = valid[other];
	std::vector<double> weights(numMatches, (double)config.denseFitWeight);
	std::fill(weights.begin(), weights.begin() + numSparse, 1.0);
	const bool bRefined = pairsMatcher.RefineRelativePose(imgA, imgB, twin, weights);
	twin.matches = std::vector<DMatch>(); // only the geometry is kept
	if (!bRefined)
		return false;
	// Reject if the twin converged back to the stored pose
	constexpr REAL minRotationDeg = 0.5, minDirectionDeg = 10;
	return R2D(ACOS(ComputeAngle(Matrix3x3(twin.relativePose->R), R0))) > minRotationDeg ||
		DirectionAngleDeg(twin.relativePose->GetT(), t0) > minDirectionDeg;
}

unsigned SFM::SwitchPlanarTwinsDense(PairsMatcher& pairsMatcher, const DenseMatchConfig& config)
{
	if (config.twinMinLoopGain == 0)
		return 0;
	TD_TIMER_START();
	Scene& scene = pairsMatcher.GetScene();
	// 1) Find the planar twin of every posed, plane-dominated dense pair
	std::vector<ImagePair> twins(scene.pairs.size());
	std::vector<uint8_t> hasTwin(scene.pairs.size(), 0);
	std::atomic<unsigned> numPlanar{0};
	{
		ScopedCvThreads cvGuard(scene.nMaxThreads);
		ScopedPoolDrain poolGuard(scene);
		scene.threadPool.detach_loop(0u, scene.pairs.size(), [&](unsigned i) {
			const ImagePair& pair = scene.pairs[i];
			if (!pair.relativePose.has_value() || !pair.IsDense(scene.images[pair.ID1], scene.images[pair.ID2]))
				return;
			float share;
			hasTwin[i] = PlanarTwinDense(pairsMatcher, scene.images[pair.ID1], scene.images[pair.ID2], config, pair, twins[i], share) ? 1 : 0;
			if (share >= config.twinMinPlaneShare)
				++numPlanar;
		});
	}
	// 2) the candidates and their triangles, each with whether its loop closes for every state
	// combination of the candidates it holds (bit s of `closed`: bit k of s set = its k-th candidate
	// at its twin)
	std::vector<IIndex> candidates; // the pair of each candidate
	std::vector<IIndex> candidateOf(scene.pairs.size(), NO_ID);
	std::unordered_map<PairIdx::PairIndex, IIndex> pairOf;
	std::vector<std::vector<IIndex>> neighbours(scene.images.size());
	FOREACH(i, scene.pairs) {
		const ImagePair& pair = scene.pairs[i];
		if (!pair.relativePose.has_value())
			continue;
		pairOf.emplace(MakePairIdx(pair.ID1, pair.ID2).idx, (IIndex)i);
		neighbours[pair.ID1].push_back(pair.ID2);
		neighbours[pair.ID2].push_back(pair.ID1);
		if (hasTwin[i]) {
			candidateOf[i] = (IIndex)candidates.size();
			candidates.push_back((IIndex)i);
		}
	}
	const auto PairOf = [&](IIndex u, IIndex v) { return pairOf.at(MakePairIdx(u, v).idx); };
	const auto RelativeRotation = [&](IIndex u, IIndex v, bool bTwin) -> Matrix3x3 { // x_v = R_uv x_u
		const IIndex i = PairOf(u, v);
		const ImagePair& pair = scene.pairs[i];
		const Matrix3x3 R(bTwin ? twins[i].relativePose->R : pair.relativePose->R);
		return pair.ID1 == u ? R : Matrix3x3(R.t());
	};
	struct Triangle {
		IIndex candidates[3];
		uint8_t numCandidates;
		uint8_t closed;
	};
	std::vector<Triangle> triangles;
	std::vector<std::vector<IIndex>> trianglesOf(candidates.size());
	const REAL minLoopCos = COS(D2R((REAL)config.twinMaxLoopAngle));
	FOREACH(ci, candidates) {
		const ImagePair& pair = scene.pairs[candidates[ci]];
		for (IIndex c : neighbours[pair.ID1]) {
			if (c == pair.ID2 || !pairOf.count(MakePairIdx(pair.ID2, c).idx))
				continue;
			// the loop a->b->c->a over the pairs (a,b), (b,c) and (c,a)
			const IIndex v[3] = {pair.ID1, pair.ID2, c};
			const IIndex e[3] = {candidates[ci], PairOf(v[1], v[2]), PairOf(v[2], v[0])};
			// a triangle holding several candidates is made once, from the first of them
			if ((candidateOf[e[1]] != NO_ID && candidateOf[e[1]] < ci) || (candidateOf[e[2]] != NO_ID && candidateOf[e[2]] < ci))
				continue;
			Triangle triangle;
			triangle.numCandidates = 0;
			int slot[3];
			for (int k = 0; k < 3; ++k) {
				slot[k] = candidateOf[e[k]] != NO_ID ? (int)triangle.numCandidates : -1;
				if (slot[k] >= 0)
					triangle.candidates[triangle.numCandidates++] = candidateOf[e[k]];
			}
			triangle.closed = 0;
			for (unsigned s = 0; s < (1u << triangle.numCandidates); ++s) {
				const auto AtTwin = [&](int k) { return slot[k] >= 0 && ((s >> slot[k]) & 1u); };
				const Matrix3x3 loop(RelativeRotation(v[1], v[2], AtTwin(1))*RelativeRotation(v[0], v[1], AtTwin(0))*RelativeRotation(v[2], v[0], AtTwin(2)));
				if (ComputeAngle(loop) > minLoopCos)
					triangle.closed |= (uint8_t)(1u << s);
			}
			for (unsigned k = 0; k < triangle.numCandidates; ++k)
				trianglesOf[triangle.candidates[k]].push_back((IIndex)triangles.size());
			triangles.push_back(triangle);
		}
	}
	// 3) the switches, largest gain first, each gain counted with the other candidates at their
	// current states, until no switch gains twinMinLoopGain closed triangles
	std::vector<uint8_t> atTwin(candidates.size(), 0);
	const auto Closed = [&](const Triangle& triangle) {
		unsigned s = 0;
		for (unsigned k = 0; k < triangle.numCandidates; ++k)
			s |= (unsigned)atTwin[triangle.candidates[k]] << k;
		return (int)((triangle.closed >> s) & 1u);
	};
	const auto NumClosed = [&](IIndex ci) {
		int numClosed = 0;
		for (IIndex t : trianglesOf[ci])
			numClosed += Closed(triangles[t]);
		return numClosed;
	};
	const auto Gain = [&](IIndex ci) {
		const int before = NumClosed(ci);
		atTwin[ci] ^= 1;
		const int after = NumClosed(ci);
		atTwin[ci] ^= 1;
		return after - before;
	};
	const auto Precedes = [&](IIndex a, int gainA, IIndex b, int gainB) { // larger gain, more sparse inliers, larger (ID1,ID2)
		if (gainA != gainB)
			return gainA > gainB;
		const ImagePair& pairA = scene.pairs[candidates[a]];
		const ImagePair& pairB = scene.pairs[candidates[b]];
		if (pairA.GetNumFilteredInliers() != pairB.GetNumFilteredInliers())
			return pairA.GetNumFilteredInliers() > pairB.GetNumFilteredInliers();
		return std::make_pair(pairA.ID1, pairA.ID2) > std::make_pair(pairB.ID1, pairB.ID2);
	};
	std::vector<int> gains(candidates.size());
	FOREACH(ci, candidates)
		gains[ci] = Gain(ci);
	#ifndef _RELEASE
	const auto NumClosedAll = [&]() {
		unsigned numClosed = 0;
		for (const Triangle& triangle : triangles)
			numClosed += (unsigned)Closed(triangle);
		return numClosed;
	};
	unsigned numClosedAll = NumClosedAll();
	#endif
	unsigned numSwitches = 0, numSwitchesBack = 0;
	while (!candidates.empty()) {
		IIndex best = 0;
		for (IIndex ci = 1; ci < candidates.size(); ++ci)
			if (Precedes(ci, gains[ci], best, gains[best]))
				best = ci;
		if (gains[best] < (int)config.twinMinLoopGain)
			break;
		if (atTwin[best])
			++numSwitchesBack;
		atTwin[best] ^= 1;
		++numSwitches;
		#ifndef _RELEASE
		// a switch changes only the triangles through its pair, so the closed triangles grow by its
		// gain, at least twinMinLoopGain: the number of switches is bounded and the loop ends
		const unsigned numClosedBefore = numClosedAll;
		numClosedAll = NumClosedAll();
		ASSERT(numClosedAll == numClosedBefore + (unsigned)gains[best] && numClosedAll >= numClosedBefore + config.twinMinLoopGain);
		#endif
		for (IIndex t : trianglesOf[best])
			for (unsigned k = 0; k < triangles[t].numCandidates; ++k)
				gains[triangles[t].candidates[k]] = Gain(triangles[t].candidates[k]);
	}
	// 4) every pair that ends at its twin takes it and re-classifies its matches against it
	unsigned numSwitched = 0;
	FOREACH(ci, candidates) {
		if (!atTwin[ci])
			continue;
		ImagePair& pair = scene.pairs[candidates[ci]];
		const ImagePair& twin = twins[candidates[ci]];
		atTwin[ci] = 0;
		const int loopsStored = NumClosed(ci);
		atTwin[ci] = 1;
		const int loopsTwin = NumClosed(ci);
		const Point3 t0 = pair.relativePose->GetT(), t1 = twin.relativePose->GetT();
		const unsigned numSparseBefore = pair.GetNumFilteredInliers(), numDenseBefore = pair.GetNumDenseInliers();
		const REAL rotationDeg = R2D(ACOS(ComputeAngle(Matrix3x3(pair.relativePose->R), Matrix3x3(twin.relativePose->R))));
		const REAL directionDeg = DirectionAngleDeg(t0, t1);
		pair.relativePose = twin.relativePose;
		pair.E = twin.E;
		pair.F = twin.F;
		ReverifyPairDense(pairsMatcher, scene.images[pair.ID1], scene.images[pair.ID2], config, pair, false);
		VERBOSE("Pair %u-%u switched to its planar twin: loops %d -> %d of %u within %g deg, rotation change %.2f deg, direction change %.1f deg; sparse inliers %u -> %u, dense %u -> %u",
			pair.ID1, pair.ID2, loopsStored, loopsTwin, (unsigned)trianglesOf[ci].size(), config.twinMaxLoopAngle, rotationDeg,
			directionDeg, numSparseBefore, pair.GetNumFilteredInliers(), numDenseBefore, pair.GetNumDenseInliers());
		++numSwitched;
	}
	VERBOSE("Planar twins: %u plane-dominated pairs (share >= %g) of %u, %u with a distinct twin, %u switches (%u back) in %u triangles, %u pairs switched (%s)",
		numPlanar.load(), config.twinMinPlaneShare, (unsigned)scene.pairs.size(), (unsigned)candidates.size(), numSwitches, numSwitchesBack,
		(unsigned)triangles.size(), numSwitched, TD_TIMER_GET_FMT().c_str());
	return numSwitched;
}
/*----------------------------------------------------------------*/

void SFM::StorePairDense(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
	ImagePair&& pair, const DenseMatches& dense, int warpSize)
{
	ASSERT(pair.ID1 < pair.ID2 && pair.ID2 < scene.images.size());
	ASSERT(warpSize > 0);
	const PairIdx::PairIndex key = PairIdx(pair.ID1, pair.ID2).idx;
	const auto it = pairIndexMap.find(key);
	IIndex idxPair;
	if (it != pairIndexMap.end()) {
		idxPair = it->second;
		ASSERT(scene.pairs[idxPair].GetNumDenseInliers() == 0);
		scene.pairs[idxPair] = std::move(pair);
	} else {
		// overlapRatio/overlapArea stay at their reset value (0): a created pair is weighted exactly
		// like any other pair, ComputePairsWeights computing its own overlap proxy for it. Stamping
		// a full 1/1 overlap here would survive PairsMatcher::Match --
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

#ifdef _USE_ONNXRUNTIME
// The summary line of the one pass, in the one field list the measurement tools parse; a pass that stopped
// before the warp pass and the store ran has every field they would have filled at 0
static void LogOnePassSummary(size_t numCandidates, unsigned numJudged, unsigned numAdmitted, unsigned numStored,
	unsigned numDenseOnly, unsigned numSlots, unsigned numLoads, unsigned numReloads, unsigned numAlreadyStored,
	unsigned numUnprepared, unsigned numFailedLoads, unsigned numFailedMatches, unsigned numDenseMatches, const String& elapsed)
{
	DEBUG("Dense one pass: %u candidates, %u judged, %u admitted, %u stored, %u dense-only; "
		"%u slots, %u loads, %u reloads; %u already stored, %u unprepared, %u failed loads, %u failed matches, %u dense matches (%s)",
		(unsigned)numCandidates, numJudged, numAdmitted, numStored, numDenseOnly,
		numSlots, numLoads, numReloads,
		numAlreadyStored, numUnprepared, numFailedLoads, numFailedMatches, numDenseMatches,
		elapsed.c_str());
}
#endif

bool SFM::MatchPairsDense(PairsMatcher& pairsMatcher, DenseMatcher& matcher, const PairIdxArr& candidatePairs,
	const DenseMatchConfig& config, unsigned& numStored)
{
	numStored = 0;
#ifdef _USE_ONNXRUNTIME
	ASSERT(matcher.IsLoaded());
	if (candidatePairs.empty())
		return true;
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
	// the two reasons a candidate never reaches the warp pass, counted apart because they mean
	// opposite things to whoever reads the summary: a pair the scene already holds is the ordinary
	// outcome of a re-run or of the feedback round, while an image without a camera or without
	// descriptors is a failure of an earlier stage that silently shrinks the view graph
	unsigned numAlreadyStored = 0, numUnprepared = 0;
	for (const PairIdx& p : sorted) {
		ASSERT(p.i < p.j); // MakePairIdx orders the two indices, so each unordered pair appears once
		if (pairIndexMap.find(p.idx) != pairIndexMap.end()) {
			++numAlreadyStored;
			continue;
		}
		const Image& imgA = scene.images[p.i];
		const Image& imgB = scene.images[p.j];
		// the verdict fits in the two cameras' bearings and the guided pass reads descriptors
		if (!imgA.HasCamera() || !imgB.HasCamera() || !imgA.HasDescriptors() || !imgB.HasDescriptors()) {
			++numUnprepared;
			continue;
		}
		pairs.push_back(p);
	}
	if (pairs.empty()) {
		// nothing past this point ran, so every field the warp pass and the store would have filled is 0
		LogOnePassSummary(candidatePairs.size(), 0, 0, 0, 0, 0, 0, 0, numAlreadyStored, numUnprepared, 0, 0, 0, TD_TIMER_GET_FMT());
		return true; // no candidate left to judge is an empty pass, not a failed one
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
	std::atomic<unsigned> numJudged{0}, numAdmitted{0}, numGated{0}, numWithCovisibility{0};
	const int warpSize = matcher.GridSize();
	WarpPassStats stats;
	if (!ForEachDenseWarp(pairsMatcher, matcher, pairs, config.slotBudget, _T("Dense match image pairs"),
		[&](size_t p, const PairIdx& pairIdx, PairOutput& warps, unsigned threadIdx) {
			const Image& imgA = scene.images[pairIdx.i];
			const Image& imgB = scene.images[pairIdx.j];
			if (warps.covisibility) {
				++numWithCovisibility;
				if (*warps.covisibility < config.minCovisibility) {
					++numGated;
					return;
				}
			}
			ImagePair pair(pairIdx.i, pairIdx.j);
			pair.covisibility = warps.covisibility;
			pair.posePrior = warps.pose;
			PairVerdict verdict;
			JudgePairDense(pairsMatcher, imgA, imgB, warps, config, pair, verdict);
			++numJudged;
			if (!verdict.admitted) {
				DEBUG_ULTIMATE("Dense pair %u-%u: conf %.4f %.4f inl %.4f %.4f REJECT",
					pairIdx.i, pairIdx.j, verdict.confidentAreaA, verdict.confidentAreaB,
					verdict.inlierAreaA, verdict.inlierAreaB);
				return;
			}
			++numAdmitted;
			std::vector<Point2f> trackedB;
			std::vector<uchar> trackStatus;
			TrackKeypointsByWarp(imgA, imgB, warps.ab.warp, warps.ab.confidence, config.minConfidence,
				trackedB, trackStatus);
			GuidedSearch search;
			const float cell = (float)MAXF(imgB.GetWidth(), imgB.GetHeight()) / (float)warpSize;
			search.length = config.guidedBandLengthCells * cell;
			search.F = pair.F;
			search.sameFeatureDistance = config.guidedSameFeatureDistance;
			search.halfWidth = config.guidedBandHalfWidthCells * cell;
			std::vector<DMatch> guided;
			MatchFeaturesGuided(pairsMatcher, imgA, imgB, trackedB, trackStatus, search, threadIdx, guided);
			AssembledPair& result = results[p];
			result.bAssembled = AssemblePairDense(pairsMatcher, imgA, imgB, verdict, guided, config,
				warpSize, pair, result.dense);
			const unsigned numSparse = result.bAssembled ? pair.GetNumFilteredInliers() : 0u;
			if (result.bAssembled)
				result.pair = std::move(pair);
			DEBUG_ULTIMATE("Dense pair %u-%u: conf %.4f %.4f inl %.4f %.4f ADMIT cap %u grid %d band %.1f guided %u sparse %u dense %u",
				pairIdx.i, pairIdx.j, verdict.confidentAreaA, verdict.confidentAreaB,
				verdict.inlierAreaA, verdict.inlierAreaB, DenseFillCeiling(config, verdict),
				DenseFillGridSide(config.denseMatchesPerFrame, warpSize), search.halfWidth, (unsigned)guided.size(), numSparse,
				(unsigned)result.dense.pointsA.size());
		}, stats))
		return false;

	unsigned numDenseOnly = 0;
	size_t numDenseMatches = 0;
	FOREACH(p, results) {
		AssembledPair& result = results[p];
		if (!result.bAssembled)
			continue;
		if (result.pair.GetNumFilteredInliers() == 0)
			++numDenseOnly;
		numDenseMatches += result.dense.pointsA.size();
		StorePairDense(scene, pairIndexMap, std::move(result.pair), result.dense, warpSize);
		++numStored;
	}
	if (config.minCovisibility > 0.f && numWithCovisibility.load() > 0)
		DEBUG("Dense covisibility gate: %u of %u warped pairs below %g", numGated.load(), numWithCovisibility.load(), config.minCovisibility);
	LogOnePassSummary(candidatePairs.size(), numJudged.load(), numAdmitted.load(), numStored, numDenseOnly,
		stats.numSlots, (unsigned)stats.numLoads, (unsigned)stats.numReloads,
		numAlreadyStored, numUnprepared, stats.numFailedLoads, stats.numFailedMatches, (unsigned)numDenseMatches, TD_TIMER_GET_FMT());
	return true;
#else // _USE_ONNXRUNTIME
	ASSERT(false);
	return false;
#endif // _USE_ONNXRUNTIME
}

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
