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
#include "GlobalDescriptors.h"

#include <deque>
#include <future>

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
// Runtime call for the previously prepared slot (the SiftGPU bulk-driver pattern,
// FeaturesExtractor.cpp:113-153, applied to ROMA2). Fixed-size ring of `capacity` planar
// buffers: Submit(image) schedules a pool task writing into slot `image % capacity`; Take()
// blocks on the oldest still-outstanding submission and returns whether it succeeded;
// Planar(image) exposes the buffer that submission wrote (or failed to write) into.
//
// Ordering invariant callers must keep: Submit(image + capacity) may only be issued after
// Take() has already consumed image's submission, since the two share slot
// `image % capacity`. ComputeGlobalDescriptorsROMA2's loop keeps this by construction
// (it always Take()s image i, and only then conditionally Submit()s i + capacity); Submit()
// itself ASSERTs the ring never holds more outstanding submissions than slots, which is the
// same invariant restated as a count and catches a caller that broke the ordering.
// Task 9's dense-matching slot loader reuses this ring unchanged.
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
			pending.front().wait(); // never get(): a destructor must not propagate an exception
			pending.pop_front();
		}
	}

	void Submit(IIndex image) {
		ASSERT(pending.size() < planars.size()); // never more outstanding submissions than slots (see class comment)
		pending.push_back(scene.threadPool.submit_task([this, image]() {
			return PrepareImageROMA2(scene.images[image], size, planars[image % planars.size()]);
		}));
	}

	bool Take() {
		ASSERT(!pending.empty());
		const bool prepared = pending.front().get();
		pending.pop_front();
		return prepared;
	}

	const PlanarImage& Planar(IIndex image) const { return planars[image % planars.size()]; }

private:
	Scene& scene;
	int size;
	std::vector<PlanarImage> planars;
	std::deque<std::future<bool>> pending;
};

// Restores the describe pass's OpenCV thread count, log-console pause, and progress bar on
// scope exit, including through an exception (they used to be plain statements reachable only
// on the non-throwing path). Finish() runs the restoration immediately and is what the normal
// path calls, right before the summary DEBUG line, so that line isn't swallowed by the paused
// console; the destructor calls it too (idempotent) as the safety net for any exceptional exit.
struct ScopedDescribeState
{
	Scene& scene;
	Util::Progress progress;
	bool bFinished = false;

	ScopedDescribeState(Scene& _scene, IIndex nImages)
		: scene(_scene), progress(_T("Describe images"), nImages)
	{
		cv::setNumThreads(1); // temporarily turn off multi-threading for OpenCV functions (the pool tasks are already parallel)
		GET_LOGCONSOLE().Pause();
	}
	ScopedDescribeState(const ScopedDescribeState&) = delete;
	ScopedDescribeState& operator=(const ScopedDescribeState&) = delete;
	~ScopedDescribeState() { Finish(); }

	void Finish() {
		if (bFinished)
			return;
		bFinished = true;
		GET_LOGCONSOLE().Play();
		progress.close();
		cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading
	}
};

#endif // _USE_ONNXRUNTIME

} // namespace


// D E S C R I B E   P A S S //////////////////////////////////////////

unsigned SFM::ComputeGlobalDescriptorsROMA2(Scene& scene, RoMa2Onnx& roma2, const ROMA2Config& config)
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
	ScopedDescribeState state(scene, nImages);
	PrefetchRing ring(scene, nPrefetch, size);
	for (IIndex i = 0; i < MINF(nPrefetch, nImages); ++i)
		ring.Submit(i);

	unsigned numDescribed = 0;
	const bool bFacets = config.retrievalRecipe == RetrievalRecipe::FACETS;
	// allocate only the tensor the active recipe actually reads: FACETS pools the host
	// `facets' readback and leaves `layers` a device-resident scratch write target it never
	// reads back; LAYERS pools `layers` itself, which must therefore be a host tensor
	OrtTensor layers, layersHost;
	if (bFacets)
		layers = roma2.MakeLayers(); // reused across every image (design decision 7: the describe pass keeps only the pooled vector, not the layers)
	else
		layersHost = OrtTensor::Host(roma2.LayersShape());
	if (!(bFacets ? layers : layersHost).IsValid()) {
		VERBOSE("error: could not allocate the ROMA2 %s tensor", bFacets ? "layers" : "layers-host");
		return 0;
	}
	const unsigned numSlices = (unsigned)roma2.LayersShape()[1];
	const unsigned numChannels = (unsigned)roma2.LayersShape()[4];
	std::vector<float> facets, descriptor;
	for (IIndex i = 0; i < nImages; ++i, ++state.progress) {
		// consume the slot this image was prefetched into before ever reusing it below
		const bool prepared = ring.Take();
		Image& img = scene.images[i];
		if (prepared && roma2.Describe(ring.Planar(i).data(), bFacets ? layers : layersHost, bFacets ? &facets : NULL)) {
			PoolRetrievalDescriptor(bFacets ? facets.data() : layersHost.HostData(), numSlices, roma2.NumPatches(), numChannels,
				config.retrievalRecipe, config.retrievalPower, descriptor);
			img.globalDescriptor = cv::Mat(1, (int)descriptor.size(), CV_32F, descriptor.data()).clone();
			++numDescribed;
		} else {
			VERBOSE("error: could not describe image %u '%s'", img.ID, img.fileName.c_str());
		}
		// only now may the slot this image just consumed be overwritten by a new submission
		if (i + nPrefetch < nImages)
			ring.Submit(i + nPrefetch);
	}
	state.Finish(); // unpause the log console before the summary line below (see struct comment)

	// descriptor may be empty (no image was ever successfully pooled): fall back to the
	// manifest's declared dimension for the recipe so the summary line stays meaningful
	const unsigned descriptorDim = descriptor.empty() ?
		(bFacets ? roma2.Manifest().facetsDim : roma2.Manifest().layersDim) : (unsigned)descriptor.size();
	DEBUG("Global descriptors computed for %u/%u images (%s recipe, %u-D, %s provider, %s)",
		numDescribed, (unsigned)nImages, bFacets ? "value-facets" : "layers", descriptorDim,
		roma2.ProviderName().c_str(), TD_TIMER_GET_FMT().c_str());
	return numDescribed;
#else // _USE_ONNXRUNTIME
	// unreachable: RoMa2Onnx::IsAvailable() is false in this build, so Scene::MatchPairs never
	// loads a model and never calls here
	ASSERT(false);
	return 0;
#endif // _USE_ONNXRUNTIME
}

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
