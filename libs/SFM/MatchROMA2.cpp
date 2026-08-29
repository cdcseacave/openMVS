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
// only after that task's future has resolved (see the prefetch ring in
// ComputeGlobalDescriptorsROMA2 below), so no locking is needed on Image.
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
	// pipelined like the SiftGPU bulk driver (FeaturesExtractor.cpp): pool tasks load+preprocess
	// image i+k while the calling thread runs Describe(i) on the previously prefetched slot
	const unsigned nPrefetch = MINF(2u*(unsigned)scene.threadPool.get_thread_count(), 8u);
	cv::setNumThreads(1); // temporarily turn off multi-threading for OpenCV functions (the pool tasks are already parallel)
	Util::Progress progress(_T("Describe images"), nImages);
	GET_LOGCONSOLE().Pause();

	std::vector<PlanarImage> planars(nPrefetch);
	std::deque<std::future<bool>> prefetched;
	const auto Prefetch = [&](IIndex i) {
		prefetched.push_back(scene.threadPool.submit_task([&scene, &planars, i, size, nPrefetch]() {
			return PrepareImageROMA2(scene.images[i], size, planars[i % nPrefetch]);
		}));
	};
	for (IIndex i = 0; i < MINF(nPrefetch, nImages); ++i)
		Prefetch(i);

	unsigned numDescribed = 0;
	OrtTensor layers = roma2.MakeLayers(); // reused across every image (design decision 7: the describe pass keeps only the pooled vector, not the layers)
	const bool bFacets = config.retrievalRecipe == RetrievalRecipe::FACETS;
	OrtTensor layersHost;
	if (!bFacets)
		layersHost = OrtTensor::Host(roma2.LayersShape()); // the LAYERS (parity) recipe pools the `layers` output itself, so it must be read back to the host
	const unsigned numSlices = (unsigned)roma2.LayersShape()[1];
	const unsigned numChannels = (unsigned)roma2.LayersShape()[4];
	std::vector<float> facets, descriptor;
	for (IIndex i = 0; i < nImages; ++i, ++progress) {
		// consume the slot this image was prefetched into before ever reusing it below
		const bool prepared = prefetched.front().get();
		prefetched.pop_front();
		Image& img = scene.images[i];
		if (prepared && roma2.Describe(planars[i % nPrefetch].data(), bFacets ? layers : layersHost, bFacets ? &facets : NULL)) {
			PoolRetrievalDescriptor(bFacets ? facets.data() : layersHost.HostData(), numSlices, roma2.NumPatches(), numChannels,
				config.retrievalRecipe, config.retrievalPower, descriptor);
			img.globalDescriptor = cv::Mat(1, (int)descriptor.size(), CV_32F, descriptor.data()).clone();
			++numDescribed;
		} else {
			VERBOSE("error: could not describe image %u '%s'", img.ID, img.fileName.c_str());
		}
		// only now may the slot this image just consumed be overwritten by a new prefetch
		if (i + nPrefetch < nImages)
			Prefetch(i + nPrefetch);
	}

	GET_LOGCONSOLE().Play();
	progress.close();
	cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading

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
