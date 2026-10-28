/*
 * FeaturesExtractor.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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

#include "Common.h"
#include "FeaturesExtractor.h"
#include "Scene.h"
#include "SphereCubeMap.h"
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _USE_SIFTGPU
#include <glad/glad.h>
#include <siftgpu/SiftGPU.h>
#endif

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("FeatExtr"));


#ifdef _USE_SIFTGPU
namespace {

/**
 * @brief Coordinates SiftGPU feature extraction
 *
 * Owns the persistent GPU context and exposes both a synchronous per-image
 * entry point (ExtractImage) and a pipelined bulk driver (ProcessImages).
 * Both share the same per-image primitives — only the scheduling differs:
 * - Per-image: run GPU + post-process, both on the calling thread.
 * - Bulk: run GPU on the main thread, dispatch post-processing to the
 *   thread pool so it overlaps with the next image's GPU work. The next
 *   images are decoded and prepared in parallel on the thread pool.
 *
 * File-local singleton: the GPU context lives for the process and is
 * shared across all FeaturesExtractor instances that use SIFTGPU.
 */
class SiftGPUFeatureCoordinator
{
public:
	// Singleton accessor: lazily constructs and initializes the coordinator on
	// first call, binding it to the given extractor's config + scene. Subsequent
	// calls reuse the existing singleton and ignore the passed extractor — if
	// you need to rebind to a different FeaturesExtractor (e.g. with a different
	// useCUDA setting), call Release() first. Returns nullptr on init failure.
	static SiftGPUFeatureCoordinator* GetCoordinator(FeaturesExtractor& extractor) {
		if (!instance) {
			std::unique_ptr<SiftGPUFeatureCoordinator> candidate(new SiftGPUFeatureCoordinator(extractor));
			if (!candidate->Initialize())
				return nullptr;
			instance = std::move(candidate);
		}
		return instance.get();
	}

	// Destroy the singleton and release the GPU context.
	static void Release() {
		instance.reset();
	}

	// Singletons are non-copyable / non-movable.
	SiftGPUFeatureCoordinator(const SiftGPUFeatureCoordinator&) = delete;
	SiftGPUFeatureCoordinator& operator=(const SiftGPUFeatureCoordinator&) = delete;
	SiftGPUFeatureCoordinator(SiftGPUFeatureCoordinator&&) = delete;
	SiftGPUFeatureCoordinator& operator=(SiftGPUFeatureCoordinator&&) = delete;

	// Synchronous per-image extraction path; caller manages scheduling.
	// Runs GPU call and post-processing on the calling thread — used by
	// FeaturesExtractor::ExtractImage for keyframe-style workflows that
	// process one frame at a time.
	bool ExtractImage(Image& img, bool skipIO = false) {
		if (!img.HasPixels() && !img.LoadPixels(true)) {
			VERBOSE("error: no pixels loaded for image %u", img.ID);
			return false;
		}
		const cv::Mat input(PrepareSIFTInput(img.pixels));
		CLISTDEF0IDX(SiftGPU::SiftKeypoint,uint32_t) keys;
		FloatArr descs;
		if (!RunSIFTOnImage(img.ID, input, keys, descs))
			return false;
		StoreFeatures(img, input.size(), keys, descs, skipIO);
		return !img.keypoints.empty();
	}

	// Bulk driver: pipelines image decoding and post-processing around the
	// serialized GPU call to keep the device busy.
	// Decoding a large image (ex. a 24 MP JPEG) takes longer than SiftGPU takes
	// to process it, so the next images are decoded in parallel on the scene
	// thread pool, bounded by the number of pool threads and by a budget on the
	// prepared data waiting for the GPU. The decode task also prepares the GPU
	// input (see PrepareSIFTInput), keeping all per-image CPU work off the GPU
	// thread.
	// Each image is decoded by exactly one task and the GPU thread waits on that
	// task's future before touching its data, so no image is ever loaded twice
	// or read while being written. The pool queue is FIFO: a decode task is
	// queued several images ahead, so it only waits behind the (short)
	// post-processing tasks of older images; the GPU thread is not a pool
	// worker, so waiting on a future can not deadlock the pool.
	size_t ProcessImages(Util::Progress& progress) {
		typedef std::chrono::steady_clock Clock;
		const Clock::time_point timeStart(Clock::now());
		std::atomic<size_t> numFeatures(0);
		Scene& scene = extractor.GetScene();
		const bool releasePixels = extractor.GetConfig().releaseImagePixels;
		// spherical images are processed inline (not prefetched),
		// as they are decoded in color and split in faces
		auto IsSpherical = [](const Image& img) {
			return img.pCamera && img.pCamera->GetType() == CameraType::SPHERICAL;
		};
		// images to process, in scene order
		IIndexArr todo;
		todo.reserve(scene.images.size());
		FOREACH(i, scene.images) {
			const Image& img = scene.images[i];
			if (!(img.HasFeatures() && img.HasDescriptors()))
				todo.push_back(i);
		}
		// look-ahead bounds: at most one decode per pool thread, and at most this
		// many bytes of prepared GPU input in flight (8 float images of 24 MP)
		constexpr size_t maxBytesInFlight = size_t(768) * 1024 * 1024;
		const size_t maxImagesInFlight = MAXF(size_t(1), (size_t)scene.threadPool.get_thread_count());
		std::vector<std::future<bool>> decoded(todo.size());
		std::vector<cv::Mat> inputs(todo.size());
		std::vector<size_t> inputBytes(todo.size(), 0);
		size_t nextDecode = 0, imagesInFlight = 0, bytesInFlight = 0;
		auto ScheduleDecodes = [&](size_t current) {
			for (; nextDecode < todo.size(); ++nextDecode) {
				Image& img = scene.images[todo[nextDecode]];
				if (IsSpherical(img))
					continue;
				const size_t bytes = img.pCamera ? sizeof(float) *
					(size_t)img.pCamera->GetWidth() * (size_t)img.pCamera->GetHeight() : size_t(0);
				// the image needed right now is always scheduled, and so is the next
				// one if nothing else is in flight, even if it exceeds the budget
				if (nextDecode > current && (imagesInFlight >= maxImagesInFlight ||
						(imagesInFlight > 0 && bytesInFlight + bytes > maxBytesInFlight)))
					break;
				decoded[nextDecode] = scene.threadPool.submit_task([&img, &input = inputs[nextDecode], releasePixels]() {
					// the future must not throw: the GPU thread would leave this
					// function while other decode tasks still reference its locals
					try {
						if (!img.HasPixels() && !img.LoadPixels(true))
							return false;
						input = PrepareSIFTInput(img.pixels);
					} catch (const std::exception&) {
						return false;
					}
					// the GPU only needs the prepared input
					if (releasePixels)
						img.ReleasePixels();
					return true;
				});
				inputBytes[nextDecode] = bytes;
				++imagesInFlight;
				bytesInFlight += bytes;
			}
		};
		double gpuSeconds = 0, waitSeconds = 0;
		size_t numGPUImages = 0;
		FOREACH(t, todo) {
			Image& img = scene.images[todo[t]];
			progress.display(todo[t]);
			// Spherical images can't be fed to SIFTGPU directly — they need
			// per-face tangent extraction. Delegate to the per-image driver,
			// which recursively re-enters this coordinator for each face.
			if (IsSpherical(img)) {
				cv::Ptr<cv::Feature2D> unusedDet;
				if (extractor.ExtractImage(img, unusedDet))
					numFeatures.fetch_add(img.keypoints.size(), std::memory_order_relaxed);
				continue;
			}
			ScheduleDecodes(t);
			// wait for this image's decode task
			const Clock::time_point timeWait(Clock::now());
			ASSERT(decoded[t].valid());
			const bool loaded = decoded[t].get();
			waitSeconds += std::chrono::duration<double>(Clock::now() - timeWait).count();
			--imagesInFlight;
			bytesInFlight -= inputBytes[t];
			if (!loaded) {
				VERBOSE("error: no pixels loaded for image %u", img.ID);
				continue;
			}
			// Main-thread GPU call; skip image on failure
			const cv::Size size(inputs[t].size());
			CLISTDEF0IDX(SiftGPU::SiftKeypoint,uint32_t) keys;
			FloatArr descs;
			const Clock::time_point timeGPU(Clock::now());
			const bool extracted = RunSIFTOnImage(img.ID, inputs[t], keys, descs);
			gpuSeconds += std::chrono::duration<double>(Clock::now() - timeGPU).count();
			inputs[t].release();
			if (!extracted)
				continue;
			++numGPUImages;
			// Offload post-processing to thread pool so it overlaps with
			// the next image's GPU work (async task)
			scene.threadPool.detach_task(
				[this, &img, size, keys = std::move(keys), descs = std::move(descs), &numFeatures]() {
					numFeatures.fetch_add(StoreFeatures(img, size, keys, descs), std::memory_order_relaxed);
				});
		}
		// Wait for all post-processing tasks to complete
		scene.threadPool.wait();
		const double totalSeconds = std::chrono::duration<double>(Clock::now() - timeStart).count();
		DEBUG_EXTRA("SiftGPU pipeline: %u images in %.3fs (%.2f images/s): GPU thread busy %.3fs (%.1f%%), waited %.3fs for decoding",
			numGPUImages, totalSeconds, numGPUImages / MAXF(totalSeconds, 1e-9),
			gpuSeconds, 100.0 * gpuSeconds / MAXF(totalSeconds, 1e-9), waitSeconds);
		return numFeatures.load(std::memory_order_relaxed);
	}

private:
	explicit SiftGPUFeatureCoordinator(FeaturesExtractor& _extractor)
		: extractor(_extractor) {}

	// Initialize SiftGPU context (returns false on failure)
	bool Initialize() {
		constexpr unsigned maxImageSize = 5120;
		constexpr int firstOctave = -1;
		constexpr int octaveResolution = 3;
		constexpr unsigned maxNumOrientations = 2;
		// Minimum scale-normalized smaller principal curvature sigma^2*|lambda_min| of
		// the DoG Hessian (SiftGPU -cmin, implemented in the CUDA kernels): a keypoint must
		// be curved enough in both directions, an absolute cornerness floor that takes over
		// from the DoG value as the main contrast test. It lets the very low DoG peak
		// threshold (SiftGPU default 0.02/3 for 3 levels per octave) keep low-contrast
		// corners and blobs of weakly textured areas (walls, floors) while rejecting flat
		// and edge-like extrema; without the floor (GLSL) the higher threshold is used.
		// The edge threshold 12.5 (maximum ratio of the principal curvatures) is the one
		// of the RSa1 detector and increases the ratio-test precision of the matches.
		constexpr float minCurvature = 0.001f;
		constexpr float edgeThreshold = 12.5f;
		constexpr unsigned maxNumFeatures = 0; // 0 = no limit
		constexpr bool upright = false;
		const bool darknessAdaptivity = extractor.GetConfig().useCUDA ? false : true;
		int gpuIndices[1] = { -1 };

		std::vector<std::string> args;
		args.push_back("./sift_gpu");
		#ifndef _RELEASE
		args.push_back("-v"); args.push_back("1");
		#else
		args.push_back("-v"); args.push_back("0");
		#endif
		#ifdef _USE_CUDA
		if (extractor.GetConfig().useCUDA && gpuIndices[0] < 0)
			gpuIndices[0] = 0;
		if (gpuIndices[0] >= 0) {
			args.push_back("-cuda"); args.push_back(std::to_string(gpuIndices[0]));
		}
		#endif
		const bool cuda = gpuIndices[0] >= 0;
		const float peakThreshold = cuda ? 0.001f : 0.0025f;
		if (darknessAdaptivity) {
			if (gpuIndices[0] >= 0)
				DEBUG("warning: darkness adaptivity only available for GLSL SiftGPU.");
			args.push_back("-da");
		}
		const int octaveFactor = 1 << -MINF(0, firstOctave);
		args.push_back("-maxd"); args.push_back(std::to_string(maxImageSize * octaveFactor));
		args.push_back("-t"); args.push_back(std::to_string(peakThreshold));
		args.push_back("-e"); args.push_back(std::to_string(edgeThreshold));
		if (cuda) {
			args.push_back("-cmin"); args.push_back(std::to_string(minCurvature));
		}
		if (maxNumFeatures > 0) {
			args.push_back("-tc2"); args.push_back(std::to_string(maxNumFeatures));
		}
		args.push_back("-fo"); args.push_back(std::to_string(firstOctave));
		args.push_back("-d"); args.push_back(std::to_string(octaveResolution));
		if (upright) {
			args.push_back("-ofix");
			args.push_back("-mo"); args.push_back("1");
		} else {
			args.push_back("-mo"); args.push_back(std::to_string(maxNumOrientations));
		}

		std::vector<const char*> argv;
		for (const auto& a : args) argv.push_back(a.c_str());
		gpu.ParseParam(argv.size(), argv.data());

		if (gpu.CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
			VERBOSE("error: SiftGPU not fully supported");
			return false;
		}
		const int maxNumFeaturesPerImage = extractor.GetConfig().GetMaxNumFeatures();
		const int maxNumFeaturesPerImageGPU = gpu.GetMaxNumFeatures();
		if (maxNumFeaturesPerImageGPU < maxNumFeaturesPerImage) {
			constexpr LPCTSTR warningMessage = "warning: SiftGPU only supports a maximum of %d features per image"
				#ifdef _USE_CUDA
				", consider using CUDA to avoid this limitation"
				#endif
				"; the max number of features will be capped from %d to %d";
			VERBOSE(warningMessage, maxNumFeaturesPerImageGPU, maxNumFeaturesPerImage, maxNumFeaturesPerImageGPU);
			extractor.GetConfig().SetMaxNumFeatures(maxNumFeaturesPerImageGPU);
		}
		DEBUG_EXTRA("SiftGPU initialized: %s mode (%d max-features-per-image)",
			gpu.GetLanguage() == SiftGPU::SIFTGPULANG_CUDA ? "CUDA" : (gpu.GetLanguage() == SiftGPU::SIFTGPULANG_OPENCL ? "OpenCL" : "GLSL"), maxNumFeaturesPerImageGPU);
		return true;
	}

	// Convert the gray pixels to the normalized float image SiftGPU works on.
	// This is exactly SiftGPU's own conversion of 8-bit luminance input
	// (value / 255.f, see DownSamplePixelDataI2F), so the features are unchanged;
	// doing it here lets it run on a decode worker instead of the GPU thread, and
	// SiftGPU then uses the float buffer directly (no per-image allocation+copy).
	// Pure function of its input, safe to call from any thread.
	static cv::Mat PrepareSIFTInput(const cv::Mat& pixels) {
		cv::Mat gray;
		if (pixels.channels() == 1)
			gray = pixels;
		else
			cv::cvtColor(pixels, gray, pixels.channels() == 4 ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
		ASSERT(gray.type() == CV_8UC1);
		float lut[256];
		for (int v = 0; v < 256; ++v)
			lut[v] = float(v) / 255.f;
		cv::Mat input(gray.size(), CV_32FC1);
		for (int r = 0; r < gray.rows; ++r) {
			const uint8_t* const src = gray.ptr<uint8_t>(r);
			float* const dst = input.ptr<float>(r);
			for (int c = 0; c < gray.cols; ++c)
				dst[c] = lut[src[c]];
		}
		return input;
	}

	// Main-thread GPU work: run SIFT on the prepared input, download results.
	// SiftGPU's context is bound to a single thread, so this must never be called
	// concurrently against the same coordinator.
	// The input must be a continuous CV_32FC1 image owned by the caller: SiftGPU
	// uses it in place and compacts its rows when the width is not a multiple of 4.
	bool RunSIFTOnImage(uint32_t imageID, const cv::Mat& input,
		CLISTDEF0IDX(SiftGPU::SiftKeypoint,uint32_t)& keys,
		FloatArr& descs)
	{
		ASSERT(input.type() == CV_32FC1 && input.isContinuous());
		if (!gpu.RunSIFT(input.cols, input.rows, input.data, GL_LUMINANCE, GL_FLOAT)) {
			VERBOSE("error: SiftGPU failed on image %u", imageID);
			return false;
		}
		const int num = gpu.GetFeatureNum();
		keys.Resize(num);
		descs.Resize(num * 128);
		gpu.GetFeatureVector(keys.data(), descs.data());
		return true;
	}

	// Adaptive grid selection of at most maxFeatures keypoints, with cells of
	// about cellSize pixels covering the whole image. When there are more
	// keypoints than the budget, every cell gets the same quota (water filling):
	// cells with fewer keypoints keep all of them and their unused quota goes to
	// the denser cells (the remainder one per cell, in cell order), so weakly
	// textured areas keep every feature SiftGPU finds there. Inside a cell the
	// largest scales are kept (ties by index). Unlike a fixed per-cell cap, no
	// feature is dropped while the image is under budget.
	// Returns the kept indices in SiftGPU order.
	static std::vector<uint32_t> SelectFeatures(const CLISTDEF0IDX(SiftGPU::SiftKeypoint,uint32_t)& keys,
		const cv::Size& size, int cellSize, size_t maxFeatures)
	{
		const uint32_t numKeys = keys.size();
		std::vector<uint32_t> selected;
		if (numKeys <= maxFeatures) {
			selected.resize(numKeys);
			std::iota(selected.begin(), selected.end(), 0u);
			return selected;
		}
		const int nx = MAXF(1, (size.width + cellSize / 2) / cellSize);
		const int ny = MAXF(1, (size.height + cellSize / 2) / cellSize);
		std::vector<std::vector<uint32_t>> cells(size_t(nx) * ny);
		for (uint32_t k = 0; k < numKeys; ++k) {
			const int x = MINF(MAXF((int)keys[k].x, 0), size.width - 1);
			const int y = MINF(MAXF((int)keys[k].y, 0), size.height - 1);
			cells[size_t(y * ny / size.height) * nx + x * nx / size.width].push_back(k);
		}
		// largest per-cell quota that fits the budget
		auto NumKept = [&cells](size_t quota) {
			size_t sum = 0;
			for (const std::vector<uint32_t>& cell : cells)
				sum += MINF(cell.size(), quota);
			return sum;
		};
		size_t lo = 0, hi = 0;
		for (const std::vector<uint32_t>& cell : cells)
			hi = MAXF(hi, cell.size());
		while (hi - lo > 1) {
			const size_t mid = (lo + hi) / 2;
			(NumKept(mid) <= maxFeatures ? lo : hi) = mid;
		}
		size_t extra = maxFeatures - NumKept(lo);
		selected.reserve(maxFeatures);
		for (std::vector<uint32_t>& cell : cells) {
			size_t quota = MINF(cell.size(), lo);
			if (cell.size() > lo && extra > 0) {
				++quota;
				--extra;
			}
			if (cell.size() > quota) {
				std::partial_sort(cell.begin(), cell.begin() + quota, cell.end(), [&keys](uint32_t a, uint32_t b) {
					return keys[a].s > keys[b].s || (keys[a].s == keys[b].s && a < b);
				});
				cell.resize(quota);
			}
			selected.insert(selected.end(), cell.begin(), cell.end());
		}
		std::sort(selected.begin(), selected.end());
		return selected;
	}

	// Pure-CPU post-processing: grid-based filtering + RootSIFT + store on Image.
	// Reads config from the bound extractor; safe to run on a worker thread
	// (bulk path) or on the calling thread (per-image path) as long as no two
	// invocations target the same Image concurrently.
	// Returns the number of features stored.
	// The image size is passed explicitly, as the bulk path releases the pixels
	// right after the upload to the GPU.
	size_t StoreFeatures(Image& img, const cv::Size& size,
		const CLISTDEF0IDX(SiftGPU::SiftKeypoint,uint32_t)& keys,
		const FloatArr& descs,
		bool skipIO = false)
	{
		const FeatureExtractionConfig& cfg = extractor.GetConfig();
		// Grid-based feature filtering
		const std::vector<uint32_t> selected(SelectFeatures(keys, size, gridCellSize, (size_t)cfg.GetMaxNumFeatures()));
		// Store keypoints and descriptors
		img.keypoints.resize(selected.size());
		img.descriptors.create((int)selected.size(), 128, CV_8U);
		FOREACH(k, selected) {
			const uint32_t idx = selected[k];
			const SiftGPU::SiftKeypoint& sk = keys[idx];
			img.keypoints[k] = cv::KeyPoint(sk.x, sk.y, sk.s, sk.o, 0.1f);
			// RootSIFT conversion
			cv::Mat siftRow(1, 128, CV_32F, const_cast<float*>(&descs[idx * 128]));
			FeaturesExtractor::ConvertToRootSIFT(siftRow).copyTo(img.descriptors.row((int)k));
		}
		const size_t numFeatures = img.keypoints.size();
		if (cfg.releaseImagePixels)
			img.ReleasePixels();
		DEBUG_ULTIMATE("Extracted features for image % 4u: % 6u features using %s (%.2f%s focal-length)",
			img.ID, numFeatures, FeatureTypeToString(cfg.detectorType).c_str(), img.pCamera->GetFocalLength(), img.TrustIntrinsics() ? "" : "*");
		if (!skipIO && !cfg.exportOpenMVGDir.empty())
			FeaturesExtractor::ExportFeaturesOpenMVG(cfg.exportOpenMVGDir, img);
		return numFeatures;
	}

	FeaturesExtractor& extractor;
	SiftGPU gpu;
	static std::unique_ptr<SiftGPUFeatureCoordinator> instance;
	// cell size (in pixels) of the adaptive feature selection grid
	static constexpr int gridCellSize = 256;
};

// Singleton instance storage
std::unique_ptr<SiftGPUFeatureCoordinator> SiftGPUFeatureCoordinator::instance;

} // namespace
#endif // _USE_SIFTGPU


FeaturesExtractor::FeaturesExtractor(Scene& _scene, const FeatureExtractionConfig& _config)
	: scene(_scene), config(_config)
{
}

FeaturesExtractor::~FeaturesExtractor() {
	#ifdef _USE_SIFTGPU
	SiftGPUFeatureCoordinator::Release();
	#endif // _USE_SIFTGPU
}


size_t FeaturesExtractor::Extract()
{
	// Per-thread feature extraction for efficient parallel processing
	cv::setNumThreads(1); // temporary turn off multi-threading for OpenCV functions
	Util::Progress progress(_T("Extract features from images"), scene.images.size());
	GET_LOGCONSOLE().Pause();
	size_t numFeatures = 0;
	#ifdef _USE_SIFTGPU
	if (config.detectorType == FeatureType::SIFTGPU) {
		// Lazily initialize the singleton GPU coordinator and run the bulk driver.
		SiftGPUFeatureCoordinator* coordinator = SiftGPUFeatureCoordinator::GetCoordinator(*this);
		if (!coordinator) {
			GET_LOGCONSOLE().Play();
			progress.close();
			return 0;
		}
		numFeatures = coordinator->ProcessImages(progress);
		SiftGPUFeatureCoordinator::Release(); // destroy singleton and release GPU context
	} else
	#endif // _USE_SIFTGPU
	{
		// CPU multi-threaded feature extraction with per-thread detectors

		// Give each pool worker its own detector. The vector is fixed before any task
		// starts, so workers only mutate distinct elements (unlike inserting into a
		// shared map from multiple threads).
		std::vector<cv::Ptr<cv::Feature2D>> detectors(scene.threadPool.get_thread_count());
		std::atomic<size_t> atomicNumFeatures{0};

		scene.threadPool.detach_loop(IIndex(0), scene.images.size(), [&](IIndex i) {
			Image& img = scene.images[i];
			if (img.HasFeatures() && (img.HasDescriptors() || scene.status.nFeaturesType != FeatureType::NONE)) {
				++progress;
				return;
			}
			const std::optional<size_t> threadIdx = BS::this_thread::get_index();
			ASSERT(threadIdx && *threadIdx < detectors.size());
			if (ExtractImage(img, detectors[*threadIdx]))
				atomicNumFeatures.fetch_add(img.keypoints.size(), std::memory_order_relaxed);
			++progress;
		});

		scene.threadPool.wait();
		numFeatures = atomicNumFeatures.load(std::memory_order_relaxed);
	}
	GET_LOGCONSOLE().Play();
	progress.close();
	cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading
	return numFeatures;
}

bool FeaturesExtractor::ExtractImage(Image& image, cv::Ptr<cv::Feature2D>& detector, bool skipIO)
{
	if (!skipIO && !config.importOpenMVGDir.empty() && ImportFeaturesOpenMVG(config.importOpenMVGDir, image)) {
		image.ReleasePixels(); // free pixel memory after feature extraction
		DEBUG_ULTIMATE("Imported features for image % 4u: % 6u features (%.2f focal-length)",
			image.ID, image.keypoints.size(), image.pCamera->GetFocalLength());
		return !image.keypoints.empty();
	}

	if (image.pCamera && image.pCamera->GetType() == CameraType::SPHERICAL)
		return ExtractImageSpherical(image, detector);

	#ifdef _USE_SIFTGPU
	if (config.detectorType == FeatureType::SIFTGPU) {
		// detector is unused on the GPU path; the SiftGPU context is owned by
		// the singleton coordinator in FeaturesExtractor.cpp.
		SiftGPUFeatureCoordinator* coordinator = SiftGPUFeatureCoordinator::GetCoordinator(*this);
		if (!coordinator)
			return false;
		return coordinator->ExtractImage(image, skipIO);
	}
	#endif

	if (!image.HasPixels() && !image.LoadPixels(true)) {
		VERBOSE("FeaturesExtractor::ExtractImage: no pixels loaded for image %u", image.ID);
		return false;
	}

	// Clear existing features
	image.keypoints.clear();
	image.descriptors.release();

	// Create the feature detector based on type
	if (!detector) {
		switch (config.detectorType) {
		case FeatureType::AKAZE:
			detector = cv::AKAZE::create();
			break;
		case FeatureType::ORB:
			detector = cv::ORB::create((int)config.maxFeaturesPerCell);
			break;
		case FeatureType::SIFT:
			detector = cv::SIFT::create();
			break;
		default:
			VERBOSE("FeaturesExtractor::ExtractImage: unknown detector type '%s'", FeatureTypeToString(config.detectorType).c_str());
			return false;
		}
	}

	// Divide image into 3x3 grid with overlapping borders
	const int cellWidth = image.pixels.cols / 3;
	const int cellHeight = image.pixels.rows / 3;
	const int borderSize = MINF(64, MINF(cellWidth, cellHeight) / 2); // overlap border size

	// Extract features from each cell
	std::vector<cv::Mat> vecDescriptors;
	image.keypoints.reserve(config.minFeaturesPerCell * 9);
	vecDescriptors.reserve(config.minFeaturesPerCell * 9);
	for (int row = 0; row < 3; ++row) {
		for (int col = 0; col < 3; ++col) {
			// Define cell region with overlapping borders
			const cv::Rect rcCell(
				col * cellWidth,
				row * cellHeight,
				col == 2 ? image.pixels.cols - col * cellWidth : cellWidth,
				row == 2 ? image.pixels.rows - row * cellHeight : cellHeight);
			// Extend cell bounds with border (clamped to image bounds)
			const cv::Rect rcCellExtended(
				col == 0 ? 0 : rcCell.x - borderSize,
				row == 0 ? 0 : rcCell.y - borderSize,
				col == 2 ? image.pixels.cols - rcCell.x + borderSize : rcCell.width + borderSize * 2,
				row == 2 ? image.pixels.rows - rcCell.y + borderSize : rcCell.height + borderSize * 2);
			// Initialize image for this cell as a ROI in the full image
			cv::Mat cellImage = image.pixels(rcCellExtended);

			// Extract features in this cell with iterative sensitivity adjustment;
			// OpenCV feature detectors (SIFT/ORB/AKAZE) report cv::KeyPoint::pt
			// already in pixel coordinates consistent with “integer = pixel center” convention
			std::vector<cv::KeyPoint> cellKeypoints;
			cv::Mat cellDescriptors;
			detector->detectAndCompute(cellImage, cv::noArray(), cellKeypoints, cellDescriptors);

			// Retry up to 5 times with progressively more sensitive settings if needed
			for (int retry = 0; retry < 5 && cellKeypoints.size() < (size_t)config.minFeaturesPerCell; ++retry) {
				cellKeypoints.clear();
				cellDescriptors.release();

				switch (config.detectorType) {
				case FeatureType::AKAZE: {
					cv::Ptr<cv::AKAZE> akaze = detector.dynamicCast<cv::AKAZE>();
					const double thresholds[] = {0.0005, 0.0001, 0.00005, 0.00001, 0.000001};
					akaze->setThreshold(thresholds[retry]);
					akaze->detectAndCompute(cellImage, cv::noArray(), cellKeypoints, cellDescriptors);
					akaze->setThreshold(0.001); // Reset to default
				} break;
				case FeatureType::ORB: {
					cv::Ptr<cv::ORB> orb = detector.dynamicCast<cv::ORB>();
					const int fastThresholds[] = {15, 10, 7, 5, 3};
					orb->setFastThreshold(fastThresholds[retry]);
					orb->detectAndCompute(cellImage, cv::noArray(), cellKeypoints, cellDescriptors);
					orb->setFastThreshold(20); // Reset to default
				} break;
				case FeatureType::SIFT: {
					cv::Ptr<cv::SIFT> sift = detector.dynamicCast<cv::SIFT>();
					const double contrastThresholds[] = {0.03, 0.02, 0.015, 0.01, 0.005};
					sift->setContrastThreshold(contrastThresholds[retry]);
					sift->detectAndCompute(cellImage, cv::noArray(), cellKeypoints, cellDescriptors);
					sift->setContrastThreshold(0.04); // Reset to default
				} break;
				}
			}

			// Build indices for features within the core cell (without border)
			// Also adjust keypoint coordinates to global image coordinates
			std::vector<int> selectedIndices;
			selectedIndices.reserve(cellKeypoints.size());
			for (size_t i = 0; i < cellKeypoints.size(); ++i) {
				cv::KeyPoint& kp = cellKeypoints[i];
				// Adjust to global coordinates
				kp.pt.x += rcCellExtended.x;
				kp.pt.y += rcCellExtended.y;
				// Check if within core cell
				if (rcCell.contains(kp.pt))
					selectedIndices.push_back(i);
			}

			// Limit features per cell if needed
			if (selectedIndices.size() > (size_t)config.maxFeaturesPerCell) {
				// Sort indices by keypoint response * size (descending)
				std::sort(selectedIndices.begin(), selectedIndices.end(),
					[&cellKeypoints](int a, int b) {
						return Image::ComputeKeypointWeight(cellKeypoints[a]) > Image::ComputeKeypointWeight(cellKeypoints[b]);
					});
				// Keep only the best
				selectedIndices.resize(config.maxFeaturesPerCell);
			}

			// Copy selected keypoints and descriptors to output arrays (only once)
			ASSERT(image.keypoints.size() == vecDescriptors.size());
			const size_t offset = image.keypoints.size();
			for (int idx : selectedIndices) {
				image.keypoints.push_back(cellKeypoints[idx]);
				if (!cellDescriptors.empty()) {
					if (config.detectorType == FeatureType::SIFT)
						vecDescriptors.push_back(ConvertToRootSIFT(cellDescriptors.row(idx)));
					else
						vecDescriptors.push_back(cellDescriptors.row(idx));
				}
			}

			// Normalize keypoint responses to linear scale
			// - SIFT: already linear (DoG value)
			// - AKAZE: quadratic (det(Hessian)) -> apply sqrt()
			// - ORB: quadratic by default (HARRIS_SCORE) -> apply sqrt()
			// This ensures responses scale linearly with image contrast for proper weighting
			auto NormalizeResponses = [](std::vector<cv::KeyPoint>& kps, size_t offset, FeatureType type) {
				if (type == FeatureType::AKAZE || type == FeatureType::ORB)
					for (size_t i = offset; i < kps.size(); ++i)
						kps[i].response = SQRT(MAXF(0.f, kps[i].response));
			};
			// Normalize responses after detection (whether initial or retry)
			NormalizeResponses(image.keypoints, offset, config.detectorType);
		}
	}
	cv::vconcat(vecDescriptors, image.descriptors);
	if (config.releaseImagePixels)
		image.ReleasePixels(); // free pixel memory after feature extraction

	DEBUG_ULTIMATE("Extracted features for image % 4u: % 6u features using %s (%.2f%s focal-length)",
	    image.ID, image.keypoints.size(), FeatureTypeToString(config.detectorType).c_str(), image.pCamera->GetFocalLength(), image.TrustIntrinsics() ? "" : "*");

	if (!skipIO && !config.exportOpenMVGDir.empty())
		ExportFeaturesOpenMVG(config.exportOpenMVGDir, image);
	return !image.keypoints.empty();
}

cv::Mat FeaturesExtractor::ConvertToRootSIFT(const cv::Mat& siftDesc)
{
	// RootSIFT: L1-normalize each descriptor, then sqrt, then quantize to uint8_t [0-255]
	// Input: CV_32F SIFT descriptors (each row is usually a 128-dim descriptor)
	// Output: CV_8U RootSIFT descriptors
	ASSERT(siftDesc.type() == CV_32F);
	cv::Mat rootsiftDesc(siftDesc.rows, siftDesc.cols, CV_8U);
	// Process each descriptor row individually
	for (int i = 0; i < siftDesc.rows; ++i) {
		cv::Mat normalized;
		// L1-normalize
		cv::normalize(siftDesc.row(i), normalized, 1.0, 0.0, cv::NORM_L1);
		// Square root
		cv::sqrt(normalized, normalized);
		// Scale to [0, 255] and quantize to uint8_t;
		// even though RootSIFT values are in [0,1] after sqrt normalization,
		// most are below 0.4, so for better precision they are quantized by scaling by 512
		normalized.convertTo(rootsiftDesc.row(i), CV_8U, 512.0);
	}
	return rootsiftDesc;
}
/*----------------------------------------------------------------*/


bool FeaturesExtractor::ExtractImageSpherical(Image& image, cv::Ptr<cv::Feature2D>& detector)
{
	// Spherical driver: render N tangent-pinhole faces via the unified
	// SphereCubeMap::SphericalToTangentialFaces entry point (same one MVS
	// export uses), then delegate per-face feature extraction to the existing
	// pinhole code path via ExtractImage(face, detector, /*skipIO=*/true).
	// No detector logic is duplicated here; this function only handles the
	// sphere↔face geometry, the per-face wrapping, and the angular-NMS dedup
	// across face seams.
	ASSERT(image.pCamera && image.pCamera->GetType() == CameraType::SPHERICAL);
	if (!image.HasPixels() && !image.LoadPixels(false)) {
		VERBOSE("FeaturesExtractor::ExtractImageSpherical: no pixels loaded for image %u", image.ID);
		return false;
	}

	// Face-set geometry from config, with sane defaults.
	const int numFaces = (config.cubemapFaces > 0 ? config.cubemapFaces : 6);
	const int faceSize = (config.cubemapFaceSize > 0
	                      ? config.cubemapFaceSize
	                      : MAXF(1024, image.pixels.cols / 4));

	// Split API: build geometry once (rotations + K), then render per-image.
	// Geometry is cheap and doesn't depend on pixels — MVS export shares the
	// same builder across all spherical images in a scene; we still rebuild
	// per-call here because ExtractImageSpherical runs one image at a time.
	const SphereCubeMap::TangentFacesGeometry geom =
		SphereCubeMap::MakeTangentFacesGeometry(numFaces, faceSize);
	if (geom.numFaces == 0) {
		VERBOSE("FeaturesExtractor::ExtractImageSpherical: unsupported numFaces=%d for image %u", numFaces, image.ID);
		return false;
	}
	const std::vector<Image8U3> faceImages =
		SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(image.GetImage8U3(), geom);

	const REAL f = geom.K(0,0), cx = geom.K(0,2), cy = geom.K(1,2);
	const SphericalCamera sphCam(image.pixels.size());

	// One shared PinholeCamera for every synthesized face Image — all faces
	// have identical intrinsics. The SPHERICAL-dispatch check at the top of
	// ExtractImage only fires for SphericalCamera, so wrapping faces in
	// a PinholeCamera guarantees the recursion terminates on the pinhole path.
	// CameraPtr is a raw Camera*; View::~View frees pCamera only when
	// cameraID == NO_ID, so we give each face a valid cameraID (0) and keep
	// ownership on a local unique_ptr that outlives every faceImage.
	std::unique_ptr<PinholeCamera> faceCamera = std::make_unique<PinholeCamera>(
		cv::Size(faceSize, faceSize), f, f, cx, cy);
	Camera* const pFaceCam = faceCamera.get();

	struct Entry { Point3 bearing; cv::KeyPoint kp; cv::Mat descRow; };
	std::vector<Entry> all;
	all.reserve(size_t(numFaces) * MAXF(1, config.GetMaxNumFeatures()));

	for (int k = 0; k < geom.numFaces; ++k) {
		// Convert face to grayscale — SIFTGPU's GL_LUMINANCE path requires
		// single-channel input; CPU detectors are unaffected either way.
		cv::Mat faceGray;
		cv::cvtColor(faceImages[k], faceGray, cv::COLOR_BGR2GRAY);

		// Synthesize a pinhole Image that satisfies ExtractImage's contract.
		// Pixels are already loaded so LoadPixels() is not re-invoked;
		// fileName is empty so the suppressed OpenMVG I/O is also a path no-op.
		// cameraID = 0 (not NO_ID) so View::~View doesn't attempt to free the
		// shared faceCamera when faceImage goes out of scope below.
		Image faceImage;
		faceImage.cameraID = 0;
		faceImage.pCamera  = pFaceCam;
		faceImage.pixels   = faceGray;
		faceImage.ID       = NO_ID;
		faceImage.fileName.clear();

		// Recursive call: every bit of detection logic (3x3 grid, retry,
		// RootSIFT, response normalization, SiftGPU dispatch) runs on this face
		// via the same code path a real pinhole image takes — no duplication.
		if (!ExtractImage(faceImage, detector, /*skipIO=*/true))
			continue;

		// Reproject face keypoints onto the equirectangular image.
		const Matrix3x3 R_face_T = geom.rotations[k].t();
		for (size_t i = 0; i < faceImage.keypoints.size(); ++i) {
			const cv::KeyPoint& fkp = faceImage.keypoints[i];
			const Point3 b_body = R_face_T * Point3((REAL(fkp.pt.x) - cx) / f,
			                                        (REAL(fkp.pt.y) - cy) / f,
			                                        REAL(1));
			const auto [eq_pt, ok] = sphCam.Project(b_body);
			if (!ok)
				continue;
			cv::KeyPoint eq_kp((float)eq_pt.x, (float)eq_pt.y,
			                   fkp.size, fkp.angle, fkp.response, k /*octave=faceID*/);
			all.push_back({normalized(b_body), eq_kp, faceImage.descriptors.row((int)i).clone()});
		}
		// faceImage/face go out of scope; their pixel buffers and descriptor
		// rows (cloned above) are released.
	}

	image.keypoints.clear();
	image.descriptors.release();
	if (all.empty()) {
		if (config.releaseImagePixels)
			image.ReleasePixels();
		return false;
	}

	// Angular-NMS across face seams. Sort by keypoint response (descending),
	// accept if the bearing is > cubemapDedupAngleDeg from all accepted.
	// A 3D octree over the unit-sphere bearings turns the per-candidate
	// neighborhood check into a chord-distance ball query; with SIFTGPU
	// producing tens of thousands of features per face the naive linear
	// scan over the growing kept list dominated runtime on 360 inputs.
	const REAL dedupCos = COS(D2R(REAL(config.cubemapDedupAngleDeg)));
	// chord²(θ) = 2(1 - cos θ) for unit vectors; query radius is the chord length.
	const REAL dedupChord = SQRT(MAXF(REAL(0), REAL(2) * (REAL(1) - dedupCos)));
	typedef CLISTDEF0(Point3::EVec) Bearings3;
	Bearings3 bearings(all.size());
	FOREACH(i, all)
		bearings[i] = all[i].bearing;
	typedef TOctree<Bearings3, REAL, 3> BearingOctree;
	BearingOctree octree(bearings,
		[dedupChord](BearingOctree::IDX_TYPE n, BearingOctree::Type r) {
			return n > 16 && r > dedupChord;
		});
	std::vector<size_t> order(all.size());
	std::iota(order.begin(), order.end(), size_t(0));
	std::sort(order.begin(), order.end(),
		[&](size_t a, size_t b) { return all[a].kp.response > all[b].kp.response; });
	std::vector<char> accepted(all.size(), 0);
	std::vector<size_t> keptIdx;
	keptIdx.reserve(all.size());
	for (size_t idx : order) {
		BearingOctree::IDXARR_TYPE neighbors;
		octree.Collect(neighbors, bearings[idx], dedupChord);
		bool dup = false;
		for (const BearingOctree::IDX_TYPE j : neighbors) {
			if ((size_t)j == idx || !accepted[j])
				continue;
			if (all[idx].bearing.dot(all[j].bearing) > dedupCos) {
				dup = true;
				break;
			}
		}
		if (!dup) {
			accepted[idx] = 1;
			keptIdx.push_back(idx);
		}
	}

	image.keypoints.reserve(keptIdx.size());
	std::vector<cv::Mat> descRows;
	descRows.reserve(keptIdx.size());
	for (size_t idx : keptIdx) {
		image.keypoints.push_back(all[idx].kp);
		descRows.push_back(all[idx].descRow);
	}
	cv::vconcat(descRows, image.descriptors);

	if (config.releaseImagePixels)
		image.ReleasePixels();

	DEBUG_ULTIMATE("Extracted features for image % 4u: % 6u features using %s cubemap (%d faces x %d px)",
		image.ID, image.keypoints.size(),
		FeatureTypeToString(config.detectorType).c_str(), numFaces, faceSize);

	if (!config.exportOpenMVGDir.empty())
		ExportFeaturesOpenMVG(config.exportOpenMVGDir, image);
	return !image.keypoints.empty();
}
/*----------------------------------------------------------------*/


bool FeaturesExtractor::ExportFeaturesOpenMVG(const String& outputDir, const Image& image)
{
	// Require keypoints; descriptors may be empty (exported count will be zero)
	if (image.keypoints.empty())
		return false;

	const String basePath = outputDir + PATH_SEPARATOR_STR + Util::getFileName(image.fileName);
	const String featPath = basePath + ".feat";
	const String descPath = basePath + ".desc";

	// Export keypoints (x y scale orientation)
	{
		std::ofstream file(featPath, std::ios::trunc);
		if (!file.is_open()) {
			VERBOSE("error: failed to open feature file: %s", featPath.c_str());
			return false;
		}
		for (const auto& kp : image.keypoints)
			file << kp.pt.x << ' ' << kp.pt.y << ' ' << kp.size << ' ' << kp.angle << '\n';
	}

	// Export descriptors as binary (size_t count + raw bytes)
	{
		std::ofstream file(descPath, std::ios::out | std::ios::binary | std::ios::trunc);
		if (!file.is_open()) {
			VERBOSE("error: failed to open descriptor file: %s", descPath.c_str());
			return false;
		}
		const size_t numDesc = (image.descriptors.type() == CV_8U) ? static_cast<size_t>(image.descriptors.rows) : 0;
		file.write(reinterpret_cast<const char*>(&numDesc), sizeof(size_t));
		if (numDesc > 0) {
			ASSERT(image.descriptors.cols > 0);
			const size_t rowBytes = static_cast<size_t>(image.descriptors.cols) * image.descriptors.elemSize();
			for (int r = 0; r < image.descriptors.rows; ++r)
				file.write(reinterpret_cast<const char*>(image.descriptors.ptr(r)), rowBytes);
		}
	}

	DEBUG_ULTIMATE("Image % 4u exported %zu OpenMVG features: %s, %s", image.ID, image.keypoints.size(), featPath.c_str(), descPath.c_str());
	return true;
}

bool FeaturesExtractor::ImportFeaturesOpenMVG(const String& inputDir, Image& image)
{
	const String basePath = inputDir + PATH_SEPARATOR_STR + Util::getFileName(image.fileName);
	const String featPath = basePath + ".feat";
	const String descPath = basePath + ".desc";

	image.keypoints.clear();
	image.descriptors.release();

	// Load keypoints (x y scale orientation)
	std::ifstream featFile(featPath);
	if (!featFile.is_open()) {
		VERBOSE("error: failed to open feature file: %s", featPath.c_str());
		return false;
	}
	double x = 0.0, y = 0.0, size = 0.0, angle = 0.0;
	while (featFile >> x >> y >> size >> angle)
		image.keypoints.emplace_back((float)x, (float)y, (float)size, (float)angle, 0.01f);
	if (image.keypoints.empty()) {
		VERBOSE("error: no keypoints read from: %s", featPath.c_str());
		return false;
	}

	// Load descriptors if available (expects the same binary layout as ExportFeaturesOpenMVG)
	std::ifstream descFile(descPath, std::ios::binary);
	if (descFile.is_open()) {
		descFile.seekg(0, std::ios::end);
		const std::streamoff fileSize = descFile.tellg();
		if (fileSize < (std::streamoff)sizeof(size_t)) {
			VERBOSE("error: descriptor file too small: %s", descPath.c_str());
			image.descriptors.release();
			return false;
		}
		descFile.seekg(0, std::ios::beg);
		size_t numDesc = 0;
		descFile.read(reinterpret_cast<char*>(&numDesc), sizeof(size_t));
		const std::streamoff dataBytes = fileSize - (std::streamoff)sizeof(size_t);
		if (numDesc == 0 || dataBytes == 0) {
			image.descriptors.release();
			DEBUG_LEVEL(3, "Image % 4u imported %zu OpenMVG features (no descriptors): %s",
				image.ID, image.keypoints.size(), featPath.c_str());
			return true;
		}
		const size_t rowBytes = static_cast<size_t>(dataBytes) / numDesc;
		if (rowBytes == 0 || rowBytes * numDesc != static_cast<size_t>(dataBytes)) {
			VERBOSE("error: descriptor file size mismatch: %s", descPath.c_str());
			image.descriptors.release();
			return false;
		}
		image.descriptors.create((int)numDesc, (int)rowBytes, CV_8U);
		for (size_t r = 0; r < numDesc; ++r) {
			descFile.read(reinterpret_cast<char*>(image.descriptors.ptr((int)r)), rowBytes);
			if (!descFile) {
				VERBOSE("error: failed to read descriptor row %zu from: %s", r, descPath.c_str());
				image.descriptors.release();
				return false;
			}
		}
		if (image.keypoints.size() != numDesc)
			VERBOSE("error: descriptor/keypoint count mismatch: %zu descriptors vs %zu keypoints", numDesc, image.keypoints.size());
		DEBUG_LEVEL(3, "Image % 4u imported %zu OpenMVG features and descriptors: %s, %s",
			image.ID, image.keypoints.size(), featPath.c_str(), descPath.c_str());
		return true;
	}

	DEBUG_LEVEL(3, "Image % 4u imported %zu OpenMVG features (descriptors missing): %s",
		image.ID, image.keypoints.size(), featPath.c_str());
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
