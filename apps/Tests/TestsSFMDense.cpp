/*
 * TestsSFMDense.cpp
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
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/SFM.h"
#include "../../libs/SFM/GlobalRotationAveraging.h"
#include "../../libs/SFM/GlobalScaleAveraging.h"
#include "../../libs/SFM/GlobalTranslationAveraging.h"
#include "../../libs/Math/DisjointSet.h"
#include "../../libs/SFM/RobustAveraging.h"
#include "../../libs/SFM/PairsWeighting.h"
#include "../../libs/SFM/ViewGraphCalibrator.h"
#include "../../libs/SFM/BundleAdjustment.h"
#include "../../libs/SFM/SceneCluster.h"
#include "../../libs/SFM/SceneBlocks.h"
#include "../../libs/SFM/GlobalAlignment.h"
#include "../../libs/SFM/PoseLink.h"
#include "../../libs/SFM/Link2D3D.h"
#include "../../libs/SFM/Triangulation.h"
#include "../../libs/SFM/MatchGeometric.h"
#include "../../libs/SFM/LensFit.h"
#include "../../libs/SFM/RoMa2Matcher.h"
#include "../../libs/SFM/FlexGlueMatcher.h"
#include "../../libs/SFM/SphereCubeMap.h"
#include "../../libs/SFM/InterfaceMVS.h"
#include "../../libs/MVS.h"
#include <nlohmann/json.hpp>
#include "Tests.h"
#include "TestsSFMFramework.h"
#include <filesystem>
#include <chrono>
#include <random>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

// Read exactly count fp32 values from a raw binary fixture, in the host byte order the
// fixtures are written in (the ROMA2 fixtures are stored headerless, so the always-on tests
// need no npy parser to read them); a file of any other size is a fixture/test disagreement
static bool ReadFloats(const String& fileName, size_t count, std::vector<float>& values)
{
	values.resize(count);
	const size_t expectedSize = count*sizeof(float);
	std::ifstream ifs(fileName.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
	if (!ifs.is_open() || (size_t)ifs.tellg() != expectedSize) {
		VERBOSE("error: fixture '%s' is not %u floats (%u bytes)",
			fileName.c_str(), (unsigned)count, (unsigned)expectedSize);
		return false;
	}
	ifs.seekg(0);
	if (!ifs.read((char*)values.data(), (std::streamsize)expectedSize)) {
		VERBOSE("error: cannot read %u floats from fixture '%s'", (unsigned)count, fileName.c_str());
		return false;
	}
	return true;
}

// Largest absolute difference between two vectors (FLT_MAX if their sizes disagree)
static float MaxAbsDiff(const std::vector<float>& a, const std::vector<float>& b)
{
	if (a.size() != b.size())
		return FLT_MAX;
	float maxDiff = 0.f;
	for (size_t i = 0; i < a.size(); ++i)
		maxDiff = MAXF(maxDiff, ABS(a[i] - b[i]));
	return maxDiff;
}

#ifdef _USE_ONNXRUNTIME
// Read a numpy .npy array of little-endian fp32 in C order (header versions 1.0 and 2.0),
// returning its values and its shape; every other layout (another dtype, another byte order,
// Fortran order, a newer header version) is rejected with a message rather than misread.
// Only the ROMA2 model reference dumps are stored this way -- the always-on fixtures are
// headerless and read by ReadFloats above.
static bool ReadNpy(const String& fileName, std::vector<float>& values, std::vector<int64_t>& shape)
{
	std::ifstream ifs(fileName.c_str(), std::ios::in | std::ios::binary);
	if (!ifs.is_open()) {
		VERBOSE("error: cannot open numpy file '%s'", fileName.c_str());
		return false;
	}
	char magic[8];
	if (!ifs.read(magic, 8) || memcmp(magic, "\x93NUMPY", 6) != 0 || (magic[6] != 1 && magic[6] != 2)) {
		VERBOSE("error: '%s' is not a numpy 1.0/2.0 array", fileName.c_str());
		return false;
	}
	// the header length is 2 bytes little-endian in version 1, 4 bytes in version 2
	uint8_t rawLen[4];
	const unsigned numLenBytes = (magic[6] == 1 ? 2u : 4u);
	if (!ifs.read((char*)rawLen, numLenBytes)) {
		VERBOSE("error: truncated numpy header in '%s'", fileName.c_str());
		return false;
	}
	uint32_t headerLen = 0;
	for (unsigned i = 0; i < numLenBytes; ++i)
		headerLen |= (uint32_t)rawLen[i] << (8*i);
	std::string header(headerLen, '\0');
	if (headerLen == 0 || !ifs.read(&header[0], headerLen)) {
		VERBOSE("error: truncated numpy header in '%s'", fileName.c_str());
		return false;
	}
	if (header.find("'descr': '<f4'") == std::string::npos || header.find("'fortran_order': False") == std::string::npos) {
		VERBOSE("error: numpy file '%s' is not little-endian fp32 in C order", fileName.c_str());
		return false;
	}
	const size_t keyPos = header.find("'shape':");
	const size_t begin = (keyPos == std::string::npos ? std::string::npos : header.find('(', keyPos));
	const size_t end = (begin == std::string::npos ? std::string::npos : header.find(')', begin));
	if (end == std::string::npos) {
		VERBOSE("error: numpy file '%s' has no shape tuple", fileName.c_str());
		return false;
	}
	shape.clear();
	size_t count = 1;
	for (size_t i = begin+1; i < end; ) {
		while (i < end && (header[i] < '0' || header[i] > '9'))
			++i;
		if (i == end)
			break;
		int64_t dim = 0;
		while (i < end && header[i] >= '0' && header[i] <= '9')
			dim = dim*10 + (header[i++] - '0');
		shape.push_back(dim);
		count *= (size_t)dim;
	}
	values.resize(count);
	if (count > 0 && !ifs.read((char*)values.data(), (std::streamsize)(count*sizeof(float)))) {
		VERBOSE("error: cannot read %u floats from numpy file '%s'", (unsigned)count, fileName.c_str());
		return false;
	}
	return true;
}

// Read a reference .npy expected to hold exactly count values; a different length is a
// disagreement between the test and the shipped reference dump
static bool ReadNpyExpect(const String& fileName, size_t count, std::vector<float>& values)
{
	std::vector<int64_t> shape;
	if (!ReadNpy(fileName, values, shape))
		return false;
	if (values.size() != count) {
		VERBOSE("error: numpy file '%s' holds %u values, expected %u",
			fileName.c_str(), (unsigned)values.size(), (unsigned)count);
		return false;
	}
	return true;
}

// Cosine similarity of two equally long float buffers, accumulated in double; 0 when either
// buffer has zero norm, which never passes a parity bound
static double CosineSimilarity(const float* a, const float* b, size_t count)
{
	double dot = 0, normA = 0, normB = 0;
	for (size_t i = 0; i < count; ++i) {
		dot += (double)a[i] * (double)b[i];
		normA += (double)a[i] * (double)a[i];
		normB += (double)b[i] * (double)b[i];
	}
	return (normA > 0 && normB > 0 ? dot/std::sqrt(normA*normB) : 0);
}

// Parity bounds shipped in a reference dump's parity.json; the defaults are the export
// script's, kept for a dump written before a bound was recorded
struct RoMa2ParityBounds {
	double minCosine = 0.998;
	double maxWarpErrorPx = 2.0;
	double minAgreementPercent = 99.5;
};

static bool ReadParityBounds(const String& fileName, RoMa2ParityBounds& bounds)
{
	std::ifstream stream(fileName.c_str());
	if (!stream.is_open()) {
		VERBOSE("error: cannot open parity file '%s'", fileName.c_str());
		return false;
	}
	const nlohmann::json data = nlohmann::json::parse(stream, nullptr, false);
	if (data.is_discarded()) {
		VERBOSE("error: cannot parse parity file '%s'", fileName.c_str());
		return false;
	}
	const auto itBounds = data.find("bounds");
	if (itBounds == data.end())
		return true; // the export script's defaults stand
	const auto ReadBound = [&itBounds](const char* key, double& value) {
		const auto it = itBounds->find(key);
		if (it != itBounds->end() && it->is_number())
			value = it->get<double>();
	};
	ReadBound("min_cosine", bounds.minCosine);
	ReadBound("max_warp_error_px", bounds.maxWarpErrorPx);
	ReadBound("min_agreement_percent", bounds.minAgreementPercent);
	return true;
}

// Linear-interpolated percentile of a sample set, as numpy.percentile computes it
// (the reference judging of polyml's check_correspondences); the values are sorted in place
static float Percentile(std::vector<float>& values, double percent)
{
	ASSERT(!values.empty() && percent >= 0 && percent <= 100);
	std::sort(values.begin(), values.end());
	const double pos = percent/100 * (double)(values.size()-1);
	const size_t lo = (size_t)pos;
	if (lo+1 >= values.size())
		return values.back();
	return (float)(values[lo] + (pos - (double)lo) * (values[lo+1] - values[lo]));
}
#endif // _USE_ONNXRUNTIME

// ROMA2 warp helpers test: keypoint tracking through an identity warp, the confidence gate
// that drops the cells the model is unsure about, and the dense append
bool DenseWarpTrackingTest()
{
	TD_TIMER_START();

	// two 640x480 images sharing a camera; an identity warp on a 160x160 grid maps A onto B
	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
	scene.images.emplace_back((IIndex)0, String("a.jpg"));
	scene.images.emplace_back((IIndex)1, String("b.jpg"));
	FOREACH(i, scene.images) {
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
	Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const int cells = 160;
	Image32F2 warp(cells, cells);
	Image32F overlap(cv::Size(cells, cells), 1.f);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x) // cell (x,y) -> the pixel at its centre, normalized (both align_corners=false)
			warp(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), cv::Size(width, height)), cv::Size(width, height));
	std::mt19937 rng(7);
	std::uniform_real_distribution<float> ux(8.f, 631.f), uy(8.f, 471.f);
	for (unsigned i = 0; i < 500; ++i)
		imgA.keypoints.emplace_back(ux(rng), uy(rng), 1.f);
	std::vector<Point2f> trackedA, trackedB;
	std::vector<uchar> status;
	if (TrackKeypointsByWarp(imgA, imgB, warp, overlap, 0.3f, trackedA, trackedB, status) != imgA.keypoints.size()) {
		VERBOSE("DenseWarpTrackingTest FAILED: not every keypoint tracked through the identity warp");
		return false;
	}
	FOREACH(i, trackedA)
		if (!status[i] || norm(trackedB[i] - trackedA[i]) > 1e-3f) {
			VERBOSE("DenseWarpTrackingTest FAILED: keypoint %u moved %g px", i, norm(trackedB[i] - trackedA[i]));
			return false;
		}

	// overlap gating: zero the left half of the grid -> keypoints left of the (bilinear) band are dropped
	overlap.colRange(0, cells/2).setTo(0.f);
	TrackKeypointsByWarp(imgA, imgB, warp, overlap, 0.3f, trackedA, trackedB, status);
	FOREACH(i, status) {
		const float x = imgA.keypoints[i].pt.x;
		if ((x < 316.f && status[i]) || (x > 324.f && !status[i])) {
			VERBOSE("DenseWarpTrackingTest FAILED: overlap gate at x=%g", x);
			return false;
		}
	}

	// the dense fill (AppendDenseMatches): the appended keypoints land past each image's described
	// prefix, while the appended matches become the pair's own middle segment -- after the sparse
	// inliers, which stay the pair's descriptor evidence, and before the strict filter's rejects,
	// so the fill is inside the track-forming prefix BuildTracks reads but outside the count every
	// view-graph weight and gate reads
	{
		Scene denseScene;
		denseScene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
			REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
		for (IIndex k = 0; k < 2; ++k) {
			Image& im = denseScene.images.emplace_back(k, String::FormatString("%u.jpg", k));
			im.cameraID = 0;
			im.pCamera = denseScene.cameras[0];
			im.keypoints.resize(10, cv::KeyPoint(1.f, 1.f, 3.f));
		}
		ImagePair& pair = denseScene.pairs.emplace_back(0u, 1u);
		for (uint32_t k = 0; k < 8; ++k)
			pair.matches.emplace_back(k, k);
		pair.numFilteredInliers = 5; // matches 5..7 are inliers the strict filter then rejected
		const std::vector<Point2f> ptsA{Point2f(100.f, 100.f), Point2f(200.f, 200.f)};
		const std::vector<Point2f> ptsB{Point2f(110.f, 100.f), Point2f(210.f, 200.f)};
		const std::vector<float> confidences{0.7f, 0.9f};
		if (AppendDenseMatches(denseScene, pair, ptsA, ptsB, confidences, cv::Size(cells, cells)) != 2) {
			VERBOSE("DenseWarpTrackingTest FAILED: dense fill not appended");
			return false;
		}
		const float cellSize = MAXF((float)width/(float)cells, (float)height/(float)cells);
		for (IIndex k = 0; k < 2; ++k) {
			const Image& im = denseScene.images[k];
			if (im.keypoints.size() != 12 || im.NumDescribedKeypoints() != 10 || im.NumDenseKeypoints() != 2 ||
				im.IsDenseKeypoint(9) || !im.IsDenseKeypoint(10)) {
				VERBOSE("DenseWarpTrackingTest FAILED: image %u has %u keypoints with boundary %u",
					k, (unsigned)im.keypoints.size(), im.NumDescribedKeypoints());
				return false;
			}
			if (im.keypoints[10].response != 0.7f || im.keypoints[11].response != 0.9f ||
				im.keypoints[10].size != cellSize) {
				VERBOSE("DenseWarpTrackingTest FAILED: image %u dense keypoint response %g / size %g, expected size %g",
					k, im.keypoints[10].response, im.keypoints[10].size, cellSize);
				return false;
			}
		}
		if (denseScene.images[0].keypoints[10].pt != cv::Point2f(100.f, 100.f) ||
			denseScene.images[1].keypoints[10].pt != cv::Point2f(110.f, 100.f)) {
			VERBOSE("DenseWarpTrackingTest FAILED: dense keypoint positions");
			return false;
		}
		// the two dense matches sit at 5 and 6, the strict-filter rejects moved after them, the
		// sparse count is untouched, and the dense count grew by exactly the supplement
		if (pair.matches.size() != 10 || pair.numFilteredInliers != 5 || pair.numDenseInliers != 2 ||
			pair.GetNumFilteredInliers() != 5 || pair.GetNumTrackFormingMatches() != 7 ||
			pair.matches[5].queryIdx != 10 || pair.matches[5].trainIdx != 10 ||
			pair.matches[6].queryIdx != 11 || pair.matches[7].queryIdx != 5) {
			VERBOSE("DenseWarpTrackingTest FAILED: dense matches not inserted as their own segment (%u matches, %d sparse, %d dense)",
				(unsigned)pair.matches.size(), pair.numFilteredInliers, pair.numDenseInliers);
			return false;
		}
		// a second dense fill on the same images appends past the first one's dense keypoints,
		// and its matches extend the dense segment rather than reopening the sparse one
		if (AppendDenseMatches(denseScene, pair, ptsA, ptsB, confidences, cv::Size(cells, cells)) != 2 ||
			denseScene.images[0].NumDescribedKeypoints() != 10 || denseScene.images[0].NumDenseKeypoints() != 4 ||
			pair.numFilteredInliers != 5 || pair.numDenseInliers != 4 ||
			pair.matches.size() != 12 || pair.matches[7].queryIdx != 12 || pair.matches[9].queryIdx != 5) {
			VERBOSE("DenseWarpTrackingTest FAILED: a second append moved the described boundary or the sparse count");
			return false;
		}
	}

	VERBOSE("DenseWarpTrackingTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Coverage-uniform warp sampling (the verdict's fitting sample): the budget, the spread the
// bucket stratification buys over a plain top-confidence selection, the frame occupancy of a
// sample that really does sit in one corner, and the determinism of the whole draw
bool DenseCoverageSampleTest()
{
	TD_TIMER_START();

	// two 640x480 images and the identity warp of DenseWarpTrackingTest, so every sampled point of
	// A must come back unmoved in B and the two coverages have to agree
	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
	scene.images.emplace_back((IIndex)0, String("a.jpg"));
	scene.images.emplace_back((IIndex)1, String("b.jpg"));
	FOREACH(i, scene.images) {
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const int cells = 160;
	Image32F2 warp(cells, cells);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x)
			warp(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), cv::Size(width, height)), cv::Size(width, height));
	const unsigned budget = 2000;
	std::vector<Point2f> sampledA, sampledB;

	// what fraction of a coarse 16x16 grid over an image a sample occupies -- the occupancy this
	// test measures the draw's spread by. Computed here rather than reported by the sampler: the
	// draw's business is the sample, and the claim under test is that a draw uniform over the whole
	// frame reads as the overlap it came from, so a corner-overlap pair must read as a corner
	constexpr unsigned coverageGrid = 16;
	const auto SampleCoverage = [](const std::vector<Point2f>& sampled, const cv::Size& size) {
		std::vector<bool> grid((size_t)coverageGrid*coverageGrid, false);
		for (const Point2f& pt : sampled) {
			const unsigned cx = MINF((unsigned)MAXF(0.f, (float)coverageGrid*pt.x/(float)size.width), coverageGrid-1);
			const unsigned cy = MINF((unsigned)MAXF(0.f, (float)coverageGrid*pt.y/(float)size.height), coverageGrid-1);
			grid[(size_t)cy*coverageGrid + cx] = true;
		}
		return (float)std::count(grid.begin(), grid.end(), true)/(float)grid.size();
	};

	// full overlap: every cell eligible, so the bucket grid is ceil(sqrt(budget)) = 45 on a side and
	// the sample lands at the budget, spread over the whole frame
	Image32F overlap(cv::Size(cells, cells), 0.6f);
	const size_t numFull = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB);
	if (numFull < budget*9/10 || numFull > budget*11/10) {
		VERBOSE("DenseCoverageSampleTest FAILED: full overlap drew %u samples, expected about the %u budgeted",
			(unsigned)numFull, budget);
		return false;
	}
	float coverageA = SampleCoverage(sampledA, imgA.GetSize()), coverageB = SampleCoverage(sampledB, imgB.GetSize());
	if (coverageA < 0.99f || coverageB < 0.99f) {
		VERBOSE("DenseCoverageSampleTest FAILED: a fully overlapping sample covers only %.3f/%.3f of the two images", coverageA, coverageB);
		return false;
	}
	FOREACH(i, sampledA)
		if (!Image8U::isInside(sampledB[i], imgB.GetSize()) || norm(sampledB[i] - sampledA[i]) > 1e-3f) {
			VERBOSE("DenseCoverageSampleTest FAILED: sample %u moved %g px through the identity warp", i, norm(sampledB[i] - sampledA[i]));
			return false;
		}

	// PARTIAL OVERLAP, the case the over-binning exists for: a 100x100-cell region of a 160x160 grid
	// is eligible, E/T = 0.39, so the bucket grid has to grow to ceil(sqrt(2000/0.39)) = 72 a side and
	// the occupied buckets must still number about the budget. The old fixed-45 grid plus a
	// descending-confidence fill-up would have hit the budget exactly while cramming ~1200 of those
	// points into the eligible region; what must NOT happen now is a sample at the budget drawn from
	// a fraction of the frame.
	overlap.setTo(0.f);
	overlap(cv::Rect(2, 2, 100, 100)).setTo(0.6f);
	const size_t numPartial = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB);
	if (numPartial < budget*3/4 || numPartial > budget*5/4) {
		VERBOSE("DenseCoverageSampleTest FAILED: a 39%%-overlap warp drew %u samples, expected near the %u budgeted",
			(unsigned)numPartial, budget);
		return false;
	}
	// and the sample must occupy the overlap it actually came from, not the whole frame: the eligible
	// region spans about 100/160 of each image side, so about (0.63)^2 = 0.4 of the coverage grid
	coverageA = SampleCoverage(sampledA, imgA.GetSize());
	coverageB = SampleCoverage(sampledB, imgB.GetSize());
	if (coverageA < 0.3f || coverageA > 0.5f || ABS(coverageB-coverageA) > 0.02f) {
		VERBOSE("DenseCoverageSampleTest FAILED: a 39%%-overlap sample reports %.3f/%.3f coverage, expected about 0.4",
			coverageA, coverageB);
		return false;
	}
	// pin the documented "one winner per bucket, and nothing else" contract itself: the occupancy
	// checks above run on a coarse 16-per-side grid and cannot tell 800 points
	// spread over the eligible region from 2000 points piled into its most confident corner --
	// both occupy the same ~0.4 of that grid. A lambda since the confidence hot-spot sub-case
	// below has to pass the identical pin.
	const auto CheckOneWinnerPerBucket = [cells, width, height](const std::vector<Point2f>& sampled, int numBuckets, const char* label) -> bool {
		// map each A-point back to the warp-grid cell it came from, then to its
		// (numBuckets x numBuckets) bucket, and require no two samples to share one: a fill-up
		// necessarily puts extra points into buckets that already have a winner, which this catches
		// directly. The mapping is PixelToCell (DenseWarp.cpp), the exact inverse of the CellToPixel
		// the sampler places a cell in image coordinates with -- any other map here would bias every
		// index and would only stay invisible while the bias sat under the rounding threshold
		std::vector<int> bucketHits((size_t)numBuckets*numBuckets, 0);
		FOREACH(i, sampled) {
			const Point2f cell(PixelToCell(sampled[i], cv::Size(width, height), cv::Size(cells, cells)));
			const int cx = ROUND2INT(cell.x);
			const int cy = ROUND2INT(cell.y);
			const int bx = MINF(cx*numBuckets/cells, numBuckets-1);
			const int by = MINF(cy*numBuckets/cells, numBuckets-1);
			++bucketHits[(size_t)by*numBuckets + bx];
		}
		FOREACH(b, bucketHits)
			if (bucketHits[b] > 1) {
				VERBOSE("DenseCoverageSampleTest FAILED: %s bucket %u holds %d samples, expected at most 1 -- a fill-up puts extra points into buckets that already have a winner",
					label, (unsigned)b, bucketHits[b]);
				return false;
			}
		// backstop that does not depend on the formula: bin on a grid fine enough to resolve the
		// eligible region and check the busiest bin is not piled up relative to the rest -- a draw
		// crammed into a fraction of the region blows past this, a one-per-bucket draw sits near 1x
		constexpr int fineGrid = 32;
		std::vector<int> fineHits((size_t)fineGrid*fineGrid, 0);
		FOREACH(i, sampled) {
			const int fx = MINF((int)((float)fineGrid*sampled[i].x/(float)width), fineGrid-1);
			const int fy = MINF((int)((float)fineGrid*sampled[i].y/(float)height), fineGrid-1);
			++fineHits[(size_t)fy*fineGrid + fx];
		}
		int maxHits = 0, sumHits = 0, numNonEmpty = 0;
		FOREACH(b, fineHits) {
			if (fineHits[b] == 0)
				continue;
			++numNonEmpty;
			sumHits += fineHits[b];
			maxHits = MAXF(maxHits, fineHits[b]);
		}
		const float meanHits = (float)sumHits/(float)numNonEmpty;
		if ((float)maxHits > 3.f*meanHits) {
			VERBOSE("DenseCoverageSampleTest FAILED: %s busiest %dx%d bin holds %d samples, %.2fx the %.2f mean over the %d non-empty bins, expected at most 3x",
				label, fineGrid, fineGrid, maxHits, (float)maxHits/meanHits, meanHits, numNonEmpty);
			return false;
		}
		return true;
	};
	// n from the documented formula, E = 100*100 eligible cells of T = cells*cells, capped at the
	// warp side (160, nowhere near binding here)
	const int E = 100*100, T = cells*cells;
	const int numBuckets = MINF((int)std::ceil(std::sqrt((double)budget*T/E)), cells);
	if (!CheckOneWinnerPerBucket(sampledA, numBuckets, "partial-overlap"))
		return false;
	// the draw must not be piled onto the most confident part of that region -- and under the shared
	// lattice winner rule (DenseWarp.cpp, WarpCellLatticePriority) confidence does not rank the
	// candidates at all, only admit them, so raising a 20x20-cell corner of an ALREADY-ELIGIBLE
	// region to full confidence must not move the draw by a single point. Pinned as exact equality,
	// not as a tolerance: a tolerance would pass whatever the rule does.
	const std::vector<Point2f> partialA(sampledA), partialB(sampledB);
	overlap(cv::Rect(2, 2, 20, 20)).setTo(1.f);
	std::vector<Point2f> skewA, skewB;
	const size_t numSkew = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, skewA, skewB);
	const float skewCoverageA = SampleCoverage(skewA, imgA.GetSize()), skewCoverageB = SampleCoverage(skewB, imgB.GetSize());
	if (numSkew != numPartial || skewA != partialA || skewB != partialB ||
		skewCoverageA != coverageA || skewCoverageB != coverageB) {
		VERBOSE("DenseCoverageSampleTest FAILED: a confidence hot-spot moved the draw from %u samples at %.3f coverage to %u at %.3f "
			"-- confidence is ranking the bucket winners, not just admitting them",
			(unsigned)numPartial, coverageA, (unsigned)numSkew, skewCoverageA);
		return false;
	}
	// the hot-spot's E and T are unchanged -- raising confidence inside an already-eligible region
	// adds no new eligible cell -- so the same bucket grid applies; this sub-case is where the old
	// fill-up's bias was strongest, so it is where the pin matters most
	if (!CheckOneWinnerPerBucket(skewA, numBuckets, "confidence-hot-spot"))
		return false;

	// the same draw twice in one process: this proves the function is pure -- it carries no state
	// between calls and reads no container whose iteration order could vary -- which is what the
	// implementation has to guarantee. It is *not* a run-to-run reproducibility claim across
	// binaries or machines; the sample is also a function of the warp the model produced, and
	// nothing here exercises that. Re-drawing from a pre-filled output pair as well, since a
	// caller reusing its buffers must get the same answer as one passing empty ones.
	std::vector<Point2f> repeatA(7, Point2f(1.f, 2.f)), repeatB(3, Point2f(3.f, 4.f));
	SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, repeatA, repeatB);
	if (repeatA != skewA || repeatB != skewB ||
		SampleCoverage(repeatA, imgA.GetSize()) != skewCoverageA || SampleCoverage(repeatB, imgB.GetSize()) != skewCoverageB) {
		VERBOSE("DenseCoverageSampleTest FAILED: the draw is not a pure function of its inputs");
		return false;
	}

	// budget boundary: the smallest sample the verdict fits on is 8, the estimator's own
	// minimum. maxSamples is a target rather than a cap here (there is no fill-up to trim against),
	// and at a budget this small the bucket grid is only a few cells on a side, so quantisation
	// dominates: what must hold is that the draw stays small, stays non-empty, and terminates
	const size_t numTiny = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, 8, sampledA, sampledB);
	if (numTiny == 0 || numTiny > 64 || sampledA.size() != numTiny) {
		VERBOSE("DenseCoverageSampleTest FAILED: a budget of 8 drew %u samples", (unsigned)numTiny);
		return false;
	}
	// and a budget of 1 degenerates to a 2x2 bucket grid over the whole warp (n is still scaled by
	// the inverse overlap fraction), not to a division by zero or an empty draw
	const size_t numOne = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, 1, sampledA, sampledB);
	if (numOne == 0 || numOne > 8) {
		VERBOSE("DenseCoverageSampleTest FAILED: a budget of 1 drew %u samples", (unsigned)numOne);
		return false;
	}

	// a warp confident in one corner only (kept off the very border, where the round trip through
	// the normalized warp coordinates can put a cell a hundredth of a pixel outside the second
	// image and TrackKeypointsByWarp's own inside test drops it): E <= budget, so the whole confident
	// overlap is taken unstratified, the sample is SMALLER than the budget, and its occupancy has to
	// show that it really does sit in that corner
	overlap.setTo(0.f);
	overlap(cv::Rect(2, 2, 40, 40)).setTo(1.f);
	if (SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB) != 40*40) {
		VERBOSE("DenseCoverageSampleTest FAILED: %u samples drawn of the 1600 confident cells", (unsigned)sampledA.size());
		return false;
	}
	coverageA = SampleCoverage(sampledA, imgA.GetSize());
	coverageB = SampleCoverage(sampledB, imgB.GetSize());
	if (coverageA > 0.15f || coverageA < 0.05f || ABS(coverageB-coverageA) > 0.01f) {
		VERBOSE("DenseCoverageSampleTest FAILED: a corner-only sample covers %.3f/%.3f of the two images", coverageA, coverageB);
		return false;
	}

	// a warp confident nowhere: no sample, no occupancy, and no estimator ever sees the pair
	overlap.setTo(0.f);
	if (SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB) != 0 ||
		!sampledA.empty() || SampleCoverage(sampledA, imgA.GetSize()) != 0.f || SampleCoverage(sampledB, imgB.GetSize()) != 0.f) {
		VERBOSE("DenseCoverageSampleTest FAILED: an unconfident warp still produced a sample");
		return false;
	}

	VERBOSE("DenseCoverageSampleTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The complementary dense draw (SampleWarpComplementary), the fill of an admitted pair: it has to
// cover the parts of the confident overlap the pair's guided sparse matches left EMPTY, cap itself
// at the pair's dense budget, and stay spread when that cap bites. The sub-checks are the three
// ways a draw of this kind goes wrong: drawing blind to the sparse matches (so a pair whose matches
// cluster in one textured region gets dense points poured back into that same region), ranking the
// bucket winners by confidence (which breaks the cross-pair keypoint identity the dense tracks rest
// on), and thinning an over-budget draw by confidence, which re-clusters the survivors onto exactly
// the well-textured part the sparse matcher already covered.
bool DenseComplementaryDrawTest()
{
	TD_TIMER_START();

	// the identity-warp two-image setup of DenseCoverageSampleTest: a drawn point of A comes back
	// unmoved in B, so every check below can be read in A's frame alone
	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
	scene.images.emplace_back((IIndex)0, String("a.jpg"));
	scene.images.emplace_back((IIndex)1, String("b.jpg"));
	FOREACH(i, scene.images) {
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const int cells = 160;
	Image32F2 warp(cells, cells);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x)
			warp(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), cv::Size(width, height)), cv::Size(width, height));
	// the cell <-> pixel map the draw places its winners with and its occupancy test inverts
	// (CellToPixel / PixelToCell, the cell-centre convention), and the documented bucket-grid formula
	// -- replayed here so the checks speak the implementation's own coordinates rather than an
	// approximation
	const auto CellPixel = [&](int cx, int cy) {
		return CellToPixel(Point2f((float)cx, (float)cy), cv::Size(cells, cells), cv::Size(width, height));
	};
	const auto PixelToBucket = [&](const Point2f& pt, int numBuckets) {
		const Point2f cell(PixelToCell(pt, cv::Size(width, height), cv::Size(cells, cells)));
		const int cx = MINF(MAXF(ROUND2INT(cell.x), 0), cells-1);
		const int cy = MINF(MAXF(ROUND2INT(cell.y), 0), cells-1);
		return (size_t)(cy*numBuckets/cells)*numBuckets + cx*numBuckets/cells;
	};
	// eligibility restated independently of the draw: a cell the confidence map admits whose warped
	// point lands inside B, and which buckets hold at least one -- because a border cell can round a
	// hundredth of a pixel outside B and drop out, and the expected winner count has to account for
	// that rather than assume a full grid
	const auto MarkCandidateBuckets = [&](const Image32F& conf, int numBuckets, std::vector<bool>& hasCandidate) {
		hasCandidate.assign((size_t)numBuckets*numBuckets, false);
		for (int y = 0; y < cells; ++y)
			for (int x = 0; x < cells; ++x)
				if (conf(y, x) >= 0.3f && Image8U::isInside(DenormCoord(warp(y, x), imgB.GetSize()), imgB.GetSize()))
					hasCandidate[(size_t)(y*numBuckets/cells)*numBuckets + x*numBuckets/cells] = true;
	};
	// which quadrant of the frame a drawn point sits in: the coarsest honest statement of "spread"
	const auto Quadrant = [&](const Point2f& pt) {
		return ((int)pt.y < height/2 ? 0 : 2) + ((int)pt.x < width/2 ? 0 : 1);
	};

	// 1) COMPLEMENTARITY on a fully confident warp. The pair's guided sparse matches cluster in the
	// top-left corner -- the one textured region descriptors could agree on -- so the fill must draw
	// over the rest of the frame and nowhere in there.
	std::vector<Point2f> sparseA;
	for (int cy = 0; cy < 40; cy += 2)
		for (int cx = 0; cx < 40; cx += 2)
			sparseA.push_back(CellPixel(cx, cy));
	const DenseMatchConfig config; // denseMatchesPerFrame 2000 (a density per full frame of overlap), minConfidence 0.1
	const unsigned denseBudget = config.denseMatchesPerFrame;
	const int numBuckets = DenseFillGridSide(denseBudget, cells);
	Image32F overlap(cv::Size(cells, cells), 0.6f);
	std::vector<Point2f> denseA, denseB;
	std::vector<float> confidences;
	const size_t numDense = SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numBuckets, denseBudget,
		sparseA, denseA, denseB, confidences);
	if (numDense == 0 || denseB.size() != numDense || confidences.size() != numDense) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the draw returned %u points with %u/%u index-parallel arrays",
			(unsigned)numDense, (unsigned)denseB.size(), (unsigned)confidences.size());
		return false;
	}
	std::vector<bool> occupied((size_t)numBuckets*numBuckets, false);
	for (const Point2f& pt : sparseA)
		occupied[PixelToBucket(pt, numBuckets)] = true;
	FOREACH(i, denseA)
		if (occupied[PixelToBucket(denseA[i], numBuckets)]) {
			VERBOSE("DenseComplementaryDrawTest FAILED: dense point %u at (%.1f, %.1f) landed in a bucket the pair's sparse matches already hold",
				i, denseA[i].x, denseA[i].y);
			return false;
		}
	// and the converse of "nothing in an occupied bucket": one winner in every OTHER bucket that
	// has a candidate at all, so the draw really did take the whole complement rather than a corner
	// of it. An occupied bucket contributes nothing, not a reduced quota.
	std::vector<bool> hasCandidate;
	MarkCandidateBuckets(overlap, numBuckets, hasCandidate);
	size_t numExpected = 0;
	FOREACH(b, hasCandidate)
		if (hasCandidate[b] && !occupied[b])
			++numExpected;
	numExpected = MINF(numExpected, (size_t)denseBudget); // the thinning's cap, if the winners ran over it
	if (numDense != numExpected) {
		VERBOSE("DenseComplementaryDrawTest FAILED: %u dense points for the %u unoccupied buckets of the %dx%d grid that hold a candidate",
			(unsigned)numDense, (unsigned)numExpected, numBuckets, numBuckets);
		return false;
	}
	// the budget is a real CAP here, not merely the target it is for the verdict's draw
	if (numDense > denseBudget) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the draw kept %u points of a %u budget",
			(unsigned)numDense, denseBudget);
		return false;
	}
	FOREACH(i, denseA) {
		// index-parallel through the identity warp, and each point carries its own cell's
		// confidence -- the value it was selected on, which is what MakeDenseKeypoint stamps
		if (norm(denseB[i] - denseA[i]) > 1e-3f || ABS(confidences[i] - 0.6f) > 1e-6f) {
			VERBOSE("DenseComplementaryDrawTest FAILED: dense point %u moved %g px and carries confidence %g, expected 0.6",
				i, norm(denseB[i] - denseA[i]), confidences[i]);
			return false;
		}
	}
	// warp-grid raster order, which is what makes the keypoint indices AppendDenseMatches hands out
	// reproducible from one run to the next
	for (size_t i = 1; i < numDense; ++i)
		if (denseA[i].y < denseA[i-1].y - 1e-3f ||
			(ABS(denseA[i].y - denseA[i-1].y) <= 1e-3f && denseA[i].x <= denseA[i-1].x)) {
			VERBOSE("DenseComplementaryDrawTest FAILED: point %u at (%.1f, %.1f) breaks the raster order after (%.1f, %.1f)",
				(unsigned)i, denseA[i].x, denseA[i].y, denseA[i-1].x, denseA[i-1].y);
			return false;
		}

	// 2) A DRAW ASKED FOR NOTHING PRODUCES NOTHING, output buffers included: a pair whose whole
	// budget went to its guided matches gets no dense segment rather than a stale one
	std::vector<Point2f> zeroA(3, Point2f(1.f, 2.f)), zeroB(5, Point2f(3.f, 4.f));
	std::vector<float> zeroC(7, 0.5f);
	if (SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numBuckets, 0, sparseA, zeroA, zeroB, zeroC) != 0 ||
		!zeroA.empty() || !zeroB.empty() || !zeroC.empty()) {
		VERBOSE("DenseComplementaryDrawTest FAILED: a zero dense budget still drew %u points", (unsigned)zeroA.size());
		return false;
	}

	// 3) OVER BUDGET, WHERE THE THINNING SHOWS. A pitch far finer than the budget needs (cells/2,
	// against a budget of 400) drives the winner count far past the budget on its own, without
	// needing to scatter the eligible cells thin -- the fixed pitch does not care how sparse the
	// overlap is -- and the confidence grows steadily toward the bottom-right corner. Thinning by
	// confidence -- what the first version did -- keeps only that corner; thinning by the lattice
	// key has to leave every quadrant hit, since the levels it keeps whole are lattices over the
	// whole frame and the scramble that breaks the level it stops inside is spatially unbiased.
	overlap.setTo(0.f);
	for (int y = 0; y < cells; y += 2)
		for (int x = 0; x < cells; x += 2)
			overlap(y, x) = 0.4f + 0.5f*(float)(x + y)/(float)(2*cells);
	const unsigned smallBudget = 400;
	const int numSmallBuckets = cells/2;
	const size_t numSmall = SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numSmallBuckets, smallBudget,
		sparseA, denseA, denseB, confidences);
	if (numSmall != smallBudget || denseB.size() != numSmall || confidences.size() != numSmall) {
		VERBOSE("DenseComplementaryDrawTest FAILED: an over-budget draw kept %u points of a %u budget",
			(unsigned)numSmall, smallBudget);
		return false;
	}
	std::vector<bool> smallOccupied((size_t)numSmallBuckets*numSmallBuckets, false);
	for (const Point2f& pt : sparseA)
		smallOccupied[PixelToBucket(pt, numSmallBuckets)] = true;
	unsigned perQuadrant[4] = {0u, 0u, 0u, 0u};
	FOREACH(i, denseA) {
		// complementarity has to survive the thinning too: it can only drop points, never move one
		// into a bucket the draw had struck out
		if (smallOccupied[PixelToBucket(denseA[i], numSmallBuckets)]) {
			VERBOSE("DenseComplementaryDrawTest FAILED: the thinned draw put point %u at (%.1f, %.1f) into a sparse-occupied bucket",
				i, denseA[i].x, denseA[i].y);
			return false;
		}
		++perQuadrant[Quadrant(denseA[i])];
	}
	// the sparse cluster only bites into the first quadrant, so all four must keep a real share; a
	// draw thinned by confidence collapses into the last one and empties the first outright
	for (int q = 0; q < 4; ++q)
		if (perQuadrant[q] < smallBudget/10) {
			VERBOSE("DenseComplementaryDrawTest FAILED: quadrant %d holds %u of the %u kept points (%u/%u/%u/%u), "
				"expected at least a tenth in each -- the thinning collapsed the draw onto one region",
				q, perQuadrant[q], smallBudget, perQuadrant[0], perQuadrant[1], perQuadrant[2], perQuadrant[3]);
			return false;
		}

	// 4) DETERMINISM: the same draw twice in one process, the second time into buffers that already
	// hold something. The draw carries no state between calls and reads no container whose iteration
	// order could vary, so a caller reusing its buffers must get exactly what a caller passing empty
	// ones gets -- which is what makes the appended keypoint indices reproducible.
	std::vector<Point2f> repeatA(11, Point2f(5.f, 6.f)), repeatB(2, Point2f(7.f, 8.f));
	std::vector<float> repeatC(4, 0.25f);
	SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numSmallBuckets, smallBudget, sparseA, repeatA, repeatB, repeatC);
	if (repeatA != denseA || repeatB != denseB || repeatC != confidences) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the draw is not a pure function of its inputs");
		return false;
	}

	// 5) CONFIDENCE IS THE ELIGIBILITY TEST, NOT THE RANKING. Two warps confident over the same
	// region with OPPOSITE confidence ramps must produce the very same draw: the winner of a bucket
	// is its highest-lattice-priority eligible cell, which depends on the cell coordinates alone.
	// This is what makes two pairs sharing image A able to agree on a point at all.
	Image32F rampUp(cv::Size(cells, cells), 0.f), rampDown(cv::Size(cells, cells), 0.f);
	for (int y = 0; y < 120; ++y)
		for (int x = 0; x < 120; ++x) {
			rampUp(y, x) = 0.35f + 0.6f*(float)(x + y)/(float)(2*cells);
			rampDown(y, x) = 0.95f - 0.6f*(float)(x + y)/(float)(2*cells);
		}
	std::vector<Point2f> upA, upB, downA, downB;
	std::vector<float> upC, downC;
	const std::vector<Point2f> noneOccupied;
	const int gridSide900 = DenseFillGridSide(900, cells);
	SampleWarpComplementary(imgA, imgB, warp, rampUp, 0.3f, 0.f, gridSide900, 900, noneOccupied, upA, upB, upC);
	SampleWarpComplementary(imgA, imgB, warp, rampDown, 0.3f, 0.f, gridSide900, 900, noneOccupied, downA, downB, downC);
	if (upA.empty() || upA != downA || upB != downB) {
		VERBOSE("DenseComplementaryDrawTest FAILED: opposite confidence ramps over one region drew %u and %u points "
			"at different positions -- confidence is still ranking the bucket winners",
			(unsigned)upA.size(), (unsigned)downA.size());
		return false;
	}
	// ...while each point still carries ITS OWN cell's confidence, which is what MakeDenseKeypoint stamps
	if (upC == downC) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the two ramps stamped identical confidences on the same cells");
		return false;
	}

	// 6) CROSS-PAIR COINCIDENCE, the reason for the lattice rule. Two pairs sharing image A, with
	// partially overlapping confident regions and budgets that give them DIFFERENT bucket grids: in
	// the region both cover, the A-side positions they draw must land on the same pixels, exactly,
	// so that the keypoint dedup downstream (FilterRedundantKeypoints, 0.1 px) can chain them into
	// one track. Confidence cannot deliver that -- the two pairs have different warps -- which is why
	// the winner rule is pair-independent.
	Image32F overlapAB(cv::Size(cells, cells), 0.f), overlapAC(cv::Size(cells, cells), 0.f);
	for (int y = 0; y < 120; ++y)
		for (int x = 0; x < 120; ++x)
			overlapAB(y, x) = 0.35f + 0.6f*(float)(x + y)/(float)(2*cells);
	for (int y = 40; y < cells; ++y)
		for (int x = 40; x < cells; ++x)
			overlapAC(y, x) = 0.95f - 0.6f*(float)(x + y)/(float)(2*cells);
	const int numBucketsAB = DenseFillGridSide(900, cells);
	const int numBucketsAC = DenseFillGridSide(1500, cells);
	std::vector<Point2f> abA, abB, acA, acB;
	std::vector<float> abC, acC;
	const size_t numAB = SampleWarpComplementary(imgA, imgB, warp, overlapAB, 0.3f, 0.f, numBucketsAB, 900, noneOccupied, abA, abB, abC);
	const size_t numAC = SampleWarpComplementary(imgA, imgB, warp, overlapAC, 0.3f, 0.f, numBucketsAC, 1500, noneOccupied, acA, acB, acC);
	if (numAB == 0 || numAC == 0 || numBucketsAB == numBucketsAC) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the two pairs drew %u/%u points on %dx%d and %dx%d grids -- "
			"the fixture must give them different grids for the coincidence to mean anything",
			(unsigned)numAB, (unsigned)numAC, numBucketsAB, numBucketsAB, numBucketsAC, numBucketsAC);
		return false;
	}
	// the cells both warps are confident about: [40, 120) on both axes, in pixels of A
	const Point2f commonLo(CellPixel(40, 40)), commonHi(CellPixel(119, 119));
	const auto InCommon = [&](const Point2f& pt) {
		return pt.x >= commonLo.x - 1e-3f && pt.x <= commonHi.x + 1e-3f &&
			pt.y >= commonLo.y - 1e-3f && pt.y <= commonHi.y + 1e-3f;
	};
	std::set<std::pair<float, float>> commonAC;
	unsigned numCommonAC = 0;
	for (const Point2f& pt : acA)
		if (InCommon(pt)) {
			commonAC.emplace(pt.x, pt.y);
			++numCommonAC;
		}
	unsigned numCommonAB = 0, numCoincident = 0;
	for (const Point2f& pt : abA)
		if (InCommon(pt)) {
			++numCommonAB;
			// EXACT equality, not a tolerance: the dedup downstream keys on position, and two draws
			// that agree only to within a pixel would leave two keypoints where one point was sampled
			if (commonAC.find(std::make_pair(pt.x, pt.y)) != commonAC.end())
				++numCoincident;
		}
	const unsigned numCommonMin = MINF(numCommonAB, numCommonAC);
	if (numCommonMin == 0 || numCoincident*5 < numCommonMin*2) {
		VERBOSE("DenseComplementaryDrawTest FAILED: only %u of the %u/%u points the two pairs drew in their common "
			"region coincide exactly, expected at least two fifths -- the draws are not chaining across pairs",
			numCoincident, numCommonAB, numCommonAC, numCommonMin);
		return false;
	}
	// still one winner per bucket on each pair's own grid, and still deterministic
	std::vector<int> hitsAB((size_t)numBucketsAB*numBucketsAB, 0), hitsAC((size_t)numBucketsAC*numBucketsAC, 0);
	for (const Point2f& pt : abA)
		++hitsAB[PixelToBucket(pt, numBucketsAB)];
	for (const Point2f& pt : acA)
		++hitsAC[PixelToBucket(pt, numBucketsAC)];
	FOREACH(b, hitsAB)
		if (hitsAB[b] > 1) {
			VERBOSE("DenseComplementaryDrawTest FAILED: bucket %u of the first pair holds %d points, expected at most 1", (unsigned)b, hitsAB[b]);
			return false;
		}
	FOREACH(b, hitsAC)
		if (hitsAC[b] > 1) {
			VERBOSE("DenseComplementaryDrawTest FAILED: bucket %u of the second pair holds %d points, expected at most 1", (unsigned)b, hitsAC[b]);
			return false;
		}
	std::vector<Point2f> repeatAC_A, repeatAC_B;
	std::vector<float> repeatAC_C;
	SampleWarpComplementary(imgA, imgB, warp, overlapAC, 0.3f, 0.f, numBucketsAC, 1500, noneOccupied, repeatAC_A, repeatAC_B, repeatAC_C);
	if (repeatAC_A != acA || repeatAC_B != acB || repeatAC_C != acC) {
		VERBOSE("DenseComplementaryDrawTest FAILED: the lattice draw is not a pure function of its inputs");
		return false;
	}

	// 7) THE THINNING, as its own entry point: it is how the draw caps itself once the unoccupied
	// buckets outnumber the budget. It hands the survivors back in the order it got them -- what
	// AppendDenseMatches labels keypoints by -- and it keeps a PREFIX of one pair-independent order
	// over the cells, so a tighter budget keeps a SUBSET of what a looser one kept. That nesting is
	// the property the fixed pitch above depends on: the ceiling is per-pair, so enforcing it by
	// anything that read this pair's own list would leave two pairs which agreed on every winner
	// holding different subsets of them.
	const auto CellIndexOf = [&](const Point2f& pt) {
		const Point2f cell(PixelToCell(pt, cv::Size(width, height), cv::Size(cells, cells)));
		const int cx = MINF(MAXF(ROUND2INT(cell.x), 0), cells-1);
		const int cy = MINF(MAXF(ROUND2INT(cell.y), 0), cells-1);
		return cy*cells + cx;
	};
	std::vector<int> acCellIdx;
	for (const Point2f& pt : acA)
		acCellIdx.push_back(CellIndexOf(pt));
	std::vector<Point2f> thinA(acA), thinB(acB);
	std::vector<float> thinC(acC);
	std::vector<int> thinCellIdx(acCellIdx);
	const unsigned thinTo = (unsigned)(acA.size()/3);
	ThinSampleByLatticePriority(thinA, thinB, thinC, thinCellIdx, cells, thinTo);
	if (thinA.size() != thinTo || thinB.size() != thinTo || thinC.size() != thinTo || thinCellIdx.size() != thinTo) {
		VERBOSE("DenseComplementaryDrawTest FAILED: thinning %u points to %u kept %u", (unsigned)acA.size(), thinTo, (unsigned)thinA.size());
		return false;
	}
	size_t src = 0;
	FOREACH(i, thinA) {
		while (src < acA.size() && !(acA[src] == thinA[i]))
			++src;
		if (src == acA.size() || !(acB[src] == thinB[i]) || acC[src] != thinC[i] || acCellIdx[src] != thinCellIdx[i]) {
			VERBOSE("DenseComplementaryDrawTest FAILED: the thinned sample is not an order-preserving index-parallel subsequence of the draw at %u", i);
			return false;
		}
	}
	// half that budget again: what survives it must be what the looser budget already kept
	std::vector<Point2f> tightA(acA), tightB(acB);
	std::vector<float> tightC(acC);
	std::vector<int> tightCellIdx(acCellIdx);
	ThinSampleByLatticePriority(tightA, tightB, tightC, tightCellIdx, cells, thinTo/2);
	const std::set<int> keptCells(thinCellIdx.begin(), thinCellIdx.end());
	FOREACH(i, tightCellIdx)
		if (keptCells.find(tightCellIdx[i]) == keptCells.end()) {
			VERBOSE("DenseComplementaryDrawTest FAILED: thinning to %u kept cell %d, which thinning the same sample to %u dropped -- "
				"the two budgets are not keeping nested prefixes of one order", thinTo/2, tightCellIdx[i], thinTo);
			return false;
		}
	// asked for at least what it holds, a sample comes back untouched; asked for nothing, it empties
	std::vector<Point2f> keepA(acA), keepB(acB);
	std::vector<float> keepC(acC);
	std::vector<int> keepCellIdx(acCellIdx);
	ThinSampleByLatticePriority(keepA, keepB, keepC, keepCellIdx, cells, (unsigned)acA.size() + 10);
	ThinSampleByLatticePriority(thinA, thinB, thinC, thinCellIdx, cells, 0);
	if (keepA != acA || keepB != acB || keepC != acC || !thinA.empty() || !thinB.empty() || !thinC.empty() || !thinCellIdx.empty()) {
		VERBOSE("DenseComplementaryDrawTest FAILED: thinning to a budget above the sample size, or to zero, did not behave");
		return false;
	}

	VERBOSE("DenseComplementaryDrawTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool DenseFillDensityTest()
{
	TD_TIMER_START();

	// --- the identity-warp setup of DenseComplementaryDrawTest -------------------------------
	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
	scene.images.emplace_back((IIndex)0, String("a.jpg"));
	scene.images.emplace_back((IIndex)1, String("b.jpg"));
	FOREACH(i, scene.images) {
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const int cells = 160;
	Image32F2 warp(cells, cells);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x)
			warp(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), cv::Size(width, height)), cv::Size(width, height));
	const auto CellPixel = [&](int cx, int cy) {
		return CellToPixel(Point2f((float)cx, (float)cy), cv::Size(cells, cells), cv::Size(width, height));
	};

	// --- 1) THE PITCH IS A FUNCTION OF THE DENSITY ALONE -------------------------------------
	// one bucket per dense match at full overlap, clamped to the warp side, never below 1
	if (DenseFillGridSide(2000, cells) != 45 || DenseFillGridSide(0, cells) != 1 ||
		DenseFillGridSide(1000000, cells) != cells) {
		VERBOSE("DenseFillDensityTest FAILED: pitch %d/%d/%d for densities 2000/0/1000000",
			DenseFillGridSide(2000, cells), DenseFillGridSide(0, cells), DenseFillGridSide(1000000, cells));
		return false;
	}

	// --- 2) THE DRAW SCALES WITH THE OVERLAP AREA, NOT WITH THE BUDGET -----------------------
	// the same pitch over two confident regions, one four times the other: the draw has to come out
	// about four times larger, which is what "a density" means and what the old per-draw grid (sized
	// so that ANY overlap yielded ~maxSamples) could not do
	const unsigned density = 2000;
	const int grid = DenseFillGridSide(density, cells);
	const std::vector<Point2f> none;
	std::vector<Point2f> bigA, bigB, smallA, smallB;
	std::vector<float> bigC, smallC;
	Image32F confidence(cells, cells);
	const auto DrawOver = [&](int cellsWide, int cellsHigh, const std::vector<Point2f>& occupied,
		std::vector<Point2f>& outA, std::vector<Point2f>& outB, std::vector<float>& outC) {
		confidence.setTo(0.f);
		for (int y = 0; y < cellsHigh; ++y)
			for (int x = 0; x < cellsWide; ++x)
				confidence(y, x) = 0.9f;
		return SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, 0.f, grid, density,
			occupied, outA, outB, outC);
	};
	const size_t numBig = DrawOver(80, 80, none, bigA, bigB, bigC);      // a quarter of the frame
	const size_t numSmall = DrawOver(40, 40, none, smallA, smallB, smallC); // a sixteenth
	if (numBig < 3*numSmall || numBig > 5*numSmall || numSmall == 0) {
		VERBOSE("DenseFillDensityTest FAILED: a 4x larger overlap drew %u against %u points",
			(unsigned)numBig, (unsigned)numSmall);
		return false;
	}

	// --- 3) THE SPARSE MATCHES TAKE THEIR SHARE OUT OF THE DRAW ------------------------------
	// the same region drawn three times: uncovered, half covered by guided matches, fully covered.
	// The yield has to fall with the uncovered area and reach zero when nothing is left uncovered.
	std::vector<Point2f> halfOccupied, allOccupied;
	for (int y = 0; y < 80; ++y)
		for (int x = 0; x < 80; ++x) {
			if (x < 40)
				halfOccupied.push_back(CellPixel(x, y));
			allOccupied.push_back(CellPixel(x, y));
		}
	std::vector<Point2f> halfA, halfB, fullA, fullB;
	std::vector<float> halfC, fullC;
	const size_t numHalf = DrawOver(80, 80, halfOccupied, halfA, halfB, halfC);
	const size_t numFull = DrawOver(80, 80, allOccupied, fullA, fullB, fullC);
	if (numHalf == 0 || numHalf > numBig*3/5 || numHalf < numBig/3) {
		VERBOSE("DenseFillDensityTest FAILED: half the overlap covered drew %u of %u",
			(unsigned)numHalf, (unsigned)numBig);
		return false;
	}
	if (numFull != 0) {
		VERBOSE("DenseFillDensityTest FAILED: a fully covered overlap still drew %u points",
			(unsigned)numFull);
		return false;
	}

	// --- 4) TWO PAIRS SHARING IMAGE A AGREE ON WHERE THEY SAMPLE IT --------------------------
	// the property the fixed pitch buys downstream: two pairs that both hold A stratify it on the
	// same grid, so their winners are the same cells and their A-side keypoints the same pixels --
	// which is what lets FilterRedundantKeypoints chain them into a track longer than two. The
	// second pair sees a LARGER confident region, so this is not two identical inputs: the shared
	// part of the two draws still has to agree pixel for pixel.
	std::vector<Point2f> wideA, wideB;
	std::vector<float> wideC;
	DrawOver(120, 120, none, wideA, wideB, wideC);
	size_t numShared = 0;
	for (const Point2f& pt : bigA)
		if (std::find_if(wideA.begin(), wideA.end(), [&](const Point2f& q) {
				return normSq(q - pt) < 1e-4f; }) != wideA.end())
			++numShared;
	// not all of them: the buckets STRADDLING the edge of the smaller pair's confident region see
	// different candidate sets in the two draws (the wider one reaches cells the narrower one has no
	// confidence in), so their winners may differ. Those are 45 of the 529 buckets here. The bar is
	// what the property is worth -- the great majority of the two draws land on the same pixels of A
	// -- not an exact count that would break on any change to the region sizes.
	if (numShared*20 < bigA.size()*17) {
		VERBOSE("DenseFillDensityTest FAILED: only %u of %u samples of A are shared by a second pair",
			(unsigned)numShared, (unsigned)bigA.size());
		return false;
	}

	// --- 5) AND THEY STILL AGREE WHEN THEIR CEILINGS DIFFER ----------------------------------
	// The pitch is shared but DenseFillCeiling is not: it is the density over the SMALLER of the
	// pair's two inlier areas, so two pairs holding A with differently sized B sides thin one
	// agreement to two different sizes. Enforcing that ceiling is therefore the step that has to
	// preserve the agreement, and it does by ranking the sample on the same pair-independent key the
	// bucket winners were picked on and keeping a prefix of it -- the tighter budget's survivors are
	// the looser one's, cell for cell. Both pairs see the SAME confident region here on purpose: the
	// only variable left is the ceiling, section 4 above having varied the region instead.
	// The arithmetic: a fully confident A fills all 45x45 = 2025 buckets, and B sides of 0.50 and
	// 0.35 inlier area cap the two draws at round(2000*0.50) = 1000 and round(2000*0.35) = 700. Two
	// prefixes of one order nest, so all 700 of the tighter draw are among the looser draw's 1000.
	// An even stride through each pair's own list -- what this used to be -- would keep the index
	// sets {floor(i*2025/1000)} and {floor(i*2025/700)}, which meet in only 400 of the 700; the bar
	// below sits above that and below the nesting.
	DenseMatchConfig ceilingConfig;
	ceilingConfig.denseMatchesPerFrame = density;
	PairVerdict verdictWideB, verdictNarrowB;
	verdictWideB.inlierAreaA = 1.00f; verdictWideB.inlierAreaB = 0.50f;
	verdictNarrowB.inlierAreaA = 1.00f; verdictNarrowB.inlierAreaB = 0.35f;
	const unsigned capWideB = DenseFillCeiling(ceilingConfig, verdictWideB);
	const unsigned capNarrowB = DenseFillCeiling(ceilingConfig, verdictNarrowB);
	confidence.setTo(0.9f);
	std::vector<Point2f> wideBA, wideBB, narrowBA, narrowBB;
	std::vector<float> wideBC, narrowBC;
	const size_t numWideB = SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, 0.f, grid, capWideB,
		none, wideBA, wideBB, wideBC);
	const size_t numNarrowB = SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, 0.f, grid, capNarrowB,
		none, narrowBA, narrowBB, narrowBC);
	if (numWideB != capWideB || numNarrowB != capNarrowB) {
		VERBOSE("DenseFillDensityTest FAILED: ceilings %u/%u drew %u/%u points -- the fixture must put both "
			"draws over their ceiling for the thinning to be under test at all",
			capWideB, capNarrowB, (unsigned)numWideB, (unsigned)numNarrowB);
		return false;
	}
	std::set<std::pair<float, float>> keptByWideB;
	for (const Point2f& pt : wideBA)
		keptByWideB.emplace(pt.x, pt.y);
	size_t numSharedCapped = 0;
	for (const Point2f& pt : narrowBA)
		if (keptByWideB.find(std::make_pair(pt.x, pt.y)) != keptByWideB.end())
			++numSharedCapped;
	if (numSharedCapped*20 < numNarrowB*19) {
		VERBOSE("DenseFillDensityTest FAILED: of the %u points the tighter ceiling kept, only %u are among the "
			"%u the looser one kept -- the two thinnings are not nested",
			(unsigned)numNarrowB, (unsigned)numSharedCapped, (unsigned)numWideB);
		return false;
	}

	VERBOSE("ROMA2 dense fill density test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool DenseFillCeilingTest()
{
	TD_TIMER_START();

	// The ceiling is the configured density over the SMALLER of the verdict's two inlier areas: the
	// bucket grid lives in A's frame and cannot see how much of B the pair's dense matches would
	// land on, so this is the only term that bounds their density in B.
	DenseMatchConfig config;
	config.denseMatchesPerFrame = 2000;
	PairVerdict verdict;

	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.50f;
	const unsigned symmetric = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.10f;
	const unsigned narrowB = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 0.10f; verdict.inlierAreaB = 0.50f;
	const unsigned narrowA = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 1.00f; verdict.inlierAreaB = 1.00f;
	const unsigned identical = DenseFillCeiling(config, verdict);
	if (symmetric != 1000 || narrowB != 200 || narrowA != 200 || identical != 2000) {
		VERBOSE("DenseFillCeilingTest FAILED: ceilings %u/%u/%u/%u for .5|.5, .5|.1, .1|.5, 1|1",
			symmetric, narrowB, narrowA, identical);
		return false;
	}
	// the two sides are symmetric -- it is min(), not "A's area" -- and the density is linear in the
	// knob, which is what makes --dense-matches readable as matches per frame of overlap
	config.denseMatchesPerFrame = 500;
	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.50f;
	if (DenseFillCeiling(config, verdict) != 250) {
		VERBOSE("DenseFillCeilingTest FAILED: ceiling %u at density 500 over half an overlap",
			DenseFillCeiling(config, verdict));
		return false;
	}

	VERBOSE("ROMA2 dense fill ceiling test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The two-camera fixture the verdict and the assembly tests below share: a 640x480 pinhole pair of
// focal 400, the second camera offset along X and Y and not rotated, looking at a NON-PLANAR
// surface -- a wedge of two planes meeting at the world plane X = 0 and receding to either side of
// it. The camera trusts its intrinsics, so a fit on this fixture takes the calibrated (essential)
// branch unless a test forces the fundamental one.
// Non-planar is what makes the fixture a test rather than a tautology: a plane is a valid two-view
// interpretation in either branch -- the calibrated fit reads it as one of the two cheirality-valid
// poses it admits, the uncalibrated one as any member of a whole family of fundamental matrices --
// so a warp of a single plane would be explained exactly by a geometry that has nothing to do with
// these two cameras.
constexpr REAL DENSE_WEDGE_SEAM_Z = 3.0; // depth of the seam, on the optical axis of the first camera
constexpr REAL DENSE_WEDGE_SLOPE = 1.6;  // how fast the two planes recede to either side of it

static void MakeDenseWedgeScene(Scene& scene)
{
	const int width = 640, height = 480;
	PinholeCamera* const camera = new PinholeCamera(cv::Size(width, height),
		REAL(400), REAL(400), REAL(width)/2, REAL(height)/2);
	camera->trustIntrinsics = true; // the calibrated branch, so every fit here reports a relative pose
	scene.cameras.emplace_back(camera);
	for (unsigned i = 0; i < 2; ++i) {
		Pose3D pose;
		pose.R = Matrix3x3::IDENTITY;
		pose.C = Point3(i == 0 ? REAL(0) : REAL(0.4), i == 0 ? REAL(0) : REAL(0.1), REAL(0));
		scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i), pose, (IIndex)0, scene.cameras[0]);
	}
}

// Where the ray through pixel `pt` of `img` meets the wedge, in world coordinates; false when it
// meets neither plane in front of the camera, which only happens far off axis -- outside the region
// the tests below make confident
static bool IntersectDenseWedge(const Image& img, const Point2f& pt, Point3& X)
{
	const Point3 dir(img.Ray(Cast<REAL>(pt)));
	const Point3& C = img.C;
	// Z = SEAM_Z - SLOPE*X where X <= 0 and Z = SEAM_Z + SLOPE*X where X > 0, along X(t) = C + t*dir
	const REAL denom[2] = { dir.z + DENSE_WEDGE_SLOPE*dir.x, dir.z - DENSE_WEDGE_SLOPE*dir.x };
	const REAL num[2] = { DENSE_WEDGE_SEAM_Z - DENSE_WEDGE_SLOPE*C.x - C.z,
	                      DENSE_WEDGE_SEAM_Z + DENSE_WEDGE_SLOPE*C.x - C.z };
	for (int side = 0; side < 2; ++side) {
		if (ABS(denom[side]) < 1e-9)
			continue; // the ray runs parallel to this plane
		const REAL t = num[side]/denom[side];
		if (t <= 0)
			continue;
		const REAL x = C.x + t*dir.x;
		if (side == 0 ? x > 0 : x <= 0)
			continue; // this root lies on the other plane's half of the wedge
		X = Point3(x, C.y + t*dir.y, C.z + t*dir.z);
		return true;
	}
	return false;
}

// The exact correspondence of a pixel of imgSrc in imgDst through the wedge; false when the ray
// misses the surface or the point it hits falls outside imgDst
static bool DenseWedgeCorrespondence(const Image& imgSrc, const Image& imgDst, const Point2f& ptSrc, Point2f& ptDst)
{
	Point3 X;
	if (!IntersectDenseWedge(imgSrc, ptSrc, X))
		return false;
	const auto [proj, valid] = imgDst.ProjectPoint(X);
	if (!valid)
		return false;
	ptDst = Point2f((float)proj.x, (float)proj.y);
	return Image8U::isInside(ptDst, imgDst.GetSize());
}

// The normalized (align_corners=false) coordinate a warp map stores for a pixel of the target image
static inline Point2f DenseNormCoord(const Point2f& pt, const cv::Size& size)
{
	return Point2f(2.f*(pt.x + 0.5f)/(float)size.width - 1.f, 2.f*(pt.y + 0.5f)/(float)size.height - 1.f);
}

// One direction of the exact bidirectional warp: every cell of imgSrc's grid projected onto imgDst
// through the wedge, with confidence 1 on the given cell rectangle and 0 everywhere else. A cell
// whose ray misses the surface is pointed far outside imgDst, the "no correspondence here" encoding
// every warp draw reads.
static void MakeDenseWedgeWarp(const Image& imgSrc, const Image& imgDst, int cells,
	int x0, int x1, int y0, int y1, WarpMaps& maps)
{
	const cv::Size gridSize(cells, cells), sizeSrc(imgSrc.GetSize()), sizeDst(imgDst.GetSize());
	maps.warp.create(gridSize);
	maps.confidence.create(gridSize);
	maps.confidence.memset(0);
	for (int y = 0; y < cells; ++y) {
		for (int x = 0; x < cells; ++x) {
			maps.warp(y, x) = Point2f(-3.f, -3.f);
			Point2f ptDst;
			if (!DenseWedgeCorrespondence(imgSrc, imgDst,
				CellToPixel(Point2f((float)x, (float)y), gridSize, sizeSrc), ptDst))
				continue;
			maps.warp(y, x) = DenseNormCoord(ptDst, sizeDst);
			if (x >= x0 && x < x1 && y >= y0 && y < y1)
				maps.confidence(y, x) = 1.f;
		}
	}
}

// A warp of the same grid that has nothing to do with the two cameras: one fixed homography of A --
// 5 degrees of rotation, a 5% scale, a shift and a mild projective term -- confident over the same
// cell rectangle. Smooth and locally consistent, exactly like a hallucinated warp.
static void MakeDenseHomographyWarp(const Image& imgSrc, const Image& imgDst, int cells,
	int x0, int x1, int y0, int y1, WarpMaps& maps)
{
	const cv::Size gridSize(cells, cells), sizeSrc(imgSrc.GetSize()), sizeDst(imgDst.GetSize());
	maps.warp.create(gridSize);
	maps.confidence.create(gridSize);
	maps.confidence.memset(0);
	const float cosT = 0.95f*(float)COS(D2R(REAL(5))), sinT = 0.95f*(float)SIN(D2R(REAL(5)));
	const float cxSrc = 0.5f*(float)sizeSrc.width, cySrc = 0.5f*(float)sizeSrc.height;
	const float cxDst = 0.5f*(float)sizeDst.width, cyDst = 0.5f*(float)sizeDst.height;
	for (int y = 0; y < cells; ++y) {
		for (int x = 0; x < cells; ++x) {
			const Point2f ptSrc(CellToPixel(Point2f((float)x, (float)y), gridSize, sizeSrc));
			const float dx = ptSrc.x - cxSrc, dy = ptSrc.y - cySrc;
			const float w = 1.f + 3e-4f*dx - 2e-4f*dy;
			const Point2f ptDst(cxDst + (cosT*dx - sinT*dy - 30.f)/w, cyDst + (sinT*dx + cosT*dy + 12.f)/w);
			maps.warp(y, x) = Image8U::isInside(ptDst, sizeDst) ?
				DenseNormCoord(ptDst, sizeDst) : Point2f(-3.f, -3.f);
			if (x >= x0 && x < x1 && y >= y0 && y < y1)
				maps.confidence(y, x) = 1.f;
		}
	}
}

// The pixel <-> cell convention of the warp grid, end to end. RoMa v2's warp and confidence maps hold
// cell (i,j) at the align_corners=False centre p = (i+0.5)*W/C - 0.5 of the source image (romav2
// geometry.py get_normalized_grid, the grid its own sampler and refiners pair the warp with), the
// convention DenormCoord already reads the warp's VALUES in. A synthetic warp generated in exactly that
// convention, of a known homography, on a 1024x768 and a 2789x2092 frame with 160 cells:
//  - CellToPixel is that cell centre and PixelToCell its exact inverse, so a cell centre rounds back
//    to its own cell (the occupancy of the complementary draw, the dense fill's rebuild of the grid);
//  - tracking (TrackKeypointsByWarp, pixel -> cell) lands every keypoint within 0.05 px of its true
//    target, and a keypoint within half a cell of a border reads the border cell's value (clamped,
//    not extrapolated), i.e. the target of the nearest cell centre in that axis;
//  - every warp draw places a cell's A end at the cell's true centre (CollectWarpCandidates through
//    SampleWarpByCoverage, CollectEligibleCells through the verdict's inlier cells);
//  - the complementary draw's occupancy (pixel -> cell, rounded) strikes the cell a position lies in.
// The convention is written out here rather than taken from DenseWarp.h, so the test states it
// independently of the implementation.
bool DenseWarpGridConventionTest()
{
	TD_TIMER_START();
	const auto CellCentre = [](float i, int size, int cells) { return (i + 0.5f)*(float)size/(float)cells - 0.5f; };
	const auto NormalizedCoord = [](float p, int size) { return 2.f*(p + 0.5f)/(float)size - 1.f; };
	const int cells = 160;
	for (const cv::Size size : {cv::Size(1024, 768), cv::Size(2789, 2092)}) {
		Scene scene;
		scene.cameras.emplace_back(new PinholeCamera(size, REAL(size.width), REAL(size.width),
			REAL(size.width - 1)/2, REAL(size.height - 1)/2));
		scene.images.emplace_back((IIndex)0, String("a.jpg"));
		scene.images.emplace_back((IIndex)1, String("b.jpg"));
		FOREACH(i, scene.images) {
			scene.images[i].cameraID = 0;
			scene.images[i].pCamera = scene.cameras[0];
		}
		Image& imgA = scene.images[0];
		const Image& imgB = scene.images[1];
		// a known homography of A onto B: 0.9 zoom and 3 degrees about the centre, a shift and a mild
		// projective term, which keeps all of A inside B
		const float cx = 0.5f*(float)(size.width - 1), cy = 0.5f*(float)(size.height - 1);
		const float cosT = 0.9f*(float)COS(D2R(REAL(3))), sinT = 0.9f*(float)SIN(D2R(REAL(3)));
		const float projective = 1e-5f*1024.f/(float)size.width;
		const auto MapAB = [&](const Point2f& a) {
			const float dx = a.x - cx, dy = a.y - cy;
			const float w = 1.f + projective*(dx - dy);
			return Point2f(cx + (cosT*dx - sinT*dy + 10.f)/w, cy + (sinT*dx + cosT*dy - 6.f)/w);
		};
		const auto CellCentreA = [&](int x, int y) {
			return Point2f(CellCentre((float)x, size.width, cells), CellCentre((float)y, size.height, cells));
		};
		// 0) THE MAPS THEMSELVES: CellToPixel is the cell centre above, PixelToCell its inverse, and
		// the rounding the occupancy and the dense fill's rebuild apply to PixelToCell lands every
		// cell centre back on its own cell
		const cv::Size gridSize(cells, cells);
		for (int i = 0; i < cells; ++i) {
			for (const float f : {0.f, 0.25f, -0.5f, 0.5f}) {
				const Point2f cell((float)i + f, (float)(cells - 1 - i) - f);
				const Point2f pixel(CellToPixel(cell, gridSize, size));
				if (ABS(pixel.x - CellCentre(cell.x, size.width, cells)) > 1e-3f ||
					ABS(pixel.y - CellCentre(cell.y, size.height, cells)) > 1e-3f ||
					norm(PixelToCell(pixel, size, gridSize) - cell) > 1e-4f) {
					VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: cell (%g,%g) -> pixel (%g,%g) -> cell (%g,%g)",
						size.width, size.height, cell.x, cell.y, pixel.x, pixel.y,
						PixelToCell(pixel, size, gridSize).x, PixelToCell(pixel, size, gridSize).y);
					return false;
				}
			}
			const Point2f back(PixelToCell(CellToPixel(Point2f((float)i, (float)i), gridSize, size), size, gridSize));
			if (ROUND2INT(back.x) != i || ROUND2INT(back.y) != i) {
				VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: cell %d rounds back to (%d,%d)",
					size.width, size.height, i, ROUND2INT(back.x), ROUND2INT(back.y));
				return false;
			}
		}

		Image32F2 warp(cells, cells);
		const Image32F confidence(cv::Size(cells, cells), 1.f);
		for (int y = 0; y < cells; ++y) {
			for (int x = 0; x < cells; ++x) {
				const Point2f b(MapAB(CellCentreA(x, y)));
				ASSERT(Image8U::isInside(b, size));
				warp(y, x) = Point2f(NormalizedCoord(b.x, size.width), NormalizedCoord(b.y, size.height));
			}
		}

		// 1) TRACKING, pixel -> cell. Interior keypoints at random, the outermost cell centres, the
		// frame's corners and edges, and keypoints within half a cell of each border: the expected
		// target is that of the keypoint clamped to the span of the cell centres, which is the keypoint
		// itself except within half a cell of a border, where the read is the border cell's value
		const float firstX = CellCentre(0.f, size.width, cells), lastX = CellCentre((float)(cells - 1), size.width, cells);
		const float firstY = CellCentre(0.f, size.height, cells), lastY = CellCentre((float)(cells - 1), size.height, cells);
		std::mt19937 rng(11);
		std::uniform_real_distribution<float> ux(0.f, (float)(size.width - 1)), uy(0.f, (float)(size.height - 1));
		imgA.keypoints.clear();
		for (unsigned i = 0; i < 400; ++i)
			imgA.keypoints.emplace_back(ux(rng), uy(rng), 1.f);
		const float W1 = (float)(size.width - 1), H1 = (float)(size.height - 1);
		for (const Point2f& pt : {Point2f(firstX, firstY), Point2f(lastX, lastY), Point2f(firstX, lastY), Point2f(lastX, firstY),
			Point2f(lastX, cy), Point2f(cx, lastY), Point2f(CellCentre(1.f, size.width, cells), cy),
			Point2f(0.f, 0.f), Point2f(W1, H1), Point2f(0.f, H1), Point2f(W1, 0.f),
			Point2f(1.f, cy), Point2f(cx, 1.f), Point2f(1.f, 1.f), Point2f(W1 - 1.f, cy), Point2f(cx, H1 - 1.f)})
			imgA.keypoints.emplace_back(pt, 1.f);
		std::vector<Point2f> trackedA, trackedB;
		std::vector<uchar> status;
		if (TrackKeypointsByWarp(imgA, imgB, warp, confidence, 0.1f, trackedA, trackedB, status) != imgA.keypoints.size()) {
			VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: not every keypoint tracked through a fully confident warp",
				size.width, size.height);
			return false;
		}
		float maxError = 0.f;
		unsigned worst = 0;
		FOREACH(i, trackedA) {
			const Point2f& a = imgA.keypoints[i].pt;
			const Point2f expected(MapAB(Point2f(CLAMP(a.x, firstX, lastX), CLAMP(a.y, firstY, lastY))));
			const float error = (float)norm(trackedB[i] - expected);
			if (maxError < error) {
				maxError = error;
				worst = (unsigned)i;
			}
		}
		if (maxError > 0.05f) {
			VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: keypoint (%g,%g) tracked %g px off its true target",
				size.width, size.height, imgA.keypoints[worst].pt.x, imgA.keypoints[worst].pt.y, maxError);
			return false;
		}

		// 2) THE DRAW, cell -> pixel (CollectWarpCandidates): with a budget past the grid every cell is
		// taken, in raster order, and each one's A end must sit at the cell's centre, its B end at the
		// cell centre's target
		std::vector<Point2f> sampledA, sampledB;
		if (SampleWarpByCoverage(imgA, imgB, warp, confidence, 0.1f, 0.f, (unsigned)(cells*cells), sampledA, sampledB) != (size_t)(cells*cells)) {
			VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: the draw did not take every cell of a fully confident warp",
				size.width, size.height);
			return false;
		}
		FOREACH(k, sampledA) {
			const Point2f centre(CellCentreA((int)k%cells, (int)k/cells));
			if (norm(sampledA[k] - centre) > 2e-3f || norm(sampledB[k] - MapAB(centre)) > 0.01f) {
				VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: cell (%d,%d) drawn at (%g,%g) -> (%g,%g), its centre is (%g,%g) -> (%g,%g)",
					size.width, size.height, (int)k%cells, (int)k/cells, sampledA[k].x, sampledA[k].y, sampledB[k].x, sampledB[k].y,
					centre.x, centre.y, MapAB(centre).x, MapAB(centre).y);
				return false;
			}
		}

		// 3) OCCUPANCY, pixel -> cell rounded: one bucket per cell, and positions 0.45 cell from the
		// centres of the last column's and the last row's cells, on the side of the frame's centre --
		// each lies in that cell, so the complementary draw must take every cell but those
		std::vector<Point2f> occupiedA;
		std::vector<bool> struck((size_t)cells*cells, false);
		const float offX = 0.45f*(float)size.width/(float)cells, offY = 0.45f*(float)size.height/(float)cells;
		for (int i = 0; i < cells; i += 7) {
			occupiedA.emplace_back(lastX - offX, CellCentre((float)i, size.height, cells));
			struck[(size_t)i*cells + (cells - 1)] = true;
			occupiedA.emplace_back(CellCentre((float)i, size.width, cells), lastY - offY);
			struck[(size_t)(cells - 1)*cells + i] = true;
		}
		const size_t numStruck = (size_t)std::count(struck.begin(), struck.end(), true);
		std::vector<float> sampledConfidences;
		SampleWarpComplementary(imgA, imgB, warp, confidence, 0.1f, 0.f, cells, (unsigned)(cells*cells), occupiedA,
			sampledA, sampledB, sampledConfidences);
		bool bStruckDrawn = false;
		for (const Point2f& pt : sampledA) {
			const int x = ROUND2INT((pt.x + 0.5f)*(float)cells/(float)size.width - 0.5f);
			const int y = ROUND2INT((pt.y + 0.5f)*(float)cells/(float)size.height - 0.5f);
			if (struck[(size_t)y*cells + x])
				bStruckDrawn = true;
		}
		if (sampledA.size() != (size_t)cells*cells - numStruck || bStruckDrawn) {
			VERBOSE("DenseWarpGridConventionTest FAILED: %dx%d: the complementary draw took %u cells around %u occupied ones (%s an occupied cell), expected %u",
				size.width, size.height, (unsigned)sampledA.size(), (unsigned)numStruck, bStruckDrawn ? "including" : "excluding",
				(unsigned)((size_t)cells*cells - numStruck));
			return false;
		}
	}

	// 4) THE VERDICT'S CELLS (CollectEligibleCells, cell -> pixel): the exact bidirectional warp of the
	// wedge, generated in the same convention, admitted, and every inlier cell of A at a cell centre
	{
		Scene scene;
		MakeDenseWedgeScene(scene);
		const Image& imgA = scene.images[0];
		const Image& imgB = scene.images[1];
		MatchConfig matchCfg;
		PairsMatcher matcher(scene, matchCfg);
		const int wedgeCells = 64;
		const int rx0 = 14, rx1 = 50, ry0 = 13, ry1 = 47;
		PairWarps warps;
		for (int direction = 0; direction < 2; ++direction) {
			const Image& imgSrc = direction == 0 ? imgA : imgB;
			const Image& imgDst = direction == 0 ? imgB : imgA;
			WarpMaps& maps = direction == 0 ? warps.ab : warps.ba;
			const cv::Size sizeSrc(imgSrc.GetSize()), sizeDst(imgDst.GetSize());
			maps.warp.create(wedgeCells, wedgeCells);
			maps.confidence.create(wedgeCells, wedgeCells);
			maps.confidence.memset(0);
			for (int y = 0; y < wedgeCells; ++y) {
				for (int x = 0; x < wedgeCells; ++x) {
					maps.warp(y, x) = Point2f(-3.f, -3.f);
					Point2f ptDst;
					if (!DenseWedgeCorrespondence(imgSrc, imgDst, Point2f(CellCentre((float)x, sizeSrc.width, wedgeCells),
						CellCentre((float)y, sizeSrc.height, wedgeCells)), ptDst))
						continue;
					maps.warp(y, x) = Point2f(NormalizedCoord(ptDst.x, sizeDst.width), NormalizedCoord(ptDst.y, sizeDst.height));
					if (x >= rx0 && x < rx1 && y >= ry0 && y < ry1)
						maps.confidence(y, x) = 1.f;
				}
			}
		}
		const DenseMatchConfig config;
		ImagePair pair(0, 1);
		PairVerdict verdict;
		JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
		if (!verdict.admitted || verdict.inliersA.empty()) {
			VERBOSE("DenseWarpGridConventionTest FAILED: the exact wedge warp was rejected (inlier areas %.4f/%.4f)",
				verdict.inlierAreaA, verdict.inlierAreaB);
			return false;
		}
		for (const Point2f& pt : verdict.inliersA) {
			const float x = (pt.x + 0.5f)*(float)wedgeCells/(float)imgA.GetWidth() - 0.5f;
			const float y = (pt.y + 0.5f)*(float)wedgeCells/(float)imgA.GetHeight() - 0.5f;
			if (ABS(x - (float)ROUND2INT(x)) > 1e-3f || ABS(y - (float)ROUND2INT(y)) > 1e-3f) {
				VERBOSE("DenseWarpGridConventionTest FAILED: a verdict inlier of A at (%g,%g) is cell (%g,%g), not a cell centre",
					pt.x, pt.y, x, y);
				return false;
			}
		}
	}

	VERBOSE("DenseWarpGridConventionTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// The pair verdict (JudgePairDense) on the exact bidirectional warp of two pinhole cameras looking
// at a non-planar surface: a warp confident over ~30% of both frames is admitted with both inlier
// areas measuring that share, a warp confident over 30% of A whose B side maps into A over only 3%
// is rejected by the min side alone, a smooth warp unrelated to the cameras is rejected however
// exactly one geometry explains its own side -- through the calibrated branch and through the
// fundamental one alike -- and minOverlap 0 admits the first two
bool DenseVerdictTest()
{
	TD_TIMER_START();

	Scene scene;
	MakeDenseWedgeScene(scene);
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	const int cells = 64;
	// the confident region: 36x34 of the 64x64 cells, 29.9% of the grid, placed so that every one
	// of its cells lands inside the other frame in both directions
	const int rx0 = 14, rx1 = 50, ry0 = 13, ry1 = 47;
	const float regionShare = (float)((rx1-rx0)*(ry1-ry0))/(float)(cells*cells);

	// (a) a region covering ~30% of both frames: admitted, both inlier areas measuring the region
	PairWarps warps;
	MakeDenseWedgeWarp(imgA, imgB, cells, rx0, rx1, ry0, ry1, warps.ab);
	MakeDenseWedgeWarp(imgB, imgA, cells, rx0, rx1, ry0, ry1, warps.ba);
	DenseMatchConfig config; // minConfidence 0.1, minOverlap 0.10
	ImagePair pair(0, 1);
	PairVerdict verdict;
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	if (!verdict.admitted ||
		ABS(verdict.inlierAreaA - regionShare) > 0.05f || ABS(verdict.inlierAreaB - regionShare) > 0.05f) {
		VERBOSE("DenseVerdictTest FAILED: a warp confident over %.4f of both frames was %s with inlier areas %.4f/%.4f",
			regionShare, verdict.admitted ? "admitted" : "rejected", verdict.inlierAreaA, verdict.inlierAreaB);
		return false;
	}
	// every confident cell of either direction lands in the other frame and is explained by the one
	// fitted geometry, so both areas are the region itself and not some fraction of it
	if (verdict.confidentAreaA != regionShare || verdict.confidentAreaB != regionShare ||
		verdict.inlierAreaA != verdict.confidentAreaA || verdict.inlierAreaB != verdict.confidentAreaB) {
		VERBOSE("DenseVerdictTest FAILED: confident areas %.4f/%.4f and inlier areas %.4f/%.4f over a region of %.4f",
			verdict.confidentAreaA, verdict.confidentAreaB, verdict.inlierAreaA, verdict.inlierAreaB, regionShare);
		return false;
	}
	// the population the dense fill draws from: A's inlier cells, index-parallel with their confidences
	if (verdict.inliersA.size() != (size_t)((rx1-rx0)*(ry1-ry0)) ||
		verdict.inliersB.size() != verdict.inliersA.size() ||
		verdict.confidences.size() != verdict.inliersA.size()) {
		VERBOSE("DenseVerdictTest FAILED: the verdict kept %u/%u cells with %u confidences, expected %u",
			(unsigned)verdict.inliersA.size(), (unsigned)verdict.inliersB.size(),
			(unsigned)verdict.confidences.size(), (unsigned)((rx1-rx0)*(ry1-ry0)));
		return false;
	}
	// an admitted pair carries the fit's geometry and no matches, and that geometry is the one the
	// two cameras really have: the fit is handed warp cells and nothing else, and it comes back
	// within a twentieth of a degree of the truth, which is what says the warp carried the geometry.
	// It is not evidence that the scene's own solution stayed out of the fit -- a leak would read
	// exactly the same; the temporary poseless Image copies JudgePairDense builds are what keeps it
	// out
	if (!pair.relativePose.has_value() || !pair.F.has_value() || !pair.matches.empty()) {
		VERBOSE("DenseVerdictTest FAILED: an admitted pair carries %u matches, pose %d, F %d",
			(unsigned)pair.matches.size(), (int)pair.relativePose.has_value(), (int)pair.F.has_value());
		return false;
	}
	const REAL angleErr = R2D(ACOS(ComputeAngle(pair.relativePose->R, poseGT.R)));
	const REAL tSim = ABS(normalized(pair.relativePose->GetT()).dot(normalized(poseGT.GetT())));
	if (angleErr > REAL(0.05) || tSim < REAL(0.9999)) {
		VERBOSE("DenseVerdictTest FAILED: the fitted geometry is %.4f deg and %.6f off the two cameras", angleErr, tSim);
		return false;
	}

	// (b) the same 30% of A, but B's confident cells map into A over only 3%: the min side rejects
	// the pair although A's own side would have admitted it
	const int bx0 = 20, bx1 = 32, by0 = 20, by1 = 30;
	const float smallShare = (float)((bx1-bx0)*(by1-by0))/(float)(cells*cells);
	MakeDenseWedgeWarp(imgB, imgA, cells, bx0, bx1, by0, by1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	if (verdict.admitted || ABS(verdict.inlierAreaA - regionShare) > 0.05f ||
		ABS(verdict.inlierAreaB - smallShare) > 0.01f || !verdict.inliersA.empty() || pair.F.has_value()) {
		VERBOSE("DenseVerdictTest FAILED: a B side of %.4f (expected %.4f) next to an A side of %.4f was %s, "
			"keeping %u cells and %d geometry",
			verdict.inlierAreaB, smallShare, verdict.inlierAreaA, verdict.admitted ? "admitted" : "rejected",
			(unsigned)verdict.inliersA.size(), (int)pair.F.has_value());
		return false;
	}

	// (c) a smooth warp that has nothing to do with the two cameras -- a homography of A's grid --
	// against the same camera-exact B side as (a). The fixture trusts its intrinsics, so this is the
	// calibrated branch: a homography is a valid planar two-view interpretation there too, one of
	// the poses it admits explaining the whole of A's own side, so A cannot tell it from a true
	// pair; the other direction can, and that is the whole point of measuring the min of the two
	MakeDenseWedgeWarp(imgB, imgA, cells, rx0, rx1, ry0, ry1, warps.ba);
	MakeDenseHomographyWarp(imgA, imgB, cells, rx0, rx1, ry0, ry1, warps.ab);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	// one geometry does explain the whole of the homography's own side, exactly as it explains a
	// true pair's -- an inlier count or ratio on A's cells cannot separate the two -- while the B
	// side reads a fraction of it, and under the bar. B's side is measured, not skipped: it comes
	// out at 0.0637 here, and demanding it above zero is what separates "the min-side rule rejected
	// the pair" from "B's side was never computed"
	if (verdict.admitted || verdict.confidentAreaA != regionShare ||
		ABS(verdict.inlierAreaA - regionShare) > 0.05f || verdict.inlierAreaB <= 0.f ||
		verdict.inlierAreaB >= config.minOverlap || verdict.inlierAreaB >= 0.5f*verdict.inlierAreaA) {
		VERBOSE("DenseVerdictTest FAILED: a homography warp confident over %.4f of A (inlier areas %.4f/%.4f, bar %.4f) was %s",
			verdict.confidentAreaA, verdict.inlierAreaA, verdict.inlierAreaB, config.minOverlap,
			verdict.admitted ? "admitted" : "rejected");
		return false;
	}

	// (d) the same homography, re-judged through the UNCALIBRATED branch. This is the case the
	// min-side rule was designed for: seven degrees of freedom, and a plane is explained exactly by
	// a whole family of fundamental matrices, so no fit on A's cells alone -- however many inliers
	// it counts -- can tell a hallucinated warp from a true pair. The verdict has to reject it here
	// for the same reason it rejects it in (c), on B's side and nothing else.
	{
		MatchConfig fundamentalCfg;
		fundamentalCfg.forceFundamental = true; // 7-DoF F over the fixture's trusted intrinsics
		const PairsMatcher fundamentalMatcher(scene, fundamentalCfg);
		if (PairsMatcher::SelectGeometryBranch(fundamentalCfg, imgA, imgB) != PairsMatcher::GeometryBranch::FUNDAMENTAL) {
			VERBOSE("DenseVerdictTest FAILED: forceFundamental did not select the fundamental branch");
			return false;
		}
		// same outcome as (c), and for the same reason: A's own side fully explained (0.2988 of the
		// grid, all of it), B's side measured and small (0.0173 here), the pair rejected by the min
		// of the two
		JudgePairDense(fundamentalMatcher, imgA, imgB, warps, config, pair, verdict);
		if (verdict.admitted || verdict.confidentAreaA != regionShare ||
			ABS(verdict.inlierAreaA - regionShare) > 0.05f || verdict.inlierAreaB <= 0.f ||
			verdict.inlierAreaB >= config.minOverlap) {
			VERBOSE("DenseVerdictTest FAILED: through the fundamental branch a homography warp confident over %.4f of A "
				"(inlier areas %.4f/%.4f, bar %.4f) was %s",
				verdict.confidentAreaA, verdict.inlierAreaA, verdict.inlierAreaB, config.minOverlap,
				verdict.admitted ? "admitted" : "rejected");
			return false;
		}
	}

	// (e) minOverlap 0 admits both (a) and (b): the threshold is the only thing that rejected (b)
	config.minOverlap = 0.f;
	MakeDenseWedgeWarp(imgA, imgB, cells, rx0, rx1, ry0, ry1, warps.ab);
	MakeDenseWedgeWarp(imgB, imgA, cells, rx0, rx1, ry0, ry1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	const bool bAdmittedFull = verdict.admitted && ABS(verdict.inlierAreaB - regionShare) <= 0.05f;
	MakeDenseWedgeWarp(imgB, imgA, cells, bx0, bx1, by0, by1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	if (!bAdmittedFull || !verdict.admitted || ABS(verdict.inlierAreaB - smallShare) > 0.01f) {
		VERBOSE("DenseVerdictTest FAILED: minOverlap 0 admitted the 30%% B side %d and the %.4f one %d",
			(int)bAdmittedFull, verdict.inlierAreaB, (int)verdict.admitted);
		return false;
	}

	VERBOSE("DenseVerdictTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// Guided sparse matching (MatchFeaturesGuided) on synthetic descriptors: the ratio is taken against
// the best descriptor OUTSIDE the search disc, so a lookalike sitting anywhere else in imgB still
// rejects the match, a keypoint whose only close descriptor is inside the disc is accepted, and a
// scale duplicate on top of the true match inside the disc -- the classic ratio-test failure -- no
// longer blocks it; the same inputs give the same matches, in the same order
bool DenseGuidedMatchTest()
{
	TD_TIMER_START();

	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(400), REAL(400), REAL(width)/2, REAL(height)/2));
	for (unsigned i = 0; i < 2; ++i) {
		Image& img = scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i));
		img.cameraID = 0;
		img.pCamera = scene.cameras[0];
	}
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];

	// four queries of imgA, each answering one question, and the keypoints of imgB the warp sends
	// them to: the prediction of query i is its own position plus one fixed offset
	const Point2f offset(20.f, 10.f);
	const float positionsA[4][2] = { {100.f, 100.f}, {200.f, 100.f}, {300.f, 100.f}, {400.f, 100.f} };
	for (const auto& pt : positionsA)
		imgA.keypoints.emplace_back(pt[0], pt[1], 10.f);
	// imgB: the true match of every query at its prediction, an equal-descriptor lookalike of query 0
	// far away from it, a scale duplicate of query 2's match on top of that match, and distractors
	const float positionsB[16][2] = {
		{120.f, 110.f}, {500.f, 300.f}, {220.f, 110.f}, {320.f, 110.f}, {320.f, 110.f}, {420.f, 110.f},
		{ 60.f, 300.f}, {160.f, 320.f}, {260.f, 340.f}, {360.f, 360.f}, {460.f, 380.f}, {560.f, 400.f},
		{ 80.f, 420.f}, {180.f, 440.f}, {280.f, 460.f}, {380.f,  40.f}
	};
	for (const auto& pt : positionsB)
		imgB.keypoints.emplace_back(pt[0], pt[1], 10.f);
	// binary descriptors: one random 32-byte row per query of imgA, copied verbatim onto the imgB
	// keypoints that stand for it, everything else random -- so a "close descriptor" is an exact
	// duplicate and the ratio test is decided by positions alone
	const int descBytes = 32;
	std::mt19937 rng(20260903u);
	imgA.descriptors.create((int)imgA.keypoints.size(), descBytes, CV_8U);
	imgB.descriptors.create((int)imgB.keypoints.size(), descBytes, CV_8U);
	for (int r = 0; r < imgA.descriptors.rows; ++r)
		for (int b = 0; b < descBytes; ++b)
			imgA.descriptors.at<uint8_t>(r, b) = (uint8_t)(rng() & 0xFF);
	for (int r = 0; r < imgB.descriptors.rows; ++r)
		for (int b = 0; b < descBytes; ++b)
			imgB.descriptors.at<uint8_t>(r, b) = (uint8_t)(rng() & 0xFF);
	const int copyDescriptor[6][2] = { {0,0}, {0,1}, {1,2}, {2,3}, {2,4}, {3,5} }; // {query of A, keypoint of B}
	for (const auto& copy : copyDescriptor)
		imgA.descriptors.row(copy[0]).copyTo(imgB.descriptors.row(copy[1]));

	// the predictions the warp would produce, with query 3 left untracked
	std::vector<Point2f> trackedB(imgA.keypoints.size());
	std::vector<uchar> trackStatus(imgA.keypoints.size(), 1);
	FOREACH(i, trackedB)
		trackedB[i] = Point2f(imgA.keypoints[i].pt) + offset;
	trackStatus[3] = 0;
	// two warp cells of a 160-cell grid over imgB, the length the one pass passes in; with no
	// geometry the search is the disc of that radius
	GuidedSearch search;
	search.length = 2.f*(float)MAXF(width, height)/160.f;
	search.halfWidth = 0.5f*(float)MAXF(width, height)/160.f;
	search.sameFeatureDistance = 3.f;

	MatchConfig matchCfg;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.crossCheck = false;      // the guided path refuses a cross-checking matcher (it needs k > 1)
	matchCfg.useFlannMatcher = false; // exact k-NN: the LSH index is approximate, and this test compares exact sets
	PairsMatcher matcher(scene, matchCfg);

	// query 0 is rejected: its lookalike sits outside the disc, so the winner has nothing to beat.
	// query 1 is accepted: its only close descriptor is the one inside the disc.
	// query 2 is accepted despite the duplicate inside its disc, and elects the smaller train index.
	// query 3 is untracked, so it never reaches a disc at all.
	std::vector<DMatch> matches;
	if (MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, matches) != 2 ||
		matches.size() != 2 ||
		matches[0].queryIdx != 1 || matches[0].trainIdx != 2 ||
		matches[1].queryIdx != 2 || matches[1].trainIdx != 3) {
		VERBOSE("DenseGuidedMatchTest FAILED: %u matches, expected (1,2) and (2,3)", (unsigned)matches.size());
		FOREACH(k, matches)
			VERBOSE("DenseGuidedMatchTest:   match %u: (%u,%u)", k, matches[k].queryIdx, matches[k].trainIdx);
		return false;
	}

	// the same query with the same lookalike moved INSIDE its disc is accepted: what rejected it was
	// the lookalike's position, not its existence -- which is exactly the rule the outside-the-disc
	// reference distance encodes
	imgB.keypoints[1].pt = cv::Point2f(121.f, 110.f);
	std::vector<DMatch> movedMatches;
	if (MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, movedMatches) != 3 ||
		movedMatches[0].queryIdx != 0 || movedMatches[0].trainIdx != 0) {
		VERBOSE("DenseGuidedMatchTest FAILED: the lookalike moved into the disc left %u matches, expected 3 starting with (0,0)",
			(unsigned)movedMatches.size());
		return false;
	}
	imgB.keypoints[1].pt = cv::Point2f(positionsB[1][0], positionsB[1][1]);

	// the same inputs give the same matches in the same order, twice over
	std::vector<DMatch> again;
	MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, again);
	if (again.size() != matches.size()) {
		VERBOSE("DenseGuidedMatchTest FAILED: a second run returned %u matches instead of %u",
			(unsigned)again.size(), (unsigned)matches.size());
		return false;
	}
	FOREACH(k, again)
		if (again[k].queryIdx != matches[k].queryIdx || again[k].trainIdx != matches[k].trainIdx) {
			VERBOSE("DenseGuidedMatchTest FAILED: match %u is (%u,%u) and was (%u,%u)",
				k, again[k].queryIdx, again[k].trainIdx, matches[k].queryIdx, matches[k].trainIdx);
			return false;
		}

	VERBOSE("DenseGuidedMatchTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The epipolar band of the guided sparse matching: the search region is a band along the epipolar
// line of the query, centred on the warp's prediction, a rival off the line no longer takes the
// disc from the true match, an ambiguous rival ON the line refuses the match, a same-feature
// duplicate does not count as a rival, every winner answers to the reference outside its band, and
// the search falls back to the disc without a geometry or near the epipole.
bool DenseGuidedBandTest()
{
	TD_TIMER_START();

	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(400), REAL(400), REAL(width)/2, REAL(height)/2));
	for (unsigned i = 0; i < 2; ++i) {
		Image& img = scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i));
		img.cameraID = 0;
		img.pCamera = scene.cameras[0];
	}
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];

	// every query of imgA sits on the row y = 100, and every geometry below puts the epipole of imgB
	// on that same row, so every epipolar line is horizontal and "along" is x, "across" is y; the
	// prediction of query i is its own position plus one fixed offset, the band is centred on it
	const Point2f offset(20.f, 10.f);
	const float positionsA[6][2] = { {100.f, 100.f}, {200.f, 100.f}, {300.f, 100.f}, {400.f, 100.f}, {500.f, 100.f}, {600.f, 100.f} };
	for (const auto& pt : positionsA)
		imgA.keypoints.emplace_back(pt[0], pt[1], 10.f);
	// imgB, per query: 0) its true match at the prediction, an impostor 5 px ACROSS the line (inside
	// the two-cell disc, outside the half-cell band) and a rival 5 px ALONG it; 1) its true match and
	// an equal-descriptor rival 6 px along the line; 2) its true match and a same-feature duplicate
	// 2 px along; 3) its true match alone, with a lookalike far away; 4) its true match, a rival 6 px
	// along the line and a lookalike far away; 5) its true match and a rival 6 px along the line, no
	// lookalike anywhere; then distractors
	const float positionsB[19][2] = {
		{120.f, 110.f}, {120.f, 115.f}, {125.f, 110.f},
		{220.f, 110.f}, {226.f, 110.f},
		{320.f, 110.f}, {322.f, 110.f},
		{420.f, 110.f}, {500.f, 300.f},
		{520.f, 110.f}, {526.f, 110.f}, {300.f, 400.f},
		{620.f, 110.f}, {626.f, 110.f},
		{ 60.f, 300.f}, {160.f, 320.f}, {260.f, 340.f}, {360.f, 360.f}, {380.f,  40.f}
	};
	for (const auto& pt : positionsB)
		imgB.keypoints.emplace_back(pt[0], pt[1], 10.f);
	// binary descriptors: one random 32-byte row per query, copied onto the imgB keypoints that stand
	// for it (query 0's true match one bit off, so its impostor is the closer descriptor), the rest
	// random
	const int descBytes = 32;
	std::mt19937 rng(20260916u);
	imgA.descriptors.create((int)imgA.keypoints.size(), descBytes, CV_8U);
	imgB.descriptors.create((int)imgB.keypoints.size(), descBytes, CV_8U);
	for (int r = 0; r < imgA.descriptors.rows; ++r)
		for (int b = 0; b < descBytes; ++b)
			imgA.descriptors.at<uint8_t>(r, b) = (uint8_t)(rng() & 0xFF);
	for (int r = 0; r < imgB.descriptors.rows; ++r)
		for (int b = 0; b < descBytes; ++b)
			imgB.descriptors.at<uint8_t>(r, b) = (uint8_t)(rng() & 0xFF);
	const int copyDescriptor[11][2] = { {0,0}, {0,1}, {1,3}, {1,4}, {2,5}, {2,6}, {3,7}, {3,8}, {4,9}, {4,11}, {5,12} }; // {query of A, keypoint of B}
	for (const auto& copy : copyDescriptor)
		imgA.descriptors.row(copy[0]).copyTo(imgB.descriptors.row(copy[1]));
	imgB.descriptors.at<uint8_t>(0, 0) ^= 0x01; // the true match of query 0: Hamming distance 1

	std::vector<Point2f> trackedB(imgA.keypoints.size());
	std::vector<uchar> trackStatus(imgA.keypoints.size(), 1);
	FOREACH(i, trackedB)
		trackedB[i] = Point2f(imgA.keypoints[i].pt) + offset;

	MatchConfig matchCfg;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.crossCheck = false;
	matchCfg.useFlannMatcher = false;
	PairsMatcher matcher(scene, matchCfg);

	// F = [e]x: the epipolar line of a point of imgA is the line through the epipole e and the same
	// coordinates in imgB, and F^T e = 0
	const auto Fundamental = [](float ex, float ey) {
		Matrix3x3 F;
		F(0,0) = 0;   F(0,1) = -1;  F(0,2) = ey;
		F(1,0) = 1;   F(1,1) = 0;   F(1,2) = -ex;
		F(2,0) = -ey; F(2,1) = ex;  F(2,2) = 0;
		return F;
	};
	// two warp cells long and half a cell wide on a 160-cell grid over imgB, as the one pass sizes it
	GuidedSearch search;
	search.length = 2.f*(float)MAXF(width, height)/160.f;
	search.halfWidth = 0.5f*(float)MAXF(width, height)/160.f;
	search.F = Fundamental(2000.f, 100.f); // the epipole far off the frame: every band is horizontal
	search.sameFeatureDistance = 3.f;
	const auto Report = [](const char* what, const std::vector<DMatch>& matches) {
		VERBOSE("DenseGuidedBandTest FAILED: %s (%u matches)", what, (unsigned)matches.size());
		FOREACH(k, matches)
			VERBOSE("DenseGuidedBandTest:   match %u: (%u,%u)", k, matches[k].queryIdx, matches[k].trainIdx);
	};

	// query 0 is refused: its impostor sits off the line, never a candidate, but closer in appearance
	// than its true match, and the reference outside the band sees it.
	// query 1 is refused: its rival on the line is exactly as close, the match is ambiguous.
	// query 2 matches, electing the smaller index of its two same-feature duplicates; its closest
	// keypoint outside the band is random.
	// query 3 is refused: alone in its band, it has to beat the far lookalike and cannot.
	// query 4 is refused: past its random rival, it has to beat the far lookalike and cannot.
	// query 5 matches: its rival and its closest keypoint outside the band are random.
	std::vector<DMatch> matches;
	if (MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, matches) != 2 ||
		matches.size() != 2 ||
		matches[0].queryIdx != 2 || matches[0].trainIdx != 5 ||
		matches[1].queryIdx != 5 || matches[1].trainIdx != 12) {
		Report("band: expected (2,5) and (5,12)", matches);
		return false;
	}

	// without a geometry the search is the disc: query 0's impostor is a candidate again and wins
	// (its true match becomes the in-disc rival the impostor beats by the ratio); everything else is
	// as with the band, the disc holding the same keypoints along the line
	search.F.reset();
	std::vector<DMatch> discMatches;
	if (MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, discMatches) != 3 ||
		discMatches.size() != 3 ||
		discMatches[0].queryIdx != 0 || discMatches[0].trainIdx != 1 ||
		discMatches[1].queryIdx != 2 || discMatches[1].trainIdx != 5 ||
		discMatches[2].queryIdx != 5 || discMatches[2].trainIdx != 12) {
		Report("disc: expected (0,1), (2,5) and (5,12)", discMatches);
		return false;
	}

	// a prediction within two band lengths of the epipole has no usable line direction and searches
	// the disc: the epipole 14 px from query 0's prediction turns that query's search into the disc
	// (the impostor wins), and leaves the other queries' bands as they were
	search.F = Fundamental(130.f, 100.f);
	std::vector<DMatch> epipoleMatches;
	if (MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, epipoleMatches) != 3 ||
		epipoleMatches.size() != 3 ||
		epipoleMatches[0].queryIdx != 0 || epipoleMatches[0].trainIdx != 1 ||
		epipoleMatches[1].queryIdx != 2 || epipoleMatches[1].trainIdx != 5) {
		Report("near the epipole: expected (0,1), (2,5) and (5,12)", epipoleMatches);
		return false;
	}

	// the same inputs give the same matches in the same order, twice over
	search.F = Fundamental(2000.f, 100.f);
	std::vector<DMatch> again;
	MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, again);
	if (again.size() != matches.size()) {
		VERBOSE("DenseGuidedBandTest FAILED: a second run returned %u matches instead of %u",
			(unsigned)again.size(), (unsigned)matches.size());
		return false;
	}
	FOREACH(k, again)
		if (again[k].queryIdx != matches[k].queryIdx || again[k].trainIdx != matches[k].trainIdx) {
			VERBOSE("DenseGuidedBandTest FAILED: match %u is (%u,%u) and was (%u,%u)",
				k, again[k].queryIdx, again[k].trainIdx, matches[k].queryIdx, matches[k].trainIdx);
			return false;
		}


	VERBOSE("DenseGuidedBandTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool DenseAssemblyTest()
{
	TD_TIMER_START();

	Scene scene;
	MakeDenseWedgeScene(scene);
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	const int cells = 64;
	const int rx0 = 14, rx1 = 50, ry0 = 13, ry1 = 47;

	// the verdict of an admitted pair: every cell of the region, projected through the wedge, with a
	// confidence of its own
	PairVerdict verdict;
	verdict.admitted = true;
	for (int y = ry0; y < ry1; ++y) {
		for (int x = rx0; x < rx1; ++x) {
			const Point2f ptA(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), imgA.GetSize()));
			Point2f ptB;
			if (!DenseWedgeCorrespondence(imgA, imgB, ptA, ptB))
				continue;
			verdict.inliersA.push_back(ptA);
			verdict.inliersB.push_back(ptB);
			verdict.confidences.push_back(0.25f + 0.5f*(float)(x + y)/(float)(2*(cells - 1)));
		}
	}
	const size_t numInlierCells = verdict.inliersA.size();
	verdict.confidentAreaA = verdict.confidentAreaB =
		verdict.inlierAreaA = verdict.inlierAreaB = (float)numInlierCells/(float)(cells*cells);
	if (numInlierCells != (size_t)((rx1-rx0)*(ry1-ry0))) {
		VERBOSE("DenseAssemblyTest FAILED: %u of the %u region cells have a correspondence",
			(unsigned)numInlierCells, (unsigned)((rx1-rx0)*(ry1-ry0)));
		return false;
	}

	// the guided matches: 80 exact descriptor correspondences on cell centres of the region, then 6
	// the geometry contradicts (their B side displaced across the epipolar lines, which run along
	// the baseline direction (0.4, 0.1) here). Both kinds occupy their cell, so the complementary
	// fill has to leave all 86 of them out.
	const size_t numSparse = 80, numOutliers = 6, numGuided = numSparse + numOutliers;
	std::vector<DMatch> guided;
	for (size_t k = 0; k < numInlierCells && guided.size() < numGuided; k += 14) {
		const uint32_t idx = (uint32_t)imgA.keypoints.size();
		imgA.keypoints.emplace_back(verdict.inliersA[k].x, verdict.inliersA[k].y, 10.f);
		imgB.keypoints.emplace_back(verdict.inliersB[k].x,
			verdict.inliersB[k].y + (guided.size() >= numSparse ? 40.f : 0.f), 10.f);
		guided.emplace_back(idx, idx);
	}
	if (guided.size() != numGuided) {
		VERBOSE("DenseAssemblyTest FAILED: the region offers %u cells, too few for %u guided matches",
			(unsigned)numInlierCells, (unsigned)numGuided);
		return false;
	}

	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	DenseMatchConfig config; // minConfidence 0.1, denseMatchesPerFrame 2000 (a density per full frame of overlap)
	// the geometry the verdict handed over, 3 degrees off the truth, so that a pair carrying the
	// union fit's geometry is told apart from one that kept the verdict's
	const REAL tilt = D2R(REAL(3));
	Pose3D poseVerdict(poseGT);
	poseVerdict.R = poseGT.R * Matrix3x3(COS(tilt), 0, SIN(tilt), 0, 1, 0, -SIN(tilt), 0, COS(tilt));
	const auto ArmVerdictGeometry = [&](ImagePair& pair, const Pose3D& pose) {
		pair.relativePose = pose;
		pair.E = ImagePair::ComposeEssentialMatrix(pose);
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), imgA.GetK(), imgB.GetK());
	};
	const auto PoseErrorFromGT = [&](const Pose3D& pose, REAL& angleErr, REAL& tSim) {
		angleErr = R2D(ACOS(ComputeAngle(pose.R, poseGT.R)));
		tSim = ABS(normalized(pose.GetT()).dot(normalized(poseGT.GetT())));
	};

	// guided u dense on an exact geometry: the 80 sound guided matches become the sparse segment,
	// the 6 the geometry contradicts the pair's outliers, and the fill draws one point per bucket of
	// the fixed-pitch grid (DenseFillGridSide) that holds a region cell and no guided match -- not one
	// point per uncovered cell, now that the pitch no longer shrinks to fit this one pair's overlap
	ImagePair pair(0, 1);
	ArmVerdictGeometry(pair, poseVerdict);
	DenseMatches dense;
	if (!AssemblePairDense(matcher, imgA, imgB, verdict, guided, config, cells, pair, dense)) {
		VERBOSE("DenseAssemblyTest FAILED: an admitted pair with %u guided matches was not assembled", (unsigned)guided.size());
		return false;
	}
	if (pair.matches.size() != numSparse || (size_t)pair.numFilteredInliers != numSparse ||
		pair.outlierMatches.size() != numOutliers || pair.numDenseInliers != 0) {
		VERBOSE("DenseAssemblyTest FAILED: the sparse segment holds %u of %u matches (%d filtered, %u outliers, %d dense)",
			(unsigned)pair.matches.size(), (unsigned)numGuided, pair.numFilteredInliers,
			(unsigned)pair.outlierMatches.size(), pair.numDenseInliers);
		return false;
	}
	FOREACH(k, pair.matches)
		if (pair.matches[k].queryIdx != k || pair.matches[k].trainIdx != k) {
			VERBOSE("DenseAssemblyTest FAILED: sparse match %u is (%u,%u), so the segment lost the guided order",
				k, pair.matches[k].queryIdx, pair.matches[k].trainIdx);
			return false;
		}
	// the region cell each verdict/guided index falls on is known without a pixel round-trip: the
	// region loop above ran y then x with no skipped cell (checked at numInlierCells above), and the
	// guided loop took every 14th of those in the same order
	// this calls the same production function the code under test calls for its pitch, so unlike the
	// bucket-mapping arithmetic just below it does not independently re-verify the pitch formula
	const int gridSide = DenseFillGridSide(config.denseMatchesPerFrame, cells);
	const int regionWidth = rx1 - rx0;
	const auto RegionBucket = [&](size_t k) {
		const int y = ry0 + (int)(k/(size_t)regionWidth), x = rx0 + (int)(k%(size_t)regionWidth);
		return (size_t)(y*gridSide/cells)*gridSide + x*gridSide/cells;
	};
	std::vector<bool> hasCandidate((size_t)gridSide*gridSide, false);
	for (size_t k = 0; k < numInlierCells; ++k)
		hasCandidate[RegionBucket(k)] = true;
	size_t numCandidateBuckets = 0; // the dense-only case below draws one point per bucket here, unfiltered
	FOREACH(b, hasCandidate)
		if (hasCandidate[b])
			++numCandidateBuckets;
	std::vector<bool> guidedBucket((size_t)gridSide*gridSide, false);
	FOREACH(g, guided)
		guidedBucket[RegionBucket(g*14)] = true;
	size_t expectedDense = 0;
	FOREACH(b, hasCandidate)
		if (hasCandidate[b] && !guidedBucket[b])
			++expectedDense;
	if (dense.pointsA.size() != expectedDense ||
		dense.pointsB.size() != dense.pointsA.size() || dense.confidences.size() != dense.pointsA.size()) {
		VERBOSE("DenseAssemblyTest FAILED: the fill drew %u/%u points with %u confidences, expected %u",
			(unsigned)dense.pointsA.size(), (unsigned)dense.pointsB.size(), (unsigned)dense.confidences.size(),
			(unsigned)expectedDense);
		return false;
	}
	// every drawn point is one of the verdict's cells, carries that cell's own confidence, and sits
	// on no cell a guided match occupies
	{
		const cv::Size gridSize(cells, cells);
		const auto CellOf = [&](const Point2f& ptA) {
			const Point2f cell(PixelToCell(ptA, imgA.GetSize(), gridSize));
			return ROUND2INT(cell.y)*cells + ROUND2INT(cell.x);
		};
		std::vector<int> inlierOfCell((size_t)cells*cells, -1);
		FOREACH(k, verdict.inliersA)
			inlierOfCell[CellOf(verdict.inliersA[k])] = (int)k;
		std::vector<bool> occupied((size_t)cells*cells, false);
		for (const DMatch& match : guided)
			occupied[CellOf(Point2f(imgA.keypoints[match.queryIdx].pt))] = true;
		FOREACH(k, dense.pointsA) {
			const int cell = CellOf(dense.pointsA[k]);
			if (inlierOfCell[cell] < 0 || occupied[cell] ||
				dense.confidences[k] != verdict.confidences[inlierOfCell[cell]]) {
				VERBOSE("DenseAssemblyTest FAILED: dense point %u at cell (%d,%d) is %s and carries confidence %g",
					k, cell%cells, cell/cells, inlierOfCell[cell] < 0 ? "not a verdict cell" :
					(occupied[cell] ? "on a guided match" : "mislabelled"), dense.confidences[k]);
				return false;
			}
		}
	}
	// ONE geometry for the pair, and it is the union fit's: the verdict handed over a pose 3 degrees
	// off the two cameras and the assembled pair carries the right one
	REAL angleErr, tSim;
	if (!pair.relativePose.has_value()) {
		VERBOSE("DenseAssemblyTest FAILED: the assembled pair carries no relative pose");
		return false;
	}
	PoseErrorFromGT(pair.relativePose.value(), angleErr, tSim);
	if (angleErr > REAL(0.05) || tSim < REAL(0.9999) || !pair.F.has_value() || !pair.E.has_value()) {
		VERBOSE("DenseAssemblyTest FAILED: the assembled pose is %.4f deg and %.6f off the two cameras (F %d, E %d)",
			angleErr, tSim, (int)pair.F.has_value(), (int)pair.E.has_value());
		return false;
	}

	// no guided match at all: the pair is still assembled, its dense segment its whole evidence, and
	// the fill draws one point per bucket of the fixed-pitch grid that holds a region cell -- nothing
	// left to strike out this time, but still a bucket count, not a cell count. The region's edge
	// does not tile the fixed pitch evenly, so a border bucket counts as used from just a sliver of
	// region inside it, and the resulting bucket count runs a little over the per-frame density --
	// so DenseFillCeiling actually binds here, unlike in the guided u dense case above where the
	// guided matches' own strike-outs already keep the draw under it.
	const size_t expectedDenseOnly = MINF(numCandidateBuckets, (size_t)DenseFillCeiling(config, verdict));
	ImagePair pairDense(0, 1);
	ArmVerdictGeometry(pairDense, poseVerdict);
	DenseMatches denseOnly;
	if (!AssemblePairDense(matcher, imgA, imgB, verdict, std::vector<DMatch>(), config, cells, pairDense, denseOnly) ||
		!pairDense.matches.empty() || pairDense.numFilteredInliers != 0 ||
		denseOnly.pointsA.size() != expectedDenseOnly) {
		VERBOSE("DenseAssemblyTest FAILED: a pair with no guided match holds %u sparse (%d filtered) and %u dense correspondences, expected 0 and %u",
			(unsigned)pairDense.matches.size(), pairDense.numFilteredInliers,
			(unsigned)denseOnly.pointsA.size(), (unsigned)expectedDenseOnly);
		return false;
	}
	if (!pairDense.relativePose.has_value()) {
		VERBOSE("DenseAssemblyTest FAILED: a dense-only pair carries no relative pose");
		return false;
	}
	PoseErrorFromGT(pairDense.relativePose.value(), angleErr, tSim);
	if (angleErr > REAL(0.05) || tSim < REAL(0.9999)) {
		VERBOSE("DenseAssemblyTest FAILED: the dense-only pose is %.4f deg and %.6f off the two cameras", angleErr, tSim);
		return false;
	}

	// a verdict whose every fourth cell lands 3 px across the epipolar lines in B, the imprecision
	// of a coarse warp: held to the pair's own epipolar bar (1.5 px here, denseEpipolarErrorFactor 1)
	// the dense segment keeps only the exact cells
	{
		PairVerdict verdictOff(verdict);
		const float acrossLen = SQRT(0.1f*0.1f + 0.4f*0.4f);
		const Point2f across(-0.1f/acrossLen*3.f, 0.4f/acrossLen*3.f); // perpendicular to the baseline (0.4, 0.1)
		for (size_t k = 0; k < verdictOff.inliersB.size(); k += 4)
			verdictOff.inliersB[k] = Point2f(verdictOff.inliersB[k].x + across.x, verdictOff.inliersB[k].y + across.y);
		MatchConfig matchCfgTight(matchCfg);
		matchCfgTight.maxEpipolarError = 1.5f;
		PairsMatcher matcherTight(scene, matchCfgTight);
		const auto CountKept = [&](float factor, unsigned& numKept, unsigned& numKeptDisplaced) {
			DenseMatchConfig configBar(config);
			configBar.denseEpipolarErrorFactor = factor;
			ImagePair pairOff(0, 1);
			ArmVerdictGeometry(pairOff, poseVerdict);
			DenseMatches denseOff;
			if (!AssemblePairDense(matcherTight, imgA, imgB, verdictOff, std::vector<DMatch>(), configBar, cells, pairOff, denseOff))
				return false;
			numKept = (unsigned)denseOff.pointsA.size();
			numKeptDisplaced = 0;
			FOREACH(k, denseOff.pointsA) {
				Point2f ptB;
				if (!DenseWedgeCorrespondence(imgA, imgB, denseOff.pointsA[k], ptB))
					return false;
				if (SQRT(SQUARE(denseOff.pointsB[k].x - ptB.x) + SQUARE(denseOff.pointsB[k].y - ptB.y)) > 1.5f)
					++numKeptDisplaced;
			}
			return true;
		};
		unsigned keptBar, keptBarDisplaced;
		if (!CountKept(1.f, keptBar, keptBarDisplaced)) {
			VERBOSE("DenseAssemblyTest FAILED: the pair whose every fourth cell is displaced was not assembled");
			return false;
		}
		if (keptBar == 0 || keptBarDisplaced != 0 || keptBar > verdictOff.inliersB.size() - verdictOff.inliersB.size()/4) {
			VERBOSE("DenseAssemblyTest FAILED: the dense segment keeps %u cells (%u displaced) of %u on the epipolar bar",
				keptBar, keptBarDisplaced, (unsigned)verdictOff.inliersB.size());
			return false;
		}
		DEBUG("DenseAssemblyTest: the dense segment keeps %u of %u cells on the epipolar bar, none of the displaced",
			keptBar, (unsigned)verdictOff.inliersB.size());
	}

	// a fill too small for a fit of its own (under the estimator's 8 correspondences): the verdict's
	// geometry stands, unchanged, and the pair is stored on it. The inlier areas stay the parent
	// verdict's -- only the SAMPLE of inlier cells shrinks here, to drive the union fit below the
	// estimator's minimum -- so DenseFillCeiling does not also clamp the fill down from the 6 cells
	// this case means to exercise.
	PairVerdict tinyVerdict;
	tinyVerdict.admitted = true;
	tinyVerdict.inlierAreaA = verdict.inlierAreaA;
	tinyVerdict.inlierAreaB = verdict.inlierAreaB;
	for (size_t k = 0; k < 6; ++k) {
		tinyVerdict.inliersA.push_back(verdict.inliersA[k*97]);
		tinyVerdict.inliersB.push_back(verdict.inliersB[k*97]);
		tinyVerdict.confidences.push_back(verdict.confidences[k*97]);
	}
	ImagePair pairTiny(0, 1);
	ArmVerdictGeometry(pairTiny, poseGT); // exact, so its own cells survive the classification
	const Matrix3x3 Fverdict(pairTiny.F.value());
	DenseMatches denseTiny;
	if (!AssemblePairDense(matcher, imgA, imgB, tinyVerdict, std::vector<DMatch>(), config, cells, pairTiny, denseTiny) ||
		denseTiny.pointsA.size() != tinyVerdict.inliersA.size()) {
		VERBOSE("DenseAssemblyTest FAILED: a %u-cell fill was not assembled (%u dense correspondences)",
			(unsigned)tinyVerdict.inliersA.size(), (unsigned)denseTiny.pointsA.size());
		return false;
	}
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			if (pairTiny.F.value()(r, c) != Fverdict(r, c) ||
				pairTiny.relativePose->R(r, c) != poseGT.R(r, c)) {
				VERBOSE("DenseAssemblyTest FAILED: a fill too small to fit still moved the pair's geometry at (%d,%d)", r, c);
				return false;
			}

	// the store: the dense keypoints of a pair land past each image's described prefix, and the
	// indices they get depend only on how many keypoints the two images already carry -- which is
	// why the pass has to store serially, in pair order. The first pair also carries one match past
	// its filtered count -- a RANSAC inlier the strict filter rejected, which lives in `matches`
	// after the sparse segment -- so that where the dense block is inserted is pinned: a fill pushed
	// onto the end of `matches` would land outside the track-forming prefix BuildTracks reads
	const auto RunStore = [cells](Scene& out) {
		out.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(400), REAL(400), REAL(320), REAL(240)));
		for (unsigned i = 0; i < 3; ++i) {
			Image& img = out.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", i));
			img.cameraID = 0;
			img.pCamera = out.cameras[0];
			for (unsigned k = 0; k < 5; ++k)
				img.keypoints.emplace_back(10.f*(float)k, 20.f*(float)i + 5.f, 10.f);
		}
		std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
		for (unsigned j = 1; j < 3; ++j) {
			ImagePair pair(0, j);
			for (uint32_t m = 0; m < j; ++m)
				pair.matches.emplace_back(m, m);
			pair.numFilteredInliers = (int)j;
			if (j == 1)
				pair.matches.emplace_back(4, 4); // the rejected tail, past the filtered count
			DenseMatches dense;
			for (unsigned d = 0; d < 5 - j; ++d) {
				dense.pointsA.emplace_back(100.f + 10.f*(float)d, 100.f + 10.f*(float)j);
				dense.pointsB.emplace_back(110.f + 10.f*(float)d, 100.f + 10.f*(float)j);
				dense.confidences.push_back(0.5f + 0.1f*(float)d);
			}
			StorePairDense(out, pairIndexMap, std::move(pair), dense, cells);
		}
		return pairIndexMap;
	};
	Scene storeScene;
	const std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap(RunStore(storeScene));
	if (storeScene.pairs.size() != 2 || pairIndexMap.size() != 2 ||
		pairIndexMap.at(PairIdx(0, 1).idx) != 0 || pairIndexMap.at(PairIdx(0, 2).idx) != 1) {
		VERBOSE("DenseAssemblyTest FAILED: the store left %u pairs and %u index entries",
			(unsigned)storeScene.pairs.size(), (unsigned)pairIndexMap.size());
		return false;
	}
	// image 0 took both fills (4 then 3), images 1 and 2 one each, all past the described 5
	const unsigned expectedKeypoints[3] = { 5 + 4 + 3, 5 + 4, 5 + 3 };
	for (unsigned i = 0; i < 3; ++i) {
		const Image& img = storeScene.images[i];
		if (img.NumDescribedKeypoints() != 5 || img.keypoints.size() != expectedKeypoints[i] ||
			img.IsDenseKeypoint(4) || !img.IsDenseKeypoint(5)) {
			VERBOSE("DenseAssemblyTest FAILED: image %u carries %u keypoints past a described prefix of %u, expected %u",
				i, (unsigned)img.keypoints.size(), img.NumDescribedKeypoints(), expectedKeypoints[i]);
			return false;
		}
	}
	// the dense segment of each pair names the keypoints the append handed out: image 0 continues
	// past its first fill, image 2 starts at its own prefix
	const ImagePair& pair01 = storeScene.pairs[0];
	const ImagePair& pair02 = storeScene.pairs[1];
	if (pair01.numFilteredInliers != 1 || pair01.numDenseInliers != 4 || pair01.matches.size() != 6 ||
		pair02.numFilteredInliers != 2 || pair02.numDenseInliers != 3 || pair02.matches.size() != 5) {
		VERBOSE("DenseAssemblyTest FAILED: the stored pairs partition %u/%u matches as %d+%d and %d+%d",
			(unsigned)pair01.matches.size(), (unsigned)pair02.matches.size(),
			pair01.numFilteredInliers, pair01.numDenseInliers, pair02.numFilteredInliers, pair02.numDenseInliers);
		return false;
	}
	for (unsigned d = 0; d < 4; ++d)
		if (pair01.matches[1 + d].queryIdx != 5 + d || pair01.matches[1 + d].trainIdx != 5 + d) {
			VERBOSE("DenseAssemblyTest FAILED: the first pair's dense match %u is (%u,%u), expected (%u,%u)",
				d, pair01.matches[1 + d].queryIdx, pair01.matches[1 + d].trainIdx, 5 + d, 5 + d);
			return false;
		}
	for (unsigned d = 0; d < 3; ++d)
		if (pair02.matches[2 + d].queryIdx != 9 + d || pair02.matches[2 + d].trainIdx != 5 + d) {
			VERBOSE("DenseAssemblyTest FAILED: the second pair's dense match %u is (%u,%u), expected (%u,%u)",
				d, pair02.matches[2 + d].queryIdx, pair02.matches[2 + d].trainIdx, 9 + d, 5 + d);
			return false;
		}
	// and the rejected match the first pair carried stays behind the whole dense block: the fill is
	// inserted into the track-forming prefix, never appended past the segments that follow it
	const unsigned idxRejected = (unsigned)pair01.matches.size() - 1;
	if (pair01.matches[idxRejected].queryIdx != 4 || pair01.matches[idxRejected].trainIdx != 4) {
		VERBOSE("DenseAssemblyTest FAILED: match %u of the first pair is (%u,%u), not the rejected (4,4), so the dense "
			"block did not land before it",
			idxRejected, pair01.matches[idxRejected].queryIdx, pair01.matches[idxRejected].trainIdx);
		return false;
	}
	// the same two stores over again label everything identically
	Scene storeSceneAgain;
	RunStore(storeSceneAgain);
	FOREACH(i, storeScene.images) {
		const Image& img = storeScene.images[i];
		const Image& imgAgain = storeSceneAgain.images[i];
		if (img.keypoints.size() != imgAgain.keypoints.size() ||
			img.NumDescribedKeypoints() != imgAgain.NumDescribedKeypoints()) {
			VERBOSE("DenseAssemblyTest FAILED: a second store run gave image %u %u keypoints instead of %u",
				i, (unsigned)imgAgain.keypoints.size(), (unsigned)img.keypoints.size());
			return false;
		}
		FOREACH(k, img.keypoints)
			if (img.keypoints[k].pt != imgAgain.keypoints[k].pt ||
				img.keypoints[k].response != imgAgain.keypoints[k].response) {
				VERBOSE("DenseAssemblyTest FAILED: a second store run moved keypoint %u of image %u", k, i);
				return false;
			}
	}
	FOREACH(p, storeScene.pairs) {
		const ImagePair& lhs = storeScene.pairs[p];
		const ImagePair& rhs = storeSceneAgain.pairs[p];
		if (lhs.matches.size() != rhs.matches.size() || lhs.numFilteredInliers != rhs.numFilteredInliers ||
			lhs.numDenseInliers != rhs.numDenseInliers) {
			VERBOSE("DenseAssemblyTest FAILED: a second store run partitioned pair %u differently", p);
			return false;
		}
		FOREACH(k, lhs.matches)
			if (lhs.matches[k].queryIdx != rhs.matches[k].queryIdx || lhs.matches[k].trainIdx != rhs.matches[k].trainIdx) {
				VERBOSE("DenseAssemblyTest FAILED: a second store run relabelled match %u of pair %u", k, p);
				return false;
			}
	}

	VERBOSE("DenseAssemblyTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The pair's relative pose leans on its sparse matches: the dense fill of an assembled pair is
// displaced 2 px across the epipolar lines everywhere, the systematic error of a coarse warp, while
// the guided matches are exact. Both kinds pass the 4 px bar and the union fit follows the far more
// numerous dense matches; the weighted refinement that closes the assembly gives each dense
// correspondence DenseMatchConfig::denseFitWeight against a sparse one's 1 and brings the pose back to the
// sparse matches, where the same assembly with the dense matches at full weight stays on the bias.
bool DenseFitWeightTest()
{
	TD_TIMER_START();

	Scene scene;
	MakeDenseWedgeScene(scene);
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	const int cells = 64;
	const int rx0 = 14, rx1 = 50, ry0 = 13, ry1 = 47;
	// across the epipolar lines, which run along the baseline direction (0.4, 0.1) here
	const Point2f across(normalized(Point2f(-0.1f, 0.4f)));
	const float bias = 2.f;
	PairVerdict verdict;
	verdict.admitted = true;
	std::vector<Point2f> exactB;
	for (int y = ry0; y < ry1; ++y) {
		for (int x = rx0; x < rx1; ++x) {
			const Point2f ptA(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), imgA.GetSize()));
			Point2f ptB;
			if (!DenseWedgeCorrespondence(imgA, imgB, ptA, ptB))
				continue;
			verdict.inliersA.push_back(ptA);
			verdict.inliersB.push_back(ptB + across*bias);
			verdict.confidences.push_back(0.5f);
			exactB.push_back(ptB);
		}
	}
	verdict.confidentAreaA = verdict.confidentAreaB = verdict.inlierAreaA = verdict.inlierAreaB =
		(float)verdict.inliersA.size()/(float)(cells*cells);
	// 60 exact guided matches spread over the region
	std::vector<DMatch> guided;
	for (size_t k = 0; k < verdict.inliersA.size() && guided.size() < 60; k += 20) {
		const uint32_t idx = (uint32_t)imgA.keypoints.size();
		imgA.keypoints.emplace_back(verdict.inliersA[k].x, verdict.inliersA[k].y, 10.f);
		imgB.keypoints.emplace_back(exactB[k].x, exactB[k].y, 10.f);
		guided.emplace_back(idx, idx);
	}
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	const auto Assemble = [&](float denseFitWeight, REAL& angleErr, size_t& numDense) {
		DenseMatchConfig config;
		config.denseFitWeight = denseFitWeight;
		ImagePair pair(0, 1);
		pair.relativePose = poseGT;
		pair.E = ImagePair::ComposeEssentialMatrix(poseGT);
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), imgA.GetK(), imgB.GetK());
		DenseMatches dense;
		if (!AssemblePairDense(matcher, imgA, imgB, verdict, guided, config, cells, pair, dense) || !pair.relativePose.has_value())
			return false;
		angleErr = R2D(ACOS(ComputeAngle(pair.relativePose->R, poseGT.R)));
		numDense = dense.pointsA.size();
		return true;
	};
	REAL errFull, errWeighted;
	size_t numDenseFull, numDenseWeighted;
	if (!Assemble(1.f, errFull, numDenseFull) || !Assemble(DenseMatchConfig().denseFitWeight, errWeighted, numDenseWeighted)) {
		VERBOSE("DenseFitWeightTest FAILED: the biased pair was not assembled with a relative pose");
		return false;
	}
	// the premise: the dense matches outnumber the sparse ones and at full weight pull the pose
	if (numDenseFull < 4*guided.size() || errFull < REAL(0.05)) {
		VERBOSE("DenseFitWeightTest FAILED: premise not met: %u dense against %u sparse, %.4f deg at full weight",
			(unsigned)numDenseFull, (unsigned)guided.size(), errFull);
		return false;
	}
	if (errWeighted > REAL(0.25)*errFull) {
		VERBOSE("DenseFitWeightTest FAILED: the weighted refinement leaves the pose %.4f deg off, %.4f deg at full weight",
			errWeighted, errFull);
		return false;
	}
	VERBOSE("DenseFitWeightTest PASSED: %.4f deg off with the dense matches at full weight, %.4f deg at %g (%u dense, %u sparse) (%s)",
		errFull, errWeighted, DenseMatchConfig().denseFitWeight, (unsigned)numDenseWeighted, (unsigned)guided.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

// The scene of PlanarTwinSwitchTest: one 1000x1000 pinhole camera at f 500 that trusts its intrinsics
// (the calibrated branch: the twin's polish needs it), shared by images looking down at the floor
// z = 0, each given by its centre and two tilts (degrees about x, then about y)
struct PlanarTwinView { REAL x, y, z, tiltX, tiltY; };
static void MakePlanarTwinScene(Scene& scene, const std::vector<PlanarTwinView>& views)
{
	PinholeCamera* const camera = new PinholeCamera(cv::Size(1000, 1000), REAL(500), REAL(500), REAL(500), REAL(500));
	camera->trustIntrinsics = true;
	scene.cameras.emplace_back(camera);
	FOREACH(i, views) {
		const PlanarTwinView& v = views[i];
		const REAL ax = D2R(v.tiltX), ay = D2R(v.tiltY);
		const Matrix3x3 Rx(1, 0, 0, 0, COS(ax), -SIN(ax), 0, SIN(ax), COS(ax));
		const Matrix3x3 Ry(COS(ay), 0, SIN(ay), 0, 1, 0, -SIN(ay), 0, COS(ay));
		const Matrix3x3 down(1, 0, 0, 0, -1, 0, 0, 0, -1); // the camera's z axis along the world's -z
		Pose3D pose;
		pose.R = Rx*Ry*down;
		pose.C = Point3(v.x, v.y, v.z);
		scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", (unsigned)i), pose, (IIndex)0, scene.cameras[0]);
	}
}

// Keypoint k of the image = the projection of points[k]; every keypoint described (the image counts as
// a RoMa2 one, with no dense keypoint yet). False when a point falls behind the image or outside it.
static bool AddPlanarTwinKeypoints(Image& img, const std::vector<Point3>& points)
{
	for (const Point3& X : points) {
		const auto [proj, valid] = img.ProjectPoint(X);
		const Point2f pt((float)proj.x, (float)proj.y);
		if (!valid || !Image8U::isInside(pt, img.GetSize()))
			return false;
		img.keypoints.emplace_back(pt.x, pt.y, 10.f);
	}
	img.numDescribedKeypoints = (uint32_t)img.keypoints.size();
	return true;
}

// A verified pair holding `pose` (with its E and F) and the first numMatches keypoints of both
// images matched one to one as sparse inliers; with no match it is a pose alone, a neighbour that
// closes loops and is no candidate (a twin needs matches)
static ImagePair MakePlanarTwinPair(const Scene& scene, IIndex a, IIndex b, const Pose3D& pose, unsigned numMatches)
{
	ImagePair pair(a, b);
	pair.relativePose = pose;
	pair.E = ImagePair::ComposeEssentialMatrix(pose);
	pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[a].GetK(), scene.images[b].GetK());
	for (uint32_t k = 0; k < numMatches; ++k)
		pair.matches.emplace_back(k, k);
	pair.numFilteredInliers = (int)numMatches;
	pair.numDenseInliers = 0;
	return pair;
}

// The planar twin switch (SwitchPlanarTwinsDense) on cameras looking at a floor, every match exact:
// - a pair whose matches all lie on the floor and whose stored pose is the wrong one of its two
//   planar solutions, while its four neighbours close their rotation loops only with the right one,
//   switches to it and keeps every match as an inlier; it does not switch with the switch off
//   (twinMinLoopGain 0) nor when the margin asks for more loops (5) than the twin gains (4);
// - the same pair with 40% of its matches off the floor is not plane dominated (a homography
//   explains 60% of them, under the 0.75 gate) and is untouched, although its stored pose is the same
//   wrong one and its loops prefer the twin: with the gate lowered to 0.5 the same pair switches;
// - two wrong twins sharing a triangle, P = (0,1) with four certain neighbours of its own and
//   Q = (0,2) with two: alone Q gains 2 loops (its triangle with P closes only once P is right), under
//   the margin 3, so a single pass on the stored poses would switch P only; best gain first switches
//   P, then counts Q again with P at its twin, now 3, and switches Q too. Without P's neighbours P
//   stays and so does Q.
bool PlanarTwinSwitchTest()
{
	TD_TIMER_START();

	// the floor, and a second layer 0.8 above it for the pair that is not plane dominated
	std::vector<Point3> floorPoints, layerPoints;
	for (int i = 0; i < 12; ++i)
		for (int j = 0; j < 10; ++j)
			floorPoints.emplace_back(REAL(-1.5) + REAL(3)*i/11, REAL(-1.2) + REAL(2.4)*j/9, REAL(0));
	for (int i = 0; i < 10; ++i)
		for (int j = 0; j < 8; ++j)
			layerPoints.emplace_back(REAL(-1) + REAL(2)*i/9, REAL(-0.8) + REAL(1.6)*j/7, REAL(0.8));
	std::vector<Point3> allPoints(floorPoints);
	allPoints.insert(allPoints.end(), layerPoints.begin(), layerPoints.end());
	const unsigned numFloor = (unsigned)floorPoints.size(), numAll = (unsigned)allPoints.size();
	// images 0, 1 and 2 see the floor (pairs (0,1) and (0,2) have a planar twin 30 and 18 degrees
	// off); images 3-8 are neighbours, poses alone
	const std::vector<PlanarTwinView> views = {
		{0, 0, 3, 0, 0}, {2, -0.1, 4.4, -2, 10}, {-1, -0.2, 3.6, -13, -10},
		{-1, 1, 3, 5, 5}, {1, 2, 3.5, -5, 0}, {-2, -1, 4, 0, 8}, {0.5, -2, 3.2, 10, -5},
		{2, 2, 4, -8, 6}, {-2, 2, 3.8, 6, -12}};
	MatchConfig matchCfg;
	const DenseMatchConfig config;
	const auto RotationDeg = [](const Pose3D& a, const Pose3D& b) { return R2D(ACOS(ComputeAngle(a.R, b.R))); };
	// the scene: images, keypoints (all points, or the floor only) and the pairs; `twinOf` lists the
	// pairs stored at their wrong planar twin (the twin of the true pose on the floor matches alone)
	struct PairSpec { IIndex a, b; unsigned numMatches; bool bWrongTwin; };
	const auto Build = [&](Scene& scene, const std::vector<Point3>& points, const std::vector<PairSpec>& specs, String& error) {
		MakePlanarTwinScene(scene, views);
		for (IIndex i = 0; i < 3; ++i)
			if (!AddPlanarTwinKeypoints(scene.images[i], points)) {
				error = String::FormatString("image %u does not see every point", i);
				return false;
			}
		PairsMatcher matcher(scene, matchCfg);
		for (const PairSpec& spec : specs) {
			const Image& imgA = scene.images[spec.a];
			const Image& imgB = scene.images[spec.b];
			ImagePair pair(MakePlanarTwinPair(scene, spec.a, spec.b, Pose3D(imgB / imgA), spec.numMatches));
			if (spec.bWrongTwin) {
				ImagePair onFloor(MakePlanarTwinPair(scene, spec.a, spec.b, Pose3D(imgB / imgA), numFloor));
				ImagePair twin;
				float share;
				if (!PlanarTwinDense(matcher, imgA, imgB, config, onFloor, twin, share) ||
					RotationDeg(twin.relativePose.value(), onFloor.relativePose.value()) < REAL(10)) {
					error = String::FormatString("the floor matches of pair %u-%u have no planar twin 10 degrees off (share %g)", spec.a, spec.b, share);
					return false;
				}
				pair.relativePose = twin.relativePose;
				pair.E = twin.E;
				pair.F = twin.F;
			}
			scene.pairs.emplace_back(std::move(pair));
		}
		return true;
	};
	const auto NeighbourSpecs = [](IIndex a, IIndex b, std::initializer_list<IIndex> neighbours) {
		std::vector<PairSpec> specs;
		for (IIndex c : neighbours) {
			specs.push_back({MINF(a, c), MAXF(a, c), 0u, false});
			specs.push_back({MINF(b, c), MAXF(b, c), 0u, false});
		}
		return specs;
	};
	const auto Run = [&](Scene& scene, const DenseMatchConfig& cfg) {
		PairsMatcher matcher(scene, matchCfg);
		return SwitchPlanarTwinsDense(matcher, cfg);
	};
	const auto IsTrue = [&](const Scene& scene, IIndex p) {
		const ImagePair& pair = scene.pairs[p];
		return RotationDeg(pair.relativePose.value(), Pose3D(scene.images[pair.ID2] / scene.images[pair.ID1])) < REAL(0.05);
	};
	String error;

	// 1) a planar pair stored at its wrong twin, four neighbours that close their loops with the true pose
	{
		std::vector<PairSpec> specs = {{0, 1, numFloor, true}};
		const std::vector<PairSpec> neighbours = NeighbourSpecs(0, 1, {3, 4, 5, 6});
		specs.insert(specs.end(), neighbours.begin(), neighbours.end());
		// the switch off, a margin of 5 loops, the default configuration (a margin of 3)
		for (int run = 0; run < 3; ++run) {
			DenseMatchConfig cfg;
			if (run < 2)
				cfg.twinMinLoopGain = run == 0 ? 0u : 5u;
			Scene scene;
			if (!Build(scene, floorPoints, specs, error)) {
				VERBOSE("PlanarTwinSwitchTest FAILED: %s", error.c_str());
				return false;
			}
			const Pose3D stored(scene.pairs[0].relativePose.value());
			const unsigned numSwitched = Run(scene, cfg);
			const ImagePair& pair = scene.pairs[0];
			if (run < 2) {
				if (numSwitched != 0 || !(pair.relativePose.value() == stored)) {
					VERBOSE("PlanarTwinSwitchTest FAILED: with twinMinLoopGain %u the planar pair switched (%u pairs switched)", cfg.twinMinLoopGain, numSwitched);
					return false;
				}
				continue;
			}
			if (numSwitched != 1 || !IsTrue(scene, 0)) {
				VERBOSE("PlanarTwinSwitchTest FAILED: the planar pair stored at its wrong twin is %.3f deg off the true pose after the switch (%u pairs switched), expected it switched to the true pose",
					RotationDeg(pair.relativePose.value(), Pose3D(scene.images[1] / scene.images[0])), numSwitched);
				return false;
			}
			if (pair.GetNumFilteredInliers() != numFloor || pair.GetNumDenseInliers() != 0 || !pair.outlierMatches.empty()) {
				VERBOSE("PlanarTwinSwitchTest FAILED: the switched pair keeps %u of its %u matches as inliers (%u outliers)",
					pair.GetNumFilteredInliers(), numFloor, (unsigned)pair.outlierMatches.size());
				return false;
			}
		}
	}

	// 2) the same pair with 40% of its matches off the floor: not plane dominated, untouched, unless
	// the gate is lowered under its plane share
	{
		std::vector<PairSpec> specs = {{0, 1, numAll, true}};
		const std::vector<PairSpec> neighbours = NeighbourSpecs(0, 1, {3, 4, 5, 6});
		specs.insert(specs.end(), neighbours.begin(), neighbours.end());
		for (float minPlaneShare : {config.twinMinPlaneShare, 0.5f}) {
			Scene scene;
			if (!Build(scene, allPoints, specs, error)) {
				VERBOSE("PlanarTwinSwitchTest FAILED: %s", error.c_str());
				return false;
			}
			const ImagePair stored(scene.pairs[0]);
			PairsMatcher matcher(scene, matchCfg);
			ImagePair twin;
			float share;
			PlanarTwinDense(matcher, scene.images[0], scene.images[1], config, stored, twin, share);
			if (share < 0.5f || share >= config.twinMinPlaneShare) {
				VERBOSE("PlanarTwinSwitchTest FAILED: premise not met: a homography explains %.3f of the two-layer matches, expected [0.5, %g)", share, config.twinMinPlaneShare);
				return false;
			}
			DenseMatchConfig cfg;
			cfg.twinMinPlaneShare = minPlaneShare;
			const unsigned numSwitched = Run(scene, cfg);
			const ImagePair& pair = scene.pairs[0];
			if (minPlaneShare == config.twinMinPlaneShare) {
				if (numSwitched != 0 || !(pair.relativePose.value() == stored.relativePose.value()) ||
					pair.matches.size() != stored.matches.size() || pair.numFilteredInliers != stored.numFilteredInliers) {
					VERBOSE("PlanarTwinSwitchTest FAILED: the pair with %.0f%% of its matches on one plane was touched (%u pairs switched)", 100*share, numSwitched);
					return false;
				}
			} else if (numSwitched != 1 || !IsTrue(scene, 0)) {
				VERBOSE("PlanarTwinSwitchTest FAILED: premise not met: with the gate at %g the two-layer pair does not switch to the true pose (%u pairs switched)", minPlaneShare, numSwitched);
				return false;
			}
		}
	}

	// 3) P = (0,1) and Q = (0,2), both at their wrong twins, sharing the triangle (0,1,2) with (1,2)
	// true; P has the neighbours 3-6, Q the neighbours 7 and 8
	for (bool bNeighboursOfP : {true, false}) {
		std::vector<PairSpec> specs = {{0, 1, numFloor, true}, {0, 2, numFloor, true}, {1, 2, 0u, false}};
		const std::vector<PairSpec> neighboursQ = NeighbourSpecs(0, 2, {7, 8});
		specs.insert(specs.end(), neighboursQ.begin(), neighboursQ.end());
		if (bNeighboursOfP) {
			const std::vector<PairSpec> neighboursP = NeighbourSpecs(0, 1, {3, 4, 5, 6});
			specs.insert(specs.end(), neighboursP.begin(), neighboursP.end());
		}
		Scene scene;
		if (!Build(scene, floorPoints, specs, error)) {
			VERBOSE("PlanarTwinSwitchTest FAILED: %s", error.c_str());
			return false;
		}
		const unsigned numSwitched = Run(scene, config);
		const bool bP = IsTrue(scene, 0), bQ = IsTrue(scene, 1);
		if (bNeighboursOfP ? (numSwitched != 2 || !bP || !bQ) : (numSwitched != 0 || bP || bQ)) {
			VERBOSE("PlanarTwinSwitchTest FAILED: %s P's neighbours %u pairs switched, P %s and Q %s, expected %s",
				bNeighboursOfP ? "with" : "without", numSwitched, bP ? "true" : "wrong", bQ ? "true" : "wrong",
				bNeighboursOfP ? "both switched to the true pose (Q once P is right)" : "neither switched");
			return false;
		}
	}

	VERBOSE("PlanarTwinSwitchTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// The RoMa2 pair the assembly tests build on the wedge: 80 sound guided matches and 6 the geometry
// contradicts, the dense fill drawn from the verdict's region, stored with its dense segment. With a
// dense bias the verdict's B side, which the fill is drawn from, lies that many pixels across the
// epipolar lines, the systematic error of a coarse warp DenseFitWeightTest builds, while the
// guided matches stay exact
static IIndex StoreDenseWedgePair(Scene& scene, PairsMatcher& matcher, const DenseMatchConfig& config, int cells, float denseBias = 0.f)
{
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	const int rx0 = 14, rx1 = 50, ry0 = 13, ry1 = 47;
	// across the epipolar lines, which run along the baseline direction (0.4, 0.1) here
	const Point2f across(normalized(Point2f(-0.1f, 0.4f)));
	PairVerdict verdict;
	verdict.admitted = true;
	std::vector<Point2f> exactB;
	for (int y = ry0; y < ry1; ++y) {
		for (int x = rx0; x < rx1; ++x) {
			const Point2f ptA(CellToPixel(Point2f((float)x, (float)y), cv::Size(cells, cells), imgA.GetSize()));
			Point2f ptB;
			if (!DenseWedgeCorrespondence(imgA, imgB, ptA, ptB))
				continue;
			verdict.inliersA.push_back(ptA);
			verdict.inliersB.push_back(ptB + across*denseBias);
			verdict.confidences.push_back(0.5f);
			exactB.push_back(ptB);
		}
	}
	verdict.confidentAreaA = verdict.confidentAreaB = verdict.inlierAreaA = verdict.inlierAreaB =
		(float)verdict.inliersA.size()/(float)(cells*cells);
	std::vector<DMatch> guided;
	for (size_t k = 0; k < verdict.inliersA.size() && guided.size() < 86; k += 14) {
		const uint32_t idx = (uint32_t)imgA.keypoints.size();
		imgA.keypoints.emplace_back(verdict.inliersA[k].x, verdict.inliersA[k].y, 10.f);
		imgB.keypoints.emplace_back(exactB[k].x, exactB[k].y + (guided.size() >= 80 ? 40.f : 0.f), 10.f);
		guided.emplace_back(idx, idx);
	}
	ImagePair pair(0, 1);
	const Pose3D poseGT(imgB / imgA);
	pair.relativePose = poseGT;
	pair.E = ImagePair::ComposeEssentialMatrix(poseGT);
	pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), imgA.GetK(), imgB.GetK());
	DenseMatches dense;
	if (!AssemblePairDense(matcher, imgA, imgB, verdict, guided, config, cells, pair, dense))
		return NO_ID;
	std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
	StorePairDense(scene, pairIndexMap, std::move(pair), dense, cells);
	return 0;
}

// The pose sub-case of the test below, the systematic error of a coarse warp DenseFitWeightTest
// builds: the dense fill's B side this many pixels across the epipolar lines, the guided matches exact
constexpr float DENSE_REVERIFY_DENSE_BIAS = 2.f;
// and the bars DenseFitWeightTest holds the assembly's pose to: the dense matches outnumber the
// sparse ones at least this many times and pull a pose fitted at full weight at least this far off
// the truth, and the pose the denseFitWeight polish closes keeps at most this share of that error
constexpr unsigned DENSE_REVERIFY_MIN_DENSE_PER_SPARSE = 4;
constexpr REAL DENSE_REVERIFY_MIN_FULL_WEIGHT_ERROR_DEG = REAL(0.05);
constexpr REAL DENSE_REVERIFY_MAX_ERROR_SHARE = REAL(0.25);

// The re-verification a camera's change sets off (PairsMatcher::ComputeRelativePoses, which the focal
// calibration runs on every camera it changed and the lens stage on every camera it fitted) keeps a
// RoMa2 pair's partition: on the wedge, with nothing about the cameras changed, the pair comes back
// with the same sparse inliers, the same dense segment of dense keypoints, and the same outliers; a
// pair whose dense fill drew nothing comes back well formed too, and so does a pair whose matches all
// sit among its outliers, re-verified from them, its dense matches back in the dense segment. A pair
// the re-fit refuses comes back invalidated, as GeometricFilter leaves a pair it cannot verify (no
// geometry, every match an outlier): one stored without a geometry, as a scene saved by an earlier
// build holds a pair GeometricFilter could not verify, and one under the minimum of matches, as
// FilterRedundantKeypoints leaves it (every match an outlier, the geometry kept). Both with the
// matcher's strict filter and without it: with it, GeometricFilter's own partition tells the dense
// matches apart by their keypoints (ImagePair::FilterMatches), without it nothing does, and
// GeometricFilter over all of a pair's matches folds the dense segment into the sparse one and leaves
// the sparse count unmaterialised (-1), which the re-verification materialises. Under the default
// configuration the change shows in the pose: with the dense fill displaced DENSE_REVERIFY_DENSE_BIAS
// px across the epipolar lines, the re-verified pose follows the sparse matches (the denseFitWeight
// polish), where GeometricFilter over all of the pair's matches at full weight follows the bias
bool DenseReverifyPartitionTest()
{
	TD_TIMER_START();
	constexpr int cells = 64;
	enum class Stored {
		DENSE,         // the pair as stored, its dense segment past its sparse inliers
		NO_DENSE,      // a pair whose fill drew nothing, in a scene the one pass matched
		OUTLIERS,      // the pair as stored, then every match moved among its outliers
		NO_GEOMETRY,   // every match an outlier and no geometry, re-verified where the re-fit is refused
		UNDER_MINIMUM  // every match an outlier, the geometry kept, re-verified where the re-fit is refused
	};
	for (const bool strictFilter : {true, false}) {
		for (const Stored stored : {Stored::DENSE, Stored::NO_DENSE, Stored::OUTLIERS, Stored::NO_GEOMETRY, Stored::UNDER_MINIMUM}) {
			const bool withDense = stored != Stored::NO_DENSE;
			const char* const name = stored == Stored::DENSE ? "dense" :
				stored == Stored::NO_DENSE ? "no dense segment" :
				stored == Stored::OUTLIERS ? "every match an outlier" :
				stored == Stored::NO_GEOMETRY ? "no geometry" : "under the minimum";
			Scene scene;
			MakeDenseWedgeScene(scene);
			MatchConfig matchCfg;
			if (!strictFilter)
				matchCfg.minTriangulationAngle = matchCfg.reprojThreshold = matchCfg.epipoleFilterThreshold = 0.f;
			PairsMatcher matcher(scene, matchCfg);
			DenseMatchConfig config;
			if (!withDense)
				config.denseMatchesPerFrame = 0;
			matcher.SetDense(NULL, config);
			if (StoreDenseWedgePair(scene, matcher, config, cells) == NO_ID) {
				VERBOSE("DenseReverifyPartitionTest FAILED: the wedge pair was not assembled");
				return false;
			}
			if (!withDense) {
				// as another pair's fill would have left them (every keypoint so far described): the images of
				// a scene the one pass matched carry a closed described prefix, which is how the
				// re-verification knows a RoMa2 pair that drew no fill
				scene.images[0].CloseDescribedKeypoints();
				scene.images[1].CloseDescribedKeypoints();
			}
			ImagePair& pair = scene.pairs[0];
			const unsigned numSparse = pair.GetNumFilteredInliers(), numDense = pair.GetNumDenseInliers();
			const size_t numOutliers = pair.outlierMatches.size();
			if (numSparse != 80 || numOutliers != 6 || (numDense > 0) != withDense) {
				VERBOSE("DenseReverifyPartitionTest FAILED: the stored pair holds %u sparse, %u dense, %u outliers",
					numSparse, numDense, (unsigned)numOutliers);
				return false;
			}
			const unsigned numCandidates = pair.GetNumMatches();
			// a pair the re-fit must refuse is verified again on a matcher whose estimator asks for more
			// inliers than the pair has candidates
			const bool bRefused = stored == Stored::NO_GEOMETRY || stored == Stored::UNDER_MINIMUM;
			MatchConfig matchCfgAgain(matchCfg);
			if (bRefused)
				matchCfgAgain.minMatches = numCandidates + 1;
			PairsMatcher matcherAgain(scene, matchCfgAgain);
			matcherAgain.SetDense(NULL, config);
			if (stored == Stored::NO_GEOMETRY) {
				// as a scene saved by an earlier build holds a pair GeometricFilter could not verify, and a
				// resumed scene verifies again every pair without a pose
				pair.ResetGeometry();
				pair.InvalidateMatches();
				matcherAgain.ComputeRelativePoses();
			} else {
				if (stored != Stored::DENSE && stored != Stored::NO_DENSE)
					pair.InvalidateMatches(); // its dense matches now among its outliers, the geometry kept
				matcherAgain.ComputeRelativePoses(true, false, {scene.cameras[0]});
			}
			if (bRefused) {
				// invalidated, as GeometricFilter leaves a pair it cannot verify
				if (pair.HasMatches() || pair.outlierMatches.size() != numCandidates || pair.numFilteredInliers != -1 ||
					pair.GetNumDenseInliers() != 0 || pair.HasGeometricVerification()) {
					VERBOSE("DenseReverifyPartitionTest FAILED: %s, %s, re-verified to %u sparse, %u dense, %u outliers of the %u matches, %s geometry, where the refused re-fit leaves it invalidated",
						name, strictFilter ? "strict filter" : "no strict filter",
						pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(), (unsigned)pair.outlierMatches.size(),
						numCandidates, pair.HasGeometricVerification() ? "with a" : "no");
					return false;
				}
				continue;
			}
			if (pair.GetNumFilteredInliers() != numSparse || pair.GetNumDenseInliers() != numDense ||
				pair.outlierMatches.size() != numOutliers || !HasDensePartition(scene, pair) || !pair.relativePose.has_value()) {
				VERBOSE("DenseReverifyPartitionTest FAILED: %s, %s, re-verified to %u sparse, %u dense, %u outliers (from %u, %u, %u), partition %s",
					name, strictFilter ? "strict filter" : "no strict filter",
					pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(),
					(unsigned)pair.outlierMatches.size(), numSparse, numDense, (unsigned)numOutliers,
					HasDensePartition(scene, pair) ? "kept" : "broken");
				return false;
			}
			// the partition the re-verification leaves: the sparse count materialised, which GeometricFilter
			// without the strict filter leaves at -1, and nothing past the track-forming matches
			if (pair.numFilteredInliers < 0) {
				VERBOSE("DenseReverifyPartitionTest FAILED: %s, %s, re-verified with the sparse count unmaterialised",
					name, strictFilter ? "strict filter" : "no strict filter");
				return false;
			}
			if (pair.matches.size() != pair.GetNumTrackFormingMatches()) {
				VERBOSE("DenseReverifyPartitionTest FAILED: %s, %s, re-verified with %u matches past the track-forming ones",
					name, strictFilter ? "strict filter" : "no strict filter",
					(unsigned)pair.matches.size() - pair.GetNumTrackFormingMatches());
				return false;
			}
		}
	}

	// the pose under the default configuration, against the truth: GeometricFilter over all of the
	// stored pair's matches (the re-verification without the change) follows the biased dense fill,
	// the re-verification follows the exact sparse matches
	Scene scene;
	MakeDenseWedgeScene(scene);
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	DenseMatchConfig config;
	matcher.SetDense(NULL, config);
	if (StoreDenseWedgePair(scene, matcher, config, cells, DENSE_REVERIFY_DENSE_BIAS) == NO_ID) {
		VERBOSE("DenseReverifyPartitionTest FAILED: the biased wedge pair was not assembled");
		return false;
	}
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	const auto RotationError = [&poseGT](const ImagePair& pair) {
		return R2D(ACOS(ComputeAngle(pair.relativePose->R, poseGT.R)));
	};
	ImagePair fullWeight(scene.pairs[0]);
	if (!matcher.GeometricFilter(imgA, imgB, fullWeight) || !fullWeight.relativePose.has_value()) {
		VERBOSE("DenseReverifyPartitionTest FAILED: GeometricFilter verified no pose on the biased pair");
		return false;
	}
	const REAL errFull = RotationError(fullWeight);
	matcher.ComputeRelativePoses(true, false, {scene.cameras[0]});
	const ImagePair& biased = scene.pairs[0];
	if (!biased.relativePose.has_value() || !HasDensePartition(scene, biased)) {
		VERBOSE("DenseReverifyPartitionTest FAILED: the biased pair was re-verified to no pose or a broken partition");
		return false;
	}
	if (biased.GetNumDenseInliers() < DENSE_REVERIFY_MIN_DENSE_PER_SPARSE*biased.GetNumFilteredInliers() ||
		errFull < DENSE_REVERIFY_MIN_FULL_WEIGHT_ERROR_DEG) {
		VERBOSE("DenseReverifyPartitionTest FAILED: premise not met: %u dense against %u sparse, %.4f deg through GeometricFilter at full weight",
			biased.GetNumDenseInliers(), biased.GetNumFilteredInliers(), errFull);
		return false;
	}
	const REAL errReverified = RotationError(biased);
	if (errReverified > DENSE_REVERIFY_MAX_ERROR_SHARE*errFull) {
		VERBOSE("DenseReverifyPartitionTest FAILED: the re-verified pose is %.4f deg off with the dense fill biased, %.4f deg through GeometricFilter at full weight",
			errReverified, errFull);
		return false;
	}
	VERBOSE("DenseReverifyPartitionTest PASSED: with the dense fill biased %g px the re-verified pose is %.4f deg off, %.4f deg through GeometricFilter at full weight (%u dense, %u sparse) (%s)",
		DENSE_REVERIFY_DENSE_BIAS, errReverified, errFull, biased.GetNumDenseInliers(), biased.GetNumFilteredInliers(), TD_TIMER_GET_FMT().c_str());
	return true;
}

// The described/dense keypoint boundary: the stored
// count is what survives a descriptor release and an .sfm round-trip of an image whose
// keypoints.size() > descriptors.rows -- the two arrays serialize independently, so nothing else
// would notice the boundary moving
bool DenseKeypointBoundaryTest()
{
	TD_TIMER_START();

	// one image with 6 described keypoints and 4 dense ones appended past them
	Scene scene;
	Image& img = scene.images.emplace_back((IIndex)0, String("a.jpg"));
	for (unsigned i = 0; i < 6; ++i)
		img.keypoints.emplace_back((float)i, 1.f, 3.f, -1.f, 0.02f);
	img.descriptors = cv::Mat::zeros(6, 32, CV_8U);
	if (img.HasDenseKeypoints() || img.NumDescribedKeypoints() != 6 || img.NumDenseKeypoints() != 0) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: an image without dense keypoints must report every keypoint described");
		return false;
	}
	img.CloseDescribedKeypoints();
	for (unsigned i = 0; i < 4; ++i)
		img.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(10.f + i, 2.f), 0.8f, 6.f));
	// closing the boundary a second time (a second supplemented pair on the same image) must not
	// swallow the dense keypoints the first one appended
	img.CloseDescribedKeypoints();
	if (!img.HasDenseKeypoints() || img.NumDescribedKeypoints() != 6 || img.NumDenseKeypoints() != 4) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: boundary %u for %u keypoints",
			img.NumDescribedKeypoints(), (unsigned)img.keypoints.size());
		return false;
	}
	for (uint32_t i = 0; i < (uint32_t)img.keypoints.size(); ++i) {
		if (img.IsDenseKeypoint(i) != (i >= 6)) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: keypoint %u misclassified", i);
			return false;
		}
	}
	// the described prefix is what a descriptor-indexed selection may return
	for (const unsigned idx : img.SelectTopKeypoints(10)) {
		if (idx >= 6) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: SelectTopKeypoints returned the dense keypoint %u", idx);
			return false;
		}
	}

	// the boundary outlives the descriptor release: this is the whole reason it is stored and not
	// derived from descriptors.rows, which is zero from here on
	img.descriptors.release();
	if (img.NumDescribedKeypoints() != 6 || img.IsDenseKeypoint(5) || !img.IsDenseKeypoint(6)) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: boundary lost with the descriptors");
		return false;
	}

	// and it round-trips through .sfm with keypoints.size() > descriptors.rows
	ScopedTempDir tmp("DenseKeypointBoundaryTest");
	if (!tmp.IsValid())
		return false;
	if (!scene.Save(tmp("scene.sfm")))
		return false;
	Scene loaded;
	if (!loaded.Load(tmp("scene.sfm"))) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: load");
		return false;
	}
	const Image& loadedImg = loaded.images[0];
	if (loadedImg.keypoints.size() != 10 || !loadedImg.HasDenseKeypoints() ||
		loadedImg.NumDescribedKeypoints() != 6 || loadedImg.NumDenseKeypoints() != 4 ||
		loadedImg.IsDenseKeypoint(5) || !loadedImg.IsDenseKeypoint(6)) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: boundary not preserved by the .sfm round-trip");
		return false;
	}
	if (loadedImg.keypoints[7].response != 0.8f || loadedImg.keypoints[7].size != 6.f) {
		VERBOSE("DenseKeypointBoundaryTest FAILED: dense keypoint response/size not preserved");
		return false;
	}

	// A described-only image (no dense keypoints at all) must reach exactly the survivors and the
	// order the filter emitted before dense supplementation existed: the dense segment is empty, the
	// cross-segment prune has nothing to do, and the described segment is the whole array -- same
	// comparator, same leader-based grouping, same survivor, same position-order compaction as
	// before the two-segment split existed. No special case is needed to make that true.
	{
		Scene noDenseScene;
		Image& im = noDenseScene.images.emplace_back(0u, String("nodense.jpg"));
		// unsorted on purpose, with one duplicate pair (30,30 twice; the higher response*size wins)
		im.keypoints.emplace_back(40.f, 40.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(10.f, 10.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(30.f, 30.f, 4.f, -1.f, 0.01f);
		im.keypoints.emplace_back(20.f, 20.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(30.f, 30.f, 4.f, -1.f, 0.05f);
		MatchConfig noDenseCfg;
		noDenseCfg.minMatches = 1;
		PairsMatcher(noDenseScene, noDenseCfg).FilterRedundantKeypoints();
		if (im.HasDenseKeypoints() || im.keypoints.size() != 4 || im.NumDescribedKeypoints() != 4) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: a described-only image must stay described-only after the filter");
			return false;
		}
		const float expectedNoDenseX[4] = {10.f, 20.f, 30.f, 40.f};
		for (uint32_t f = 0; f < 4; ++f) {
			// the invariant, checked directly rather than only by count: nothing below the boundary is
			// ever a dense keypoint when there never was one
			if (im.keypoints[f].pt.x != expectedNoDenseX[f] || im.IsDenseKeypoint(f)) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: described-only filter order at %u: x=%g",
					f, im.keypoints[f].pt.x);
				return false;
			}
		}
		if (im.keypoints[2].response != 0.05f) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: described-only filter kept the weaker duplicate");
			return false;
		}
	}

	// FilterRedundantKeypoints must move the boundary through the same remap it applies to the
	// keypoint indices. Two images, each with 5 described keypoints of which two coincide (the
	// removal *inside* the prefix that shrinks the boundary), plus 5 dense keypoints of which one
	// sits exactly on a described keypoint (described-wins), two coincide with each other (the higher
	// warp confidence wins), and one is a standalone dense point positioned *before* several described
	// survivors (x=15, against described survivors at 10/20/30/40) -- it must still land in the dense
	// suffix, and its position must still order it against the other dense survivors on its own,
	// independently of the described segment. A stale boundary here would silently reclassify the
	// surviving described keypoints that shifted down into it as dense.
	Scene filterScene;
	for (IIndex k = 0; k < 2; ++k) {
		Image& im = filterScene.images.emplace_back(k, String::FormatString("%u.jpg", k));
		// described: 0..4, with 1 and 2 at the same position (1 has the larger response*size)
		im.keypoints.emplace_back(10.f, 10.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(20.f, 20.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(20.f, 20.f, 4.f, -1.f, 0.01f);
		im.keypoints.emplace_back(30.f, 30.f, 4.f, -1.f, 0.05f);
		im.keypoints.emplace_back(40.f, 40.f, 4.f, -1.f, 0.05f);
		im.CloseDescribedKeypoints();
		// dense: 5 lands on the described keypoint 3, 6 and 7 coincide (7 is the more confident),
		// 8 is on its own, 9 is on its own and positioned before the described segment's tail
		im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(30.f, 30.f), 0.9f, 6.f));
		im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(50.f, 50.f), 0.4f, 6.f));
		im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(50.f, 50.f), 0.95f, 6.f));
		im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(60.f, 60.f), 0.9f, 6.f));
		im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(15.f, 15.f), 0.7f, 6.f));
	}
	ImagePair& filterPair = filterScene.pairs.emplace_back(0u, 1u);
	// one match per keypoint index, so the remap of every keypoint is observable through it, and the
	// partition is the one AppendDenseMatches leaves behind: the five described matches are the
	// sparse segment, the five dense ones the supplement (a dense match inside the sparse count is
	// the drift ImagePair::CheckSparseSegmentIsDescribed asserts against)
	for (uint32_t k = 0; k < 10; ++k)
		filterPair.matches.emplace_back(k, k);
	filterPair.numFilteredInliers = 5;
	filterPair.numDenseInliers = 5;
	MatchConfig filterCfg;
	filterCfg.minMatches = 1;
	PairsMatcher(filterScene, filterCfg).FilterRedundantKeypoints();
	for (IIndex k = 0; k < 2; ++k) {
		const Image& im = filterScene.images[k];
		// 10 keypoints, 3 removed (the duplicated described one, the dense one on a described
		// keypoint, the weaker of the two coincident dense ones); the standalone dense point at x=15
		// is not a duplicate of anything and survives
		if (im.keypoints.size() != 7 || im.NumDescribedKeypoints() != 4 || im.NumDenseKeypoints() != 3) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: image %u kept %u keypoints with boundary %u, expected 7 and 4",
				k, (unsigned)im.keypoints.size(), im.NumDescribedKeypoints());
			return false;
		}
		// the described survivors are still the leading prefix, in position order; the dense survivors
		// follow, in *their own* position order -- x=15 sorts before x=50/x=60 among dense survivors
		// even though it is less than every described survivor from index 1 on, because the two segments
		// are compacted independently rather than merged into one global position order. The invariant
		// is checked directly at every index, not just by count: below the boundary is never dense,
		// at or above it always is.
		const float expectedX[7] = {10.f, 20.f, 30.f, 40.f, 15.f, 50.f, 60.f};
		for (uint32_t f = 0; f < 7; ++f) {
			if (im.keypoints[f].pt.x != expectedX[f] || im.IsDenseKeypoint(f) != (f >= 4)) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: image %u keypoint %u at x=%g, dense=%d",
					k, f, im.keypoints[f].pt.x, (int)im.IsDenseKeypoint(f));
				return false;
			}
		}
		// described-wins: the dense keypoint that coincided with described keypoint 3 collapsed
		// onto it, so what survives at x=30 is the described one
		if (im.keypoints[2].response != 0.05f || im.keypoints[2].size != 4.f) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: image %u lost a described keypoint to a dense one", k);
			return false;
		}
		// the standalone dense point survives unmodified, ahead of the other dense survivors
		if (im.keypoints[4].response != 0.7f) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: image %u lost the standalone dense keypoint at x=15 (%g)",
				k, im.keypoints[4].response);
			return false;
		}
		// between two dense points the more confident one survives
		if (im.keypoints[5].response != 0.95f) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: image %u kept the less confident dense keypoint (%g)",
				k, im.keypoints[5].response);
			return false;
		}
	}

	// The match-level described-wins rule (NumDenseEnds in the duplicate-match filter) must actually
	// decide something: two matches that remap onto the same queryIdx, where one's trainIdx is
	// described and the other's is dense. Image A carries two described keypoints at the same
	// position (one match to each), which the keypoint filter collapses onto a single survivor, so
	// both matches end up sharing that queryIdx; image B carries one described and one dense
	// keypoint, untouched by the filter, so the two matches keep pointing at different kinds of
	// endpoint in B. On weight alone the dense endpoint would win -- ComputeKeypointWeight rates a
	// dense point's high, saturated confidence and large warp-cell size well above a modest SIFT
	// response at a small size -- so if this case passes, the rule is doing real work, not agreeing
	// with what the weight comparison would have done anyway.
	{
		Scene weightScene;
		Image& wA = weightScene.images.emplace_back(0u, String("wA.jpg"));
		wA.keypoints.emplace_back(5.f, 5.f, 3.f, -1.f, 0.05f);
		wA.keypoints.emplace_back(5.f, 5.f, 3.f, -1.f, 0.01f);
		Image& wB = weightScene.images.emplace_back(1u, String("wB.jpg"));
		wB.keypoints.emplace_back(50.f, 50.f, 3.f, -1.f, 0.02f); // described, modest response/size
		wB.CloseDescribedKeypoints();
		wB.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(60.f, 60.f), 0.9f, 6.f)); // dense, high weight
		ImagePair& weightPair = weightScene.pairs.emplace_back(0u, 1u);
		weightPair.matches.emplace_back(0, 0); // A's first duplicate -> B's described keypoint
		weightPair.matches.emplace_back(1, 1); // A's second duplicate -> B's dense keypoint
		weightPair.numFilteredInliers = 2;
		MatchConfig weightCfg;
		weightCfg.minMatches = 1;
		PairsMatcher(weightScene, weightCfg).FilterRedundantKeypoints();
		if (weightPair.matches.size() != 1 || weightPair.matches[0].trainIdx != 0) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: match-level described-wins rule did not keep the described endpoint (%u matches left, trainIdx %d)",
				(unsigned)weightPair.matches.size(), weightPair.matches.empty() ? -1 : weightPair.matches[0].trainIdx);
			return false;
		}
	}

	// The match-level partition must survive the duplicate-match filter (step 3). A supplemented pair
	// carries three segments -- sparse inliers | dense supplement | RANSAC inliers the strict filter
	// rejected -- and step 3 runs on any pair whose images lost a keypoint. A compaction that sorts
	// `matches` moves matches ACROSS those bounds: dense keypoints hold the largest indices by
	// construction, so every dense match sorts to the end while the rejects migrate into the prefix,
	// which makes part of the supplement inert in BuildTracks and lets an equal number of deliberately
	// rejected matches form tracks -- with nothing in any counter to show it. So this pair has both:
	// rejects present, and a described keypoint removed by the keypoint filter.
	{
		Scene partScene;
		for (IIndex k = 0; k < 2; ++k) {
			Image& im = partScene.images.emplace_back(k, String::FormatString("p%u.jpg", k));
			// described 0..4, with 1 and 2 coincident (1 wins on response*size, so 2 is removed and
			// every later index shifts down -- that removal is what makes step 3 run at all)
			im.keypoints.emplace_back(10.f, 10.f, 4.f, -1.f, 0.05f);
			im.keypoints.emplace_back(20.f, 20.f, 4.f, -1.f, 0.05f);
			im.keypoints.emplace_back(20.f, 20.f, 4.f, -1.f, 0.01f);
			im.keypoints.emplace_back(30.f, 30.f, 4.f, -1.f, 0.05f);
			im.keypoints.emplace_back(40.f, 40.f, 4.f, -1.f, 0.05f);
			im.CloseDescribedKeypoints();
			// dense 5..7, none coincident with anything, so all three survive as themselves
			im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(100.f, 100.f), 0.6f, 6.f));
			im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(110.f, 110.f), 0.7f, 6.f));
			im.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(120.f, 120.f), 0.8f, 6.f));
		}
		ImagePair& partPair = partScene.pairs.emplace_back(0u, 1u);
		partPair.matches.emplace_back(0, 0); // sparse inliers: described keypoints the filter kept
		partPair.matches.emplace_back(3, 3);
		partPair.matches.emplace_back(4, 4);
		partPair.matches.emplace_back(5, 5); // the dense supplement, where AppendDenseMatches puts it
		partPair.matches.emplace_back(6, 6);
		partPair.matches.emplace_back(7, 7);
		partPair.matches.emplace_back(1, 1); // rejects: they must stay outside the prefix
		partPair.matches.emplace_back(2, 2);
		partPair.numFilteredInliers = 3;
		partPair.numDenseInliers = 3;
		MatchConfig partCfg;
		partCfg.minMatches = 1;
		PairsMatcher(partScene, partCfg).FilterRedundantKeypoints();
		const Image& pimg1 = partScene.images[0];
		const Image& pimg2 = partScene.images[1];
		// the two rejects remapped onto the same (queryIdx, trainIdx) = (1, 1) and one of them was
		// dropped as a duplicate; both counts are recomputed from the survivors of their own segment,
		// so neither can absorb a removal that happened in the other
		if (partPair.matches.size() != 7 || partPair.numFilteredInliers != 3 || partPair.numDenseInliers != 3 ||
			partPair.GetNumFilteredInliers() != 3 || partPair.GetNumTrackFormingMatches() != 6) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: partition not recomputed per segment (%u matches, %d sparse, %d dense)",
				(unsigned)partPair.matches.size(), partPair.numFilteredInliers, partPair.numDenseInliers);
			return false;
		}
		const uint32_t expectedQuery[7] = {0, 2, 3, 4, 5, 6, 1};
		for (uint32_t m = 0; m < 7; ++m) {
			if (partPair.matches[m].queryIdx != expectedQuery[m] || partPair.matches[m].trainIdx != expectedQuery[m]) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: match %u is (%u, %u), expected (%u, %u) -- the compaction reordered the segments",
					m, partPair.matches[m].queryIdx, partPair.matches[m].trainIdx, expectedQuery[m], expectedQuery[m]);
				return false;
			}
		}
		// every match below the track-forming boundary really is track-forming, and no reject entered
		// it: the sparse segment is described at both ends and never the reject's keypoint (index 1),
		// the dense segment is dense at both ends, and the single reject sits past the boundary
		for (uint32_t m = 0; m < partPair.GetNumFilteredInliers(); ++m) {
			if (pimg1.IsDenseKeypoint(partPair.matches[m].queryIdx) ||
				pimg2.IsDenseKeypoint(partPair.matches[m].trainIdx) ||
				partPair.matches[m].queryIdx == 1) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: sparse segment match %u is not a described non-reject match", m);
				return false;
			}
		}
		for (uint32_t m = partPair.GetNumFilteredInliers(); m < partPair.GetNumTrackFormingMatches(); ++m) {
			if (!pimg1.IsDenseKeypoint(partPair.matches[m].queryIdx) ||
				!pimg2.IsDenseKeypoint(partPair.matches[m].trainIdx)) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: dense segment match %u lost its dense endpoints", m);
				return false;
			}
		}
		if (partPair.matches[partPair.GetNumTrackFormingMatches()].queryIdx != 1) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: the strict-filter reject is not the match past the track-forming boundary");
			return false;
		}
	}

	// The cross-segment prune can leave a two-hop survivor chain: a dense duplicate points at its
	// intra-segment leader, and that leader is then itself reused into a coincident described
	// survivor. The remap fixup is a single dereference, so without a flattening pass a chain
	// resolves correctly only when the leader's index happens to be the smaller one. Three mutually
	// coincident dense keypoints whose leader is NOT the first of them, plus a described keypoint at
	// the same position, produce exactly that: all three must remap onto the described survivor and
	// none onto an intermediate dense index. Each dense index is observed through its own pair, so
	// the duplicate-match filter cannot collapse the three observations into one.
	{
		Scene chainScene;
		Image& cimg = chainScene.images.emplace_back(0u, String("chain.jpg"));
		cimg.keypoints.emplace_back(10.f, 10.f, 4.f, -1.f, 0.05f); // described, on its own
		cimg.keypoints.emplace_back(50.f, 50.f, 4.f, -1.f, 0.05f); // described, the survivor of the cluster
		cimg.CloseDescribedKeypoints();
		// three coincident dense keypoints; the middle index is the most confident, so the group's
		// leader is index 3 and indices 2 and 4 are the two-hop followers
		cimg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(50.f, 50.f), 0.5f, 6.f));
		cimg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(50.f, 50.f), 0.95f, 6.f));
		cimg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(50.f, 50.f), 0.7f, 6.f));
		for (IIndex k = 1; k <= 3; ++k) {
			Image& other = chainScene.images.emplace_back(k, String::FormatString("c%u.jpg", k));
			other.keypoints.emplace_back(70.f, 70.f, 4.f, -1.f, 0.05f);
		}
		for (IIndex k = 1; k <= 3; ++k) {
			ImagePair& p = chainScene.pairs.emplace_back(0u, k);
			p.matches.emplace_back(1 + k, 0); // dense keypoint 2, 3, 4 respectively
			p.numFilteredInliers = 1;
		}
		MatchConfig chainCfg;
		chainCfg.minMatches = 1;
		PairsMatcher(chainScene, chainCfg).FilterRedundantKeypoints();
		// the whole dense cluster collapsed onto the described keypoint it coincides with
		if (cimg.keypoints.size() != 2 || !cimg.HasDenseKeypoints() ||
			cimg.NumDescribedKeypoints() != 2 || cimg.NumDenseKeypoints() != 0) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: coincident dense cluster left %u keypoints with boundary %u",
				(unsigned)cimg.keypoints.size(), cimg.NumDescribedKeypoints());
			return false;
		}
		for (IIndex k = 0; k < 3; ++k) {
			const ImagePair& p = chainScene.pairs[k];
			if (p.matches.size() != 1 || p.matches[0].queryIdx != 1) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: dense keypoint %u remapped to %d instead of the described survivor 1",
					k + 2, p.matches.empty() ? -1 : (int)p.matches[0].queryIdx);
				return false;
			}
		}
	}

	// The keypoint dedup comparator must be a strict weak ordering. "Within 0.1px" is NOT transitive,
	// so a comparator that arbitrates near-coincident pairs by response*size instead of by position
	// can report a < b, b < c and c < a -- undefined behaviour in std::sort, not merely an unspecified
	// order. Each segment below is exactly that intransitive triple, at 0.08px spacing: a ~ b and
	// b ~ c while a !~ c, and the MIDDLE point is the most confident, so the duplicate-aware
	// comparator's cycle was b < a (duplicates, response), b < c (duplicates, response) and a < c
	// (not duplicates, position) -- an order that sorts the triple b, a, c and lets the leader-based
	// grouping swallow all three into one run, collapsing a keypoint pair that is 0.16px apart. Pure
	// position order has no cycle: the run is the leader plus what is within 0.1px OF THE LEADER, so
	// the triple splits into {a,b} keeping b, and {c}. Repeated to check the outcome is the same on
	// every run of the same input, which an order-dependent comparator cannot promise.
	{
		for (unsigned rep = 0; rep < 4; ++rep) {
			Scene triScene;
			Image& triImg = triScene.images.emplace_back(0u, String("tri.jpg"));
			// described triple at y = 10, dense triple at y = 100, both with the middle one strongest
			triImg.keypoints.emplace_back(0.00f, 10.f, 4.f, -1.f, 0.01f);
			triImg.keypoints.emplace_back(0.08f, 10.f, 4.f, -1.f, 0.03f);
			triImg.keypoints.emplace_back(0.16f, 10.f, 4.f, -1.f, 0.02f);
			triImg.CloseDescribedKeypoints();
			triImg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(0.00f, 100.f), 0.5f, 6.f));
			triImg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(0.08f, 100.f), 0.9f, 6.f));
			triImg.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(0.16f, 100.f), 0.7f, 6.f));
			// the far image's keypoints are all distinct and carry identical response/size, so a
			// duplicate-match group is an exact weight tie and the stable order keeps the earlier one
			Image& triOther = triScene.images.emplace_back(1u, String("tri2.jpg"));
			for (unsigned k = 0; k < 6; ++k)
				triOther.keypoints.emplace_back(1000.f + 10.f * (float)k, 1000.f, 4.f, -1.f, 0.05f);
			triOther.CloseDescribedKeypoints();
			ImagePair& triPair = triScene.pairs.emplace_back(0u, 1u);
			for (uint32_t k = 0; k < 6; ++k)
				triPair.matches.emplace_back(k, k);
			triPair.numFilteredInliers = 3;
			triPair.numDenseInliers = 3;
			MatchConfig triCfg;
			triCfg.minMatches = 1;
			PairsMatcher(triScene, triCfg).FilterRedundantKeypoints();
			// two survivors per segment: the run's most confident member and the point 0.16px out,
			// which is a duplicate of neither survivor. Under the intransitive comparator the whole
			// triple sorted into a single run and only one survivor per segment came out.
			if (triImg.keypoints.size() != 4 || triImg.NumDescribedKeypoints() != 2 || triImg.NumDenseKeypoints() != 2) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: intransitive triple (rep %u) left %u keypoints with boundary %u, expected 4 and 2",
					rep, (unsigned)triImg.keypoints.size(), triImg.NumDescribedKeypoints());
				return false;
			}
			const float expectedX[4] = {0.08f, 0.16f, 0.08f, 0.16f};
			const float expectedY[4] = {10.f, 10.f, 100.f, 100.f};
			const float expectedResponse[4] = {0.03f, 0.02f, 0.9f, 0.7f};
			for (unsigned f = 0; f < 4; ++f) {
				if (ABS(triImg.keypoints[f].pt.x - expectedX[f]) > 1e-5f ||
					triImg.keypoints[f].pt.y != expectedY[f] ||
					ABS(triImg.keypoints[f].response - expectedResponse[f]) > 1e-5f) {
					VERBOSE("DenseKeypointBoundaryTest FAILED: intransitive triple (rep %u) keypoint %u is (%g, %g) response %g",
						rep, f, triImg.keypoints[f].pt.x, triImg.keypoints[f].pt.y, triImg.keypoints[f].response);
					return false;
				}
			}
			// the partition follows the runs: in each segment the two matches that remapped onto the
			// run's survivor collapse to the earlier one, so both counts drop by exactly one
			if (triPair.matches.size() != 4 || triPair.numFilteredInliers != 2 || triPair.numDenseInliers != 2) {
				VERBOSE("DenseKeypointBoundaryTest FAILED: intransitive triple (rep %u) partition is %u matches, %d sparse, %d dense",
					rep, (unsigned)triPair.matches.size(), triPair.numFilteredInliers, triPair.numDenseInliers);
				return false;
			}
			const uint32_t expectedTrain[4] = {0, 2, 3, 5};
			for (uint32_t m = 0; m < 4; ++m) {
				if (triPair.matches[m].queryIdx != m || triPair.matches[m].trainIdx != expectedTrain[m]) {
					VERBOSE("DenseKeypointBoundaryTest FAILED: intransitive triple (rep %u) match %u is (%u, %u), expected (%u, %u)",
						rep, m, triPair.matches[m].queryIdx, triPair.matches[m].trainIdx, m, expectedTrain[m]);
					return false;
				}
			}
		}
	}

	// The duplicate-match filter's winner selection must know which segment a candidate sits in.
	// Described-wins is right when the rival forms tracks; when the rival is a match the strict
	// cheirality/angle/reprojection filter deliberately REJECTED, keeping it costs a track-forming
	// observation and gains nothing, because the survivor then sits past the track-forming boundary
	// and is never unioned. Here a dense supplement match whose query endpoint was cross-pruned onto
	// a described survivor (1 dense endpoint) shares its remapped queryIdx with a reject described at
	// both ends (0 dense endpoints), which on (NumDenseEnds, weight) alone would win.
	{
		Scene segScene;
		Image& sA = segScene.images.emplace_back(0u, String("segA.jpg"));
		sA.keypoints.emplace_back(10.f, 10.f, 4.f, -1.f, 0.05f); // described: the reject's endpoint
		sA.keypoints.emplace_back(50.f, 50.f, 4.f, -1.f, 0.05f); // described: the sparse inlier's
		sA.CloseDescribedKeypoints();
		// dense, coincident with described 0, so the cross-segment prune reuses that keypoint and
		// the supplement match below ends up sharing queryIdx with the reject
		sA.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(10.f, 10.f), 0.9f, 6.f));
		Image& sB = segScene.images.emplace_back(1u, String("segB.jpg"));
		sB.keypoints.emplace_back(100.f, 100.f, 4.f, -1.f, 0.05f); // the sparse inlier's endpoint
		sB.keypoints.emplace_back(110.f, 110.f, 4.f, -1.f, 0.05f); // the reject's endpoint
		sB.CloseDescribedKeypoints();
		sB.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(200.f, 200.f), 0.9f, 6.f));
		ImagePair& segPair = segScene.pairs.emplace_back(0u, 1u);
		segPair.matches.emplace_back(1, 0); // sparse inlier
		segPair.matches.emplace_back(2, 2); // dense supplement, query endpoint about to be pruned
		segPair.matches.emplace_back(0, 1); // strict-filter reject
		segPair.numFilteredInliers = 1;
		segPair.numDenseInliers = 1;
		MatchConfig segCfg;
		segCfg.minMatches = 1;
		PairsMatcher(segScene, segCfg).FilterRedundantKeypoints();
		// the supplement match survives the group and stays track-forming; the reject is the one
		// dropped, since it could never have formed a track
		if (segPair.matches.size() != 2 || segPair.numFilteredInliers != 1 || segPair.numDenseInliers != 1 ||
			segPair.matches[1].queryIdx != 0 || segPair.matches[1].trainIdx != 2) {
			VERBOSE("DenseKeypointBoundaryTest FAILED: the duplicate-match filter kept the strict-filter reject over "
				"the dense supplement (%u matches, %d sparse, %d dense, second match (%u, %u))",
				(unsigned)segPair.matches.size(), segPair.numFilteredInliers, segPair.numDenseInliers,
				segPair.matches.size() > 1 ? segPair.matches[1].queryIdx : 0u,
				segPair.matches.size() > 1 ? segPair.matches[1].trainIdx : 0u);
			return false;
		}
	}

	VERBOSE("DenseKeypointBoundaryTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

// RoMa2 CPU preprocessing test: a constant image maps to constant planes with the expected
// R/G/B channel swap, and resampling a real fixture image reproduces torch's own
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) to within 1e-5 (fixture
// generated on CPU by scripts/python/roma2/parity.py)
bool RoMa2PreprocessTest()
{
	#ifndef _USE_ONNXRUNTIME
	VERBOSE("RoMa2PreprocessTest: skipped (built without ONNX Runtime)");
	return true;
	#else
	Image8U3 bgr(cv::Size(64, 48), Pixel8U(30, 20, 10)); // 64x48, TPixel(r,g,b): r=30 g=20 b=10
	std::vector<float> planar;
	const int S = 64;
	PreprocessImageRoMa2(bgr, S, planar);
	if (planar.size() != 3u*S*S) {
		VERBOSE("RoMa2PreprocessTest FAILED: unexpected output size %u (expected %u)",
			(unsigned)planar.size(), (unsigned)(3u*S*S));
		return false;
	}
	const float* R = planar.data();
	const float* G = R + S*S;
	const float* B = G + S*S;
	for (int i = 0; i < S*S; ++i) {
		if (!ISEQUAL(R[i], 30.f/255.f, 1e-5f) || !ISEQUAL(G[i], 20.f/255.f, 1e-5f) || !ISEQUAL(B[i], 10.f/255.f, 1e-5f)) {
			VERBOSE("RoMa2PreprocessTest FAILED: constant image");
			return false;
		}
	}

	Image8U3 source;
	if (!source.Load(MAKE_PATH("roma2/preprocess_source.png"))) {
		VERBOSE("RoMa2PreprocessTest FAILED: fixture missing");
		return false;
	}
	std::vector<float> expected;
	if (!ReadFloats(MAKE_PATH("roma2/preprocess_64.bin"), 3u*S*S, expected))
		return false;
	PreprocessImageRoMa2(source, S, planar);
	const float maxErr = MaxAbsDiff(planar, expected);
	if (maxErr > 1e-5f) {
		VERBOSE("RoMa2PreprocessTest FAILED: max preprocessing error %g", maxErr);
		return false;
	}
	DEBUG_EXTRA("RoMa2PreprocessTest: max preprocessing error %g", maxErr);
	return true;
	#endif
}

// RoMa2 manifest format-version test: a manifest declaring format_version 1 (this build's schema)
// loads, and one declaring format_version 3 (a schema this build no longer reads) is rejected.
// RoMa2Manifest::Load is plain JSON parsing, compiled in every build, so this test needs neither
// ONNX Runtime nor OPENMVS_ROMA2_MODEL_PATH.
bool RoMa2ManifestVersionTest()
{
	const ScopedTempDir tmpDir(_T("RoMa2ManifestVersionTest"));
	if (!tmpDir.IsValid())
		return false;

	// A minimal manifest carrying every key RoMa2Manifest::Load reads, at made-up but internally
	// consistent sizes (S=32, cells=4, facetsDim=8): every declared io shape is cross-checked
	// against these, so a mismatch here would fail the test for the wrong reason.
	const auto WriteManifest = [&tmpDir](const String& fileName, int formatVersion) -> String {
		const String path = tmpDir(fileName);
		std::ofstream os(path.c_str());
		os << "{"
		      "\"format_version\":" << formatVersion << ","
		      "\"model\":\"roma2\","
		      "\"setting\":\"turbo\","
		      "\"image_size\":32,"
		      "\"patch\":16,"
		      "\"layers\":[11,17],"
		      "\"descriptor_layers_shape\":[1,2,2,2,8],"
		      "\"warp_size\":4,"
		      "\"confidence_channels\":1,"
		      "\"value_facet_blocks\":[15,20],"
		      "\"value_facets_shape\":[1,2,2,2,8],"
		      "\"opset\":18,"
		      "\"retrieval_recipes\":{\"facets\":{\"dim\":8}},"
		      "\"files\":{\"descriptor\":\"d.onnx\",\"descriptor_data\":\"d.onnx.data\","
		                 "\"match_coarse\":\"m.onnx\",\"match_coarse_data\":\"m.onnx.data\"},"
		      "\"io\":{"
		        "\"descriptor\":{\"inputs\":{\"image\":[1,3,32,32]},"
		                        "\"outputs\":{\"layers\":[1,2,2,2,8],\"value_facets\":[1,2,2,2,8],\"retrieval\":[1,8]}},"
		        "\"match_coarse\":{\"inputs\":{\"descriptors_A\":[1,2,2,2,8],\"descriptors_B\":[1,2,2,2,8]},"
		                          "\"outputs\":{\"warp\":[1,4,4,2],\"confidence\":[1,4,4,1],"
		                                       "\"warp_BA\":[1,4,4,2],\"confidence_BA\":[1,4,4,1]}}"
		      "}"
		   "}";
		return path;
	};

	RoMa2Manifest manifestV1;
	if (!manifestV1.Load(WriteManifest(_T("roma_v1.json"), 1))) {
		VERBOSE("RoMa2ManifestVersionTest FAILED: a format_version 1 manifest did not load");
		return false;
	}
	if (manifestV1.imageSize != 32 || manifestV1.warpSize != 4 || manifestV1.facetsDim != 8) {
		VERBOSE("RoMa2ManifestVersionTest FAILED: a format_version 1 manifest loaded with unexpected sizes");
		return false;
	}

	RoMa2Manifest manifestV3;
	if (manifestV3.Load(WriteManifest(_T("roma_v3.json"), 3))) {
		VERBOSE("RoMa2ManifestVersionTest FAILED: a format_version 3 manifest loaded, expected rejection");
		return false;
	}

	VERBOSE("RoMa2ManifestVersionTest PASSED");
	return true;
}

// FlexGlue manifest test: a manifest of format_version 1 naming its two graphs and the H1 pose head loads with its sizes; another
// format version, another model, a missing key and another pose head are each rejected. Pure JSON parsing.
bool FlexGlueManifestTest()
{
	const ScopedTempDir tmpDir(_T("FlexGlueManifestTest"));
	if (!tmpDir.IsValid())
		return false;
	const auto WriteManifest = [&tmpDir](const String& fileName, int formatVersion, const char* model, const char* poseHead, bool bFiles) -> String {
		const String path = tmpDir(fileName);
		std::ofstream os(path.c_str());
		os << "{\"format_version\":" << formatVersion << ",\"model\":\"" << model << "\",\"setting\":\"base\",\"image_size\":512,\"warp_size\":128,"
		      "\"retrieval_recipes\":{\"features\":{\"dim\":256,\"gem_p\":3}},";
		if (bFiles)
			os << "\"files\":{\"descriptor\":\"d.onnx\",\"match\":\"m.onnx\"},";
		os << "\"heads\":{\"covisibility\":{\"output\":\"covisibility\"},\"pose\":{\"name\":\"" << poseHead << "\"}}}";
		return path;
	};
	FlexGlueManifest manifest;
	if (!manifest.Load(WriteManifest(_T("flexglue_ok.json"), 1, "flexglue", "H1", true)) || manifest.setting != "base" || manifest.imageSize != 512 ||
		manifest.warpSize != 128 || manifest.retrievalDim != 256 || manifest.descriptorFile != "d.onnx" || manifest.matchFile != "m.onnx") {
		VERBOSE("FlexGlueManifestTest FAILED: a valid manifest did not load with its sizes and files");
		return false;
	}
	const struct { const char* name; int version; const char* model; const char* head; bool bFiles; } bad[] = {
		{"flexglue_v2.json", 2, "flexglue", "H1", true}, {"flexglue_roma.json", 1, "roma2", "H1", true},
		{"flexglue_nofiles.json", 1, "flexglue", "H1", false}, {"flexglue_h0.json", 1, "flexglue", "H0", true}};
	for (const auto& b : bad) {
		FlexGlueManifest rejected;
		if (rejected.Load(WriteManifest(b.name, b.version, b.model, b.head, b.bFiles))) {
			VERBOSE("FlexGlueManifestTest FAILED: '%s' loaded, expected rejection", b.name);
			return false;
		}
	}
	VERBOSE("FlexGlueManifestTest PASSED");
	return true;
}

// DenseMatchConfig::ResolveModelPath precedence test, for each matcher (roma2: modelPath and $OPENMVS_ROMA2_MODEL_PATH, flexglue:
// flexglueModelPath and $OPENMVS_FLEXGLUE_MODEL_PATH): the explicit path wins over everything (even a set environment variable);
// with it empty, the matcher's environment variable wins; with both absent, the result falls back to the install prefix's own
// copy (OPENMVS_MODEL_INSTALL_DIR/<matcher>) when that directory actually exists on disk, else it is empty. And a matcher never
// resolves to the other one's explicit path or environment variable. This is the precedence the whole --dense-matcher pipeline depends on
// -- it decides whether the CLI's own "needs a model" error fires -- so it is checked here directly rather than only indirectly
// through CreateStructure. The fallback case must not create that directory, so it reads the same File::isFolder check the
// implementation uses and asserts against whichever outcome is already true on this machine.
bool ResolveModelPathTest()
{
	// Save/restore both variables around this test: the parity and reconstruct tests run right after it in the same process and
	// read them to decide whether to skip, so every exit path here must put the environment back exactly as it was found.
	struct EnvGuard {
		const char* const name;
		const bool hadPrev;
		const String prevValue;
		explicit EnvGuard(const char* _name) : name(_name), hadPrev(getenv(_name) != NULL), prevValue(hadPrev ? getenv(_name) : "") {}
		~EnvGuard() { Set(hadPrev ? prevValue.c_str() : NULL); }
		void Set(const char* value) const {
			#ifdef _MSC_VER
			_putenv_s(name, value ? value : "");
			#else
			if (value)
				setenv(name, value, 1);
			else
				unsetenv(name);
			#endif
		}
	};
	const EnvGuard roma2Env("OPENMVS_ROMA2_MODEL_PATH"), flexglueEnv("OPENMVS_FLEXGLUE_MODEL_PATH");
	for (const String& matcher : {String("roma2"), String("flexglue")}) {
		const bool bFlexGlue = matcher == "flexglue";
		const EnvGuard& ownEnv = bFlexGlue ? flexglueEnv : roma2Env;
		const EnvGuard& otherEnv = bFlexGlue ? roma2Env : flexglueEnv;
		DenseMatchConfig config;
		config.matcher = matcher;
		String& ownPath = bFlexGlue ? config.flexglueModelPath : config.modelPath;
		String& otherPath = bFlexGlue ? config.modelPath : config.flexglueModelPath;
		// the other matcher's explicit path and variable are set throughout: they must never be what this matcher resolves to
		otherPath = "/other/explicit/path";
		otherEnv.Set("/other/env/path");

		// explicit path wins over the environment, even when the environment is also set
		ownPath = "/explicit/model/path";
		ownEnv.Set("/env/model/path");
		String resolved = config.ResolveModelPath();
		if (resolved != "/explicit/model/path") {
			VERBOSE("ResolveModelPathTest FAILED: %s: the explicit path did not win over the environment (got '%s')", matcher.c_str(), resolved.c_str());
			return false;
		}

		// explicit path empty, the matcher's variable set: the environment variable wins
		ownPath.clear();
		resolved = config.ResolveModelPath();
		if (resolved != "/env/model/path") {
			VERBOSE("ResolveModelPathTest FAILED: %s: its environment variable did not win with the explicit path empty (got '%s')", matcher.c_str(), resolved.c_str());
			return false;
		}

		// both absent: the result depends on whether the install-prefix fallback directory actually exists on disk. Read that with
		// the same File::isFolder check the implementation uses rather than creating (or assuming the absence of) the directory, so
		// this test is hermetic on any machine, installed or not.
		ownEnv.Set(NULL);
		resolved = config.ResolveModelPath();
		#ifdef OPENMVS_MODEL_INSTALL_DIR
		const String installPath(String(OPENMVS_MODEL_INSTALL_DIR "/") + matcher);
		if (File::isFolder(installPath)) {
			if (resolved != installPath) {
				VERBOSE("ResolveModelPathTest FAILED: %s: expected the install-prefix fallback '%s' since it exists on disk, got '%s'", matcher.c_str(), installPath.c_str(), resolved.c_str());
				return false;
			}
		} else
		#endif
		if (!resolved.empty()) {
			VERBOSE("ResolveModelPathTest FAILED: %s: expected an empty result with the explicit path and the environment both absent and no install-prefix folder on disk, got '%s'", matcher.c_str(), resolved.c_str());
			return false;
		}
		otherEnv.Set(NULL);
	}

	VERBOSE("ResolveModelPathTest PASSED");
	return true;
}

#ifdef _USE_ONNXRUNTIME
// The reference dump folder of one preset and stage, with its trailing path separator
static String RoMa2ReferenceDir(const String& modelDir, const String& setting, const char* stage)
{
	String dir(modelDir + "real_" + setting + "_" + stage + ".reference");
	return Util::ensureFolderSlash(dir);
}

// The describe stage of one preset against real_<setting>_descriptor.reference: the CPU
// preprocessing of the reference's own source image, the raw `value_facets` and `layers`
// tensors, and the graph's own on-device retrieval pooling, judged by the bounds in its
// parity.json (value_facets/layers) and against the CPU pool_retrieval reference's own
// pooled_facets_A.npy fixture at a tighter bar (retrieval)
static bool RoMa2OnnxParityDescribe(RoMa2Onnx& model, const String& descDir, const String& setting)
{
	RoMa2ParityBounds bounds;
	if (!ReadParityBounds(descDir + "parity.json", bounds))
		return false;
	const RoMa2Manifest& manifest = model.Manifest();
	const int S = model.ImageSize();
	const unsigned numPatches = model.NumPatches();
	const unsigned numSlices = (unsigned)model.LayersShape()[1], channels = (unsigned)model.LayersShape()[4];
	const size_t numTensor = (size_t)numSlices * numPatches * channels;

	// The CPU preprocessing of the reference's own source image reproduces the tensor the
	// reference outputs were computed from. The bound is the fp32 noise floor of a 768x1024 ->
	// SxS antialiased Keys resample, not RoMa2PreprocessTest's 1e-5: the shipped in_image.npy
	// is itself up to 2.72e-5 away from an exact float64 evaluation of the very same filter
	// (1.14e-5 at turbo, 3.6e-7 at fast, 2.72e-5 at base), because the widened antialiasing
	// kernel has large negative lobes and cancels catastrophically on 0..255 samples -- so no
	// fp32 implementation, however ordered, can reproduce it to 1e-5. 1e-4 still separates the
	// right filter from every wrong one by more than an order of magnitude (plain bicubic A=-0.75
	// resamples ~2e-3 differently, align_corners=true ~1e-2), and the exact-arithmetic agreement
	// of the kernel itself stays pinned by RoMa2PreprocessTest's own 1e-5 fixture.
	Image8U3 sourceA;
	if (!sourceA.Load(descDir + "source_A.png")) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: cannot read '%ssource_A.png'", setting.c_str(), descDir.c_str());
		return false;
	}
	std::vector<float> planarA, reference;
	if (!ReadNpyExpect(descDir + "in_image.npy", (size_t)3*S*S, reference))
		return false;
	PreprocessImageRoMa2(sourceA, S, planarA);
	const float diffImage = MaxAbsDiff(planarA, reference);
	if (diffImage > 1e-4f) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: preprocessing differs from the reference by %g", setting.c_str(), diffImage);
		return false;
	}
	DEBUG("RoMa2OnnxParityTest[%s]: max preprocessing difference from the reference %g", setting.c_str(), diffImage);

	// the raw value facets, read back from the device, and the graph's own on-device retrieval pooling
	OrtTensor layers(model.MakeLayers());
	if (!layers.IsValid()) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: cannot allocate the layers tensor", setting.c_str());
		return false;
	}
	std::vector<float> facets, retrieval;
	{
		TD_TIMER_STARTD();
		// this is the same call the describe pass in MatchDense.cpp makes (facetsOut NULL there):
		// it exercises RoMa2Onnx::Describe's retrievalOut readback, which otherwise has no automated coverage
		if (!model.Describe(planarA.data(), layers, &facets, retrieval)) {
			VERBOSE("RoMa2OnnxParityTest[%s] FAILED: describe with the facets read-back", setting.c_str());
			return false;
		}
		DEBUG_EXTRA("RoMa2OnnxParityTest[%s]: Describe (facets read back) %s", setting.c_str(), TD_TIMER_GET_FMT().c_str());
	}
	if (facets.size() != numTensor) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: %u facet values, expected %u",
			setting.c_str(), (unsigned)facets.size(), (unsigned)numTensor);
		return false;
	}
	if (!ReadNpyExpect(descDir + "out_value_facets.npy", numTensor, reference))
		return false;
	const double cosFacets = CosineSimilarity(facets.data(), reference.data(), numTensor);

	// the graph's own on-device retrieval pooling, judged against the CPU pool_retrieval reference's
	// pooled_facets_A.npy fixture at the tighter bar (export.py check's --retrieval-min-cosine
	// default) rather than the looser bounds.minCosine below -- this parity gate exists because
	// this readback path had no automated coverage otherwise; it must keep running now that the CPU
	// pooling it used to also be judged against (PoolRetrievalDescriptor) is gone
	if (retrieval.size() != manifest.facetsDim) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: the graph's retrieval output has %u values, the manifest declares %u",
			setting.c_str(), (unsigned)retrieval.size(), manifest.facetsDim);
		return false;
	}
	if (!ReadNpyExpect(descDir + "pooled_facets_A.npy", manifest.facetsDim, reference))
		return false;
	const double cosRetrieval = CosineSimilarity(retrieval.data(), reference.data(), retrieval.size());
	if (cosRetrieval < 0.99999) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: on-device retrieval cosine %.8f is below 0.99999 (vs pooled_facets_A.npy)",
			setting.c_str(), cosRetrieval);
		return false;
	}

	// the same image described again into a host tensor, so the `layers` output itself -- the
	// device tensor above is never read back -- can be compared too; the retrieval readback is
	// already checked above, so this call's copy of it is discarded, not reused
	OrtTensor layersHost(OrtTensor::Host(model.LayersShape()));
	std::vector<float> discardedRetrieval;
	{
		TD_TIMER_STARTD();
		if (!model.Describe(planarA.data(), layersHost, NULL, discardedRetrieval)) {
			VERBOSE("RoMa2OnnxParityTest[%s] FAILED: describe into a host tensor", setting.c_str());
			return false;
		}
		DEBUG_EXTRA("RoMa2OnnxParityTest[%s]: Describe (layers on the host) %s", setting.c_str(), TD_TIMER_GET_FMT().c_str());
	}
	if (!ReadNpyExpect(descDir + "out_layers.npy", numTensor, reference))
		return false;
	const double cosLayers = CosineSimilarity(layersHost.HostData(), reference.data(), numTensor);
	DEBUG("RoMa2OnnxParityTest[%s]: cosine value_facets %.6f, layers %.6f, retrieval %.8f",
		setting.c_str(), cosFacets, cosLayers, cosRetrieval);
	if (cosFacets < bounds.minCosine || cosLayers < bounds.minCosine) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: describe cosine below %g (value_facets %.6f, layers %.6f)",
			setting.c_str(), bounds.minCosine, cosFacets, cosLayers);
		return false;
	}
	VERBOSE("RoMa2OnnxParityTest[%s] describe passed on %s: cosine %.6f (value_facets) / %.6f (layers), retrieval %.8f",
		setting.c_str(), model.ProviderName().c_str(), cosFacets, cosLayers, cosRetrieval);
	return true;
}

// polyml's check_correspondences in C++, for one match direction: the warp error in pixels of the
// SxS grid over the cells the reference calls overlapping, and the agreement of the overlap
// logit's sign. Shared by both directions of RoMa2OnnxParityMatchCoarse below, which compares a
// computed (warp, confidence) pair to a same-shaped reference dump twice, once per direction.
static bool RoMa2OnnxCheckDirection(const Image32F2& warp, const Image32F& confidence,
	const std::vector<float>& refWarp, const std::vector<float>& refConfidence,
	int S, int cells, const RoMa2ParityBounds& bounds, const String& setting, const char* direction,
	float& outP99, double& outAgreement)
{
	const cv::Size gridSize(S, S);
	std::vector<float> errors;
	errors.reserve((size_t)cells*cells);
	unsigned numAgree = 0;
	for (int r = 0; r < cells; ++r) {
		for (int c = 0; c < cells; ++c) {
			const int i = r*cells + c;
			const float refLogit = refConfidence[i];
			// sign(logit) == sign(refLogit), read off the sigmoid MatchCoarse already applied
			if ((confidence(r, c) >= 0.5f) == (refLogit >= 0.f))
				++numAgree;
			if (1.f/(1.f + std::exp(-refLogit)) < 0.5f)
				continue; // the reference calls this cell non-overlapping: its warp is unconstrained
			const Point2f coord(DenormCoord(warp(r, c), gridSize));
			const Point2f refCoord(DenormCoord(Point2f(refWarp[i*2], refWarp[i*2+1]), gridSize));
			errors.push_back(std::sqrt(SQUARE(coord.x-refCoord.x) + SQUARE(coord.y-refCoord.y)));
		}
	}
	if (errors.empty()) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: the reference overlap gates every %s warp cell", setting.c_str(), direction);
		return false;
	}
	outP99 = Percentile(errors, 99);
	outAgreement = 100. * numAgree / (double)(cells*cells);
	DEBUG("RoMa2OnnxParityTest[%s]: %s warp error p99 %.4f px over %u/%d overlapping cells, logit sign agreement %.4f%%",
		setting.c_str(), direction, outP99, (unsigned)errors.size(), cells*cells, outAgreement);
	if (outP99 > bounds.maxWarpErrorPx || outAgreement < bounds.minAgreementPercent) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: %s warp error p99 %.4f px (bound %g), logit sign agreement %.4f%% (bound %g)",
			setting.c_str(), direction, outP99, bounds.maxWarpErrorPx, outAgreement, bounds.minAgreementPercent);
		return false;
	}
	return true;
}

// The coarse-match stage of one preset against real_<setting>_match_coarse.reference, driven end
// to end: both sources preprocessed, described, and matched, then judged as polyml's
// check_correspondences does, by the bounds in that dump's own parity.json, on both directions
// of the bidirectional graph (warp/confidence A->B, warp_BA/confidence_BA B->A)
static bool RoMa2OnnxParityMatchCoarse(RoMa2Onnx& model, const String& matchDir, const String& setting)
{
	RoMa2ParityBounds bounds;
	if (!ReadParityBounds(matchDir + "parity.json", bounds))
		return false;
	const int S = model.ImageSize(), cells = model.WarpSize();
	Image8U3 sourceA, sourceB;
	if (!sourceA.Load(matchDir + "source_A.png") || !sourceB.Load(matchDir + "source_B.png")) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: cannot read the match sources from '%s'", setting.c_str(), matchDir.c_str());
		return false;
	}
	std::vector<float> planarA, planarB;
	PreprocessImageRoMa2(sourceA, S, planarA);
	PreprocessImageRoMa2(sourceB, S, planarB);
	OrtTensor layersA(model.MakeLayers()), layersB(model.MakeLayers());
	if (!layersA.IsValid() || !layersB.IsValid()) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: cannot allocate the pair descriptor tensors", setting.c_str());
		return false;
	}
	std::vector<float> discardedRetrieval; // this stage matches, it never needs a retrieval descriptor
	if (!model.Describe(planarA.data(), layersA, NULL, discardedRetrieval) ||
		!model.Describe(planarB.data(), layersB, NULL, discardedRetrieval)) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: describe of the match pair", setting.c_str());
		return false;
	}
	Image32F2 warpAB, warpBA;
	Image32F confidenceAB, confidenceBA;
	{
		TD_TIMER_STARTD();
		if (!model.MatchCoarse(layersA, layersB, warpAB, confidenceAB, warpBA, confidenceBA)) {
			VERBOSE("RoMa2OnnxParityTest[%s] FAILED: coarse match", setting.c_str());
			return false;
		}
		// the only pair of the test, so this timing also carries the lazy match-graph load
		DEBUG_EXTRA("RoMa2OnnxParityTest[%s]: MatchCoarse, match-graph load included, %s", setting.c_str(), TD_TIMER_GET_FMT().c_str());
	}
	if (warpAB.cols != cells || warpAB.rows != cells || confidenceAB.cols != cells || confidenceAB.rows != cells ||
		warpBA.cols != cells || warpBA.rows != cells || confidenceBA.cols != cells || confidenceBA.rows != cells) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: warp %dx%d/%dx%d, expected %dx%d",
			setting.c_str(), warpAB.cols, warpAB.rows, warpBA.cols, warpBA.rows, cells, cells);
		return false;
	}
	std::vector<float> refWarpAB, refConfidenceAB, refWarpBA, refConfidenceBA;
	if (!ReadNpyExpect(matchDir + "out_warp.npy", (size_t)cells*cells*2, refWarpAB) ||
		!ReadNpyExpect(matchDir + "out_confidence.npy", (size_t)cells*cells, refConfidenceAB) ||
		!ReadNpyExpect(matchDir + "out_warp_BA.npy", (size_t)cells*cells*2, refWarpBA) ||
		!ReadNpyExpect(matchDir + "out_confidence_BA.npy", (size_t)cells*cells, refConfidenceBA))
		return false;
	float p99AB = 0, p99BA = 0;
	double agreementAB = 0, agreementBA = 0;
	if (!RoMa2OnnxCheckDirection(warpAB, confidenceAB, refWarpAB, refConfidenceAB, S, cells, bounds, setting, "A->B", p99AB, agreementAB) ||
		!RoMa2OnnxCheckDirection(warpBA, confidenceBA, refWarpBA, refConfidenceBA, S, cells, bounds, setting, "B->A", p99BA, agreementBA))
		return false;
	VERBOSE("RoMa2OnnxParityTest[%s] match passed on %s: A->B warp p99 %.4f px agreement %.4f%%, B->A warp p99 %.4f px agreement %.4f%%",
		setting.c_str(), model.ProviderName().c_str(), p99AB, agreementAB, p99BA, agreementBA);
	return true;
}

// One preset of RoMa2OnnxParityTest: load the model, then run both stages against their own dumps
static bool RoMa2OnnxParitySetting(const String& modelDir, const String& setting, const String& provider)
{
	RoMa2Onnx model;
	if (!model.Load(modelDir, setting, provider)) {
		VERBOSE("RoMa2OnnxParityTest[%s] FAILED: cannot load the model from '%s'", setting.c_str(), modelDir.c_str());
		return false;
	}
	DEBUG_EXTRA("RoMa2OnnxParityTest[%s]: %dx%d image, %u slices of %u patches x %u channels, %dx%d warp cells, provider %s",
		setting.c_str(), model.ImageSize(), model.ImageSize(), (unsigned)model.LayersShape()[1],
		model.NumPatches(), (unsigned)model.LayersShape()[4], model.WarpSize(), model.WarpSize(),
		model.ProviderName().c_str());
	return RoMa2OnnxParityDescribe(model, RoMa2ReferenceDir(modelDir, setting, "descriptor"), setting) &&
		RoMa2OnnxParityMatchCoarse(model, RoMa2ReferenceDir(modelDir, setting, "match_coarse"), setting);
}
#endif // _USE_ONNXRUNTIME

// RoMa2 ONNX parity test: runs the exported descriptor and coarse-match graphs through
// RoMa2Onnx and compares them to the Python reference dumps shipped with the models.
// Configured by the environment (the Tests binary takes no options of its own):
// OPENMVS_ROMA2_MODEL_PATH (unset => skipped), OPENMVS_ROMA2_PROVIDER (auto|cuda|coreml|dml|cpu),
// OPENMVS_ROMA2_SETTING (unset => every preset with a reference dump under the model path)
bool RoMa2OnnxParityTest()
{
	#ifndef _USE_ONNXRUNTIME
	VERBOSE("RoMa2OnnxParityTest: skipped (built without ONNX Runtime)");
	return true;
	#else
	if (!RoMa2Onnx::IsAvailable()) {
		VERBOSE("RoMa2OnnxParityTest: skipped (no ONNX Runtime support in this build)");
		return true;
	}
	const char* const envModelPath = getenv("OPENMVS_ROMA2_MODEL_PATH");
	if (envModelPath == NULL || *envModelPath == 0) {
		VERBOSE("RoMa2OnnxParityTest: skipped (OPENMVS_ROMA2_MODEL_PATH not set)");
		return true;
	}
	TD_TIMER_STARTD();
	String modelDir(envModelPath);
	Util::ensureFolderSlash(modelDir);
	const char* const envProvider = getenv("OPENMVS_ROMA2_PROVIDER");
	const String provider(envProvider != NULL && *envProvider != 0 ? String(envProvider) : String("auto"));
	// one setting if OPENMVS_ROMA2_SETTING names it, else every preset whose reference dump is shipped
	const char* const envSetting = getenv("OPENMVS_ROMA2_SETTING");
	std::vector<String> settings;
	if (envSetting != NULL && *envSetting != 0) {
		settings.emplace_back(envSetting);
	} else {
		for (const char* const knownSetting : {"turbo", "fast", "base"})
			if (File::isFile(RoMa2ReferenceDir(modelDir, knownSetting, "descriptor") + "parity.json"))
				settings.emplace_back(knownSetting);
	}
	if (settings.empty()) {
		// nothing to compare against: this model directory ships the graphs but no reference dumps,
		// which the other two ROMA2 tests are perfectly happy with, so skip loudly instead of
		// failing the whole suite. Naming a preset explicitly through OPENMVS_ROMA2_SETTING still
		// fails hard below (RoMa2OnnxParityDescribe cannot open its parity.json): the user then
		// asked for a comparison that cannot be made.
		VERBOSE("RoMa2OnnxParityTest: skipped (no reference dumps under '%s', parity not checked)", modelDir.c_str());
		return true;
	}
	// a failed Load() must leave the model unloaded, never half-built; an unknown setting has no
	// manifest to open, the only Load() failure reachable without corrupting a shipped model file
	DEBUG("RoMa2OnnxParityTest: the 'failed to open RoMa2 manifest' error below is an expected negative check");
	RoMa2Onnx unloaded;
	if (unloaded.Load(modelDir, "no-such-setting", provider) || unloaded.IsLoaded() || unloaded.MakeLayers().IsValid()) {
		VERBOSE("RoMa2OnnxParityTest FAILED: a failed Load() did not leave the model unloaded");
		return false;
	}
	for (const String& setting : settings)
		if (!RoMa2OnnxParitySetting(modelDir, setting, provider)) {
			VERBOSE("RoMa2OnnxParityTest FAILED: preset '%s'", setting.c_str());
			return false;
		}
	VERBOSE("RoMa2OnnxParityTest PASSED: %u preset(s) match the reference dumps (%s)",
		(unsigned)settings.size(), TD_TIMER_GET_FMT().c_str());
	return true;
	#endif
}

#ifdef _USE_ONNXRUNTIME
// One matched pair of a DenseReconstructScene run, reduced to what the checks below compare:
// which pair it is and how large each of its two evidence segments is. A pair the one pass stored
// carries no marker of its own, so a pair the pass created is read off the difference between two
// runs' pair sets; the dense count is what says the pass's fill reached the scene, and it is part
// of the tuple so the determinism runs compare the fill too -- the dense keypoint indices are
// handed out by a serial append whose order is exactly what a parallel pass could disturb.
struct DensePairSummary {
	IIndex ID1, ID2;
	unsigned numMatches, numFilteredInliers, numDenseInliers;

	// order by pair identity alone: two runs of the same scene match the same pairs
	bool operator<(const DensePairSummary& r) const {
		return ID1 != r.ID1 ? ID1 < r.ID1 : ID2 < r.ID2;
	}
	// the (ID1, ID2, numMatches, numFilteredInliers, numDenseInliers) tuple the determinism check compares
	bool operator==(const DensePairSummary& r) const {
		return ID1 == r.ID1 && ID2 == r.ID2 && numMatches == r.numMatches &&
			numFilteredInliers == r.numFilteredInliers && numDenseInliers == r.numDenseInliers;
	}
};
typedef std::vector<DensePairSummary> DensePairSummaries;

// The scene's pairs as those tuples, sorted so that two runs are comparable whatever order
// they happened to store the pairs in
static DensePairSummaries SummarizePairs(const Scene& scene)
{
	DensePairSummaries summaries;
	summaries.reserve(scene.pairs.size());
	for (const ImagePair& pair : scene.pairs)
		summaries.push_back(DensePairSummary{pair.ID1, pair.ID2, (unsigned)pair.matches.size(),
			pair.GetNumFilteredInliers(), pair.GetNumDenseInliers()});
	std::sort(summaries.begin(), summaries.end());
	return summaries;
}

// One configuration of ROMA2ReconstructTest: import the bundled 4-image scene, extract AKAZE features,
// then MatchPairs with the in-process ROMAv2 model describing every image and, when bUseMatching
// is set, matching every candidate pair in the one dense pass INSTEAD of the descriptor batch --
// the verdict on its bidirectional warp, the guided sparse matching of what the verdict admits,
// the dense fill and the store. Checks the global retrieval descriptors the describe pass stores
// and that RETRIEVAL matching (every pair, on four images) connects and geometrically verifies every pair, and hands the
// matched pairs back so the caller can compare whole runs against each other. The caller varies
// nThreads and slotBudget so that both the describe pass's prefetch ring (MINF(2*nThreads, 8)
// buffers) and the dense pass's slot pool have to reuse a buffer/slot mid-pass at least once
// (nThreads=1 -> 2 buffers < 4 images; slotBudget=2 -> 8 loads for the 6 pairs of 4 images, i.e.
// 4 reloads); the pairwise-distinctness check below, and the pair checks, are what would catch a
// stale-buffer or stale-slot bug.
// bViewGraphCalibration turns off the post-matching view-graph calibration, which these runs
// keep off: it optimizes one focal length per camera and then recomputes every relative pose from
// it, so its result depends on how Scene::Import grouped the images into cameras, and that is one
// more moving part between two runs that are compared pair by pair. (The grouping itself is now
// stable: the intermittent EXIF sensor-size parse that used to yield inf/-inf on one of the two
// HEIC images and split the shared camera was an uninitialized read in the container-EXIF path of
// Image::LoadMetadata, fixed there and pinned by ImportMetadataDeterminismTest.)
// With it off, the stored pairs are exactly what the matching produced.
static bool DenseReconstructScene(Scene& scene, DenseMatchConfig denseCfg,
	int expectedDim, bool bViewGraphCalibration, DensePairSummaries& pairSummaries, bool bStaleDescriptors = false)
{
	const String& setting = denseCfg.setting;
	const String& provider = denseCfg.provider;
	const bool bUseMatching = denseCfg.useMatching;
	const unsigned slotBudget = denseCfg.slotBudget;
	// the images' own focal, given and so trusted (720.51 px, no distortion), unlike RealImagesPipelineTest,
	// which forces a deliberately wrong 900 px / k1=0.6 / k2=-0.09 and leans on the view-graph
	// calibrator to recover them from the fundamental matrices. That recovery is exactly what
	// warp-guided matching cannot feed: it keeps, per keypoint of A, the descriptor-best keypoint
	// of B inside a disc around the position the warp predicts, so the stored correspondences are
	// consistent with a whole family of F near the warp's own geometry and the focal a calibrator
	// extracts from them is wildly unstable (measured on these 4 images at turbo with the forced
	// 900 px: 690 px from the descriptor matches, 133 px from the guided ones on the CPU provider,
	// which then drags the bundle to f=315, and a rejected 34450 px estimate on CUDA). With the
	// intrinsics trusted the pairs carry a relative pose straight from matching, which is what the
	// reconstruction stage below actually consumes.
	// The focal is given, not left to the import: the images carry only a 35 mm-equivalent focal
	// (39 mm on the 640x479 frame, 720.51 px), which the import takes as a guess, untrusted, for the
	// lens fit to estimate (Image::LoadMetadata, CalibrateFocalsAndFitLenses); four images give the
	// fit 4 eligible pairs of the MatchConfig::lensFitMinPairs it needs, so it leaves the camera
	// untrusted with its pairs back to their F, and with the view-graph calibration off
	// (bViewGraphCalibration) no pair would carry a relative pose: the star initializer would find
	// no reference view.
	ImportConfig importCfg;
	importCfg.focalLength = 720.51f;
	if (!scene.Import(MAKE_PATH("images"), importCfg)) {
		VERBOSE("ROMA2ReconstructTest FAILED: Import failed");
		return false;
	}
	// two of the four bundled images are HEIC, which only libheif can decode (OpenCV has no
	// HEIF codec), so a build without it enumerates just the two JPGs (RealImagesPipelineTest carries
	// the same guard); every pair-count expectation below is expressed in terms of however
	// many images were actually enumerated, so this test still asserts real numbers either way
	#ifdef _IMAGE_HEIF
	constexpr const char* reason = "";
	constexpr IIndex expectedImages = 4;
	#else
	constexpr const char* reason = " (built without libheif, so the two HEIC images were skipped)";
	constexpr IIndex expectedImages = 2;
	#endif
	if (scene.images.size() != expectedImages) {
		VERBOSE("ROMA2ReconstructTest FAILED: Expected %u images, got %u%s", expectedImages, (unsigned)scene.images.size(), reason);
		return false;
	}
	const IIndex nImages = (IIndex)scene.images.size();
	const size_t expectedPairs = (size_t)nImages*(nImages-1)/2;

	FeatureExtractionConfig featuresCfg;
	featuresCfg.detectorType = FeatureType::AKAZE;
	featuresCfg.maxFeaturesPerCell = 900;
	featuresCfg.minFeaturesPerCell = 400;
	if (!scene.ExtractFeatures(featuresCfg)) {
		VERBOSE("ROMA2ReconstructTest FAILED: ExtractFeatures failed");
		return false;
	}
	if (bStaleDescriptors) {
		// the scene already carries global descriptors of another matcher (2048-D, RoMa2's size): MatchPairs must describe the images again
		FOREACH(i, scene.images) {
			scene.images[i].globalDescriptor = cv::Mat::zeros(1, 2048, CV_32F);
			scene.images[i].globalDescriptor.at<float>(0, i) = 1.f;
		}
		scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
	}

	MatchConfig matchCfg;
	matchCfg.mode = MatchConfig::RETRIEVAL; // the describe pass and the dense pass, one model load
	matchCfg.DefaultsForFeatureType(featuresCfg.detectorType);
	matchCfg.viewGraphCalibrationEnabled = bViewGraphCalibration;

	denseCfg.enabled = true;
	// MatchPairs runs the describe pass for RETRIEVAL mode alone (the match mode decides it), not
	// merely because ROMA2 is enabled; on four images the retrieval proposes every pair, so this
	// test exercises both of ROMA2's passes together in one model load over the whole graph
	if (!scene.MatchPairs(matchCfg, denseCfg, ViewGraphCalibratorConfig())) {
		VERBOSE("ROMA2ReconstructTest FAILED: MatchPairs failed");
		return false;
	}

	if (!scene.status.nState.isSet(Scene::Status::STATE::GLOBAL_DESCRIPTORS)) {
		VERBOSE("ROMA2ReconstructTest FAILED: GLOBAL_DESCRIPTORS state not set");
		return false;
	}
	FOREACH(i, scene.images) {
		const Image& img = scene.images[i];
		if (!img.HasGlobalDescriptor()) {
			VERBOSE("ROMA2ReconstructTest FAILED: image %u has no global descriptor", img.ID);
			return false;
		}
		if (img.globalDescriptor.rows != 1 || img.globalDescriptor.cols != expectedDim) {
			VERBOSE("ROMA2ReconstructTest FAILED: image %u global descriptor is %dx%d, expected 1x%d",
				img.ID, img.globalDescriptor.rows, img.globalDescriptor.cols, expectedDim);
			return false;
		}
		const double descNorm = cv::norm(img.globalDescriptor, cv::NORM_L2);
		if (ABS(descNorm - 1.0) > 1e-4) {
			VERBOSE("ROMA2ReconstructTest FAILED: image %u global descriptor norm %.6f, expected 1", img.ID, descNorm);
			return false;
		}
	}

	// every image's descriptor must actually depend on its own pixels, not on whichever image
	// last occupied a reused prefetch-ring slot: two images described from the same planar
	// buffer would carry bit-for-bit (or near bit-for-bit) identical descriptors, i.e. cosine
	// (== dot product, both unit-norm) essentially 1: the graph is deterministic, so describing
	// twice from one buffer returns the very same vector and the cosine lands within float
	// rounding of exactly 1. The 4 bundled images are close-up shots of the same small scene
	// (RealImagesPipelineTest reconstructs ~2000 shared tracks from them), so even genuinely distinct
	// descriptors sit close to 1 by content similarity alone -- measured empirically at up to
	// 0.99985546 (turbo) and 0.99994838 (base) across both providers here, on this dataset.
	// The worst of those sits 5.2e-5 below 1, so 1-1e-6
	// leaves ~50x of margin above the measured ceiling while still catching a duplicate, which
	// cannot come out below 1-1e-7. The measured maximum is logged so every run carries the
	// evidence for that margin.
	double maxCosine = 0.0;
	for (IIndex i = 0; i < nImages; ++i) {
		for (IIndex j = i + 1; j < nImages; ++j) {
			const double cosine = scene.images[i].globalDescriptor.dot(scene.images[j].globalDescriptor);
			maxCosine = MAXF(maxCosine, cosine);
			if (cosine >= 1.0 - 1e-6) {
				VERBOSE("ROMA2ReconstructTest FAILED: images %u and %u global descriptors are near-identical (cosine %.8f)",
					scene.images[i].ID, scene.images[j].ID, cosine);
				return false;
			}
		}
	}

	// exhaustive matching: all n*(n-1)/2 candidate pairs, unaffected by the describe pass. The one
	// pass never proposes a pair outside the candidate list it was handed, and on these four
	// close-up shots of one small scene its verdict admits every one of them, so the count is the
	// same either way -- what differs is the evidence each pair carries
	// (a covisibility gate drops pairs on purpose: the caller then checks which ones)
	const bool bGated = denseCfg.matcher == "flexglue" && denseCfg.minCovisibility > 0.f;
	if (bGated ? (scene.pairs.empty() || scene.pairs.size() > expectedPairs) : scene.pairs.size() != expectedPairs) {
		VERBOSE("ROMA2ReconstructTest FAILED: expected %u geometrically matched pairs, got %u",
			(unsigned)expectedPairs, (unsigned)scene.pairs.size());
		return false;
	}
	for (const ImagePair& pair : scene.pairs) {
		// the one pass's evidence may be dense as well as sparse (a pair whose guided matching
		// found little is still stored, its dense fill its whole evidence), so the bar there is the
		// track-forming set rather than the descriptor inliers alone
		const unsigned numEvidence = bUseMatching ? pair.GetNumTrackFormingMatches() : pair.GetNumFilteredInliers();
		if (!pair.HasGeometricVerification() || numEvidence < matchCfg.minMatches) {
			VERBOSE("ROMA2ReconstructTest FAILED: pair (% 4u, % 4u) not geometrically verified (%u sparse + %u dense, %s geometry)",
				pair.ID1, pair.ID2, pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(),
				pair.HasGeometricVerification() ? "with" : "without");
			return false;
		}
	}
	pairSummaries = SummarizePairs(scene);
	DEBUG("ROMA2ReconstructTest[%s]: %u images described (%d-D, %s provider, %u threads, max pairwise cosine %.8f), %u pairs matched (dense matching %s, %u slots)",
		setting.c_str(), (unsigned)scene.images.size(), expectedDim, provider.c_str(), scene.nMaxThreads, maxCosine,
		(unsigned)scene.pairs.size(), bUseMatching ? "on" : "off", slotBudget);
	return true;
}
#endif // _USE_ONNXRUNTIME

// ROMA2 reconstruct test: runs the whole import/AKAZE/RETRIEVAL pipeline of RealImagesPipelineTest
// with the in-process ROMAv2 model attached, and checks both of its passes -- the describe pass
// (per-image global retrieval descriptors, 2048-D, the graph's own on-device pooling) and the ONE
// PASS that replaces the descriptor batch outright when --dense-match is on: every candidate pair
// judged on its bidirectional warp, the admitted ones guided, filled densely and stored. The one
// pass is measured against a baseline run of the very same configuration with it switched off, so
// what is asserted is what the warps actually produced; the run is then repeated on a fresh scene
// to prove determinism (design decision 11), round-tripped through Scene::Save/Load, and finished
// with RealImagesPipelineTest's own reconstruction stage. Configured by the environment like
// RoMa2OnnxParityTest: OPENMVS_ROMA2_MODEL_PATH (unset => skipped), OPENMVS_ROMA2_SETTING
// (default "base", matching DenseMatchConfig::setting's own default and the published model bundle),
// OPENMVS_ROMA2_PROVIDER (default "auto")
bool ROMA2ReconstructTest()
{
	#ifndef _USE_ONNXRUNTIME
	VERBOSE("ROMA2ReconstructTest: skipped (built without ONNX Runtime)");
	return true;
	#else
	if (!RoMa2Onnx::IsAvailable()) {
		VERBOSE("ROMA2ReconstructTest: skipped (no ONNX Runtime support in this build)");
		return true;
	}
	const char* const envModelPath = getenv("OPENMVS_ROMA2_MODEL_PATH");
	if (envModelPath == NULL || *envModelPath == 0) {
		VERBOSE("ROMA2ReconstructTest: skipped (OPENMVS_ROMA2_MODEL_PATH not set)");
		return true;
	}
	TD_TIMER_STARTD();
	const char* const envSetting = getenv("OPENMVS_ROMA2_SETTING");
	const String setting(envSetting != NULL && *envSetting != 0 ? String(envSetting) : String("base"));
	const char* const envProvider = getenv("OPENMVS_ROMA2_PROVIDER");
	const String provider(envProvider != NULL && *envProvider != 0 ? String(envProvider) : String("auto"));
	const auto denseConfig = [&](bool bUseMatching, unsigned slotBudget) {
		DenseMatchConfig cfg;
		cfg.setting = setting;
		cfg.provider = provider;
		cfg.useMatching = bUseMatching;
		cfg.slotBudget = slotBudget;
		return cfg;
	};

	// 1) baseline: the same run with the one pass switched off, i.e. the ordinary descriptor
	// matching, so that comparing it against the dense run below isolates exactly what the warps
	// produced. Both runs keep the view-graph calibration off, like the determinism runs and for
	// the same reason: it re-solves a focal per camera and then re-filters every pair's inliers
	// through it (Scene.cpp:604-613), so how the import grouped the images into cameras would move
	// inlier counts on its own -- and inlier counts are exactly what the assertions below read.
	// (The grouping no longer varies between runs: the intermittent import camera split that
	// originally forced this choice was an uninitialized read in Image::LoadMetadata's
	// container-EXIF path, fixed there and pinned by ImportMetadataDeterminismTest.)
	DensePairSummaries baseline;
	{
		Scene scene(2);
		if (!DenseReconstructScene(scene, denseConfig(false, 64), 2048, false, baseline)) {
			VERBOSE("ROMA2ReconstructTest FAILED: baseline run (one pass off)");
			return false;
		}
	}

	// 2) the same scene through the one pass
	DensePairSummaries guided;
	{
		Scene scene(2);
		if (!DenseReconstructScene(scene, denseConfig(true, 64), 2048, false, guided)) {
			VERBOSE("ROMA2ReconstructTest FAILED: one-pass dense matching");
			return false;
		}
		// the pass is judged on the pairs it stored, read off the two runs' summaries: every
		// candidate of these four close-up shots is co-visible, so the verdict admits them all and
		// the pair set is the baseline's (both summaries are sorted by pair identity, so this is a
		// plain set comparison), while the evidence is the pass's own -- a dense segment on every
		// pair, and a sparse segment that came from the guided matching rather than the descriptor
		// batch, so it cannot be the baseline's. Nothing here demands MORE sparse inliers than the
		// baseline: the guided pass restricts each keypoint of A to a disc around the warp's
		// prediction and tests it against the best keypoint OUTSIDE that disc, which is a different
		// -- and stricter -- selection, not a superset of the descriptor batch's.
		if (baseline.size() != guided.size()) {
			VERBOSE("ROMA2ReconstructTest FAILED: %u pairs through the one pass, %u through descriptor matching",
				(unsigned)guided.size(), (unsigned)baseline.size());
			return false;
		}
		unsigned numDenseFilled = 0, numSparseDiffers = 0, numDense = 0;
		FOREACH(i, guided) {
			if (guided[i].ID1 != baseline[i].ID1 || guided[i].ID2 != baseline[i].ID2) {
				VERBOSE("ROMA2ReconstructTest FAILED: the one pass stored a different pair set");
				return false;
			}
			if (guided[i].numDenseInliers > 0)
				++numDenseFilled;
			if (guided[i].numFilteredInliers != baseline[i].numFilteredInliers)
				++numSparseDiffers;
			numDense += guided[i].numDenseInliers;
		}
		// the sparse segment is compared on its own count, not on the whole summary tuple: the
		// baseline carries no dense correspondence at all, so any tuple comparison would report
		// every pair as different whatever the guided matching did, and say nothing about it
		if (numDenseFilled != guided.size() || 2*numSparseDiffers <= guided.size()) {
			VERBOSE("ROMA2ReconstructTest FAILED: %u of the %u stored pairs carry a dense segment and %u a sparse "
				"segment of a different size than the descriptor batch's, expected all and most "
				"(see the 'ROMA2 one pass' summary line above)",
				numDenseFilled, (unsigned)guided.size(), numSparseDiffers);
			return false;
		}
		DEBUG("ROMA2ReconstructTest: the one pass stored %u pairs with %u dense correspondences, %u of them "
			"carrying a sparse segment of a different size than the descriptor batch's",
			(unsigned)guided.size(), numDense, numSparseDiffers);

		// the global descriptors and the matched pairs survive a scene file round-trip
		{
			const ScopedTempDir tmpDir(_T("ROMA2ReconstructTest"));
			if (!tmpDir.IsValid())
				return false;
			const String sfmPath(tmpDir(_T("dense_matched.sfm")));
			if (!scene.Save(sfmPath)) {
				VERBOSE("ROMA2ReconstructTest FAILED: cannot save '%s'", sfmPath.c_str());
				return false;
			}
			Scene loaded(2);
			if (!loaded.Load(sfmPath)) {
				VERBOSE("ROMA2ReconstructTest FAILED: cannot load back '%s'", sfmPath.c_str());
				return false;
			}
			if (loaded.images.size() != scene.images.size() ||
				!loaded.status.nState.isSet(Scene::Status::STATE::GLOBAL_DESCRIPTORS)) {
				VERBOSE("ROMA2ReconstructTest FAILED: round-trip lost the images or the GLOBAL_DESCRIPTORS state");
				return false;
			}
			FOREACH(i, loaded.images) {
				const cv::Mat& saved = scene.images[i].globalDescriptor;
				const cv::Mat& read = loaded.images[i].globalDescriptor;
				if (read.size() != saved.size() || read.type() != saved.type() || cv::norm(read, saved, cv::NORM_INF) != 0.0) {
					VERBOSE("ROMA2ReconstructTest FAILED: round-trip lost image %u's global descriptor", loaded.images[i].ID);
					return false;
				}
			}
			if (SummarizePairs(loaded) != guided) {
				VERBOSE("ROMA2ReconstructTest FAILED: round-trip changed the matched pairs");
				return false;
			}
		}

		// the rest of the RealImagesPipelineTest flow still passes on the one-pass scene; it needs all four
		// bundled images (the star initializer asks for three views per track), so it only runs in
		// a build that can decode the two HEIC ones.
		// RealImagesPipelineTest's expectations, at two bounds this path has to set for itself:
		//  - THE TRACK COUNT. The dense fill draws --dense-matches correspondences per full
		//    frame of the overlap its guided matches did not already cover, each one a keypoint in
		//    both images, so the pass hands the track builder several times what the descriptor
		//    batch does: ~7.7k inlier tracks (turbo, CUDA) against the ~2.2k of the descriptor-only
		//    run. [5000, 12000] is that measurement with margin either side, and nothing more: the
		//    dense and guided correspondences of six pairs merge into tracks at a ratio nothing here
		//    bounds, so the fill's budget does not derive the window. Both numbers were measured
		//    under the flat per-pair cap the density rule replaced, and the density draws strictly
		//    less, so the window is loose at its top rather than wrong -- re-measure it. A model,
		//    preset or budget change moves it too: re-measure against the track count this stage
		//    prints and the cap, pitch and dense total the line above reports, and set the window
		//    to the new measurement rather than reasoning about it.
		//  - THE RESIDUAL DISTORTION. Guided matching keeps, per keypoint of A, the descriptor-best
		//    keypoint of B inside a disc around the warp's prediction, and the dense segment is the
		//    warp itself, so the correspondences carry that warp's bias and the bundle absorbs it
		//    into the distortion coefficients of a scene that has none: measured at 10.1 px (turbo,
		//    CUDA) against the 1.5 px of RealImagesPipelineTest. 20 px keeps the check meaningful (a
		//    blown-up distortion is still caught) while leaving room above the measured value.
		// Note what the focal check means here, which differs from RealImagesPipelineTest: the focal given
		// at import is 720.51 px, so the `focal_error > 100` bound is already satisfied before the
		// bundle runs and this stage asserts that the one-pass scene keeps the focal stable, not
		// that it recovers it -- recovery from a deliberately wrong 900 px is what RealImagesPipelineTest
		// still tests, on the descriptor matches that can support it.
		#ifdef _IMAGE_HEIF
		if (!ReconstructMatchedScene(scene, "ROMA2ReconstructTest", 5000, 12000, 20))
			return false;
		#endif
	}

	// 3+4) determinism (design decision 11): the very same configuration run twice on two fresh
	// scenes must store the very same pairs, sparse and dense segments alike, the pairs being
	// stored serially in (ID1, ID2) order however the pool interleaved them. The configuration is
	// deliberately the awkward one: a single thread, with a slot pool too small to hold the scene,
	// so that the describe pass reuses a prefetch buffer and the one pass reloads evicted slots (see the
	// DenseReconstructScene comment) -- the paths where a stale buffer or slot would show up as a
	// difference. The view-graph calibration is off here so that the comparison sees the matching
	// alone (again, see DenseReconstructScene).
	DensePairSummaries tightPool, repeated;
	{
		Scene scene(1);
		if (!DenseReconstructScene(scene, denseConfig(true, 2), 2048, false, tightPool)) {
			VERBOSE("ROMA2ReconstructTest FAILED: 2-slot pool run");
			return false;
		}
	}
	{
		Scene scene(1);
		if (!DenseReconstructScene(scene, denseConfig(true, 2), 2048, false, repeated)) {
			VERBOSE("ROMA2ReconstructTest FAILED: determinism run");
			return false;
		}
	}
	if (repeated != tightPool) {
		VERBOSE("ROMA2ReconstructTest FAILED: two runs of the same configuration matched different pairs");
		FOREACH(i, repeated)
			if (i >= tightPool.size() || !(repeated[i] == tightPool[i]))
				VERBOSE("  pair (% 4u, % 4u): %u matches / %u sparse / %u dense, was (% 4u, % 4u): %u matches / %u sparse / %u dense",
					repeated[i].ID1, repeated[i].ID2, repeated[i].numMatches, repeated[i].numFilteredInliers, repeated[i].numDenseInliers,
					i < tightPool.size() ? tightPool[i].ID1 : NO_ID, i < tightPool.size() ? tightPool[i].ID2 : NO_ID,
					i < tightPool.size() ? tightPool[i].numMatches : 0, i < tightPool.size() ? tightPool[i].numFilteredInliers : 0,
					i < tightPool.size() ? tightPool[i].numDenseInliers : 0);
		return false;
	}
	VERBOSE("ROMA2ReconstructTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
	#endif
}

#ifdef _USE_ONNXRUNTIME
// The angle in degrees of the rotation R
static double RotationAngleDeg(const Matrix3x3f& R)
{
	return R2D(acos(CLAMP(((double)R(0,0) + R(1,1) + R(2,2) - 1.0) * 0.5, -1.0, 1.0)));
}
#endif

// FlexGlue as the dense matcher, on the bundled 4-image scene through Scene::MatchPairs (docs/design/DenseMatching.md): the 256-D retrieval
// descriptors its backbone pools (unit, distinct per image), every pair stored with its covisibility score and its coarse relative pose
// (a rotation, a unit baseline direction, close to the pose the pair's own matches give: the check of the pose head's convention),
// the pairs CSV carrying both, the covisibility gate dropping exactly the pairs below it, and RealImagesPipelineTest's
// reconstruction stage on the matched scene. The run sees no RoMa2 model at all (its explicit path names a folder that does not exist and
// OPENMVS_ROMA2_MODEL_PATH is unset for the duration): the describe pass, the retrieval and the matching load the FlexGlue bundle alone.
// Skipped unless OPENMVS_FLEXGLUE_MODEL_PATH points at a FlexGlue bundle (flexglue_base.json and its two graphs); OPENMVS_ROMA2_PROVIDER
// narrows the execution provider
bool FlexGlueReconstructTest()
{
	#ifndef _USE_ONNXRUNTIME
	VERBOSE("FlexGlueReconstructTest: skipped (built without ONNX Runtime)");
	return true;
	#else
	if (!RoMa2Onnx::IsAvailable()) {
		VERBOSE("FlexGlueReconstructTest: skipped (no ONNX Runtime support in this build)");
		return true;
	}
	const char* const envModelPath = getenv("OPENMVS_FLEXGLUE_MODEL_PATH");
	if (envModelPath == NULL || *envModelPath == 0) {
		VERBOSE("FlexGlueReconstructTest: skipped (OPENMVS_FLEXGLUE_MODEL_PATH not set)");
		return true;
	}
	TD_TIMER_STARTD();
	const char* const envProvider = getenv("OPENMVS_ROMA2_PROVIDER");
	DenseMatchConfig cfg;
	cfg.matcher = "flexglue";
	cfg.flexglueModelPath = envModelPath;
	cfg.provider = envProvider != NULL && *envProvider != 0 ? String(envProvider) : String("auto");
	cfg.useMatching = true;
	cfg.slotBudget = 64;
	cfg.minCovisibility = 0.f;
	// no RoMa2 model reachable: a RoMa2 load anywhere on the FlexGlue route fails the matching below
	cfg.modelPath = "/nonexistent/roma2-model";
	const String prevRoMa2Path(getenv("OPENMVS_ROMA2_MODEL_PATH") ? getenv("OPENMVS_ROMA2_MODEL_PATH") : "");
	const bool bHadRoMa2Path = getenv("OPENMVS_ROMA2_MODEL_PATH") != NULL;
	#ifdef _MSC_VER
	_putenv_s("OPENMVS_ROMA2_MODEL_PATH", "");
	#else
	unsetenv("OPENMVS_ROMA2_MODEL_PATH");
	#endif

	// 1) every pair kept: covisibility, pose prior and the CSV
	DensePairSummaries summaries;
	Scene scene(2);
	const bool bMatched = DenseReconstructScene(scene, cfg, FlexGlueMatcher::RETRIEVAL_SIZE, false, summaries, true);
	if (bHadRoMa2Path) {
		#ifdef _MSC_VER
		_putenv_s("OPENMVS_ROMA2_MODEL_PATH", prevRoMa2Path.c_str());
		#else
		setenv("OPENMVS_ROMA2_MODEL_PATH", prevRoMa2Path.c_str(), 1);
		#endif
	}
	if (!bMatched) {
		VERBOSE("FlexGlueReconstructTest FAILED: matching with FlexGlue (no RoMa2 model reachable)");
		return false;
	}
	std::vector<float> scores;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.covisibility || !(*pair.covisibility >= 0.f && *pair.covisibility <= 1.f) || !pair.posePrior || !pair.relativePose) {
			VERBOSE("FlexGlueReconstructTest FAILED: pair (%u, %u) lacks a covisibility score in [0,1], a pose prior or its own relative pose", pair.ID1, pair.ID2);
			return false;
		}
		scores.push_back(*pair.covisibility);
		const PosePrior& prior = *pair.posePrior;
		const Matrix3x3f RtR(prior.R.t() * prior.R);
		const double orthonormality = cv::norm(RtR - Matrix3x3f::IDENTITY), norm = cv::norm(prior.t);
		if (orthonormality > 1e-4 || ABS(cv::determinant(prior.R) - 1.0) > 1e-4 || ABS(norm - 1.0) > 1e-4) {
			VERBOSE("FlexGlueReconstructTest FAILED: pair (%u, %u) pose prior is not a rotation and a unit direction (orthonormality %g, |t| %g)", pair.ID1, pair.ID2, orthonormality, norm);
			return false;
		}
		// the prior against the pose the pair's own matches give, both in the pair's convention x2 = R*x1 + s*t: a convention error of the head
		// (a transposed rotation, the baseline's sign, the frame of t) is far from a few degrees on these pairs of 5 to 14 degrees of rotation and a baseline
		const Matrix3x3f own(pair.relativePose->R);
		const double rotationError = RotationAngleDeg(own.t() * prior.R);
		const CMatrix t(pair.relativePose->GetT());
		const double directionError = R2D(acos(CLAMP((t.x*prior.t.x + t.y*prior.t.y + t.z*prior.t.z) / cv::norm(t), -1.0, 1.0)));
		DEBUG_EXTRA("FlexGlueReconstructTest: pair (%u, %u) covisibility %.4f, prior %.2f deg in rotation and %.2f deg in direction from the pair's own pose", pair.ID1, pair.ID2, *pair.covisibility, rotationError, directionError);
		if (rotationError > 12. || directionError > 30.) {
			VERBOSE("FlexGlueReconstructTest FAILED: pair (%u, %u) pose prior is %.1f deg in rotation and %.1f deg in direction from the pair's own pose", pair.ID1, pair.ID2, rotationError, directionError);
			return false;
		}
	}
	{
		const ScopedTempDir tmpDir(_T("FlexGlueReconstructTest"));
		if (!tmpDir.IsValid())
			return false;
		const String csvPath(tmpDir(_T("pairs.csv")));
		if (!PairsMatcher::ExportPairsCSV(scene, csvPath, 0.f, 0.f, PairsWeightingConfig())) {
			VERBOSE("FlexGlueReconstructTest FAILED: cannot export the pairs CSV");
			return false;
		}
		std::ifstream ifs(csvPath);
		std::string header, row;
		std::getline(ifs, header);
		const auto numFields = [](const std::string& line) { return (size_t)std::count(line.begin(), line.end(), ',') + 1; };
		if (header.find(",Covisibility,PriorQw,PriorQx,PriorQy,PriorQz,PriorTx,PriorTy,PriorTz") == std::string::npos || numFields(header) != 25) {
			VERBOSE("FlexGlueReconstructTest FAILED: the pairs CSV header lacks the covisibility and pose prior columns: %s", header.c_str());
			return false;
		}
		size_t numRows = 0;
		while (std::getline(ifs, row)) {
			if (numFields(row) != 25) {
				VERBOSE("FlexGlueReconstructTest FAILED: a pairs CSV row has %u fields, expected 25", (unsigned)numFields(row));
				return false;
			}
			++numRows;
		}
		if (numRows != scene.pairs.size()) {
			VERBOSE("FlexGlueReconstructTest FAILED: the pairs CSV has %u rows for %u pairs", (unsigned)numRows, (unsigned)scene.pairs.size());
			return false;
		}
	}

	// 2) the gate keeps the pairs at or above its bar: the median score of the first run, the scores of a second run moving a little with the GPU
	std::vector<float> sorted(scores);
	std::sort(sorted.begin(), sorted.end());
	const float bar = 0.5f * (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]);
	if (!(sorted.front() < bar && bar < sorted.back())) {
		VERBOSE("FlexGlueReconstructTest FAILED: the six scores do not spread (%g .. %g), the gate has nothing to separate", sorted.front(), sorted.back());
		return false;
	}
	{
		Scene gated(2);
		DenseMatchConfig gateCfg(cfg);
		gateCfg.minCovisibility = bar;
		DensePairSummaries gatedSummaries;
		if (!DenseReconstructScene(gated, gateCfg, FlexGlueMatcher::RETRIEVAL_SIZE, false, gatedSummaries)) {
			VERBOSE("FlexGlueReconstructTest FAILED: matching with the covisibility gate at %g", bar);
			return false;
		}
		if (gated.pairs.empty() || gated.pairs.size() >= scene.pairs.size()) {
			VERBOSE("FlexGlueReconstructTest FAILED: the gate at %g kept %u of %u pairs", bar, (unsigned)gated.pairs.size(), (unsigned)scene.pairs.size());
			return false;
		}
		// both ways: every pair the first run scored clearly above the bar is kept, every one clearly below it is gone (a second GPU run moves a score
		// by around 1e-5, so the pairs within that of the bar are not judged)
		constexpr float noise = 2e-5f;
		for (const ImagePair& pair : scene.pairs) {
			const bool bKept = std::any_of(gated.pairs.begin(), gated.pairs.end(), [&](const ImagePair& g) { return g.ID1 == pair.ID1 && g.ID2 == pair.ID2; });
			if ((*pair.covisibility > bar + noise && !bKept) || (*pair.covisibility < bar - noise && bKept)) {
				VERBOSE("FlexGlueReconstructTest FAILED: with the gate at %g pair (%u, %u) of covisibility %g was %s", bar, pair.ID1, pair.ID2, *pair.covisibility, bKept ? "kept" : "dropped");
				return false;
			}
		}
		for (const ImagePair& pair : gated.pairs)
			if (!pair.covisibility || *pair.covisibility < bar) {
				VERBOSE("FlexGlueReconstructTest FAILED: the gate at %g kept pair (%u, %u) of covisibility %g", bar, pair.ID1, pair.ID2, pair.covisibility ? *pair.covisibility : -1.f);
				return false;
			}
	}

	// 3) the reconstruction stage of RealImagesPipelineTest on the matched scene
	#ifdef _IMAGE_HEIF
	if (!ReconstructMatchedScene(scene, "FlexGlueReconstructTest", 4000, 12000, 20))
		return false;
	#endif
	VERBOSE("FlexGlueReconstructTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
	#endif
}

// ===============================================================================
// Spherical bottom band of the dense matching (DenseMatchConfig::sphericalBottomBand, the value of
// FeatureExtractionConfig::sphericalBottomBand): the dense correspondences both warp draws take
// (the verdict's SampleWarpByCoverage and the fill's SampleWarpComplementary, hence both matchers) have no
// end in the bottom band of a spherical image, whichever image it is. With an identity warp the rows
// above the band keep exactly the samples they have without the band; with a warp that sends rows above
// A's band into B's band, the survivors still hold none in the band and every sample of the band-0 draw
// with both ends outside it. A perspective pair is unaffected by the option.
// ===============================================================================
bool SphericalBottomBandDenseTest()
{
	VERBOSE("\n=== SphericalBottomBandDenseTest: dense samples in the bottom rows of a spherical image ===");
	constexpr int width = 2048, height = 1024, cells = 40;
	constexpr float band = 0.2f;
	const cv::Size size(width, height), grid(cells, cells);
	Scene scene;
	scene.cameras.emplace_back(new SphericalCamera(size));
	scene.cameras.emplace_back(new PinholeCamera(size, REAL(1000), REAL(1000), REAL(width)/2, REAL(height)/2));
	for (IIndex i = 0; i < 4; ++i) {
		scene.images.emplace_back(i, String("i.jpg"));
		scene.images[i].cameraID = i < 2 ? 0 : 1; // 0, 1 spherical, 2, 3 perspective
		scene.images[i].pCamera = scene.cameras[scene.images[i].cameraID];
	}
	const Image& sphA = scene.images[0];
	const Image& sphB = scene.images[1];
	const Image& perA = scene.images[2];
	const Image& perB = scene.images[3];
	const float bandRow = (1.f - band) * (float)height;
	const auto InBand = [&](const Point2f& pt) { return pt.y > bandRow; };
	// the identity warp, and a warp sending every cell rows 8 lower (cells 24..31 above A's band land in B's band)
	Image32F2 identity(grid), shifted(grid);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x) {
			const Point2f px(CellToPixel(Point2f((float)x, (float)y), grid, size));
			identity(y, x) = NormCoord(px, size);
			shifted(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)(y+8)), grid, size), size);
		}
	const Image32F confidence(grid, 1.f);
	// the buckets of the fill, 20 x 20: the band's edge (row 32 of 40) is a bucket edge, so a bucket never straddles it
	constexpr int numBuckets = 20;
	const std::vector<Point2f> noneOccupied;

	// the same draw with the band off and on, through both draws
	struct Draw { std::vector<Point2f> a, b; std::vector<float> c; };
	const auto Fill = [&](const Image& imgA, const Image& imgB, const Image32F2& warp, float bandValue) {
		Draw d;
		SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, bandValue, numBuckets, cells*cells, noneOccupied, d.a, d.b, d.c);
		return d;
	};
	const auto Coverage = [&](const Image& imgA, const Image& imgB, const Image32F2& warp, float bandValue) {
		Draw d;
		SampleWarpByCoverage(imgA, imgB, warp, confidence, 0.3f, bandValue, cells*cells, d.a, d.b);
		return d;
	};
	const auto SameDraw = [](const Draw& x, const Draw& y) { return x.a == y.a && x.b == y.b; };
	const auto Survivors = [&](const Draw& off, const Draw& on) {
		// every sample of `off` with both ends out of the band is in `on`
		size_t j = 0;
		for (size_t i = 0; i < off.a.size(); ++i) {
			if (InBand(off.a[i]) || InBand(off.b[i]))
				continue;
			while (j < on.a.size() && !(on.a[j] == off.a[i] && on.b[j] == off.b[i]))
				++j;
			if (j == on.a.size())
				return false;
		}
		return true;
	};
	const auto NoneInBand = [&](const Draw& d) {
		for (size_t i = 0; i < d.a.size(); ++i)
			if (InBand(d.a[i]) || InBand(d.b[i]))
				return false;
		return true;
	};
	const auto CountInBand = [&](const Draw& d) {
		size_t n = 0;
		for (size_t i = 0; i < d.a.size(); ++i)
			n += InBand(d.a[i]) || InBand(d.b[i]);
		return n;
	};
	const std::vector<std::pair<const char*, bool>> draws{{"fill", true}, {"verdict sample", false}};
	for (const auto& [name, bFill] : draws) {
		const auto Run = [&](const Image& a, const Image& b, const Image32F2& warp, float bandValue) {
			return bFill ? Fill(a, b, warp, bandValue) : Coverage(a, b, warp, bandValue);
		};
		for (const Image32F2* warp : {&identity, &shifted}) {
			const bool bIdentity = warp == &identity;
			const Draw off = Run(sphA, sphB, *warp, 0.f), on = Run(sphA, sphB, *warp, band);
			VERBOSE("  %s, %s warp: band 0 %u samples (%u with an end in the band), band %g %u samples", name,
				bIdentity ? "identity" : "shifted", (unsigned)off.a.size(), (unsigned)CountInBand(off), band, (unsigned)on.a.size());
			if (CountInBand(off) < 40) {
				VERBOSE("SphericalBottomBandDenseTest FAILED: the band-0 %s has only %u samples in the band, the setup does not exercise it", name, (unsigned)CountInBand(off));
				return false;
			}
			if (!NoneInBand(on) || on.a.empty()) {
				VERBOSE("SphericalBottomBandDenseTest FAILED: the %s keeps %u samples with an end in the band (%u samples)", name, (unsigned)CountInBand(on), (unsigned)on.a.size());
				return false;
			}
			if (!Survivors(off, on)) {
				VERBOSE("SphericalBottomBandDenseTest FAILED: the band removed a %s sample of the rows above the band", name);
				return false;
			}
			if (bIdentity && bFill) {
				// the identity warp puts both ends in the same row: the fill's samples above the band are exactly the band-0 ones
				Draw offAbove;
				for (size_t i = 0; i < off.a.size(); ++i)
					if (!InBand(off.a[i])) {
						offAbove.a.push_back(off.a[i]);
						offAbove.b.push_back(off.b[i]);
					}
				if (!SameDraw(offAbove, on)) {
					VERBOSE("SphericalBottomBandDenseTest FAILED: the rows above the band hold %u fill samples, %u without the band", (unsigned)on.a.size(), (unsigned)offAbove.a.size());
					return false;
				}
			}
		}
		// a spherical image in either role: the band holds for the end in the spherical image, the perspective end is free
		const Draw sphPerOff = Run(sphA, perB, identity, 0.f), sphPerOn = Run(sphA, perB, identity, band);
		const Draw perSphOff = Run(perA, sphB, identity, 0.f), perSphOn = Run(perA, sphB, identity, band);
		if (sphPerOn.a.empty() || perSphOn.a.empty() || sphPerOn.a.size() >= sphPerOff.a.size() || perSphOn.a.size() >= perSphOff.a.size()) {
			VERBOSE("SphericalBottomBandDenseTest FAILED: the %s of a spherical and a perspective image holds %u/%u samples with the band, %u/%u without", name,
				(unsigned)sphPerOn.a.size(), (unsigned)perSphOn.a.size(), (unsigned)sphPerOff.a.size(), (unsigned)perSphOff.a.size());
			return false;
		}
		for (const Point2f& pt : sphPerOn.a)
			if (InBand(pt)) {
				VERBOSE("SphericalBottomBandDenseTest FAILED: the %s keeps a sample in the band of the spherical image A", name);
				return false;
			}
		for (const Point2f& pt : perSphOn.b)
			if (InBand(pt)) {
				VERBOSE("SphericalBottomBandDenseTest FAILED: the %s keeps a sample in the band of the spherical image B", name);
				return false;
			}
		// a perspective pair is untouched by the option
		if (!SameDraw(Run(perA, perB, identity, 0.f), Run(perA, perB, identity, band)) || Run(perA, perB, identity, band).a.empty()) {
			VERBOSE("SphericalBottomBandDenseTest FAILED: the band changed the %s of a perspective pair", name);
			return false;
		}
	}
	VERBOSE("SphericalBottomBandDenseTest PASSED");
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM
