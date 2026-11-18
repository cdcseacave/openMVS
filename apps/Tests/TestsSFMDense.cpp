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

// The identity-warp scene of the draw tests: two images of one 640x480 camera at 600 px and a warp of 160x160 cells that maps every
// cell of A onto the pixel at its own centre in B (both align_corners=false), so a drawn point of A must come back unmoved in B and
// every check can be read in A's frame alone
struct IdentityWarpScene {
	static constexpr int WIDTH = 640, HEIGHT = 480, CELLS = 160;
	Scene scene;
	Image32F2 warp;

	IdentityWarpScene()
		: scene(MakeRig(cv::Size(WIDTH, HEIGHT), 600, {Pose3D::Identity(), Pose3D::Identity()})), warp(CELLS, CELLS)
	{
		for (int y = 0; y < CELLS; ++y)
			for (int x = 0; x < CELLS; ++x)
				warp(y, x) = NormCoord(CellPixel(x, y), Size());
	}
	cv::Size Size() const { return cv::Size(WIDTH, HEIGHT); }
	const Image& A() const { return scene.images[0]; }
	const Image& B() const { return scene.images[1]; }
	// the cell <-> pixel map the draws place their winners with and the occupancy checks invert (the cell-centre convention)
	Point2f CellPixel(int cx, int cy) const { return CellToPixel(Point2f((float)cx, (float)cy), cv::Size(CELLS, CELLS), Size()); }
	// the warp cell a pixel of A falls in, rounded and kept on the grid
	Point2i Cell(const Point2f& pt) const {
		const Point2f cell(PixelToCell(pt, Size(), cv::Size(CELLS, CELLS)));
		return Point2i(MINF(MAXF(ROUND2INT(cell.x), 0), CELLS-1), MINF(MAXF(ROUND2INT(cell.y), 0), CELLS-1));
	}
	// the bucket of a numBuckets x numBuckets grid over the warp that a pixel of A falls in
	size_t Bucket(const Point2f& pt, int numBuckets) const {
		const Point2i cell(Cell(pt));
		return (size_t)(cell.y*numBuckets/CELLS)*numBuckets + cell.x*numBuckets/CELLS;
	}
};

// What fraction of a coarse 16x16 grid over an image a sample occupies: the spread of a draw, computed here and not reported by the
// sampler, since the claim is that a draw uniform over the confident region reads as the overlap it came from
static float SampleCoverage(const std::vector<Point2f>& sampled, const cv::Size& size)
{
	constexpr unsigned coverageGrid = 16;
	std::vector<bool> grid((size_t)coverageGrid*coverageGrid, false);
	for (const Point2f& pt : sampled) {
		const unsigned cx = MINF((unsigned)MAXF(0.f, (float)coverageGrid*pt.x/(float)size.width), coverageGrid-1);
		const unsigned cy = MINF((unsigned)MAXF(0.f, (float)coverageGrid*pt.y/(float)size.height), coverageGrid-1);
		grid[(size_t)cy*coverageGrid + cx] = true;
	}
	return (float)std::count(grid.begin(), grid.end(), true)/(float)grid.size();
}

// The draw's contract of one winner per bucket and nothing else, on the bucket grid of numBuckets per side: the coarse occupancy of
// SampleCoverage cannot tell 800 points spread over a region from 2000 points piled into its most confident corner, this can
static bool CheckOneWinnerPerBucket(const TestScope& scope, const IdentityWarpScene& fx, const std::vector<Point2f>& sampled,
	int numBuckets, const char* label)
{
	std::vector<int> bucketHits((size_t)numBuckets*numBuckets, 0);
	for (const Point2f& pt : sampled)
		++bucketHits[fx.Bucket(pt, numBuckets)];
	FOREACH(b, bucketHits)
		TEST_CHECK(bucketHits[b] <= 1, "%s: bucket %u holds %d samples, expected at most 1 -- a fill-up puts extra points into buckets that already have a winner",
			label, (unsigned)b, bucketHits[b]);
	return true;
}

// A backstop that does not depend on the bucket formula: on a grid of 32 per side the busiest bin holds at most 3 times the mean of
// the non-empty ones; a draw crammed into a fraction of the region blows past this, a one-per-bucket draw sits near 1
static bool CheckNotPiledUp(const TestScope& scope, const IdentityWarpScene& fx, const std::vector<Point2f>& sampled, const char* label)
{
	constexpr int fineGrid = 32;
	std::vector<int> fineHits((size_t)fineGrid*fineGrid, 0);
	for (const Point2f& pt : sampled) {
		const int fx32 = MINF((int)((float)fineGrid*pt.x/(float)fx.WIDTH), fineGrid-1);
		const int fy32 = MINF((int)((float)fineGrid*pt.y/(float)fx.HEIGHT), fineGrid-1);
		++fineHits[(size_t)fy32*fineGrid + fx32];
	}
	int maxHits = 0, sumHits = 0, numNonEmpty = 0;
	for (const int hits : fineHits) {
		if (hits == 0)
			continue;
		++numNonEmpty;
		sumHits += hits;
		maxHits = MAXF(maxHits, hits);
	}
	const float meanHits = (float)sumHits/(float)numNonEmpty;
	TEST_CHECK((float)maxHits <= 3.f*meanHits, "%s: the busiest %dx%d bin holds %d samples, %.2fx the %.2f mean over the %d non-empty bins, expected at most 3x",
		label, fineGrid, fineGrid, maxHits, (float)maxHits/meanHits, meanHits, numNonEmpty);
	return true;
}

// The complementary draw into buffers that already hold something comes back as the draw into empty ones: it carries no state between
// calls and reads no container whose iteration order could vary, which is what makes the appended keypoint indices reproducible
static bool CheckComplementaryRepeat(const TestScope& scope, const IdentityWarpScene& fx, const Image32F& overlap, int numBuckets,
	unsigned budget, const std::vector<Point2f>& occupied, const std::vector<Point2f>& drawnA, const std::vector<Point2f>& drawnB,
	const std::vector<float>& drawnC, const char* label)
{
	std::vector<Point2f> repeatA(11, Point2f(5.f, 6.f)), repeatB(2, Point2f(7.f, 8.f));
	std::vector<float> repeatC(4, 0.25f);
	SampleWarpComplementary(fx.A(), fx.B(), fx.warp, overlap, 0.3f, 0.f, numBuckets, budget, occupied, repeatA, repeatB, repeatC);
	TEST_CHECK(repeatA == drawnA && repeatB == drawnB && repeatC == drawnC, "%s: the draw is not a pure function of its inputs", label);
	return true;
}

// The spherical bottom band (DenseMatchConfig::sphericalBottomBand): both warp draws (the verdict's SampleWarpByCoverage and the fill's
// SampleWarpComplementary, hence both matchers) have no end in the bottom band of a spherical image, whichever image it is. With an
// identity warp the rows above the band keep exactly the samples they have without the band; with a warp that sends rows above A's
// band into B's band, the survivors still hold none in the band and every sample of the band-0 draw with both ends outside it. A
// perspective pair is unaffected by the option.
static bool CheckBottomBandDraws(const TestScope& scope)
{
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
			identity(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)y), grid, size), size);
			shifted(y, x) = NormCoord(CellToPixel(Point2f((float)x, (float)(y+8)), grid, size), size);
		}
	const Image32F confidence(grid, 1.f);
	// the buckets of the fill, 20 x 20: the band's edge (row 32 of 40) is a bucket edge, so a bucket never straddles it
	constexpr int numBuckets = 20;
	const std::vector<Point2f> noneOccupied;
	struct Draw { std::vector<Point2f> a, b; std::vector<float> c; };
	const auto SameDraw = [](const Draw& x, const Draw& y) { return x.a == y.a && x.b == y.b; };
	const auto CountInBand = [&](const Draw& d) {
		size_t n = 0;
		for (size_t i = 0; i < d.a.size(); ++i)
			n += InBand(d.a[i]) || InBand(d.b[i]);
		return n;
	};
	// every sample of `off` with both ends out of the band is in `on`, in the same order
	const auto Survivors = [&](const Draw& off, const Draw& on) {
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
	for (const auto& [name, isFill] : std::vector<std::pair<const char*, bool>>{{"fill", true}, {"verdict sample", false}}) {
		const bool bFill = isFill;
		// the same draw with the band off and on, through each of the two draws
		const auto Run = [&](const Image& a, const Image& b, const Image32F2& warp, float bandValue) {
			Draw d;
			if (bFill)
				SampleWarpComplementary(a, b, warp, confidence, 0.3f, bandValue, numBuckets, cells*cells, noneOccupied, d.a, d.b, d.c);
			else
				SampleWarpByCoverage(a, b, warp, confidence, 0.3f, bandValue, cells*cells, d.a, d.b);
			return d;
		};
		for (const Image32F2* warp : {&identity, &shifted}) {
			const bool bIdentity = warp == &identity;
			const Draw off = Run(sphA, sphB, *warp, 0.f), on = Run(sphA, sphB, *warp, band);
			TEST_CHECK(CountInBand(off) >= 40, "the band-0 %s has only %u samples in the band, the setup does not exercise it", name, (unsigned)CountInBand(off));
			TEST_CHECK(CountInBand(on) == 0 && !on.a.empty(), "the %s keeps %u samples with an end in the band (%u samples)", name, (unsigned)CountInBand(on), (unsigned)on.a.size());
			TEST_CHECK(Survivors(off, on), "the band removed a %s sample of the rows above the band", name);
			if (bIdentity && bFill) {
				// the identity warp puts both ends in the same row: the fill's samples above the band are exactly the band-0 ones
				Draw offAbove;
				for (size_t i = 0; i < off.a.size(); ++i)
					if (!InBand(off.a[i])) {
						offAbove.a.push_back(off.a[i]);
						offAbove.b.push_back(off.b[i]);
					}
				TEST_CHECK(SameDraw(offAbove, on), "the rows above the band hold %u fill samples, %u without the band", (unsigned)on.a.size(), (unsigned)offAbove.a.size());
			}
		}
		// a spherical image in either role: the band holds for the end in the spherical image, the perspective end is free
		const Draw sphPerOff = Run(sphA, perB, identity, 0.f), sphPerOn = Run(sphA, perB, identity, band);
		const Draw perSphOff = Run(perA, sphB, identity, 0.f), perSphOn = Run(perA, sphB, identity, band);
		TEST_CHECK(!sphPerOn.a.empty() && !perSphOn.a.empty() && sphPerOn.a.size() < sphPerOff.a.size() && perSphOn.a.size() < perSphOff.a.size(),
			"the %s of a spherical and a perspective image holds %u/%u samples with the band, %u/%u without", name,
			(unsigned)sphPerOn.a.size(), (unsigned)perSphOn.a.size(), (unsigned)sphPerOff.a.size(), (unsigned)perSphOff.a.size());
		for (const Point2f& pt : sphPerOn.a)
			TEST_CHECK(!InBand(pt), "the %s keeps a sample in the band of the spherical image A", name);
		for (const Point2f& pt : perSphOn.b)
			TEST_CHECK(!InBand(pt), "the %s keeps a sample in the band of the spherical image B", name);
		// a perspective pair is untouched by the option
		const Draw perOn = Run(perA, perB, identity, band);
		TEST_CHECK(SameDraw(Run(perA, perB, identity, 0.f), perOn) && !perOn.a.empty(), "the band changed the %s of a perspective pair", name);
	}
	return true;
}

// The warp draws on an identity warp: SampleWarpByCoverage (the verdict's fitting sample), SampleWarpComplementary (the fill of an
// admitted pair), their shared thinning and bucket pitch, and the spherical bottom band both honour.
// 1. the coverage draw: its budget, the spread the bucket stratification buys, one winner per bucket, no ranking by confidence (a
//    confidence hot-spot inside an eligible region moves the draw by not one point), the small budgets, a corner-only warp, no warp;
// 2. the complementary draw: it covers what the pair's sparse matches left empty (one winner in every unoccupied bucket that holds a
//    candidate, none in an occupied one), caps itself at the dense budget and stays spread when the cap bites (thinning by the
//    lattice key, not by confidence), opposite confidence ramps draw the same points, two pairs sharing image A with different
//    grids draw the same pixels in their common region, and the thinning is an order-preserving subsequence whose tighter budget
//    keeps a subset of the looser one;
// 3. the pitch is a function of the density alone, the draw scales with the overlap area and the sparse matches take their share
//    out of it, and the ceiling is the density over the smaller of the verdict's two inlier areas;
// 4. the bottom band.
// Every draw twice gives the same samples.
bool DenseWarpSamplingTest()
{
	TestScope scope("DenseWarpSamplingTest");
	IdentityWarpScene fx;
	constexpr int cells = IdentityWarpScene::CELLS;
	const Image& imgA = fx.A();
	const Image& imgB = fx.B();
	const Image32F2& warp = fx.warp;
	const cv::Size size(fx.Size());
	std::vector<Point2f> sampledA, sampledB;

	// --- 1. the coverage draw ---------------------------------------------------------------------------------------------
	const unsigned budget = 2000;
	// full overlap: every cell eligible, so the bucket grid is ceil(sqrt(budget)) = 45 on a side and the sample lands at the budget,
	// spread over the whole frame, every point unmoved through the identity warp
	Image32F overlap(cv::Size(cells, cells), 0.6f);
	const size_t numFull = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB);
	TEST_CHECK(numFull >= budget*9/10 && numFull <= budget*11/10, "full overlap drew %u samples, expected about the %u budgeted", (unsigned)numFull, budget);
	float coverageA = SampleCoverage(sampledA, size), coverageB = SampleCoverage(sampledB, size);
	TEST_CHECK(coverageA >= 0.99f && coverageB >= 0.99f, "a fully overlapping sample covers only %.3f/%.3f of the two images", coverageA, coverageB);
	FOREACH(i, sampledA)
		TEST_CHECK(Image8U::isInside(sampledB[i], imgB.GetSize()) && norm(sampledB[i] - sampledA[i]) <= 1e-3f,
			"sample %u moved %g px through the identity warp", (unsigned)i, norm(sampledB[i] - sampledA[i]));

	// PARTIAL OVERLAP, the case the over-binning exists for: a 100x100-cell region is eligible, E/T = 0.39, so the bucket grid has to
	// grow to ceil(sqrt(2000/0.39)) = 72 a side and the occupied buckets must still number about the budget. The old fixed-45 grid
	// plus a descending-confidence fill-up hit the budget exactly while cramming ~1200 of those points into the eligible region
	overlap.setTo(0.f);
	overlap(cv::Rect(2, 2, 100, 100)).setTo(0.6f);
	const size_t numPartial = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB);
	TEST_CHECK(numPartial >= budget*3/4 && numPartial <= budget*5/4, "a 39%%-overlap warp drew %u samples, expected near the %u budgeted", (unsigned)numPartial, budget);
	// and the sample occupies the overlap it came from, not the whole frame: the region spans about 100/160 of each side, so about 0.4
	coverageA = SampleCoverage(sampledA, size);
	coverageB = SampleCoverage(sampledB, size);
	TEST_CHECK(coverageA >= 0.3f && coverageA <= 0.5f && ABS(coverageB - coverageA) <= 0.02f,
		"a 39%%-overlap sample reports %.3f/%.3f coverage, expected about 0.4", coverageA, coverageB);
	// one winner per bucket, on n from the documented formula (E = 100*100 eligible cells of T = cells*cells, capped at the warp side)
	const int numBuckets = MINF((int)std::ceil(std::sqrt((double)budget*cells*cells/(100*100))), cells);
	if (!CheckOneWinnerPerBucket(scope, fx, sampledA, numBuckets, "partial-overlap") || !CheckNotPiledUp(scope, fx, sampledA, "partial-overlap"))
		return false;
	// the draw is not piled onto the most confident part of that region: under the shared lattice winner rule (DenseWarp.cpp,
	// WarpCellLatticePriority) confidence only admits the candidates, so raising a 20x20-cell corner of an ALREADY-ELIGIBLE region to
	// full confidence must not move the draw by a single point. Pinned as exact equality: a tolerance would pass whatever the rule does
	const std::vector<Point2f> partialA(sampledA), partialB(sampledB);
	overlap(cv::Rect(2, 2, 20, 20)).setTo(1.f);
	std::vector<Point2f> skewA, skewB;
	const size_t numSkew = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, skewA, skewB);
	TEST_CHECK(numSkew == numPartial && skewA == partialA && skewB == partialB &&
		SampleCoverage(skewA, size) == coverageA && SampleCoverage(skewB, size) == coverageB,
		"a confidence hot-spot moved the draw from %u samples at %.3f coverage to %u at %.3f -- confidence is ranking the bucket winners, not just admitting them",
		(unsigned)numPartial, coverageA, (unsigned)numSkew, SampleCoverage(skewA, size));
	// the hot-spot's E and T are unchanged, so the same bucket grid applies; this is where the old fill-up's bias was strongest
	if (!CheckOneWinnerPerBucket(scope, fx, skewA, numBuckets, "confidence-hot-spot") || !CheckNotPiledUp(scope, fx, skewA, "confidence-hot-spot"))
		return false;
	// the same draw twice in one process, the second time into pre-filled buffers: the function is pure (not a claim of reproducibility
	// across binaries or machines; the sample is also a function of the warp the model produced)
	std::vector<Point2f> repeatA(7, Point2f(1.f, 2.f)), repeatB(3, Point2f(3.f, 4.f));
	SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, repeatA, repeatB);
	TEST_CHECK(repeatA == skewA && repeatB == skewB && SampleCoverage(repeatA, size) == SampleCoverage(skewA, size) &&
		SampleCoverage(repeatB, size) == SampleCoverage(skewB, size), "the coverage draw is not a pure function of its inputs");
	// budget boundary: the smallest sample the verdict fits on is 8, the estimator's own minimum; maxSamples is a target rather than a
	// cap and the bucket grid is a few cells on a side, so quantisation dominates: the draw stays small, non-empty and terminates
	const size_t numTiny = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, 8, sampledA, sampledB);
	TEST_CHECK(numTiny != 0 && numTiny <= 64 && sampledA.size() == numTiny, "a budget of 8 drew %u samples", (unsigned)numTiny);
	// and a budget of 1 degenerates to a 2x2 bucket grid over the whole warp, not to a division by zero or an empty draw
	const size_t numOne = SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, 1, sampledA, sampledB);
	TEST_CHECK(numOne != 0 && numOne <= 8, "a budget of 1 drew %u samples", (unsigned)numOne);
	// a warp confident in one corner only (kept off the very border, where the round trip through the normalized coordinates can put a
	// cell a hundredth of a pixel outside the second image): E <= budget, so the whole confident overlap is taken unstratified, the
	// sample is SMALLER than the budget, and its occupancy shows that it sits in that corner
	overlap.setTo(0.f);
	overlap(cv::Rect(2, 2, 40, 40)).setTo(1.f);
	TEST_CHECK(SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB) == 40*40,
		"%u samples drawn of the 1600 confident cells", (unsigned)sampledA.size());
	coverageA = SampleCoverage(sampledA, size);
	coverageB = SampleCoverage(sampledB, size);
	TEST_CHECK(coverageA <= 0.15f && coverageA >= 0.05f && ABS(coverageB - coverageA) <= 0.01f,
		"a corner-only sample covers %.3f/%.3f of the two images", coverageA, coverageB);
	// a warp confident nowhere: no sample, no occupancy, and no estimator ever sees the pair
	overlap.setTo(0.f);
	TEST_CHECK(SampleWarpByCoverage(imgA, imgB, warp, overlap, 0.3f, 0.f, budget, sampledA, sampledB) == 0 && sampledA.empty() &&
		SampleCoverage(sampledA, size) == 0.f && SampleCoverage(sampledB, size) == 0.f, "an unconfident warp still produced a sample");

	// --- 2. the complementary draw ----------------------------------------------------------------------------------------
	// COMPLEMENTARITY on a fully confident warp. The pair's guided sparse matches cluster in the top-left corner -- the one textured
	// region descriptors could agree on -- so the fill must draw over the rest of the frame and nowhere in there.
	std::vector<Point2f> sparseA;
	for (int cy = 0; cy < 40; cy += 2)
		for (int cx = 0; cx < 40; cx += 2)
			sparseA.push_back(fx.CellPixel(cx, cy));
	const DenseMatchConfig config; // denseMatchesPerFrame 2000 (a density per full frame of overlap), minConfidence 0.1
	const unsigned denseBudget = config.denseMatchesPerFrame;
	const int numDenseBuckets = DenseFillGridSide(denseBudget, cells);
	overlap = Image32F(cv::Size(cells, cells), 0.6f);
	std::vector<Point2f> denseA, denseB;
	std::vector<float> confidences;
	const size_t numDense = SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numDenseBuckets, denseBudget, sparseA, denseA, denseB, confidences);
	TEST_CHECK(numDense != 0 && denseB.size() == numDense && confidences.size() == numDense,
		"the draw returned %u points with %u/%u index-parallel arrays", (unsigned)numDense, (unsigned)denseB.size(), (unsigned)confidences.size());
	std::vector<bool> occupied((size_t)numDenseBuckets*numDenseBuckets, false);
	for (const Point2f& pt : sparseA)
		occupied[fx.Bucket(pt, numDenseBuckets)] = true;
	FOREACH(i, denseA)
		TEST_CHECK(!occupied[fx.Bucket(denseA[i], numDenseBuckets)], "dense point %u at (%.1f, %.1f) landed in a bucket the pair's sparse matches already hold",
			(unsigned)i, denseA[i].x, denseA[i].y);
	// and the converse: one winner in every OTHER bucket that has a candidate at all (a cell the confidence map admits whose warped point
	// lands inside B; a border cell can round a hundredth of a pixel outside B), so the draw took the whole complement and an occupied
	// bucket contributes nothing, not a reduced quota
	std::vector<bool> hasCandidate((size_t)numDenseBuckets*numDenseBuckets, false);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x)
			if (overlap(y, x) >= 0.3f && Image8U::isInside(DenormCoord(warp(y, x), imgB.GetSize()), imgB.GetSize()))
				hasCandidate[(size_t)(y*numDenseBuckets/cells)*numDenseBuckets + x*numDenseBuckets/cells] = true;
	size_t numExpected = 0;
	FOREACH(b, hasCandidate)
		if (hasCandidate[b] && !occupied[b])
			++numExpected;
	numExpected = MINF(numExpected, (size_t)denseBudget); // the thinning's cap, if the winners ran over it
	TEST_CHECK(numDense == numExpected, "%u dense points for the %u unoccupied buckets of the %dx%d grid that hold a candidate",
		(unsigned)numDense, (unsigned)numExpected, numDenseBuckets, numDenseBuckets);
	// the budget is a real CAP here, not merely the target it is for the verdict's draw
	TEST_CHECK(numDense <= denseBudget, "the draw kept %u points of a %u budget", (unsigned)numDense, denseBudget);
	FOREACH(i, denseA)
		// index-parallel through the identity warp, each point carrying its own cell's confidence (what MakeDenseKeypoint stamps)
		TEST_CHECK(norm(denseB[i] - denseA[i]) <= 1e-3f && ABS(confidences[i] - 0.6f) <= 1e-6f,
			"dense point %u moved %g px and carries confidence %g, expected 0.6", (unsigned)i, norm(denseB[i] - denseA[i]), confidences[i]);
	// warp-grid raster order, which makes the keypoint indices AppendDenseMatches hands out reproducible from one run to the next
	for (size_t i = 1; i < numDense; ++i)
		TEST_CHECK(denseA[i].y >= denseA[i-1].y - 1e-3f && (ABS(denseA[i].y - denseA[i-1].y) > 1e-3f || denseA[i].x > denseA[i-1].x),
			"point %u at (%.1f, %.1f) breaks the raster order after (%.1f, %.1f)", (unsigned)i, denseA[i].x, denseA[i].y, denseA[i-1].x, denseA[i-1].y);
	// a draw asked for nothing produces nothing, output buffers included: a pair whose whole budget went to its guided matches gets no
	// dense segment rather than a stale one
	std::vector<Point2f> zeroA(3, Point2f(1.f, 2.f)), zeroB(5, Point2f(3.f, 4.f));
	std::vector<float> zeroC(7, 0.5f);
	TEST_CHECK(SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numDenseBuckets, 0, sparseA, zeroA, zeroB, zeroC) == 0 &&
		zeroA.empty() && zeroB.empty() && zeroC.empty(), "a zero dense budget still drew %u points", (unsigned)zeroA.size());

	// OVER BUDGET, WHERE THE THINNING SHOWS. A pitch far finer than the budget needs (cells/2, against a budget of 400) drives the winner
	// count far past the budget on its own, and the confidence grows steadily toward the bottom-right corner. Thinning by confidence
	// keeps only that corner; thinning by the lattice key leaves every quadrant hit, since the levels it keeps whole are lattices over
	// the whole frame and the scramble that breaks the level it stops inside is spatially unbiased
	overlap.setTo(0.f);
	for (int y = 0; y < cells; y += 2)
		for (int x = 0; x < cells; x += 2)
			overlap(y, x) = 0.4f + 0.5f*(float)(x + y)/(float)(2*cells);
	const unsigned smallBudget = 400;
	const int numSmallBuckets = cells/2;
	const size_t numSmall = SampleWarpComplementary(imgA, imgB, warp, overlap, 0.3f, 0.f, numSmallBuckets, smallBudget, sparseA, denseA, denseB, confidences);
	TEST_CHECK(numSmall == smallBudget && denseB.size() == numSmall && confidences.size() == numSmall,
		"an over-budget draw kept %u points of a %u budget", (unsigned)numSmall, smallBudget);
	std::vector<bool> smallOccupied((size_t)numSmallBuckets*numSmallBuckets, false);
	for (const Point2f& pt : sparseA)
		smallOccupied[fx.Bucket(pt, numSmallBuckets)] = true;
	unsigned perQuadrant[4] = {0u, 0u, 0u, 0u};
	FOREACH(i, denseA) {
		// complementarity survives the thinning too: it can only drop points, never move one into a bucket the draw had struck out
		TEST_CHECK(!smallOccupied[fx.Bucket(denseA[i], numSmallBuckets)], "the thinned draw put point %u at (%.1f, %.1f) into a sparse-occupied bucket",
			(unsigned)i, denseA[i].x, denseA[i].y);
		++perQuadrant[((int)denseA[i].y < fx.HEIGHT/2 ? 0 : 2) + ((int)denseA[i].x < fx.WIDTH/2 ? 0 : 1)];
	}
	// the sparse cluster only bites into the first quadrant, so all four must keep a real share; a draw thinned by confidence collapses
	// into the last one and empties the first outright
	for (int q = 0; q < 4; ++q)
		TEST_CHECK(perQuadrant[q] >= smallBudget/10, "quadrant %d holds %u of the %u kept points (%u/%u/%u/%u), expected at least a tenth in each -- the thinning collapsed the draw onto one region",
			q, perQuadrant[q], smallBudget, perQuadrant[0], perQuadrant[1], perQuadrant[2], perQuadrant[3]);
	if (!CheckComplementaryRepeat(scope, fx, overlap, numSmallBuckets, smallBudget, sparseA, denseA, denseB, confidences, "the thinned draw"))
		return false;

	// CONFIDENCE IS THE ELIGIBILITY TEST, NOT THE RANKING. Two warps confident over the same region with OPPOSITE confidence ramps
	// produce the very same draw: the winner of a bucket is its highest-lattice-priority eligible cell, which depends on the cell
	// coordinates alone. This is what makes two pairs sharing image A able to agree on a point at all.
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
	TEST_CHECK(!upA.empty() && upA == downA && upB == downB, "opposite confidence ramps over one region drew %u and %u points at different positions -- confidence is still ranking the bucket winners",
		(unsigned)upA.size(), (unsigned)downA.size());
	// ...while each point still carries ITS OWN cell's confidence, which is what MakeDenseKeypoint stamps
	TEST_CHECK(upC != downC, "the two ramps stamped identical confidences on the same cells");

	// CROSS-PAIR COINCIDENCE, the reason for the lattice rule. Two pairs sharing image A, with partially overlapping confident regions
	// and budgets that give them DIFFERENT bucket grids: in the region both cover, the A-side positions they draw land on the same pixels,
	// exactly, so that the keypoint dedup downstream (FilterRedundantKeypoints, 0.1 px) can chain them into one track. Confidence cannot
	// deliver that -- the two pairs have different warps -- which is why the winner rule is pair-independent.
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
	TEST_CHECK(numAB != 0 && numAC != 0 && numBucketsAB != numBucketsAC, "the two pairs drew %u/%u points on %dx%d and %dx%d grids -- the fixture must give them different grids for the coincidence to mean anything",
		(unsigned)numAB, (unsigned)numAC, numBucketsAB, numBucketsAB, numBucketsAC, numBucketsAC);
	// the cells both warps are confident about: [40, 120) on both axes, in pixels of A
	const Point2f commonLo(fx.CellPixel(40, 40)), commonHi(fx.CellPixel(119, 119));
	const auto InCommon = [&](const Point2f& pt) {
		return pt.x >= commonLo.x - 1e-3f && pt.x <= commonHi.x + 1e-3f && pt.y >= commonLo.y - 1e-3f && pt.y <= commonHi.y + 1e-3f;
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
			// EXACT equality, not a tolerance: the dedup downstream keys on position, and two draws that agree only to within a pixel
			// would leave two keypoints where one point was sampled
			if (commonAC.find(std::make_pair(pt.x, pt.y)) != commonAC.end())
				++numCoincident;
		}
	const unsigned numCommonMin = MINF(numCommonAB, numCommonAC);
	TEST_CHECK(numCommonMin != 0 && numCoincident*5 >= numCommonMin*2, "only %u of the %u/%u points the two pairs drew in their common region coincide exactly, expected at least two fifths of %u -- the draws are not chaining across pairs",
		numCoincident, numCommonAB, numCommonAC, numCommonMin);
	// still one winner per bucket on each pair's own grid, and still deterministic
	if (!CheckOneWinnerPerBucket(scope, fx, abA, numBucketsAB, "the first pair") || !CheckOneWinnerPerBucket(scope, fx, acA, numBucketsAC, "the second pair") ||
		!CheckComplementaryRepeat(scope, fx, overlapAC, numBucketsAC, 1500, noneOccupied, acA, acB, acC, "the lattice draw"))
		return false;

	// THE THINNING, as its own entry point: how the draw caps itself once the unoccupied buckets outnumber the budget. It hands the
	// survivors back in the order it got them (what AppendDenseMatches labels keypoints by) and keeps a PREFIX of one pair-independent
	// order over the cells, so a tighter budget keeps a SUBSET of what a looser one kept: the ceiling is per pair, so enforcing it by
	// anything that read this pair's own list would leave two pairs which agreed on every winner holding different subsets of them
	std::vector<int> acCellIdx;
	for (const Point2f& pt : acA) {
		const Point2i cell(fx.Cell(pt));
		acCellIdx.push_back(cell.y*cells + cell.x);
	}
	std::vector<Point2f> thinA(acA), thinB(acB);
	std::vector<float> thinC(acC);
	std::vector<int> thinCellIdx(acCellIdx);
	const unsigned thinTo = (unsigned)(acA.size()/3);
	ThinSampleByLatticePriority(thinA, thinB, thinC, thinCellIdx, cells, thinTo);
	TEST_CHECK(thinA.size() == thinTo && thinB.size() == thinTo && thinC.size() == thinTo && thinCellIdx.size() == thinTo,
		"thinning %u points to %u kept %u", (unsigned)acA.size(), thinTo, (unsigned)thinA.size());
	size_t src = 0;
	FOREACH(i, thinA) {
		while (src < acA.size() && !(acA[src] == thinA[i]))
			++src;
		TEST_CHECK(src != acA.size() && acB[src] == thinB[i] && acC[src] == thinC[i] && acCellIdx[src] == thinCellIdx[i],
			"the thinned sample is not an order-preserving index-parallel subsequence of the draw at %u", (unsigned)i);
	}
	// half that budget again: what survives it must be what the looser budget already kept
	std::vector<Point2f> tightA(acA), tightB(acB);
	std::vector<float> tightC(acC);
	std::vector<int> tightCellIdx(acCellIdx);
	ThinSampleByLatticePriority(tightA, tightB, tightC, tightCellIdx, cells, thinTo/2);
	const std::set<int> keptCells(thinCellIdx.begin(), thinCellIdx.end());
	FOREACH(i, tightCellIdx)
		TEST_CHECK(keptCells.find(tightCellIdx[i]) != keptCells.end(), "thinning to %u kept cell %d, which thinning the same sample to %u dropped -- the two budgets are not keeping nested prefixes of one order",
			thinTo/2, tightCellIdx[i], thinTo);
	// asked for at least what it holds, a sample comes back untouched; asked for nothing, it empties
	std::vector<Point2f> keepA(acA), keepB(acB);
	std::vector<float> keepC(acC);
	std::vector<int> keepCellIdx(acCellIdx);
	ThinSampleByLatticePriority(keepA, keepB, keepC, keepCellIdx, cells, (unsigned)acA.size() + 10);
	ThinSampleByLatticePriority(thinA, thinB, thinC, thinCellIdx, cells, 0);
	TEST_CHECK(keepA == acA && keepB == acB && keepC == acC && thinA.empty() && thinB.empty() && thinC.empty() && thinCellIdx.empty(),
		"thinning to a budget above the sample size, or to zero, did not behave");

	// --- 3. the fill density and its ceiling ------------------------------------------------------------------------------
	// THE PITCH IS A FUNCTION OF THE DENSITY ALONE: one bucket per dense match at full overlap, clamped to the warp side, never below 1
	TEST_CHECK(DenseFillGridSide(2000, cells) == 45 && DenseFillGridSide(0, cells) == 1 && DenseFillGridSide(1000000, cells) == cells,
		"pitch %d/%d/%d for densities 2000/0/1000000", DenseFillGridSide(2000, cells), DenseFillGridSide(0, cells), DenseFillGridSide(1000000, cells));
	// THE DRAW SCALES WITH THE OVERLAP AREA, NOT WITH THE BUDGET: the same pitch over two confident regions, one four times the other,
	// draws about four times as much, which is what "a density" means and what the old per-draw grid (sized so that ANY overlap
	// yielded ~maxSamples) could not do
	const unsigned density = 2000;
	const int grid = DenseFillGridSide(density, cells);
	std::vector<Point2f> bigA, bigB, smallA, smallB;
	std::vector<float> bigC, smallC;
	Image32F confidence(cells, cells);
	const auto DrawOver = [&](int cellsWide, int cellsHigh, const std::vector<Point2f>& occupiedPts,
		std::vector<Point2f>& outA, std::vector<Point2f>& outB, std::vector<float>& outC) {
		confidence.setTo(0.f);
		for (int y = 0; y < cellsHigh; ++y)
			for (int x = 0; x < cellsWide; ++x)
				confidence(y, x) = 0.9f;
		return SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, 0.f, grid, density, occupiedPts, outA, outB, outC);
	};
	const size_t numBig = DrawOver(80, 80, noneOccupied, bigA, bigB, bigC);         // a quarter of the frame
	const size_t numSmallRegion = DrawOver(40, 40, noneOccupied, smallA, smallB, smallC); // a sixteenth
	TEST_CHECK(numSmallRegion != 0 && numBig >= 3*numSmallRegion && numBig <= 5*numSmallRegion, "a 4x larger overlap drew %u against %u points",
		(unsigned)numBig, (unsigned)numSmallRegion);
	// THE SPARSE MATCHES TAKE THEIR SHARE OUT OF THE DRAW: the same region uncovered, half covered by guided matches, fully covered; the
	// yield falls with the uncovered area and reaches zero when nothing is left uncovered
	std::vector<Point2f> halfOccupied, allOccupied;
	for (int y = 0; y < 80; ++y)
		for (int x = 0; x < 80; ++x) {
			if (x < 40)
				halfOccupied.push_back(fx.CellPixel(x, y));
			allOccupied.push_back(fx.CellPixel(x, y));
		}
	std::vector<Point2f> halfA, halfB, fullA, fullB;
	std::vector<float> halfC, fullC;
	const size_t numHalf = DrawOver(80, 80, halfOccupied, halfA, halfB, halfC);
	const size_t numFullCovered = DrawOver(80, 80, allOccupied, fullA, fullB, fullC);
	TEST_CHECK(numHalf != 0 && numHalf <= numBig*3/5 && numHalf >= numBig/3, "half the overlap covered drew %u of %u", (unsigned)numHalf, (unsigned)numBig);
	TEST_CHECK(numFullCovered == 0, "a fully covered overlap still drew %u points", (unsigned)numFullCovered);
	// THE CEILING is the configured density over the SMALLER of the verdict's two inlier areas: the bucket grid lives in A's frame and
	// cannot see how much of B the pair's dense matches would land on, so this is the only term that bounds their density in B
	DenseMatchConfig ceilingConfig;
	ceilingConfig.denseMatchesPerFrame = 2000;
	PairVerdict verdict;
	const auto Ceiling = [&](float areaA, float areaB) {
		verdict.inlierAreaA = areaA;
		verdict.inlierAreaB = areaB;
		return DenseFillCeiling(ceilingConfig, verdict);
	};
	const unsigned symmetric = Ceiling(0.50f, 0.50f), narrowB = Ceiling(0.50f, 0.10f), narrowA = Ceiling(0.10f, 0.50f), identical = Ceiling(1.f, 1.f);
	TEST_CHECK(symmetric == 1000 && narrowB == 200 && narrowA == 200 && identical == 2000, "ceilings %u/%u/%u/%u for .5|.5, .5|.1, .1|.5, 1|1",
		symmetric, narrowB, narrowA, identical);
	// the two sides are symmetric (min(), not "A's area") and the density is linear in the knob, which makes --dense-matches readable
	// as matches per frame of overlap
	ceilingConfig.denseMatchesPerFrame = 500;
	TEST_CHECK(Ceiling(0.50f, 0.50f) == 250, "ceiling %u at density 500 over half an overlap", Ceiling(0.50f, 0.50f));

	// --- 4. the spherical bottom band -------------------------------------------------------------------------------------
	if (!CheckBottomBandDraws(scope))
		return false;
	return scope.Pass("the coverage and complementary draws (budget, spread, one winner per bucket, no ranking by confidence, cross-pair coincidence, nested thinning, purity), the fill density and ceiling, and the spherical bottom band");
}
/*----------------------------------------------------------------*/

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
// Then the verdict's cells on the exact warp of the wedge, and the overlap gate of the tracking: zeroing the left half of the
// confidence drops the keypoints left of the (bilinear) band and keeps the rest.
// The convention is written out here rather than taken from DenseWarp.h, so the test states it
// independently of the implementation.
bool DenseWarpGridConventionTest()
{
	TestScope scope("DenseWarpGridConventionTest");
	const auto CellCentre = [](float i, int size, int cells) { return (i + 0.5f)*(float)size/(float)cells - 0.5f; };
	const auto NormalizedCoord = [](float p, int size) { return 2.f*(p + 0.5f)/(float)size - 1.f; };
	const int cells = 160;
	for (const cv::Size size : {cv::Size(1024, 768), cv::Size(2789, 2092)}) {
		Scene scene(MakeRig(size, REAL(size.width), {Pose3D::Identity(), Pose3D::Identity()}));
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
				const Point2f back(PixelToCell(pixel, size, gridSize));
				TEST_CHECK(ABS(pixel.x - CellCentre(cell.x, size.width, cells)) <= 1e-3f && ABS(pixel.y - CellCentre(cell.y, size.height, cells)) <= 1e-3f &&
					norm(back - cell) <= 1e-4f, "%dx%d: cell (%g,%g) -> pixel (%g,%g) -> cell (%g,%g)", size.width, size.height, cell.x, cell.y,
					pixel.x, pixel.y, back.x, back.y);
			}
			const Point2f centreBack(PixelToCell(CellToPixel(Point2f((float)i, (float)i), gridSize, size), size, gridSize));
			TEST_CHECK(ROUND2INT(centreBack.x) == i && ROUND2INT(centreBack.y) == i, "%dx%d: cell %d rounds back to (%d,%d)",
				size.width, size.height, i, ROUND2INT(centreBack.x), ROUND2INT(centreBack.y));
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
		TEST_CHECK(TrackKeypointsByWarp(imgA, imgB, warp, confidence, 0.1f, trackedA, trackedB, status) == imgA.keypoints.size(),
			"%dx%d: not every keypoint tracked through a fully confident warp", size.width, size.height);
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
		TEST_CHECK(maxError <= 0.05f, "%dx%d: keypoint (%g,%g) tracked %g px off its true target, expected at most 0.05",
			size.width, size.height, imgA.keypoints[worst].pt.x, imgA.keypoints[worst].pt.y, maxError);

		// 2) THE DRAW, cell -> pixel (CollectWarpCandidates): with a budget past the grid every cell is
		// taken, in raster order, and each one's A end must sit at the cell's centre, its B end at the
		// cell centre's target
		std::vector<Point2f> sampledA, sampledB;
		TEST_CHECK(SampleWarpByCoverage(imgA, imgB, warp, confidence, 0.1f, 0.f, (unsigned)(cells*cells), sampledA, sampledB) == (size_t)(cells*cells),
			"%dx%d: the draw did not take every cell of a fully confident warp", size.width, size.height);
		FOREACH(k, sampledA) {
			const Point2f centre(CellCentreA((int)k%cells, (int)k/cells));
			TEST_CHECK(norm(sampledA[k] - centre) <= 2e-3f && norm(sampledB[k] - MapAB(centre)) <= 0.01f,
				"%dx%d: cell (%d,%d) drawn at (%g,%g) -> (%g,%g), its centre is (%g,%g) -> (%g,%g)",
				size.width, size.height, (int)k%cells, (int)k/cells, sampledA[k].x, sampledA[k].y, sampledB[k].x, sampledB[k].y,
				centre.x, centre.y, MapAB(centre).x, MapAB(centre).y);
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
		TEST_CHECK(sampledA.size() == (size_t)cells*cells - numStruck && !bStruckDrawn,
			"%dx%d: the complementary draw took %u cells around %u occupied ones (%s an occupied cell), expected %u",
			size.width, size.height, (unsigned)sampledA.size(), (unsigned)numStruck, bStruckDrawn ? "including" : "excluding",
			(unsigned)((size_t)cells*cells - numStruck));
	}

	// 4) THE VERDICT'S CELLS (CollectEligibleCells, cell -> pixel): the exact bidirectional warp of the
	// wedge, generated in the same convention, admitted, and every inlier cell of A at a cell centre
	{
		Scene scene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
		const Image& imgA = scene.images[0];
		const Image& imgB = scene.images[1];
		MatchConfig matchCfg;
		PairsMatcher matcher(scene, matchCfg);
		const int wedgeCells = 64;
		PairWarps warps;
		MakeDenseWedgeWarp(imgA, imgB, wedgeCells, 14, 50, 13, 47, warps.ab);
		MakeDenseWedgeWarp(imgB, imgA, wedgeCells, 14, 50, 13, 47, warps.ba);
		const DenseMatchConfig config;
		ImagePair pair(0, 1);
		PairVerdict verdict;
		JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
		TEST_CHECK(verdict.admitted && !verdict.inliersA.empty(), "the exact wedge warp was rejected (inlier areas %.4f/%.4f)",
			verdict.inlierAreaA, verdict.inlierAreaB);
		for (const Point2f& pt : verdict.inliersA) {
			const float x = (pt.x + 0.5f)*(float)wedgeCells/(float)imgA.GetWidth() - 0.5f;
			const float y = (pt.y + 0.5f)*(float)wedgeCells/(float)imgA.GetHeight() - 0.5f;
			TEST_CHECK(ABS(x - (float)ROUND2INT(x)) <= 1e-3f && ABS(y - (float)ROUND2INT(y)) <= 1e-3f,
				"a verdict inlier of A at (%g,%g) is cell (%g,%g), not a cell centre", pt.x, pt.y, x, y);
		}
	}

	// 5) THE OVERLAP GATE of the tracking: zeroing the left half of the confidence grid drops the keypoints left of the (bilinear)
	// band, x < 316, and keeps those right of it, x > 324
	{
		IdentityWarpScene fx;
		Image& imgA = fx.scene.images[0];
		std::mt19937 rng(7);
		std::uniform_real_distribution<float> ux(8.f, 631.f), uy(8.f, 471.f);
		for (unsigned i = 0; i < 500; ++i)
			imgA.keypoints.emplace_back(ux(rng), uy(rng), 1.f);
		Image32F overlap(cv::Size(fx.CELLS, fx.CELLS), 1.f);
		overlap.colRange(0, fx.CELLS/2).setTo(0.f);
		std::vector<Point2f> trackedA, trackedB;
		std::vector<uchar> status;
		TrackKeypointsByWarp(imgA, fx.B(), fx.warp, overlap, 0.3f, trackedA, trackedB, status);
		FOREACH(i, status) {
			const float x = imgA.keypoints[i].pt.x;
			TEST_CHECK(!(x < 316.f && status[i]) && !(x > 324.f && !status[i]), "overlap gate at x=%g (%s)", x, status[i] ? "kept" : "dropped");
		}
	}
	return scope.Pass("cell <-> pixel, tracking within 0.05 px, the draws and the occupancy on two frame sizes, the verdict's cell centres, the overlap gate");
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
	TestScope scope("DenseVerdictTest");
	Scene scene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
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
	TEST_CHECK(verdict.admitted && ABS(verdict.inlierAreaA - regionShare) <= 0.05f && ABS(verdict.inlierAreaB - regionShare) <= 0.05f,
		"a warp confident over %.4f of both frames was %s with inlier areas %.4f/%.4f",
		regionShare, verdict.admitted ? "admitted" : "rejected", verdict.inlierAreaA, verdict.inlierAreaB);
	// every confident cell of either direction lands in the other frame and is explained by the one
	// fitted geometry, so both areas are the region itself and not some fraction of it
	TEST_CHECK(verdict.confidentAreaA == regionShare && verdict.confidentAreaB == regionShare &&
		verdict.inlierAreaA == verdict.confidentAreaA && verdict.inlierAreaB == verdict.confidentAreaB,
		"confident areas %.4f/%.4f and inlier areas %.4f/%.4f over a region of %.4f",
		verdict.confidentAreaA, verdict.confidentAreaB, verdict.inlierAreaA, verdict.inlierAreaB, regionShare);
	// the population the dense fill draws from: A's inlier cells, index-parallel with their confidences
	TEST_CHECK(verdict.inliersA.size() == (size_t)((rx1-rx0)*(ry1-ry0)) && verdict.inliersB.size() == verdict.inliersA.size() &&
		verdict.confidences.size() == verdict.inliersA.size(), "the verdict kept %u/%u cells with %u confidences, expected %u",
		(unsigned)verdict.inliersA.size(), (unsigned)verdict.inliersB.size(), (unsigned)verdict.confidences.size(), (unsigned)((rx1-rx0)*(ry1-ry0)));
	// an admitted pair carries the fit's geometry and no matches, and that geometry is the one the
	// two cameras really have: the fit is handed warp cells and nothing else, and it comes back
	// within a twentieth of a degree of the truth, which is what says the warp carried the geometry.
	// It is not evidence that the scene's own solution stayed out of the fit -- a leak would read
	// exactly the same; the temporary poseless Image copies JudgePairDense builds are what keeps it
	// out
	TEST_CHECK(pair.relativePose.has_value() && pair.F.has_value() && pair.matches.empty(), "an admitted pair carries %u matches, pose %d, F %d",
		(unsigned)pair.matches.size(), (int)pair.relativePose.has_value(), (int)pair.F.has_value());
	const REAL angleErr = RotationErrorDeg(pair.relativePose->R, poseGT.R);
	const REAL tSim = ABS(normalized(pair.relativePose->GetT()).dot(normalized(poseGT.GetT())));
	TEST_CHECK(angleErr <= REAL(0.05) && tSim >= REAL(0.9999), "the fitted geometry is %.4f deg and %.6f off the two cameras", angleErr, tSim);

	// (b) the same 30% of A, but B's confident cells map into A over only 3%: the min side rejects
	// the pair although A's own side would have admitted it
	const int bx0 = 20, bx1 = 32, by0 = 20, by1 = 30;
	const float smallShare = (float)((bx1-bx0)*(by1-by0))/(float)(cells*cells);
	MakeDenseWedgeWarp(imgB, imgA, cells, bx0, bx1, by0, by1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	TEST_CHECK(!verdict.admitted && ABS(verdict.inlierAreaA - regionShare) <= 0.05f && ABS(verdict.inlierAreaB - smallShare) <= 0.01f &&
		verdict.inliersA.empty() && !pair.F.has_value(),
		"a B side of %.4f (expected %.4f) next to an A side of %.4f was %s, keeping %u cells and %d geometry",
		verdict.inlierAreaB, smallShare, verdict.inlierAreaA, verdict.admitted ? "admitted" : "rejected", (unsigned)verdict.inliersA.size(), (int)pair.F.has_value());

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
	TEST_CHECK(!verdict.admitted && verdict.confidentAreaA == regionShare && ABS(verdict.inlierAreaA - regionShare) <= 0.05f &&
		verdict.inlierAreaB > 0.f && verdict.inlierAreaB < config.minOverlap && verdict.inlierAreaB < 0.5f*verdict.inlierAreaA,
		"a homography warp confident over %.4f of A (inlier areas %.4f/%.4f, bar %.4f) was %s",
		verdict.confidentAreaA, verdict.inlierAreaA, verdict.inlierAreaB, config.minOverlap, verdict.admitted ? "admitted" : "rejected");

	// (d) the same homography, re-judged through the UNCALIBRATED branch. This is the case the
	// min-side rule was designed for: seven degrees of freedom, and a plane is explained exactly by
	// a whole family of fundamental matrices, so no fit on A's cells alone -- however many inliers
	// it counts -- can tell a hallucinated warp from a true pair. The verdict has to reject it here
	// for the same reason it rejects it in (c), on B's side and nothing else.
	{
		MatchConfig fundamentalCfg;
		fundamentalCfg.forceFundamental = true; // 7-DoF F over the fixture's trusted intrinsics
		const PairsMatcher fundamentalMatcher(scene, fundamentalCfg);
		TEST_CHECK(PairsMatcher::SelectGeometryBranch(fundamentalCfg, imgA, imgB) == PairsMatcher::GeometryBranch::FUNDAMENTAL,
			"forceFundamental did not select the fundamental branch");
		// same outcome as (c), and for the same reason: A's own side fully explained (0.2988 of the
		// grid, all of it), B's side measured and small (0.0173 here), the pair rejected by the min
		// of the two
		JudgePairDense(fundamentalMatcher, imgA, imgB, warps, config, pair, verdict);
		TEST_CHECK(!verdict.admitted && verdict.confidentAreaA == regionShare && ABS(verdict.inlierAreaA - regionShare) <= 0.05f &&
			verdict.inlierAreaB > 0.f && verdict.inlierAreaB < config.minOverlap,
			"through the fundamental branch a homography warp confident over %.4f of A (inlier areas %.4f/%.4f, bar %.4f) was %s",
			verdict.confidentAreaA, verdict.inlierAreaA, verdict.inlierAreaB, config.minOverlap, verdict.admitted ? "admitted" : "rejected");
	}

	// (e) minOverlap 0 admits both (a) and (b): the threshold is the only thing that rejected (b)
	config.minOverlap = 0.f;
	MakeDenseWedgeWarp(imgA, imgB, cells, rx0, rx1, ry0, ry1, warps.ab);
	MakeDenseWedgeWarp(imgB, imgA, cells, rx0, rx1, ry0, ry1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	const bool bAdmittedFull = verdict.admitted && ABS(verdict.inlierAreaB - regionShare) <= 0.05f;
	MakeDenseWedgeWarp(imgB, imgA, cells, bx0, bx1, by0, by1, warps.ba);
	JudgePairDense(matcher, imgA, imgB, warps, config, pair, verdict);
	TEST_CHECK(bAdmittedFull && verdict.admitted && ABS(verdict.inlierAreaB - smallShare) <= 0.01f,
		"minOverlap 0 admitted the 30%% B side %d and the %.4f one %d", (int)bAdmittedFull, verdict.inlierAreaB, (int)verdict.admitted);
	return scope.Pass("the exact wedge warp admitted with both areas at the region, the min side alone rejecting a 3%% B side, a homography warp rejected through both branches, minOverlap 0 admitting");
}
/*----------------------------------------------------------------*/

// Guided sparse matching (MatchFeaturesGuided) on synthetic binary descriptors, two perspective images of one rig (MakeRig)
// whose geometry is given by hand: every query of A has a prediction in B at its own position plus (20, 10), the warp's, and
// a "close descriptor" is an exact copy of the query's random 32-byte row, so the outcome is decided by positions alone.
// 1. the disc (no geometry): the ratio is taken against the best descriptor OUTSIDE the search disc, so a lookalike anywhere
//    else in B rejects the match, a keypoint whose only close descriptor is inside the disc is accepted, and a scale duplicate
//    on top of the true match inside the disc (the classic ratio-test failure) no longer blocks it; the same lookalike moved
//    inside the disc is accepted;
// 2. the epipolar band: the region is a band along the query's epipolar line centred on the prediction, so a rival off the
//    line no longer takes the disc from the true match, an ambiguous rival ON the line refuses it, a same-feature duplicate
//    is no rival, every winner answers to the reference outside its band, and the search falls back to the disc without a
//    geometry and within two band lengths of the epipole;
// both give the same matches in the same order on a second run.
bool DenseGuidedMatchTest()
{
	TestScope scope("DenseGuidedMatchTest");
	const cv::Size size(640, 480);
	// two warp cells long and half a cell wide on a 160-cell grid over B, as the one pass sizes it
	GuidedSearch search;
	search.length = 2.f*(float)MAXF(size.width, size.height)/160.f;
	search.halfWidth = 0.5f*(float)MAXF(size.width, size.height)/160.f;
	search.sameFeatureDistance = 3.f;
	MatchConfig matchCfg;
	matchCfg.descriptorsAreBinary = true;
	matchCfg.crossCheck = false;      // the guided path refuses a cross-checking matcher (it needs k > 1)
	matchCfg.useFlannMatcher = false; // exact k-NN: the LSH index is approximate, and this test compares exact sets
	// the rig's two images with the given keypoints, every descriptor random from seed and the copies {query of A, keypoint of B}
	// made exact; returns the predictions of A's queries in B
	struct Copy { int query, keypoint; };
	const auto Rig = [&](Scene& scene, const std::vector<Point2f>& positionsA, const std::vector<Point2f>& positionsB,
		const std::vector<Copy>& copies, uint32_t seed) {
		scene = MakeRig(size, 400, {Pose3D::Identity(), Pose3D::Identity()});
		Image& imgA = scene.images[0];
		Image& imgB = scene.images[1];
		for (const Point2f& pt : positionsA)
			imgA.keypoints.emplace_back(pt.x, pt.y, 10.f);
		for (const Point2f& pt : positionsB)
			imgB.keypoints.emplace_back(pt.x, pt.y, 10.f);
		std::mt19937 rng(seed);
		for (Image* img : {&imgA, &imgB}) {
			img->descriptors.create((int)img->keypoints.size(), 32, CV_8U);
			for (int r = 0; r < img->descriptors.rows; ++r)
				for (int b = 0; b < 32; ++b)
					img->descriptors.at<uint8_t>(r, b) = (uint8_t)(rng() & 0xFF);
		}
		for (const Copy& copy : copies)
			imgA.descriptors.row(copy.query).copyTo(imgB.descriptors.row(copy.keypoint));
		std::vector<Point2f> trackedB;
		for (const auto& kp : imgA.keypoints)
			trackedB.push_back(Point2f(kp.pt) + Point2f(20.f, 10.f));
		return trackedB;
	};
	// the matches of one run as "(q,t) (q,t) ..."
	const auto List = [](const std::vector<DMatch>& matches) {
		String list;
		for (const DMatch& m : matches)
			list += String::FormatString("%s(%u,%u)", list.empty() ? "" : " ", m.queryIdx, m.trainIdx);
		return list;
	};

	// 1. the disc. Four queries; B: the true match of every query at its prediction, an equal-descriptor lookalike of query 0 far
	// away, a scale duplicate of query 2's match on top of it, distractors; query 3 untracked
	{
		Scene scene;
		const std::vector<Point2f> trackedB = Rig(scene, {{100, 100}, {200, 100}, {300, 100}, {400, 100}},
			{{120, 110}, {500, 300}, {220, 110}, {320, 110}, {320, 110}, {420, 110}, {60, 300}, {160, 320}, {260, 340}, {360, 360},
			 {460, 380}, {560, 400}, {80, 420}, {180, 440}, {280, 460}, {380, 40}},
			{{0, 0}, {0, 1}, {1, 2}, {2, 3}, {2, 4}, {3, 5}}, 20260903u);
		Image& imgA = scene.images[0];
		Image& imgB = scene.images[1];
		std::vector<uchar> trackStatus(imgA.keypoints.size(), 1);
		trackStatus[3] = 0;
		PairsMatcher matcher(scene, matchCfg);
		// query 0 rejected: its lookalike sits outside the disc, so the winner has nothing to beat; query 1 accepted: its only
		// close descriptor is inside the disc; query 2 accepted despite the duplicate inside its disc, electing the smaller
		// train index; query 3 untracked, never reaching a disc
		std::vector<DMatch> matches, again, moved;
		const unsigned numMatches = MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, matches);
		TEST_CHECK(numMatches == 2 && List(matches) == "(1,2) (2,3)", "disc: %s, expected (1,2) (2,3)", List(matches).c_str());
		// the lookalike moved INSIDE the disc of query 0 is accepted: what rejected the match was its position
		imgB.keypoints[1].pt = cv::Point2f(121.f, 110.f);
		TEST_CHECK(MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, moved) == 3 && moved[0].queryIdx == 0 &&
			moved[0].trainIdx == 0, "disc: the lookalike moved into the disc left %s, expected 3 matches starting with (0,0)", List(moved).c_str());
		imgB.keypoints[1].pt = cv::Point2f(500.f, 300.f);
		MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, again);
		TEST_CHECK(List(again) == List(matches), "disc: a second run gave %s, the first %s", List(again).c_str(), List(matches).c_str());
	}

	// 2. the band. Six queries on the row y = 100 and every epipole on that row, so every epipolar line is horizontal ("along" is x,
	// "across" is y). B, per query: 0) its true match one bit off, an impostor 5 px ACROSS the line (inside the two-cell disc,
	// outside the half-cell band, the closer descriptor) and a rival 5 px ALONG it; 1) its true match and an equal-descriptor
	// rival 6 px along; 2) its true match and a same-feature duplicate 2 px along; 3) its true match alone, a lookalike far away;
	// 4) its true match, a rival 6 px along and a lookalike far away; 5) its true match and a random rival 6 px along; distractors
	{
		Scene scene;
		const std::vector<Point2f> trackedB = Rig(scene, {{100, 100}, {200, 100}, {300, 100}, {400, 100}, {500, 100}, {600, 100}},
			{{120, 110}, {120, 115}, {125, 110}, {220, 110}, {226, 110}, {320, 110}, {322, 110}, {420, 110}, {500, 300}, {520, 110},
			 {526, 110}, {300, 400}, {620, 110}, {626, 110}, {60, 300}, {160, 320}, {260, 340}, {360, 360}, {380, 40}},
			{{0, 0}, {0, 1}, {1, 3}, {1, 4}, {2, 5}, {2, 6}, {3, 7}, {3, 8}, {4, 9}, {4, 11}, {5, 12}}, 20260916u);
		Image& imgA = scene.images[0];
		Image& imgB = scene.images[1];
		imgB.descriptors.at<uint8_t>(0, 0) ^= 0x01;
		const std::vector<uchar> trackStatus(imgA.keypoints.size(), 1);
		PairsMatcher matcher(scene, matchCfg);
		// F = [e]x: the epipolar line of a point of A is the line through the epipole e and the same coordinates in B
		const auto Fundamental = [](float ex, float ey) {
			Matrix3x3 F;
			F(0,0) = 0;   F(0,1) = -1;  F(0,2) = ey;
			F(1,0) = 1;   F(1,1) = 0;   F(1,2) = -ex;
			F(2,0) = -ey; F(2,1) = ex;  F(2,2) = 0;
			return F;
		};
		const auto Run = [&](std::optional<Matrix3x3> F) {
			search.F = F;
			std::vector<DMatch> matches;
			const unsigned numMatches = MatchFeaturesGuided(matcher, imgA, imgB, trackedB, trackStatus, search, 0, matches);
			// a returned count other than the matches' is part of the answer
			return List(matches) + (numMatches == matches.size() ? String() : String::FormatString(" (%u returned)", numMatches));
		};
		// the epipole far off the frame: query 0 refused (its impostor, off the line, is never a candidate but the reference
		// outside the band sees it), 1 refused (ambiguous on the line), 2 matches electing the smaller of its duplicates, 3 and
		// 4 refused (they have to beat the far lookalike), 5 matches
		const String band = Run(Fundamental(2000.f, 100.f));
		TEST_CHECK(band == "(2,5) (5,12)", "band: %s, expected (2,5) (5,12)", band.c_str());
		// no geometry: the disc, where query 0's impostor is a candidate again and wins
		const String disc = Run(std::nullopt);
		TEST_CHECK(disc == "(0,1) (2,5) (5,12)", "disc: %s, expected (0,1) (2,5) (5,12)", disc.c_str());
		// the epipole 14 px from query 0's prediction: no usable line direction there, so that query searches the disc
		const String nearEpipole = Run(Fundamental(130.f, 100.f));
		TEST_CHECK(nearEpipole == "(0,1) (2,5) (5,12)", "near the epipole: %s, expected (0,1) (2,5) (5,12)", nearEpipole.c_str());
		const String again = Run(Fundamental(2000.f, 100.f));
		TEST_CHECK(again == band, "band: a second run gave %s, the first %s", again.c_str(), band.c_str());
	}
	return scope.Pass("the ratio against the best descriptor outside the disc or band, the band along the epipolar line, its "
		"fallbacks to the disc, and the same matches on a second run");
}

/*----------------------------------------------------------------*/

// The wedge pair the assembly and re-verification tests build on (the scene is MakeRig's wedge of two cameras): the verdict of an
// admitted pair, every cell of the region [X0,X1) x [Y0,Y1) of the 64x64 warp grid projected through the wedge, each with a confidence
// of its own and its B side lying denseBias pixels across the epipolar lines (which run along the baseline direction (0.4, 0.1)): the
// systematic error of a coarse warp; and the guided matches, one per `stride` cells of the region: numSparse exact descriptor
// correspondences, then numOutliers whose B side is displaced 40 px across the epipolar lines. Both kinds occupy their cell. The
// guided keypoints are appended to the scene's two images.
struct WedgePair {
	static constexpr int CELLS = 64, X0 = 14, X1 = 50, Y0 = 13, Y1 = 47;
	PairVerdict verdict;
	std::vector<DMatch> guided;
};
static WedgePair BuildWedgePair(Scene& scene, float denseBias, unsigned numSparse, unsigned numOutliers, size_t stride)
{
	ASSERT(scene.images.size() == 2 && stride > 0);
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	const Point2f across(normalized(Point2f(-0.1f, 0.4f)));
	WedgePair wedge;
	wedge.verdict.admitted = true;
	std::vector<Point2f> exactB;
	for (int y = WedgePair::Y0; y < WedgePair::Y1; ++y) {
		for (int x = WedgePair::X0; x < WedgePair::X1; ++x) {
			const Point2f ptA(CellToPixel(Point2f((float)x, (float)y), cv::Size(WedgePair::CELLS, WedgePair::CELLS), imgA.GetSize()));
			Point2f ptB;
			if (!DenseWedgeCorrespondence(imgA, imgB, ptA, ptB))
				continue;
			wedge.verdict.inliersA.push_back(ptA);
			wedge.verdict.inliersB.push_back(ptB + across*denseBias);
			wedge.verdict.confidences.push_back(0.25f + 0.5f*(float)(x + y)/(float)(2*(WedgePair::CELLS - 1)));
			exactB.push_back(ptB);
		}
	}
	wedge.verdict.confidentAreaA = wedge.verdict.confidentAreaB = wedge.verdict.inlierAreaA = wedge.verdict.inlierAreaB =
		(float)wedge.verdict.inliersA.size()/(float)(WedgePair::CELLS*WedgePair::CELLS);
	const size_t numGuided = numSparse + numOutliers;
	for (size_t k = 0; k < wedge.verdict.inliersA.size() && wedge.guided.size() < numGuided; k += stride) {
		const uint32_t idx = (uint32_t)imgA.keypoints.size();
		imgA.keypoints.emplace_back(wedge.verdict.inliersA[k].x, wedge.verdict.inliersA[k].y, 10.f);
		imgB.keypoints.emplace_back(exactB[k].x, exactB[k].y + (wedge.guided.size() >= numSparse ? 40.f : 0.f), 10.f);
		wedge.guided.emplace_back(idx, idx);
	}
	ASSERT(wedge.guided.size() == numGuided); // the region offers cells enough for the guided matches
	return wedge;
}

// The wedge pair assembled and stored with its dense segment: 80 sound guided matches and 6 the geometry contradicts, the fill drawn
// from the verdict's region (the verdict's B side denseBias px across the epipolar lines, the guided matches exact); the pair is
// armed with the truth; false when the pair was not assembled
static bool StoreDenseWedgePair(Scene& scene, PairsMatcher& matcher, const DenseMatchConfig& config, float denseBias = 0.f)
{
	const WedgePair wedge(BuildWedgePair(scene, denseBias, 80, 6, 14));
	ImagePair pair(0, 1);
	ArmPair(pair, scene);
	DenseMatches dense;
	if (!AssemblePairDense(matcher, scene.images[0], scene.images[1], wedge.verdict, wedge.guided, config, WedgePair::CELLS, pair, dense))
		return false;
	std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
	StorePairDense(scene, pairIndexMap, std::move(pair), dense, WedgePair::CELLS);
	return true;
}

// Pair assembly and storage (AssemblePairDense, StorePairDense) on the wedge:
// - guided u dense on an exact geometry: the 80 sound guided matches become the sparse segment, the 6 the geometry contradicts the
//   pair's outliers, the fill draws one point per bucket of the fixed-pitch grid (DenseFillGridSide) that holds a region cell and no
//   guided match, each carrying its cell's own confidence, and ONE geometry stands for the pair: the union fit's, though the verdict
//   handed over a pose 3 degrees off;
// - no guided match: the dense segment is the whole evidence (a bucket count, capped by DenseFillCeiling), the pose still exact;
// - a verdict whose every fourth cell lies 3 px across the epipolar lines keeps only the exact cells on the pair's own bar;
// - a fill too small for a fit of its own leaves the verdict's geometry untouched;
// - the store: dense keypoints land past each image's described prefix, their indices depend only on how many keypoints the images
//   already carry, the dense block is inserted before the strict filter's rejects, and a second run labels everything identically;
// - the append (AppendDenseMatches): keypoint positions, response and size, the pair's own middle segment, a second fill extending it;
// - the dense fit weight: a dense fill displaced 2 px across the epipolar lines everywhere against exact guided matches, both passing
//   the 4 px bar, the union fit following the far more numerous dense matches; the weighted refinement that closes the assembly gives
//   each dense correspondence DenseMatchConfig::denseFitWeight against a sparse one's 1 and brings the pose back to the sparse matches,
//   where the same assembly at full weight stays on the bias.
bool DenseAssemblyTest()
{
	TestScope scope("DenseAssemblyTest");
	Scene scene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
	Image& imgA = scene.images[0];
	Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	constexpr int cells = WedgePair::CELLS;
	const int rx0 = WedgePair::X0, rx1 = WedgePair::X1, ry0 = WedgePair::Y0, ry1 = WedgePair::Y1;

	// 86 guided matches over the region: 80 exact descriptor correspondences on cell centres, then 6 the geometry contradicts
	const WedgePair wedge(BuildWedgePair(scene, 0.f, 80, 6, 14));
	const PairVerdict& verdict = wedge.verdict;
	const std::vector<DMatch>& guided = wedge.guided;
	const size_t numSparse = 80, numOutliers = 6, numGuided = guided.size();
	const size_t numInlierCells = verdict.inliersA.size();
	TEST_CHECK(numInlierCells == (size_t)((rx1-rx0)*(ry1-ry0)), "%u of the %u region cells have a correspondence", (unsigned)numInlierCells, (unsigned)((rx1-rx0)*(ry1-ry0)));

	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	DenseMatchConfig config; // minConfidence 0.1, denseMatchesPerFrame 2000 (a density per full frame of overlap)
	// the geometry the verdict handed over, 3 degrees off the truth, so that a pair carrying the union fit's geometry is told apart from
	// one that kept the verdict's
	const REAL tilt = D2R(REAL(3));
	Pose3D poseVerdict(poseGT);
	poseVerdict.R = poseGT.R * Matrix3x3(COS(tilt), 0, SIN(tilt), 0, 1, 0, -SIN(tilt), 0, COS(tilt));
	const auto ArmVerdictGeometry = [&](ImagePair& pair, const Pose3D& pose) {
		pair.relativePose = pose;
		pair.E = ImagePair::ComposeEssentialMatrix(pose);
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), imgA.GetK(), imgB.GetK());
	};
	const auto PoseErrorFromGT = [&](const Pose3D& pose, REAL& angleErr, REAL& tSim) {
		angleErr = RotationErrorDeg(pose.R, poseGT.R);
		tSim = ABS(normalized(pose.GetT()).dot(normalized(poseGT.GetT())));
	};

	// guided u dense on an exact geometry
	ImagePair pair(0, 1);
	ArmVerdictGeometry(pair, poseVerdict);
	DenseMatches dense;
	TEST_CHECK(AssemblePairDense(matcher, imgA, imgB, verdict, guided, config, cells, pair, dense),
		"an admitted pair with %u guided matches was not assembled", (unsigned)numGuided);
	TEST_CHECK(pair.matches.size() == numSparse && (size_t)pair.numFilteredInliers == numSparse && pair.outlierMatches.size() == numOutliers &&
		pair.numDenseInliers == 0, "the sparse segment holds %u of %u matches (%d filtered, %u outliers, %d dense)",
		(unsigned)pair.matches.size(), (unsigned)numGuided, pair.numFilteredInliers, (unsigned)pair.outlierMatches.size(), pair.numDenseInliers);
	FOREACH(k, pair.matches)
		TEST_CHECK(pair.matches[k].queryIdx == k && pair.matches[k].trainIdx == k, "sparse match %u is (%u,%u), so the segment lost the guided order",
			(unsigned)k, pair.matches[k].queryIdx, pair.matches[k].trainIdx);
	// the region cell each verdict/guided index falls on is known without a pixel round-trip: the region loop ran y then x with no skipped
	// cell (checked at numInlierCells above), and the guided loop took every 14th of those in the same order. This calls the production
	// function the code under test calls for its pitch, so unlike the bucket-mapping arithmetic below it does not independently
	// re-verify the pitch formula
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
	TEST_CHECK(dense.pointsA.size() == expectedDense && dense.pointsB.size() == dense.pointsA.size() && dense.confidences.size() == dense.pointsA.size(),
		"the fill drew %u/%u points with %u confidences, expected %u", (unsigned)dense.pointsA.size(), (unsigned)dense.pointsB.size(),
		(unsigned)dense.confidences.size(), (unsigned)expectedDense);
	// every drawn point is one of the verdict's cells, carries that cell's own confidence, and sits on no cell a guided match occupies
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
			TEST_CHECK(inlierOfCell[cell] >= 0 && !occupied[cell] && dense.confidences[k] == verdict.confidences[inlierOfCell[cell]],
				"dense point %u at cell (%d,%d) is %s and carries confidence %g", (unsigned)k, cell%cells, cell/cells,
				inlierOfCell[cell] < 0 ? "not a verdict cell" : (occupied[cell] ? "on a guided match" : "mislabelled"), dense.confidences[k]);
		}
	}
	// ONE geometry for the pair, and it is the union fit's: the verdict handed over a pose 3 degrees off the two cameras and the
	// assembled pair carries the right one
	REAL angleErr, tSim;
	TEST_CHECK(pair.relativePose.has_value(), "the assembled pair carries no relative pose");
	PoseErrorFromGT(pair.relativePose.value(), angleErr, tSim);
	TEST_CHECK(angleErr <= REAL(0.05) && tSim >= REAL(0.9999) && pair.F.has_value() && pair.E.has_value(),
		"the assembled pose is %.4f deg and %.6f off the two cameras (F %d, E %d)", angleErr, tSim, (int)pair.F.has_value(), (int)pair.E.has_value());

	// no guided match at all: the pair is still assembled, its dense segment its whole evidence, and the fill draws one point per bucket
	// of the fixed-pitch grid that holds a region cell -- nothing left to strike out this time, but still a bucket count, not a cell
	// count. The region's edge does not tile the fixed pitch evenly, so a border bucket counts as used from just a sliver of region
	// inside it, and the resulting bucket count runs a little over the per-frame density -- so DenseFillCeiling actually binds here,
	// unlike in the guided u dense case above where the guided matches' own strike-outs keep the draw under it.
	const size_t expectedDenseOnly = MINF(numCandidateBuckets, (size_t)DenseFillCeiling(config, verdict));
	ImagePair pairDense(0, 1);
	ArmVerdictGeometry(pairDense, poseVerdict);
	DenseMatches denseOnly;
	TEST_CHECK(AssemblePairDense(matcher, imgA, imgB, verdict, std::vector<DMatch>(), config, cells, pairDense, denseOnly) &&
		pairDense.matches.empty() && pairDense.numFilteredInliers == 0 && denseOnly.pointsA.size() == expectedDenseOnly,
		"a pair with no guided match holds %u sparse (%d filtered) and %u dense correspondences, expected 0 and %u",
		(unsigned)pairDense.matches.size(), pairDense.numFilteredInliers, (unsigned)denseOnly.pointsA.size(), (unsigned)expectedDenseOnly);
	TEST_CHECK(pairDense.relativePose.has_value(), "a dense-only pair carries no relative pose");
	PoseErrorFromGT(pairDense.relativePose.value(), angleErr, tSim);
	TEST_CHECK(angleErr <= REAL(0.05) && tSim >= REAL(0.9999), "the dense-only pose is %.4f deg and %.6f off the two cameras", angleErr, tSim);

	// a verdict whose every fourth cell lands 3 px across the epipolar lines in B, the imprecision of a coarse warp: held to the pair's
	// own epipolar bar (1.5 px here, denseEpipolarErrorFactor 1) the dense segment keeps only the exact cells
	{
		PairVerdict verdictOff(verdict);
		const float acrossLen = SQRT(0.1f*0.1f + 0.4f*0.4f);
		const Point2f across(-0.1f/acrossLen*3.f, 0.4f/acrossLen*3.f); // perpendicular to the baseline (0.4, 0.1)
		for (size_t k = 0; k < verdictOff.inliersB.size(); k += 4)
			verdictOff.inliersB[k] = Point2f(verdictOff.inliersB[k].x + across.x, verdictOff.inliersB[k].y + across.y);
		MatchConfig matchCfgTight(matchCfg);
		matchCfgTight.maxEpipolarError = 1.5f;
		PairsMatcher matcherTight(scene, matchCfgTight);
		ImagePair pairOff(0, 1);
		ArmVerdictGeometry(pairOff, poseVerdict);
		DenseMatches denseOff;
		TEST_CHECK(AssemblePairDense(matcherTight, imgA, imgB, verdictOff, std::vector<DMatch>(), config, cells, pairOff, denseOff),
			"the pair whose every fourth cell is displaced was not assembled");
		unsigned numKeptDisplaced = 0;
		FOREACH(k, denseOff.pointsA) {
			Point2f ptB;
			TEST_CHECK(DenseWedgeCorrespondence(imgA, imgB, denseOff.pointsA[k], ptB), "a kept dense point %u has no correspondence through the wedge", (unsigned)k);
			if (SQRT(SQUARE(denseOff.pointsB[k].x - ptB.x) + SQUARE(denseOff.pointsB[k].y - ptB.y)) > 1.5f)
				++numKeptDisplaced;
		}
		const unsigned numKept = (unsigned)denseOff.pointsA.size();
		TEST_CHECK(numKept != 0 && numKeptDisplaced == 0 && numKept <= verdictOff.inliersB.size() - verdictOff.inliersB.size()/4,
			"the dense segment keeps %u cells (%u displaced) of %u on the epipolar bar", numKept, numKeptDisplaced, (unsigned)verdictOff.inliersB.size());
	}

	// a fill too small for a fit of its own (under the estimator's 8 correspondences): the verdict's geometry stands, unchanged, and the
	// pair is stored on it. The inlier areas stay the parent verdict's -- only the SAMPLE of inlier cells shrinks here, to drive the
	// union fit below the estimator's minimum -- so DenseFillCeiling does not also clamp the fill down from the 6 cells this case means
	// to exercise.
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
	ArmPair(pairTiny, scene); // exact, so its own cells survive the classification
	const Matrix3x3 Fverdict(pairTiny.F.value());
	DenseMatches denseTiny;
	TEST_CHECK(AssemblePairDense(matcher, imgA, imgB, tinyVerdict, std::vector<DMatch>(), config, cells, pairTiny, denseTiny) &&
		denseTiny.pointsA.size() == tinyVerdict.inliersA.size(), "a %u-cell fill was not assembled (%u dense correspondences)",
		(unsigned)tinyVerdict.inliersA.size(), (unsigned)denseTiny.pointsA.size());
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			TEST_CHECK(pairTiny.F.value()(r, c) == Fverdict(r, c) && pairTiny.relativePose->R(r, c) == poseGT.R(r, c),
				"a fill too small to fit still moved the pair's geometry at (%d,%d)", r, c);

	// the store: the dense keypoints of a pair land past each image's described prefix, and the indices they get depend only on how many
	// keypoints the two images already carry -- which is why the pass has to store serially, in pair order. The first pair also carries
	// one match past its filtered count -- a RANSAC inlier the strict filter rejected, which lives in `matches` after the sparse
	// segment -- so that where the dense block is inserted is pinned: a fill pushed onto the end of `matches` would land outside the
	// track-forming prefix BuildTracks reads
	const auto RunStore = [cells](Scene& out) {
		out = MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D::Identity(), Pose3D::Identity()});
		for (unsigned i = 0; i < 3; ++i)
			for (unsigned k = 0; k < 5; ++k)
				out.images[i].keypoints.emplace_back(10.f*(float)k, 20.f*(float)i + 5.f, 10.f);
		std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap;
		for (unsigned j = 1; j < 3; ++j) {
			ImagePair pairStored(0, j);
			for (uint32_t m = 0; m < j; ++m)
				pairStored.matches.emplace_back(m, m);
			pairStored.numFilteredInliers = (int)j;
			if (j == 1)
				pairStored.matches.emplace_back(4, 4); // the rejected tail, past the filtered count
			DenseMatches denseStored;
			for (unsigned d = 0; d < 5 - j; ++d) {
				denseStored.pointsA.emplace_back(100.f + 10.f*(float)d, 100.f + 10.f*(float)j);
				denseStored.pointsB.emplace_back(110.f + 10.f*(float)d, 100.f + 10.f*(float)j);
				denseStored.confidences.push_back(0.5f + 0.1f*(float)d);
			}
			StorePairDense(out, pairIndexMap, std::move(pairStored), denseStored, cells);
		}
		return pairIndexMap;
	};
	Scene storeScene;
	const std::unordered_map<PairIdx::PairIndex, IIndex> pairIndexMap(RunStore(storeScene));
	TEST_CHECK(storeScene.pairs.size() == 2 && pairIndexMap.size() == 2 && pairIndexMap.at(PairIdx(0, 1).idx) == 0 && pairIndexMap.at(PairIdx(0, 2).idx) == 1,
		"the store left %u pairs and %u index entries", (unsigned)storeScene.pairs.size(), (unsigned)pairIndexMap.size());
	// image 0 took both fills (4 then 3), images 1 and 2 one each, all past the described 5
	const unsigned expectedKeypoints[3] = { 5 + 4 + 3, 5 + 4, 5 + 3 };
	for (unsigned i = 0; i < 3; ++i) {
		const Image& img = storeScene.images[i];
		TEST_CHECK(img.NumDescribedKeypoints() == 5 && img.keypoints.size() == expectedKeypoints[i] && !img.IsDenseKeypoint(4) && img.IsDenseKeypoint(5),
			"image %u carries %u keypoints past a described prefix of %u, expected %u", i, (unsigned)img.keypoints.size(), img.NumDescribedKeypoints(), expectedKeypoints[i]);
	}
	// the dense segment of each pair names the keypoints the append handed out: image 0 continues past its first fill, image 2 starts at
	// its own prefix
	const ImagePair& pair01 = storeScene.pairs[0];
	const ImagePair& pair02 = storeScene.pairs[1];
	TEST_CHECK(pair01.numFilteredInliers == 1 && pair01.numDenseInliers == 4 && pair01.matches.size() == 6 &&
		pair02.numFilteredInliers == 2 && pair02.numDenseInliers == 3 && pair02.matches.size() == 5,
		"the stored pairs partition %u/%u matches as %d+%d and %d+%d", (unsigned)pair01.matches.size(), (unsigned)pair02.matches.size(),
		pair01.numFilteredInliers, pair01.numDenseInliers, pair02.numFilteredInliers, pair02.numDenseInliers);
	for (unsigned d = 0; d < 4; ++d)
		TEST_CHECK(pair01.matches[1 + d].queryIdx == 5 + d && pair01.matches[1 + d].trainIdx == 5 + d, "the first pair's dense match %u is (%u,%u), expected (%u,%u)",
			d, pair01.matches[1 + d].queryIdx, pair01.matches[1 + d].trainIdx, 5 + d, 5 + d);
	for (unsigned d = 0; d < 3; ++d)
		TEST_CHECK(pair02.matches[2 + d].queryIdx == 9 + d && pair02.matches[2 + d].trainIdx == 5 + d, "the second pair's dense match %u is (%u,%u), expected (%u,%u)",
			d, pair02.matches[2 + d].queryIdx, pair02.matches[2 + d].trainIdx, 9 + d, 5 + d);
	// and the rejected match the first pair carried stays behind the whole dense block: the fill is inserted into the track-forming
	// prefix, never appended past the segments that follow it
	const unsigned idxRejected = (unsigned)pair01.matches.size() - 1;
	TEST_CHECK(pair01.matches[idxRejected].queryIdx == 4 && pair01.matches[idxRejected].trainIdx == 4,
		"match %u of the first pair is (%u,%u), not the rejected (4,4), so the dense block did not land before it",
		idxRejected, pair01.matches[idxRejected].queryIdx, pair01.matches[idxRejected].trainIdx);
	// the same two stores over again label everything identically
	Scene storeSceneAgain;
	RunStore(storeSceneAgain);
	FOREACH(i, storeScene.images) {
		const Image& img = storeScene.images[i];
		const Image& imgAgain = storeSceneAgain.images[i];
		TEST_CHECK(img.keypoints.size() == imgAgain.keypoints.size() && img.NumDescribedKeypoints() == imgAgain.NumDescribedKeypoints(),
			"a second store run gave image %u %u keypoints instead of %u", (unsigned)i, (unsigned)imgAgain.keypoints.size(), (unsigned)img.keypoints.size());
		FOREACH(k, img.keypoints)
			TEST_CHECK(img.keypoints[k].pt == imgAgain.keypoints[k].pt && img.keypoints[k].response == imgAgain.keypoints[k].response,
				"a second store run moved keypoint %u of image %u", (unsigned)k, (unsigned)i);
	}
	FOREACH(p, storeScene.pairs) {
		const ImagePair& lhs = storeScene.pairs[p];
		const ImagePair& rhs = storeSceneAgain.pairs[p];
		TEST_CHECK(lhs.matches.size() == rhs.matches.size() && lhs.numFilteredInliers == rhs.numFilteredInliers && lhs.numDenseInliers == rhs.numDenseInliers,
			"a second store run partitioned pair %u differently", (unsigned)p);
		FOREACH(k, lhs.matches)
			TEST_CHECK(lhs.matches[k].queryIdx == rhs.matches[k].queryIdx && lhs.matches[k].trainIdx == rhs.matches[k].trainIdx,
				"a second store run relabelled match %u of pair %u", (unsigned)k, (unsigned)p);
	}

	// the dense fill (AppendDenseMatches): the appended keypoints land past each image's described prefix, while the appended matches
	// become the pair's own middle segment -- after the sparse inliers, which stay the pair's descriptor evidence, and before the strict
	// filter's rejects, so the fill is inside the track-forming prefix BuildTracks reads but outside the count every view-graph weight
	// and gate reads
	{
		constexpr int appendCells = 160;
		Scene denseScene(MakeRig(cv::Size(640, 480), 600, {Pose3D::Identity(), Pose3D::Identity()}));
		for (Image& im : denseScene.images)
			im.keypoints.resize(10, cv::KeyPoint(1.f, 1.f, 3.f));
		ImagePair& appendPair = denseScene.pairs.emplace_back(0u, 1u);
		for (uint32_t k = 0; k < 8; ++k)
			appendPair.matches.emplace_back(k, k);
		appendPair.numFilteredInliers = 5; // matches 5..7 are inliers the strict filter then rejected
		const std::vector<Point2f> ptsA{Point2f(100.f, 100.f), Point2f(200.f, 200.f)};
		const std::vector<Point2f> ptsB{Point2f(110.f, 100.f), Point2f(210.f, 200.f)};
		const std::vector<float> confidencesAppended{0.7f, 0.9f};
		TEST_CHECK(AppendDenseMatches(denseScene, appendPair, ptsA, ptsB, confidencesAppended, cv::Size(appendCells, appendCells)) == 2, "dense fill not appended");
		const float cellSize = MAXF((float)640/(float)appendCells, (float)480/(float)appendCells);
		for (IIndex k = 0; k < 2; ++k) {
			const Image& im = denseScene.images[k];
			TEST_CHECK(im.keypoints.size() == 12 && im.NumDescribedKeypoints() == 10 && im.NumDenseKeypoints() == 2 && !im.IsDenseKeypoint(9) && im.IsDenseKeypoint(10),
				"image %u has %u keypoints with boundary %u", (unsigned)k, (unsigned)im.keypoints.size(), im.NumDescribedKeypoints());
			TEST_CHECK(im.keypoints[10].response == 0.7f && im.keypoints[11].response == 0.9f && im.keypoints[10].size == cellSize,
				"image %u dense keypoint response %g / size %g, expected size %g", (unsigned)k, im.keypoints[10].response, im.keypoints[10].size, cellSize);
		}
		TEST_CHECK(denseScene.images[0].keypoints[10].pt == cv::Point2f(100.f, 100.f) && denseScene.images[1].keypoints[10].pt == cv::Point2f(110.f, 100.f),
			"dense keypoint positions");
		// the two dense matches sit at 5 and 6, the strict-filter rejects moved after them, the sparse count is untouched, and the dense
		// count grew by exactly the supplement
		TEST_CHECK(appendPair.matches.size() == 10 && appendPair.numFilteredInliers == 5 && appendPair.numDenseInliers == 2 &&
			appendPair.GetNumFilteredInliers() == 5 && appendPair.GetNumTrackFormingMatches() == 7 &&
			appendPair.matches[5].queryIdx == 10 && appendPair.matches[5].trainIdx == 10 && appendPair.matches[6].queryIdx == 11 && appendPair.matches[7].queryIdx == 5,
			"dense matches not inserted as their own segment (%u matches, %d sparse, %d dense)",
			(unsigned)appendPair.matches.size(), appendPair.numFilteredInliers, appendPair.numDenseInliers);
		// a second dense fill on the same images appends past the first one's dense keypoints, and its matches extend the dense segment
		// rather than reopening the sparse one
		TEST_CHECK(AppendDenseMatches(denseScene, appendPair, ptsA, ptsB, confidencesAppended, cv::Size(appendCells, appendCells)) == 2 &&
			denseScene.images[0].NumDescribedKeypoints() == 10 && denseScene.images[0].NumDenseKeypoints() == 4 &&
			appendPair.numFilteredInliers == 5 && appendPair.numDenseInliers == 4 && appendPair.matches.size() == 12 &&
			appendPair.matches[7].queryIdx == 12 && appendPair.matches[9].queryIdx == 5,
			"a second append moved the described boundary or the sparse count");
	}

	// the dense fit weight: the dense fill displaced 2 px across the epipolar lines everywhere, 60 exact guided matches, on a scene of
	// its own. Both kinds pass the 4 px bar and the union fit follows the far more numerous dense matches
	Scene biasedScene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
	const WedgePair biased(BuildWedgePair(biasedScene, 2.f, 60, 0, 20));
	PairsMatcher biasedMatcher(biasedScene, matchCfg);
	const auto Assemble = [&](float denseFitWeight, REAL& error, size_t& numDense) {
		DenseMatchConfig configWeight;
		configWeight.denseFitWeight = denseFitWeight;
		ImagePair pairWeight(0, 1);
		ArmPair(pairWeight, biasedScene);
		DenseMatches denseWeight;
		if (!AssemblePairDense(biasedMatcher, biasedScene.images[0], biasedScene.images[1], biased.verdict, biased.guided, configWeight, cells,
				pairWeight, denseWeight) || !pairWeight.relativePose.has_value())
			return false;
		error = RotationErrorDeg(pairWeight.relativePose->R, poseGT.R);
		numDense = denseWeight.pointsA.size();
		return true;
	};
	REAL errFull, errWeighted;
	size_t numDenseFull, numDenseWeighted;
	TEST_CHECK(Assemble(1.f, errFull, numDenseFull) && Assemble(DenseMatchConfig().denseFitWeight, errWeighted, numDenseWeighted),
		"the biased pair was not assembled with a relative pose");
	// the premise: the dense matches outnumber the sparse ones and at full weight pull the pose
	TEST_CHECK(numDenseFull >= 4*biased.guided.size() && errFull >= REAL(0.05), "premise not met: %u dense against %u sparse, %.4f deg at full weight",
		(unsigned)numDenseFull, (unsigned)biased.guided.size(), errFull);
	TEST_CHECK(errWeighted <= REAL(0.25)*errFull, "the weighted refinement leaves the pose %.4f deg off, %.4f deg at full weight", errWeighted, errFull);
	return scope.Pass("%u guided + %u dense assembled on the union fit's pose, dense-only, the epipolar bar, the tiny fill, the store and the append; the dense fit weight %.4f deg against %.4f deg at full weight (%u dense, %u sparse)",
		(unsigned)numGuided, (unsigned)expectedDense, errWeighted, errFull, (unsigned)numDenseWeighted, (unsigned)biased.guided.size());
}
/*----------------------------------------------------------------*/

// The scene of PlanarTwinSwitchTest: one 1000x1000 pinhole camera at f 500 that trusts its intrinsics
// (the calibrated branch: the twin's polish needs it), shared by images looking down at the floor
// z = 0, each given by its centre and two tilts (degrees about x, then about y)
struct PlanarTwinView { REAL x, y, z, tiltX, tiltY; };
static Scene MakePlanarTwinScene(const std::vector<PlanarTwinView>& views)
{
	std::vector<Pose3D> poses;
	for (const PlanarTwinView& v : views) {
		const REAL ax = D2R(v.tiltX), ay = D2R(v.tiltY);
		const Matrix3x3 Rx(1, 0, 0, 0, COS(ax), -SIN(ax), 0, SIN(ax), COS(ax));
		const Matrix3x3 Ry(COS(ay), 0, SIN(ay), 0, 1, 0, -SIN(ay), 0, COS(ay));
		const Matrix3x3 down(1, 0, 0, 0, -1, 0, 0, 0, -1); // the camera's z axis along the world's -z
		Pose3D pose;
		pose.R = Rx*Ry*down;
		pose.C = Point3(v.x, v.y, v.z);
		poses.push_back(pose);
	}
	return MakeRig(cv::Size(1000, 1000), 500, poses);
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

// A verified pair armed with the truth and the first numMatches keypoints of both images matched one to one as sparse inliers; with no
// match it is a pose alone, a neighbour that closes loops and is no candidate (a twin needs matches)
static ImagePair MakePlanarTwinPair(const Scene& scene, IIndex a, IIndex b, unsigned numMatches)
{
	ImagePair pair(a, b);
	ArmPair(pair, scene);
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
	TestScope scope("PlanarTwinSwitchTest");

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
	const auto RotationDeg = [](const Pose3D& a, const Pose3D& b) { return RotationErrorDeg(a.R, b.R); };
	// the scene: images, keypoints (all points, or the floor only) and the pairs; `bWrongTwin` stores a pair at its wrong planar
	// twin (the twin of the true pose on the floor matches alone)
	struct PairSpec { IIndex a, b; unsigned numMatches; bool bWrongTwin; };
	const auto Build = [&](Scene& scene, const std::vector<Point3>& points, const std::vector<PairSpec>& specs) -> bool {
		scene = MakePlanarTwinScene(views);
		for (IIndex i = 0; i < 3; ++i)
			TEST_CHECK(AddPlanarTwinKeypoints(scene.images[i], points), "image %u does not see every point", i);
		PairsMatcher matcher(scene, matchCfg);
		for (const PairSpec& spec : specs) {
			const Image& imgA = scene.images[spec.a];
			const Image& imgB = scene.images[spec.b];
			ImagePair pair(MakePlanarTwinPair(scene, spec.a, spec.b, spec.numMatches));
			if (spec.bWrongTwin) {
				ImagePair onFloor(MakePlanarTwinPair(scene, spec.a, spec.b, numFloor));
				ImagePair twin;
				float share;
				const bool bTwin = PlanarTwinDense(matcher, imgA, imgB, config, onFloor, twin, share);
				TEST_CHECK(bTwin && RotationDeg(twin.relativePose.value(), onFloor.relativePose.value()) >= REAL(10),
					"the floor matches of pair %u-%u have no planar twin 10 degrees off (share %g)", spec.a, spec.b, share);
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
			if (!Build(scene, floorPoints, specs))
				return false;
			const Pose3D stored(scene.pairs[0].relativePose.value());
			const unsigned numSwitched = Run(scene, cfg);
			const ImagePair& pair = scene.pairs[0];
			if (run < 2) {
				TEST_CHECK(numSwitched == 0 && pair.relativePose.value() == stored, "with twinMinLoopGain %u the planar pair switched (%u pairs switched)", cfg.twinMinLoopGain, numSwitched);
				continue;
			}
			TEST_CHECK(numSwitched == 1 && IsTrue(scene, 0), "the planar pair stored at its wrong twin is %.3f deg off the true pose after the switch (%u pairs switched), expected it switched to the true pose",
				RotationDeg(pair.relativePose.value(), Pose3D(scene.images[1] / scene.images[0])), numSwitched);
			TEST_CHECK(pair.GetNumFilteredInliers() == numFloor && pair.GetNumDenseInliers() == 0 && pair.outlierMatches.empty(),
				"the switched pair keeps %u of its %u matches as inliers (%u outliers)", pair.GetNumFilteredInliers(), numFloor, (unsigned)pair.outlierMatches.size());
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
			if (!Build(scene, allPoints, specs))
				return false;
			const ImagePair stored(scene.pairs[0]);
			PairsMatcher matcher(scene, matchCfg);
			ImagePair twin;
			float share;
			PlanarTwinDense(matcher, scene.images[0], scene.images[1], config, stored, twin, share);
			TEST_CHECK(share >= 0.5f && share < config.twinMinPlaneShare, "premise not met: a homography explains %.3f of the two-layer matches, expected [0.5, %g)", share, config.twinMinPlaneShare);
			DenseMatchConfig cfg;
			cfg.twinMinPlaneShare = minPlaneShare;
			const unsigned numSwitched = Run(scene, cfg);
			const ImagePair& pair = scene.pairs[0];
			if (minPlaneShare == config.twinMinPlaneShare)
				TEST_CHECK(numSwitched == 0 && pair.relativePose.value() == stored.relativePose.value() &&
					pair.matches.size() == stored.matches.size() && pair.numFilteredInliers == stored.numFilteredInliers,
					"the pair with %.0f%% of its matches on one plane was touched (%u pairs switched)", 100*share, numSwitched);
			else
				TEST_CHECK(numSwitched == 1 && IsTrue(scene, 0), "premise not met: with the gate at %g the two-layer pair does not switch to the true pose (%u pairs switched)", minPlaneShare, numSwitched);
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
		if (!Build(scene, floorPoints, specs))
			return false;
		const unsigned numSwitched = Run(scene, config);
		const bool bP = IsTrue(scene, 0), bQ = IsTrue(scene, 1);
		TEST_CHECK(bNeighboursOfP ? (numSwitched == 2 && bP && bQ) : (numSwitched == 0 && !bP && !bQ),
			"%s P's neighbours %u pairs switched, P %s and Q %s, expected %s", bNeighboursOfP ? "with" : "without", numSwitched,
			bP ? "true" : "wrong", bQ ? "true" : "wrong", bNeighboursOfP ? "both switched to the true pose (Q once P is right)" : "neither switched");
	}
	return scope.Pass("a floor pair at its wrong twin switched (not with the switch off or a margin of 5), a two-layer pair untouched until the gate is lowered, two twins switched best gain first");
}

// The pose sub-case of the test below, the systematic error of a coarse warp the dense fit weight test builds: the dense fill's B side
// this many pixels across the epipolar lines, the guided matches exact
constexpr float DENSE_REVERIFY_DENSE_BIAS = 2.f;
// and the bars the assembly's pose is held to: the dense matches outnumber the sparse ones at least this many times and pull a pose
// fitted at full weight at least this far off the truth, and the pose the denseFitWeight polish closes keeps at most this share of that error
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
	TestScope scope("DenseReverifyPartitionTest");
	const auto MakeWedgeScene = []() {
		return Scene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
	};
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
			const char* const filter = strictFilter ? "strict filter" : "no strict filter";
			Scene scene(MakeWedgeScene());
			MatchConfig matchCfg;
			if (!strictFilter)
				matchCfg.minTriangulationAngle = matchCfg.reprojThreshold = matchCfg.epipoleFilterThreshold = 0.f;
			PairsMatcher matcher(scene, matchCfg);
			DenseMatchConfig config;
			if (!withDense)
				config.denseMatchesPerFrame = 0;
			matcher.SetDense(NULL, config);
			TEST_CHECK(StoreDenseWedgePair(scene, matcher, config), "%s, %s: the wedge pair was not assembled", name, filter);
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
			TEST_CHECK(numSparse == 80 && numOutliers == 6 && (numDense > 0) == withDense, "%s, %s: the stored pair holds %u sparse, %u dense, %u outliers",
				name, filter, numSparse, numDense, (unsigned)numOutliers);
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
				TEST_CHECK(!pair.HasMatches() && pair.outlierMatches.size() == numCandidates && pair.numFilteredInliers == -1 &&
					pair.GetNumDenseInliers() == 0 && !pair.HasGeometricVerification(),
					"%s, %s, re-verified to %u sparse, %u dense, %u outliers of the %u matches, %s geometry, where the refused re-fit leaves it invalidated",
					name, filter, pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(), (unsigned)pair.outlierMatches.size(), numCandidates,
					pair.HasGeometricVerification() ? "with a" : "no");
				continue;
			}
			TEST_CHECK(pair.GetNumFilteredInliers() == numSparse && pair.GetNumDenseInliers() == numDense && pair.outlierMatches.size() == numOutliers &&
				HasDensePartition(scene, pair) && pair.relativePose.has_value(),
				"%s, %s, re-verified to %u sparse, %u dense, %u outliers (from %u, %u, %u), partition %s", name, filter,
				pair.GetNumFilteredInliers(), pair.GetNumDenseInliers(), (unsigned)pair.outlierMatches.size(), numSparse, numDense, (unsigned)numOutliers,
				HasDensePartition(scene, pair) ? "kept" : "broken");
			// the partition the re-verification leaves: the sparse count materialised, which GeometricFilter
			// without the strict filter leaves at -1, and nothing past the track-forming matches
			TEST_CHECK(pair.numFilteredInliers >= 0, "%s, %s, re-verified with the sparse count unmaterialised", name, filter);
			TEST_CHECK(pair.matches.size() == pair.GetNumTrackFormingMatches(), "%s, %s, re-verified with %u matches past the track-forming ones", name, filter,
				(unsigned)pair.matches.size() - pair.GetNumTrackFormingMatches());
		}
	}

	// the pose under the default configuration, against the truth: GeometricFilter over all of the
	// stored pair's matches (the re-verification without the change) follows the biased dense fill,
	// the re-verification follows the exact sparse matches
	Scene scene(MakeWedgeScene());
	MatchConfig matchCfg;
	PairsMatcher matcher(scene, matchCfg);
	DenseMatchConfig config;
	matcher.SetDense(NULL, config);
	TEST_CHECK(StoreDenseWedgePair(scene, matcher, config, DENSE_REVERIFY_DENSE_BIAS), "the biased wedge pair was not assembled");
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const Pose3D poseGT(imgB / imgA);
	const auto RotationError = [&poseGT](const ImagePair& pair) {
		return RotationErrorDeg(pair.relativePose->R, poseGT.R);
	};
	ImagePair fullWeight(scene.pairs[0]);
	TEST_CHECK(matcher.GeometricFilter(imgA, imgB, fullWeight) && fullWeight.relativePose.has_value(), "GeometricFilter verified no pose on the biased pair");
	const REAL errFull = RotationError(fullWeight);
	matcher.ComputeRelativePoses(true, false, {scene.cameras[0]});
	const ImagePair& biased = scene.pairs[0];
	TEST_CHECK(biased.relativePose.has_value() && HasDensePartition(scene, biased), "the biased pair was re-verified to no pose or a broken partition");
	TEST_CHECK(biased.GetNumDenseInliers() >= DENSE_REVERIFY_MIN_DENSE_PER_SPARSE*biased.GetNumFilteredInliers() && errFull >= DENSE_REVERIFY_MIN_FULL_WEIGHT_ERROR_DEG,
		"premise not met: %u dense against %u sparse, %.4f deg through GeometricFilter at full weight", biased.GetNumDenseInliers(), biased.GetNumFilteredInliers(), errFull);
	const REAL errReverified = RotationError(biased);
	TEST_CHECK(errReverified <= DENSE_REVERIFY_MAX_ERROR_SHARE*errFull, "the re-verified pose is %.4f deg off with the dense fill biased, %.4f deg through GeometricFilter at full weight",
		errReverified, errFull);
	return scope.Pass("with the dense fill biased %g px the re-verified pose is %.4f deg off, %.4f deg through GeometricFilter at full weight (%u dense, %u sparse)",
		DENSE_REVERIFY_DENSE_BIAS, errReverified, errFull, biased.GetNumDenseInliers(), biased.GetNumFilteredInliers());
}

// ===============================================================================
// The described/dense keypoint boundary: the stored count is what survives a descriptor release and an .sfm round-trip of an image
// whose keypoints.size() > descriptors.rows -- the two arrays serialize independently, so nothing else would notice the boundary
// moving -- and what PairsMatcher::FilterRedundantKeypoints moves through the same remap it applies to the keypoint indices
// ===============================================================================
// A keypoint of a test image: position and, for a described one, its response (size 4), for a dense one its warp confidence (size 6)
struct TestKeypoint { float x, y, value; };
// A new image of the scene holding the described keypoints, then (the described prefix closed first) the dense ones; `close` also closes the
// prefix of an image with no dense keypoint, as a scene a RoMa2 pass matched leaves it
static Image& AddKeypointImage(Scene& scene, const std::vector<TestKeypoint>& described, const std::vector<TestKeypoint>& dense = {}, bool close = false)
{
	Image& img = scene.images.emplace_back((IIndex)scene.images.size(), String::FormatString("%u.jpg", (unsigned)scene.images.size()));
	for (const TestKeypoint& kp : described)
		img.keypoints.emplace_back(kp.x, kp.y, 4.f, -1.f, kp.value);
	if (close || !dense.empty())
		img.CloseDescribedKeypoints();
	for (const TestKeypoint& kp : dense)
		img.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(kp.x, kp.y), kp.value, 6.f));
	return img;
}
// A pair of the scene with the given matches (query, train) and the sparse and dense counts
static ImagePair& AddKeypointPair(Scene& scene, IIndex a, IIndex b, const std::vector<std::pair<uint32_t, uint32_t>>& matches, int numSparse, int numDense = 0)
{
	ImagePair& pair = scene.pairs.emplace_back(a, b);
	for (const auto& [query, train] : matches)
		pair.matches.emplace_back(query, train);
	pair.numFilteredInliers = numSparse;
	pair.numDenseInliers = numDense;
	return pair;
}
// The scene's duplicate keypoints and matches removed (PairsMatcher::FilterRedundantKeypoints)
static void FilterRedundantKeypointsOf(Scene& scene)
{
	MatchConfig cfg;
	cfg.minMatches = 1;
	PairsMatcher(scene, cfg).FilterRedundantKeypoints();
}

// Nine hand-built micro-scenes, each pinning one defect of the described/dense boundary:
// - the boundary of an image (the dense keypoints appended past the described ones, closing it twice, a descriptor-indexed selection
//   returning described keypoints only, the boundary outliving the descriptor release and the .sfm round trip);
// - an image that never had dense keypoints stays described-only through the filter, in the order and with the survivors it had before
//   dense supplementation existed;
// - the boundary moves through the filter's remap (a removal inside the described prefix shrinks it; a dense keypoint on a described one
//   collapses onto it; of two coincident dense ones the more confident survives; a standalone dense point stays in the dense suffix);
// - the match-level described-wins rule decides something against the weight comparison;
// - the match partition survives the duplicate-match filter segment by segment (a compaction that sorts the matches moves rejects into
//   the track-forming prefix);
// - a two-hop chain of coincident dense keypoints collapses onto the described survivor;
// - the dedup comparator is a strict weak ordering (an intransitive triple at 0.08 px spacing);
// - the duplicate-match winner knows its segment (a reject does not beat a track-forming supplement match).
bool DenseKeypointBoundaryTest()
{
	TestScope scope("DenseKeypointBoundaryTest");

	// one image with 6 described keypoints and 4 dense ones appended past them
	Scene scene;
	Image& img = AddKeypointImage(scene, {{0, 1, 0.02f}, {1, 1, 0.02f}, {2, 1, 0.02f}, {3, 1, 0.02f}, {4, 1, 0.02f}, {5, 1, 0.02f}});
	img.descriptors = cv::Mat::zeros(6, 32, CV_8U);
	TEST_CHECK(!img.HasDenseKeypoints() && img.NumDescribedKeypoints() == 6 && img.NumDenseKeypoints() == 0, "an image without dense keypoints must report every keypoint described");
	img.CloseDescribedKeypoints();
	for (unsigned i = 0; i < 4; ++i)
		img.keypoints.push_back(Image::MakeDenseKeypoint(Point2f(10.f + i, 2.f), 0.8f, 6.f));
	// closing the boundary a second time (a second supplemented pair on the same image) must not swallow the dense keypoints the first
	// one appended
	img.CloseDescribedKeypoints();
	TEST_CHECK(img.HasDenseKeypoints() && img.NumDescribedKeypoints() == 6 && img.NumDenseKeypoints() == 4, "boundary %u for %u keypoints",
		img.NumDescribedKeypoints(), (unsigned)img.keypoints.size());
	for (uint32_t i = 0; i < (uint32_t)img.keypoints.size(); ++i)
		TEST_CHECK(img.IsDenseKeypoint(i) == (i >= 6), "keypoint %u misclassified", i);
	// the described prefix is what a descriptor-indexed selection may return
	for (const unsigned idx : img.SelectTopKeypoints(10))
		TEST_CHECK(idx < 6, "SelectTopKeypoints returned the dense keypoint %u", idx);
	// the boundary outlives the descriptor release: this is the whole reason it is stored and not derived from descriptors.rows, which is
	// zero from here on
	img.descriptors.release();
	TEST_CHECK(img.NumDescribedKeypoints() == 6 && !img.IsDenseKeypoint(5) && img.IsDenseKeypoint(6), "boundary lost with the descriptors");
	// and it round-trips through .sfm with keypoints.size() > descriptors.rows
	ScopedTempDir tmp("DenseKeypointBoundaryTest");
	TEST_CHECK(tmp.IsValid() && scene.Save(tmp("scene.sfm")), "the scene was not saved");
	Scene loaded;
	TEST_CHECK(loaded.Load(tmp("scene.sfm")), "load");
	const Image& loadedImg = loaded.images[0];
	TEST_CHECK(loadedImg.keypoints.size() == 10 && loadedImg.HasDenseKeypoints() && loadedImg.NumDescribedKeypoints() == 6 && loadedImg.NumDenseKeypoints() == 4 &&
		!loadedImg.IsDenseKeypoint(5) && loadedImg.IsDenseKeypoint(6), "boundary not preserved by the .sfm round-trip");
	TEST_CHECK(loadedImg.keypoints[7].response == 0.8f && loadedImg.keypoints[7].size == 6.f, "dense keypoint response/size not preserved");

	// A described-only image (no dense keypoints at all) must reach exactly the survivors and the order the filter emitted before dense
	// supplementation existed: the dense segment is empty, the cross-segment prune has nothing to do, and the described segment is the
	// whole array -- same comparator, same leader-based grouping, same survivor, same position-order compaction as before the two-segment
	// split existed. Unsorted on purpose, with one duplicate pair (30,30 twice; the higher response*size wins)
	{
		Scene noDenseScene;
		Image& im = AddKeypointImage(noDenseScene, {{40, 40, 0.05f}, {10, 10, 0.05f}, {30, 30, 0.01f}, {20, 20, 0.05f}, {30, 30, 0.05f}});
		FilterRedundantKeypointsOf(noDenseScene);
		TEST_CHECK(!im.HasDenseKeypoints() && im.keypoints.size() == 4 && im.NumDescribedKeypoints() == 4, "a described-only image must stay described-only after the filter");
		const float expectedNoDenseX[4] = {10.f, 20.f, 30.f, 40.f};
		for (uint32_t f = 0; f < 4; ++f)
			// the invariant, checked directly rather than only by count: nothing below the boundary is ever a dense keypoint when there never was one
			TEST_CHECK(im.keypoints[f].pt.x == expectedNoDenseX[f] && !im.IsDenseKeypoint(f), "described-only filter order at %u: x=%g", f, im.keypoints[f].pt.x);
		TEST_CHECK(im.keypoints[2].response == 0.05f, "described-only filter kept the weaker duplicate");
	}

	// FilterRedundantKeypoints must move the boundary through the same remap it applies to the keypoint indices. Two images, each with
	// 5 described keypoints of which two coincide (the removal *inside* the prefix that shrinks the boundary), plus 5 dense keypoints of
	// which one sits exactly on a described keypoint (described-wins), two coincide with each other (the higher warp confidence wins), and
	// one is a standalone dense point positioned *before* several described survivors (x=15, against described survivors at
	// 10/20/30/40) -- it must still land in the dense suffix, and its position must still order it against the other dense survivors on
	// its own, independently of the described segment. A stale boundary here would silently reclassify the surviving described keypoints
	// that shifted down into it as dense.
	{
		Scene filterScene;
		for (IIndex k = 0; k < 2; ++k)
			// described 0..4 (1 and 2 at the same position, 1 has the larger response*size); dense: 5 lands on the described keypoint 3, 6 and
			// 7 coincide (7 is the more confident), 8 is on its own, 9 is on its own and positioned before the described segment's tail
			AddKeypointImage(filterScene, {{10, 10, 0.05f}, {20, 20, 0.05f}, {20, 20, 0.01f}, {30, 30, 0.05f}, {40, 40, 0.05f}},
				{{30, 30, 0.9f}, {50, 50, 0.4f}, {50, 50, 0.95f}, {60, 60, 0.9f}, {15, 15, 0.7f}});
		// one match per keypoint index, so the remap of every keypoint is observable through it, and the partition is the one
		// AppendDenseMatches leaves behind: the five described matches are the sparse segment, the five dense ones the supplement (a dense
		// match inside the sparse count is the drift ImagePair::CheckSparseSegmentIsDescribed asserts against)
		AddKeypointPair(filterScene, 0, 1, {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8}, {9, 9}}, 5, 5);
		FilterRedundantKeypointsOf(filterScene);
		for (IIndex k = 0; k < 2; ++k) {
			const Image& im = filterScene.images[k];
			// 10 keypoints, 3 removed (the duplicated described one, the dense one on a described keypoint, the weaker of the two
			// coincident dense ones); the standalone dense point at x=15 is not a duplicate of anything and survives
			TEST_CHECK(im.keypoints.size() == 7 && im.NumDescribedKeypoints() == 4 && im.NumDenseKeypoints() == 3, "image %u kept %u keypoints with boundary %u, expected 7 and 4",
				(unsigned)k, (unsigned)im.keypoints.size(), im.NumDescribedKeypoints());
			// the described survivors are still the leading prefix, in position order; the dense survivors follow, in *their own* position
			// order -- x=15 sorts before x=50/x=60 among dense survivors even though it is less than every described survivor from index 1
			// on, because the two segments are compacted independently rather than merged into one global position order. The invariant is
			// checked directly at every index: below the boundary is never dense, at or above it always is.
			const float expectedX[7] = {10.f, 20.f, 30.f, 40.f, 15.f, 50.f, 60.f};
			for (uint32_t f = 0; f < 7; ++f)
				TEST_CHECK(im.keypoints[f].pt.x == expectedX[f] && im.IsDenseKeypoint(f) == (f >= 4), "image %u keypoint %u at x=%g, dense=%d",
					(unsigned)k, f, im.keypoints[f].pt.x, (int)im.IsDenseKeypoint(f));
			// described-wins: the dense keypoint that coincided with described keypoint 3 collapsed onto it, so what survives at x=30 is the described one
			TEST_CHECK(im.keypoints[2].response == 0.05f && im.keypoints[2].size == 4.f, "image %u lost a described keypoint to a dense one", (unsigned)k);
			// the standalone dense point survives unmodified, ahead of the other dense survivors
			TEST_CHECK(im.keypoints[4].response == 0.7f, "image %u lost the standalone dense keypoint at x=15 (%g)", (unsigned)k, im.keypoints[4].response);
			// between two dense points the more confident one survives
			TEST_CHECK(im.keypoints[5].response == 0.95f, "image %u kept the less confident dense keypoint (%g)", (unsigned)k, im.keypoints[5].response);
		}
	}

	// The match-level described-wins rule (NumDenseEnds in the duplicate-match filter) must actually decide something: two matches that
	// remap onto the same queryIdx, where one's trainIdx is described and the other's is dense. Image A carries two described keypoints at
	// the same position (one match to each), which the keypoint filter collapses onto a single survivor, so both matches end up sharing
	// that queryIdx; image B carries one described and one dense keypoint, untouched by the filter, so the two matches keep pointing at
	// different kinds of endpoint in B. On weight alone the dense endpoint would win -- ComputeKeypointWeight rates a dense point's high,
	// saturated confidence and large warp-cell size well above a modest SIFT response at a small size -- so if this case passes, the rule
	// is doing real work, not agreeing with what the weight comparison would have done anyway.
	{
		Scene weightScene;
		AddKeypointImage(weightScene, {{5, 5, 0.05f}, {5, 5, 0.01f}});
		AddKeypointImage(weightScene, {{50, 50, 0.02f}}, {{60, 60, 0.9f}}); // described, modest response/size; dense, high weight
		ImagePair& weightPair = AddKeypointPair(weightScene, 0, 1, {{0, 0}, {1, 1}}, 2); // A's first duplicate -> B's described, the second -> B's dense
		FilterRedundantKeypointsOf(weightScene);
		TEST_CHECK(weightPair.matches.size() == 1 && weightPair.matches[0].trainIdx == 0, "match-level described-wins rule did not keep the described endpoint (%u matches left, trainIdx %d)",
			(unsigned)weightPair.matches.size(), weightPair.matches.empty() ? -1 : weightPair.matches[0].trainIdx);
	}

	// The match-level partition must survive the duplicate-match filter (step 3). A supplemented pair carries three segments -- sparse
	// inliers | dense supplement | RANSAC inliers the strict filter rejected -- and step 3 runs on any pair whose images lost a keypoint. A
	// compaction that sorts `matches` moves matches ACROSS those bounds: dense keypoints hold the largest indices by construction, so every
	// dense match sorts to the end while the rejects migrate into the prefix, which makes part of the supplement inert in BuildTracks and
	// lets an equal number of deliberately rejected matches form tracks -- with nothing in any counter to show it. So this pair has both:
	// rejects present, and a described keypoint removed by the keypoint filter.
	{
		Scene partScene;
		for (IIndex k = 0; k < 2; ++k)
			// described 0..4, with 1 and 2 coincident (1 wins on response*size, so 2 is removed and every later index shifts down -- that
			// removal is what makes step 3 run at all); dense 5..7, none coincident with anything, so all three survive as themselves
			AddKeypointImage(partScene, {{10, 10, 0.05f}, {20, 20, 0.05f}, {20, 20, 0.01f}, {30, 30, 0.05f}, {40, 40, 0.05f}},
				{{100, 100, 0.6f}, {110, 110, 0.7f}, {120, 120, 0.8f}});
		// the sparse inliers (described keypoints the filter kept), the dense supplement where AppendDenseMatches puts it, and the rejects,
		// which must stay outside the prefix
		ImagePair& partPair = AddKeypointPair(partScene, 0, 1, {{0, 0}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}, {1, 1}, {2, 2}}, 3, 3);
		FilterRedundantKeypointsOf(partScene);
		const Image& pimg1 = partScene.images[0];
		const Image& pimg2 = partScene.images[1];
		// the two rejects remapped onto the same (queryIdx, trainIdx) = (1, 1) and one of them was dropped as a duplicate; both counts are
		// recomputed from the survivors of their own segment, so neither can absorb a removal that happened in the other
		TEST_CHECK(partPair.matches.size() == 7 && partPair.numFilteredInliers == 3 && partPair.numDenseInliers == 3 && partPair.GetNumFilteredInliers() == 3 &&
			partPair.GetNumTrackFormingMatches() == 6, "partition not recomputed per segment (%u matches, %d sparse, %d dense)",
			(unsigned)partPair.matches.size(), partPair.numFilteredInliers, partPair.numDenseInliers);
		const uint32_t expectedQuery[7] = {0, 2, 3, 4, 5, 6, 1};
		for (uint32_t m = 0; m < 7; ++m)
			TEST_CHECK(partPair.matches[m].queryIdx == expectedQuery[m] && partPair.matches[m].trainIdx == expectedQuery[m],
				"match %u is (%u, %u), expected (%u, %u) -- the compaction reordered the segments", m, partPair.matches[m].queryIdx, partPair.matches[m].trainIdx,
				expectedQuery[m], expectedQuery[m]);
		// every match below the track-forming boundary really is track-forming, and no reject entered it: the sparse segment is described at
		// both ends and never the reject's keypoint (index 1), the dense segment is dense at both ends, and the single reject sits past the boundary
		for (uint32_t m = 0; m < partPair.GetNumFilteredInliers(); ++m)
			TEST_CHECK(!pimg1.IsDenseKeypoint(partPair.matches[m].queryIdx) && !pimg2.IsDenseKeypoint(partPair.matches[m].trainIdx) && partPair.matches[m].queryIdx != 1,
				"sparse segment match %u is not a described non-reject match", m);
		for (uint32_t m = partPair.GetNumFilteredInliers(); m < partPair.GetNumTrackFormingMatches(); ++m)
			TEST_CHECK(pimg1.IsDenseKeypoint(partPair.matches[m].queryIdx) && pimg2.IsDenseKeypoint(partPair.matches[m].trainIdx), "dense segment match %u lost its dense endpoints", m);
		TEST_CHECK(partPair.matches[partPair.GetNumTrackFormingMatches()].queryIdx == 1, "the strict-filter reject is not the match past the track-forming boundary");
	}

	// The cross-segment prune can leave a two-hop survivor chain: a dense duplicate points at its intra-segment leader, and that leader is
	// then itself reused into a coincident described survivor. The remap fixup is a single dereference, so without a flattening pass a
	// chain resolves correctly only when the leader's index happens to be the smaller one. Three mutually coincident dense keypoints whose
	// leader is NOT the first of them, plus a described keypoint at the same position, produce exactly that: all three must remap onto the
	// described survivor and none onto an intermediate dense index. Each dense index is observed through its own pair, so the
	// duplicate-match filter cannot collapse the three observations into one.
	{
		Scene chainScene;
		// described, on its own, and described, the survivor of the cluster; three coincident dense keypoints, the middle index the most
		// confident, so the group's leader is index 3 and indices 2 and 4 are the two-hop followers
		Image& cimg = AddKeypointImage(chainScene, {{10, 10, 0.05f}, {50, 50, 0.05f}}, {{50, 50, 0.5f}, {50, 50, 0.95f}, {50, 50, 0.7f}});
		for (IIndex k = 1; k <= 3; ++k)
			AddKeypointImage(chainScene, {{70, 70, 0.05f}});
		for (IIndex k = 1; k <= 3; ++k)
			AddKeypointPair(chainScene, 0, k, {{1 + k, 0}}, 1); // dense keypoint 2, 3, 4 respectively
		FilterRedundantKeypointsOf(chainScene);
		// the whole dense cluster collapsed onto the described keypoint it coincides with
		TEST_CHECK(cimg.keypoints.size() == 2 && cimg.HasDenseKeypoints() && cimg.NumDescribedKeypoints() == 2 && cimg.NumDenseKeypoints() == 0,
			"coincident dense cluster left %u keypoints with boundary %u", (unsigned)cimg.keypoints.size(), cimg.NumDescribedKeypoints());
		for (IIndex k = 0; k < 3; ++k) {
			const ImagePair& p = chainScene.pairs[k];
			TEST_CHECK(p.matches.size() == 1 && p.matches[0].queryIdx == 1, "dense keypoint %u remapped to %d instead of the described survivor 1",
				(unsigned)k + 2, p.matches.empty() ? -1 : (int)p.matches[0].queryIdx);
		}
	}

	// The keypoint dedup comparator must be a strict weak ordering. "Within 0.1px" is NOT transitive, so a comparator that arbitrates
	// near-coincident pairs by response*size instead of by position can report a < b, b < c and c < a -- undefined behaviour in std::sort,
	// not merely an unspecified order. Each segment below is exactly that intransitive triple, at 0.08px spacing: a ~ b and b ~ c while
	// a !~ c, and the MIDDLE point is the most confident, so the duplicate-aware comparator's cycle was b < a (duplicates, response), b < c
	// (duplicates, response) and a < c (not duplicates, position) -- an order that sorts the triple b, a, c and lets the leader-based
	// grouping swallow all three into one run, collapsing a keypoint pair that is 0.16px apart. Pure position order has no cycle: the run
	// is the leader plus what is within 0.1px OF THE LEADER, so the triple splits into {a,b} keeping b, and {c}. Repeated to check the
	// outcome is the same on every run of the same input, which an order-dependent comparator cannot promise.
	for (unsigned rep = 0; rep < 4; ++rep) {
		Scene triScene;
		// described triple at y = 10, dense triple at y = 100, both with the middle one strongest
		Image& triImg = AddKeypointImage(triScene, {{0.00f, 10, 0.01f}, {0.08f, 10, 0.03f}, {0.16f, 10, 0.02f}}, {{0.00f, 100, 0.5f}, {0.08f, 100, 0.9f}, {0.16f, 100, 0.7f}});
		// the far image's keypoints are all distinct and carry identical response/size, so a duplicate-match group is an exact weight tie
		// and the stable order keeps the earlier one
		AddKeypointImage(triScene, {{1000, 1000, 0.05f}, {1010, 1000, 0.05f}, {1020, 1000, 0.05f}, {1030, 1000, 0.05f}, {1040, 1000, 0.05f}, {1050, 1000, 0.05f}}, {}, true);
		ImagePair& triPair = AddKeypointPair(triScene, 0, 1, {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}}, 3, 3);
		FilterRedundantKeypointsOf(triScene);
		// two survivors per segment: the run's most confident member and the point 0.16px out, which is a duplicate of neither survivor.
		// Under the intransitive comparator the whole triple sorted into a single run and only one survivor per segment came out.
		TEST_CHECK(triImg.keypoints.size() == 4 && triImg.NumDescribedKeypoints() == 2 && triImg.NumDenseKeypoints() == 2,
			"intransitive triple (rep %u) left %u keypoints with boundary %u, expected 4 and 2", rep, (unsigned)triImg.keypoints.size(), triImg.NumDescribedKeypoints());
		const float expectedX[4] = {0.08f, 0.16f, 0.08f, 0.16f};
		const float expectedY[4] = {10.f, 10.f, 100.f, 100.f};
		const float expectedResponse[4] = {0.03f, 0.02f, 0.9f, 0.7f};
		for (unsigned f = 0; f < 4; ++f)
			TEST_CHECK(ABS(triImg.keypoints[f].pt.x - expectedX[f]) <= 1e-5f && triImg.keypoints[f].pt.y == expectedY[f] && ABS(triImg.keypoints[f].response - expectedResponse[f]) <= 1e-5f,
				"intransitive triple (rep %u) keypoint %u is (%g, %g) response %g", rep, f, triImg.keypoints[f].pt.x, triImg.keypoints[f].pt.y, triImg.keypoints[f].response);
		// the partition follows the runs: in each segment the two matches that remapped onto the run's survivor collapse to the earlier
		// one, so both counts drop by exactly one
		TEST_CHECK(triPair.matches.size() == 4 && triPair.numFilteredInliers == 2 && triPair.numDenseInliers == 2,
			"intransitive triple (rep %u) partition is %u matches, %d sparse, %d dense", rep, (unsigned)triPair.matches.size(), triPair.numFilteredInliers, triPair.numDenseInliers);
		const uint32_t expectedTrain[4] = {0, 2, 3, 5};
		for (uint32_t m = 0; m < 4; ++m)
			TEST_CHECK(triPair.matches[m].queryIdx == m && triPair.matches[m].trainIdx == expectedTrain[m], "intransitive triple (rep %u) match %u is (%u, %u), expected (%u, %u)",
				rep, m, triPair.matches[m].queryIdx, triPair.matches[m].trainIdx, m, expectedTrain[m]);
	}

	// The duplicate-match filter's winner selection must know which segment a candidate sits in. Described-wins is right when the rival
	// forms tracks; when the rival is a match the strict cheirality/angle/reprojection filter deliberately REJECTED, keeping it costs a
	// track-forming observation and gains nothing, because the survivor then sits past the track-forming boundary and is never unioned.
	// Here a dense supplement match whose query endpoint was cross-pruned onto a described survivor (1 dense endpoint) shares its remapped
	// queryIdx with a reject described at both ends (0 dense endpoints), which on (NumDenseEnds, weight) alone would win.
	{
		Scene segScene;
		// A: described 0 is the reject's endpoint, 1 the sparse inlier's; dense, coincident with described 0, so the cross-segment prune
		// reuses that keypoint and the supplement match ends up sharing queryIdx with the reject
		AddKeypointImage(segScene, {{10, 10, 0.05f}, {50, 50, 0.05f}}, {{10, 10, 0.9f}});
		AddKeypointImage(segScene, {{100, 100, 0.05f}, {110, 110, 0.05f}}, {{200, 200, 0.9f}});
		// the sparse inlier, the dense supplement (query endpoint about to be pruned), the strict-filter reject
		ImagePair& segPair = AddKeypointPair(segScene, 0, 1, {{1, 0}, {2, 2}, {0, 1}}, 1, 1);
		FilterRedundantKeypointsOf(segScene);
		// the supplement match survives the group and stays track-forming; the reject is the one dropped, since it could never have formed a track
		TEST_CHECK(segPair.matches.size() == 2 && segPair.numFilteredInliers == 1 && segPair.numDenseInliers == 1 && segPair.matches[1].queryIdx == 0 && segPair.matches[1].trainIdx == 2,
			"the duplicate-match filter kept the strict-filter reject over the dense supplement (%u matches, %d sparse, %d dense, second match (%u, %u))",
			(unsigned)segPair.matches.size(), segPair.numFilteredInliers, segPair.numDenseInliers,
			segPair.matches.size() > 1 ? segPair.matches[1].queryIdx : 0u, segPair.matches.size() > 1 ? segPair.matches[1].trainIdx : 0u);
	}
	return scope.Pass("the boundary through a descriptor release and an .sfm round trip, the described-only filter order, the boundary through the remap, described-wins at the match level, the partition per segment, the two-hop chain, the strict weak ordering of the dedup, the segment-aware winner");
}
/*----------------------------------------------------------------*/

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
	TestScope scope("RoMa2PreprocessTest");
	Image8U3 bgr(cv::Size(64, 48), Pixel8U(30, 20, 10)); // 64x48, TPixel(r,g,b): r=30 g=20 b=10
	std::vector<float> planar;
	const int S = 64;
	PreprocessImageRoMa2(bgr, S, planar);
	TEST_CHECK(planar.size() == 3u*S*S, "unexpected output size %u (expected %u)", (unsigned)planar.size(), (unsigned)(3u*S*S));
	const float* R = planar.data();
	const float* G = R + S*S;
	const float* B = G + S*S;
	for (int i = 0; i < S*S; ++i)
		TEST_CHECK(ISEQUAL(R[i], 30.f/255.f, 1e-5f) && ISEQUAL(G[i], 20.f/255.f, 1e-5f) && ISEQUAL(B[i], 10.f/255.f, 1e-5f), "constant image, pixel %d", i);

	Image8U3 source;
	TEST_CHECK(source.Load(MAKE_PATH("roma2/preprocess_source.png")), "fixture missing");
	std::vector<float> expected;
	TEST_CHECK(ReadFloats(MAKE_PATH("roma2/preprocess_64.bin"), 3u*S*S, expected), "the preprocessing reference could not be read");
	PreprocessImageRoMa2(source, S, planar);
	const float maxErr = MaxAbsDiff(planar, expected);
	TEST_CHECK(maxErr <= 1e-5f, "max preprocessing error %g", maxErr);
	return scope.Pass("the constant image and the fixture image preprocessed, max error %g", maxErr);
	#endif
}

// RoMa2 and FlexGlue model files, none of which needs ONNX Runtime or a model path (plain JSON parsing and path rules):
// - the manifests: a RoMa2 manifest declaring format_version 1 (this build's schema) loads with its sizes and one declaring
//   format_version 3 (a schema this build no longer reads) is rejected; a FlexGlue manifest of format_version 1 naming its two graphs and
//   the H1 pose head loads with its sizes, and another format version, another model, a missing key and another pose head are each
//   rejected;
// - DenseMatchConfig::ResolveModelPath precedence, for each matcher (roma2: modelPath and $OPENMVS_ROMA2_MODEL_PATH, flexglue:
//   flexglueModelPath and $OPENMVS_FLEXGLUE_MODEL_PATH): the explicit path wins over everything (even a set environment variable); with
//   it empty, the matcher's environment variable wins; with both absent, the result falls back to the install prefix's own copy
//   (OPENMVS_MODEL_INSTALL_DIR/<matcher>) when that directory actually exists on disk, else it is empty. And a matcher never resolves to
//   the other one's explicit path or environment variable. This is the precedence the whole --dense-matcher pipeline depends on -- it
//   decides whether the CLI's own "needs a model" error fires -- so it is checked here directly rather than only indirectly through
//   CreateStructure. The fallback case must not create that directory, so it reads the same File::isFolder check the implementation
//   uses and asserts against whichever outcome is already true on this machine.
bool DenseModelFilesTest()
{
	TestScope scope("DenseModelFilesTest");
	const ScopedTempDir tmpDir(_T("DenseModelFilesTest"));
	TEST_CHECK(tmpDir.IsValid(), "the temp dir was not created");

	// the manifests, one table of (matcher, format_version, model, pose head, files, loads). A RoMa2 manifest carries every key
	// RoMa2Manifest::Load reads, at made-up but internally consistent sizes (S=32, cells=4, facetsDim=8): every declared io shape is
	// cross-checked against these, so a mismatch would fail the case for the wrong reason
	const auto WriteRoMa2 = [&tmpDir](const String& fileName, int formatVersion) -> String {
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
	const auto WriteFlexGlue = [&tmpDir](const String& fileName, int formatVersion, const char* model, const char* poseHead, bool bFiles) -> String {
		const String path = tmpDir(fileName);
		std::ofstream os(path.c_str());
		os << "{\"format_version\":" << formatVersion << ",\"model\":\"" << model << "\",\"setting\":\"base\",\"image_size\":512,\"warp_size\":128,"
		      "\"retrieval_recipes\":{\"features\":{\"dim\":256,\"gem_p\":3}},";
		if (bFiles)
			os << "\"files\":{\"descriptor\":\"d.onnx\",\"match\":\"m.onnx\"},";
		os << "\"heads\":{\"covisibility\":{\"output\":\"covisibility\"},\"pose\":{\"name\":\"" << poseHead << "\"}}}";
		return path;
	};
	const struct { const char* name; bool bFlexGlue; int version; const char* model; const char* head; bool bFiles; bool bLoads; } manifests[] = {
		{"roma_v1.json", false, 1, "", "", true, true}, {"roma_v3.json", false, 3, "", "", true, false},
		{"flexglue_ok.json", true, 1, "flexglue", "H1", true, true}, {"flexglue_v2.json", true, 2, "flexglue", "H1", true, false},
		{"flexglue_roma.json", true, 1, "roma2", "H1", true, false}, {"flexglue_nofiles.json", true, 1, "flexglue", "H1", false, false},
		{"flexglue_h0.json", true, 1, "flexglue", "H0", true, false}};
	for (const auto& m : manifests) {
		if (m.bFlexGlue) {
			FlexGlueManifest manifest;
			const bool bLoaded = manifest.Load(WriteFlexGlue(m.name, m.version, m.model, m.head, m.bFiles));
			TEST_CHECK(bLoaded == m.bLoads, "'%s' %s, expected %s", m.name, bLoaded ? "loaded" : "was rejected", m.bLoads ? "a load" : "a rejection");
			TEST_CHECK(!m.bLoads || (manifest.setting == "base" && manifest.imageSize == 512 && manifest.warpSize == 128 && manifest.retrievalDim == 256 &&
				manifest.descriptorFile == "d.onnx" && manifest.matchFile == "m.onnx"), "'%s' loaded with unexpected sizes or files", m.name);
		} else {
			RoMa2Manifest manifest;
			const bool bLoaded = manifest.Load(WriteRoMa2(m.name, m.version));
			TEST_CHECK(bLoaded == m.bLoads, "'%s' (format_version %d) %s, expected %s", m.name, m.version, bLoaded ? "loaded" : "was rejected", m.bLoads ? "a load" : "a rejection");
			TEST_CHECK(!m.bLoads || (manifest.imageSize == 32 && manifest.warpSize == 4 && manifest.facetsDim == 8), "'%s' loaded with unexpected sizes", m.name);
		}
	}

	// ResolveModelPath. Save/restore both variables around this part: the parity and reconstruct tests run right after it in the same
	// process and read them to decide whether to skip, so every exit path here must put the environment back exactly as it was found.
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
		TEST_CHECK(resolved == "/explicit/model/path", "%s: the explicit path did not win over the environment (got '%s')", matcher.c_str(), resolved.c_str());

		// explicit path empty, the matcher's variable set: the environment variable wins
		ownPath.clear();
		resolved = config.ResolveModelPath();
		TEST_CHECK(resolved == "/env/model/path", "%s: its environment variable did not win with the explicit path empty (got '%s')", matcher.c_str(), resolved.c_str());

		// both absent: the result depends on whether the install-prefix fallback directory actually exists on disk. Read that with the same
		// File::isFolder check the implementation uses rather than creating (or assuming the absence of) the directory, so this test is
		// hermetic on any machine, installed or not.
		ownEnv.Set(NULL);
		resolved = config.ResolveModelPath();
		#ifdef OPENMVS_MODEL_INSTALL_DIR
		const String installPath(String(OPENMVS_MODEL_INSTALL_DIR "/") + matcher);
		if (File::isFolder(installPath))
			TEST_CHECK(resolved == installPath, "%s: expected the install-prefix fallback '%s' since it exists on disk, got '%s'", matcher.c_str(), installPath.c_str(), resolved.c_str());
		else
		#endif
		TEST_CHECK(resolved.empty(), "%s: expected an empty result with the explicit path and the environment both absent and no install-prefix folder on disk, got '%s'", matcher.c_str(), resolved.c_str());
		otherEnv.Set(NULL);
	}
	return scope.Pass("the RoMa2 and FlexGlue manifests (the valid ones loaded with their sizes, the bad versions, model, keys and pose head rejected) and the model path precedence of both matchers");
}
/*----------------------------------------------------------------*/

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
static bool RoMa2OnnxParityDescribe(const TestScope& scope, RoMa2Onnx& model, const String& descDir, const String& setting)
{
	RoMa2ParityBounds bounds;
	TEST_CHECK(ReadParityBounds(descDir + "parity.json", bounds), "[%s] cannot read the parity bounds of '%s'", setting.c_str(), descDir.c_str());
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
	TEST_CHECK(sourceA.Load(descDir + "source_A.png"), "[%s] cannot read '%ssource_A.png'", setting.c_str(), descDir.c_str());
	std::vector<float> planarA, reference;
	TEST_CHECK(ReadNpyExpect(descDir + "in_image.npy", (size_t)3*S*S, reference), "[%s] cannot read in_image.npy", setting.c_str());
	PreprocessImageRoMa2(sourceA, S, planarA);
	const float diffImage = MaxAbsDiff(planarA, reference);
	TEST_CHECK(diffImage <= 1e-4f, "[%s] preprocessing differs from the reference by %g, expected at most 1e-4", setting.c_str(), diffImage);

	// the raw value facets, read back from the device, and the graph's own on-device retrieval pooling
	OrtTensor layers(model.MakeLayers());
	TEST_CHECK(layers.IsValid(), "[%s] cannot allocate the layers tensor", setting.c_str());
	std::vector<float> facets, retrieval;
	// this is the same call the describe pass in MatchDense.cpp makes (facetsOut NULL there):
	// it exercises RoMa2Onnx::Describe's retrievalOut readback, which otherwise has no automated coverage
	TEST_CHECK(model.Describe(planarA.data(), layers, &facets, retrieval), "[%s] describe with the facets read-back failed", setting.c_str());
	TEST_CHECK(facets.size() == numTensor, "[%s] %u facet values, expected %u", setting.c_str(), (unsigned)facets.size(), (unsigned)numTensor);
	TEST_CHECK(ReadNpyExpect(descDir + "out_value_facets.npy", numTensor, reference), "[%s] cannot read out_value_facets.npy", setting.c_str());
	const double cosFacets = CosineSimilarity(facets.data(), reference.data(), numTensor);

	// the graph's own on-device retrieval pooling, judged against the CPU pool_retrieval reference's
	// pooled_facets_A.npy fixture at the tighter bar (export.py check's --retrieval-min-cosine
	// default) rather than the looser bounds.minCosine below -- this parity gate exists because
	// this readback path had no automated coverage otherwise; it must keep running now that the CPU
	// pooling it used to also be judged against (PoolRetrievalDescriptor) is gone
	TEST_CHECK(retrieval.size() == manifest.facetsDim, "[%s] the graph's retrieval output has %u values, the manifest declares %u",
		setting.c_str(), (unsigned)retrieval.size(), manifest.facetsDim);
	TEST_CHECK(ReadNpyExpect(descDir + "pooled_facets_A.npy", manifest.facetsDim, reference), "[%s] cannot read pooled_facets_A.npy", setting.c_str());
	const double cosRetrieval = CosineSimilarity(retrieval.data(), reference.data(), retrieval.size());
	TEST_CHECK(cosRetrieval >= 0.99999, "[%s] on-device retrieval cosine %.8f is below 0.99999 (vs pooled_facets_A.npy)", setting.c_str(), cosRetrieval);

	// the same image described again into a host tensor, so the `layers` output itself -- the
	// device tensor above is never read back -- can be compared too; the retrieval readback is
	// already checked above, so this call's copy of it is discarded, not reused
	OrtTensor layersHost(OrtTensor::Host(model.LayersShape()));
	std::vector<float> discardedRetrieval;
	TEST_CHECK(model.Describe(planarA.data(), layersHost, NULL, discardedRetrieval), "[%s] describe into a host tensor failed", setting.c_str());
	TEST_CHECK(ReadNpyExpect(descDir + "out_layers.npy", numTensor, reference), "[%s] cannot read out_layers.npy", setting.c_str());
	const double cosLayers = CosineSimilarity(layersHost.HostData(), reference.data(), numTensor);
	TEST_CHECK(cosFacets >= bounds.minCosine && cosLayers >= bounds.minCosine, "[%s] describe cosine below %g (value_facets %.6f, layers %.6f)",
		setting.c_str(), bounds.minCosine, cosFacets, cosLayers);
	VERBOSE("RoMa2OnnxParityTest[%s] describe passed on %s: cosine %.6f (value_facets) / %.6f (layers), retrieval %.8f",
		setting.c_str(), model.ProviderName().c_str(), cosFacets, cosLayers, cosRetrieval);
	return true;
}

// polyml's check_correspondences in C++, for one match direction: the warp error in pixels of the
// SxS grid over the cells the reference calls overlapping, and the agreement of the overlap
// logit's sign. Shared by both directions of RoMa2OnnxParityMatchCoarse below, which compares a
// computed (warp, confidence) pair to a same-shaped reference dump twice, once per direction.
static bool RoMa2OnnxCheckDirection(const TestScope& scope, const Image32F2& warp, const Image32F& confidence,
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
	TEST_CHECK(!errors.empty(), "[%s] the reference overlap gates every %s warp cell", setting.c_str(), direction);
	outP99 = Percentile(errors, 99);
	outAgreement = 100. * numAgree / (double)(cells*cells);
	TEST_CHECK(outP99 <= bounds.maxWarpErrorPx && outAgreement >= bounds.minAgreementPercent,
		"[%s] %s warp error p99 %.4f px (bound %g), logit sign agreement %.4f%% (bound %g)",
		setting.c_str(), direction, outP99, bounds.maxWarpErrorPx, outAgreement, bounds.minAgreementPercent);
	return true;
}

// The coarse-match stage of one preset against real_<setting>_match_coarse.reference, driven end
// to end: both sources preprocessed, described, and matched, then judged as polyml's
// check_correspondences does, by the bounds in that dump's own parity.json, on both directions
// of the bidirectional graph (warp/confidence A->B, warp_BA/confidence_BA B->A)
static bool RoMa2OnnxParityMatchCoarse(const TestScope& scope, RoMa2Onnx& model, const String& matchDir, const String& setting)
{
	RoMa2ParityBounds bounds;
	TEST_CHECK(ReadParityBounds(matchDir + "parity.json", bounds), "[%s] cannot read the parity bounds of '%s'", setting.c_str(), matchDir.c_str());
	const int S = model.ImageSize(), cells = model.WarpSize();
	Image8U3 sourceA, sourceB;
	TEST_CHECK(sourceA.Load(matchDir + "source_A.png") && sourceB.Load(matchDir + "source_B.png"), "[%s] cannot read the match sources from '%s'", setting.c_str(), matchDir.c_str());
	std::vector<float> planarA, planarB;
	PreprocessImageRoMa2(sourceA, S, planarA);
	PreprocessImageRoMa2(sourceB, S, planarB);
	OrtTensor layersA(model.MakeLayers()), layersB(model.MakeLayers());
	TEST_CHECK(layersA.IsValid() && layersB.IsValid(), "[%s] cannot allocate the pair descriptor tensors", setting.c_str());
	std::vector<float> discardedRetrieval; // this stage matches, it never needs a retrieval descriptor
	TEST_CHECK(model.Describe(planarA.data(), layersA, NULL, discardedRetrieval) && model.Describe(planarB.data(), layersB, NULL, discardedRetrieval),
		"[%s] describe of the match pair failed", setting.c_str());
	Image32F2 warpAB, warpBA;
	Image32F confidenceAB, confidenceBA;
	TEST_CHECK(model.MatchCoarse(layersA, layersB, warpAB, confidenceAB, warpBA, confidenceBA), "[%s] coarse match failed", setting.c_str());
	TEST_CHECK(warpAB.cols == cells && warpAB.rows == cells && confidenceAB.cols == cells && confidenceAB.rows == cells &&
		warpBA.cols == cells && warpBA.rows == cells && confidenceBA.cols == cells && confidenceBA.rows == cells,
		"[%s] warp %dx%d/%dx%d, expected %dx%d", setting.c_str(), warpAB.cols, warpAB.rows, warpBA.cols, warpBA.rows, cells, cells);
	std::vector<float> refWarpAB, refConfidenceAB, refWarpBA, refConfidenceBA;
	TEST_CHECK(ReadNpyExpect(matchDir + "out_warp.npy", (size_t)cells*cells*2, refWarpAB) &&
		ReadNpyExpect(matchDir + "out_confidence.npy", (size_t)cells*cells, refConfidenceAB) &&
		ReadNpyExpect(matchDir + "out_warp_BA.npy", (size_t)cells*cells*2, refWarpBA) &&
		ReadNpyExpect(matchDir + "out_confidence_BA.npy", (size_t)cells*cells, refConfidenceBA), "[%s] cannot read the match reference dumps", setting.c_str());
	float p99AB = 0, p99BA = 0;
	double agreementAB = 0, agreementBA = 0;
	if (!RoMa2OnnxCheckDirection(scope, warpAB, confidenceAB, refWarpAB, refConfidenceAB, S, cells, bounds, setting, "A->B", p99AB, agreementAB) ||
		!RoMa2OnnxCheckDirection(scope, warpBA, confidenceBA, refWarpBA, refConfidenceBA, S, cells, bounds, setting, "B->A", p99BA, agreementBA))
		return false;
	VERBOSE("RoMa2OnnxParityTest[%s] match passed on %s: A->B warp p99 %.4f px agreement %.4f%%, B->A warp p99 %.4f px agreement %.4f%%",
		setting.c_str(), model.ProviderName().c_str(), p99AB, agreementAB, p99BA, agreementBA);
	return true;
}

// One preset of RoMa2OnnxParityTest: load the model, then run both stages against their own dumps
static bool RoMa2OnnxParitySetting(const TestScope& scope, const String& modelDir, const String& setting, const String& provider)
{
	RoMa2Onnx model;
	TEST_CHECK(model.Load(modelDir, setting, provider), "[%s] cannot load the model from '%s'", setting.c_str(), modelDir.c_str());
	DEBUG_EXTRA("RoMa2OnnxParityTest[%s]: %dx%d image, %u slices of %u patches x %u channels, %dx%d warp cells, provider %s",
		setting.c_str(), model.ImageSize(), model.ImageSize(), (unsigned)model.LayersShape()[1],
		model.NumPatches(), (unsigned)model.LayersShape()[4], model.WarpSize(), model.WarpSize(),
		model.ProviderName().c_str());
	return RoMa2OnnxParityDescribe(scope, model, RoMa2ReferenceDir(modelDir, setting, "descriptor"), setting) &&
		RoMa2OnnxParityMatchCoarse(scope, model, RoMa2ReferenceDir(modelDir, setting, "match_coarse"), setting);
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
	TestScope scope("RoMa2OnnxParityTest");
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
		// which the other ROMA2 tests are perfectly happy with, so skip loudly instead of
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
	TEST_CHECK(!unloaded.Load(modelDir, "no-such-setting", provider) && !unloaded.IsLoaded() && !unloaded.MakeLayers().IsValid(), "a failed Load() did not leave the model unloaded");
	for (const String& setting : settings)
		if (!RoMa2OnnxParitySetting(scope, modelDir, setting, provider))
			return false;
	return scope.Pass("%u preset(s) match the reference dumps", (unsigned)settings.size());
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

// One matching run of DenseMatcherReconstructTest: the bundled images and their AKAZE features (BundledImagesFeatures) at the
// images' own focal, given and so trusted (720.51 px, no distortion), then MatchPairs in RETRIEVAL mode with the dense matcher
// of denseCfg attached, the describe pass and (useMatching) the one pass in one model load. Unlike RealImagesPipelineTest, which
// forces a wrong 900 px / k1 0.6 / k2 -0.09 and leans on the view-graph calibrator to recover them from the fundamental
// matrices, this run cannot feed that recovery: warp-guided matching keeps, per keypoint of A, the descriptor-best keypoint of
// B inside a disc around the position the warp predicts, so the stored correspondences fit a whole family of F near the warp's
// own geometry and the focal a calibrator extracts from them is wildly unstable (measured at turbo with the forced 900 px: 690
// px from the descriptor matches, 133 px from the guided ones on the CPU provider, a rejected 34450 px estimate on CUDA). With
// the intrinsics trusted the pairs carry a relative pose straight from matching, which the reconstruction stage consumes. The
// focal is given, not left to the import: the images carry only a 35 mm-equivalent focal (720.51 px on the 640x479 frame),
// which the import takes as an untrusted guess for the lens fit, and four images give the fit too few pairs to trust it, so no
// pair would carry a relative pose. The view-graph calibration is off: it re-solves a focal per camera and re-filters every
// pair's inliers through it, one more moving part between two runs compared pair by pair.
// Checks: the GLOBAL_DESCRIPTORS state; every image's global descriptor 1 x expectedDim and unit; no two images' descriptors
// near-identical (cosine under 1-1e-6: the four close-up shots measure up to 0.99994838 by content alone, a descriptor
// described twice from one reused prefetch buffer comes out within float rounding of 1); every candidate pair stored (a
// covisibility gate may drop some: then at least one and no more) and geometrically verified on minMatches of evidence (the
// track-forming set when the one pass ran, its dense fill counting). Hands the stored pairs back as summaries.
// staleDescriptors: the scene first carries global descriptors of another matcher (2048-D), which MatchPairs must replace.
static bool DenseReconstructScene(const TestScope& scope, Scene& scene, DenseMatchConfig denseCfg, int expectedDim,
	DensePairSummaries& pairSummaries, bool staleDescriptors = false)
{
	ImportConfig importCfg;
	importCfg.focalLength = 720.51f;
	TEST_CHECK(BundledImagesFeatures(scene, importCfg), "the bundled images were not imported");
	const IIndex nImages = (IIndex)scene.images.size();
	const size_t expectedPairs = (size_t)nImages*(nImages-1)/2;
	if (staleDescriptors) {
		FOREACH(i, scene.images) {
			scene.images[i].globalDescriptor = cv::Mat::zeros(1, 2048, CV_32F);
			scene.images[i].globalDescriptor.at<float>(0, i) = 1.f;
		}
		scene.status.nState.set(Scene::Status::STATE::GLOBAL_DESCRIPTORS);
	}
	MatchConfig matchCfg = BundledImagesConfig().matchCfg;
	matchCfg.mode = MatchConfig::RETRIEVAL; // the describe pass and the dense pass, one model load; on four images every pair
	matchCfg.viewGraphCalibrationEnabled = false;
	denseCfg.enabled = true;
	TEST_CHECK(scene.MatchPairs(matchCfg, denseCfg, ViewGraphCalibratorConfig()), "%s: MatchPairs failed", denseCfg.matcher.c_str());
	TEST_CHECK(scene.status.nState.isSet(Scene::Status::STATE::GLOBAL_DESCRIPTORS), "%s: GLOBAL_DESCRIPTORS state not set", denseCfg.matcher.c_str());
	double maxCosine = 0.0;
	FOREACH(i, scene.images) {
		const cv::Mat& desc = scene.images[i].globalDescriptor;
		TEST_CHECK(desc.rows == 1 && desc.cols == expectedDim && ABS(cv::norm(desc, cv::NORM_L2) - 1.0) <= 1e-4,
			"%s: image %u's global descriptor is %dx%d of norm %.6f, expected 1x%d of norm 1", denseCfg.matcher.c_str(), (unsigned)i,
			desc.rows, desc.cols, desc.empty() ? 0.0 : cv::norm(desc, cv::NORM_L2), expectedDim);
		for (IIndex j = 0; j < i; ++j) {
			const double cosine = desc.dot(scene.images[j].globalDescriptor);
			maxCosine = MAXF(maxCosine, cosine);
			TEST_CHECK(cosine < 1.0 - 1e-6, "%s: images %u and %u have near-identical global descriptors (cosine %.8f)",
				denseCfg.matcher.c_str(), j, (unsigned)i, cosine);
		}
	}
	const bool gated = denseCfg.matcher == "flexglue" && denseCfg.minCovisibility > 0.f;
	TEST_CHECK(gated ? !scene.pairs.empty() && scene.pairs.size() <= expectedPairs : scene.pairs.size() == expectedPairs,
		"%s: %u pairs matched, %u candidates", denseCfg.matcher.c_str(), (unsigned)scene.pairs.size(), (unsigned)expectedPairs);
	for (const ImagePair& pair : scene.pairs) {
		const unsigned numEvidence = denseCfg.useMatching ? pair.GetNumTrackFormingMatches() : pair.GetNumFilteredInliers();
		TEST_CHECK(pair.HasGeometricVerification() && numEvidence >= matchCfg.minMatches, "%s: pair (%u, %u) not geometrically "
			"verified (%u sparse + %u dense, %s geometry)", denseCfg.matcher.c_str(), pair.ID1, pair.ID2, pair.GetNumFilteredInliers(),
			pair.GetNumDenseInliers(), pair.HasGeometricVerification() ? "with" : "without");
	}
	pairSummaries = SummarizePairs(scene);
	DEBUG("DenseMatcherReconstructTest[%s %s]: %u images described (%d-D, %s provider, %u threads, max pairwise cosine %.8f), %u pairs "
		"matched (one pass %s, %u slots)", denseCfg.matcher.c_str(), denseCfg.setting.c_str(), nImages, expectedDim,
		denseCfg.provider.c_str(), scene.nMaxThreads, maxCosine, (unsigned)scene.pairs.size(), denseCfg.useMatching ? "on" : "off",
		denseCfg.slotBudget);
	return true;
}

// The RoMa2 half of DenseMatcherReconstructTest, on the model at modelPath (OPENMVS_ROMA2_SETTING picks the preset, default
// "base"; OPENMVS_ROMA2_PROVIDER the provider, default "auto"):
// 1. a baseline run with the one pass off (the descriptor matching) against a run with it on: the same pair set, a dense segment
//    on every pair and a sparse segment of another size than the descriptor batch's on most (the guided matching tests each
//    keypoint of A against the best one OUTSIDE its disc, a different and stricter selection, so no more inliers are asked for);
// 2. the dense run survives a Scene::Save/Load round trip (global descriptors bit for bit, the same pairs), and the
//    reconstruction stage of RealImagesPipelineTest passes on it: 5000-12000 tracks (the fill hands the track builder several
//    times the ~2.2k of the descriptor-only run, measured ~7.7k at turbo on CUDA under the flat cap the density rule replaced, so
//    the window is loose at its top; a model, preset or budget change moves it: re-measure) and up to 20 px of residual
//    distortion (the warp's bias absorbed by the bundle into a scene that has none, measured 10.1 px; the focal given at import
//    already meets the 100 px bound, so here it checks the focal stays, not that it is recovered);
// 3. determinism (design decision 11): two runs on one thread with a slot pool of 2, so the describe pass reuses a prefetch
//    buffer (2 buffers for 4 images) and the one pass reloads evicted slots (8 loads for 6 pairs), store the very same pairs,
//    sparse and dense segments alike. Both start from the same cached features: what is compared is the matching.
static bool RoMa2Reconstruct(const TestScope& scope, const String& modelPath)
{
	const char* const envSetting = getenv("OPENMVS_ROMA2_SETTING");
	const char* const envProvider = getenv("OPENMVS_ROMA2_PROVIDER");
	const auto Config = [&](bool useMatching, unsigned slotBudget) {
		DenseMatchConfig cfg;
		cfg.modelPath = modelPath;
		cfg.setting = envSetting != NULL && *envSetting != 0 ? String(envSetting) : String("base");
		cfg.provider = envProvider != NULL && *envProvider != 0 ? String(envProvider) : String("auto");
		cfg.useMatching = useMatching;
		cfg.slotBudget = slotBudget;
		return cfg;
	};
	DensePairSummaries baseline, guided;
	{
		Scene scene(2);
		if (!DenseReconstructScene(scope, scene, Config(false, 64), 2048, baseline))
			return false;
	}
	Scene scene(2);
	if (!DenseReconstructScene(scope, scene, Config(true, 64), 2048, guided))
		return false;
	TEST_CHECK(baseline.size() == guided.size(), "roma2: %u pairs through the one pass, %u through descriptor matching",
		(unsigned)guided.size(), (unsigned)baseline.size());
	unsigned numDenseFilled = 0, numSparseDiffers = 0;
	FOREACH(i, guided) {
		TEST_CHECK(guided[i].ID1 == baseline[i].ID1 && guided[i].ID2 == baseline[i].ID2, "roma2: the one pass stored a different pair set");
		if (guided[i].numDenseInliers > 0)
			++numDenseFilled;
		if (guided[i].numFilteredInliers != baseline[i].numFilteredInliers)
			++numSparseDiffers;
	}
	TEST_CHECK(numDenseFilled == guided.size() && 2*numSparseDiffers > guided.size(), "roma2: %u of the %u stored pairs carry a dense "
		"segment and %u a sparse segment of another size than the descriptor batch's, expected all and most", numDenseFilled,
		(unsigned)guided.size(), numSparseDiffers);
	{
		const ScopedTempDir tmpDir(_T("DenseMatcherReconstructTest"));
		TEST_CHECK(tmpDir.IsValid(), "no temporary folder");
		const String sfmPath(tmpDir(_T("dense_matched.sfm")));
		Scene loaded(2);
		TEST_CHECK(scene.Save(sfmPath) && loaded.Load(sfmPath), "roma2: cannot save and load back '%s'", sfmPath.c_str());
		TEST_CHECK(loaded.images.size() == scene.images.size() && loaded.status.nState.isSet(Scene::Status::STATE::GLOBAL_DESCRIPTORS),
			"roma2: the round trip lost the images or the GLOBAL_DESCRIPTORS state");
		FOREACH(i, loaded.images) {
			const cv::Mat& saved = scene.images[i].globalDescriptor;
			const cv::Mat& read = loaded.images[i].globalDescriptor;
			TEST_CHECK(read.size() == saved.size() && read.type() == saved.type() && cv::norm(read, saved, cv::NORM_INF) == 0.0,
				"roma2: the round trip lost image %u's global descriptor", (unsigned)i);
		}
		TEST_CHECK(SummarizePairs(loaded) == guided, "roma2: the round trip changed the matched pairs");
	}
	TEST_CHECK(ReconstructMatchedScene(scene, "DenseMatcherReconstructTest[roma2]", 5000, 12000, 20), "roma2: the reconstruction stage");
	DensePairSummaries tightPool, repeated;
	for (DensePairSummaries* summaries : {&tightPool, &repeated}) {
		Scene scene(1);
		if (!DenseReconstructScene(scope, scene, Config(true, 2), 2048, *summaries))
			return false;
	}
	if (repeated != tightPool) {
		FOREACH(i, repeated)
			if (i >= tightPool.size() || !(repeated[i] == tightPool[i]))
				VERBOSE("  pair (% 4u, % 4u): %u matches / %u sparse / %u dense, was (% 4u, % 4u): %u matches / %u sparse / %u dense",
					repeated[i].ID1, repeated[i].ID2, repeated[i].numMatches, repeated[i].numFilteredInliers, repeated[i].numDenseInliers,
					i < tightPool.size() ? tightPool[i].ID1 : NO_ID, i < tightPool.size() ? tightPool[i].ID2 : NO_ID,
					i < tightPool.size() ? tightPool[i].numMatches : 0, i < tightPool.size() ? tightPool[i].numFilteredInliers : 0,
					i < tightPool.size() ? tightPool[i].numDenseInliers : 0);
		TEST_CHECK(false, "roma2: two runs of the same configuration matched different pairs (listed above)");
	}
	return true;
}

// The FlexGlue half of DenseMatcherReconstructTest, on the bundle at modelPath (docs/design/DenseMatching.md), with no RoMa2 model
// reachable (its explicit path names a folder that does not exist and OPENMVS_ROMA2_MODEL_PATH is unset for the duration): the
// describe pass, the retrieval and the matching load the FlexGlue bundle alone, on scenes carrying stale 2048-D descriptors.
// 1. every pair kept: each carries a covisibility score in [0,1], a pose prior that is a rotation and a unit direction, close to
//    the pose its own matches give (12 deg in rotation, 30 in direction: a convention error of the pose head, a transposed
//    rotation, the baseline's sign or frame, is far from that on pairs of 5-14 deg), and the pairs CSV carries both (25 fields);
// 2. the covisibility gate at the median score keeps the pairs clearly above it and drops those clearly below (a second GPU run
//    moves a score by about 1e-5);
// 3. the reconstruction stage of RealImagesPipelineTest on the ungated scene (4000-12000 tracks, 20 px of distortion).
static bool FlexGlueReconstruct(const TestScope& scope, const String& modelPath)
{
	const char* const envProvider = getenv("OPENMVS_ROMA2_PROVIDER");
	DenseMatchConfig cfg;
	cfg.matcher = "flexglue";
	cfg.flexglueModelPath = modelPath;
	cfg.provider = envProvider != NULL && *envProvider != 0 ? String(envProvider) : String("auto");
	cfg.useMatching = true;
	cfg.slotBudget = 64;
	cfg.minCovisibility = 0.f;
	cfg.modelPath = "/nonexistent/roma2-model";
	const char* const envRoMa2 = getenv("OPENMVS_ROMA2_MODEL_PATH");
	const String prevRoMa2Path(envRoMa2 != NULL ? envRoMa2 : "");
	const bool hadRoMa2Path = envRoMa2 != NULL;
	#ifdef _MSC_VER
	_putenv_s("OPENMVS_ROMA2_MODEL_PATH", "");
	#else
	unsetenv("OPENMVS_ROMA2_MODEL_PATH");
	#endif
	DensePairSummaries summaries;
	Scene scene(2);
	const bool matched = DenseReconstructScene(scope, scene, cfg, FlexGlueMatcher::RETRIEVAL_SIZE, summaries, true);
	if (hadRoMa2Path) {
		#ifdef _MSC_VER
		_putenv_s("OPENMVS_ROMA2_MODEL_PATH", prevRoMa2Path.c_str());
		#else
		setenv("OPENMVS_ROMA2_MODEL_PATH", prevRoMa2Path.c_str(), 1);
		#endif
	}
	if (!matched)
		return false;
	std::vector<float> scores;
	for (const ImagePair& pair : scene.pairs) {
		TEST_CHECK(pair.covisibility && *pair.covisibility >= 0.f && *pair.covisibility <= 1.f && pair.posePrior && pair.relativePose,
			"flexglue: pair (%u, %u) lacks a covisibility score in [0,1], a pose prior or its own relative pose", pair.ID1, pair.ID2);
		scores.push_back(*pair.covisibility);
		const PosePrior& prior = *pair.posePrior;
		const double orthonormality = cv::norm(Matrix3x3f(prior.R.t() * prior.R) - Matrix3x3f::IDENTITY), tNorm = cv::norm(prior.t);
		TEST_CHECK(orthonormality <= 1e-4 && ABS(cv::determinant(prior.R) - 1.0) <= 1e-4 && ABS(tNorm - 1.0) <= 1e-4, "flexglue: pair "
			"(%u, %u)'s pose prior is not a rotation and a unit direction (orthonormality %g, |t| %g)", pair.ID1, pair.ID2, orthonormality, tNorm);
		// both in the pair's convention x2 = R*x1 + s*t
		const double rotationError = RotationErrorDeg(RMatrix(pair.relativePose->R), RMatrix(Matrix3x3(prior.R)));
		const CMatrix t(pair.relativePose->GetT());
		const double directionError = R2D(acos(CLAMP((t.x*prior.t.x + t.y*prior.t.y + t.z*prior.t.z) / cv::norm(t), -1.0, 1.0)));
		DEBUG_EXTRA("DenseMatcherReconstructTest[flexglue]: pair (%u, %u) covisibility %.4f, prior %.2f deg in rotation and %.2f deg in "
			"direction from the pair's own pose", pair.ID1, pair.ID2, *pair.covisibility, rotationError, directionError);
		TEST_CHECK(rotationError <= 12. && directionError <= 30., "flexglue: pair (%u, %u)'s pose prior is %.1f deg in rotation and "
			"%.1f deg in direction from the pair's own pose", pair.ID1, pair.ID2, rotationError, directionError);
	}
	{
		const ScopedTempDir tmpDir(_T("DenseMatcherReconstructTest"));
		TEST_CHECK(tmpDir.IsValid(), "no temporary folder");
		const String csvPath(tmpDir(_T("pairs.csv")));
		TEST_CHECK(PairsMatcher::ExportPairsCSV(scene, csvPath, 0.f, 0.f, PairsWeightingConfig()), "flexglue: cannot export the pairs CSV");
		std::ifstream ifs(csvPath);
		std::string header, row;
		std::getline(ifs, header);
		const auto NumFields = [](const std::string& line) { return (size_t)std::count(line.begin(), line.end(), ',') + 1; };
		TEST_CHECK(header.find(",Covisibility,PriorQw,PriorQx,PriorQy,PriorQz,PriorTx,PriorTy,PriorTz") != std::string::npos &&
			NumFields(header) == 25, "flexglue: the pairs CSV header lacks the covisibility and pose prior columns: %s", header.c_str());
		size_t numRows = 0;
		for (; std::getline(ifs, row); ++numRows)
			TEST_CHECK(NumFields(row) == 25, "flexglue: a pairs CSV row has %u fields, expected 25", (unsigned)NumFields(row));
		TEST_CHECK(numRows == scene.pairs.size(), "flexglue: the pairs CSV has %u rows for %u pairs", (unsigned)numRows, (unsigned)scene.pairs.size());
	}
	std::vector<float> sorted(scores);
	std::sort(sorted.begin(), sorted.end());
	const float bar = 0.5f * (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]);
	TEST_CHECK(sorted.front() < bar && bar < sorted.back(), "flexglue: the scores do not spread (%g .. %g), the gate has nothing to separate",
		sorted.front(), sorted.back());
	{
		Scene gated(2);
		DenseMatchConfig gateCfg(cfg);
		gateCfg.minCovisibility = bar;
		DensePairSummaries gatedSummaries;
		if (!DenseReconstructScene(scope, gated, gateCfg, FlexGlueMatcher::RETRIEVAL_SIZE, gatedSummaries))
			return false;
		TEST_CHECK(!gated.pairs.empty() && gated.pairs.size() < scene.pairs.size(), "flexglue: the gate at %g kept %u of %u pairs", bar,
			(unsigned)gated.pairs.size(), (unsigned)scene.pairs.size());
		constexpr float noise = 2e-5f;
		for (const ImagePair& pair : scene.pairs) {
			const bool kept = std::any_of(gated.pairs.begin(), gated.pairs.end(), [&](const ImagePair& g) { return g.ID1 == pair.ID1 && g.ID2 == pair.ID2; });
			TEST_CHECK(!(*pair.covisibility > bar + noise && !kept) && !(*pair.covisibility < bar - noise && kept), "flexglue: with the "
				"gate at %g pair (%u, %u) of covisibility %g was %s", bar, pair.ID1, pair.ID2, *pair.covisibility, kept ? "kept" : "dropped");
		}
		for (const ImagePair& pair : gated.pairs)
			TEST_CHECK(pair.covisibility && *pair.covisibility >= bar, "flexglue: the gate at %g kept pair (%u, %u) of covisibility %g", bar,
				pair.ID1, pair.ID2, pair.covisibility ? *pair.covisibility : -1.f);
	}
	TEST_CHECK(ReconstructMatchedScene(scene, "DenseMatcherReconstructTest[flexglue]", 4000, 12000, 20), "flexglue: the reconstruction stage");
	return true;
}
#endif // _USE_ONNXRUNTIME

// The dense matchers in the whole import/AKAZE/RETRIEVAL pipeline on the bundled images, each half run when its model is given
// (OPENMVS_ROMA2_MODEL_PATH, OPENMVS_FLEXGLUE_MODEL_PATH) and skipped otherwise: the shared checks of every matching run
// (DenseReconstructScene: the describe pass's retrieval descriptors, every pair verified) and each matcher's own (RoMa2Reconstruct:
// the one pass against the descriptor batch, the round trip, determinism; FlexGlueReconstruct: covisibility, pose prior, CSV,
// the gate), each finished with RealImagesPipelineTest's reconstruction stage
bool DenseMatcherReconstructTest()
{
	#ifndef _USE_ONNXRUNTIME
	VERBOSE("DenseMatcherReconstructTest: skipped (built without ONNX Runtime)");
	return true;
	#else
	if (!RoMa2Onnx::IsAvailable()) {
		VERBOSE("DenseMatcherReconstructTest: skipped (no ONNX Runtime support in this build)");
		return true;
	}
	TestScope scope("DenseMatcherReconstructTest");
	String ran;
	const auto Half = [&](const char* matcher, const char* variable, bool (*Run)(const TestScope&, const String&)) {
		const char* const path = getenv(variable);
		if (path == NULL || *path == 0) {
			ran += String::FormatString("%s%s skipped (%s not set)", ran.empty() ? "" : ", ", matcher, variable);
			return true;
		}
		ran += String::FormatString("%s%s", ran.empty() ? "" : ", ", matcher);
		return Run(scope, String(path));
	};
	if (!Half("roma2", "OPENMVS_ROMA2_MODEL_PATH", RoMa2Reconstruct) || !Half("flexglue", "OPENMVS_FLEXGLUE_MODEL_PATH", FlexGlueReconstruct))
		return false;
	return scope.Pass("%s", ran.c_str());
	#endif
}

} // namespace SFM
