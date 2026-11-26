/*
 * TestsSFMFramework.h
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

#ifndef _TESTS_SFM_FRAMEWORK_H_
#define _TESTS_SFM_FRAMEWORK_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../../libs/SFM.h"
#include "../../libs/SFM/SceneBlocks.h"
#include "../../libs/SFM/GlobalAlignment.h"
#include <random>


// S T R U C T S ///////////////////////////////////////////////////

// The building blocks the SfM tests share: how a test reports, and the scenes they build.
namespace SFM {

// ===============================================================================
// The frame of one test. It names the test in every failure, times it, and prints its PASSED line:
//   TestScope scope("MyTest");
//   TEST_CHECK(scene.images[4].HasPose(), "image 4 was refused (%u inliers)", numInliers);
//   return scope.Pass("image 4 registered");
// A failed TEST_CHECK prints "MyTest FAILED: <file>:<line>: <condition>: <message>" and returns false
// from the enclosing function, which must return bool and hold the TestScope as `scope`.
// ===============================================================================
class TestScope
{
public:
	explicit TestScope(const char* name);
	// prints "<name> PASSED: <message> (<time since the scope began>)"; returns true
	bool Pass(LPCTSTR fmt, ...) const;
	// prints "<name> FAILED: <file>:<line>: <condition>: <message>"; returns false
	bool Fail(const char* file, int line, const char* condition, LPCTSTR fmt, ...) const;

	const char* const name;
private:
	const Timer::SysType start;
};
#define TEST_CHECK(condition, ...) \
	do { if (!(condition)) return scope.Fail(__FILE__, __LINE__, #condition, __VA_ARGS__); } while (0)
/*----------------------------------------------------------------*/


// ===============================================================================
// Everything the log recorded while it was alive, so a test can assert on what the merge reported.
// The verbosity is set for its lifetime: 3 by default, the per-candidate gate reports being written
// at the ultimate level; 4 for the lines written above it (one per vetoed pair of a sequence chain).
// ===============================================================================
struct LogCapture
{
	String text;
	int verbosity;

	explicit LogCapture(int level = 3) : verbosity(g_nVerbosityLevel) {
		g_nVerbosityLevel = level;
		GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogCapture::Record, this));
	}
	~LogCapture() {
		GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogCapture::Record, this));
		g_nVerbosityLevel = verbosity;
	}
	void Record(const String& msg) { text += msg; }
	bool Contains(const char* what) const { return text.find(what) != String::npos; }
	unsigned Count(const char* what) const {
		unsigned num = 0;
		for (size_t pos = text.find(what); pos != String::npos; pos = text.find(what, pos + 1))
			++num;
		return num;
	}
};
/*----------------------------------------------------------------*/


// ===============================================================================
// Helper: Generate Test Scene with configurable cameras, images, and points
// ===============================================================================
struct SceneConfig {
	enum CameraType { PINHOLE, SPHERICAL };
	enum PoseMode { SIMPLE_TRANSLATION, RANDOM_POSES, CIRCULAR_ARRANGEMENT };
	enum PerturbOptions {
		PERTURB_NONE = 0,
		PERTURB_POSES = 1 << 0,
		PERTURB_POINTS = 1 << 1,
		PERTURB_INTRINSICS = 1 << 2,
		PERTURB_KEYPOINTS = 1 << 3,
		PERTURB_PAIR_POSES = 1 << 4,
		PERTURB_ALL = PERTURB_POSES | PERTURB_POINTS | PERTURB_INTRINSICS | PERTURB_KEYPOINTS | PERTURB_PAIR_POSES
	};
	struct CameraSpec {
		CameraType type{PINHOLE};
		int width{640}, height{480};
		REAL focal{400.0}; // For pinhole
		REAL cx{width/2.0}, cy{height/2.0}; // For pinhole principal point
		REAL k1{0}, k2{0}; // Radial distortion
	};
	std::vector<CameraSpec> cameras{CameraSpec()}; // Camera specifications
	unsigned numImages{3}; // Number of images/views
	unsigned numPoints{60}; // Number of 3D points (0 for none)
	PoseMode poseMode{CIRCULAR_ARRANGEMENT}; // Pose generation mode
	bool addPoseRotations{false}; // Add Y-axis rotations (for SIMPLE_TRANSLATION, crucial for gauge ambiguity)
	int perturbOptions{PERTURB_NONE}; // Bitmask of PerturbOptions
	REAL rotationAngleStep{15.0}; // Rotation step in degrees
	REAL cameraSeparation{0.3}; // Distance between cameras (SIMPLE_TRANSLATION)
	REAL circularRadius{3.0}; // Radius for CIRCULAR_ARRANGEMENT
	bool generateDescriptors{false}; // Generate random descriptors
	bool binaryDescriptors{false}; // Binary vs float descriptors
	int descriptorDim{128}; // Descriptor dimensionality
	bool generatePairs{false}; // Generate image pairs with matches from tracks
	bool generateGPS{false}; // Generate GPS metadata for images
	uint32_t randomSeed{42}; // For reproducible random generation
};

// Helper: Generate random rotation
Matrix3x3 GenerateRandomRotation(std::mt19937& rng, REAL angleRng = 0.3);
// GenerateTestScene: Returns ground truth scene, optionally perturbed scene
void GenerateTestScene(Scene& scene, const SceneConfig& cfg, Scene* scenePerturbed = nullptr);

// The arc of the view-graph tests: 16 cameras every 16 degrees on the default circle (240 degrees of
// it), numPoints points, and every pair generated with its true relative pose
//   Scene sceneGT, scene;
//   GenerateTestScene(sceneGT, RingConfig(300), &scene);
SceneConfig RingConfig(unsigned numPoints);

// Helper: Generate a scene with known 2-cluster structure for clustering tests
void GenerateTwoClusterScene(
	Scene& scene,
	unsigned clusterSizeA,
	unsigned clusterSizeB,
	unsigned numCrossPairs,
	unsigned matchesPerCrossPair,
	unsigned numPoints = 100,
	uint32_t seed = 42,
	bool hubBridge = false);
/*----------------------------------------------------------------*/


// ===============================================================================
// A view graph without geometry, the scene of the view-graph, clustering and star tests: numImages
// images of one 640x480 pinhole of 600 px focal, without pose, and pairs added in call order (a test
// may index them by that order; duplicates are allowed, the stages under test meet them):
// - Pair: a pair whose composite weight is the given number (weightedInliers = weight, the three
//   quality factors 1 and the triplet strength 0.75, so 0.25 + 0.75 = 1; the weighted inliers are
//   capped at 1000 in the composite), no matches;
// - Matched: a pair of numMatches track-forming matches on new keypoints at the centres of the first
//   cellsA (cellsB) cells of a 10x10 grid over each image (100: full coverage, so the triplet strength
//   equals the inlier count), numFilteredInliers set and F the identity standing in for the geometric
//   verification when verified; no weight: the stage under test computes it;
// - Hub, Chain, Clique: weighted pairs over the inclusive range [first, last].
//   PairGraph g(scene, 25);
//   g.Hub(0, 9, 200); g.Chain(1, 9, 100); g.Clique(21, 24, 50); g.Pair(21, 9, 4);
// ===============================================================================
struct PairGraph
{
	Scene& scene;

	PairGraph(Scene& scene, IIndex numImages);
	ImagePair& Pair(IIndex a, IIndex b, float weight);
	ImagePair& Matched(IIndex a, IIndex b, unsigned numMatches, bool verified = true, unsigned cellsA = 100, unsigned cellsB = 100);
	void Hub(IIndex hub, IIndex last, float weight);      // (hub, k) for k in (hub, last]
	void Chain(IIndex first, IIndex last, float weight);  // (k, k + 1) for k in [first, last)
	void Clique(IIndex first, IIndex last, float weight); // every two images of [first, last]
};
/*----------------------------------------------------------------*/


// ===============================================================================
// An arc of posed cameras, the scene of the resection tests. Camera i sits on a circle of radius
// RADIUS in the XZ plane at i*STEP degrees, looking at the origin, all sharing one 640x480 pinhole of
// 500 px focal: consecutive centres are 0.8716 apart and seven of them span EXTENT world units, the
// unit position tolerances are a share of. Points are drawn from the 2-unit box around the origin,
// which projects well inside the frame from every camera. Every image starts registered at its true
// pose (kept in `truth`); a test adds tracks, correspondences and pairs, then unregisters the images
// to resect. Random draws come from the scene's own generator, so a seed fixes the whole scene.
// ===============================================================================
// What an ArcScene pair adds to the true relative pose it carries: the ray angle it reports (0: not
// measured), its translation turned away by turnTranslationDeg (length kept) and its rotation turned
// about the vertical axis by turnRotationDeg (translation kept); withMatches gives it numInliers
// placeholder matches and as many filtered inliers, for the stages that count them
struct ArcLinkSpec { REAL rayAngleDeg{0}, turnTranslationDeg{0}, turnRotationDeg{0}; bool withMatches{false}; };

struct ArcScene
{
	static constexpr REAL RADIUS = 5;
	static constexpr REAL STEP = 10;  // degrees between consecutive cameras
	static constexpr REAL EXTENT = 5; // 2*RADIUS*sin(3*STEP)

	Scene scene;
	std::vector<Pose3D> truth; // per image, the pose it was built at
	std::mt19937 rng;

	ArcScene(unsigned numImages, uint32_t seed);

	// the camera pose at the given angle along the circle, in degrees
	static Pose3D Pose(REAL angleDeg);
	// move an image (and its truth) to the given angle along the circle; before any track is added
	void Place(IIndex imageID, REAL angleDeg);
	// one track at X, observed at its exact projection by every given image; false (nothing added) when
	// X does not project inside all of them
	bool TrackAt(const Point3& X, const IIndexArr& imageIDs);
	// numTracks tracks at random points of the box, its height scaled by spreadY (0: the plane of the
	// arc), each observed at its exact projection by every given image (points not projecting inside
	// all of them are drawn again)
	void Tracks(const IIndexArr& imageIDs, unsigned numTracks, REAL spreadY = 1);
	// move the given image's observations of the tracks [firstTrack, firstTrack + numTracks) by delta pixels
	void Displace(IIndex imageID, unsigned firstTrack, unsigned numTracks, const Point2f& delta);
	// the image's observations of the first numTrue + numNoise tracks: the first numTrue at the
	// projections of `projecting` (the image's own pose when NULL), the rest at random pixels
	void Correspondences(IIndex imageID, unsigned numTrue, unsigned numNoise, const Pose3D* projecting = NULL);
	// a verified pair of the two images (either order) carrying the relative pose their stored poses
	// give, numInliers weighted inliers and full spatial, coverage and connectivity weights (so its
	// composite weight is valid), changed as the spec says; no matches: the resection reads none
	ImagePair& Pair(IIndex idA, IIndex idB, unsigned numInliers, const ArcLinkSpec& spec = {});
	// invalidate the given images' poses, leaving the others registered, and triangulate the tracks
	void Unregister(const IIndexArr& imageIDs);
	// triangulate the tracks from the registered images, at the resection's bars (4 px, 1 degree)
	void Triangulate();
};
/*----------------------------------------------------------------*/


// ===============================================================================
// A capture walking around a loop: blocks of cameras on a circle looking outward at a cylinder
// wall. Every merge test builds its scene from these helpers, so a defect one of them reproduces
// is described by the same handful of numbers.
// ===============================================================================
struct RingSceneConfig
{
	unsigned numBlocks{12}, camsPerBlock{10}, numPoints{3000};
	REAL cameraRadius{5}, wallRadius{9}, wallHeight{4};
	REAL driftDegPerBlock{0};    // internal bend of every block (rotation ramp about the ring axis)
	REAL driftScalePerBlock{0};  // internal scale ramp of every block (centres scaled about the block's first camera)
	REAL noisePx{0.3};           // keypoint noise
	std::set<uint32_t> tinyBlocks; // these blocks get two cameras 1 mm apart instead of camsPerBlock
	uint32_t foldedBlock{NO_ID}; // this block's second half is reconstructed folded by foldDeg
	REAL foldDeg{180};           // the turn of the folded half about the ring axis
	uint32_t seed{7};
};

// The ring scene: every camera posed (its pose also in gtPoses), the blocks as contiguous arcs of
// global IDs, the wall's points as tracks with noisy keypoints, and a verified pair, matched through
// those tracks, per image couple that at least kRingMinSharedTracks tracks join
void GenerateRingScene(const RingSceneConfig& cfg, Scene& scene, std::vector<IIndexArr>& blocks, std::vector<Pose3D>& gtPoses);
/*----------------------------------------------------------------*/


// ===============================================================================
// The blocks of a scene as their reconstructions hand them to the merge: each block a sub-scene
// (SceneBlocks::Extract: features and the pairs inside the block moved in, local IDs), its cameras at
// the given true poses, then `distort` applied (the error its reconstruction made; may be empty), its
// tracks triangulated from those cameras and the whole block moved by a random similarity drawn from
// `seed` (seed NO_ID: every block left in the true frame). The global scene is left as the merge
// receives it: no tracks and no pose. Images in no block form the pool. sceneBlocks is an empty
// SceneBlocks of `scene`; truth holds a pose per image of it. Returns the similarity applied to each block.
// ===============================================================================
std::vector<SEACAVE::Transform> SplitOnTruth(Scene& scene, SceneBlocks& sceneBlocks, const std::vector<IIndexArr>& blocks,
	const std::vector<Pose3D>& truth, uint32_t seed, const std::function<void(uint32_t block, Scene& sub)>& distort = {});
/*----------------------------------------------------------------*/


// ===============================================================================
// A ring capture ready for the merge. Construction generates the ring (GenerateRingScene); a test may
// then edit the scene's tracks and pairs (cut a seam, plant a false one) before Split() hands the
// blocks out (SplitOnTruth, `split` holding them); Merge() then runs one merge of them into the scene.
//   RingFixture ring({4, 10});
//   ThinSeam(ring.scene, ring.blocks, 1, 2, 0, 10); // cut a seam
//   ring.Split();
//   const MergeReport rep = ring.Merge();
// ===============================================================================
struct RingFixture
{
	const RingSceneConfig cfg;
	Scene scene;                    // the global scene the merge writes into
	std::vector<IIndexArr> blocks;  // per block, its global image IDs
	std::vector<Pose3D> truth;      // per global image, its true pose
	SceneBlocks split;              // the blocks, once Split
	std::vector<SEACAVE::Transform> applied; // per block, the similarity its frame is off by

	explicit RingFixture(const RingSceneConfig& cfg);
	RingFixture(const RingFixture&) = delete;
	RingFixture& operator=(const RingFixture&) = delete;

	// hand the blocks out as their reconstructions would; once
	void Split();
	// one merge of the blocks (GlobalAlignment::MergeScenes with these arguments) into the scene
	MergeReport Merge(bool releaseUnplaced = true, uint32_t anchorBlock = NO_ID,
		const std::unordered_set<IIndex>& poolImages = {}, const GlobalAlignmentConfig& alignCfg = {});
};
/*----------------------------------------------------------------*/


// ===============================================================================
// The four photographs bundled in data/images, two of them HEIC (decoded only with libheif), and the
// configuration every real-images test runs them through: AKAZE at 400-900 features per cell, every
// pair matched at the thresholds of AKAZE.
// ===============================================================================
ReconstructionConfig BundledImagesConfig();
// the bundled images imported with importCfg (only its forced focal, k1 and k2 may differ from the defaults) and their
// features extracted with BundledImagesConfig(), computed once per process for each such import and copied into
// scene, which must be empty and keeps its own thread count; false (and the reason logged) unless all four imported
//   Scene scene(1);
//   TEST_CHECK(BundledImagesFeatures(scene, importCfg), "the bundled images were not imported");
bool BundledImagesFeatures(Scene& scene, const ImportConfig& importCfg);
/*----------------------------------------------------------------*/


// ===============================================================================
// A rig of perspective images: one trusted pinhole camera of the given size and focal, principal point
// at the frame centre and no distortion, shared by one image per pose (named "<i>.jpg", posed,
// counted calibrated), with no keypoints: the test adds what it observes. An image whose pose the test
// does not read takes Pose3D::Identity().
//   Scene scene(MakeRig(cv::Size(640, 480), 400, {Pose3D::Identity(), Pose3D(RMatrix::IDENTITY, CMatrix(0.4, 0.1, 0))}));
// ===============================================================================
Scene MakeRig(cv::Size size, REAL focal, const std::vector<Pose3D>& poses);
// a pair of two posed images armed with their true geometry: relativePose = images[ID2] / images[ID1]
// (the baseline at its true length), E and F composed from it and the two cameras
void ArmPair(ImagePair& pair, const Scene& scene);
/*----------------------------------------------------------------*/


// ===============================================================================
// Panoramas looking at a sphere of points: one spherical camera of the given size shared by images at
// the given centres (rotation identity, named ""), and numPoints points at directions drawn uniformly
// over the sphere and distances 4.5-5.5 from the origin, each a track with a keypoint at its exact
// projection in every image (a point seen by fewer than two is dropped). With numPoints 0 the cameras
// alone. Every camera sees about half the points behind it, the case the bearing-vector geometry exists for.
//   Scene scene(MakeSphereScene({Point3(-0.5, 0, 0), Point3(0.5, 0, 0.2)}, 120, 2027));
// ===============================================================================
Scene MakeSphereScene(const std::vector<Point3>& centres, unsigned numPoints, uint32_t seed, cv::Size size = cv::Size(2048, 1024));
/*----------------------------------------------------------------*/


// Angle in degrees between two rotations
inline double RotationErrorDeg(const RMatrix& a, const RMatrix& b)
{
	return R2D(ACOS(ComputeAngle(a, b)));
}

// ===============================================================================
// How far a reconstruction's registered images are from the truth, in the frame the reconstruction
// was free to choose: the best similarity of their centres onto the true ones is applied first, then
// per image the angle between its rotation and the true one (degrees) and the distance of its centre
// from the true one as a share of the extent of the truth (the largest distance between two true
// centres, over every image, so the unit is the same however many came back registered).
//   const PoseError e = PoseErrors(scene, truth);
//   TEST_CHECK(e.count == 24 && e.rotationMax <= 0.5 && e.positionMax <= 0.01, ...);
// ===============================================================================
struct PoseError
{
	unsigned count{0};                                  // registered images measured
	REAL rotationMedian{FLT_MAX}, rotationMax{FLT_MAX}; // degrees
	REAL positionMedian{FLT_MAX}, positionMax{FLT_MAX}; // share of the extent of the truth
};
// over the registered images of subset (every image when empty); with fewer than three of them no
// similarity is defined and the errors stay FLT_MAX
PoseError PoseErrors(const Scene& scene, const std::vector<Pose3D>& truth, const IIndexArr& subset = {});
/*----------------------------------------------------------------*/


// ===============================================================================
// Helpers that two or more topic files use
// ===============================================================================
// A pair's sparse segment is all described keypoints and its dense segment all dense ones
bool HasDensePartition(const Scene& scene, const ImagePair& pair);
// The reconstruction stage of RealImagesPipelineTest (tracks, star initialization, colors and the
// checks on what they produce), shared with DenseMatcherReconstructTest so that both hold a matched
// scene to exactly the same expectations
bool ReconstructMatchedScene(Scene& scene, const char* testName, unsigned minTracks, unsigned maxTracks, REAL maxDistortion);
// The blocks of a partition as the reconstruction extracts them, with their local-to-global maps
std::vector<Scene> ExtractBlocks(Scene& scene, const ScenePartition& partition, std::vector<IIndexArr>* localToGlobals = NULL);
// The middle value of a sample, its own numbers left in place
REAL Median(std::vector<REAL>& values);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _TESTS_SFM_FRAMEWORK_H_
