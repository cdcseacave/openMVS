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
// The verbosity is raised for its lifetime, the per-candidate gate reports being written at the
// ultimate level.
// ===============================================================================
struct LogCapture
{
	String text;
	int verbosity;

	LogCapture() : verbosity(g_nVerbosityLevel) {
		g_nVerbosityLevel = 3;
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
		CameraType type;
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
// about the vertical axis by turnRotationDeg (translation kept)
struct ArcLinkSpec { REAL rayAngleDeg{0}, turnTranslationDeg{0}, turnRotationDeg{0}; };

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
	// numTracks tracks at random points of the box, each observed at its exact projection by every
	// given image (points not projecting inside all of them are drawn again)
	void Tracks(const IIndexArr& imageIDs, unsigned numTracks);
	// the image's observations of the first numTrue + numNoise tracks: the first numTrue at the
	// projections of `projecting` (the image's own pose when NULL), the rest at random pixels
	void Correspondences(IIndex imageID, unsigned numTrue, unsigned numNoise, const Pose3D* projecting = NULL);
	// a verified pair of the two images (either order) carrying the relative pose their stored poses
	// give, numInliers weighted inliers and full spatial, coverage and connectivity weights (so its
	// composite weight is valid), changed as the spec says; no matches: the resection reads none
	ImagePair& Pair(IIndex idA, IIndex idB, unsigned numInliers, const ArcLinkSpec& spec = {});
	// invalidate the given images' poses, leaving the others registered, and triangulate the tracks
	void Unregister(const IIndexArr& imageIDs);
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
// `seed`. The global scene is left as the merge receives it: no tracks and no pose. Images in no
// block form the pool. sceneBlocks is an empty SceneBlocks of `scene`; truth holds a pose per image of
// it. Returns the similarity applied to each block.
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
// import the bundled images; false (and the reason logged) unless all four were imported
bool ImportBundledImages(Scene& scene, const ImportConfig& importCfg);
/*----------------------------------------------------------------*/


// Angle in degrees between two rotations
inline double RotationErrorDeg(const RMatrix& a, const RMatrix& b)
{
	return R2D(ACOS(ComputeAngle(a, b)));
}
/*----------------------------------------------------------------*/


// ===============================================================================
// Helpers that two or more topic files use (moved unchanged from the single TestsSFM.cpp)
// ===============================================================================
// A pair's sparse segment is all described keypoints and its dense segment all dense ones
bool HasDensePartition(const Scene& scene, const ImagePair& pair);
// A synthetic full-sphere scene with two spherical cameras in a tight cluster
void BuildSphericalTwoViewScene(Scene& scene, Pose3D& poseRel);
// The reconstruction stage of RealImagesPipelineTest (tracks, star initialization, colors and the
// checks on what they produce), shared with ROMA2ReconstructTest so that both hold a matched
// scene to exactly the same expectations
bool ReconstructMatchedScene(Scene& scene, const char* testName, unsigned minTracks, unsigned maxTracks, REAL maxDistortion);
// A scene of numImages images without pose sharing one camera, and a pair of numInliers track-forming matches
void AddTripletImages(Scene& scene, IIndex numImages);
void AddTripletPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers, bool verified = true,
	unsigned cellsA = 100, unsigned cellsB = 100);
// The resection arc scenes
// Scenes for the two resection tests. Cameras sit on a 5-unit circle in the XZ plane, one every
// 10 degrees, all looking at the origin, so consecutive centres are 2*5*sin(5 deg) = 0.8716 apart
// and the seven of a full arc span 2*5*sin(30 deg) = 5.0000 world units -- the scene extent the
// position tolerances below are a share of. The points they see fill a 2-unit box around the
// origin, which projects well inside a 640x480 frame of 500-pixel focal from every one of them.
constexpr REAL RESECTION_ARC_RADIUS = 5;
constexpr REAL RESECTION_ARC_STEP = 10; // degrees between consecutive cameras
Pose3D ResectionArcPose(REAL angleDegrees);
void BuildResectionArc(Scene& scene, unsigned numImages);
bool AddResectionTrack(Scene& scene, const Point3& X, const std::vector<IIndex>& imageIDs);
void AddResectionTracks(Scene& scene, const std::vector<IIndex>& imageIDs, unsigned numTracks, std::mt19937& rng);
void AddResectionPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers);
// The blocks of a partition as the reconstruction extracts them, with their local-to-global maps
std::vector<Scene> ExtractBlocks(Scene& scene, const ScenePartition& partition, std::vector<IIndexArr>* localToGlobals = NULL);
// The middle value of a sample, its own numbers left in place
REAL Median(std::vector<REAL>& values);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _TESTS_SFM_FRAMEWORK_H_
