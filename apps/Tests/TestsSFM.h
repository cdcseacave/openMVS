/*
 * TestsSFM.h
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


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// VocabularyTree save/load roundtrip test
bool VocabularyTreeTest();

// ROMA2 warp helpers test: keypoint tracking through an identity warp (the
// pixel<->grid<->normalized coordinate conventions), the confidence gate, and the dense append
bool ROMA2WarpTrackingTest();

// Coverage-uniform warp sampling, the sample the verdict fits its geometry on: the sample budget,
// the spread the bucket stratification buys over a plain top-confidence selection, the coverage a
// genuinely one-sided sample reports, and the determinism of the draw
bool ROMA2CoverageSampleTest();

// The complementary dense draw, the fill of an admitted pair: it is drawn only where that pair's
// guided sparse matches are NOT, capped at the pair's dense budget, thinned when over budget by the
// pair-independent lattice key rather than by confidence -- so a tighter budget keeps a subset of
// what a looser one keeps -- identical across two pairs sharing an image, and deterministic
bool ROMA2ComplementaryDrawTest();

// The dense fill's bucket pitch (DenseFillGridSide): a function of the density and the warp side
// alone, not of the pair, so the draw it feeds is a density over the overlap the sparse matches did
// not cover rather than a fixed count per pair -- a larger confident region draws proportionally
// more, an occupied bucket still yields nothing so the sparse share comes out of the same draw, and
// two pairs sharing an image stratify it on the same grid and so agree on the pixels they sample,
// an agreement that survives the one term of the draw that IS per-pair: two pairs whose ceilings
// thin that agreement to different sizes keep nested samples of it rather than divergent ones
bool ROMA2DenseFillDensityTest();

// The dense fill ceiling (DenseFillCeiling): the configured density over the SMALLER of the
// verdict's two inlier areas, symmetric in A and B, and linear in the density knob
bool ROMA2DenseFillCeilingTest();

// The pair verdict (JudgePairROMA2) on the exact bidirectional warp of two pinhole cameras looking
// at a non-planar surface: a warp confident over ~30% of both frames is admitted with both inlier
// areas measuring that share, a warp confident over 30% of A whose B side maps into A over only 3%
// is rejected by the min side alone, a smooth warp unrelated to the cameras (a homography of A's
// grid) is rejected however exactly one geometry explains its own side -- through the calibrated
// branch and, with forceFundamental, through the 7-DoF one the min-side rule was designed for --
// and minOverlap 0 admits the first two
bool ROMA2VerdictTest();

// Guided sparse matching (MatchFeaturesGuided): the ratio taken against the best descriptor OUTSIDE
// the search disc rejects a keypoint whose lookalike sits elsewhere in the other image, accepts one
// whose only close descriptor is inside the disc, and is no longer defeated by a scale duplicate
// inside it; the same inputs give the same matches, in the same order
bool ROMA2GuidedMatchTest();

// Pair assembly and storage (AssemblePairROMA2, StorePairROMA2): the union of the guided matches
// and the dense fill is fitted once and splits into the pair's sparse and dense segments under one
// relative pose, a pair with no guided match is still assembled as a dense-only pair, a fill too
// small to fit leaves the verdict's geometry untouched, and the store appends the dense keypoints
// past each image's described prefix with reproducible indices, inserting the dense block ahead of
// a pair's rejected tail rather than past it
bool ROMA2AssemblyTest();

// The described/dense keypoint boundary: an image whose keypoints.size() > descriptors.rows keeps
// its stored described-keypoint count across a descriptor release and an .sfm round-trip, the
// index tests read it, and PairsMatcher::FilterRedundantKeypoints moves it through the same remap
// it applies to the keypoint indices
bool DenseKeypointBoundaryTest();

// Dense supplementation is real evidence, not noise to filter around: on a supplemented pair,
// FilterMatches' meanRayAngle and ComputeIntrinsicWeight's grid occupancy are both measured over the
// whole track-forming set (sparse + dense), the minimum-support floor reads that same set so a pair
// under the sparse-only bar still counts, and GetNumWeightedInliers() discounts the dense share by
// DENSE_OBSERVATION_WEIGHT rather than dropping or fully counting it. A degenerate all-dense baseline
// is still demoted, because the angle term finally has something to measure.
bool SupplementEvidenceIsolationTest();

// Global-descriptor retrieval test: cosine ranking of the per-image global descriptors and its
// deterministic tie order, PairsMatcher's RETRIEVAL-mode pair selection over the same
// descriptors, the rankings CSV export, and the .sfm round-trip of the descriptors
bool GlobalDescriptorsQueryTest();

// RETRIEVAL match-mode test: candidate selection ranks purely by the global descriptors --
// VOCABULARY and RETRIEVAL name two backends and neither consults a gate to borrow the
// other's -- a missing descriptor is a hard error rather than a vocabulary-tree fallback, and
// the mode dispatches correctly end-to-end through Match()
bool RetrievalModeTest();

// VOCABULARY/RETRIEVAL backend-isolation test: on a scene where the local descriptors and the
// global descriptors cluster the 12 images into two different partitions, VOCABULARY's pair
// set follows the local-descriptor clustering and RETRIEVAL's follows the global one -- the
// two provably disagree, so neither mode can be silently ranking with the other's backend
bool VocabularyIgnoresGlobalDescriptorsTest();

// RoMa2 CPU preprocessing test: a constant image maps to constant planes with the expected
// R/G/B channel swap, and resampling a real fixture image reproduces torch's own
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) to within 1e-5
bool RoMa2PreprocessTest();

// RoMa2 manifest format-version test: a manifest declaring format_version 1 loads, and one
// declaring format_version 3 is rejected. Pure JSON parsing, so it needs neither ONNX Runtime
// nor OPENMVS_ROMA2_MODEL_PATH.
bool RoMa2ManifestVersionTest();

// RoMa2 ONNX parity test: runs the exported descriptor and coarse-match graphs through
// RoMa2Onnx and compares them to the Python reference dumps shipped with the models.
// Skipped unless OPENMVS_ROMA2_MODEL_PATH points at an exported model folder;
// OPENMVS_ROMA2_PROVIDER (auto|cuda|coreml|dml|cpu) and OPENMVS_ROMA2_SETTING (turbo|fast|base)
// narrow the execution provider and the preset it exercises
bool RoMa2OnnxParityTest();

// ROMA2 reconstruct test: runs the full in-process path on the bundled 4-image scene through
// Scene::MatchPairs, four times. The per-image global retrieval descriptor (2048-D, the graph's
// own on-device pooling) is checked, EXHAUSTIVE geometric matching still connecting and verifying
// every pair; a run with the dense matching pass on is compared against a baseline run with it off
// (the same pair set, every pair carrying a dense segment of its own and a sparse segment the
// guided matching produced), survives an .sfm round-trip of the descriptors and the pairs,
// and then goes through ReconstructTest's own reconstruction stage; and two runs of one awkward
// configuration (single thread, a slot pool too small for the scene) must match identical pairs,
// which is what pins the determinism of the pass. Skipped unless OPENMVS_ROMA2_MODEL_PATH points
// at an exported model folder; OPENMVS_ROMA2_PROVIDER and OPENMVS_ROMA2_SETTING narrow the
// execution provider and the preset it exercises
bool ROMA2ReconstructTest();

// Test Bundle-Adjustment PinholeReprojectionErrorAnalytic Jacobians against AutoDiff
bool BAPinholeReprojectionJacobianTest();

// ComputeObservationSigmas recovers the displacement the BULK of a synthetic scene's described and
// dense observations carry, with a tenth of each population thrown far out and a tenth left almost
// on the point so that only a median answers it, and reports zero dense observations on a scene that
// has none
bool ObservationSigmasTest();

// EstimateDenseObservationWeight returns 1/k^2 for the k a synthetic scene's described/dense
// displacement ratio gives, clamps to 1 when the dense population is no less precise than the
// described one, floors a far coarser one, and falls back to the configured constant both on a scene
// with no dense keypoints and on one whose dense population exists but is under the sigma threshold
bool DenseObservationWeightEstimateTest();

// Small SFM smoke test: build tiny scene and run BundleAdjustment::Adjust
bool PipelineTest();

// GPS-prior BA on a geo-aligned scene: absolute (datum-free) pose covariance
// and the missing-accuracy fallback
bool GPSPriorPoseUncertaintyTest();

// Pose-quality report roundtrip: pose uncertainty recorded on the scene from the last
// BA, CSV export re-read, ExportMVS preserving the SFM image IDs the report is
// correlated by, world-transform covariance mapping, and .sfm serialization
bool PoseUncertaintyExportTest();

// GPS alignment degeneracy test: coincident/collinear GPS positions must be
// rejected without modifying the scene; well-spread GPS must still align
bool AlignToGPSDegenerateTest();

// Full-hemisphere spherical reconstruction regression test: exercises the
// Triangulation + BA pipeline on a spherical scene with 3D points distributed
// across the entire sphere (front AND back hemispheres). Pins the correctness
// of the Unproject / TriangulateDLT path for spherical cameras.
bool ReconstructSphericalSyntheticTest();

// Integration test for the PairsMatcher -> poselib::estimate_relative_pose_bearings
// path on a full-sphere spherical scene. Validates RANSAC scoring with
// cheirality disabled for spherical cameras, plus the Sampson-on-sphere
// refinement in refine_relpose_bearing.
bool PairsMatcherSphericalTest();

// Integration test for MatchFeaturesGeometric on a spherical pair. Exercises
// the post-RANSAC epipolar-constrained descriptor matching step which must
// fall back to Sampson-on-sphere + angular threshold when pair.F is absent
// (pure spherical pairs don't have a meaningful fundamental matrix).
bool MatchGeometricSphericalTest();

// Phase 5 cube-map bridge tests: verify that SFM::ExportMVS can expand
// every spherical source image into 6 (or 4) pinhole cube-map faces,
// emit them as a rig platform in MVS::Interface format, and produce a
// file tree that MVS::Scene::Load reads back without any pinhole
// regression.
bool CubeMapFaceRenderTest();
bool CubeMapBridgeGeometryTest();
bool CubeMapBridgeEndToEndTest();
bool CubeMapBridgeMVSLoadTest();
bool CubeMapBridgeMixedSceneTest();
bool CubeMapBridgeDropTopBottomTest();

// Triplet star-initialization test: 3-view scene with tracks + StarInitializer + BA
bool TripletStarInitTest();

#ifdef _IMAGE_HEIF
// HEIF/HEIC integration at the SFM layer (the reader itself is covered by CImageHEIF::Test):
// the decoded resolution the MVS camera is paired against, the EXIF metadata bridge (focal
// length agreeing with the classic stream scan, GPS), the "don't rotate twice" orientation
// guard for a container 'irot' that duplicates an EXIF Orientation, and the LoadPixels
// fallback for a format cv::imread cannot decode -- in both color and gray, the latter being
// what feature extraction uses. Pixel content as a whole is covered by ReconstructTest and the
// MVS PipelineTest, two of whose four images are HEIC.
bool HEIFMetadataTest();
#endif

// Import determinism: repeated Scene::Import of the bundled 2 JPG + 2 HEIC folder, with the
// metadata loop running in parallel, must yield finite/positive focals, finite/non-negative
// sensor sizes, and the very same per-image cameras and camera count every time; then the same
// per-file read once more on a deliberately dirtied stack, which is what pins the metadata to
// the file rather than to whatever the process did before it
bool ImportMetadataDeterminismTest();

// Pose-frame detection: recover both the camera-axes convention and, for EXIF-rotated images,
// the in-plane rotation of an imported frames.json from the matched pairs
bool FramesPoseFrameDetectionTest();

// Known-pose import, pair selection, and prior-frame alignment tests
bool KnownPosesImportTest();
bool KnownPosePairSelectionTest();
bool AlignToPriorPosesTest();
bool AlignToPriorPosesCollinearTest();

// Two-view geometry test: PairsMatcher and ImagePair matrix operations
bool TwoViewTest();

// Reconstruction test: Import images, extract features, match pairs, build tracks, and initialize
bool ReconstructTest(bool verbose = false);

// Task 1 (roma2-followups-20260830): --export-pairs-csv is written by Scene::Reconstruct()
// right after pair matching, from both call sites (fresh import + match, and an already-matched
// .sfm given back as source), before any reconstruction step can drop pairs
bool ReconstructExportCSVTest();

// Task 5 (roma2-followups-20260830): the camera-triplet view-graph disambiguation of
// Manam & Govindu (CVPR 2024) on the brief's hand-computed 8-node graph -- scores, tau, the
// graph statistics, the kept sets at m = 0.3 and 0.6, duplicate and unverified pairs, and a
// graph with no triplet at all
bool TripletFilterTest();

// Test function for rotation estimation
bool RotationEstimatorTest();

// Test function for global scale estimation
bool ScaleEstimatorTest();

// Test function for global translation estimation
bool TranslationEstimatorTest();

// Pairs weighting test
bool PairsWeightingTest();

// PairsMatcher sequential mode test
bool PairMatcherTest();

// Scene::MatchPairs() fails the stage when matching two or more images leaves the view graph empty
bool MatchPairsFailureTest();

// PairsMatcher::Match()'s fatal-round signal is not "this round stored nothing"
bool MatchRoundFatalSignalTest();

// Pre-matching optimization test
bool PreMatchTest();

// View graph calibrator test: Refine focal length using view graph optimization
bool ViewGraphCalibratorTest();

// Phase 1: Scene Clustering tests
bool SceneClusterSingleClusterTest();
bool SceneClusterSizeConstraintsTest();
bool SceneClusterDisconnectedComponentsTest();
bool SceneClusterMemoryProtocolTest();
bool SceneClusterIDRemappingTest();
bool SceneClusterSmallClusterRescueTest();

// Phase 3: Global Alignment tests
bool GlobalAlignmentBuildGlobalToLocalMapTest();
bool GlobalAlignmentRotationAveragingExtendedTest();
bool GlobalAlignmentScaleAveragingExtendedTest();
bool GlobalAlignmentScaleAveragingFallbackTest();
bool GlobalAlignmentTranslationAveragingExtendedTest();
bool GlobalAlignmentMergeSingleSceneTest();
bool GlobalAlignmentTrackMergeDuplicateImageGuardTest();
bool GlobalAlignmentTrackMerge3DProximityGuardTest();

// End-to-end hierarchical SFM tests
bool HierarchicalSFMSplitMergeRoundtripTest();
bool HierarchicalSFMWithRandomTransformTest();
/*----------------------------------------------------------------*/

} // namespace SFM
