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
// pixel<->grid<->normalized coordinate conventions), overlap gating, confidence-map
// erosion, and the store/replace-by-inlier-count policy of the guided pairs
bool ROMA2WarpTrackingTest();

// Global-descriptor retrieval test: cosine ranking of the per-image global descriptors and its
// deterministic tie order, the PairsMatcher dispatch that ranks the candidate pairs through them
// instead of the vocabulary tree, the rankings CSV export, the .sfm round-trip of the
// descriptors, and the two host-side pooling recipes against the export script's fixtures
bool GlobalDescriptorsQueryTest();

// RoMa2 CPU preprocessing test: a constant image maps to constant planes with the expected
// R/G/B channel swap, and resampling a real fixture image reproduces torch's own
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) to within 1e-5
bool RoMa2PreprocessTest();

// RoMa2 ONNX parity test: runs the exported descriptor and coarse-match graphs through
// RoMa2Onnx and compares them to the Python reference dumps shipped with the models.
// Skipped unless OPENMVS_ROMA2_MODEL_PATH points at an exported model folder;
// OPENMVS_ROMA2_PROVIDER (auto|cuda|coreml|dml|cpu) and OPENMVS_ROMA2_SETTING (turbo|fast|base)
// narrow the execution provider and the preset it exercises
bool RoMa2OnnxParityTest();

// Test Bundle-Adjustment PinholeReprojectionErrorAnalytic Jacobians against AutoDiff
bool BAPinholeReprojectionJacobianTest();

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
