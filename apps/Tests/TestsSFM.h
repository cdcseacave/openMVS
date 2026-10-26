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

// The warp grid's pixel <-> cell convention (align_corners=False cell centres) end to end: keypoint
// tracking, the draws' cell positions, the occupancy rounding and the verdict's cells
bool ROMA2WarpGridConventionTest();

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

// Guided sparse matching (MatchFeaturesGuided) without a geometry, the disc: a keypoint alone in
// its disc whose lookalike sits elsewhere in the other image is refused by the outside reference,
// one whose only close descriptor is inside the disc is accepted, and a scale duplicate inside it is
// no rival; the same inputs give the same matches, in the same order
bool ROMA2GuidedMatchTest();

// The epipolar band of the guided sparse matching (GuidedSearch): a rival off the epipolar line is
// no candidate, an equally close rival on it refuses the match, a same-feature duplicate is no
// rival, every winner answers to the reference outside its band, and the search is the disc
// without a geometry or near the epipole
bool ROMA2GuidedBandTest();

// The track builder on a component holding one image twice (BuildTracks, TrackConflictConfig): the
// cut removes the least-supported link on the path between the two keypoints and keeps the
// corroborated side where the pair-order veto keeps whichever arrived first, two keypoints within
// the merge distance are one feature and the one with more links stays, a component above the size
// bar takes the veto, the cut can be switched off, and a conflict-free scene is untouched
bool BuildTracksConflictTest();

// Pair assembly and storage (AssemblePairROMA2, StorePairROMA2): the union of the guided matches
// and the dense fill is fitted once and splits into the pair's sparse and dense segments under one
// relative pose, a pair with no guided match is still assembled as a dense-only pair, a fill too
// small to fit leaves the verdict's geometry untouched, and the store appends the dense keypoints
// past each image's described prefix with reproducible indices, inserting the dense block ahead of
// a pair's rejected tail rather than past it
bool ROMA2AssemblyTest();

// The weighted refinement that closes a pair's assembly: with the dense fill displaced 2 px across
// the epipolar lines and the guided matches exact, the pose follows the sparse matches at
// ROMA2Config::denseFitWeight and the dense bias at full weight
bool ROMA2DenseFitWeightTest();

// The inlier tests of a RoMa2 pair through a lens (SampsonTest): on two views through a barrel lens
// the camera knows, every guided match on the true geometry is a sparse inlier, the corner ones
// included, while the pixel fundamental matrix of the same geometry, composed without the lens, puts
// the corner ones past the matcher's epipolar bar
bool ROMA2LensInlierTest();

// The keyframe extractor's geometric match through a lens (MatchFeaturesGeometric): every corner
// correspondence of the same two views is matched, though the pixel F's epipolar line misses it by
// more than the bar the match is held to
bool MatchGeometricLensTest();

// The lens fit from pairs (FitLensesFromPairs) recovers the lenses that rendered them: one camera
// from 40 pairs that turn and move, to 2 px over the radii its matches reach, the lens set on the
// camera, also where the first joint fit falls far short of the lens and the pose re-estimation
// takes more than three rounds to reach it, and where it is held short of the lens from no lens, a
// barrel lens and a pincushion of 160 px, which the fit started again reaches; two cameras with
// different lenses joined only by 40 cross pairs, each fitted, the weak lens to 2 px and the strong one
// to 5 px
bool LensFitRecoveryTest();
// The focal check of the lens fit corrects a trusted focal its pairs clearly reject and keeps a right one
bool LensFitFocalTest();

// The lens fit refuses what no lens explains: pairs of a camera without a lens, forward pairs that
// hide a strong lens, and exact positions whose error is float rounding, without a lens or through one
// too faint to matter (1e-3 px at the corner); every refused camera is left without a lens
bool LensFitRefusalTest();

// A camera the lens fit cannot hold leaves it with every pair it has: one under the minimum of
// eligible pairs, one whose given lens has terms past k1 and k2, one whose focal is not trusted and
// one whose given lens folds within its frame are each left as they are, and the camera they pair
// with is fitted from its own pairs alone; a camera given its true lens as k1, k2 alone is held, and
// its cross pairs, either camera first, inform the fitted camera
bool LensFitMinimumPairsTest();

// The re-verification a camera's change sets off (PairsMatcher::ComputeRelativePoses) keeps a RoMa2
// pair's partition: on the wedge, with nothing about the cameras changed, the pair comes back with
// the same sparse inliers, the same dense segment of dense keypoints and the same outliers, a pair
// whose dense fill drew nothing comes back well formed too, and so does a pair whose matches all sit
// among its outliers, its dense matches back in the dense segment; a pair the re-fit refuses (stored
// without a geometry, or under the minimum of matches with its geometry kept) comes back invalidated,
// with the matcher's strict filter and without it; under the default configuration, with the dense
// fill biased across the epipolar lines, the re-verified pose follows the sparse matches where
// GeometricFilter at full weight follows the bias
bool ROMA2ReverifyPartitionTest();

// A RoMa2 pair matched through the pinhole and re-verified once its camera knows its lens
// (ReverifyPairROMA2): with the pinhole geometry of the true pose kept at matching, the guided matches
// at the corners the pinhole rejected come back as sparse inliers, and the dense samples the pinhole
// bar let through off the true geometry are dropped while the true ones stay
bool ROMA2ReverifyLensTest();

// The lens stage of the matching (FitLensesAndReverifyPairs): on 40 pairs of one camera verified
// through the pinhole, whose rotations come out degrees off the truth, the lens is fitted and set and
// every pair verified again through it lands within half a degree of its true rotation and, in the
// median, half a degree of its true baseline direction; a second run, the camera now carrying its
// lens, holds it and fits nothing
bool LensFitStageTest();
// The lens stage also verifies again the pairs of a camera whose focal the check moved, with no lens kept
// at it: a camera without a lens given a focal 15% short has its pairs' rotations come out within half a
// degree of the true ones
bool LensFitFocalStageTest();
// An untrusted focal fitted with its lens straight from its import focal, and its fallbacks
bool LensFitUntrustedFocalTest();

// The described/dense keypoint boundary: an image whose keypoints.size() > descriptors.rows keeps
// its stored described-keypoint count across a descriptor release and an .sfm round-trip, the
// index tests read it, and PairsMatcher::FilterRedundantKeypoints moves it through the same remap
// it applies to the keypoint indices
bool DenseKeypointBoundaryTest();

// Dense supplementation is real evidence, not noise to filter around: on a supplemented pair,
// FilterMatches' meanRayAngle and ComputeIntrinsicWeight's grid occupancy are both measured over the
// whole track-forming set (sparse + dense), the minimum-support floor reads that same set so a pair
// under the sparse-only bar still counts, and GetNumWeightedInliers() scales the dense share to the
// matcher's frame target (PairsWeightingConfig::denseFrameInliers) rather than dropping or fully
// counting it. A degenerate all-dense baseline is still demoted, because the angle term finally has
// something to measure.
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

// ROMA2Config::ResolveModelPath precedence test: an explicit modelPath wins over everything
// (including a set environment variable); with modelPath empty, $OPENMVS_ROMA2_MODEL_PATH wins;
// with both absent, the result falls back to this build's install-prefix directory when that
// directory actually exists on disk, else it is empty -- read with the same File::isFolder check
// the implementation uses, so this test does not create anything under the install prefix. Pure
// config logic, so it needs neither ONNX Runtime nor an actual model.
bool ResolveModelPathTest();

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

// PinholeReprojectionErrorAnalytic's hand-written Jacobians match the ones auto-diff derives from
// the same projection, over randomized poses, points and full 12-parameter intrinsics (the whole
// distortion model live, k3/p1/p2 and the rational denominator k4-k6 included); the quaternion
// columns are compared in the manifold's tangent space, the only place the solver sees them
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

// Pins SetBAIntrinsicFlags (ReconstructionConfig::IntrinsicFlags -> BAConfig) and
// ResectionConfig::DeriveBAConfigs, the two places the reconstruction's intrinsic flags turn
// into the bundle-adjustment switches that are actually solved
bool BAIntrinsicFlagsTest();

// The bundle adjustment hears the verified pairs: a chain bent at a joint whose only structure is
// two-view and degenerate stays bent under the reprojection residuals alone, and the relative-pose
// residuals of the pairs across the joint (BAConfig::relativeRotationSigma /
// relativeTranslationSigma) put it back, in the global solve and in a local window alike
bool BundleAdjustmentPairConstraintTest();

// The per-image cap on the observations a solve takes does not move the solution: on a scene whose
// images carry far more of them than the cap allows, and which holds both the tracks the cap must
// top back up and the ones it may let go entirely, the capped solve recovers the poses the uncapped
// one recovers, dense-matched and described-only alike; below the threshold the cap applies from,
// the solve fits every observation
bool BAObservationCapTest();

// A bundle adjustment large enough to be refined by inner iterations ends at the same bits when it is
// solved twice from the same start, the second time with the small blocks it allocates laid out in
// the reverse order
bool BAInnerIterationOrderTest();

// An image rich in described observations weighs its dense ones less: a bias shared by every dense
// keypoint of one such image moves the model at most half as far as under a uniform dense weight
bool BADenseWeightPerImageTest();

// No image's dense residuals weigh more than BAConfig::maxDenseObservationWeight: a bias shared by
// every dense keypoint of one image moves the model at most half as far as at the measured weight
bool BADenseWeightCeilingTest();

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
// An EXIF 35 mm-equivalent focal seeds the camera's focal untrusted (Image::LoadMetadata)
bool ExifFocalUntrustedTest();
// The camera model an import gives every image: read from the metadata by default, forced spherical or
// pinhole by ImportConfig::cameraType (an equirectangular panorama that carries no projection tag)
bool CameraTypeImportTest();

// Pose-frame detection: recover both the camera-axes convention and, for EXIF-rotated images,
// the in-plane rotation of an imported frames.json from the matched pairs
bool FramesPoseFrameDetectionTest();

// Known-pose import, pair selection, and prior-frame alignment tests
bool KnownPosesImportTest();
bool KnownPosePairSelectionTest();
bool AlignToPriorPosesTest();
bool AlignToPriorPosesCollinearTest();

// PairsMatcher::AddSequentialPairs: the sequential prior added to the VOCABULARY/RETRIEVAL
// candidates (MatchConfig::matchSequenceOverlap)
bool SequentialPairsPriorTest();

// Two-view geometry test: PairsMatcher and ImagePair matrix operations
bool TwoViewTest();

// Reconstruction test: Import images, extract features, match pairs, build tracks, and initialize
bool ReconstructTest(bool verbose = false);

// --export-pairs-csv is written by Scene::Reconstruct()
// right after pair matching, from both call sites (fresh import + match, and an already-matched
// .sfm given back as source), before any reconstruction step can drop pairs
bool ReconstructExportCSVTest();

// The camera-triplet view-graph filter of Manam & Govindu (CVPR 2024): the scores, the yield
// rule and both modes on hand-computed graphs.
bool TripletFilterTest();

// The star initializer's reference view is chosen among the seed views the caller names -- the
// triplet filter's largest ceiling piece -- and only among every image when none is named, or
// none of the named has a valid pair. After the filter, the seed's side of a symmetric building
// becomes the model, and the heaviest image overall sits in the densest cluster of look-alike
// views, which is the worst place to start.
bool StarReferenceViewTest();

// The star initializer lists a neighbour of the reference view by the inliers that fix its distance,
// those whose ray angle at the pair's pose reaches the reconstruction's triangulation bar, not by
// all of them: a far view that shares only far points with the reference is no arm of the star,
// whatever its inlier count, and its unit baseline guesses a distance no match backs. The reference
// stays chosen by inlier count.
bool StarEvidenceTest();

// A resected pose enters the model only when its support is credible: enough of its 2D-3D
// correspondences agree with it (ResectionConfig::minInlierRatio / minInliersAbsolute), and a
// weakly supported one also agrees with the rotation the quorum of its verified pairs to registered
// images composes -- the largest group of them agreeing on one rotation, so that one pair among
// several cannot speak for the image (ResectionConfig::maxRelativeRotationError) -- and with the
// baseline direction that quorum predicts (ResectionConfig::maxLinkDirectionError); each of the
// three alone keeps this misregistration out, and with all three off it enters the model.
bool ResectionAcceptanceTest();

// A weakly supported pose must also agree with the baseline direction its verified pairs to
// registered images predict: it is refused when the quorum links that contradict the direction
// carry more weighted inliers than the ones that agree (ResectionConfig::maxLinkDirectionError) --
// the weight they carry and not their number, so the strongest link alone outweighs two weak ones
// that agree -- each link's tolerance growing as the larger of its ray angle and the model's
// parallax shrinks, so a near-duplicate pair fixes no direction and a small-baseline pair forgives
// no more than the model's parallax of the two cameras allows (ResectionModelParallaxTest)
bool ResectionLinkDirectionTest();

// The direction a quorum link predicts is also held to the model's parallax of the link (the
// baseline the pose makes with the neighbour over the median distance of its inlier points): a
// link the model sees under 2 degrees abstains, or agrees when its pair is a near-duplicate too,
// and otherwise the larger of the pair's ray angle and the model's parallax sets the tolerance, so
// the pose under test cannot widen its own; ResectionConfig::maxLinkDirectionError 0 turns every
// direction check off and leaves the rotation check on
bool ResectionModelParallaxTest();

// The resection's next image is the candidate whose verified pairs to registered images weigh most
// (their composite weight), among every image holding ResectionConfig::minCorrespondences 2D-3D
// correspondences, whatever its share of the best correspondence count; the candidates are tried
// down that list until one registers, and the list is read again after each registration. An image
// whose evidence lies almost entirely in unregistered images is tried last and registered all the
// same, its pose right
bool ResectionWeakEvidenceTest();
bool ResectionNextImageOrderTest();
bool ResectionTriesEveryCandidateTest();

// The average inlier ratio that forces a full bundle adjustment when it falls under
// ResectionConfig::avgInliersRatioForceBA reads the registered images alone: an image whose pose is
// refused adds nothing to it, so the refusals of one candidate force no full adjustment while the
// images registered after it hold the average above the bar
bool ResectionRefusedAverageTest();

// A resection that stalls -- no image holds minCorrespondences correspondences, or every candidate
// failed -- with images registered since its last full adjustment runs that adjustment and tries
// again, and stops at a stall that follows no registration; it always closes with its full
// adjustment, even when it registered nothing since the last one
bool ResectionStallRetryTest();

// The resection's inlier bar admits a correspondence measured with the precision of a warp-sampled
// (dense) keypoint on a full-resolution image, several pixels off its projection, and not only one
// measured with a detected keypoint's: an image seen through such correspondences alone reaches a
// nearly complete inlier share, the share the resection reads its model's health from, where a 4 px
// bar counts barely half of them (ResectionConfig::ransac.threshold).
bool ResectionInlierBarTest();

// The polish that closes an image's absolute pose weighs each dense (warp-sampled) correspondence at
// ResectionConfig::denseFitWeight against a described one's 1: dense correspondences that share a
// systematic error and outnumber the described ones pull the pose of the robust fit over all of
// them, and the weighted polish brings it back to the described ones.
bool ResectionDenseFitWeightTest();
// A full adjustment fits the observations within ResectionConfig::fullBAInputErrorFactor x the reprojection bar of the
// model and runs again while some camera's input to the next solve differs enough from what it gave this one
// (AdjustRepeated): a camera whose observations all lie between the bar and 3x it comes back in one solve, which a
// single solve fed by the inlier prefix alone does not do, one whose input grows once it is back takes a second
// solve, and with ResectionConfig::fullBARepeats 0 the adjustment is one solve of the wider input
bool RepeatedFullAdjustmentTest();
// A global solve with the per-image observation cap, one camera held and no GPS, keeps the cameras' scale, so the
// tracks the cap left out of it are still where their observations see them after it
bool CappedSolveScaleTest();

// A dense (warp-sampled) observation is the less precise measurement, so the track filter and the
// triangulation hold it to its own multiple of the reprojection bar a described one answers to
// (FilterTracks / TriangulateTracks denseReprojErrorFactor).
bool DenseReprojectionBarTest();

// The image filter tells the images it removed for contradicting the model (their verified pairs to
// the kept images disagree with the pose the model gives them, by more inlier weight than agrees)
// apart from the ones it removed for want of evidence: only the former are withheld from the
// resection that follows, the latter get another chance once the model has grown.
bool ContradictingImagesTest();

// A pair's baseline direction is held to the rotation tolerance as seen from the structure the pair
// triangulates: a near-parallel pair forgives a wide direction error, a wide-baseline pair little.
bool PairDirectionToleranceTest();

// Test function for rotation estimation
bool RotationEstimatorTest();

// The contradiction cut's rotation stage: the look-alikes of a symmetric ring, the cross-face pairs of
// a roof (the tree check) and a wrong pair at the weight cap go, a bridge and a pendant pair never,
// the unscored start is the symmetric mode, and the translation stage's bar takes 0 and 180
bool ContradictionCutTest();

// The contradiction cut's translation stage: a flipped and a turned baseline go at the default bar,
// a bridge and a pair under the near-duplicate ray angle are never judged, bar 0 is the rotation
// stage alone, and a graph under a scale component's pairs removes nothing
bool TranslationCutTest();

// The flat pair-scale build equals the node-map one it replaced
bool PairScalesTest();

// Test function for global scale estimation
bool ScaleEstimatorTest();

// Test function for global translation estimation
bool TranslationEstimatorTest();

// The translation estimator's normal-equations solve factors the graph Laplacian of the pairs and
// gives the SparseQR solution of their incidence matrix
bool TranslationNormalEquationsTest();

// The scale estimator's sparse route (conjugate gradients on the normal equations, started from the
// previous solve) gives the sparse Cholesky solution, converged, and a near start saves iterations
bool ScaleConjugateGradientsTest();

// Pairs weighting test
bool PairsWeightingTest();

// A pair's dense inliers count as evidence in proportion to the matcher's frame target
// (PairsWeightingConfig::denseFrameInliers over denseMatchesPerFrame): two dense-only pairs of one
// image keep the order of their counts, a whole dense frame is a modest descriptor pair's evidence,
// and the connectivity's inlier ratio counts the dense matches at the same scale
bool PairsWeightingDenseFrameEvidenceTest();

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
bool SceneClusterTargetSizeTest();
bool SceneClusterSeamTracksTest();
bool SceneClusterLeafMergeTest();
bool SceneClusterSeamRepairTest();
bool SceneClusterMemoryProtocolTest();
bool SceneClusterIDRemappingTest();
bool SceneClusterSmallClusterRescueTest();

// A community under the floor is held to the same coupling test as an established cluster: the
// greedy merge does not absorb it over an interface carrying less than minClusterCoupling of its
// own internal weight, and the small-cluster pass then places it with the neighbour it shares the
// most weight with (ClusterConfig::minClusterCoupling)
bool SceneClusterSubFloorCouplingTest();

// A small, cohesive community no neighbour is coupled to stands alone as its own sub-scene when it
// has the strong seams the merge places a block by, and is absorbed into its heaviest neighbour as
// before when it does not (SceneCluster::IsStandaloneCommunity)
bool SceneClusterStandaloneCommunityTest();

// A cluster under the floor that IS coupled to a neighbour but that no neighbour has room for stays
// where it is; it must not come out as a sub-scene whatever position it holds in the cluster order
bool SceneClusterCoupledOrphanSkippedTest();

// The balance pass never takes an image whose departure would leave part of its own cluster coupled
// under minClusterCoupling, even when the image's own affinity favours the move
// (SceneCluster::RefineClustersBalance, SceneCluster::KeepsCoupling)
bool SceneClusterBalanceCouplingTest();

// The thin-waist check weighs every prefix cut along the Fiedler order, not only the balanced one: a
// stretch too small to stand on its own moves whole to the cluster it is most coupled to when one
// holds it at the floor, and otherwise stays; a stretch large enough to stand is split off as before
// (SceneCluster::RefineClustersSplitThinWaist)
bool SceneClusterThinStretchTest();

// Phase 3: Global Alignment tests
bool GlobalAlignmentBuildGlobalToLocalMapTest();
bool GlobalAlignmentRotationAveragingExtendedTest();
bool GlobalAlignmentScaleAveragingExtendedTest();
bool GlobalAlignmentScaleAveragingFallbackTest();
bool GlobalAlignmentTranslationAveragingExtendedTest();
bool RobustAveragingScaleTest();
bool RobustAveragingTranslationTest();
bool SeamGraphConsensusTest();
// The selection over planted block graphs: a chord heavier than any one true seam and wrong by 30
// degrees across two cycles is left off; a bridge keeps a reading only where its two directions
// agree, two cameras hold its weaker side and the verified pairs across it are not against it; the
// L1 reweighting does not follow a lighter copy that a least-squares solve would sit between; a
// reading without scale is judged on its rotation
bool HeavyWrongChordTest();
bool TwoDirectionBridgeTest();
bool SymmetricBridgePairsTest();
bool OneCameraBridgeTest();
bool L1StartTest();
bool RotationOnlyReadingTest();
bool RingPlacementTest();
bool UnobservableScaleTest();
bool UnobservableScalePairTest();
bool AmbiguousPairTest();
// A seam its cameras carry but the verified image pairs across it contradict places no block
// (GlobalAlignmentConfig::maxPairRotationResidual)
bool ContradictedPlacementTest();

// The verified pairs across a seam decide by inlier weight over all of them, not by counting pairs:
// two thin disagreeing pairs cannot outvote twenty-four heavy agreeing ones.
bool PairAgreementWeightTest();
bool RingLoopClosureTest();
bool BiasedClosingSeamTest();
bool WeakClosingSeamTest();
// A ring pair carrying the true reading and a heavier copy of one block's structure one period
// along the wall: the cycle the pair closes chooses the true one by the translation it implies
bool RivalModesOnCycleTest();
// A seam one camera holds on its side weighs one camera, however many correspondences it carries,
// and no block is placed on it
bool OneCameraSideTest();
// A one-camera door closes the triangle that chooses between a pair's true reading and a copy of
// equal weight
bool OneCameraDoorTest();
// The correspondences a ring of bent blocks leaves past the bar and within the loose bar join their
// tracks at the merge and end inliers within the bar after the final adjustment
bool BentBlockClosureTest();
bool SecondModelTest();
bool FoldedBlockTest();
// A ring of blocks bent half a degree each keeps every seam between neighbours and ends on the truth
bool BentRingKeepsSeamsTest();
// A block folded by 45 degrees does not put the rest of the model wrong
bool FoldedBlock45Test();
bool FalseChordTest();
bool SeamInliersBecomeTracksTest();
bool UnplacedBlocksReportedTest();
bool BentBlocksRelaxationTest();
bool GlobalAlignmentMergeSingleSceneTest();
bool GlobalAlignmentTrackMergeDuplicateImageGuardTest();
bool GlobalAlignmentTrackMerge3DProximityGuardTest();
bool HierarchicalCameraAlignmentTest();

// End-to-end hierarchical SFM tests
bool HierarchicalSFMSplitMergeRoundtripTest();
bool HierarchicalSFMWithRandomTransformTest();
/*----------------------------------------------------------------*/

} // namespace SFM
