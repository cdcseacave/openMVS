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

// TestsSFMImport.cpp ------------------------------------------

#ifdef _IMAGE_HEIF
// HEIF/HEIC integration at the SFM layer (the reader itself is covered by CImageHEIF::Test):
// the decoded resolution the MVS camera is paired against, the EXIF metadata bridge (focal
// length agreeing with the classic stream scan, GPS), the "don't rotate twice" orientation
// guard for a container 'irot' that duplicates an EXIF Orientation, and the LoadPixels
// fallback for a format cv::imread cannot decode -- in both color and gray, the latter being
// what feature extraction uses. Pixel content as a whole is covered by RealImagesPipelineTest and the
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
// pinhole by ImportConfig::cameraType (an equirectangular panorama that carries no projection tag); and
// a spherical import of an image that is not 2:1 logs an error naming the image and leaves it without a
// camera, like any image whose metadata fails
bool CameraTypeImportTest();

// The import of external poses (frames.json, pose CSV: names, intrinsics, duplicates, an invalid row
// changing nothing) and the pose-frame detection that recovers both the camera-axes convention and, for
// EXIF-rotated images, the in-plane rotation of an imported frames.json from the matched pairs
bool KnownPosesImportTest();

// TestsSFMFeatures.cpp ----------------------------------------

// VocabularyTree save/load roundtrip test
bool VocabularyTreeTest();

// The track builder on a component holding one image twice (BuildTracks, TrackConflictConfig): the
// cut removes the least-supported link on the path between the two keypoints and keeps the
// corroborated side where the pair-order veto keeps whichever arrived first, two keypoints within
// the merge distance are one feature and the one with more links stays, a component above the size
// bar takes the veto, the cut can be switched off, and a conflict-free scene is untouched
bool BuildTracksConflictTest();

// Global-descriptor retrieval on twelve images whose local and global descriptors cluster them in two
// different partitions: the cosine ranking and its deterministic tie order, the RETRIEVAL match mode
// (a connected view graph, a missing descriptor a hard error, end-to-end through Match()), VOCABULARY
// following the local clustering where RETRIEVAL follows the global one (neither borrows the other's
// backend), and the .sfm round-trip of the descriptors
bool RetrievalTest();

// Spherical bottom band of the feature extraction: with FeatureExtractionConfig::sphericalBottomBand
// at 0.2 no keypoint of an equirectangular image lies in its bottom 20% of rows and the rest of the
// image keeps its features, at 0 the band has features; a perspective image extracts the same
// keypoints and descriptors with the band at 0.2 or at 0.
bool SphericalBottomBandTest();

// The detector words of FeatureTypeFromString (AKAZE, ORB, SIFT and SIFTGPU in any letter case, any other word is NONE)
// and the one default detector the library and the application share, with the thresholds of each detector
bool DetectorConfigTest();

// The relative pose of two panoramas through PairsMatcher::MatchPair (bearing vectors) and MatchFeaturesGeometric
// (angular epipolar filter, no F), back-hemisphere bearings admitted.
bool SphericalPairGeometryTest();

// The four ways a candidate pair is made or pruned: known poses with an unposed image, the sequential prior
// (PairsMatcher::AddSequentialPairs), the SEQUENTIAL match mode, and the pre-match threshold
bool PairSelectionTest();

// Two-view geometry: PairsMatcher and ImagePair matrix operations, the two-view calibration refinement, and the
// relative pose recovered from the fundamental matrix (GeometricFilter's F branch, then DecomposeFundamentalToPose)
// for two cameras of different intrinsics and image sizes as for two of one size
bool TwoViewGeometryTest();

// The pipeline on the bundled images, each stage from the state the one before left: the refusals before any work
// (SKIP without matches, a GPS weight with panoramas), the match-images-only run writing --export-pairs-csv right
// after the matching (from the images and from the saved .sfm), --match-mode -1 (SKIP) from the saved matches,
// --use-global-solver (Scene::ReconstructGlobal), and the incremental reconstruction recovering wrong intrinsics
bool RealImagesPipelineTest(bool verbose = false);

// Scene::MatchPairs() fails the stage when matching two or more images leaves the view graph empty, and
// PairsMatcher::Match()'s fatal-round signal is not "this round stored nothing"
bool MatchOutcomeSignalsTest();

// TestsSFMDense.cpp -------------------------------------------

// The warp grid's pixel <-> cell convention (align_corners=False cell centres) end to end: keypoint
// tracking, the draws' cell positions, the occupancy rounding and the verdict's cells
bool DenseWarpGridConventionTest();

// ROMA2 warp helpers test: keypoint tracking through an identity warp (the
// pixel<->grid<->normalized coordinate conventions), the confidence gate, and the dense append
bool DenseWarpTrackingTest();

// Coverage-uniform warp sampling, the sample the verdict fits its geometry on: the sample budget,
// the spread the bucket stratification buys over a plain top-confidence selection, the coverage a
// genuinely one-sided sample reports, and the determinism of the draw
bool DenseCoverageSampleTest();

// The complementary dense draw, the fill of an admitted pair: it is drawn only where that pair's
// guided sparse matches are NOT, capped at the pair's dense budget, thinned when over budget by the
// pair-independent lattice key rather than by confidence -- so a tighter budget keeps a subset of
// what a looser one keeps -- identical across two pairs sharing an image, and deterministic
bool DenseComplementaryDrawTest();

// The dense fill's bucket pitch (DenseFillGridSide): a function of the density and the warp side
// alone, not of the pair, so the draw it feeds is a density over the overlap the sparse matches did
// not cover rather than a fixed count per pair -- a larger confident region draws proportionally
// more, an occupied bucket still yields nothing so the sparse share comes out of the same draw, and
// two pairs sharing an image stratify it on the same grid and so agree on the pixels they sample,
// an agreement that survives the one term of the draw that IS per-pair: two pairs whose ceilings
// thin that agreement to different sizes keep nested samples of it rather than divergent ones
bool DenseFillDensityTest();

// The dense fill ceiling (DenseFillCeiling): the configured density over the SMALLER of the
// verdict's two inlier areas, symmetric in A and B, and linear in the density knob
bool DenseFillCeilingTest();

// The pair verdict (JudgePairDense) on the exact bidirectional warp of two pinhole cameras looking
// at a non-planar surface: a warp confident over ~30% of both frames is admitted with both inlier
// areas measuring that share, a warp confident over 30% of A whose B side maps into A over only 3%
// is rejected by the min side alone, a smooth warp unrelated to the cameras (a homography of A's
// grid) is rejected however exactly one geometry explains its own side -- through the calibrated
// branch and, with forceFundamental, through the 7-DoF one the min-side rule was designed for --
// and minOverlap 0 admits the first two
bool DenseVerdictTest();

// Guided sparse matching (MatchFeaturesGuided): the ratio against the best descriptor outside the
// search disc (a lookalike elsewhere refuses, a scale duplicate inside is no rival), the epipolar
// band (a rival off the line is no candidate, one on it refuses, a same-feature duplicate is no
// rival), the fallback to the disc without a geometry or near the epipole, the same matches twice
bool DenseGuidedMatchTest();

// Pair assembly and storage (AssemblePairDense, StorePairDense): the union of the guided matches
// and the dense fill is fitted once and splits into the pair's sparse and dense segments under one
// relative pose, a pair with no guided match is still assembled as a dense-only pair, a fill too
// small to fit leaves the verdict's geometry untouched, and the store appends the dense keypoints
// past each image's described prefix with reproducible indices, inserting the dense block ahead of
// a pair's rejected tail rather than past it
bool DenseAssemblyTest();

// The weighted refinement that closes a pair's assembly: with the dense fill displaced 2 px across
// the epipolar lines and the guided matches exact, the pose follows the sparse matches at
// DenseMatchConfig::denseFitWeight and the dense bias at full weight
bool DenseFitWeightTest();

// The planar twin switch (SwitchPlanarTwinsDense): a floor pair stored at its wrong planar twin
// switches to the true pose its neighbours' rotation loops close with (not with the switch off or a
// margin above its gain), a pair with 40% of its matches off the floor is untouched (under the plane
// gate), and of two wrong twins sharing a triangle the second switches once the first has (best
// gain first, where one pass on the stored poses would leave it)
bool PlanarTwinSwitchTest();

// The re-verification a camera's change sets off (PairsMatcher::ComputeRelativePoses) keeps a RoMa2
// pair's partition: on the wedge, with nothing about the cameras changed, the pair comes back with
// the same sparse inliers, the same dense segment of dense keypoints and the same outliers, a pair
// whose dense fill drew nothing comes back well formed too, and so does a pair whose matches all sit
// among its outliers, its dense matches back in the dense segment; a pair the re-fit refuses (stored
// without a geometry, or under the minimum of matches with its geometry kept) comes back invalidated,
// with the matcher's strict filter and without it; under the default configuration, with the dense
// fill biased across the epipolar lines, the re-verified pose follows the sparse matches where
// GeometricFilter at full weight follows the bias
bool DenseReverifyPartitionTest();

// The described/dense keypoint boundary: an image whose keypoints.size() > descriptors.rows keeps
// its stored described-keypoint count across a descriptor release and an .sfm round-trip, the
// index tests read it, and PairsMatcher::FilterRedundantKeypoints moves it through the same remap
// it applies to the keypoint indices
bool DenseKeypointBoundaryTest();

// RoMa2 CPU preprocessing test: a constant image maps to constant planes with the expected
// R/G/B channel swap, and resampling a real fixture image reproduces torch's own
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) to within 1e-5
bool RoMa2PreprocessTest();

// RoMa2 manifest format-version test: a manifest declaring format_version 1 loads, and one
// declaring format_version 3 is rejected. Pure JSON parsing, so it needs neither ONNX Runtime
// nor OPENMVS_ROMA2_MODEL_PATH.
bool RoMa2ManifestVersionTest();

// FlexGlue manifest test: a format_version 1 manifest with its two graphs and the H1 pose head loads with its sizes; another
// version, another model, a missing key and another pose head are rejected. Pure JSON parsing, needs no model.
bool FlexGlueManifestTest();

// DenseMatchConfig::ResolveModelPath precedence test, for each matcher (roma2, flexglue): its explicit path wins over everything
// (including a set environment variable); with it empty, its $OPENMVS_<MATCHER>_MODEL_PATH wins; with both absent, the result
// falls back to this build's install-prefix directory share/openMVS/<matcher> when that directory actually exists on disk, else
// it is empty -- read with the same File::isFolder check the implementation uses, so this test does not create anything under the
// install prefix; and neither matcher ever resolves to the other one's path or variable. Pure config logic, so it needs neither
// ONNX Runtime nor an actual model.
bool ResolveModelPathTest();

// RoMa2 ONNX parity test: runs the exported descriptor and coarse-match graphs through
// RoMa2Onnx and compares them to the Python reference dumps shipped with the models.
// Skipped unless OPENMVS_ROMA2_MODEL_PATH points at an exported model folder;
// OPENMVS_ROMA2_PROVIDER (auto|cuda|coreml|dml|cpu) and OPENMVS_ROMA2_SETTING (turbo|fast|base)
// narrow the execution provider and the preset it exercises
bool RoMa2OnnxParityTest();

// The dense matchers (RoMa2, FlexGlue) in the import/AKAZE/RETRIEVAL pipeline on the bundled images, each half skipped unless
// its model variable (OPENMVS_ROMA2_MODEL_PATH, OPENMVS_FLEXGLUE_MODEL_PATH) is set: retrieval descriptors, every pair verified,
// RoMa2's one pass against the descriptor batch with its round trip and determinism, FlexGlue's covisibility, pose prior, CSV
// and gate, and the reconstruction stage of RealImagesPipelineTest on each
bool DenseMatcherReconstructTest();

// Spherical bottom band of the dense matching: with DenseMatchConfig::sphericalBottomBand at 0.2 neither warp draw
// (the verdict's sample and the fill) has an end of a sample in the bottom 20% of rows of a spherical image,
// the rows above keep the samples they have without the band, and a perspective pair is unaffected.
bool SphericalBottomBandDenseTest();

// TestsSFMCalibration.cpp -------------------------------------

// The inlier tests of a RoMa2 pair through a lens (SampsonTest): on two views through a barrel lens
// the camera knows, every guided match on the true geometry is a sparse inlier, the corner ones
// included, while the pixel fundamental matrix of the same geometry, composed without the lens, puts
// the corner ones past the matcher's epipolar bar
bool DenseLensInlierTest();

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

// A RoMa2 pair matched through the pinhole and re-verified once its camera knows its lens
// (ReverifyPairDense): with the pinhole geometry of the true pose kept at matching, the guided matches
// at the corners the pinhole rejected come back as sparse inliers, and the dense samples the pinhole
// bar let through off the true geometry are dropped while the true ones stay
bool DenseReverifyLensTest();

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

// View graph calibrator test: Refine focal length using view graph optimization
bool ViewGraphCalibratorTest();

// TestsSFMViewGraph.cpp ---------------------------------------

// The camera-triplet view-graph filter of Manam & Govindu (CVPR 2024): the scores, the yield
// rule and both modes on hand-computed graphs.
bool TripletFilterTest();

// The pair weights (ComputePairsWeights, ImagePair::GetCompositeWeight): the spatial, connectivity and triplet terms of
// hand-built pairs, the composite formula term by term, the dense inliers counted in proportion to the matcher's frame target
// (PairsWeightingConfig::denseFrameInliers over denseMatchesPerFrame), and the dense supplement of a pair as real evidence:
// FilterMatches' meanRayAngle and the grid occupancy are measured over the whole track-forming set (sparse + dense), the
// minimum-support floor reads that same set, GetNumWeightedInliers() scales the dense share, and a degenerate all-dense
// baseline is still demoted
bool PairWeightTest();

// The rotation estimator on the 16-camera ring, and its independence of the absolute scale of the pair weights
// (GlobalRotationEstimatorOptions::weightMedian): the same rotations and the same pairs past the bar at weights x1, x1/128, x128
bool RotationAveragingTest();

// The contradiction cut: its rotation stage (the look-alikes of a symmetric ring, the cross-face pairs
// of a roof and a wrong pair at the weight cap go, a bridge and a pendant pair never, the unscored
// start is the symmetric mode, the translation stage's bar takes 0 and 180) and its translation stage
// (a flipped and a turned baseline go at the default bar, a bridge and a pair under the near-duplicate
// ray angle are never judged, bar 0 is the rotation stage alone, the repair, a graph under a scale
// component's pairs removes nothing, two components solved apart)
bool ContradictionCutTest();

// A matched scene is filtered once (the triplet filter, the contradiction cut and its repair), on load as on a fresh
// run, and a scene saved after the filters is reloaded with its pairs, repair marks and seed views and not filtered again
bool FilteredSceneReloadTest();

// The flat pair-scale build equals the node-map one it replaced
bool PairScalesTest();

// The scale estimator solves every connected component of its equations on its own: the one holding the
// fixed index (given, or the best-connected index when none is, through the 3-argument overload too) pinned at it, any other at
// mean zero log scale (the dense SVD's minimum-norm answer), an index no equation names at scale 1
bool ScaleAveragingTest();

// The translation estimator's normal-equations solve factors the graph Laplacian of the pairs and gives the SparseQR solution
// of their incidence matrix (connected, bridged, two components, floating rings, the no-gauge 3-argument call, a view-graph size)
bool TranslationAveragingTest();

// The scale estimator's solve (conjugate gradients on the normal equations, started from the previous
// solve) gives the sparse Cholesky solution of the gauge-pinned system, converged, and a near start saves iterations
bool ScaleConjugateGradientsTest();

// The robust averaging ignores a wrong chord, in five scales and in five translations on a ring
bool RobustAveragingTest();

// TestsSFMCluster.cpp -----------------------------------------

// Phase 1: Scene Clustering tests
// The clustering into sub-scenes: a single one within the view limit and with the limit off, the size constraints,
// disconnected groups never mixed, the memory protocol (keypoints moved, cross-pairs left) and the ID remapping
bool SceneClusterBasicsTest();

// Clustering for the merge on chains of images: the sub-scene sizes aimed at the target with a strong seam between every
// two that touch, the seam-usable tracks counted as the merge counts them, a weakly joined tail merged into the last sub-scene
bool SceneClusterSeamTest();

// The small-cluster pass: weak images rescued, a community under the floor held to the coupling test
// (ClusterConfig::minClusterCoupling) and placed with the neighbour it shares the most weight with,
// standing alone with a strong seam and no coupled neighbour (SceneCluster::IsStandaloneCommunity),
// and skipped when coupled to a cluster with no room for it
bool SceneClusterSmallClusterPassTest();

// The refinement passes on hand-made graphs: the balance pass never takes an image whose departure would leave part of its
// own cluster coupled under minClusterCoupling (SceneCluster::RefineClustersBalance, SceneCluster::KeepsCoupling); the
// thin-waist check weighs every prefix cut along the Fiedler order: a stretch too small to stand on its own moves whole to
// the cluster that holds it at the floor, otherwise stays, and one large enough to stand is split off
// (SceneCluster::RefineClustersSplitThinWaist)
bool SceneClusterRefinePassesTest();

// TestsSFMBlock.cpp -------------------------------------------

// Triplet star-initialization test: 3-view scene with tracks + StarInitializer + BA
bool TripletStarInitTest();

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

bool StarScaleGaugeTest();

bool NearDuplicateYardstickTest();

// A resected pose enters the model only when its support is credible: enough of its 2D-3D correspondences agree
// with it (ResectionConfig::minInlierRatio), and a weakly supported one also agrees with the rotation the quorum of
// its verified pairs to registered images composes (maxRelativeRotationError: one pair among several cannot speak for
// the image), with the baseline direction each link predicts, weighed in weighted inliers with the pose's own inliers
// voting for it and forgiven by the larger of the pair's ray angle and the model's parallax (maxLinkDirectionError),
// and with the rotation all its links predict, weighted (maxLinkRotationDisagreement); one pair's direction tolerance
// follows its ray angle (MeasurePairDisagreement)
bool ResectionLinkChecksTest();

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

// The full adjustment that ends a resection runs whether or not the resection had an image to try
// -- every image holds a pose, or the only ones without are withheld by ImagesRemoved -- and brings
// back the tracks the final adjustment's tighter bar dropped
bool ResectionClosingAlwaysTest();

// Images a filter removed right before a resection count as a change of the model since its last
// full adjustment: a stall that follows runs the adjustment and another pass, which registers the
// image the re-triangulated tracks hold; a stall with nothing removed or registered ends the resection
bool ResectionRemovalStallTest();

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

bool ResectionWeightedInlierBarTest();

// The shared walks: an image's 2D-3D correspondences to a model's inlier tracks (its first keypoint on
// each, in track order), its observation counts, and the links of a model (its registered pairs carrying
// a relative pose) seen from their other side, strongest first
bool LinkWalksTest();

// An image is tried for resection only once it has a link (a verified pair with a relative pose to a
// registered image): one whose pairs with a relative pose go to unregistered images waits until one
// of them is registered, however many 2D-3D correspondences it holds; the candidates are tried in the order
// of their links' weight, a pair without a relative pose counting for nothing
bool ResectionLinkGateTest();

// The image filter tells the images it removed for contradicting the model (their verified pairs to
// the kept images disagree with the pose the model gives them, by more inlier weight than agrees)
// apart from the ones it removed for want of evidence: only the former are withheld from the
// resection that follows, the latter get another chance once the model has grown.
bool ContradictingImagesTest();

bool SceneBlocksInsertReleaseTest();

bool SceneBlocksInsertRebuildTest();

bool PoolResectionWrongClusterTest();

// A pool image is offered only to the blocks it has links in (verified pairs with a relative pose to their
// registered images): pairs without a relative pose, however heavy, do not send it to a block
bool PoolResectionLinkOnlyTest();

// TestsSFMBundleAdjustment.cpp --------------------------------

// The weights a bundle adjustment gives dense observations: ComputeObservationSigmas reads the bulk of a scene's described
// and dense displacements (a median), EstimateDenseObservationWeight returns 1/k^2 of their ratio (clamped to 1, floored,
// the configured constant when the dense sample is absent or too small), and the per-image scaling and the ceiling each at
// least halve the rotation error a bias shared by one image's dense keypoints costs under a uniform dense weight
bool BADenseWeightTest();

// Pins SetBAIntrinsicFlags (ReconstructionConfig::IntrinsicFlags -> BAConfig) and
// ResectionConfig::DeriveBAConfigs, the two places the reconstruction's intrinsic flags turn
// into the bundle-adjustment switches that are actually solved
bool BAIntrinsicFlagsTest();

// Full-hemisphere spherical reconstruction regression test: exercises the Triangulation + BA pipeline on a spherical scene
// with 3D points across the entire sphere (front AND back hemispheres), pins the equirectangular convention against the
// left-right mirror and checks the handedness of the reconstruction
bool ReconstructSphericalSyntheticTest();

// The smoke tests of the bundle adjustment on synthetic scenes: the plain and the spherical solve, the refined focal length
// and radial distortion, and Scene::Transform preserving the projections
bool BundleAdjustmentBasicsTest();

// The pose uncertainty a solve records: the GPS-prior covariance (datum-free, agreeing with Ceres' estimator, missing
// accuracy tags tolerated) and the quality report round trip (CSV, .mvs image IDs, Scene::Transform, .sfm)
bool PoseUncertaintyTest();

// A full adjustment fits the observations within ResectionConfig::fullBAInputErrorFactor x the reprojection bar of the
// model and runs again while some camera's input to the next solve differs enough from what it gave this one
// (AdjustRepeated): a camera whose observations all lie between the bar and 3x it comes back in one solve, which a
// single solve fed by the inlier prefix alone does not do, one whose input grows once it is back takes a second
// solve, with ResectionConfig::fullBARepeats 0 the adjustment is one solve of the wider input, and a solve stopped by
// its cap triangulates the tracks without a point again
bool RepeatedFullAdjustmentTest();

// The track filter and the triangulation on the arc: a dense observation is held to its own multiple of the reprojection
// bar, a track with a single posed view (the star's reference at the origin, default position) is not judged, and a track
// of two posed views never triangulated holds the improbable default position and is rejected
bool FilterTracksTest();

// A triangulation that fails (TriangulateSkewLLS, TriangulateDLT, and so TriangulateTracks), for the minimum angle
// or for want of a second inlier, returns 0 and leaves the track with no inliers, and the next TriangulateTracks of
// the tracks without inliers takes it up again; TriangulateSkewLLS on panoramas determines a point whose bearing
// lies across the optical axis (dz near 0), which the two rows of [d]x R it once kept did not.
bool TriangulationTest();

// The bundle adjustment hears the verified pairs: a chain bent at a joint whose only structure is
// two-view and degenerate stays bent under the reprojection residuals alone, and the relative-pose
// residuals of the pairs across the joint (BAConfig::relativeRotationSigma /
// relativeTranslationSigma) put it back, in the global solve and in a local window alike
bool BundleAdjustmentPairConstraintTest();

// The reprojection error a budgeted solve reports is the mean over the observations it used: on a scene whose
// image 0 is left out of the solve, with one of its observations a million pixels off, the mean over the others
// is the 0.5 px they carry, and the unfiltered mean is dominated by the left-out outlier
bool ReprojectionReportOverUsedTest();

// The per-image cap on the observations a solve takes does not move the solution (dense-matched and described-only scenes,
// and a scene under the threshold fits every observation), an image over the budget takes back the wide view it would lose,
// and a capped global solve with one camera held and no GPS keeps the cameras' scale
bool BAObservationCapTest();

// A bundle adjustment large enough to be refined by inner iterations ends at the same bits when it is
// solved twice from the same start, the second time with the small blocks it allocates laid out in
// the reverse order
bool BAInnerIterationOrderTest();

// TestsSFMMerge.cpp -------------------------------------------

// Phase 3: Global Alignment tests

bool SeamGraphConsensusTest();

// The selection over planted block graphs: a reading weighs the supporting cameras of its weaker side
// times its verified pairs' share; a chord heavier than any one true seam and wrong by 30 degrees
// across two cycles is left off; of the two directions of a block pair no cycle tests (a bridge), the
// one turned past the rotation bar is cut; a reading whose weaker side is one camera does not enter,
// nor one the verified pairs across it are against; the inverse-error reweighting does not follow a
// lighter copy that a least-squares solve would sit between; a reading cut by one round's averaging
// is judged again in the next
bool ReadingWeightCamerasTest();

bool HeavyWrongChordTest();

bool TwoDirectionBridgeTest();

bool SymmetricBridgePairsTest();

bool OneCameraReadingTest();

bool ReweightingStartTest();

bool BarsReEntryTest();

bool RingPlacementTest();

// The cross-block track join: seam matches first, every match on the duplicate-image and length guards, the merged
// model's adjustment keeping the right observations
bool CrossBlockJoinTest();

bool MergeJoinReprojectionTest();

bool MergeTrackLengthTest();

bool PooledSeamReadingTest();

// The merge in rounds: a block kept whole, the regrouping around the merged model, the release (driven by hand); the
// merge's own loop (SceneBlocks::MergeInRounds) releasing at the fixed point; a seam through pool-registered cameras
// alone refused; from round 2 on the merged model is the one holding block 0, whatever the other models weigh
bool MergeRoundsTest();

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

// A direction seen by one camera gives no reading; a block joined by single cameras only is released
bool OneCameraDirectionTest();

bool RigScaleTwoCamerasTest();

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

bool BentBlocksAdjustmentTest();

bool GlobalAlignmentTrackMergeDuplicateImageGuardTest();

bool GlobalAlignmentTrackMerge3DProximityGuardTest();

bool HierarchicalCameraAlignmentTest();

// A scene split into blocks at the true poses and merged back, in the true frame and moved by random similarities
bool MergeRoundtripTest();

// TestsSFMPipeline.cpp ----------------------------------------

// The alignment of a scene onto external positions: EstimateSimilarityTransform recovers a known similarity (and
// refuses coincident destinations), Scene::AlignToPriorPoses realigns a circle and a collinear capture,
// Scene::AlignToGPS rejects coincident and collinear fixes leaving the scene untouched and aligns spread ones
bool SceneAlignmentTest();

// The cube-map bridge of SFM::ExportMVS: a panorama rendered into 6 pinhole faces, and one export read back as the
// MVS::Interface (rig platform, face cameras, wiring, vertex views), as face files and as an MVS::Scene, plus the
// mixed pinhole and spherical export and the 4-face export that drops the zenith and nadir tracks
bool CubeMapExportTest();

/*----------------------------------------------------------------*/

} // namespace SFM
