/*
 * Tests.cpp
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
#include "../../libs/MVS.h"
#include "../../libs/Math/LeastAbsoluteDeviationSolver.h"
#include "../../libs/Math/ConfidenceInterval.h"
#include "TestsMath.h"
#include "TestsSFM.h"
#include "TestsMVS.h"
#include "Tests.h"
#include <atomic>
#include <functional>
#include <new>


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("Tests")


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Test    "));

// The pool of ScopedDescendingHeap and the operator new that hands it out: constant-initialized, so it
// is in place before any allocation of the program
namespace {
constexpr size_t DESCENDING_HEAP_ALIGNMENT = alignof(std::max_align_t);
char* descendingHeapBegin = nullptr;
char* descendingHeapEnd = nullptr;
std::atomic<char*> descendingHeapTop{nullptr};
std::atomic<bool> descendingHeapActive{false};
} // namespace

ScopedDescendingHeap::ScopedDescendingHeap()
{
	ASSERT(!descendingHeapActive.load());
	if (descendingHeapBegin == nullptr) {
		// with no memory for the pool every request goes to the allocator, and the scope changes nothing
		descendingHeapBegin = static_cast<char*>(std::malloc(POOL_SIZE));
		if (descendingHeapBegin != nullptr) {
			descendingHeapEnd = descendingHeapBegin + POOL_SIZE;
			descendingHeapTop.store(descendingHeapEnd);
		}
	}
	startTop = descendingHeapTop.load();
	descendingHeapActive.store(true);
}
ScopedDescendingHeap::~ScopedDescendingHeap()
{
	descendingHeapActive.store(false);
}
size_t ScopedDescendingHeap::BytesHandedOut() const
{
	return (size_t)(startTop - descendingHeapTop.load());
}

void* operator new(std::size_t size)
{
	if (descendingHeapActive.load(std::memory_order_relaxed) && size <= ScopedDescendingHeap::MAX_BLOCK) {
		const std::size_t bytes = (MAXF(size, (std::size_t)1) + DESCENDING_HEAP_ALIGNMENT - 1) & ~(DESCENDING_HEAP_ALIGNMENT - 1);
		char* top = descendingHeapTop.load();
		while ((std::size_t)(top - descendingHeapBegin) >= bytes) {
			if (descendingHeapTop.compare_exchange_weak(top, top - bytes))
				return top - bytes;
		}
	}
	for (;;) {
		if (void* const ptr = std::malloc(MAXF(size, (std::size_t)1)))
			return ptr;
		const std::new_handler handler = std::get_new_handler();
		if (handler == nullptr)
			throw std::bad_alloc();
		handler();
	}
}
// GCC inlines this delete where a new-expression's block is released and reads the free() below as
// releasing operator new's memory with the wrong function; the operator new above takes it from malloc
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
void operator delete(void* ptr) noexcept
{
	const std::uintptr_t address = reinterpret_cast<std::uintptr_t>(ptr);
	if (address >= reinterpret_cast<std::uintptr_t>(descendingHeapBegin) && address < reinterpret_cast<std::uintptr_t>(descendingHeapEnd))
		return; // a pooled block: the pool is never given back
	std::free(ptr);
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
void operator delete(void* ptr, std::size_t) noexcept
{
	operator delete(ptr);
}

// test various algorithms independently
bool UnitTests()
{
	TD_TIMER_START();

	if (!SEACAVE::cListTest<true>(100)) {
		VERBOSE("ERROR: cListTest failed!");
		return false;
	}
	if (!SEACAVE::OctreeTest<double, 2>(100)) {
		VERBOSE("ERROR: OctreeTest<double,2> failed!");
		return false;
	}
	if (!SEACAVE::OctreeTest<float, 3>(100)) {
		VERBOSE("ERROR: OctreeTest<float,3> failed!");
		return false;
	}
	if (!SEACAVE::OctreeLODTest<double, 2>(100)) {
		VERBOSE("ERROR: OctreeLODTest<double,2> failed!");
		return false;
	}
	if (!SEACAVE::OctreeLODTest<float, 3>(100)) {
		VERBOSE("ERROR: OctreeLODTest<float,3> failed!");
		return false;
	}
	if (!SEACAVE::TestRayTriangleIntersection<float>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<float> failed!");
		return false;
	}
	if (!SEACAVE::TestRayTriangleIntersection<double>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<double> failed!");
		return false;
	}
	if (!SEACAVE::TestLeastAbsoluteDeviationSolver()) {
		VERBOSE("ERROR: TestLeastAbsoluteDeviationSolver failed!");
		return false;
	}
	if (!SEACAVE::TestConfidenceInterval()) {
		VERBOSE("ERROR: TestConfidenceInterval failed!");
		return false;
	}
	if (!SEACAVE::TestTetraFlow()) {
		VERBOSE("ERROR: TestTetraFlow failed!");
		return false;
	}
	if (!SEACAVE::TestLBPPotts()) {
		VERBOSE("ERROR: TestLBPPotts failed!");
		return false;
	}
	if (Util::toString(L"\x00A9\U0001F600") != "\xC2\xA9\xF0\x9F\x98\x80") {
		VERBOSE("ERROR: wide string UTF-8 conversion failed!");
		return false;
	}
	if (!MVS::MeshVertexColorsPLYTest()) {
		VERBOSE("ERROR: MeshVertexColorsPLYTest failed!");
		return false;
	}
	if (!MVS::PLYCodecTest()) {
		VERBOSE("ERROR: PLYCodecTest failed!");
		return false;
	}
	if (!MVS::MeshBipyramidFixtureTest()) {
		VERBOSE("ERROR: MeshBipyramidFixtureTest failed!");
		return false;
	}
	if (!MVS::MeshTetraInteriorPointFixtureTest()) {
		VERBOSE("ERROR: MeshTetraInteriorPointFixtureTest failed!");
		return false;
	}
	if (!MVS::MeshHalfMeshProcessingTest()) {
		VERBOSE("ERROR: MeshHalfMeshProcessingTest failed!");
		return false;
	}
	if (!MVS::PlatformArrGrowOwnershipTest()) {
		VERBOSE("ERROR: PlatformArrGrowOwnershipTest failed!");
		return false;
	}
	if (!MVS::MeshRefineStepTest()) {
		VERBOSE("ERROR: MeshRefineStepTest failed!");
		return false;
	}
	if (!MVS::MeshRefineWindowStatsTest()) {
		VERBOSE("ERROR: MeshRefineWindowStatsTest failed!");
		return false;
	}
	if (!MVS::MeshCleanPerVertexTest()) {
		VERBOSE("ERROR: MeshCleanPerVertexTest failed!");
		return false;
	}
	if (!MVS::ImageResizeCameraTest()) {
		VERBOSE("ERROR: ImageResizeCameraTest failed!");
		return false;
	}
	#ifdef _IMAGE_HEIF
	// the reader's own semantics are tested next to the reader, in libs/IO/ImageHEIF.cpp
	if (!CImageHEIF::Test(MAKE_PATH("images"))) {
		VERBOSE("ERROR: CImageHEIF::Test failed!");
		return false;
	}
	if (!SFM::HEIFMetadataTest()) {
		VERBOSE("ERROR: HEIFMetadataTest failed!");
		return false;
	}
	#endif
	#ifdef _IMAGE_TIFF
	// the TIFF codec's own semantics (header parsing, channel order, read/write roundtrip,
	// OpenCV compatibility) are tested in libs/IO/ImageTIFF.cpp
	{
		const ScopedTempDir tmpDir(_T("ImageTIFF"));
		if (!tmpDir.IsValid() || !CImageTIFF::Test(tmpDir.Path())) {
			VERBOSE("ERROR: CImageTIFF::Test failed!");
			return false;
		}
	}
	#endif
	VERBOSE("All unit tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// test OpenMVS functionality




// one entry of the table of SFM tests that main() walks in order
#define SFM_TEST(name) {#name, SFM::name}

int main(int argc, LPCTSTR* argv)
{
	// Flush stdout/stderr per write so CI logs aren't lost on SIGKILL.
	// MSVC's ucrtbase rejects (buf=NULL, size=0) with mode!=_IONBF as an invalid
	// parameter (fatal), and treats _IOLBF as _IOFBF anyway — so use _IONBF there.
	#ifdef _MSC_VER
	std::setvbuf(stdout, NULL, _IONBF, 0);
	std::setvbuf(stderr, NULL, _IONBF, 0);
	#else
	std::setvbuf(stdout, NULL, _IOLBF, 0);
	std::setvbuf(stderr, NULL, _IOLBF, 0);
	#endif
	OPEN_LOG();
	OPEN_LOGCONSOLE();
	Initialize(APPNAME);
	WORKING_FOLDER = _DATA_PATH;
	INIT_WORKING_FOLDER;
	// Second argument is the verbosity level: non-zero also opens a log file, without which
	// every VERBOSE()/LOG() line the tests emit is discarded (the console sink does not reach
	// stdout, so failures would otherwise report nothing but an exit code). The log is written
	// to the current directory, not WORKING_FOLDER, to keep the source data folder clean.
	const int nVerbosity = (argc > 2 ? std::atoi(argv[2]) : 0);
	const bool verbose = (nVerbosity != 0);
	if (verbose) {
		g_nVerbosityLevel = nVerbosity;
		OPEN_LOGFILE((APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());
	}
	const bool forceCPU = (argc > 3 && std::atoi(argv[3]) != 0);
	// run the selected suite inside a lambda so the teardown below is reached on failure
	// too: the log file in particular must be closed, or a failing run truncates its own log
	const bool succeeded = [&]() {
		if (argc < 2 || std::atoi(argv[1]) == 0) {
			if (!UnitTests())
				return false;
		} else if (std::atoi(argv[1]) == 1) {
			// Run SFM smoke tests, in this order: the table names the test that failed
			const std::vector<std::pair<const char*, std::function<bool()>>> sfmTests = {
				// import is the pipeline's first stage, and every test below starts with one
				SFM_TEST(ImportMetadataDeterminismTest),
				SFM_TEST(ExifFocalUntrustedTest),
				SFM_TEST(CameraTypeImportTest),
				SFM_TEST(SphericalImageSizeImportTest),
				SFM_TEST(TestSimilarityTransform),
				SFM_TEST(KnownPosesImportTest),
				SFM_TEST(FramesPoseFrameDetectionTest),
				SFM_TEST(KnownPosePairSelectionTest),
				SFM_TEST(SequentialPairsPriorTest),
				SFM_TEST(AlignToPriorPosesTest),
				SFM_TEST(AlignToPriorPosesCollinearTest),
				SFM_TEST(AlignToGPSDegenerateTest),
				SFM_TEST(PairsWeightingTest),
				SFM_TEST(PairsWeightingDenseFrameEvidenceTest),
				SFM_TEST(PairCompositeWeightTest),
				SFM_TEST(ViewGraphCalibratorTest),
				SFM_TEST(BAPinholeReprojectionJacobianTest),
				SFM_TEST(ObservationSigmasTest),
				SFM_TEST(DenseObservationWeightEstimateTest),
				SFM_TEST(BAIntrinsicFlagsTest),
				SFM_TEST(BundleAdjustmentPairConstraintTest),
				SFM_TEST(ReprojectionReportOverUsedTest),
				SFM_TEST(BAObservationCapTest),
				SFM_TEST(BAObservationCapWideViewTest),
				SFM_TEST(BAInnerIterationOrderTest),
				SFM_TEST(BADenseWeightPerImageTest),
				SFM_TEST(BADenseWeightCeilingTest),
				SFM_TEST(RotationEstimatorTest),
				SFM_TEST(RotationWeightScaleTest),
				SFM_TEST(ContradictionCutTest),
				SFM_TEST(TranslationCutTest),
				SFM_TEST(FilteredSceneReloadTest),
				SFM_TEST(ScaleEstimatorTest),
				SFM_TEST(TranslationEstimatorTest),
				SFM_TEST(TranslationNormalEquationsTest),
				SFM_TEST(ScaleComponentsTest),
				SFM_TEST(ScaleConjugateGradientsTest),
				SFM_TEST(TripletStarInitTest),
				SFM_TEST(PairScalesTest),
				SFM_TEST(PreMatchTest),
				SFM_TEST(PairMatcherTest),
				SFM_TEST(MatchPairsFailureTest),
				SFM_TEST(MatchRoundFatalSignalTest),
				SFM_TEST(TwoViewTest),
				SFM_TEST(FundamentalPoseTwoCamerasTest),
				SFM_TEST(VocabularyTreeTest),
				SFM_TEST(DenseWarpGridConventionTest),
				SFM_TEST(DenseWarpTrackingTest),
				SFM_TEST(DenseCoverageSampleTest),
				SFM_TEST(DenseComplementaryDrawTest),
				SFM_TEST(DenseFillDensityTest),
				SFM_TEST(DenseFillCeilingTest),
				SFM_TEST(DenseVerdictTest),
				SFM_TEST(DenseGuidedMatchTest),
				SFM_TEST(DenseGuidedBandTest),
				SFM_TEST(BuildTracksConflictTest),
				SFM_TEST(DenseAssemblyTest),
				SFM_TEST(DenseFitWeightTest),
				SFM_TEST(PlanarTwinSwitchTest),
				SFM_TEST(DenseLensInlierTest),
				SFM_TEST(DenseKeypointBoundaryTest),
				SFM_TEST(SupplementEvidenceIsolationTest),
				SFM_TEST(GlobalDescriptorsQueryTest),
				SFM_TEST(RetrievalModeTest),
				SFM_TEST(VocabularyIgnoresGlobalDescriptorsTest),
				SFM_TEST(RoMa2PreprocessTest),
				SFM_TEST(RoMa2ManifestVersionTest),
				SFM_TEST(FlexGlueManifestTest),
				SFM_TEST(ResolveModelPathTest),
				SFM_TEST(RoMa2OnnxParityTest),
				SFM_TEST(DenseMatcherReconstructTest),
				SFM_TEST(PipelineTest),
				SFM_TEST(GPSPriorPoseUncertaintyTest),
				SFM_TEST(PoseUncertaintyExportTest),
				SFM_TEST(ReconstructSphericalSyntheticTest),
				SFM_TEST(SphericalBottomBandTest),
				SFM_TEST(SphericalBottomBandDenseTest),
				SFM_TEST(FeatureTypeNameTest),
				SFM_TEST(SphericalPairGeometryTest),
				SFM_TEST(MatchGeometricLensTest),
				SFM_TEST(LensFitRecoveryTest),
				SFM_TEST(LensFitFocalTest),
				SFM_TEST(LensFitRefusalTest),
				SFM_TEST(LensFitMinimumPairsTest),
				SFM_TEST(DenseReverifyPartitionTest),
				SFM_TEST(DenseReverifyLensTest),
				SFM_TEST(LensFitStageTest),
				SFM_TEST(LensFitFocalStageTest),
				SFM_TEST(LensFitUntrustedFocalTest),
				SFM_TEST(CubeMapFaceRenderTest),
				SFM_TEST(CubeMapBridgeGeometryTest),
				SFM_TEST(CubeMapBridgeEndToEndTest),
				SFM_TEST(CubeMapBridgeMVSLoadTest),
				SFM_TEST(CubeMapBridgeMixedSceneTest),
				SFM_TEST(CubeMapBridgeDropTopBottomTest),
				{"RealImagesPipelineTest", [&]() { return SFM::RealImagesPipelineTest(verbose); }},
				SFM_TEST(DefaultDetectorTest),
				SFM_TEST(TripletFilterTest),
				SFM_TEST(StarReferenceViewTest),
				SFM_TEST(StarEvidenceTest),
				SFM_TEST(StarScaleGaugeTest),
				SFM_TEST(NearDuplicateYardstickTest),
				SFM_TEST(ResectionLinkChecksTest),
				SFM_TEST(ResectionWeakEvidenceTest),
				SFM_TEST(ResectionNextImageOrderTest),
				SFM_TEST(ResectionTriesEveryCandidateTest),
				SFM_TEST(ResectionRefusedAverageTest),
				SFM_TEST(ResectionStallRetryTest),
				SFM_TEST(ResectionClosingAlwaysTest),
				SFM_TEST(ResectionRemovalStallTest),
				SFM_TEST(ResectionInlierBarTest),
				SFM_TEST(ResectionDenseFitWeightTest),
				SFM_TEST(ResectionWeightedInlierBarTest),
				SFM_TEST(LinkWalksTest),
				SFM_TEST(ResectionLinkGateTest),
				SFM_TEST(RepeatedFullAdjustmentTest),
				SFM_TEST(CappedAdjustmentTriangulationTest),
				SFM_TEST(CappedSolveScaleTest),
				SFM_TEST(DenseReprojectionBarTest),
				SFM_TEST(FilterTracksUnposedTrackTest),
				SFM_TEST(FilterTracksUntriangulatedTrackTest),
				SFM_TEST(TriangulationFailureTest),
				SFM_TEST(SphericalTriangulationTest),
				SFM_TEST(ContradictingImagesTest),
				// Hierarchical SFM tests - Phase 1: Scene Clustering
				SFM_TEST(SceneClusterSingleClusterTest),
				SFM_TEST(SceneClusterSizeConstraintsTest),
				SFM_TEST(SceneClusterDisconnectedComponentsTest),
				SFM_TEST(SceneClusterTargetSizeTest),
				SFM_TEST(SceneClusterSeamTracksTest),
				SFM_TEST(SceneClusterLeafMergeTest),
				SFM_TEST(SceneClusterMemoryProtocolTest),
				SFM_TEST(SceneClusterIDRemappingTest),
				SFM_TEST(SceneClusterSmallClusterRescueTest),
				SFM_TEST(SceneClusterSubFloorCouplingTest),
				SFM_TEST(SceneClusterStandaloneCommunityTest),
				SFM_TEST(SceneClusterCoupledOrphanSkippedTest),
				SFM_TEST(SceneClusterBalanceCouplingTest),
				SFM_TEST(SceneClusterThinStretchTest),
				SFM_TEST(SceneBlocksInsertReleaseTest),
				SFM_TEST(SceneBlocksInsertRebuildTest),
				SFM_TEST(PoolResectionWrongClusterTest),
				SFM_TEST(PoolResectionLinkOnlyTest),
				// Hierarchical SFM tests - Phase 3: Global Alignment
				SFM_TEST(GlobalAlignmentRotationAveragingExtendedTest),
				SFM_TEST(GlobalAlignmentScaleAveragingExtendedTest),
				SFM_TEST(GlobalAlignmentScaleAveragingFallbackTest),
				SFM_TEST(GlobalAlignmentTranslationAveragingExtendedTest),
				SFM_TEST(RobustAveragingScaleTest),
				SFM_TEST(RobustAveragingTranslationTest),
				SFM_TEST(SeamGraphConsensusTest),
				SFM_TEST(ReadingWeightCamerasTest),
				SFM_TEST(HeavyWrongChordTest),
				SFM_TEST(TwoDirectionBridgeTest),
				SFM_TEST(SymmetricBridgePairsTest),
				SFM_TEST(OneCameraReadingTest),
				SFM_TEST(ReweightingStartTest),
				SFM_TEST(BarsReEntryTest),
				SFM_TEST(RingPlacementTest),
				SFM_TEST(CrossBlockJoinTest),
				SFM_TEST(MergeJoinReprojectionTest),
				SFM_TEST(MergeTrackLengthTest),
				SFM_TEST(PooledSeamReadingTest),
				SFM_TEST(MergeRoundsTest),
				SFM_TEST(UnobservableScaleTest),
				SFM_TEST(UnobservableScalePairTest),
				SFM_TEST(AmbiguousPairTest),
				SFM_TEST(ContradictedPlacementTest),
				SFM_TEST(PairAgreementWeightTest),
				SFM_TEST(RivalModesOnCycleTest),
				SFM_TEST(OneCameraSideTest),
				SFM_TEST(OneCameraDirectionTest),
				SFM_TEST(RigScaleTwoCamerasTest),
				SFM_TEST(RingLoopClosureTest),
				SFM_TEST(BiasedClosingSeamTest),
				SFM_TEST(WeakClosingSeamTest),
				SFM_TEST(SecondModelTest),
				SFM_TEST(FoldedBlockTest),
				SFM_TEST(FoldedBlock45Test),
				SFM_TEST(FalseChordTest),
				SFM_TEST(SeamInliersBecomeTracksTest),
				SFM_TEST(UnplacedBlocksReportedTest),
				SFM_TEST(BentBlocksAdjustmentTest),
				SFM_TEST(BentBlockClosureTest),
				SFM_TEST(BentRingKeepsSeamsTest),
				SFM_TEST(GlobalAlignmentTrackMergeDuplicateImageGuardTest),
				SFM_TEST(GlobalAlignmentTrackMerge3DProximityGuardTest),
				SFM_TEST(HierarchicalCameraAlignmentTest),
				// Hierarchical SFM tests - End-to-End
				SFM_TEST(MergeRoundtripTest),
			};
			for (const auto& test : sfmTests) {
				if (!test.second()) {
					VERBOSE("ERROR: %s failed!", test.first);
					return false;
				}
			}
		} else {
			// Run MVS pipeline test
			if (!MVS::PipelineTest(forceCPU, verbose))
				return false;
			if (!MVS::MeshRefineSyntheticTest(forceCPU, verbose))
				return false;
			if (!MVS::MeshRefineEnergyGradientTest(verbose))
				return false;
			if (!MVS::MeshUnseenFacesTest())
				return false;
			if (!MVS::SelectNeighborViewsScaleTest())
				return false;
			if (!MVS::DMapCacheKeepsRequestedImageTest())
				return false;
		}
		return true;
	}();
	Finalize();
	if (verbose)
		CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
	return succeeded ? EXIT_SUCCESS : EXIT_FAILURE;
}
/*----------------------------------------------------------------*/
