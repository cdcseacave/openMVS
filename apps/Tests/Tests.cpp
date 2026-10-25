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
	if (Util::toString(L"\x00A9\U0001F600") != "\xC2\xA9\xF0\x9F\x98\x80") {
		VERBOSE("ERROR: wide string UTF-8 conversion failed!");
		return false;
	}
	if (!MVS::MeshVertexColorsPLYTest()) {
		VERBOSE("ERROR: MeshVertexColorsPLYTest failed!");
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
	VERBOSE("All unit tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// test OpenMVS functionality
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
			// Run SFM smoke tests
			// import is the pipeline's first stage, and every test below starts with one
			if (!SFM::ImportMetadataDeterminismTest())
				return false;
			if (!SFM::ExifFocalUntrustedTest())
				return false;
			if (!SFM::CameraTypeImportTest())
				return false;
			if (!SFM::TestSimilarityTransform())
				return false;
			if (!SFM::KnownPosesImportTest())
				return false;
			if (!SFM::FramesPoseFrameDetectionTest())
				return false;
			if (!SFM::KnownPosePairSelectionTest())
				return false;
			if (!SFM::SequentialPairsPriorTest())
				return false;
			if (!SFM::AlignToPriorPosesTest())
				return false;
			if (!SFM::AlignToPriorPosesCollinearTest())
				return false;
			if (!SFM::AlignToGPSDegenerateTest())
				return false;
			if (!SFM::PairsWeightingTest())
				return false;
			if (!SFM::PairsWeightingDenseFrameEvidenceTest())
				return false;
			if (!SFM::ViewGraphCalibratorTest())
				return false;
			if (!SFM::BAPinholeReprojectionJacobianTest())
				return false;
			if (!SFM::ObservationSigmasTest())
				return false;
			if (!SFM::DenseObservationWeightEstimateTest())
				return false;
			if (!SFM::BAIntrinsicFlagsTest())
				return false;
			if (!SFM::BundleAdjustmentPairConstraintTest())
				return false;
			if (!SFM::BAObservationCapTest())
				return false;
			if (!SFM::BAInnerIterationOrderTest())
				return false;
			if (!SFM::BADenseWeightPerImageTest())
				return false;
			if (!SFM::BADenseWeightCeilingTest())
				return false;
			if (!SFM::RotationEstimatorTest())
				return false;
			if (!SFM::ContradictionCutTest())
				return false;
			if (!SFM::TranslationCutTest())
				return false;
			if (!SFM::ScaleEstimatorTest())
				return false;
			if (!SFM::TranslationEstimatorTest())
				return false;
			if (!SFM::TranslationNormalEquationsTest())
				return false;
			if (!SFM::ScaleConjugateGradientsTest())
				return false;
			if (!SFM::TripletStarInitTest())
				return false;
			if (!SFM::PairScalesTest())
				return false;
			if (!SFM::PreMatchTest())
				return false;
			if (!SFM::PairMatcherTest())
				return false;
			if (!SFM::MatchPairsFailureTest())
				return false;
			if (!SFM::MatchRoundFatalSignalTest())
				return false;
			if (!SFM::TwoViewTest())
				return false;
			if (!SFM::VocabularyTreeTest())
				return false;
			if (!SFM::ROMA2WarpGridConventionTest())
				return false;
			if (!SFM::ROMA2WarpTrackingTest())
				return false;
			if (!SFM::ROMA2CoverageSampleTest())
				return false;
			if (!SFM::ROMA2ComplementaryDrawTest())
				return false;
			if (!SFM::ROMA2DenseFillDensityTest())
				return false;
			if (!SFM::ROMA2DenseFillCeilingTest())
				return false;
			if (!SFM::ROMA2VerdictTest())
				return false;
			if (!SFM::ROMA2GuidedMatchTest())
				return false;
			if (!SFM::ROMA2GuidedBandTest())
				return false;
			if (!SFM::BuildTracksConflictTest())
				return false;
			if (!SFM::ROMA2AssemblyTest())
				return false;
			if (!SFM::ROMA2DenseFitWeightTest())
				return false;
			if (!SFM::ROMA2LensInlierTest())
				return false;
			if (!SFM::DenseKeypointBoundaryTest())
				return false;
			if (!SFM::SupplementEvidenceIsolationTest())
				return false;
			if (!SFM::GlobalDescriptorsQueryTest())
				return false;
			if (!SFM::RetrievalModeTest())
				return false;
			if (!SFM::VocabularyIgnoresGlobalDescriptorsTest())
				return false;
			if (!SFM::RoMa2PreprocessTest())
				return false;
			if (!SFM::RoMa2ManifestVersionTest())
				return false;
			if (!SFM::ResolveModelPathTest())
				return false;
			if (!SFM::RoMa2OnnxParityTest())
				return false;
			if (!SFM::ROMA2ReconstructTest())
				return false;
			if (!SFM::PipelineTest())
				return false;
			if (!SFM::GPSPriorPoseUncertaintyTest())
				return false;
			if (!SFM::PoseUncertaintyExportTest())
				return false;
			if (!SFM::ReconstructSphericalSyntheticTest())
				return false;
			if (!SFM::PairsMatcherSphericalTest())
				return false;
			if (!SFM::MatchGeometricSphericalTest())
				return false;
			if (!SFM::MatchGeometricLensTest())
				return false;
			if (!SFM::LensFitRecoveryTest())
				return false;
			if (!SFM::LensFitFocalTest())
				return false;
			if (!SFM::LensFitRefusalTest())
				return false;
			if (!SFM::LensFitMinimumPairsTest())
				return false;
			if (!SFM::ROMA2ReverifyPartitionTest())
				return false;
			if (!SFM::ROMA2ReverifyLensTest())
				return false;
			if (!SFM::LensFitStageTest())
				return false;
			if (!SFM::LensFitFocalStageTest())
				return false;
			if (!SFM::LensFitUntrustedFocalTest())
				return false;
			if (!SFM::CubeMapFaceRenderTest())
				return false;
			if (!SFM::CubeMapBridgeGeometryTest())
				return false;
			if (!SFM::CubeMapBridgeEndToEndTest())
				return false;
			if (!SFM::CubeMapBridgeMVSLoadTest())
				return false;
			if (!SFM::CubeMapBridgeMixedSceneTest())
				return false;
			if (!SFM::CubeMapBridgeDropTopBottomTest())
				return false;
			if (!SFM::ReconstructTest(verbose))
				return false;
			if (!SFM::ReconstructExportCSVTest())
				return false;
			if (!SFM::TripletFilterTest())
				return false;
			if (!SFM::StarReferenceViewTest())
				return false;
			if (!SFM::StarEvidenceTest())
				return false;
			if (!SFM::ResectionAcceptanceTest())
				return false;
			if (!SFM::ResectionLinkDirectionTest())
				return false;
			if (!SFM::ResectionModelParallaxTest())
				return false;
			if (!SFM::ResectionWeakEvidenceTest())
				return false;
			if (!SFM::ResectionNextImageOrderTest())
				return false;
			if (!SFM::ResectionTriesEveryCandidateTest())
				return false;
			if (!SFM::ResectionRefusedAverageTest())
				return false;
			if (!SFM::ResectionStallRetryTest())
				return false;
			if (!SFM::ResectionInlierBarTest())
				return false;
			if (!SFM::ResectionDenseFitWeightTest())
				return false;
			if (!SFM::RepeatedFullAdjustmentTest())
				return false;
			if (!SFM::CappedSolveScaleTest())
				return false;
			if (!SFM::DenseReprojectionBarTest())
				return false;
			if (!SFM::ContradictingImagesTest())
				return false;
			if (!SFM::PairDirectionToleranceTest())
				return false;
			// Hierarchical SFM tests - Phase 1: Scene Clustering
			if (!SFM::SceneClusterSingleClusterTest())
				return false;
			if (!SFM::SceneClusterSizeConstraintsTest())
				return false;
			if (!SFM::SceneClusterDisconnectedComponentsTest())
				return false;
			if (!SFM::SceneClusterTargetSizeTest())
				return false;
			if (!SFM::SceneClusterSeamTracksTest())
				return false;
			if (!SFM::SceneClusterLeafMergeTest())
				return false;
			if (!SFM::SceneClusterSeamRepairTest())
				return false;
			if (!SFM::SceneClusterMemoryProtocolTest())
				return false;
			if (!SFM::SceneClusterIDRemappingTest())
				return false;
			if (!SFM::SceneClusterSmallClusterRescueTest())
				return false;
			if (!SFM::SceneClusterSubFloorCouplingTest())
				return false;
			if (!SFM::SceneClusterStandaloneCommunityTest())
				return false;
			if (!SFM::SceneClusterCoupledOrphanSkippedTest())
				return false;
			if (!SFM::SceneClusterBalanceCouplingTest())
				return false;
			if (!SFM::SceneClusterThinStretchTest())
				return false;
			// Hierarchical SFM tests - Phase 3: Global Alignment
			if (!SFM::GlobalAlignmentBuildGlobalToLocalMapTest())
				return false;
			if (!SFM::GlobalAlignmentRotationAveragingExtendedTest())
				return false;
			if (!SFM::GlobalAlignmentScaleAveragingExtendedTest())
				return false;
			if (!SFM::GlobalAlignmentScaleAveragingFallbackTest())
				return false;
			if (!SFM::GlobalAlignmentTranslationAveragingExtendedTest())
				return false;
			if (!SFM::RobustAveragingScaleTest())
				return false;
			if (!SFM::RobustAveragingTranslationTest())
				return false;
			if (!SFM::SeamGraphConsensusTest())
				return false;
			if (!SFM::BlockJointRefinementTest())
				return false;
			if (!SFM::RingPlacementTest())
				return false;
			if (!SFM::UnobservableScaleTest())
				return false;
			if (!SFM::UnobservableScalePairTest())
				return false;
			if (!SFM::AmbiguousPairTest())
				return false;
			if (!SFM::ContradictedPlacementTest())
				return false;
			if (!SFM::PairAgreementWeightTest())
				return false;
			if (!SFM::RingLoopClosureTest())
				return false;
			if (!SFM::BiasedClosingSeamTest())
				return false;
			if (!SFM::WeakClosingSeamTest())
				return false;
			if (!SFM::SecondModelTest())
				return false;
			if (!SFM::FoldedBlockTest())
				return false;
			if (!SFM::FalseChordTest())
				return false;
			if (!SFM::HeavyFalseChordTest())
				return false;
			if (!SFM::SeamInliersBecomeTracksTest())
				return false;
			if (!SFM::UnplacedBlocksReportedTest())
				return false;
			if (!SFM::RevalidationKeepsOneBlockTest())
				return false;
			if (!SFM::BentBlocksRelaxationTest())
				return false;
			if (!SFM::GlobalAlignmentMergeSingleSceneTest())
				return false;
			if (!SFM::GlobalAlignmentTrackMergeDuplicateImageGuardTest())
				return false;
			if (!SFM::GlobalAlignmentTrackMerge3DProximityGuardTest())
				return false;
			if (!SFM::HierarchicalCameraAlignmentTest())
				return false;
			// Hierarchical SFM tests - End-to-End
			if (!SFM::HierarchicalSFMSplitMergeRoundtripTest())
				return false;
			if (!SFM::HierarchicalSFMWithRandomTransformTest())
				return false;
		} else {
			// Run MVS pipeline test
			if (!MVS::PipelineTest(forceCPU, verbose))
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
