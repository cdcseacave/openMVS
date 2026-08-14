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
#include "TestsSFM.h"
#include "TestsMVS.h"


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("Tests")


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Test    "));

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
	#ifdef _IMAGE_HEIF
	// the reader's own semantics are tested next to the reader, in libs/IO/ImageHEIF.cpp
	if (!CImageHEIF::Test(MAKE_PATH("images/heif"))) {
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
			if (!SFM::TestSimilarityTransform())
				return false;
			if (!SFM::KnownPosesImportTest())
				return false;
			if (!SFM::FramesPoseFrameDetectionTest())
				return false;
			if (!SFM::KnownPosePairSelectionTest())
				return false;
			if (!SFM::AlignToPriorPosesTest())
				return false;
			if (!SFM::AlignToPriorPosesCollinearTest())
				return false;
			if (!SFM::AlignToGPSDegenerateTest())
				return false;
			if (!SFM::PairsWeightingTest())
				return false;
			if (!SFM::ViewGraphCalibratorTest())
				return false;
			if (!SFM::BAPinholeReprojectionJacobianTest())
				return false;
			if (!SFM::RotationEstimatorTest())
				return false;
			if (!SFM::ScaleEstimatorTest())
				return false;
			if (!SFM::TranslationEstimatorTest())
				return false;
			if (!SFM::TripletStarInitTest())
				return false;
			if (!SFM::PreMatchTest())
				return false;
			if (!SFM::PairMatcherTest())
				return false;
			if (!SFM::TwoViewTest())
				return false;
			if (!SFM::VocabularyTreeTest())
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
			// Hierarchical SFM tests - Phase 1: Scene Clustering
			if (!SFM::SceneClusterSingleClusterTest())
				return false;
			if (!SFM::SceneClusterSizeConstraintsTest())
				return false;
			if (!SFM::SceneClusterDisconnectedComponentsTest())
				return false;
			if (!SFM::SceneClusterMemoryProtocolTest())
				return false;
			if (!SFM::SceneClusterIDRemappingTest())
				return false;
			if (!SFM::SceneClusterSmallClusterRescueTest())
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
			if (!SFM::GlobalAlignmentMergeSingleSceneTest())
				return false;
			if (!SFM::GlobalAlignmentTrackMergeDuplicateImageGuardTest())
				return false;
			if (!SFM::GlobalAlignmentTrackMerge3DProximityGuardTest())
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
