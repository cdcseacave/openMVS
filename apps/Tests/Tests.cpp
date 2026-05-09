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
	const bool verbose = (argc > 2 && std::atoi(argv[2]) != 0);
	const bool forceCPU = (argc > 3 && std::atoi(argv[3]) != 0);
	if (argc < 2 || std::atoi(argv[1]) == 0) {
		if (!UnitTests())
			return EXIT_FAILURE;
	} else if (std::atoi(argv[1]) == 1) {
		// Run SFM smoke tests
		if (!SFM::TestSimilarityTransform())
			return EXIT_FAILURE;
		if (!SFM::PairsWeightingTest())
			return EXIT_FAILURE;
		if (!SFM::ViewGraphCalibratorTest())
			return EXIT_FAILURE;
		if (!SFM::BAPinholeReprojectionJacobianTest())
			return EXIT_FAILURE;
		if (!SFM::RotationEstimatorTest())
			return EXIT_FAILURE;
		if (!SFM::ScaleEstimatorTest())
			return EXIT_FAILURE;
		if (!SFM::TranslationEstimatorTest())
			return EXIT_FAILURE;
		if (!SFM::TripletStarInitTest())
			return EXIT_FAILURE;
		if (!SFM::PreMatchTest())
			return EXIT_FAILURE;
		if (!SFM::PairMatcherTest())
			return EXIT_FAILURE;
		if (!SFM::TwoViewTest())
			return EXIT_FAILURE;
		if (!SFM::VocabularyTreeTest())
			return EXIT_FAILURE;
		if (!SFM::PipelineTest())
			return EXIT_FAILURE;
		if (!SFM::ReconstructSphericalSyntheticTest())
			return EXIT_FAILURE;
		if (!SFM::PairsMatcherSphericalTest())
			return EXIT_FAILURE;
		if (!SFM::MatchGeometricSphericalTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapFaceRenderTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapBridgeGeometryTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapBridgeEndToEndTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapBridgeMVSLoadTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapBridgeMixedSceneTest())
			return EXIT_FAILURE;
		if (!SFM::CubeMapBridgeDropTopBottomTest())
			return EXIT_FAILURE;
		if (!SFM::ReconstructTest(verbose))
			return EXIT_FAILURE;
		// Hierarchical SFM tests - Phase 1: Scene Clustering
		if (!SFM::SceneClusterSingleClusterTest())
			return EXIT_FAILURE;
		if (!SFM::SceneClusterSizeConstraintsTest())
			return EXIT_FAILURE;
		if (!SFM::SceneClusterDisconnectedComponentsTest())
			return EXIT_FAILURE;
		if (!SFM::SceneClusterMemoryProtocolTest())
			return EXIT_FAILURE;
		if (!SFM::SceneClusterIDRemappingTest())
			return EXIT_FAILURE;
		if (!SFM::SceneClusterSmallClusterRescueTest())
			return EXIT_FAILURE;
		// Hierarchical SFM tests - Phase 3: Global Alignment
		if (!SFM::GlobalAlignmentBuildGlobalToLocalMapTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentRotationAveragingExtendedTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentScaleAveragingExtendedTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentScaleAveragingFallbackTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentTranslationAveragingExtendedTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentMergeSingleSceneTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentTrackMergeDuplicateImageGuardTest())
			return EXIT_FAILURE;
		if (!SFM::GlobalAlignmentTrackMerge3DProximityGuardTest())
			return EXIT_FAILURE;
		// Hierarchical SFM tests - End-to-End
		if (!SFM::HierarchicalSFMSplitMergeRoundtripTest())
			return EXIT_FAILURE;
		if (!SFM::HierarchicalSFMWithRandomTransformTest())
			return EXIT_FAILURE;
	} else {
		// Run MVS pipeline test
		if (!MVS::PipelineTest(forceCPU, verbose))
			return EXIT_FAILURE;
	}
	Finalize();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
