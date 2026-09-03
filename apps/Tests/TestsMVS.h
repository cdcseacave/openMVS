/*
 * TestsMVS.h
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

namespace MVS {

// test vertex-colored PLY export and geometry reload
bool MeshVertexColorsPLYTest();
bool MeshHalfMeshProcessingTest();

// test the pixel-unit bold-driver optimizer (MeshRefineStep, SceneRefineCommon.h) against
// hand-built Terms arrays: no images, no scene, milliseconds to run
bool MeshRefineStepTest();

// test the scalar window statistics and ZNCC math both refinement backends share
// (SceneRefineCommon.h) against closed-form identities and a finite difference
bool MeshRefineWindowStatsTest();

// test the Delaunay mesh cut on the two hand-solved synthetic fixtures
// (docs/design/DelaunayMeshReconstruction.md, Appendix)
bool MeshBipyramidFixtureTest();
bool MeshTetraInteriorPointFixtureTest();

// test MVS stages on a small sample dataset
bool PipelineTest(bool forceCPU = false, bool verbose = false);

// synthetic end-to-end test of the mesh-refinement photometric pipeline (rasterize, warp,
// window statistics, ZNCC gradient, smoothing, stepper) against a textured flat plane with a
// known ground truth: no other automated test exercises Scene::RefineMesh/RefineMeshCUDA
bool MeshRefineSyntheticTest(bool forceCPU = false, bool verbose = false);

// finite-difference consistency gate for the exact energy the Ceres arm of Scene::RefineMesh
// (--use-ceres) minimizes: on the same synthetic fixture, E(v+t*u)-E(v) must agree with
// t*<grad E(v),u> to within 5% for the photometric term alone, the thin-plate term alone and
// their sum. Runs on the CPU with no Ceres dependency, so a build without _USE_CERES still proves
// that the cost and the gradient the solver would be handed are the same functional
bool MeshRefineEnergyGradientTest(bool verbose = false);
/*----------------------------------------------------------------*/

} // namespace MVS
