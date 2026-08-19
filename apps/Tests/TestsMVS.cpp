/*
 * TestsMVS.cpp
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

#include "../../libs/MVS.h"
#include "Tests.h"
#include "TestsMVS.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestMVS "));

namespace MVS {

bool MeshVertexColorsPLYTest()
{
	const ScopedTempDir tmpDir(_T("MeshVertexColorsPLYTest"));
	if (!tmpDir.IsValid())
		return false;

	Mesh mesh;
	mesh.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	mesh.faces = {{0, 1, 2}};
	mesh.vertexColors = {Pixel8U::RED, Pixel8U::GREEN, Pixel8U::BLUE};
	for (const bool bBinary: {false, true}) {
		const String fileName(tmpDir(bBinary ? _T("mesh-binary.ply") : _T("mesh-ascii.ply")));
		if (!mesh.Save(fileName, cList<String>(), bBinary))
			return false;
		// the mesh must reload with its colors
		Mesh loaded;
		if (!loaded.Load(fileName) || loaded.vertices != mesh.vertices || loaded.faces != mesh.faces ||
			loaded.vertexColors != mesh.vertexColors)
			return false;
		// the same file must read as a colored point-cloud
		PointCloud pointCloud;
		if (!pointCloud.Load(fileName) || pointCloud.points.size() != mesh.vertices.size() || pointCloud.colors.size() != mesh.vertexColors.size())
			return false;
		FOREACH(idxVertex, mesh.vertices)
			if (pointCloud.points[idxVertex] != mesh.vertices[idxVertex] || pointCloud.colors[idxVertex] != mesh.vertexColors[idxVertex])
				return false;
	}
	return true;
}

// test MVS stages on a small sample dataset
bool PipelineTest(bool forceCPU, bool verbose)
{
	TD_TIMER_START();
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	// force CPU for testing even if a GPU backend is available
	if (forceCPU)
		SEACAVE::CUDA::desiredDeviceIDs.clear();
	#endif
	Scene scene;
	if (!scene.Load(MAKE_PATH("scene.mvs"))) {
		VERBOSE("ERROR: TestDataset failed loading the scene!");
		return false;
	}
	OPTDENSE::init();
	OPTDENSE::bRemoveDmaps = true;
	// The point/face counts and quality vary run-to-run (multi-threaded
	// densify/mesh) and differ between the CPU and GPU PatchMatch backends, so
	// these are deliberately wide plausibility windows, not tight regression
	// bounds: they bracket both backends' observed spread with margin.
	// Re-baselined 2026-08-10 for the current defaults. Note the backends now
	// differ by design, not just by numerical spread: nOptimize defaults to
	// ADJUST_CONFIDENCE_AUTO, so the confidence recalibration runs on the GPU
	// backend (fused into the last geometric-consistency iteration, nearly free)
	// and is skipped on the CPU backend (where it would cost a separate pass).
	// Also on: fusion rescue (fFusePriorWeight=3, ~+90% dense points on this
	// scene); pointWeights hold the plain [0,1] per-view confidence consumed by
	// the weighted mesh visibility (this test keeps pointWeights, unlike the
	// ReconstructMesh app whose constant-weight default discards them).
	// Measured (GPU adjust-ON / CPU adjust-OFF): recon faces 52.9k / 71.4k,
	// cleaned faces 37.0k / 49.8k, quality 50.4 / 52.2.
	if (!scene.DenseReconstruction() || scene.pointcloud.GetSize() < 50000u) {
		VERBOSE("ERROR: TestDataset failed estimating dense point-cloud (%u points)!", scene.pointcloud.GetSize());
		return false;
	}
	if (verbose)
		scene.pointcloud.Save(MAKE_PATH("scene_dense.ply"));
	// Exercise ROI integration with the first point outside the ROI; this used to leave
	// default entries in the spatial-sort index and could reconstruct the wrong points.
	{
		PointCloud pointcloud(scene.pointcloud);
		const OBB3f initialOBB(scene.obb);
		const OBB3f roi(scene.pointcloud.GetAABB(0.1f, 0.9f));
		PointCloud::Index idxOutside(NO_ID);
		FOREACH(idxPoint, scene.pointcloud.points) {
			if (!roi.Intersects(scene.pointcloud.points[idxPoint])) {
				idxOutside = idxPoint;
				break;
			}
		}
		if (idxOutside == NO_ID) {
			VERBOSE("ERROR: TestDataset failed finding a point outside the test ROI!");
			return false;
		}
		if (idxOutside != 0) {
			std::swap(scene.pointcloud.points[0], scene.pointcloud.points[idxOutside]);
			std::swap(scene.pointcloud.pointViews[0], scene.pointcloud.pointViews[idxOutside]);
			if (!scene.pointcloud.pointWeights.IsEmpty())
				std::swap(scene.pointcloud.pointWeights[0], scene.pointcloud.pointWeights[idxOutside]);
			if (!scene.pointcloud.normals.IsEmpty())
				std::swap(scene.pointcloud.normals[0], scene.pointcloud.normals[idxOutside]);
			if (!scene.pointcloud.colors.IsEmpty())
				std::swap(scene.pointcloud.colors[0], scene.pointcloud.colors[idxOutside]);
			if (!scene.pointcloud.labels.IsEmpty())
				std::swap(scene.pointcloud.labels[0], scene.pointcloud.labels[idxOutside]);
		}
		scene.obb = roi;
		if (!scene.ReconstructMesh(2.f, false, true) || scene.mesh.IsEmpty()) {
			VERBOSE("ERROR: TestDataset failed reconstructing the ROI mesh (%u faces)!", scene.mesh.faces.size());
			return false;
		}
		scene.pointcloud.Swap(pointcloud);
		scene.obb = initialOBB;
	}
	// Phase-0.B test D.1: a region-of-interest that intersects none of the dense points must
	// fail cleanly through the "no points available" guard, not silently fall back to using
	// every point
	{
		const OBB3f initialOBB(scene.obb);
		// a small valid OBB (positive extent, so IsBounded() stays true) placed far outside the
		// synthetic scene's coordinate range: it intersects none of the dense points; the extent
		// must exceed the float ulp at this magnitude (0.0625 at 1e6) or it collapses to zero
		scene.obb.Set(Matrix3x3f::IDENTITY, Point3f(1.e6f,1.e6f,1.e6f), Point3f(1.e6f+1.f,1.e6f+1.f,1.e6f+1.f));
		if (!scene.obb.IsValid()) {
			VERBOSE("ERROR: TestDataset built an invalid empty-ROI OBB for the degenerate-input test!");
			return false;
		}
		if (scene.ReconstructMesh(2.f, false, true)) {
			VERBOSE("ERROR: TestDataset should have failed reconstructing an empty ROI!");
			return false;
		}
		scene.obb = initialOBB;
	}
	// Phase-0.B test A: reconstructing with pointWeights released (each view's vote falls back
	// to the implicit constant 1, see vert_info_t::InsertViews in SceneReconstruct.cpp) must
	// behave the same as reconstructing with pointWeights present and every entry explicitly 1
	{
		const PointCloud pointcloudBackup(scene.pointcloud);
		// run 1: no pointWeights at all
		scene.pointcloud = pointcloudBackup;
		scene.pointcloud.pointWeights.Release();
		if (!scene.ReconstructMesh(2.f, false, false) || scene.mesh.IsEmpty()) {
			VERBOSE("ERROR: TestDataset failed reconstructing with empty pointWeights!");
			return false;
		}
		const Mesh::FIndex numFacesEmptyWeights(scene.mesh.faces.size());
		// run 2: identical views layout, pointWeights explicitly all 1
		scene.pointcloud = pointcloudBackup;
		scene.pointcloud.pointWeights.resize(scene.pointcloud.pointViews.size());
		FOREACH(idxPoint, scene.pointcloud.pointViews) {
			const PointCloud::ViewArr& views = scene.pointcloud.pointViews[idxPoint];
			PointCloud::WeightArr& weights = scene.pointcloud.pointWeights[idxPoint];
			weights.resize(views.size());
			FOREACH(idxView, weights)
				weights[idxView] = PointCloud::Weight(1);
		}
		if (!scene.ReconstructMesh(2.f, false, false) || scene.mesh.IsEmpty()) {
			VERBOSE("ERROR: TestDataset failed reconstructing with explicit unit pointWeights!");
			return false;
		}
		const Mesh::FIndex numFacesUnitWeights(scene.mesh.faces.size());
		// Scene::ReconstructMesh takes no thread-count parameter, and its weighting pass runs
		// under OpenMP with atomic float adds keyed off the process-wide thread count (see
		// SceneReconstruct.cpp), so float summation order -- and so the exact face count -- is
		// not guaranteed to match run to run even for identical per-view weights. Forcing
		// omp_set_num_threads(1) here would make it exact, but as a global runtime setting it
		// would also serialize every later OpenMP stage in this same pipeline test (Clean,
		// vertex coloring, texturing), so it is deliberately not used; both meshes are non-empty
		// (checked above) and must agree within 1% of face count instead of exactly
		const Mesh::FIndex faceTol(numFacesEmptyWeights/100+1);
		if (numFacesEmptyWeights > numFacesUnitWeights+faceTol || numFacesUnitWeights > numFacesEmptyWeights+faceTol) {
			VERBOSE("ERROR: TestDataset empty-weights fallback diverged from explicit unit weights (%u vs %u faces)!", numFacesEmptyWeights, numFacesUnitWeights);
			return false;
		}
		scene.pointcloud = pointcloudBackup;
	}
	// Phase-0.B test B: pointWeights must round-trip through the interface archive bit-for-bit
	{
		const ScopedTempDir tmpDir(_T("WeightsRoundTripTest"));
		if (!tmpDir.IsValid())
			return false;
		const PointCloud pointcloudBackup(scene.pointcloud);
		// non-trivial per-point weights: point i gets weight i/N (point 0 is intentionally 0;
		// the rest are strictly positive so LoadInterface's all-zero-weights drop guard does not
		// discard them)
		const float N(static_cast<float>(scene.pointcloud.points.size()));
		scene.pointcloud.pointWeights.resize(scene.pointcloud.pointViews.size());
		FOREACH(idxPoint, scene.pointcloud.pointViews) {
			const PointCloud::Weight w(static_cast<float>(idxPoint)/N);
			PointCloud::WeightArr& weights = scene.pointcloud.pointWeights[idxPoint];
			weights.resize(scene.pointcloud.pointViews[idxPoint].size());
			FOREACH(idxView, weights)
				weights[idxView] = w;
		}
		const String mvsPath(tmpDir(_T("weights.mvs")));
		Scene reloaded;
		if (!scene.SaveInterface(mvsPath) || !reloaded.LoadInterface(mvsPath)) {
			VERBOSE("ERROR: TestDataset failed the pointWeights round-trip archive I/O!");
			return false;
		}
		if (reloaded.pointcloud.points.size() != scene.pointcloud.points.size() ||
			reloaded.pointcloud.pointViews.size() != scene.pointcloud.pointViews.size() ||
			reloaded.pointcloud.pointWeights.size() != scene.pointcloud.pointWeights.size()) {
			VERBOSE("ERROR: TestDataset pointWeights round-trip changed the point-cloud size!");
			return false;
		}
		FOREACH(idxPoint, scene.pointcloud.pointWeights) {
			if (scene.pointcloud.pointWeights[idxPoint] != reloaded.pointcloud.pointWeights[idxPoint]) {
				VERBOSE("ERROR: TestDataset pointWeights round-trip changed point %u's weights!", idxPoint);
				return false;
			}
		}
		scene.pointcloud = pointcloudBackup;
	}
	// Phase-0.B test D.2: too few points for the Delaunay triangulation to ever reach
	// dimension 3 must fail cleanly via the dimension guard in SceneReconstruct.cpp, not
	// crash or produce garbage; 3 points can reach at most dimension 2, so this is
	// deterministic regardless of their actual layout
	{
		const PointCloud pointcloudBackup(scene.pointcloud);
		PointCloud pointcloudTiny;
		pointcloudTiny.points.resize(3);
		pointcloudTiny.pointViews.resize(3);
		for (PointCloud::Index i=0; i<3; ++i) {
			pointcloudTiny.points[i] = pointcloudBackup.points[i];
			pointcloudTiny.pointViews[i] = pointcloudBackup.pointViews[i];
		}
		scene.pointcloud = pointcloudTiny;
		if (scene.ReconstructMesh(2.f, false, false)) {
			VERBOSE("ERROR: TestDataset should have failed reconstructing from only 3 points!");
			return false;
		}
		scene.pointcloud = pointcloudBackup;
	}
	if (!scene.ReconstructMesh() || !ISINSIDE(scene.mesh.faces.size(), 40000u, 100000u)) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
	// Phase-0.B test C: SamplePoints with an explicit seed must be reproducible, and a
	// different seed must draw a different sample; per-face point counts are themselves
	// seed-dependent (see the fractional-area coin-flip draw in Mesh::SamplePoints), so the
	// cross-seed check compares the sampled points directly rather than assuming counts differ
	{
		constexpr unsigned numSamples = 1000;
		PointCloud pcSeed42a, pcSeed42b, pcSeed7;
		scene.mesh.SamplePoints(numSamples, pcSeed42a, 42);
		scene.mesh.SamplePoints(numSamples, pcSeed42b, 42);
		scene.mesh.SamplePoints(numSamples, pcSeed7, 7);
		if (pcSeed42a.points.empty()) {
			VERBOSE("ERROR: TestDataset SamplePoints produced no points!");
			return false;
		}
		if (pcSeed42a.points != pcSeed42b.points) {
			VERBOSE("ERROR: TestDataset SamplePoints(seed=42) was not reproducible (%u vs %u points)!", pcSeed42a.points.size(), pcSeed42b.points.size());
			return false;
		}
		if (pcSeed42a.points == pcSeed7.points) {
			VERBOSE("ERROR: TestDataset SamplePoints(seed=42) and SamplePoints(seed=7) produced identical points!");
			return false;
		}
	}
	constexpr float decimate = 0.7f;
	scene.mesh.Clean(decimate);
	if (!ISINSIDE(scene.mesh.faces.size(), 28000u, 70000u)) {
		VERBOSE("ERROR: TestDataset failed cleaning the mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_clean.ply"));
	#ifdef _USE_OPENMP
	TestMeshProjectionMT(scene.mesh, scene.images[1]);
	#endif
	// color the mesh per vertex; this releases the images, which texturing below reloads
	const Mesh::Color colEmpty(255, 127, 39);
	if (!scene.ComputeVertexColors(0, 0, 0, 0.f, 0.3f, colEmpty) || scene.mesh.vertexColors.size() != scene.mesh.vertices.size()) {
		VERBOSE("ERROR: TestDataset failed computing the mesh vertex colors!");
		return false;
	}
	// most vertices are seen by at least one view, so they must be sampled and not left empty
	Mesh::VIndex numColored(0);
	for (const Mesh::Color& color: scene.mesh.vertexColors)
		if (color != colEmpty)
			++numColored;
	if (numColored*2 < scene.mesh.vertexColors.size()) {
		VERBOSE("ERROR: TestDataset colored only %u of %u mesh vertices!", numColored, scene.mesh.vertexColors.size());
		return false;
	}
	// the colors must survive the project archive round-trip
	{
		const ScopedTempDir tmpDir(_T("PipelineTest"));
		if (!tmpDir.IsValid())
			return false;
		const String mvsPath(tmpDir(_T("colored.mvs")));
		Scene reloaded;
		if (!scene.Save(mvsPath) || !reloaded.Load(mvsPath) || reloaded.mesh.vertexColors != scene.mesh.vertexColors) {
			VERBOSE("ERROR: TestDataset failed reloading the mesh vertex colors!");
			return false;
		}
	}
	scene.mesh.vertexColors.Release();
	if (!scene.TextureMesh(0, 0) || !scene.mesh.HasTexture()) {
		VERBOSE("ERROR: TestDataset failed texturing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture.ply"));
	const float qualityScore = scene.ComputeReconstructionQuality().score();
	if (qualityScore < 43.f) {
		VERBOSE("ERROR: TestDataset reconstruction quality too low (%.1f)!", qualityScore);
		return false;
	}
	VERBOSE("All pipeline stages passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace MVS
