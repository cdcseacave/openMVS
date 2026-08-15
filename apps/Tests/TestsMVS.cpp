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


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestMVS "));

namespace MVS {

bool MeshVertexColorsPLYTest()
{
	namespace fs = std::filesystem;
	const fs::path tmpDir(fs::temp_directory_path() / ("openmvs-colored-ply-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())));
	std::error_code ec;
	fs::create_directories(tmpDir, ec);
	if (ec)
		return false;
	struct CleanupGuard {
		fs::path path;
		~CleanupGuard() { std::error_code ec; fs::remove_all(path, ec); }
	} cleanup{tmpDir};

	Mesh mesh;
	mesh.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	mesh.faces = {{0, 1, 2}};
	const Mesh::ColorArr colors({Pixel8U::RED, Pixel8U::GREEN, Pixel8U::BLUE});
	for (const bool bBinary: {false, true}) {
		const String fileName((tmpDir / (bBinary ? "mesh-binary.ply" : "mesh-ascii.ply")).string());
		if (!mesh.SaveVertexColors(fileName, colors, cList<String>(), bBinary))
			return false;
		Mesh loaded;
		if (!loaded.Load(fileName) || loaded.vertices != mesh.vertices || loaded.faces != mesh.faces)
			return false;
		PointCloud pointCloud;
		if (!pointCloud.Load(fileName) || pointCloud.points.size() != mesh.vertices.size() || pointCloud.colors.size() != colors.size())
			return false;
		FOREACH(idxVertex, mesh.vertices)
			if (pointCloud.points[idxVertex] != mesh.vertices[idxVertex] || pointCloud.colors[idxVertex] != colors[idxVertex])
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
	if (!scene.ReconstructMesh() || !ISINSIDE(scene.mesh.faces.size(), 40000u, 100000u)) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
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
