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
#include <fstream>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestMVS "));

namespace MVS {

// test MVS stages on a small sample dataset
bool PipelineTest(bool forceCPU, bool verbose)
{
	TD_TIMER_START();
	#ifdef _USE_CUDA
	// force CPU for testing even if CUDA is available
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
	if (!scene.DenseReconstruction() || scene.pointcloud.GetSize() < 50000u) {
		VERBOSE("ERROR: TestDataset failed estimating dense point-cloud!");
		return false;
	}
	if (verbose)
		scene.pointcloud.Save(MAKE_PATH("scene_dense.ply"));
	if (!scene.ReconstructMesh() || !ISINSIDE(scene.mesh.faces.size(), 25000u, 38000u)) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
	constexpr float decimate = 0.7f;
	scene.mesh.Clean(decimate);
	if (!ISINSIDE(scene.mesh.faces.size(), 17000u, 25000u)) {
		VERBOSE("ERROR: TestDataset failed cleaning the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_clean.ply"));
	#ifdef _USE_OPENMP
	TestMeshProjectionMT(scene.mesh, scene.images[1]);
	#endif
	// snapshot the cleaned, untextured mesh so we can run both texturing backends
	// (CPU mapmap+seam-leveling vs CUDA xatlas+sequential-blending) from the same baseline
	const Mesh meshUntextured = scene.mesh;
	if (!scene.TextureMesh(0, 0) || !scene.mesh.HasTexture()) {
		VERBOSE("ERROR: TestDataset failed texturing the mesh (CPU)!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture_cpu.ply"));
	const float qualityScoreCPU = scene.ComputeReconstructionQuality().score();
	if (qualityScoreCPU < 45.f) {
		VERBOSE("ERROR: TestDataset CPU texturing quality too low (%.1f)!", qualityScoreCPU);
		return false;
	}
	#ifdef _USE_CUDA
	if (!forceCPU) {
		scene.mesh = meshUntextured;
		if (!scene.TextureMeshCuda(/*maxTexRes*/8192, /*maxImgRes*/0, /*rePack*/false, /*reParametrize*/true) || !scene.mesh.HasTexture()) {
			VERBOSE("ERROR: TestDataset failed texturing the mesh (CUDA)!");
			return false;
		}
		if (verbose)
			scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture_cuda.ply"));
		const float qualityScoreCUDA = scene.ComputeReconstructionQuality().score();
		if (qualityScoreCUDA < 45.f) {
			VERBOSE("ERROR: TestDataset CUDA texturing quality too low (%.1f)!", qualityScoreCUDA);
			return false;
		}
		VERBOSE("Texturing scores: CPU=%.1f CUDA=%.1f", qualityScoreCPU, qualityScoreCUDA);
		// also persist scores to disk so callers can extract them without parsing console output
		// (OpenMVS LogConsole redirects stdout to CONOUT$, defeating shell pipes)
		std::ofstream(MAKE_PATH("scene_dense_mesh_texture_scores.txt"))
			<< "CPU=" << qualityScoreCPU << " CUDA=" << qualityScoreCUDA << std::endl;
	} else
	#endif
	{
		VERBOSE("Texturing score: CPU=%.1f", qualityScoreCPU);
		std::ofstream(MAKE_PATH("scene_dense_mesh_texture_scores.txt"))
			<< "CPU=" << qualityScoreCPU << std::endl;
	}
	VERBOSE("All pipeline stages passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace MVS
