/*
* PythonWrapper.cpp
*
* Copyright (c) 2014-2022 SEACAVE
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

#include "ConfigLocal.h"

#ifdef _USE_BOOST_PYTHON

// In static-only builds the previous code did `#undef _USRDLL`/`#define _LIB`
// around these includes to neutralize *_API macros. With shared libs we keep
// _USRDLL set so consumer TUs see __declspec(dllimport) and resolve symbols
// against the import libs of Common.dll / MVS.dll / SFM.dll at link time.
#include "Common.h"
#include "Scene.h"
#ifndef BOOST_PYTHON_STATIC_LIB
#define BOOST_PYTHON_STATIC_LIB
#endif
#include <boost/python.hpp>

// Forward declaration — defined in libs/SFM/PythonWrapper.cpp; called from
// inside our single BOOST_PYTHON_MODULE so SFM and MVS bindings end up in
// the same `pyOpenMVS` extension module.
namespace pySFM { void RegisterBindings(); }


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace pyMVS {

class Scene : public MVS::Scene
{
public:
	Scene(unsigned _nMaxThreads=0) : MVS::Scene(_nMaxThreads) {
		INIT_WORKING_FOLDER;
		MVS::Initialize("pyMVS", _nMaxThreads);
		MVS::OPTDENSE::init();
	}
	~Scene() {
		MVS::Finalize();
	}

	bool pyLoad(const std::string& fileName, bool bImport=false) {
		return Load(MAKE_PATH_SAFE(fileName), bImport);
	}
	bool pySave(const std::string& fileName, int type=ARCHIVE_DEFAULT) const {
		return Save(MAKE_PATH_SAFE(fileName), static_cast<ARCHIVE_TYPE>(type));
	}

	bool pySavePointCloud(const std::string& fileName) const {
		return pointcloud.Save(MAKE_PATH_SAFE(fileName));
	}

	bool pyLoadMesh(const std::string& fileName) {
		return mesh.Load(MAKE_PATH_SAFE(fileName));
	}
	bool pySaveMesh(const std::string& fileName) const {
		return mesh.Save(MAKE_PATH_SAFE(fileName));
	}

	bool pyDenseReconstruction(unsigned nResolutionLevel=1, int nFusionMode=0, bool bCrop2ROI=true, float fBorderROI=0) {
		MVS::OPTDENSE::nResolutionLevel = nResolutionLevel;
		return DenseReconstruction(nFusionMode, bCrop2ROI, fBorderROI);
	}
	bool pyReconstructMesh(float distInsert=2, bool bUseFreeSpaceSupport=false, bool bUseOnlyROI=false) {
		return ReconstructMesh(distInsert, bUseFreeSpaceSupport, bUseOnlyROI);
	}
	void pyCleanMesh(float fDecimate=1.f, float fRemoveSpurious=20.f, bool bRemoveSpikes=true, unsigned nCloseHoles=30, unsigned nSmoothMesh=2, float fEdgeLength=0.f, bool bCrop2ROI=false) {
		if (bCrop2ROI && IsBounded())
			mesh.RemoveFacesOutside(obb);
		mesh.Clean(fDecimate, fRemoveSpurious, bRemoveSpikes, nCloseHoles, nSmoothMesh, fEdgeLength);
	}
	bool pyRefineMesh(unsigned nResolutionLevel=0, unsigned nEnsureEdgeSize=1, unsigned nMaxFaceArea=32, unsigned nScales=2, float fScaleStep=0.5f, float fRegularityWeight=0.2f) {
		return RefineMesh(nResolutionLevel, 640/*nMinResolution*/, 8/*nMaxViews*/, 0.f/*fDecimateMesh*/, 30/*nCloseHoles*/, nEnsureEdgeSize,
			nMaxFaceArea, nScales, fScaleStep, 0/*nAlternatePair*/, fRegularityWeight, 0.9f/*fRatioRigidityElasticity*/, 45.05f/*fGradientStep*/);
	}
	bool pyTextureMesh(unsigned nResolutionLevel=0, uint32_t nColEmpty=0x00FF7F27) {
		return TextureMesh(nResolutionLevel, 640/*nMinResolution*/, 0/*minCommonCameras*/, 0.f/*fOutlierThreshold*/, 0.3f/*fRatioDataSmoothness*/,
			true/*bGlobalSeamLeveling*/, true/*bLocalSeamLeveling*/, 0/*nTextureSizeMultiple*/, Pixel8U(nColEmpty));
	}

	// SceneGeometry exposures (commit df48bf5): KD-tree-based normal estimation,
	// sparse surface estimation, and ROI cropping operate on the in-memory
	// pointcloud and don't need MVS_API tagging because Scene already has it.
	bool pyEstimatePointCloudNormals(bool bRefine=true) {
		return EstimatePointCloudNormals(bRefine);
	}
	bool pyEstimateSparseSurface(unsigned kNeighbors=16, float sizeScale=0.9f, float normalAngleMaxDeg=0.f) {
		return EstimateSparseSurface(kNeighbors, sizeScale, FD2R(normalAngleMaxDeg));
	}
	void pyCropToROI(unsigned minNumPoints=3) {
		if (IsBounded())
			CropToROI(static_cast<const OBB3f&>(obb), minNumPoints);
	}
};

void SetWorkingFolder(const std::string& folder) {
	WORKING_FOLDER = folder;
	Util::ensureValidFolderPath(WORKING_FOLDER);
	INIT_WORKING_FOLDER;
}

BOOST_PYTHON_MODULE(pyOpenMVS) {
	using namespace boost::python;

	class_<Scene, boost::noncopyable, boost::shared_ptr<Scene>>("Scene", init<unsigned>(arg("max_threads")=0))
		.def("load", &Scene::pyLoad, (arg("file_path"), arg("import")=true))
		.def("save", &Scene::pySave, (arg("file_path"), arg("type")=static_cast<int>(ARCHIVE_DEFAULT)))
		.def("save_pointcloud", &Scene::pySavePointCloud, (arg("file_path")))
		.def("load_mesh", &Scene::pyLoadMesh, (arg("file_path")))
		.def("save_mesh", &Scene::pySaveMesh, (arg("file_path")))
		.def("scale_images", &Scene::ScaleImages)
		.def("transform", static_cast<void (Scene:: *)(const Matrix3x3&, const Point3&, REAL)>(&Scene::Transform))
		.def("transform34", static_cast<void (Scene:: *)(const Matrix3x4&)>(&Scene::Transform))
		.def("align_to", &Scene::AlignTo)
		.def("dense_reconstruction", &Scene::pyDenseReconstruction, (arg("resolution_level")=0, arg("fusion_mode")=0, arg("crop_to_roi")=true, arg("roi_border")=0.f))
		.def("reconstruct_mesh", &Scene::pyReconstructMesh, (arg("dist_insert")=2, arg("use_free_space_support")=false, arg("use_only_roi")=false))
		.def("clean_mesh", &Scene::pyCleanMesh, (arg("decimate")=1.f, arg("remove_spurious")=20.f, arg("remove_spikes")=true, arg("close_holes")=30, arg("smooth_mesh")=2, arg("edge_length")=0.f, arg("crop_to_roi")=true))
		.def("refine_mesh", &Scene::pyRefineMesh, (arg("resolution_level")=0, arg("ensure_edge_size")=1, arg("max_face_area")=32, arg("scales")=2, arg("scale_step")=0.5f, arg("regularity_weight")=0.2f))
		.def("texture_mesh", &Scene::pyTextureMesh, (arg("resolution_level")=0, arg("empty_color")=0x00FF7F27))
		.def("compute_leveled_volume", &Scene::ComputeLeveledVolume)
		.def("estimate_normals", &Scene::pyEstimatePointCloudNormals, (arg("refine")=true))
		.def("estimate_sparse_surface", &Scene::pyEstimateSparseSurface,
				(arg("k_neighbors")=16, arg("size_scale")=0.9f, arg("normal_angle_max_deg")=0.f))
		.def("crop_to_roi", &Scene::pyCropToROI, (arg("min_num_points")=3));

	def("set_working_folder", &SetWorkingFolder);

	// Register SFM-side bindings (SfMScene + ImportConfig/ReconstructionConfig/...).
	pySFM::RegisterBindings();
} // BOOST_PYTHON_MODULE
/*----------------------------------------------------------------*/

} // namespace pyMVS

#endif // _USE_BOOST_PYTHON
