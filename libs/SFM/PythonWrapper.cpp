/*
* PythonWrapper.cpp
*
* Copyright (c) 2014-2026 SEACAVE
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

// Keep _USRDLL set: in shared builds we want __declspec(dllimport) so the
// wrapper resolves SFM/MVS/Common symbols via their import libs.
#include "Common.h"
#include "Scene.h"
#include "InterfaceMVS.h"
#ifndef BOOST_PYTHON_STATIC_LIB
#define BOOST_PYTHON_STATIC_LIB
#endif
#include <boost/python.hpp>


// S T R U C T S ///////////////////////////////////////////////////

namespace pySFM {

// Lightweight subclass of SFM::Scene that adds path-safe convenience methods,
// mirroring the pattern used by pyMVS::Scene in libs/MVS/PythonWrapper.cpp so
// Python users get the same WORKING_FOLDER-relative path handling.
class Scene : public SFM::Scene
{
public:
	Scene(unsigned _nMaxThreads=0) : SFM::Scene(_nMaxThreads) {
		INIT_WORKING_FOLDER;
	}

	bool pyLoad(const std::string& fileName, int archiveType=ARCHIVE_DEFAULT) {
		return Load(MAKE_PATH_SAFE(fileName), static_cast<ARCHIVE_TYPE>(archiveType));
	}
	bool pySave(const std::string& fileName, int archiveType=ARCHIVE_DEFAULT) const {
		return Save(MAKE_PATH_SAFE(fileName), static_cast<ARCHIVE_TYPE>(archiveType));
	}

	// Full incremental SfM pipeline: scan images from a folder (or
	// semicolon-separated list), extract features, match pairs, build tracks,
	// initialize, resect, and bundle-adjust into a 3D reconstruction.
	bool pyReconstruct(const std::string& source, const SFM::ReconstructionConfig& config) {
		return Reconstruct(MAKE_PATH_SAFE(source), config);
	}
	bool pyReconstructHierarchical(const SFM::ReconstructionConfig& config) {
		return ReconstructHierarchical(config);
	}
	bool pyReconstructGlobal(const SFM::ReconstructionConfig& config) {
		return ReconstructGlobal(config);
	}

	// Per-stage entry points (for fine-grained pipeline control).
	bool pyImport(const std::string& source, const SFM::ImportConfig& config) {
		return Import(MAKE_PATH_SAFE(source), config);
	}
	bool pyExtractFeatures(const SFM::FeatureExtractionConfig& config) {
		return ExtractFeatures(config);
	}
	bool pyMatchPairs(const SFM::MatchConfig& matchCfg,
	                  const SFM::ROMA2Config& roma2Cfg,
	                  const SFM::ViewGraphCalibratorConfig& vgConfig) {
		return MatchPairs(matchCfg, roma2Cfg, vgConfig);
	}

	bool pyAlignToGPS(double threshold=0.0) {
		return AlignToGPS(threshold);
	}
	bool pySampleColors() {
		return SampleColors();
	}

	// Read-only inspection helpers.
	unsigned pyNumImages() const   { return static_cast<unsigned>(images.size()); }
	unsigned pyNumCameras() const  { return static_cast<unsigned>(cameras.size()); }
	unsigned pyNumPairs() const    { return static_cast<unsigned>(pairs.size()); }
	unsigned pyNumTracks() const   { return static_cast<unsigned>(tracks.size()); }
	unsigned pyNumCalibrated() const { return status.nCalibratedImages; }
	bool pyIsEmpty() const { return IsEmpty(); }

	// Per-image metadata - returns a Python list of dicts so callers can
	// recover image and camera metadata without dragging the full C++ Image
	// type into Python; can be used for ex to map video keyframes
	// back to their source video frame index via timestamp*fps.
	boost::python::list pyGetImageRecords() const {
		boost::python::list out;
		for (size_t i = 0; i < images.size(); ++i) {
			const SFM::Image& img = images[i];
			boost::python::dict r;
			const SFM::Camera* pCamera = img.pCamera;
			r["id"]        = static_cast<uint32_t>(img.ID);
			r["camera_id"] = static_cast<uint32_t>(img.cameraID);
			r["file_name"] = static_cast<std::string>(img.fileName);
			r["timestamp"] = static_cast<double>(img.timestamp);
			r["has_pose"]  = img.HasPose();
			r["num_keypoints"] = static_cast<uint32_t>(img.keypoints.size());
			r["width"] = pCamera ? pCamera->GetWidth() : 0;
			r["height"] = pCamera ? pCamera->GetHeight() : 0;
			r["camera_type"] = static_cast<std::string>(
				pCamera ? SFM::CameraTypeToString(pCamera->GetType()) : SFM::CameraTypeToString(SFM::CameraType::UNDEFINED));
			out.append(r);
		}
		return out;
	}

	// Per-pair metadata - returns a Python list of tuples to keep allocation
	// overhead down when exporting large pair graphs. Tuple schema:
	// (id1, id2, num_matches, num_inliers, num_filtered_inliers,
	//  geometry_flags,
	//  overlap_ratio, overlap_area, mean_ray_angle,
	//  weight_spatial, weight_connectivity, weight_triplet, composite_weight)
	boost::python::list pyGetPairRecords() const {
		boost::python::list out;
		for (size_t i = 0; i < pairs.size(); ++i) {
			const SFM::ImagePair& pair = pairs[i];
			// bitmask: 1=relative_pose, 2=fundamental, 4=essential, 8=homography
			uint32_t geometryFlags = 0;
			if (pair.relativePose.has_value())
				geometryFlags |= 1u;
			if (pair.F.has_value())
				geometryFlags |= 2u;
			if (pair.E.has_value())
				geometryFlags |= 4u;
			if (pair.H.has_value())
				geometryFlags |= 8u;
			out.append(boost::python::make_tuple(
				static_cast<uint32_t>(pair.ID1),
				static_cast<uint32_t>(pair.ID2),
				pair.GetNumMatches(),
				pair.GetNumInliers(),
				pair.GetNumFilteredInliers(),
				geometryFlags,
				static_cast<double>(pair.overlapRatio),
				static_cast<double>(pair.overlapArea),
				static_cast<double>(pair.meanRayAngle),
				static_cast<double>(pair.weightSpatial),
				static_cast<double>(pair.weightConnectivity),
				static_cast<double>(pair.weightTriplet),
				static_cast<double>(pair.GetCompositeWeight())
			));
		}
		return out;
	}
};

// Bridge: feed an SFM result into an existing MVS::Scene via the canonical
// InterfaceMVS round-trip (writes a temp .mvs and loads it back). This is the
// pragmatic chain since SFM::Scene and MVS::Scene own different image/camera
// types — a direct in-memory converter would duplicate ExportMVS/ImportMVS.
static bool ExportToMVSFile(const Scene& scene, const std::string& fileName,
                            const SFM::ExportMVSConfig& config = {}) {
	return SFM::ExportMVS(MAKE_PATH_SAFE(fileName), scene, config);
}


// SEACAVE::String <-> Python str converters.
// SEACAVE::String publicly derives from std::string, but Boost.Python keys
// type converters by exact type_id and so the built-in std::string converter
// does NOT cover String. Without these registrations, every `def_readwrite`
// on a String field (e.g. ExportMVSConfig::undistortImageDir) returns an
// opaque "SEACAVE::String" object on read and raises
//   TypeError: No Python class registered for C++ class class SEACAVE::String
// on write. Registering this once makes all current and future String fields
// behave like ordinary Python strings.
struct StringToPython {
	static PyObject* convert(const SEACAVE::String& s) {
		return PyUnicode_FromStringAndSize(s.data(), static_cast<Py_ssize_t>(s.size()));
	}
};
struct StringFromPython {
	StringFromPython() {
		boost::python::converter::registry::push_back(
			&convertible, &construct, boost::python::type_id<SEACAVE::String>());
	}
	static void* convertible(PyObject* obj) {
		return (PyUnicode_Check(obj) || PyBytes_Check(obj)) ? obj : nullptr;
	}
	static void construct(PyObject* obj,
			boost::python::converter::rvalue_from_python_stage1_data* data) {
		const char* value;
		Py_ssize_t length = 0;
		if (PyUnicode_Check(obj)) {
			value = PyUnicode_AsUTF8AndSize(obj, &length);
			if (value == nullptr)
				boost::python::throw_error_already_set();
		} else {
			value = PyBytes_AsString(obj);
			length = PyBytes_GET_SIZE(obj);
		}
		void* storage = reinterpret_cast<
			boost::python::converter::rvalue_from_python_storage<SEACAVE::String>*>(data)->storage.bytes;
		new (storage) SEACAVE::String(value, static_cast<size_t>(length));
		data->convertible = storage;
	}
};

// Boost.Python registrar — called from inside the single
// BOOST_PYTHON_MODULE(pyOpenMVS) block in libs/MVS/PythonWrapper.cpp so that
// SFM and MVS bindings share one .pyd. Boost.Python only allows one module
// init function per shared library; multiple TUs collaborate via plain
// registrar functions invoked from inside that single init block.
void RegisterBindings()
{
	using namespace boost::python;

	to_python_converter<SEACAVE::String, StringToPython>();
	StringFromPython();

	// For SEACAVE::String members, def_readwrite would pick
	// return_internal_reference (the default for class-type members) and bypass
	// the to_python_converter registered above — yielding the same opaque
	// "SEACAVE::String" handle on reads. Routing through add_property with
	// return_by_value forces a copy that engages the converter.
	#define DEF_STR_RW(NAME, MEMBER_PTR) \
		add_property(NAME, \
			make_getter(MEMBER_PTR, return_value_policy<return_by_value>()), \
			make_setter(MEMBER_PTR))

	// SFM::ImportConfig — image import + camera priors
	class_<SFM::ImportConfig>("ImportConfig")
		.def_readwrite("use_exif", &SFM::ImportConfig::useExif)
		.def_readwrite("default_focal_ratio", &SFM::ImportConfig::defaultFocalRatio)
		.def_readwrite("focal_length", &SFM::ImportConfig::focalLength)
		.def_readwrite("k1", &SFM::ImportConfig::k1)
		.def_readwrite("k2", &SFM::ImportConfig::k2)
		.DEF_STR_RW("import_poses_csv", &SFM::ImportConfig::importPosesCSV)
		.def_readwrite("import_poses_mode", &SFM::ImportConfig::importPosesMode);

	// SFM::ROMA2Config — semi-dense matching
	class_<SFM::ROMA2Config>("ROMA2Config")
		.DEF_STR_RW("import_path", &SFM::ROMA2Config::importROMA2Path)
		.def_readwrite("min_pair_weight", &SFM::ROMA2Config::minPairWeight)
		.def_readwrite("epipolar_threshold", &SFM::ROMA2Config::epipolarThreshold)
		.def_readwrite("erode_border", &SFM::ROMA2Config::erodeBorder);

	// SFM::ViewGraphCalibratorConfig — focal-length verification
	class_<SFM::ViewGraphCalibratorConfig>("ViewGraphCalibratorConfig");

	// SFM::FeatureExtractionConfig — SIFT/AKAZE/ORB extraction
	class_<SFM::FeatureExtractionConfig>("FeatureExtractionConfig");

	// SFM::MatchConfig — pair matching strategy/thresholds
	class_<SFM::MatchConfig>("MatchConfig");

	// SFM::ReconstructionConfig — combined pipeline configuration
	class_<SFM::ReconstructionConfig>("ReconstructionConfig")
		.def_readwrite("import_cfg", &SFM::ReconstructionConfig::importCfg)
		.def_readwrite("features_cfg", &SFM::ReconstructionConfig::featuresCfg)
		.def_readwrite("roma2_cfg", &SFM::ReconstructionConfig::roma2Cfg)
		.def_readwrite("match_cfg", &SFM::ReconstructionConfig::matchCfg)
		.def_readwrite("match_images_only", &SFM::ReconstructionConfig::matchImagesOnly)
		.def_readwrite("viewgraph_cfg", &SFM::ReconstructionConfig::viewgraphCfg)
		.def_readwrite("min_pair_weight", &SFM::ReconstructionConfig::minPairWeight)
		.def_readwrite("max_reproj_error", &SFM::ReconstructionConfig::maxReprojError)
		.def_readwrite("max_fine_reproj_error", &SFM::ReconstructionConfig::maxFineReprojError)
		.def_readwrite("min_angle_threshold", &SFM::ReconstructionConfig::minAngleThreshold)
		.def_readwrite("use_global_solver", &SFM::ReconstructionConfig::useGlobalSolver)
		.def_readwrite("ba_intrinsic_flags", &SFM::ReconstructionConfig::baIntrinsicFlags)
		.def_readwrite("th_align_gps", &SFM::ReconstructionConfig::thAlignGPS)
		.def_readwrite("extract_colors", &SFM::ReconstructionConfig::extractColors);

	// ExportMVSConfig — undistortion + spherical cube-map options for ExportMVS
	class_<SFM::ExportMVSConfig>("ExportMVSConfig")
		.DEF_STR_RW("undistort_image_dir", &SFM::ExportMVSConfig::undistortImageDir)
		.DEF_STR_RW("extension", &SFM::ExportMVSConfig::extension)
		.def_readwrite("undistort_alpha", &SFM::ExportMVSConfig::undistortAlpha)
		.def_readwrite("only_inlier_tracks", &SFM::ExportMVSConfig::onlyInlierTracks)
		.def_readwrite("include_colors", &SFM::ExportMVSConfig::includeColors)
		.def_readwrite("spherical_face_size", &SFM::ExportMVSConfig::sphericalFaceSize)
		.def_readwrite("spherical_num_faces", &SFM::ExportMVSConfig::sphericalNumFaces);

	#undef DEF_STR_RW

	// pySFM::Scene — main entry point
	class_<Scene, boost::noncopyable, boost::shared_ptr<Scene>>(
			"SfMScene", init<unsigned>((arg("max_threads")=0)))
		.def("load", &Scene::pyLoad,
				(arg("file_path"), arg("archive_type")=static_cast<int>(ARCHIVE_DEFAULT)))
		.def("save", &Scene::pySave,
				(arg("file_path"), arg("archive_type")=static_cast<int>(ARCHIVE_DEFAULT)))
		.def("import_images", &Scene::pyImport,
				(arg("source"), arg("config")))
		.def("extract_features", &Scene::pyExtractFeatures, (arg("config")))
		.def("match_pairs", &Scene::pyMatchPairs,
				(arg("match_config"), arg("roma2_config"), arg("viewgraph_config")))
		.def("reconstruct", &Scene::pyReconstruct,
				(arg("source"), arg("config")))
		.def("reconstruct_hierarchical", &Scene::pyReconstructHierarchical, (arg("config")))
		.def("reconstruct_global", &Scene::pyReconstructGlobal, (arg("config")))
		.def("sample_colors", &Scene::pySampleColors)
		.def("align_to_gps", &Scene::pyAlignToGPS, (arg("threshold")=0.0))
		.def("export_to_mvs", &ExportToMVSFile,
				(arg("file_path"), arg("config")=SFM::ExportMVSConfig()))
		.add_property("num_images",      &Scene::pyNumImages)
		.add_property("num_cameras",     &Scene::pyNumCameras)
		.add_property("num_pairs",       &Scene::pyNumPairs)
		.add_property("num_tracks",      &Scene::pyNumTracks)
		.add_property("num_calibrated",  &Scene::pyNumCalibrated)
		.add_property("is_empty",        &Scene::pyIsEmpty)
		.def("get_image_records",        &Scene::pyGetImageRecords)
		.def("get_pair_records",         &Scene::pyGetPairRecords);

	// Free function: convenience SFM->MVS bridge via .mvs file.
	def("export_sfm_to_mvs", &ExportToMVSFile,
			(arg("scene"), arg("file_path"), arg("config")=SFM::ExportMVSConfig()));
}

} // namespace pySFM

#endif // _USE_BOOST_PYTHON
