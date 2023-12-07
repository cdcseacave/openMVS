/*
* Scene.cpp
*
* Copyright (c) 2014-2015 SEACAVE
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

#include "Common.h"
#include "Scene.h"
#include "../Math/SimilarityTransform.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define PROJECT_ID "MVS\0" // identifies the project stream
#define PROJECT_VER ((uint32_t)1) // identifies the version of a project stream

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define SCENE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

void Scene::Release()
{
	platforms.Release();
	images.Release();
	pointcloud.Release();
	mesh.Release();
}

bool Scene::IsValid() const
{
	return !platforms.IsEmpty() && !images.IsEmpty();
}

bool Scene::IsEmpty() const
{
	return pointcloud.IsEmpty() && mesh.IsEmpty();
}

bool Scene::ImagesHaveNeighbors() const
{
	for (const Image& image: images)
		if (!image.neighbors.IsEmpty())
			return true;
	return false;
}


bool Scene::LoadInterface(const String & fileName)
{
	TD_TIMER_STARTD();
	Interface obj;

	// serialize in the current state
	if (!ARCHIVE::SerializeLoad(obj, fileName))
		return false;

	// import platforms and cameras
	ASSERT(!obj.platforms.empty());
	platforms.reserve((uint32_t)obj.platforms.size());
	for (const Interface::Platform& itPlatform: obj.platforms) {
		Platform& platform = platforms.emplace_back();
		platform.name = itPlatform.name;
		platform.cameras.reserve((uint32_t)itPlatform.cameras.size());
		for (const Interface::Platform::Camera& itCamera: itPlatform.cameras) {
			Platform::Camera& camera = platform.cameras.emplace_back();
			camera.K = itCamera.K;
			camera.R = itCamera.R;
			camera.C = itCamera.C;
			if (!itCamera.IsNormalized()) {
				// normalize K
				ASSERT(itCamera.HasResolution());
				camera.K = camera.GetScaledK(REAL(1)/Camera::GetNormalizationScale(itCamera.width, itCamera.height));
			}
			DEBUG_EXTRA("Camera model loaded: platform %u; camera %2u; f %.3fx%.3f; poses %u", platforms.size()-1, platform.cameras.size()-1, camera.K(0,0), camera.K(1,1), itPlatform.poses.size());
		}
		ASSERT(platform.cameras.size() == itPlatform.cameras.size());
		platform.poses.reserve((uint32_t)itPlatform.poses.size());
		for (const Interface::Platform::Pose& itPose: itPlatform.poses) {
			Platform::Pose& pose = platform.poses.emplace_back();
			pose.R = itPose.R;
			pose.C = itPose.C;
		}
		ASSERT(platform.poses.size() == itPlatform.poses.size());
	}
	ASSERT(platforms.size() == obj.platforms.size());
	if (platforms.empty())
		return false;

	// import images
	nCalibratedImages = 0;
	size_t nTotalPixels(0);
	ASSERT(!obj.images.empty());
	images.reserve((uint32_t)obj.images.size());
	for (const Interface::Image& image: obj.images) {
		const uint32_t ID(images.size());
		Image& imageData = images.emplace_back();
		imageData.ID = (image.ID == NO_ID ? ID : image.ID);
		imageData.name = image.name;
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, imageData.name);
		if (!image.maskName.empty()) {
			imageData.maskName = image.maskName;
			Util::ensureUnifySlash(imageData.maskName);
			imageData.maskName = MAKE_PATH_FULL(WORKING_FOLDER_FULL, imageData.maskName);
		}
		imageData.poseID = image.poseID;
		if (imageData.poseID == NO_ID) {
			DEBUG_EXTRA("warning: uncalibrated image '%s'", image.name.c_str());
			continue;
		}
		imageData.platformID = image.platformID;
		imageData.cameraID = image.cameraID;
		// init camera
		const Interface::Platform::Camera& camera = obj.platforms[image.platformID].cameras[image.cameraID];
		if (camera.HasResolution()) {
			// use stored resolution
			imageData.width = camera.width;
			imageData.height = camera.height;
			imageData.scale = 1;
		} else {
			// read image header for resolution
			if (!imageData.ReloadImage(0, false))
				return false;
		}
		imageData.UpdateCamera(platforms);
		// init neighbors
		imageData.neighbors.CopyOf(image.viewScores.data(), (uint32_t)image.viewScores.size());
		imageData.avgDepth = image.avgDepth;
		++nCalibratedImages;
		nTotalPixels += imageData.width * imageData.height;
		DEBUG_ULTIMATE("Image loaded %3u: %s", ID, Util::getFileNameExt(imageData.name).c_str());
	}
	if (images.size() < 2)
		return false;

	// import 3D points
	if (!obj.vertices.empty()) {
		bool bValidWeights(false);
		pointcloud.points.resize(obj.vertices.size());
		pointcloud.pointViews.resize(obj.vertices.size());
		pointcloud.pointWeights.resize(obj.vertices.size());
		FOREACH(i, pointcloud.points) {
			const Interface::Vertex& vertex = obj.vertices[i];
			PointCloud::Point& point = pointcloud.points[i];
			point = vertex.X;
			PointCloud::ViewArr& views = pointcloud.pointViews[i];
			views.resize((PointCloud::ViewArr::IDX)vertex.views.size());
			PointCloud::WeightArr& weights = pointcloud.pointWeights[i];
			weights.resize((PointCloud::ViewArr::IDX)vertex.views.size());
			CLISTDEF0(PointCloud::ViewArr::IDX) indices(views.size());
			std::iota(indices.begin(), indices.end(), 0);
			std::sort(indices.begin(), indices.end(), [&](IndexArr::Type i0, IndexArr::Type i1) -> bool {
				return vertex.views[i0].imageID < vertex.views[i1].imageID;
			});
			ASSERT(vertex.views.size() >= 2);
			views.ForEach([&](PointCloud::ViewArr::IDX v) {
				const Interface::Vertex::View& view = vertex.views[indices[v]];
				views[v] = view.imageID;
				weights[v] = view.confidence;
				if (view.confidence != 0)
					bValidWeights = true;
			});
		}
		if (!bValidWeights)
			pointcloud.pointWeights.Release();
		if (!obj.verticesNormal.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesNormal.size());
			pointcloud.normals.CopyOf((const Point3f*)&obj.verticesNormal[0].n, obj.vertices.size());
		}
		if (!obj.verticesColor.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesColor.size());
			pointcloud.colors.CopyOf((const Pixel8U*)&obj.verticesColor[0].c, obj.vertices.size());
		}
	}

	// import region of interest
	obb.Set(Matrix3x3f(obj.obb.rot), Point3f(obj.obb.ptMin), Point3f(obj.obb.ptMax));

	DEBUG_EXTRA("Scene loaded from interface format (%s):\n"
				"\t%u images (%u calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.size(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.size(), mesh.vertices.size(), mesh.faces.size());
	return true;
} // LoadInterface

bool Scene::SaveInterface(const String & fileName, int version) const
{
	TD_TIMER_STARTD();
	Interface obj;

	// export platforms
	obj.platforms.reserve(platforms.size());
	for (const Platform& platform: platforms) {
		Interface::Platform plat;
		plat.cameras.reserve(platform.cameras.size());
		for (const Platform::Camera& camera: platform.cameras) {
			Interface::Platform::Camera cam;
			cam.K = camera.K;
			cam.R = camera.R;
			cam.C = camera.C;
			plat.cameras.emplace_back(cam);
		}
		plat.poses.reserve(platform.poses.size());
		for (const Platform::Pose& pose: platform.poses) {
			Interface::Platform::Pose p;
			p.R = pose.R;
			p.C = pose.C;
			plat.poses.emplace_back(p);
		}
		obj.platforms.emplace_back(std::move(plat));
	}

	// export images
	obj.images.resize(images.size());
	FOREACH(i, images) {
		const Image& imageData = images[i];
		MVS::Interface::Image& image = obj.images[i];
		image.name = MAKE_PATH_REL(WORKING_FOLDER_FULL, imageData.name);
		if (!imageData.maskName.empty())
			image.maskName = MAKE_PATH_REL(WORKING_FOLDER_FULL, imageData.maskName);
		image.poseID = imageData.poseID;
		image.platformID = imageData.platformID;
		image.cameraID = imageData.cameraID;
		image.ID = imageData.ID;
		if (imageData.IsValid() && imageData.HasResolution()) {
			Interface::Platform& platform = obj.platforms[image.platformID];;
			if (!platform.cameras[image.cameraID].HasResolution())
				platform.SetFullK(image.cameraID, imageData.camera.K, imageData.width, imageData.height);
		}
		image.viewScores = std::vector<MVS::Interface::Image::ViewScore>(imageData.neighbors.begin(), imageData.neighbors.end());
		image.avgDepth = imageData.avgDepth;
	}

	// export 3D points
	obj.vertices.resize(pointcloud.points.size());
	FOREACH(i, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[i];
		const PointCloud::ViewArr& views = pointcloud.pointViews[i];
		MVS::Interface::Vertex& vertex = obj.vertices[i];
		ASSERT(sizeof(vertex.X.x) == sizeof(point.x));
		vertex.X = point;
		vertex.views.resize(views.size());
		views.ForEach([&](PointCloud::ViewArr::IDX v) {
			MVS::Interface::Vertex::View& view = vertex.views[v];
			view.imageID = views[v];
			view.confidence = (pointcloud.pointWeights.IsEmpty() ? 0.f : pointcloud.pointWeights[i][v]);
		});
	}
	if (!pointcloud.normals.IsEmpty()) {
		obj.verticesNormal.resize(pointcloud.normals.size());
		FOREACH(i, pointcloud.normals) {
			const PointCloud::Normal& normal = pointcloud.normals[i];
			MVS::Interface::Normal& vertexNormal = obj.verticesNormal[i];
			vertexNormal.n = normal;
		}
	}
	if (!pointcloud.colors.IsEmpty()) {
		obj.verticesColor.resize(pointcloud.colors.size());
		FOREACH(i, pointcloud.colors) {
			const PointCloud::Color& color = pointcloud.colors[i];
			MVS::Interface::Color& vertexColor = obj.verticesColor[i];
			vertexColor.c = color;
		}
	}

	// export region of interest
	obj.obb.rot = Matrix3x3f(obb.m_rot);
	obj.obb.ptMin = Point3f((obb.m_pos-obb.m_ext).eval());
	obj.obb.ptMax = Point3f((obb.m_pos+obb.m_ext).eval());

	// serialize out the current state
	if (!ARCHIVE::SerializeSave(obj, fileName, version>=0?uint32_t(version):MVSI_PROJECT_VER))
		return false;

	DEBUG_EXTRA("Scene saved to interface format (%s):\n"
				"\t%u images (%u calibrated)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.size(), nCalibratedImages,
				pointcloud.points.size(), mesh.vertices.size(), mesh.faces.size());
	return true;
} // SaveInterface
/*----------------------------------------------------------------*/


// load depth-map and generate a Multi-View Stereo scene
bool Scene::LoadDMAP(const String& fileName)
{
	TD_TIMER_STARTD();

	// load depth-map data
	String imageFileName;
	IIndexArr IDs;
	cv::Size imageSize;
	Camera camera;
	Depth dMin, dMax;
	DepthMap depthMap;
	NormalMap normalMap;
	ConfidenceMap confMap;
	ViewsMap viewsMap;
	if (!ImportDepthDataRaw(fileName, imageFileName, IDs, imageSize, camera.K, camera.R, camera.C, dMin, dMax, depthMap, normalMap, confMap, viewsMap))
		return false;

	// create image
	Platform& platform = platforms.AddEmpty();
	platform.name = _T("platform0");
	platform.cameras.emplace_back(camera.GetScaledK(REAL(1)/CameraIntern::GetNormalizationScale((uint32_t)imageSize.width,(uint32_t)imageSize.height)), RMatrix::IDENTITY, CMatrix::ZERO);
	platform.poses.emplace_back(Platform::Pose{camera.R, camera.C});
	Image& image = images.AddEmpty();
	image.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, imageFileName);
	image.platformID = 0;
	image.cameraID = 0;
	image.poseID = 0;
	image.ID = IDs.front();
	image.scale = 1;
	image.avgDepth = (dMin+dMax)/2;
	image.width = (uint32_t)imageSize.width;
	image.height = (uint32_t)imageSize.height;
	image.UpdateCamera(platforms);
	nCalibratedImages = 1;

	// load image pixels
	const Image8U3 imageDepth(DepthMap2Image(depthMap));
	Image8U3 imageColor;
	if (image.ReloadImage(MAXF(image.width,image.height)))
		cv::resize(image.image, imageColor, depthMap.size());
	else
		imageColor = imageDepth;

	// create point-cloud
	camera.K = camera.GetScaledK(imageSize, depthMap.size());
	pointcloud.points.reserve(depthMap.area());
	pointcloud.pointViews.reserve(depthMap.area());
	pointcloud.colors.reserve(depthMap.area());
	if (!normalMap.empty())
		pointcloud.normals.reserve(depthMap.area());
	if (!confMap.empty())
		pointcloud.pointWeights.reserve(depthMap.area());
	for (int r=0; r<depthMap.rows; ++r) {
		for (int c=0; c<depthMap.cols; ++c) {
			const Depth depth = depthMap(r,c);
			if (depth <= 0)
				continue;
			pointcloud.points.emplace_back(camera.TransformPointI2W(Point3(c,r,depth)));
			pointcloud.pointViews.emplace_back(PointCloud::ViewArr{0});
			pointcloud.colors.emplace_back(imageColor(r,c));
			if (!normalMap.empty())
				pointcloud.normals.emplace_back(Cast<PointCloud::Normal::Type>(camera.R.t()*Cast<REAL>(normalMap(r,c))));
			if (!confMap.empty())
				pointcloud.pointWeights.emplace_back(PointCloud::WeightArr{confMap(r,c)});
		}
	}

	// replace color-image with depth-image
	image.image = imageDepth;
	cv::resize(image.image, image.image, imageSize);

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		ExportDepthMap(ComposeDepthFilePath(image.ID, "png"), depthMap);
		ExportConfidenceMap(ComposeDepthFilePath(image.ID, "conf.png"), confMap);
		ExportPointCloud(ComposeDepthFilePath(image.ID, "ply"), image, depthMap, normalMap);
		if (VERBOSITY_LEVEL > 4) {
			ExportNormalMap(ComposeDepthFilePath(image.ID, "normal.png"), normalMap);
			confMap.Save(ComposeDepthFilePath(image.ID, "conf.pfm"));
		}
	}
	#endif

	DEBUG_EXTRA("Scene loaded from depth-map format - %dx%d size, %.2f%%%% coverage (%s):\n"
		"\t1 images (1 calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
		"\t%u points, 0 lines",
		depthMap.width(), depthMap.height(), 100.0*pointcloud.GetSize()/depthMap.area(), TD_TIMER_GET_FMT().c_str(),
		(double)image.image.area()/(1024.0*1024.0), (double)image.image.area()/(1024.0*1024.0*nCalibratedImages),
		pointcloud.GetSize());
	return true;
} // LoadDMAP
/*----------------------------------------------------------------*/

// load a text list of views and their neighbors and assign them to the scene images;
// each line store the view ID followed by the 3+ closest view IDs, ordered in decreasing overlap:
//
// <cam-id> <neighbor-cam-id-0> <neighbor-cam-id-1> <neighbor-cam-id-2> <...>
// 
// for example:
// 0 1 2 3 4
// 1 0 2 3 4
// 2 1 3 0 4
// ...
bool Scene::LoadViewNeighbors(const String& fileName)
{
	TD_TIMER_STARTD();

	// parse image list
	SML smlImages(_T("ImageNeighbors"));
	smlImages.Load(fileName);
	ASSERT(smlImages.GetArrChildren().size() <= 1);

	// fetch image IDs list
	size_t argc;
	for (SML::const_iterator it=smlImages.begin(); it!=smlImages.end(); ++it) {
		// parse image element
		CAutoPtrArr<LPSTR> argv(Util::CommandLineToArgvA(it->second.val, argc));
		if (argc > 0 && argv[0][0] == _T('#'))
			continue;
		if (argc < 2) {
			VERBOSE("Invalid image IDs list: %s", it->second.val.c_str());
			continue;
		}
		// add neighbors to this image
		const IIndex ID(String::FromString<IIndex>(argv[0], NO_ID));
		ASSERT(ID != NO_ID);
		Image& imageData = images[ID];
		imageData.neighbors.resize(static_cast<IIndex>(argc-1));
		FOREACH(i, imageData.neighbors) {
			const IIndex nID(String::FromString<IIndex>(argv[i+1], NO_ID));
			ASSERT(nID != NO_ID);
			imageData.neighbors[i] = ViewScore{nID, 0, 1.f, FD2R(15.f), 0.5f, 3.f};
		}
	}

	DEBUG_EXTRA("View neighbors list loaded (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
} // LoadViewNeighbors
bool Scene::SaveViewNeighbors(const String& fileName) const
{
	ASSERT(ImagesHaveNeighbors());

	TD_TIMER_STARTD();

	File file(fileName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!file.isOpen()) {
		VERBOSE("error: unable to write file '%s'", fileName.c_str());
		return false;
	}
	FOREACH(ID, images) {
		const Image& imageData = images[ID];
		file.print("%u", ID);
		for (const ViewScore& neighbor: imageData.neighbors)
			file.print(" %u", neighbor.ID);
		file.print("\n");
	}

	DEBUG_EXTRA("View neighbors list saved (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
} // SaveViewNeighbors
/*----------------------------------------------------------------*/

// try to load known point-cloud or mesh files
bool Scene::Import(const String& fileName)
{
	const String ext(Util::getFileExt(fileName).ToLower());
	if (ext == _T(".dmap")) {
		// import point-cloud from dmap file
		Release();
		return LoadDMAP(fileName);
	}
	if (ext == _T(".obj") || ext == _T(".gltf") || ext == _T(".glb")) {
		// import mesh from obj/gltf file
		Release();
		return mesh.Load(fileName);
	}
	if (ext == _T(".ply")) {
		// import point-cloud/mesh from ply file
		Release();
		int nVertices(0), nFaces(0);
		{
		PLY ply;
		if (!ply.read(fileName)) {
			DEBUG_EXTRA("error: invalid PLY file");
			return false;
		}
		for (int i = 0; i < ply.get_elements_count(); ++i) {
			int elem_count;
			LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
			if (PLY::equal_strings("vertex", elem_name)) {
				nVertices = elem_count;
			} else
			if (PLY::equal_strings("face", elem_name)) {
				nFaces = elem_count;
			}
		}
		}
		if (nVertices && nFaces)
			return mesh.Load(fileName);
		if (nVertices)
			return pointcloud.Load(fileName);
	}
	return false;
} // Import
/*----------------------------------------------------------------*/

Scene::SCENE_TYPE Scene::Load(const String& fileName, bool bImport)
{
	TD_TIMER_STARTD();
	Release();

	#ifdef _USE_BOOST
	// open the input stream
	std::ifstream fs(fileName, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		return SCENE_NA;
	// load project header ID
	char szHeader[4];
	fs.read(szHeader, 4);
	if (!fs || _tcsncmp(szHeader, PROJECT_ID, 4) != 0) {
		fs.close();
		if (bImport && Import(fileName))
			return SCENE_IMPORT;
		if (LoadInterface(fileName))
			return SCENE_INTERFACE;
		VERBOSE("error: invalid project");
		return SCENE_NA;
	}
	// load project version
	uint32_t nVer;
	fs.read((char*)&nVer, sizeof(uint32_t));
	if (!fs || nVer != PROJECT_VER) {
		VERBOSE("error: different project version");
		return SCENE_NA;
	}
	// load stream type
	uint32_t nType;
	fs.read((char*)&nType, sizeof(uint32_t));
	// skip reserved bytes
	uint64_t nReserved;
	fs.read((char*)&nReserved, sizeof(uint64_t));
	// serialize in the current state
	if (!SerializeLoad(*this, fs, (ARCHIVE_TYPE)nType))
		return SCENE_NA;
	// init images
	nCalibratedImages = 0;
	size_t nTotalPixels(0);
	FOREACH(ID, images) {
		Image& imageData = images[ID];
		if (imageData.poseID == NO_ID)
			continue;
		imageData.UpdateCamera(platforms);
		++nCalibratedImages;
		nTotalPixels += imageData.width * imageData.height;
	}
	DEBUG_EXTRA("Scene loaded (%s):\n"
				"\t%u images (%u calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return SCENE_MVS;
	#else
	if (bImport && Import(fileName))
		return SCENE_IMPORT;
	if (LoadInterface(fileName))
		return SCENE_INTERFACE;
	return SCENE_NA;
	#endif
} // Load

bool Scene::Save(const String& fileName, ARCHIVE_TYPE type) const
{
	TD_TIMER_STARTD();
	// save using MVS interface if requested
	if (type == ARCHIVE_MVS) {
		if (mesh.IsEmpty())
			return SaveInterface(fileName);
		type = ARCHIVE_DEFAULT;
	}
	#ifdef _USE_BOOST
	// open the output stream
	std::ofstream fs(fileName, std::ios::out | std::ios::binary);
	if (!fs.is_open())
		return false;
	// save project ID
	fs.write(PROJECT_ID, 4);
	// save project version
	const uint32_t nVer(PROJECT_VER);
	fs.write((const char*)&nVer, sizeof(uint32_t));
	// save stream type
	const uint32_t nType = type;
	fs.write((const char*)&nType, sizeof(uint32_t));
	// reserve some bytes
	const uint64_t nReserved = 0;
	fs.write((const char*)&nReserved, sizeof(uint64_t));
	// serialize out the current state
	if (!SerializeSave(*this, fs, type))
		return false;
	DEBUG_EXTRA("Scene saved (%s):\n"
				"\t%u images (%u calibrated)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages,
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
	#else
	return false;
	#endif
} // Save
/*----------------------------------------------------------------*/


// compute point-cloud with visibility info from the existing mesh
void Scene::SampleMeshWithVisibility(unsigned maxResolution)
{
	ASSERT(!mesh.IsEmpty());
	const Depth thFrontDepth(0.985f);
	pointcloud.Release();
	pointcloud.points.resize(mesh.vertices.size());
	pointcloud.pointViews.resize(mesh.vertices.size());
	#ifdef SCENE_USE_OPENMP
	#pragma omp parallel for
	for (int64_t _ID=0; _ID<images.size(); ++_ID) {
		const IIndex ID(static_cast<IIndex>(_ID));
	#else
	FOREACH(ID, images) {
	#endif
		const Image& imageData = images[ID];
		unsigned level(0);
		const unsigned nMaxResolution(Image8U::computeMaxResolution(imageData.width, imageData.height, level, 0, maxResolution));
		const REAL scale(imageData.width > imageData.height ? (REAL)nMaxResolution/imageData.width : (REAL)nMaxResolution/imageData.height);
		const cv::Size scaledSize(Image8U::computeResize(imageData.GetSize(), scale));
		const Camera camera(imageData.GetCamera(platforms, scaledSize));
		DepthMap depthMap(scaledSize);
		mesh.Project(camera, depthMap);
		FOREACH(idxVertex, mesh.vertices) {
			const Point3f xz(camera.TransformPointW2I3(Cast<REAL>(mesh.vertices[idxVertex])));
			if (xz.z <= 0)
				continue;
			const Point2f x(xz.x, xz.y);
			if (depthMap.isInsideWithBorder<float,1>(x) && xz.z * thFrontDepth < depthMap(ROUND2INT(x))) {
				#ifdef SCENE_USE_OPENMP
				#pragma omp critical
				#endif
				pointcloud.pointViews[idxVertex].emplace_back(ID);
			}
		}
	}
	RFOREACH(idx, pointcloud.points) {
		if (pointcloud.pointViews[idx].size() < 2) {
			pointcloud.RemovePoint(idx);
			continue;
		}
		pointcloud.points[idx] = mesh.vertices[(Mesh::VIndex)idx];
		pointcloud.pointViews[idx].Sort();
	}
} // SampleMeshWithVisibility
/*----------------------------------------------------------------*/

bool Scene::ExportMeshToDepthMaps(const String& baseName)
{
	ASSERT(!images.empty() && !mesh.IsEmpty());
	const String ext(Util::getFileExt(baseName).ToLower());
	const int nType(ext == _T(".dmap") ? 2 : (ext == _T(".pfm") ? 1 : 0));
	if (nType == 2)
		mesh.ComputeNormalVertices();
	DepthMap depthMap;
	NormalMap normalMap;
	#ifdef SCENE_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for private(depthMap, normalMap) schedule(dynamic)
	for (int _i=0; _i<(int)images.size(); ++_i) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const IIndex idxImage((IIndex)_i);
	#else
	FOREACH(idxImage, images) {
	#endif
		Image& image = images[idxImage];
		if (!image.IsValid())
			continue;
		const unsigned imageSize(image.RecomputeMaxResolution(OPTDENSE::nResolutionLevel, OPTDENSE::nMinResolution, OPTDENSE::nMaxResolution));
		image.ResizeImage(imageSize);
		image.UpdateCamera(platforms);
		depthMap.create(image.GetSize());
		if (nType == 2)
			mesh.Project(image.camera, depthMap, normalMap);
		else
			mesh.Project(image.camera, depthMap);
		const String fileName(Util::insertBeforeFileExt(baseName, String::FormatString("%04u", image.ID)));
		if ((nType == 2 && ![&]() {
				IIndexArr IDs(0, image.neighbors.size()+1);
				IDs.push_back(idxImage);
				for (const ViewScore& neighbor: image.neighbors)
					IDs.push_back(neighbor.ID);
				return ExportDepthDataRaw(fileName, image.name, IDs, image.GetSize(), image.camera.K, image.camera.R, image.camera.C, 0.001f, FLT_MAX, depthMap, normalMap, ConfidenceMap(), ViewsMap());
			} ()) ||
			(nType == 1 && !depthMap.Save(fileName)) ||
			(nType == 0 && !ExportDepthMap(fileName, depthMap)))
		{
			#ifdef SCENE_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
	}
	#ifdef SCENE_USE_OPENMP
	if (bAbort)
		return false;
	#endif
	return true;
} // ExportMeshToDepthMaps
/*----------------------------------------------------------------*/


// create a virtual point-cloud to be used to initialize the neighbor view
// from image pair points at the intersection of the viewing directions
bool Scene::EstimateNeighborViewsPointCloud(unsigned maxResolution)
{
	constexpr Depth minPercentDepthPerturb(0.3f);
	constexpr Depth maxPercentDepthPerturb(1.3f);
	const auto ProjectGridToImage = [&](IIndex idI, IIndex idJ, Depth depth) {
		const Depth minDepthPerturb(depth * minPercentDepthPerturb);
		const Depth maxDepthPerturb(depth * maxPercentDepthPerturb);
		const Image& imageData = images[idI];
		const Image& imageData2 = images[idJ];
		const float stepW((float)imageData.width / maxResolution);
		const float stepH((float)imageData.height / maxResolution);
		for (unsigned r = 0; r < maxResolution; ++r) {
			for (unsigned c = 0; c < maxResolution; ++c) {
				const Point2f x(c*stepW + stepW/2, r*stepH + stepH/2);
				const Depth depthPerturb(randomRange(minDepthPerturb, maxDepthPerturb));
				const Point3 X(imageData.camera.TransformPointI2W(Point3(x.x, x.y, depthPerturb)));
				const Point3 X2(imageData2.camera.TransformPointW2C(X));
				if (X2.z < 0)
					continue;
				const Point2f x2(imageData2.camera.TransformPointC2I(X2));
				if (!Image8U::isInside(x2, imageData2.GetSize()))
					continue;
				pointcloud.points.emplace_back(X);
				pointcloud.pointViews.emplace_back(idI < idJ ? PointCloud::ViewArr{idI, idJ} : PointCloud::ViewArr{idJ, idI});
			}
		}
	};
	pointcloud.Release();
	FOREACH(i, images) {
		const Image& imageData = images[i];
		if (!imageData.IsValid())
			continue;
		FOREACH(j, images) {
			if (i == j)
				continue;
			const Image& imageData2 = images[j];
			Point3 X;
			TriangulatePoint3D(
				imageData.camera.K, imageData2.camera.K,
				imageData.camera.R, imageData2.camera.R,
				imageData.camera.C, imageData2.camera.C,
				Point2::ZERO, Point2::ZERO, X);
			const Depth depth((Depth)imageData.camera.PointDepth(X));
			const Depth depth2((Depth)imageData2.camera.PointDepth(X));
			if (depth <= 0 || depth2 <= 0)
				continue;
			ProjectGridToImage(i, j, depth);
			ProjectGridToImage(j, i, depth2);
		}
	}
	return true;
} // EstimateNeighborViewsPointCloud
/*----------------------------------------------------------------*/

// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
//  - nInsideROI: 0 - ignore ROI, 1 - weight more ROI points, 2 - consider only ROI points
bool Scene::SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, unsigned nInsideROI)
{
	ASSERT(points.empty());

	// extract the estimated 3D points and the corresponding 2D projections for the reference image
	Image& imageData = images[ID];
	ASSERT(imageData.IsValid());
	ViewScoreArr& neighbors = imageData.neighbors;
	ASSERT(neighbors.empty());
	struct Score {
		float score;
		float avgScale;
		float avgAngle;
		uint32_t points;
	};
	CLISTDEF0(Score) scores(images.size());
	scores.Memset(0);
	if (nMinPointViews > nCalibratedImages)
		nMinPointViews = nCalibratedImages;
	unsigned nPoints = 0;
	imageData.avgDepth = 0;
	const float sigmaAngleSmall(-1.f/(2.f*SQUARE(fOptimAngle*0.38f)));
	const float sigmaAngleLarge(-1.f/(2.f*SQUARE(fOptimAngle*0.7f)));
	const bool bCheckInsideROI(nInsideROI > 0 && IsBounded());
	FOREACH(idx, pointcloud.points) {
		const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
		ASSERT(views.IsSorted());
		if (views.FindFirst(ID) == PointCloud::ViewArr::NO_INDEX)
			continue;
		const PointCloud::Point& point = pointcloud.points[idx];
		float wROI(1.f);
		if (bCheckInsideROI && !obb.Intersects(point)) {
			if (nInsideROI > 1)
				continue;
			wROI = 0.7f;
		}
		const Depth depth((float)imageData.camera.PointDepth(point));
		ASSERT(depth > 0);
		if (depth <= 0)
			continue;
		// store this point
		if (views.size() >= nMinPointViews)
			points.push_back((uint32_t)idx);
		imageData.avgDepth += depth;
		++nPoints;
		// score shared views
		const Point3f V1(imageData.camera.C - Cast<REAL>(point));
		const float footprint1(imageData.camera.GetFootprintImage(point));
		for (const PointCloud::View& view: views) {
			if (view == ID)
				continue;
			const Image& imageData2 = images[view];
			const Point3f V2(imageData2.camera.C - Cast<REAL>(point));
			const float fAngle(ACOS(ComputeAngle(V1.ptr(), V2.ptr())));
			const float wAngle(EXP(SQUARE(fAngle-fOptimAngle)*(fAngle<fOptimAngle?sigmaAngleSmall:sigmaAngleLarge)));
			const float footprint2(imageData2.camera.GetFootprintImage(point));
			const float fScaleRatio(footprint1/footprint2);
			float wScale;
			if (fScaleRatio > 1.6f)
				wScale = SQUARE(1.6f/fScaleRatio);
			else if (fScaleRatio >= 1.f)
				wScale = 1.f;
			else
				wScale = SQUARE(fScaleRatio);
			Score& score = scores[view];
			score.score += MAXF(wAngle,0.1f) * wScale * wROI;
			score.avgScale += fScaleRatio;
			score.avgAngle += fAngle;
			++score.points;
		}
	}
	if(nPoints > 3)
		imageData.avgDepth /= nPoints;

	// select best neighborViews
	if (neighbors.empty()) {
		Point2fArr projs(0, points.size());
		FOREACH(IDB, images) {
			const Image& imageDataB = images[IDB];
			if (!imageDataB.IsValid())
				continue;
			const Score& score = scores[IDB];
			if (score.points < 3)
				continue;
			ASSERT(ID != IDB);
			// compute how well the matched features are spread out (image covered area)
			const Point2f boundsA(imageData.GetSize());
			const Point2f boundsB(imageDataB.GetSize());
			ASSERT(projs.empty());
			for (uint32_t idx: points) {
				const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
				ASSERT(views.IsSorted());
				ASSERT(views.FindFirst(ID) != PointCloud::ViewArr::NO_INDEX);
				if (views.FindFirst(IDB) == PointCloud::ViewArr::NO_INDEX)
					continue;
				const PointCloud::Point& point = pointcloud.points[idx];
				Point2f& ptA = projs.emplace_back(imageData.camera.ProjectPointP(point));
				Point2f ptB = imageDataB.camera.ProjectPointP(point);
				if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB))
					projs.RemoveLast();
			}
			ASSERT(projs.size() <= score.points);
			if (projs.empty())
				continue;
			const float area(ComputeCoveredArea<float,2,16,false>((const float*)projs.data(), projs.size(), boundsA.ptr()));
			projs.Empty();
			// store image score
			ViewScore& neighbor = neighbors.AddEmpty();
			neighbor.ID = IDB;
			neighbor.points = score.points;
			neighbor.scale = score.avgScale/score.points;
			neighbor.angle = score.avgAngle/score.points;
			neighbor.area = area;
			neighbor.score = score.score*MAXF(area,0.01f);
		}
		neighbors.Sort([](const ViewScore& i, const ViewScore& j) {
			return i.score > j.score;
		});
		#if TD_VERBOSE != TD_VERBOSE_OFF
		// print neighbor views
		if (VERBOSITY_LEVEL > 2) {
			String msg;
			FOREACH(n, neighbors)
				msg += String::FormatString(" %3u(%upts,%.2fscl)", neighbors[n].ID, neighbors[n].points, neighbors[n].scale);
			VERBOSE("Reference image %3u sees %u views:%s (%u shared points)", ID, neighbors.size(), msg.c_str(), nPoints);
		}
		#endif
	}
	if (points.size() <= 3 || neighbors.size() < MINF(nMinViews,nCalibratedImages-1)) {
		DEBUG_EXTRA("error: reference image %3u has not enough images in view", ID);
		return false;
	}
	return true;
} // SelectNeighborViews

void Scene::SelectNeighborViews(unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle, unsigned nInsideROI)
{
	#ifdef SCENE_USE_OPENMP
	for (int_t ID=0; ID<(int_t)images.size(); ++ID) {
		const IIndex idxImage((IIndex)ID);
	#else
	FOREACH(idxImage, images) {
	#endif
		// select image neighbors
		IndexArr points;
		SelectNeighborViews(idxImage, points, nMinViews, nMinPointViews, fOptimAngle, nInsideROI);
	}
} // SelectNeighborViews
/*----------------------------------------------------------------*/


// keep only the best neighbors for the reference image
bool Scene::FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea, float fMinScale, float fMaxScale, float fMinAngle, float fMaxAngle, unsigned nMaxViews)
{
	// remove invalid neighbor views
	const unsigned nMinViews(MAXF(4u, nMaxViews*3/4));
	RFOREACH(n, neighbors) {
		const ViewScore& neighbor = neighbors[n];
		if (neighbors.size() > nMinViews &&
			(neighbor.area < fMinArea ||
			 !ISINSIDE(neighbor.scale, fMinScale, fMaxScale) ||
			 !ISINSIDE(neighbor.angle, fMinAngle, fMaxAngle)))
			neighbors.RemoveAtMove(n);
	}
	if (neighbors.size() > nMaxViews)
		neighbors.resize(nMaxViews);
	return !neighbors.empty();
} // FilterNeighborViews
/*----------------------------------------------------------------*/


// export all estimated cameras in a MeshLab MLP project as raster layers
bool Scene::ExportCamerasMLP(const String& fileName, const String& fileNameScene) const
{
	static const char mlp_header[] =
		"<!DOCTYPE MeshLabDocument>\n"
		"<MeshLabProject>\n"
		" <MeshGroup>\n"
		"  <MLMesh label=\"%s\" filename=\"%s\">\n"
		"   <MLMatrix44>\n"
		"1 0 0 0 \n"
		"0 1 0 0 \n"
		"0 0 1 0 \n"
		"0 0 0 1 \n"
		"   </MLMatrix44>\n"
		"  </MLMesh>\n"
		" </MeshGroup>\n";
	static const char mlp_raster_pos[] =
		"  <MLRaster label=\"%s\">\n"
		"   <VCGCamera TranslationVector=\"%0.6g %0.6g %0.6g 1\"";
	static const char mlp_raster_cam[] =
		" LensDistortion=\"%0.6g %0.6g\""
		" ViewportPx=\"%u %u\""
		" PixelSizeMm=\"1 %0.4f\""
		" FocalMm=\"%0.4f\""
		" CenterPx=\"%0.4f %0.4f\"";
	static const char mlp_raster_rot[] =
		" RotationMatrix=\"%0.6g %0.6g %0.6g 0 %0.6g %0.6g %0.6g 0 %0.6g %0.6g %0.6g 0 0 0 0 1\"/>\n"
		"   <Plane semantic=\"\" fileName=\"%s\"/>\n"
		"  </MLRaster>\n";

	Util::ensureFolder(fileName);
	File f(fileName, File::WRITE, File::CREATE | File::TRUNCATE);

	// write MLP header containing the referenced PLY file
	f.print(mlp_header, Util::getFileName(fileNameScene).c_str(), MAKE_PATH_REL(WORKING_FOLDER_FULL, fileNameScene).c_str());

	// write the raster layers
	f <<  " <RasterGroup>\n";
	FOREACH(i, images) {
		const Image& imageData = images[i];
		// skip invalid, uncalibrated or discarded images
		if (!imageData.IsValid())
			continue;
		const Camera& camera = imageData.camera;
		f.print(mlp_raster_pos,
			Util::getFileName(imageData.name).c_str(),
			-camera.C.x, -camera.C.y, -camera.C.z
		);
		f.print(mlp_raster_cam,
			0, 0,
			imageData.width, imageData.height,
			camera.K(1,1)/camera.K(0,0), camera.K(0,0),
			camera.K(0,2), camera.K(1,2)
		);
		f.print(mlp_raster_rot,
			 camera.R(0,0),  camera.R(0,1),  camera.R(0,2),
			-camera.R(1,0), -camera.R(1,1), -camera.R(1,2),
			-camera.R(2,0), -camera.R(2,1), -camera.R(2,2),
			MAKE_PATH_REL(WORKING_FOLDER_FULL, imageData.name).c_str()
		);
	}
	f << " </RasterGroup>\n</MeshLabProject>\n";

	return true;
} // ExportCamerasMLP

bool Scene::ExportLinesPLY(const String& fileName, const CLISTDEF0IDX(Line3f,uint32_t)& lines, const Pixel8U* colors, bool bBinary) {
	// define a PLY file format composed only of vertices and edges
	// vertex definition
	struct PLYVertex {
		float x, y, z;
	};
	// list of property information for a vertex
	static PLY::PlyProperty vert_props[] = {
		{"x", PLY::Float32, PLY::Float32, offsetof(PLYVertex,x), 0, 0, 0, 0},
		{"y", PLY::Float32, PLY::Float32, offsetof(PLYVertex,y), 0, 0, 0, 0},
		{"z", PLY::Float32, PLY::Float32, offsetof(PLYVertex,z), 0, 0, 0, 0},
	};
	// edge definition
	struct PLYEdge {
		int v1, v2;
		uint8_t r, g, b;
	};
	// list of property information for a edge
	static PLY::PlyProperty edge_props[] = {
		{"vertex1", PLY::Uint32, PLY::Uint32, offsetof(PLYEdge,v1), 0, 0, 0, 0},
		{"vertex2", PLY::Uint32, PLY::Uint32, offsetof(PLYEdge,v2), 0, 0, 0, 0},
		{"red", PLY::Uint8, PLY::Uint8, offsetof(PLYEdge,r), 0, 0, 0, 0},
		{"green", PLY::Uint8, PLY::Uint8, offsetof(PLYEdge,g), 0, 0, 0, 0},
		{"blue", PLY::Uint8, PLY::Uint8, offsetof(PLYEdge,b), 0, 0, 0, 0},
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex", "edge"
	};

	// create PLY object
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	const size_t memBufferSize(2 * (8 * 3/*pos*/ + 3 * 3/*color*/ + 6/*space*/ + 2/*eol*/) + 2048/*extra size*/);
	PLY ply;
	if (!ply.write(fileName, 2, elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII, memBufferSize))
		return false;

	// describe what properties go into the vertex elements
	ply.describe_property("vertex", 3, vert_props);
	PLYVertex v;
	FOREACH(i, lines) {
		const Line3f& line = lines[i];
		v.x = line.pt1.x(); v.y = line.pt1.y(); v.z = line.pt1.z();
		ply.put_element(&v);
		v.x = line.pt2.x(); v.y = line.pt2.y(); v.z = line.pt2.z();
		ply.put_element(&v);
	}
	
	// describe what properties go into the edge elements
	if (colors) {
		ply.describe_property("edge", 5, edge_props);
		PLYEdge edge;
		FOREACH(i, lines) {
			const Pixel8U& color = colors[i];
			edge.r = color.r; edge.g = color.g; edge.b = color.b;
			edge.v1 = i*2+0; edge.v2 = i*2+1;
			ply.put_element(&edge);
		}
	} else {
		ply.describe_property("edge", 2, edge_props);
		PLYEdge edge;
		FOREACH(i, lines) {
			edge.v1 = i*2+0; edge.v2 = i*2+1;
			ply.put_element(&edge);
		}
	}
	
	// write to file
	return ply.header_complete();
} // ExportLinesPLY
/*----------------------------------------------------------------*/


// split the scene in sub-scenes such that each sub-scene surface does not exceed the given
// maximum sampling area; the area is composed of overlapping samples from different cameras
// taking into account the footprint of each sample (pixels/unit-length, GSD inverse),
// including overlapping samples;
// the indirect goals this method tries to achieve are:
//  - limit the maximum number of images in each sub-scene such that the depth-map fusion
//    can load all sub-scene's depth-maps into memory at once
//  - limit in the same time maximum accumulated images resolution (total number of pixels)
//    per sub-scene in order to allow all images to be loaded and processed during mesh refinement
unsigned Scene::Split(ImagesChunkArr& chunks, float maxArea, int depthMapStep) const
{
	TD_TIMER_STARTD();
	// gather samples from all depth-maps
	const float areaScale(0.01f);
	typedef cList<Point3f::EVec,const Point3f::EVec&,0,4096,uint32_t> Samples;
	typedef TOctree<Samples,float,3> Octree;
	Octree octree;
	FloatArr areas(0, images.size()*4192);
	IIndexArr visibility(0, (IIndex)areas.capacity());
	Unsigned32Arr imageAreas(images.size()); {
		Samples samples(0, (uint32_t)areas.capacity());
		FOREACH(idxImage, images) {
			const Image& imageData = images[idxImage];
			if (!imageData.IsValid())
				continue;
			DepthData depthData;
			depthData.Load(ComposeDepthFilePath(imageData.ID, "dmap"), 1);
			if (depthData.IsEmpty())
				continue;
			const IIndex numPointsBegin(visibility.size());
			const Camera camera(imageData.GetCamera(platforms, depthData.depthMap.size()));
			for (int r=(depthData.depthMap.rows%depthMapStep)/2; r<depthData.depthMap.rows; r+=depthMapStep) {
				for (int c=(depthData.depthMap.cols%depthMapStep)/2; c<depthData.depthMap.cols; c+=depthMapStep) {
					const Depth depth = depthData.depthMap(r,c);
					if (depth <= 0)
						continue;
					const Point3f X(Cast<float>(camera.TransformPointI2W(Point3(c,r,depth))));
					if (IsBounded() && !obb.Intersects(X))
						continue;
					areas.emplace_back(camera.GetFootprintImage(X)*areaScale);
					visibility.emplace_back(idxImage);
					samples.emplace_back(X);
				}
			}
			imageAreas[idxImage] = visibility.size()-numPointsBegin;
		}
		const AABB3f aabb(IsBounded() ? obb.GetAABB() : [&samples]() {
			#if 0
			return AABB3f(samples.data(), samples.size());
			#else
			// try to find a dominant plane, and set the bounding-box center on the plane bottom
			OBB3f obbSamples(samples.data(), samples.size());
			obbSamples.m_ext(0) *= 2;
			#if 1
			// dump box for visualization
			OBB3f::POINT pts[8];
			obbSamples.GetCorners(pts);
			PointCloud pc;
			for (int i=0; i<8; ++i)
				pc.points.emplace_back(pts[i]);
			pc.Save(MAKE_PATH("scene_obb.ply"));
			#endif
			return obbSamples.GetAABB();
			#endif
		}());
		octree.Insert(samples, aabb, [](Octree::IDX_TYPE size, Octree::Type /*radius*/) {
			return size > 128;
		});
		#if 0 && !defined(_RELEASE)
		Octree::DEBUGINFO_TYPE info;
		octree.GetDebugInfo(&info);
		Octree::LogDebugInfo(info);
		#endif
		octree.ResetItems();
	}
	struct AreaInserter {
		const FloatArr& areas;
		float area;
		inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
			FOREACHRAWPTR(pIdx, indices, size)
				area += areas[*pIdx];
		}
		inline float PopArea() {
			const float a(area);
			area = 0;
			return a;
		}
	} areaEstimator{areas, 0.f};
	struct ChunkInserter {
		const IIndex numImages;
		const Octree& octree;
		const IIndexArr& visibility;
		ImagesChunkArr& chunks;
		CLISTDEF2(Unsigned32Arr) imagesAreas;
		void operator() (const Octree::CELL_TYPE& parentCell, Octree::Type parentRadius, const UnsignedArr& children) {
			ASSERT(!children.empty());
			ImagesChunk& chunk = chunks.AddEmpty();
			Unsigned32Arr& imageAreas = imagesAreas.AddEmpty();
			imageAreas.resize(numImages);
			imageAreas.Memset(0);
			struct Inserter {
				const IIndexArr& visibility;
				std::unordered_set<IIndex>& images;
				Unsigned32Arr& imageAreas;
				inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
					FOREACHRAWPTR(pIdx, indices, size) {
						const IIndex idxImage(visibility[*pIdx]);
						images.emplace(idxImage);
						++imageAreas[idxImage];
					}
				}
			} inserter{visibility, chunk.images, imageAreas};
			if (children.size() == 1) {
				octree.CollectCells(parentCell.GetChild(children.front()), inserter);
				chunk.aabb = parentCell.GetChildAabb(children.front(), parentRadius);
			} else {
				chunk.aabb.Reset();
				for (unsigned c: children) {
					octree.CollectCells(parentCell.GetChild(c), inserter);
					chunk.aabb.Insert(parentCell.GetChildAabb(c, parentRadius));
				}
			}
			if (chunk.images.empty()) {
				chunks.RemoveLast();
				imagesAreas.RemoveLast();
			}
		}
	} chunkInserter{images.size(), octree, visibility, chunks};
	octree.SplitVolume(maxArea, areaEstimator, chunkInserter);
	if (chunks.size() < 2)
		return 0;
	// remove images with very little contribution
	const float minImageContributionRatio(0.3f);
	FOREACH(c, chunks) {
		ImagesChunk& chunk = chunks[c];
		const Unsigned32Arr& chunkImageAreas = chunkInserter.imagesAreas[c];
		float maxAreaRatio = 0;
		for (const IIndex idxImage : chunk.images) {
			const float areaRatio(static_cast<float>(chunkImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]));
			if (maxAreaRatio < areaRatio)
				maxAreaRatio = areaRatio;
		}
		const float minImageContributionRatioChunk(maxAreaRatio * minImageContributionRatio);
		for (auto it = chunk.images.begin(); it != chunk.images.end(); ) {
			const IIndex idxImage(*it);
			if (static_cast<float>(chunkImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]) < minImageContributionRatioChunk)
				it = chunk.images.erase(it);
			else
				++it;
		}
	}
	#if 1
	// remove images already completely contained by a larger chunk
	const float minImageContributionRatioLargerChunk(0.9f);
	FOREACH(cSmall, chunks) {
		ImagesChunk& chunkSmall = chunks[cSmall];
		const Unsigned32Arr& chunkSmallImageAreas = chunkInserter.imagesAreas[cSmall];
		FOREACH(cLarge, chunks) {
			const ImagesChunk& chunkLarge = chunks[cLarge];
			if (chunkLarge.images.size() <= chunkSmall.images.size())
				continue;
			const Unsigned32Arr& chunkLargeImageAreas = chunkInserter.imagesAreas[cLarge];
			for (auto it = chunkSmall.images.begin(); it != chunkSmall.images.end(); ) {
				const IIndex idxImage(*it);
				if (chunkSmallImageAreas[idxImage] < chunkLargeImageAreas[idxImage] &&
					static_cast<float>(chunkLargeImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]) > minImageContributionRatioLargerChunk)
					it = chunkSmall.images.erase(it);
				else
					++it;
			}
		}
	}
	#endif
	#if 1
	// merge small chunks into larger chunk neighbors
	// TODO: better manage the bounding-box merge
	const unsigned minNumImagesPerChunk(4);
	RFOREACH(cSmall, chunks) {
		ImagesChunk& chunkSmall = chunks[cSmall];
		if (chunkSmall.images.size() > minNumImagesPerChunk)
			continue;
		// find the chunk having the most images in common
		IIndex idxBestChunk;
		unsigned numLargestCommonImages(0);
		FOREACH(cLarge, chunks) {
			if (cSmall == cLarge)
				continue;
			const ImagesChunk& chunkLarge = chunks[cLarge];
			unsigned numCommonImages(0);
			for (const IIndex idxImage: chunkSmall.images)
				if (chunkLarge.images.find(idxImage) != chunkLarge.images.end())
					++numCommonImages;
			if (numCommonImages == 0)
				continue;
			if (numLargestCommonImages < numCommonImages ||
				(numLargestCommonImages == numCommonImages && chunks[idxBestChunk].images.size() < chunkLarge.images.size()))
			{
				numLargestCommonImages = numCommonImages;
				idxBestChunk = cLarge;
			}
		}
		if (numLargestCommonImages == 0) {
			DEBUG_ULTIMATE("warning: small chunk can not be merged (%u chunk, %u images)",
				cSmall, chunkSmall.images.size());
			continue;
		}
		// merge the small chunk and remove it
		ImagesChunk& chunkLarge = chunks[idxBestChunk];
		DEBUG_ULTIMATE("Small chunk merged: %u chunk (%u images) -> %u chunk (%u images)",
			cSmall, chunkSmall.images.size(), idxBestChunk, chunkLarge.images.size());
		chunkLarge.aabb.Insert(chunkSmall.aabb);
		chunkLarge.images.insert(chunkSmall.images.begin(), chunkSmall.images.end());
		chunks.RemoveAt(cSmall);
	}
	#endif
	if (IsBounded()) {
		// make sure the chunks bounding box do not exceed the scene bounding box
		const AABB3f aabb(obb.GetAABB());
		RFOREACH(c, chunks) {
			ImagesChunk& chunk = chunks[c];
			chunk.aabb.BoundBy(aabb);
			if (chunk.aabb.IsEmpty()) {
				DEBUG_ULTIMATE("warning: chunk bounding box is empty");
				chunks.RemoveAt(c);
			}
		}
	}
	DEBUG_EXTRA("Scene split (%g max-area): %u chunks (%s)", maxArea, chunks.size(), TD_TIMER_GET_FMT().c_str());
	#if 0 || defined(_DEBUG)
	// dump chunks for visualization
	FOREACH(c, chunks) {
		const ImagesChunk& chunk = chunks[c];
		PointCloud pc = pointcloud;
		pc.RemovePointsOutside(OBB3f(OBB3f::MATRIX::Identity(), chunk.aabb.ptMin, chunk.aabb.ptMax));
		pc.Save(String::FormatString(MAKE_PATH("scene_%04u.ply"), c));
	}
	#endif
	return chunks.size();
} // Split

// split the scene in sub-scenes according to the given chunks array, and save them to disk
bool Scene::ExportChunks(const ImagesChunkArr& chunks, const String& path, ARCHIVE_TYPE type) const
{
	FOREACH(chunkID, chunks) {
		const ImagesChunk& chunk = chunks[chunkID];
		Scene subset;
		subset.nCalibratedImages = (IIndex)chunk.images.size();
		// extract chunk images
		typedef std::unordered_map<IIndex,IIndex> MapIIndex;
		MapIIndex mapPlatforms(platforms.size());
		MapIIndex mapImages(images.size());
		FOREACH(idxImage, images) {
			if (chunk.images.find(idxImage) == chunk.images.end())
				continue;
			const Image& image = images[idxImage];
			if (!image.IsValid())
				continue;
			// copy platform
			const Platform& platform = platforms[image.platformID];
			MapIIndex::iterator itSubPlatformMVS = mapPlatforms.find(image.platformID);
			uint32_t subPlatformID;
			if (itSubPlatformMVS == mapPlatforms.end()) {
				ASSERT(subset.platforms.size() == mapPlatforms.size());
				subPlatformID = subset.platforms.size();
				mapPlatforms.emplace(image.platformID, subPlatformID);
				Platform subPlatform;
				subPlatform.name = platform.name;
				subPlatform.cameras = platform.cameras;
				subset.platforms.emplace_back(std::move(subPlatform));
			} else {
				subPlatformID = itSubPlatformMVS->second;
			}
			Platform& subPlatform = subset.platforms[subPlatformID];
			// copy image
			const IIndex idxImageNew((IIndex)mapImages.size());
			mapImages[idxImage] = idxImageNew;
			Image subImage(image);
			subImage.platformID = subPlatformID;
			subImage.poseID = subPlatform.poses.size();
			subImage.ID = idxImage;
			subset.images.emplace_back(std::move(subImage));
			// copy pose
			subPlatform.poses.emplace_back(platform.poses[image.poseID]);
		}
		// map image IDs from global to local
		for (Image& image: subset.images) {
			RFOREACH(i, image.neighbors) {
				ViewScore& neighbor = image.neighbors[i];
				const auto itImage(mapImages.find(neighbor.ID));
				if (itImage == mapImages.end()) {
					image.neighbors.RemoveAtMove(i);
					continue;
				}
				ASSERT(itImage->second < subset.images.size());
				neighbor.ID = itImage->second;
			}
		}
		// extract point-cloud
		FOREACH(idxPoint, pointcloud.points) {
			PointCloud::ViewArr subViews;
			PointCloud::WeightArr subWeights;
			const PointCloud::ViewArr& views = pointcloud.pointViews[idxPoint];
			FOREACH(i, views) {
				const IIndex idxImage(views[i]);
				const auto itImage(mapImages.find(idxImage));
				if (itImage == mapImages.end())
					continue;
				subViews.emplace_back(itImage->second);
				if (!pointcloud.pointWeights.empty())
					subWeights.emplace_back(pointcloud.pointWeights[idxPoint][i]);
			}
			if (subViews.size() < 2)
				continue;
			subset.pointcloud.points.emplace_back(pointcloud.points[idxPoint]);
			subset.pointcloud.pointViews.emplace_back(std::move(subViews));
			if (!pointcloud.pointWeights.empty())
				subset.pointcloud.pointWeights.emplace_back(std::move(subWeights));
			if (!pointcloud.colors.empty())
				subset.pointcloud.colors.emplace_back(pointcloud.colors[idxPoint]);
		}
		// set scene ROI
		subset.obb.Set(OBB3f::MATRIX::Identity(), chunk.aabb.ptMin, chunk.aabb.ptMax);
		// serialize out the current state
		if (!subset.Save(String::FormatString("%s" PATH_SEPARATOR_STR "scene_%04u.mvs", path.c_str(), chunkID), type))
			return false;
	}
	return true;
} // ExportChunks
/*----------------------------------------------------------------*/


// move scene such that the center is the given point;
// if center is not given, center it to the center of the bounding-box
bool Scene::Center(const Point3* pCenter)
{
	Point3 center;
	if (pCenter)
		center = *pCenter;
	else if (IsBounded())
		center = -Point3f(obb.GetCenter());
	else if (!pointcloud.IsEmpty())
		center = -Point3f(pointcloud.GetAABB().GetCenter());
	else if (!mesh.IsEmpty())
		center = -Point3f(mesh.GetAABB().GetCenter());
	else
		return false;
	const Point3f centerf(Cast<float>(center));
	if (IsBounded())
		obb.Translate(centerf);
	for (Platform& platform: platforms)
		for (Platform::Pose& pose: platform.poses)
			pose.C += center;
	for (Image& image: images)
		if (image.IsValid())
			image.UpdateCamera(platforms);
	for (PointCloud::Point& X: pointcloud.points)
		X += centerf;
	for (Mesh::Vertex& X: mesh.vertices)
		X += centerf;
	return true;
} // Center

// scale scene with the given scale;
// if the scale is not given, scale it such that the bounding-box has largest size 1
bool Scene::Scale(const REAL* pScale)
{
	REAL scale;
	if (pScale)
		scale = *pScale;
	else if (IsBounded())
		scale = REAL(1)/obb.GetSize().maxCoeff();
	else if (!pointcloud.IsEmpty())
		scale = REAL(1)/pointcloud.GetAABB().GetSize().maxCoeff();
	else if (!mesh.IsEmpty())
		scale = REAL(1)/mesh.GetAABB().GetSize().maxCoeff();
	else
		return false;
	const float scalef(static_cast<float>(scale));
	if (IsBounded())
		obb.Transform(OBB3f::MATRIX::Identity() * scalef);
	for (Platform& platform: platforms)
		for (Platform::Pose& pose: platform.poses)
			pose.C *= scale;
	for (Image& image: images)
		if (image.IsValid())
			image.UpdateCamera(platforms);
	for (PointCloud::Point& X: pointcloud.points)
		X *= scalef;
	for (Mesh::Vertex& X: mesh.vertices)
		X *= scalef;
	return true;
} // Scale

// scale image resolutions with the given scale or max-resolution;
// if folderName is specified, the scaled images are stored there
bool Scene::ScaleImages(unsigned nMaxResolution, REAL scale, const String& folderName)
{
	ASSERT(nMaxResolution > 0 || scale > 0);
	Util::ensureFolder(folderName);
	FOREACH(idx, images) {
		Image& image = images[idx];
		if (!image.IsValid())
			continue;
		unsigned nResolutionLevel(0);
		unsigned nResolution(image.RecomputeMaxResolution(nResolutionLevel, 0));
		if (scale > 0)
			nResolution = ROUND2INT(nResolution*scale);
		if (nMaxResolution > 0 && nResolution > nMaxResolution)
			nResolution = nMaxResolution;
		if (!image.ReloadImage(nResolution, !folderName.empty()))
			return false;
		image.UpdateCamera(platforms);
		if (!folderName.empty()) {
			if (image.ID == NO_ID)
				image.ID = idx;
			image.name = folderName + String::FormatString("%05u%s", image.ID, Util::getFileExt(image.name).c_str());
			image.image.Save(image.name);
			image.ReleaseImage();
		}
	}
	return true;
} // ScaleImages

// apply similarity transform
void Scene::Transform(const Matrix3x3& rotation, const Point3& translation, REAL scale)
{
	const Matrix3x3 rotationScale(rotation * scale);
	for (Platform& platform : platforms) {
		for (Platform::Pose& pose : platform.poses) {
			pose.R = pose.R * rotation.t();
			pose.C = rotationScale * pose.C + translation;
		}
	}
	for (Image& image : images) {
		if (image.IsValid())
			image.UpdateCamera(platforms);
	}
	FOREACH(i, pointcloud.points) {
		pointcloud.points[i] = rotationScale * Cast<REAL>(pointcloud.points[i]) + translation;
		if (!pointcloud.normals.empty())
			pointcloud.normals[i] = rotation * Cast<REAL>(pointcloud.normals[i]);
	}
	FOREACH(i, mesh.vertices) {
		mesh.vertices[i] = rotationScale * Cast<REAL>(mesh.vertices[i]) + translation;
		if (!mesh.vertexNormals.empty())
			mesh.vertexNormals[i] = rotation * Cast<REAL>(mesh.vertexNormals[i]);
	}
	FOREACH(i, mesh.faceNormals) {
		mesh.faceNormals[i] = rotation * Cast<REAL>(mesh.faceNormals[i]);
	}
	if (obb.IsValid()) {
		obb.Transform(Cast<float>(rotationScale));
		obb.Translate(Cast<float>(translation));
	}
}
void Scene::Transform(const Matrix3x4& transform)
{
	#if 1
	Matrix3x3 mscale, rotation;
	RQDecomp3x3<REAL>(cv::Mat(3,4,cv::DataType<REAL>::type,const_cast<REAL*>(transform.val))(cv::Rect(0,0, 3,3)), mscale, rotation);
	const Point3 translation = transform.col(3);
	#else
	Eigen::Matrix<REAL,4,4> transform4x4 = Eigen::Matrix<REAL,4,4>::Identity();
	transform4x4.topLeftCorner<3,4>() = static_cast<const Matrix3x4::CEMatMap>(transform);
	Eigen::Transform<REAL, 3, Eigen::Isometry> transformIsometry(transform4x4);
	Eigen::Matrix<REAL,3,3> mrotation;
	Eigen::Matrix<REAL,3,3> mscale;
	transformIsometry.computeRotationScaling(&mrotation, &mscale);
	const Point3 translation = transformIsometry.translation();
	const Matrix3x3 rotation = mrotation;
	#endif
	ASSERT(mscale(0,0) > 0 && ISEQUAL(mscale(0,0), mscale(1,1)) && ISEQUAL(mscale(0,0), mscale(2,2)));
	Transform(rotation, translation, mscale(0,0));
} // Transform

// transform this scene such that it best aligns with the given scene based on the camera positions
bool Scene::AlignTo(const Scene& scene)
{
	if (images.size() < 3) {
		DEBUG("error: insufficient number of cameras to perform a similarity transform alignment");
		return false;
	}
	if (images.size() != scene.images.size()) {
		DEBUG("error: the two scenes differ in number of cameras");
		return false;
	}
	CLISTDEF0(Point3) points, pointsRef;
	FOREACH(idx, images) {
		const Image& image = images[idx];
		if (!image.IsValid())
			continue;
		const Image& imageRef = scene.images[idx];
		if (!imageRef.IsValid())
			continue;
		points.emplace_back(image.camera.C);
		pointsRef.emplace_back(imageRef.camera.C);
	}
	Matrix4x4 transform;
	SimilarityTransform(points, pointsRef, transform);
	Matrix3x3 rotation; Point3 translation; REAL scale;
	DecomposeSimilarityTransform(transform, rotation, translation, scale);
	Transform(rotation, translation, scale);
	return true;
} // AlignTo

// estimate ground plane, transform scene such that it is positioned at origin, and compute the volume of the mesh;
//  - planeThreshold: threshold used to estimate the ground plane (0 - auto)
//  - sampleMesh: uniformly samples points on the mesh (0 - disabled, <0 - number of points, >0 - sample density per square unit)
// returns <0 if an error occurred
REAL Scene::ComputeLeveledVolume(float planeThreshold, float sampleMesh, unsigned upAxis, bool verbose)
{
	ASSERT(!mesh.IsEmpty());
	if (planeThreshold >= 0 && !mesh.IsWatertight()) {
		// assume the mesh is opened only at the contact with the ground plane;
		// move mesh such that the ground plane is at the origin so that the volume can be computed
		TD_TIMER_START();
		Planef groundPlane(mesh.EstimateGroundPlane(images, sampleMesh, planeThreshold, verbose?MAKE_PATH("ground_plane.ply"):String()));
		if (!groundPlane.IsValid()) {
			VERBOSE("error: can not estimate the ground plane");
			return -1;
		}
		const Point3f up(upAxis==0?1.f:0.f, upAxis==1?1.f:0.f, upAxis==2?1.f:0.f);
		if (groundPlane.m_vN.dot(Point3f::EVec(up)) < 0.f)
			groundPlane.Negate();
		VERBOSE("Ground plane estimated at: (%.2f,%.2f,%.2f) %.2f (%s)",
			groundPlane.m_vN.x(), groundPlane.m_vN.y(), groundPlane.m_vN.z(), groundPlane.m_fD, TD_TIMER_GET_FMT().c_str());
		// transform the scene such that the up vector aligns with ground plane normal,
		// and the mesh center projected on the ground plane is at the origin
		const Matrix3x3 rotation(RMatrix(Cast<REAL>(up), Cast<REAL>(Point3f(groundPlane.m_vN))).t());
		const Point3 translation(rotation*-Cast<REAL>(Point3f(groundPlane.ProjectPoint(mesh.GetCenter()))));
		const REAL scale(1);
		Transform(rotation, translation, scale);
	}
	return mesh.ComputeVolume();
}
/*----------------------------------------------------------------*/


// estimate region-of-interest based on camera positions, directions and sparse points
// scale specifies the ratio of the ROI's diameter
bool Scene::EstimateROI(int nEstimateROI, float scale)
{
	ASSERT(nEstimateROI >= 0 && nEstimateROI <= 2 && scale > 0);
	if (nEstimateROI == 0) {
		DEBUG_ULTIMATE("The scene will be considered as unbounded (no ROI)");
		return false;
	}
	if (!pointcloud.IsValid()) {
		VERBOSE("error: no valid point-cloud for the ROI estimation");
		return false;
	}
	CameraArr cameras;
	FOREACH(i, images) {
		const Image& imageData = images[i];
		if (!imageData.IsValid())
			continue;
		cameras.emplace_back(imageData.camera);
	}
	const unsigned nCameras = cameras.size();
	if (nCameras < 3) {
		VERBOSE("warning: not enough valid views for the ROI estimation");
		return false;
	}
	// compute the camera center and the direction median
	FloatArr x(nCameras), y(nCameras), z(nCameras), nx(nCameras), ny(nCameras), nz(nCameras);
	FOREACH(i, cameras) {
		const Point3f camC(cameras[i].C);
		x[i] = camC.x;
		y[i] = camC.y;
		z[i] = camC.z;
		const Point3f camDirect(cameras[i].Direction());
		nx[i] = camDirect.x;
		ny[i] = camDirect.y;
		nz[i] = camDirect.z;
	}
	const CMatrix camCenter(x.GetMedian(), y.GetMedian(), z.GetMedian());
	CMatrix camDirectMean(nx.GetMean(), ny.GetMean(), nz.GetMean());
	const float camDirectMeanLen = (float)norm(camDirectMean);
	if (!ISZERO(camDirectMeanLen))
		camDirectMean /= camDirectMeanLen;
	if (camDirectMeanLen > FSQRT_2 / 2.f && nEstimateROI == 2) {
		VERBOSE("The camera directions mean is unbalanced; the scene will be considered unbounded (no ROI)");
		return false;
	}
	DEBUG_ULTIMATE("The camera positions median is (%f,%f,%f), directions mean and norm are (%f,%f,%f), %f",
				   camCenter.x, camCenter.y, camCenter.z, camDirectMean.x, camDirectMean.y, camDirectMean.z, camDirectMeanLen);
	FloatArr cameraDistances(nCameras);
	FOREACH(i, cameras)
		cameraDistances[i] = (float)cameras[i].Distance(camCenter);
	// estimate scene center and radius
	const float camDistMed = cameraDistances.GetMedian();
	const float camShiftCoeff = TAN(ASIN(CLAMP(camDirectMeanLen, 0.f, 0.999f)));
	const CMatrix sceneCenter = camCenter + camShiftCoeff * camDistMed * camDirectMean;
	FOREACH(i, cameras) {
		if (cameras[i].PointDepth(sceneCenter) <= 0 && nEstimateROI == 2) {
			VERBOSE("Found a camera not pointing towards the scene center; the scene will be considered unbounded (no ROI)");
			return false;
		}
		cameraDistances[i] = (float)cameras[i].Distance(sceneCenter);
	}
	const float sceneRadius = cameraDistances.GetMax();
	DEBUG_ULTIMATE("The estimated scene center is (%f,%f,%f), radius is %f",
				   sceneCenter.x, sceneCenter.y, sceneCenter.z, sceneRadius);
	Point3fArr ptsInROI;
	FOREACH(i, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[i];
		const PointCloud::ViewArr& views = pointcloud.pointViews[i];
		FOREACH(j, views) {
			const Image& imageData = images[views[j]];
			if (!imageData.IsValid())
				continue;
			const Camera& camera = imageData.camera;
			if (camera.PointDepth(point) < sceneRadius * 2.0f) {
				ptsInROI.emplace_back(point);
				break;
			}
		}
	}
	obb.Set(AABB3f(ptsInROI.begin(), ptsInROI.size()).EnlargePercent(scale));
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		VERBOSE("Set the ROI with the AABB of position (%f,%f,%f) and extent (%f,%f,%f)",
			    obb.m_pos[0], obb.m_pos[1], obb.m_pos[2], obb.m_ext[0], obb.m_ext[1], obb.m_ext[2]);
	} else {
		VERBOSE("Set the ROI by the estimated core points");
	}
	#endif
	return true;
} // EstimateROI
/*----------------------------------------------------------------*/


// calculate the center(X,Y) of the cylinder, the radius and min/max Z
// from camera position and sparse point cloud, if that exists
// returns result of checks if the scene camera positions satisfies tower criteria:
//	- cameras fit a long and slim bounding box
//  - majority of cameras focus toward a middle line
bool Scene::ComputeTowerCylinder(Point2f& centerPoint, float& fRadius, float& fROIRadius, float& zMin, float& zMax, float& minCamZ, const int towerMode)
{
	// disregard tower mode for scenes with less than 20 cameras
	if (towerMode > 0 && images.size() < 20) {
		DEBUG_ULTIMATE("error: too few images to be a tower: '%d'", images.size());
		return false;
	}

	AABB3f aabbOutsideCameras(true);
	CLISTDEF0(Point2f) cameras2D(images.size());
	FloatArr camHeigths;
	FitLineOnline<float> fitline;
	FOREACH(imgIdx, images) {
		const Eigen::Vector3f camPos(Cast<float>(images[imgIdx].camera.C));
		fitline.Update(camPos);
		aabbOutsideCameras.InsertFull(camPos);
		cameras2D[imgIdx] = Point2f(camPos.x(), camPos.y());
		camHeigths.InsertSortUnique(camPos.z());
	}
	Line3f camCenterLine;
	Point3f quality = fitline.GetLine(camCenterLine);
	// check if ROI is mostly long and narrow on one direction
	if (quality.y / quality.z > 0.6f || quality.x / quality.y < 0.8f) {
		// does not seem to be a line
		if (towerMode > 0) {
			DEBUG_ULTIMATE("error: does not seem to be a tower: X(%.2f), Y(%.2f), Z(%.2f)", quality.x, quality.y, quality.z);
			return false;
		}
	}

	// get the height of the lowest camera
	minCamZ = aabbOutsideCameras.ptMin.z();
	centerPoint = ((camCenterLine.pt1+camCenterLine.pt2)*0.5f).topLeftCorner<2,1>();
	zMin = MINF(aabbOutsideCameras.ptMax.z(), aabbOutsideCameras.ptMin.z()) - 5;
	// if sparse point cloud is loaded use lowest point as zMin
	float fMinPointsZ = std::numeric_limits<float>::max();
	float fMaxPointsZ = std::numeric_limits<float>::lowest();
	FOREACH(pIdx, pointcloud.points) {
		if (!obb.IsValid() || obb.Intersects(pointcloud.points[pIdx])) {
			const float pz = pointcloud.points[pIdx].z;
			if (pz < fMinPointsZ)
				fMinPointsZ = pz;
			if (pz > fMaxPointsZ)
				fMaxPointsZ = pz;			
		}
	}
	zMin = MINF(zMin, fMinPointsZ);
	zMax = MAXF(aabbOutsideCameras.ptMax.z(), fMaxPointsZ);
	
	// calculate tower radius as median distance from tower center to cameras
	FloatArr cameraDistancesToMiddle(cameras2D.size());
	FOREACH (camIdx, cameras2D)
		cameraDistancesToMiddle[camIdx] = (float)norm(cameras2D[camIdx] - centerPoint);
	const float fMedianDistance = cameraDistancesToMiddle.GetMedian();
	fRadius = MAXF(0.2f, (fMedianDistance - 1.f) / 3.f);
	// get the average of top 85 to 95% of the highest distances to center
	if (!cameraDistancesToMiddle.empty()) {
		float avgTopDistance(0);
		cameraDistancesToMiddle.Sort();
		const size_t topIdx(CEIL2INT(cameraDistancesToMiddle.size() * 0.95f));
		const size_t botIdx(FLOOR2INT(cameraDistancesToMiddle.size() * 0.85f));
		for (size_t i = botIdx; i < topIdx; ++i) {
			avgTopDistance += cameraDistancesToMiddle[i];
		}
		avgTopDistance /= topIdx - botIdx;
		fROIRadius = avgTopDistance;
	} else {
		fROIRadius = fRadius;
	}
	return true;
} // ComputeTowerCylinder

size_t Scene::DrawCircle(PointCloud& pc, PointCloud::PointArr& outCircle, const Point3f& circleCenter, const float circleRadius, const unsigned nTargetPoints, const float fStartAngle, const float fAngleBetweenPoints)
{
	outCircle.Release();
	for (unsigned pIdx = 0; pIdx < nTargetPoints; ++pIdx) {
		const float fAngle(fStartAngle + fAngleBetweenPoints * pIdx);
		ASSERT(fAngle <= FTWO_PI);
		const Normal n(cos(fAngle), sin(fAngle), 0);
		ASSERT(ISEQUAL(norm(n), 1.f));
		const Point3f newPoint(circleCenter + circleRadius * n);
		// select cameras seeing this point
		PointCloud::ViewArr views;
		FOREACH(idxImg, images) {
			const Image& image = images[idxImg];
			const Point3f xz(image.camera.TransformPointW2I3(Cast<REAL>(newPoint)));
			const Point2f x(xz.x, xz.y);
			if (!Image8U::isInside<float>(x, image.GetSize()) ||
				xz.z <= 0)
				continue;
			if (n.dot(Cast<float>(image.camera.RayPoint<REAL>(x))) >= 0)
				continue;
			views.emplace_back(idxImg);
		}
		if (views.size() >= 2) {
			outCircle.emplace_back(newPoint);
			pc.points.emplace_back(newPoint);
			pc.pointViews.emplace_back(views);
			pc.normals.emplace_back(n);
			pc.colors.emplace_back(Pixel8U::YELLOW);
		}
	}
	return outCircle.size();
} // DrawCircle

PointCloud Scene::BuildTowerMesh(const PointCloud& origPointCloud, const Point2f& centerPoint, const float fRadius, const float fROIRadius, const float zMin, const float zMax, const float minCamZ, bool bFixRadius)
{
	const unsigned nTargetDensity(10);
	const unsigned nTargetCircles(ROUND2INT((zMax - zMin) * nTargetDensity)); // how many circles in cylinder
	const float fCircleFrequence((zMax - zMin) / nTargetCircles); // the distance between neighbor circles
	PointCloud towerPC;
	PointCloud::PointArr circlePoints;
	Mesh::VertexVerticesArr meshCircles;
	if (bFixRadius) {
		const unsigned nTargetPoints(MAX(10, ROUND2INT(FTWO_PI * fRadius * nTargetDensity))); // how many points on each circle
		const float fAngleBetweenPoints(FTWO_PI / nTargetPoints); // the angle between neighbor points on the circle	
		for (unsigned cIdx = 0; cIdx < nTargetCircles; ++cIdx) {
			const Point3f circleCenter(centerPoint, zMin + fCircleFrequence * cIdx); // center point of the circle
			const float fStartAngle(fAngleBetweenPoints * SEACAVE::random()); // starting angle for the first point
			DrawCircle(towerPC, circlePoints, circleCenter, fRadius, nTargetPoints, fStartAngle, fAngleBetweenPoints);
			if (!circlePoints.empty()) {
				// add points to vertex  list
				Mesh::VertexIdxArr circleVertices;
				Mesh::VIndex vIdx = mesh.vertices.size();
				for (const Point3f& p: circlePoints) {
					mesh.vertices.emplace_back(p);
					circleVertices.emplace_back(vIdx++);
				}
				meshCircles.emplace_back(circleVertices);
			}
		}
	} else {
		cList<FloatArr> sliceDistances(nTargetCircles);
		for (const Point3f& P : origPointCloud.points) {
			const float d((float)norm(Point2f(P.x, P.y) - centerPoint));
			if (d <= fROIRadius) {
				const float fIdx((zMax - P.z) * nTargetDensity);
				int bIdx(FLOOR2INT(fIdx));
				int tIdx(FLOOR2INT(fIdx+0.5f));
				if (bIdx == tIdx && bIdx > 0)
					bIdx--;
				if (tIdx >= (int)nTargetCircles)
					tIdx = nTargetCircles - 1;
				if (bIdx < (int)nTargetCircles - 1)
					sliceDistances[bIdx].emplace_back(d);
				if (tIdx > 0)
					sliceDistances[tIdx].emplace_back(d);
			}
		}
		FloatArr circleRadii;
		for (unsigned cIdx = 0; cIdx < nTargetCircles; ++cIdx) {
			const float circleZ(zMax - fCircleFrequence * cIdx);
			FloatArr& pDistances = sliceDistances[cIdx];
			float circleRadius(fRadius);
			if (circleZ < minCamZ) {
				// use fixed radius under lowest camera position
				circleRadius = fRadius;
			} else {
				if (pDistances.size() > 2) {
					pDistances.Sort();
					const size_t topIdx(MIN(pDistances.size() - 1, CEIL2INT<size_t>(pDistances.size() * 0.95f)));
					const size_t botIdx(MAX(1u, FLOOR2INT<unsigned>(pDistances.size() * 0.5f)));
					float avgTopDistance(0);
					for (size_t i = botIdx; i < topIdx; ++i)
						avgTopDistance += pDistances[i];
					avgTopDistance /= topIdx - botIdx;
					if (avgTopDistance < fROIRadius * 0.8f)
						circleRadius = avgTopDistance;
				}
			}
			circleRadii.emplace_back(circleRadius);
		}
		// smoothen radii
		if (circleRadii.size() > 2) {
			for (size_t ri = 1; ri < circleRadii.size() - 1; ++ri) {
				const float aboveRad(circleRadii[ri - 1]);
				float& circleRadius = circleRadii[ri];
				const float belowRad(circleRadii[ri + 1]);
				// set current radius as average of the most similar values in the closest 7 neighbors
				if (ri > 2 && ri < circleRadii.size() - 5) {
					FloatArr neighSeven(7);
					FOREACH(i, neighSeven)
						neighSeven[i] = circleRadii[ri - 2 + i];
					const float medianRadius(neighSeven.GetMedian());
					circleRadius = ABS(medianRadius-aboveRad) < ABS(medianRadius-belowRad) ? aboveRad : belowRad;
				} else {
					circleRadius = (aboveRad + belowRad) / 2.f;
				}
			}
		}
		// add circles
		FOREACH(rIdx, circleRadii) {
			float circleRadius(circleRadii[rIdx]);
			const float circleZ(zMax - fCircleFrequence * rIdx);
			const Point3f circleCenter(centerPoint, circleZ); // center point of the circle
			const unsigned nTargetPoints(MAX(10, ROUND2INT(FTWO_PI * circleRadius * nTargetDensity))); // how many points on each circle
			const float fAngleBetweenPoints(FTWO_PI / nTargetPoints); // the angle between neighbor points on the circle
			const float fStartAngle(fAngleBetweenPoints * SEACAVE::random()); // starting angle for the first point
			DrawCircle(towerPC, circlePoints, circleCenter, circleRadius, nTargetPoints, fStartAngle, fAngleBetweenPoints);
			if (!circlePoints.IsEmpty()) {
				//add points to vertex  list
				Mesh::VertexIdxArr circleVertices;
				Mesh::VIndex vIdx = mesh.vertices.size();
				FOREACH(pIdx, circlePoints) {
					const Point3f& p = circlePoints[pIdx];
					mesh.vertices.emplace_back(p);
					circleVertices.emplace_back(vIdx);
					++vIdx;
				}
				meshCircles.emplace_back(circleVertices);
			}
		}
	}
	
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		// Build faces from meshCircles
		for (Mesh::VIndex cIdx = 1; cIdx < meshCircles.size(); ++cIdx) {
			if (meshCircles[cIdx - 1].size() > 1 || meshCircles[cIdx].size() > 1) {
				Mesh::VertexIdxArr& topPoints = meshCircles[cIdx - 1];
				Mesh::VertexIdxArr& botPoints = meshCircles[cIdx];
				// build faces with all the points in the two lists
				bool bInverted(false);
				if (topPoints.size() > botPoints.size()) {
					topPoints.swap(botPoints);
					bInverted = true;
				}
				const float topStep(1.0f / topPoints.size());
				const float botStep(1.0f / botPoints.size());
				for (Mesh::VIndex ti=0, bi=0; ti < topPoints.size() && bi<botPoints.size(); ++ti) {
					do {
						const Mesh::VIndex& v0(topPoints[ti]);
						const Mesh::VIndex& v1(botPoints[bi]);
						const Mesh::VIndex& v2(botPoints[(++bi)%botPoints.size()]);
						if (!bInverted)
							mesh.faces.emplace_back(v0, v1, v2);
						else
							mesh.faces.emplace_back(v0, v2, v1);
					} while (bi<botPoints.size() && (ti+1)*topStep > (bi+1)*botStep);
					if (topPoints.size() > 1) {
						const Mesh::VIndex& v0(topPoints[ti]);
						const Mesh::VIndex& v1(botPoints[bi%botPoints.size()]);
						const Mesh::VIndex& v2(topPoints[(ti+1)%topPoints.size()]);
						if (!bInverted)
							mesh.faces.emplace_back(v0, v1, v2);
						else
							mesh.faces.emplace_back(v0, v2, v1);
					}
					if (topPoints.size() != botPoints.size()) {
						// add closing face
						const Mesh::VIndex& v0(topPoints[0]);
						const Mesh::VIndex& v1(botPoints[botPoints.size()-1]);
						const Mesh::VIndex& v2(botPoints[0]);
						if (!bInverted)
							mesh.faces.emplace_back(v0, v1, v2);
						else
							mesh.faces.emplace_back(v0, v2, v1);
					}
				}
				if (bInverted)
					topPoints.swap(botPoints);
			}
		}
		mesh.Save("tower_mesh.ply");
	} else
	#endif
	{
		mesh.Release();
	}
	towerPC.Save("tower.ply");
	return towerPC;
}


// compute points on a cylinder placed in the middle of scene's cameras
// this function assumes the scene is Z-up and units are meters
//  - towerMode:  0 - disabled, 1 - replace, 2 - append, 3 - select neighbors, 4 - select neighbors and append, <0 - force tower mode
void Scene::InitTowerScene(const int towerMode)
{
	float fRadius;
	float fROIRadius;
	float zMax, zMin, minCamZ;
	Point2f centerPoint;
	if (!ComputeTowerCylinder(centerPoint, fRadius, fROIRadius, zMin, zMax, minCamZ, towerMode))
		return;

	// add nTargetPoints points on each circle
	PointCloud towerPC(BuildTowerMesh(pointcloud, centerPoint, fRadius, fROIRadius, zMin, zMax, minCamZ, false));
	mesh.Release();

	const auto AppendPointCloud = [this](const PointCloud& towerPC) {
		bool bHasNormal(pointcloud.normals.size() == pointcloud.GetSize());
		bool bHasColor(pointcloud.colors.size() == pointcloud.GetSize());
		bool bHasWeights(pointcloud.pointWeights.size() == pointcloud.GetSize());
		FOREACH(idxPoint, towerPC.points) {
			pointcloud.points.emplace_back(towerPC.points[idxPoint]);
			pointcloud.pointViews.emplace_back(towerPC.pointViews[idxPoint]);
			if (bHasNormal)
				pointcloud.normals.emplace_back(towerPC.normals[idxPoint]);
			if (bHasColor)
				pointcloud.colors.emplace_back(towerPC.colors[idxPoint]);
			if (bHasWeights)
				pointcloud.pointWeights.emplace_back(towerPC.pointWeights[idxPoint]);
		}
	};

	switch (ABS(towerMode)) {
	case 1: // replace
		pointcloud = std::move(towerPC);
		VERBOSE("Scene identified as tower-like; replace existing point-cloud with detected tower point-cloud");
		break;
	case 2: // append
		AppendPointCloud(towerPC);
		VERBOSE("Scene identified as tower-like; append to existing point-cloud the detected tower point-cloud");
		break;
	case 3: // select neighbors
		pointcloud.Swap(towerPC);
		SelectNeighborViews(OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, FD2R(OPTDENSE::fOptimAngle), OPTDENSE::nPointInsideROI);
		pointcloud.Swap(towerPC);
		VERBOSE("Scene identified as tower-like; only select view neighbors from detected tower point-cloud");
		break;
	case 4: // select neighbors and append tower points
		pointcloud.Swap(towerPC);
		SelectNeighborViews(OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, FD2R(OPTDENSE::fOptimAngle), OPTDENSE::nPointInsideROI);
		pointcloud.Swap(towerPC);
		AppendPointCloud(towerPC);
		VERBOSE("Scene identified as tower-like; select view neighbors from detected tower point-cloud and next append it to existing point-cloud");
		break;
	}
} // InitTowerScene
