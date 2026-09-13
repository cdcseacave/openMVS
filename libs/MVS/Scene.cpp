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


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define PROJECT_ID "MVS\0" // identifies the project stream
#define PROJECT_VER ((uint32_t)2) // identifies the version of a project stream

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));

void Scene::Release()
{
	platforms.Release();
	images.Release();
	pointcloud.Release();
	mesh.Release();
	obb.Reset();
	transform = Matrix4x4f::IDENTITY;
	// the images it counts are gone, so it goes back to what the constructor leaves (see Scene.h)
	nCalibratedImages = 0;
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


bool Scene::LoadInterface(const String& fileName)
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

	// import transform
	transform = obj.transform;

	DEBUG_EXTRA("Scene loaded in interface format from '%s' (%s):\n"
				"\t%u images (%u calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
				"\t%u points, %u vertices, %u faces",
				Util::getFileNameExt(fileName).c_str(), TD_TIMER_GET_FMT().c_str(),
				images.size(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.size(), mesh.vertices.size(), mesh.faces.size());
	return true;
} // LoadInterface

bool Scene::SaveInterface(const String& fileName, int version) const
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
			Interface::Platform& platform = obj.platforms[image.platformID];
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

	// export transform
	obj.transform = transform;

	// serialize out the current state
	if (!ARCHIVE::SerializeSave(obj, fileName, version>=0?uint32_t(version):MVSI_PROJECT_VER))
		return false;

	DEBUG_EXTRA("Scene saved in interface format to '%s' (%s):\n"
				"\t%u images (%u calibrated)\n"
				"\t%u points, %u vertices, %u faces",
				Util::getFileNameExt(fileName).c_str(), TD_TIMER_GET_FMT().c_str(),
				images.size(), nCalibratedImages,
				pointcloud.points.size(), mesh.vertices.size(), mesh.faces.size());
	return true;
} // SaveInterface
/*----------------------------------------------------------------*/


// load region-of-interest from a text file
bool Scene::LoadROI(const String& fileName)
{
	TD_TIMER_STARTD();

	std::ifstream fs(fileName);
	if (!fs)
		return false;
	// try to read OBB
	fs >> obb;
	if (fs.fail()) {
		// reset fs to the beginning position
		fs.clear();
		fs.seekg(0, std::ios::beg);
		// try to read AABB
		AABB3f box;
		fs >> box;
		if (fs.fail())
			return false;
		obb = OBB3f(box);
	}

	DEBUG_EXTRA("Region-of-interest loaded from file '%s' (%s)",
				fileName.c_str(), TD_TIMER_GET_FMT().c_str());
	return true;
} // LoadROI
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
	if (image.ReloadImageAtPreparedResolution())
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

	DEBUG_EXTRA("Scene loaded from depth-map format - %dx%d size, %.2f%% coverage (%s):\n"
		"\t1 images (%u neighbors, %.2f FOV) with a total of %.2f MPixels (%.2f MPixels/image)\n"
		"\t%u points, 0 lines",
		depthMap.width(), depthMap.height(), 100.0*pointcloud.GetSize()/depthMap.area(), TD_TIMER_GET_FMT().c_str(),
		IDs.size()-1, R2D(image.ComputeFOV()), (double)image.image.area()/(1024.0*1024.0), (double)image.image.area()/(1024.0*1024.0*nCalibratedImages),
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
			imageData.neighbors[i] = ViewScore{nID, 0, 1.f, D2R(15.f), 0.5f, 2.f+(argc-i)*0.5f};
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
	if (ext == _T(".obj")) {
		// import mesh from obj file
		return mesh.Load(fileName);
	}
	if (ext == _T(".ply") || ext == _T(".gltf") || ext == _T(".glb")) {
		// import point-cloud/mesh from ply/gltf file
		if (mesh.Load(fileName))
			return true;
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
	if (!fs.is_open()) {
		VERBOSE("error: unable to open file '%s'", fileName.c_str());
		return SCENE_NA;
	}
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
	if (!SerializeLoad(*this, fs, (ARCHIVE_TYPE)nType)) {
		VERBOSE("error: unable to load project data");
		return SCENE_NA;
	}
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
	// each line stores two vertices and one edge
	const size_t memBufferSize(PLY::ComputeMemBufferSize(lines.size(), 2*sizeof(PLYVertex) + sizeof(PLYEdge)));
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

#pragma pop_macro("VERBOSE")
