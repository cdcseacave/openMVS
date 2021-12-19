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
#define _USE_OPENCV
#include "Interface.h"

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
	platforms.Reserve((uint32_t)obj.platforms.size());
	for (const Interface::Platform& itPlatform: obj.platforms) {
		Platform& platform = platforms.AddEmpty();
		platform.name = itPlatform.name;
		platform.cameras.Reserve((uint32_t)itPlatform.cameras.size());
		for (const Interface::Platform::Camera& itCamera: itPlatform.cameras) {
			Platform::Camera& camera = platform.cameras.AddEmpty();
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
		ASSERT(platform.cameras.GetSize() == itPlatform.cameras.size());
		platform.poses.Reserve((uint32_t)itPlatform.poses.size());
		for (const Interface::Platform::Pose& itPose: itPlatform.poses) {
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = itPose.R;
			pose.C = itPose.C;
		}
		ASSERT(platform.poses.GetSize() == itPlatform.poses.size());
	}
	ASSERT(platforms.GetSize() == obj.platforms.size());
	if (platforms.IsEmpty())
		return false;

	// import images
	nCalibratedImages = 0;
	size_t nTotalPixels(0);
	ASSERT(!obj.images.empty());
	images.Reserve((uint32_t)obj.images.size());
	for (const Interface::Image& image: obj.images) {
		const uint32_t ID(images.size());
		Image& imageData = images.AddEmpty();
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
		imageData.ID = (image.ID == NO_ID ? ID : image.ID);
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
		++nCalibratedImages;
		nTotalPixels += imageData.width * imageData.height;
		DEBUG_ULTIMATE("Image loaded %3u: %s", ID, Util::getFileNameExt(imageData.name).c_str());
	}
	if (images.GetSize() < 2)
		return false;

	// import 3D points
	if (!obj.vertices.empty()) {
		bool bValidWeights(false);
		pointcloud.points.Resize(obj.vertices.size());
		pointcloud.pointViews.Resize(obj.vertices.size());
		pointcloud.pointWeights.Resize(obj.vertices.size());
		FOREACH(i, pointcloud.points) {
			const Interface::Vertex& vertex = obj.vertices[i];
			PointCloud::Point& point = pointcloud.points[i];
			point = vertex.X;
			PointCloud::ViewArr& views = pointcloud.pointViews[i];
			views.Resize((PointCloud::ViewArr::IDX)vertex.views.size());
			PointCloud::WeightArr& weights = pointcloud.pointWeights[i];
			weights.Resize((PointCloud::ViewArr::IDX)vertex.views.size());
			CLISTDEF0(PointCloud::ViewArr::IDX) indices(views.GetSize());
			std::iota(indices.Begin(), indices.End(), 0);
			std::sort(indices.Begin(), indices.End(), [&](IndexArr::Type i0, IndexArr::Type i1) -> bool {
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
				images.GetSize(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
} // LoadInterface

bool Scene::SaveInterface(const String & fileName, int version) const
{
	TD_TIMER_STARTD();
	Interface obj;

	// export platforms
	obj.platforms.reserve(platforms.GetSize());
	for (const Platform& platform: platforms) {
		Interface::Platform plat;
		plat.cameras.reserve(platform.cameras.GetSize());
		for (const Platform::Camera& camera: platform.cameras) {
			Interface::Platform::Camera cam;
			cam.K = camera.K;
			cam.R = camera.R;
			cam.C = camera.C;
			plat.cameras.push_back(cam);
		}
		plat.poses.reserve(platform.poses.GetSize());
		for (const Platform::Pose& pose: platform.poses) {
			Interface::Platform::Pose p;
			p.R = pose.R;
			p.C = pose.C;
			plat.poses.push_back(p);
		}
		obj.platforms.push_back(plat);
	}

	// export images
	obj.images.resize(images.GetSize());
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
	}

	// export 3D points
	obj.vertices.resize(pointcloud.points.GetSize());
	FOREACH(i, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[i];
		const PointCloud::ViewArr& views = pointcloud.pointViews[i];
		MVS::Interface::Vertex& vertex = obj.vertices[i];
		ASSERT(sizeof(vertex.X.x) == sizeof(point.x));
		vertex.X = point;
		vertex.views.resize(views.GetSize());
		views.ForEach([&](PointCloud::ViewArr::IDX v) {
			MVS::Interface::Vertex::View& view = vertex.views[v];
			view.imageID = views[v];
			view.confidence = (pointcloud.pointWeights.IsEmpty() ? 0.f : pointcloud.pointWeights[i][v]);
		});
	}
	if (!pointcloud.normals.IsEmpty()) {
		obj.verticesNormal.resize(pointcloud.normals.GetSize());
		FOREACH(i, pointcloud.normals) {
			const PointCloud::Normal& normal = pointcloud.normals[i];
			MVS::Interface::Normal& vertexNormal = obj.verticesNormal[i];
			vertexNormal.n = normal;
		}
	}
	if (!pointcloud.colors.IsEmpty()) {
		obj.verticesColor.resize(pointcloud.colors.GetSize());
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
				images.GetSize(), nCalibratedImages,
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
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
	if (!ImportDepthDataRaw(fileName, imageFileName, IDs, imageSize, camera.K, camera.R, camera.C, dMin, dMax, depthMap, normalMap, confMap))
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
	const LPSMLARR&	arrSmlChild = smlImages.GetArrChildren();
	ASSERT(arrSmlChild.size() <= 1);

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
			imageData.neighbors[i] = ViewScore{ViewInfo{nID, 0, 1.f, FD2R(15.f), 0.5f}, 3.f};
		}
	}

	DEBUG_EXTRA("View neighbors list loaded (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
} // LoadViewNeighbors
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
		for (int i = 0; i < (int)ply.elems.size(); ++i) {
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

bool Scene::Load(const String& fileName, bool bImport)
{
	TD_TIMER_STARTD();
	Release();

	#ifdef _USE_BOOST
	// open the input stream
	std::ifstream fs(fileName, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		return false;
	// load project header ID
	char szHeader[4];
	fs.read(szHeader, 4);
	if (!fs || _tcsncmp(szHeader, PROJECT_ID, 4) != 0) {
		fs.close();
		if (bImport && Import(fileName))
			return true;
		if (LoadInterface(fileName))
			return true;
		VERBOSE("error: invalid project");
		return false;
	}
	// load project version
	uint32_t nVer;
	fs.read((char*)&nVer, sizeof(uint32_t));
	if (!fs || nVer != PROJECT_VER) {
		VERBOSE("error: different project version");
		return false;
	}
	// load stream type
	uint32_t nType;
	fs.read((char*)&nType, sizeof(uint32_t));
	// skip reserved bytes
	uint64_t nReserved;
	fs.read((char*)&nReserved, sizeof(uint64_t));
	// serialize in the current state
	if (!SerializeLoad(*this, fs, (ARCHIVE_TYPE)nType))
		return false;
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
	return true;
	#else
	return false;
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
			const Point2f& x(reinterpret_cast<const Point2f&>(xz));
			if (depthMap.isInsideWithBorder<float,1>(x) && xz.z * thFrontDepth < depthMap(ROUND2INT(x))) {
				#ifdef SCENE_USE_OPENMP
				#pragma omp critical
				#endif
				pointcloud.pointViews[idxVertex].emplace_back(ID);
			}
		}
	}
	RFOREACH(idx, pointcloud.points) {
		if (pointcloud.pointViews[idx].size() < 2)
			pointcloud.RemovePoint(idx);
		else
			pointcloud.points[idx] = mesh.vertices[(Mesh::VIndex)idx];
	}
} // SampleMeshWithVisibility
/*----------------------------------------------------------------*/


inline float Footprint(const Camera& camera, const Point3f& X) {
	#if 0
	const REAL fSphereRadius(1);
	const Point3 cX(camera.TransformPointW2C(Cast<REAL>(X)));
	return (float)norm(camera.TransformPointC2I(Point3(cX.x+fSphereRadius,cX.y,cX.z))-camera.TransformPointC2I(cX))+std::numeric_limits<float>::epsilon();
	#else
	return (float)(camera.GetFocalLength()/camera.PointDepth(X));
	#endif
}

// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
bool Scene::SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle)
{
	ASSERT(points.IsEmpty());

	// extract the estimated 3D points and the corresponding 2D projections for the reference image
	Image& imageData = images[ID];
	ASSERT(imageData.IsValid());
	ViewScoreArr& neighbors = imageData.neighbors;
	ASSERT(neighbors.IsEmpty());
	struct Score {
		float score;
		float avgScale;
		float avgAngle;
		uint32_t points;
	};
	CLISTDEF0(Score) scores(images.GetSize());
	scores.Memset(0);
	if (nMinPointViews > nCalibratedImages)
		nMinPointViews = nCalibratedImages;
	unsigned nPoints = 0;
	imageData.avgDepth = 0;
	const float sigmaAngle(-1.f/(2.f*SQUARE(fOptimAngle*1.3f)));
	FOREACH(idx, pointcloud.points) {
		const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
		ASSERT(views.IsSorted());
		if (views.FindFirst(ID) == PointCloud::ViewArr::NO_INDEX)
			continue;
		// store this point
		const PointCloud::Point& point = pointcloud.points[idx];
		if (views.GetSize() >= nMinPointViews)
			points.Insert((uint32_t)idx);
		imageData.avgDepth += (float)imageData.camera.PointDepth(point);
		++nPoints;
		// score shared views
		const Point3f V1(imageData.camera.C - Cast<REAL>(point));
		const float footprint1(Footprint(imageData.camera, point));
		for (const PointCloud::View& view: views) {
			if (view == ID)
				continue;
			const Image& imageData2 = images[view];
			const Point3f V2(imageData2.camera.C - Cast<REAL>(point));
			const float fAngle(ACOS(ComputeAngle<float,float>(V1.ptr(), V2.ptr())));
			const float wAngle(fAngle<fOptimAngle ? POW(fAngle/fOptimAngle, 1.5f) : EXP(SQUARE(fAngle-fOptimAngle)*sigmaAngle));
			const float footprint2(Footprint(imageData2.camera, point));
			const float fScaleRatio(footprint1/footprint2);
			float wScale;
			if (fScaleRatio > 1.6f)
				wScale = SQUARE(1.6f/fScaleRatio);
			else if (fScaleRatio >= 1.f)
				wScale = 1.f;
			else
				wScale = SQUARE(fScaleRatio);
			Score& score = scores[view];
			score.score += wAngle * wScale;
			score.avgScale += fScaleRatio;
			score.avgAngle += fAngle;
			++score.points;
		}
	}
	imageData.avgDepth /= nPoints;
	ASSERT(nPoints > 3);

	// select best neighborViews
	Point2fArr projs(0, points.GetSize());
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
		ASSERT(projs.IsEmpty());
		for (uint32_t idx: points) {
			const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
			ASSERT(views.IsSorted());
			ASSERT(views.FindFirst(ID) != PointCloud::ViewArr::NO_INDEX);
			if (views.FindFirst(IDB) == PointCloud::ViewArr::NO_INDEX)
				continue;
			const PointCloud::Point& point = pointcloud.points[idx];
			Point2f& ptA = projs.AddConstruct(imageData.camera.ProjectPointP(point));
			Point2f ptB = imageDataB.camera.ProjectPointP(point);
			if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB))
				projs.RemoveLast();
		}
		ASSERT(projs.GetSize() <= score.points);
		if (projs.IsEmpty())
			continue;
		const float area(ComputeCoveredArea<float,2,16,false>((const float*)projs.Begin(), projs.GetSize(), boundsA.ptr()));
		projs.Empty();
		// store image score
		ViewScore& neighbor = neighbors.AddEmpty();
		neighbor.idx.ID = IDB;
		neighbor.idx.points = score.points;
		neighbor.idx.scale = score.avgScale/score.points;
		neighbor.idx.angle = score.avgAngle/score.points;
		neighbor.idx.area = area;
		neighbor.score = score.score*area;
	}
	neighbors.Sort();
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print neighbor views
	if (VERBOSITY_LEVEL > 2) {
		String msg;
		FOREACH(n, neighbors)
			msg += String::FormatString(" %3u(%upts,%.2fscl)", neighbors[n].idx.ID, neighbors[n].idx.points, neighbors[n].idx.scale);
		VERBOSE("Reference image %3u sees %u views:%s (%u shared points)", ID, neighbors.GetSize(), msg.c_str(), nPoints);
	}
	#endif
	if (points.GetSize() <= 3 || neighbors.GetSize() < MINF(nMinViews,nCalibratedImages-1)) {
		DEBUG_EXTRA("error: reference image %3u has not enough images in view", ID);
		return false;
	}
	return true;
} // SelectNeighborViews
/*----------------------------------------------------------------*/

// keep only the best neighbors for the reference image
bool Scene::FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea, float fMinScale, float fMaxScale, float fMinAngle, float fMaxAngle, unsigned nMaxViews)
{
	// remove invalid neighbor views
	RFOREACH(n, neighbors) {
		const ViewScore& neighbor = neighbors[n];
		if (neighbor.idx.area < fMinArea ||
			!ISINSIDE(neighbor.idx.scale, fMinScale, fMaxScale) ||
			!ISINSIDE(neighbor.idx.angle, fMinAngle, fMaxAngle))
			neighbors.RemoveAtMove(n);
	}
	if (neighbors.GetSize() > nMaxViews)
		neighbors.Resize(nMaxViews);
	return !neighbors.IsEmpty();
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
			depthData.Load(ComposeDepthFilePath(imageData.ID, "dmap"));
			if (depthData.IsEmpty())
				continue;
			const IIndex numPointsBegin(visibility.size());
			const Camera camera(imageData.GetCamera(platforms, depthData.depthMap.size()));
			for (int r=(depthData.depthMap.rows%depthMapStep)/2; r<depthData.depthMap.rows; r+=depthMapStep) {
				for (int c=(depthData.depthMap.cols%depthMapStep)/2; c<depthData.depthMap.cols; c+=depthMapStep) {
					const Depth depth = depthData.depthMap(r,c);
					if (depth <= 0)
						continue;
					const Point3f& X = samples.emplace_back(Cast<float>(camera.TransformPointI2W(Point3(c,r,depth))));
					areas.emplace_back(Footprint(camera, X)*areaScale);
					visibility.emplace_back(idxImage);
				}
			}
			imageAreas[idxImage] = visibility.size()-numPointsBegin;
		}
		#if 0
		const AABB3f aabb(samples.data(), samples.size());
		#else
		// try to find a dominant plane, and set the bounding-box center on the plane bottom
		OBB3f obb(samples.data(), samples.size());
		obb.m_ext(0) *= 2;
		#if 1
		// dump box for visualization
		OBB3f::POINT pts[8];
		obb.GetCorners(pts);
		PointCloud pc;
		for (int i=0; i<8; ++i)
			pc.points.emplace_back(pts[i]);
		pc.Save(MAKE_PATH("scene_obb.ply"));
		#endif
		const AABB3f aabb(obb.GetAABB());
		#endif
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
				const auto itImage(mapImages.find(neighbor.idx.ID));
				if (itImage == mapImages.end()) {
					image.neighbors.RemoveAtMove(i);
					continue;
				}
				ASSERT(itImage->second < subset.images.size());
				neighbor.idx.ID = itImage->second;
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
