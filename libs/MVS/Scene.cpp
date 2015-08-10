/*
* Scene.cpp
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
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
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
*/

#include "Common.h"
#include "Scene.h"
#include "Interface.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define PROJECT_ID "MVS\0" // identifies the project stream
#define PROJECT_VER ((uint32_t)1) // identifies the version of a project stream


// S T R U C T S ///////////////////////////////////////////////////

bool Scene::LoadInterface(const String & fileName)
{
	Interface obj;

	#ifdef _USE_BOOST
	// serialize in the current state
	if (!SerializeLoad(obj, fileName, ARCHIVE_BINARY_ZIP))
		return false;
	#else
	return false;
	#endif

	// import platforms and cameras
	ASSERT(!obj.platforms.empty());
	platforms.Reserve((uint32_t)obj.platforms.size());
	for (Interface::PlatformArr::const_iterator itPlatform=obj.platforms.begin(); itPlatform!=obj.platforms.end(); ++itPlatform) {
		Platform& platform = platforms.AddEmpty();
		platform.name = itPlatform->name;
		platform.cameras.Reserve((uint32_t)itPlatform->cameras.size());
		for (Interface::Platform::CameraArr::const_iterator itCamera=itPlatform->cameras.begin(); itCamera!=itPlatform->cameras.end(); ++itCamera) {
			Platform::Camera& camera = platform.cameras.AddEmpty();
			camera.K = itCamera->K;
			camera.R = itCamera->R;
			camera.C = itCamera->C;
			DEBUG_EXTRA("Camera model loaded: platform %u; camera %2u; f %.3fx%.3f; poses %u", platforms.GetSize()-1, platform.cameras.GetSize()-1, camera.K(0,0), camera.K(1,1), itPlatform->poses.size());
		}
		ASSERT(platform.cameras.GetSize() == itPlatform->cameras.size());
		platform.poses.Reserve((uint32_t)itPlatform->poses.size());
		for (Interface::Platform::PoseArr::const_iterator itPose=itPlatform->poses.begin(); itPose!=itPlatform->poses.end(); ++itPose) {
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = itPose->R;
			pose.C = itPose->C;
		}
		ASSERT(platform.poses.GetSize() == itPlatform->poses.size());
	}
	ASSERT(platforms.GetSize() == obj.platforms.size());
	if (platforms.IsEmpty())
		return false;

	// import images
	nCalibratedImages = 0;
	ASSERT(!obj.images.empty());
	images.Reserve((uint32_t)obj.images.size());
	for (Interface::ImageArr::const_iterator it=obj.images.begin(); it!=obj.images.end(); ++it) {
		const Interface::Image& image = *it;
		const uint32_t ID(images.GetSize());
		Image& imageData = images.AddEmpty();
		imageData.name = image.name;
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, imageData.name);
		imageData.poseID = image.poseID;
		if (imageData.poseID == NO_ID)
			continue;
		imageData.platformID = image.platformID;
		imageData.cameraID = image.cameraID;
		#if 1
		// load image
		if (!imageData.ReloadImage(0, false))
			return false;
		imageData.UpdateCamera(platforms);
		#else
		imageData.camera = platforms[imageData.platformID].GetCamera(imageData.cameraID, imageData.poseID);
		imageData.camera.ComposeP();
		#endif
		++nCalibratedImages;
		DEBUG_EXTRA("Image loaded %3u: %s", ID, Util::getFileFullName(imageData.name).c_str());
	}
	if (images.GetSize() < 2)
		return false;

	// import 3D points
	if (!obj.vertices.empty()) {
		pointcloud.points.Resize(obj.vertices.size());
		pointcloud.weights.Resize(obj.vertices.size());
		FOREACH(i, pointcloud.points) {
			const Interface::Vertex& vertex = obj.vertices[i];
			PointCloud::Point& point = pointcloud.points[i];
			point.X = vertex.X;
			point.views.Resize((PointCloud::ViewArr::IDX)vertex.views.size());
			ASSERT(point.views.GetSize() >= 2);
			float& w = pointcloud.weights[0];
			w = 0;
			point.views.ForEach([&](PointCloud::ViewArr::IDX v) {
				const Interface::Vertex::View& view = vertex.views[v];
				point.views[v] = view.imageID;
				w += view.confidence;
			});
		}
		if (!obj.verticesNormal.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesNormal.size());
			pointcloud.normals.CopyOf((const Point3f*)&obj.verticesNormal[0].n, obj.vertices.size());
		}
		if (!obj.verticesColor.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesColor.size());
			pointcloud.colors.CopyOf((const Pixel8U*)&obj.verticesColor[0].c, obj.vertices.size());
		}
	}

	// score neighbors
	FOREACH(ID, images) {
		Image& imageData = images[ID];
		if (imageData.poseID == NO_ID)
			continue;
		ASSERT(imageData.neighbors.IsEmpty());
        IndexArr points;
		SelectNeighborViews(ID, points);
	}
	return true;
} // LoadInterface

bool Scene::SaveInterface(const String & fileName) const
{
	Interface obj;

	// export platforms
	obj.platforms.reserve(platforms.GetSize());
	FOREACH(i, platforms) {
		const Platform& platform = platforms[i];
		Interface::Platform plat;
		plat.cameras.reserve(platform.cameras.GetSize());
		FOREACH(j, platform.cameras) {
			const Platform::Camera& camera = platform.cameras[j];
			Interface::Platform::Camera cam;
			cam.K = camera.K;
			cam.R = camera.R;
			cam.C = camera.C;
			plat.cameras.push_back(cam);
		}
		plat.poses.reserve(platform.poses.GetSize());
		FOREACH(j, platform.poses) {
			const Platform::Pose& pose = platform.poses[j];
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
		image.poseID = imageData.poseID;
		image.platformID = imageData.platformID;
		image.cameraID = imageData.cameraID;
	}

	// export 3D points
	obj.vertices.resize(pointcloud.points.GetSize());
	FOREACH(i, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[i];
		MVS::Interface::Vertex& vertex = obj.vertices[i];
		ASSERT(sizeof(MVS::Interface::Real) == sizeof(point.X.x));
		vertex.X = point.X;
		const float w(pointcloud.weights.IsEmpty() ? 0.f : pointcloud.weights[i]/point.views.GetSize());
		vertex.views.resize(point.views.GetSize());
		point.views.ForEach([&](PointCloud::ViewArr::IDX v) {
			MVS::Interface::Vertex::View& view = vertex.views[v];
			view.imageID = point.views[v];
			view.confidence = w;
		});
	}
	if (!pointcloud.normals.IsEmpty()) {
		obj.verticesNormal.resize(pointcloud.normals.GetSize());
		FOREACH(i, pointcloud.normals) {
			const PointCloud::Normal& normal = pointcloud.normals[i];
			MVS::Interface::VertexNormal& vertexNormal = obj.verticesNormal[i];
			vertexNormal.n = normal;
		}
	}
	if (!pointcloud.normals.IsEmpty()) {
		obj.verticesColor.resize(pointcloud.colors.GetSize());
		FOREACH(i, pointcloud.colors) {
			const PointCloud::Color& color = pointcloud.colors[i];
			MVS::Interface::VertexColor& vertexColor = obj.verticesColor[i];
			vertexColor.c = color;
		}
	}

	#ifdef _USE_BOOST
	// serialize out the current state
	return SerializeSave(obj, fileName, ARCHIVE_BINARY_ZIP);
	#else
	return false;
	#endif
} // SaveInterface
/*----------------------------------------------------------------*/

bool Scene::Load(const String& fileName)
{
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
	// load images
	nCalibratedImages = 0;
	FOREACH(ID, images) {
		Image& imageData = images[ID];
		if (imageData.poseID == NO_ID)
			continue;
		imageData.UpdateCamera(platforms);
		++nCalibratedImages;
	}
	// score neighbors
	FOREACH(ID, images) {
		Image& imageData = images[ID];
		if (imageData.poseID == NO_ID)
			continue;
		if (imageData.neighbors.IsEmpty()) {
			IndexArr points;
			SelectNeighborViews(ID, points);
        }
	}
	return true;
	#else
	return false;
	#endif
} // Load

bool Scene::Save(const String& fileName, ARCHIVE_TYPE type) const
{
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
	return SerializeSave(*this, fs, type);
	#else
	return false;
	#endif
} // Save
/*----------------------------------------------------------------*/


inline float Footprint(const Camera& camera, const Point3f& X) {
	const REAL fSphereRadius(1);
	const Point3 cX(camera.TransformPointW2C(CastReal(X)));
	return (float)norm(camera.TransformPointC2I(Point3(cX.x+fSphereRadius,cX.y,cX.z))-camera.TransformPointC2I(cX))+std::numeric_limits<float>::epsilon();
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
	FOREACH(idx, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[idx];
		if (point.views.FindFirst(ID) == NO_IDX)
			continue;
		// store this point
		if (point.views.GetSize() >= nMinPointViews)
			points.Insert((uint32_t)idx);
		imageData.avgDepth += (float)imageData.camera.PointDepth(point.X);
		++nPoints;
		// score shared views
		const Point3f V1(imageData.camera.C - CastReal(point.X));
		const float footprint1(Footprint(imageData.camera, point.X));
		FOREACHPTR(pView, point.views) {
			const PointCloud::View& view = *pView;
			if (view == ID)
				continue;
			const Image& imageData2 = images[view];
			const Point3f V2(imageData2.camera.C - CastReal(point.X));
			const float footprint2(Footprint(imageData2.camera, point.X));
			const float fAngle(ACOS(ComputeAngle<float,float>(V1.ptr(), V2.ptr())));
			const float fScaleRatio(footprint1/footprint2);
			const float wAngle(MINF(POW(fAngle/fOptimAngle, 1.5f), 1.f));
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
	Point2fArr pointsA(0, points.GetSize()), pointsB(0, points.GetSize());
	FOREACH(IDB, images) {
		const Image& imageDataB = images[IDB];
		if (!imageDataB.IsValid())
			continue;
		const Score& score = scores[IDB];
		if (score.points == 0)
			continue;
		ASSERT(ID != IDB);
		ViewScore& neighbor = neighbors.AddEmpty();
		// compute how well the matched features are spread out (image covered area)
		const Point2f boundsA(imageData.GetSize());
		const Point2f boundsB(imageDataB.GetSize());
		ASSERT(pointsA.IsEmpty() && pointsB.IsEmpty());
		FOREACHPTR(pIdx, points) {
			const PointCloud::Point& point = pointcloud.points[*pIdx];
			ASSERT(point.views.FindFirst(ID) != NO_IDX);
			if (point.views.FindFirst(IDB) == NO_IDX)
				continue;
			Point2f& ptA = pointsA.AddConstruct(imageData.camera.ProjectPointP(point.X));
			Point2f& ptB = pointsB.AddConstruct(imageDataB.camera.ProjectPointP(point.X));
			if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB)) {
				pointsA.RemoveLast();
				pointsB.RemoveLast();
			}
		}
		ASSERT(pointsA.GetSize() == pointsB.GetSize() && pointsA.GetSize() <= score.points);
		const float areaA(ComputeCoveredArea<float, 2, 16, false>((const float*)pointsA.Begin(), pointsA.GetSize(), boundsA.ptr()));
		const float areaB(ComputeCoveredArea<float, 2, 16, false>((const float*)pointsB.Begin(), pointsB.GetSize(), boundsB.ptr()));
		const float area(MINF(areaA, areaB));
		pointsA.Empty(); pointsB.Empty();
		// store image score
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
			msg += String::FormatString(" % 3u(%upts,%.2fscl)", neighbors[n].idx.ID, neighbors[n].idx.points, neighbors[n].idx.scale);
		VERBOSE("Reference image % 3u sees %u views:%s (%u shared points)", ID, neighbors.GetSize(), msg.c_str(), nPoints);
	}
	#endif
	if (points.GetSize() <= 3 || neighbors.GetSize() < MINF(nMinViews,nCalibratedImages-1)) {
		DEBUG_EXTRA("error: reference image % 3u has not enough images in view", ID);
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



// export depth map as an image (dark - far depth, light - close depth)
bool MVS::ExportDepthMap(const String& fileName, const DepthMap& depthMap, Depth minDepth, Depth maxDepth)
{
	// find min and max values
	if (minDepth == FLT_MAX && maxDepth == 0) {
		cList<Depth, const Depth, 0> depths(0, depthMap.area());
		for (size_t i=depthMap.area(); i>0; ) {
			const Depth depth = depthMap[--i];
			ASSERT(depth == 0 || depth > 0);
			if (depth > 0)
				depths.Insert(depth);
		}
		if (!depths.IsEmpty()) {
			const std::pair<Depth,Depth> th(ComputeX84Threshold<Depth,Depth>(depths.Begin(), depths.GetSize()));
			maxDepth = th.first+th.second;
			minDepth = th.first-th.second;
		}
		if (minDepth < 0.1f)
			minDepth = 0.1f;
		if (maxDepth < 0.1f)
			maxDepth = 30.f;
		DEBUG_ULTIMATE("Depth range: %g min - %g max", minDepth, maxDepth);
	}
	const Depth deltaDepth = maxDepth - minDepth;
	// save image
	Image8U img(depthMap.size());
	for (size_t i=depthMap.area(); i>0; ) {
		const Depth depth = depthMap[--i];
		img[i] = (depth > 0 ? (uint8_t)CLAMP((maxDepth-depth)*255.f/deltaDepth, 0.f, 255.f) : 0);
	}
	return img.Save(fileName);
} // ExportDepthMap
/*----------------------------------------------------------------*/

// export normal map as an image
bool MVS::ExportNormalMap(const String& fileName, const NormalMap& normalMap)
{
	if (normalMap.empty())
		return false;
	Image8U3 img(normalMap.size());
	normalMap.convertTo(img, img.type(), 255.f/2.f, 255.f/2.f);
	return img.Save(fileName);
} // ExportNormalMap
/*----------------------------------------------------------------*/

// export confidence map as an image (dark - low confidence, light - high confidence)
bool MVS::ExportConfidenceMap(const String& fileName, const ConfidenceMap& confMap)
{
	// find min and max values
	FloatArr confs(0, confMap.area());
	for (size_t i=confMap.area(); i>0; ) {
		const float conf = confMap[--i];
		ASSERT(conf == 0 || conf > 0);
		if (conf > 0)
			confs.Insert(conf);
	}
	if (confs.IsEmpty())
		return false;
	float minConf, maxConf;
	if (!confs.IsEmpty()) {
		const std::pair<float,float> th(ComputeX84Threshold<float,float>(confs.Begin(), confs.GetSize()));
		minConf = th.first-th.second;
		maxConf = th.first+th.second;
	}
	if (minConf < 0.1f)
		minConf = 0.1f;
	if (maxConf < 0.1f)
		maxConf = 30.f;
	DEBUG_ULTIMATE("Confidence range: %g min - %g max", minConf, maxConf);
	const float deltaConf = maxConf - minConf;
	// save image
	Image8U img(confMap.size());
	for (size_t i=confMap.area(); i>0; ) {
		const float conf = confMap[--i];
		img[i] = (conf > 0 ? (uint8_t)CLAMP((conf-minConf)*255.f/deltaConf, 0.f, 255.f) : 0);
	}
	return img.Save(fileName);
} // ExportConfidenceMap
/*----------------------------------------------------------------*/
