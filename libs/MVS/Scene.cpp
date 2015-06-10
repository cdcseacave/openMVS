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


// S T R U C T S ///////////////////////////////////////////////////

bool Scene::Load(const String & fileName, const String& workingFolderFull)
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
	platforms.Reserve(obj.platforms.size());
	for (Interface::PlatformArr::const_iterator itPlatform=obj.platforms.begin(); itPlatform!=obj.platforms.end(); ++itPlatform) {
		Platform& platform = platforms.AddEmpty();
		platform.name = itPlatform->name;
		platform.cameras.Reserve(itPlatform->cameras.size());
		for (Interface::Platform::CameraArr::const_iterator itCamera=itPlatform->cameras.begin(); itCamera!=itPlatform->cameras.end(); ++itCamera) {
			Platform::Camera& camera = platform.cameras.AddEmpty();
			camera.K = itCamera->K;
			camera.R = itCamera->R;
			camera.C = itCamera->C;
			DEBUG_EXTRA("Camera model loaded: platform %u; camera %2u; f %.3fx%.3f; poses %u", platforms.GetSize()-1, platform.cameras.GetSize()-1, camera.K(0,0), camera.K(1,1), itPlatform->poses.size());
		}
		ASSERT(platform.cameras.GetSize() == itPlatform->cameras.size());
		platform.poses.Reserve(itPlatform->poses.size());
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
	ASSERT(!obj.images.empty());
	images.Reserve(obj.images.size());
	for (Interface::ImageArr::const_iterator it=obj.images.begin(); it!=obj.images.end(); ++it) {
		const Interface::Image& image = *it;
		Image& imageData = images.AddEmpty();
		imageData.name = image.name;
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(workingFolderFull, imageData.name);
		imageData.poseID = image.poseID;
		if (imageData.poseID == NO_ID)
			continue;
		imageData.platformID = image.platformID;
		imageData.cameraID = image.cameraID;
		if (!imageData.ReloadImage(0, false))
			return false;
		imageData.UpdateCamera(platforms);
		DEBUG_EXTRA("Image loaded %3u: %s", images.GetSize()-1, Util::getFileFullName(imageData.name).c_str());
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

	return true;
}

bool Scene::Save(const String & fileName, const String& workingFolderFull)
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
		image.name = MAKE_PATH_REL(workingFolderFull, imageData.name);
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
}
/*----------------------------------------------------------------*/
