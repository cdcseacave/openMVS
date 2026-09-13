/*
* SceneTransform.cpp
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

// Rigid/similarity transforms of a scene: Center, Scale, ScaleImages,
// ComputeNormalizationTransform, Transform, AlignTo, ComputeLeveledVolume, AddNoiseCameraPoses.

#include "Common.h"
#include "Scene.h"
#include "../Math/SimilarityTransform.h"


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


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

// compute translation and scale (optional) such that the scene coordinates center at 0 and
// most scene geomatry is in the unit cube ([-0.5,0.5]^3);
// return the transformation matrix that restores the scene to its original coordinates
Matrix4x4 Scene::ComputeNormalizationTransform(bool bScale) const
{
	ASSERT(!pointcloud.IsEmpty() || !mesh.IsEmpty());
	// compute the center of the scene geometry (point-cloud or mesh)
	Point3 center = Point3::ZERO;
	if (!mesh.IsEmpty()) {
		for (const Mesh::Vertex& X: mesh.vertices)
			center += Cast<REAL>(X);
		center /= static_cast<REAL>(mesh.vertices.size());
	} else {
		for (const PointCloud::Point& X: pointcloud.points)
			center += Cast<REAL>(X);
		center /= static_cast<REAL>(pointcloud.points.size());
	}
	// compute the scale of the scene geometry (point-cloud or mesh)
	REAL scale = 1;
	if (bScale) {
		REAL avgDist = 0;
		if (!mesh.IsEmpty()) {
			for (const Mesh::Vertex& X: mesh.vertices)
				avgDist += norm(Cast<REAL>(X)-center);
			avgDist /= static_cast<REAL>(mesh.vertices.size());
		} else {
			for (const PointCloud::Point& X: pointcloud.points)
				avgDist += norm(Cast<REAL>(X)-center);
			avgDist /= static_cast<REAL>(pointcloud.points.size());
		}
		scale = REAL(2) * avgDist;
	}
	// compute the transformation matrix
	Matrix4x4 transform = Matrix4x4::ZERO;
	transform(0,0) = scale;
	transform(1,1) = scale;
	transform(2,2) = scale;
	transform(0,3) = center.x;
	transform(1,3) = center.y;
	transform(2,3) = center.z;
	transform(3,3) = 1;
    return transform;
} // ComputeNormalizationTransform

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
	transform = Matrix4x4::IDENTITY;
	Matrix4x4::EMatMap mapTransform(transform);
	mapTransform.topLeftCorner<3,3>() = static_cast<Matrix3x3::CEMatMap>(rotationScale);
	mapTransform.topRightCorner<3,1>() = static_cast<Point3::CEVecMap>(translation);
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
	Point3Arr points, pointsRef;
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
	Matrix4x4 transform = SimilarityTransform(points, pointsRef);
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

// add noise to camera poses:
//  - epsPosition: noise in camera position (in scene units)
//  - epsRotation: noise in camera rotation (in radians)
void Scene::AddNoiseCameraPoses(float epsPosition, float epsRotation)
{
	for (Platform& platform: platforms) {
		for (Platform::Pose& pose: platform.poses) {
			pose.C += Point3((Point3::EVec::Random() * epsPosition).eval());
			pose.R = RMatrix(RMatrix::Vec(Point3((epsRotation * Point3::EVec::Random()).eval()))) * pose.R;
		}
	}
	for (Image& imageData: images) {
		if (!imageData.IsValid())
			continue;
		imageData.UpdateCamera(platforms);
	}
}

#pragma pop_macro("VERBOSE")
