/*
* SceneTower.cpp
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

// Tower-like scene detection and synthesis: ComputeCenterLine, ComputeTowerCylinder,
// DrawCircle, BuildTowerMesh, InitTowerScene.

#include "Common.h"
#include "Scene.h"


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


// Compute the center line of the tower by fitting a line to the camera positions
// Returns true if the camera poses describe a cylinder, false otherwise;
// if the up direction is given, the line must additionally be (near) vertical, so that
// horizontal linear trajectories (corridors, single flight lines) are not misclassified
bool Scene::ComputeCenterLine(Line3f &camCenterLine, const Point3f* up) const {
	if (images.size() < 20) {
		DEBUG_ULTIMATE("error: too few images to be a tower: '%d'", images.size());
		return false;
	}
	FitLineOnline<float> fitline;
	FOREACH(imgIdx, images) {
		const Eigen::Vector3f camPos(Cast<float>(images[imgIdx].camera.C));
		fitline.Update(camPos);
	}
	Point3f quality = fitline.GetLine(camCenterLine);
	// check if ROI is mostly long and narrow on one direction
	if (quality.y / quality.z > 0.6f || quality.x / quality.y < 0.8f) {
		// does not seem to be a line
		DEBUG_ULTIMATE("scene does not seem to be a tower: X(%.2f), Y(%.2f), Z(%.2f)", quality.x, quality.y, quality.z);
		return false;
	}
	if (up) {
		const Eigen::Vector3f dir((camCenterLine.pt2 - camCenterLine.pt1).normalized());
		const float cosVertical(ABS(dir.dot(Eigen::Vector3f(up->x, up->y, up->z))));
		if (cosVertical < COS(D2R(30.f))) {
			DEBUG_ULTIMATE("scene does not seem to be a tower: camera line %.1f deg off vertical", R2D(ACOS(MINF(cosVertical, 1.f))));
			return false;
		}
	}
	return true;
}

// calculate the center(X,Y) of the cylinder, the radius and min/max Z
// from camera position and sparse point-cloud, if that exists
// returns result of checks if the scene camera positions satisfies tower criteria:
//	- cameras fit a long and slim bounding box
//  - majority of cameras focus toward a middle line
// Tower mode is assumed to be nonzero
bool Scene::ComputeTowerCylinder(Point2f& centerPoint, float& fRadius, float& fROIRadius, float& zMin, float& zMax, float& minCamZ, const int towerMode)
{
	// disregard tower mode for scenes with less than 20 cameras
	if (towerMode > 0 && images.size() < 20) {
		DEBUG_ULTIMATE("error: too few images to be a tower: '%d'", images.size());
		return false;
	}

	Line3f camCenterLine;
	if (!ComputeCenterLine(camCenterLine))
		return false;

	AABB3f aabbOutsideCameras(true);
	CLISTDEF0(Point2f) cameras2D(images.size());
	FloatArr camHeigths;
	FOREACH(imgIdx, images) {
		const Eigen::Vector3f camPos(Cast<float>(images[imgIdx].camera.C));
		aabbOutsideCameras.InsertFull(camPos);
		cameras2D[imgIdx] = Point2f(camPos.x(), camPos.y());
		camHeigths.InsertSortUnique(camPos.z());
	}

	// get the height of the lowest camera
	minCamZ = aabbOutsideCameras.ptMin.z();
	centerPoint = ((camCenterLine.pt1+camCenterLine.pt2)*0.5f).topLeftCorner<2,1>();
	zMin = MINF(aabbOutsideCameras.ptMax.z(), aabbOutsideCameras.ptMin.z()) - 5;
	// if sparse point-cloud is loaded use lowest point as zMin
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
		fROIRadius = cameraDistancesToMiddle.GetTrimmedMean(0.85f, 0.05f);
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
		const Normal n(COS(fAngle), SIN(fAngle), 0);
		ASSERT(ISEQUAL(norm(n), 1.f), "Norm = ", norm(n));
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
		const unsigned nTargetPoints(MAXF(10, ROUND2INT(FTWO_PI * fRadius * nTargetDensity))); // how many points on each circle
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
				if (bIdx < (int)nTargetCircles - 1 && bIdx >= 0)
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
					// the average of the top 50 to 95% distances, dropping at least the nearest and the farthest
					const float avgTopDistance(pDistances.GetTrimmedMean(0.5f, 0.05f, 1));
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
			const unsigned nTargetPoints(MAXF(10, ROUND2INT(FTWO_PI * circleRadius * nTargetDensity))); // how many points on each circle
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
		mesh.Save(MAKE_PATH("tower_mesh.ply"));
	} else
	#endif
	{
		mesh.Release();
	}
	towerPC.Save(MAKE_PATH("tower.ply"));
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
	if (towerMode == 0)
		return;
	if (!ComputeTowerCylinder(centerPoint, fRadius, fROIRadius, zMin, zMax, minCamZ, towerMode))
		return;

	// add nTargetPoints points on each circle
	PointCloud towerPC(BuildTowerMesh(pointcloud, centerPoint, fRadius, fROIRadius, zMin, zMax, minCamZ, false));
	mesh.Release();

	const auto AppendPointCloud = [this](const PointCloud& towerPC) {
		bool bHasNormal(towerPC.normals.size() == towerPC.GetSize());
		bool bHasColor(towerPC.colors.size() == towerPC.GetSize());
		bool bHasWeights(towerPC.pointWeights.size() == towerPC.GetSize());
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
		SelectNeighborViews(OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, D2R(OPTDENSE::fOptimAngle), OPTDENSE::fWeightPointInsideROI);
		pointcloud.Swap(towerPC);
		VERBOSE("Scene identified as tower-like; only select view neighbors from detected tower point-cloud");
		break;
	case 4: // select neighbors and append tower points
		pointcloud.Swap(towerPC);
		SelectNeighborViews(OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, D2R(OPTDENSE::fOptimAngle), OPTDENSE::fWeightPointInsideROI);
		pointcloud.Swap(towerPC);
		AppendPointCloud(towerPC);
		VERBOSE("Scene identified as tower-like; select view neighbors from detected tower point-cloud and next append it to existing point-cloud");
		break;
	}
} // InitTowerScene
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
