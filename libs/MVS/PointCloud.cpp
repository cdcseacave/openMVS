/*
* PointCloud.cpp
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
#include "PointCloud.h"
#include "DepthMap.h"
// GLTF: mesh import/export
#include <tiny_gltf.h>
#include "../IO/json.hpp"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("PointCld"));

PointCloud& MVS::PointCloud::Swap(PointCloud& rhs)
{
	points.Swap(rhs.points);
	pointViews.Swap(rhs.pointViews);
	pointWeights.Swap(rhs.pointWeights);
	normals.Swap(rhs.normals);
	colors.Swap(rhs.colors);
	labels.Swap(rhs.labels);
	return *this;
}
/*----------------------------------------------------------------*/

void PointCloud::Release()
{
	points.Release();
	pointViews.Release();
	pointWeights.Release();
	normals.Release();
	colors.Release();
	labels.Release();
}
/*----------------------------------------------------------------*/


void PointCloud::RemovePoint(IDX idx)
{
	ASSERT(pointViews.empty() || pointViews.size() == points.size());
	if (!pointViews.empty())
		pointViews.RemoveAt(idx);
	ASSERT(pointWeights.empty() || pointWeights.size() == points.size());
	if (!pointWeights.empty())
		pointWeights.RemoveAt(idx);
	ASSERT(normals.empty() || normals.size() == points.size());
	if (!normals.empty())
		normals.RemoveAt(idx);
	ASSERT(colors.empty() || colors.size() == points.size());
	if (!colors.empty())
		colors.RemoveAt(idx);
	ASSERT(labels.empty() || labels.size() == points.size());
	if (!labels.empty())
		labels.RemoveAt(idx);
	points.RemoveAt(idx);
}

// remove multiple points based on the indices provided;
// the indices must be sorted in ascending order
void PointCloud::RemovePoints(IndexArr& indices)
{
	ASSERT(!indices.empty());
	indices.Sort();
	RFOREACH(idx, indices)
		RemovePoint(indices[idx]);
}

void PointCloud::RemovePointsOutside(const OBB3f &obb)
{
	ASSERT(obb.IsValid());
	RFOREACH(i, points)
		if (!obb.Intersects(points[i]))
			RemovePoint(i);
}
void PointCloud::RemoveMinViews(uint32_t thMinViews) {
	ASSERT(!pointViews.empty());
	RFOREACH(i, points)
		if (pointViews[i].size() < thMinViews)
			RemovePoint(i);
}
/*----------------------------------------------------------------*/


// compute the axis-aligned bounding-box of the point-cloud
PointCloud::Box PointCloud::GetAABB() const
{
	Box box(true);
	for (const Point& X: points)
		box.InsertFull(X);
	return box;
}
// same, but only for points inside the given AABB
// optionally consider only points with more than the given number of views
PointCloud::Box PointCloud::GetAABB(const Box& bound, unsigned minViews) const
{
	Box box(true);
	if (!pointViews.empty() && minViews > 0) {
		FOREACH(idx, points) {
			if (pointViews[idx].size() < minViews)
				continue;
			const Point& X = points[idx];
			if (bound.Intersects(X))
				box.InsertFull(X);
		}
	} else {
		for (const Point& X: points)
			if (bound.Intersects(X))
				box.InsertFull(X);
	}
	return box;
}
// compute the axis-aligned bounding-box of the point-cloud
// with more than the given number of views
PointCloud::Box PointCloud::GetAABB(unsigned minViews) const
{
	if (pointViews.empty())
		return GetAABB();
	Box box(true);
	FOREACH(idx, points)
		if (pointViews[idx].size() >= minViews)
			box.InsertFull(points[idx]);
	return box;
}
// compute the axis-aligned bounding-box of the point-cloud
// considering only points within the given percentile range per axis
// optionally with more than the given number of views
PointCloud::Box PointCloud::GetAABB(float minPercentile, float maxPercentile, unsigned minViews) const
{
	// get percentile bounds for each axis
	const Box percentileBounds(GetPercentileAABB(minPercentile, maxPercentile, minViews));
	// compute AABB from points within percentile bounds
	return GetAABB(percentileBounds, minViews);
}
// compute the percentile axis-aligned bounding-box of the point-cloud
// optionally with more than the given number of views
PointCloud::Box PointCloud::GetPercentileAABB(float minPercentile, float maxPercentile, unsigned minViews) const
{
	ASSERT(minPercentile >= 0.f && minPercentile <= 1.f);
	ASSERT(maxPercentile >= 0.f && maxPercentile <= 1.f);
	ASSERT(minPercentile < maxPercentile);
	// collect points that meet the minViews requirement
	typedef CLISTDEF0IDX(Point::Type,Index) Scalars;
	Scalars x, y, z;
	x.reserve(points.size());
	y.reserve(points.size());
	z.reserve(points.size());
	if (!pointViews.empty() && minViews > 0) {
		FOREACH(idx, points) {
			if (pointViews[idx].size() >= minViews) {
				const Point& X = points[idx];
				x.push_back(X.x);
				y.push_back(X.y);
				z.push_back(X.z);
			}
		}
	} else {
		for (const Point& X: points) {
			x.push_back(X.x);
			y.push_back(X.y);
			z.push_back(X.z);
		}
	}
	if (x.empty())
		return Box(true);
	// compute percentile indices
	x.Sort();
	y.Sort();
	z.Sort();
	const float numPoints(x.size() - 1);
	const Index idxMin(MAXF(Index(0), ROUND2INT<Index>(minPercentile * numPoints)));
	const Index idxMax(MINF(static_cast<Index>(numPoints), ROUND2INT<Index>(maxPercentile * numPoints)));
	// return percentile bounds for each axis
	return Box(
		Box::POINT(x[idxMin], y[idxMin], z[idxMin]),
		Box::POINT(x[idxMax], y[idxMax], z[idxMax]));
}

// compute the center of the point-cloud as the median
PointCloud::Point PointCloud::GetCenter() const
{
	const Index step(5);
	const Index numPoints(points.size()/step);
	if (numPoints == 0)
		return Point::INF;
	typedef CLISTDEF0IDX(Point::Type,Index) Scalars;
	Scalars x(numPoints), y(numPoints), z(numPoints);
	for (Index i=0; i<numPoints; ++i) {
		const Point& X = points[i*step];
		x[i] = X.x;
		y[i] = X.y;
		z[i] = X.z;
	}
	return Point(x.GetMedian(), y.GetMedian(), z.GetMedian());
}
/*----------------------------------------------------------------*/


// estimate the ground-plane as the plane agreeing with most points
//  - planeThreshold: threshold used to estimate the ground plane (0 - auto)
Planef PointCloud::EstimateGroundPlane(const ImageArr& images, float planeThreshold, const String& fileExportPlane) const
{
	ASSERT(!IsEmpty());

	// remove some random points to speed up plane fitting
	const unsigned randMinPoints(150000);
	PointArr workPoints;
	const PointArr* pPoints;
	if (GetSize() > randMinPoints) {
		#ifndef _RELEASE
		SEACAVE::Random rnd(SEACAVE::Random::default_seed);
		#else
		SEACAVE::Random rnd;
		#endif
		const REAL randPointsRatio(MAXF(REAL(1e-4),(REAL)randMinPoints/GetSize()));
		const SEACAVE::Random::result_type randPointsTh(CEIL2INT<SEACAVE::Random::result_type>(randPointsRatio*SEACAVE::Random::max()));
		workPoints.reserve(CEIL2INT<PointArr::IDX>(randPointsRatio*GetSize()));
		for (const Point& X: points)
			if (rnd() <= randPointsTh)
				workPoints.emplace_back(X);
		pPoints = &workPoints;
	} else {
		pPoints = &points;
	}

	// fit plane to the point-cloud
	Planef plane;
	const float minInliersRatio(0.05f);
	double threshold(planeThreshold>0 ? (double)planeThreshold : DBL_MAX);
	const unsigned numInliers(planeThreshold > 0 ? EstimatePlaneTh(*pPoints, plane, threshold) : EstimatePlane(*pPoints, plane, threshold));
	if (numInliers < MINF(ROUND2INT<unsigned>(minInliersRatio*pPoints->size()), 1000u)) {
		plane.Invalidate();
		return plane;
	}
	if (planeThreshold <= 0)
		DEBUG("Ground plane estimated threshold: %g", threshold);

	// refine plane to inliers
	CLISTDEF0(Planef::POINT) inliers;
	const float maxThreshold(static_cast<float>(threshold * 2));
	for (const Point& X: *pPoints)
		if (plane.DistanceAbs(X) < maxThreshold)
			inliers.emplace_back(X);
	const RobustNorm::GemanMcClure<double> robust(threshold);
	plane.Optimize(inliers.data(), inliers.size(), robust);

	// make sure the plane is well oriented, negate plane normal if it faces same direction as cameras on average
	if (!images.empty()) {
		FloatArr cosView(0, images.size());
		for (const Image& imageData: images) {
			if (!imageData.IsValid())
				continue;
			cosView.push_back(plane.m_vN.dot((const Point3f::EVecMap&)Cast<float>(imageData.camera.Direction())));
		}
		if (cosView.GetMedian() > 0)
			plane.Negate();
	}

	// export points on the found plane if requested
	if (!fileExportPlane.empty()) {
		PointCloud pc;
		const Point orig(Point(plane.m_vN)*-plane.m_fD);
		pc.colors.emplace_back(Color::RED);
		pc.points.emplace_back(orig);
		for (const Point& X: *pPoints) {
			const float dist(plane.DistanceAbs(X));
			if (dist < threshold) {
				pc.points.emplace_back(X);
				const uint8_t color((uint8_t)(255.f*(1.f-dist/threshold)));
				pc.colors.emplace_back(color, color, color);
			}
		}
		pc.Save(fileExportPlane);
	}
	return plane;
}
/*----------------------------------------------------------------*/


// define a PLY file format composed only of vertices
namespace PointCloudInternal {
namespace BasicPLY {
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex"
	};
	// list of property information for a vertex
	struct Vertex {
		PointCloud::Point p;
		PointCloud::Color c;
		PointCloud::Normal n;
		struct Views {
			uint8_t num;
			uint32_t* pIndices;
			float* pWeights;
		} views;
		PointCloud::Label label;
		float confidence;
		float scale;
		static void InitLoadProps(PLY& ply, int elem_count,
			PointCloud::PointArr& points, PointCloud::ColorArr& colors, PointCloud::NormalArr& normals, PointCloud::LabelArr& labels, PointCloud::PointViewArr& views, PointCloud::PointWeightArr& weights)
		{
			PLY::PlyElement* elm = ply.find_element(elem_names[0]);
			const size_t nMaxProps(SizeOfArray(props));
			for (size_t p=0; p<nMaxProps; ++p) {
				if (ply.find_property(elm, props[p].name.c_str()) < 0)
					continue;
				ply.setup_property(props[p]);
				switch (p) {
				case 0: points.resize((IDX)elem_count); break;
				case 3: case 13: colors.resize((IDX)elem_count); break;
				case 6: normals.resize((IDX)elem_count); break;
				case 9: views.resize((IDX)elem_count); break;
				case 10: weights.resize((IDX)elem_count); break;
				case 11: labels.resize((IDX)elem_count); break;
				}
			}
		}
		static void InitSaveProps(PLY& ply, int elem_count,
			bool bColors, bool bNormals, bool bViews, bool bWeights, bool bLabel=false, bool bConfidence=false, bool bScale=false)
		{
			ply.describe_property(elem_names[0], 3, props+0);
			if (bColors)
				ply.describe_property(elem_names[0], 3, props+3);
			if (bNormals)
				ply.describe_property(elem_names[0], 3, props+6);
			if (bViews)
				ply.describe_property(elem_names[0], props[9]);
			if (bWeights)
				ply.describe_property(elem_names[0], props[10]);
			if (bLabel)
				ply.describe_property(elem_names[0], props[11]);
			if (bConfidence)
				ply.describe_property(elem_names[0], props[12]);
			if (bScale)
				ply.describe_property(elem_names[0], props[13]);
			if (elem_count)
				ply.element_count(elem_names[0], elem_count);
		}
		static const PLY::PlyProperty props[17];
	};
	const PLY::PlyProperty Vertex::props[17] = {
		{"x",             PLY::Float32, PLY::Float32, offsetof(Vertex,p.x), 0, 0, 0, 0},
		{"y",             PLY::Float32, PLY::Float32, offsetof(Vertex,p.y), 0, 0, 0, 0},
		{"z",             PLY::Float32, PLY::Float32, offsetof(Vertex,p.z), 0, 0, 0, 0},
		{"red",           PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.r), 0, 0, 0, 0},
		{"green",         PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.g), 0, 0, 0, 0},
		{"blue",          PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.b), 0, 0, 0, 0},
		{"nx",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.x), 0, 0, 0, 0},
		{"ny",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.y), 0, 0, 0, 0},
		{"nz",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.z), 0, 0, 0, 0},
		{"view_indices",  PLY::Uint32,  PLY::Uint32,  offsetof(Vertex,views.pIndices), 1, PLY::Uint8, PLY::Uint8, offsetof(Vertex,views.num)},
		{"view_weights",  PLY::Float32, PLY::Float32, offsetof(Vertex,views.pWeights), 1, PLY::Uint8, PLY::Uint8, offsetof(Vertex,views.num)},
		{"label",         PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,label), 0, 0, 0, 0},
		{"confidence",    PLY::Float32, PLY::Float32, offsetof(Vertex,confidence), 0, 0, 0, 0},
		{"value",         PLY::Float32, PLY::Float32, offsetof(Vertex,scale), 0, 0, 0, 0},
		// duplicates
		{"diffuse_red",   PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.r), 0, 0, 0, 0},
		{"diffuse_green", PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.g), 0, 0, 0, 0},
		{"diffuse_blue",  PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.b), 0, 0, 0, 0}
	};
} // namespace BasicPLY
} // namespace PointCloudInternal

// load the dense point-cloud from a PLY file
bool PointCloud::Load(const String& fileName)
{
	TD_TIMER_STARTD();
	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = LoadGLTF(fileName, ext == _T(".glb"));
	else
		ret = LoadPLY(fileName);
	if (!ret)
		return false;
	DEBUG_EXTRA("Point-cloud '%s' loaded: %u points (%s)", Util::getFileNameExt(fileName).c_str(), points.size(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Load

// import the point-cloud as a PLY file
bool PointCloud::LoadPLY(const String& fileName)
{
	ASSERT(!fileName.empty());
	Release();

	// open PLY file and read header
	using namespace PointCloudInternal;
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}

	// read PLY body
	for (int i = 0; i < ply.get_elements_count(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			BasicPLY::Vertex::InitLoadProps(ply, elem_count, points, colors, normals, labels, pointViews, pointWeights);
			BasicPLY::Vertex vertex;
			for (int v=0; v<elem_count; ++v) {
				ply.get_element(&vertex);
				points[v] = vertex.p;
				if (!colors.empty())
					colors[v] = vertex.c;
				if (!normals.empty())
					normals[v] = vertex.n;
				if (!labels.empty())
					labels[v] = vertex.label;
				if (!pointViews.empty()) {
					ViewArr pv(vertex.views.num, vertex.views.pIndices);
					pointViews[v].CopyOfRemove(pv);
				}
				if (!pointWeights.empty()){
					WeightArr pw(vertex.views.num, vertex.views.pWeights);
					pointWeights[v].CopyOfRemove(pw);
				}
			}
		} else {
			ply.get_other_element();
		}
	}
	if (points.empty()) {
		DEBUG_EXTRA("error: invalid point-cloud");
		return false;
	}
	return true;
}

// import the point-cloud as a GLTF file
bool PointCloud::LoadGLTF(const String& fileName, bool bBinary)
{
	ASSERT(!fileName.empty());
	Release();

	// load model
	tinygltf::Model gltfModel; {
		tinygltf::TinyGLTF loader;
		std::string err, warn;
		if (bBinary ?
			!loader.LoadBinaryFromFile(&gltfModel, &err, &warn, fileName) :
			!loader.LoadASCIIFromFile(&gltfModel, &err, &warn, fileName))
			return false;
		if (!err.empty()) {
			VERBOSE("error: %s", err.c_str());
			return false;
		}
		if (!warn.empty())
			DEBUG("warning: %s", warn.c_str());
	}

	// parse model
	for (const tinygltf::Mesh& gltfMesh : gltfModel.meshes) {
		for (const tinygltf::Primitive& gltfPrimitive : gltfMesh.primitives) {
			if (gltfPrimitive.mode != TINYGLTF_MODE_POINTS)
				continue;
			// read vertices
			{
				const tinygltf::Accessor& gltfAccessor = gltfModel.accessors[gltfPrimitive.attributes.at("POSITION")];
				if (gltfAccessor.type != TINYGLTF_TYPE_VEC3)
					continue;
				const tinygltf::BufferView& gltfBufferView = gltfModel.bufferViews[gltfAccessor.bufferView];
				const tinygltf::Buffer& buffer = gltfModel.buffers[gltfBufferView.buffer];
				const uint8_t* pData = buffer.data.data() + gltfBufferView.byteOffset + gltfAccessor.byteOffset;
				const size_t oldSize = points.size();
				points.resize(oldSize + (Index)gltfAccessor.count);
				if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
					const int stride = gltfAccessor.ByteStride(gltfBufferView);
					if (stride == sizeof(Point)) {
						memcpy(points.data() + oldSize, pData, sizeof(Point) * gltfAccessor.count);
					} else {
						for (size_t i = 0; i < gltfAccessor.count; ++i)
							points[oldSize+i] = *(const Point*)(pData + i * stride);
					}
				}
				else if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_DOUBLE) {
					const int stride = gltfAccessor.ByteStride(gltfBufferView);
					for (Index i = 0; i < gltfAccessor.count; ++i) {
						const double* pVal = (const double*)(pData + i * stride);
						points[oldSize+i] = Point(pVal[0], pVal[1], pVal[2]);
					}
				}
				else {
					VERBOSE("error: unsupported vertices (component type)");
					continue;
				}
			}
			// read colors (COLOR_0)
			if (gltfPrimitive.attributes.find("COLOR_0") != gltfPrimitive.attributes.end()) {
				const tinygltf::Accessor& gltfAccessor = gltfModel.accessors[gltfPrimitive.attributes.at("COLOR_0")];
				const tinygltf::BufferView& gltfBufferView = gltfModel.bufferViews[gltfAccessor.bufferView];
				const tinygltf::Buffer& buffer = gltfModel.buffers[gltfBufferView.buffer];
				const uint8_t* pData = buffer.data.data() + gltfBufferView.byteOffset + gltfAccessor.byteOffset;
				const size_t oldSize = colors.size();
				colors.resize(points.size());

				const int stride = gltfAccessor.ByteStride(gltfBufferView);
				if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
					if (gltfAccessor.type == TINYGLTF_TYPE_VEC3) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const uint8_t* pVal = (const uint8_t*)(pData + i * stride);
							colors[oldSize+i] = Color(pVal[0], pVal[1], pVal[2]);
						}
					} else if (gltfAccessor.type == TINYGLTF_TYPE_VEC4) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const uint8_t* pVal = (const uint8_t*)(pData + i * stride);
							colors[oldSize+i] = Color(pVal[0], pVal[1], pVal[2]);
						}
					}
				} else if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
					if (gltfAccessor.type == TINYGLTF_TYPE_VEC3) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const uint16_t* pVal = (const uint16_t*)(pData + i * stride);
							colors[oldSize+i] = Color(pVal[0]>>8, pVal[1]>>8, pVal[2]>>8);
						}
					} else if (gltfAccessor.type == TINYGLTF_TYPE_VEC4) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const uint16_t* pVal = (const uint16_t*)(pData + i * stride);
							colors[oldSize+i] = Color(pVal[0]>>8, pVal[1]>>8, pVal[2]>>8);
						}
					}
				} else if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
					if (gltfAccessor.type == TINYGLTF_TYPE_VEC3) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const float* pVal = (const float*)(pData + i * stride);
							colors[oldSize+i] = Color((uint8_t)(pVal[0]*255), (uint8_t)(pVal[1]*255), (uint8_t)(pVal[2]*255));
						}
					} else if (gltfAccessor.type == TINYGLTF_TYPE_VEC4) {
						for (size_t i = 0; i < gltfAccessor.count; ++i) {
							const float* pVal = (const float*)(pData + i * stride);
							colors[oldSize+i] = Color((uint8_t)(pVal[0]*255), (uint8_t)(pVal[1]*255), (uint8_t)(pVal[2]*255));
						}
					}
				}
			}
			// read normals (NORMAL)
			if (gltfPrimitive.attributes.find("NORMAL") != gltfPrimitive.attributes.end()) {
				const tinygltf::Accessor& gltfAccessor = gltfModel.accessors[gltfPrimitive.attributes.at("NORMAL")];
				const tinygltf::BufferView& gltfBufferView = gltfModel.bufferViews[gltfAccessor.bufferView];
				const tinygltf::Buffer& buffer = gltfModel.buffers[gltfBufferView.buffer];
				const uint8_t* pData = buffer.data.data() + gltfBufferView.byteOffset + gltfAccessor.byteOffset;
				const size_t oldSize = normals.size();
				normals.resize(points.size());

				const int stride = gltfAccessor.ByteStride(gltfBufferView);
				if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
					if (stride == sizeof(Normal)) {
						memcpy(normals.data() + oldSize, pData, sizeof(Normal) * gltfAccessor.count);
					} else {
						for (size_t i = 0; i < gltfAccessor.count; ++i)
							normals[oldSize+i] = *(const Normal*)(pData + i * stride);
					}
				}
			}
		}
	}
	if (points.empty()) {
		DEBUG_EXTRA("error: invalid point-cloud");
		return false;
	}
	return true;
} // LoadGLTF

bool PointCloud::Save(const String& fileName, bool bViews, bool bLegacyTypes, bool bBinary) const
{
	if (IsEmpty())
		return false;
	TD_TIMER_STARTD();

	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".potree") || ext.empty())
		ret = SavePotree(fileName);
	else if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = SaveGLTF(fileName, ext == _T(".glb"));
	else
		ret = SavePLY(fileName, bViews, bLegacyTypes, bBinary);
	if (!ret)
		return false;

	DEBUG_EXTRA("Point-cloud '%s' saved: %u points (%s)", Util::getFileNameExt(fileName).c_str(), points.size(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Save

// save the dense point-cloud as PLY file
bool PointCloud::SavePLY(const String& fileName, bool bViews, bool bLegacyTypes, bool bBinary) const
{
	if (IsEmpty())
		return false;

	// create PLY object
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	using namespace PointCloudInternal;
	PLY ply;
	if (bLegacyTypes)
		ply.set_legacy_type_names();
	if (!ply.write(fileName, 1, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII))
		return false;

	// write the header
	BasicPLY::Vertex::InitSaveProps(ply, (int)points.size(), !colors.empty(), !normals.empty(),
		bViews && !pointViews.empty(), bViews && !pointWeights.empty(), bViews && !labels.empty());
	if (!ply.header_complete())
		return false;

	// export the array of 3D points
	BasicPLY::Vertex vertex;
	FOREACH(i, points) {
		// export the vertex position, color, normal and views
		vertex.p = points[i];
		if (!colors.empty())
			vertex.c = colors[i];
		if (!normals.empty())
			vertex.n = normals[i];
		if (!labels.empty())
			vertex.label = labels[i];
		if (!pointViews.empty()) {
			vertex.views.num = pointViews[i].size();
			vertex.views.pIndices = pointViews[i].data();
		}
		if (!pointWeights.empty()) {
			ASSERT(vertex.views.num == pointWeights[i].size());
			vertex.views.pWeights = pointWeights[i].data();
		}
		ply.put_element(&vertex);
	}
	ASSERT(ply.get_current_element_count() == (int)points.size());
	return true;
}

// save the dense point-cloud as PLY file
template <typename T>
void ExtendBufferGLTF(const T* src, size_t size, tinygltf::Buffer& dst, size_t& byte_offset, size_t& byte_length) {
	byte_offset = dst.data.size();
	byte_length = sizeof(T) * size;
	byte_length = ((byte_length + 3) / 4) * 4;
	dst.data.resize(byte_offset + byte_length);
	memcpy(&dst.data[byte_offset], &src[0], byte_length);
}

// export the point-cloud to the given file
bool PointCloud::SaveGLTF(const String& fileName, bool bBinary) const
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);

	// create GLTF model
	tinygltf::Model gltfModel;
	tinygltf::Scene gltfScene;
	tinygltf::Mesh gltfMesh;
	tinygltf::Buffer gltfBuffer;
	gltfScene.name = "scene";
	gltfMesh.name = "pointcloud";

	tinygltf::Primitive gltfPrimitive;
	gltfPrimitive.mode = TINYGLTF_MODE_POINTS;

	// setup vertices
	{
		STATIC_ASSERT(3 * sizeof(Point::Type) == sizeof(Point)); // PointArr should be continuous
		const Box box(GetAABB());
		gltfPrimitive.attributes["POSITION"] = (int)gltfModel.accessors.size();
		tinygltf::Accessor vertexPositionAccessor;
		vertexPositionAccessor.name = "vertexPositionAccessor";
		vertexPositionAccessor.bufferView = (int)gltfModel.bufferViews.size();
		vertexPositionAccessor.type = TINYGLTF_TYPE_VEC3;
		vertexPositionAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
		vertexPositionAccessor.count = points.size();
		vertexPositionAccessor.minValues = {box.ptMin.x(), box.ptMin.y(), box.ptMin.z()};
		vertexPositionAccessor.maxValues = {box.ptMax.x(), box.ptMax.y(), box.ptMax.z()};
		gltfModel.accessors.emplace_back(std::move(vertexPositionAccessor));
		// setup vertices buffer
		tinygltf::BufferView vertexPositionBufferView;
		vertexPositionBufferView.name = "vertexPositionBufferView";
		vertexPositionBufferView.buffer = (int)gltfModel.buffers.size();
		ExtendBufferGLTF(points.data(), points.size(), gltfBuffer,
			vertexPositionBufferView.byteOffset, vertexPositionBufferView.byteLength);
		gltfModel.bufferViews.emplace_back(std::move(vertexPositionBufferView));
	}

	// setup colors
	if (!colors.empty()) {
		STATIC_ASSERT(3 * sizeof(Color::Type) == sizeof(Color)); // ColorArr should be continuous
		gltfPrimitive.attributes["COLOR_0"] = (int)gltfModel.accessors.size();
		tinygltf::Accessor vertexColorAccessor;
		vertexColorAccessor.name = "vertexColorAccessor";
		vertexColorAccessor.bufferView = (int)gltfModel.bufferViews.size();
		vertexColorAccessor.type = TINYGLTF_TYPE_VEC3;
		vertexColorAccessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
		vertexColorAccessor.normalized = true;
		vertexColorAccessor.count = colors.size();
		gltfModel.accessors.emplace_back(std::move(vertexColorAccessor));
		// setup colors buffer
		tinygltf::BufferView vertexColorBufferView;
		vertexColorBufferView.name = "vertexColorBufferView";
		vertexColorBufferView.buffer = (int)gltfModel.buffers.size();
		ExtendBufferGLTF(colors.data(), colors.size(), gltfBuffer,
			vertexColorBufferView.byteOffset, vertexColorBufferView.byteLength);
		// our colors are in BGR order, need to swizzle to RGB
		uint8_t* const pColorData = &gltfBuffer.data[vertexColorBufferView.byteOffset];
		FOREACH(i, colors)
			std::swap(pColorData[i * 3 + 0], pColorData[i * 3 + 2]);
		gltfModel.bufferViews.emplace_back(std::move(vertexColorBufferView));
	}

	// setup normals
	if (!normals.empty()) {
		STATIC_ASSERT(3 * sizeof(Normal::Type) == sizeof(Normal)); // NormalArr should be continuous
		gltfPrimitive.attributes["NORMAL"] = (int)gltfModel.accessors.size();
		tinygltf::Accessor vertexNormalAccessor;
		vertexNormalAccessor.name = "vertexNormalAccessor";
		vertexNormalAccessor.bufferView = (int)gltfModel.bufferViews.size();
		vertexNormalAccessor.type = TINYGLTF_TYPE_VEC3;
		vertexNormalAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
		vertexNormalAccessor.count = normals.size();
		gltfModel.accessors.emplace_back(std::move(vertexNormalAccessor));
		// setup normals buffer
		tinygltf::BufferView vertexNormalBufferView;
		vertexNormalBufferView.name = "vertexNormalBufferView";
		vertexNormalBufferView.buffer = (int)gltfModel.buffers.size();
		ExtendBufferGLTF(normals.data(), normals.size(), gltfBuffer,
			vertexNormalBufferView.byteOffset, vertexNormalBufferView.byteLength);
		gltfModel.bufferViews.emplace_back(std::move(vertexNormalBufferView));
	}

	gltfMesh.primitives.emplace_back(std::move(gltfPrimitive));
	gltfModel.meshes.emplace_back(std::move(gltfMesh));
	gltfModel.buffers.emplace_back(std::move(gltfBuffer));

	// setup scene
	tinygltf::Node gltfNode;
	gltfNode.name = "node";
	gltfNode.mesh = 0;
	gltfModel.nodes.emplace_back(std::move(gltfNode));
	gltfScene.nodes.push_back(0);
	gltfModel.scenes.emplace_back(std::move(gltfScene));
	gltfModel.defaultScene = 0;

	// save model
	tinygltf::TinyGLTF gltf;
	return gltf.WriteGltfSceneToFile(&gltfModel, fileName, false, false, !bBinary, bBinary);
}

// save the dense point-cloud as Potree 2.0 format
// outputs 3 files in the given directory: metadata.json, hierarchy.bin, octree.bin
bool PointCloud::SavePotree(const String& dirName) const
{
	if (IsEmpty())
		return false;
	TD_TIMER_STARTD();

	// ensure output directory exists (append separator if needed)
	String outputDir(dirName);
	if (!outputDir.empty() && outputDir.back() != PATH_SEPARATOR)
		outputDir += PATH_SEPARATOR;
	Util::ensureFolder(outputDir);

	// compute bounding box and make it cubic (Potree requirement)
	const Box aabb(GetAABB());
	const Eigen::Vector3d center(aabb.GetCenter().cast<double>());
	const Eigen::Vector3d aabbSize(aabb.GetSize().cast<double>());
	const double halfSize = aabbSize.maxCoeff() / 2.0;
	const Eigen::Vector3f cubeMin((float)(center.x() - halfSize), (float)(center.y() - halfSize), (float)(center.z() - halfSize));
	const Eigen::Vector3f cubeMax((float)(center.x() + halfSize), (float)(center.y() + halfSize), (float)(center.z() + halfSize));
	const AABB3f cubeAABB(cubeMin, cubeMax);
	const double cubeSize = halfSize * 2.0;

	// build LOD octree using grid-based subsampling
	typedef TOctreeLOD<PointArr, Point::Type, 3> OctreeLOD;
	OctreeLOD lodOctree;
	lodOctree.Insert(points, cubeAABB, GridSubsample<PointArr, Point::Type, 3>(128));
	DEBUG_EXTRA("Point-cloud Potree: LOD octree built with %zu nodes, depth %u",
		lodOctree.GetTotalNodes(), lodOctree.GetMaxDepth());

	// compute encoding parameters
	const double offsetX = cubeAABB.ptMin.x(), offsetY = cubeAABB.ptMin.y(), offsetZ = cubeAABB.ptMin.z();
	const double scale = cubeSize > 0 ? cubeSize / double(INT32_MAX) : 1.0;
	const double invScale = 1.0 / scale;
	const bool hasColors = !colors.empty();
	const bool hasNormals = !normals.empty();

	// compute per-point byte size
	const size_t bytesPerPointPosition = 3 * sizeof(int32_t); // 12 bytes
	const size_t bytesPerPointColor = hasColors ? 4 * sizeof(uint8_t) : 0; // 4 bytes (RGBA)
	const size_t bytesPerPointNormal = hasNormals ? 3 * sizeof(float) : 0; // 12 bytes
	const size_t bytesPerPoint = bytesPerPointPosition + bytesPerPointColor + bytesPerPointNormal;

	// write octree.bin and collect hierarchy records via BFS traversal
	struct NodeRecord {
		uint8_t type;
		uint8_t childMask;
		uint32_t numPoints;
		uint64_t byteOffset;
		uint64_t byteSize;
	};
	std::vector<NodeRecord> records;
	records.reserve(lodOctree.GetTotalNodes());

	const String octreeFile(outputDir + _T("octree.bin"));
	{
		File file(octreeFile, File::WRITE, File::CREATE | File::TRUNCATE);
		if (!file.isOpen()) {
			DEBUG("error: cannot create Potree octree file '%s'", octreeFile.c_str());
			return false;
		}

		// BFS traversal — write point data per node and record offsets
		const auto& lodIndices = lodOctree.GetIndexArr();
		uint64_t currentOffset = 0;
		lodOctree.TraverseBFS([&](const OctreeLOD::Node& node, const auto& /*center*/, auto /*radius*/) {
			NodeRecord rec;
			rec.type = node.IsLeaf() ? uint8_t(1) : uint8_t(0);
			rec.childMask = node.childMask;
			rec.numPoints = node.GetNumItems();
			rec.byteOffset = currentOffset;
			rec.byteSize = (uint64_t)node.GetNumItems() * bytesPerPoint;

			if (node.GetNumItems() > 0) {
				// write positions as int32
				for (IDX i = node.GetFirstItemIdx(); i < node.GetLastItemIdx(); ++i) {
					const Point& pt = points[lodIndices[i]];
					const int32_t encoded[3] = {
						(int32_t)ROUND2INT(((double)pt.x - offsetX) * invScale),
						(int32_t)ROUND2INT(((double)pt.y - offsetY) * invScale),
						(int32_t)ROUND2INT(((double)pt.z - offsetZ) * invScale)
					};
					file.write(encoded, sizeof(encoded));
				}
				// write colors as RGBA (BGR→RGB swizzle)
				if (hasColors) {
					for (IDX i = node.GetFirstItemIdx(); i < node.GetLastItemIdx(); ++i) {
						const Color& c = colors[lodIndices[i]];
						const uint8_t rgba[4] = {c[2], c[1], c[0], 255};
						file.write(rgba, sizeof(rgba));
					}
				}
				// write normals as float32
				if (hasNormals) {
					for (IDX i = node.GetFirstItemIdx(); i < node.GetLastItemIdx(); ++i) {
						const Normal& n = normals[lodIndices[i]];
						const float nf[3] = {n.x, n.y, n.z};
						file.write(nf, sizeof(nf));
					}
				}
			}

			currentOffset += rec.byteSize;
			records.push_back(rec);
		});
	}

	// write hierarchy.bin — 22 bytes per node, BFS order (same order as records)
	const String hierarchyFile(outputDir + _T("hierarchy.bin"));
	{
		File file(hierarchyFile, File::WRITE, File::CREATE | File::TRUNCATE);
		if (!file.isOpen()) {
			DEBUG("error: cannot create Potree hierarchy file '%s'", hierarchyFile.c_str());
			return false;
		}
		for (const NodeRecord& rec : records) {
			file.write(&rec.type, 1);
			file.write(&rec.childMask, 1);
			file.write(&rec.numPoints, 4);
			file.write(&rec.byteOffset, 8);
			file.write(&rec.byteSize, 8);
		}
	}

	// write metadata.json
	const String metadataFile(outputDir + _T("metadata.json"));
	{
		using json = nlohmann::json;
		json metadata;
		metadata["version"] = "2.0";
		metadata["name"] = "pointcloud";
		metadata["description"] = "";
		metadata["points"] = points.size();
		metadata["projection"] = "";

		// bounding box
		metadata["boundingBox"]["min"] = {(double)cubeAABB.ptMin.x(), (double)cubeAABB.ptMin.y(), (double)cubeAABB.ptMin.z()};
		metadata["boundingBox"]["max"] = {(double)cubeAABB.ptMax.x(), (double)cubeAABB.ptMax.y(), (double)cubeAABB.ptMax.z()};

		// encoding
		metadata["offset"] = {offsetX, offsetY, offsetZ};
		metadata["scale"] = {scale, scale, scale};
		metadata["spacing"] = (double)lodOctree.GetSpacing();

		// hierarchy
		metadata["hierarchy"]["firstChunkSize"] = records.size();
		metadata["hierarchy"]["stepSize"] = 4;
		metadata["hierarchy"]["depth"] = lodOctree.GetMaxDepth();

		// encoding: position range for int32
		const double maxEncoded = cubeSize * invScale;
		const json posMin = {0, 0, 0};
		const json posMax = {maxEncoded, maxEncoded, maxEncoded};

		// attributes
		json attributes = json::array();
		{
			// position (int32 x 3)
			json attr;
			attr["name"] = "position";
			attr["description"] = "";
			attr["size"] = 12;
			attr["numElements"] = 3;
			attr["elementSize"] = 4;
			attr["type"] = "int32";
			attr["min"] = posMin;
			attr["max"] = posMax;
			attributes.push_back(std::move(attr));
		}
		if (hasColors) {
			// rgba (uint8 x 4)
			json attr;
			attr["name"] = "rgba";
			attr["description"] = "";
			attr["size"] = 4;
			attr["numElements"] = 4;
			attr["elementSize"] = 1;
			attr["type"] = "uint8";
			attr["min"] = {0, 0, 0, 0};
			attr["max"] = {255, 255, 255, 255};
			attributes.push_back(std::move(attr));
		}
		if (hasNormals) {
			// normal (float32 x 3)
			json attr;
			attr["name"] = "NORMAL";
			attr["description"] = "";
			attr["size"] = 12;
			attr["numElements"] = 3;
			attr["elementSize"] = 4;
			attr["type"] = "float";
			attr["min"] = {-1.0, -1.0, -1.0};
			attr["max"] = {1.0, 1.0, 1.0};
			attributes.push_back(std::move(attr));
		}
		metadata["attributes"] = std::move(attributes);

		// write JSON file
		std::ofstream ofs(metadataFile);
		if (!ofs.is_open()) {
			DEBUG("error: cannot create Potree metadata file '%s'", metadataFile.c_str());
			return false;
		}
		ofs << metadata.dump(2);
	}

	DEBUG_EXTRA("Point-cloud Potree '%s' saved: %u points, %zu nodes, depth %u (%s)",
		dirName.c_str(), points.size(), records.size(), lodOctree.GetMaxDepth(), TD_TIMER_GET_FMT().c_str());
	return true;
} // SavePotree

// save the dense point-cloud having >=N views as PLY file
bool PointCloud::SaveNViews(const String& fileName, uint32_t minViews, bool bLegacyTypes, bool bBinary) const
{
	if (points.IsEmpty())
		return false;
	TD_TIMER_STARTD();

	// create PLY object
	ASSERT(!fileName.IsEmpty());
	Util::ensureFolder(fileName);
	using namespace PointCloudInternal;
	PLY ply;
	if (bLegacyTypes)
		ply.set_legacy_type_names();
	if (!ply.write(fileName, 1, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII, 64*1024))
		return false;

	BasicPLY::Vertex vertex;
	if (normals.IsEmpty()) {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 6, BasicPLY::Vertex::props);

		// export the array of 3D points
		FOREACH(i, points) {
			if (pointViews[i].size() < minViews)
				continue;
			// export the vertex position and color
			vertex.p = points[i];
			vertex.c = colors.empty() ? Pixel8U::WHITE : colors[i];
			ply.put_element(&vertex);
		}
	} else {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 9, BasicPLY::Vertex::props);

		// export the array of 3D points
		FOREACH(i, points) {
			if (pointViews[i].size() < minViews)
				continue;
			// export the vertex position, normal and color
			vertex.p = points[i];
			vertex.n = normals[i];
			vertex.c = colors.empty() ? Pixel8U::WHITE : colors[i];
			ply.put_element(&vertex);
		}
	}
	const int numPoints(ply.get_current_element_count());

	// write the header
	if (!ply.header_complete())
		return false;

	DEBUG_EXTRA("Point-cloud saved: %u points with at least %u views each (%s)", numPoints, minViews, TD_TIMER_GET_FMT().c_str());
	return true;
} // SaveNViews

// save the dense point-cloud + scale as PLY file
bool PointCloud::SaveWithScale(const String& fileName, const ImageArr& images, float scaleMult, bool bLegacyTypes, bool bBinary) const
{
	if (points.empty())
		return false;

	TD_TIMER_STARTD();

	// create PLY object
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	using namespace PointCloudInternal;
	PLY ply;
	if (bLegacyTypes)
		ply.set_legacy_type_names();
	if (!ply.write(fileName, 1, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII))
		return false;

	// export the array of 3D points
	BasicPLY::Vertex::InitSaveProps(ply, (int)points.size(), !colors.empty(), !normals.empty(), false, false, false, true, true);
	if (!ply.header_complete())
		return false;
	BasicPLY::Vertex vertex;
	FOREACH(i, points) {
		// export the vertex position, normal and scale
		vertex.p = points[i];
		if (!colors.empty())
			vertex.c = colors[i];
		if (!normals.empty())
			vertex.n = normals[i];
		#if 0
		// one sample per view
		vertex.confidence = 1;
		for (IIndex idxView: pointViews[i]) {
			const float scale((float)images[idxView].camera.GetFootprintWorld(Cast<REAL>(vertex.p)));
			ASSERT(scale > 0);
			vertex.scale = scale*scaleMult;
			ply.put_element(&vertex);
		}
		#else
		// one sample per point
		vertex.scale = FLT_MAX;
		if (pointWeights.empty()) {
			vertex.confidence = (float)pointViews[i].size();
			for (IIndex idxView: pointViews[i]) {
				const float scale((float)images[idxView].camera.GetFootprintWorld(Cast<REAL>(vertex.p)));
				ASSERT(scale > 0);
				if (vertex.scale > scale)
					vertex.scale = scale;
			}
		} else {
			vertex.confidence = 0;
			float scaleWeightBest = FLT_MAX;
			FOREACH(j, pointViews[i]) {
				const IIndex idxView = pointViews[i][j];
				const float scale((float)images[idxView].camera.GetFootprintWorld(Cast<REAL>(vertex.p)));
				ASSERT(scale > 0);
				const float conf(pointWeights[i][j]);
				const float scaleWeight(scale/conf);
				if (scaleWeightBest > scaleWeight) {
					scaleWeightBest = scaleWeight;
					vertex.scale = scale;
					vertex.confidence = conf;
				}
			}
		}
		ASSERT(vertex.scale != FLT_MAX);
		vertex.scale *= scaleMult;
		ply.put_element(&vertex);
		#endif
	}
	ASSERT(ply.get_current_element_count() == (int)points.size());

	DEBUG_EXTRA("Point-cloud saved: %u points with scale (%s)", points.size(), TD_TIMER_GET_FMT().c_str());
	return true;
} // SaveWithScale
/*----------------------------------------------------------------*/


// print various statistics about the point-cloud
void PointCloud::PrintStatistics(const Image* pImages, const OBB3f* pObb) const
{
	String strPoints;
	if (pObb && pObb->IsValid()) {
		// print points distribution
		size_t nInsidePoints(0);
		MeanStdMinMax<double> accInside, accOutside;
		FOREACH(idx, points) {
			const bool bInsideROI(pObb->Intersects(points[idx]));
			if (bInsideROI)
				++nInsidePoints;
			if (!pointViews.empty()) {
				if (bInsideROI)
					accInside.Update(pointViews[idx].size());
				else
					accOutside.Update(pointViews[idx].size());
			}
		}
		strPoints = String::FormatString(
			"\n - points info:"
			"\n\t%u points inside ROI (%.2f%%)",
			nInsidePoints, 100.0*nInsidePoints/GetSize()
		);
		if (!pointViews.empty()) {
			strPoints += String::FormatString(
				"\n\t inside ROI track length: %g min / %g mean (%g std) / %g max"
				"\n\toutside ROI track length: %g min / %g mean (%g std) / %g max",
				accInside.minVal, accInside.GetMean(), accInside.GetStdDev(), accInside.maxVal,
				accOutside.minVal, accOutside.GetMean(), accOutside.GetStdDev(), accOutside.maxVal
			);
		}
	}
	if (pointViews.empty() && normals.empty() && pointWeights.empty() && colors.empty())
		return;
	String strViews;
	if (!pointViews.empty()) {
		// print views distribution
		size_t nViews(0);
		size_t nPoints1m(0), nPoints2(0), nPoints3(0), nPoints4p(0);
		size_t nPointsOpposedViews(0);
		MeanStdMinMax<double> acc;
		FOREACH(idx, points) {
			const PointCloud::ViewArr& views = pointViews[idx];
			nViews += views.size();
			switch (views.size()) {
			case 0:
			case 1:
				++nPoints1m;
				break;
			case 2:
				++nPoints2;
				break;
			case 3:
				++nPoints3;
				break;
			default:
				++nPoints4p;
			}
			acc.Update(views.size());
		}
		strViews = String::FormatString(
			"\n - visibility info (%u views - %.2f views/point)%s:"
			"\n\t% 9u points with 1- views (%.2f%%)"
			"\n\t% 9u points with 2  views (%.2f%%)"
			"\n\t% 9u points with 3  views (%.2f%%)"
			"\n\t% 9u points with 4+ views (%.2f%%)"
			"\n\t%g min / %g mean (%g std) / %g max",
			nViews, (REAL)nViews/GetSize(),
			nPointsOpposedViews ? String::FormatString(" (%u (%.2f%%) points with opposed views)", nPointsOpposedViews, 100.f*nPointsOpposedViews/GetSize()).c_str() : "",
			nPoints1m, 100.f*nPoints1m/GetSize(), nPoints2, 100.f*nPoints2/GetSize(), nPoints3, 100.f*nPoints3/GetSize(), nPoints4p, 100.f*nPoints4p/GetSize(),
			acc.minVal, acc.GetMean(), acc.GetStdDev(), acc.maxVal
		);
	}
	String strNormals;
	if (!normals.empty()) {
		if (!pointViews.empty() && pImages != NULL) {
			// print normal/views angle distribution
			size_t nViews(0);
			size_t nPointsm(0), nPoints3(0), nPoints10(0), nPoints25(0), nPoints40(0), nPoints60(0), nPoints90p(0);
			const REAL thCosAngle3(COS(D2R(3.0)));
			const REAL thCosAngle10(COS(D2R(10.0)));
			const REAL thCosAngle25(COS(D2R(25.0)));
			const REAL thCosAngle40(COS(D2R(40.0)));
			const REAL thCosAngle60(COS(D2R(60.0)));
			const REAL thCosAngle90(COS(D2R(90.0)));
			FOREACH(idx, points) {
				const PointCloud::Point& X = points[idx];
				const PointCloud::Normal& N = normals[idx];
				const PointCloud::ViewArr& views = pointViews[idx];
				nViews += views.size();
				for (IIndex idxImage: views) {
					const Point3f X2Cam(Cast<float>(pImages[idxImage].camera.C)-X);
					const REAL cosAngle(ComputeAngle(X2Cam.ptr(), N.ptr()));
					if (cosAngle <= thCosAngle90)
						++nPoints90p;
					else if (cosAngle <= thCosAngle60)
						++nPoints60;
					else if (cosAngle <= thCosAngle40)
						++nPoints40;
					else if (cosAngle <= thCosAngle25)
						++nPoints25;
					else if (cosAngle <= thCosAngle10)
						++nPoints10;
					else if (cosAngle <= thCosAngle3)
						++nPoints3;
					else
						++nPointsm;
				}
			}
			strNormals = String::FormatString(
				"\n - normals visibility info:"
				"\n\t% 9u points with 3- degrees (%.2f%%)"
				"\n\t% 9u points with 10 degrees (%.2f%%)"
				"\n\t% 9u points with 25 degrees (%.2f%%)"
				"\n\t% 9u points with 40 degrees (%.2f%%)"
				"\n\t% 9u points with 60 degrees (%.2f%%)"
				"\n\t% 9u points with 90+ degrees (%.2f%%)",
				nPointsm, 100.f*nPointsm/nViews, nPoints3, 100.f*nPoints3/nViews, nPoints10, 100.f*nPoints10/nViews,
				nPoints40, 100.f*nPoints40/nViews, nPoints60, 100.f*nPoints60/nViews, nPoints90p, 100.f*nPoints90p/nViews
			);
		} else {
			strNormals = "\n - normals info";
		}
	}
	String strWeights;
	if (!pointWeights.empty()) {
		// print weights statistics
		MeanStdMinMax<double> acc;
		for (const PointCloud::WeightArr& weights: pointWeights) {
			float avgWeight(0);
			for (PointCloud::Weight w: weights)
				avgWeight += w;
			acc.Update(avgWeight/weights.size());
		}
		strWeights = String::FormatString(
			"\n - weights info:"
			"\n\t%g min / %g mean (%g std) / %g max",
			acc.minVal, acc.GetMean(), acc.GetStdDev(), acc.maxVal
		);
	}
	String strColors;
	if (!colors.empty()) {
		// print colors statistics
		strColors = "\n - colors";
	}
	VERBOSE("Point-cloud composed of %u points with:%s%s%s%s",
		GetSize(),
		strPoints.c_str(),
		strViews.c_str(),
		strNormals.c_str(),
		strWeights.c_str(),
		strColors.c_str()
	);
} // PrintStatistics
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
