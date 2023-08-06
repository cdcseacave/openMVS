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

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

PointCloud& MVS::PointCloud::Swap(PointCloud& rhs)
{
	points.Swap(rhs.points);
	pointViews.Swap(rhs.pointViews);
	pointWeights.Swap(rhs.pointWeights);
	normals.Swap(rhs.normals);
	colors.Swap(rhs.colors);
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
}
/*----------------------------------------------------------------*/


void PointCloud::RemovePoint(IDX idx)
{
	ASSERT(pointViews.IsEmpty() || pointViews.GetSize() == points.GetSize());
	if (!pointViews.IsEmpty())
		pointViews.RemoveAt(idx);
	ASSERT(pointWeights.IsEmpty() || pointWeights.GetSize() == points.GetSize());
	if (!pointWeights.IsEmpty())
		pointWeights.RemoveAt(idx);
	ASSERT(normals.IsEmpty() || normals.GetSize() == points.GetSize());
	if (!normals.IsEmpty())
		normals.RemoveAt(idx);
	ASSERT(colors.IsEmpty() || colors.GetSize() == points.GetSize());
	if (!colors.IsEmpty())
		colors.RemoveAt(idx);
	points.RemoveAt(idx);
}
void PointCloud::RemovePointsOutside(const OBB3f& obb) {
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
PointCloud::Box PointCloud::GetAABB(const Box& bound) const
{
	Box box(true);
	for (const Point& X: points)
		if (bound.Intersects(X))
			box.InsertFull(X);
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
		float confidence;
		float scale;
		static void InitLoadProps(PLY& ply, int elem_count,
			PointCloud::PointArr& points, PointCloud::ColorArr& colors, PointCloud::NormalArr& normals, PointCloud::PointViewArr& views, PointCloud::PointWeightArr& weights)
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
				}
			}
		}
		static void InitSaveProps(PLY& ply, int elem_count,
			bool bColors, bool bNormals, bool bViews, bool bWeights, bool bConfidence=false, bool bScale=false)
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
			if (bConfidence)
				ply.describe_property(elem_names[0], props[11]);
			if (bScale)
				ply.describe_property(elem_names[0], props[12]);
			if (elem_count)
				ply.element_count(elem_names[0], elem_count);
		}
		static const PLY::PlyProperty props[16];
	};
	const PLY::PlyProperty Vertex::props[16] = {
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
		{"confidence",    PLY::Float32, PLY::Float32, offsetof(Vertex,confidence), 0, 0, 0, 0},
		{"value",         PLY::Float32, PLY::Float32, offsetof(Vertex,scale), 0, 0, 0, 0},
		// duplicates
		{"diffuse_red",   PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.r), 0, 0, 0, 0},
		{"diffuse_green", PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.g), 0, 0, 0, 0},
		{"diffuse_blue",  PLY::Uint8,   PLY::Uint8,   offsetof(Vertex,c.b), 0, 0, 0, 0}
	};
} // namespace BasicPLY
} // namespace PointCloudInternal

// load the dense point cloud from a PLY file
bool PointCloud::Load(const String& fileName)
{
	TD_TIMER_STARTD();

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
			BasicPLY::Vertex::InitLoadProps(ply, elem_count, points, colors, normals, pointViews, pointWeights);
			BasicPLY::Vertex vertex;
			for (int v=0; v<elem_count; ++v) {
				ply.get_element(&vertex);
				points[v] = vertex.p;
				if (!colors.empty())
					colors[v] = vertex.c;
				if (!normals.empty())
					normals[v] = vertex.n;
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

	DEBUG_EXTRA("Point-cloud '%s' loaded: %u points (%s)", Util::getFileNameExt(fileName).c_str(), points.size(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Load

// save the dense point cloud as PLY file
bool PointCloud::Save(const String& fileName, bool bViews, bool bLegacyTypes, bool bBinary) const
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

	// write the header
	BasicPLY::Vertex::InitSaveProps(ply, (int)points.size(), !colors.empty(), !normals.empty(),
		bViews && !pointViews.empty(), bViews && !pointWeights.empty());
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

	DEBUG_EXTRA("Point-cloud '%s' saved: %u points (%s)", Util::getFileNameExt(fileName).c_str(), points.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Save

// save the dense point cloud having >=N views as PLY file
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

// save the dense point cloud + scale as PLY file
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
	BasicPLY::Vertex::InitSaveProps(ply, (int)points.size(), !colors.empty(), !normals.empty(), false, false, true, true);
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


// print various statistics about the point cloud
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
			const REAL thCosAngle3(COS(D2R(3.f)));
			const REAL thCosAngle10(COS(D2R(10.f)));
			const REAL thCosAngle25(COS(D2R(25.f)));
			const REAL thCosAngle40(COS(D2R(40.f)));
			const REAL thCosAngle60(COS(D2R(60.f)));
			const REAL thCosAngle90(COS(D2R(90.f)));
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
