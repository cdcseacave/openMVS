/*
* PointCloud.h
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

#ifndef _MVS_POINTCLOUD_H_
#define _MVS_POINTCLOUD_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a point-cloud containing the points with the corresponding views
// and optionally weights, normals and colors
// (same size as the number of points or zero)
class MVS_API PointCloud
{
public:
	typedef IDX Index;

	typedef TPoint3<float> Point;
	typedef SEACAVE::cList<Point,const Point&,2,8192> PointArr;

	typedef uint32_t View;
	typedef SEACAVE::cList<View,const View,0,4,uint32_t> ViewArr;
	typedef SEACAVE::cList<ViewArr> PointViewArr;

	typedef float Weight;
	typedef SEACAVE::cList<Weight,const Weight,0,4,uint32_t> WeightArr;
	typedef SEACAVE::cList<WeightArr> PointWeightArr;

	typedef TPoint3<float> Normal;
	typedef CLISTDEF0(Normal) NormalArr;

	typedef Pixel8U Color;
	typedef CLISTDEF0(Color) ColorArr;

	typedef AABB3f Box;

	typedef TOctree<PointArr,Point::Type,3> Octree;

public:
	PointArr points;
	PointViewArr pointViews; // array of views for each point (ordered increasing)
	PointWeightArr pointWeights;
	NormalArr normals;
	ColorArr colors;

public:
	inline PointCloud() {}

	void Release();

	inline bool IsEmpty() const { ASSERT(points.GetSize() == pointViews.GetSize() || pointViews.IsEmpty()); return points.IsEmpty(); }
	inline bool IsValid() const { ASSERT(points.GetSize() == pointViews.GetSize() || pointViews.IsEmpty()); return !pointViews.IsEmpty(); }
	inline size_t GetSize() const { ASSERT(points.GetSize() == pointViews.GetSize() || pointViews.IsEmpty()); return points.GetSize(); }

	void RemovePoint(IDX);
	void RemovePointsOutside(const OBB3f&);

	Box GetAABB() const;
	Box GetAABB(const Box& bound) const;
	Box GetAABB(unsigned minViews) const;
	Point GetCenter() const;

	bool Load(const String& fileName);
	bool Save(const String& fileName, bool bLegacyTypes=false) const;
	bool SaveNViews(const String& fileName, uint32_t minViews, bool bLegacyTypes=false) const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & points;
		ar & pointViews;
		ar & pointWeights;
		ar & normals;
		ar & colors;
	}
	#endif
};
/*----------------------------------------------------------------*/


struct IndexDist {
	IDX idx;
	REAL dist;

	inline IndexDist() : dist(REAL(FLT_MAX)) {}
	inline bool IsValid() const { return dist < REAL(FLT_MAX); }
};

struct IntersectRayPoints {
	typedef PointCloud::Octree Octree;
	typedef typename Octree::IDX_TYPE IDX;
	typedef TCone<REAL, 3> Cone3;
	typedef TConeIntersect<REAL, 3> Cone3Intersect;

	const PointCloud& pointcloud;
	const Cone3 cone;
	const Cone3Intersect coneIntersect;
	const unsigned minViews;
	IndexDist pick;

	IntersectRayPoints(const Octree& octree, const Ray3& _ray, const PointCloud& _pointcloud, unsigned _minViews)
		: pointcloud(_pointcloud), cone(_ray, D2R(REAL(0.5))), coneIntersect(cone), minViews(_minViews)
	{
		octree.Collect(*this, *this);
	}

	inline bool Intersects(const typename Octree::POINT_TYPE& center, typename Octree::Type radius) const {
		return coneIntersect(Sphere3(center.cast<REAL>(), REAL(radius) * SQRT_3));
	}

	void operator () (const IDX* idices, IDX size) {
		// test ray-point intersection and keep the closest
		FOREACHRAWPTR(pIdx, idices, size) {
			const PointCloud::Index idx(*pIdx);
			if (!pointcloud.pointViews.IsEmpty() && pointcloud.pointViews[idx].size() < minViews)
				continue;
			const PointCloud::Point& X = pointcloud.points[idx];
			REAL dist;
			if (coneIntersect.Classify(Cast<REAL>(X), dist) == VISIBLE) {
				ASSERT(dist >= 0);
				if (pick.dist > dist) {
					pick.dist = dist;
					pick.idx = idx;
				}
			}
		}
	}
};
/*----------------------------------------------------------------*/


typedef MVS_API float Depth;
typedef MVS_API Point3f Normal;
typedef MVS_API TImage<Depth> DepthMap;
typedef MVS_API TImage<Normal> NormalMap;
typedef MVS_API TImage<float> ConfidenceMap;
typedef MVS_API SEACAVE::cList<Depth,Depth,0> DepthArr;
typedef MVS_API SEACAVE::cList<DepthMap,const DepthMap&,2> DepthMapArr;
typedef MVS_API SEACAVE::cList<NormalMap,const NormalMap&,2> NormalMapArr;
typedef MVS_API SEACAVE::cList<ConfidenceMap,const ConfidenceMap&,2> ConfidenceMapArr;
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_POINTCLOUD_H_
