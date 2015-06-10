/*
* PointCloud.h
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

#ifndef _MVS_POINTCLOUD_H_
#define _MVS_POINTCLOUD_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a point-cloud containing the points with the corresponding views
// and optionally normals, colors and wights
// (same size as the number of points or zero)
class PointCloud
{
public:
	typedef TPoint3<float> Position;
	typedef SEACAVE::cList<uint32_t,const uint32_t,0,4,uint32_t> ViewArr;
	struct Point {
		Position X;
		ViewArr views;
	};
	typedef TPoint3<float> Normal;
	typedef Pixel8U Color;
	typedef float Weight;

	typedef SEACAVE::cList<Point,const Point&,2,8192> PointArr;
	typedef CLISTDEF0(Normal) NormalArr;
	typedef CLISTDEF0(Color) ColorArr;
	typedef CLISTDEF0(Weight) WeightArr;

public:
	PointArr points;
	NormalArr normals;
	ColorArr colors;
	WeightArr weights;

public:
	inline PointCloud() {}

	void Release();

	inline bool IsEmpty() const { return points.IsEmpty(); }
	inline size_t GetSize() const { return points.GetSize(); }
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_POINTCLOUD_H_
