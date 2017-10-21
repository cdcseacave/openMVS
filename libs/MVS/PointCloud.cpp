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

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

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
	if (pointViews.IsEmpty())
		return GetAABB();
	Box box(true);
	FOREACH(idx, points)
		if (pointViews[idx].GetSize() >= minViews)
			box.InsertFull(points[idx]);
	return box;
}
/*----------------------------------------------------------------*/


// define a PLY file format composed only of vertices
namespace BasicPLY {
	typedef PointCloud::Point Point;
	typedef PointCloud::Color Color;
	typedef PointCloud::Normal Normal;
	// list of property information for a vertex
	struct PointColNormal {
		Point p;
		Color c;
		Normal n;
	};
	static const PLY::PlyProperty vert_props[] = {
		{"x",     PLY::Float32, PLY::Float32, offsetof(PointColNormal,p.x), 0, 0, 0, 0},
		{"y",     PLY::Float32, PLY::Float32, offsetof(PointColNormal,p.y), 0, 0, 0, 0},
		{"z",     PLY::Float32, PLY::Float32, offsetof(PointColNormal,p.z), 0, 0, 0, 0},
		{"red",   PLY::Uint8,   PLY::Uint8,   offsetof(PointColNormal,c.r), 0, 0, 0, 0},
		{"green", PLY::Uint8,   PLY::Uint8,   offsetof(PointColNormal,c.g), 0, 0, 0, 0},
		{"blue",  PLY::Uint8,   PLY::Uint8,   offsetof(PointColNormal,c.b), 0, 0, 0, 0},
		{"nx",    PLY::Float32, PLY::Float32, offsetof(PointColNormal,n.x), 0, 0, 0, 0},
		{"ny",    PLY::Float32, PLY::Float32, offsetof(PointColNormal,n.y), 0, 0, 0, 0},
		{"nz",    PLY::Float32, PLY::Float32, offsetof(PointColNormal,n.z), 0, 0, 0, 0}
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex"
	};
} // namespace BasicPLY

// load the dense point cloud from a PLY file
bool PointCloud::Load(const String& fileName)
{
	TD_TIMER_STARTD();

	ASSERT(!fileName.IsEmpty());
	Release();

	// open PLY file and read header
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}

	// read PLY body
	BasicPLY::PointColNormal vertex;
	for (int i = 0; i < (int)ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			PLY::PlyElement* elm = ply.find_element(elem_name);
			const size_t nMaxProps(SizeOfArray(BasicPLY::vert_props));
			for (size_t p=0; p<nMaxProps; ++p) {
				if (ply.find_property(elm, BasicPLY::vert_props[p].name.c_str()) < 0)
					continue;
				ply.setup_property(BasicPLY::vert_props[p]);
				switch (p) {
				case 0: points.Resize((IDX)elem_count); break;
				case 3: colors.Resize((IDX)elem_count); break;
				case 6: normals.Resize((IDX)elem_count); break;
				}
			}
			for (int v=0; v<elem_count; ++v) {
				ply.get_element(&vertex);
				points[v] = vertex.p;
				if (!colors.IsEmpty())
					colors[v] = vertex.c;
				if (!normals.IsEmpty())
					normals[v] = vertex.n;
			}
		} else {
			ply.get_other_element();
		}
	}
	if (points.IsEmpty()) {
		DEBUG_EXTRA("error: invalid point-cloud");
		return false;
	}

	DEBUG_EXTRA("Point-cloud loaded: %u points (%s)", points.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Load

// save the dense point cloud as PLY file
bool PointCloud::Save(const String& fileName) const
{
	if (points.IsEmpty())
		return false;
	TD_TIMER_STARTD();

	// create PLY object
	ASSERT(!fileName.IsEmpty());
	Util::ensureFolder(fileName);
	PLY ply;
	if (!ply.write(fileName, 1, BasicPLY::elem_names, PLY::BINARY_LE, 64*1024))
		return false;

	if (normals.IsEmpty()) {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 6, BasicPLY::vert_props);

		// write the header
		ply.element_count(BasicPLY::elem_names[0], (int)points.GetSize());
		if (!ply.header_complete())
			return false;

		// export the array of 3D points
		BasicPLY::PointColNormal vertex;
		FOREACH(i, points) {
			// export the vertex position, normal and color
			vertex.p = points[i];
			vertex.c = (!colors.IsEmpty() ? colors[i] : Color::WHITE);
			ply.put_element(&vertex);
		}
	} else {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 9, BasicPLY::vert_props);

		// write the header
		ply.element_count(BasicPLY::elem_names[0], (int)points.GetSize());
		if (!ply.header_complete())
			return false;

		// export the array of 3D points
		BasicPLY::PointColNormal vertex;
		FOREACH(i, points) {
			// export the vertex position, normal and color
			vertex.p = points[i];
			vertex.n = normals[i];
			vertex.c = (!colors.IsEmpty() ? colors[i] : Color::WHITE);
			ply.put_element(&vertex);
		}
	}

	DEBUG_EXTRA("Point-cloud saved: %u points (%s)", points.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Save
/*----------------------------------------------------------------*/
