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

// save the dense point cloud as PLY file
bool PointCloud::Save(const String& fileName)
{
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex"
	};

	if (points.IsEmpty())
		return false;

	// create PLY object
	ASSERT(!fileName.IsEmpty());
	Util::ensureDirectory(fileName);
	PLY ply;
	if (!ply.write(fileName, 1, elem_names, PLY::BINARY_LE, 64*1024))
		return false;

	if (normals.IsEmpty()) {
		// vertex definition
		struct Vertex {
			float x, y, z;
			uint8_t r, g, b;
		};
		// list of property information for a vertex
		static PLY::PlyProperty vert_props[] = {
			{"x", PLY::Float32, PLY::Float32, offsetof(Vertex,x), 0, 0, 0, 0},
			{"y", PLY::Float32, PLY::Float32, offsetof(Vertex,y), 0, 0, 0, 0},
			{"z", PLY::Float32, PLY::Float32, offsetof(Vertex,z), 0, 0, 0, 0},
			{"red", PLY::Uint8, PLY::Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
			{"green", PLY::Uint8, PLY::Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
			{"blue", PLY::Uint8, PLY::Uint8, offsetof(Vertex,b), 0, 0, 0, 0},
		};

		// describe what properties go into the vertex elements
		ply.describe_property("vertex", 6, vert_props);

		// write the header
		ply.element_count("vertex", (int)points.GetSize());
		if (!ply.header_complete())
			return false;

		// export the array of 3D points
		Vertex vertex;
		FOREACH(i, points) {
			// export the vertex position, normal and color
			const Point& point = points[i];
			vertex.x = point.x; vertex.y = point.y; vertex.z = point.z;
			if (!colors.IsEmpty()) {
				const Color& color = colors[i];
				vertex.r = color.r; vertex.g = color.g; vertex.b = color.b;
			} else {
				vertex.r = 255; vertex.g = 255; vertex.b = 255;
			}
			ply.put_element(&vertex);
		}
	} else {
		// vertex definition
		struct Vertex {
			float x, y, z;
			float nx, ny, nz;
			uint8_t r, g, b;
		};
		// list of property information for a vertex
		static PLY::PlyProperty vert_props[] = {
			{"x", PLY::Float32, PLY::Float32, offsetof(Vertex,x), 0, 0, 0, 0},
			{"y", PLY::Float32, PLY::Float32, offsetof(Vertex,y), 0, 0, 0, 0},
			{"z", PLY::Float32, PLY::Float32, offsetof(Vertex,z), 0, 0, 0, 0},
			{"nx", PLY::Float32, PLY::Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
			{"ny", PLY::Float32, PLY::Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
			{"nz", PLY::Float32, PLY::Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
			{"red", PLY::Uint8, PLY::Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
			{"green", PLY::Uint8, PLY::Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
			{"blue", PLY::Uint8, PLY::Uint8, offsetof(Vertex,b), 0, 0, 0, 0},
		};

		// describe what properties go into the vertex elements
		ply.describe_property("vertex", 9, vert_props);

		// write the header
		ply.element_count("vertex", (int)points.GetSize());
		if (!ply.header_complete())
			return false;

		// export the array of 3D points
		Vertex vertex;
		FOREACH(i, points) {
			// export the vertex position, normal and color
			const Point& point = points[i];
			vertex.x = point.x; vertex.y = point.y; vertex.z = point.z;
			const Normal& normal = normals[i];
			vertex.nx = normal.x; vertex.ny = normal.y; vertex.nz = normal.z;
			if (!colors.IsEmpty()) {
				const Color& color = colors[i];
				vertex.r = color.r; vertex.g = color.g; vertex.b = color.b;
			} else {
				vertex.r = 255; vertex.g = 255; vertex.b = 255;
			}
			ply.put_element(&vertex);
		}
	}

	return true;
} // Save
/*----------------------------------------------------------------*/
