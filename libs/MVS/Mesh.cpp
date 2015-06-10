/*
* Mesh.cpp
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
#include "Mesh.h"
#include "../Common/PLY.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// free all memory
void Mesh::Release()
{
	vertices.Empty();
	faces.Empty();
} // Release
/*----------------------------------------------------------------*/

// define a PLY file format composed only of vertices and triangles
namespace BasicPLY {
	// list of property information for a vertex
	static const PLY::PlyProperty vert_props[] = {
		{"x", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,x), 0, 0, 0, 0},
		{"y", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,y), 0, 0, 0, 0},
		{"z", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,z), 0, 0, 0, 0}
	};
	// list of property information for a face
	struct Facet {
		uint8_t num;
		Mesh::Face* pFace;
	};
	static const PLY::PlyProperty face_props[] = {
		{"vertex_indices", PLY::Uint32, PLY::Uint32, offsetof(Facet,pFace), 1, PLY::Uint8, PLY::Uint8, offsetof(Facet,num)}
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex",
		"face"
	};
} // namespace BasicPLY

// import the mesh as a PLY file
bool Mesh::Load(const String& fileName)
{
	ASSERT(!fileName.IsEmpty());
	Release();

	// open PLY file and read header
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}
	for (int i = 0; i < ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			vertices.Resize(elem_count);
		} else
			if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
				faces.Resize(elem_count);
			}
	}
	if (vertices.IsEmpty() || faces.IsEmpty()) {
		DEBUG_EXTRA("error: invalid mesh file");
		return false;
	}

	// read PLY body
	BasicPLY::Facet facet;
	for (int i = 0; i < ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			ASSERT(vertices.GetSize() == elem_count);
			ply.setup_property(BasicPLY::vert_props[0]);
			ply.setup_property(BasicPLY::vert_props[1]);
			ply.setup_property(BasicPLY::vert_props[2]);
			FOREACHPTR(pVert, vertices)
				ply.get_element(pVert);
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			ASSERT(faces.GetSize() == elem_count);
			ply.setup_property(BasicPLY::face_props[0]);
			FOREACHPTR(pFace, faces) {
				facet.pFace = pFace;
				ply.get_element(&facet);
				if (facet.num != 3) {
					DEBUG_EXTRA("error: unsupported mesh file (face not triangle)");
					return false;
				}
			}
		} else {
			ply.get_other_element();
		}
	}
	return true;
} // Load
/*----------------------------------------------------------------*/

// export the mesh as a PLY file
bool Mesh::Save(const String& fileName, bool bBinary) const
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureDirectory(fileName);

	// create PLY object
	const size_t bufferSize(vertices.GetSize()*(4*3/*pos*/+2/*eol*/) + faces.GetSize()*(1*1/*len*/+4*3/*idx*/+2/*eol*/) + 2048/*extra size*/);
	PLY ply;
	if (!ply.write(fileName, 2, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII, bufferSize)) {
		DEBUG_EXTRA("error: can not create the mesh file");
		return false;
	}

	// describe what properties go into the vertex elements
	ply.describe_property(BasicPLY::elem_names[0], 3, BasicPLY::vert_props);

	// export the array of vertices
	FOREACHPTR(pVert, vertices)
		ply.put_element(pVert);
	if (ply.get_current_element_count() == 0)
		return false;

	// describe what properties go into the vertex elements
	ply.describe_property(BasicPLY::elem_names[1], 1, BasicPLY::face_props);

	// export the array of faces
	BasicPLY::Facet facet; facet.num = 3;
	FOREACHPTR(pFace, faces) {
		facet.pFace = pFace;
		ply.put_element(&facet);
	}
	if (ply.get_current_element_count() == 0)
		return false;

	// write to file
	if (!ply.header_complete())
		return false;
	return true;
} // Save
/*----------------------------------------------------------------*/
