/*
* Mesh.h
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

#ifndef _MVS_MESH_H_
#define _MVS_MESH_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Platform.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a mesh represented by a list vertices and triangles (faces)
class Mesh
{
public:
	typedef TPoint3<float> Vertex;
	typedef uint32_t Index;
	typedef TPoint3<Index> Face;

	typedef CLISTDEF0(Vertex) VertexArr;
	typedef CLISTDEF0(Face) FaceArr;

public:
	VertexArr vertices;
	FaceArr faces;

public:
	inline Mesh() {}

	void Release();

	// file IO
	bool Load(const String& fileName);
	bool Save(const String& fileName, bool bBinary=true) const;
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_MESH_H_
