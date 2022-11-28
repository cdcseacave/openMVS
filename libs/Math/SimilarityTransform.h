/*
* SimilarityTransform.h
*
* Copyright (c) 2014-2022 SEACAVE
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

#ifndef _SEACAVE_SIMILARITY_TRANSFORM_H_
#define _SEACAVE_SIMILARITY_TRANSFORM_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

// find the similarity transform that best aligns the given two sets of corresponding 3D points
bool SimilarityTransform(const CLISTDEF0(Point3)& points, const CLISTDEF0(Point3)& pointsRef, Matrix4x4& transform);

// decompose similarity transform into rotation, translation and scale
void DecomposeSimilarityTransform(const Matrix4x4& transform, Matrix3x3& R, Point3& t, REAL& s);
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // _SEACAVE_SIMILARITY_TRANSFORM_H_
