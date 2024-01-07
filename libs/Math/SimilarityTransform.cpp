/*
* SimilarityTransform.cpp
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

#include "Common.h"
#include "SimilarityTransform.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// find the similarity transform that best aligns the given two sets of corresponding 3D points
bool SEACAVE::SimilarityTransform(const CLISTDEF0(Point3)& points, const CLISTDEF0(Point3)& pointsRef, Matrix4x4& transform)
{
	ASSERT(points.size() == pointsRef.size());
	typedef Eigen::Matrix<REAL,3,Eigen::Dynamic> PointsVec;
	PointsVec p(3, points.size());
	PointsVec pRef(3, pointsRef.size());
	FOREACH(i, points) {
		p.col(i) = static_cast<const Point3::EVec&>(points[i]);
		pRef.col(i) = static_cast<const Point3::EVec&>(pointsRef[i]);
	}
	transform = Eigen::umeyama(p, pRef);
	return true;
} // SimilarityTransform
/*----------------------------------------------------------------*/

void SEACAVE::DecomposeSimilarityTransform(const Matrix4x4& transform, Matrix3x3& R, Point3& t, REAL& s)
{
	const Eigen::Transform<REAL,3,Eigen::Affine,Eigen::RowMajor> T(static_cast<const Matrix4x4::EMat&>(transform));
	Eigen::Matrix<REAL,3,3> rotation, scaling;
	T.computeRotationScaling(&rotation, &scaling);
	R = rotation;
	t = T.translation();
	s = scaling.diagonal().mean();
} // DecomposeSimilarityTransform
/*----------------------------------------------------------------*/
