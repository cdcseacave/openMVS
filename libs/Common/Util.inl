////////////////////////////////////////////////////////////////////
// Util.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// I N L I N E /////////////////////////////////////////////////////

// compute relative pose of the given two cameras
template<typename TYPE>
inline void ComputeRelativeRotation(const TMatrix<TYPE,3,3>& Ri, const TMatrix<TYPE,3,3>& Rj, TMatrix<TYPE,3,3>& Rij) {
	Rij = Rj * Ri.t();
} // ComputeRelativeRotation
template<typename TYPE>
inline void ComputeRelativePose(const TMatrix<TYPE,3,3>& Ri, const TPoint3<TYPE>& Ci, const TMatrix<TYPE,3,3>& Rj, const TPoint3<TYPE>& Cj, TMatrix<TYPE,3,3>& Rij, TPoint3<TYPE>& Cij) {
	Rij = Rj * Ri.t();
	Cij = Ri * (Cj - Ci);
} // ComputeRelativePose
/*----------------------------------------------------------------*/

// compute the homography matrix transforming points from image A to image B,
// given the relative pose of B with respect to A, and the plane
// (see R. Hartley, "Multiple View Geometry," 2004, pp. 234)
template <typename TYPE>
TMatrix<TYPE,3,3> HomographyMatrixComposition(const TMatrix<TYPE,3,3>& R, const TPoint3<TYPE>& C, const TMatrix<TYPE,3,1>& n, const TYPE& d) {
	const TMatrix<TYPE,3,1> t(R*(-C));
	return TMatrix<TYPE,3,3>(R - t*(n.t()*INVERT(d)));
}
template <typename TYPE>
TMatrix<TYPE,3,3> HomographyMatrixComposition(const TMatrix<TYPE,3,3>& R, const TPoint3<TYPE>& C, const TMatrix<TYPE,3,1>& n, const TMatrix<TYPE,3,1>& X) {
	return HomographyMatrixComposition(R, C, n, -n.dot(X));
}
template <typename TYPE>
TMatrix<TYPE,3,3> HomographyMatrixComposition(const TMatrix<TYPE,3,3>& Ri, const TPoint3<TYPE>& Ci, const TMatrix<TYPE,3,3>& Rj, const TPoint3<TYPE>& Cj, const TMatrix<TYPE,3,1>& n, const TYPE& d) {
	#if 0
	Matrix3x3 R; Point3 C;
	ComputeRelativePose(Ri, Ci, Rj, Cj, R, C);
	return HomographyMatrixComposition(R, C, n, d);
	#else
	const TMatrix<TYPE,3,1> t((Ci - Cj)*INVERT(d));
	return TMatrix<TYPE,3,3>(Rj * (Ri.t() - t*n.t()));
	#endif
}
template <typename TYPE>
TMatrix<TYPE,3,3> HomographyMatrixComposition(const TMatrix<TYPE,3,3>& Ri, const TPoint3<TYPE>& Ci, const TMatrix<TYPE,3,3>& Rj, const TPoint3<TYPE>& Cj, const TMatrix<TYPE,3,1>& n, const TMatrix<TYPE,3,1>& X) {
	#if 0
	Matrix3x3 R; Point3 C;
	ComputeRelativePose(Ri, Ci, Rj, Cj, R, C);
	return HomographyMatrixComposition(R, C, n, X);
	#else
	const TMatrix<TYPE,3,1> t((Ci - Cj)*INVERT(n.dot(X)));
	return TMatrix<TYPE,3,3>(Rj * (Ri.t() + t*n.t()));
	#endif
}
/*----------------------------------------------------------------*/

// transform essential matrix to fundamental matrix
template<typename TYPE>
inline TMatrix<TYPE,3,3> TransformE2F(const TMatrix<TYPE,3,3>& E, const TMatrix<TYPE,3,3>& K1, const TMatrix<TYPE,3,3>& K2) {
	return K2.inv().t() * E * K1.inv();
} // TransformE2F
// transform fundamental matrix to essential matrix
template<typename TYPE>
inline TMatrix<TYPE,3,3> TransformF2E(const TMatrix<TYPE,3,3>& F, const TMatrix<TYPE,3,3>& K1, const TMatrix<TYPE,3,3>& K2) {
	return K2.t() * F * K1;
} // TransformF2E
/*----------------------------------------------------------------*/

// Creates a skew symmetric cross product matrix from the provided tuple.
// compose the cross-product matrix from the given vector: axb=[a]xb
template<typename TYPE>
inline TMatrix<TYPE,3,3> CreateCrossProductMatrix3(const TMatrix<TYPE,3,1>& x) {
	TMatrix<TYPE,3,3> X;
	X(0, 0) =  TYPE(0);  X(1, 0) = -x(2);     X(2, 0) =  x(1);
	X(0, 1) =  x(2);     X(1, 1) =  TYPE(0);  X(2, 1) = -x(0);
	X(0, 2) = -x(1);     X(1, 2) =  x(0);	  X(2, 2) =  TYPE(0);
	return X;
} // CreateCrossProductMatrix3
// same as above, but transposed: axb=[b]xTa
template<typename TYPE>
inline TMatrix<TYPE,3,3> CreateCrossProductMatrix3T(const TMatrix<TYPE,3,1>& x) {
	TMatrix<TYPE,3,3> X;
	X(0, 0) =  TYPE(0);  X(1, 0) =  x(2);     X(2, 0) = -x(1);
	X(0, 1) = -x(2);     X(1, 1) =  TYPE(0);  X(2, 1) =  x(0);
	X(0, 2) =  x(1);     X(1, 2) = -x(0);     X(2, 2) =  TYPE(0);
	return X;
} // CreateCrossProductMatrix3T
/*----------------------------------------------------------------*/

// compose essential matrix from the given rotation and direction
template<typename TYPE>
inline TMatrix<TYPE,3,3> CreateE(const TMatrix<TYPE,3,3>& R, const TMatrix<TYPE,3,1>& C) {
	const TMatrix<TYPE,3,1> t = -(R * C);
	return CreateCrossProductMatrix3(t) * R;
} // CreateE
// compose fundamental matrix from the given rotation, direction and camera matrix
template<typename TYPE>
inline TMatrix<TYPE,3,3> CreateF(const TMatrix<TYPE,3,3>& R, const TMatrix<TYPE,3,1>& C, const TMatrix<TYPE,3,3>& K1, const TMatrix<TYPE,3,3>& K2) {
	const TMatrix<TYPE,3,3> E = CreateE(R, C);
	return TransformE2F(E, K1, K2);
} // CreateF
/*----------------------------------------------------------------*/


// if the matrix is a rotation, then the the transpose should be the inverse
template<typename TYPE>
inline bool IsRotationMatrix(const TMatrix<TYPE,3,3>& R) {
	ASSERT(sizeof(TMatrix<TYPE,3,3>) == sizeof(TRMatrixBase<TYPE>));
	return ((const TRMatrixBase<TYPE>&)R).IsValid();
} // IsRotationMatrix
template<typename TYPE, int O>
inline bool IsRotationMatrix(const Eigen::Matrix<TYPE,3,3,O>& R) {
	// the trace should be three and the determinant should be one
	return (ISEQUAL(R.determinant(), TYPE(1)) && ISEQUAL((R*R.transpose()).trace(), TYPE(3)));
} // IsRotationMatrix
/*----------------------------------------------------------------*/

// enforce matrix orthogonality
template<typename TYPE>
inline void EnsureRotationMatrix(TMatrix<TYPE,3,3>& R) {
	ASSERT(sizeof(TMatrix<TYPE,3,3>) == sizeof(TRMatrixBase<TYPE>));
	((TRMatrixBase<TYPE>&)R).EnforceOrthogonality();
} // EnsureRotationMatrix
/*----------------------------------------------------------------*/


// compute the angle between the two rotations given
// as in: "Disambiguating Visual Relations Using Loop Constraints", 2010
// returns cos(angle) (same as cos(ComputeAngleSO3(I))
template<typename TYPE>
inline TYPE ComputeAngle(const TMatrix<TYPE,3,3>& I) {
	return CLAMP(((TYPE)cv::trace(I)-TYPE(1))*TYPE(0.5), TYPE(-1), TYPE(1));
} // ComputeAngle
template<typename TYPE>
FORCEINLINE TYPE ComputeAngle(const TMatrix<TYPE,3,3>& R1, const TMatrix<TYPE,3,3>& R2) {
	return ComputeAngle(TMatrix<TYPE,3,3>(R1*R2.t()));
} // ComputeAngle
// compute the distance on SO(3) between the two given rotations
// using log(R) as in: "Efficient and Robust Large-Scale Rotation Averaging", 2013
// same result as above, but returns directly the angle
template<typename TYPE>
inline TYPE ComputeAngleSO3(const TMatrix<TYPE,3,3>& I) {
	return TYPE(norm(((const TRMatrixBase<TYPE>&)I).GetRotationAxisAngle()));
} // ComputeAngleSO3
template<typename TYPE>
FORCEINLINE TYPE ComputeAngleSO3(const TMatrix<TYPE,3,3>& R1, const TMatrix<TYPE,3,3>& R2) {
	return ComputeAngleSO3(TMatrix<TYPE,3,3>(R1*R2.t()));
} // ComputeAngleSO3
/*----------------------------------------------------------------*/


// compute the Frobenius (or Euclidean) norm of the distance between the two matrices
template<typename TYPE, int m, int n>
inline TYPE FrobeniusNorm(const TMatrix<TYPE,m,n>& M) {
	return SQRT(((typename TMatrix<TYPE,m,n>::EMatMap)M).cwiseAbs2().sum());
} // FrobeniusNorm
template<typename TYPE, int m, int n>
FORCEINLINE TYPE FrobeniusNorm(const TMatrix<TYPE,m,n>& M1, const TMatrix<TYPE,m,n>& M2) {
	return FrobeniusNorm(TMatrix<TYPE,m,n>(M1-M2));
} // FrobeniusNorm
/*----------------------------------------------------------------*/


// check if any three of the given 2D points are on the same line
template<typename TYPE>
bool CheckCollinearity(const TPoint2<TYPE>* ptr, int count, bool checkPartialSubsets=false) {
	for (int i = (checkPartialSubsets?count-1:2); i < count; ++i) {
		// check that the i-th selected point does not belong
		// to a line connecting some previously selected points
		for (int j = 1; j < i; ++j) {
			const TPoint2<TYPE> d1 = ptr[j] - ptr[i];
			for (int k = 0; k < j; ++k) {
				const TPoint2<TYPE> d2 = ptr[k] - ptr[i];
				if (ABS(d2.x*d1.y - d2.y*d1.x) <= FLT_EPSILON*(ABS(d1.x) + ABS(d1.y) + ABS(d2.x) + ABS(d2.y)))
					return true;
			}
		}
	}
	return false;
}
// check if any three of the given 3D points are on the same line
template<typename TYPE>
bool CheckCollinearity(const TPoint3<TYPE>* ptr, int count, bool checkPartialSubsets=false) {
	for (int i = (checkPartialSubsets?count-1:2); i < count; ++i) {
		// check that the i-th selected point does not belong
		// to a line connecting some previously selected points
		for (int j = 1; j < i; ++j) {
			const TPoint3<TYPE> d1 = ptr[j] - ptr[i];
			for (int k = 0; k < j; ++k) {
				const TPoint3<TYPE> d2 = ptr[k] - ptr[i];
				if (normSq(d1.cross(d2)) < 1.e-16)
					return true;
			}
		}
	}
	return false;
}
/*----------------------------------------------------------------*/


// compute the corresponding ray for a given projection matrix P[3,4] and image point pt[2,1]
// output ray[3,1]
template<typename TYPE1, typename TYPE2>
inline void RayPoint_3x4_2_3(const TYPE1* P, const TYPE2* pt, TYPE1* ray) {
	Eigen::Map< const Eigen::Matrix<TYPE1,3,4> > mP(P);
	const Eigen::Matrix<TYPE1,3,3> mM(mP.template topLeftCorner<3,3>());
	TYPE1 M[9];
	InvertMatrix3x3(mM.data(), M);
	ray[0] = M[0*3+0]*pt[0] + M[0*3+1]*pt[1] + M[0*3+2];
	ray[1] = M[1*3+0]*pt[0] + M[1*3+1]*pt[1] + M[1*3+2];
	ray[2] = M[2*3+0]*pt[0] + M[2*3+1]*pt[1] + M[2*3+2];
} // RayPoint_3x4_2_3
/*----------------------------------------------------------------*/

// project column vertex - used only for visualization purposes
// (optimized ProjectVertex for P[3,4] and X[3,1], output pt[3,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x4_3_3(const TYPE1* P, const TYPE1* X, TYPE2* pt) {
	pt[0] = (TYPE2)(P[0*4+0]*X[0] + P[0*4+1]*X[1] + P[0*4+2]*X[2] + P[0*4+3]);
	pt[1] = (TYPE2)(P[1*4+0]*X[0] + P[1*4+1]*X[1] + P[1*4+2]*X[2] + P[1*4+3]);
	pt[2] = (TYPE2)(P[2*4+0]*X[0] + P[2*4+1]*X[1] + P[2*4+2]*X[2] + P[2*4+3]);
} // ProjectVertex_3x4_3_3
// (optimized ProjectVertex for P[3,4] and X[4,1], output pt[3,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x4_4_3(const TYPE1* P, const TYPE1* X, TYPE2* pt) {
	pt[0] = (TYPE2)(P[0*4+0]*X[0] + P[0*4+1]*X[1] + P[0*4+2]*X[2] + P[0*4+3]*X[3]);
	pt[1] = (TYPE2)(P[1*4+0]*X[0] + P[1*4+1]*X[1] + P[1*4+2]*X[2] + P[1*4+3]*X[3]);
	pt[2] = (TYPE2)(P[2*4+0]*X[0] + P[2*4+1]*X[1] + P[2*4+2]*X[2] + P[2*4+3]*X[3]);
} // ProjectVertex_3x4_4_3
// (optimized ProjectVertex for R[3,3], C[3,1] and X[3,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x3_3_3_2(const TYPE1* R, const TYPE1* C, const TYPE1* X, TYPE2* pt) {
	const TYPE1 T[3] = {X[0]-C[0], X[1]-C[1], X[2]-C[2]};
	const TYPE1 invW(INVERT(R[2*3+0]*T[0] + R[2*3+1]*T[1] + R[2*3+2]*T[2]));
	pt[0] = (TYPE2)((R[0*3+0]*T[0] + R[0*3+1]*T[1] + R[0*3+2]*T[2]) * invW);
	pt[1] = (TYPE2)((R[1*3+0]*T[0] + R[1*3+1]*T[1] + R[1*3+2]*T[2]) * invW);
} // ProjectVertex_3x3_3_3_2
// (optimized ProjectVertex for R[3,3], C[3,1] and X[3,1], output pt[3,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x3_3_3_3(const TYPE1* R, const TYPE1* C, const TYPE1* X, TYPE2* pt) {
	const TYPE1 T[3] = {X[0]-C[0], X[1]-C[1], X[2]-C[2]};
	pt[0] = (TYPE2)(R[0*3+0]*T[0] + R[0*3+1]*T[1] + R[0*3+2]*T[2]);
	pt[1] = (TYPE2)(R[1*3+0]*T[0] + R[1*3+1]*T[1] + R[1*3+2]*T[2]);
	pt[2] = (TYPE2)(R[2*3+0]*T[0] + R[2*3+1]*T[1] + R[2*3+2]*T[2]);
} // ProjectVertex_3x3_3_3_3
// (optimized ProjectVertex for H[3,3] and X[2,1], output pt[3,1])
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void ProjectVertex_3x3_2_3(const TYPE1* H, const TYPE2* X, TYPE3* pt) {
	pt[0] = (TYPE3)(H[0*3+0]*X[0] + H[0*3+1]*X[1] + H[0*3+2]);
	pt[1] = (TYPE3)(H[1*3+0]*X[0] + H[1*3+1]*X[1] + H[1*3+2]);
	pt[2] = (TYPE3)(H[2*3+0]*X[0] + H[2*3+1]*X[1] + H[2*3+2]);
} // ProjectVertex_3x3_2_3
// (optimized ProjectVertex for H[3,3] and X[2,1], output pt[2,1])
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void ProjectVertex_3x3_2_2(const TYPE1* H, const TYPE2* X, TYPE3* pt) {
	const TYPE1 invZ(INVERT(H[2*3+0]*X[0] + H[2*3+1]*X[1] + H[2*3+2]));
	pt[0] = (TYPE3)((H[0*3+0]*X[0] + H[0*3+1]*X[1] + H[0*3+2])*invZ);
	pt[1] = (TYPE3)((H[1*3+0]*X[0] + H[1*3+1]*X[1] + H[1*3+2])*invZ);
} // ProjectVertex_3x3_2_2
/*----------------------------------------------------------------*/

// project column vertex - used only for visualization purposes
// (optimized ProjectVertex for P[3,4] and X[3,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x4_3_2(const TYPE1* P, const TYPE1* X, TYPE2* pt) {
	const TYPE1 invW(INVERT(P[2*4+0]*X[0] + P[2*4+1]*X[1] + P[2*4+2]*X[2] + P[2*4+3]));
	pt[0] = (TYPE2)((P[0*4+0]*X[0] + P[0*4+1]*X[1] + P[0*4+2]*X[2] + P[0*4+3]) * invW);
	pt[1] = (TYPE2)((P[1*4+0]*X[0] + P[1*4+1]*X[1] + P[1*4+2]*X[2] + P[1*4+3]) * invW);
} // ProjectVertex_3x4_3_2
// (optimized ProjectVertex for P[3,4] and X[4,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline void ProjectVertex_3x4_4_2(const TYPE1* P, const TYPE1* X, TYPE2* pt) {
	const TYPE1 invW(INVERT(P[2*4+0]*X[0] + P[2*4+1]*X[1] + P[2*4+2]*X[2] + P[2*4+3]*X[3]));
	pt[0] = (TYPE2)((P[0*4+0]*X[0] + P[0*4+1]*X[1] + P[0*4+2]*X[2] + P[0*4+3]*X[3]) * invW);
	pt[1] = (TYPE2)((P[1*4+0]*X[0] + P[1*4+1]*X[1] + P[1*4+2]*X[2] + P[1*4+3]*X[3]) * invW);
} // ProjectVertex_3x4_4_2
/*----------------------------------------------------------------*/

// project column vertex using the transpose of the given projective matrix
// (optimized ProjectVertex for P[3,4].t() and X[3,1], output pt[4,1])
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void ProjectVertex_3x4t_3_4(const TYPE1* P, const TYPE2* X, TYPE3* pt) {
	pt[0] = (TYPE3)(P[0*4+0]*X[0] + P[1*4+0]*X[1] + P[2*4+0]*X[2]);
	pt[1] = (TYPE3)(P[0*4+1]*X[0] + P[1*4+1]*X[1] + P[2*4+1]*X[2]);
	pt[2] = (TYPE3)(P[0*4+2]*X[0] + P[1*4+2]*X[1] + P[2*4+2]*X[2]);
	pt[3] = (TYPE3)(P[0*4+3]*X[0] + P[1*4+3]*X[1] + P[2*4+3]*X[2]);
} // ProjectVertex_3x4t_3_4
// (optimized ProjectVertex for P[3,4].t() and X[3,1], output pt[3,1] - the first 3 coordinates)
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void ProjectVertex_3x4t_3_3(const TYPE1* P, const TYPE2* X, TYPE3* pt) {
	pt[0] = (TYPE3)(P[0*4+0]*X[0] + P[1*4+0]*X[1] + P[2*4+0]*X[2]);
	pt[1] = (TYPE3)(P[0*4+1]*X[0] + P[1*4+1]*X[1] + P[2*4+1]*X[2]);
	pt[2] = (TYPE3)(P[0*4+2]*X[0] + P[1*4+2]*X[1] + P[2*4+2]*X[2]);
} // ProjectVertex_3x4t_3_3
/*----------------------------------------------------------------*/


// given the camera matrix, compute the error between the given 2D point and the reprojected 3D point;
// note that here we're using "dirty" projection method which is suitable only for visualization; on the upside, it shouldn't matter because
// if a point is projected as infinite, there's something wrong with it anyway;
// (optimized ComputeReprojectionError for P[3,4] and X[3,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeReprojectionError_3x4_3_2(const TYPE1* P, const TYPE1* X, const TYPE2* pt) {
	TYPE2 reprojection[2];
	ProjectVertex_3x4_3_2(P, X, reprojection);
	return SQUARE(reprojection[0]-pt[0])+SQUARE(reprojection[1]-pt[1]);
} // ComputeReprojectionError_3x4_3_2
// (optimized ComputeReprojectionError for P[3,4] and X[4,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeReprojectionError_3x4_4_2(const TYPE1* P, const TYPE1* X, const TYPE2* pt) {
	TYPE2 reprojection[2];
	ProjectVertex_3x4_4_2(P, X, reprojection);
	return SQUARE(reprojection[0]-pt[0])+SQUARE(reprojection[1]-pt[1]);
} // ComputeReprojectionError_3x4_4_2
// (optimized ComputeReprojectionError for R[3,3], C[3,1] and X[3,1], output pt[2,1])
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeReprojectionError_3x3_3_3_2(const TYPE1* R, const TYPE1* C, const TYPE1* X, const TYPE2* pt) {
	TYPE2 reprojection[2];
	ProjectVertex_3x3_3_3_2(R, C, X, reprojection);
	return SQUARE(reprojection[0]-pt[0])+SQUARE(reprojection[1]-pt[1]);
} // ComputeReprojectionError_3x3_3_3_2
/*----------------------------------------------------------------*/


// computes the depth of the given point
// from the point of view of the given camera pose
template<typename TYPE>
inline TYPE PointDepth(const TYPE* R, const TYPE* C, const TYPE* X) {
	return R[2*3+0]*X[0] + R[2*3+1]*X[1] + R[2*3+2]*X[2] + C[2];
} // PointDepth
template<typename TYPE>
inline TYPE PointDepth(const TYPE* P, const TYPE* X) {
	return P[2*4+0]*X[0] + P[2*4+1]*X[1] + P[2*4+2]*X[2] + P[2*4+3];
} // PointDepth
/*----------------------------------------------------------------*/


// reproject the given 3D point with the given view;
// return squared reprojection error
// or FLT_MAX if the point is behind the camera
// reproj return the homogeneous reprojection
template<typename TYPE1, typename TYPE2>
inline TYPE2 ProjectPoint(const TYPE1* P, const TYPE1* X, const TYPE2* proj, TYPE2* reproj) {
	// reproject the 3D point on this view
	ProjectVertex_3x4_3_3(P, X, reproj);
	// filter out points behind
	if (reproj[2] <= TYPE2(0))
		return FLT_MAX;
	// return the reprojection error
	return SQUARE(reproj[0]/reproj[2]-proj[0])+SQUARE(reproj[1]/reproj[2]-proj[1]);
} // ProjectPoint
template<typename TYPE1, typename TYPE2>
inline TYPE2 ProjectPoint(const TYPE1* R, const TYPE1* C, const TYPE1* X, const TYPE2* proj, TYPE2* reproj) {
	// reproject the 3D point on this view
	ProjectVertex_3x3_3_3_3(R, C, X, reproj);
	// filter out points behind
	if (reproj[2] <= TYPE2(0))
		return FLT_MAX;
	// return the reprojection error
	return SQUARE(reproj[0]/reproj[2]-proj[0])+SQUARE(reproj[1]/reproj[2]-proj[1]);
} // ProjectPoint
/*----------------------------------------------------------------*/

// reproject the given 3D point with the given view;
// return true if the point is not behind the camera
// and the reprojection error is small enough;
// returns false otherwise
// reproj returns the homogeneous reprojection (first 3) and errSq (last 1)
template<typename TYPE1, typename TYPE2>
inline bool IsPointInlier(const TYPE1* P, const TYPE1* X, const TYPE2* proj, TYPE2* reproj, TYPE2 thresholdSq) {
	// reproject the 3D point on this view
	ProjectVertex_3x4_3_3(P, X, reproj);
	// filter out points behind the camera or
	// having the reprojection error bigger than the given threshold
	return (reproj[2] > TYPE2(0) &&
		(reproj[3]=(SQUARE(reproj[0]/reproj[2]-proj[0])+SQUARE(reproj[1]/reproj[2]-proj[1]))) <= thresholdSq);
} // IsPointInlier
template<typename TYPE1, typename TYPE2>
inline bool IsPointInlier(const TYPE1* R, const TYPE1* C, const TYPE1* X, const TYPE2* proj, TYPE2* reproj, TYPE2 thresholdSq) {
	// reproject the 3D point on this view
	ProjectVertex_3x3_3_3_3(R, C, X, reproj);
	// filter out points behind the camera or
	// having the reprojection error bigger than the given threshold
	return (reproj[2] > TYPE2(0) &&
		(reproj[3]=(SQUARE(reproj[0]/reproj[2]-proj[0])+SQUARE(reproj[1]/reproj[2]-proj[1]))) <= thresholdSq);
} // IsPointInlier
/*----------------------------------------------------------------*/


// given a rotation matrix, return the angle in radians corresponding to the axis/angle decomposition
template<typename TYPE>
inline TYPE AngleFromRotationMatrix(const TYPE* R) {
	const TYPE a = (R[0*3+0]+R[1*3+1]+R[2*3+2]-TYPE(1))/TYPE(2);
	if (a < TYPE(-1))
		return acos(TYPE(-1));
	if (a > TYPE(1))
		return acos(TYPE(1));
	return acos(a);
}
template<typename TYPE>
FORCEINLINE TYPE AngleFromRotationMatrix(const TMatrix<TYPE,3,3>& R) {
	return AngleFromRotationMatrix(R.val);
}
/*----------------------------------------------------------------*/

// given two 3D vectors,
// compute the angle between them
// returns cos(angle)
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeAngle(const TYPE1* V1, const TYPE1* V2) {
	// compute the angle between the rays
	return CLAMP(TYPE2((V1[0]*V2[0]+V1[1]*V2[1]+V1[2]*V2[2])/SQRT((V1[0]*V1[0]+V1[1]*V1[1]+V1[2]*V1[2])*(V2[0]*V2[0]+V2[1]*V2[1]+V2[2]*V2[2]))), TYPE2(-1), TYPE2(1));
} // ComputeAngle
// given three 3D points,
// compute the angle between the vectors formed by the first point with the other two
// returns cos(angle)
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeAngle(const TYPE1* B, const TYPE1* X1, const TYPE1* X2) {
	// create the two vectors
	const TYPE1 V1[] = {X1[0]-B[0], X1[1]-B[1], X1[2]-B[2]};
	const TYPE1 V2[] = {X2[0]-B[0], X2[1]-B[1], X2[2]-B[2]};
	return ComputeAngle<TYPE1,TYPE2>(V1, V2);
} // ComputeAngle
// given four 3D points,
// compute the angle between the two vectors formed by the first and second pair of points
// returns cos(angle)
template<typename TYPE1, typename TYPE2>
inline TYPE2 ComputeAngle(const TYPE1* X1, const TYPE1* C1, const TYPE1* X2, const TYPE1* C2) {
	// subtract out the camera center
	const TYPE1 V1[] = {X1[0]-C1[0], X1[1]-C1[1], X1[2]-C1[2]};
	const TYPE1 V2[] = {X2[0]-C2[0], X2[1]-C2[1], X2[2]-C2[2]};
	return ComputeAngle<TYPE1,TYPE2>(V1, V2);
} // ComputeAngle
/*----------------------------------------------------------------*/

// given a triangle defined by three 3D points,
// compute its normal (plane's normal oriented according to the given points order)
template<typename TYPE>
inline TPoint3<TYPE> ComputeTriangleNormal(const TPoint3<TYPE>& v0, const TPoint3<TYPE>& v1, const TPoint3<TYPE>& v2) {
	return (v1-v0).cross(v2-v0);
} // ComputeTriangleNormal
/*----------------------------------------------------------------*/

// compute the area of a triangle using Heron's formula
template <typename TYPE>
TYPE ComputeTriangleAreaSqLen(TYPE lena, TYPE lenb, TYPE lenc) {
	const TYPE s((lena+lenb+lenc)/TYPE(2));
	return s*(s-lena)*(s-lenb)*(s-lenc);
}
template <typename TYPE>
TYPE ComputeTriangleAreaLen(TYPE lena, TYPE lenb, TYPE lenc) {
	return (TYPE)sqrt(ComputeTriangleAreaSqLen(lena, lenb, lenc));
}
template <typename TYPE>
TYPE ComputeTriangleAreaSqLenSq(TYPE lenaSq, TYPE lenbSq, TYPE lencSq) {
	return SQUARE(lenaSq+lenbSq+lencSq)/TYPE(2)-(lenaSq*lenaSq+lenbSq*lenbSq+lencSq*lencSq);
}
template <typename TYPE>
TYPE ComputeTriangleAreaLenSq(TYPE lenaSq, TYPE lenbSq, TYPE lencSq) {
	return (TYPE)sqrt(ComputeTriangleAreaSqLenSq(lenaSq, lenbSq, lencSq));
}
// compute area for a triangle defined by three 2D points
template <typename TYPE>
TYPE ComputeTriangleArea(const TPoint2<TYPE>& x0, const TPoint2<TYPE>& x1, const TPoint2<TYPE>& x2) {
	return abs((TYPE)((x0-x2).cross(x0-x1)/TYPE(2)));
}
// compute area for a triangle defined by three 3D points
template <typename TYPE>
TYPE ComputeTriangleAreaSq(const TPoint3<TYPE>& x0, const TPoint3<TYPE>& x1, const TPoint3<TYPE>& x2) {
	return (TYPE)(normSq((x1-x0).cross(x2-x0))/TYPE(4));
}
template <typename TYPE>
TYPE ComputeTriangleArea(const TPoint3<TYPE>& x0, const TPoint3<TYPE>& x1, const TPoint3<TYPE>& x2) {
	return (TYPE)sqrt(ComputeTriangleAreaSq(x0, x1, x2));
} // ComputeTriangleArea
/*----------------------------------------------------------------*/

// compute signed volume of the tetrahedron defined by the given triangle connected to origin
template <typename TYPE>
TYPE ComputeTriangleVolume(const TPoint3<TYPE>& x0, const TPoint3<TYPE>& x1, const TPoint3<TYPE>& x2) {
	return (TYPE)(x0.dot(x1.cross(x2)) / TYPE(6));
} // ComputeTriangleVolume
/*----------------------------------------------------------------*/

// compute a shape quality measure of the triangle composed by vertices (v0,v1,v2)
// returns 2*AreaTri/(MaxEdge^2) in range [0, 0.866]
// (ex: equilateral sqrt(3)/2, half-square 1/2, up to a line that has zero quality)
template<typename TYPE>
inline TYPE ComputeTriangleQuality(const TPoint3<TYPE>& v0, const TPoint3<TYPE>& v1, const TPoint3<TYPE>& v2) {
	const TPoint3<TYPE> d10(v1-v0);
	const TPoint3<TYPE> d20(v2-v0);
	const TPoint3<TYPE> d12(v1-v2);
	const TYPE a((TYPE)norm(d10.cross(d20)));
	if (a == 0) return 0; // area zero triangles have surely zero quality
	const TYPE nd10(normSq(d10));
	if (nd10 == 0) return 0; // area zero triangles have surely zero quality
	const TYPE b(MAXF3(nd10, normSq(d20), normSq(d12)));
	return a/b;
} // ComputeTriangleQuality
/*----------------------------------------------------------------*/

// given a triangle defined by 3 vertex positions and a point,
// compute the barycentric coordinates corresponding to that point
// (only the first two values: alpha and beta)
template <typename TYPE>
inline TPoint2<TYPE> BarycentricCoordinatesUV(const TPoint3<TYPE>& A, const TPoint3<TYPE>& B, const TPoint3<TYPE>& C, const TPoint3<TYPE>& P) {
	#if 0
	// the triangle normal
	const TPoint3<TYPE> normalVec((B-A).cross(C-A));
	//const TYPE normalLen(norm(normalVec));
	// the area of the triangles
	const TYPE invAreaABC(INVERT(normSq(normalVec)/*/(2*normalLen)*/));
	const TPoint3<TYPE> CP(C-P);
	const TYPE areaPBC(normalVec.dot((B-P).cross(CP))/*/(2*normalLen)*/);
	const TYPE areaPCA(normalVec.dot(CP.cross(A-P))/*/(2*normalLen)*/);
	// the barycentric coordinates
	return TPoint2<TYPE>(
		areaPBC * invAreaABC, // alpha
		areaPCA * invAreaABC  // beta
	);
	#else
	// using the Cramer's rule for solving a linear system
	const TPoint3<TYPE> v0(A-C), v1(B-C), v2(P-C);
	const TYPE d00(normSq(v0));
	const TYPE d01(v0.dot(v1));
	const TYPE d11(normSq(v1));
	const TYPE d20(v2.dot(v0));
	const TYPE d21(v2.dot(v1));
	const TYPE invDenom(INVERT(d00 * d11 - d01 * d01));
	return TPoint2<TYPE>(
		(d11 * d20 - d01 * d21) * invDenom, // alpha
		(d00 * d21 - d01 * d20) * invDenom  // beta
	);
	#endif
}
// same as above, but returns all three barycentric coordinates
template <typename TYPE>
inline TPoint3<TYPE> BarycentricCoordinates(const TPoint3<TYPE>& A, const TPoint3<TYPE>& B, const TPoint3<TYPE>& C, const TPoint3<TYPE>& P) {
	TPoint2<TYPE> b(BarycentricCoordinatesUV(A, B, C, P));
	return TPoint3<TYPE>(
		b.x,            // alpha
		b.y,            // beta
		TYPE(1)-b.x-b.y // gamma
	);
}
// same as above, but for 2D triangle case
template <typename TYPE>
inline TPoint2<TYPE> BarycentricCoordinatesUV(const TPoint2<TYPE>& A, const TPoint2<TYPE>& B, const TPoint2<TYPE>& C, const TPoint2<TYPE>& P) {
	const TPoint2<TYPE> D(P - C);
	const TYPE d00(A.x - C.x);
	const TYPE d01(B.x - C.x);
	const TYPE d10(A.y - C.y);
	const TYPE d11(B.y - C.y);
	const TYPE invDet(INVERT(d00 * d11 - d10 * d01));
	return TPoint2<TYPE>(
		(d11 * D.x - d01 * D.y) * invDet, // alpha
		(d00 * D.y - d10 * D.x) * invDet  // beta
	);
}
// same as above, but returns all three barycentric coordinates
template <typename TYPE>
inline TPoint3<TYPE> BarycentricCoordinates(const TPoint2<TYPE>& A, const TPoint2<TYPE>& B, const TPoint2<TYPE>& C, const TPoint2<TYPE>& P) {
	TPoint2<TYPE> b(BarycentricCoordinatesUV(A, B, C, P));
	return TPoint3<TYPE>(
		b.x,            // alpha
		b.y,            // beta
		TYPE(1)-b.x-b.y // gamma
	);
}
// correct the barycentric coordinates in case of numerical errors
// (the corresponding point is assumed to be inside the triangle)
template <typename TYPE>
inline TPoint3<TYPE> CorrectBarycentricCoordinates(TPoint2<TYPE> b) {
	if (b.x < TYPE(0)) // alpha
		b.x = TYPE(0);
	else if (b.x > TYPE(1))
		b.x = TYPE(1);
	if (b.y < TYPE(0)) // beta
		b.y = TYPE(0);
	else if (b.y > TYPE(1))
		b.y = TYPE(1);
	TYPE z(TYPE(1) - b.x - b.y); // gamma
	if (z < 0) {
		// equally distribute the error
		const TYPE half(-z/TYPE(2));
		if (half > b.x) {
			b.x = TYPE(0);
			b.y = TYPE(1);
		} else
		if (half > b.y) {
			b.x = TYPE(1);
			b.y = TYPE(0);
		} else {
			b.x -= half;
			b.y -= half;
		}
		z = TYPE(0);
	}
	// check that the given point is inside the triangle
	ASSERT((b.x >= 0) && (b.y >= 0) && (b.x+b.y <= 1) && ISEQUAL(b.x+b.y+z, TYPE(1)));
	return TPoint3<TYPE>(b.x, b.y, z);
}
/*----------------------------------------------------------------*/

// Encodes/decodes a normalized 3D vector in two parameters for the direction
template<typename T, typename TR>
inline void Normal2Dir(const TPoint3<T>& d, TPoint2<TR>& p) {
	ASSERT(ISEQUAL(norm(d), T(1)));
	#if 0
	// empirically tested
	p.y = TR(atan2(sqrt(d.x*d.x + d.y*d.y), d.z));
	p.x = TR(atan2(d.y, d.x));
	#elif 0
	// empirically tested
	if (d.x == T(0) && d.y == T(0)) {
		ASSERT(ISEQUAL(ABS(d.z), T(1)));
		p.y = TR(M_PI_2);
		p.x = TR(M_PI_2);
	} else {
		p.y = TR(atan2(d.z, sqrt(d.x*d.x + d.y*d.y)));
		p.x = TR(atan2(d.y, d.x));
	}
	#else
	// as in PMVS
	const T b(asin(CLAMP(d.y, T(-1), T(1))));
	p.y = TR(b);
	const T cosb(cos(b));
	if (cosb == T(0)) {
		p.x = TR(0);
	} else {
		const T scl(T(1) / cosb);
		const T sina( d.x * scl);
		const T cosa(-d.z * scl);
		p.x = TR(acos(CLAMP(cosa, T(-1), T(1))));
		if (sina < T(0))
			p.x = -p.x;
	}
	#endif
}
template<typename T, typename TR>
inline void Dir2Normal(const TPoint2<T>& p, TPoint3<TR>& d) {
	#if 0
	// empirically tested
	const T siny(sin(p.y));
	d.x = TR(cos(p.x)*siny);
	d.y = TR(sin(p.x)*siny);
	d.z = TR(cos(p.y));
	#elif 0
	// empirically tested
	const T cosy(cos(p.y));
	d.x = TR(cos(p.x)*cosy);
	d.y = TR(sin(p.x)*cosy);
	d.z = TR(sin(p.y));
	#else
	// as in PMVS
	const T cosy(cos(p.y));
	d.x = TR( sin(p.x)*cosy);
	d.y = TR( sin(p.y));
	d.z = TR(-cos(p.x)*cosy);
	#endif
	ASSERT(ISEQUAL(norm(d), TR(1)));
}
// Encodes/decodes a 3D vector in two parameters for the direction and one parameter for the scale
#if 0
template<typename T, typename TR>
inline void Vector2DirScale(const T vect[3], TR dir[2], TR* scale)
{
	const T x2y2(SQUARE(vect[0])+SQUARE(vect[1]));
	const T scl(sqrt(x2y2+SQUARE(vect[2])));
	scale[0] = TR(scl);
	const T nrm(T(1) / scl);
	dir[1] = TR(atan2(sqrt(x2y2)*nrm, vect[2]*nrm));
	dir[0] = TR(atan2(vect[1]*nrm, vect[0]*nrm));
}
template<typename T, typename TR>
inline void DirScale2Vector(const T dir[2], const T* scale, TR vect[3])
{
	const T cosy(cos(dir[1])*scale[0]);
	vect[0] = TR(cos(dir[0])*cosy);
	vect[1] = TR(sin(dir[0])*cosy);
	vect[2] = TR(sin(dir[1])*scale[0]);
}
#elif 0
template<typename T, typename TR>
inline void Vector2DirScale(const T vect[3], TR dir[2], TR* scale)
{
	const T x2y2(SQUARE(vect[0])+SQUARE(vect[1]));
	const T scl(sqrt(x2y2+SQUARE(vect[2])));
	scale[0] = TR(scl);
	const T nrm(T(1) / scl);
	if (vect[0] == T(0) && vect[1] == T(0)) {
		ASSERT(ISEQUAL(ABS(vect[2]*nrm), T(1)));
		dir[1] = TR(M_PI_2);
		dir[0] = TR(M_PI_2);
	} else {
		dir[1] = TR(atan2(vect[2]*nrm, sqrt(x2y2)*nrm));
		dir[0] = TR(atan2(vect[1]*nrm, vect[0]*nrm));
	}
}
template<typename T, typename TR>
inline void DirScale2Vector(const T dir[2], const T* scale, TR vect[3])
{
	const T cosy(cos(dir[1])*scale[0]);
	vect[0] = TR(cos(dir[0])*cosy);
	vect[1] = TR(sin(dir[0])*cosy);
	vect[2] = TR(sin(dir[1])*scale[0]);
}
#else
template<typename T, typename TR>
inline void Vector2DirScale(const T vect[3], TR dir[2], TR* scale)
{
	const T scl(sqrt(SQUARE(vect[0])+SQUARE(vect[1])+SQUARE(vect[2])));
	scale[0] = TR(scl);
	const T b(asin(CLAMP(vect[1]/scale[0], T(-1), T(1))));
	dir[1] = TR(b);
	const T cosb(cos(b));
	if (cosb == T(0)) {
		dir[0] = TR(0);
	} else {
		const T invscl(T(1) / (scl*cosb));
		const T sina( vect[0] * invscl);
		const T cosa(-vect[2] * invscl);
		dir[0] = TR(acos(CLAMP(cosa, T(-1), T(1))));
		if (sina < T(0))
			dir[0] = -dir[0];
	}
}
template<typename T, typename TR>
inline void DirScale2Vector(const T dir[2], const T* scale, TR vect[3])
{
	const T cos_dir1(cos(dir[1])*scale[0]);
	vect[0] = TR( sin(dir[0]) * cos_dir1);
	vect[1] = TR( sin(dir[1]) * scale[0]);
	vect[2] = TR(-cos(dir[0]) * cos_dir1);
}
#endif
#if TD_VERBOSE == TD_VERBOSE_DEBUG
inline void TestDir() {
	typedef REAL Real;
	for (int i=0; i<10000; ++i) {
		TPoint3<Real> d(
			SEACAVE::randomMeanRange(0.0, 1.0),
			SEACAVE::randomMeanRange(0.0, 1.0),
			SEACAVE::randomMeanRange(0.0, 1.0)
		);
		normalize(d);
		TPoint2<Real> p;
		Normal2Dir(d, p);
		TPoint3<Real> _d;
		Dir2Normal(p, _d);
		ASSERT(ISEQUAL(d, _d));
	}
}
#endif
/*----------------------------------------------------------------*/


template<typename T>
inline T MaxDepthDifference(T d, T threshold) {
	#if 0
	return (d*threshold*T(2))/(threshold+T(2));
	#else
	return d*threshold;
	#endif
}
template<typename T>
inline T DepthSimilarity(T d0, T d1) {
	#if 0
	return ABS(d0-d1)*T(2)/(d0+d1);
	#else
	return ABS(d0-d1)/d0;
	#endif
}
template<typename T>
inline bool IsDepthSimilar(T d0, T d1, T threshold=T(0.01)) {
	return DepthSimilarity(d0, d1) < threshold;
}
template<typename T>
inline bool IsNormalSimilar(const TPoint3<T>& n0, const TPoint3<T>& n1, T threshold=T(0.996194698)/*COS(FD2R(5.f))*/) {
	return ComputeAngle<T,T>(n0.ptr(), n1.ptr()) > threshold;
}
/*----------------------------------------------------------------*/


// searches min/max value
template<typename TYPE>
inline TYPE FindMinElement(const TYPE* values, size_t n) {
	TYPE m = std::numeric_limits<TYPE>::max();
	while (n) if (values[--n] < m) m = values[n];
	return m;
}
template<typename TYPE>
inline TYPE FindMaxElement(const TYPE* values, size_t n) {
	TYPE m = -std::numeric_limits<TYPE>::max();
	while (n) if (values[--n] > m) m = values[n];
	return m;
}
// like above, but considers absolute value only
template<typename TYPE>
inline TYPE FindAbsMinElement(const TYPE* values, size_t n) {
	TYPE m = std::numeric_limits<TYPE>::max();
	while (n) if (ABS(values[--n]) < m) m = ABS(values[n]);
	return m;
}
template<typename TYPE>
inline TYPE FindAbsMaxElement(const TYPE* values, size_t n) {
	TYPE m = -std::numeric_limits<TYPE>::max();
	while (n) if (ABS(values[--n]) > m) m = ABS(values[n]);
	return m;
}
/*----------------------------------------------------------------*/


// given an array of values and their bound, approximate the area covered, in percentage
template<typename TYPE, int n, int s, bool bCentered>
inline TYPE ComputeCoveredArea(const TYPE* values, size_t size, const TYPE* bound, int stride=n) {
	ASSERT(size > 0);
	typedef Eigen::Matrix<TYPE,1,n,Eigen::RowMajor> Vector;
	typedef Eigen::Map<const Vector,Eigen::Unaligned> MapVector;
	typedef Eigen::Matrix<TYPE,Eigen::Dynamic,n,Eigen::RowMajor> Matrix;
	typedef Eigen::Map<const Matrix,Eigen::Unaligned,Eigen::OuterStride<> > MapMatrix;
	typedef Eigen::Matrix<unsigned,s,s,Eigen::RowMajor> MatrixSurface;
	const MapMatrix points(values, size, n, Eigen::OuterStride<>(stride));
	const Vector norm = MapVector(bound);
	const Vector offset(Vector::Constant(bCentered ? TYPE(0.5) : TYPE(0)));
	MatrixSurface surface;
	surface.setZero();
	for (size_t i=0; i<size; ++i) {
		const Vector point((points.row(i).cwiseQuotient(norm)+offset)*TYPE(s));
		ASSERT((point(0)>=0 && point(0)<s) && (point(1)>=0 && point(1)<s));
		surface(FLOOR2INT(point(0)), FLOOR2INT(point(1))) = 1;
	}
	return TYPE(surface.sum())/(s*s);
} // ComputeCoveredArea
/*----------------------------------------------------------------*/


// given an array of values, compute the variance-covariance matrix
template<typename TYPE, int n>
inline void ComputeCovarianceMatrix(const TYPE* values, size_t size, TYPE* cov, int stride=n) {
	ASSERT(size > 0);
	typedef Eigen::Matrix<TYPE,1,n,Eigen::RowMajor> Vector;
	typedef Eigen::Matrix<TYPE,Eigen::Dynamic,n,Eigen::RowMajor> Matrix;
	typedef Eigen::Map<const Matrix,Eigen::Unaligned,Eigen::OuterStride<> > MapMatrix;
	typedef Eigen::Matrix<TYPE,n,n,Eigen::RowMajor> MatrixOut;
	typedef Eigen::Map<MatrixOut> MapMatrixOut;
	const MapMatrix points(values, size, n, Eigen::OuterStride<>(stride));
	const Vector mean(points.colwise().sum() * (TYPE(1)/size));
	const Matrix transPoints(points.rowwise() - mean);
	MapMatrixOut mapCov(cov);
	mapCov = transPoints.transpose() * transPoints * (TYPE(1)/(size-1));
} // ComputeCovarianceMatrix
/*----------------------------------------------------------------*/

// given an array of values, compute the mean
template<typename TYPE, typename TYPEW>
inline void ComputeMean(const TYPE* values, size_t size, TYPEW& mean) {
	ASSERT(size > 0);
	TYPEW sum(0);
	for (size_t i=0; i<size; ++i)
		sum += TYPEW(values[i]);
	mean = sum / size;
} // ComputeMean
/*----------------------------------------------------------------*/

// given an array of values, compute the geometric mean
template<typename TYPE, typename TYPEW>
inline void ComputeGeometricMean(const TYPE* values, size_t size, TYPEW& gmean) {
	ASSERT(size > 0);
	TYPEW prod(1);
	for (size_t i=0; i<size; ++i)
		prod *= TYPEW(values[i]);
	gmean = POW(prod, 1.f/size);
} // ComputeGeometricMean
/*----------------------------------------------------------------*/

// given an array of values, compute the mean and standard deviance
template<typename TYPE, typename TYPEW>
inline void ComputeMeanStdOffline(const TYPE* values, size_t size, TYPEW& mean, TYPEW& stddev) {
	ASSERT(size > 0);
	TYPEW sum(0);
	FOREACHRAWPTR(pVal, values, size)
		sum += TYPEW(*pVal);
	mean = sum / (float)size;
	TYPEW sumSq(0);
	FOREACHRAWPTR(pVal, values, size)
		sumSq += SQUARE(TYPEW(*pVal)-mean);
	const TYPEW variance(sumSq / (size - 1));
	stddev = SQRT(variance);
} // ComputeMeanStdOffline
// same as above, but uses one pass only (online)
template<typename TYPE, typename TYPEW=TYPE, typename ARGTYPE=const TYPE&>
struct MeanStdOnline {
	TYPEW mean;
	TYPEW stddev;
	size_t size;
	MeanStdOnline() : mean(0), stddev(0), size(0) {}
	MeanStdOnline(const TYPE* values, size_t _size) : mean(0), stddev(0), size(0) {
		Compute(values, _size);
	}
	void Update(ARGTYPE v) {
		const TYPEW val(v);
		const TYPEW delta(val-mean);
		mean += delta / (float)(++size);
		stddev += delta * (val-mean);
	}
	void Compute(const TYPE* values, size_t _size) {
		for (size_t i=0; i<_size; ++i)
			Update(values[i]);
	}
	TYPEW GetMean() const { return mean; }
	TYPEW GetVariance() const { return (stddev / (float)(size - 1)); }
	TYPEW GetStdDev() const { return SQRT(GetVariance()); }
};
template<typename TYPE, typename TYPEW>
inline void ComputeMeanStdOnline(const TYPE* values, size_t size, TYPEW& mean, TYPEW& stddev) {
	ASSERT(size > 0);
	mean = TYPEW(0);
	stddev = TYPEW(0);
	for (size_t i=0; i<size; ++i) {
		const TYPEW val(values[i]);
		const TYPEW delta(val-mean);
		mean += delta / (i+1);
		stddev += delta * (val-mean);
	}
	const TYPEW variance(stddev / (float)(size - 1));
	stddev = SQRT(variance);
} // ComputeMeanStdOnline
// same as above, but less stable in general (and faster)
template<typename TYPE, typename TYPEW=TYPE, typename ARGTYPE=const TYPE&>
struct MeanStdOnlineFast {
	TYPEW sum;
	TYPEW sumSq;
	size_t size;
	MeanStdOnlineFast() : sum(0), sumSq(0), size(0) {}
	MeanStdOnlineFast(const TYPE* values, size_t _size) : sum(0), sumSq(0), size(0) {
		Compute(values, _size);
	}
	void Update(ARGTYPE v) {
		const TYPEW val(v);
		sum += val;
		sumSq += SQUARE(val);
		++size;
	}
	void Compute(const TYPE* values, size_t _size) {
		for (size_t i=0; i<_size; ++i)
			Update(values[i]);
	}
	TYPEW GetMean() const { return sum / (float)size; }
	TYPEW GetVariance() const { return (sumSq - SQUARE(sum) / (float)size) / (float)(size - 1); }
	TYPEW GetStdDev() const { return SQRT(GetVariance()); }
};
template<typename TYPE, typename TYPEW>
inline void ComputeMeanStdOnlineFast(const TYPE* values, size_t size, TYPEW& mean, TYPEW& stddev) {
	ASSERT(size > 0);
	TYPEW sum(0), sumSq(0);
	FOREACHRAWPTR(pVal, values, size) {
		const TYPEW val(*pVal);
		sum += val;
		sumSq += SQUARE(val);
	}
	const TYPEW invSize(TYPEW(1)/(float)size);
	mean = sum * invSize;
	const TYPEW variance((sumSq - SQUARE(sum) * invSize) / (float)(size - 1));
	stddev = SQRT(variance);
} // ComputeMeanStdOnlineFast
#define ComputeMeanStd ComputeMeanStdOnlineFast
#define MeanStd MeanStdOnlineFast
/*----------------------------------------------------------------*/

// given an array of values, compute the X84 threshold as in:
// Hampel FR, Rousseeuw PJ, Ronchetti EM, Stahel WA
// "Robust Statistics: the Approach Based on Influence Functions"
// Wiley Series in Probability and Mathematical Statistics, John Wiley & Sons, 1986
// returns the pair(median,trust_region)
// upper-bound threshold = median+trust_region
// lower-bound threshold = median-trust_region
template<typename TYPE, typename TYPEW>
inline std::pair<TYPEW,TYPEW> ComputeX84Threshold(const TYPE* const values, size_t size, TYPEW mul=TYPEW(5.2), const uint8_t* mask=NULL) {
	ASSERT(size > 0);
	// median = MEDIAN(values);
	cList<TYPE, const TYPE&, 0> data;
	if (mask) {
		// use only masked data
		data.Reserve(size);
		for (size_t i=0; i<size; ++i)
			if (mask[i])
				data.Insert(values[i]);
	} else {
		// use all data
		data.CopyOf(values, size);
	}
	TYPE* const mid = data.Begin() + size / 2;
	std::nth_element(data.Begin(), mid, data.End());
	const TYPEW median(*mid);
	// threshold = 5.2 * MEDIAN(ABS(values-median));
	FOREACHPTR(pVal, data)
		*pVal = ABS((*pVal)-median);
	std::nth_element(data.Begin(), mid, data.End());
	return std::make_pair(median, mul*TYPEW(*mid));
} // ComputeX84Threshold
/*----------------------------------------------------------------*/

// given an array of values, compute the upper/lower threshold using quartiles
template<typename TYPE, typename TYPEW>
inline std::tuple<TYPEW,TYPEW,TYPEW,TYPEW,TYPEW> ComputeQuartileThreshold(const TYPE* const values, size_t size, TYPEW mul=TYPEW(1.5), const uint8_t* mask=NULL) {
	ASSERT(size > 0);
	cList<TYPE, const TYPE&, 0> data(size);
	if (mask) {
		// use only masked data
		data.Reserve(size);
		for (size_t i=0; i<size; ++i)
			if (mask[i])
				data.Insert(values[i]);
	} else {
		// use all data
		data.CopyOf(values, size);
	}
	TYPE* const pQB = data.Begin();
	TYPE* const pQE = data.End();
	TYPE* const pQ1 = pQB + size / 4;
	TYPE* const pQ2 = pQB + size / 2;
	TYPE* const pQ3 = pQB + size * 3 / 4;
	std::nth_element(  pQB, pQ2, pQE);
	std::nth_element(  pQB, pQ1, pQ2);
	std::nth_element(pQ2+1, pQ3, pQE);
	const TYPEW Q1(*pQ1), Q2(*pQ2), Q3(*pQ3);
	const TYPEW IQR(Q3 - Q1);
	return std::make_tuple(Q1-mul*IQR, Q3+mul*IQR, Q1, Q2, Q3);
} // ComputeQuartileThreshold
/*----------------------------------------------------------------*/


template <typename TYPE>
inline REAL ComputeSNR(const TDMatrix<TYPE>& x0, const TDMatrix<TYPE>& x) {
	ASSERT(x0.area() == x.area());
	const REAL err(norm(TDMatrix<TYPE>(x0 - x)));
	const REAL x0Norm(norm(x0));
	REAL ret(std::numeric_limits<REAL>::infinity());
	if (ISZERO(x0Norm) && err > 0) ret = 0;
	else if (err > 0)              ret = 20.0 * std::log10(x0Norm / err);
	return ret;
} // ComputeSNR
template<typename TYPE, int N>
inline REAL ComputeSNR(const TMatrix<TYPE,N,1>& x0, const TMatrix<TYPE,N,1>& x) {
	return ComputeSNR(TDMatrix<TYPE>(N,1,(TYPE*)x0.val), TDMatrix<TYPE>(N,1,(TYPE*)x.val));
} // ComputeSNR
template<typename TYPE>
inline REAL ComputePSNR(const TDMatrix<TYPE>& x0, const TDMatrix<TYPE>& x) {
	ASSERT(x0.area() == x.area());
	const size_t N(x0.area());
	const REAL err(normSq(TDMatrix<TYPE>(x0 - x)) / N);
	TYPE max1(0), max2(0);
	for (unsigned i=0; i<N; ++i) {
		max1 = MAXF(max1, x0[i]);
		max2 = MAXF(max2, x[i]);
	}
	const TYPE maxBoth(MAXF(max1, max2));
	REAL ret(std::numeric_limits<REAL>::infinity());
	if (ISZERO(maxBoth) && err > 0) ret = 0;
	else if (err > 0)               ret = 10.0 * std::log10(static_cast<REAL>(maxBoth*maxBoth) / err);
	return ret;
} // ComputePSNR
template<typename TYPE, int N>
inline REAL ComputePSNR(const TMatrix<TYPE,N,1>& x0, const TMatrix<TYPE,N,1>& x) {
	return ComputePSNR(TDMatrix<TYPE>(N,1,(TYPE*)x0.val), TDMatrix<TYPE>(N,1,(TYPE*)x.val));
} // ComputePSNR
/*----------------------------------------------------------------*/


// Builds histograms using "Kernel Density Estimation" and Gaussian kernels;
// see: L. Wassermann: "All of Statistics" for example.
// Samples should be composed of two point: value and weight
template <typename TYPE>
inline TYPE BandwidthGaussianKDE(const TMatrix<TYPE,2,1>* samples, size_t numSamples, TYPE sigma=0, TYPE* pSigma=NULL) {
	if (sigma == 0) {
		TYPE avg(0);
		FOREACHRAWPTR(pSample, samples, numSamples) {
			const TYPE& sample((*pSample)(0));
			avg   += sample;
			sigma += SQUARE(sample);
		}
		avg /= numSamples;
		sigma = SQRT(sigma/numSamples - avg*avg); // Standard Deviation
		if (pSigma) *pSigma = sigma;
	}
	// This is the optimal bandwidth if the point distribution is Gaussian.
	// (see "Applied Smoothing Techniques for Data Analysis" by Adrian W, Bowman & Adelchi Azzalini (1997))
	return POW(TYPE(4)/(TYPE(3)*numSamples), TYPE(1)/TYPE(5))*sigma;
}
// estimate density at the given value
template <typename TYPE>
inline TYPE KernelDensityEstimation(const TMatrix<TYPE,2,1>* samples, size_t numSamples, TYPE x, TYPE bandwidth) {
	TYPE y(0);
	const TYPE invBandwidth(TYPE(1)/bandwidth);
	FOREACHRAWPTR(pSample, samples, numSamples) {
		const TMatrix<TYPE,2,1>& sample(*pSample);
		const TYPE val((x - sample(0))*invBandwidth);
		y += sample(1) * EXP(-TYPE(0.5)*val*val)*invBandwidth;
	}
	return (y / numSamples) * INV_SQRT_2PI;
}
// estimate density at the given range of values;
// INSERTER should have defined the operator(TYPE,TYPE) that receives a pair of value and density
template <typename TYPE, typename INSERTER>
void KernelDensityEstimation(const TMatrix<TYPE,2,1>* samples, size_t numSamples, TYPE bandwidth, TYPE xMin, TYPE xMax, size_t steps, INSERTER& inserter) {
	const TYPE step((xMax-xMin)/steps);
	for (size_t i=0; i<steps; ++i) {
		const TYPE x(xMin+step*i);
		const TYPE y(KernelDensityEstimation(samples, numSamples, x, bandwidth));
		inserter(x, y);
	}
}
// same as above, but tries to detect local minimum/maximum regions and sample them denser;
// INSERTER should have defined also the function SamplePeak(bool)
// that receives true/false if the region contains a maximum/minimum point respectively
// and returns true/false if the region should or not be sampled more often;
// at the end of the peak sampling EndPeak() is called
template <typename TYPE, typename INSERTER>
void KernelDensityEstimation(const TMatrix<TYPE,2,1>* samples, size_t numSamples, TYPE bandwidth, TYPE xMin, TYPE xMax, TYPE stepMax, INSERTER& inserter) {
	size_t smallSteps(0);
	TYPE step(stepMax);
	TYPE x(xMin), last_x;
	TYPE last_y(KernelDensityEstimation(samples, numSamples, x, bandwidth));
	TYPE last_delta(FLT_EPSILON);
	inserter(x, last_y);
	x += step;
	do {
		const TYPE y(KernelDensityEstimation(samples, numSamples, x, bandwidth));
		inserter(x, y);
		if (smallSteps) {
			if (--smallSteps == 0) {
				// continue from the last coarse step
				step = stepMax;
				inserter.EndPeak();
				x = last_x;
			}
		} else {
			const TYPE delta(y-last_y);
			if (ABS(SIGN(last_delta)+SIGN(delta)) < 2 && inserter.SamplePeak(last_delta>0)) {
				// last step contains a local peak (minimum/maximum),
				// so go back and sample more often;
				// assuming the curve is locally symmetric and the step small enough,
				// the peak is inside the first or second step behind
				last_x = x;
				x -= (ABS(delta) < ABS(last_delta) ? step : step*TYPE(2));
				smallSteps = 5;
				step /= TYPE(smallSteps+1);
			}
			last_y = y;
			last_delta = delta;
		}
	} while ((x+=step) < xMax);
}
#if TD_VERBOSE == TD_VERBOSE_DEBUG
// example from http://en.wikipedia.org/wiki/Kernel_density_estimation
inline void ExampleKDE() {
	typedef REAL Real;
	const Real values[6] ={-2.1, -1.3, -0.4, 1.9, 5.1, 6.2};
	TMatrix<Real,2,1> samples[6];
	for (int i=0; i<6; ++i) {
		samples[i](0) = values[i];
		samples[i](1) = 1;
	}
	struct SamplesInserter {
		File f;
		SamplesInserter(const String& fileName)
			: f(fileName, File::WRITE, File::CREATE | File::TRUNCATE) {}
		inline void operator () (Real x, Real y) {
			f.print("%g\t%g\n", x, y);
		}
		inline bool SamplePeak(bool bMaxRegion) const {
			return bMaxRegion;
		}
		inline void EndPeak() {
		}
	};
	const size_t steps(30);
	const Real bandwidth = BandwidthGaussianKDE(samples, 6)/2;
	const Real xMin(-15), xMax(25), stepMax((xMax-xMin)/steps);
	SamplesInserter inserter("kde.txt");
	KernelDensityEstimation(samples, 6, bandwidth, xMin, xMax, stepMax, inserter);
}
#endif
/*----------------------------------------------------------------*/


// normalize image points (inhomogeneous 2D) so that their centroid and typical magnitude of the vector is (1,1)
template <typename TYPE, typename HTYPE>
void NormalizePoints(const CLISTDEF0(TPoint2<TYPE>)& pointsIn, CLISTDEF0(TPoint2<TYPE>)& pointsOut, TMatrix<HTYPE,3,3>* pH=NULL) {
	// find centroid
	TPoint2<HTYPE> ptAvg(0,0);
	FOREACHPTR(pPt, pointsIn)
		ptAvg.x += TPoint2<HTYPE>(*pPt);
	ptAvg *= HTYPE(1)/HTYPE(pointsIn.GetSize());
	// move centroid to origin
	if (pointsOut.GetSize() != pointsIn.GetSize())
		pointsOut.Resize(pointsIn.GetSize());
	HTYPE var(0);
	const TPoint2<TYPE> ptAvgF(ptAvg);
	FOREACH(i, pointsIn) {
		const TPoint2<TYPE>& ptIn = pointsIn[i];
		TPoint2<TYPE>& ptOut = pointsOut[i];
		ptOut = ptIn - ptAvgF;
		var += norm(ptOut);
	}
	// calculate variance
	var /= HTYPE(pointsIn.GetSize());
	// scale points
	HTYPE scale(1);
	if (!ISZERO(var))
		scale = HTYPE(SQRT_2) / var;
	FOREACHPTR(pPt, pointsOut)
		*pPt *= scale;
	// initialize normalizing homography matrix from Scale and Translation (H = S * T);
	if (pH != NULL) {
		TMatrix<HTYPE,3,3>& H = *pH;
		H = TMatrix<HTYPE,3,3>::IDENTITY;
		H(0,2) = -scale * ptAvg.x;
		H(1,2) = -scale * ptAvg.y;
		H(0,0) =  scale;
		H(1,1) =  scale;
	}
}
// normalize scene points (inhomogeneous 3D) so that their centroid and typical magnitude of the vector is (1,1,1)
// This normalization algorithm is suitable only for compact distribution of points (see Hartley04 p180)
template <typename TYPE, typename HTYPE>
void NormalizePoints(const CLISTDEF0(TPoint3<TYPE>)& pointsIn, CLISTDEF0(TPoint3<TYPE>)& pointsOut, TMatrix<HTYPE,4,4>* pH=NULL) {
	// find centroid
	TPoint3<HTYPE> ptAvg(0, 0, 0);
	FOREACHPTR(pPt, pointsIn)
		ptAvg += TPoint3<HTYPE>(*pPt);
	ptAvg *= HTYPE(1)/HTYPE(pointsIn.GetSize());
	// move centroid to origin
	if (pointsOut.GetSize() != pointsIn.GetSize())
		pointsOut.Resize(pointsIn.GetSize());
	HTYPE var(0);
	FOREACH(i, pointsIn) {
		const TPoint3<TYPE>& ptIn = pointsIn[i];
		TPoint3<TYPE>& ptOut = pointsOut[i];
		ptOut = ptIn - ptAvg;
		var += norm(ptOut);
	}
	// calculate variance
	var /= HTYPE(pointsIn.GetSize());
	// scale points
	HTYPE scale(1);
	if (!ISZERO(var))
		scale = HTYPE(SQRT_3) / var;
	FOREACHPTR(pPt, pointsOut)
		*pPt *= scale;
	// initialize normalizing homography matrix
	if (pH != NULL) {
		TMatrix<HTYPE,4,4>& H = *pH;
		H = TMatrix<HTYPE,4,4>::IDENTITY;
		H(0,3) = -scale * ptAvg.x;
		H(1,3) = -scale * ptAvg.y;
		H(2,3) = -scale * ptAvg.z;
		H(0,0) =  scale;
		H(1,1) =  scale;
		H(2,2) =  scale;
	}
}
/*----------------------------------------------------------------*/


// Least squares fits a plane to a 3D point set.
// See http://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
// Returns a fitting quality (1 - lambda_min/lambda_max):
//  1 is best (zero variance orthogonally to the fitting line)
//  0 is worst (isotropic case, returns a plane with default direction)
template <typename TYPE>
TYPE FitPlane(const TPoint3<TYPE>* points, size_t size, TPlane<TYPE>& plane) {
	// compute a point on the plane, which is shown to be the centroid of the points
	const Eigen::Map< const Eigen::Matrix<TYPE,Eigen::Dynamic,3,Eigen::RowMajor> > vPoints((const TYPE*)points, size, 3);
	const TPoint3<TYPE> c(vPoints.colwise().mean());

	// assemble covariance matrix; matrix numbering:
	// 0          
	// 1 2
	// 3 4 5          
	Eigen::Matrix<TYPE,3,3,Eigen::RowMajor> A(Eigen::Matrix<TYPE,3,3,Eigen::RowMajor>::Zero());
	FOREACHRAWPTR(pPt, points, size) {
		const TPoint3<TYPE> X(*pPt - c);
		A(0,0) += X.x*X.x;
		A(1,0) += X.x*X.y;
		A(1,1) += X.y*X.y;
		A(2,0) += X.x*X.z;
		A(2,1) += X.y*X.z;
		A(2,2) += X.z*X.z;
	}

	// the plane normal is simply the eigenvector corresponding to least eigenvalue
	const Eigen::SelfAdjointEigenSolver< Eigen::Matrix<TYPE,3,3,Eigen::RowMajor> > es(A);
	ASSERT(ISEQUAL(es.eigenvectors().col(0).norm(), TYPE(1)));
	plane.Set(es.eigenvectors().col(0), c);
	const TYPE* const vals(es.eigenvalues().data());
	ASSERT(vals[0] <= vals[1] && vals[1] <= vals[2]);
	return TYPE(1) - vals[0]/vals[1];
}
/*----------------------------------------------------------------*/


template<typename TYPE>
struct CLeastSquares {
	inline TYPE operator()(const TYPE) const {
		return TYPE(1);
	}
};

/*
template<typename TYPE>
struct CHuber_TorrMurray {
	inline TYPE operator()(const TYPE d) const {
		const TYPE b = 0.02;

		const TYPE d_abs = ABS(d);
		if (d_abs < b)
			return TYPE(1);
		if (d_abs < 3 * b)
			return b / d; //I think this is possibly wrong--should use 1/root(d)
		return TYPE(0); //TODO, probably best to just return SIGMA/d;
	}
};
*/

template<typename TYPE>
struct CHuber {
	inline CHuber(TYPE _threshold = 0.005) : threshold(_threshold) {}
	inline TYPE operator()(const TYPE d) const {
		const TYPE d_abs(ABS(d));
		if (d_abs < threshold)
			return TYPE(1);
		return SQRT(threshold * (TYPE(2) * d_abs - threshold)) / d_abs;
	}
	const TYPE threshold;
};

template<typename TYPE>
struct CBlakeZisserman { // Blake-Zisserman Gaussian + uniform
	inline CBlakeZisserman(TYPE _threshold = 0.005) : threshold(_threshold) {}
	inline TYPE operator()(const TYPE d) const {
		const TYPE SD_INV = TYPE(1) / (0.5 * threshold);
		const TYPE eps = exp(-SQUARE(threshold * SD_INV)); //Equally likely to be inlier or outlier at thresh
		const TYPE d_abs = ABS(d) + 1e-12;
		const TYPE zeroPoint = log(TYPE(1) + eps); //Needed for LS...

		const TYPE dCost_sq = zeroPoint - log(exp(-SQUARE(d * SD_INV)) + eps);
		ASSERT(dCost_sq >= 0); // Cost computation failed?

		return SQRT(dCost_sq) / d_abs;
	}
	const TYPE threshold;
};

template<typename TYPE>
struct CPseudoHuber {
	inline CPseudoHuber(TYPE _threshold = 0.005) : threshold(_threshold) {}
	inline TYPE operator()(const TYPE d) const {
		const TYPE b_sq = SQUARE(threshold);
		const TYPE d_abs = ABS(d) + 1e-12;

		//C(delta) = 2*b^2*(sqrt(1+(delta/b)^2) - 1);
		return SQRT(TYPE(2) * b_sq * (sqrt(1 + SQUARE(d * (1.0 / threshold))) - 1)) / d_abs;
	}
	const TYPE threshold;
};

template<typename TYPE>
struct CL1 {
	inline TYPE operator()(const TYPE d) const {
		return TYPE(1) / (SQRT(ABS(d)) + 1e-12);
	}
};

typedef CBlakeZisserman<REAL> CRobustNorm;
/*----------------------------------------------------------------*/


// makes sure the inverse NCC score does not exceed 0.3
#if 0
template <typename T>
inline T robustincc(const T x) { return x / (T(1) + T(3) * x); }
template <typename T>
inline T robustinccg(const T x) { return T(1)/SQUARE(T(1) + T(3) * x); }
template <typename T>
inline T unrobustincc(const T y) { return y / (T(1) - T(3) * y); }
#elif 0
template <typename T>
inline T robustincc(const T x) { return T(1)-EXP(x*x*T(-4)); }
template <typename T>
inline T robustinccg(const T x) { return T(8)*x*EXP(x*x*T(-4)); }
template <typename T>
inline T unrobustincc(const T y) { return SQRT(-LOGN(T(1) - y))/T(2); }
#else
template <typename T>
inline T robustincc(const T x) { return x/SQRT(T(0.3)+x*x); }
template <typename T>
inline T robustinccg(const T x) { return T(0.3)/((T(0.3)+x*x)*SQRT(T(0.3)+x*x)); }
template <typename T>
inline T unrobustincc(const T y) { return (SQRT(30)*y)/(T(10)*SQRT(T(1) - y*y)); }
#endif
/*----------------------------------------------------------------*/

} // namespace SEACAVE
