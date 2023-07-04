////////////////////////////////////////////////////////////////////
// Util.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// I N L I N E /////////////////////////////////////////////////////

// normalize inhomogeneous 2D point by the given camera intrinsics K
// K is assumed to be the [3,3] triangular matrix with: fx, fy, s, cx, cy and scale 1
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void NormalizeProjection(const TYPE1* K, const TYPE2* x, TYPE3* n) {
	ASSERT(ISZERO(K[3]) && ISZERO(K[6]) && ISZERO(K[7]) && ISEQUAL(K[8], TYPE1(1)));
	n[0] = TYPE3(K[0]*x[0] + K[1]*x[1] + K[2]);
	n[1] = TYPE3(            K[4]*x[1] + K[5]);
} // NormalizeProjection
// same as above, but using K inverse
template<typename TYPE1, typename TYPE2, typename TYPE3>
inline void NormalizeProjectionInv(const TYPE1* K, const TYPE2* x, TYPE3* n) {
	ASSERT(ISZERO(K[3]) && ISZERO(K[6]) && ISZERO(K[7]) && ISEQUAL(K[8], TYPE1(1)));
	n[0] = TYPE3((K[4]*x[0] - K[1]*x[1] + (K[5]*K[1]-K[2]*K[4])) / (K[0]*K[4]));
	n[1] = TYPE3((x[1] - K[5]) / K[4]);
} // NormalizeProjectionInv
/*----------------------------------------------------------------*/

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

// Triangulate the position of a 3D point
// given two corresponding normalized projections and the pose of the second camera relative to the first one;
// returns the triangulated 3D point from the point of view of the first camera
template<typename TYPE1, typename TYPE2>
bool TriangulatePoint3D(
	const TMatrix<TYPE1,3,3>& R, const TPoint3<TYPE1>& C,
	const TPoint2<TYPE2>& pt1, const TPoint2<TYPE2>& pt2,
	TPoint3<TYPE1>& X
) {
	// convert image points to 3-vectors (of unit length)
	// used to describe landmark observations/bearings in camera frames
	const TPoint3<TYPE1> f1(pt1.x,pt1.y,1);
	const TPoint3<TYPE1> f2(pt2.x,pt2.y,1);
	const TPoint3<TYPE1> f2_unrotated = R.t() * f2;
	const TPoint2<TYPE1> b(C.dot(f1), C.dot(f2_unrotated));
	// optimized inversion of A
	const TYPE1 a = normSq(f1);
	const TYPE1 c = f1.dot(f2_unrotated);
	const TYPE1 d = -normSq(f2_unrotated);
	const TYPE1 det = a*d+c*c;
	if (ABS(det) < EPSILONTOLERANCE<TYPE1>())
		return false;
	const TYPE1 invDet = TYPE1(1)/det;
	const TPoint2<TYPE1> lambda((d*b.x+c*b.y)*invDet, (a*b.y-c*b.x)*invDet);
	const TPoint3<TYPE1> xm = lambda.x * f1;
	const TPoint3<TYPE1> xn = C + lambda.y * f2_unrotated;
	X = (xm + xn)*TYPE1(0.5);
	return true;
} // TriangulatePoint3D
// same as above, but using the two camera poses;
// returns the 3D point in world coordinates
template<typename TYPE1, typename TYPE2>
bool TriangulatePoint3D(
	const TMatrix<TYPE1,3,3>& K1, const TMatrix<TYPE1,3,3>& K2,
	const TMatrix<TYPE1,3,3>& R1, const TMatrix<TYPE1,3,3>& R2,
	const TPoint3<TYPE1>& C1, const TPoint3<TYPE1>& C2,
	const TPoint2<TYPE2>& x1, const TPoint2<TYPE2>& x2,
	TPoint3<TYPE1>& X
) {
	TPoint2<TYPE1> pt1, pt2;
	NormalizeProjectionInv(K1.val, x1.ptr(), pt1.ptr());
	NormalizeProjectionInv(K2.val, x2.ptr(), pt2.ptr());
	TMatrix<TYPE1,3,3> R; TPoint3<TYPE1> C;
	ComputeRelativePose(R1, C1, R2, C2, R, C);
	if (!TriangulatePoint3D(R, C, pt1, pt2, X))
		return false;
	X = R1.t() * X + C1;
	return true;
} // TriangulatePoint3D
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


// computes an RQ decomposition of 3x3 matrices as in:
// "Computing euler angles from a rotation matrix", Gregory G Slabaugh, 1999
template <typename Scalar>
inline void RQDecomp3x3(Eigen::Matrix<Scalar,3,3> M, Eigen::Matrix<Scalar,3,3>& R, Eigen::Matrix<Scalar,3,3>& Q) {
	// find Givens rotation for x axis:
	//       | 1  0  0 |
	//  Qx = | 0  c  s |, c = m33/sqrt(m32^2 + m33^2), s = m32/sqrt(m32^2 + m33^2)
	//       | 0 -s  c |
	Eigen::Matrix<Scalar,2,1> cs = Eigen::Matrix<Scalar,2,1>(M(2,2), M(2,1)).normalized();
	Eigen::Matrix<Scalar,3,3> Qx{ {1, 0, 0}, {0, cs.x(), cs.y()}, {0, -cs.y(), cs.x()} };
	R.noalias() = M * Qx;
	ASSERT(std::abs(R(2,1)) < FLT_EPSILON);
	R(2,1) = 0;
	// find Givens rotation for y axis:
	//       | c  0 -s |
	//  Qy = | 0  1  0 |, c = m33/sqrt(m31^2 + m33^2), s = -m31/sqrt(m31^2 + m33^2)
	//       | s  0  c |
	cs = Eigen::Matrix<Scalar,2,1>(R(2,2), -R(2,0)).normalized();
	Eigen::Matrix<Scalar,3,3> Qy{ {cs.x(), 0, -cs.y()}, {0, 1, 0}, {cs.y(), 0, cs.x()} };
	M.noalias() = R * Qy;
	ASSERT(std::abs(M(2,0)) < FLT_EPSILON);
	M(2,0) = 0;
	// find Givens rotation for z axis:
	//       | c  s  0 |
	//  Qz = |-s  c  0 |, c = m22/sqrt(m21^2 + m22^2), s = m21/sqrt(m21^2 + m22^2)
	//       | 0  0  1 |
	cs = Eigen::Matrix<Scalar,2,1>(M(1,1), M(1,0)).normalized();
	Eigen::Matrix<Scalar,3,3> Qz{ {cs.x(), cs.y(), 0}, {-cs.y(), cs.x(), 0}, {0, 0, 1} };
	R.noalias() = M * Qz;
	ASSERT(std::abs(R(1,0)) < FLT_EPSILON);
	R(1,0) = 0;
	// solve the decomposition ambiguity:
	//  - diagonal entries of R, except the last one, shall be positive
	//  - rotate R by 180 degree if necessary
	if (R(0,0) < 0) {
		if (R(1,1) < 0) {
			// rotate around z for 180 degree:
			//  |-1,  0,  0|
			//  | 0, -1,  0|
			//  | 0,  0,  1|
			R(0,0) *= -1;
			R(0,1) *= -1;
			R(1,1) *= -1;
			Qz(0,0) *= -1;
			Qz(0,1) *= -1;
			Qz(1,0) *= -1;
			Qz(1,1) *= -1;
		} else {
			// rotate around y for 180 degree:
			//  |-1,  0,  0|
			//  | 0,  1,  0|
			//  | 0,  0, -1|
			R(0,0) *= -1;
			R(0,2) *= -1;
			R(1,2) *= -1;
			R(2,2) *= -1;
			Qz.transposeInPlace();
			Qy(0,0) *= -1;
			Qy(0,2) *= -1;
			Qy(2,0) *= -1;
			Qy(2,2) *= -1;
		}
	} else if (R(1,1) < 0) {
		// rotate around x for 180 degree:
		//  | 1,  0,  0|
		//  | 0, -1,  0|
		//  | 0,  0, -1|
		R(0,1) *= -1;
		R(0,2) *= -1;
		R(1,1) *= -1;
		R(1,2) *= -1;
		R(2,2) *= -1;
		Qz.transposeInPlace();
		Qy.transposeInPlace();
		Qx(1,1) *= -1;
		Qx(1,2) *= -1;
		Qx(2,1) *= -1;
		Qx(2,2) *= -1;
	}
	// calculate orthogonal matrix
	Q.noalias() = Qz.transpose() * Qy.transpose() * Qx.transpose();
}
template <typename TYPE>
inline void RQDecomp3x3(const TMatrix<TYPE,3,3>& M, TMatrix<TYPE,3,3>& R, TMatrix<TYPE,3,3>& Q) {
	const Eigen::Matrix<TYPE,3,3> _M((typename TMatrix<TYPE,3,3>::CEMatMap)M);
	Eigen::Matrix<TYPE,3,3> _R, _Q;
	RQDecomp3x3<TYPE>(_M, _R, _Q);
	R = _R; Q = _Q;
} // RQDecomp3x3
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
template<typename TYPE1, typename TYPE2 = TYPE1>
inline TYPE2 ComputeAngle(const TYPE1* V1, const TYPE1* V2) {
	return CLAMP(TYPE2((V1[0]*V2[0]+V1[1]*V2[1]+V1[2]*V2[2])/SQRT((V1[0]*V1[0]+V1[1]*V1[1]+V1[2]*V1[2])*(V2[0]*V2[0]+V2[1]*V2[1]+V2[2]*V2[2]))), TYPE2(-1), TYPE2(1));
} // ComputeAngle
// same as above, but with the vectors normalized
template<typename TYPE1, typename TYPE2 = TYPE1>
inline TYPE2 ComputeAngleN(const TYPE1* V1, const TYPE1* V2) {
	return CLAMP(TYPE2(V1[0]*V2[0]+V1[1]*V2[1]+V1[2]*V2[2]), TYPE2(-1), TYPE2(1));
} // ComputeAngleN
// given three 3D points,
// compute the angle between the vectors formed by the first point with the other two
// returns cos(angle)
template<typename TYPE1, typename TYPE2 = TYPE1>
inline TYPE2 ComputeAngle(const TYPE1* B, const TYPE1* X1, const TYPE1* X2) {
	// create the two vectors
	const TYPE1 V1[] = {X1[0]-B[0], X1[1]-B[1], X1[2]-B[2]};
	const TYPE1 V2[] = {X2[0]-B[0], X2[1]-B[1], X2[2]-B[2]};
	return ComputeAngle<TYPE1,TYPE2>(V1, V2);
} // ComputeAngle
// given four 3D points,
// compute the angle between the two vectors formed by the first and second pair of points
// returns cos(angle)
template<typename TYPE1, typename TYPE2 = TYPE1>
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
TYPE EdgeFunction(const TPoint2<TYPE>& x0, const TPoint2<TYPE>& x1, const TPoint2<TYPE>& x2) {
	return TYPE((x2-x0).cross(x1-x0));
}
template <typename TYPE>
TYPE ComputeTriangleArea(const TPoint2<TYPE>& x0, const TPoint2<TYPE>& x1, const TPoint2<TYPE>& x2) {
	return (TYPE)abs(EdgeFunction(x0, x1, x2)/TYPE(2));
}
// compute area for a triangle defined by three 3D points
template <typename TYPE>
TYPE ComputeTriangleAreaSq(const TPoint3<TYPE>& x0, const TPoint3<TYPE>& x1, const TPoint3<TYPE>& x2) {
	return (TYPE)normSq((x1-x0).cross(x2-x0))/TYPE(4);
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
template <typename TYPE>
inline TPoint3<TYPE> PerspectiveCorrectBarycentricCoordinates(const TPoint3<TYPE>& b, TYPE z0, TYPE z1, TYPE z2) {
	const TPoint3<TYPE> pb(b.x * z1 * z2, b.y * z0 * z2, b.z * z0 * z1);
	return pb / (pb.x + pb.y + pb.z);
}
/*----------------------------------------------------------------*/

// Encodes/decodes a normalized 3D vector in two parameters for the direction
template<typename T, typename TR>
inline void Normal2Dir(const TPoint3<T>& d, TPoint2<TR>& p) {
	ASSERT(ISEQUAL(norm(d), T(1)));
	p.x = TR(atan2(d.y, d.x));
	p.y = TR(acos(d.z));
}
template<typename T, typename TR>
inline void Dir2Normal(const TPoint2<T>& p, TPoint3<TR>& d) {
	const T siny(sin(p.y));
	d.x = TR(cos(p.x)*siny);
	d.y = TR(sin(p.x)*siny);
	d.z = TR(cos(p.y));
	ASSERT(ISEQUAL(norm(d), TR(1)));
}
// Encodes/decodes a 3D vector in two parameters for the direction and one parameter for the scale
template<typename T, typename TR>
inline void Vector2DirScale(const T vect[3], TR dir[2], TR* scale)
{
	const T scl(sqrt(SQUARE(vect[0])+SQUARE(vect[1])+SQUARE(vect[2])));
	ASSERT(!ISZERO(scl));
	scale[0] = TR(scl);
	dir[0] = TR(atan2(vect[1], vect[0]));
	dir[1] = TR(acos(vect[2] / scl));
}
template<typename T, typename TR>
inline void DirScale2Vector(const T dir[2], const T* scale, TR vect[3])
{
	ASSERT(!ISZERO(*scale));
	const T siny(*scale*sin(dir[1]));
	vect[0] = TR(cos(dir[0])*siny);
	vect[1] = TR(sin(dir[0])*siny);
	vect[2] = TR(*scale*cos(dir[1]));
}
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
	ASSERT(d0 > 0);
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
template<typename TYPE, typename TYPEW>
inline void ComputeMeanStdOnline(const TYPE* values, size_t size, TYPEW& mean, TYPEW& stddev) {
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
#define ComputeMeanStd ComputeMeanStdOnline
// same as above, but an interactive version
template<typename TYPE, typename TYPEW=TYPE, typename ARGTYPE=const TYPE&, typename TYPER=REAL>
struct MeanStd {
	typedef TYPE Type;
	typedef TYPEW TypeW;
	typedef TYPER TypeR;
	typedef ARGTYPE ArgType;
	TYPEW sum, sumSq;
	size_t size;
	MeanStd() : sum(0), sumSq(0), size(0) {}
	MeanStd(const Type* values, size_t _size) : MeanStd() { Compute(values, _size); }
	void Update(ArgType v) {
		const TYPEW val(static_cast<TYPEW>(v));
		sum += val;
		sumSq += SQUARE(val);
		++size;
	}
	void Compute(const Type* values, size_t _size) {
		for (size_t i=0; i<_size; ++i)
			Update(values[i]);
	}
	TYPEW GetSum() const { return sum; }
	TYPEW GetMean() const { return static_cast<TYPEW>(sum / static_cast<TypeR>(size)); }
	TYPEW GetRMS() const { return static_cast<TYPEW>(SQRT(sumSq / static_cast<TypeR>(size))); }
	TYPEW GetVarianceN() const { return static_cast<TYPEW>(sumSq - SQUARE(sum) / static_cast<TypeR>(size)); }
	TYPEW GetVariance() const { return static_cast<TYPEW>(GetVarianceN() / static_cast<TypeR>(size)); }
	TYPEW GetStdDev() const { return SQRT(GetVariance()); }
	void Clear() { sum = sumSq = TYPEW(0); size = 0; }
};
// same as above, but records also min/max values
template<typename TYPE, typename TYPEW=TYPE, typename ARGTYPE=const TYPE&, typename TYPER=REAL>
struct MeanStdMinMax : MeanStd<TYPE,TYPEW,ARGTYPE,TYPER> {
	typedef MeanStd<TYPE,TYPEW,ARGTYPE,TYPER> Base;
	typedef TYPE Type;
	typedef TYPEW TypeW;
	typedef TYPER TypeR;
	typedef ARGTYPE ArgType;
	Type minVal, maxVal;
	MeanStdMinMax() : minVal(std::numeric_limits<Type>::max()), maxVal(std::numeric_limits<Type>::lowest()) {}
	MeanStdMinMax(const Type* values, size_t _size) : MeanStdMinMax() { Compute(values, _size); }
	void Update(ArgType v) {
		if (minVal > v)
			minVal = v;
		if (maxVal < v)
			maxVal = v;
		Base::Update(v);
	}
	void Compute(const Type* values, size_t _size) {
		for (size_t i=0; i<_size; ++i)
			Update(values[i]);
	}
};
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
	using TYPEI = typename MakeSigned<TYPE>::type;
	for (TYPE& val: data)
		val = TYPE(ABS(TYPEI(val)-TYPEI(median)));
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
		inline void operator() (Real x, Real y) {
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

} // namespace SEACAVE
