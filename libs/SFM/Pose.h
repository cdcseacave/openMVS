////////////////////////////////////////////////////////////////////
// Pose.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_POSE_H_
#define _SFM_POSE_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// 3D pose representation following MVS convention:
// P = KR[I|-C]
// where R is rotation from world to camera coordinates
// and C is the camera center position in world coordinates
class SFM_API Pose3D
{
public:
	RMatrix R; // rotation matrix from world to camera coordinates
	CMatrix C; // camera center position in world coordinates

public:
	inline Pose3D() {}
	inline Pose3D(const RMatrix& _R, const CMatrix& _C) : R(_R), C(_C) {}

	// Identity pose
	static inline Pose3D Identity() {
		return Pose3D(RMatrix::IDENTITY, CMatrix::ZERO);
	}
	inline bool operator == (const Pose3D& rhs) const {
		return R == rhs.R && C == rhs.C;
	}

	// Set/Get translation vector (t = -R*C)
	inline void SetT(const CMatrix& T) { C = R.t()*(-T); }
	inline CMatrix GetT() const { return R*(-C); }

	// Returns the camera's view forward direction
	inline Point3 Direction() const { return R.row(2); /* equivalent to R.t() * Vec(0,0,1) */ }

	// Returns the depth of a 3D point from world to camera coordinates
	inline REAL Depth(const Point3& X) const { return Direction().dot(X - C); }

	// Returns the ray from camera to world coordinates
	inline Point3 RayCameraToWorld(const Point3& X) const { return R.t() * X; }

	// Compose/decompose projection matrix P = K*R*[I|-C]
	PMatrix GetPfromRC() const; // compose P from R and C only

	// Compute the inverse pose (world to camera -> camera to world)
	inline Pose3D Inverse() const {
		return Pose3D(
			R.t(),
			-(R * C)
		);
	}

	// Compose two poses: this * other
	// Applies the relative transformation 'this' to pose 'other'.
	// Usage: Pose2 = RelPose * Pose1
	// Logic: World -> [Pose1] -> Camera1 -> [RelPose] -> Camera2
	inline Pose3D operator * (const Pose3D& other) const {
		return Pose3D(
			R * other.R,
			other.C + other.R.t() * C
		);
	}

	// Compute the relative pose from this to other: result * this = other
	// Returns the transformation that when composed with 'this' yields 'other'.
	// Usage: RelPose = Pose2 / Pose1 ('this' is Pose2, 'other' is Pose1)
	inline Pose3D operator / (const Pose3D& other) const {
		return Pose3D(
			R * other.R.t(),
			other.R * (C - other.C)
		);
	}

	// Transform a 3D point from world to camera coordinates
	inline Point3 TransformPointW2C(const Point3& X) const {
		return R * (X - C);
	}

	// Transform a 3D point from camera to world coordinates
	inline Point3 TransformPointC2W(const Point3& X) const {
		return R.t() * X + C;
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & R;
		ar & C;
	}
	#endif
};

// Build an orthonormal tangent basis (e1, e2) perpendicular to unit vector n
inline void OrthonormalBasis(const Point3& n, Point3& e1, Point3& e2)
{
	ASSERT(ABS(normSq(n) - REAL(1)) < REAL(1e-4));
	const Point3 ref = (ABS(n.x) < REAL(0.9)) ? Point3(1, 0, 0) : Point3(0, 1, 0);
	e1 = normalized(n.cross(ref));
	e2 = n.cross(e1);
}

// Angle in degrees between two non-zero 3D directions
inline REAL DirectionAngleDeg(const Point3& a, const Point3& b)
{
	const REAL na = norm(a), nb = norm(b);
	ASSERT(na > ZEROTOLERANCE<REAL>() && nb > ZEROTOLERANCE<REAL>());
	return R2D(ACOS(CLAMP(a.dot(b) / (na * nb), REAL(-1), REAL(1))));
}

} // namespace SFM

#endif // _SFM_POSE_H_

