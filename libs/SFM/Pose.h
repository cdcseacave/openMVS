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
	// Useful for computing how to transform from one absolute pose to another.
    // Usage: RelPose = Pose2 / Pose1
    // 'this' is the Target Pose (Pose2)
    // 'other' is the Source Pose (Pose1)
    // Logic: Find RelPose such that Pose2 = RelPose * Pose1
	// This is equivalent to: this * other.Inverse()
	// or: other * (this / other) = this
	// (equivalent to ComputeRelativePose, but swapped i and j)
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

} // namespace SFM

#endif // _SFM_POSE_H_

