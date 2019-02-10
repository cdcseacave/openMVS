////////////////////////////////////////////////////////////////////
// Rotation.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_ROTATION_H__
#define __SEACAVE_ROTATION_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#ifndef _RELEASE
#define _CPC_DEBUG
#endif

#define CPC_ERROR(msg) { SLOG(msg); ASSERT(false); }
#if TD_VERBOSE != TD_VERBOSE_OFF
#define CPC_SLOG(msg) if (VERBOSITY_LEVEL > 0) SLOG(msg)
#define CPC_LOG(t, msg) CPC_SLOG(msg)
#else
#define CPC_SLOG(msg)
#define CPC_LOG(t, msg)
#endif


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// generic matrix struct
#define QUATERNION_EPSILON 1E-10 // DBL_EPSILON, too small (dherzog)

// forward declaration to avoid mutual inclusion
template <typename TYPE> class TRMatrixBase;

  /** @class TQuaternion
   *  @test tested with TestRotationConversion.cpp
	  @ingroup g_geometry
	  @brief class for rotation with axis and angle

	  Quaternions can be used for rotation around an axis through the
	  origin by spcecifing the normal direction vector of the axis and
	  an angle.

	  The rotation for positive angles is clockwise,
	  when looking in direction of the axis.

	  Contains qx=[0], qy=[1], qz=[2], qw=[3].

	  Not every quaternion describes a rotation.
	  Only quaternions of the form:

	  qx = x * sin(phi/2) , which is the imaginary part i

	  qy = y * sin(phi/2)  , which is the imaginary part j

	  qz = z * sin(phi/2)  , which is the imaginary part k

	  qw = cos(phi/2),     , which is the real part

	  where (x, y, z) is the normalized direction vector of the axis
	  and phi is the angle of rtoation.

	  Some usefull quaternions:
	  x 	       y 	        z 	       w          Description
	  0 	       0 	        0 	       1          Identity quaternion,
													  no rotation
	  1 	       0 	        0 	       0 	      180' turn around X axis
	  0 	       1 	        0 	       0 	      180' turn around Y axis
	  0 	       0 	        1 	       0 	      180' turn around Z axis
	  sqrt(0.5)    0 	        0 	       sqrt(0.5)  90' rotation around X axis
	  0 	       sqrt(0.5) 	0 	       sqrt(0.5)  90' rotation around Y axis
	  0 	       0 	        sqrt(0.5)  sqrt(0.5)  90' rotation around Z axis
	 -sqrt(0.5)    0 	        0 	       sqrt(0.5)  -90' rotation around X axis
	  0           -sqrt(0.5)    0 	       sqrt(0.5)  -90' rotation around Y axis
	  0 	       0 	       -sqrt(0.5)  sqrt(0.5)  -90' rotation around Z axis

	  @author grest 06/2003
  */
template <typename TYPE>
class TQuaternion : public TMatrix<TYPE,4,1>
{
public:
	typedef TMatrix<TYPE,4,1> Base;
	typedef TMatrix<TYPE,4,4> Mat;
	typedef TMatrix<TYPE,3,1> Vec;
	typedef TMatrix<TYPE,3,3> Rot;

	using Base::val;

public:
	static const TQuaternion IDENTITY;

public:
	inline ~TQuaternion();

	inline TQuaternion();

	inline TQuaternion(const Base& q);

	inline TQuaternion(TYPE i, TYPE j, TYPE k,	TYPE r);

	inline TQuaternion(const Rot& R);

	inline void SetIdentity();

	/**
	* Sets all parts of quaternion in mathematical order,
	* real part first, then 1.,2. and 3. imaginary part
	*/
	inline void SetQuaternion(TYPE real, TYPE i, TYPE j, TYPE k);

	/** returns the Inverse rotation TQuaternion
	*/
	inline TQuaternion<TYPE> Inverse() const;

	/** inverts this, by changing the rotation axis by *=-1
	*/
	inline void Invert();

	/** makes TQuaternion-representation uniqe, ensuring vector lies
	in upper (by real part) hemisphere. (inplace)
	*/
	inline void MakeUnique();

	/** quaternion multiplication: this= this * quat
	*/
	inline void Mult(const TQuaternion<TYPE> &quat);

	/** quaternion multiplication: res = this * arg
	*/
	inline void Mult(const TQuaternion<TYPE> &arg,
		TQuaternion<TYPE> &res) const;

	/** quaternion multiplication: this = quat * this
	*/
	inline void MultLeft(const TQuaternion<TYPE> &quat);

	/** rotates the given Vector qith the quaternion ( q v q* )
	the resulting vector is given in res
	@returns 0 in case of no error
	@author Daniel Grest, June 2003
	*/
	inline int MultVec(const Vec &vec, Vec &res) const;

	/* rets result from MultVec()
	@author herzog 2005-07-15 */
	inline  Vec MultVec(const Vec &vec) const;

	/** Computes this^(scale), which scales the angle from axis/angle-
	representation with 'scale'. This is the same like inter-/extra-
	polating between (0,0,0,1)-quaternion and this!
	*/
	TQuaternion<TYPE> Power(const TYPE & scale) const;

	/** Linear interpolation between this and given quaternion.
	@note Quaternions are assumed to be unit quaternions! */
	TQuaternion<TYPE> InterpolateLinear(const TQuaternion<TYPE> &to, const TYPE & t) const;

	/** Spherical interpolation between this and given quaternion.
	@note Quaternions are assumed to be unit quaternions! */
	TQuaternion<TYPE> Interpolate(const TQuaternion<TYPE> &to, const TYPE & t) const;

	/** sets the quaternion with given rotation axis and angle (in rad)
	@returns 0 in case of no error
	@author Daniel Grest, June 2003
	*/
	inline void SetValueAsAxisRad(const Vec &axis, TYPE angle);

	/** sets the quaternion with given rotation axis and angle (in rad)
	@returns 0 in case of no error
	@author Daniel Grest, June 2003
	*/

	inline void SetValueAsAxisRad(TYPE axisX, TYPE axisY, TYPE axisZ, TYPE angle);

	/** Returns matrix which expresses (left) quaternion multiplication.
	* A quaternions stored in an object can be understood as a 4-vector
	* q =(ix iy iz r)^T.
	* Left multiplying a quaternion q2 with a quaternion q1 can be
	* expressed in terms of matrix multiplication as
	* qRes = q1*q2 = M(q1)*q2.
	* This method returns the matrix M(*this).
	*/
	Mat GetQuaternionMultMatrixLeft() const;

	/** Returns matrix which expresses right quaternion multiplication.
	* Right multiplying a quaternion q1 with quaternion q2 can be
	* expressed as qRes = q1*q2 = N(q2)*q1.
	* This method returns the matrix N(*this).
	*/
	Mat GetQuaternionMultMatrixRight() const;

	/** Returns rotation axis. See GetRotationAxisAngle(). */
	Vec GetRotationAxis() const;

	/** Returns rotation angle notation in radians.
		Note that the returned angle resides in the interval [0,PI)! */
	TYPE GetRotationAngle() const;

	/** Returns rotation in axis and angle notation (angle in radians).
		Note that the returned angle resides in the interval [0,PI) and
		the axis points into the according direction!
		@author woelk 12 2002 */
	int GetAxisAngle(Vec& axis, TYPE& angle) const;

	/** Sets quaternion as concatenated rotations around
		x,y,z-axis; this = q_x * q_y * q_z (BIAS-RMatrix conform)
		@author herzog 2005-07-19 */
	int SetXYZ(TYPE radX, TYPE radY, TYPE radZ);

	/** Sets quaternion as concatenated rotations around
		x,y,z-axis; this = q_z * q_y * q_x (BIAS-RMatrix conform)
		@author herzog 2005-07-19 */
	int SetZYX(TYPE radX, TYPE radY, TYPE radZ);


	/** Scales quaternion to unit length, i.e. norm equals 1. */
	inline void Normalize() {
		const TYPE n = norm(*this);
		if (ISZERO(n))
			*this = IDENTITY;
		else
			*this *= TYPE(1) / n;
	}
	inline bool IsNormalized() const {
		return ISEQUAL(norm(*this), TYPE(1));
	}

	/** assignment operator
		@author grest 06 2003 */
	TQuaternion<TYPE>& operator=(const TQuaternion<TYPE>& vec);

	/** Enforces rigid coupling constraint for this and the other given unit
		quaternion (i.e. equal rotation angle resp. scalar part of both).
		Computes direct simple interpolation of both unit quaternion where
		the scalar parts of the resulting unit quaternion are identical.
		This solution can be used as initial guess for numerical refinement.
		@author esquivel 02/2012 */
	void EnforceRigidCouplingConstraint(TQuaternion<TYPE> &other);

	/** Enforces rigid coupling constraint for all quaternion in given vector.
		@author esquivel 02/2012 */
	static void EnforceRigidCouplingConstraint(std::vector< TQuaternion<TYPE> > &quats);
}; // class
template <typename TYPE> const TQuaternion<TYPE> TQuaternion<TYPE>::IDENTITY(0,0,0,1);
/*----------------------------------------------------------------*/
typedef TQuaternion<float> QuaternionF;
typedef TQuaternion<double> QuaternionD;
/*----------------------------------------------------------------*/


// generic rotation matrix struct
#define ROTATION_MATRIX_EPSILON DBL_EPSILON

/** @class TRMatrixBase
	@ingroup g_geometry
	@brief 3D rotation matrix
	@author frahm, woelk **/
template <typename TYPE>
class TRMatrixBase : public TMatrix<TYPE,3,3>
{
public:
	typedef TMatrix<TYPE,3,3> Base;
	typedef Base Mat;
	typedef TMatrix<TYPE,3,1> Vec;
	typedef TQuaternion<TYPE> Quat;
	typedef typename Base::Base BaseBase;

	using Base::val;

public:
	/** @brief Constructor setting matrix to identity */
	inline TRMatrixBase();
	inline TRMatrixBase(const BaseBase& rhs) : Base(rhs) {}
	template <typename T> inline TRMatrixBase(const cv::Matx<T,3,3>& rhs) : Base(rhs) {}

	/** @brief Copy constructor from rotation matrix */
	inline TRMatrixBase(const TRMatrixBase& r);

	/** @brief Copy constructor from 3x3 matrix
		@attention Orthonormality of matrix is enforced automatically! */
	inline TRMatrixBase(const Mat& mat);

	/** @brief Initialization from parametrized rotation (axis-angle) */
	inline TRMatrixBase(const Vec& rot);

	/** @brief Initialization from rotation axis w and angle phi (in rad) using Rodrigues' formula */
	inline TRMatrixBase(const Vec& w, const TYPE phi);

	/** @brief Initialization from quaternion */
	inline TRMatrixBase(const Quat& q);

	/** @brief Initialization with the rotation from roll/pitch/yaw (in rad) */
	inline TRMatrixBase(TYPE roll, TYPE pitch, TYPE yaw);

	inline ~TRMatrixBase();

	template <typename T> inline TRMatrixBase& operator = (const cv::Matx<T,3,3>& rhs) { BaseBase::operator = (rhs); return *this; }
	inline TRMatrixBase& operator = (const cv::Mat& rhs) { BaseBase::operator = (rhs); return *this; }
	#ifdef _USE_EIGEN
	inline TRMatrixBase& operator = (const typename Base::EMat& rhs) { Base::operator = (rhs); return *this; }
	#endif

	/** @brief Set Euler angles (in rad) in order XYZ

		Set angles either in order 1. z, 2. y, 3. x and fixed axes
		or in order 1. x, 2. y, 3. z and moving axes.
		Angles are measured as in mathematics (counter-clockwise), i.e.
		Set(x, 0, 0) results in the following matrix:

		| 1  0        0      |
		| 0  cos(x)  -sin(x) |
		| 0  sin(x)   cos(x) |

		(*this) = Rx*Ry*Rz =

		|  c(y)c(z)               -c(y)s(z)                s(y)     |
		|  c(z)s(x)s(y)+c(x)s(z)   c(x)c(z)-s(x)s(y)s(z)  -c(y)s(x) |
		| -c(x)c(z)s(y)+s(x)s(z)   c(z)s(x)+c(x)s(y)s(z)   c(x)c(y) |

		with s(g) = sin(g), c(g) = cos(g), x = PhiX, y = PhiY and z = PhiZ

		@author frahm, woelk */
	void SetXYZ(TYPE PhiX, TYPE PhiY, TYPE PhiZ);


	/** @brief Set Euler angles (in rad) in order XYZ from vector */
	inline void SetXYZ(const Vec& r)
	{ SetXYZ(r[0], r[1], r[2]); }

	/** @brief Set Euler angles (in rad) in order ZYX

		Set angles either in order 1. z, 2. y, 3. x and moving axes
		or in order 1. x, 2. y, 3. z and fixed axes.
		Angles are measured as in mathematics (counter-clockwise), i.e.
		Set(x, 0, 0) results in the following matrix:

		| 1  0        0      |
		| 0  cos(x)  -sin(x) |
		| 0  sin(x)   cos(x) |

		(*this) = Rz*Ry*Rx =

		|  c(y)c(z)  s(x)s(y)c(z)-c(x)s(z)  c(x)s(y)c(z)+s(x)s(z) |
		|  c(y)s(z)  s(x)s(y)s(z)+c(x)c(z)  c(x)s(y)s(z)-s(x)s(z) |
		| -s(y)      s(x)c(y)               c(x)c(y)              |

		with s(g) = sin(g), c(g) = cos(g), x = PhiX, y = PhiY and z = PhiZ

		@author frahm, woelk */
	void SetZYX(TYPE PhiX, TYPE PhiY, TYPE PhiZ);

	/** @brief Set Euler angles (in rad) in order ZYX from vector */
	inline void SetZYX(const Vec& r)
	{ SetZYX(r[0], r[1], r[2]); }

	/** @brief Set Euler angles (in rad) in order ZXY

		Set angles either in order 1. z, 2. x, 3. y and moving axes
		or in order 1. y, 2. x, 3. z and fixed axes.
		Angles are measured as in mathematics (counter-clockwise), i.e.
		Set(x, 0, 0) results in the following matrix:

		| 1  0        0      |
		| 0  cos(x)  -sin(x) |
		| 0  sin(x)   cos(x) |

		(*this) = Rz*Rx*Ry =

		| c(y)c(z)-s(x)s(y)s(z), c(x)s(z), s(y)c(z)+s(x)c(y)s(z) |
		| s(z)c(y)+s(x)s(y)c(z), c(x)c(z),  s(y)*s(z)+s(x)-c(y)c(z) |
		| -c(x)s(y)             ,  s(x)  ,  c(x)c(y)              |

		with s(g) = sin(g), c(g) = cos(g), x = PhiX, y = PhiY and z = PhiZ

		@author haase */
	void SetZXY(TYPE PhiX, TYPE PhiY, TYPE PhiZ);

	/** @brief Set Euler angles (in rad) in order ZXY from vector */
	inline void SetZXY(const Vec& r)
	{ SetZXY(r[0], r[1], r[2]); }

	 /** @brief Set Euler angles (in rad) in order YXZ

		 Set angles either in order 1. y, 2. x, 3. z and moving axes
		 or in order 1. z, 2. x, 3. y and fixed axes.
		 Angles are measured as in mathematics (counter-clockwise), i.e.
		 Set(x, 0, 0) results in the following matrix:

		 | 1  0        0      |
		 | 0  cos(x)  sin(x) |
		 | 0  -sin(x)   cos(x) |

		 (*this) = Rz*Rx*Ry =

		 | c(y)c(z)+s(x)s(y)s(z), c(x)s(z), -s(y)c(z)+s(x)c(y)s(z) |
		 | -s(z)c(y)+s(x)s(y)c(z), c(x)c(z),  -s(y)*-s(z)+s(x)c(y)c(z) |
		 | c(x)s(y)             ,  -s(x)  ,  c(x)c(y)              |

		 with s(g) = sin(g), c(g) = cos(g), x = PhiX, y = PhiY and z = PhiZ

		 @author haase */
	void SetYXZ(TYPE PhiZ, TYPE PhiX, TYPE PhiY);

	/** @brief Set Euler angles (in rad) in order YXZ from vector */
	inline void SetYXZ(const Vec& r)
	{ SetYXZ(r[0], r[1], r[2]); }

	/** @brief Set from rotation axis w and angle phi (in rad)
		@param w Axis vector w will be normalized to length 1, so we need
				 |w|>1e-6 if phi != 0, otherwise an exception is thrown
		@param phi Rotation angle is given in radians
		@author evers, woelk */
	void Set(const Vec& w, TYPE phi);

	/** set this matrix from 3 vectors each representing a column*/
	void SetFromColumnVectors(const Vec& v0,
							  const Vec& v1,
							  const Vec& v2);

	/** set this matrix from 3 vectors, each representing a row */
	void SetFromRowVectors(const Vec& v0,
						   const Vec& v1,
						   const Vec& v2);

	/** @brief Set from rotation axis * angle (modified Rodrigues vector)
		@author evers */
	void SetFromAxisAngle(const Vec& w);

	/* @brief Set rotation matrix from an orthogonal basis given in world
	   coordinate system (WCS)

	   As R' is expressing the transformation from WCS to ICS (image
	   coordinate system), R is ICS to WCS and hereby represents the ICS
	   base vectors expressed in WCS. These are the *rows* of R because
	   the transformation is the scalar product.

	   Assume xxx,yyy,zzz are right-hand orthogonal (RHS), e.g. the orthogonal
	   image coordinate system (ICS) base vectors expressed in world
	   coordinates (WCS).
	   (Inverse doesn't make sense because base would be identity then).
	   Normal vectors of length 1 are computed.

	   @todo Warning if (xxx,yyy,zzz) are not an orthogonal basis!

	   @author jw 09/2003, added exception */
	void SetFromOrthogonalBasis(const Vec &xxx, const Vec &yyy, const Vec &zzz);

	/** @brief Set rotation matrix from two vectors from an orthonormal basis
		@param xh represents the first base vector and is left unchanged
		@param vy should be orthogonal to xh, it is orthogonalized otherwise

		You can think of this routine as computing R from an image plane
		given by two (usually orthogonal) base vectors.
		If the given base vectors are not orthogonal, xh is kept and yv is
		orthogonalized appropriately.

		@author jw */
	void SetFromHV(const Vec& xh, const Vec& vy);

	/** @brief Calculates quaternion representation for this rotation matrix
		@attention Scalar part of quaternion will always be non-negative
				   for sake of uniqueness of the resulting quaternion!
		@author woelk 12/2003
		@author esquivel 03/2011 (changed algorithm, bugfix) */
	void GetQuaternion(Quat& q) const;
	inline Quat GetQuaternion() const { Quat q; GetQuaternion(q); return q; }

	/** @brief Set rotation matrix from a quaternion
		@author grest 06/2003 */
	void SetFromQuaternion(const Quat& q);

	/** @brief Set rotation matrix from direction and up vector
		@note This is openGL conform, similar to gluLookAt(), i.e. if
		direction is (0,0,-1) and up is (0,1,0) the resulting matrix is identity.
		@author grest 12/2005 */
	void SetFromDirUpGL(const Vec& viewDir, const Vec& viewUp);

	/** @brief Set rotation matrix from direction and up vector */
	void SetFromDirUp(const Vec& viewDir, const Vec& viewUp);

	/** @brief Set rotation matrix from an eye point and a look-at target point and the up vector */
	void LookAt(const Vec& from, const Vec& to, const Vec& up);

	// get parametrized rotation (axis-angle) from the rotation matrix
	inline void SetRotationAxisAngle(const Vec& rot);

	// modify the rotation matrix by the given parametrized delta rotation (axis-angle)
	inline void Apply(const Vec& delta);

	// set rotation matrix to the given parametrized rotation (axis-angle)
	inline Vec GetRotationAxisAngle() const;

	/** @brief Calculates angle and rotation axis representation for
			   this rotation matrix
		@param angle Rotation angle is returned in radians
		@author woelk 12/2003 */
	void GetRotationAxisAngle(Vec& axis, TYPE& angle) const;

	/** @brief Interface for axis component of GetRotationAxisAngle() */
	Vec GetRotationAxis() const;

	/** @brief Interface for angle component of GetRotationAxisAngle() */
	TYPE GetRotationAngle() const;

	/** @brief Get Euler angles for this rotation matrix in order XYZ
		@attention Representation is not unique and has singularities at
		+-pi/2. Assume for example (*this) = Rx * Ry * Rz, then we have
		either Euler angles in order 1. x, 2. y, 3. z and moving axes
		or in order 1. z, 2. y, 3. x and fixed axes.
		@author woelk 01/2003 */
	int GetRotationAnglesXYZ(TYPE& PhiX, TYPE& PhiY, TYPE& PhiZ) const;

	/** @brief Get Euler angles for this rotation matrix in order XYZ
		@see GetRotationAnglesXYZ(TYPE&, TYPE&, TYPE&) */
	inline int GetRotationAnglesXYZ(Vec& r) const
	{ return GetRotationAnglesXYZ(r[0], r[1], r[2]); }

	/** @brief Get Euler angles for this rotation matrix in order YZX
		@attention Representation is not unique and has singularities at
		+-pi/2. Assume for example (*this) = Rz * Ry * Rx, then we have
		either Euler angles in order 1. x, 2. y, 3. z and fixed axes
		or in order 1. z, 2. y, 3. x and moving axes.
		@author woelk 01 2003 */
	int GetRotationAnglesZYX(TYPE& PhiX, TYPE& PhiY, TYPE& PhiZ) const;

	/** @brief Get Euler angles for this rotation matrix in order ZYX
		@see GetRotationAnglesZYX(TYPE&, TYPE&, TYPE&) */
	inline int GetRotationAnglesZYX(Vec& r) const
	{ return GetRotationAnglesZYX(r[0], r[1], r[2]); }

	/** @brief Get Euler angles for this rotation matrix in order ZXY
		@attention Representation is not unique and has singularities at
		+-pi/2. Rotation order ZXY refers to rotation around point/vector!
		@author haase 2007 */
	int GetRotationAnglesZXY(TYPE& PhiZ, TYPE& PhiX, TYPE& PhiY) const;

	/** @brief Get Euler angles for this rotation matrix in order ZXY
		@see GetRotationAnglesZXY(TYPE&, TYPE&, TYPE&) */
	inline int GetRotationAnglesZXY(Vec& r) const
	{ return GetRotationAnglesZXY(r[2], r[0], r[1]); }

	/** @brief Get Euler angles for this rotation matrix in order YXZ
		@attention Representation is not unique and has singularities at
		+-pi/2. Rotation order YXZ refers to rotation with moving axes!
		@author haase 2007 */
	int GetRotationAnglesYXZ(TYPE& PhiY, TYPE& PhiX, TYPE& PhiZ) const;

	/** @brief Get Euler angles for this rotation matrix in order YXZ
		@see GetRotationAnglesYXZ(TYPE&, TYPE&, TYPE&) */
	inline int GetRotationAnglesYXZ(Vec& r) const
	{ return GetRotationAnglesYXZ(r[1], r[0], r[2]); }

	/** @brief Check that the matrix is a valid rotation matrix (orthogonal) */
	inline bool IsValid() const {
		// the trace should be three and the determinant should be one
		#if 1
		return (ISEQUAL((float)cv::determinant(*this), 1.f) && ISEQUAL((float)cv::trace((*this)*(*this).t()), 3.f));
		#else
		return (ISEQUAL((TYPE)cv::determinant(*this), TYPE(1)) && ISEQUAL((TYPE)cv::trace((*this)*(*this).t()), TYPE(3)));
		#endif
	}

	/** @brief Check if this is a rotation matrix, i.e. if the determinant
			   is +1 and the columns are orthonormal
		@param eps Numerical limit for constraint evaluation
		@param verbose Show reason in case of failure */
	bool Check(const TYPE eps = std::numeric_limits<TYPE>::epsilon(),
			   int verbose = 0) const;

	/** @brief Enforce orthogonality constraint on rotation matrix and
			   sets determinant to +1 */
	void EnforceOrthogonality();
}; // class
/*----------------------------------------------------------------*/
typedef TRMatrixBase<float> RMatrixBaseF;
typedef TRMatrixBase<double> RMatrixBaseD;
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#include "Rotation.inl"

#endif // __SEACAVE_ROTATION_H__

