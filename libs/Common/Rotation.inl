// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////


// G L O B A L S ///////////////////////////////////////////////////


namespace SEACAVE {

// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE>
inline TQuaternion<TYPE>::~TQuaternion()
{}

template <typename TYPE>
inline TQuaternion<TYPE>::TQuaternion()
	: Base()
{}

template <typename TYPE>
inline TQuaternion<TYPE>::TQuaternion(const Base& q)
	: Base(q)
{}

template <typename TYPE>
inline TQuaternion<TYPE>::TQuaternion(TYPE i, TYPE j, TYPE k, TYPE r)
	: Base(cv::Vec<TYPE,4>(i, j, k, r))
{}

template <typename TYPE>
inline TQuaternion<TYPE>::TQuaternion(const Rot& R)
	: Base(((const TRMatrixBase<TYPE>&)R).GetQuaternion())
{}


template <typename TYPE>
inline void TQuaternion<TYPE>::Invert()
{
	Base::operator()(0) *=-1;
	Base::operator()(1) *=-1;
	Base::operator()(2) *=-1;
}

template <typename TYPE>
inline TQuaternion<TYPE> TQuaternion<TYPE>::Inverse() const
{
	TQuaternion<TYPE> res(*this);
	res.Invert();
	return res;
}

template <typename TYPE>
inline void TQuaternion<TYPE>::MakeUnique()
{
	//project quaternion to upper hemisphere (first ima.part >= 0)
	TYPE *qThis = this->val;
	if ((qThis[3] <= QUATERNION_EPSILON)
		&&(qThis[3] >= -QUATERNION_EPSILON)) {
			if ((qThis[1] <= QUATERNION_EPSILON)
				&&(qThis[1] >= -QUATERNION_EPSILON)) {
					if ((qThis[2] <= QUATERNION_EPSILON)
						&&(qThis[2] >= -QUATERNION_EPSILON)) {
							if (qThis[0] < 0.0) (*this) *= -1.0;
					} else if (qThis[2] < 0.0) {
						(*this) *= -1.0;
					}
			} else if (qThis[1] < 0.0) {
				(*this) *= -1.0;
			}
	} else if (qThis[3] < 0.0) {
		(*this) *= -1.0;
	}
}

template <typename TYPE>
inline void TQuaternion<TYPE>::Mult(const TQuaternion<TYPE> &quat)
{
	TYPE s1, s2, s3, s4, s5, s6, s7, s8, s9, t;
	TYPE *qThis = this->val;

	s1 = (qThis[2] - qThis[1]) * (quat[1] - quat[2]);
	s2 = (qThis[3] + qThis[0]) * (quat[3] + quat[0]);
	s3 = (qThis[3] - qThis[0]) * (quat[1] + quat[2]);
	s4 = (qThis[2] + qThis[1]) * (quat[3] - quat[0]);
	s5 = (qThis[2] - qThis[0]) * (quat[0] - quat[1]);
	s6 = (qThis[2] + qThis[0]) * (quat[0] + quat[1]);
	s7 = (qThis[3] + qThis[1]) * (quat[3] - quat[2]);
	s8 = (qThis[3] - qThis[1]) * (quat[3] + quat[2]);

	s9 = s6 + s7 + s8;

	t  = (s5 + s9) / 2.0f;

	qThis[3] = s1 + t - s6;
	qThis[0] = s2 + t - s9;
	qThis[1] = s3 + t - s8;
	qThis[2] = s4 + t - s7;
}

template <typename TYPE>
inline void TQuaternion<TYPE>::Mult(const TQuaternion<TYPE> &arg, TQuaternion<TYPE> &res) const
{
	res = *this;
	res.Mult(arg);
}

template <typename TYPE>
inline void TQuaternion<TYPE>::MultLeft(const TQuaternion<TYPE> &quatLeft)
{
	TYPE s1, s2, s3, s4, s5, s6, s7, s8, s9, t;
	TYPE *qThis = this->val;

	s1 = (quatLeft[2] - quatLeft[1]) * (qThis[1] - qThis[2]);
	s2 = (quatLeft[3] + quatLeft[0]) * (qThis[3] + qThis[0]);
	s3 = (quatLeft[3] - quatLeft[0]) * (qThis[1] + qThis[2]);
	s4 = (quatLeft[2] + quatLeft[1]) * (qThis[3] - qThis[0]);
	s5 = (quatLeft[2] - quatLeft[0]) * (qThis[0] - qThis[1]);
	s6 = (quatLeft[2] + quatLeft[0]) * (qThis[0] + qThis[1]);
	s7 = (quatLeft[3] + quatLeft[1]) * (qThis[3] - qThis[2]);
	s8 = (quatLeft[3] - quatLeft[1]) * (qThis[3] + qThis[2]);

	s9 = s6 + s7 + s8;

	t  = (s5 + s9) / 2.0f;

	qThis[3] = s1 + t - s6;
	qThis[0] = s2 + t - s9;
	qThis[1] = s3 + t - s8;
	qThis[2] = s4 + t - s7;
}

/** rotates the given Vector with the quaternion ( q v q* )
the resulting vector is given in res
@returns 0 in case of no error
@author Daniel Grest, June 2003
*/
template <typename TYPE>
inline int TQuaternion<TYPE>::MultVec(const Vec &vec, Vec &res) const {
	TYPE rx,ry,rz;
	TYPE Vx= vec[0], Vy=vec[1], Vz=vec[2];
	const TYPE *q = this->val;
	{
		TYPE QwQx, QwQy, QwQz, QxQy, QxQz, QyQz;
		QwQx = q[3] * q[0]; QwQy = q[3] * q[1]; QwQz = q[3] * q[2];
		QxQy = q[0] * q[1]; QxQz = q[0] * q[2]; QyQz = q[1] * q[2];
		rx = 2* (Vy * (-QwQz + QxQy) + Vz *( QwQy + QxQz));
		ry = 2* (Vx * ( QwQz + QxQy) + Vz *(-QwQx + QyQz));
		rz = 2* (Vx * (-QwQy + QxQz) + Vy *( QwQx + QyQz));
	}
	TYPE QwQw, QxQx, QyQy, QzQz;
	QwQw= q[3]*q[3]; QxQx = q[0]*q[0]; QyQy = q[1]*q[1]; QzQz= q[2]*q[2];
	rx+= Vx * (QwQw + QxQx - QyQy - QzQz);
	ry+= Vy * (QwQw - QxQx + QyQy - QzQz);
	rz+= Vz * (QwQw - QxQx - QyQy + QzQz);
	res[0]=rx; res[1]=ry; res[2]=rz;
	return 0;
}

template <typename TYPE>
inline typename TQuaternion<TYPE>::Vec TQuaternion<TYPE>::MultVec(const Vec &vec) const
{
	Vec r;
	MultVec(vec,r);
	return r;
}

template <typename TYPE>
inline void TQuaternion<TYPE>::SetIdentity()
{
	Base::operator()(0) = Base::operator()(1) = Base::operator()(2) = 0;
	Base::operator()(3) = 1;
}

template <typename TYPE>
inline void TQuaternion<TYPE>::SetQuaternion(TYPE real, TYPE i,
	TYPE j, TYPE k)
{
	//Components are stored in Vec4 in order imaginary part 1,2,3 and real part
	this->Set(i,j,k,real);
}

template <typename TYPE>
inline void TQuaternion<TYPE>::SetValueAsAxisRad(const Vec &axis, TYPE angle)
{
	SetValueAsAxisRad(axis[0], axis[1], axis[2], angle);
}

template <typename TYPE>
inline void TQuaternion<TYPE>::SetValueAsAxisRad(TYPE x, TYPE y, TYPE z, TYPE w)
{
	TYPE rTmp = TYPE(sqrt(x * x + y * y + z * z));

	if(rTmp > QUATERNION_EPSILON)
	{
		rTmp = TYPE(sin(w / 2.0f)) / rTmp;

		Base::operator()(0) = x * rTmp;
		Base::operator()(1) = y * rTmp;
		Base::operator()(2) = z * rTmp;
		Base::operator()(3) = TYPE(cos(w / 2.0f));
	}
	else
	{
		SetIdentity();
	}
}

template <typename TYPE>
typename TQuaternion<TYPE>::Mat TQuaternion<TYPE>::GetQuaternionMultMatrixLeft() const
{
	Mat res;

	res(0,0) = (*this)[3];
	res(0,1) = -(*this)[2];
	res(0,2) = (*this)[1];
	res(0,3) = (*this)[0];

	res(1,0) = (*this)[2];
	res(1,1) = (*this)[3];
	res(1,2) = -(*this)[0];
	res(1,3) = (*this)[1];

	res(2,0) = -(*this)[1];
	res(2,1) = (*this)[0];
	res(2,2) = (*this)[3];
	res(2,3) = (*this)[2];

	res(3,0) = -(*this)[0];
	res(3,1) = -(*this)[1];
	res(3,2) = -(*this)[2];
	res(3,3) = (*this)[3];

	return res;
}

template <typename TYPE>
typename TQuaternion<TYPE>::Mat TQuaternion<TYPE>::GetQuaternionMultMatrixRight() const
{
	Mat res;

	res(0,0) = (*this)[3];
	res(0,1) = (*this)[2];
	res(0,2) = -(*this)[1];
	res(0,3) = (*this)[0];

	res(1,0) = -(*this)[2];
	res(1,1) = (*this)[3];
	res(1,2) = (*this)[0];
	res(1,3) = (*this)[1];

	res(2,0) = (*this)[1];
	res(2,1) = -(*this)[0];
	res(2,2) = (*this)[3];
	res(2,3) = (*this)[2];

	res(3,0) = -(*this)[0];
	res(3,1) = -(*this)[1];
	res(3,2) = -(*this)[2];
	res(3,3) = (*this)[3];

	return res;
}

template <typename TYPE>
int TQuaternion<TYPE>::GetAxisAngle(Vec& axis, TYPE& angle) const
{
	// nicked from OpenSG
	int res=0;
	Vec q(cv::Vec<TYPE,3>((*this)[0], (*this)[1], (*this)[2]));
	TYPE len = TYPE(norm(q));

	if(len > QUATERNION_EPSILON) {
		q *= TYPE(1.0) / len;
		axis[0]  = q[0];
		axis[1]  = q[1];
		axis[2]  = q[2];
		angle = TYPE(2.0 * acos((*this)[3]));
	} else {
		axis[0] = 0.0;
		axis[1] = 0.0;
		axis[2] = 1.0;
		angle = 0.0;
	}
	return res;
}

template <typename TYPE>
typename TQuaternion<TYPE>::Vec TQuaternion<TYPE>::GetRotationAxis() const
{
	Vec r(cv::Vec<TYPE,3>((*this)[0], (*this)[1], (*this)[2]));
	TYPE len = TYPE(norm(r));
	if (len > QUATERNION_EPSILON)
		return r * (1.0 / len);
	else
		return Vec(cv::Vec<TYPE,3>(0,0,1));
}

template <typename TYPE>
TYPE TQuaternion<TYPE>::GetRotationAngle() const
{
	const TYPE angle = TYPE(2 * acos((*this)[3]));
	return std::isnan(angle) && ABS(ABS((*this)[3]) - TYPE(1)) < QUATERNION_EPSILON ?
		TYPE(0) :
		angle;
}

template <typename TYPE>
TQuaternion<TYPE> TQuaternion<TYPE>::Power(const TYPE & scale) const
{
	TQuaternion<TYPE> res = *this;
	if ((scale < 1e-7) && (scale > -1e-7)) {
		//scale near zero -> return identity
		res.SetIdentity();
	} else if ((res[3] > 0.999999) || (res[3] < -0.999999)) {
		//this is near identity (numerical unstable) -> return identity
		//res = *this
	} else {
		//else use slerp interpolation between identity and this with scale
		TYPE temp1 = sin(scale * acos(res[3]))/sin(acos(res[3]));
		TYPE res3 = res[3];
		res *= temp1;
		res[3] += TYPE(sin((1.0-scale) * acos(res3))/sin(acos(res3)));
	}
	return res;
}

template <typename TYPE>
TQuaternion<TYPE> TQuaternion<TYPE>::InterpolateLinear(const TQuaternion<TYPE> &to, const TYPE & t) const
{
	BIASASSERT(t >= 0.0 && t <= 1.0);
	TQuaternion<TYPE> p = *this;
	TQuaternion<TYPE> q = to;
	//p.Normalize();
	//q.Normalize();
	if (t < 1e-7) {
		//t near zero -> return p
		return p;
	} else if (1.0-t < 1e-7) {
		//t near one -> return q
		return q;
	}
	//else use linear interpolation between this and q
	TQuaternion<TYPE> res;
	const TYPE cosPhi = (TYPE)p.ScalarProduct(q);
	const TYPE a = (TYPE)1.0 - t;
	const TYPE b = (cosPhi < 0 ? -t : t);
	res[0] = a*p[0] + b*q[0];
	res[1] = a*p[1] + b*q[1];
	res[2] = a*p[2] + b*q[2];
	res[3] = a*p[3] + b*q[3];
	res.Normalize();
	return res;
}

template <typename TYPE>
TQuaternion<TYPE> TQuaternion<TYPE>::
	Interpolate(const TQuaternion<TYPE> &to, const TYPE & t) const
{
	BIASASSERT(t >= 0.0 && t <= 1.0);
	TQuaternion<TYPE> p = *this;
	TQuaternion<TYPE> q = to;
	//p.Normalize();
	//q.Normalize();
	if (t < 1e-7) {
		//t near zero -> return p
		return p;
	} else if (1.0-t < 1e-7) {
		//t near one -> return q
		return q;
	}
	TQuaternion<TYPE> res;
	TYPE a, b;
	TYPE cosPhi = (TYPE)p.ScalarProduct(q);
	//adjust angle if neccessary
	if (cosPhi < 0) {
		q.MultiplyIP(-1.0);
		cosPhi = -cosPhi;
	}
	if (cosPhi > 0.9999 || cosPhi < -0.9999) {
		// p is very close to q, do linear interpolation instead of slerp
		a = (TYPE)1.0 - t;
		b = (cosPhi < 0 ? -t : t);
	} else {
		//else use slerp interpolation between p and q
		const TYPE phi = acos(cosPhi < 0 ? -cosPhi : cosPhi);
		const TYPE sinPhi = sin(phi);
		a = sin(((TYPE)1.0 - t) * phi) / sinPhi;
		b = sin(t * phi) / sinPhi;
		if (cosPhi < 0) b = -b;
	}
	res[0] = a*p[0] + b*q[0];
	res[1] = a*p[1] + b*q[1];
	res[2] = a*p[2] + b*q[2];
	res[3] = a*p[3] + b*q[3];
	res.Normalize();
	return res;
}

template <typename TYPE>
int TQuaternion<TYPE>::SetXYZ(TYPE radX, TYPE radY, TYPE radZ)
{
	TQuaternion<TYPE> q_x;
	TQuaternion<TYPE> q_y;
	TQuaternion<TYPE> q_z;

	q_x.SetValueAsAxisRad( 1.,0.,0.,  radX );
	q_y.SetValueAsAxisRad( 0.,1.,0.,  radY );
	q_z.SetValueAsAxisRad( 0.,0.,1.,  radZ );

	q_x.Mult(q_y);
	q_x.Mult(q_z);

	(*this) = q_x;

	return 0;
}

template <typename TYPE>
int TQuaternion<TYPE>::SetZYX(TYPE radX, TYPE radY, TYPE radZ)
{
	TQuaternion<TYPE> q_x;
	TQuaternion<TYPE> q_y;
	TQuaternion<TYPE> q_z;

	q_x.SetValueAsAxisRad( 1.,0.,0.,  radX );
	q_y.SetValueAsAxisRad( 0.,1.,0.,  radY );
	q_z.SetValueAsAxisRad( 0.,0.,1.,  radZ );

	q_z.Mult(q_y);
	q_z.Mult(q_x);

	(*this) = q_z;

	return 0;
}

template <typename TYPE>
TQuaternion<TYPE>& TQuaternion<TYPE>::operator=(const TQuaternion<TYPE>& vec)
{
	memcpy((void *)val, (void *)vec.val, sizeof(TQuaternion<TYPE>));
	return *this;
}

template <typename TYPE>
void TQuaternion<TYPE>::EnforceRigidCouplingConstraint(TQuaternion<TYPE> &other)
{
	// @todo check unit norm constraint of this and other quaternion!!

	// make this and other quaternion unique
	MakeUnique();
	other.MakeUnique();

	// get aliases for both quaternions
	TQuaternion<TYPE> &q0 = *this, &q1 = other;

	// interpolate equal angle
	q0[3] = q1[3] = TYPE(0.5) * (q0[3] + q1[3]);

	// re-enforce unit length constraint for q0 and q1
	TYPE q0norm = q0[0]*q0[0] + q0[1]*q0[1] + q0[2]*q0[2];
	TYPE q1norm = q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2];
	if (q0norm > TYPE(1e-8) && q1norm > TYPE(1e-8)) {
		TYPE qscale0 = (TYPE)sqrt(double((1-q0[3]*q0[3])/q0norm));
		TYPE qscale1 = (TYPE)sqrt(double((1-q1[3]*q1[3])/q1norm));
		for (int l = 0; l < 3; l++) {
			q0[l] *= qscale0;
			q1[l] *= qscale1;
		}
	}

	// @todo check unit norm constraint of resulting quaternions!!
}

template <typename TYPE>
void TQuaternion<TYPE>::EnforceRigidCouplingConstraint(std::vector<TQuaternion<TYPE> > &quats)
{
	// @todo check unit norm constraint of all quaternions!!

	// make all quaternions unique
	const int n = quats.size();
	for (int i = 0; i < n; i++)
		quats[i].MakeUnique();

	// interpolate equal angle
	double a = 0;
	for (int i = 0; i < n; i++)
		a += quats[i][3];
	a /= TYPE(n);
	for (int i = 0; i < n; i++)
		quats[i][3] = a;

	// re-enforce unit length constraint for all quaternions
	for (int i = 0; i < n; i++) {
		TQuaternion<TYPE> &q = quats[i];
		TYPE qnorm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2];
		if (qnorm > TYPE(1e-8)) {
			TYPE qscale = (TYPE)sqrt(double((1-q[3]*q[3])/qnorm));
			for (int l = 0; l < 3; l++)
				q[l] *= qscale;
		}
	}

	// @todo check unit norm constraint of resulting quaternions!!
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// accuracy for constraint check in Check() function calls
#define R_CONSTRAINT_ACCURACY 1e-5 //ZEROTOLERANCE<TYPE>()

// if rotation angle is below this value, assume zero rotation
#define MINIMUM_ROTATION 1e-7

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase()
  : Base(Base::IDENTITY)
{}

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase(const TRMatrixBase<TYPE>& r)
  : Base(r)
{
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
}

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase(const Mat& mat)
  : Base(mat)
{
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
}

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase(const Vec& w, TYPE phi)
{
	Set(w, phi);
}

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase(const Vec& rot)
{
	SetRotationAxisAngle(rot);
}

template <typename TYPE>
inline TRMatrixBase<TYPE>::TRMatrixBase(const Quat& q)
{
	SetFromQuaternion(q);
}

template <typename TYPE>
inline TRMatrixBase<TYPE>::~TRMatrixBase()
{}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetXYZ(TYPE PhiX, TYPE PhiY, TYPE PhiZ)
{
  TRMatrixBase<TYPE> Rx(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Ry(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Rz(TRMatrixBase<TYPE>::IDENTITY);

  /* set Rz, Ry, Rx as rotation matrices */
  Rz(0,0) = cos(PhiZ);
  Rz(0,1) = -sin(PhiZ);
  Rz(1,0) = -Rz(0,1);
  Rz(1,1) = Rz(0,0);
  //Rz(2,2) = 1;

  Ry(0,0) = cos(PhiY);
  Ry(0,2) = sin(PhiY);
  //Ry(1,1) = 1;
  Ry(2,0) = -Ry(0,2); //-sin(PhiY);
  Ry(2,2) = Ry(0,0); //cos(PhiY);

  //Rx(0,0) = 1;
  Rx(1,1) = cos(PhiX);
  Rx(1,2) = -sin(PhiX);
  Rx(2,1) = -Rx(1,2); //sin(PhiX);
  Rx(2,2) = Rx(1,1); //cos(PhiX);

//   Mat Result=Rx*Ry*Rz;
//   for (int i =0; i<9; i++)
//     this->val[i]=Result.GetData()[i];
  (*this) = Rx*Ry*Rz;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetZYX(TYPE PhiX, TYPE PhiY, TYPE PhiZ)
{
  TRMatrixBase<TYPE> Rx(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Ry(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Rz(TRMatrixBase<TYPE>::IDENTITY);

  /* set Rz, Ry, Rx as rotation matrices */
  Rz(0,0) = cos(PhiZ);
  Rz(0,1) = -sin(PhiZ);
  Rz(1,0) = -Rz(0,1);
  Rz(1,1) = Rz(0,0);
  //Rz(2,2) = 1;

  Ry(0,0) = cos(PhiY);
  Ry(0,2) = sin(PhiY);
  //Ry(1,1) = 1;
  Ry(2,0) = -Ry(0,2); //-sin(PhiY);
  Ry(2,2) = Ry(0,0); //cos(PhiY);

  //Rx(0,0) = 1;
  Rx(1,1) = cos(PhiX);
  Rx(1,2) = -sin(PhiX);
  Rx(2,1) = -Rx(1,2); //sin(PhiX);
  Rx(2,2) = Rx(1,1); //cos(PhiX);

  (*this) = Rz*Ry*Rx;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetYXZ(TYPE PhiY, TYPE PhiX, TYPE PhiZ)
{
  TRMatrixBase<TYPE> Rx(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Ry(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Rz(TRMatrixBase<TYPE>::IDENTITY);

  /* set Rz, Ry, Rx as rotation matrices */
  Rz(0,0) = cos(PhiZ);
  Rz(0,1) = -sin(PhiZ);
  Rz(1,0) = -Rz(0,1);
  Rz(1,1) = Rz(0,0);


  Ry(0,0) = cos(PhiY);
  Ry(0,2) = sin(PhiY);

  Ry(2,0) = -Ry(0,2); //-sin(PhiY);
  Ry(2,2) = Ry(0,0); //cos(PhiY);


  Rx(1,1) = cos(PhiX);
  Rx(1,2) = -sin(PhiX);
  Rx(2,1) = -Rx(1,2); //sin(PhiX);
  Rx(2,2) = Rx(1,1); //cos(PhiX);

  (*this) = Ry*Rx*Rz;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetZXY(TYPE PhiX, TYPE PhiY, TYPE PhiZ)
{
  TRMatrixBase<TYPE> Rx(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Ry(TRMatrixBase<TYPE>::IDENTITY);
  TRMatrixBase<TYPE> Rz(TRMatrixBase<TYPE>::IDENTITY);

  /* set Rz, Ry, Rx as rotation matrices */
  Rz(0,0) = cos(PhiZ);
  Rz(0,1) = -sin(PhiZ);
  Rz(1,0) = -Rz(0,1);
  Rz(1,1) = Rz(0,0);


  Ry(0,0) = cos(PhiY);
  Ry(0,2) = sin(PhiY);

  Ry(2,0) = -Ry(0,2); //-sin(PhiY);
  Ry(2,2) = Ry(0,0); //cos(PhiY);


  Rx(1,1) = cos(PhiX);
  Rx(1,2) = -sin(PhiX);
  Rx(2,1) = -Rx(1,2); //sin(PhiX);
  Rx(2,2) = Rx(1,1); //cos(PhiX);

  (*this) = Rz*Rx*Ry;
}



template <typename TYPE>
void TRMatrixBase<TYPE>::Set(const Vec& wa, TYPE phi)
{
  // zero rotation  results in identity matrix
  if (ISZERO(phi)) {
	(*this) = Base::IDENTITY;
	return;
  }

  const TYPE wnorm(norm(wa));
  if (wnorm < TYPE(1e-7)) {
	CPC_ERROR("Vector "<<wa<<" is close to zero (norm = "<<wnorm<<"), solution is instable!");
  }
  const Vec w(wa*(TYPE(1)/wnorm));

  Mat Omega;
  Omega(0,0) = 0.0;
  Omega(0,1) = -w[2];
  Omega(0,2) = w[1];
  Omega(1,0) = w[2];
  Omega(1,1) = 0.0;
  Omega(1,2) = -w[0];
  Omega(2,0) = -w[1] ;
  Omega(2,1) = w[0];
  Omega(2,2) = 0.0;

  const Mat Sin_O(Omega *  sin(phi));
  const Mat Cos_O_O((Omega * Omega) * (TYPE(1)-cos(phi)));
  (*this) = Base::IDENTITY + Sin_O + Cos_O_O;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromColumnVectors(const Vec& v0, const Vec& v1, const Vec& v2)
{
	val[0] = v0[0];
	val[1] = v1[0];
	val[2] = v2[0];
	val[3] = v0[1];
	val[4] = v1[1];
	val[5] = v2[1];
	val[6] = v0[2];
	val[7] = v1[2];
	val[8] = v2[2];
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromRowVectors(const Vec& v0, const Vec& v1, const Vec& v2)
{
	val[0] = v0[0];
	val[1] = v0[1];
	val[2] = v0[2];
	val[3] = v1[0];
	val[4] = v1[1];
	val[5] = v1[2];
	val[6] = v2[0];
	val[7] = v2[1];
	val[8] = v2[2];
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromAxisAngle(const Vec& w)
{
	#if 0
	const TYPE dAngle(norm(w));
	Set(w*INVERT(dAngle), dAngle);
	#else
	SetRotationAxisAngle(w);
	#endif
}


template <typename TYPE>
void TRMatrixBase<TYPE>::GetQuaternion(Quat& quat) const
{
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
	#if 1

  // Conversion method from OpenSG
  // @attention Removed old conversion in Linux build which had numerical
  //            issues for rotations close to 180 degree! (esquivel 03/2011)

  TYPE tr = Base::operator()(0,0) + Base::operator()(1,1) + Base::operator()(2,2);

  if (tr > 0.0)
  {
	// Use default formula if trace is large enough
	TYPE s = sqrt(tr + 1.0);

	quat[3] = s * 0.5;

	s *= 2.0;
	quat[0] = (Base::operator()(2,1) - Base::operator()(1,2)) / s;
	quat[1] = (Base::operator()(0,2) - Base::operator()(2,0)) / s;
	quat[2] = (Base::operator()(1,0) - Base::operator()(0,1)) / s;
  }
  else
  {
	// Use alternate formula using largest trace element
	TYPE s;
	unsigned i;
	unsigned j;
	unsigned k;

	// Find largest trace element i and its successors j, k
	if(Base::operator()(1,1) > Base::operator()(0,0))
	  i = 1;
	else
	  i = 0;
	if(Base::operator()(2,2) > Base::operator()(i,i))
	  i = 2;

	j = (i + 1) % 3;
	k = (j + 1) % 3;

	// Compute discriminator using largest trace element
	s = sqrt(Base::operator()(i,i) - (Base::operator()(j,j) + Base::operator()(k,k)) + 1.0);

	// Compute quaternion entries
	quat[i] = s * 0.5;
	s *= 2.0;

	quat[j] = (Base::operator()(i,j) + Base::operator()(j,i)) / s;
	quat[k] = (Base::operator()(i,k) + Base::operator()(k,i)) / s;

	quat[3] = (Base::operator()(k,j) - Base::operator()(j,k) ) / s;
  }

	#elif 1

  // Conversion method using derivation depending on max. denominator
  // @see http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29#Rotation_matrix_.E2.86.94_quaternion
  // @deprecated

  // Determine max. denominator
  TYPE d[4] = { 1 + Base::operator()(0,0) - Base::operator()(1,1) - Base::operator()(2,2),
				1 - Base::operator()(0,0) + Base::operator()(1,1) - Base::operator()(2,2),
				1 - Base::operator()(0,0) - Base::operator()(1,1) + Base::operator()(2,2),
				1 + Base::operator()(0,0) + Base::operator()(1,1) + Base::operator()(2,2) };
  int maxidx = 0;
  for (int i = 1; i < 4; i++) {
	if (d[maxidx] < d[i]) maxidx = i;
  }

  // Compute quaternion starting with max. denominator
  if (maxidx == 0) {
	quat[0]=0.5*sqrt(1 + Base::operator()(0,0) - Base::operator()(1,1) - Base::operator()(2,2));
	quat[3]=0.25*(Base::operator()(2,1)-Base::operator()(1,2))/quat[0];
	quat[1]=0.25*(Base::operator()(0,1)+Base::operator()(1,0))/quat[0];
	quat[2]=0.25*(Base::operator()(0,2)+Base::operator()(2,0))/quat[0];
  } else if (maxidx == 1) {
	quat[1]=0.5*sqrt(1 - Base::operator()(0,0) + Base::operator()(1,1) - Base::operator()(2,2));
	quat[3]=0.25*(Base::operator()(0,2)-Base::operator()(2,0))/quat[1];
	quat[2]=0.25*(Base::operator()(1,2)+Base::operator()(2,1))/quat[1];
	quat[0]=0.25*(Base::operator()(1,0)+Base::operator()(0,1))/quat[1];
  } else if (maxidx == 2) {
	quat[2]=0.5*sqrt(1 - Base::operator()(0,0) - Base::operator()(1,1) + Base::operator()(2,2));
	quat[3]=0.25*(Base::operator()(1,0)-Base::operator()(0,1))/quat[2];
	quat[0]=0.25*(Base::operator()(2,0)+Base::operator()(0,2))/quat[2];
	quat[1]=0.25*(Base::operator()(2,1)+Base::operator()(1,2))/quat[2];
  } else {
	quat[3]=0.5*sqrt(1 + Base::operator()(0,0) + Base::operator()(1,1) + Base::operator()(2,2));
	quat[0]=0.25*(Base::operator()(2,1)-Base::operator()(1,2))/quat[3];
	quat[1]=0.25*(Base::operator()(0,2)-Base::operator()(2,0))/quat[3];
	quat[2]=0.25*(Base::operator()(1,0)-Base::operator()(0,1))/quat[3];
  }

	#elif 1

  // More compact implementation of the conversion method from OpenSG
  // @deprecated

  TYPE trace = Base::operator()(0,0) + Base::operator()(1,1) + Base::operator()(2,2);
  if ( trace > 0 ) {
	TYPE s = 2.0 * sqrt(trace + 1.0);
	quat[3] = 0.25 * s;
	quat[0] = ( Base::operator()(2,1) - Base::operator()(1,2) ) / s;
	quat[1] = ( Base::operator()(0,2) - Base::operator()(2,0) ) / s;
	quat[2] = ( Base::operator()(1,0) - Base::operator()(0,1) ) / s;
  } else {
	if ( Base::operator()(0,0) > Base::operator()(1,1) && Base::operator()(0,0) > Base::operator()(2,2) ) {
	  TYPE s = 2.0 * sqrt( 1.0 + Base::operator()(0,0) - Base::operator()(1,1) - Base::operator()(2,2));
	  quat[3] = (Base::operator()(2,1) - Base::operator()(1,2) ) / s;
	  quat[0] = 0.25 * s;
	  quat[1] = (Base::operator()(0,1) + Base::operator()(1,0) ) / s;
	  quat[2] = (Base::operator()(0,2) + Base::operator()(2,0) ) / s;
	} else if (Base::operator()(1,1) > Base::operator()(2,2)) {
	  TYPE s = 2.0 * sqrt( 1.0 + Base::operator()(1,1) - Base::operator()(0,0) - Base::operator()(2,2));
	  quat[3] = (Base::operator()(0,2) - Base::operator()(2,0) ) / s;
	  quat[0] = (Base::operator()(0,1) + Base::operator()(1,0) ) / s;
	  quat[1] = 0.25 * s;
	  quat[2] = (Base::operator()(1,2) + Base::operator()(2,1) ) / s;
	} else {
	  TYPE s = 2.0 * sqrt( 1.0 + Base::operator()(2,2) - Base::operator()(0,0) - Base::operator()(1,1) );
	  quat[3] = (Base::operator()(1,0) - Base::operator()(0,1) ) / s;
	  quat[0] = (Base::operator()(0,2) + Base::operator()(2,0) ) / s;
	  quat[1] = (Base::operator()(1,2) + Base::operator()(2,1) ) / s;
	  quat[2] = 0.25 * s;
	}
  }

	#else

  // Old conversion method
  // @see http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
  // @deprecated
  // @attention Computes wrong quaternion for rotation close to 180 degree!

  quat[3] = sqrt( max( 0.0, 1.0 + Base::operator()(0,0) + Base::operator()(1,1) + Base::operator()(2,2) ) ) / 2.0;
  quat[0] = sqrt( max( 0.0, 1.0 + Base::operator()(0,0) - Base::operator()(1,1) - Base::operator()(2,2) ) ) / 2.0;
  quat[1] = sqrt( max( 0.0, 1.0 - Base::operator()(0,0) + Base::operator()(1,1) - Base::operator()(2,2) ) ) / 2.0;
  quat[2] = sqrt( max( 0.0, 1.0 - Base::operator()(0,0) - Base::operator()(1,1) + Base::operator()(2,2) ) ) / 2.0;
  quat[0] = copysign( quat[0], Base::operator()(2,1) - Base::operator()(1,2) ) ;
  quat[1] = copysign( quat[1], Base::operator()(0,2) - Base::operator()(2,0) ) ;
  quat[2] = copysign( quat[2], Base::operator()(1,0) - Base::operator()(0,1) ) ;

	#endif

  // Normalize resulting quaternion
  quat.Normalize();

  // Correct quaternion so that scalar part is non-negative
  if (quat[3] < 0) quat *= -1.0;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::GetRotationAxisAngle(Vec& Axis, TYPE& Angle) const
{
  // nicked from OpenSG
  GetQuaternion().GetAxisAngle(Axis, Angle);
}


template <typename TYPE>
typename TRMatrixBase<TYPE>::Vec TRMatrixBase<TYPE>::GetRotationAxis() const
{
  Vec Axis;
  TYPE Angle;
  GetRotationAxisAngle(Axis, Angle);
  return Axis;
}


template <typename TYPE>
TYPE TRMatrixBase<TYPE>::GetRotationAngle() const
{
  Vec Axis;
  TYPE Angle;
  GetRotationAxisAngle(Axis, Angle);
  return Angle;
}


template <typename TYPE>
int TRMatrixBase<TYPE>::GetRotationAnglesXYZ(TYPE& PhiX, TYPE& PhiY, TYPE& PhiZ) const
{
	#if 1
	if (Base::operator()(0,2) < 1)
	{
		if (Base::operator()(0,2) > -1)
		{
			PhiY = asin(Base::operator()(0,2));
			PhiX = atan2(-Base::operator()(1,2), Base::operator()(2,2));
			PhiZ = atan2(-Base::operator()(0,1), Base::operator()(0,0));
		}
		else
		{	// WARNING.  Not a unique solution.
			PhiY = -FHALF_PI;
			PhiX = -atan2(Base::operator()(1,0), Base::operator()(1,1));
			PhiZ = 0.f; // any angle works
		}
	}
	else
	{	// WARNING.  Not a unique solution.
		PhiY = FHALF_PI;
		PhiX = atan2(Base::operator()(1,0), Base::operator()(1,1));
		PhiZ = 0.f; // any angle works
	}
	#else
	if ((Base::operator()(0,2) == 1.0)||(Base::operator()(0,2) == -1.0)) {
		ASSERT("Rotation for Y-axis is +-PI/2, can not decompose X-/Z-component!" == NULL);
		return -1;
	}
	PhiY = asin(Base::operator()(0,2));
	if (ISINFORNAN(PhiY)) return -1;
	// changed this from atan to atan2 (koeser)
	PhiX = atan2(-Base::operator()(1,2),Base::operator()(2,2));
	if (ISINFORNAN(PhiX)) return -2;
	PhiZ = atan2(-Base::operator()(0,1),Base::operator()(0,0));
	if (ISINFORNAN(PhiZ)) return -3;
	#endif
// #ifdef _CPC_DEBUG
//   TRMatrixBase<TYPE> R;
//   R.SetXYZ(PhiX, PhiY, PhiZ);
//   if (R == (*this))
//     res = 0;
//   else
//     res = -1;
// #endif
	return 0;
}


template <typename TYPE>
int TRMatrixBase<TYPE>::GetRotationAnglesZXY(TYPE& PhiX, TYPE& PhiY, TYPE& PhiZ) const
{
  if ((Base::operator()(0,2) == 1.0)||(Base::operator()(0,2) == -1.0)) {
	DEBUG("Rotation for Y-axis is +-PI/2, can not decompose X-/Z-component!");
	return -1;
  }

  PhiZ = atan2(-Base::operator()(0,1),Base::operator()(1,1));
  if (ISINFORNAN(PhiZ)) return -3;

  PhiX = asin(Base::operator()(2,1));
  if (ISINFORNAN(PhiX)) return -2;
  PhiY =  atan2(-Base::operator()(2,0),Base::operator()(2,2));
  if (ISINFORNAN(PhiY)) return -1;


  // #ifdef _CPC_DEBUG
  //  TRMatrixBase<TYPE> R;
  // R.SetZXY(PhiX, PhiY, PhiZ);
  // cout<<"Erg R: "<<R<<endl;
  //   if (R == (*this))
  //     res = 0;
  //   else
  //     res = -1;
  // #endif
  return 0;
}


template <typename TYPE>
int TRMatrixBase<TYPE>::GetRotationAnglesYXZ(TYPE& PhiY, TYPE& PhiX, TYPE& PhiZ) const
{
  if ((Base::operator()(0,2) == 1.0)||(Base::operator()(0,2) == -1.0)) {
	DEBUG("Rotation for Y-axis is +-PI/2, can not decompose X-/Z-component!");
	return -1;
  }

  PhiZ = atan2(Base::operator()(0,2),Base::operator()(2,2));
  if (ISINFORNAN(PhiZ)) return -3;

  PhiX = asin(-Base::operator()(1,2));
  if (ISINFORNAN(PhiX)) return -2;
  PhiY =  atan2(Base::operator()(1,0),Base::operator()(1,1));
  if (ISINFORNAN(PhiY)) return -1;


  // #ifdef _CPC_DEBUG
  //  TRMatrixBase<TYPE> R;
  // R.SetZXY(PhiX, PhiY, PhiZ);
  // cout<<"Erg R: "<<R<<endl;
  //   if (R == (*this))
  //     res = 0;
  //   else
  //     res = -1;
  // #endif
  return 0;
}


template <typename TYPE>
int TRMatrixBase<TYPE>::GetRotationAnglesZYX(TYPE& PhiX, TYPE& PhiY, TYPE& PhiZ) const
{
  if ((Base::operator()(0,2) == 1.0)||(Base::operator()(0,2) == -1.0)) {
	DEBUG("Rotation for Y-axis is +-PI/2, cannot decompose X-/Z-component!");
	return -1;
  }
  PhiY = asin(-Base::operator()(2,0));
  if (ISINFORNAN(PhiY)) return -1;
  // changed this from atan to atan2 (koeser)
  PhiX = atan2(Base::operator()(2,1),Base::operator()(2,2));
  if (ISINFORNAN(PhiX)) return -2;
  PhiZ = atan2(Base::operator()(1,0),Base::operator()(0,0));
  if (ISINFORNAN(PhiZ)) return -3;
// #ifdef _CPC_DEBUG
//   TRMatrixBase<TYPE> R;
//   R.SetZYX(PhiX, PhiY, PhiZ);
//   if (R == (*this))
//     res = 0;
//   else
//     res = -1;
// #endif
  return 0;
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromQuaternion(const Quat& qu)
{
  // fast version, see below for old code:
  const TYPE nrm = 1.0/norm(qu);
  const TYPE q0 = qu[0]*nrm;
  const TYPE q1 = qu[1]*nrm;
  const TYPE q2 = qu[2]*nrm;
  const TYPE q3 = qu[3]*nrm;

  TYPE* pData = val;

  *pData++ = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
  *pData++ =       2.0 * (q0 * q1 - q2 * q3);
  *pData++ =       2.0 * (q2 * q0 + q1 * q3);

  *pData++ =       2.0 * (q0 * q1 + q2 * q3);
  *pData++ = 1.0 - 2.0 * (q2 * q2 + q0 * q0);
  *pData++ =       2.0 * (q1 * q2 - q0 * q3);

  *pData++ =       2.0 * (q2 * q0 - q1 * q3);
  *pData++ =       2.0 * (q1 * q2 + q0 * q3);
  *pData++ = 1.0 - 2.0 * (q1 * q1 + q0 * q0);


#ifdef WANT_TO_COMPARE_WITH_OLD_CODE

  Quat q = qu; // de-const
  q.Normalize();

  // nicked from OpenSG
  TRMatrixBase<TYPE> RCompare(MatrixZero);
  RCompare[0][0] = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  RCompare[1][0] =       2.0 * (q[0] * q[1] + q[2] * q[3]);
  RCompare[2][0] =       2.0 * (q[2] * q[0] - q[1] * q[3]);

  RCompare[0][1] =       2.0 * (q[0] * q[1] - q[2] * q[3]);
  RCompare[1][1] = 1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]);
  RCompare[2][1] =       2.0 * (q[1] * q[2] + q[0] * q[3]);

  RCompare[0][2] =       2.0 * (q[2] * q[0] + q[1] * q[3]);
  RCompare[1][2] =       2.0 * (q[1] * q[2] - q[0] * q[3]);
  RCompare[2][2] = 1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]);

  for (unsigned i=0; i<3; i++) {
	for (unsigned j=0; j<3; j++) {
	  if (!equal(Base::operator()(i,j), RCompare[i][j], TYPE(1e-12))) {
		SLOG("Matrices are not same !"<<*this<<" "<<RCompare<<std::endl);
		SLOG(" at "<<i<<" "<<j<<" "<< (Base::operator()(i,j))-(RCompare[i][j])<<std::endl);
		ABORT("BIASABORT");
	  }
	}
  }
#endif
}


template <typename TYPE>
void  TRMatrixBase<TYPE>::SetFromOrthogonalBasis(const Vec& xxx, const Vec& yyy, const Vec& zzz)
{
	SetFromRowVectors(normalized(xxx),
					  normalized(yyy),
					  normalized(zzz));
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromHV(const Vec& xxx, const Vec& yyy)
{
	// normalize
	const Vec x0(normalized(xxx));

	// compute base z in rhs:
	const Vec z0(normalized(cross(xxx, yyy)));

	// compute y base orthogonal in rhs
	const Vec y0(normalized(cross(z0, x0)));

	(*this).SetFromColumnVectors(x0,  y0,  z0);
}


template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromDirUpGL(const Vec& viewDir, const Vec& viewUp)
{
	ASSERT(ISEQUAL(norm(viewDir), TYPE(1)));
	ASSERT(ISEQUAL(norm(viewUp), TYPE(1)));
	const Vec right(normalized(cross(viewDir, viewUp)));
	const Vec up(normalized(cross(right, viewDir)));
	const Vec forward(viewDir * TYPE(-1)); // convert to right handed system
	SetFromColumnVectors(right, up, forward);
}

template <typename TYPE>
void TRMatrixBase<TYPE>::SetFromDirUp(const Vec& viewDir, const Vec& viewUp)
{
	ASSERT(ISEQUAL(norm(viewDir), TYPE(1)));
	ASSERT(ISEQUAL(norm(viewUp), TYPE(1)));
	const Vec right(normalized(cross(viewDir, viewUp)));
	const Vec up(normalized(cross(viewDir, right)));
	const Vec& forward(viewDir);
	SetFromColumnVectors(right, up, forward);
}

template <typename TYPE>
void TRMatrixBase<TYPE>::LookAt(const Vec& from, const Vec& to, const Vec& up)
{
	SetFromDirUp(normalized(to-from), up);
}


template <typename TYPE>
inline void TRMatrixBase<TYPE>::SetRotationAxisAngle(const Vec& rot)
{
	#if 0
	// set rotation using Rodriguez formula
	(*this) = Mat::IDENTITY;
	const TYPE thetaSq = normSq(rot);
	if (thetaSq < TYPE(1e-12))
		return;
	const TYPE theta = sqrt(thetaSq);
	// make cross product matrix of rot
	Mat J;
	J(0,0) =  TYPE(0); J(0,1) = -rot.z;   J(0,2) =  rot.y;
	J(1,0) =  rot.z;   J(1,1) =  TYPE(0); J(1,2) = -rot.x;
	J(2,0) = -rot.y;   J(2,1) =  rot.x;   J(2,2) =  TYPE(0);
	// compute R matrix
	const Mat J2 = J * J;
	const TYPE c1 = sin(theta)/theta;
	const TYPE c2 = (TYPE(1)-cos(theta))/thetaSq;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			(*this)(i,j) += c1*J(i,j) + c2*J2(i,j);
	#elif !defined(_USE_EIGEN)
	// set rotation using Rodriguez formula
	const TYPE x2 = rot.x*rot.x;
	const TYPE y2 = rot.y*rot.y;
	const TYPE z2 = rot.z*rot.z;
	const TYPE t2 = x2+y2+z2;
	if (t2 > TYPE(1e-12)) {
		// We want to be careful to only evaluate the square root if the
		// norm of the angle_axis vector is greater than zero. Otherwise
		// we get a division by zero.
		const TYPE t  = sqrt(t2);
		const TYPE ct = /*t==0.0?0.5:*/(TYPE(1)-cos(t))/t2;
		const TYPE st = /*t==0.0?1.0:*/sin(t)/t;
		val[0] = TYPE(1) - (y2 + z2)*ct;
		val[1] = rot.x*rot.y*ct - rot.z*st;
		val[2] = rot.z*rot.x*ct + rot.y*st;
		val[3] = rot.x*rot.y*ct + rot.z*st;
		val[4] = TYPE(1) - (z2 + x2)*ct;
		val[5] = rot.y*rot.z*ct - rot.x*st;
		val[6] = rot.z*rot.x*ct - rot.y*st;
		val[7] = rot.y*rot.z*ct + rot.x*st;
		val[8] = TYPE(1) - (x2 + y2)*ct;
	} else {
		// At zero, we switch to using the first order Taylor expansion.
		val[0] = TYPE(1);
		val[1] =-rot.z;
		val[2] = rot.y;
		val[3] = rot.z;
		val[4] = TYPE(1);
		val[5] =-rot.x;
		val[6] =-rot.y;
		val[7] = rot.x;
		val[8] = TYPE(1);
	}
	#else
	(*this) = Eigen::SO3<TYPE>(rot).get_matrix();
	#endif
}

template <typename TYPE>
inline void TRMatrixBase<TYPE>::Apply(const Vec& delta)
{
	const TRMatrixBase dR(delta);
	(*this) = dR * (*this);
}

template <typename TYPE>
inline typename TRMatrixBase<TYPE>::Vec TRMatrixBase<TYPE>::GetRotationAxisAngle() const
{
	#if 0
	// get parametrized rotation using quaternions
	Vec angle_axis;
	TYPE angle;
	GetQuaternion().GetAxisAngle(angle_axis, angle);
	angle_axis.Normalize();
	angle_axis *= angle;
	return angle_axis;
	#elif 0
	// get parametrized rotation using Rodriguez formula
	Vec angle_axis;
	// x = k * 2 * sin(theta), where k is the axis of rotation.
	angle_axis[0] = (*this)(2, 1) - (*this)(1, 2);
	angle_axis[1] = (*this)(0, 2) - (*this)(2, 0);
	angle_axis[2] = (*this)(1, 0) - (*this)(0, 1);
	static const TYPE kZero(0);
	static const TYPE kOne(1);
	static const TYPE kTwo(2);
	// Since the right hand side may give numbers just above 1.0 or
	// below -1.0 leading to atan misbehaving, we threshold.
	const TYPE costheta(MINF(MAXF(((*this)(0, 0) + (*this)(1, 1) + (*this)(2, 2) - kOne)/kTwo, TYPE(-1.0)), kOne));
	// sqrt is guaranteed to give non-negative results, so we only
	// threshold above.
	const TYPE sintheta(MINF(norm(angle_axis)/kTwo, kOne));
	// Use the arctan2 to get the right sign on theta
	const TYPE theta(atan2(sintheta, costheta));
	// Case 1: sin(theta) is large enough, so dividing by it is not a
	// problem. We do not use abs here, because while jets.h imports
	// std::abs into the namespace, here in this file, abs resolves to
	// the int version of the function, which returns zero always.
	//
	// We use a threshold much larger then the machine epsilon, because
	// if sin(theta) is small, not only do we risk overflow but even if
	// that does not occur, just dividing by a small number will result
	// in numerical garbage. So we play it safe.
	static const TYPE kThreshold(TYPE(1e-12));
	if ((sintheta > kThreshold) || (sintheta < -kThreshold)) {
		angle_axis *= theta / (kTwo * sintheta);
		return angle_axis;
	}
	// Case 2: theta ~ 0, means sin(theta) ~ theta to a good
	// approximation.
	if (costheta > kZero) {
		angle_axis *= TYPE(0.5);
		return angle_axis;
	}
	// Case 3: theta ~ pi, this is the hard case. Since theta is large,
	// and sin(theta) is small. Dividing theta by sin(theta) will either
	// give an overflow or worse still numerically meaningless results.
	// Thus we use an alternate more complicated and expensive formula.
	#ifndef _USE_EIGEN
	// Since cos(theta) is negative, division by (1-cos(theta)) cannot
	// overflow.
	const TYPE inv_one_minus_costheta(kOne / (kOne - costheta));
	// We now compute the absolute value of coordinates of the axis
	// vector using the diagonal entries of R. To resolve the sign of
	// these entries, we compare the sign of angle_axis[i]*sin(theta)
	// with the sign of sin(theta). If they are the same, then
	// angle_axis[i] should be positive, otherwise negative.
	for (int i = 0; i < 3; ++i) {
		angle_axis[i] = theta * sqrt(((*this)(i, i) - costheta) * inv_one_minus_costheta);
		if (((sintheta < kZero) && (angle_axis[i] > kZero)) ||
			((sintheta > kZero) && (angle_axis[i] < kZero)))
			angle_axis[i] = -angle_axis[i];
	}
	#else
	Eigen::AngleAxis<TYPE> aa;
	aa.fromRotationMatrix(Base::EMatMap((TYPE*)this));
	angle_axis[0] = aa.angle() * aa.axis()[0];
	angle_axis[1] = aa.angle() * aa.axis()[1];
	angle_axis[2] = aa.angle() * aa.axis()[2];
	#endif
	return angle_axis;
	#else
	return reinterpret_cast<const Eigen::SO3<TYPE>*>(this)->ln();
	#endif
}


template <typename TYPE>
bool TRMatrixBase<TYPE>::Check(const TYPE eps, int verbose) const
{
	// check determinant
	bool ok = equal(TYPE(cv::determinant(*this)), TYPE(1), eps);
	// check unit column vector length
	Vec cl[3];
	for (unsigned i = 0; i < 3 && ok; i++) {
		cl[i] = Base::col(i);
		ok = equal(norm(cl[i]), TYPE(1), eps);
	}
	// check orthogonality
	if (ok)
		ok = equal(cl[0].dot(cl[1]), TYPE(0), eps);
	if (ok)
		ok = equal(cl[0].dot(cl[2]), TYPE(0), eps);
	if (ok)
		ok = equal(cl[1].dot(cl[2]), TYPE(0), eps);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print the reason in case of failure
	if (!ok && VERBOSITY_LEVEL > verbose) {
		std::ostringstream res(" ");
		const TYPE det(cv::determinant(*this));
		if (!equal(det, TYPE(1), eps)) {
			res << "det != 1 (det=" << std::setprecision(30) << det << ") ";
		}
		for (unsigned i = 0; i < 3; i++) {
			cl[i] = Base::col(i);
			if (!equal(norm(cl[i]), TYPE(1), eps)) {
				res << "col[" << i << "].NormL2() != 1) (= "
					<< norm(cl[i]) << ") ";
			}
		}
		if (!equal(cl[0].dot(cl[1]), TYPE(0), eps)) {
			res << "col[0].ScalarProduct(c[1]) != 0 (= "
				<< cl[0].dot(cl[1]) << ") ";
		}
		if (!equal(cl[0].dot(cl[2]), TYPE(0), eps)) {
			res << "col[0].ScalarProduct(c[2]) != 0 (= "
				<< cl[0].dot(cl[2]) << ") ";
		}
		if (!equal(cl[1].dot(cl[2]), TYPE(0), eps)) {
			res << "col[1].ScalarProduct(c[2]) != 0 (= "
				<< cl[1].dot(cl[2]) << ") ";
		}
		SLOG("Rotation matrix is invalid: " << res.str() << std::endl);
	}
	#endif
	return ok;
}

template <typename TYPE>
void TRMatrixBase<TYPE>::EnforceOrthogonality()
{
	#if 1
	// use SVD decomposition to find the closest orthogonal matrix
	const cv::SVD svd(*this, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
	*this = svd.u*svd.vt;
	#elif 1
	// replaces the 3 x 3 matrix with the nearest (least squares) orthogonal matrix;
	// a combination of Babylonian iteration and Berthold KP Horn's formula R = M (MT M)^-1/2
	TYPE* r = this->val;
	TYPE m[9], ri[9], x[9];
	memcpy(m, r, 9*sizeof(TYPE));
	//By trial and error I found that 4 iterates always works unless your matrix is waaay off. Limit 10.
	for (unsigned iterate = 0; iterate != 10; iterate++) {
		//Invert r. Use inverse and not transpose since r is only approximately orthogonal (transpose will result in slow, oscillating convergence).
		InvertMatrix3x3(r, ri);

		//Get x = ri m + mt r.
		x[0] = m[6]*r[6]+ri[2]*m[6]+m[3]*r[3]+ri[1]*m[3]+m[0]*ri[0]+m[0]*r[0];
		x[1] = m[6]*r[7]+ri[2]*m[7]+m[3]*r[4]+ri[1]*m[4]+m[0]*r[1]+ri[0]*m[1];
		x[2] = m[6]*r[8]+ri[2]*m[8]+m[3]*r[5]+ri[1]*m[5]+m[0]*r[2]+ri[0]*m[2];
		x[3] = r[6]*m[7]+ri[5]*m[6]+m[3]*ri[4]+r[3]*m[4]+m[0]*ri[3]+r[0]*m[1];
		x[4] = m[7]*r[7]+ri[5]*m[7]+m[4]*ri[4]+m[4]*r[4]+m[1]*ri[3]+m[1]*r[1];
		x[5] = m[7]*r[8]+ri[5]*m[8]+m[4]*r[5]+ri[4]*m[5]+m[2]*ri[3]+m[1]*r[2];
		x[6] = m[6]*ri[8]+r[6]*m[8]+m[3]*ri[7]+m[0]*ri[6]+r[3]*m[5]+r[0]*m[2];
		x[7] = m[7]*ri[8]+r[7]*m[8]+m[4]*ri[7]+m[1]*ri[6]+r[4]*m[5]+r[1]*m[2];
		x[8] = m[8]*ri[8]+m[8]*r[8]+m[5]*ri[7]+m[2]*ri[6]+m[5]*r[5]+m[2]*r[2];

		//Invert into ri.
		InvertMatrix3x3(x, ri);

		//Replace r with 2 m ri, where ri is as replaced above.
		r[0] = 2*(m[2]*ri[6]+m[1]*ri[3]+m[0]*ri[0]);
		r[1] = 2*(m[2]*ri[7]+m[1]*ri[4]+m[0]*ri[1]);
		r[2] = 2*(m[2]*ri[8]+m[1]*ri[5]+m[0]*ri[2]);
		r[3] = 2*(m[5]*ri[6]+ri[3]*m[4]+ri[0]*m[3]);
		r[4] = 2*(m[5]*ri[7]+m[4]*ri[4]+ri[1]*m[3]);
		r[5] = 2*(m[5]*ri[8]+m[4]*ri[5]+ri[2]*m[3]);
		r[6] = 2*(ri[6]*m[8]+ri[3]*m[7]+ri[0]*m[6]);
		r[7] = 2*(ri[7]*m[8]+ri[4]*m[7]+ri[1]*m[6]);
		r[8] = 2*(m[8]*ri[8]+ri[5]*m[7]+ri[2]*m[6]);

		//Check how off it is.
		x[0] = ABS(r[0]*r[0] + r[3]*r[3] + r[6]*r[6] - TYPE(1));
		x[1] = ABS(r[1]*r[1] + r[4]*r[4] + r[7]*r[7] - TYPE(1));
		x[2] = ABS(r[2]*r[2] + r[5]*r[5] + r[8]*r[8] - TYPE(1));
		x[3] = ABS(r[0]*r[1] + r[3]*r[4] + r[6]*r[7]);
		x[4] = ABS(r[1]*r[2] + r[4]*r[5] + r[7]*r[8]);
		x[5] = ABS(r[2]*r[0] + r[5]*r[3] + r[8]*r[6]);

		//Max norm, by all means, really make it as accurate as possible.
		TYPE maxElement(-FLT_MAX);
		for (unsigned n=6; n != 0; )
			if(x[--n] > maxElement)
				maxElement = x[n];
		if (maxElement < TYPE(1e-15)) return;
	}
	#else
	// Matrix Orthogonalization
	// by Eric Raible from "Graphics Gems", Academic Press, 1990
	//
	// Reorthogonalize matrix R - that is find an orthogonal matrix that is
	// "close" to R by computing an approximation to the orthogonal matrix
	//
	//           T  -1/2
	//   RC = R(R R)
	//                              T      -1
	// [RC is orthogonal because (RC) = (RC) ]
	//                                                                -1/2
	// To compute C, we evaluate the Taylor expansion of F(x) = (I + x)
	// (where x = C - I) about x=0.
	// This gives C = I - (1/2)x + (3/8)x^2 - (5/16)x^3 + ...
	// By default, limit should be 8.
	const unsigned max_limit = 8;
	static const TYPE coef[10] = { 1, -1/2., 3/8., -5/16., 35/128., -63/256., 231/1024., -429/2048., 6435/32768., -12155/65536. };
	Base& R(*this);
	const Base RtR(R.t()*R);
	const Base X(RtR - Matrix3x3::IDENTITY);	/* RtR - I */
	Base X_power(Matrix3x3::IDENTITY);			/* X^0 */
	Base Sum(Matrix3x3::IDENTITY);				/* coef[0] * X^0 */
	const unsigned limit(MINF(max_limit, (unsigned)10));
	for (unsigned power=1; power<limit; ++power) {
		X_power = X_power * X;
		Sum += coef[power] * X_power;
	}
	R = R * Sum;
	#endif
	if (cv::determinant(*this) < 0)
		*this *= TYPE(-1);
	ASSERT(Check(R_CONSTRAINT_ACCURACY));
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE
