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

Transform Transform::Invert() const
{
	Transform inv;
	inv.scale = REAL(1) / scale;
	inv.R = R.t();
	inv.t = -(inv.scale) * (inv.R * t);
	return inv;
}

Transform Transform::operator*(const Transform& other) const
{
	Transform res;
	res.scale = scale * other.scale;
	res.R = R * other.R;
	res.t = scale * (R * other.t) + t;
	return res;
}

Point3 Transform::operator*(const Point3& p) const
{
	return scale * (R * p) + t;
}

Transform Transform::Random(std::mt19937& rng, REAL maxRotationAngleDeg, REAL maxTranslation, REAL minScalePercent)
{
	Transform T;
	// Generate random rotation using axis-angle representation
	std::uniform_real_distribution<REAL> angleDist(-maxRotationAngleDeg, maxRotationAngleDeg);
	std::uniform_real_distribution<REAL> axisDist(-1.0, 1.0);
	Point3 axis(axisDist(rng), axisDist(rng), axisDist(rng));
	const REAL axisNorm = norm(axis);
	if (!ISZERO(axisNorm)) {
		axis *= D2R(angleDist(rng)) / axisNorm;
		T.R.SetRotationAxisAngle(axis);
	} else {
		T.R = RMatrix::IDENTITY;
	}
	// Generate random translation
	std::uniform_real_distribution<REAL> transDist(-maxTranslation, maxTranslation);
	T.t = Point3(transDist(rng), transDist(rng), transDist(rng));
	// Generate random scale
	std::uniform_real_distribution<REAL> scaleDist(REAL(1)-minScalePercent, REAL(1)+minScalePercent);
	T.scale = scaleDist(rng);
	return T;
}

Eigen::Affine3d Transform::ToEigen() const
{
	Eigen::Affine3d T;
	T.linear() = static_cast<double>(scale) * static_cast<Matrix3x3::CEMatMap>(R);
	T.translation() = static_cast<Point3::CEVecMap>(t);
	return T;
}

Transform& Transform::FromEigen(const Eigen::Affine3d& T)
{
	Eigen::Matrix3d linear = T.linear();
	scale = linear.col(0).norm();
	if (scale > 1e-9)
		R = linear / scale;
	else
		R = Matrix3x3::IDENTITY;
	t = T.translation();
	return *this;
}
/*----------------------------------------------------------------*/


// compute the similarity transform that best aligns the given two sets of corresponding 3D points
Matrix4x4 SEACAVE::SimilarityTransform(const Point3Arr& points, const Point3Arr& pointsRef)
{
	ASSERT(points.size() == pointsRef.size());
	typedef Eigen::Matrix<REAL,3,Eigen::Dynamic> PointsVec;
	PointsVec p(3, points.size());
	PointsVec pRef(3, pointsRef.size());
	FOREACH(i, points) {
		p.col(i) = static_cast<const Point3::CEVecMap>(points[i]);
		pRef.col(i) = static_cast<const Point3::CEVecMap>(pointsRef[i]);
	}
	Matrix4x4 transform = Eigen::umeyama(p, pRef);
	return transform;
} // SimilarityTransform

void SEACAVE::DecomposeSimilarityTransform(const Matrix4x4& transform, Matrix3x3& R, Point3& t, REAL& s)
{
	const Eigen::Transform<REAL,3,Eigen::Affine,Eigen::RowMajor> T(static_cast<const Matrix4x4::CEMatMap>(transform));
	Eigen::Matrix<REAL,3,3> rotation, scaling;
	T.computeRotationScaling(&rotation, &scaling);
	R = rotation;
	t = T.translation();
	s = scaling.diagonal().mean();
} // DecomposeSimilarityTransform

Transform SEACAVE::EstimateSimilarityTransform(const Point3Arr& srcPoints, const Point3Arr& dstPoints)
{
	// Use Umeyama's algorithm for closed-form solution (with scaling)
	const Matrix4x4 T = SimilarityTransform(srcPoints, dstPoints);
	// Decompose Scale/Rotation/Translation
	Transform transform;
	DecomposeSimilarityTransform(T, transform.R, transform.t, transform.scale);
	return transform;
} // EstimateSimilarityTransform
/*----------------------------------------------------------------*/


// estimate robustly the rotation that best maps srcRots to dstRots (dstR = srcR * alignR)
bool SEACAVE::EstimateRotationAlignment(
	const Matrix3x3Arr& srcRots, const Matrix3x3Arr& dstRots,
	Matrix3x3& alignR,
	REAL inlierThresholdDeg, unsigned maxRefineIters)
{
	if (srcRots.size() < 2 || srcRots.size() != dstRots.size())
		return false; // need at least two correspondences and equal size

	const REAL inlierThresholdRad = D2R(inlierThresholdDeg);
	const REAL cosInlierThreshold = COS(inlierThresholdRad);

	// 1) Build per-image alignment candidates for the documented model dstR = srcR * alignR,
	// i.e. alignR_i = srcR_i^T * dstR_i; a right-side rotation difference (a world-gauge
	// change acts on world-to-camera rotations as R -> R * W) makes these constant across
	// the set, which is what the consensus seed and the IRLS below assume
	// (the previous dst * src^T candidates fit the left-side model instead, which no caller
	// uses: for gauge-differing sets they are conjugates that disagree with each other)
	Matrix3x3Arr relRots;
	relRots.reserve(srcRots.size());
	FOREACH(i, srcRots)
		relRots.emplace_back(srcRots[i].t() * dstRots[i]);

	// 2) Consensus seed: pick the candidate with the largest inlier support
	unsigned bestInliers = 0;
	size_t bestIdx = 0;
	FOREACH(i, relRots) {
		unsigned inliers = 0;
		FOREACH(j, relRots) {
			if (i == j)
				continue;
			const REAL cosAng = ComputeAngle(relRots[i], relRots[j]);
			if (cosAng >= cosInlierThreshold)
				++inliers;
		}
		if (inliers > bestInliers) {
			bestInliers = inliers;
			bestIdx = i;
		}
	}
	if (bestInliers < 2)
		return false;
	alignR = relRots[bestIdx];

	// Robust Tukey weight
	unsigned numInliers;
	auto weight = [inlierThresholdRad,&numInliers](REAL ang) {
		if (ang >= inlierThresholdRad || !ISFINITE(ang))
			return REAL(0);
		++numInliers;
		const REAL r = ang / inlierThresholdRad;
		const REAL t = REAL(1) - r*r;
		return t*t;
	};

	// 3) IRLS refinement on SO(3)
	unsigned iter = 0;
	for (; iter < maxRefineIters; ++iter) {
		Vec3 accum(0, 0, 0);
		double wSum = 0.0;
		numInliers = 0;
		for (const Matrix3x3& R_i : relRots) {
			const RMatrix delta(alignR.t() * R_i);
			const Vec3 rotVec(delta.GetRotationAxisAngle()); // axis * angle
			const REAL ang = norm(rotVec);
			const REAL w = weight(ang);
			if (w == REAL(0))
				continue;
			accum += rotVec * w;
			wSum += w;
		}
		if (wSum == 0)
			break;
		const Vec3 step(accum * REAL(1.0 / wSum));
		if (norm(step) < REAL(1e-6))
			break;
		RMatrix dR(step);
		alignR = alignR * dR;
	}
	DEBUG_EXTRA("Rotation alignment robust estimation converged in %u iters with %u inliers",
		iter, numInliers);
	return true;
} // EstimateRotationAlignment
/*----------------------------------------------------------------*/


// decomposition of projection matrix into KR[I|-C]: internal calibration ([3,3]), rotation ([3,3]) and translation ([3,1])
// (comparable with OpenCV: normalized cv::decomposeProjectionMatrix)
void SEACAVE::DecomposeProjectionMatrix(const PMatrix& P, KMatrix& K, RMatrix& R, CMatrix& C)
{
	// extract camera center as the right null vector of P
	const Vec4 hC(P.RightNullVector());
	C = CMatrix(hC[0],hC[1],hC[2]) * INVERT(hC[3]);
	// perform RQ decomposition
	RQDecomp3x3<REAL>(cv::Mat(3,4,cv::DataType<REAL>::type,const_cast<REAL*>(P.val))(cv::Rect(0,0, 3,3)), K, R);
	// normalize calibration matrix
	K *= INVERT(K(2,2));
	// ensure positive focal length
	if (K(0,0) < 0) {
		ASSERT(K(1,1) < 0);
		NEGATE(K(0,0));
		NEGATE(K(1,1));
		NEGATE(K(0,1));
		NEGATE(K(0,2));
		NEGATE(K(1,2));
		(TMatrix<REAL,2,3>&)R *= REAL(-1);
	}
	ASSERT(R.IsValid());
} // DecomposeProjectionMatrix
void SEACAVE::DecomposeProjectionMatrix(const PMatrix& P, RMatrix& R, CMatrix& C)
{
	#ifndef _RELEASE
	KMatrix K;
	DecomposeProjectionMatrix(P, K, R, C);
	ASSERT(K.IsEqual(Matrix3x3::IDENTITY, 1e-5));
	#endif
	// extract camera center as the right null vector of P
	const Vec4 hC(P.RightNullVector());
	C = CMatrix(hC[0],hC[1],hC[2]) * INVERT(hC[3]);
	// get rotation
	const cv::Mat mP(3,4,cv::DataType<REAL>::type,const_cast<REAL*>(P.val));
	mP(cv::Rect(0,0, 3,3)).copyTo(R);
	ASSERT(R.IsValid());
} // DecomposeProjectionMatrix
/*----------------------------------------------------------------*/

// assemble projection matrix: P=KR[I|-C]
void SEACAVE::AssembleProjectionMatrix(const KMatrix& K, const RMatrix& R, const CMatrix& C, PMatrix& P)
{
	// compute temporary matrices
	#if 0
	cv::Mat mP(3,4,cv::DataType<REAL>::type,const_cast<REAL*>(P.val));
	cv::Mat M(mP, cv::Rect(0,0, 3,3));
	cv::Mat(K * R).copyTo(M); //3x3
	mP.col(3) = M * cv::Mat(-C); //3x1
	#else
	const Matrix3x3 KR = K * R;
	const Point3 KRC = KR * (-C);
	// Manually construct P = [K*R | -K*R*C]
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j)
			P(i, j) = KR(i, j);
		P(i, 3) = KRC[i];
	}
	#endif
} // AssembleProjectionMatrix
void SEACAVE::AssembleProjectionMatrix(const RMatrix& R, const CMatrix& C, PMatrix& P)
{
	Eigen::Map<Matrix3x3::EMat,0,Eigen::Stride<4,0> >(P.val) = (const Matrix3x3::EMat)R;
	Eigen::Map<Point3::EVec,0,Eigen::Stride<0,4> >(P.val+3) = ((const Matrix3x3::EMat)R) * (-((const Point3::EVec)C));
} // AssembleProjectionMatrix
/*----------------------------------------------------------------*/
