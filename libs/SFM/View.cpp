////////////////////////////////////////////////////////////////////
// View.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "View.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

cv::Mat& View::ToWorkingOrientation(cv::Mat& mat) const
{
	if (IsRotated())
		cv::rotate(mat, mat, cv::ROTATE_90_CLOCKWISE);
	return mat;
}
cv::Mat& View::ToOriginalOrientation(cv::Mat& mat) const
{
	if (IsRotated())
		cv::rotate(mat, mat, cv::ROTATE_90_COUNTERCLOCKWISE);
	return mat;
}

Point2f View::ToOriginalOrientation(const Point2f& pw) const {
	if (!IsRotated())
		return pw;
	// Derivation for 90° CCW about pixel centers (top-left origin, y down):
	//   1) Centers: working C_w = ((w_w-1)/2, (h_w-1)/2). Original (after rotate)
	//      has w_o = h_w, h_o = w_w and C_o = ((w_o-1)/2, (h_o-1)/2).
	//   2) Move pixel to centered coords: u_w = x_w - C_w.x,  v_w = y_w - C_w.y.
	//   3) Rotate 90° CCW about origin: [u_o; v_o] = [ -v_w ; u_w ].
	//   4) Move back to original image center: x_o = C_o.x + u_o,  y_o = C_o.y + v_o.
	//   5) Substitute centers:
	//        x_o = (w_o-1)/2 - (y_w - (h_w-1)/2) = (w_o-1 - y_w + h_w-1)/2
	//        y_o = (h_o-1)/2 + (x_w - (w_w-1)/2) = (h_o-1 + x_w - w_w + 1)/2
	//   6) Use w_o = h_w and h_o = w_w => x_o = (h_w - 1) - y_w,  y_o = x_w.
	//   7) Since h_w = w_o, rewrite as (x_o, y_o) = (y_w, w_w-1 - x_w) for clarity in code.
	const float landscapeWidth = (float)GetWidth(); // working width
	return Point2f(pw.y, landscapeWidth - 1.f - pw.x); // CCW: (y, width-1-x)
}
Point2f View::ToWorkingOrientation(const Point2f& po) const {
	if (!IsRotated())
		return po;
	// Inverse 90° CW about centers: (x_o, y_o) -> (w_w-1-y_o, x_o).
	const float portraitHeight = (float)GetWidth(); // working width
	return Point2f(portraitHeight - 1.f - po.y, po.x); // CW: (height-1-y, x)
}

cv::Size View::RevertRotation(Matrix3x3::Base* pK, Matrix3x3::Base* pR) const
{
	const cv::Size workingSize = GetSize();
	if (!IsRotated())
		return workingSize;
	const cv::Size orig = GetOriginalSize();
	if (pK) {
		Matrix3x3::Base& K = *pK;
		// Swap focal lengths and map the principal point back to original orientation (90° CCW)
		// similar to ToOriginalOrientation()
		std::swap(K(0, 0), K(1, 1)); // swap fx, fy
		std::swap(K(0, 2), K(1, 2)); // swap cx, cy
		K(1, 2) = (REAL)(workingSize.width - 1) - K(1, 2); // cy = w-1 - cy
	}
	if (pR) {
		Matrix3x3::Base& R = *pR;
		R = RMatrix(0, 0, -M_PI_2) * R;
	}
	return orig;
}
/*----------------------------------------------------------------*/


PMatrix View::GetP() const
{
	// Get the projection matrix P = K*R*[I|-C]
	ASSERT(IsValid());
	PMatrix P;
	AssembleProjectionMatrix(pCamera->GetK(), R, C, P);
	return P;
} // GetP

void View::DecomposeP(const PMatrix& P)
{
	ASSERT(IsValid() && GetCameraType() == CameraType::PINHOLE);
	KMatrix K;
	DecomposeProjectionMatrix(P, K, R, C);
	static_cast<PinholeCamera*>(pCamera)->SetK(K);
} // DecomposeP
/*----------------------------------------------------------------*/
