/*
 * Camera.cpp
 *
 * Copyright (c) 2014-2015 SEACAVE
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
#include "Camera.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Camera::Camera(const AABB3d& _box, const Point3d& _center, float _scaleF, float _fov)
	:
	boxScene(_box),
	centerScene(_center),
	rotation(Eigen::Quaterniond::Identity()),
	center(Eigen::Vector3d::Zero()),
	dist(0), radius(100),
	fovDef(_fov), scaleFDef(_scaleF),
	prevCamID(NO_ID), currentCamID(NO_ID), maxCamID(0)
{
	Reset();
}

void Camera::Reset()
{
	if (boxScene.IsEmpty()) {
		center = Point3d::ZERO;
		radius = 1;
	} else {
		center = centerScene;
		radius = boxScene.GetSize().norm()*0.5;
	}
	rotation = Eigen::Quaterniond::Identity();
	scaleF = scaleFDef;
	prevCamID = currentCamID = NO_ID;
	fov = fovDef;
	dist = radius*0.5 / SIN(D2R((double)fov));
	if (size.area())
		Resize(size);
}

void Camera::Resize(const cv::Size& _size)
{
	ASSERT(MINF(_size.width, _size.height) > 0);
	size = _size;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const GLfloat zNear = 1e-3f;
	const GLfloat zFar = (float)boxScene.GetSize().norm()*10;
	const GLfloat aspect = float(size.width)/float(size.height);
	if (fov == 5.f) {
		// orthographic projection
		const GLfloat fH = (float)boxScene.GetSize().norm()*0.5f;
		const GLfloat fW = fH * aspect;
		glOrtho(-fW, fW, -fH, fH, zNear, zFar);
	} else {
		// perspective projection
		const GLfloat fH = TAN(FD2R(fov)) * zNear;
		const GLfloat fW = fH * aspect;
		glFrustum(-fW, fW, -fH, fH, zNear, zFar);
	}
}

void Camera::SetFOV(float _fov)
{
	fov = MAXF(_fov, 5.f);
	Resize(size);
}


Eigen::Vector3d Camera::GetPosition() const
{
	const Eigen::Matrix3d R(GetRotation());
	return center + R.col(2) * dist;
}

Eigen::Matrix3d Camera::GetRotation() const
{
	return rotation.toRotationMatrix();
}

Eigen::Matrix4d Camera::GetLookAt() const
{
	const Eigen::Matrix3d R(GetRotation());
	const Eigen::Vector3d eye(center + R.col(2) * dist);
	const Eigen::Vector3d up(R.col(1));

	const Eigen::Vector3d n((center-eye).normalized());
	const Eigen::Vector3d s(n.cross(up));
	const Eigen::Vector3d v(s.cross(n));

	Eigen::Matrix4d m; m <<
		 s(0),  s(1),  s(2), -eye.dot(s),
		 v(0),  v(1),  v(2), -eye.dot(v),
		-n(0), -n(1), -n(2),  eye.dot(n),
		 0.0, 0.0, 0.0, 1.0;
	return m;
}
void Camera::GetLookAt(Eigen::Vector3d& _eye, Eigen::Vector3d& _center, Eigen::Vector3d& _up) const
{
	const Eigen::Matrix3d R(GetRotation());
	_eye = center + R.col(2) * dist;
	_center = center;
	_up = R.col(1);
}


void Camera::Rotate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos)
{
	if (pos.isApprox(prevPos, ZEROTOLERANCE<double>()))
		return;

	Eigen::Vector3d oldp(prevPos.x(), prevPos.y(), 0);
	Eigen::Vector3d newp(pos.x(), pos.y(), 0);
	const double radiusSphere(0.9);
	ProjectOnSphere(radiusSphere, oldp);
	ProjectOnSphere(radiusSphere, newp);
	rotation *= Eigen::Quaterniond().setFromTwoVectors(newp, oldp);

	// disable camera view mode
	prevCamID = currentCamID;
}

void Camera::Translate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos)
{
	if (pos.isApprox(prevPos, ZEROTOLERANCE<double>()))
		return;

	Eigen::Matrix<double,4,4,Eigen::ColMajor> P, V;
	glGetDoublev(GL_MODELVIEW_MATRIX, V.data());
	glGetDoublev(GL_PROJECTION_MATRIX, P.data());
	Eigen::Vector3d centerScreen((P*V*center.homogeneous().eval()).hnormalized());
	centerScreen.head<2>() += prevPos - pos;
	center = (V.inverse()*P.inverse()*centerScreen.homogeneous().eval()).hnormalized();

	// disable camera view mode
	prevCamID = currentCamID;
}

void Camera::ProjectOnSphere(double radius, Eigen::Vector3d& p) const
{
	p.z() = 0;
	const double d = p.x()* p.x()+ p.y() * p.y();
	const double r = radius * radius;
	if (d < r)	p.z() = SQRT(r - d);
	else		p *= radius / p.norm();
}
/*----------------------------------------------------------------*/
