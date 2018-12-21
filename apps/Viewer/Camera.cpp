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

Camera::Camera(const AABB3d& _box, double _fov)
	:
	box(_box),
	width(0), height(0),
	rotation(Eigen::Quaterniond::Identity()),
	center(Eigen::Vector3d::Zero()),
	dist(0), radius(100), fov(_fov),
	scaleF(1.f),
	prevCamID(NO_ID), currentCamID(NO_ID), maxCamID(0)
{
	Reset();
}

void Camera::CopyOf(const Camera& rhs)
{
	rotation = rhs.rotation;
	center = rhs.center;
	dist   = rhs.dist;
	radius = rhs.radius;
	fov    = rhs.fov;
}


void Camera::Init(const AABB3d& _box)
{
	box = _box;
	Reset();
}

void Camera::Reset()
{
	center = box.GetCenter();
	radius = box.GetSize().norm()*0.5;
	rotation = Eigen::Quaterniond::Identity();
	scaleF = 1.f;
	prevCamID = currentCamID = NO_ID;
	fov = 40;
	dist = radius * 0.5 / SIN(D2R(fov));
	Resize(width, height);
}

void Camera::Resize(int _width, int _height)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const GLfloat zNear = 1e-2f;
	const GLfloat zFar = 1e5f;
	width = _width; height = _height;
	GLfloat aspect = float(width)/float(height);
	GLfloat fH = TAN(FD2R((float)fov)) * zNear;
	GLfloat fW = fH * aspect;
	glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

void Camera::SetFOV(double _fov)
{
	fov = _fov;
	Resize(width, height);
}


Eigen::Vector3d Camera::GetPosition() const
{
	const Eigen::Matrix3d R(rotation.toRotationMatrix());
	const Eigen::Vector3d eye(0, 0, dist);
	return R * eye + center;
}

Eigen::Matrix4d Camera::GetLookAt() const
{
	const Eigen::Matrix3d R(rotation.toRotationMatrix());
	const Eigen::Vector3d eye(R.col(2) * dist + center);
	const Eigen::Vector3d up(R.col(1));
	
	const Eigen::Vector3d n((center-eye).normalized());
	const Eigen::Vector3d s(n.cross(up));
	const Eigen::Vector3d v(s.cross(n));
	
	Eigen::Matrix4d m;
	m <<
	s(0),  s(1),  s(2), -eye.dot(s),
	v(0),  v(1),  v(2), -eye.dot(v),
	-n(0), -n(1), -n(2),  eye.dot(n),
	0.0, 0.0, 0.0, 1.0;
	return m;
}
void Camera::GetLookAt(Eigen::Vector3d& _eye, Eigen::Vector3d& _center, Eigen::Vector3d& _up) const
{
	const Eigen::Matrix3d R(rotation.toRotationMatrix());
	const Eigen::Vector3d eye(0, 0, dist);
	const Eigen::Vector3d up(0, 1, 0);

	_eye = R * eye + center;
	_center = center;
	_up  = R * up;
}

void Camera::Rotate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos)
{
	if (pos.isApprox(prevPos, ZERO_TOLERANCE))
		return;

	Eigen::Vector3d oldp(prevPos.x(), prevPos.y(), 0);
	Eigen::Vector3d newp(pos.x(), pos.y(), 0);
	const double radius_virtual_sphere(0.9);
	Project2Sphere(radius_virtual_sphere, oldp);
	Project2Sphere(radius_virtual_sphere, newp);
	Eigen::Quaterniond dr;
	dr.setFromTwoVectors(newp, oldp);
	rotation *= dr;

	// disable camera view mode
	prevCamID = currentCamID;
}

void Camera::Project2Sphere(double radius, Eigen::Vector3d& p) const
{
	p.z() = 0;
	const double d = p.x()* p.x()+ p.y() * p.y();
	const double r = radius * radius;
	if (d < r)	p.z() = SQRT(r - d);
	else		p *= radius / p.norm();
}
/*----------------------------------------------------------------*/
