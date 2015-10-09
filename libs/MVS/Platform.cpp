/*
* Platform.cpp
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
#include "Platform.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// return the normalized absolute camera pose
Platform::Camera Platform::GetCamera(uint32_t cameraID, uint32_t poseID) const
{
	const Camera& camera = cameras[cameraID];
	const Pose& pose = poses[poseID];
	// add the relative camera pose to the platform
	Camera cam;
	cam.K = camera.K;
	cam.R = camera.R*pose.R;
	cam.C = pose.R.t()*camera.C+pose.C;
	return cam;
} // GetCamera
/*----------------------------------------------------------------*/
