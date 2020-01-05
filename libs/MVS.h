/*
* MVS.h
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

#ifndef _MVS_MVS_H_
#define _MVS_MVS_H_


// D E F I N E S ///////////////////////////////////////////////////

#define OpenMVS_VERSION_AT_LEAST(x,y,z) \
	(OpenMVS_MAJOR_VERSION>x || (OpenMVS_MAJOR_VERSION==x && \
	(OpenMVS_MINOR_VERSION>y || (OpenMVS_MINOR_VERSION==y && OpenMVS_PATCH_VERSION>=z))))


// I N C L U D E S /////////////////////////////////////////////////

#include "ConfigLocal.h"
#include "Common/Common.h"
#include "IO/Common.h"
#include "Math/Common.h"
#include "MVS/Common.h"
#include "MVS/Scene.h"


#endif // _MVS_MVS_H_
