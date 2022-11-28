/*
* PatchMatchCUDA.h
*
* Copyright (c) 2014-2021 SEACAVE
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

#ifndef _MVS_PATCHMATCHCUDA_H_
#define _MVS_PATCHMATCHCUDA_H_

#ifdef _USE_CUDA


// I N C L U D E S /////////////////////////////////////////////////

#include "SceneDensify.h"
#pragma push_macro("EIGEN_DEFAULT_DENSE_INDEX_TYPE")
#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#include "PatchMatchCUDA.inl"
#pragma pop_macro("EIGEN_DEFAULT_DENSE_INDEX_TYPE")


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

} // namespace MVS

#endif // _USE_CUDA

#endif // _MVS_PATCHMATCHCUDA_H_
