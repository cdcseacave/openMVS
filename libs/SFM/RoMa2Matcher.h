/*
 * RoMa2Matcher.h
 *
 * Copyright (c) 2014-2026 SEACAVE
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
 */

#ifndef _SFM_ROMA2MATCHER_H_
#define _SFM_ROMA2MATCHER_H_

// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

#ifdef _USE_ONNXRUNTIME

// Resample an 8-bit BGR image to size x size planar RGB in [0,1], reproducing torch's
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) exactly (separable Keys
// cubic with A=-0.5, antialiasing support scaled by the downsampling ratio) -- the
// preprocessing the exported RoMa2 descriptor graph expects on its `image` input.
// Pure CPU, thread-safe (no shared state). Task 7 adds RoMa2Onnx, which runs the exported
// graphs on this preprocessed tensor, to this same file.
SFM_API void PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb);

#endif // _USE_ONNXRUNTIME

/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROMA2MATCHER_H_
