/*
 * RoMa2Matcher.cpp
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

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "RoMa2Matcher.h"

#ifdef _USE_ONNXRUNTIME

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

namespace {

// Keys cubic convolution kernel, A=-0.5: torch's ANTIALIASED bicubic coefficient (not the
// -0.75 of its plain bicubic path, which resamples ~2e-3 differently)
inline float CubicFilter(float x)
{
	constexpr float kA = -0.5f;
	x = ABS(x);
	if (x < 1.f)
		return ((kA + 2.f) * x - (kA + 3.f)) * x * x + 1.f;
	if (x < 2.f)
		return ((kA * x - 5.f * kA) * x + 8.f * kA) * x - 4.f * kA;
	return 0.f;
}

// Per-output-sample resampling taps for one axis: the source range each output sample reads
// and the normalized cubic weights over it. Mirrors torch's antialiased weight computation
// (aten/src/ATen/native/UpSample.h): the filter support widens with the downsampling ratio,
// which is what antialias=true buys over a plain cubic.
struct AxisWeights {
	std::vector<int> start, count; // per output sample: first source index read, tap count
	std::vector<float> weights;    // maxTaps-strided, count[i] valid entries per output sample
	int maxTaps;                   // weights stride: the widest tap count over all output samples
};

AxisWeights ComputeAxisWeights(int in, int out)
{
	AxisWeights w;
	w.start.resize(out);
	w.count.resize(out);
	const float scale = (float)in / (float)out;
	const float support = scale >= 1.f ? 2.f * scale : 2.f;
	const float invScale = scale >= 1.f ? 1.f / scale : 1.f;
	w.maxTaps = MINF((int)std::ceil(support) * 2 + 1, in);
	w.weights.assign((size_t)out * w.maxTaps, 0.f);
	for (int i = 0; i < out; ++i) {
		const float center = scale * (i + 0.5f);
		const int start = MAXF((int)(center - support + 0.5f), 0);
		const int count = MINF((int)(center + support + 0.5f), in) - start;
		float* taps = &w.weights[(size_t)i * w.maxTaps];
		float sum = 0.f;
		for (int t = 0; t < count; ++t)
			sum += (taps[t] = CubicFilter((t + start - center + 0.5f) * invScale));
		if (sum != 0.f)
			for (int t = 0; t < count; ++t)
				taps[t] /= sum;
		w.start[i] = start;
		w.count[i] = count;
	}
	return w;
}

} // namespace

void SFM::PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb)
{
	ASSERT(!bgr.empty() && size > 0);
	const AxisWeights wx = ComputeAxisWeights(bgr.cols, size), wy = ComputeAxisWeights(bgr.rows, size);
	// horizontal pass: rows x size x 3 (RGB), scaled to [0,1]; NO clamping (bicubic overshoot is in the reference too)
	std::vector<float> tmp((size_t)bgr.rows * size * 3);
	for (int y = 0; y < bgr.rows; ++y) {
		const Pixel8U* row = bgr.ptr<Pixel8U>(y);
		float* dst = &tmp[(size_t)y * size * 3];
		for (int x = 0; x < size; ++x, dst += 3) {
			float r = 0, g = 0, b = 0;
			const float* taps = &wx.weights[(size_t)x * wx.maxTaps];
			for (int t = 0; t < wx.count[x]; ++t) {
				const Pixel8U& p = row[wx.start[x] + t];
				r += taps[t] * p.r; g += taps[t] * p.g; b += taps[t] * p.b;
			}
			dst[0] = r / 255.f; dst[1] = g / 255.f; dst[2] = b / 255.f;
		}
	}
	// vertical pass straight into planar R, G, B
	planarRgb.assign((size_t)3 * size * size, 0.f);
	for (int y = 0; y < size; ++y) {
		const float* taps = &wy.weights[(size_t)y * wy.maxTaps];
		for (int t = 0; t < wy.count[y]; ++t) {
			const float* src = &tmp[(size_t)(wy.start[y] + t) * size * 3];
			const float wt = taps[t];
			for (int x = 0; x < size; ++x, src += 3)
				for (int c = 0; c < 3; ++c)
					planarRgb[((size_t)c * size + y) * size + x] += wt * src[c];
		}
	}
}

#pragma pop_macro("VERBOSE")

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/
