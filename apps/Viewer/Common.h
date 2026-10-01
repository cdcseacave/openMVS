/*
 * Common.h
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

#ifndef _VIEWER_COMMON_H_
#define _VIEWER_COMMON_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"

#define GLAD_GL_IMPLEMENTATION
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE 
#include <GLFW/glfw3.h>

// OpenGL debugging utilities
#include "OpenGLDebug.h"


// D E F I N E S ///////////////////////////////////////////////////


// P R O T O T Y P E S /////////////////////////////////////////////

using namespace SEACAVE;

namespace VIEWER {

// the conversion matrix from OpenGL default coordinate system
//  to the camera coordinate system (NADIR orientation):
// [ 1  0  0  0] * [ x ] = [ x ]
//   0 -1  0  0      y      -y
//   0  0 -1  0      z      -z
//   0  0  0  1      1       1
static const Eigen::Matrix4d gs_convert = [] {
	Eigen::Matrix4d tmp; tmp <<
		1,  0,  0,  0,
		0, -1,  0,  0,
		0,  0, -1,  0,
		0,  0,  0,  1;
	return tmp;
}();

/// given rotation matrix R and translation vector t,
/// column-major matrix m is equal to:
/// [ R11 R12 R13 t.x ]
/// | R21 R22 R23 t.y |
/// | R31 R32 R33 t.z |
/// [ 0.0 0.0 0.0 1.0 ]
//
// World to Local
inline Eigen::Matrix4d TransW2L(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
	Eigen::Matrix4d m(Eigen::Matrix4d::Identity());
	m.block(0,0,3,3) = R;
	m.block(0,3,3,1) = t;
	return m;
}
// Local to World
// same as above, but with the inverse of the two
inline Eigen::Matrix4d TransL2W(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
	Eigen::Matrix4d m(Eigen::Matrix4d::Identity());
	m.block(0,0,3,3) = R.transpose();
	m.block(0,3,3,1) = -t;
	return m;
}
/*----------------------------------------------------------------*/

// confidence filter of a point-cloud layer: a point's confidence (the largest of its view weights,
// the depth-map confidence for a .dmap cloud) is quantized to 8 bits over the cloud's range -- the
// byte the renderer keeps in the alpha of the point color -- and only the points whose quantized
// confidence is inside the [levelMin, levelMax] window are shown ([0, 255] shows every point)
struct PointConfidenceFilter {
	float minConf{0.f}, maxConf{0.f}; // confidence range over the cloud
	uint8_t levelMin{0}, levelMax{255}; // shown window of quantized confidence

	bool IsAll() const { return levelMin == 0 && levelMax == 255; }

	static float Confidence(const MVS::PointCloud::WeightArr& weights) {
		ASSERT(!weights.empty());
		float conf(weights.front());
		for (const MVS::PointCloud::Weight w: weights)
			conf = MAXF(conf, w);
		return conf;
	}
	uint8_t Quantize(float conf) const {
		ASSERT(conf >= minConf && conf <= maxConf);
		return maxConf > minConf ? (uint8_t)ROUND2INT((conf-minConf)*255.f/(maxConf-minConf)) : uint8_t(0);
	}
	float Dequantize(uint8_t q) const { return minConf + q*(maxConf-minConf)/255.f; }
	bool IsShown(const MVS::PointCloud& pointcloud, MVS::PointCloud::Index idx) const {
		if (IsAll())
			return true;
		// a narrowed window exists only over a cloud with confidence (Reset widens it otherwise)
		ASSERT(pointcloud.pointWeights.size() == pointcloud.points.size());
		const uint8_t q(Quantize(Confidence(pointcloud.pointWeights[idx])));
		return q >= levelMin && q <= levelMax;
	}
	// the window as the shader compares it against the normalized confidence byte:
	// widened by half a step, so the end levels include every point
	Eigen::Vector2f ShaderWindow() const { return Eigen::Vector2f((levelMin-0.5f)/255.f, (levelMax+0.5f)/255.f); }
	// set the range of the given cloud, keeping the threshold values of the previous range, if any
	// (an open end stays open); a cloud without confidence shows every point
	void Reset(const MVS::PointCloud& pointcloud) {
		if (pointcloud.pointWeights.empty()) {
			*this = PointConfidenceFilter();
			return;
		}
		ASSERT(pointcloud.pointWeights.size() == pointcloud.points.size());
		const bool bRemap(maxConf > minConf);
		const float thresholdMin(Dequantize(levelMin)), thresholdMax(Dequantize(levelMax));
		minConf = FLT_MAX; maxConf = -FLT_MAX;
		for (const MVS::PointCloud::WeightArr& weights: pointcloud.pointWeights) {
			const float conf(Confidence(weights));
			minConf = MINF(minConf, conf);
			maxConf = MAXF(maxConf, conf);
		}
		if (bRemap && levelMin != 0)
			levelMin = Quantize(CLAMP(thresholdMin, minConf, maxConf));
		if (bRemap && levelMax != 255)
			levelMax = Quantize(CLAMP(thresholdMax, minConf, maxConf));
	}
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _VIEWER_COMMON_H_
