/*
 * TestsSynthetic.h
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
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#ifndef _TESTS_SYNTHETIC_H_
#define _TESTS_SYNTHETIC_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../../libs/MVS.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Synthetic depth ground truth, generated without data files: an analytic closed room with
// spheres, a ramp and a pillar, carrying a solid (world-space) procedural texture, ray-cast
// exactly into equirectangular panoramas, their 6 cube-face pinhole views (the SphereCubeMap
// faces) and Brown-distorted pinhole views.
// World frame: x along the room, y across, z up; the room spans [-8,8]x[-5,5]x[0,5] m.

// camera of a synthetic view; model + Params() is the Interface v8 camera description
struct SynthCamera {
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;

	CameraModelType model;
	Pinhole<double> pinhole; // PINHOLE, BROWN
	BrownDistortion<double> distortion; // BROWN
	Spherical<double> spherical; // SPHERICAL
	int width, height;
	Matrix3x3 R; // world to camera rotation: X_cam = R (X - C)
	Point3 C; // camera center

	// camera-frame ray of pixel x, scaled so that X_cam = lambda*Ray(x) for the GT lambda:
	// z for PINHOLE, z of the undistorted ray for BROWN, the range for SPHERICAL
	Vec3 Ray(const Vec2& x) const;
	// camera-frame point to pixel; false if behind the camera
	bool Project(const Vec3& X, Vec2& x) const;
	// lambda of a camera-frame point
	double Depth(const Vec3& X) const;
	// fx, fy, cx, cy as a camera matrix (the spherical ones too)
	KMatrix GetK() const;
	// fx, fy, cx, cy [, k1, k2, p1, p2, k3, k4, k5, k6 for BROWN]
	std::vector<double> Params() const;
};

struct SynthView {
	String name;
	SynthCamera camera;
	int pano; // trajectory index of the panorama the view belongs to (Brown: the one before it)
	int face; // cube-face index (SphereCubeMap::FaceRotations(6)) of a face view, -1 otherwise
	Image8U3 image; // supersampled color
	DepthMap depthMap; // GT lambda at the pixel centers, every pixel (the room is closed)
	NormalMap normalMap; // GT camera-frame unit normal, facing the camera
};
typedef CLISTDEF2IDX(SynthView,IIndex) SynthViewArr;

struct SynthConfig {
	int panoWidth = 512; // equirect width (height width/2); faces width/4; Brown width/2 x 3/8 width
	unsigned numPanos = 8; // panoramas on the trajectory
	unsigned firstPano = 0, numRenderPanos = 8; // the rendered range of them
	bool spherical = true; // render the panoramas
	bool faces = true; // render their 6 cube faces
	unsigned numBrown = 4; // Brown views, one between each pair of consecutive rendered panoramas
	unsigned supersample = 4; // SxS stratified color samples per pixel
};

// render the views: per panorama the panorama then its faces, then the Brown views
void SynthGenerate(const SynthConfig&, SynthViewArr& views);
// median relative mismatch between the GT lambda of the pixels of a reprojected into b and the
// GT lambda of b there (~0 for consistent GT), over the pixels visible in b (numPixels > 0)
double SynthGTMismatch(const SynthView& a, const SynthView& b, size_t& numPixels);
// MVS scene of the given face and panorama views: one platform per panorama (its 6 faces and the
// spherical panorama as cameras, the panorama pose), images saved as PNG in folder, sparse points
// sampled from the GT (every 8th pixel of every view, seen by the views whose GT agrees)
bool SynthBuildScene(const SynthViewArr& views, const IIndexArr& viewIDs, const String& folder, Scene& scene);
// save every view (PNG, GT .dmap with lambda and normals, depth preview PNG), cameras.json and,
// if there are face views, scene.mvs of them (paths relative to folder)
bool SynthSave(const SynthViewArr& views, const String& folder);

// depth-map accuracy against GT lambda, accumulated over any number of maps, per image band
class DepthEvaluator {
public:
	enum Band { ALL, EQUATOR, NORTH, SOUTH, SEAM, CENTER, BORDER, NUM_BANDS };
	struct Stats {
		size_t numGT, numEst; // GT-valid pixels, those with an estimate
		double completeness; // numEst/numGT
		double absRelMean, absRelMedian; // |est-gt|/gt
		double within1, within2, within5; // fraction of the estimates with abs-rel below 1/2/5%
		double normalMedianDeg; // median normal angle error
	};
	// gt/est maps of the same size; est 0 = no estimate; estNormals may be empty
	void Add(const DepthMap& gt, const NormalMap& gtNormals, const DepthMap& est, const NormalMap& estNormals, CameraModelType model);
	Stats GetStats(Band) const;
	// header plus one line per non-empty band
	String Print() const;

protected:
	struct Accum {
		size_t numGT = 0;
		FloatArr absRel, normalAngle;
	};
	Accum bands[NUM_BANDS];
};

// GT consistency across the camera models, then CPU PatchMatch on the cube faces of three
// panoramas scored against the GT
bool SyntheticDepthBaselineTest();

// CPU PatchMatch on three equirectangular panoramas of the synthetic scene scored against the GT,
// overall and per band (the seam and the poles against the equator)
bool SyntheticSphericalTest();

// Brown views of the synthetic scene loaded through an Interface v8 Brown camera: MVS::Image reads
// them undistorted as the pinhole render at K', the pixels without a source are in the ignore mask,
// and the observations of GT points land on their K' projections
bool BrownUndistortOnLoadTest();

// Tests.exe tool modes; argv[0] is the mode:
//   synth <outDir> [panoWidth=512]: write the default dataset
//   evaldepth <gt.dmap> <est.dmap> [pinhole|brown|spherical]: print the evaluation table
//   synthtest [verbosity]: run SyntheticDepthBaselineTest alone
//   sphtest <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [maxElevation] [maxViews] [maxThreads]: CPU PatchMatch on panoramas and on
//     their cube faces; maxElevation overrides DepthEstimatorBase::fSphericalWarpMaxElevation (0: exact warp only)
//   facescene <scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize]: the cube-face scene of a spherical scene
//     and the face GT depth-maps, the baseline of its spherical depth-maps
bool SynthTool(int argc, LPCTSTR* argv);

} // namespace MVS

#endif // _TESTS_SYNTHETIC_H_
