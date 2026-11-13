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

	CameraModelType model{};
	Pinhole<double> pinhole{}; // PINHOLE, BROWN
	BrownDistortion<double> distortion{}; // BROWN
	Spherical<double> spherical{}; // SPHERICAL
	int width = 0, height = 0;
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
	int pano = -1; // trajectory index of the panorama the view belongs to (Brown: the one before it)
	int face = -1; // cube-face index (SphereCubeMap::FaceRotations(6)) of a face view, -1 otherwise
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
	// spherical: |elevation| below 30 (EQUATOR), above 70 and below -70 deg, the 16 columns either side
	// of the seam (SEAM) and of longitude 0 (CENTER); pinhole: the 16-pixel BORDER and the CENTER inside
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

	// the latitude band (EQUATOR, NORTH, SOUTH) of an elevation in degrees, ALL if none
	static Band LatitudeBand(double elevationDeg);
	static const char* BandName(Band);

protected:
	struct Accum {
		size_t numGT = 0;
		FloatArr absRel, normalAngle;
	};
	Accum bands[NUM_BANDS];
};

// a point-cloud against the GT range maps of panoramas: precision (the share of the cloud within
// th*range of a GT point) and recall (the share of the GT points within th*range of a cloud point),
// the range being the distance to the nearest panorama, per band of the point's direction from that
// panorama (the bands of DepthEvaluator, the seam band as wide as the angle 16 pixels span at
// bandWidth); a cloud point whose direction holds no GT in that panorama (masked) is not scored
class CloudEvaluator {
public:
	enum { NUM_TH = 3 };
	struct Stats {
		size_t numGT, numCloud; // GT and scored cloud points in the band
		double precision[NUM_TH], recall[NUM_TH]; // per threshold
	};
	CloudEvaluator(int bandWidth, const float thresholds[NUM_TH]);
	// the GT range map of a panorama (0 = no GT) with its pose: X = C + R^T*Ray(x)*range; every pixel is
	// a GT point of the precision, every recallStride-th pixel of every recallStride-th row one of the
	// recall (the GT on the grid of the evaluated images, as dense as the cloud)
	void AddView(const Matrix3x3& R, const Point3& C, const DepthMap& ranges, int recallStride);
	// score the cloud against the GT of the views added so far
	void Evaluate(const PointCloud::PointArr& cloud);
	const Stats& GetStats(DepthEvaluator::Band band) const { return bands[band]; }
	// header plus one line per non-empty band
	String Print() const;

protected:
	struct View { Matrix3x3 R; Point3 C; DepthMap ranges; };
	// the nearest view, the range from it and the bands of the direction of X; false if that view
	// holds no GT in this direction
	bool Classify(const Point3f& X, float& range, int bands[3], int& numBands) const;

	const double seamDeg; // the longitude band around the seam
	float thresholds[NUM_TH]; // relative to the range
	std::vector<View> views;
	PointCloud::PointArr gtPoints, gtRecallPoints;
	Stats bands[DepthEvaluator::NUM_BANDS] = {};
};

// GT consistency across the camera models, then CPU PatchMatch on the cube faces of three
// panoramas scored against the GT
bool SyntheticDepthBaselineTest();

// CPU PatchMatch on three equirectangular panoramas of the synthetic scene scored against the GT,
// overall and per band (the seam and the poles against the equator)
bool SyntheticSphericalTest();

// GPU PatchMatch (CUDA, or Metal on Apple) on the panoramas of SyntheticSphericalTest, held to that
// test's CPU accuracy; skipped without a GPU
bool SyntheticSphericalGPUTest(bool forceCPU);

// the confidence recalibration on the GPU (CUDA, or Metal on Apple) against the CPU sweep on the same
// inputs; skipped without a GPU
bool GPUConfidenceTest(bool forceCPU);

#ifdef _USE_METAL
// the Metal kernels' hand mirror of CameraModels.h and DepthGeometry.h against the C++ models on
// random inputs, evaluated on the device; skipped without a Metal device
bool MetalModelMathTest(bool forceCPU);
#endif

// CPU PatchMatch and dense fusion of three equirectangular panoramas of the synthetic scene: the fused
// cloud scored against the analytic surface, overall and per band (the seam and the poles against the
// equator)
bool SyntheticSphericalFusionTest();

// Brown views of the synthetic scene loaded through an Interface v8 Brown camera: MVS::Image reads
// them undistorted as the pinhole render at K', the pixels without a source are in the ignore mask,
// and the observations of GT points land on their K' projections
bool BrownUndistortOnLoadTest();

// Scene::EstimateNeighborViewsPointCloud on two rigs (a panorama and its 6 cube faces at one center each) with
// an empty point cloud: no assertion fires, every point is finite and is seen by the views of two rigs
bool NeighborViewsPointCloudRigTest();

// Scene::EstimatePointCloudNormals on three panoramas of the synthetic scene prepared at half resolution: the
// scene images keep their resolution, camera and pixels; every point of three views or more whose target view
// projects within 4 columns of the seam is refined, and the mean normal error of the points projecting there
// is within 1.5x that of the other points
bool EstimatePointCloudNormalsSphericalTest();

// Tests.exe tool modes producing the validation numbers of the design docs (argv[0] is the mode;
// without a known mode it prints their usage): the synthetic dataset, the synthetic PatchMatch and
// fusion runs, the ScorePixel cost, depth-map and cloud evaluation against GT, the cube-face
// baseline of a spherical scene, the undistortion cost on load
bool SynthTool(int argc, LPCTSTR* argv);

} // namespace MVS

#endif // _TESTS_SYNTHETIC_H_
