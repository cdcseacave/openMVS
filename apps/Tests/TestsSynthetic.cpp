/*
 * TestsSynthetic.cpp
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

#include "../../libs/MVS.h"
#include "../../libs/Math/Camera/CameraUndistort.h"
#include "Tests.h"
#include "TestsMVS.h"
#include "TestsSynthetic.h"
#include <chrono>
#include <fstream>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSyn "));

namespace {

typedef Eigen::Vector2d EVec2;
typedef Eigen::Vector3d EVec3;
typedef Eigen::Matrix3d EMat3;

// analytic scene; the surface ids index the texture parameters
enum SynthSurface { SURF_WALL_XMIN, SURF_WALL_XMAX, SURF_WALL_YMIN, SURF_WALL_YMAX, SURF_FLOOR, SURF_CEILING, SURF_SPHERE0, SURF_SPHERE1, SURF_SPHERE2, SURF_PILLAR, SURF_RAMP, NUM_SURFACES };
const EVec3 roomMin(-8, -5, 0), roomMax(8, 5, 5);
const EVec3 sphereCenters[3] = {EVec3(-3.5, 2.5, 1.0), EVec3(4.0, -3.0, 1.2), EVec3(5.5, 3.0, 3.6)};
const double sphereRadii[3] = {1.0, 1.2, 0.7};
const EVec2 pillarAxis(1.5, 2.2); // vertical cylinder from floor to ceiling
const double pillarRadius(0.15);
const EVec3 rampOrigin(-6.5, -4.5, 0), rampEdge1(4.0, 0, 1.6), rampEdge2(0, 2.0, 0); // rectangle rising along x
// low-texture patch on the SURF_WALL_YMAX wall: x and z ranges
const double lowTextureX[2] = {-1.5, 0.5}, lowTextureZ[2] = {1.8, 3.2};

struct SynthHit {
	double t; // ray parameter
	EVec3 normal; // world unit normal facing the ray origin
	int surface;
};

// closest hit of the ray o + t*d (d any length) from a point inside the room and outside the objects
SynthHit SynthTrace(const EVec3& o, const EVec3& d)
{
	SynthHit hit{std::numeric_limits<double>::max(), EVec3::Zero(), -1};
	for (int i = 0; i < 3; ++i) {
		ASSERT(o[i] > roomMin[i] && o[i] < roomMax[i]);
		if (d[i] == 0)
			continue;
		const double t(((d[i] > 0 ? roomMax[i] : roomMin[i]) - o[i]) / d[i]);
		if (t < hit.t) {
			hit.t = t;
			hit.normal = EVec3::Zero();
			hit.normal[i] = d[i] > 0 ? -1 : 1;
			hit.surface = 2*i + (d[i] > 0 ? 1 : 0);
		}
	}
	ASSERT(hit.surface >= 0);
	const double a(d.squaredNorm());
	for (int s = 0; s < 3; ++s) {
		const EVec3 oc(o - sphereCenters[s]);
		const double b(oc.dot(d)), c(oc.squaredNorm() - SQUARE(sphereRadii[s]));
		ASSERT(c > 0);
		const double disc(b*b - a*c);
		if (disc < 0)
			continue;
		const double t((-b - std::sqrt(disc)) / a);
		if (t > 0 && t < hit.t) {
			hit.t = t;
			hit.normal = (oc + t*d) / sphereRadii[s];
			hit.surface = SURF_SPHERE0 + s;
		}
	}
	{
		const EVec2 oc(o.x() - pillarAxis.x(), o.y() - pillarAxis.y());
		const double a2(d.x()*d.x() + d.y()*d.y()), b(oc.x()*d.x() + oc.y()*d.y()), c(oc.squaredNorm() - SQUARE(pillarRadius));
		ASSERT(c > 0);
		const double disc(b*b - a2*c);
		if (a2 > 0 && disc >= 0) {
			const double t((-b - std::sqrt(disc)) / a2);
			if (t > 0 && t < hit.t) {
				hit.t = t;
				hit.normal = EVec3(oc.x() + t*d.x(), oc.y() + t*d.y(), 0) / pillarRadius;
				hit.surface = SURF_PILLAR;
			}
		}
	}
	{
		const EVec3 n(rampEdge1.cross(rampEdge2).normalized());
		const double denom(d.dot(n));
		if (denom != 0) {
			const double t((rampOrigin - o).dot(n) / denom);
			if (t > 0 && t < hit.t) {
				const EVec3 X(o + t*d - rampOrigin);
				const double u(X.dot(rampEdge1) / rampEdge1.squaredNorm()), v(X.dot(rampEdge2) / rampEdge2.squaredNorm());
				if (u >= 0 && u <= 1 && v >= 0 && v <= 1) {
					hit.t = t;
					hit.normal = denom < 0 ? n : EVec3(-n);
					hit.surface = SURF_RAMP;
				}
			}
		}
	}
	return hit;
}

// deterministic lattice value in [0,1)
inline double SynthLattice(int64_t x, int64_t y, int64_t z, uint32_t seed)
{
	uint32_t h((uint32_t)x*0x8da6b343u ^ (uint32_t)y*0xd8163841u ^ (uint32_t)z*0xcb1ab31fu ^ seed*0x9e3779b9u);
	h ^= h >> 16; h *= 0x7feb352du; h ^= h >> 15; h *= 0x846ca68bu; h ^= h >> 16;
	return double(h) * (1.0/4294967296.0);
}
// 3D value noise in [0,1], C2 (quintic) interpolation, unit lattice period
double SynthValueNoise(const EVec3& p, uint32_t seed)
{
	const EVec3 f(p.array().floor());
	const int64_t x((int64_t)f.x()), y((int64_t)f.y()), z((int64_t)f.z());
	const EVec3 t(p - f);
	const EVec3 u(t.array()*t.array()*t.array()*(t.array()*(t.array()*6.0 - 15.0) + 10.0));
	double v[2][2];
	for (int j = 0; j < 2; ++j)
		for (int k = 0; k < 2; ++k)
			v[j][k] = SynthLattice(x, y+j, z+k, seed) + u.x()*(SynthLattice(x+1, y+j, z+k, seed) - SynthLattice(x, y+j, z+k, seed));
	const double v0(v[0][0] + u.y()*(v[1][0] - v[0][0])), v1(v[0][1] + u.y()*(v[1][1] - v[0][1]));
	return v0 + u.z()*(v1 - v0);
}
// band-limited multi-octave value noise in [0,1]; each octave is rotated against the lattice
double SynthFbm(EVec3 p, int octaves, uint32_t seed)
{
	static const EMat3 octave(Eigen::AngleAxisd(0.7, EVec3(0.48, 0.6, 0.64).normalized()).toRotationMatrix() * 2.03);
	double sum(0), amplitude(1), norm(0);
	for (int i = 0; i < octaves; ++i) {
		sum += amplitude * SynthValueNoise(p, seed + 17u*i);
		norm += amplitude;
		amplitude *= 0.55;
		p = octave * p;
	}
	return sum / norm;
}
inline double SynthSmoothStep(double e0, double e1, double x)
{
	const double t(CLAMP((x - e0) / (e1 - e0), 0.0, 1.0));
	return t*t*(3 - 2*t);
}

// view-independent RGB in [0,1] of the world point X on the surface: two albedos blended by a
// low-frequency noise, modulated by a 6-octave fbm (periods ~0.6 m down to ~2 cm) and darkened
// by sharp-edged spots; the low-texture patch keeps only a faint 3-octave fbm
EVec3 SynthColor(const EVec3& X, int surface)
{
	static const EVec3 albedos[NUM_SURFACES][2] = {
		{EVec3(0.85, 0.75, 0.60), EVec3(0.70, 0.80, 0.90)}, // SURF_WALL_XMIN
		{EVec3(0.90, 0.70, 0.65), EVec3(0.75, 0.85, 0.70)}, // SURF_WALL_XMAX
		{EVec3(0.75, 0.80, 0.90), EVec3(0.90, 0.85, 0.65)}, // SURF_WALL_YMIN
		{EVec3(0.80, 0.90, 0.75), EVec3(0.90, 0.75, 0.80)}, // SURF_WALL_YMAX
		{EVec3(0.70, 0.55, 0.40), EVec3(0.55, 0.50, 0.45)}, // SURF_FLOOR
		{EVec3(0.92, 0.92, 0.88), EVec3(0.80, 0.85, 0.92)}, // SURF_CEILING
		{EVec3(0.90, 0.35, 0.30), EVec3(0.95, 0.65, 0.30)}, // SURF_SPHERE0
		{EVec3(0.30, 0.55, 0.90), EVec3(0.40, 0.85, 0.80)}, // SPHERE1
		{EVec3(0.45, 0.85, 0.35), EVec3(0.85, 0.85, 0.30)}, // SPHERE2
		{EVec3(0.85, 0.85, 0.85), EVec3(0.60, 0.60, 0.70)}, // SURF_PILLAR
		{EVec3(0.75, 0.45, 0.80), EVec3(0.55, 0.40, 0.85)}, // SURF_RAMP
	};
	ASSERT(surface >= 0 && surface < NUM_SURFACES);
	const double blend(SynthValueNoise(X*0.45, 11u + surface));
	const EVec3 albedo(albedos[surface][0]*(1 - blend) + albedos[surface][1]*blend);
	if (surface == SURF_WALL_YMAX && X.x() > lowTextureX[0] && X.x() < lowTextureX[1] && X.z() > lowTextureZ[0] && X.z() < lowTextureZ[1])
		return albedo * (0.6 + 0.05*(SynthFbm(X*1.7, 3, 101u + surface) - 0.5));
	const double detail(CLAMP(0.5 + 2.5*(SynthFbm(X*1.7, 6, 101u + surface) - 0.5), 0.0, 1.0));
	const double spot(SynthSmoothStep(0.62, 0.65, SynthValueNoise(X*3.1 + EVec3(7.3, 1.9, 4.1), 202u + surface)));
	return albedo * ((0.2 + 0.8*detail) * (1 - 0.65*spot));
}

// rotation of a camera looking along forward with the world z up, rolled about its axis
Matrix3x3 SynthLookRotation(const EVec3& forward, double roll)
{
	const EVec3 z(forward.normalized());
	const EVec3 x(EVec3(0, 0, -1).cross(z).normalized());
	ASSERT(ISFINITE(x.x()));
	EMat3 R;
	R.row(0) = x;
	R.row(1) = z.cross(x);
	R.row(2) = z;
	return Matrix3x3(EMat3(Eigen::AngleAxisd(roll, EVec3::UnitZ()).toRotationMatrix() * R));
}

// pose of panorama i of n: x from -5.5 to 5.5 m weaving in y and height, heading along x with
// small yaw, pitch and roll
void SynthPanoPose(unsigned i, unsigned n, Matrix3x3& R, Point3& C)
{
	ASSERT(n >= 2 && i < n);
	const double s(double(i) / (n - 1));
	C = Point3(-5.5 + 11*s, -0.3 + 0.8*std::sin(PI*1.7*s), 1.5 + 0.4*std::sin(2*PI*1.3*s));
	const double yaw(0.3*std::sin(2*PI*s)), pitch(0.06*std::cos(3*PI*s)), roll(0.05*std::sin(4.2*PI*s));
	R = Matrix3x3(EMat3(Eigen::AngleAxisd(pitch, EVec3::UnitX()).toRotationMatrix() * EMat3(SynthLookRotation(EVec3(std::cos(yaw), std::sin(yaw), 0), roll))));
}

// ray-cast GT lambda and normal at the pixel centers and the supersampled color
void SynthRender(SynthView& view, unsigned supersample)
{
	ASSERT(supersample > 0);
	const SynthCamera& cam(view.camera);
	const EMat3 R(cam.R), Rt(R.transpose());
	const EVec3 C(cam.C);
	view.image.create(cam.height, cam.width);
	view.depthMap.create(cam.height, cam.width);
	view.normalMap.create(cam.height, cam.width);
	const double step(1.0 / supersample), weight(1.0 / (supersample*supersample));
	#ifdef _USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	#endif
	for (int r = 0; r < cam.height; ++r) {
		for (int c = 0; c < cam.width; ++c) {
			const EVec3 ray(cam.Ray(EVec2(c, r)));
			const SynthHit hit(SynthTrace(C, Rt*ray));
			const EVec3 normal(R*hit.normal);
			ASSERT(normal.dot(ray) <= 0);
			view.depthMap(r, c) = (Depth)hit.t;
			view.normalMap(r, c) = Normal((float)normal.x(), (float)normal.y(), (float)normal.z());
			EVec3 color(EVec3::Zero());
			for (unsigned sy = 0; sy < supersample; ++sy) {
				for (unsigned sx = 0; sx < supersample; ++sx) {
					const EVec3 d(Rt*cam.Ray(EVec2(c - 0.5 + (sx + 0.5)*step, r - 0.5 + (sy + 0.5)*step)));
					const SynthHit h(SynthTrace(C, d));
					color += SynthColor(C + h.t*d, h.surface);
				}
			}
			color *= weight * 255.0;
			view.image(r, c) = Pixel8U(
				(uint8_t)ROUND2INT(CLAMP(color.x(), 0.0, 255.0)),
				(uint8_t)ROUND2INT(CLAMP(color.y(), 0.0, 255.0)),
				(uint8_t)ROUND2INT(CLAMP(color.z(), 0.0, 255.0)));
		}
	}
}

// points WORKING_FOLDER, where MAKE_PATH places the depth-maps and Scene::Save makes the image
// paths relative to, at folder for its lifetime
struct SynthWorkingFolder {
	String prev, prevFull;
	explicit SynthWorkingFolder(const String& folder) : prev(WORKING_FOLDER), prevFull(WORKING_FOLDER_FULL) {
		WORKING_FOLDER = folder;
		INIT_WORKING_FOLDER;
	}
	~SynthWorkingFolder() {
		WORKING_FOLDER = prev;
		WORKING_FOLDER_FULL = prevFull;
	}
};

const char* SynthModelName(CameraModelType model)
{
	switch (model) {
	case CameraModelType::PINHOLE: return "PINHOLE";
	case CameraModelType::BROWN: return "BROWN";
	case CameraModelType::SPHERICAL: return "SPHERICAL";
	}
	ASSERT("unknown camera model" == NULL);
	return "";
}

} // namespace


SynthCamera::Vec3 SynthCamera::Ray(const EVec2& x) const
{
	switch (model) {
	case CameraModelType::PINHOLE:
		return pinhole.Ray(x);
	case CameraModelType::BROWN: {
		const EVec2 u(distortion.Undistort(pinhole.PixelToNormalized(x)));
		return EVec3(u.x(), u.y(), 1); }
	case CameraModelType::SPHERICAL:
		return spherical.Ray(x);
	}
	ASSERT("unknown camera model" == NULL);
	return EVec3::Zero();
}
bool SynthCamera::Project(const EVec3& X, EVec2& x) const
{
	switch (model) {
	case CameraModelType::PINHOLE:
		return pinhole.Project(X, x);
	case CameraModelType::BROWN:
		x = pinhole.NormalizedToPixel(distortion.Distort(Pinhole<double>::Perspective(X)));
		return X.z() > 0;
	case CameraModelType::SPHERICAL:
		return spherical.Project(X, x);
	}
	ASSERT("unknown camera model" == NULL);
	return false;
}
double SynthCamera::Depth(const EVec3& X) const
{
	return model == CameraModelType::SPHERICAL ? Spherical<double>::Depth(X) : Pinhole<double>::Depth(X);
}
KMatrix SynthCamera::GetK() const
{
	const std::vector<double> params(Params());
	KMatrix K(KMatrix::IDENTITY);
	K(0,0) = params[0]; K(1,1) = params[1];
	K(0,2) = params[2]; K(1,2) = params[3];
	return K;
}
std::vector<double> SynthCamera::Params() const
{
	if (model == CameraModelType::SPHERICAL)
		return {spherical.fx, spherical.fy, spherical.cx, spherical.cy};
	if (model == CameraModelType::PINHOLE)
		return {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy};
	return {pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy,
		distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3, distortion.k4, distortion.k5, distortion.k6};
}
/*----------------------------------------------------------------*/


void MVS::SynthGenerate(const SynthConfig& config, SynthViewArr& views)
{
	ASSERT(config.panoWidth > 0 && config.panoWidth % 8 == 0);
	ASSERT(config.numRenderPanos > 0 && config.firstPano + config.numRenderPanos <= config.numPanos);
	ASSERT(config.numBrown < config.numRenderPanos || config.numBrown == 0);
	views.clear();
	const int faceSize(config.panoWidth / 4);
	const SphereCubeMap::TangentFacesGeometry faces(SphereCubeMap::MakeTangentFacesGeometry(6, faceSize));
	ASSERT(faces.numFaces == 6);
	std::vector<Matrix3x3> panoRs(config.numPanos);
	std::vector<Point3> panoCs(config.numPanos);
	for (unsigned p = 0; p < config.numPanos; ++p)
		SynthPanoPose(p, config.numPanos, panoRs[p], panoCs[p]);
	for (unsigned p = config.firstPano; p < config.firstPano + config.numRenderPanos; ++p) {
		if (config.spherical) {
			SynthView& view = views.AddEmpty();
			view.name = String::FormatString("pano%03u", p);
			view.pano = (int)p;
			view.face = -1;
			view.camera.model = CameraModelType::SPHERICAL;
			view.camera.width = config.panoWidth;
			view.camera.height = config.panoWidth / 2;
			view.camera.spherical = Spherical<double>::FromSize(view.camera.width, view.camera.height);
			view.camera.R = panoRs[p];
			view.camera.C = panoCs[p];
		}
		if (config.faces) {
			for (int f = 0; f < 6; ++f) {
				SynthView& view = views.AddEmpty();
				view.name = String::FormatString("pano%03u_face%d", p, f);
				view.pano = (int)p;
				view.face = f;
				view.camera.model = CameraModelType::PINHOLE;
				view.camera.width = view.camera.height = faceSize;
				view.camera.pinhole = Pinhole<double>{faces.K(0,0), faces.K(1,1), faces.K(0,2), faces.K(1,2)};
				view.camera.R = faces.rotations[f] * panoRs[p];
				view.camera.C = panoCs[p];
			}
		}
	}
	// Brown views halfway between consecutive panoramas, 15 cm higher, looking at the objects
	const EVec3 targets[5] = {sphereCenters[0], rampOrigin + 0.5*(rampEdge1 + rampEdge2), EVec3(pillarAxis.x(), pillarAxis.y(), 2.0), sphereCenters[1], sphereCenters[2]};
	for (unsigned k = 0; k < config.numBrown; ++k) {
		const unsigned p(config.firstPano + k);
		SynthView& view = views.AddEmpty();
		view.name = String::FormatString("brown%03u", p);
		view.pano = (int)p;
		view.face = -1;
		SynthCamera& cam = view.camera;
		cam.model = CameraModelType::BROWN;
		cam.width = config.panoWidth / 2;
		cam.height = cam.width * 3 / 4;
		cam.pinhole = Pinhole<double>{0.6*cam.width, 0.6*cam.width, 0.5*(cam.width - 1) + 0.01*cam.width, 0.5*(cam.height - 1) - 0.01*cam.height};
		cam.distortion = BrownDistortion<double>{-0.22, 0.06, 0.0008, -0.0006, 0, 0, 0, 0};
		cam.C = (panoCs[p] + panoCs[p+1]) * 0.5 + Point3(0, 0, 0.15);
		cam.R = SynthLookRotation(targets[k % 5] - EVec3(cam.C), k % 2 ? 0.05 : -0.05);
	}
	FOREACH(i, views)
		SynthRender(views[i], config.supersample);
}
/*----------------------------------------------------------------*/

double MVS::SynthGTMismatch(const SynthView& a, const SynthView& b, size_t& numPixels)
{
	const EMat3 Ra(a.camera.R), Rb(b.camera.R);
	const EVec3 Ca(a.camera.C), Cb(b.camera.C);
	const bool wrap(b.camera.model == CameraModelType::SPHERICAL);
	const int W(b.depthMap.width()), H(b.depthMap.height());
	FloatArr errors;
	for (int r = 0; r < a.depthMap.height(); ++r) {
		for (int c = 0; c < a.depthMap.width(); ++c) {
			const EVec3 X(Rb*(Ca + Ra.transpose()*(a.camera.Ray(EVec2(c, r))*(double)a.depthMap(r, c)) - Cb));
			// the bounds are tested on the doubles: far outside the Brown field of view the
			// distortion polynomial overflows to inf/NaN
			EVec2 x;
			if (!b.camera.Project(X, x) || !(x.y() >= 0 && x.y() < H - 1) || !(wrap || (x.x() >= 0 && x.x() < W - 1)))
				continue;
			int x0(FLOOR2INT(x.x())), x1(x0 + 1);
			const int y0(FLOOR2INT(x.y()));
			if (wrap) {
				x0 = (x0 + W) % W;
				x1 = (x1 + W) % W;
			}
			const double fx(x.x() - std::floor(x.x())), fy(x.y() - y0);
			const double d00(b.depthMap(y0, x0)), d01(b.depthMap(y0, x1)), d10(b.depthMap(y0+1, x0)), d11(b.depthMap(y0+1, x1));
			// skip depth discontinuities; interpolate 1/lambda, linear in the pixel on pinhole planes
			if (MAXF(MAXF(d00, d01), MAXF(d10, d11)) > 1.02*MINF(MINF(d00, d01), MINF(d10, d11)))
				continue;
			const double lambda(1.0 / ((1/d00*(1-fx) + 1/d01*fx)*(1-fy) + (1/d10*(1-fx) + 1/d11*fx)*fy));
			const double depth(b.camera.Depth(X));
			if (depth > 1.01*lambda)
				continue; // occluded in b
			errors.push_back((float)(ABS(depth - lambda) / lambda));
		}
	}
	numPixels = errors.size();
	ASSERT(numPixels > 0);
	return errors.GetMedian();
}
/*----------------------------------------------------------------*/

bool MVS::SynthBuildScene(const SynthViewArr& views, const IIndexArr& viewIDs, const String& folder, Scene& scene)
{
	ASSERT(!viewIDs.empty());
	scene.Release();
	const std::vector<Matrix3x3> faceRotations(SphereCubeMap::FaceRotations(6));
	std::unordered_map<int, uint32_t> panoPlatforms;
	// the cameras of a panorama rig: its 6 faces, then the panorama itself, the rig pose being the
	// panorama's; a face camera is set up by the first face view of the rig, the spherical one by the panorama
	constexpr uint32_t sphericalCamera(6);
	FOREACH(i, viewIDs) {
		const SynthView& view = views[viewIDs[i]];
		ASSERT((view.camera.model == CameraModelType::PINHOLE && view.face >= 0 && view.face < 6) ||
			(view.camera.model == CameraModelType::SPHERICAL && view.face < 0));
		const auto itPlatform(panoPlatforms.emplace(view.pano, scene.platforms.size()));
		Platform& platform = itPlatform.second ? scene.platforms.AddEmpty() : scene.platforms[itPlatform.first->second];
		if (itPlatform.second) {
			platform.name = String::FormatString("pano%03d", view.pano);
			platform.cameras.resize(sphericalCamera+1);
			for (Platform::Camera& camera: platform.cameras) {
				camera.K = KMatrix::IDENTITY;
				camera.R = RMatrix::IDENTITY;
				camera.C = Point3::ZERO;
			}
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = view.face < 0 ? Matrix3x3(view.camera.R) : Matrix3x3(faceRotations[view.face].t() * view.camera.R);
			pose.C = view.camera.C;
		}
		const float scale(Camera::GetNormalizationScale(view.camera.width, view.camera.height));
		if (view.face < 0) {
			Platform::Camera& camera = platform.cameras[sphericalCamera];
			camera.model = CameraModelType::SPHERICAL;
			camera.K(0,0) = view.camera.spherical.fx / scale;
			camera.K(1,1) = view.camera.spherical.fy / scale;
		} else {
			for (int f = 0; f < 6; ++f) {
				Platform::Camera& camera = platform.cameras[f];
				camera.K(0,0) = view.camera.pinhole.fx / scale;
				camera.K(1,1) = view.camera.pinhole.fy / scale;
				camera.R = faceRotations[f];
			}
		}
		Image& image = scene.images.AddEmpty();
		image.ID = i;
		image.platformID = itPlatform.first->second;
		image.cameraID = view.face < 0 ? sphericalCamera : (uint32_t)view.face;
		image.poseID = 0;
		image.width = view.camera.width;
		image.height = view.camera.height;
		image.scale = 1;
		image.name = folder + view.name + ".png";
		if (!view.image.Save(image.name)) {
			VERBOSE("error: cannot save image '%s'", image.name.c_str());
			return false;
		}
		image.UpdateCamera(scene.platforms);
		ASSERT(view.face < 0 ?
			ABS(image.camera.K(0,2) - view.camera.spherical.cx) < 1e-9 && ABS(image.camera.K(1,1) - view.camera.spherical.fy) < 1e-4 :
			ABS(image.camera.K(0,2) - view.camera.pinhole.cx) < 1e-9 && ABS(image.camera.K(0,0) - view.camera.pinhole.fx) < 1e-4);
	}
	scene.nCalibratedImages = viewIDs.size();
	// GT sparse points: every 8th pixel of every view, seen by every view whose GT agrees within 1%
	constexpr int pointStep(8);
	FOREACH(i, viewIDs) {
		const SynthView& view = views[viewIDs[i]];
		const EMat3 Rt(EMat3(view.camera.R).transpose());
		for (int r = pointStep/2; r < view.camera.height; r += pointStep) {
			for (int c = pointStep/2; c < view.camera.width; c += pointStep) {
				const EVec3 X(EVec3(view.camera.C) + Rt*(view.camera.Ray(EVec2(c, r))*(double)view.depthMap(r, c)));
				PointCloud::ViewArr pointViews;
				FOREACH(j, viewIDs) {
					const SynthView& other = views[viewIDs[j]];
					const EVec3 Xj(EMat3(other.camera.R)*(X - EVec3(other.camera.C)));
					EVec2 x;
					if (!other.camera.Project(Xj, x) || !(x.x() >= 0 && x.y() >= 0 && x.x() <= other.camera.width - 1 && x.y() <= other.camera.height - 1))
						continue;
					const int xc(ROUND2INT(x.x())), yc(ROUND2INT(x.y()));
					if (ABS(other.camera.Depth(Xj) - other.depthMap(yc, xc)) < 0.01*other.depthMap(yc, xc))
						pointViews.push_back(j);
				}
				ASSERT(pointViews.FindFirst(i) != PointCloud::ViewArr::NO_INDEX);
				if (pointViews.size() < 2)
					continue;
				scene.pointcloud.points.emplace_back(Cast<float>(Point3(X)));
				scene.pointcloud.pointViews.emplace_back(std::move(pointViews));
			}
		}
	}
	return true;
}
/*----------------------------------------------------------------*/

bool MVS::SynthSave(const SynthViewArr& views, const String& folder)
{
	Util::ensureFolder(folder);
	IIndexArr faceIDs;
	std::ofstream json((folder + "cameras.json").c_str());
	json << std::setprecision(17);
	json << "{\n\"convention\": \"X_cam = R (X - C); camera x-right, y-down, z-forward; pixel centers at integers; "
		"lambda (the .dmap depth) = z for PINHOLE, z of the undistorted ray for BROWN, range for SPHERICAL; "
		"params PINHOLE/SPHERICAL fx,fy,cx,cy, BROWN fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6\",\n\"views\": [\n";
	FOREACH(i, views) {
		const SynthView& view = views[i];
		const String imageName(folder + view.name + ".png");
		if (view.face >= 0)
			faceIDs.push_back(i);
		else if (!view.image.Save(imageName))
			return false;
		Depth dMin(FLT_MAX), dMax(0);
		for (const Depth d: view.depthMap) {
			dMin = MINF(dMin, d);
			dMax = MAXF(dMax, d);
		}
		if (!ExportDepthDataRaw(folder + view.name + ".dmap", imageName, IIndexArr{i}, view.depthMap.size(),
				view.camera.GetK(), RMatrix(view.camera.R), CMatrix(view.camera.C), dMin, dMax,
				view.depthMap, view.normalMap, ConfidenceMap(), ViewsMap()) ||
			!ExportDepthMap(folder + view.name + ".depth.png", view.depthMap))
			return false;
		json << (i ? ",\n" : "") << "{\"name\": \"" << view.name << "\", \"pano\": " << view.pano << ", \"face\": " << view.face
			<< ", \"model\": \"" << SynthModelName(view.camera.model) << "\", \"width\": " << view.camera.width << ", \"height\": " << view.camera.height
			<< ", \"params\": [";
		const std::vector<double> params(view.camera.Params());
		for (size_t p = 0; p < params.size(); ++p)
			json << (p ? ", " : "") << params[p];
		json << "], \"R\": [";
		for (int k = 0; k < 9; ++k)
			json << (k ? ", " : "") << view.camera.R.val[k];
		json << "], \"C\": [" << view.camera.C.x << ", " << view.camera.C.y << ", " << view.camera.C.z
			<< "], \"image\": \"" << view.name << ".png\", \"gt\": \"" << view.name << ".dmap\"}";
	}
	json << "\n]\n}\n";
	if (!json.good())
		return false;
	if (faceIDs.empty())
		return true;
	const SynthWorkingFolder workingFolder(folder);
	Scene scene;
	return SynthBuildScene(views, faceIDs, folder, scene) && scene.Save(folder + "scene.mvs");
}
/*----------------------------------------------------------------*/


void DepthEvaluator::Add(const DepthMap& gt, const NormalMap& gtNormals, const DepthMap& est, const NormalMap& estNormals, CameraModelType model)
{
	ASSERT(!gt.empty() && gt.size() == est.size() && gt.size() == gtNormals.size());
	ASSERT(estNormals.empty() || estNormals.size() == est.size());
	constexpr int band(16); // seam and border band width in pixels
	const int W(gt.width()), H(gt.height());
	const bool spherical(model == CameraModelType::SPHERICAL);
	const Spherical<double> sphere(Spherical<double>::FromSize(W, H));
	for (int r = 0; r < H; ++r) {
		const double elevation(R2D(sphere.Angles(EVec2(0, r)).y()));
		for (int c = 0; c < W; ++c) {
			const Depth g(gt(r, c));
			if (!(g > 0))
				continue;
			int pixelBands[3], numPixelBands(0);
			pixelBands[numPixelBands++] = ALL;
			if (spherical) {
				if (ABS(elevation) < 30)
					pixelBands[numPixelBands++] = EQUATOR;
				else if (elevation > 70)
					pixelBands[numPixelBands++] = NORTH;
				else if (elevation < -70)
					pixelBands[numPixelBands++] = SOUTH;
				if (c < band || c >= W - band)
					pixelBands[numPixelBands++] = SEAM;
				else if (c >= W/2 - band && c < W/2 + band)
					pixelBands[numPixelBands++] = CENTER; // the antipode of the seam band
			} else {
				pixelBands[numPixelBands++] = (c < band || r < band || c >= W - band || r >= H - band) ? BORDER : CENTER;
			}
			const Depth e(est(r, c));
			float angle(-1);
			if (e > 0 && !estNormals.empty() && estNormals(r, c) != Normal::ZERO) {
				angle = R2D(ACOS(ComputeAngle(estNormals(r, c).ptr(), gtNormals(r, c).ptr())));
			}
			for (int b = 0; b < numPixelBands; ++b) {
				Accum& accum = bands[pixelBands[b]];
				++accum.numGT;
				if (e > 0) {
					accum.absRel.push_back(ABS(e - g) / g);
					if (angle >= 0)
						accum.normalAngle.push_back(angle);
				}
			}
		}
	}
}

DepthEvaluator::Stats DepthEvaluator::GetStats(Band band) const
{
	const Accum& accum = bands[band];
	Stats stats{accum.numGT, accum.absRel.size(), 0, 0, 0, 0, 0, 0, 0};
	if (accum.numGT)
		stats.completeness = double(stats.numEst) / accum.numGT;
	if (!accum.absRel.empty()) {
		double sum(0);
		size_t n1(0), n2(0), n5(0);
		for (const float e: accum.absRel) {
			sum += e;
			n1 += e < 0.01f;
			n2 += e < 0.02f;
			n5 += e < 0.05f;
		}
		stats.absRelMean = sum / stats.numEst;
		stats.absRelMedian = FloatArr(accum.absRel).GetMedian();
		stats.within1 = double(n1) / stats.numEst;
		stats.within2 = double(n2) / stats.numEst;
		stats.within5 = double(n5) / stats.numEst;
	}
	if (!accum.normalAngle.empty())
		stats.normalMedianDeg = FloatArr(accum.normalAngle).GetMedian();
	return stats;
}

String DepthEvaluator::Print() const
{
	static const char* names[NUM_BANDS] = {"all", "equator", "north", "south", "seam", "center", "border"};
	String table("band       GT-px  compl  absrel-mean  absrel-med    <1%    <2%    <5%  normal-med\n");
	for (int b = 0; b < NUM_BANDS; ++b) {
		const Stats s(GetStats((Band)b));
		if (s.numGT == 0)
			continue;
		table += String::FormatString("%-8s %9zu  %5.3f  %11.5f  %10.5f  %5.3f  %5.3f  %5.3f  %8.2fdeg\n",
			names[b], s.numGT, s.completeness, s.absRelMean, s.absRelMedian, s.within1, s.within2, s.within5, s.normalMedianDeg);
	}
	return table;
}
/*----------------------------------------------------------------*/


namespace {
// nanoflann adaptor of a point array
struct PointsAdaptor {
	const PointCloud::PointArr& points;
	inline size_t kdtree_get_point_count() const { return points.size(); }
	inline float kdtree_get_pt(size_t idx, int dim) const { return points[idx][dim]; }
	template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointsAdaptor>, PointsAdaptor, 3> PointsKDTree;
// the squared distance from every query point to its nearest point of the tree
FloatArr NearestDistancesSq(const PointsKDTree& tree, const PointCloud::PointArr& queries)
{
	FloatArr distSq(queries.size());
	#ifdef _USE_OPENMP
	#pragma omp parallel for
	#endif
	for (int64_t i = 0; i < (int64_t)queries.size(); ++i) {
		size_t idx;
		nanoflann::KNNResultSet<float> result(1);
		result.init(&idx, &distSq[(size_t)i]);
		tree.findNeighbors(result, queries[(size_t)i].ptr(), nanoflann::SearchParameters(0, false));
	}
	return distSq;
}
} // namespace

CloudEvaluator::CloudEvaluator(int bandWidth, const float _thresholds[NUM_TH])
	:
	seamDeg(16.0*360.0/bandWidth)
{
	ASSERT(bandWidth > 0);
	for (int t = 0; t < NUM_TH; ++t)
		thresholds[t] = _thresholds[t];
}

void CloudEvaluator::AddView(const Matrix3x3& R, const Point3& C, const DepthMap& ranges, int recallStride)
{
	ASSERT(!ranges.empty() && ranges.width() == 2*ranges.height() && recallStride > 0);
	const Spherical<double> sphere(Spherical<double>::FromSize(ranges.width(), ranges.height()));
	const EMat3 Rt(EMat3(R).transpose());
	for (int r = 0; r < ranges.height(); ++r) {
		for (int c = 0; c < ranges.width(); ++c) {
			const Depth range(ranges(r, c));
			if (!(range > 0))
				continue;
			gtPoints.emplace_back(Cast<float>(C + Point3(EVec3(Rt*(sphere.Ray(EVec2(c, r))*(double)range)))));
			if (r % recallStride == recallStride/2 && c % recallStride == recallStride/2)
				gtRecallPoints.emplace_back(gtPoints.back());
		}
	}
	views.push_back(View{R, C, ranges});
}

bool CloudEvaluator::Classify(const Point3f& X, float& range, int pointBands[3], int& numBands) const
{
	ASSERT(!views.empty());
	const Point3 Xd(Cast<REAL>(X));
	size_t best(0);
	REAL bestDistSq(normSq(Xd - views[0].C));
	for (size_t v = 1; v < views.size(); ++v) {
		const REAL distSq(normSq(Xd - views[v].C));
		if (bestDistSq > distSq) {
			bestDistSq = distSq;
			best = v;
		}
	}
	const View& view = views[best];
	const Point3 Xc(view.R*(Xd - view.C));
	const Spherical<double> sphere(Spherical<double>::FromSize(view.ranges.width(), view.ranges.height()));
	EVec2 x;
	sphere.Project(EVec3(Xc.x, Xc.y, Xc.z), x);
	const ImageRef ir(ModelCamera::WrapColumn(ROUND2INT(x.x()), view.ranges.width()), MINF(MAXF(ROUND2INT(x.y()), 0), view.ranges.height()-1));
	if (!(view.ranges(ir.y, ir.x) > 0))
		return false;
	range = (float)SQRT(bestDistSq);
	const EVec2 angles(sphere.Angles(x));
	const double elevation(R2D(angles.y())), longitude(R2D(angles.x()));
	numBands = 0;
	pointBands[numBands++] = DepthEvaluator::ALL;
	if (ABS(elevation) < 30)
		pointBands[numBands++] = DepthEvaluator::EQUATOR;
	else if (elevation > 70)
		pointBands[numBands++] = DepthEvaluator::NORTH;
	else if (elevation < -70)
		pointBands[numBands++] = DepthEvaluator::SOUTH;
	if (ABS(longitude) > 180 - seamDeg)
		pointBands[numBands++] = DepthEvaluator::SEAM;
	return true;
}

void CloudEvaluator::Evaluate(const PointCloud::PointArr& cloud)
{
	ASSERT(!gtPoints.empty() && !cloud.empty());
	for (Stats& s: bands)
		s = Stats{};
	size_t hits[DepthEvaluator::NUM_BANDS][2][NUM_TH] = {};
	// 0: the cloud points against the GT (precision), 1: the GT points against the cloud (recall)
	for (int side = 0; side < 2; ++side) {
		const PointCloud::PointArr& queries(side ? gtRecallPoints : cloud);
		const PointsAdaptor adaptor{side ? cloud : gtPoints};
		PointsKDTree tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(16));
		tree.buildIndex();
		const FloatArr distSq(NearestDistancesSq(tree, queries));
		FOREACH(i, queries) {
			float range;
			int pointBands[3], numBands;
			if (!Classify(queries[i], range, pointBands, numBands))
				continue;
			for (int b = 0; b < numBands; ++b) {
				++(side ? bands[pointBands[b]].numGT : bands[pointBands[b]].numCloud);
				for (int t = 0; t < NUM_TH; ++t)
					if (distSq[i] <= SQUARE(thresholds[t]*range))
						++hits[pointBands[b]][side][t];
			}
		}
	}
	for (int b = 0; b < DepthEvaluator::NUM_BANDS; ++b) {
		Stats& s = bands[b];
		for (int t = 0; t < NUM_TH; ++t) {
			s.precision[t] = s.numCloud ? double(hits[b][0][t]) / s.numCloud : 0;
			s.recall[t] = s.numGT ? double(hits[b][1][t]) / s.numGT : 0;
		}
	}
}

String CloudEvaluator::Print() const
{
	static const char* names[DepthEvaluator::NUM_BANDS] = {"all", "equator", "north", "south", "seam", "center", "border"};
	String table("band       GT-pts  cloud-pts");
	for (int t = 0; t < NUM_TH; ++t)
		table += String::FormatString("   P@%.1f%%  R@%.1f%%  F@%.1f%%", thresholds[t]*100, thresholds[t]*100, thresholds[t]*100);
	table += "\n";
	for (int b = 0; b < DepthEvaluator::NUM_BANDS; ++b) {
		const Stats& s = bands[b];
		if (s.numGT == 0 && s.numCloud == 0)
			continue;
		table += String::FormatString("%-8s %9zu  %9zu", names[b], s.numGT, s.numCloud);
		for (int t = 0; t < NUM_TH; ++t) {
			const double sum(s.precision[t] + s.recall[t]);
			table += String::FormatString("   %6.4f  %6.4f  %6.4f", s.precision[t], s.recall[t], sum > 0 ? 2*s.precision[t]*s.recall[t]/sum : 0.0);
		}
		table += "\n";
	}
	return table;
}
/*----------------------------------------------------------------*/


// CPU (or, if gpu, CUDA) PatchMatch on the given views of the synthetic scene, at full resolution,
// depth-maps only, in a temporary folder; the estimates are scored against the GT into evaluator, the
// estimation wall time goes to seconds; maxViews > 0 caps the neighbor views of a depth-map, maxThreads > 0
// the threads; the caller checks that a CUDA device is available for gpu (SynthCUDAAvailable)
static bool SynthEstimateDepth(const SynthViewArr& views, const IIndexArr& viewIDs, DepthEvaluator& evaluator, double& seconds, unsigned maxViews=0, unsigned maxThreads=0, bool gpu=false)
{
	const ScopedTempDir tmpDir(_T("SyntheticDepth"));
	if (!tmpDir.IsValid())
		return false;
	const String folder(tmpDir.Path() + "/");
	const SynthWorkingFolder workingFolder(folder);
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	struct DevicesRestore {
		const String prev;
		~DevicesRestore() { SEACAVE::CUDA::desiredDeviceIDs = prev; }
	} const devicesRestore{SEACAVE::CUDA::desiredDeviceIDs};
	SEACAVE::CUDA::desiredDeviceIDs = gpu ? String("-1") : String();
	#else
	ASSERT(!gpu);
	#endif
	Scene scene;
	if (!SynthBuildScene(views, viewIDs, folder, scene))
		return false;
	if (maxThreads)
		scene.nMaxThreads = maxThreads;
	OPTDENSE::init();
	OPTDENSE::update();
	OPTDENSE::nResolutionLevel = 0;
	OPTDENSE::nMinResolution = 64;
	// full resolution, as the GT: the default cap (3200 px) would scale a 4K panorama down
	for (const IIndex viewID: viewIDs)
		OPTDENSE::nMaxResolution = MAXF(OPTDENSE::nMaxResolution, (unsigned)MAXF(views[viewID].camera.width, views[viewID].camera.height));
	if (maxViews)
		OPTDENSE::nMaxViews = OPTDENSE::nNumViews = maxViews;
	const auto t0(std::chrono::steady_clock::now());
	if (!scene.DenseReconstruction(1)) {
		VERBOSE("ERROR: synthetic depth: depth-map estimation failed!");
		return false;
	}
	seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	FOREACH(i, scene.images) {
		String imageFileName;
		IIndexArr IDs;
		cv::Size imageSize;
		KMatrix K; RMatrix R; CMatrix C;
		Depth dMin, dMax;
		DepthMap depthMap; NormalMap normalMap; ConfidenceMap confMap; ViewsMap viewsMap;
		if (!ImportDepthDataRaw(ComposeDepthFilePath(scene.images[i].ID, "dmap"), imageFileName, IDs, imageSize, K, R, C, dMin, dMax,
				depthMap, normalMap, confMap, viewsMap, HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL)) {
			VERBOSE("ERROR: synthetic depth: missing depth-map of image %u!", i);
			return false;
		}
		const SynthView& view = views[viewIDs[i]];
		evaluator.Add(view.depthMap, view.normalMap, depthMap, normalMap, view.camera.model);
	}
	return true;
}

bool MVS::SyntheticDepthBaselineTest()
{
	TD_TIMER_START();
	SynthConfig config;
	config.firstPano = 3;
	config.numRenderPanos = 3;
	config.numBrown = 2;
	config.supersample = 3;
	SynthViewArr views;
	SynthGenerate(config, views);
	// GT consistency across the models: panorama -> panorama, panorama <-> face, Brown <-> panorama/face
	IIndex panos[3], faces[3][6], browns[2];
	FOREACH(i, views) {
		const SynthView& view = views[i];
		if (view.camera.model == CameraModelType::BROWN)
			browns[view.pano - 3] = i;
		else if (view.face < 0)
			panos[view.pano - 3] = i;
		else
			faces[view.pano - 3][view.face] = i;
	}
	// the face of the first panorama looking closest to where the first Brown view looks
	const auto forward([&](IIndex i) { return EVec3(EMat3(views[i].camera.R).row(2).transpose()); });
	int brownFace(0);
	for (int f = 1; f < 6; ++f)
		if (forward(faces[0][f]).dot(forward(browns[0])) > forward(faces[0][brownFace]).dot(forward(browns[0])))
			brownFace = f;
	const std::pair<IIndex,IIndex> pairs[] = {
		{panos[0], panos[1]}, {panos[1], faces[0][0]}, {faces[2][2], panos[1]}, {faces[0][4], faces[1][4]},
		{browns[0], panos[1]}, {panos[0], browns[1]}, {faces[0][brownFace], browns[0]}};
	for (const auto& pair: pairs) {
		size_t numPixels;
		const double mismatch(SynthGTMismatch(views[pair.first], views[pair.second], numPixels));
		VERBOSE("SyntheticDepthBaselineTest: GT %s -> %s: median lambda mismatch %.2e over %zu px",
			views[pair.first].name.c_str(), views[pair.second].name.c_str(), mismatch, numPixels);
		if (!(mismatch < 1e-4) || numPixels < 1000) {
			VERBOSE("ERROR: SyntheticDepthBaselineTest: inconsistent GT %s -> %s (%.2e over %zu px)!",
				views[pair.first].name.c_str(), views[pair.second].name.c_str(), mismatch, numPixels);
			return false;
		}
	}

	// CPU PatchMatch on the 18 faces, full resolution, depth-maps only
	IIndexArr faceIDs;
	FOREACH(i, views)
		if (views[i].face >= 0)
			faceIDs.push_back(i);
	DepthEvaluator evaluator;
	double seconds;
	if (!SynthEstimateDepth(views, faceIDs, evaluator, seconds))
		return false;
	VERBOSE("SyntheticDepthBaselineTest: CPU PatchMatch on %u cube faces of %dx%d px:\n%s",
		faceIDs.size(), views[faceIDs[0]].camera.width, views[faceIDs[0]].camera.height, evaluator.Print().c_str());
	// measured (Debug, CPU, two runs): all: completeness 0.854, abs-rel median 0.0027, <1% 0.749..0.754,
	// <5% 0.920..0.922, normal median 4.0..4.1 deg; center completeness 0.988, border 0.682
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL));
	const DepthEvaluator::Stats center(evaluator.GetStats(DepthEvaluator::CENTER)), border(evaluator.GetStats(DepthEvaluator::BORDER));
	if (!(all.completeness > 0.80) || !(all.absRelMedian < 0.004) || !(all.within1 > 0.70) || !(all.within5 > 0.88) ||
		!(all.normalMedianDeg < 6) || !(center.completeness > 0.95) || !(border.completeness > 0.60)) {
		VERBOSE("ERROR: SyntheticDepthBaselineTest: depth accuracy below the baseline!");
		return false;
	}
	VERBOSE("SyntheticDepthBaselineTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// whether a CUDA device runs the GPU PatchMatch (CI machines have none)
static bool SynthCUDAAvailable()
{
	#ifdef _USE_CUDA
	return SEACAVE::CUDA::isEnabled() || SEACAVE::CUDA::initDevices("-1") == CUDA_SUCCESS;
	#else
	return false;
	#endif
}

// CPU (or, if gpu, CUDA) PatchMatch on panoramas of the synthetic scene (and, if faces, separately on
// their cube faces): prints the accuracy per band and the estimation wall time per estimated pixel;
// returns the evaluators
static bool SynthSphericalRun(int panoWidth, unsigned numPanos, bool faces, DepthEvaluator& sphereEval, DepthEvaluator& faceEval, unsigned maxViews=0, unsigned maxThreads=0, bool gpu=false)
{
	LPCTSTR const backend(gpu ? _T("CUDA") : _T("CPU"));
	SynthConfig config;
	config.panoWidth = panoWidth;
	config.firstPano = 3;
	config.numRenderPanos = numPanos;
	config.numBrown = 0;
	config.faces = faces;
	config.supersample = 3;
	SynthViewArr views;
	SynthGenerate(config, views);
	IIndexArr panoIDs, faceIDs;
	FOREACH(i, views)
		(views[i].face < 0 ? panoIDs : faceIDs).push_back(i);
	double seconds;
	if (!SynthEstimateDepth(views, panoIDs, sphereEval, seconds, maxViews, maxThreads, gpu))
		return false;
	const DepthEvaluator::Stats sphereAll(sphereEval.GetStats(DepthEvaluator::ALL));
	VERBOSE("synthetic spherical: %s PatchMatch on %u panoramas of %dx%d px: %.2f s, %.3f us/px\n%s",
		backend, panoIDs.size(), views[panoIDs[0]].camera.width, views[panoIDs[0]].camera.height,
		seconds, seconds*1e6/sphereAll.numGT, sphereEval.Print().c_str());
	if (faces) {
		if (!SynthEstimateDepth(views, faceIDs, faceEval, seconds, maxViews, maxThreads, gpu))
			return false;
		const DepthEvaluator::Stats faceAll(faceEval.GetStats(DepthEvaluator::ALL));
		VERBOSE("synthetic spherical: %s PatchMatch on their %u cube faces of %dx%d px: %.2f s, %.3f us/px\n%s",
			backend, faceIDs.size(), views[faceIDs[0]].camera.width, views[faceIDs[0]].camera.height,
			seconds, seconds*1e6/faceAll.numGT, faceEval.Print().c_str());
	}
	return true;
}

// CPU PatchMatch and fusion (OPTDENSE::nFuseFilter = fusionFilter) of the given views of the synthetic
// scene, at full resolution, in a temporary folder: the fused points, and the wall time in seconds
static bool SynthFuseDepth(const SynthViewArr& views, const IIndexArr& viewIDs, unsigned fusionFilter, PointCloud::PointArr& points, double& seconds, unsigned maxThreads=0)
{
	const ScopedTempDir tmpDir(_T("SyntheticFusion"));
	if (!tmpDir.IsValid())
		return false;
	const String folder(tmpDir.Path() + "/");
	const SynthWorkingFolder workingFolder(folder);
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	struct DevicesRestore {
		const String prev;
		~DevicesRestore() { SEACAVE::CUDA::desiredDeviceIDs = prev; }
	} const devicesRestore{SEACAVE::CUDA::desiredDeviceIDs};
	SEACAVE::CUDA::desiredDeviceIDs.clear();
	#endif
	Scene scene;
	if (!SynthBuildScene(views, viewIDs, folder, scene))
		return false;
	if (maxThreads)
		scene.nMaxThreads = maxThreads;
	OPTDENSE::init();
	OPTDENSE::update();
	OPTDENSE::nResolutionLevel = 0;
	OPTDENSE::nMinResolution = 64;
	OPTDENSE::nFuseFilter = fusionFilter;
	const auto t0(std::chrono::steady_clock::now());
	if (!scene.DenseReconstruction(0) || scene.pointcloud.IsEmpty()) {
		VERBOSE("ERROR: synthetic fusion: dense reconstruction failed!");
		return false;
	}
	seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	points = std::move(scene.pointcloud.points);
	return true;
}

// the GT range map of a panorama of the synthetic scene ray-cast at the given width
static DepthMap SynthPanoRanges(const SynthView& pano, int width)
{
	ASSERT(pano.camera.model == CameraModelType::SPHERICAL);
	const Spherical<double> sphere(Spherical<double>::FromSize(width, width/2));
	const EMat3 Rt(EMat3(pano.camera.R).transpose());
	DepthMap ranges(cv::Size(width, width/2));
	for (int r = 0; r < ranges.height(); ++r)
		for (int c = 0; c < ranges.width(); ++c)
			ranges(r, c) = (Depth)SynthTrace(EVec3(pano.camera.C), Rt*sphere.Ray(EVec2(c, r))).t;
	return ranges;
}

// CPU PatchMatch and fusion of panoramas of the synthetic scene and, if faces is 1, separately of
// their cube faces, if 2, of the panoramas and their faces together (a mixed scene); the clouds
// scored by CloudEvaluator against the GT of the panoramas ray-cast at up to 4x their width; prints
// the tables
static bool SynthSphericalFusionRun(int panoWidth, unsigned numPanos, int faces, unsigned fusionFilter, const float thresholds[CloudEvaluator::NUM_TH],
	CloudEvaluator& sphereEval, CloudEvaluator& faceEval, unsigned maxThreads=0)
{
	SynthConfig config;
	config.panoWidth = panoWidth;
	config.firstPano = 3;
	config.numRenderPanos = numPanos;
	config.numBrown = 0;
	config.faces = faces;
	config.supersample = 3;
	SynthViewArr views;
	SynthGenerate(config, views);
	IIndexArr panoIDs, faceIDs;
	FOREACH(i, views)
		(views[i].face < 0 ? panoIDs : faceIDs).push_back(i);
	const int gtWidth(MINF(4*panoWidth, 4096));
	for (IIndex i: panoIDs) {
		const DepthMap ranges(SynthPanoRanges(views[i], gtWidth));
		sphereEval.AddView(views[i].camera.R, views[i].camera.C, ranges, gtWidth/panoWidth);
		if (faces)
			faceEval.AddView(views[i].camera.R, views[i].camera.C, ranges, gtWidth/panoWidth);
	}
	PointCloud::PointArr points;
	double seconds;
	if (!SynthFuseDepth(views, panoIDs, fusionFilter, points, seconds, maxThreads))
		return false;
	sphereEval.Evaluate(points);
	VERBOSE("synthetic spherical fusion: %u panoramas of %dx%d px: %u points, %.2f s\n%s",
		panoIDs.size(), panoWidth, panoWidth/2, points.size(), seconds, sphereEval.Print().c_str());
	if (faces) {
		IIndexArr viewIDs(faceIDs);
		if (faces == 2)
			viewIDs.Join(panoIDs);
		if (!SynthFuseDepth(views, viewIDs, fusionFilter, points, seconds, maxThreads))
			return false;
		faceEval.Evaluate(points);
		VERBOSE("synthetic spherical fusion: their %u cube faces of %dx%d px%s: %u points, %.2f s\n%s",
			faceIDs.size(), views[faceIDs[0]].camera.width, views[faceIDs[0]].camera.height, faces == 2 ? " with the panoramas" : "",
			points.size(), seconds, faceEval.Print().c_str());
	}
	return true;
}

// the cost of scoring one plane hypothesis at one pixel against one view, through the PatchMatch
// estimator of the reference model: ns per ScorePixel of the GT plane over the reference pixels whose
// patch the target sees whole (score below thRobust), each pixel prepared once and scored rounds times
template<template<typename> class MODEL>
static double SynthScoreBench(const SynthView& ref, const SynthView& trg, unsigned rounds, size_t& numPixels)
{
	Image images[2];
	DepthData depthData;
	const SynthView* views[2] = {&ref, &trg};
	for (int i = 0; i < 2; ++i) {
		const SynthView& view = *views[i];
		Image& image = images[i];
		image.ID = i;
		image.poseID = 0;
		image.model = view.camera.model;
		image.width = view.camera.width;
		image.height = view.camera.height;
		image.camera.K = view.camera.GetK();
		image.camera.R = view.camera.R;
		image.camera.C = view.camera.C;
		image.camera.ComposeP();
		DepthData::ViewData& viewData = depthData.images.AddEmpty();
		viewData.scale = 1;
		viewData.pImageData = &image;
		viewData.camera = image.camera;
		Image8U gray;
		cv::cvtColor(view.image, gray, cv::COLOR_BGR2GRAY);
		gray.convertTo(viewData.image, CV_32F, 1.0/255.0);
	}
	depthData.images[1].Init(depthData.images[0].camera);
	const cv::Size size(ref.camera.width, ref.camera.height);
	depthData.depthMap.create(size); depthData.normalMap.create(size); depthData.confMap.create(size);
	depthData.dMin = 0.1f; depthData.dMax = 100.f;
	DepthEstimatorBase::WeightMap weightMap;
	weightMap.resize(size.area());
	DepthEstimatorBase::MapRefArr coords;
	volatile Thread::safe_t idxPixel(-1);
	std::unique_ptr<DepthEstimatorBase::PixelPending[]> pending(new DepthEstimatorBase::PixelPending[size.area()]);
	DepthEstimatorT<MODEL> estimator(0, depthData, idxPixel, pending.get(), weightMap, coords);
	// the reference pixels seen whole by the target
	std::vector<ImageRef> pixels;
	for (int r = DepthEstimatorBase::nSizeHalfWindow; r < size.height-DepthEstimatorBase::nSizeHalfWindow; r += 2) {
		for (int c = DepthEstimatorBase::nSizeHalfWindow; c < size.width-DepthEstimatorBase::nSizeHalfWindow; c += 2) {
			const ImageRef x(c, r);
			if (!estimator.PreparePixelPatch(x) || !estimator.FillPixelPatch())
				continue;
			if (estimator.ScorePixel(ref.depthMap(x), ref.normalMap(x)) < estimator.thRobust)
				pixels.push_back(x);
		}
	}
	numPixels = pixels.size();
	if (pixels.empty())
		return 0;
	float sum(0);
	const auto t0(std::chrono::steady_clock::now());
	for (const ImageRef& x: pixels) {
		estimator.PreparePixelPatch(x);
		estimator.FillPixelPatch();
		for (unsigned round = 0; round < rounds; ++round)
			sum += estimator.ScorePixel(ref.depthMap(x), ref.normalMap(x));
	}
	const double seconds(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
	ASSERT(sum >= 0);
	return seconds*1e9/(double(rounds)*pixels.size());
}

bool MVS::SyntheticSphericalTest()
{
	TD_TIMER_START();
	DepthEvaluator evaluator, unused;
	if (!SynthSphericalRun(512, 3, false, evaluator, unused))
		return false;
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL)), equator(evaluator.GetStats(DepthEvaluator::EQUATOR));
	const DepthEvaluator::Stats seam(evaluator.GetStats(DepthEvaluator::SEAM)), center(evaluator.GetStats(DepthEvaluator::CENTER));
	const DepthEvaluator::Stats north(evaluator.GetStats(DepthEvaluator::NORTH)), south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// measured (Release, CPU, 5 runs): all: completeness 0.955, abs-rel median 0.0015, <1% 0.92, <5% 0.982,
	// normal median 4.0 deg; equator completeness 0.991, <5% 0.956..0.960; north 0.857, <5% 1.000; south
	// 0.797, <5% 0.981..0.986 (the 4 rows next to each pole are not estimated); seam 0.948, <5%
	// 0.976..0.978; center 0.953, <5% 0.979..0.980.
	// The panoramas look along their trajectory, so the epipoles of their neighbors lie next to the seam
	// and next to longitude 0 (pano004 at columns 15 and 271 of 512, pano005 at 3-4 and 259-260): there a
	// neighbor sees the scene below fMinAngle, the per-pixel view gate drops it, and the pixels no view
	// constrains get wrong depths, so the seam band cannot be expected to match the equator. It is
	// compared with its antipodal band (CENTER) instead, which holds the mirrored epipoles of the same
	// views at the same latitudes. The seam is 0.001..0.003 <5% below it (Release and Debug: it holds a few
	// more of those pixels), while a seam artifact costs it 0.03 or more (no propagation across the seam:
	// <5% 0.950 vs 0.979; target patches across the seam unseen: 0.91 vs 0.97, completeness 0.940 vs
	// 0.953), hence the 0.01 margin.
	if (!(all.completeness > 0.93) || !(all.absRelMedian < 0.0025) || !(all.within1 > 0.88) || !(all.within5 > 0.97) || !(all.normalMedianDeg < 5.5) ||
		!(seam.completeness > 0.99*center.completeness) || !(seam.within5 > center.within5 - 0.01) ||
		!(north.completeness > 0.8) || !(north.within5 > 0.95) || !(south.completeness > 0.75) || !(south.within5 > 0.95)) {
		VERBOSE("ERROR: SyntheticSphericalTest: depth accuracy below the baseline!");
		return false;
	}
	VERBOSE("SyntheticSphericalTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool MVS::SyntheticSphericalCUDATest(bool forceCPU)
{
	if (forceCPU || !SynthCUDAAvailable()) {
		VERBOSE("SyntheticSphericalCUDATest skipped: no CUDA device");
		return true;
	}
	TD_TIMER_START();
	DepthEvaluator evaluator, unused;
	if (!SynthSphericalRun(512, 3, false, evaluator, unused, 0, 0, true))
		return false;
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL)), equator(evaluator.GetStats(DepthEvaluator::EQUATOR));
	const DepthEvaluator::Stats seam(evaluator.GetStats(DepthEvaluator::SEAM)), center(evaluator.GetStats(DepthEvaluator::CENTER));
	const DepthEvaluator::Stats north(evaluator.GetStats(DepthEvaluator::NORTH)), south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// measured (Release, CUDA): all: completeness 0.964, abs-rel median 0.0014, <1% 0.93, <5% 0.989, normal
	// median 3.2 deg; equator completeness 0.991, <5% 0.974; seam 0.969, <5% 0.994; north 0.857, <5% 1.000;
	// south 0.849, <5% 0.989; center (the seam's antipode) 0.966, <5% 0.990; every band at or above the CPU
	// estimator (SyntheticSphericalTest); the seam is held to its antipodal band as there
	if (!(all.completeness > 0.94) || !(all.absRelMedian < 0.002) || !(all.within1 > 0.90) || !(all.within5 > 0.975) || !(all.normalMedianDeg < 4.5) ||
		!(seam.completeness > 0.99*center.completeness) || !(seam.within5 > center.within5 - 0.01) ||
		!(north.completeness > 0.8) || !(north.within5 > 0.97) || !(south.completeness > 0.8) || !(south.within5 > 0.97)) {
		VERBOSE("ERROR: SyntheticSphericalCUDATest: depth accuracy below the CPU baseline!");
		return false;
	}
	VERBOSE("SyntheticSphericalCUDATest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool MVS::SyntheticSphericalFusionTest()
{
	TD_TIMER_START();
	const float thresholds[CloudEvaluator::NUM_TH] = {0.005f, 0.01f, 0.02f};
	CloudEvaluator evaluator(512, thresholds), unused(512, thresholds);
	if (!SynthSphericalFusionRun(512, 3, 0, OPTDENSE::FUSE_DENSEFILTER, thresholds, evaluator, unused))
		return false;
	const CloudEvaluator::Stats& all(evaluator.GetStats(DepthEvaluator::ALL)), &equator(evaluator.GetStats(DepthEvaluator::EQUATOR));
	const CloudEvaluator::Stats& seam(evaluator.GetStats(DepthEvaluator::SEAM));
	const CloudEvaluator::Stats& north(evaluator.GetStats(DepthEvaluator::NORTH)), &south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// measured (Release, CPU, dense fusion): all: precision@1% 0.984, recall@2% 0.871 (82k points);
	// equator 0.963 / 0.722; seam 0.980 / 0.899; north 1.000 / 1.000; south 0.998 / 0.902
	if (!(all.precision[1] > 0.96) || !(all.recall[2] > 0.83) ||
		!(seam.precision[1] > 0.97*equator.precision[1]) || !(seam.recall[2] > equator.recall[2]) ||
		!(north.precision[1] > 0.97) || !(south.precision[1] > 0.97) || !(north.recall[2] > 0.95) || !(south.recall[2] > 0.85)) {
		VERBOSE("ERROR: SyntheticSphericalFusionTest: fused cloud below the baseline!");
		return false;
	}
	VERBOSE("SyntheticSphericalFusionTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


bool MVS::BrownUndistortOnLoadTest()
{
	TD_TIMER_START();
	SynthConfig config;
	config.panoWidth = 1024;
	config.spherical = false;
	config.faces = false;
	config.numRenderPanos = 3;
	config.numBrown = 2;
	SynthViewArr views;
	SynthGenerate(config, views);
	ASSERT(views.size() == 2 && views[0].camera.model == CameraModelType::BROWN);
	const ScopedTempDir tmpDir(_T("BrownUndistortOnLoadTest"));
	if (!tmpDir.IsValid())
		return false;

	// the Brown views as an Interface v8 scene: one camera (the views share the lens), its
	// original image files, and GT points observed in the distorted images
	const SynthCamera& lens = views[0].camera;
	Interface obj;
	Interface::Platform& platform = obj.platforms.emplace_back();
	Interface::Platform::Camera& camera = platform.cameras.emplace_back();
	camera.model = Interface::Platform::Camera::BROWN;
	camera.width = (uint32_t)lens.width;
	camera.height = (uint32_t)lens.height;
	camera.K = lens.GetK();
	camera.R = Interface::Mat33d::eye();
	camera.C = Interface::Pos3d(0, 0, 0);
	camera.params = {lens.distortion.k1, lens.distortion.k2, lens.distortion.p1, lens.distortion.p2, lens.distortion.k3};
	// two images per view (the views look at different objects, so a point seen by two images
	// is seen by the two images of one view): images 2*v and 2*v+1 are view v
	FOREACH(v, views) {
		const SynthView& view = views[v];
		ASSERT(view.camera.model == CameraModelType::BROWN && view.camera.Params() == lens.Params());
		if (!view.image.Save(tmpDir(view.name + ".png")))
			return false;
		Interface::Platform::Pose& pose = platform.poses.emplace_back();
		pose.R = view.camera.R;
		pose.C = view.camera.C;
		for (int k = 0; k < 2; ++k) {
			Interface::Image& image = obj.images.emplace_back();
			image.name = tmpDir(view.name + ".png");
			image.platformID = 0;
			image.cameraID = 0;
			image.poseID = (uint32_t)v;
			image.ID = (uint32_t)obj.images.size()-1;
		}
	}
	// GT points at a grid of pixel centers of each view, observed at their distorted projection
	std::vector<std::pair<Point3,IIndex>> points; // point, view
	FOREACH(v, views) {
		const SynthCamera& cam = views[v].camera;
		for (int r = 8; r < lens.height; r += 16) {
			for (int c = 8; c < lens.width; c += 16) {
				const EVec3 X(EMat3(cam.R).transpose() * (views[v].depthMap(r, c) * cam.Ray(EVec2(c, r))) + EVec3(cam.C));
				EVec2 x;
				if (!cam.Project(EMat3(cam.R) * (X - EVec3(cam.C)), x))
					continue;
				Interface::Vertex& vertex = obj.vertices.emplace_back();
				vertex.X = Interface::Pos3f((float)X.x(), (float)X.y(), (float)X.z());
				for (int k = 0; k < 2; ++k) {
					Interface::Vertex::View& view = vertex.views.emplace_back();
					view.imageID = (uint32_t)(2*v + k);
					view.confidence = 0;
					view.x = Interface::Pos2f((float)x.x(), (float)x.y());
				}
				points.emplace_back(Point3(X.x(), X.y(), X.z()), v);
			}
		}
	}
	obj.flags |= Interface::HAS_OBSERVATIONS;
	const String fileName(tmpDir("scene.mvs"));
	if (!ARCHIVE::SerializeSave(obj, fileName))
		return false;

	// load the scene: each image read through MVS::Image is the pinhole render at K'
	Scene scene;
	if (scene.Load(fileName) != Scene::SCENE_INTERFACE || scene.pointcloud.pointObservations.size() != points.size()) {
		VERBOSE("ERROR: BrownUndistortOnLoadTest: scene load failed");
		return false;
	}
	FOREACH(v, views) {
		const IIndex i(2*v);
		Image& imageData = scene.images[i];
		if (!imageData.ReloadImage(0))
			return false;
		ASSERT(imageData.IsUndistortedOnRead() && imageData.image.size() == cv::Size(lens.width, lens.height));
		SynthView pinhole;
		pinhole.camera.model = CameraModelType::PINHOLE;
		pinhole.camera.width = lens.width;
		pinhole.camera.height = lens.height;
		const KMatrix& K = imageData.camera.K;
		pinhole.camera.pinhole = Pinhole<double>{K(0,0), K(1,1), K(0,2), K(1,2)};
		pinhole.camera.R = views[v].camera.R;
		pinhole.camera.C = views[v].camera.C;
		SynthRender(pinhole, config.supersample);
		// the valid pixels match the render, the others are masked through the ignore mask
		const Image8U& valid = imageData.GetUndistortMaps()->mask;
		BitMatrix unusedMask;
		Image8U ignoreMask;
		if (!DepthEstimator::ImportIgnoreMask(imageData, imageData.image.size(), -1, unusedMask, &ignoreMask) ||
			cv::countNonZero(ignoreMask != valid) != 0) {
			VERBOSE("ERROR: BrownUndistortOnLoadTest: the ignore mask is not the undistortion mask");
			return false;
		}
		double sumDiff(0);
		size_t numValid(0);
		for (int r = 0; r < lens.height; ++r) {
			for (int c = 0; c < lens.width; ++c) {
				if (!valid(r, c))
					continue;
				const Pixel8U& a = imageData.image(r, c);
				const Pixel8U& b = pinhole.image(r, c);
				sumDiff += (std::abs((int)a.r - (int)b.r) + std::abs((int)a.g - (int)b.g) + std::abs((int)a.b - (int)b.b)) / 3.0;
				++numValid;
			}
		}
		const double meanDiff(sumDiff / numValid);
		const double invalidRatio(1.0 - (double)numValid / valid.area());
		// observations: the GT points of this view through the pinhole camera K', in both its images
		double maxObsErr(0);
		size_t numPoints(0);
		FOREACH(j, points) {
			if (points[j].second != v)
				continue;
			EVec2 x;
			const EVec3 X(points[j].first.x, points[j].first.y, points[j].first.z);
			pinhole.camera.Project(EMat3(pinhole.camera.R) * (X - EVec3(pinhole.camera.C)), x);
			const PointCloud::ViewArr& pointViews = scene.pointcloud.pointViews[(uint32_t)j];
			const PointCloud::ObservationArr& observations = scene.pointcloud.pointObservations[(uint32_t)j];
			ASSERT(pointViews.size() == 2 && pointViews[0] == i && pointViews[1] == i+1);
			for (const Point2f& o: observations)
				maxObsErr = MAXF(maxObsErr, (EVec2(o.x, o.y) - x).norm());
			++numPoints;
		}
		VERBOSE("BrownUndistortOnLoadTest: view %u: mean abs difference %.3f over %zu valid px, %.2f%% invalid, max observation error %.2e px (%u points)",
			v, meanDiff, numValid, invalidRatio*100, maxObsErr, (unsigned)numPoints);
		// measured: mean abs difference 0.96 (supersampled render vs bilinear remap), 4.6% invalid
		if (!(meanDiff < 1.5) || !(invalidRatio > 0.01 && invalidRatio < 0.1) || !(maxObsErr < 1e-3) || numPoints < 100) {
			VERBOSE("ERROR: BrownUndistortOnLoadTest: the undistorted view %u does not match the pinhole render at K'", v);
			return false;
		}
	}
	VERBOSE("BrownUndistortOnLoadTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


bool MVS::SynthTool(int argc, LPCTSTR* argv)
{
	ASSERT(argc > 0);
	const String mode(argv[0]);
	if (mode == "synth" && argc >= 2) {
		SynthConfig config;
		if (argc > 2)
			config.panoWidth = std::atoi(argv[2]);
		if (config.panoWidth <= 0 || config.panoWidth % 8) {
			std::printf("error: panoWidth must be a positive multiple of 8\n");
			return false;
		}
		String folder(Util::getFullPath(argv[1]));
		Util::ensureValidFolderPath(folder);
		TD_TIMER_START();
		SynthViewArr views;
		SynthGenerate(config, views);
		std::printf("rendered %u views (%s)\n", views.size(), TD_TIMER_GET_FMT().c_str());
		// GT consistency: each panorama against the next, its first face and the Brown view after it
		for (const SynthView& a: views) {
			for (const SynthView& b: views) {
				if (&a == &b || a.camera.model != CameraModelType::SPHERICAL ||
					!((b.camera.model == CameraModelType::SPHERICAL && b.pano == a.pano + 1) || (b.pano == a.pano && b.face <= 0)))
					continue;
				size_t numPixels;
				const double mismatch(SynthGTMismatch(a, b, numPixels));
				std::printf("GT %s -> %s: median lambda mismatch %.2e over %zu px\n", a.name.c_str(), b.name.c_str(), mismatch, numPixels);
			}
		}
		if (!SynthSave(views, folder)) {
			std::printf("error: cannot write the dataset to '%s'\n", folder.c_str());
			return false;
		}
		std::printf("saved to '%s' (%s)\n", folder.c_str(), TD_TIMER_GET_FMT().c_str());
		return true;
	}
	if (mode == "synthtest")
		return SyntheticDepthBaselineTest();
	if (mode == "scorebench") {
		// ns per hypothesis and view: scorebench <verbosity> [panoWidth=1536] [rounds=4] [maxElevation] [faceWidth=panoWidth/4]
		// (faces as large as the panorama, panoWidth/sqrt(2), compare at the same image memory footprint)
		const int panoWidth(argc > 2 ? std::atoi(argv[2]) : 1536);
		const unsigned rounds(argc > 3 ? (unsigned)std::atoi(argv[3]) : 4u);
		struct MaxElevationRestore {
			const float prev;
			~MaxElevationRestore() { DepthEstimatorBase::fSphericalWarpMaxElevation = prev; }
		} const maxElevationRestore{DepthEstimatorBase::fSphericalWarpMaxElevation};
		if (argc > 4)
			DepthEstimatorBase::fSphericalWarpMaxElevation = (float)std::atof(argv[4]);
		const int faceWidth(argc > 5 ? std::atoi(argv[5]) : panoWidth/4);
		OPTDENSE::init();
		OPTDENSE::update();
		// panorama 3 against panorama 4, and each cube face of 3 against the same face of 4
		SynthConfig config;
		config.firstPano = 3;
		config.numRenderPanos = 2;
		config.numBrown = 0;
		config.supersample = 1;
		config.panoWidth = panoWidth;
		config.faces = false;
		SynthViewArr panos;
		SynthGenerate(config, panos);
		config.panoWidth = faceWidth*4;
		config.spherical = false;
		config.faces = true;
		SynthViewArr faces;
		SynthGenerate(config, faces);
		// alternate the two measurements and keep the fastest of each, the machine being shared
		double nsSphereMin(DBL_MAX), nsFacesMin(DBL_MAX);
		for (int repeat = 0; repeat < 5; ++repeat) {
			size_t numPixels;
			const double nsSphere(SynthScoreBench<Spherical>(panos[0], panos[1], rounds, numPixels));
			double nsFaces(0); size_t numFacePixels(0);
			for (int f = 0; f < 6; ++f) {
				size_t n;
				const double ns(SynthScoreBench<Pinhole>(faces[f], faces[6+f], rounds, n));
				nsFaces += ns*n;
				numFacePixels += n;
			}
			nsFaces /= numFacePixels;
			VERBOSE("scorebench: panorama %dx%d %.1f ns over %zu pixels, faces %dx%d %.1f ns over %zu pixels",
				panoWidth, panoWidth/2, nsSphere, numPixels, faceWidth, faceWidth, nsFaces, numFacePixels);
			nsSphereMin = MINF(nsSphereMin, nsSphere);
			nsFacesMin = MINF(nsFacesMin, nsFaces);
		}
		VERBOSE("scorebench: spherical %.1f ns, pinhole faces %.1f ns per ScorePixel (one view): ratio %.3f", nsSphereMin, nsFacesMin, nsSphereMin/nsFacesMin);
		return true;
	}
	if (mode == "sphtest") {
		// panoramas vs their cube faces: sphtest <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [maxElevation] [maxViews] [maxThreads] [gpu=0]
		const int panoWidth(argc > 2 ? std::atoi(argv[2]) : 512);
		const unsigned numPanos(argc > 3 ? (unsigned)std::atoi(argv[3]) : 3u);
		const bool faces(argc > 4 ? std::atoi(argv[4]) != 0 : true);
		struct MaxElevationRestore {
			const float prev;
			~MaxElevationRestore() { DepthEstimatorBase::fSphericalWarpMaxElevation = prev; }
		} const maxElevationRestore{DepthEstimatorBase::fSphericalWarpMaxElevation};
		if (argc > 5)
			DepthEstimatorBase::fSphericalWarpMaxElevation = (float)std::atof(argv[5]);
		const unsigned maxViews(argc > 6 ? (unsigned)std::atoi(argv[6]) : 0u);
		const unsigned maxThreads(argc > 7 ? (unsigned)std::atoi(argv[7]) : 0u);
		const bool gpu(argc > 8 && std::atoi(argv[8]) != 0);
		if (gpu && !SynthCUDAAvailable()) {
			std::printf("error: no CUDA device\n");
			return false;
		}
		if (panoWidth <= 0 || panoWidth % 8 || numPanos < 2 || numPanos > 5) {
			std::printf("error: panoWidth must be a positive multiple of 8, numPanos in [2,5]\n");
			return false;
		}
		DepthEvaluator sphereEval, faceEval;
		if (!SynthSphericalRun(panoWidth, numPanos, faces, sphereEval, faceEval, maxViews, maxThreads, gpu))
			return false;
		std::fputs(sphereEval.Print().c_str(), stdout);
		if (faces)
			std::fputs(faceEval.Print().c_str(), stdout);
		return true;
	}
	if (mode == "sphfuse") {
		// panoramas vs their cube faces, fused: sphfuse <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [fusionFilter=2] [maxThreads]
		const int panoWidth(argc > 2 ? std::atoi(argv[2]) : 512);
		const unsigned numPanos(argc > 3 ? (unsigned)std::atoi(argv[3]) : 3u);
		const int faces(argc > 4 ? std::atoi(argv[4]) : 1);
		const unsigned fusionFilter(argc > 5 ? (unsigned)std::atoi(argv[5]) : (unsigned)OPTDENSE::FUSE_DENSEFILTER);
		const unsigned maxThreads(argc > 6 ? (unsigned)std::atoi(argv[6]) : 0u);
		if (panoWidth <= 0 || panoWidth % 8 || numPanos < 2 || numPanos > 5 || fusionFilter > 2) {
			std::printf("error: panoWidth must be a positive multiple of 8, numPanos in [2,5], fusionFilter in [0,2]\n");
			return false;
		}
		const float thresholds[CloudEvaluator::NUM_TH] = {0.005f, 0.01f, 0.02f};
		CloudEvaluator sphereEval(panoWidth, thresholds), faceEval(panoWidth, thresholds);
		if (!SynthSphericalFusionRun(panoWidth, numPanos, faces, fusionFilter, thresholds, sphereEval, faceEval, maxThreads))
			return false;
		std::fputs(sphereEval.Print().c_str(), stdout);
		if (faces)
			std::fputs(faceEval.Print().c_str(), stdout);
		return true;
	}
	if (mode == "evalcloud" && argc >= 3) {
		// a cloud against the GT range maps depth*.dmap of a folder: evalcloud <cloud.ply|scene.mvs> <gtFolder> <evalWidth> [th1% th2% th3%]
		// (evalWidth: the panorama width the cloud was estimated at, for the recall grid and the seam band)
		const int evalWidth(argc > 3 ? std::atoi(argv[3]) : 0);
		float thresholds[CloudEvaluator::NUM_TH] = {0.01f, 0.02f, 0.05f};
		for (int t = 0; t < CloudEvaluator::NUM_TH && 4+t < argc; ++t)
			thresholds[t] = (float)std::atof(argv[4+t])/100;
		const String cloudFileName(Util::getFullPath(argv[1]));
		PointCloud pointcloud;
		if (Util::getFileExt(cloudFileName).ToLower() == ".mvs") {
			Scene scene;
			const SynthWorkingFolder workingFolder(Util::getFilePath(cloudFileName));
			if (scene.Load(cloudFileName) == Scene::SCENE_NA)
				return false;
			pointcloud = std::move(scene.pointcloud);
		} else if (!pointcloud.Load(cloudFileName)) {
			std::printf("error: cannot read the cloud '%s'\n", cloudFileName.c_str());
			return false;
		}
		if (pointcloud.IsEmpty()) {
			std::printf("error: empty cloud '%s'\n", cloudFileName.c_str());
			return false;
		}
		std::vector<std::string> names;
		for (const auto& entry: std::filesystem::directory_iterator(argv[2]))
			if (entry.is_regular_file() && entry.path().extension() == ".dmap")
				names.push_back(entry.path().filename().string());
		std::sort(names.begin(), names.end());
		std::unique_ptr<CloudEvaluator> evaluator;
		for (const std::string& name: names) {
			String imageFileName; IIndexArr IDs; cv::Size imageSize; KMatrix K; RMatrix R; CMatrix C; Depth dMin, dMax;
			DepthMap ranges; NormalMap normals; ConfidenceMap confMap; ViewsMap viewsMap;
			if (!ImportDepthDataRaw((std::filesystem::path(argv[2]) / name).string(), imageFileName, IDs, imageSize, K, R, C, dMin, dMax,
					ranges, normals, confMap, viewsMap, HeaderDepthDataRaw::HAS_DEPTH) || ranges.width() != 2*ranges.height()) {
				std::printf("error: '%s' is not an equirectangular range map\n", name.c_str());
				return false;
			}
			if (!evaluator)
				evaluator.reset(new CloudEvaluator(evalWidth > 0 ? evalWidth : ranges.width(), thresholds));
			evaluator->AddView(R, C, ranges, evalWidth > 0 ? MAXF(ROUND2INT(float(ranges.width())/evalWidth), 1) : 1);
		}
		if (!evaluator) {
			std::printf("error: no GT range map in '%s'\n", argv[2]);
			return false;
		}
		evaluator->Evaluate(pointcloud.points);
		VERBOSE("evalcloud: %zu GT range maps, %u cloud points\n%s", names.size(), pointcloud.points.size(), evaluator->Print().c_str());
		return true;
	}
	if (mode == "cameratest")
		return InterfaceV8Test() && BrownUndistortOnLoadTest();
	if (mode == "facescene" && argc >= 5) {
		// the cube-face baseline of a spherical scene: facescene <scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize]:
		// every panorama resized to panoWidth is rendered into 6 cube faces (faceSize, default the same pixels
		// per steradian: panoWidth/sqrt(12)), its GT range depth-map (gtFolder/depthXXXX.dmap) into face GT z
		// depth-maps (outFolder/gt), and the sparse points are kept with the faces they project into
		const String sceneFileName(Util::getFullPath(argv[1])), gtFolder(Util::getFullPath(argv[2]));
		String outFolder(Util::getFullPath(argv[3]));
		Util::ensureValidFolderPath(outFolder);
		Util::ensureFolder(outFolder + "gt/");
		const int panoWidth(std::atoi(argv[4]));
		const int faceSize(argc > 5 ? std::atoi(argv[5]) : ROUND2INT(panoWidth/SQRT(12.0)/8)*8);
		Scene scene;
		{
			const SynthWorkingFolder workingFolder(Util::getFilePath(sceneFileName));
			if (scene.Load(sceneFileName) == Scene::SCENE_NA)
				return false;
		}
		const SphereCubeMap::TangentFacesGeometry faces(SphereCubeMap::MakeTangentFacesGeometry(6, faceSize));
		const Pinhole<double> facePinhole{faces.K(0,0), faces.K(1,1), faces.K(0,2), faces.K(1,2)};
		Scene faceScene;
		const float scale(Camera::GetNormalizationScale(faceSize, faceSize));
		FOREACH(i, scene.images) {
			const Image& imageData = scene.images[i];
			ASSERT(imageData.IsValid() && imageData.model == CameraModelType::SPHERICAL);
			Image8U3 pano;
			if (imageData.ReadPixels(pano) == NULL)
				return false;
			cv::resize(pano, pano, cv::Size(panoWidth, panoWidth/2), 0, 0, cv::INTER_AREA);
			const std::vector<Image8U3> faceImages(SphereCubeMap::SphericalToTangentialFaces(pano, faces));
			// the GT range of the panorama, sampled at the nearest pixel of each face ray
			String imageFileName; IIndexArr IDs; cv::Size imageSize; KMatrix K; RMatrix R; CMatrix C; Depth dMin, dMax;
			DepthMap gtRange; NormalMap gtNormals; ConfidenceMap confMap; ViewsMap viewsMap;
			if (!ImportDepthDataRaw(gtFolder + String::FormatString("/depth%04u.dmap", imageData.ID), imageFileName, IDs, imageSize, K, R, C, dMin, dMax,
					gtRange, gtNormals, confMap, viewsMap, HeaderDepthDataRaw::HAS_DEPTH))
				return false;
			const Spherical<double> sphere(Spherical<double>::FromSize(gtRange.width(), gtRange.height()));
			Platform& platform = faceScene.platforms.AddEmpty();
			platform.name = String::FormatString("pano%04u", imageData.ID);
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = imageData.camera.R;
			pose.C = imageData.camera.C;
			for (int f = 0; f < 6; ++f) {
				Platform::Camera& camera = platform.cameras.AddEmpty();
				camera.K = KMatrix::IDENTITY;
				camera.K(0,0) = faces.K(0,0) / scale;
				camera.K(1,1) = faces.K(1,1) / scale;
				camera.R = faces.rotations[f];
				camera.C = Point3::ZERO;
				Image& face = faceScene.images.AddEmpty();
				face.ID = faceScene.images.size()-1;
				face.platformID = faceScene.platforms.size()-1;
				face.cameraID = f;
				face.poseID = 0;
				face.width = face.height = faceSize;
				face.scale = 1;
				face.name = outFolder + String::FormatString("pano%04u_face%d.png", imageData.ID, f);
				if (!faceImages[f].Save(face.name))
					return false;
				face.UpdateCamera(faceScene.platforms);
				DepthMap gt(faceSize, faceSize);
				const Eigen::Matrix3d Rt(EMat3(faces.rotations[f]).transpose());
				for (int r = 0; r < faceSize; ++r) {
					for (int c = 0; c < faceSize; ++c) {
						const EVec3 ray(facePinhole.Ray(EVec2(c, r)));
						EVec2 x;
						sphere.Project(Rt*ray, x);
						const Depth range(gtRange(MINF(ROUND2INT(x.y()), gtRange.height()-1), (ROUND2INT(x.x()) + gtRange.width()) % gtRange.width()));
						gt(r, c) = range > 0 ? (Depth)(range/ray.norm()) : 0.f;
					}
				}
				const NormalMap noNormals;
				if (!ExportDepthDataRaw(outFolder + String::FormatString("gt/depth%04u.dmap", face.ID), face.name, IIndexArr{face.ID}, gt.size(),
						face.camera.K, face.camera.R, face.camera.C, 0.f, FLT_MAX, gt, noNormals, ConfidenceMap(), ViewsMap()))
					return false;
			}
		}
		faceScene.nCalibratedImages = faceScene.images.size();
		// the sparse points with the faces they project into, among the faces of their panoramas
		FOREACH(p, scene.pointcloud.points) {
			const Point3 X(Cast<REAL>(scene.pointcloud.points[p]));
			PointCloud::ViewArr views;
			for (const PointCloud::View v: scene.pointcloud.pointViews[p]) {
				for (int f = 0; f < 6; ++f) {
					const Image& face = faceScene.images[v*6+f];
					const Point3 Xc(face.camera.TransformPointW2C(X));
					if (Xc.z > 0 && face.camera.IsInside(Point2f(face.camera.TransformPointC2I(Xc)), Point2f(face.GetSize())))
						views.push_back(v*6+f);
				}
			}
			if (views.size() < 2)
				continue;
			views.Sort();
			faceScene.pointcloud.points.emplace_back(scene.pointcloud.points[p]);
			faceScene.pointcloud.pointViews.emplace_back(std::move(views));
		}
		const SynthWorkingFolder workingFolder(outFolder);
		if (!faceScene.SaveInterface(outFolder + "scene_faces.mvs"))
			return false;
		std::printf("%u faces of %dx%d px, %zu sparse points\n", faceScene.images.size(), faceSize, faceSize, (size_t)faceScene.pointcloud.points.size());
		return true;
	}
	if (mode == "undistortbench" && argc >= 2) {
		// image load cost of the undistortion over the first numImages images undistorted on read:
		// the file decode and the remap ReadPixels adds to it, timed apart; the first image also
		// builds the maps shared by its camera; the image paths of the scene are relative to its folder
		String fileName(argv[1]);
		Util::ensureValidPath(fileName);
		fileName = MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName);
		WORKING_FOLDER = Util::getFilePath(fileName);
		INIT_WORKING_FOLDER;
		Scene scene;
		if (scene.Load(fileName) == Scene::SCENE_NA)
			return false;
		const unsigned numImages(argc >= 3 ? (unsigned)std::atoi(argv[2]) : 8u);
		typedef std::chrono::steady_clock Clock;
		const auto ms = [](Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double,std::milli>(b - a).count(); };
		double decodeMs(0), remapMs(0), mapsMs(0);
		unsigned n(0);
		for (const Image& imageData: scene.images) {
			if (n == numImages)
				break;
			if (!imageData.IsUndistortedOnRead())
				continue;
			Image8U3 pixels, undistorted;
			const Clock::time_point t0(Clock::now());
			if (Image::ReadImage(imageData.name, pixels) == NULL)
				return false;
			const Clock::time_point t1(Clock::now());
			const std::shared_ptr<const CameraUndistortMaps> maps(imageData.GetUndistortMaps());
			const Clock::time_point t2(Clock::now());
			maps->Remap(pixels, undistorted, cv::INTER_LINEAR);
			const Clock::time_point t3(Clock::now());
			VERBOSE("image %u %dx%d: decode %.1f ms, maps %.1f ms, remap %.1f ms", imageData.ID, pixels.width(), pixels.height(),
				ms(t0, t1), ms(t1, t2), ms(t2, t3));
			decodeMs += ms(t0, t1);
			mapsMs += ms(t1, t2);
			remapMs += ms(t2, t3);
			++n;
		}
		if (n == 0) {
			VERBOSE("error: no image undistorted on read");
			return false;
		}
		VERBOSE("%u images undistorted on read: decode %.1f ms/image, remap %.1f ms/image, maps %.1f ms in total",
			n, decodeMs/n, remapMs/n, mapsMs);
		return true;
	}
	if (mode == "evaldepth" && argc >= 3) {
		// one GT/estimate pair, or every GT depth*.dmap of a folder against the same file name in the
		// estimate folder (a missing estimate counts as no estimate)
		const String model(argc > 3 ? String(argv[3]) : String());
		DepthEvaluator evaluator;
		const auto Evaluate = [&](const String& gtFileName, const String& estFileName) {
			DepthMap maps[2]; NormalMap normals[2]; KMatrix Ks[2];
			for (int i = 0; i < 2; ++i) {
				String imageFileName;
				IIndexArr IDs;
				cv::Size imageSize;
				RMatrix R; CMatrix C;
				Depth dMin, dMax;
				ConfidenceMap confMap; ViewsMap viewsMap;
				if (!ImportDepthDataRaw(i ? estFileName : gtFileName, imageFileName, IDs, imageSize, Ks[i], R, C, dMin, dMax, maps[i], normals[i], confMap, viewsMap,
						HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL)) {
					if (i == 0) {
						std::printf("error: cannot read the depth-map '%s'\n", gtFileName.c_str());
						return false;
					}
					std::printf("warning: no estimate '%s'\n", estFileName.c_str());
					maps[1].create(maps[0].size());
					maps[1].memset(0);
				}
			}
			if (normals[0].empty()) {
				normals[0].create(maps[0].size());
				normals[0].memset(0);
			}
			// the model only selects the bands: SPHERICAL if named or if the GT camera matrix is the equirect one
			const Spherical<double> sphere(Spherical<double>::FromSize(maps[0].width(), maps[0].height()));
			const bool spherical(model.empty() ? ABS(Ks[0](0,0) - sphere.fx) < 1e-9*sphere.fx && ABS(Ks[0](1,1) - sphere.fy) < 1e-9*sphere.fy : model == "spherical");
			// a GT of another resolution is sampled at the nearest pixel of each estimated pixel center
			if (maps[0].size() != maps[1].size()) {
				if (int64_t(maps[0].width())*maps[1].height() != int64_t(maps[0].height())*maps[1].width()) {
					std::printf("error: GT %dx%d and estimate %dx%d differ in aspect ratio\n", maps[0].width(), maps[0].height(), maps[1].width(), maps[1].height());
					return false;
				}
				const double scale(double(maps[0].width()) / maps[1].width());
				DepthMap gt(maps[1].size());
				NormalMap gtNormals(maps[1].size());
				for (int r = 0; r < gt.height(); ++r) {
					for (int c = 0; c < gt.width(); ++c) {
						const int rg(MINF(FLOOR2INT((r + 0.5)*scale), maps[0].height() - 1)), cg(MINF(FLOOR2INT((c + 0.5)*scale), maps[0].width() - 1));
						gt(r, c) = maps[0](rg, cg);
						gtNormals(r, c) = normals[0](rg, cg);
					}
				}
				maps[0] = std::move(gt);
				normals[0] = std::move(gtNormals);
			}
			evaluator.Add(maps[0], normals[0], maps[1], normals[1], spherical ? CameraModelType::SPHERICAL : CameraModelType::PINHOLE);
			return true;
		};
		if (std::filesystem::is_directory(argv[1])) {
			std::vector<std::string> names;
			for (const auto& entry: std::filesystem::directory_iterator(argv[1]))
				if (entry.is_regular_file() && entry.path().extension() == ".dmap")
					names.push_back(entry.path().filename().string());
			std::sort(names.begin(), names.end());
			for (const std::string& name: names)
				if (!Evaluate((std::filesystem::path(argv[1]) / name).string(), (std::filesystem::path(argv[2]) / name).string()))
					return false;
			std::printf("%zu depth-maps\n", names.size());
		} else if (!Evaluate(argv[1], argv[2])) {
			return false;
		}
		VERBOSE("evaldepth:\n%s", evaluator.Print().c_str());
		return true;
	}
	std::printf("usage:\n"
		"  Tests synth <outDir> [panoWidth=512]\n"
		"  Tests evaldepth <gt.dmap> <est.dmap> [pinhole|brown|spherical, default from the GT camera matrix]\n"
		"  Tests sphtest <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [maxElevation: exact warp above, degrees] [maxViews] [maxThreads] [gpu=0]\n"
		"  Tests facescene <scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize]\n"
		"  Tests sphfuse <verbosity> [panoWidth=512] [numPanos=3] [faces=1] [fusionFilter=2] [maxThreads]\n"
		"  Tests evalcloud <cloud.ply|scene.mvs> <gtFolder> <evalWidth> [th1%=1 th2%=2 th3%=5]\n");
	return false;
}
/*----------------------------------------------------------------*/
