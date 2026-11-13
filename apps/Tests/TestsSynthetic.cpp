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
#include "TestsSynthetic.h"
#include <chrono>
#include <fstream>
#ifdef _USE_METAL
#include "../../libs/MVS/PatchMatchMetal.h"
#include <random>
#endif

using namespace MVS;


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

// points WORKING_FOLDER, where MAKE_PATH places the depth-maps and Scene::Load/Save resolve the
// image paths, at folder for its lifetime
struct ScopedWorkingFolder {
	String prev, prevFull;
	explicit ScopedWorkingFolder(const String& folder) : prev(WORKING_FOLDER), prevFull(WORKING_FOLDER_FULL) {
		WORKING_FOLDER = folder;
		INIT_WORKING_FOLDER;
	}
	~ScopedWorkingFolder() {
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

// the maps (the normals if asked and stored) and the camera of a .dmap file
struct DMapFile {
	DepthMap depthMap;
	NormalMap normalMap;
	KMatrix K; RMatrix R; CMatrix C;
	IIndexArr IDs; // the reference view, then its neighbors
	bool Load(const String& fileName, bool normals) {
		String imageFileName; cv::Size imageSize; Depth dMin, dMax;
		ConfidenceMap confMap; ViewsMap viewsMap;
		return ImportDepthDataRaw(fileName, imageFileName, IDs, imageSize, K, R, C, dMin, dMax, depthMap, normalMap, confMap, viewsMap,
			normals ? HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL : HeaderDepthDataRaw::HAS_DEPTH);
	}
};

// the sorted names of the .dmap files of a folder
StringArr ListDMaps(const String& folder)
{
	StringArr names;
	for (const auto& entry: std::filesystem::directory_iterator(folder.c_str()))
		if (entry.is_regular_file() && entry.path().extension() == ".dmap")
			names.emplace_back(entry.path().filename().string());
	names.Sort();
	return names;
}
String JoinPath(const String& folder, const String& name)
{
	return String((std::filesystem::path(folder.c_str()) / name.c_str()).string());
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
			faceIDs.push_back(i); // its image is saved with the scene
		else if (!view.image.Save(imageName))
			return false;
		Depth dMin(FLT_MAX), dMax(0);
		for (const Depth d: view.depthMap) {
			dMin = MINF(dMin, d);
			dMax = MAXF(dMax, d);
		}
		if (!ExportDepthDataRaw(folder + view.name + ".dmap", imageName, IIndexArr{i}, view.depthMap.size(),
				view.camera.GetK(), RMatrix(view.camera.R), CMatrix(view.camera.C), dMin, dMax,
				view.depthMap, view.normalMap, ConfidenceMap(), ViewsMap(), false, view.camera.model == CameraModelType::SPHERICAL) ||
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
	const ScopedWorkingFolder workingFolder(folder);
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
		const Band latitudeBand(LatitudeBand(R2D(sphere.Angles(EVec2(0, r)).y())));
		for (int c = 0; c < W; ++c) {
			const Depth g(gt(r, c));
			if (!(g > 0))
				continue;
			int pixelBands[3], numPixelBands(0);
			pixelBands[numPixelBands++] = ALL;
			if (spherical) {
				if (latitudeBand != ALL)
					pixelBands[numPixelBands++] = latitudeBand;
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

DepthEvaluator::Band DepthEvaluator::LatitudeBand(double elevationDeg)
{
	if (ABS(elevationDeg) < 30)
		return EQUATOR;
	if (elevationDeg > 70)
		return NORTH;
	if (elevationDeg < -70)
		return SOUTH;
	return ALL;
}

const char* DepthEvaluator::BandName(Band band)
{
	static const char* const names[NUM_BANDS] = {"all", "equator", "north", "south", "seam", "center", "border"};
	ASSERT(band >= 0 && band < NUM_BANDS);
	return names[band];
}

String DepthEvaluator::Print() const
{
	String table("band       GT-px  compl  absrel-mean  absrel-med    <1%    <2%    <5%  normal-med\n");
	for (int b = 0; b < NUM_BANDS; ++b) {
		const Stats s(GetStats((Band)b));
		if (s.numGT == 0)
			continue;
		table += String::FormatString("%-8s %9zu  %5.3f  %11.5f  %10.5f  %5.3f  %5.3f  %5.3f  %8.2fdeg\n",
			BandName((Band)b), s.numGT, s.completeness, s.absRelMean, s.absRelMedian, s.within1, s.within2, s.within5, s.normalMedianDeg);
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
	const ImageRef ir(WrapColumn(ROUND2INT(x.x()), view.ranges.width()), MINF(MAXF(ROUND2INT(x.y()), 0), view.ranges.height()-1));
	if (!(view.ranges(ir.y, ir.x) > 0))
		return false;
	range = (float)SQRT(bestDistSq);
	const EVec2 angles(sphere.Angles(x));
	numBands = 0;
	pointBands[numBands++] = DepthEvaluator::ALL;
	const DepthEvaluator::Band latitudeBand(DepthEvaluator::LatitudeBand(R2D(angles.y())));
	if (latitudeBand != DepthEvaluator::ALL)
		pointBands[numBands++] = latitudeBand;
	if (ABS(R2D(angles.x())) > 180 - seamDeg)
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
	String table("band       GT-pts  cloud-pts");
	for (int t = 0; t < NUM_TH; ++t)
		table += String::FormatString("   P@%.1f%%  R@%.1f%%  F@%.1f%%", thresholds[t]*100, thresholds[t]*100, thresholds[t]*100);
	table += "\n";
	for (int b = 0; b < DepthEvaluator::NUM_BANDS; ++b) {
		const Stats& s = bands[b];
		if (s.numGT == 0 && s.numCloud == 0)
			continue;
		table += String::FormatString("%-8s %9zu  %9zu", DepthEvaluator::BandName((DepthEvaluator::Band)b), s.numGT, s.numCloud);
		for (int t = 0; t < NUM_TH; ++t) {
			const double sum(s.precision[t] + s.recall[t]);
			table += String::FormatString("   %6.4f  %6.4f  %6.4f", s.precision[t], s.recall[t], sum > 0 ? 2*s.precision[t]*s.recall[t]/sum : 0.0);
		}
		table += "\n";
	}
	return table;
}
/*----------------------------------------------------------------*/


// how SynthDense runs the dense reconstruction; 0 keeps the default
struct SynthDenseOptions {
	unsigned maxViews = 0; // neighbor views of a depth-map
	unsigned maxThreads = 0;
	bool gpu = false; // GPU PatchMatch (CUDA, or Metal on Apple); the caller checks SynthGPUAvailable()
	unsigned fusionFilter = OPTDENSE::FUSE_DENSEFILTER; // OPTDENSE::nFuseFilter
	Point3 translation = Point3::ZERO; // the scene is moved by it before the reconstruction
};

// Scene::DenseReconstruction(fusionMode) of the given views in a temporary folder, at full resolution (as
// the GT); result(scene) runs inside that folder, seconds receives the reconstruction wall time
template <typename RESULT>
static bool SynthDense(const SynthViewArr& views, const IIndexArr& viewIDs, const SynthDenseOptions& options, int fusionMode, double& seconds, const RESULT& result)
{
	const ScopedTempDir tmpDir(_T("Synthetic"));
	if (!tmpDir.IsValid())
		return false;
	const String folder(tmpDir.Path() + "/");
	const ScopedWorkingFolder workingFolder(folder);
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	struct DevicesRestore {
		const String prev;
		~DevicesRestore() { SEACAVE::CUDA::desiredDeviceIDs = prev; }
	} const devicesRestore{SEACAVE::CUDA::desiredDeviceIDs};
	SEACAVE::CUDA::desiredDeviceIDs = options.gpu ? String("-1") : String();
	#else
	ASSERT(!options.gpu);
	#endif
	Scene scene;
	if (!SynthBuildScene(views, viewIDs, folder, scene))
		return false;
	if (options.maxThreads)
		scene.nMaxThreads = options.maxThreads;
	if (options.translation != Point3::ZERO)
		scene.Transform(Matrix3x3::IDENTITY, options.translation, 1);
	OPTDENSE::init();
	OPTDENSE::update();
	OPTDENSE::nResolutionLevel = 0;
	OPTDENSE::nMinResolution = 64;
	// the default cap (3200 px) would scale a 4K panorama down
	for (const IIndex viewID: viewIDs)
		OPTDENSE::nMaxResolution = MAXF(OPTDENSE::nMaxResolution, (unsigned)MAXF(views[viewID].camera.width, views[viewID].camera.height));
	if (options.maxViews)
		OPTDENSE::nMaxViews = OPTDENSE::nNumViews = options.maxViews;
	OPTDENSE::nFuseFilter = options.fusionFilter;
	const auto t0(std::chrono::steady_clock::now());
	if (!scene.DenseReconstruction(fusionMode)) {
		VERBOSE("ERROR: synthetic scene: dense reconstruction failed!");
		return false;
	}
	seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	return result(scene);
}

// PatchMatch depth-maps of the given views scored against their GT into evaluator
static bool SynthEstimateDepth(const SynthViewArr& views, const IIndexArr& viewIDs, const SynthDenseOptions& options, DepthEvaluator& evaluator, double& seconds)
{
	return SynthDense(views, viewIDs, options, 1, seconds, [&](const Scene& scene) {
		FOREACH(i, scene.images) {
			DMapFile dmap;
			if (!dmap.Load(ComposeDepthFilePath(scene.images[i].ID, "dmap"), true)) {
				VERBOSE("ERROR: synthetic scene: missing depth-map of image %u!", i);
				return false;
			}
			const SynthView& view = views[viewIDs[i]];
			evaluator.Add(view.depthMap, view.normalMap, dmap.depthMap, dmap.normalMap, view.camera.model);
		}
		return true;
	});
}

// PatchMatch and fusion of the given views: the fused points
static bool SynthFuseDepth(const SynthViewArr& views, const IIndexArr& viewIDs, const SynthDenseOptions& options, PointCloud::PointArr& points, double& seconds)
{
	return SynthDense(views, viewIDs, options, 0, seconds, [&](Scene& scene) {
		if (scene.pointcloud.IsEmpty()) {
			VERBOSE("ERROR: synthetic scene: empty fused cloud!");
			return false;
		}
		points = std::move(scene.pointcloud.points);
		return true;
	});
}

// numPanos panoramas from the middle of the trajectory (index 3 on), with their cube faces if asked,
// no Brown views, 3x3 supersampled
static SynthConfig SynthMiddleConfig(int panoWidth, unsigned numPanos, bool faces)
{
	SynthConfig config;
	config.panoWidth = panoWidth;
	config.firstPano = 3;
	config.numRenderPanos = numPanos;
	config.numBrown = 0;
	config.faces = faces;
	config.supersample = 3;
	return config;
}

// the panorama and the cube-face views
static void SynthSplitViews(const SynthViewArr& views, IIndexArr& panoIDs, IIndexArr& faceIDs)
{
	FOREACH(i, views) {
		ASSERT(views[i].camera.model != CameraModelType::BROWN);
		(views[i].face < 0 ? panoIDs : faceIDs).push_back(i);
	}
}

// whether a GPU runs the GPU PatchMatch: a CUDA device, or Metal on Apple (CI machines have none)
static bool SynthGPUAvailable()
{
	#ifdef _USE_CUDA
	return SEACAVE::CUDA::isEnabled() || SEACAVE::CUDA::initDevices("-1") == CUDA_SUCCESS;
	#elif defined(_USE_METAL)
	return SEACAVE::METAL::isRuntimeAvailable();
	#else
	return false;
	#endif
}
// the name of the GPU backend
#ifdef _USE_CUDA
#define SYNTH_GPU_BACKEND _T("CUDA")
#else
#define SYNTH_GPU_BACKEND _T("Metal")
#endif

// PatchMatch on numPanos panoramas of the synthetic scene scored into sphereEval and, if faceEval, separately
// on their cube faces scored into faceEval; logs the tables and the wall time per GT pixel
static bool SynthSphericalRun(int panoWidth, unsigned numPanos, const SynthDenseOptions& options, DepthEvaluator& sphereEval, DepthEvaluator* faceEval)
{
	LPCTSTR const backend(options.gpu ? SYNTH_GPU_BACKEND : _T("CPU"));
	SynthViewArr views;
	SynthGenerate(SynthMiddleConfig(panoWidth, numPanos, faceEval != NULL), views);
	IIndexArr panoIDs, faceIDs;
	SynthSplitViews(views, panoIDs, faceIDs);
	const auto Run = [&](const IIndexArr& viewIDs, DepthEvaluator& evaluator, const String& what) {
		double seconds;
		if (!SynthEstimateDepth(views, viewIDs, options, evaluator, seconds))
			return false;
		const SynthCamera& camera = views[viewIDs[0]].camera;
		VERBOSE("synthetic spherical: %s PatchMatch on %s of %dx%d px: %.2f s, %.3f us/px\n%s",
			backend, what.c_str(), camera.width, camera.height,
			seconds, seconds*1e6/evaluator.GetStats(DepthEvaluator::ALL).numGT, evaluator.Print().c_str());
		return true;
	};
	return Run(panoIDs, sphereEval, String::FormatString("%u panoramas", panoIDs.size())) &&
		(faceEval == NULL || Run(faceIDs, *faceEval, String::FormatString("their %u cube faces", faceIDs.size())));
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

// how SynthSphericalFusionRun treats the cube faces
enum SynthFaces { FACES_NONE, FACES_APART, FACES_WITH_PANORAMAS };

// PatchMatch and fusion of numPanos panoramas of the synthetic scene scored into sphereEval and, unless
// FACES_NONE, of their cube faces (FACES_APART) or of the panoramas and their faces together (a mixed
// scene, FACES_WITH_PANORAMAS) scored into faceEval; the GT is the panoramas ray-cast at 4x their width
// (at most 4096); logs the tables
static bool SynthSphericalFusionRun(int panoWidth, unsigned numPanos, SynthFaces faces, const SynthDenseOptions& options,
	CloudEvaluator& sphereEval, CloudEvaluator* faceEval)
{
	ASSERT((faces == FACES_NONE) == (faceEval == NULL));
	SynthViewArr views;
	SynthGenerate(SynthMiddleConfig(panoWidth, numPanos, faces != FACES_NONE), views);
	IIndexArr panoIDs, faceIDs;
	SynthSplitViews(views, panoIDs, faceIDs);
	const int gtWidth(MINF(4*panoWidth, 4096));
	for (IIndex i: panoIDs) {
		const DepthMap ranges(SynthPanoRanges(views[i], gtWidth));
		sphereEval.AddView(views[i].camera.R, views[i].camera.C, ranges, gtWidth/panoWidth);
		if (faceEval)
			faceEval->AddView(views[i].camera.R, views[i].camera.C, ranges, gtWidth/panoWidth);
	}
	PointCloud::PointArr points;
	double seconds;
	if (!SynthFuseDepth(views, panoIDs, options, points, seconds))
		return false;
	sphereEval.Evaluate(points);
	VERBOSE("synthetic spherical fusion: %u panoramas of %dx%d px: %u points, %.2f s\n%s",
		panoIDs.size(), panoWidth, panoWidth/2, points.size(), seconds, sphereEval.Print().c_str());
	if (faces == FACES_NONE)
		return true;
	IIndexArr viewIDs(faceIDs);
	if (faces == FACES_WITH_PANORAMAS)
		viewIDs.Join(panoIDs);
	if (!SynthFuseDepth(views, viewIDs, options, points, seconds))
		return false;
	faceEval->Evaluate(points);
	VERBOSE("synthetic spherical fusion: their %u cube faces of %dx%d px%s: %u points, %.2f s\n%s",
		faceIDs.size(), views[faceIDs[0]].camera.width, views[faceIDs[0]].camera.height, faces == FACES_WITH_PANORAMAS ? " with the panoramas" : "",
		points.size(), seconds, faceEval->Print().c_str());
	return true;
}

bool MVS::SyntheticDepthBaselineTest()
{
	TD_TIMER_START();
	SynthConfig config(SynthMiddleConfig(512, 3, true));
	config.numBrown = 2;
	SynthViewArr views;
	SynthGenerate(config, views);
	// GT consistency across the models: panorama -> panorama, panorama <-> face, Brown <-> panorama/face
	IIndex panos[3], faces[3][6], browns[2] = {NO_ID, NO_ID};
	FOREACH(i, views) {
		const SynthView& view = views[i];
		if (view.camera.model == CameraModelType::BROWN)
			browns[view.pano - 3] = i;
		else if (view.face < 0)
			panos[view.pano - 3] = i;
		else
			faces[view.pano - 3][view.face] = i;
	}
	ASSERT(browns[0] != NO_ID && browns[1] != NO_ID);
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
	if (!SynthEstimateDepth(views, faceIDs, SynthDenseOptions(), evaluator, seconds))
		return false;
	VERBOSE("SyntheticDepthBaselineTest: CPU PatchMatch on %u cube faces of %dx%d px:\n%s",
		faceIDs.size(), views[faceIDs[0]].camera.width, views[faceIDs[0]].camera.height, evaluator.Print().c_str());
	// bounds: the measured accuracy less a margin
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
	DepthEvaluator evaluator;
	if (!SynthSphericalRun(512, 3, SynthDenseOptions(), evaluator, NULL))
		return false;
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL)), equator(evaluator.GetStats(DepthEvaluator::EQUATOR));
	const DepthEvaluator::Stats seam(evaluator.GetStats(DepthEvaluator::SEAM)), center(evaluator.GetStats(DepthEvaluator::CENTER));
	const DepthEvaluator::Stats north(evaluator.GetStats(DepthEvaluator::NORTH)), south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// bounds: the measured accuracy less a margin. The panoramas look along their trajectory, so the
	// epipoles of their neighbors lie next to the seam and next to longitude 0, where the view gate acts:
	// the seam band is held to its antipodal band (CENTER), not to the equator. The pole bands miss the
	// rows next to each pole.
	if (!(all.completeness > 0.93) || !(all.absRelMedian < 0.0025) || !(all.within1 > 0.88) || !(all.within5 > 0.97) || !(all.normalMedianDeg < 5.5) ||
		!(seam.completeness > 0.99*center.completeness) || !(seam.within5 > center.within5 - 0.01) ||
		!(north.completeness > 0.8) || !(north.within5 > 0.95) || !(south.completeness > 0.75) || !(south.within5 > 0.95)) {
		VERBOSE("ERROR: SyntheticSphericalTest: depth accuracy below the baseline!");
		return false;
	}
	VERBOSE("SyntheticSphericalTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}

bool MVS::SyntheticSphericalGPUTest(bool forceCPU)
{
	if (forceCPU || !SynthGPUAvailable()) {
		VERBOSE("SyntheticSphericalGPUTest skipped: no GPU");
		return true;
	}
	TD_TIMER_START();
	// the scene 1e6 m away from its origin along every axis, where single precision is 6 cm apart: the
	// estimator only relates the views to each other, so it is as accurate as at the origin
	SynthDenseOptions options;
	options.gpu = true;
	options.translation = Point3(1e6, 1e6, 1e6);
	DepthEvaluator evaluator;
	if (!SynthSphericalRun(512, 3, options, evaluator, NULL))
		return false;
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL));
	const DepthEvaluator::Stats seam(evaluator.GetStats(DepthEvaluator::SEAM)), center(evaluator.GetStats(DepthEvaluator::CENTER));
	const DepthEvaluator::Stats north(evaluator.GetStats(DepthEvaluator::NORTH)), south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// bounds: CUDA, the measured accuracy less a margin (every band at or above the CPU estimator); Metal,
	// the CPU bounds of SyntheticSphericalTest; the seam band is held to its antipodal band as there
	struct Bounds { double completeness, absRel, within1, within5, normalDeg, poleCompleteness[2], poleWithin5; };
	#ifdef _USE_CUDA
	const Bounds bounds{0.94, 0.002, 0.90, 0.975, 4.5, {0.8, 0.8}, 0.97};
	#else
	const Bounds bounds{0.93, 0.0025, 0.88, 0.97, 5.5, {0.8, 0.75}, 0.95};
	#endif
	if (!(all.completeness > bounds.completeness) || !(all.absRelMedian < bounds.absRel) || !(all.within1 > bounds.within1) ||
		!(all.within5 > bounds.within5) || !(all.normalMedianDeg < bounds.normalDeg) ||
		!(seam.completeness > 0.99*center.completeness) || !(seam.within5 > center.within5 - 0.01) ||
		!(north.completeness > bounds.poleCompleteness[0]) || !(north.within5 > bounds.poleWithin5) ||
		!(south.completeness > bounds.poleCompleteness[1]) || !(south.within5 > bounds.poleWithin5)) {
		VERBOSE("ERROR: SyntheticSphericalGPUTest: %s depth accuracy below the CPU baseline!", SYNTH_GPU_BACKEND);
		return false;
	}
	VERBOSE("SyntheticSphericalGPUTest passed on %s (%s)", SYNTH_GPU_BACKEND, TD_TIMER_GET_FMT().c_str());
	return true;
}

// PatchMatch on the cube faces of the middle of three panorama rigs with the panoramas of the two other rigs:
// the faces (pinhole references whose every neighbor is spherical: the MODEL_PINHOLE_REF kernels on the GPU)
// scored into evaluator; the panoramas are references too, not scored; logs the table
static bool SynthMixedRun(int panoWidth, const SynthDenseOptions& options, DepthEvaluator& evaluator)
{
	LPCTSTR const backend(options.gpu ? SYNTH_GPU_BACKEND : _T("CPU"));
	const SynthConfig config(SynthMiddleConfig(panoWidth, 3, true));
	const int middlePano(config.firstPano + 1);
	SynthViewArr views;
	SynthGenerate(config, views);
	// the faces of the middle rig, then the panoramas of the two others
	IIndexArr viewIDs;
	FOREACH(i, views)
		if (views[i].face >= 0 && views[i].pano == middlePano)
			viewIDs.push_back(i);
	const IIndex numFaces(viewIDs.size());
	FOREACH(i, views)
		if (views[i].face < 0 && views[i].pano != middlePano)
			viewIDs.push_back(i);
	ASSERT(numFaces == 6 && viewIDs.size() == numFaces + 2);
	double seconds;
	if (!SynthDense(views, viewIDs, options, 1, seconds, [&](const Scene& scene) {
		for (IIndex i = 0; i < numFaces; ++i) {
			DMapFile dmap;
			if (!dmap.Load(ComposeDepthFilePath(scene.images[i].ID, "dmap"), true)) {
				VERBOSE("ERROR: synthetic mixed scene: missing depth-map of face %u!", i);
				return false;
			}
			ASSERT(dmap.IDs.front() == scene.images[i].ID);
			if (dmap.IDs.size() < 2) {
				VERBOSE("ERROR: synthetic mixed scene: face %u has no neighbor!", i);
				return false;
			}
			for (IIndex j = 1; j < dmap.IDs.size(); ++j) {
				if (!scene.images[dmap.IDs[j]].IsSpherical()) {
					VERBOSE("ERROR: synthetic mixed scene: face %u has the pinhole neighbor %u!", i, dmap.IDs[j]);
					return false;
				}
			}
			const SynthView& view = views[viewIDs[i]];
			evaluator.Add(view.depthMap, view.normalMap, dmap.depthMap, dmap.normalMap, view.camera.model);
		}
		return true;
	}))
		return false;
	const SynthCamera& camera = views[viewIDs[0]].camera;
	VERBOSE("synthetic mixed: %s PatchMatch on %u cube faces of %dx%d px, their neighbors the %u panoramas of the other rigs: %.2f s\n%s",
		backend, numFaces, camera.width, camera.height, viewIDs.size() - numFaces, seconds, evaluator.Print().c_str());
	return true;
}

bool MVS::SyntheticMixedGPUTest(bool forceCPU)
{
	if (forceCPU || !SynthGPUAvailable()) {
		VERBOSE("SyntheticMixedGPUTest skipped: no GPU");
		return true;
	}
	TD_TIMER_START();
	SynthDenseOptions options;
	options.gpu = true;
	DepthEvaluator evaluator;
	if (!SynthMixedRun(384, options, evaluator))
		return false;
	const DepthEvaluator::Stats all(evaluator.GetStats(DepthEvaluator::ALL));
	const DepthEvaluator::Stats center(evaluator.GetStats(DepthEvaluator::CENTER)), border(evaluator.GetStats(DepthEvaluator::BORDER));
	// bounds: the CPU accuracy of SynthMixedRun on the same views (`Tests mixtest 1`) less a margin
	if (!(all.completeness > 0.80) || !(all.absRelMedian < 0.003) || !(all.within1 > 0.80) || !(all.within5 > 0.96) ||
		!(all.normalMedianDeg < 3.5) || !(center.completeness > 0.97) || !(border.completeness > 0.65)) {
		VERBOSE("ERROR: SyntheticMixedGPUTest: %s depth accuracy below the CPU baseline!", SYNTH_GPU_BACKEND);
		return false;
	}
	VERBOSE("SyntheticMixedGPUTest passed on %s (%s)", SYNTH_GPU_BACKEND, TD_TIMER_GET_FMT().c_str());
	return true;
}

// the confidence recalibration on the GPU against the CPU sweep, on the same inputs: GT depths with
// noise and outliers, random photometric confidences and the GT normals (the inputs of the last
// geometric-consistency iteration: DepthMapsData::AdjustConfidence(DepthData&) and AdjustConfidenceGPU)
// of the synthetic scene, for every pair of camera models: a cube face of the middle panorama against
// the faces and the panoramas of the two others, then that panorama against the same views
bool MVS::GPUConfidenceTest(bool forceCPU)
{
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	if (forceCPU || !SynthGPUAvailable()) {
		VERBOSE("GPUConfidenceTest skipped: no GPU");
		return true;
	}
	TD_TIMER_START();
	SynthConfig config;
	config.panoWidth = 1024;
	config.numRenderPanos = 3;
	config.numBrown = 0;
	config.supersample = 1;
	SynthViewArr views;
	SynthGenerate(config, views);
	// the neighbors: every face and panorama of the other panoramas
	IIndex idxFace(NO_ID), idxPano(NO_ID);
	IIndexArr neighborIDs;
	FOREACH(i, views) {
		if (views[i].pano != 1)
			neighborIDs.push_back(i);
		else if (views[i].face == 0)
			idxFace = i;
		else if (views[i].face < 0)
			idxPano = i;
	}
	ASSERT(idxFace != NO_ID && idxPano != NO_ID && neighborIDs.size() == 14);
	OPTDENSE::init();
	OPTDENSE::update();
	#ifdef _USE_CUDA
	const auto allocatePool([](DepthMapsData& depthMaps) { return depthMaps.AllocateCudaPool(1); });
	#else
	const auto allocatePool([](DepthMapsData& depthMaps) { return depthMaps.AllocateMetalPool(1); });
	#endif
	// the noisy estimates of a view: depth within 0.2% of the GT, 5% outliers within 20%, the
	// confidence uniform in [0.2,1]
	std::mt19937 rnd(7);
	std::normal_distribution<float> noise(0.f, 0.002f);
	std::uniform_real_distribution<float> uniform(0.f, 1.f);
	struct Estimates { DepthMap depthMap; NormalMap normalMap; ConfidenceMap confMap; BitMatrix outliers; };
	const auto makeEstimates([&](const SynthView& view, Estimates& e) {
		e.depthMap.create(view.depthMap.size());
		e.confMap.create(view.depthMap.size());
		e.normalMap = view.normalMap.clone();
		e.outliers.create(view.depthMap.size());
		e.outliers.memset(0);
		for (int r = 0; r < e.depthMap.rows; ++r) {
			for (int c = 0; c < e.depthMap.cols; ++c) {
				const bool bOutlier(uniform(rnd) < 0.05f);
				e.depthMap(r,c) = view.depthMap(r,c) * (bOutlier ? 0.8f + 0.4f*uniform(rnd) : 1.f + noise(rnd));
				e.confMap(r,c) = 0.2f + 0.8f*uniform(rnd);
				if (bOutlier)
					e.outliers.set(r, c);
			}
		}
	});
	for (const IIndex idxRef: {idxFace, idxPano}) {
		IIndexArr viewIDs(neighborIDs);
		viewIDs.InsertAt(0, idxRef);
		const ScopedTempDir tmpDir(_T("GPUConfidence"));
		if (!tmpDir.IsValid())
			return false;
		Scene scene;
		if (!SynthBuildScene(views, viewIDs, tmpDir.Path() + "/", scene))
			return false;
		std::vector<Estimates> estimates(viewIDs.size());
		FOREACH(i, viewIDs)
			makeEstimates(views[viewIDs[i]], estimates[i]);
		// the reference's DepthData, the neighbors' maps as InitViews loads them on the last iteration
		const auto makeDepthData([&]() {
			DepthData depthData;
			depthData.images.resize(viewIDs.size());
			FOREACH(i, viewIDs) {
				DepthData::ViewData& view = depthData.images[i];
				view.pImageData = &scene.images[i];
				view.camera = view.cameraDepthMap = scene.images[i].camera;
				view.scale = 1;
				if (i > 0) {
					view.depthMap = estimates[i].depthMap.clone();
					view.normalMap = estimates[i].normalMap.clone();
					view.confMap = estimates[i].confMap.clone();
				}
			}
			depthData.depthMap = estimates[0].depthMap.clone();
			depthData.normalMap = estimates[0].normalMap.clone();
			depthData.confMap = estimates[0].confMap.clone();
			return depthData;
		});
		const char* const refName(views[idxRef].face < 0 ? "panorama" : "face");
		DepthMapsData depthMaps(scene);
		if (!allocatePool(depthMaps) || !depthMaps.HasGPUConfidence()) {
			VERBOSE("ERROR: GPUConfidenceTest: the %s PatchMatch can not be created!", SYNTH_GPU_BACKEND);
			return false;
		}
		DepthData depthDataCPU(makeDepthData()), depthDataGPU(makeDepthData());
		if (!depthMaps.AdjustConfidence(depthDataCPU) || !depthMaps.AdjustConfidenceGPU(depthDataGPU)) {
			VERBOSE("ERROR: GPUConfidenceTest: the confidence recalibration of the %s did not run!", refName);
			return false;
		}
		// per pixel: the GPU differs from the CPU in rounding only (single-precision exp, fused
		// multiply-adds, the trigonometry of a spherical reference), but for the rare pixel a rounding
		// moves across a gate threshold
		const ConfidenceMap& confCPU = depthDataCPU.confMap;
		const ConfidenceMap& confGPU = depthDataGPU.confMap;
		const Estimates& ref = estimates[0];
		size_t numPixels(0), numDiffering(0), numInliers(0), numOutliers(0);
		double maxDiff(0), sumDiff(0), sumInliers(0), sumOutliers(0), sumInliersCPU(0), sumOutliersCPU(0);
		for (int r = 0; r < confCPU.rows; ++r) {
			for (int c = 0; c < confCPU.cols; ++c) {
				const double diff(ABS((double)confCPU(r,c) - (double)confGPU(r,c)));
				maxDiff = MAXF(maxDiff, diff);
				sumDiff += diff;
				numDiffering += diff > 1e-4;
				++numPixels;
				if (ref.outliers.isSet(r, c)) {
					sumOutliers += confGPU(r,c); sumOutliersCPU += confCPU(r,c); ++numOutliers;
				} else {
					sumInliers += confGPU(r,c); sumInliersCPU += confCPU(r,c); ++numInliers;
				}
			}
		}
		const double meanInliers(sumInliers/numInliers), meanOutliers(sumOutliers/numOutliers);
		const double meanInliersCPU(sumInliersCPU/numInliers), meanOutliersCPU(sumOutliersCPU/numOutliers);
		VERBOSE("GPUConfidenceTest: %s reference, %s against CPU over %zu px: mean |diff| %.2e, max %.2e, %zu px (%.3f%%) above 1e-4; "
			"mean confidence of the inliers %.3f (CPU %.3f), of the outliers %.3f (CPU %.3f)",
			refName, SYNTH_GPU_BACKEND, numPixels, sumDiff/numPixels, maxDiff, numDiffering, 100.0*numDiffering/numPixels,
			meanInliers, meanInliersCPU, meanOutliers, meanOutliersCPU);
		if (!(sumDiff/numPixels < 1e-5) || !(numDiffering < numPixels/1000) ||
			!(meanOutliers < 0.5*meanInliers) || !(meanOutliersCPU < 0.5*meanInliersCPU)) {
			VERBOSE("ERROR: GPUConfidenceTest: the %s confidence of the %s differs from the CPU one, or does not separate the outliers!", SYNTH_GPU_BACKEND, refName);
			return false;
		}
	}
	VERBOSE("GPUConfidenceTest passed on %s (%s)", SYNTH_GPU_BACKEND, TD_TIMER_GET_FMT().c_str());
	return true;
	#else
	VERBOSE("GPUConfidenceTest skipped: no GPU backend");
	return true;
	#endif
}

#ifdef _USE_METAL
bool MVS::MetalModelMathTest(bool forceCPU)
{
	if (forceCPU || !SEACAVE::METAL::isRuntimeAvailable()) {
		VERBOSE("MetalModelMathTest skipped: no Metal device");
		return true;
	}
	const METAL::PatchMatch pm;
	if (!pm.IsValid()) {
		VERBOSE("ERROR: MetalModelMathTest: the Metal PatchMatch kernels do not build!");
		return false;
	}
	typedef Eigen::Vector2f Vec2;
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Matrix3f Mat3;
	const int width(4096), height(2048);
	const Spherical<double> sphD(Spherical<double>::FromSize(width, height));
	const Spherical<float> sph(Spherical<float>::FromSize(width, height));
	const Pinhole<float> pin{1500.5f, 1510.25f, 959.5f, 539.75f};
	std::mt19937 rng(11);
	std::uniform_real_distribution<float> U(0, 1);
	std::normal_distribution<float> N(0, 1);
	const auto R = [&](float a, float b) { return a + (b - a)*U(rng); };
	const auto RandomUnit = [&]() { return Vec3(N(rng), N(rng), N(rng)).normalized(); };
	const auto Copy = [](float* dst, const float* src, int n) { std::copy(src, src+n, dst); };
	// random points, pixels and planes; the rows of the ray frames on the equirectangular pixel centers
	// with the trig tables of the kernels (sin and cos in double of the pixel's angles)
	const int numSamples(1 << 14);
	std::vector<METAL::ModelMathSample> samples(numSamples);
	std::vector<Eigen::Vector2i> pixels(numSamples);
	for (int i = 0; i < numSamples; ++i) {
		METAL::ModelMathSample& s = samples[i];
		std::fill((float*)&s, (float*)(&s+1), 0.f);
		const float sphParams[4] = {sph.fx, sph.fy, sph.cx, sph.cy}, pinParams[4] = {pin.fx, pin.fy, pin.cx, pin.cy};
		Copy(s.spherical, sphParams, 4);
		Copy(s.pinhole, pinParams, 4);
		const Vec3 X(RandomUnit()*R(0.1f, 50.f));
		Copy(s.X, X.data(), 3);
		pixels[i] = Eigen::Vector2i((int)R(0, width-1), (int)R(0, height-1));
		const Eigen::Vector2d a(sphD.Angles(pixels[i].cast<double>()));
		const float trig[4] = {(float)std::sin(a.x()), (float)std::cos(a.x()), (float)std::sin(a.y()), (float)std::cos(a.y())};
		Copy(s.trig, trig, 4);
		s.pixel[0] = R(-0.5f, 1919.5f); s.pixel[1] = R(-0.5f, 1079.5f);
		const Vec3 ray(RandomUnit()), nRay(RandomUnit());
		Vec3 normal;
		do normal = RandomUnit(); while (std::abs(normal.dot(ray)) < 0.05f);
		Copy(s.ray, ray.data(), 3);
		Copy(s.nRay, nRay.data(), 3);
		Copy(s.normal, normal.data(), 3);
		s.nDepth = R(1, 10); s.dMin = 0.01f; s.dMax = 1000.f;
		s.depth = R(1, 10);
		s.gradient[0] = R(-1, 1)*s.depth/sph.fx; s.gradient[1] = R(-1, 1)*s.depth/sph.fy;
	}
	if (!pm.EvaluateModelMath(samples)) {
		VERBOSE("ERROR: MetalModelMathTest: the test kernel did not run!");
		return false;
	}
	// the largest error of each output against the C++ models in single precision
	const auto MaxRel = [](const float* dev, const Mat3& M) {
		const float scale(std::max(M.cwiseAbs().maxCoeff(), 1e-30f));
		float err(0);
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				err = std::max(err, std::abs(dev[r*3+c] - M(r,c)) / scale);
		return err;
	};
	float errAtan(0), errProject(0), errRange(0), errRay(0), errFrame(0), errHomography(0), errPinFrame(0), errPinProject(0), errInterp(0), errNormal(0);
	unsigned numVisibility(0);
	for (int i = 0; i < numSamples; ++i) {
		const METAL::ModelMathSample& s = samples[i];
		const Vec3 X(s.X[0], s.X[1], s.X[2]);
		errAtan = std::max(errAtan, std::abs(s.fastAtan2 - FastAtan2(X.y(), X.x())));
		const float rho(std::sqrt(X.x()*X.x() + X.z()*X.z()));
		Vec2 x;
		numVisibility += sph.ProjectFast(X, x, rho) != (s.visible != 0.f);
		errProject = std::max(errProject, std::max(std::abs(s.projected[0] - x.x()), std::abs(s.projected[1] - x.y())));
		errRange = std::max(errRange, std::abs(s.range - Spherical<float>::Depth(X)) / X.norm());
		const Vec3 r(Spherical<float>::Ray(s.trig[0], s.trig[1], s.trig[2], s.trig[3]));
		errRay = std::max(errRay, (Vec3(s.sphRay[0], s.sphRay[1], s.sphRay[2]) - r).cwiseAbs().maxCoeff());
		errFrame = std::max(errFrame, MaxRel(s.sphFrame, sph.RayFrame(s.trig[0], s.trig[1], s.trig[2], s.trig[3])));
		errHomography = std::max(errHomography, MaxRel(s.homography, sph.PixelHomography(X, x, rho)));
		errPinFrame = std::max(errPinFrame, MaxRel(s.pinFrame, pin.RayFrame(Vec2(s.pixel[0], s.pixel[1]))));
		Vec2 xp;
		numVisibility += pin.Project(X, xp) != (s.pinVisible != 0.f);
		// relative to the magnitude of the operands of fx*n+cx (one rounding of an FMA the Metal compiler fuses
		// and the host may not is an ulp of max(|x|, c), not of |x| when the two terms cancel)
		errPinProject = std::max(errPinProject, std::max(std::abs(s.pinProjected[0] - xp.x()) / std::max({1.f, std::abs(xp.x()), pin.cx}), std::abs(s.pinProjected[1] - xp.y()) / std::max({1.f, std::abs(xp.y()), pin.cy})));
		const float interpolated(InterpolatePlaneDepth(Vec3(s.ray[0], s.ray[1], s.ray[2]), Vec3(s.nRay[0], s.nRay[1], s.nRay[2]),
			s.nDepth, Vec3(s.normal[0], s.normal[1], s.normal[2]), s.dMin, s.dMax));
		errInterp = std::max(errInterp, std::abs(s.interpolated - interpolated) / interpolated);
		const Vec3 n(NormalFromDepthGradient(sph, pixels[i], s.depth, Vec2(s.gradient[0], s.gradient[1])));
		errNormal = std::max(errNormal, (Vec3(s.normalOut[0], s.normalOut[1], s.normalOut[2]) - n).cwiseAbs().maxCoeff());
	}
	VERBOSE("MetalModelMathTest: %d samples, max error: FastAtan2 %.2e rad, Spherical ProjectFast %.2e px, Depth %.2e, Ray %.2e, "
		"RayFrame %.2e, PixelHomography %.2e, Pinhole RayFrame %.2e, Project %.2e, InterpolatePlaneDepth %.2e, NormalFromDepthGradient %.2e; "
		"%u visibility mismatches", numSamples, errAtan, errProject, errRange, errRay, errFrame, errHomography, errPinFrame, errPinProject,
		errInterp, errNormal, numVisibility);
	// single precision with the fast math of the Metal compiler: a few ulp; the normal also differs by
	// the host computing the ray frame from the float angles, the kernels from the trig tables
	if (!(errAtan < 2e-6f) || !(errProject < 2e-3f) || !(errRange < 1e-5f) || !(errRay < 1e-6f) || !(errFrame < 1e-5f) ||
		!(errHomography < 1e-5f) || !(errPinFrame < 1e-5f) || !(errPinProject < 1e-5f) || !(errInterp < 1e-4f) || !(errNormal < 1e-4f) ||
		numVisibility) {
		VERBOSE("ERROR: MetalModelMathTest: the Metal camera models differ from CameraModels.h!");
		return false;
	}
	VERBOSE("MetalModelMathTest passed");
	return true;
}
#endif // _USE_METAL
/*----------------------------------------------------------------*/

bool MVS::SyntheticSphericalFusionTest()
{
	TD_TIMER_START();
	const float thresholds[CloudEvaluator::NUM_TH] = {0.005f, 0.01f, 0.02f};
	CloudEvaluator evaluator(512, thresholds);
	if (!SynthSphericalFusionRun(512, 3, FACES_NONE, SynthDenseOptions(), evaluator, NULL))
		return false;
	const CloudEvaluator::Stats& all(evaluator.GetStats(DepthEvaluator::ALL)), &equator(evaluator.GetStats(DepthEvaluator::EQUATOR));
	const CloudEvaluator::Stats& seam(evaluator.GetStats(DepthEvaluator::SEAM));
	const CloudEvaluator::Stats& north(evaluator.GetStats(DepthEvaluator::NORTH)), &south(evaluator.GetStats(DepthEvaluator::SOUTH));
	// bounds: the measured precision@1% and recall@2% less a margin; the seam band against the equator
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


bool MVS::NeighborViewsPointCloudRigTest()
{
	TD_TIMER_START();
	SynthViewArr views;
	SynthGenerate(SynthMiddleConfig(256, 2, true), views);
	IIndexArr viewIDs;
	FOREACH(i, views)
		viewIDs.push_back(i);
	const ScopedTempDir tmpDir(_T("NeighborViewsPointCloudRig"));
	if (!tmpDir.IsValid())
		return false;
	const ScopedWorkingFolder workingFolder(tmpDir.Path() + "/");
	Scene scene;
	if (!SynthBuildScene(views, viewIDs, tmpDir.Path() + "/", scene))
		return false;
	scene.pointcloud.Release();
	if (!scene.EstimateNeighborViewsPointCloud() || scene.pointcloud.IsEmpty()) {
		VERBOSE("ERROR: NeighborViewsPointCloudRigTest: no points between the rigs!");
		return false;
	}
	FOREACH(p, scene.pointcloud.points) {
		const PointCloud::Point& X = scene.pointcloud.points[p];
		const PointCloud::ViewArr& pointViews = scene.pointcloud.pointViews[p];
		if (!ISFINITE(X.x) || !ISFINITE(X.y) || !ISFINITE(X.z) || pointViews.size() != 2 ||
			scene.images[pointViews[0]].platformID == scene.images[pointViews[1]].platformID) {
			VERBOSE("ERROR: NeighborViewsPointCloudRigTest: point %u (%g, %g, %g) is not seen by views of two rigs!", p, X.x, X.y, X.z);
			return false;
		}
	}
	VERBOSE("NeighborViewsPointCloudRigTest passed: %u points (%s)", scene.pointcloud.points.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool MVS::EstimatePointCloudNormalsSphericalTest()
{
	TD_TIMER_START();
	constexpr int seamBand(4); // columns on each side of the seam
	constexpr int patchRadius(3); // the target patch of EstimatePointCloudNormals
	constexpr float sigmaAngleInv(-1.f / (2.f * SQUARE(D2R(15.f)))); // its view score
	SynthViewArr views;
	SynthGenerate(SynthMiddleConfig(256, 3, false), views);
	IIndexArr viewIDs;
	FOREACH(i, views)
		viewIDs.push_back(i);
	const ScopedTempDir tmpDir(_T("EstimatePointCloudNormalsSpherical"));
	if (!tmpDir.IsValid())
		return false;
	const ScopedWorkingFolder workingFolder(tmpDir.Path() + "/");
	Scene scene;
	if (!SynthBuildScene(views, viewIDs, tmpDir.Path() + "/", scene))
		return false;
	// the images prepared at half resolution, as a pipeline stage leaves them, the first one with its pixels
	FOREACH(i, scene.images) {
		Image& image = scene.images[i];
		if (!image.ReloadImage(image.width/2, i == 0))
			return false;
		image.UpdateCamera(scene.platforms);
	}
	const ImageArr prepared(scene.images);
	// the coarse normals (the mean viewing direction), then the refined ones
	if (!scene.EstimatePointCloudNormals(false))
		return false;
	const PointCloud::NormalArr coarse(scene.pointcloud.normals);
	scene.pointcloud.normals.Release();
	if (!scene.EstimatePointCloudNormals(true))
		return false;
	FOREACH(i, scene.images) {
		const Image& image = scene.images[i];
		const Image& before = prepared[i];
		if (image.width != before.width || image.height != before.height || image.scale != before.scale ||
			image.camera.K != before.camera.K || image.image.empty() != before.image.empty() || image.image.data != before.image.data) {
			VERBOSE("ERROR: EstimatePointCloudNormalsSphericalTest: image %u changed from %ux%u (scale %g, fx %g, cx %g, %s) to %ux%u (scale %g, fx %g, cx %g, %s)!", i,
				before.width, before.height, before.scale, before.camera.K(0,0), before.camera.K(0,2), before.image.empty() ? "no pixels" : "pixels",
				image.width, image.height, image.scale, image.camera.K(0,0), image.camera.K(0,2), image.image.empty() ? "no pixels" : "pixels");
			return false;
		}
	}
	// the images at the resolution the normals are refined at
	ImageArr refineImages(scene.images);
	for (Image& image: refineImages) {
		image.ReleaseImage();
		if (!image.ReloadImage(1024, false))
			return false;
		image.UpdateCamera(scene.platforms);
	}
	// the target view of each point, chosen as EstimatePointCloudNormals does: every column of a panorama
	// takes a patch, the rows keep the patch radius from the poles; a point that one of its views sees within
	// patchRadius+1 rows of a pole is left out: its patch in that view crosses the pole, whose rows are not
	// wrapped, so the residual of that view is constant and lmmin stops before its first step (the camera
	// path runs through the seam: a point below one panorama is seen on the seam of the next one)
	unsigned numSeam(0), numSeamSkipped(0), numRest(0), numPole(0);
	double errSeam(0), errRest(0);
	FOREACH(p, scene.pointcloud.points) {
		const PointCloud::Point& X = scene.pointcloud.points[p];
		IIndex bestView(NO_ID);
		float bestScore(-1.f);
		Point2f bestProjection;
		bool bPole(false);
		for (const IIndex idxView: scene.pointcloud.pointViews[p]) {
			const Image& image = refineImages[idxView];
			const Point3f viewDir = normalized(image.camera.C - Cast<REAL>(X));
			const auto [projection, depth] = image.GetModelCamera().ProjectPointP(X);
			if (projection.y < patchRadius+1 || projection.y > image.height - (patchRadius+2))
				bPole = true;
			if (depth <= 0 || projection.y < patchRadius || projection.y > image.height - (patchRadius+1))
				continue;
			const float angle = ACOS(ComputeAngleN(coarse[p].ptr(), viewDir.ptr()));
			const float score = EXP(SQUARE(angle) * sigmaAngleInv) / image.camera.GetFootprintImage(depth);
			if (score > bestScore) {
				bestScore = score;
				bestView = idxView;
				bestProjection = projection;
			}
		}
		if (bPole) {
			++numPole;
			continue;
		}
		ASSERT(bestView != NO_ID);
		const Image& target = refineImages[bestView];
		const Point3f& normal = scene.pointcloud.normals[p];
		const EVec3 C(target.camera.C);
		const SynthHit hit(SynthTrace(C, EVec3(Cast<REAL>(X)) - C));
		const double err(ACOS(MINF(ABS(EVec3(Cast<REAL>(normal)).dot(hit.normal)), 1.0)));
		if (bestProjection.x < seamBand - 0.5f || bestProjection.x > target.width - seamBand - 0.5f) {
			if (normal == coarse[p])
				++numSeamSkipped;
			errSeam += err;
			++numSeam;
		} else {
			errRest += err;
			++numRest;
		}
	}
	if (numSeam == 0 || numRest == 0) {
		VERBOSE("ERROR: EstimatePointCloudNormalsSphericalTest: %u points on the seam and %u elsewhere!", numSeam, numRest);
		return false;
	}
	errSeam = R2D(errSeam / numSeam);
	errRest = R2D(errRest / numRest);
	if (numSeamSkipped > 0) {
		VERBOSE("ERROR: EstimatePointCloudNormalsSphericalTest: %u of the %u points whose target view projects on the seam are not refined!", numSeamSkipped, numSeam);
		return false;
	}
	if (errSeam > 1.5 * errRest) {
		VERBOSE("ERROR: EstimatePointCloudNormalsSphericalTest: mean normal error %.2f deg on the seam, %.2f deg elsewhere!", errSeam, errRest);
		return false;
	}
	VERBOSE("EstimatePointCloudNormalsSphericalTest passed: %u points on the seam (all refined), %u elsewhere, %u next to a pole left out: mean normal error %.2f deg on the seam, %.2f deg elsewhere (%s)",
		numSeam, numRest, numPole, errSeam, errRest, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

bool MVS::TowerSphericalTest()
{
	TD_TIMER_START();
	constexpr int width(512), height(256);
	constexpr unsigned numRings(3), numPerRing(8);
	constexpr double ringRadius(1.5), ringZ[numRings] = {0.5, 2.5, 4.5};
	const ScopedTempDir tmpDir(_T("TowerSpherical"));
	if (!tmpDir.IsValid())
		return false;
	const ScopedWorkingFolder workingFolder(tmpDir.Path() + "/");
	Scene scene(1);
	const Spherical<REAL> sphere(Spherical<REAL>::FromSize(width, height));
	KMatrix K(KMatrix::IDENTITY);
	K(0,0) = sphere.fx; K(1,1) = sphere.fy; K(0,2) = sphere.cx; K(1,2) = sphere.cy;
	Platform& platform = scene.platforms.emplace_back();
	platform.cameras.emplace_back(ScaleK(K, REAL(1)/Camera::GetNormalizationScale(width, height)), RMatrix::IDENTITY, CMatrix::ZERO).model = CameraModelType::SPHERICAL;
	// three stacked rings of 8 panoramas around the pillar, 24 headings 15 deg apart, each panorama
	// level (its rows down -z) and looking away from the pillar, which so lies on its seam
	for (unsigned r=0; r<numRings; ++r) {
		for (unsigned j=0; j<numPerRing; ++j) {
			const double angle(D2R(45.0*j + 15.0*r));
			const Point3 dir(cos(angle), sin(angle), 0);
			Platform::Pose& pose = platform.poses.emplace_back();
			pose.C = Point3(pillarAxis.x() + ringRadius*dir.x, pillarAxis.y() + ringRadius*dir.y, ringZ[r]);
			pose.R = RMatrix(cv::Matx33d(dir.y, -dir.x, 0, 0, 0, -1, dir.x, dir.y, 0));
			Image& image = scene.images.emplace_back();
			image.platformID = image.cameraID = 0;
			image.poseID = image.ID = scene.images.size()-1;
			image.width = width; image.height = height; image.scale = 1;
			image.UpdateCamera(scene.platforms);
		}
	}
	scene.nCalibratedImages = scene.images.size();
	// the sparse points on the pillar surface, seen by every panorama
	for (int z=0; z<=20; ++z) {
		for (int a=0; a<8; ++a) {
			const double angle(D2R(45.0*a));
			scene.pointcloud.points.emplace_back(float(pillarAxis.x() + pillarRadius*cos(angle)), float(pillarAxis.y() + pillarRadius*sin(angle)), float(0.25*z));
			PointCloud::ViewArr& views = scene.pointcloud.pointViews.emplace_back();
			FOREACH(v, scene.images)
				views.push_back(v);
		}
	}
	scene.InitTowerScene(-1);
	// the tower cloud: a horizontal unit normal per point, and at least two panoramas seeing the point
	// from the side its normal faces (within the angle of two pixels, DrawCircle reading the ray of the
	// rounded pixel)
	const PointCloud& pc = scene.pointcloud;
	if (pc.IsEmpty() || pc.normals.size() != pc.points.size()) {
		VERBOSE("ERROR: TowerSphericalTest: %u tower points with %u normals!", pc.points.size(), pc.normals.size());
		return false;
	}
	const float maxCos(2*FTWO_PI/width);
	FOREACH(p, pc.points) {
		const Point3f& X = pc.points[p];
		const Normal& n = pc.normals[p];
		const PointCloud::ViewArr& views = pc.pointViews[p];
		if (views.size() < 2 || !ISEQUAL(norm(n), 1.f) || !ISZERO(n.z)) {
			VERBOSE("ERROR: TowerSphericalTest: point %u (%g, %g, %g) has %u views, normal (%g, %g, %g)!", p, X.x, X.y, X.z, views.size(), n.x, n.y, n.z);
			return false;
		}
		for (const PointCloud::View v: views) {
			const Point3f ray(normalized(X - Cast<float>(scene.images[v].camera.C)));
			if (n.dot(ray) >= maxCos) {
				VERBOSE("ERROR: TowerSphericalTest: point %u (%g, %g, %g) is seen by view %u from behind its normal (cos %g)!", p, X.x, X.y, X.z, v, n.dot(ray));
				return false;
			}
		}
	}
	VERBOSE("TowerSphericalTest passed: %u tower points (%s)", pc.points.size(), TD_TIMER_GET_FMT().c_str());
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
		// bounds: the supersampled render and the bilinear remap differ by about one grey level; the lens
		// leaves a few % of K' without source pixels
		if (!(meanDiff < 1.5) || !(invalidRatio > 0.01 && invalidRatio < 0.1) || !(maxObsErr < 1e-3) || numPoints < 100) {
			VERBOSE("ERROR: BrownUndistortOnLoadTest: the undistorted view %u does not match the pinhole render at K'", v);
			return false;
		}
	}
	// the MeshLab project: the raster of a Brown image is its file undistorted onto the exported pinhole camera
	{
		const String mlpFileName(tmpDir("scene.mlp"));
		if (!scene.ExportCamerasMLP(mlpFileName, tmpDir("scene.ply")))
			return false;
		std::ifstream mlp(mlpFileName);
		const std::string text((std::istreambuf_iterator<char>(mlp)), std::istreambuf_iterator<char>());
		// an attribute of the first raster, the one of image 0
		const auto Attribute = [&text](const std::string& name) {
			const size_t pos(text.find(name + "=\""));
			ASSERT(pos != std::string::npos);
			const size_t begin(pos + name.size() + 2);
			return text.substr(begin, text.find('"', begin) - begin);
		};
		const Image& imageData = scene.images[0];
		double focal, cx, cy;
		std::istringstream(Attribute("FocalMm")) >> focal;
		std::istringstream(Attribute("CenterPx")) >> cx >> cy;
		Image8U3 raster, pixels;
		if (!raster.Load(MAKE_PATH_FULL(WORKING_FOLDER_FULL, String(Attribute("fileName")))) || imageData.ReadPixels(pixels) == NULL ||
			raster.size() != imageData.GetSize() || cv::norm(raster, pixels, cv::NORM_INF) != 0 ||
			!(std::abs(focal - imageData.camera.K(0,0)) < 1e-3) ||
			!(std::abs(cx - imageData.camera.K(0,2)) < 1e-3) || !(std::abs(cy - imageData.camera.K(1,2)) < 1e-3)) {
			VERBOSE("ERROR: BrownUndistortOnLoadTest: the MeshLab raster does not hold the pixels of its camera");
			return false;
		}
	}
	// the images stored at half the resolution (Scene::ScaleImages): the files keep their distorted pixels,
	// the cameras their lens at the new resolution, and the observations move with the undistorted cameras
	if (!scene.ScaleImages((unsigned)lens.width/2, 0, tmpDir("scaled/"))) {
		VERBOSE("ERROR: BrownUndistortOnLoadTest: the images can not be scaled");
		return false;
	}
	double maxScaledObsErr(0);
	FOREACH(j, points) {
		const PointCloud::ViewArr& pointViews = scene.pointcloud.pointViews[(uint32_t)j];
		const PointCloud::ObservationArr& observations = scene.pointcloud.pointObservations[(uint32_t)j];
		FOREACH(k, pointViews) {
			const Image& imageData = scene.images[pointViews[k]];
			ASSERT(imageData.IsUndistortedOnRead() && imageData.width == (uint32_t)lens.width/2 && imageData.lens.size == imageData.GetSize());
			maxScaledObsErr = MAXF(maxScaledObsErr, norm(imageData.camera.TransformPointW2I(points[j].first) - Cast<REAL>(observations[k])));
		}
	}
	Image8U3 scaledPixels;
	if (scene.images[0].ReadPixels(scaledPixels) == NULL || scaledPixels.size() != scene.images[0].GetSize()) {
		VERBOSE("ERROR: BrownUndistortOnLoadTest: the scaled images can not be read through their lens");
		return false;
	}
	if (!(maxScaledObsErr < 1e-3)) {
		VERBOSE("ERROR: BrownUndistortOnLoadTest: the observations of the scaled images are %.2e px off", maxScaledObsErr);
		return false;
	}
	VERBOSE("BrownUndistortOnLoadTest passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// T O O L S ///////////////////////////////////////////////////////

namespace {

// the optional argument i of a tool mode, def if absent
int ArgInt(int argc, LPCTSTR* argv, int i, int def) { return i < argc ? std::atoi(argv[i]) : def; }
// a path argument as a full path with the separators the Util path helpers split on
String ArgPath(LPCTSTR arg)
{
	String path(arg);
	Util::ensureValidPath(path);
	return Util::getFullPath(path);
}

// sets DepthEstimatorBase::fSphericalWarpMaxElevation from the optional argument i while it lives
struct ScopedMaxElevation {
	const float prev;
	ScopedMaxElevation(int argc, LPCTSTR* argv, int i) : prev(DepthEstimatorBase::fSphericalWarpMaxElevation) {
		if (i < argc)
			DepthEstimatorBase::fSphericalWarpMaxElevation = (float)std::atof(argv[i]);
	}
	~ScopedMaxElevation() { DepthEstimatorBase::fSphericalWarpMaxElevation = prev; }
};

bool ValidPanoramas(int panoWidth, unsigned numPanos)
{
	if (panoWidth > 0 && panoWidth % 8 == 0 && numPanos >= 2 && numPanos <= 5)
		return true;
	std::printf("error: panoWidth must be a positive multiple of 8, numPanos in [2,5]\n");
	return false;
}

// synth <outDir> [panoWidth]: write the default dataset (SynthSave) after printing its GT consistency
bool ToolSynth(int argc, LPCTSTR* argv)
{
	SynthConfig config;
	config.panoWidth = ArgInt(argc, argv, 2, config.panoWidth);
	if (config.panoWidth <= 0 || config.panoWidth % 8) {
		std::printf("error: panoWidth must be a positive multiple of 8\n");
		return false;
	}
	String folder(ArgPath(argv[1]));
	Util::ensureValidFolderPath(folder);
	TD_TIMER_START();
	SynthViewArr views;
	SynthGenerate(config, views);
	std::printf("rendered %u views (%s)\n", views.size(), TD_TIMER_GET_FMT().c_str());
	// each panorama against the next, its first face and the Brown view after it
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

// scorebench <verbosity> [panoWidth] [rounds] [maxElevation] [faceWidth]: ns per ScorePixel against one
// view, panorama 3 against panorama 4 and each cube face of 3 against the same face of 4 (faceWidth
// panoWidth/sqrt(2) compares at the same image size)
bool ToolScoreBench(int argc, LPCTSTR* argv)
{
	const int panoWidth(ArgInt(argc, argv, 2, 1536));
	const unsigned rounds((unsigned)ArgInt(argc, argv, 3, 4));
	const ScopedMaxElevation maxElevation(argc, argv, 4);
	const int faceWidth(ArgInt(argc, argv, 5, panoWidth/4));
	OPTDENSE::init();
	OPTDENSE::update();
	SynthConfig config(SynthMiddleConfig(panoWidth, 2, false));
	config.supersample = 1;
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

// sphtest <verbosity> [panoWidth] [numPanos] [faces] [maxElevation] [maxViews] [maxThreads] [gpu]: PatchMatch on
// panoramas and on their cube faces, the depth-maps scored per band
bool ToolSphTest(int argc, LPCTSTR* argv)
{
	const int panoWidth(ArgInt(argc, argv, 2, 512));
	const unsigned numPanos((unsigned)ArgInt(argc, argv, 3, 3));
	const bool faces(ArgInt(argc, argv, 4, 1) != 0);
	const ScopedMaxElevation maxElevation(argc, argv, 5);
	SynthDenseOptions options;
	options.maxViews = (unsigned)ArgInt(argc, argv, 6, 0);
	options.maxThreads = (unsigned)ArgInt(argc, argv, 7, 0);
	options.gpu = ArgInt(argc, argv, 8, 0) != 0;
	if (options.gpu && !SynthGPUAvailable()) {
		std::printf("error: no GPU\n");
		return false;
	}
	if (!ValidPanoramas(panoWidth, numPanos))
		return false;
	DepthEvaluator sphereEval, faceEval;
	if (!SynthSphericalRun(panoWidth, numPanos, options, sphereEval, faces ? &faceEval : NULL))
		return false;
	std::fputs(sphereEval.Print().c_str(), stdout);
	if (faces)
		std::fputs(faceEval.Print().c_str(), stdout);
	return true;
}

// mixtest <verbosity> [gpu]: PatchMatch on the cube faces of a panorama rig against the panoramas of two other rigs
// (the run of SyntheticMixedGPUTest), the depth-maps of the faces scored; the table goes to the log
bool ToolMixTest(int argc, LPCTSTR* argv)
{
	SynthDenseOptions options;
	options.gpu = ArgInt(argc, argv, 2, 0) != 0;
	if (options.gpu && !SynthGPUAvailable()) {
		std::printf("error: no GPU\n");
		return false;
	}
	DepthEvaluator evaluator;
	return SynthMixedRun(384, options, evaluator);
}

// conftest <verbosity>: GPUConfidenceTest alone
bool ToolConfTest(int, LPCTSTR*)
{
	return GPUConfidenceTest(false);
}

// sphfuse <verbosity> [panoWidth] [numPanos] [faces] [fusionFilter] [maxThreads]: PatchMatch and fusion of
// panoramas and of their cube faces (SynthFaces), the clouds scored per band
bool ToolSphFuse(int argc, LPCTSTR* argv)
{
	const int panoWidth(ArgInt(argc, argv, 2, 512));
	const unsigned numPanos((unsigned)ArgInt(argc, argv, 3, 3));
	const int faces(ArgInt(argc, argv, 4, FACES_APART));
	SynthDenseOptions options;
	options.fusionFilter = (unsigned)ArgInt(argc, argv, 5, OPTDENSE::FUSE_DENSEFILTER);
	options.maxThreads = (unsigned)ArgInt(argc, argv, 6, 0);
	if (!ValidPanoramas(panoWidth, numPanos))
		return false;
	if (faces < FACES_NONE || faces > FACES_WITH_PANORAMAS || options.fusionFilter > OPTDENSE::FUSE_DENSEFILTER) {
		std::printf("error: faces and fusionFilter must be in [0,2]\n");
		return false;
	}
	const float thresholds[CloudEvaluator::NUM_TH] = {0.005f, 0.01f, 0.02f};
	CloudEvaluator sphereEval(panoWidth, thresholds), faceEval(panoWidth, thresholds);
	if (!SynthSphericalFusionRun(panoWidth, numPanos, (SynthFaces)faces, options, sphereEval, faces != FACES_NONE ? &faceEval : NULL))
		return false;
	std::fputs(sphereEval.Print().c_str(), stdout);
	if (faces != FACES_NONE)
		std::fputs(faceEval.Print().c_str(), stdout);
	return true;
}

// the point-cloud of a .ply file or of a scene
bool LoadCloud(const String& fileName, PointCloud& pointcloud)
{
	if (Util::getFileExt(fileName).ToLower() == ".mvs") {
		Scene scene;
		const ScopedWorkingFolder workingFolder(Util::getFilePath(fileName));
		if (scene.Load(fileName) == Scene::SCENE_NA)
			return false;
		pointcloud = std::move(scene.pointcloud);
	} else if (!pointcloud.Load(fileName)) {
		std::printf("error: cannot read the cloud '%s'\n", fileName.c_str());
		return false;
	}
	if (pointcloud.IsEmpty()) {
		std::printf("error: empty cloud '%s'\n", fileName.c_str());
		return false;
	}
	return true;
}

// evalcloud <cloud.ply|scene.mvs> <gtFolder> [evalWidth] [th1% th2% th3%]: a cloud against the GT range maps
// *.dmap of a folder (their poses from the .dmap headers); evalWidth is the panorama width the cloud was
// estimated at, for the recall grid and the seam band (default the GT width)
bool ToolEvalCloud(int argc, LPCTSTR* argv)
{
	const int evalWidth(ArgInt(argc, argv, 3, 0));
	float thresholds[CloudEvaluator::NUM_TH] = {0.01f, 0.02f, 0.05f};
	for (int t = 0; t < CloudEvaluator::NUM_TH && 4+t < argc; ++t)
		thresholds[t] = (float)std::atof(argv[4+t])/100;
	PointCloud pointcloud;
	if (!LoadCloud(ArgPath(argv[1]), pointcloud))
		return false;
	const String gtFolder(argv[2]);
	const StringArr names(ListDMaps(gtFolder));
	std::unique_ptr<CloudEvaluator> evaluator;
	for (const String& name: names) {
		DMapFile gt;
		if (!gt.Load(JoinPath(gtFolder, name), false) || gt.depthMap.width() != 2*gt.depthMap.height()) {
			std::printf("error: '%s' is not an equirectangular range map\n", name.c_str());
			return false;
		}
		if (!evaluator)
			evaluator.reset(new CloudEvaluator(evalWidth > 0 ? evalWidth : gt.depthMap.width(), thresholds));
		evaluator->AddView(gt.R, gt.C, gt.depthMap, evalWidth > 0 ? MAXF(ROUND2INT(float(gt.depthMap.width())/evalWidth), 1) : 1);
	}
	if (!evaluator) {
		std::printf("error: no GT range map in '%s'\n", gtFolder.c_str());
		return false;
	}
	evaluator->Evaluate(pointcloud.points);
	VERBOSE("evalcloud: %zu GT range maps, %u cloud points\n%s", (size_t)names.size(), pointcloud.points.size(), evaluator->Print().c_str());
	return true;
}

// facescene <scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize]: the cube-face baseline of a spherical
// scene: every panorama resized to panoWidth is rendered into 6 cube faces (faceSize, default the same pixels
// per steradian: panoWidth/sqrt(12)), its GT range map (gtFolder/depthXXXX.dmap) into face GT z depth-maps
// (outFolder/gt), and the sparse points are kept with the faces they project into
bool ToolFaceScene(int argc, LPCTSTR* argv)
{
	const String sceneFileName(ArgPath(argv[1])), gtFolder(ArgPath(argv[2]));
	String outFolder(ArgPath(argv[3]));
	Util::ensureValidFolderPath(outFolder);
	Util::ensureFolder(outFolder + "gt/");
	const int panoWidth(std::atoi(argv[4]));
	const int faceSize(ArgInt(argc, argv, 5, ROUND2INT(panoWidth/SQRT(12.0)/8)*8));
	Scene scene;
	{
		const ScopedWorkingFolder workingFolder(Util::getFilePath(sceneFileName));
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
		if (imageData.ReadPixels(pano) == NULL) {
			std::printf("error: cannot read the image '%s'\n", imageData.name.c_str());
			return false;
		}
		cv::resize(pano, pano, cv::Size(panoWidth, panoWidth/2), 0, 0, cv::INTER_AREA);
		const std::vector<Image8U3> faceImages(SphereCubeMap::SphericalToTangentialFaces(pano, faces));
		DMapFile gt;
		const String gtFileName(gtFolder + String::FormatString("/depth%04u.dmap", imageData.ID));
		if (!gt.Load(gtFileName, false)) {
			std::printf("error: cannot read the GT range map '%s'\n", gtFileName.c_str());
			return false;
		}
		const DepthMap& gtRange = gt.depthMap;
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
			if (!faceImages[f].Save(face.name)) {
				std::printf("error: cannot write '%s'\n", face.name.c_str());
				return false;
			}
			face.UpdateCamera(faceScene.platforms);
			// the GT range of the panorama at the nearest pixel of each face ray, as z
			DepthMap faceGT(faceSize, faceSize);
			const EMat3 Rt(EMat3(faces.rotations[f]).transpose());
			for (int r = 0; r < faceSize; ++r) {
				for (int c = 0; c < faceSize; ++c) {
					const EVec3 ray(facePinhole.Ray(EVec2(c, r)));
					EVec2 x;
					sphere.Project(Rt*ray, x);
					const Depth range(gtRange(MINF(ROUND2INT(x.y()), gtRange.height()-1), (ROUND2INT(x.x()) + gtRange.width()) % gtRange.width()));
					faceGT(r, c) = range > 0 ? (Depth)(range/ray.norm()) : 0.f;
				}
			}
			if (!ExportDepthDataRaw(outFolder + String::FormatString("gt/depth%04u.dmap", face.ID), face.name, IIndexArr{face.ID}, faceGT.size(),
					face.camera.K, face.camera.R, face.camera.C, 0.f, FLT_MAX, faceGT, NormalMap(), ConfidenceMap(), ViewsMap())) {
				std::printf("error: cannot write the face GT of '%s'\n", face.name.c_str());
				return false;
			}
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
	const ScopedWorkingFolder workingFolder(outFolder);
	if (!faceScene.SaveInterface(outFolder + "scene_faces.mvs"))
		return false;
	std::printf("%u faces of %dx%d px, %zu sparse points\n", faceScene.images.size(), faceSize, faceSize, (size_t)faceScene.pointcloud.points.size());
	return true;
}

// undistortbench <scene.mvs> [numImages]: the load cost of the undistortion over the first numImages images
// undistorted on read: the file decode and the remap ReadPixels adds to it, timed apart; the first image of
// a camera also builds its maps
bool ToolUndistortBench(int argc, LPCTSTR* argv)
{
	const String fileName(ArgPath(argv[1]));
	const ScopedWorkingFolder workingFolder(Util::getFilePath(fileName));
	Scene scene;
	if (scene.Load(fileName) == Scene::SCENE_NA)
		return false;
	const unsigned numImages((unsigned)ArgInt(argc, argv, 2, 8));
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

// add one GT/estimate pair to evaluator; a missing estimate counts as no estimate, a GT of another
// resolution is sampled at the nearest pixel of each estimated pixel center; the model only selects the
// bands: spherical if named so, or if not named and the GT camera matrix is the equirectangular one
bool EvalDepthPair(const String& gtFileName, const String& estFileName, const String& model, DepthEvaluator& evaluator)
{
	DMapFile gt, est;
	if (!gt.Load(gtFileName, true)) {
		std::printf("error: cannot read the depth-map '%s'\n", gtFileName.c_str());
		return false;
	}
	if (!est.Load(estFileName, true)) {
		std::printf("warning: no estimate '%s'\n", estFileName.c_str());
		est.depthMap.create(gt.depthMap.size());
		est.depthMap.memset(0);
	}
	if (gt.normalMap.empty()) {
		gt.normalMap.create(gt.depthMap.size());
		gt.normalMap.memset(0);
	}
	const Spherical<double> sphere(Spherical<double>::FromSize(gt.depthMap.width(), gt.depthMap.height()));
	const bool spherical(model.empty() ? ABS(gt.K(0,0) - sphere.fx) < 1e-9*sphere.fx && ABS(gt.K(1,1) - sphere.fy) < 1e-9*sphere.fy : model == "spherical");
	if (gt.depthMap.size() != est.depthMap.size()) {
		if (int64_t(gt.depthMap.width())*est.depthMap.height() != int64_t(gt.depthMap.height())*est.depthMap.width()) {
			std::printf("error: GT %dx%d and estimate %dx%d differ in aspect ratio\n", gt.depthMap.width(), gt.depthMap.height(), est.depthMap.width(), est.depthMap.height());
			return false;
		}
		const double scale(double(gt.depthMap.width()) / est.depthMap.width());
		DepthMap depthMap(est.depthMap.size());
		NormalMap normalMap(est.depthMap.size());
		for (int r = 0; r < depthMap.height(); ++r) {
			for (int c = 0; c < depthMap.width(); ++c) {
				const int rg(MINF(FLOOR2INT((r + 0.5)*scale), gt.depthMap.height() - 1)), cg(MINF(FLOOR2INT((c + 0.5)*scale), gt.depthMap.width() - 1));
				depthMap(r, c) = gt.depthMap(rg, cg);
				normalMap(r, c) = gt.normalMap(rg, cg);
			}
		}
		gt.depthMap = std::move(depthMap);
		gt.normalMap = std::move(normalMap);
	}
	evaluator.Add(gt.depthMap, gt.normalMap, est.depthMap, est.normalMap, spherical ? CameraModelType::SPHERICAL : CameraModelType::PINHOLE);
	return true;
}

// evaldepth <gt.dmap|gtFolder> <est.dmap|estFolder> [pinhole|spherical]: one GT/estimate pair, or every GT
// *.dmap of a folder against the same file name in the estimate folder, scored per band
bool ToolEvalDepth(int argc, LPCTSTR* argv)
{
	const String model(argc > 3 ? String(argv[3]) : String());
	DepthEvaluator evaluator;
	if (std::filesystem::is_directory(argv[1])) {
		const String gtFolder(argv[1]), estFolder(argv[2]);
		const StringArr names(ListDMaps(gtFolder));
		for (const String& name: names)
			if (!EvalDepthPair(JoinPath(gtFolder, name), JoinPath(estFolder, name), model, evaluator))
				return false;
		std::printf("%zu depth-maps\n", (size_t)names.size());
	} else if (!EvalDepthPair(argv[1], argv[2], model, evaluator)) {
		return false;
	}
	VERBOSE("evaldepth:\n%s", evaluator.Print().c_str());
	return true;
}

} // namespace

bool MVS::SynthTool(int argc, LPCTSTR* argv)
{
	ASSERT(argc > 0);
	struct Mode {
		LPCTSTR name;
		int minArgs; // the mode included
		bool (*run)(int, LPCTSTR*);
		LPCTSTR usage;
	};
	static const Mode modes[] = {
		{_T("synth"), 2, ToolSynth, _T("<outDir> [panoWidth=512]")},
		{_T("mixtest"), 1, ToolMixTest, _T("<verbosity> [gpu=0]")},
		{_T("sphtest"), 1, ToolSphTest, _T("<verbosity> [panoWidth=512] [numPanos=3] [faces=1] [maxElevation: exact warp above, degrees] [maxViews] [maxThreads] [gpu=0]")},
		{_T("conftest"), 1, ToolConfTest, _T("<verbosity>")},
		{_T("sphfuse"), 1, ToolSphFuse, _T("<verbosity> [panoWidth=512] [numPanos=3] [faces=1 (0: none, 1: apart, 2: with the panoramas)] [fusionFilter=2] [maxThreads]")},
		{_T("scorebench"), 1, ToolScoreBench, _T("<verbosity> [panoWidth=1536] [rounds=4] [maxElevation] [faceWidth=panoWidth/4]")},
		{_T("evaldepth"), 3, ToolEvalDepth, _T("<gt.dmap|gtFolder> <est.dmap|estFolder> [pinhole|spherical, default from the GT camera matrix]")},
		{_T("evalcloud"), 3, ToolEvalCloud, _T("<cloud.ply|scene.mvs> <gtFolder> [evalWidth=GT width] [th1%=1 th2%=2 th3%=5]")},
		{_T("facescene"), 5, ToolFaceScene, _T("<scene.mvs> <gtFolder> <outFolder> <panoWidth> [faceSize=panoWidth/sqrt(12)]")},
		{_T("undistortbench"), 2, ToolUndistortBench, _T("<scene.mvs> [numImages=8]")},
	};
	const String name(argv[0]);
	for (const Mode& mode: modes)
		if (name == mode.name && argc >= mode.minArgs)
			return mode.run(argc, argv);
	std::printf("usage:\n");
	for (const Mode& mode: modes)
		std::printf("  Tests %s %s\n", mode.name, mode.usage);
	return false;
}
/*----------------------------------------------------------------*/
