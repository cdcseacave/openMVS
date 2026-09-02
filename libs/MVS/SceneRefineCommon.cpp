/*
* SceneRefineCommon.cpp
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

#include "Common.h"
// Tell DEFVAR_OPTION/DEFOPT_SPACE (defined in libs/Common/Common.h) to tag
// the OPTREFINE namespace's data symbols and helpers with MVS_API so they
// are exported from MVS.dll instead of the default Common-side tag -- same
// trick DepthMap.cpp uses for OPTDENSE.
#undef OPTCONFIG_API
#define OPTCONFIG_API MVS_API
#include "SceneRefineCommon.h"
#include "Scene.h"
#include <fstream>
#include <cstdio>
#include <utility>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define DEFVAR_OPTREFINE_int32(name, title, desc, ...)  DEFVAR_int32(OPTREFINE, name, title, desc, __VA_ARGS__)
#define DEFVAR_OPTREFINE_float(name, title, desc, ...)  DEFVAR_float(OPTREFINE, name, title, desc, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {
DEFOPT_SPACE(OPTREFINE, _T("Refine"))

DEFVAR_OPTREFINE_int32(nIgnoreMaskLabel, "Ignore Mask Label", "label id used during ignore mask filter (<0 - disabled)", "-1")
DEFVAR_OPTREFINE_int32(nImageGradient, "Image Gradient", "image derivative stencil (0 - 3x5 separable, 1 - central, 2 - Sobel, 3 - bilinear interpolant derivative)", "1")
DEFVAR_OPTREFINE_float(fGateMeanDiff, "Gate Mean Diff", "reject a pixel pair whose local mean differs by more than this (0 - disabled)", "0.4")
DEFVAR_OPTREFINE_float(fGateVarRatio, "Gate Var Ratio", "reject a pixel pair whose local variance ratio exceeds this (0 - disabled)", "8.0")
DEFVAR_OPTREFINE_int32(nOptimizer, "Optimizer", "vertex position optimizer (0 - bold, 1 - fixed)", "0")

} // namespace MVS


// the neighbor views one image contributes refinement pairs with; hoisted out of both backends'
// constructors (CPU MeshRefine::ThSelectNeighbors, SceneRefine.cpp; CUDA MeshRefineCUDA's
// constructor, SceneRefineCUDA.cpp) so the filter thresholds and the missing-neighbor recovery
// exist once -- see the doc comment in SceneRefineCommon.h
bool MVS::SelectRefineNeighbors(Scene& scene, uint32_t idxImage, unsigned nMaxViews, ViewScoreArr& neighbors)
{
	// keep only best neighbor views
	const float fMinArea(0.1f);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(D2R(2.5f)), fMaxAngle(D2R(45.f));
	Image& imageData = scene.images[idxImage];
	if (!imageData.IsValid())
		return false;
	if (imageData.neighbors.IsEmpty()) {
		IndexArr points;
		scene.SelectNeighborViews(idxImage, points);
	}
	neighbors = imageData.neighbors;
	Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
	return true;
}

// load, gray-convert, blur and resize one refine image at the given scale;
// hoisted common part of CPU MeshRefine::ThInitImage (SceneRefine.cpp) and
// CUDA MeshRefineCUDA::InitImages (SceneRefineCUDA.cpp) -- see the doc
// comment in SceneRefineCommon.h; behaviour must stay byte-identical to what
// each backend did inline before this hoist.
bool MVS::PrepareRefineImage(Image& imageData, const PlatformArr& platforms,
	unsigned nResolutionLevel, unsigned nMinResolution, float scale, float sigma, Image32F& gray)
{
	ASSERT(imageData.IsValid());
	// load and init image
	unsigned level(nResolutionLevel);
	const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
	if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && !imageData.ReloadImage(imageSize))
		return false;
	imageData.image.toGray(gray, cv::COLOR_BGR2GRAY, true);
	imageData.image.release();
	if (sigma > 0)
		cv::GaussianBlur(gray, gray, cv::Size(), sigma);
	if (scale < 1.0) {
		cv::resize(gray, gray, cv::Size(), scale, scale, cv::INTER_AREA);
		imageData.width = gray.width(); imageData.height = gray.height();
	}
	imageData.UpdateCamera(platforms);
	return true;
}

// per-view keep-mask, called right after PrepareRefineImage -- see the doc comment in
// SceneRefineCommon.h; leaves keepMask empty unless masking is enabled and the image has a mask
// file to load (DepthEstimator::ImportKeepMask itself reports a missing file, but only at a level
// below what apps/RefineMesh's own once-per-image entry check surfaces)
void MVS::PrepareRefineImageMask(const Image& imageData, const cv::Size& size, BitMatrix& keepMask)
{
	keepMask.release();
	if (OPTREFINE::nIgnoreMaskLabel < 0)
		return;
	DepthEstimator::ImportKeepMask(imageData, size, (uint8_t)OPTREFINE::nIgnoreMaskLabel, keepMask);
}

void MVS::ComputeRefineImageGradient(const Image32F& gray, Image32F& gradX, Image32F& gradY)
{
	ASSERT(!gray.empty());
	switch (OPTREFINE::nImageGradient) {
	case 1: {
		// central differences
		const cv::Matx13f kernel(-0.5f, 0.f, 0.5f);
		cv::filter2D(gray, gradX, CV_32F, kernel);
		cv::filter2D(gray, gradY, CV_32F, kernel.t());
		break; }
	case 2:
		cv::Sobel(gray, gradX, CV_32F, 1, 0, 3, 1.0/8.0);
		cv::Sobel(gray, gradY, CV_32F, 0, 1, 3, 1.0/8.0);
		break;
	default: {
		const TMatrix<float,3,5> kernel(CreateDerivativeKernel3x5());
		cv::filter2D(gray, gradX, CV_32F, kernel);
		cv::filter2D(gray, gradY, CV_32F, kernel.t());
		break; }
	}
}
/*----------------------------------------------------------------*/


// R E F I N E   D E B U G /////////////////////////////////////////

// PLY vertex layout for RefineDebug::ExportGradients(); mirrors the
// BasicPLY pattern in Mesh.cpp (same PLY class, same offsetof-on-member
// style) but is local to this diagnostic export -- it has nothing to do
// with the mesh's own vertex/face PLY elements.
namespace {
struct GradPlyVertex {
	Point3f pos;
	Point3f combined;
	Point3f photo;
	float pnorm;
	Point3f smooth1;
	Point3f smooth2;
	uint8_t boundary;
};
const char* gradElemNames[] = { "vertex" };
const PLY::PlyProperty gradProps[] = {
	{"x",        PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,pos.x),      0, 0, 0, 0},
	{"y",        PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,pos.y),      0, 0, 0, 0},
	{"z",        PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,pos.z),      0, 0, 0, 0},
	{"gx",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,combined.x), 0, 0, 0, 0},
	{"gy",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,combined.y), 0, 0, 0, 0},
	{"gz",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,combined.z), 0, 0, 0, 0},
	{"px",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,photo.x),    0, 0, 0, 0},
	{"py",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,photo.y),    0, 0, 0, 0},
	{"pz",       PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,photo.z),    0, 0, 0, 0},
	{"pnorm",    PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,pnorm),      0, 0, 0, 0},
	{"s1x",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth1.x),  0, 0, 0, 0},
	{"s1y",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth1.y),  0, 0, 0, 0},
	{"s1z",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth1.z),  0, 0, 0, 0},
	{"s2x",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth2.x),  0, 0, 0, 0},
	{"s2y",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth2.y),  0, 0, 0, 0},
	{"s2z",      PLY::Float32, PLY::Float32, offsetof(GradPlyVertex,smooth2.z),  0, 0, 0, 0},
	{"boundary", PLY::Uint8,   PLY::Uint8,   offsetof(GradPlyVertex,boundary),   0, 0, 0, 0},
};
} // namespace

const String& RefineDebug::Dir()
{
	static const String dir = [] {
		String d;
		const char* env = getenv("OMVS_REFINE_DEBUG_DIR");
		if (env && *env) {
			d = env;
			Util::ensureValidFolderPath(d);
			Util::ensureFolder(d);
		}
		return d;
	}();
	return dir;
}

bool RefineDebug::Pair(uint32_t& idxImageA, uint32_t& idxImageB)
{
	static const std::pair<uint32_t,uint32_t> pair = [] {
		uint32_t a(NO_ID), b(NO_ID);
		const char* env = getenv("OMVS_REFINE_DEBUG_PAIR");
		if (env && sscanf(env, "%u,%u", &a, &b) != 2) {
			// a set-but-malformed request must not silently disable the export it asked for
			VERBOSE("error: OMVS_REFINE_DEBUG_PAIR '%s' does not parse as A,B: pair export disabled", env);
			a = b = NO_ID;
		}
		return std::make_pair(a, b);
	}();
	if (pair.first == NO_ID)
		return false;
	idxImageA = pair.first;
	idxImageB = pair.second;
	return true;
}

void RefineDebug::ExportGradients(unsigned nScale, unsigned iter, uint32_t numVertices,
	const Point3f* pos, const Point3f* combined,
	const Point3f* photo, const float* photoCount,
	const Point3f* smooth1, const Point3f* smooth2,
	const uint8_t* boundary)
{
	if (Dir().empty())
		return;
	ASSERT(numVertices > 0 && pos && combined && photo && photoCount && smooth1 && smooth2 && boundary);
	const String fileName(Dir()+String::FormatString("refine_grad_s%u_i%u.ply", nScale, iter));
	PLY ply;
	if (!ply.write(fileName, 1, gradElemNames, PLY::BINARY_LE)) {
		VERBOSE("error: failed to export '%s'", fileName.c_str());
		return;
	}
	ply.describe_property(gradElemNames[0], (int)SizeOfArray(gradProps), gradProps);
	ply.element_count(gradElemNames[0], (int)numVertices);
	if (!ply.header_complete()) {
		VERBOSE("error: failed to export '%s'", fileName.c_str());
		return;
	}
	ply.put_element_setup(gradElemNames[0]);
	GradPlyVertex v;
	for (uint32_t i=0; i<numVertices; ++i) {
		v.pos = pos[i];
		v.combined = combined[i];
		v.photo = photo[i];
		v.pnorm = photoCount[i];
		v.smooth1 = smooth1[i];
		v.smooth2 = smooth2[i];
		v.boundary = boundary[i];
		ply.put_element(&v);
	}
}

// write one 32-bit float, little-endian PFM (single channel); verified by
// reading it back with numpy (bench/refine_parity.py) -- rows are stored
// bottom-to-top per the PFM spec, hence the reversed loop below.
static void WritePFM(const String& fileName, const float* data, int width, int height)
{
	ASSERT(data && width > 0 && height > 0);
	std::ofstream f(fileName.c_str(), std::ios::binary);
	if (!f.is_open()) {
		VERBOSE("error: failed to export '%s'", fileName.c_str());
		return;
	}
	f << "Pf\n" << width << ' ' << height << "\n-1.0\n";
	for (int r=height-1; r>=0; --r)
		f.write((const char*)(data+(size_t)r*width), sizeof(float)*width);
	// a truncated map (disk full) must not pass as evidence: the consumer is the parity
	// harness comparing the two backends, and a plausible-looking partial file is worse
	// than a missing one
	f.flush();
	if (!f.good())
		VERBOSE("error: failed to export '%s' completely", fileName.c_str());
}

void RefineDebug::ExportPairMap(unsigned nScale, unsigned iter, uint32_t idxImageA, uint32_t idxImageB,
	const char* name, const float* data, int width, int height)
{
	if (Dir().empty())
		return;
	WritePFM(Dir()+String::FormatString("pair_%u_%u_s%u_i%u_%s.pfm", idxImageA, idxImageB, nScale, iter, name), data, width, height);
}

void RefineDebug::ExportPairMask(unsigned nScale, unsigned iter, uint32_t idxImageA, uint32_t idxImageB,
	const uint8_t* mask, int width, int height)
{
	if (Dir().empty())
		return;
	ASSERT(mask && width > 0 && height > 0);
	Image8U img(height, width);
	uint8_t* pDst = img.getData();
	for (size_t i=0, n=(size_t)width*height; i<n; ++i)
		pDst[i] = mask[i] ? 255 : 0;
	const String fileName(Dir()+String::FormatString("pair_%u_%u_s%u_i%u_mask.png", idxImageA, idxImageB, nScale, iter));
	if (!img.Save(fileName))
		VERBOSE("error: failed to export '%s'", fileName.c_str());
}
/*----------------------------------------------------------------*/
