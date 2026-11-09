/*
* PatchMatchMetal.mm
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

/*
* Metal compute backend for PatchMatch dense densification (Apple Silicon) contributed by leNeo.
* Objective-C++ implementation behind the pure-C++ PIMPL in PatchMatchMetal.h.
* Mirrors MVS::CUDA::PatchMatch::EstimateDepthMap: multi-resolution PatchMatch
* with both photometric and geometric-consistency passes (the latter selected
* via the GEOM function constant and neighbor depth-map textures).
*/

#include "Common.h"
#include "PatchMatchMetal.h"
#include "DepthMap.h"

#ifdef _USE_METAL

#import <Metal/Metal.h>
#import <Foundation/Foundation.h>
#include <simd/simd.h>
#include "PatchMatchMetal_msl.h"

#define METAL_MAX_VIEWS 32   // must match MAX_VIEWS in the shader

namespace MVS {

namespace METAL {

// POD mirrors of the MSL structs (simd guarantees matching layout)
struct MtlCamera { simd_float2 f; simd_float2 pp; simd_float3x3 R; simd_float3 C; simd_int2 size; };
struct MtlParams {
	float fDepthMin, fDepthMax, fThresholdKeepCost;
	int nNumViews, nEstimationIters, nInitTopK, bLowResProcessed, width, height;
	uint32_t sphericalViews;
	float fSinSqWarpMaxElevation, cosMinViewAngle;
};
struct MtlModelView { simd_float3x3 R; simd_float3 t; simd_float3 C; simd_float2 texScale; };

// the camera models a kernel serves: the MODEL function constant of the shader
enum KernelModel { MODEL_PINHOLE, MODEL_PINHOLE_REF, MODEL_SPHERICAL_REF, NUM_MODELS };

struct PatchMatch::Impl {
	id<MTLDevice> device = nil;
	id<MTLCommandQueue> queue = nil;
	id<MTLLibrary> library = nil;
	// the kernels specialized on the function constants [GEOM][MODEL]
	id<MTLComputePipelineState> psInit[2][NUM_MODELS] = {}, psBlack[2][NUM_MODELS] = {}, psRed[2][NUM_MODELS] = {};
	id<MTLComputePipelineState> psFilter = nil;
	id<MTLTexture> dummyDepth = nil;   // 1x1 zero, stands in for neighbors without a depth-map
	id<MTLBuffer> dummyTrig = nil;     // stands in for the trig tables of a pinhole reference
	bool valid = false;
};

static id<MTLComputePipelineState> MakePipe(id<MTLDevice> dev, id<MTLLibrary> lib,
		NSString* name, MTLFunctionConstantValues* fc) {
	NSError* e = nil;
	id<MTLFunction> fn = fc ? [lib newFunctionWithName:name constantValues:fc error:&e]
	                        : [lib newFunctionWithName:name];
	id<MTLComputePipelineState> ps = fn ? [dev newComputePipelineStateWithFunction:fn error:&e] : nil;
	if (!ps)
		VERBOSE("error: Metal kernel %s: %s", name.UTF8String, e ? e.localizedDescription.UTF8String : "not found");
	return ps;
}

PatchMatch::PatchMatch()
{
	impl = new Impl();
	@autoreleasepool {
		impl->device = MTLCreateSystemDefaultDevice();
		if (!impl->device)
			return;
		impl->queue = [impl->device newCommandQueue];
		NSError* err = nil;
		NSString* src = [NSString stringWithUTF8String:kPatchMatchMSL];
		impl->library = [impl->device newLibraryWithSource:src options:[MTLCompileOptions new] error:&err];
		if (!impl->library) {
			VERBOSE("error: the Metal PatchMatch shaders do not compile: %s", err ? err.localizedDescription.UTF8String : "unknown error");
			return;
		}
		bool valid = true;
		for (int geom = 0; geom < 2; ++geom) {
			for (int model = 0; model < NUM_MODELS; ++model) {
				MTLFunctionConstantValues* fc = [MTLFunctionConstantValues new];
				const bool bGeom = geom != 0;
				[fc setConstantValue:&bGeom type:MTLDataTypeBool atIndex:0];
				[fc setConstantValue:&model type:MTLDataTypeInt atIndex:1];
				impl->psInit[geom][model]  = MakePipe(impl->device, impl->library, @"InitializeScore", fc);
				impl->psBlack[geom][model] = MakePipe(impl->device, impl->library, @"BlackPixelProcess", fc);
				impl->psRed[geom][model]   = MakePipe(impl->device, impl->library, @"RedPixelProcess", fc);
				valid = valid && impl->psInit[geom][model] && impl->psBlack[geom][model] && impl->psRed[geom][model];
			}
		}
		impl->psFilter = MakePipe(impl->device, impl->library, @"FilterPlanes", nil);
		// 1x1 zero depth texture for neighbors that have no depth-map yet
		MTLTextureDescriptor* ddesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float width:1 height:1 mipmapped:NO];
		ddesc.usage = MTLTextureUsageShaderRead;
		impl->dummyDepth = [impl->device newTextureWithDescriptor:ddesc];
		const float zero = 0.f;
		[impl->dummyDepth replaceRegion:MTLRegionMake2D(0,0,1,1) mipmapLevel:0 withBytes:&zero bytesPerRow:sizeof(float)];
		impl->dummyTrig = [impl->device newBufferWithLength:sizeof(simd_float2) options:MTLResourceStorageModeShared];
		impl->valid = valid && impl->psFilter && impl->dummyTrig;
	}
}

PatchMatch::~PatchMatch()
{
	Release();
	delete impl;
	impl = nullptr;
}

bool PatchMatch::IsValid() const { return impl && impl->valid; }

void PatchMatch::Init(bool bGeomConsistency)
{
	if (bGeomConsistency) {
		params.bGeomConsistency = true;
		params.nEstimationIters = 1;
	} else {
		params.bGeomConsistency = false;
		params.nEstimationIters = OPTDENSE::nEstimationIters;
	}
}

// No-op: mirrors the CUDA interface, but the Metal backend keeps no persistent
// per-estimate GPU state to free between phases -- every buffer/texture is
// allocated and freed inside EstimateDepthMap's per-scale @autoreleasepool (ARC),
// while device/queue/pipelines live for the whole PatchMatch object lifetime.
void PatchMatch::Release() {}

static MtlCamera ConvertCamera(const Camera& cam, int cols, int rows)
{
	MtlCamera mc;
	mc.f  = simd_make_float2((float)cam.K(0,0), (float)cam.K(1,1));
	mc.pp = simd_make_float2((float)cam.K(0,2), (float)cam.K(1,2));
	for (int j = 0; j < 3; ++j)
		mc.R.columns[j] = simd_make_float3((float)cam.R(0,j), (float)cam.R(1,j), (float)cam.R(2,j));
	mc.C = simd_make_float3((float)cam.C.x, (float)cam.C.y, (float)cam.C.z);
	mc.size = simd_make_int2(cols, rows);
	return mc;
}

void PatchMatch::EstimateDepthMap(DepthData& depthData)
{
	if (!IsValid())
		return;
	TD_TIMER_STARTD();
	ASSERT(depthData.images.size() > 1);

	DepthData& fullResDepthData(depthData);
	const bool geom = params.bGeomConsistency;
	const unsigned totalScaleNumber(geom ? 0u : OPTDENSE::nSubResolutionLevels);
	DepthMap lowResDepthMap;
	NormalMap lowResNormalMap;
	ViewsMap lowResViewsMap;
	// the shader's texture arrays hold METAL_MAX_VIEWS entries (reference + neighbors);
	// clamp so a user-configured OPTDENSE::nMaxViews beyond the cap cannot overflow the
	// texture bindings or index past the shader arrays (CUDA guards the same cap with an
	// ASSERT in UploadCameras). images are score-ordered, so we keep the best neighbors.
	ASSERT(depthData.images.size() <= METAL_MAX_VIEWS);
	const IIndex numImages = MINF((IIndex)depthData.images.size(), (IIndex)METAL_MAX_VIEWS);
	params.nNumViews = (int)numImages - 1;
	params.nInitTopK = MINF(params.nInitTopK, params.nNumViews);
	params.fDepthMin = depthData.dMin;
	params.fDepthMax = depthData.dMax;
	const int maxPixelViews(MINF(params.nNumViews, 4));
	// the camera models: bit i set if view i (0 the reference) is equirectangular; they select the
	// kernels (MODEL), as in CUDA::PatchMatch::RunCUDA
	uint32_t sphericalViews(0);
	for (IIndex i = 0; i < numImages; ++i)
		if (depthData.images[i].IsSpherical())
			sphericalViews |= 1u << i;
	const KernelModel model(sphericalViews == 0 ? MODEL_PINHOLE : (sphericalViews & 1u ? MODEL_SPHERICAL_REF : MODEL_PINHOLE_REF));
	const float fSinSqWarpMaxElevation(SQUARE(SIN(D2R(DepthEstimatorBase::fSphericalWarpMaxElevation))));
	const float cosMinViewAngle(COS(D2R(OPTDENSE::fMinAngle)));
	// the view gate of the model kernels holds one bit per neighbor view
	ASSERT(model == MODEL_PINHOLE || params.nNumViews < 32);

	for (unsigned scaleNumber = totalScaleNumber + 1; scaleNumber-- > 0; ) {
		// per-scale pool: drains this scale's textures/buffers/command buffer before
		// the next scale allocates, instead of holding every scale's Metal objects
		// resident until the whole multi-resolution loop returns
		@autoreleasepool {
		const float scale = 1.f / POWI(2, scaleNumber);
		DepthData currentDepthData(DepthMapsData::ScaleDepthData(fullResDepthData, scale));
		DepthData& dd(scaleNumber == 0 ? fullResDepthData : currentDepthData);
		const Image8U::Size size(dd.images.front().image.size());
		params.bLowResProcessed = false;
		if (scaleNumber != totalScaleNumber) {
			params.bLowResProcessed = true;
			cv::resize(lowResDepthMap, dd.depthMap, size, 0, 0, cv::INTER_NEAREST);
			cv::resize(lowResNormalMap, dd.normalMap, size, 0, 0, cv::INTER_NEAREST);
			cv::resize(lowResViewsMap, dd.viewsMap, size, 0, 0, cv::INTER_NEAREST);
		} else {
			if (totalScaleNumber > 0) {
				fullResDepthData.depthMap.release();
				fullResDepthData.normalMap.release();
				fullResDepthData.confMap.release();
				fullResDepthData.viewsMap.release();
			}
			if (dd.viewsMap.empty())
				dd.viewsMap.create(size);
		}
		if (scaleNumber == 0 && dd.confMap.empty())
			dd.confMap.create(size);

		params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep;
		if (totalScaleNumber) {
			if (scaleNumber > 0 && scaleNumber != totalScaleNumber)
				params.fThresholdKeepCost = 0.f;
			else if (scaleNumber == totalScaleNumber || (!geom && OPTDENSE::nEstimationGeometricIters))
				params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep * 1.2f;
		} else if (!geom && OPTDENSE::nEstimationGeometricIters) {
			params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep * 1.2f;
		}

		const int W = size.width, Hh = size.height;
		const int area = W * Hh;
		id<MTLDevice> dev = impl->device;

		// upload cameras + image textures; each view is sized to its own image
		// (neighbor views can differ in resolution from the reference, so a shared
		// reference-sized descriptor would overflow replaceRegion for larger neighbors)
		std::vector<MtlCamera> cams(numImages);
		// the views relative to the reference, for the model kernels (CUDA::PatchMatch::UploadCameras)
		std::vector<MtlModelView> views(numImages);
		const Eigen::Matrix3d Rref(Eigen::Map<const SEACAVE::Matrix3x3::EMat>(dd.images[0].camera.R.val));
		const Eigen::Vector3d Cref(Eigen::Map<const SEACAVE::Point3::EVec>(dd.images[0].camera.C.ptr()));
		NSMutableArray<id<MTLTexture>>* texs = [NSMutableArray arrayWithCapacity:numImages];
		for (IIndex i = 0; i < numImages; ++i) {
			const DepthData::ViewData& view = dd.images[i];
			const Image32F& image = view.image;
			cams[i] = ConvertCamera(view.camera, image.cols, image.rows);
			const bool bSpherical = ((sphericalViews >> i) & 1u) != 0;
			// an equirectangular camera keeps its angular K on the grid of every level (fx=W/2pi, fy=H/pi)
			ASSERT(!bSpherical || (ISEQUAL(view.camera.K(0,0)*REAL(TWO_PI), REAL(image.cols)) && ISEQUAL(view.camera.K(1,1)*REAL(PI), REAL(image.rows))));
			const Eigen::Matrix3d Ri(Eigen::Map<const SEACAVE::Matrix3x3::EMat>(view.camera.R.val));
			const Eigen::Vector3d Ci(Eigen::Map<const SEACAVE::Point3::EVec>(view.camera.C.ptr()));
			const Eigen::Matrix3d Rrel(Ri * Rref.transpose());
			const Eigen::Vector3d t(Ri * (Cref - Ci));
			const Eigen::Vector3d C(Rref * (Ci - Cref)); // -Rrel^T*t
			MtlModelView& mv = views[i];
			for (int j = 0; j < 3; ++j)
				mv.R.columns[j] = simd_make_float3((float)Rrel(0,j), (float)Rrel(1,j), (float)Rrel(2,j));
			mv.t = simd_make_float3((float)t.x(), (float)t.y(), (float)t.z());
			mv.C = simd_make_float3((float)C.x(), (float)C.y(), (float)C.z());
			mv.texScale = bSpherical ? simd_make_float2(1.f/image.cols, 1.f/image.rows) : simd_make_float2(1.f, 1.f);
			MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
				width:image.cols height:image.rows mipmapped:NO];
			td.usage = MTLTextureUsageShaderRead;
			id<MTLTexture> t = [dev newTextureWithDescriptor:td];
			[t replaceRegion:MTLRegionMake2D(0, 0, image.cols, image.rows) mipmapLevel:0
				withBytes:image.ptr<float>() bytesPerRow:image.step[0]];
			[texs addObject:t];
		}

		// device buffers
		id<MTLBuffer> bCams = [dev newBufferWithBytes:cams.data() length:sizeof(MtlCamera)*numImages options:MTLResourceStorageModeShared];
		id<MTLBuffer> bPlanes = [dev newBufferWithLength:sizeof(simd_float4)*area options:MTLResourceStorageModeShared];
		id<MTLBuffer> bLow    = [dev newBufferWithLength:sizeof(float)*area options:MTLResourceStorageModeShared];
		id<MTLBuffer> bCosts  = [dev newBufferWithLength:sizeof(float)*area options:MTLResourceStorageModeShared];
		id<MTLBuffer> bRng    = [dev newBufferWithLength:sizeof(uint32_t)*area options:MTLResourceStorageModeShared];
		id<MTLBuffer> bSel    = [dev newBufferWithLength:sizeof(uint32_t)*area options:MTLResourceStorageModeShared];
		memset(bRng.contents, 0, sizeof(uint32_t)*area);
		memset(bSel.contents, 0, sizeof(uint32_t)*area);

		// seed planes (normal.xyz, depth) from the (possibly empty) maps
		simd_float4* planes = (simd_float4*)bPlanes.contents;
		const bool haveDepth = !dd.depthMap.empty();
		const bool haveNormal = !dd.normalMap.empty();
		for (int r = 0; r < Hh; ++r)
			for (int c = 0; c < W; ++c) {
				const int idx = r*W + c;
				simd_float4 pl = simd_make_float4(0,0,0,0);
				if (haveNormal) { const Normal& n = dd.normalMap(r,c); pl.x=n.x; pl.y=n.y; pl.z=n.z; }
				if (haveDepth) pl.w = dd.depthMap(r,c);
				planes[idx] = pl;
			}
		if (params.bLowResProcessed && haveDepth) {
			float* low = (float*)bLow.contents;
			for (int r = 0; r < Hh; ++r)
				for (int c = 0; c < W; ++c)
					low[r*W+c] = dd.depthMap(r,c);
		}

		MtlParams mp{ params.fDepthMin, params.fDepthMax, params.fThresholdKeepCost,
			params.nNumViews, params.nEstimationIters, params.nInitTopK,
			params.bLowResProcessed ? 1 : 0, W, Hh,
			sphericalViews, fSinSqWarpMaxElevation, cosMinViewAngle };
		id<MTLBuffer> bPrm = [dev newBufferWithBytes:&mp length:sizeof(MtlParams) options:MTLResourceStorageModeShared];
		id<MTLBuffer> bViews = [dev newBufferWithBytes:views.data() length:sizeof(MtlModelView)*numImages options:MTLResourceStorageModeShared];

		// the sines and cosines of the longitude of every column and of the elevation of every row of an
		// equirectangular reference, its rays and ray frames being built from them
		id<MTLBuffer> bTrig = impl->dummyTrig;
		if (model == MODEL_SPHERICAL_REF) {
			// the seam-wrapped propagation keeps the red/black parity of a pixel and its wrapped
			// neighbors only for an even width
			ASSERT(W % 2 == 0);
			const Spherical<double> camera(MakeCameraModel<Spherical<double>>(dd.images.front().camera));
			bTrig = [dev newBufferWithLength:sizeof(simd_float2)*(W + Hh) options:MTLResourceStorageModeShared];
			simd_float2* trig = (simd_float2*)bTrig.contents;
			for (int c = 0; c < W; ++c) {
				const double lon(camera.Angles(Eigen::Vector2d(c, 0)).x());
				trig[c] = simd_make_float2((float)std::sin(lon), (float)std::cos(lon));
			}
			for (int r = 0; r < Hh; ++r) {
				const double el(camera.Angles(Eigen::Vector2d(0, r)).y());
				trig[W + r] = simd_make_float2((float)std::sin(el), (float)std::cos(el));
			}
		}

		// geometric-consistency: upload each neighbor's depth-map as a texture; a dummy
		// 1x1 zero stands in when a neighbor has none (GeometricConsistencyWeight -> maxDist).
		// depthTexs[imgId] is the depth-map of view (imgId+1), matching the shader's pairing.
		NSMutableArray<id<MTLTexture>>* depthTexs = [NSMutableArray arrayWithCapacity:params.nNumViews];
		for (IIndex i = 1; i < numImages; ++i) {
			id<MTLTexture> dt = impl->dummyDepth;
			if (geom) {
				const DepthMap& dmSrc = dd.images[i].depthMap;
				if (!dmSrc.empty()) {
					DepthMap dmap = dmSrc;
					const Image8U::Size nsz = dd.images[i].image.size();
					if (dmap.size() != nsz) // a neighbor image scaled onto its uniform grid
						MVS::Image::ResizeUniform(dmap, dmap, nsz, cv::INTER_LINEAR);
					MTLTextureDescriptor* dtd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
						width:dmap.cols height:dmap.rows mipmapped:NO];
					dtd.usage = MTLTextureUsageShaderRead;
					dt = [dev newTextureWithDescriptor:dtd];
					[dt replaceRegion:MTLRegionMake2D(0,0,dmap.cols,dmap.rows) mipmapLevel:0
						withBytes:dmap.ptr<float>() bytesPerRow:dmap.step[0]];
				}
			}
			[depthTexs addObject:dt];
		}

		auto bindCommon = [&](id<MTLComputeCommandEncoder> enc) {
			for (IIndex i = 0; i < numImages; ++i) [enc setTexture:texs[i] atIndex:i];
			for (int v = 0; v < params.nNumViews; ++v) [enc setTexture:depthTexs[v] atIndex:METAL_MAX_VIEWS+v];
			[enc setBuffer:bCams offset:0 atIndex:0];
			[enc setBuffer:bPlanes offset:0 atIndex:1];
			[enc setBuffer:bLow offset:0 atIndex:2];
			[enc setBuffer:bCosts offset:0 atIndex:3];
			[enc setBuffer:bRng offset:0 atIndex:4];
			[enc setBuffer:bSel offset:0 atIndex:5];
			[enc setBuffer:bPrm offset:0 atIndex:6];
			[enc setBuffer:bViews offset:0 atIndex:8];
			[enc setBuffer:bTrig offset:0 atIndex:9];
		};
		const MTLSize tg = MTLSizeMake(32, 8, 1);
		id<MTLComputePipelineState> psI = impl->psInit[geom][model];
		id<MTLComputePipelineState> psB = impl->psBlack[geom][model];
		id<MTLComputePipelineState> psR = impl->psRed[geom][model];

		// one command buffer per scale: each pass gets its own encoder so Metal's
		// automatic hazard tracking serializes the read-after-write dependencies on
		// the shared device buffers, while a single host sync at the end replaces the
		// per-kernel waitUntilCompleted round-trips (the kernels were never able to
		// overlap anyway, so results are identical — only the CPU stalls are removed).
		id<MTLCommandBuffer> cb = [impl->queue commandBuffer];
		// InitializeScore (full grid)
		{
			id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
			[enc setComputePipelineState:psI];
			bindCommon(enc);
			[enc dispatchThreads:MTLSizeMake(W, Hh, 1) threadsPerThreadgroup:tg];
			[enc endEncoding];
		}
		// checkerboard iterations
		for (int iter = 0; iter < params.nEstimationIters; ++iter) {
			for (int pass = 0; pass < 2; ++pass) {
				id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
				[enc setComputePipelineState:(pass==0 ? psB : psR)];
				bindCommon(enc);
				// setBytes copies the value into the command buffer at encode time,
				// so each pass captures its own iter without a per-iter MTLBuffer
				[enc setBytes:&iter length:sizeof(int) atIndex:7];
				[enc dispatchThreads:MTLSizeMake(W, (Hh+1)/2, 1) threadsPerThreadgroup:tg];
				[enc endEncoding];
			}
		}
		// FilterPlanes (full grid) if requested
		if (params.fThresholdKeepCost > 0) {
			id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
			[enc setComputePipelineState:impl->psFilter];
			[enc setBuffer:bPlanes offset:0 atIndex:1];
			[enc setBuffer:bCosts offset:0 atIndex:3];
			[enc setBuffer:bSel offset:0 atIndex:5];
			[enc setBuffer:bPrm offset:0 atIndex:6];
			[enc dispatchThreads:MTLSizeMake(W, Hh, 1) threadsPerThreadgroup:tg];
			[enc endEncoding];
		}
		[cb commit];
		[cb waitUntilCompleted];

		// readback: planes -> depth/normal, costs -> conf, sel -> views
		const float* costs = (const float*)bCosts.contents;
		const uint32_t* sel = (const uint32_t*)bSel.contents;
		for (int r = 0; r < Hh; ++r)
			for (int c = 0; c < W; ++c) {
				const int idx = r*W + c;
				const simd_float4 pl = planes[idx];
				const Depth depth = pl.w;
				dd.depthMap(r,c) = depth;
				dd.normalMap(r,c) = Normal(pl.x, pl.y, pl.z);
				if (scaleNumber == 0) {
					float& conf = dd.confMap(r,c);
					conf = costs[idx];
					conf = conf >= 1.f ? 0.f : 1.f - conf;
					ViewsID& views = dd.viewsMap(r,c);
					if (depth > 0) {
						const uint32_t bitviews = sel[idx];
						int j = 0;
						for (int i = 0; i < 32; ++i)
							if (bitviews & (1u << i)) {
								views[j] = (uint8_t)i;
								if (++j == maxPixelViews) break;
							}
						while (j < 4) views[j++] = 255;
					} else {
						views = ViewsID(255,255,255,255);
					}
				}
			}

		if (scaleNumber > 0) {
			lowResDepthMap = dd.depthMap;
			lowResNormalMap = dd.normalMap;
			lowResViewsMap = dd.viewsMap;
		}
		} // @autoreleasepool (per scale)
	}

	// apply the ignore mask (with the validity mask of an undistorted image), as CUDA does
	{
		const DepthData::ViewData& view = depthData.GetView();
		BitMatrix mask;
		if (DepthEstimator::ImportIgnoreMask(*view.pImageData, depthData.depthMap.size(), OPTDENSE::nIgnoreMaskLabel, mask))
			depthData.ApplyIgnoreMask(mask);
	}

	DEBUG_EXTRA("Depth-map for image %3u estimated via Metal: %dx%d (%s)",
		depthData.images.front().GetID(),
		depthData.images.front().image.cols, depthData.images.front().image.rows,
		TD_TIMER_GET_FMT().c_str());
}

bool PatchMatch::EvaluateModelMath(std::vector<ModelMathSample>& samples) const
{
	ASSERT(IsValid() && !samples.empty());
	@autoreleasepool {
		id<MTLComputePipelineState> ps = MakePipe(impl->device, impl->library, @"TestModelMath", nil);
		if (!ps)
			return false;
		const size_t size(sizeof(ModelMathSample)*samples.size());
		id<MTLBuffer> buffer = [impl->device newBufferWithBytes:samples.data() length:size options:MTLResourceStorageModeShared];
		const uint32_t numSamples((uint32_t)samples.size());
		id<MTLCommandBuffer> cb = [impl->queue commandBuffer];
		id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
		[enc setComputePipelineState:ps];
		[enc setBuffer:buffer offset:0 atIndex:0];
		[enc setBytes:&numSamples length:sizeof(numSamples) atIndex:1];
		[enc dispatchThreads:MTLSizeMake(numSamples, 1, 1) threadsPerThreadgroup:MTLSizeMake(64, 1, 1)];
		[enc endEncoding];
		[cb commit];
		[cb waitUntilCompleted];
		if (cb.status != MTLCommandBufferStatusCompleted)
			return false;
		memcpy(samples.data(), buffer.contents, size);
	}
	return true;
}

} // namespace METAL

} // namespace MVS

#endif // _USE_METAL
