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
};

struct PatchMatch::Impl {
	id<MTLDevice> device = nil;
	id<MTLCommandQueue> queue = nil;
	id<MTLComputePipelineState> psInit = nil, psBlack = nil, psRed = nil, psFilter = nil;       // GEOM=false
	id<MTLComputePipelineState> psInitG = nil, psBlackG = nil, psRedG = nil;                    // GEOM=true
	id<MTLTexture> dummyDepth = nil;   // 1x1 zero, stands in for neighbors without a depth-map
	bool valid = false;
};

static id<MTLComputePipelineState> MakePipe(id<MTLDevice> dev, id<MTLLibrary> lib,
		NSString* name, MTLFunctionConstantValues* fc) {
	NSError* e = nil;
	id<MTLFunction> fn = fc ? [lib newFunctionWithName:name constantValues:fc error:&e]
	                        : [lib newFunctionWithName:name];
	if (!fn) return nil;
	return [dev newComputePipelineStateWithFunction:fn error:&e];
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
		id<MTLLibrary> lib = [impl->device newLibraryWithSource:src options:[MTLCompileOptions new] error:&err];
		if (!lib)
			return;
		MTLFunctionConstantValues* fcN = [MTLFunctionConstantValues new];
		bool gf = false; [fcN setConstantValue:&gf type:MTLDataTypeBool atIndex:0];
		MTLFunctionConstantValues* fcG = [MTLFunctionConstantValues new];
		bool gt = true;  [fcG setConstantValue:&gt type:MTLDataTypeBool atIndex:0];
		impl->psInit   = MakePipe(impl->device, lib, @"InitializeScore", fcN);
		impl->psBlack  = MakePipe(impl->device, lib, @"BlackPixelProcess", fcN);
		impl->psRed    = MakePipe(impl->device, lib, @"RedPixelProcess", fcN);
		impl->psFilter = MakePipe(impl->device, lib, @"FilterPlanes", nil);
		impl->psInitG  = MakePipe(impl->device, lib, @"InitializeScore", fcG);
		impl->psBlackG = MakePipe(impl->device, lib, @"BlackPixelProcess", fcG);
		impl->psRedG   = MakePipe(impl->device, lib, @"RedPixelProcess", fcG);
		// 1x1 zero depth texture for neighbors that have no depth-map yet
		MTLTextureDescriptor* ddesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float width:1 height:1 mipmapped:NO];
		ddesc.usage = MTLTextureUsageShaderRead;
		impl->dummyDepth = [impl->device newTextureWithDescriptor:ddesc];
		const float zero = 0.f;
		[impl->dummyDepth replaceRegion:MTLRegionMake2D(0,0,1,1) mipmapLevel:0 withBytes:&zero bytesPerRow:sizeof(float)];
		impl->valid = impl->psInit && impl->psBlack && impl->psRed && impl->psFilter
			&& impl->psInitG && impl->psBlackG && impl->psRedG;
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
		NSMutableArray<id<MTLTexture>>* texs = [NSMutableArray arrayWithCapacity:numImages];
		for (IIndex i = 0; i < numImages; ++i) {
			const DepthData::ViewData& view = dd.images[i];
			const Image32F& image = view.image;
			cams[i] = ConvertCamera(view.camera, image.cols, image.rows);
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
			params.bLowResProcessed ? 1 : 0, W, Hh };
		id<MTLBuffer> bPrm = [dev newBufferWithBytes:&mp length:sizeof(MtlParams) options:MTLResourceStorageModeShared];

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
					if (dmap.size() != nsz)
						cv::resize(dmap, dmap, nsz, 0, 0, cv::INTER_LINEAR);
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
		};
		const MTLSize tg = MTLSizeMake(32, 8, 1);
		id<MTLComputePipelineState> psI = geom ? impl->psInitG : impl->psInit;
		id<MTLComputePipelineState> psB = geom ? impl->psBlackG : impl->psBlack;
		id<MTLComputePipelineState> psR = geom ? impl->psRedG : impl->psRed;

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

	DEBUG_EXTRA("Depth-map for image %3u estimated via Metal: %dx%d (%s)",
		depthData.images.front().GetID(),
		depthData.images.front().image.cols, depthData.images.front().image.rows,
		TD_TIMER_GET_FMT().c_str());
}

} // namespace METAL

} // namespace MVS

#endif // _USE_METAL
