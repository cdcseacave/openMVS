////////////////////////////////////////////////////////////////////
// UtilMetal.mm
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "UtilMetal.h"

#import <Metal/Metal.h>
#import <Foundation/Foundation.h>
#include <dispatch/dispatch.h>

#define METAL_HEALTH_CHECK_MAGIC 0x4D56534Du
#define METAL_HEALTH_CHECK_TIMEOUT_NS (500ull * NSEC_PER_MSEC)

namespace SEACAVE {
namespace METAL {

static const char* kHealthCheckMSL = R"METALSRC(
#include <metal_stdlib>
using namespace metal;
kernel void MetalHealthCheck(device uint* result [[buffer(0)]], uint index [[thread_position_in_grid]]) {
	if (index == 0)
		result[0] = 0x4D56534Du;
}
)METALSRC";

static bool CheckRuntime()
{
	@autoreleasepool {
		id<MTLDevice> device = MTLCreateSystemDefaultDevice();
		if (!device)
			return false;
		id<MTLCommandQueue> queue = [device newCommandQueue];
		if (!queue)
			return false;
		NSError* error = nil;
		id<MTLLibrary> library = [device newLibraryWithSource:[NSString stringWithUTF8String:kHealthCheckMSL] options:[MTLCompileOptions new] error:&error];
		if (!library)
			return false;
		id<MTLFunction> function = [library newFunctionWithName:@"MetalHealthCheck"];
		if (!function)
			return false;
		id<MTLComputePipelineState> pipeline = [device newComputePipelineStateWithFunction:function error:&error];
		if (!pipeline)
			return false;
		id<MTLBuffer> result = [device newBufferWithLength:sizeof(uint32_t) options:MTLResourceStorageModeShared];
		if (!result)
			return false;
		uint32_t* value = static_cast<uint32_t*>(result.contents);
		if (!value)
			return false;
		*value = 0;
		id<MTLCommandBuffer> commandBuffer = [queue commandBuffer];
		if (!commandBuffer)
			return false;
		id<MTLComputeCommandEncoder> encoder = [commandBuffer computeCommandEncoder];
		if (!encoder)
			return false;
		[encoder setComputePipelineState:pipeline];
		[encoder setBuffer:result offset:0 atIndex:0];
		[encoder dispatchThreads:MTLSizeMake(1, 1, 1) threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
		[encoder endEncoding];
		dispatch_semaphore_t semaphore = dispatch_semaphore_create(0);
		[commandBuffer addCompletedHandler:^(id<MTLCommandBuffer>) {
			dispatch_semaphore_signal(semaphore);
		}];
		[commandBuffer commit];
		if (dispatch_semaphore_wait(semaphore, dispatch_time(DISPATCH_TIME_NOW, METAL_HEALTH_CHECK_TIMEOUT_NS)) != 0)
			return false;
		return [commandBuffer status] == MTLCommandBufferStatusCompleted && [commandBuffer error] == nil && *value == METAL_HEALTH_CHECK_MAGIC;
	}
}

bool isRuntimeAvailable()
{
	static const bool bAvailable(CheckRuntime());
	return bAvailable;
}

} // namespace METAL
} // namespace SEACAVE