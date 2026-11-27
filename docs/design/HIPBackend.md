# HIP Backend (AMD GPUs)

## 1. Purpose and scope

The GPU code of OpenMVS is written once, in CUDA: the PatchMatch depth estimation
(`libs/MVS/PatchMatchCUDA.cu`), the confidence recalibration (`ConfidenceCUDA.cu`), the mesh refinement
(`SceneRefineCUDA.cu`) and the host code driving them. A HIP build (`-DOpenMVS_USE_HIP=ON`) compiles that same
code with hipcc for AMD GPUs through ROCm. There is no HIP copy of any kernel; every algorithm change made to
the CUDA code reaches AMD GPUs with it. The Apple GPUs keep their own Metal port, which exists only because
the Metal Shading Language cannot compile the CUDA sources.

Scope: everything `_USE_CUDA` enables (densify, confidence recalibration, mesh refinement), Linux with ROCm 7
or newer. Out of scope: SiftGPU's CUDA backend and the Ceres CUDA solvers, which are NVIDIA libraries (a HIP
build does not ask vcpkg for them): SiftGPU then runs its GLSL backend, its CUDA extraction and the GPU
matching branch being gated on SiftGPU's own `SIFTGPU_CUDA`, and the Ceres CUDA solver selection
(`GlobalPositioning.cpp`) is compiled out.

## 2. Design as implemented

### One source, a typed shim instead of the CUDA headers

`libs/Common/UtilCUDARuntime.h` is the only place the GPU runtime is included (`UtilCUDA.h`,
`UtilCUDADevice.h`, `MVS/CUDA/Maths.h`, `ConfidenceCUDA.cu` include it instead of the CUDA headers). A CUDA
build gets the CUDA toolkit headers from it, unchanged. A HIP build gets `<hip/hip_runtime.h>` and
`<hiprand/hiprand_kernel.h>` plus the CUDA names the code uses, declared as typed aliases of their HIP
counterparts:

- types as `using` (`cudaStream_t = hipStream_t`, `CUresult = hipError_t`, `curandState = hiprandState`, ...);
- enumerators as `constexpr` constants of the HIP enum type (`cudaSuccess`, `cudaAddressModeClamp`,
  `CU_AD_FORMAT_FLOAT`, ...);
- functions as `inline` forwarders with the CUDA signature (`cudaMalloc`, `cuMemcpy2D`, ...), so every call
  is type-checked against the HIP function it lands on; `curand_init`/`curand_uniform` are `__device__`
  forwarders, declared only when hipcc compiles.

`CUdeviceptr` stays CUDA's integer device address (`unsigned long long`; the refine host code offsets it in
bytes), not HIP's `void*` `hipDeviceptr_t`: the driver forwarders convert. `CUDA_SUCCESS` is declared only
when the HIP runtime header has not defined it itself (it does when hipcc compiles).

No macro maps a CUDA name: nothing rewrites the tokens of the translation units that include the header,
and no reserved name (`__expf`, ...) is redefined. A host translation unit sees the aliases as ordinary
declarations, as a CUDA build sees the CUDA runtime's (`UtilCUDA.h` is included by `Common/Types.h`), and no
CUDA-named macro beyond OpenMVS's own `CUDA_CHECK`. ROCm provides the device intrinsics the kernels use under
their CUDA names: `__expf` (`__builtin_amdgcn_exp2f`), `__fdividef` (a plain division), `__ldg` (`hip_ldg.h`,
float/float2/float4), `__syncthreads_or`, `tex2D`, `surf2Dread/Write` (byte x coordinate, as CUDA), the `_rn`
arithmetic, 64-bit `atomicMin`.

hipcc does not define `__CUDACC__`/`__CUDA_ARCH__`; every compiler test has its HIP twin next to it:
`__HIP__` (both passes of a hipcc compilation) beside `__CUDACC__` and `__HIP_DEVICE_COMPILE__` (the device
pass) beside `__CUDA_ARCH__` (`Config.h` `HOST_DEVICE`/`FORCEINLINE`/device `ASSERT`, `MVS/CUDA/Maths.h`,
`ConfidenceRefine.h`, `SceneRefineCommon.h`). `_USE_HIP` reaches every translation unit, the installed
headers included, through `ConfigLocal.h`, like `_USE_CUDA`, which a HIP build also sets.

### The spots that differ, each written once

| Where | CUDA | HIP | Why |
|---|---|---|---|
| `cudaGraphInstantiate` forwarder | `(exec, graph, flags)` | `hipGraphInstantiateWithFlags` | `hipGraphInstantiate` takes an error-log buffer instead of the flags |
| `FULL_WARP_MASK` (`SceneRefineCUDA.cu`) | 32-bit `0xffffffffu` | 64-bit `0xffffffffull` | ROCm 7 `*_sync` intrinsics static-assert a 64-bit mask |
| device check (`UtilCUDA.cpp` `_initDevice`) | compute capability vs. the compiled SASS/PTX | `gcnArchName` processor vs. `CMAKE_HIP_ARCHITECTURES` | a GPU without code is reported and the CPU used, on both |
| refine init (`MeshRefineCUDA::InitKernels`) | — | device must run 32-lane wavefronts | the ballot packs a warp into one 32-bit word, the block scan works on 32-lane warps; a wave64 device (`Device::warpSize`) falls back to the CPU refinement, and `MeshRefineSyntheticTest` skips its GPU run there |

Everything else in the kernels is the same code on both. The pinhole textures of `PatchMatchCUDA`
(`AllocateImageCUDA`) clamp explicitly (`cudaAddressModeClamp` on both axes, unnormalized coordinates):
CUDA clamps unnormalized coordinates whatever their address mode, so the NVIDIA result is the one it was
with the former wrap mode, and HIP, which need not ignore the mode, samples the same border texels. The
equirectangular views keep normalized coordinates, wrapping the columns and clamping the rows.

### Build

`OpenMVS_USE_HIP` (`CMakeLists.txt`) turns `OpenMVS_USE_CUDA` off before the vcpkg feature list (no CUDA
toolkit, SiftGPU-CUDA or Ceres-CUDA is requested), enables the `HIP` language, finds `hip` and `hiprand`
(linked as `hip::host` and `hip::hiprand`: `hip::device` would put `-x hip` on every host `.cpp`), and sets
`_USE_HIP` and `_USE_CUDA`. `libs/MVS/CMakeLists.txt` compiles the `.cu` files as `LANGUAGE HIP`. The targets
are `CMAKE_HIP_ARCHITECTURES` (ex. `gfx1100;gfx1201`), no default is hard-coded (left unset, CMake takes the
GPUs of the build machine); the configure stops on an empty list or an entry that is not a gfx target ID
(`native`, `all`, generic targets), as the device check could match no GPU against it. The list reaches
`UtilCUDA.cpp` as `_HIP_ARCHS`. A Debug build compiles the device code at `-O1` (`-Xarch_device -O1`
appended to `CMAKE_HIP_FLAGS_DEBUG`) and the host code as configured: at `-O0`, the HIP Debug default
since CMake 4, ROCm 7.2 keeps the gfx8/9 branch of the device library's block reduction behind
`__syncthreads_or` (`SceneRefineCUDA.cu`), whose wavefront-shift DPP no gfx10+ target can encode, and the
compile fails; ROCm 7.13 compiles `-O0`. With the option off the CUDA configure is the one it was: the generated
`ConfigLocal.h` differs by the `/* #undef _USE_HIP */` line only.

## 3. Validation

| Check | Result |
|---|---|
| CUDA depth-maps, this code vs. the kernel without the shim (Windows, RelWithDebInfo) | byte-identical: ETH3D pipes (level 1, max 3072) 14/14 `.dmap` and the fused cloud, Holo360D Outdoor_004 (spherical, 40 panoramas) 40/40 `.dmap` and the fused cloud |
| CUDA configure with `OpenMVS_USE_HIP` off | same cache, flags and projects; `ConfigLocal.h` gains `/* #undef _USE_HIP */` |
| Debug `Tests 0/1/2` (CUDA, `_HEADLESS_DEBUG`) | exit 0, no `[ASSERT]` |
| HIP build, Linux, ROCm 7.13 (user-space wheels), gfx1100;gfx1201, Release | every library and app compiles and links; the GPU sources warn only on ignored `[[nodiscard]]` `hipError_t` returns |
| HIP Debug build (host `-g -O0`), ROCm 7.2 (clang 22) and 7.13 (clang 23), gfx1100;gfx1201 | the three `.cu` files compile; without the device `-O1`, ROCm 7.2 rejects `SceneRefineCUDA.cu` |
| HIP build without an AMD GPU | `DensifyPointCloud` reports no device and densifies on the CPU |
| Radeon RX 9070 (gfx1201, wave32), ROCm 7.2, Release and Debug | `Tests 0/1/2` exit 0, no `[ASSERT]`; the GPU refinement ends at 1.0001x the CPU's RMS; `apps/Tests/data` bit-identical over two runs; SfM with SiftGPU (GLSL) and CPU matching runs |
| ETH3D office, the ETH3D undistorted COLMAP model (level 1, max 3072), RX 9070 vs. CUDA | 5 017 279 vs. 5 015 629 points (+0.03 %), 42 820 392 vs. 42 771 224 depths fused; CUDA scores F@2cm 0.5881, F@5cm 0.6851; the RX 9070 densifies in 100 s, its CPU run in 338 s |

## 4. Rejected alternatives

- A macro translator (`#define cudaX hipX`): reaches every translation unit through `Common/Types.h` and
  redefines reserved names.
- A HIP hand port of the kernels, Metal style: a third copy of ~2600 kernel lines on hardware we do not own.
- A shared-memory (Hillis-Steele) block scan replacing the shuffle scan: the shuffle scan is correct on 32-lane
  wavefronts (ROCm lowers `__shfl_up` to `ds_bpermute` at every optimization level), the other is slower.
- Replacing `__syncthreads_or` to compile the device code at `-O0`: the defect is in ROCm 7.2 (7.13 compiles
  it), and the device code keeps its `ASSERT`s at `-O1`.
- `hip::device` linked like `hip::host`: its compile options make every host `.cpp` a HIP source.
- Rejecting a PatchMatch patch with a sample outside the image instead of clamping the texture: changes the
  NVIDIA result (ETH3D F -0.0001..-0.0013, kernel +40 %).

## 5. Open items

- The F-score of the RX 9070 office cloud (its point count matches CUDA's).
- A wave64 GPU (Instinct, GCN) is not run: the densify path and the refinement's CPU fallback there.
- A user-space ROCm (Python wheels) puts its libraries outside the loader path: `LD_LIBRARY_PATH` must name
  `<root>/lib` to run the binaries.
