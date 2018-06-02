/*
* SceneRefineCUDA.cpp
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

#include "Common.h"
#include "Scene.h"

using namespace MVS;

#ifdef _USE_CUDA

// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define MESHCUDAOPT_USE_OPENMP
#endif

// uncomment to ensure edge size and improve vertex valence
// (should enable more stable flow)
#define MESHOPT_ENSUREEDGESIZE 1 // 0 - at all resolution


// S T R U C T S ///////////////////////////////////////////////////

static LPCSTR const g_szMeshRefineModule =
	".version 3.2\n"
	".target sm_20\n"
	".address_size 64\n"
	"\n"
	".global .texref texImageRef;\n"
	".global .surfref surfImageRef;\n"
	".global .surfref surfImageProjRef;\n"
	"\n"
	// kernel used to project the given mesh to a given camera plane:
	// the depth-map is computed by rasterizing all triangles (using a brute force scan-line approach)
	// and storing only the closest ones;
	// additionally the face index and barycentric coordinates are stored for each pixel
	".visible .entry ProjectMesh(\n"
	"	.param .u64 .ptr param_1, // array vertices (float*3 * numVertices)\n"
	"	.param .u64 .ptr param_2, // array faces (uint32_t*3 * numFaces)\n"
	"	.param .u64 .ptr param_3, // array face IDs (uint32_t * numFacesView)\n"
	"	.param .u64 .ptr param_4, // depth-map (float) [out]\n"
	"	.param .u64 .ptr param_5, // face-map (uint32_t) [out]\n"
	"	.param .u64 .ptr param_6, // bary-map (hfloat*3) [out]\n"
	"	.param .align 4 .b8 param_7[176], // camera\n"
	"	.param .u32 param_8 // numFacesView (uint32_t)\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<234>;\n"
	"	.reg .pred %p<20>;\n"
	"	.reg .s16 %rs<5>;\n"
	"	.reg .s32 %r<105>;\n"
	"	.reg .s64 %rl<42>;\n"
	"\n"
	"	ld.param.u32 %r16, [param_8];\n"
	"	mov.u32 %r17, %ntid.x;\n"
	"	mov.u32 %r18, %ctaid.x;\n"
	"	mov.u32 %r19, %tid.x;\n"
	"	mad.lo.s32 %r5, %r17, %r18, %r19;\n"
	"	setp.ge.s32 %p3, %r5, %r16;\n"
	"	@%p3 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl4, [param_3];\n"
	"	cvta.to.global.u64 %rl3, %rl4;\n"
	"	mul.wide.u32 %rl5, %r5, 4;\n"
	"	add.u64 %rl6, %rl3, %rl5;\n"
	"	ld.global.u32 %r6, [%rl6];\n"
	"	mul.lo.s32 %r20, %r6, 3;\n"
	"	ld.param.u64 %rl36, [param_2];\n"
	"	cvta.to.global.u64 %rl10, %rl36;\n"
	"	mul.wide.s32 %rl11, %r20, 4;\n"
	"	add.s64 %rl12, %rl10, %rl11;\n"
	"	mad.lo.s32 %r21, %r6, 3, 1;\n"
	"	mul.wide.s32 %rl13, %r21, 4;\n"
	"	add.s64 %rl14, %rl10, %rl13;\n"
	"	ld.global.u32 %r22, [%rl12];\n"
	"	mul.lo.s32 %r24, %r22, 3;\n"
	"	ld.param.u64 %rl35, [param_1];\n"
	"	cvta.to.global.u64 %rl15, %rl35;\n"
	"	mul.wide.s32 %rl16, %r24, 4;\n"
	"	add.s64 %rl17, %rl15, %rl16;\n"
	"	ld.global.u32 %r25, [%rl14];\n"
	"	mul.lo.s32 %r27, %r25, 3;\n"
	"	mul.wide.s32 %rl18, %r27, 4;\n"
	"	add.s64 %rl19, %rl15, %rl18;\n"
	"	ld.global.f32 %f33, [%rl19];\n"
	"	ld.global.f32 %f34, [%rl19+4];\n"
	"	ld.global.f32 %f35, [%rl19+8];\n"
	"	ld.global.u32 %r31, [%rl14+4];\n"
	"	mul.lo.s32 %r33, %r31, 3;\n"
	"	mul.wide.s32 %rl20, %r33, 4;\n"
	"	add.s64 %rl21, %rl15, %rl20;\n"
	"	ld.global.f32 %f36, [%rl21];\n"
	"	ld.global.f32 %f37, [%rl21+4];\n"
	"	ld.global.f32 %f38, [%rl21+8];\n"
	"	ld.global.f32 %f39, [%rl17];\n"
	"	ld.global.f32 %f40, [%rl17+4];\n"
	"	ld.param.f32 %f217, [param_7+4];\n"
	"	mul.f32 %f95, %f217, %f40;\n"
	"	ld.param.f32 %f220, [param_7];\n"
	"	fma.rn.f32 %f96, %f220, %f39, %f95;\n"
	"	ld.global.f32 %f41, [%rl17+8];\n"
	"	ld.param.f32 %f214, [param_7+8];\n"
	"	fma.rn.f32 %f97, %f214, %f41, %f96;\n"
	"	ld.param.f32 %f211, [param_7+12];\n"
	"	add.f32 %f42, %f97, %f211;\n"
	"	ld.param.f32 %f205, [param_7+20];\n"
	"	mul.f32 %f98, %f205, %f40;\n"
	"	ld.param.f32 %f208, [param_7+16];\n"
	"	fma.rn.f32 %f99, %f208, %f39, %f98;\n"
	"	ld.param.f32 %f202, [param_7+24];\n"
	"	fma.rn.f32 %f100, %f202, %f41, %f99;\n"
	"	ld.param.f32 %f197, [param_7+28];\n"
	"	add.f32 %f43, %f100, %f197;\n"
	"	ld.param.f32 %f191, [param_7+36];\n"
	"	mul.f32 %f101, %f191, %f40;\n"
	"	ld.param.f32 %f194, [param_7+32];\n"
	"	fma.rn.f32 %f102, %f194, %f39, %f101;\n"
	"	ld.param.f32 %f188, [param_7+40];\n"
	"	fma.rn.f32 %f103, %f188, %f41, %f102;\n"
	"	ld.param.f32 %f185, [param_7+44];\n"
	"	add.f32 %f44, %f103, %f185;\n"
	"	setp.gt.f32 %p4, %f44, 0f00000000;\n"
	"	@%p4 bra BB00_8;\n"
	"\n"
	"	mov.f32 %f222, 0fBF800000;\n"
	"	mov.f32 %f221, %f222;\n"
	"	bra.uni BB00_9;\n"
	"\n"
	"	BB00_8:\n"
	"	div.rn.f32 %f221, %f42, %f44;\n"
	"	div.rn.f32 %f222, %f43, %f44;\n"
	"\n"
	"	BB00_9:\n"
	"	ld.param.f32 %f216, [param_7+4];\n"
	"	mul.f32 %f106, %f216, %f34;\n"
	"	ld.param.f32 %f219, [param_7];\n"
	"	fma.rn.f32 %f107, %f219, %f33, %f106;\n"
	"	ld.param.f32 %f213, [param_7+8];\n"
	"	fma.rn.f32 %f108, %f213, %f35, %f107;\n"
	"	ld.param.f32 %f210, [param_7+12];\n"
	"	add.f32 %f49, %f108, %f210;\n"
	"	ld.param.f32 %f204, [param_7+20];\n"
	"	mul.f32 %f109, %f204, %f34;\n"
	"	ld.param.f32 %f207, [param_7+16];\n"
	"	fma.rn.f32 %f110, %f207, %f33, %f109;\n"
	"	ld.param.f32 %f201, [param_7+24];\n"
	"	fma.rn.f32 %f111, %f201, %f35, %f110;\n"
	"	ld.param.f32 %f199, [param_7+28];\n"
	"	add.f32 %f50, %f111, %f199;\n"
	"	ld.param.f32 %f193, [param_7+36];\n"
	"	mul.f32 %f112, %f193, %f34;\n"
	"	ld.param.f32 %f196, [param_7+32];\n"
	"	fma.rn.f32 %f113, %f196, %f33, %f112;\n"
	"	ld.param.f32 %f190, [param_7+40];\n"
	"	fma.rn.f32 %f114, %f190, %f35, %f113;\n"
	"	ld.param.f32 %f187, [param_7+44];\n"
	"	add.f32 %f51, %f114, %f187;\n"
	"	setp.gt.f32 %p5, %f51, 0f00000000;\n"
	"	@%p5 bra BB00_10;\n"
	"\n"
	"	mov.f32 %f224, 0fBF800000;\n"
	"	mov.f32 %f223, %f224;\n"
	"	bra.uni BB00_11;\n"
	"\n"
	"	BB00_10:\n"
	"	div.rn.f32 %f223, %f49, %f51;\n"
	"	div.rn.f32 %f224, %f50, %f51;\n"
	"\n"
	"	BB00_11:\n"
	"	ld.param.f32 %f215, [param_7+4];\n"
	"	mul.f32 %f117, %f215, %f37;\n"
	"	ld.param.f32 %f218, [param_7];\n"
	"	fma.rn.f32 %f118, %f218, %f36, %f117;\n"
	"	ld.param.f32 %f212, [param_7+8];\n"
	"	fma.rn.f32 %f119, %f212, %f38, %f118;\n"
	"	ld.param.f32 %f209, [param_7+12];\n"
	"	add.f32 %f56, %f119, %f209;\n"
	"	ld.param.f32 %f203, [param_7+20];\n"
	"	mul.f32 %f120, %f203, %f37;\n"
	"	ld.param.f32 %f206, [param_7+16];\n"
	"	fma.rn.f32 %f121, %f206, %f36, %f120;\n"
	"	ld.param.f32 %f200, [param_7+24];\n"
	"	fma.rn.f32 %f122, %f200, %f38, %f121;\n"
	"	ld.param.f32 %f198, [param_7+28];\n"
	"	add.f32 %f57, %f122, %f198;\n"
	"	ld.param.f32 %f192, [param_7+36];\n"
	"	mul.f32 %f123, %f192, %f37;\n"
	"	ld.param.f32 %f195, [param_7+32];\n"
	"	fma.rn.f32 %f124, %f195, %f36, %f123;\n"
	"	ld.param.f32 %f189, [param_7+40];\n"
	"	fma.rn.f32 %f125, %f189, %f38, %f124;\n"
	"	ld.param.f32 %f186, [param_7+44];\n"
	"	add.f32 %f58, %f125, %f186;\n"
	"	setp.gt.f32 %p6, %f58, 0f00000000;\n"
	"	@%p6 bra BB00_12;\n"
	"\n"
	"	mov.f32 %f226, 0fBF800000;\n"
	"	mov.f32 %f225, %f226;\n"
	"	bra.uni BB00_13;\n"
	"\n"
	"	BB00_12:\n"
	"	div.rn.f32 %f225, %f56, %f58;\n"
	"	div.rn.f32 %f226, %f57, %f58;\n"
	"\n"
	"	BB00_13:\n"
	"	add.f32 %f2, %f221, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r40, %f2;\n"
	"	add.f32 %f4, %f223, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r41, %f4;\n"
	"	min.s32 %r42, %r40, %r41;\n"
	"	add.f32 %f6, %f225, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r43, %f6;\n"
	"	min.s32 %r44, %r42, %r43;\n"
	"	add.s32 %r103, %r44, -1;\n"
	"	add.f32 %f7, %f221, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r45, %f7;\n"
	"	add.f32 %f8, %f223, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r46, %f8;\n"
	"	max.s32 %r47, %r45, %r46;\n"
	"	add.f32 %f9, %f225, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r48, %f9;\n"
	"	max.s32 %r49, %r47, %r48;\n"
	"	add.s32 %r7, %r49, 1;\n"
	"	add.f32 %f11, %f222, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r50, %f11;\n"
	"	add.f32 %f13, %f224, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r51, %f13;\n"
	"	min.s32 %r52, %r50, %r51;\n"
	"	add.f32 %f15, %f226, 0fBF000000;\n"
	"	cvt.rzi.s32.f32 %r53, %f15;\n"
	"	min.s32 %r54, %r52, %r53;\n"
	"	add.s32 %r8, %r54, -1;\n"
	"	add.f32 %f16, %f222, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r55, %f16;\n"
	"	add.f32 %f17, %f224, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r56, %f17;\n"
	"	max.s32 %r57, %r55, %r56;\n"
	"	add.f32 %f18, %f226, 0f3F000000;\n"
	"	cvt.rzi.s32.f32 %r58, %f18;\n"
	"	max.s32 %r59, %r57, %r58;\n"
	"	add.s32 %r9, %r59, 1;\n"
	"	mov.s32 %r60, 10;\n"
	"	setp.lt.s32 %p7, %r103, %r60;\n"
	"	@%p7 bra BB00_1;\n"
	"\n"
	"	mov.s32 %r61, 10;\n"
	"	setp.lt.s32 %p8, %r8, %r61;\n"
	"	@%p8 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r101, [param_7+168];\n"
	"	add.s32 %r63, %r101, -10;\n"
	"	setp.gt.s32 %p9, %r7, %r63;\n"
	"	@%p9 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r102, [param_7+172];\n"
	"	add.s32 %r65, %r102, -10;\n"
	"	setp.gt.s32 %p10, %r9, %r65;\n"
	"	@%p10 bra BB00_1;\n"
	"\n"
	"	sub.f32 %f128, %f224, %f222;\n"
	"	sub.f32 %f129, %f225, %f221;\n"
	"	mul.f32 %f130, %f128, %f129;\n"
	"	sub.f32 %f131, %f226, %f222;\n"
	"	sub.f32 %f132, %f223, %f221;\n"
	"	neg.f32 %f133, %f132;\n"
	"	fma.rn.f32 %f134, %f133, %f131, %f130;\n"
	"	rcp.rn.f32 %f63, %f134;\n"
	"	sub.f32 %f153, %f221, %f223;\n"
	"	mul.f32 %f67, %f63, %f153;\n"
	"	mul.f32 %f68, %f63, %f128;\n"
	"	mul.f32 %f69, %f63, %f129;\n"
	"	sub.f32 %f154, %f222, %f226;\n"
	"	mul.f32 %f70, %f63, %f154;\n"
	"	cvt.rn.f32.s32 %f155, %r103;\n"
	"	sub.f32 %f156, %f155, %f225;\n"
	"	mul.f32 %f157, %f154, %f156;\n"
	"	cvt.rn.f32.s32 %f158, %r8;\n"
	"	sub.f32 %f159, %f158, %f226;\n"
	"	sub.f32 %f160, %f221, %f225;\n"
	"	neg.f32 %f161, %f160;\n"
	"	fma.rn.f32 %f162, %f161, %f159, %f157;\n"
	"	mul.f32 %f232, %f63, %f162;\n"
	"	sub.f32 %f163, %f155, %f221;\n"
	"	mul.f32 %f164, %f128, %f163;\n"
	"	sub.f32 %f165, %f158, %f222;\n"
	"	fma.rn.f32 %f166, %f133, %f165, %f164;\n"
	"	mul.f32 %f229, %f63, %f166;\n"
	"	setp.gt.s32 %p11, %r103, %r7;\n"
	"	@%p11 bra BB00_1;\n"
	"\n"
	"	setp.gt.s32 %p1, %r8, %r9;\n"
	"	setp.gt.f32 %p2, %f63, 0f00000000;\n"
	"	ld.param.u64 %rl37, [param_4];\n"
	"	cvta.to.global.u64 %rl22, %rl37;\n"
	"	ld.param.u64 %rl39, [param_5];\n"
	"	cvta.to.global.u64 %rl28, %rl39;\n"
	"	ld.param.u64 %rl40, [param_6];\n"
	"	cvta.to.global.u64 %rl31, %rl40;\n"
	"\n"
	"	BB00_2:\n"
	"	mov.f32 %f230, %f232;\n"
	"	mov.f32 %f74, %f230;\n"
	"	mov.f32 %f227, %f229;\n"
	"	mov.f32 %f73, %f227;\n"
	"	@%p1 bra BB00_6;\n"
	"\n"
	"	mov.u32 %r11, %r8;\n"
	"	mov.f32 %f75, %f73;\n"
	"	mov.f32 %f76, %f74;\n"
	"\n"
	"	BB00_7:\n"
	"	setp.ge.f32 %p12, %f75, 0f00000000;\n"
	"	setp.ge.f32 %p13, %f76, 0f00000000;\n"
	"	and.pred %p14, %p13, %p12;\n"
	"	@!%p14 bra BB00_5;\n"
	"\n"
	"	add.f32 %f167, %f76, %f75;\n"
	"	setp.gtu.f32 %p15, %f167, 0f3F800000;\n"
	"	@%p15 bra BB00_5;\n"
	"\n"
	"	sub.f32 %f77, 0f3F800000, %f76;\n"
	"	sub.f32 %f77, %f77, %f75;\n"
	"	mul.f32 %f171, %f76, %f51;\n"
	"	fma.rn.f32 %f172, %f77, %f44, %f171;\n"
	"	fma.rn.f32 %f78, %f75, %f58, %f172;\n"
	"	ld.param.u32 %r100, [param_7+168];\n"
	"	mad.lo.s32 %r67, %r11, %r100, %r103;\n"
	"	mul.wide.s32 %rl23, %r67, 4;\n"
	"	mov.b32 %r13, %f78;\n"
	"	add.s64 %rl24, %rl22, %rl23;\n"
	"	ld.global.f32 %f233, [%rl24];\n"
	"\n"
	"	BB00_3:\n"
	"	setp.lt.f32 %p16, %f233, %f78;\n"
	"	@%p16 bra BB00_5;\n"
	"\n"
	"	mov.b32 %r72, %f233;\n"
	"	add.s64 %rl27, %rl22, %rl23;\n"
	"	atom.global.cas.b32 %r73, [%rl27], %r72, %r13;\n"
	"	ld.global.f32 %f80, [%rl27];\n"
	"	setp.neu.f32 %p17, %f233, %f80;\n"
	"	mov.f32 %f233, %f80;\n"
	"	@%p17 bra BB00_3;\n"
	"\n"
	"	add.s64 %rl7, %rl28, %rl23;\n"
	"	mul.wide.s32 %rl30, %r67, 6;\n"
	"	add.s64 %rl8, %rl31, %rl30;\n"
	"	@%p2 bra BB00_4;\n"
	"\n"
	"	mov.s32 %r77, -1;\n"
	"	st.global.u32 [%rl7], %r77;\n"
	"	mov.u16 %rs4, 0;\n"
	"	st.global.b16 [%rl8], %rs4;\n"
	"	st.global.b16 [%rl8+2], %rs4;\n"
	"	st.global.b16 [%rl8+4], %rs4;\n"
	"	bra.uni BB00_5;\n"
	"\n"
	"	BB00_4:\n"
	"	st.global.u32 [%rl7], %r6;\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	cvt.rn.f16.f32 %temp, %f77;\n"
	"	mov.b16 %rs1, %temp;\n"
	"	}\n"
	"	st.global.b16 [%rl8], %rs1;\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	cvt.rn.f16.f32 %temp, %f76;\n"
	"	mov.b16 %rs2, %temp;\n"
	"	}\n"
	"	st.global.b16 [%rl8+2], %rs2;\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	cvt.rn.f16.f32 %temp, %f75;\n"
	"	mov.b16 %rs3, %temp;\n"
	"	}\n"
	"	st.global.b16 [%rl8+4], %rs3;\n"
	"\n"
	"	BB00_5:\n"
	"	add.f32 %f76, %f76, %f69;\n"
	"	add.f32 %f75, %f75, %f67;\n"
	"	add.s32 %r11, %r11, 1;\n"
	"	setp.le.s32 %p18, %r11, %r9;\n"
	"	@%p18 bra BB00_7;\n"
	"\n"
	"	BB00_6:\n"
	"	add.f32 %f83, %f74, %f70;\n"
	"	add.f32 %f84, %f73, %f68;\n"
	"	add.s32 %r103, %r103, 1;\n"
	"	setp.le.s32 %p19, %r103, %r7;\n"
	"	mov.f32 %f229, %f84;\n"
	"	mov.f32 %f232, %f83;\n"
	"	@%p19 bra BB00_2;\n"
	"\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to invalidate pixels that don't have valid both depth and face index
	".visible .entry CrossCheckProjection(\n"
	"	.param .u64 .ptr param_1, // depth-map (float) [in/out]\n"
	"	.param .u64 .ptr param_2, // face-map (uint32_t) [in/out]\n"
	"	.param .u32 param_3, // width\n"
	"	.param .u32 param_4  // height\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<2>;\n"
	"	.reg .pred %p<10>;\n"
	"	.reg .s32 %r<14>;\n"
	"	.reg .s64 %rl<8>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_3];\n"
	"	ld.param.u32 %r2, [param_4];\n"
	"	mov.u32 %r6, %ntid.x;\n"
	"	mov.u32 %r7, %ctaid.x;\n"
	"	mov.u32 %r8, %tid.x;\n"
	"	mad.lo.s32 %r3, %r6, %r7, %r8;\n"
	"	mov.u32 %r9, %ntid.y;\n"
	"	mov.u32 %r10, %ctaid.y;\n"
	"	mov.u32 %r11, %tid.y;\n"
	"	mad.lo.s32 %r4, %r9, %r10, %r11;\n"
	"	setp.gt.s32 %p1, %r3, -1;\n"
	"	setp.lt.s32 %p2, %r3, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r4, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r4, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	mad.lo.s32 %r12, %r4, %r1, %r3;\n"
	"	mul.wide.s32 %rl7, %r12, 4;\n"
	"\n"
	"	ld.param.u64 %rl2, [param_1];\n"
	"	cvta.to.global.u64 %rl1, %rl2;\n"
	"	add.s64 %rl3, %rl1, %rl7;\n"
	"\n"
	"	ld.param.u64 %rl5, [param_2];\n"
	"	cvta.to.global.u64 %rl4, %rl5;\n"
	"	add.s64 %rl6, %rl4, %rl7;\n"
	"\n"
	"	ld.global.f32 %f1, [%rl3];\n"
	"	setp.eq.f32 %p8, %f1, 0f7F7FFFFF;\n"
	"	@%p8 bra BB00_2;\n"
	"\n"
	"	ld.global.s32 %r13, [%rl6];\n"
	"	setp.eq.s32 %p9, %r13, -1;\n"
	"	@%p9 bra BB00_2;\n"
	"\n"
	"	ret;\n"
	"\n"
	"	BB00_2:\n"
	"	st.global.f32 [%rl3], 0f00000000;\n"
	"	st.global.s32 [%rl6], -1;\n"
	"\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to project image from view B to view A through the mesh;
	// additionally the mask is computed
	".visible .entry ImageMeshWarp(\n"
	"	.param .u64 .ptr param_1, // depth-map A (float)\n"
	"	.param .u64 .ptr param_2, // depth-map B (float)\n"
	"	.param .u64 .ptr param_3, // mask [out]\n"
	"	.param .align 4 .b8 param_4[176], // camera A \n"
	"	.param .align 4 .b8 param_5[176] // camera B \n"
	")\n"
	"{\n"
	"	.reg .f32 %f<187>;\n"
	"	.reg .pred %p<19>;\n"
	"	.reg .s16 %rs<2>;\n"
	"	.reg .s32 %r<36>;\n"
	"	.reg .s64 %rl<23>;\n"
	"	.reg .s16 %rc<2>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_4+168];\n"
	"	ld.param.u32 %r2, [param_4+172];\n"
	"	mov.u32 %r6, %ntid.x;\n"
	"	mov.u32 %r7, %ctaid.x;\n"
	"	mov.u32 %r8, %tid.x;\n"
	"	mad.lo.s32 %r3, %r6, %r7, %r8;\n"
	"	mov.u32 %r9, %ntid.y;\n"
	"	mov.u32 %r10, %ctaid.y;\n"
	"	mov.u32 %r11, %tid.y;\n"
	"	mad.lo.s32 %r4, %r9, %r10, %r11;\n"
	"	setp.gt.s32 %p1, %r3, -1;\n"
	"	setp.lt.s32 %p2, %r3, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r4, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r4, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_0;\n"
	"\n"
	"	mov.f32 %f141, 0f00000000;\n"
	"	mov.u16 %rc1, 0;\n"
	"	shl.b32 %r13, %r3, 1;\n"
	"	ld.param.u64 %rl18, [param_1];\n"
	"	cvta.to.global.u64 %rl10, %rl18;\n"
	"	mad.lo.s32 %r12, %r4, %r1, %r3;\n"
	"	mul.wide.s32 %rl9, %r12, 4;\n"
	"	add.s64 %rl11, %rl10, %rl9;\n"
	"	ld.global.f32 %f51, [%rl11];\n"
	"	setp.gt.f32 %p8, %f51, 0f00000000;\n"
	"	@!%p8 bra BB00_1;\n"
	"\n"
	"	cvt.rn.f32.s32 %f88, %r3;\n"
	"	ld.param.f32 %f147, [param_4+104];\n"
	"	sub.f32 %f89, %f88, %f147;\n"
	"	ld.param.f32 %f148, [param_4+96];\n"
	"	div.rn.f32 %f90, %f89, %f148;\n"
	"	cvt.rn.f32.s32 %f91, %r4;\n"
	"	ld.param.f32 %f145, [param_4+116];\n"
	"	sub.f32 %f92, %f91, %f145;\n"
	"	ld.param.f32 %f146, [param_4+112];\n"
	"	div.rn.f32 %f93, %f92, %f146;\n"
	"	ld.param.f32 %f157, [param_4+60];\n"
	"	mul.f32 %f94, %f157, %f93;\n"
	"	ld.param.f32 %f160, [param_4+48];\n"
	"	fma.rn.f32 %f95, %f160, %f90, %f94;\n"
	"	ld.param.f32 %f154, [param_4+72];\n"
	"	add.f32 %f96, %f95, %f154;\n"
	"	ld.param.f32 %f156, [param_4+64];\n"
	"	mul.f32 %f97, %f156, %f93;\n"
	"	ld.param.f32 %f159, [param_4+52];\n"
	"	fma.rn.f32 %f98, %f159, %f90, %f97;\n"
	"	ld.param.f32 %f153, [param_4+76];\n"
	"	add.f32 %f99, %f98, %f153;\n"
	"	ld.param.f32 %f155, [param_4+68];\n"
	"	mul.f32 %f100, %f155, %f93;\n"
	"	ld.param.f32 %f158, [param_4+56];\n"
	"	fma.rn.f32 %f101, %f158, %f90, %f100;\n"
	"	ld.param.f32 %f152, [param_4+80];\n"
	"	add.f32 %f102, %f101, %f152;\n"
	"	ld.param.f32 %f72, [param_4+84];\n"
	"	fma.rn.f32 %f55, %f51, %f96, %f72;\n"
	"	ld.param.f32 %f75, [param_4+88];\n"
	"	fma.rn.f32 %f56, %f51, %f99, %f75;\n"
	"	ld.param.f32 %f78, [param_4+92];\n"
	"	fma.rn.f32 %f57, %f51, %f102, %f78;\n"
	"	ld.param.f32 %f183, [param_5+4];\n"
	"	mul.f32 %f110, %f183, %f56;\n"
	"	ld.param.f32 %f184, [param_5];\n"
	"	fma.rn.f32 %f111, %f184, %f55, %f110;\n"
	"	ld.param.f32 %f182, [param_5+8];\n"
	"	fma.rn.f32 %f112, %f182, %f57, %f111;\n"
	"	ld.param.f32 %f181, [param_5+12];\n"
	"	add.f32 %f58, %f112, %f181;\n"
	"	ld.param.f32 %f179, [param_5+20];\n"
	"	mul.f32 %f113, %f179, %f56;\n"
	"	ld.param.f32 %f180, [param_5+16];\n"
	"	fma.rn.f32 %f114, %f180, %f55, %f113;\n"
	"	ld.param.f32 %f178, [param_5+24];\n"
	"	fma.rn.f32 %f115, %f178, %f57, %f114;\n"
	"	ld.param.f32 %f177, [param_5+28];\n"
	"	add.f32 %f59, %f115, %f177;\n"
	"	ld.param.f32 %f175, [param_5+36];\n"
	"	mul.f32 %f116, %f175, %f56;\n"
	"	ld.param.f32 %f176, [param_5+32];\n"
	"	fma.rn.f32 %f117, %f176, %f55, %f116;\n"
	"	ld.param.f32 %f174, [param_5+40];\n"
	"	fma.rn.f32 %f118, %f174, %f57, %f117;\n"
	"	ld.param.f32 %f173, [param_5+44];\n"
	"	add.f32 %f60, %f118, %f173;\n"
	"	setp.gt.f32 %p9, %f60, 0f00000000;\n"
	"	@!%p9 bra BB00_1;\n"
	"\n"
	"	div.rn.f32 %f185, %f58, %f60;\n"
	"	div.rn.f32 %f186, %f59, %f60;\n"
	"	setp.leu.f32 %p10, %f185, 0f41200000;\n"
	"	@%p10 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r35, [param_5+168];\n"
	"	add.s32 %r18, %r35, -10;\n"
	"	cvt.rn.f32.s32 %f127, %r18;\n"
	"	setp.lt.f32 %p11, %f185, %f127;\n"
	"	setp.gt.f32 %p12, %f186, 0f41200000;\n"
	"	and.pred %p13, %p11, %p12;\n"
	"	@!%p13 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r14, [param_5+172];\n"
	"	add.s32 %r19, %r14, -10;\n"
	"	cvt.rn.f32.s32 %f128, %r19;\n"
	"	setp.geu.f32 %p14, %f186, %f128;\n"
	"	@%p14 bra BB00_1;\n"
	"\n"
	"	cvt.rzi.s32.f32 %r20, %f186;\n"
	"	cvt.rzi.s32.f32 %r21, %f185;\n"
	"	mad.lo.s32 %r22, %r20, %r35, %r21;\n"
	"	ld.param.u64 %rl20, [param_2];\n"
	"	cvta.to.global.u64 %rl14, %rl20;\n"
	"	mul.wide.s32 %rl15, %r22, 4;\n"
	"	add.s64 %rl6, %rl14, %rl15;\n"
	"	ld.global.f32 %f129, [%rl6];\n"
	"	sub.f32 %f130, %f129, %f60;\n"
	"	abs.f32 %f131, %f130;\n"
	"	mul.f32 %f1, %f60, 0f3C23D70A;\n"
	"	setp.lt.f32 %p15, %f131, %f1;\n"
	"	@%p15 bra BB00_2;\n"
	"\n"
	"	ld.global.f32 %f135, [%rl6+4];\n"
	"	sub.f32 %f136, %f135, %f60;\n"
	"	abs.f32 %f137, %f136;\n"
	"	setp.lt.f32 %p16, %f137, %f1;\n"
	"	@%p16 bra BB00_2;\n"
	"\n"
	"	add.s32 %r23, %r22, %r35;\n"
	"	mul.wide.s32 %rl17, %r23, 4;\n"
	"	add.s64 %rl7, %rl14, %rl17;\n"
	"	ld.global.f32 %f132, [%rl7];\n"
	"	sub.f32 %f133, %f132, %f60;\n"
	"	abs.f32 %f134, %f133;\n"
	"	setp.lt.f32 %p17, %f134, %f1;\n"
	"	@%p17 bra BB00_2;\n"
	"\n"
	"	ld.global.f32 %f138, [%rl7+4];\n"
	"	sub.f32 %f139, %f138, %f60;\n"
	"	abs.f32 %f140, %f139;\n"
	"	setp.lt.f32 %p18, %f140, %f1;\n"
	"	@%p18 bra BB00_2;\n"
	"\n"
	"	BB00_1:\n"
	"	suld.b.2d.b16.trap {%rs1}, [surfImageRef, {%r13, %r4}];\n"
	"	bra.uni BB00_3;\n"
	"\n"
	"	BB00_2:\n"
	"	tex.2d.v4.f32.f32 {%f141, %f142, %f143, %f144}, [texImageRef, {%f185, %f186}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	cvt.rn.f16.f32 %temp, %f141;\n"
	"	mov.b16 %rs1, %temp;\n"
	"	}\n"
	"	mov.u16 %rc1, 1;\n"
	"\n"
	"	BB00_3:\n"
	"	sust.b.2d.b16.trap [surfImageProjRef, {%r13, %r4}], {%rs1};\n"
	"	ld.param.u64 %rl1, [param_3];\n"
	"	cvta.to.global.u64 %rl4, %rl1;\n"
	"	cvt.s64.s32 %rl3, %r12;\n"
	"	add.s64 %rl2, %rl4, %rl3;\n"
	"	st.global.u8 [%rl2], %rc1;\n"
	"	BB00_0:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the mean for all image pixels for a given windows size
	".visible .entry ComputeImageMean(\n"
	"	.param .u64 .ptr param_1, // image mask\n"
	"	.param .u64 .ptr param_2, // image pixels mean [out]\n"
	"	.param .u32 param_3, // image width\n"
	"	.param .u32 param_4, // image height\n"
	"	.param .u32 param_5 // half-window size\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<10>;\n"
	"	.reg .pred %p<19>;\n"
	"	.reg .s16 %rc<4>;\n"
	"	.reg .s32 %r<40>;\n"
	"	.reg .s64 %rl<13>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_3];\n"
	"	ld.param.u32 %r2, [param_4];\n"
	"	mov.u32 %r12, %ntid.x;\n"
	"	mov.u32 %r13, %ctaid.x;\n"
	"	mov.u32 %r14, %tid.x;\n"
	"	mad.lo.s32 %r4, %r12, %r13, %r14;\n"
	"	mov.u32 %r15, %ntid.y;\n"
	"	mov.u32 %r16, %ctaid.y;\n"
	"	mov.u32 %r17, %tid.y;\n"
	"	mad.lo.s32 %r5, %r15, %r16, %r17;\n"
	"	setp.gt.s32 %p1, %r4, -1;\n"
	"	setp.lt.s32 %p2, %r4, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r5, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r5, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl7, [param_1];\n"
	"	ld.param.u64 %rl8, [param_2];\n"
	"	cvta.to.global.u64 %rl2, %rl7;\n"
	"	cvta.to.global.u64 %rl3, %rl8;\n"
	"	ld.param.u32 %r36, [param_5];\n"
	"	shl.b32 %r18, %r36, 1;\n"
	"	or.b32 %r19, %r18, 1;\n"
	"	cvt.rn.f32.s32 %f6, %r19;\n"
	"	mul.f32 %f1, %f6, %f6;\n"
	"	ld.param.u32 %r31, [param_3];\n"
	"	mad.lo.s32 %r20, %r5, %r31, %r4;\n"
	"	cvt.s64.s32 %rl4, %r20;\n"
	"	mul.wide.s32 %rl9, %r20, 4;\n"
	"	add.s64 %rl5, %rl3, %rl9;\n"
	"	mov.u32 %r21, 0;\n"
	"	st.global.u32 [%rl5], %r21;\n"
	"	sub.s32 %r23, %r31, %r36;\n"
	"	setp.lt.s32 %p8, %r4, %r23;\n"
	"	setp.ge.s32 %p9, %r4, %r36;\n"
	"	and.pred %p10, %p8, %p9;\n"
	"	setp.ge.s32 %p11, %r5, %r36;\n"
	"	and.pred %p12, %p10, %p11;\n"
	"	ld.param.u32 %r32, [param_4];\n"
	"	sub.s32 %r24, %r32, %r36;\n"
	"	setp.lt.s32 %p13, %r5, %r24;\n"
	"	and.pred %p14, %p12, %p13;\n"
	"	@!%p14 bra BB00_1;\n"
	"\n"
	"	add.s64 %rl10, %rl2, %rl4;\n"
	"	ld.global.u8 %rc1, [%rl10];\n"
	"	cvt.s16.s8 %rc1, %rc1;\n"
	"	mov.b16 %rc2, 1;\n"
	"	cvt.s16.s8 %rc2, %rc2;\n"
	"	setp.eq.s16 %p15, %rc1, %rc2;\n"
	"	@!%p15 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r35, [param_5];\n"
	"	neg.s32 %r6, %r35;\n"
	"	setp.gt.s32 %p16, %r6, %r35;\n"
	"	@%p16 bra BB00_5;\n"
	"\n"
	"	mov.f32 %f9, 0f00000000;\n"
	"	mov.u32 %r39, %r6;\n"
	"\n"
	"	BB00_3:\n"
	"	mov.u32 %r7, %r39;\n"
	"	add.s32 %r8, %r7, %r4;\n"
	"	mov.u32 %r38, %r6;\n"
	"\n"
	"	BB00_4:\n"
	"	add.s32 %r26, %r38, %r5;\n"
	"	shl.b32 %r27, %r8, 1;\n"
	"	suld.b.2d.b16.trap {%rc3}, [surfImageRef, {%r27, %r26}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rc3;\n"
	"	cvt.f32.f16 %f7, %temp;\n"
	"	}\n"
	"	add.f32 %f9, %f9, %f7;\n"
	"	add.s32 %r38, %r38, 1;\n"
	"	ld.param.u32 %r34, [param_5];\n"
	"	setp.le.s32 %p17, %r38, %r34;\n"
	"	@%p17 bra BB00_4;\n"
	"\n"
	"	add.s32 %r11, %r7, 1;\n"
	"	ld.param.u32 %r33, [param_5];\n"
	"	setp.le.s32 %p18, %r11, %r33;\n"
	"	mov.u32 %r39, %r11;\n"
	"	@%p18 bra BB00_3;\n"
	"	bra.uni BB00_2;\n"
	"\n"
	"	BB00_5:\n"
	"	mov.f32 %f9, 0f00000000;\n"
	"\n"
	"	BB00_2:\n"
	"	div.rn.f32 %f8, %f9, %f1;\n"
	"	st.global.f32 [%rl5], %f8;\n"
	"\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the variance for all image pixels for a given windows size
	".visible .entry ComputeImageVar(\n"
	"	.param .u64 .ptr param_1, // image pixels mean\n"
	"	.param .u64 .ptr param_2, // image mask\n"
	"	.param .u64 .ptr param_3, // image pixels variance [out]\n"
	"	.param .u32 param_4, // image width\n"
	"	.param .u32 param_5, // image height\n"
	"	.param .u32 param_6 // half-window size\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<15>;\n"
	"	.reg .pred %p<19>;\n"
	"	.reg .s16 %rc<4>;\n"
	"	.reg .s32 %r<43>;\n"
	"	.reg .s64 %rl<17>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_4];\n"
	"	ld.param.u32 %r2, [param_5];\n"
	"	mov.u32 %r12, %ntid.x;\n"
	"	mov.u32 %r13, %ctaid.x;\n"
	"	mov.u32 %r14, %tid.x;\n"
	"	mad.lo.s32 %r4, %r12, %r13, %r14;\n"
	"	mov.u32 %r15, %ntid.y;\n"
	"	mov.u32 %r16, %ctaid.y;\n"
	"	mov.u32 %r17, %tid.y;\n"
	"	mad.lo.s32 %r5, %r15, %r16, %r17;\n"
	"	setp.gt.s32 %p1, %r4, -1;\n"
	"	setp.lt.s32 %p2, %r4, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r5, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r5, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl8, [param_1];\n"
	"	ld.param.u64 %rl9, [param_2];\n"
	"	ld.param.u64 %rl10, [param_3];\n"
	"	cvta.to.global.u64 %rl1, %rl8;\n"
	"	cvta.to.global.u64 %rl3, %rl9;\n"
	"	cvta.to.global.u64 %rl4, %rl10;\n"
	"	ld.param.u32 %r39, [param_6];\n"
	"	shl.b32 %r18, %r39, 1;\n"
	"	or.b32 %r19, %r18, 1;\n"
	"	cvt.rn.f32.s32 %f7, %r19;\n"
	"	mul.f32 %f1, %f7, %f7;\n"
	"	ld.param.u32 %r34, [param_4];\n"
	"	mad.lo.s32 %r20, %r5, %r34, %r4;\n"
	"	cvt.s64.s32 %rl5, %r20;\n"
	"	shl.b64 %rl11, %rl5, 2;\n"
	"	add.s64 %rl6, %rl4, %rl11;\n"
	"	mov.f32 %f3, 0f00000000;\n"
	"	st.global.f32 [%rl6], %f3;\n"
	"	sub.s32 %r23, %r34, %r39;\n"
	"	setp.lt.s32 %p8, %r4, %r23;\n"
	"	setp.ge.s32 %p9, %r4, %r39;\n"
	"	and.pred %p10, %p8, %p9;\n"
	"	setp.ge.s32 %p11, %r5, %r39;\n"
	"	and.pred %p12, %p10, %p11;\n"
	"	ld.param.u32 %r35, [param_5];\n"
	"	sub.s32 %r24, %r35, %r39;\n"
	"	setp.lt.s32 %p13, %r5, %r24;\n"
	"	and.pred %p14, %p12, %p13;\n"
	"	@!%p14 bra BB00_1;\n"
	"\n"
	"	add.s64 %rl12, %rl3, %rl5;\n"
	"	ld.global.u8 %rc1, [%rl12];\n"
	"	cvt.s16.s8 %rc1, %rc1;\n"
	"	mov.b16 %rc2, 1;\n"
	"	cvt.s16.s8 %rc2, %rc2;\n"
	"	setp.eq.s16 %p15, %rc1, %rc2;\n"
	"	@!%p15 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r38, [param_6];\n"
	"	neg.s32 %r6, %r38;\n"
	"	add.s64 %rl14, %rl1, %rl11;\n"
	"	ld.global.f32 %f2, [%rl14];\n"
	"	mov.f32 %f14, 0f00000000;\n"
	"	mov.u32 %r42, %r6;\n"
	"\n"
	"	BB00_2:\n"
	"	mov.u32 %r7, %r42;\n"
	"	add.s32 %r8, %r7, %r4;\n"
	"	mov.u32 %r41, %r6;\n"
	"\n"
	"	BB00_3:\n"
	"	add.s32 %r26, %r41, %r5;\n"
	"	shl.b32 %r27, %r8, 1;\n"
	"	suld.b.2d.b16.trap {%rc3}, [surfImageRef, {%r27, %r26}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rc3;\n"
	"	cvt.f32.f16 %f9, %temp;\n"
	"	}\n"
	"	sub.f32 %f10, %f9, %f2;\n"
	"	fma.rn.f32 %f14, %f10, %f10, %f14;\n"
	"	add.s32 %r41, %r41, 1;\n"
	"	ld.param.u32 %r37, [param_6];\n"
	"	setp.le.s32 %p17, %r41, %r37;\n"
	"	@%p17 bra BB00_3;\n"
	"\n"
	"	add.s32 %r11, %r7, 1;\n"
	"	ld.param.u32 %r36, [param_6];\n"
	"	setp.le.s32 %p18, %r11, %r36;\n"
	"	mov.u32 %r42, %r11;\n"
	"	@%p18 bra BB00_2;\n"
	"\n"
	"	div.rn.f32 %f12, %f14, %f1;\n"
	"	max.f32 %f12, %f12, 0f38D1B717;\n"
	"	st.global.f32 [%rl6], %f12;\n"
	"\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the covariance for all image pixels for a given windows size
	".visible .entry ComputeImageCov(\n"
	"	.param .u64 .ptr param_1, // meanA\n"
	"	.param .u64 .ptr param_2, // meanB\n"
	"	.param .u64 .ptr param_3, // mask\n"
	"	.param .u64 .ptr param_4, // cov [out]\n"
	"	.param .u32 param_5, // image width\n"
	"	.param .u32 param_6, // image height\n"
	"	.param .u32 param_7 // window size\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<17>;\n"
	"	.reg .pred %p<19>;\n"
	"	.reg .s16 %rs<4>;\n"
	"	.reg .s32 %r<53>;\n"
	"	.reg .s64 %rl<27>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_5];\n"
	"	ld.param.u32 %r2, [param_6];\n"
	"	ld.param.u64 %rl10, [param_1];\n"
	"	ld.param.u64 %rl12, [param_2];\n"
	"	ld.param.u64 %rl13, [param_3];\n"
	"	ld.param.u64 %rl1, [param_4];\n"
	"	cvta.to.global.u64 %rl2, %rl12;\n"
	"	cvta.to.global.u64 %rl4, %rl10;\n"
	"	cvta.to.global.u64 %rl6, %rl13;\n"
	"	cvta.to.global.u64 %rl7, %rl1;\n"
	"	mov.u32 %r12, %ntid.x;\n"
	"	mov.u32 %r13, %ctaid.x;\n"
	"	mov.u32 %r14, %tid.x;\n"
	"	mad.lo.s32 %r4, %r12, %r13, %r14;\n"
	"	mov.u32 %r15, %ntid.y;\n"
	"	mov.u32 %r16, %ctaid.y;\n"
	"	mov.u32 %r17, %tid.y;\n"
	"	mad.lo.s32 %r5, %r15, %r16, %r17;\n"
	"	setp.gt.s32 %p1, %r4, -1;\n"
	"	setp.lt.s32 %p2, %r4, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r5, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r5, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r49, [param_7];\n"
	"	shl.b32 %r18, %r49, 1;\n"
	"	or.b32 %r19, %r18, 1;\n"
	"	cvt.rn.f32.s32 %f8, %r19;\n"
	"	mul.f32 %f1, %f8, %f8;\n"
	"	ld.param.u32 %r44, [param_5];\n"
	"	mad.lo.s32 %r20, %r5, %r44, %r4;\n"
	"	cvt.s64.s32 %rl8, %r20;\n"
	"	mul.wide.s32 %rl14, %r20, 4;\n"
	"	add.s64 %rl15, %rl7, %rl14;\n"
	"	mov.u32 %r21, 0;\n"
	"	st.global.u32 [%rl15], %r21;\n"
	"	sub.s32 %r23, %r44, %r49;\n"
	"	setp.lt.s32 %p8, %r4, %r23;\n"
	"	setp.ge.s32 %p9, %r4, %r49;\n"
	"	and.pred %p10, %p8, %p9;\n"
	"	setp.ge.s32 %p11, %r5, %r49;\n"
	"	and.pred %p12, %p10, %p11;\n"
	"	ld.param.u32 %r45, [param_6];\n"
	"	sub.s32 %r24, %r45, %r49;\n"
	"	setp.lt.s32 %p13, %r5, %r24;\n"
	"	and.pred %p14, %p12, %p13;\n"
	"	@!%p14 bra BB00_1;\n"
	"\n"
	"	add.s64 %rl16, %rl6, %rl8;\n"
	"	ld.global.u8 %rs3, [%rl16];\n"
	"	{\n"
	"	.reg .s16 %temp1;\n"
	"	.reg .s16 %temp2;\n"
	"	cvt.s16.s8 %temp1, %rs3;\n"
	"	mov.b16 %temp2, 1;\n"
	"	cvt.s16.s8 %temp2, %temp2;\n"
	"	setp.eq.s16 %p15, %temp1, %temp2;\n"
	"	}\n"
	"	@!%p15 bra BB00_1;\n"
	"\n"
	"	neg.s32 %r6, %r49;\n"
	"	setp.gt.s32 %p16, %r6, %r49;\n"
	"	@%p16 bra BB00_4;\n"
	"\n"
	"	shl.b64 %rl17, %rl8, 2;\n"
	"	add.s64 %rl18, %rl4, %rl17;\n"
	"	ld.global.f32 %f2, [%rl18];\n"
	"	add.s64 %rl19, %rl2, %rl17;\n"
	"	ld.global.f32 %f3, [%rl19];\n"
	"	mov.f32 %f16, 0f00000000;\n"
	"	mov.u32 %r52, %r6;\n"
	"\n"
	"	BB00_2:\n"
	"	mov.u32 %r7, %r52;\n"
	"	add.s32 %r8, %r7, %r4;\n"
	"	mov.u32 %r51, %r6;\n"
	"\n"
	"	BB00_3:\n"
	"	add.s32 %r26, %r51, %r5;\n"
	"	shl.b32 %r27, %r8, 1;\n"
	"	suld.b.2d.b16.trap {%rs1}, [surfImageRef, {%r27, %r26}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rs1;\n"
	"	cvt.f32.f16 %f10, %temp;\n"
	"	}\n"
	"	sub.f32 %f11, %f10, %f2;\n"
	"	suld.b.2d.b16.trap {%rs2}, [surfImageProjRef, {%r27, %r26}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rs2;\n"
	"	cvt.f32.f16 %f12, %temp;\n"
	"	}\n"
	"	sub.f32 %f13, %f12, %f3;\n"
	"	fma.rn.f32 %f16, %f11, %f13, %f16;\n"
	"	add.s32 %r51, %r51, 1;\n"
	"	setp.le.s32 %p17, %r51, %r49;\n"
	"	@%p17 bra BB00_3;\n"
	"\n"
	"	add.s32 %r11, %r7, 1;\n"
	"	setp.le.s32 %p18, %r11, %r49;\n"
	"	mov.u32 %r52, %r11;\n"
	"	@%p18 bra BB00_2;\n"
	"	bra.uni BB00_5;\n"
	"\n"
	"	BB00_4:\n"
	"	mov.f32 %f16, 0f00000000;\n"
	"\n"
	"	BB00_5:\n"
	"	ld.param.u32 %r42, [param_5];\n"
	"	mad.lo.s32 %r40, %r5, %r42, %r4;\n"
	"	ld.param.u64 %rl26, [param_4];\n"
	"	cvta.to.global.u64 %rl23, %rl26;\n"
	"	mul.wide.s32 %rl24, %r40, 4;\n"
	"	add.s64 %rl25, %rl23, %rl24;\n"
	"	div.rn.f32 %f15, %f16, %f1;\n"
	"	st.global.f32 [%rl25], %f15;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the ZNCC score for all image pixels
	".visible .entry ComputeImageZNCC(\n"
	"	.param .u64 .ptr param_2, // cov\n"
	"	.param .u64 .ptr param_3, // varA\n"
	"	.param .u64 .ptr param_4, // varB\n"
	"	.param .u64 .ptr param_5, // mask\n"
	"	.param .u64 .ptr param_6, // ZNCC [out]\n"
	"	.param .u32 param_0, // image width\n"
	"	.param .u32 param_1, // image height\n"
	"	.param .u32 param_7 // window size\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<7>;\n"
	"	.reg .pred %p<16>;\n"
	"	.reg .s32 %r<25>;\n"
	"	.reg .s64 %rl<19>;\n"
	"	.reg .s16 %rc<2>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_0];\n"
	"	ld.param.u32 %r2, [param_1];\n"
	"	ld.param.u64 %rl8, [param_2];\n"
	"	ld.param.u64 %rl9, [param_3];\n"
	"	ld.param.u64 %rl10, [param_4];\n"
	"	ld.param.u64 %rl11, [param_5];\n"
	"	ld.param.u64 %rl12, [param_6];\n"
	"	cvta.to.global.u64 %rl1, %rl10;\n"
	"	cvta.to.global.u64 %rl2, %rl9;\n"
	"	cvta.to.global.u64 %rl3, %rl8;\n"
	"	cvta.to.global.u64 %rl4, %rl11;\n"
	"	cvta.to.global.u64 %rl5, %rl12;\n"
	"	mov.u32 %r6, %ntid.x;\n"
	"	mov.u32 %r7, %ctaid.x;\n"
	"	mov.u32 %r8, %tid.x;\n"
	"	mad.lo.s32 %r4, %r6, %r7, %r8;\n"
	"	mov.u32 %r9, %ntid.y;\n"
	"	mov.u32 %r10, %ctaid.y;\n"
	"	mov.u32 %r11, %tid.y;\n"
	"	mad.lo.s32 %r5, %r9, %r10, %r11;\n"
	"	setp.gt.s32 %p1, %r4, -1;\n"
	"	setp.lt.s32 %p2, %r4, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r5, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r5, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	ld.param.u32 %r22, [param_0];\n"
	"	mad.lo.s32 %r12, %r5, %r22, %r4;\n"
	"	cvt.s64.s32 %rl6, %r12;\n"
	"	mul.wide.s32 %rl13, %r12, 4;\n"
	"	add.s64 %rl7, %rl5, %rl13;\n"
	"	mov.u32 %r13, 0;\n"
	"	st.global.u32 [%rl7], %r13;\n"
	"	ld.param.u32 %r24, [param_7];\n"
	"	sub.s32 %r15, %r22, %r24;\n"
	"	setp.lt.s32 %p8, %r4, %r15;\n"
	"	setp.ge.s32 %p9, %r4, %r24;\n"
	"	and.pred %p10, %p8, %p9;\n"
	"	setp.ge.s32 %p11, %r5, %r24;\n"
	"	and.pred %p12, %p10, %p11;\n"
	"	ld.param.u32 %r23, [param_1];\n"
	"	sub.s32 %r16, %r23, %r24;\n"
	"	setp.lt.s32 %p13, %r5, %r16;\n"
	"	and.pred %p14, %p12, %p13;\n"
	"	@!%p14 bra BB00_1;\n"
	"\n"
	"	add.s64 %rl14, %rl4, %rl6;\n"
	"	ld.global.u8 %rc1, [%rl14];\n"
	"	{\n"
	"	.reg .s16 %temp1;\n"
	"	.reg .s16 %temp2;\n"
	"	cvt.s16.s8 %temp1, %rc1;\n"
	"	mov.b16 %temp2, 1;\n"
	"	cvt.s16.s8 %temp2, %temp2;\n"
	"	setp.eq.s16 %p15, %temp1, %temp2;\n"
	"	}\n"
	"	@!%p15 bra BB00_1;\n"
	"\n"
	"	shl.b64 %rl15, %rl6, 2;\n"
	"	add.s64 %rl16, %rl3, %rl15;\n"
	"	add.s64 %rl17, %rl1, %rl15;\n"
	"	ld.global.f32 %f1, [%rl17];\n"
	"	add.s64 %rl18, %rl2, %rl15;\n"
	"	ld.global.f32 %f2, [%rl18];\n"
	"	mul.f32 %f3, %f2, %f1;\n"
	"	sqrt.rn.f32 %f4, %f3;\n"
	"	ld.global.f32 %f5, [%rl16];\n"
	"	div.rn.f32 %f6, %f5, %f4;\n"
	"	st.global.f32 [%rl7], %f6;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the gradient of the ZNCC score for all image pixels
	".visible .entry ComputeImageDZNCC(\n"
	"	.param .u64 .ptr param_1, // meanA\n"
	"	.param .u64 .ptr param_2, // meanB\n"
	"	.param .u64 .ptr param_3, // varA\n"
	"	.param .u64 .ptr param_4, // varB\n"
	"	.param .u64 .ptr param_5, // ZNCC\n"
	"	.param .u64 .ptr param_6, // mask\n"
	"	.param .u64 .ptr param_7, // NCCGrad [out]\n"
	"	.param .u32 param_8, // image width\n"
	"	.param .u32 param_9, // image height\n"
	"	.param .u32 param_10 // window size\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<69>;\n"
	"	.reg .pred %p<20>;\n"
	"	.reg .s16 %rc<7>;\n"
	"	.reg .s32 %r<90>;\n"
	"	.reg .s64 %rl<81>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_8];\n"
	"	ld.param.u32 %r2, [param_9];\n"
	"	mov.u32 %r17, %ntid.x;\n"
	"	mov.u32 %r18, %ctaid.x;\n"
	"	mov.u32 %r19, %tid.x;\n"
	"	mad.lo.s32 %r5, %r17, %r18, %r19;\n"
	"	mov.u32 %r20, %ntid.y;\n"
	"	mov.u32 %r21, %ctaid.y;\n"
	"	mov.u32 %r7, %tid.y;\n"
	"	mad.lo.s32 %r8, %r20, %r21, %r7;\n"
	"	setp.gt.s32 %p1, %r5, -1;\n"
	"	setp.lt.s32 %p2, %r5, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r8, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r8, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	mad.lo.s32 %r22, %r8, %r1, %r5;\n"
	"	ld.param.u64 %rl55, [param_7];\n"
	"	cvta.to.global.u64 %rl13, %rl55;\n"
	"	mul.wide.s32 %rl14, %r22, 4;\n"
	"	add.s64 %rl15, %rl13, %rl14;\n"
	"	mov.u32 %r23, 0;\n"
	"	st.global.u32 [%rl15], %r23;\n"
	"	ld.param.u32 %r74, [param_10];\n"
	"	sub.s32 %r27, %r1, %r74;\n"
	"	setp.lt.s32 %p8, %r5, %r27;\n"
	"	setp.ge.s32 %p9, %r5, %r74;\n"
	"	and.pred %p10, %p8, %p9;\n"
	"	setp.ge.s32 %p11, %r8, %r74;\n"
	"	and.pred %p12, %p10, %p11;\n"
	"	sub.s32 %r28, %r2, %r74;\n"
	"	setp.lt.s32 %p13, %r8, %r28;\n"
	"	and.pred %p14, %p12, %p13;\n"
	"	@!%p14 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl48, [param_3];\n"
	"	cvta.to.global.u64 %rl23, %rl48;\n"
	"	ld.param.u64 %rl50, [param_4];\n"
	"	cvta.to.global.u64 %rl26, %rl50;\n"
	"	ld.param.u64 %rl53, [param_6];\n"
	"	cvta.to.global.u64 %rl18, %rl53;\n"
	"	ld.param.u32 %r66, [param_8];\n"
	"	mad.lo.s32 %r29, %r8, %r66, %r5;\n"
	"	cvt.s64.s32 %rl19, %r29;\n"
	"	add.s64 %rl20, %rl18, %rl19;\n"
	"	ld.global.u8 %rc1, [%rl20];\n"
	"	{\n"
	"	.reg .s16 %temp1;\n"
	"	.reg .s16 %temp2;\n"
	"	cvt.s16.s8 %temp1, %rc1;\n"
	"	mov.b16 %temp2, 1;\n"
	"	cvt.s16.s8 %temp2, %temp2;\n"
	"	setp.eq.s16 %p15, %temp1, %temp2;\n"
	"	}\n"
	"	@!%p15 bra BB00_1;\n"
	"\n"
	"	neg.s32 %r31, %r74;\n"
	"	setp.gt.s32 %p16, %r31, %r74;\n"
	"	@%p16 bra BB00_6;\n"
	"\n"
	"	neg.s32 %r77, %r74;\n"
	"	mov.f32 %f56, 0f00000000;\n"
	"	mov.f32 %f57, %f56;\n"
	"	mov.f32 %f58, %f56;\n"
	"	mov.f32 %f59, %f56;\n"
	"\n"
	"	BB00_2:\n"
	"	add.s32 %r12, %r77, %r5;\n"
	"	neg.s32 %r78, %r74;\n"
	"\n"
	"	BB00_3:\n"
	"	ld.param.u64 %rl52, [param_6];\n"
	"	cvta.to.global.u64 %rl21, %rl52;\n"
	"	add.s32 %r36, %r78, %r8;\n"
	"	ld.param.u32 %r65, [param_8];\n"
	"	mad.lo.s32 %r37, %r36, %r65, %r12;\n"
	"	cvt.s64.s32 %rl12, %r37;\n"
	"	add.s64 %rl22, %rl21, %rl12;\n"
	"	ld.global.u8 %rc2, [%rl22];\n"
	"	{\n"
	"	.reg .s16 %temp1;\n"
	"	.reg .s16 %temp2;\n"
	"	cvt.s16.s8 %temp1, %rc2;\n"
	"	mov.b16 %temp2, 1;\n"
	"	cvt.s16.s8 %temp2, %temp2;\n"
	"	setp.eq.s16 %p17, %temp1, %temp2;\n"
	"	}\n"
	"	@%p17 bra BB00_4;\n"
	"	bra.uni BB00_5;\n"
	"\n"
	"	BB00_4:\n"
	"	shl.b64 %rl24, %rl12, 2;\n"
	"	add.s64 %rl25, %rl23, %rl24;\n"
	"	add.s64 %rl27, %rl26, %rl24;\n"
	"	ld.global.f32 %f25, [%rl27];\n"
	"	ld.global.f32 %f26, [%rl25];\n"
	"	mul.f32 %f27, %f26, %f25;\n"
	"	sqrt.rn.f32 %f28, %f27;\n"
	"	rcp.rn.f32 %f29, %f28;\n"
	"	sub.f32 %f58, %f58, %f29;\n"
	"	ld.param.u64 %rl51, [param_5];\n"
	"	cvta.to.global.u64 %rl28, %rl51;\n"
	"	add.s64 %rl29, %rl28, %rl24;\n"
	"	ld.global.f32 %f30, [%rl29];\n"
	"	div.rn.f32 %f31, %f30, %f25;\n"
	"	add.f32 %f57, %f57, %f31;\n"
	"	ld.param.u64 %rl45, [param_1];\n"
	"	cvta.to.global.u64 %rl30, %rl45;\n"
	"	add.s64 %rl31, %rl30, %rl24;\n"
	"	ld.global.f32 %f32, [%rl31];\n"
	"	ld.param.u64 %rl46, [param_2];\n"
	"	cvta.to.global.u64 %rl32, %rl46;\n"
	"	add.s64 %rl33, %rl32, %rl24;\n"
	"	ld.global.f32 %f33, [%rl33];\n"
	"	mul.f32 %f34, %f33, %f30;\n"
	"	div.rn.f32 %f35, %f34, %f25;\n"
	"	neg.f32 %f36, %f35;\n"
	"	fma.rn.f32 %f37, %f32, %f29, %f36;\n"
	"	add.f32 %f56, %f56, %f37;\n"
	"	add.f32 %f59, %f59, 0f3F800000;\n"
	"\n"
	"	BB00_5:\n"
	"	add.s32 %r78, %r78, 1;\n"
	"	setp.le.s32 %p18, %r78, %r74;\n"
	"	@%p18 bra BB00_3;\n"
	"\n"
	"	add.s32 %r77, %r77, 1;\n"
	"	setp.le.s32 %p19, %r77, %r74;\n"
	"	@%p19 bra BB00_2;\n"
	"	bra.uni BB00_7;\n"
	"\n"
	"	BB00_6:\n"
	"	mov.f32 %f59, 0f00000000;\n"
	"	mov.f32 %f58, %f59;\n"
	"	mov.f32 %f57, %f59;\n"
	"	mov.f32 %f56, %f59;\n"
	"\n"
	"	BB00_7:\n"
	"	shl.b32 %r9, %r5, 1;\n"
	"	suld.b.2d.b16.trap {%rc5}, [surfImageProjRef, {%r9, %r8}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rc5;\n"
	"	cvt.f32.f16 %f22, %temp;\n"
	"	}\n"
	"	div.rn.f32 %f43, %f57, %f59;\n"
	"	mul.f32 %f65, %f43, %f22;\n"
	"	div.rn.f32 %f42, %f58, %f59;\n"
	"	suld.b.2d.b16.trap {%rc6}, [surfImageRef, {%r9, %r8}];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rc6;\n"
	"	cvt.f32.f16 %f21, %temp;\n"
	"	}\n"
	"	fma.rn.f32 %f66, %f42, %f21, %f65;\n"
	"	div.rn.f32 %f44, %f56, %f59;\n"
	"	add.f32 %f68, %f66, %f44;\n"
	"	add.s64 %rl42, %rl23, %rl14;\n"
	"	add.s64 %rl44, %rl26, %rl14;\n"
	"	ld.global.f32 %f45, [%rl44];\n"
	"	ld.global.f32 %f46, [%rl42];\n"
	"	min.f32 %f47, %f46, %f45;\n"
	"	add.f32 %f48, %f47, 0f3AC49BA6;\n"
	"	div.rn.f32 %f49, %f47, %f48;\n"
	"	mul.f32 %f51, %f68, %f49;\n"
	"	st.global.f32 [%rl15], %f51;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the photometric gradient for all vertices seen by an image pair
	".visible .entry ComputePhotometricGradient(\n"
	"	.param .u64 .ptr param_1, // faces\n"
	"	.param .u64 .ptr param_2, // normals\n"
	"	.param .u64 .ptr param_3, // depth-map A (float)\n"
	"	.param .u64 .ptr param_4, // face-map A (uint32_t)\n"
	"	.param .u64 .ptr param_5, // bary-map A (hfloat*3)\n"
	"	.param .u64 .ptr param_6, // DZNCC\n"
	"	.param .u64 .ptr param_7, // mask\n"
	"	.param .u64 .ptr param_8, // photo-grad [in/out]\n"
	"	.param .u64 .ptr param_9, // photo-grad-norm [in/out]\n"
	"	.param .align 4 .b8 param_10[176], // camera A\n"
	"	.param .align 4 .b8 param_11[176], // camera B\n"
	"	.param .f32 param_12 // square(avg-depth/f) scale (float)\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<232>;\n"
	"	.reg .pred %p<11>;\n"
	"	.reg .s16 %rs<5>;\n"
	"	.reg .s32 %r<69>;\n"
	"	.reg .s64 %rl<66>;\n"
	"\n"
	"	ld.param.u32 %r1, [param_10+168];\n"
	"	ld.param.u32 %r2, [param_10+172];\n"
	"	mov.u32 %r9, %ntid.x;\n"
	"	mov.u32 %r10, %ctaid.x;\n"
	"	mov.u32 %r11, %tid.x;\n"
	"	mad.lo.s32 %r3, %r9, %r10, %r11;\n"
	"	mov.u32 %r12, %ntid.y;\n"
	"	mov.u32 %r13, %ctaid.y;\n"
	"	mov.u32 %r14, %tid.y;\n"
	"	mad.lo.s32 %r4, %r12, %r13, %r14;\n"
	"	setp.gt.s32 %p1, %r3, -1;\n"
	"	setp.lt.s32 %p2, %r3, %r1;\n"
	"	and.pred %p3, %p1, %p2;\n"
	"	setp.gt.s32 %p4, %r4, -1;\n"
	"	and.pred %p5, %p3, %p4;\n"
	"	setp.lt.s32 %p6, %r4, %r2;\n"
	"	and.pred %p7, %p5, %p6;\n"
	"	@!%p7 bra BB00_1;\n"
	"\n"
	"	mad.lo.s32 %r15, %r4, %r1, %r3;\n"
	"	cvt.s64.s32 %rl11, %r15;\n"
	"	ld.param.u64 %rl12, [param_7];\n"
	"	cvta.to.global.u64 %rl10, %rl12;\n"
	"	add.s64 %rl13, %rl10, %rl11;\n"
	"	ld.global.u8 %rs4, [%rl13];\n"
	"	{\n"
	"	.reg .s16 %temp1;\n"
	"	.reg .s16 %temp2;\n"
	"	cvt.s16.s8 %temp1, %rs4;\n"
	"	mov.b16 %temp2, 1;\n"
	"	cvt.s16.s8 %temp2, %temp2;\n"
	"	setp.ne.s16 %p8, %temp1, %temp2;\n"
	"	}\n"
	"	@%p8 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl59, [param_3];\n"
	"	cvta.to.global.u64 %rl14, %rl59;\n"
	"	shl.b64 %rl15, %rl11, 2;\n"
	"	add.s64 %rl16, %rl14, %rl15;\n"
	"	ld.global.f32 %f4, [%rl16];\n"
	"	ld.param.u64 %rl60, [param_4];\n"
	"	cvta.to.global.u64 %rl17, %rl60;\n"
	"	add.s64 %rl18, %rl17, %rl15;\n"
	"	ld.param.u64 %rl61, [param_5];\n"
	"	cvta.to.global.u64 %rl19, %rl61;\n"
	"	mul.wide.s32 %rl9, %r15, 6;\n"
	"	add.s64 %rl20, %rl19, %rl9;\n"
	"	ld.global.b16 %rs1, [%rl20];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rs1;\n"
	"	cvt.f32.f16 %f41, %temp;\n"
	"	}\n"
	"	ld.global.b16 %rs2, [%rl20+2];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rs2;\n"
	"	cvt.f32.f16 %f42, %temp;\n"
	"	}\n"
	"	ld.global.b16 %rs3, [%rl20+4];\n"
	"	{\n"
	"	.reg .b16 %temp;\n"
	"	mov.b16 %temp, %rs3;\n"
	"	cvt.f32.f16 %f43, %temp;\n"
	"	}\n"
	"	ld.param.f32 %f44, [param_12];\n"
	"	ld.global.u32 %r20, [%rl18];\n"
	"	mul.lo.s32 %r22, %r20, 3;\n"
	"	ld.param.u64 %rl57, [param_1];\n"
	"	cvta.to.global.u64 %rl23, %rl57;\n"
	"	mul.wide.s32 %rl24, %r22, 4;\n"
	"	add.s64 %rl25, %rl23, %rl24;\n"
	"	ld.global.u32 %r5, [%rl25];\n"
	"	ld.global.u32 %r6, [%rl25+4];\n"
	"	ld.global.u32 %r7, [%rl25+8];\n"
	"	ld.param.u64 %rl58, [param_2];\n"
	"	cvta.to.global.u64 %rl26, %rl58;\n"
	"	add.s64 %rl27, %rl26, %rl24;\n"
	"	cvt.rn.f32.s32 %f71, %r3;\n"
	"	ld.param.f32 %f193, [param_10+104];\n"
	"	sub.f32 %f72, %f71, %f193;\n"
	"	ld.param.f32 %f194, [param_10+96];\n"
	"	div.rn.f32 %f73, %f72, %f194;\n"
	"	cvt.rn.f32.s32 %f74, %r4;\n"
	"	ld.param.f32 %f191, [param_10+116];\n"
	"	sub.f32 %f75, %f74, %f191;\n"
	"	ld.param.f32 %f192, [param_10+112];\n"
	"	div.rn.f32 %f76, %f75, %f192;\n"
	"	ld.param.f32 %f203, [param_10+60];\n"
	"	mul.f32 %f77, %f203, %f76;\n"
	"	ld.param.f32 %f206, [param_10+48];\n"
	"	fma.rn.f32 %f78, %f206, %f73, %f77;\n"
	"	ld.param.f32 %f200, [param_10+72];\n"
	"	add.f32 %f79, %f78, %f200;\n"
	"	ld.param.f32 %f202, [param_10+64];\n"
	"	mul.f32 %f80, %f202, %f76;\n"
	"	ld.param.f32 %f205, [param_10+52];\n"
	"	fma.rn.f32 %f81, %f205, %f73, %f80;\n"
	"	ld.param.f32 %f199, [param_10+76];\n"
	"	add.f32 %f82, %f81, %f199;\n"
	"	ld.param.f32 %f201, [param_10+68];\n"
	"	mul.f32 %f83, %f201, %f76;\n"
	"	ld.param.f32 %f204, [param_10+56];\n"
	"	fma.rn.f32 %f84, %f204, %f73, %f83;\n"
	"	ld.param.f32 %f198, [param_10+80];\n"
	"	add.f32 %f85, %f84, %f198;\n"
	"	mul.f32 %f86, %f82, %f82;\n"
	"	fma.rn.f32 %f87, %f79, %f79, %f86;\n"
	"	fma.rn.f32 %f88, %f85, %f85, %f87;\n"
	"	sqrt.rn.f32 %f89, %f88;\n"
	"	div.rn.f32 %f45, %f79, %f89;\n"
	"	div.rn.f32 %f46, %f82, %f89;\n"
	"	div.rn.f32 %f47, %f85, %f89;\n"
	"	ld.global.f32 %f48, [%rl27];\n"
	"	ld.global.f32 %f49, [%rl27+4];\n"
	"	mul.f32 %f90, %f49, %f46;\n"
	"	fma.rn.f32 %f91, %f48, %f45, %f90;\n"
	"	ld.global.f32 %f50, [%rl27+8];\n"
	"	fma.rn.f32 %f51, %f50, %f47, %f91;\n"
	"	setp.gt.f32 %p9, %f51, 0fBDCCCCCD;\n"
	"	@%p9 bra BB00_1;\n"
	"\n"
	"	ld.param.f32 %f38, [param_10+84];\n"
	"	fma.rn.f32 %f92, %f4, %f79, %f38;\n"
	"	ld.param.f32 %f39, [param_10+88];\n"
	"	fma.rn.f32 %f93, %f4, %f82, %f39;\n"
	"	ld.param.f32 %f40, [param_10+92];\n"
	"	fma.rn.f32 %f94, %f4, %f85, %f40;\n"
	"	ld.param.f32 %f227, [param_11+4];\n"
	"	mul.f32 %f95, %f227, %f93;\n"
	"	ld.param.f32 %f229, [param_11];\n"
	"	fma.rn.f32 %f96, %f229, %f92, %f95;\n"
	"	ld.param.f32 %f225, [param_11+8];\n"
	"	fma.rn.f32 %f97, %f225, %f94, %f96;\n"
	"	ld.param.f32 %f223, [param_11+12];\n"
	"	add.f32 %f52, %f97, %f223;\n"
	"	ld.param.f32 %f220, [param_11+20];\n"
	"	mul.f32 %f98, %f220, %f93;\n"
	"	ld.param.f32 %f222, [param_11+16];\n"
	"	fma.rn.f32 %f99, %f222, %f92, %f98;\n"
	"	ld.param.f32 %f218, [param_11+24];\n"
	"	fma.rn.f32 %f100, %f218, %f94, %f99;\n"
	"	ld.param.f32 %f216, [param_11+28];\n"
	"	add.f32 %f53, %f100, %f216;\n"
	"	ld.param.f32 %f213, [param_11+36];\n"
	"	mul.f32 %f101, %f213, %f93;\n"
	"	ld.param.f32 %f215, [param_11+32];\n"
	"	fma.rn.f32 %f102, %f215, %f92, %f101;\n"
	"	ld.param.f32 %f211, [param_11+40];\n"
	"	fma.rn.f32 %f103, %f211, %f94, %f102;\n"
	"	ld.param.f32 %f209, [param_11+44];\n"
	"	add.f32 %f54, %f103, %f209;\n"
	"	setp.gt.f32 %p10, %f54, 0f00000000;\n"
	"	@%p10 bra BB00_2;\n"
	"\n"
	"	mov.f32 %f231, 0fBF800000;\n"
	"	mov.f32 %f230, %f231;\n"
	"	bra.uni BB00_3;\n"
	"\n"
	"	BB00_2:\n"
	"	div.rn.f32 %f230, %f52, %f54;\n"
	"	div.rn.f32 %f231, %f53, %f54;\n"
	"\n"
	"	BB00_3:\n"
	"	ld.param.f32 %f228, [param_11];\n"
	"	mul.f32 %f118, %f228, %f54;\n"
	"	neg.f32 %f119, %f52;\n"
	"	ld.param.f32 %f214, [param_11+32];\n"
	"	fma.rn.f32 %f120, %f119, %f214, %f118;\n"
	"	mul.f32 %f121, %f54, %f54;\n"
	"	div.rn.f32 %f122, %f120, %f121;\n"
	"	ld.param.f32 %f226, [param_11+4];\n"
	"	mul.f32 %f123, %f226, %f54;\n"
	"	ld.param.f32 %f212, [param_11+36];\n"
	"	fma.rn.f32 %f124, %f119, %f212, %f123;\n"
	"	div.rn.f32 %f125, %f124, %f121;\n"
	"	ld.param.f32 %f224, [param_11+8];\n"
	"	mul.f32 %f126, %f224, %f54;\n"
	"	ld.param.f32 %f210, [param_11+40];\n"
	"	fma.rn.f32 %f127, %f119, %f210, %f126;\n"
	"	div.rn.f32 %f128, %f127, %f121;\n"
	"	ld.param.f32 %f221, [param_11+16];\n"
	"	mul.f32 %f129, %f221, %f54;\n"
	"	neg.f32 %f130, %f53;\n"
	"	fma.rn.f32 %f131, %f130, %f214, %f129;\n"
	"	div.rn.f32 %f132, %f131, %f121;\n"
	"	ld.param.f32 %f219, [param_11+20];\n"
	"	mul.f32 %f133, %f219, %f54;\n"
	"	fma.rn.f32 %f134, %f130, %f212, %f133;\n"
	"	div.rn.f32 %f135, %f134, %f121;\n"
	"	ld.param.f32 %f217, [param_11+24];\n"
	"	mul.f32 %f136, %f217, %f54;\n"
	"	fma.rn.f32 %f137, %f130, %f210, %f136;\n"
	"	div.rn.f32 %f138, %f137, %f121;\n"
	"	add.f32 %f106, %f230, 0f3F800000;\n"
	"	tex.2d.v4.u32.f32 {%r29, %r30, %r31, %r32}, [texImageRef, {%f106, %f231}];\n"
	"	mov.b32 %f140, %r29;\n"
	"	tex.2d.v4.u32.f32 {%r34, %r35, %r36, %r37}, [texImageRef, {%f230, %f231}];\n"
	"	mov.b32 %f141, %r34;\n"
	"	sub.f32 %f142, %f140, %f141;\n"
	"	add.f32 %f113, %f231, 0f3F800000;\n"
	"	tex.2d.v4.u32.f32 {%r39, %r40, %r41, %r42}, [texImageRef, {%f230, %f113}];\n"
	"	mov.b32 %f143, %r39;\n"
	"	sub.f32 %f145, %f143, %f141;\n"
	"	ld.param.u64 %rl63, [param_6];\n"
	"	cvta.to.global.u64 %rl28, %rl63;\n"
	"	mul.wide.s32 %rl29, %r15, 4;\n"
	"	add.s64 %rl30, %rl28, %rl29;\n"
	"	mul.f32 %f146, %f145, %f132;\n"
	"	fma.rn.f32 %f147, %f142, %f122, %f146;\n"
	"	ld.global.f32 %f148, [%rl30];\n"
	"	mul.f32 %f149, %f148, %f147;\n"
	"	mul.f32 %f150, %f145, %f135;\n"
	"	fma.rn.f32 %f151, %f142, %f125, %f150;\n"
	"	mul.f32 %f152, %f148, %f151;\n"
	"	mul.f32 %f153, %f145, %f138;\n"
	"	fma.rn.f32 %f154, %f142, %f128, %f153;\n"
	"	mul.f32 %f155, %f148, %f154;\n"
	"	mul.f32 %f156, %f152, %f46;\n"
	"	fma.rn.f32 %f157, %f149, %f45, %f156;\n"
	"	fma.rn.f32 %f158, %f155, %f47, %f157;\n"
	"	div.rn.f32 %f159, %f158, %f51;\n"
	"	mul.lo.s32 %r60, %r5, 3;\n"
	"	ld.param.u64 %rl64, [param_8];\n"
	"	cvta.to.global.u64 %rl31, %rl64;\n"
	"	mul.wide.s32 %rl32, %r60, 4;\n"
	"	add.s64 %rl33, %rl31, %rl32;\n"
	"	mul.f32 %f162, %f44, %f41;\n"
	"	mul.f32 %f163, %f162, %f159;\n"
	"	mul.f32 %f164, %f163, %f48;\n"
	"	atom.global.add.f32 %f165, [%rl33], %f164;\n"
	"	mad.lo.s32 %r61, %r5, 3, 1;\n"
	"	mul.wide.s32 %rl34, %r61, 4;\n"
	"	add.s64 %rl35, %rl31, %rl34;\n"
	"	mul.f32 %f166, %f163, %f49;\n"
	"	atom.global.add.f32 %f167, [%rl35], %f166;\n"
	"	mad.lo.s32 %r62, %r5, 3, 2;\n"
	"	mul.wide.s32 %rl36, %r62, 4;\n"
	"	add.s64 %rl37, %rl31, %rl36;\n"
	"	mul.f32 %f168, %f163, %f50;\n"
	"	atom.global.add.f32 %f169, [%rl37], %f168;\n"
	"	mul.lo.s32 %r63, %r6, 3;\n"
	"	mul.wide.s32 %rl38, %r63, 4;\n"
	"	add.s64 %rl39, %rl31, %rl38;\n"
	"	mul.f32 %f170, %f44, %f42;\n"
	"	mul.f32 %f171, %f170, %f159;\n"
	"	mul.f32 %f172, %f171, %f48;\n"
	"	atom.global.add.f32 %f173, [%rl39], %f172;\n"
	"	mad.lo.s32 %r64, %r6, 3, 1;\n"
	"	mul.wide.s32 %rl40, %r64, 4;\n"
	"	add.s64 %rl41, %rl31, %rl40;\n"
	"	mul.f32 %f174, %f171, %f49;\n"
	"	atom.global.add.f32 %f175, [%rl41], %f174;\n"
	"	mad.lo.s32 %r65, %r6, 3, 2;\n"
	"	mul.wide.s32 %rl42, %r65, 4;\n"
	"	add.s64 %rl43, %rl31, %rl42;\n"
	"	mul.f32 %f176, %f171, %f50;\n"
	"	atom.global.add.f32 %f177, [%rl43], %f176;\n"
	"	mul.lo.s32 %r66, %r7, 3;\n"
	"	mul.wide.s32 %rl44, %r66, 4;\n"
	"	add.s64 %rl45, %rl31, %rl44;\n"
	"	mul.f32 %f178, %f44, %f43;\n"
	"	mul.f32 %f179, %f178, %f159;\n"
	"	mul.f32 %f180, %f179, %f48;\n"
	"	atom.global.add.f32 %f181, [%rl45], %f180;\n"
	"	mad.lo.s32 %r67, %r7, 3, 1;\n"
	"	mul.wide.s32 %rl46, %r67, 4;\n"
	"	add.s64 %rl47, %rl31, %rl46;\n"
	"	mul.f32 %f182, %f179, %f49;\n"
	"	atom.global.add.f32 %f183, [%rl47], %f182;\n"
	"	mad.lo.s32 %r68, %r7, 3, 2;\n"
	"	mul.wide.s32 %rl48, %r68, 4;\n"
	"	add.s64 %rl49, %rl31, %rl48;\n"
	"	mul.f32 %f184, %f179, %f50;\n"
	"	atom.global.add.f32 %f185, [%rl49], %f184;\n"
	"	ld.param.u64 %rl65, [param_9];\n"
	"	cvta.to.global.u64 %rl50, %rl65;\n"
	"	mul.wide.s32 %rl51, %r5, 4;\n"
	"	add.s64 %rl52, %rl50, %rl51;\n"
	"	atom.global.add.f32 %f186, [%rl52], 0f3F800000;\n"
	"	mul.wide.s32 %rl53, %r6, 4;\n"
	"	add.s64 %rl54, %rl50, %rl53;\n"
	"	atom.global.add.f32 %f187, [%rl54], 0f3F800000;\n"
	"	mul.wide.s32 %rl55, %r7, 4;\n"
	"	add.s64 %rl56, %rl50, %rl55;\n"
	"	atom.global.add.f32 %f188, [%rl56], 0f3F800000;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to update the norm of the photo gradient for all vertices
	".visible .entry UpdatePhotoGradNorm(\n"
	"	.param .u64 .ptr param_1, // photoGradNorm [in/out]\n"
	"	.param .u64 .ptr param_2, // photoGradPixels [in]\n"
	"	.param .u32 param_3 // numVertices\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<4>;\n"
	"	.reg .pred %p<3>;\n"
	"	.reg .s32 %r<9>;\n"
	"	.reg .s64 %rl<9>;\n"
	"\n"
	"	ld.param.u32 %r2, [param_3];\n"
	"	mov.u32 %r3, %ntid.x;\n"
	"	mov.u32 %r4, %ctaid.x;\n"
	"	mov.u32 %r5, %tid.x;\n"
	"	mad.lo.s32 %r1, %r3, %r4, %r5;\n"
	"	setp.ge.s32 %p1, %r1, %r2;\n"
	"	@%p1 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl5, [param_2];\n"
	"	cvta.to.global.u64 %rl2, %rl5;\n"
	"	mul.wide.s32 %rl6, %r1, 4;\n"
	"	add.s64 %rl7, %rl2, %rl6;\n"
	"	ld.global.f32 %f1, [%rl7];\n"
	"	setp.le.f32 %p2, %f1, 0f00000000;\n"
	"	@%p2 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl4, [param_1];\n"
	"	cvta.to.global.u64 %rl1, %rl4;\n"
	"	add.s64 %rl8, %rl1, %rl6;\n"
	"	ld.global.f32 %f2, [%rl8];\n"
	"	add.f32 %f3, %f2, 0f3F800000;\n"
	"	st.global.f32 [%rl8], %f3;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to compute the smoothness gradient for all vertices
	".visible .entry ComputeSmoothnessGradient(\n"
	"	.param .u64 .ptr param_1, // vertices\n"
	"	.param .u64 .ptr param_2, // vert-vertices [in]\n"
	"	.param .u64 .ptr param_3, // vert-sizes [in]\n"
	"	.param .u64 .ptr param_4, // vert-pos [in]\n"
	"	.param .u64 .ptr param_5, // smooth-grad [out]\n"
	"	.param .u32 param_6, // numVertices\n"
	"	.param .u8 param_7 // switch 0/1\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<38>;\n"
	"	.reg .pred %p<5>;\n"
	"	.reg .s32 %r<69>;\n"
	"	.reg .s64 %rl<42>;\n"
	"	.reg .s16 %rc<4>;\n"
	"\n"
	"	ld.param.u32 %r9, [param_6];\n"
	"	ld.param.u64 %rl1, [param_1];\n"
	"	ld.param.u64 %rl12, [param_2];\n"
	"	ld.param.u64 %rl2, [param_3];\n"
	"	ld.param.u64 %rl41, [param_4];\n"
	"	ld.param.u64 %rl13, [param_5];\n"
	"	cvta.to.global.u64 %rl4, %rl12;\n"
	"	cvta.to.global.u64 %rl20, %rl41;\n"
	"	cvta.to.global.u64 %rl5, %rl2;\n"
	"	cvta.to.global.u64 %rl6, %rl13;\n"
	"	cvta.to.global.u64 %rl7, %rl1;\n"
	"	mov.u32 %r10, %ntid.x;\n"
	"	mov.u32 %r11, %ctaid.x;\n"
	"	mov.u32 %r12, %tid.x;\n"
	"	mad.lo.s32 %r1, %r10, %r11, %r12;\n"
	"	setp.ge.s32 %p1, %r1, %r9;\n"
	"	@%p1 bra BB00_1;\n"
	"\n"
	"	mul.lo.s32 %r13, %r1, 3;\n"
	"	mul.wide.s32 %rl14, %r13, 4;\n"
	"	add.s64 %rl8, %rl6, %rl14;\n"
	"	mul.wide.s32 %rl19, %r1, 4;\n"
	"	add.s64 %rl10, %rl5, %rl19;\n"
	"	ld.global.u32 %r67, [%rl10];\n"
	"	setp.gt.s32 %p2, %r67, 0;\n"
	"	@%p2 bra BB00_5;\n"
	"\n"
	"	mov.f32 %f37, 0f00000000;\n"
	"	st.global.f32 [%rl8], %f37;\n"
	"	st.global.f32 [%rl8+4], %f37;\n"
	"	st.global.f32 [%rl8+8], %f37;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"\n"
	"	BB00_5:\n"
	"	add.s64 %rl15, %rl7, %rl14;\n"
	"	ld.global.f32 %f36, [%rl15];\n"
	"	ld.global.f32 %f35, [%rl15+4];\n"
	"	ld.global.f32 %f34, [%rl15+8];\n"
	"	ld.param.u8 %rc3, [param_7];\n"
	"	cvt.s16.s8 %rc2, %rc3;\n"
	"	mov.b16 %rc1, 0;\n"
	"	setp.eq.s16 %p3, %rc2, %rc1;\n"
	"	mov.f32 %f37, 0f3F800000;\n"
	"	add.s64 %rl11, %rl20, %rl19;\n"
	"	ld.global.u32 %r27, [%rl11];\n"
	"	cvt.rn.f32.s32 %f22, %r67;\n"
	"	rcp.rn.f32 %f30, %f22;\n"
	"	mov.u32 %r68, 0;\n"
	"\n"
	"	BB00_2:\n"
	"	add.s32 %r29, %r27, %r68;\n"
	"	mul.wide.s32 %rl22, %r29, 4;\n"
	"	add.s64 %rl23, %rl4, %rl22;\n"
	"	ld.global.u32 %r30, [%rl23];\n"
	"	mul.lo.s32 %r32, %r30, 3;\n"
	"	mul.wide.s32 %rl25, %r32, 4;\n"
	"	add.s64 %rl26, %rl7, %rl25;\n"
	"	ld.global.f32 %f20, [%rl26];\n"
	"	mul.f32 %f21, %f20, %f30;\n"
	"	sub.f32 %f36, %f36, %f21;\n"
	"	ld.global.f32 %f23, [%rl26+4];\n"
	"	mul.f32 %f24, %f23, %f30;\n"
	"	sub.f32 %f35, %f35, %f24;\n"
	"	ld.global.f32 %f26, [%rl26+8];\n"
	"	mul.f32 %f27, %f26, %f30;\n"
	"	sub.f32 %f34, %f34, %f27;\n"
	"	@%p3 bra BB00_3;\n"
	"\n"
	"	mul.wide.s32 %rl39, %r30, 4;\n"
	"	add.s64 %rl40, %rl5, %rl39;\n"
	"	ld.global.u32 %r59, [%rl40];\n"
	"	cvt.rn.f32.s32 %f28, %r59;\n"
	"	rcp.rn.f32 %f29, %f28;\n"
	"	fma.rn.f32 %f37, %f29, %f30, %f37;\n"
	"\n"
	"	BB00_3:\n"
	"	add.s32 %r68, %r68, 1;\n"
	"	setp.lt.s32 %p4, %r68, %r67;\n"
	"	@%p4 bra BB00_2;\n"
	"\n"
	"	@%p3 bra BB00_4;\n"
	"\n"
	"	div.rn.f32 %f36, %f36, %f37;\n"
	"	div.rn.f32 %f35, %f35, %f37;\n"
	"	div.rn.f32 %f34, %f34, %f37;\n"
	"\n"
	"	BB00_4:\n"
	"	st.global.f32 [%rl8], %f36;\n"
	"	st.global.f32 [%rl8+4], %f35;\n"
	"	st.global.f32 [%rl8+8], %f34;\n"
	"	ret;\n"
	"}\n"
	"\n"
	// kernel used to combine the photo and smoothness gradient for all vertices
	".visible .entry CombineGradients(\n"
	"	.param .u64 .ptr param_1, // photo-gradient [in/out]\n"
	"	.param .u64 .ptr param_2, // photo-norm [in]\n"
	"	.param .u64 .ptr param_3, // smoothness-gradient [in]\n"
	"	.param .u32 param_4, // numVertices\n"
	"	.param .f32 param_5 // smoothness-weight\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<17>;\n"
	"	.reg .pred %p<3>;\n"
	"	.reg .s32 %r<6>;\n"
	"	.reg .s64 %rl<15>;\n"
	"\n"
	"	ld.param.u32 %r2, [param_4];\n"
	"	mov.u32 %r3, %ntid.x;\n"
	"	mov.u32 %r4, %ctaid.x;\n"
	"	mov.u32 %r5, %tid.x;\n"
	"	mad.lo.s32 %r1, %r3, %r4, %r5;\n"
	"	setp.lt.s32 %p1, %r1, %r2;\n"
	"	@!%p1 bra BB00_1;\n"
	"\n"
	"	ld.param.u64 %rl4, [param_3];\n"
	"	cvta.to.global.u64 %rl2, %rl4;\n"
	"	ld.param.u64 %rl11, [param_2];\n"
	"	cvta.to.global.u64 %rl12, %rl11;\n"
	"	ld.param.u64 %rl13, [param_1];\n"
	"	cvta.to.global.u64 %rl14, %rl13;\n"
	"	mul.wide.s32 %rl4, %r1, 4;\n"
	"	mul.wide.s32 %rl5, %r1, 12;\n"
	"	add.s64 %rl9, %rl12, %rl4;\n"
	"	add.s64 %rl6, %rl2, %rl5;\n"
	"	add.s64 %rl8, %rl14, %rl5;\n"
	"	ld.param.f32 %f8, [param_5];\n"
	"	ld.global.f32 %f2, [%rl6];\n"
	"	mul.f32 %f3, %f2, %f8;\n"
	"	ld.global.f32 %f4, [%rl6+4];\n"
	"	mul.f32 %f5, %f4, %f8;\n"
	"	ld.global.f32 %f6, [%rl6+8];\n"
	"	mul.f32 %f7, %f6, %f8;\n"
	"	ld.global.f32 %f9, [%rl9];\n"
	"	setp.gt.f32 %p2, %f9, 0f00000000;\n"
	"	@%p2 bra BB00_2;\n"
	"\n"
	"	st.global.f32 [%rl8], %f3;\n"
	"	st.global.f32 [%rl8+4], %f5;\n"
	"	st.global.f32 [%rl8+8], %f7;\n"
	"	ret;\n"
	"\n"
	"	BB00_2:\n"
	"	rcp.rn.f32 %f10, %f9;\n"
	"	ld.global.f32 %f11, [%rl8];\n"
	"	fma.rn.f32 %f12, %f11, %f10, %f3;\n"
	"	ld.global.f32 %f13, [%rl8+4];\n"
	"	fma.rn.f32 %f14, %f13, %f10, %f5;\n"
	"	ld.global.f32 %f15, [%rl8+8];\n"
	"	fma.rn.f32 %f16, %f15, %f10, %f7;\n"
	"	st.global.f32 [%rl8], %f12;\n"
	"	st.global.f32 [%rl8+4], %f14;\n"
	"	st.global.f32 [%rl8+8], %f16;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n"
	// kernel used to combine the photo and both smoothness gradients for all vertices
	".visible .entry CombineAllGradients(\n"
	"	.param .u64 .ptr param_1, // photo-gradient [in/out]\n"
	"	.param .u64 .ptr param_2, // photo-norm [in]\n"
	"	.param .u64 .ptr param_3, // smoothness-gradient 1 [in]\n"
	"	.param .u64 .ptr param_4, // smoothness-gradient 2 [in]\n"
	"	.param .u32 param_5, // numVertices\n"
	"	.param .f32 param_6, // rigidity-weight\n"
	"	.param .f32 param_7 // elasticity-weight\n"
	")\n"
	"{\n"
	"	.reg .f32 %f<19>;\n"
	"	.reg .pred %p<3>;\n"
	"	.reg .s32 %r<17>;\n"
	"	.reg .s64 %rl<15>;\n"
	"\n"
	"	ld.param.u32 %r2, [param_5];\n"
	"	mov.u32 %r3, %ntid.x;\n"
	"	mov.u32 %r4, %ctaid.x;\n"
	"	mov.u32 %r5, %tid.x;\n"
	"	mad.lo.s32 %r1, %r3, %r4, %r5;\n"
	"	setp.lt.s32 %p1, %r1, %r2;\n"
	"	@!%p1 bra BB00_1;\n"
	"\n"
	"	ld.global.f32 %f11, [%rl7];\n"
	"\n"
	"	ld.param.u64 %rl3, [param_4];\n"
	"	cvta.to.global.u64 %rl1, %rl3;\n"
	"	ld.param.u64 %rl4, [param_3];\n"
	"	cvta.to.global.u64 %rl2, %rl4;\n"
	"	ld.param.u64 %rl11, [param_2];\n"
	"	cvta.to.global.u64 %rl12, %rl11;\n"
	"	ld.param.u64 %rl13, [param_1];\n"
	"	cvta.to.global.u64 %rl14, %rl13;\n"
	"	mul.wide.s32 %rl4, %r1, 4;\n"
	"	mul.wide.s32 %rl5, %r1, 12;\n"
	"	add.s64 %rl7, %rl1, %rl5;\n"
	"	add.s64 %rl6, %rl2, %rl5;\n"
	"	add.s64 %rl9, %rl12, %rl4;\n"
	"	add.s64 %rl8, %rl14, %rl5;\n"
	"	ld.param.f32 %f8, [param_6];\n"
	"	ld.param.f32 %f18, [param_7];\n"
	"	ld.global.f32 %f2, [%rl6];\n"
	"	mul.f32 %f3, %f2, %f8;\n"
	"	ld.global.f32 %f12, [%rl7];\n"
	"	fma.rn.f32 %f11, %f12, %f18, %f3;\n"
	"	ld.global.f32 %f4, [%rl6+4];\n"
	"	mul.f32 %f5, %f4, %f8;\n"
	"	ld.global.f32 %f14, [%rl7+4];\n"
	"	fma.rn.f32 %f13, %f14, %f18, %f5;\n"
	"	ld.global.f32 %f6, [%rl6+8];\n"
	"	mul.f32 %f7, %f6, %f8;\n"
	"	ld.global.f32 %f16, [%rl7+8];\n"
	"	fma.rn.f32 %f15, %f16, %f18, %f7;\n"
	"	ld.global.f32 %f9, [%rl9];\n"
	"	setp.gt.f32 %p2, %f9, 0f00000000;\n"
	"	@%p2 bra BB00_2;\n"
	"\n"
	"	st.global.f32 [%rl8], %f11;\n"
	"	st.global.f32 [%rl8+4], %f13;\n"
	"	st.global.f32 [%rl8+8], %f15;\n"
	"	ret;\n"
	"\n"
	"	BB00_2:\n"
	"	rcp.rn.f32 %f10, %f9;\n"
	"	ld.global.f32 %f2, [%rl8];\n"
	"	fma.rn.f32 %f3, %f2, %f10, %f11;\n"
	"	ld.global.f32 %f4, [%rl8+4];\n"
	"	fma.rn.f32 %f5, %f4, %f10, %f13;\n"
	"	ld.global.f32 %f6, [%rl8+8];\n"
	"	fma.rn.f32 %f7, %f6, %f10, %f15;\n"
	"	st.global.f32 [%rl8], %f3;\n"
	"	st.global.f32 [%rl8+4], %f5;\n"
	"	st.global.f32 [%rl8+8], %f7;\n"
	"	BB00_1:\n"
	"	ret;\n"
	"}\n";


// S T R U C T S ///////////////////////////////////////////////////

typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;

class MeshRefineCUDA {
public:
	typedef std::unordered_set<FIndex> CameraFaces;
	typedef SEACAVE::cList<CameraFaces,const CameraFaces&,2> CameraFacesArr;

	// store necessary data about a view
	struct View {
		Image32F imageHost; // store temporarily the image pixels
		Image8U::Size size;
		CUDA::ArrayRT16F image;
		CUDA::MemDevice depthMap;
		CUDA::MemDevice faceMap;
		CUDA::MemDevice baryMap;
		inline View() {}
		inline View(View&) {}
	};
	typedef SEACAVE::cList<View,const View&,2> ViewsArr;

	struct CameraCUDA {
		Matrix3x4f P;
		Matrix3x3f R;
		Point3f C;
		Matrix3x3f K;
		Matrix3x3f invK;
		Image8U::Size size;

		inline CameraCUDA() {}
		inline CameraCUDA(const Camera& camera, const Image8U::Size& _size) : P(camera.P), R(camera.R), C(camera.C), K(camera.K), invK(camera.GetInvK()), size(_size) {}
	};


public:
	MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair=true, float _weightRegularity=1.5f, float _ratioRigidityElasticity=0.8f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8);
	~MeshRefineCUDA();

	bool IsValid() const { return module != NULL && module->IsValid() && !pairs.IsEmpty(); }

	bool InitKernels(int device=-1);
	bool InitImages(float scale, float sigma=0);

	void ListVertexFacesPre();
	void ListVertexFacesPost();
	void ListCameraFaces();

	void ListFaceAreas(Mesh::AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);

	void ComputeNormalFaces();

	void ScoreMesh(float* gradients);

	void ProjectMesh(
		const CameraFaces& cameraFaces,
		const Camera& camera, const Image8U::Size& size, uint32_t idxImage);
	void ProcessPair(uint32_t idxImageA, uint32_t idxImageB);
	void ImageMeshWarp(
		const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
		uint32_t idxImageA, uint32_t idxImageB);
	void ComputeLocalVariance(const CUDA::ArrayRT16F& image, const Image8U::Size& size,
		CUDA::MemDevice& imageMean, CUDA::MemDevice& imageVar);
	void ComputeLocalZNCC(const Image8U::Size& size);
	void ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
		uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale);
	void ComputeSmoothnessGradient(uint32_t numVertices);
	void CombineGradients(uint32_t numVertices);

public:
	const float weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	float ratioRigidityElasticity; // a scalar ratio used to compute the regularity gradient as a combination of rigidity and elasticity
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization
	unsigned nAlternatePair; // using an image pair alternatively as reference image (0 - both, 1 - alternate, 2 - only left, 3 - only right)
	unsigned iteration; // current refinement iteration

	Scene& scene; // the mesh vertices and faces

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	CUDA::ModuleRTPtr module;
	CUDA::KernelRT kernelProjectMesh;
	CUDA::KernelRT kernelCrossCheckProjection;
	CUDA::KernelRT kernelImageMeshWarp;
	CUDA::KernelRT kernelComputeImageMean;
	CUDA::KernelRT kernelComputeImageVar;
	CUDA::KernelRT kernelComputeImageCov;
	CUDA::KernelRT kernelComputeImageZNCC;
	CUDA::KernelRT kernelComputeImageDZNCC;
	CUDA::KernelRT kernelComputePhotometricGradient;
	CUDA::KernelRT kernelUpdatePhotoGradNorm;
	CUDA::KernelRT kernelComputeSmoothnessGradient;
	CUDA::KernelRT kernelCombineGradients;
	CUDA::KernelRT kernelCombineAllGradients;

	CUDA::MemDevice vertices;
	CUDA::MemDevice vertexVertices;
	CUDA::MemDevice faces;
	CUDA::MemDevice faceNormals;
	CUDA::TextureRT16F texImageRef;
	CUDA::SurfaceRT16F surfImageRef;
	CUDA::SurfaceRT16F surfImageProjRef;
	CUDA::MemDevice mask;
	CUDA::MemDevice imageMeanA;
	CUDA::MemDevice imageVarA;
	CUDA::ArrayRT16F imageAB;
	CUDA::MemDevice imageMeanAB;
	CUDA::MemDevice imageVarAB;
	CUDA::MemDevice imageCov;
	CUDA::MemDevice imageZNCC;
	CUDA::MemDevice imageDZNCC;
	CUDA::MemDevice photoGrad;
	CUDA::MemDevice photoGradNorm;
	CUDA::MemDevice photoGradPixels;
	CUDA::MemDevice vertexVerticesCont;
	CUDA::MemDevice vertexVerticesSizes;
	CUDA::MemDevice vertexVerticesPointers;
	CUDA::MemDevice smoothGrad1;
	CUDA::MemDevice smoothGrad2;

	enum { HalfSize = 2 }; // half window size used to compute ZNCC
};

MeshRefineCUDA::MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair, float _weightRegularity, float _ratioRigidityElasticity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews)
	:
	weightRegularity(_weightRegularity),
	ratioRigidityElasticity(_ratioRigidityElasticity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	nAlternatePair(_nAlternatePair),
	scene(_scene),
	images(_scene.images)
{
	if (!InitKernels())
		return;
	// keep only best neighbor views for each image
	std::unordered_set<uint64_t> mapPairs;
	mapPairs.reserve(images.GetSize()*nMaxViews);
	FOREACH(idxImage, images) {
		// keep only best neighbor views
		const float fMinArea(0.1f);
		const float fMinScale(0.2f), fMaxScale(3.2f);
		const float fMinAngle(FD2R(2.5f)), fMaxAngle(FD2R(45.f));
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		ViewScoreArr neighbors(imageData.neighbors);
		Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
		FOREACHPTR(pNeighbor, neighbors) {
			ASSERT(images[pNeighbor->idx.ID].IsValid());
			mapPairs.insert(MakePairIdx((uint32_t)idxImage, pNeighbor->idx.ID));
		}
	}
	pairs.Reserve(mapPairs.size());
	for (uint64_t pair: mapPairs)
		pairs.AddConstruct(pair);
}
MeshRefineCUDA::~MeshRefineCUDA()
{
	scene.mesh.ReleaseExtra();
}

bool MeshRefineCUDA::InitKernels(int device)
{
	STATIC_ASSERT(sizeof(CameraCUDA) == 176);

	// initialize CUDA device if needed
	if (CUDA::devices.IsEmpty() && CUDA::initDevice(device) != CUDA_SUCCESS)
		return false;

	// initialize CUDA kernels
	if (module != NULL && module->IsValid())
		return true;
	module = new CUDA::ModuleRT(g_szMeshRefineModule);
	if (!module->IsValid()) {
		module.Release();
		return false;
	}
	if (kernelProjectMesh.Reset(module, "ProjectMesh") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelProjectMesh.IsValid());
	if (kernelCrossCheckProjection.Reset(module, "CrossCheckProjection") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelCrossCheckProjection.IsValid());
	if (kernelImageMeshWarp.Reset(module, "ImageMeshWarp") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelImageMeshWarp.IsValid());
	if (kernelComputeImageMean.Reset(module, "ComputeImageMean") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeImageMean.IsValid());
	if (kernelComputeImageVar.Reset(module, "ComputeImageVar") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeImageVar.IsValid());
	if (kernelComputeImageCov.Reset(module, "ComputeImageCov") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeImageCov.IsValid());
	if (kernelComputeImageZNCC.Reset(module, "ComputeImageZNCC") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeImageZNCC.IsValid());
	if (kernelComputeImageDZNCC.Reset(module, "ComputeImageDZNCC") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeImageDZNCC.IsValid());
	if (kernelComputePhotometricGradient.Reset(module, "ComputePhotometricGradient") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputePhotometricGradient.IsValid());
	if (kernelUpdatePhotoGradNorm.Reset(module, "UpdatePhotoGradNorm") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelUpdatePhotoGradNorm.IsValid());
	if (kernelComputeSmoothnessGradient.Reset(module, "ComputeSmoothnessGradient") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelComputeSmoothnessGradient.IsValid());
	if (kernelCombineGradients.Reset(module, "CombineGradients") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelCombineGradients.IsValid());
	if (kernelCombineAllGradients.Reset(module, "CombineAllGradients") != CUDA_SUCCESS)
		return false;
	ASSERT(kernelCombineAllGradients.IsValid());

	// init textures
	if (texImageRef.Reset(module, "texImageRef", CU_TR_FILTER_MODE_LINEAR) != CUDA_SUCCESS)
		return false;
	if (surfImageRef.Reset(module, "surfImageRef") != CUDA_SUCCESS)
		return false;
	if (surfImageProjRef.Reset(module, "surfImageProjRef") != CUDA_SUCCESS)
		return false;
	return true;
}

// load and initialize all images at the given scale
// and compute the gradient for each input image
// optional: blur them using the given sigma
bool MeshRefineCUDA::InitImages(float scale, float sigma)
{
	views.Resize(images.GetSize());
	#ifdef MESHCUDAOPT_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for
	for (int_t ID=0; ID<(int_t)images.GetSize(); ++ID) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const uint32_t idxImage((uint32_t)ID);
	#else
	FOREACH(idxImage, images) {
	#endif
		Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		// load and init image
		unsigned level(nResolutionLevel);
		const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
		if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && !imageData.ReloadImage(imageSize)) {
			#ifdef MESHCUDAOPT_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
		View& view = views[idxImage];
		Image32F& img = view.imageHost;
		imageData.image.toGray(img, cv::COLOR_BGR2GRAY, true);
		imageData.image.release();
		if (sigma > 0)
			cv::GaussianBlur(img, img, cv::Size(), sigma);
		if (scale < 1.0) {
			cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_AREA);
			imageData.width = img.width(); imageData.height = img.height();
		}
		imageData.UpdateCamera(scene.platforms);
	}
	#ifdef MESHCUDAOPT_USE_OPENMP
	if (bAbort)
		return false;
	#endif
	// init GPU memory
	Image8U::Size maxSize(0,0);
	FOREACH(idxImage, views) {
		View& view = views[idxImage];
		if (view.imageHost.empty())
			continue;
		Image8U::Size& size(view.size);
		size = view.imageHost.size();
		reportCudaError(view.image.Reset(size, CUDA_ARRAY3D_SURFACE_LDST));
		reportCudaError(view.image.SetData(cvtImage<float,hfloat>(view.imageHost)));
		view.imageHost.release();
		const size_t area((size_t)size.area());
		reportCudaError(view.depthMap.Reset(sizeof(float)*area));
		reportCudaError(view.faceMap.Reset(sizeof(FIndex)*area));
		reportCudaError(view.baryMap.Reset(sizeof(hfloat)*3*area));
		if (maxSize.width < size.width)
			maxSize.width = size.width;
		if (maxSize.height < size.height)
			maxSize.height = size.height;
	}
	const size_t area(maxSize.area());
	reportCudaError(mask.Reset(sizeof(uint8_t)*area));
	reportCudaError(imageMeanA.Reset(sizeof(float)*area));
	reportCudaError(imageVarA.Reset(sizeof(float)*area));
	reportCudaError(imageAB.Reset(maxSize, CUDA_ARRAY3D_SURFACE_LDST));
	reportCudaError(imageMeanAB.Reset(sizeof(float)*area));
	reportCudaError(imageVarAB.Reset(sizeof(float)*area));
	reportCudaError(imageCov.Reset(sizeof(float)*area));
	reportCudaError(imageZNCC.Reset(sizeof(float)*area));
	reportCudaError(imageDZNCC.Reset(sizeof(float)*area));
	surfImageProjRef.Bind(imageAB);
	iteration = 0;
	return true;
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshRefineCUDA::ListVertexFacesPre()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidenteFaces();
	reportCudaError(faces.Reset(scene.mesh.faces));
}
void MeshRefineCUDA::ListVertexFacesPost()
{
	scene.mesh.ListIncidenteVertices();
	scene.mesh.ListBoundaryVertices();
	ASSERT(!scene.mesh.vertices.IsEmpty() && scene.mesh.vertices.GetSize() == scene.mesh.vertexVertices.GetSize());
	// set vertex vertices
	reportCudaError(vertexVertices.Reset(scene.mesh.vertexVertices));
	// list adjacent vertices for each vertex
	const size_t numVertices(scene.mesh.vertices.GetSize());
	Unsigned32Arr _vertexVerticesCont(0, numVertices*6);
	Unsigned32Arr _vertexVerticesSizes(0, numVertices);
	Unsigned32Arr _vertexVerticesPointers(0, numVertices);
	uint32_t lastPosition(0);
	FOREACH(idxV, scene.mesh.vertices) {
		if (scene.mesh.vertexBoundary[idxV]) {
			_vertexVerticesSizes.Insert(0);
			_vertexVerticesPointers.Insert(lastPosition);
			continue;
		}
		const Mesh::VertexIdxArr& verts = scene.mesh.vertexVertices[idxV];
		_vertexVerticesCont.Join(verts.GetData(), verts.GetSize());
		_vertexVerticesSizes.Insert(verts.GetSize());
		_vertexVerticesPointers.Insert(lastPosition); lastPosition += verts.GetSize();
	}
	reportCudaError(vertexVerticesCont.Reset(_vertexVerticesCont));
	reportCudaError(vertexVerticesSizes.Reset(_vertexVerticesSizes));
	reportCudaError(vertexVerticesPointers.Reset(_vertexVerticesPointers));
	// init memory
	reportCudaError(photoGrad.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(photoGradNorm.Reset(sizeof(float)*numVertices));
	reportCudaError(photoGradPixels.Reset(sizeof(float)*numVertices));
	reportCudaError(smoothGrad1.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(smoothGrad2.Reset(sizeof(Point3f)*numVertices));
}

// extract array of faces viewed by each image
void MeshRefineCUDA::ListCameraFaces()
{
	// extract array of faces viewed by each camera
	CameraFacesArr arrCameraFaces(images.GetSize());
	{
	struct FacesInserter {
		FacesInserter(const Mesh::VertexFacesArr& _vertexFaces, CameraFaces& _cameraFaces)
			: vertexFaces(_vertexFaces), cameraFaces(_cameraFaces) {}
		inline void operator() (IDX idxVertex) {
			const Mesh::FaceIdxArr& vertexTris = vertexFaces[idxVertex];
			FOREACHPTR(pTri, vertexTris)
				cameraFaces.emplace(*pTri);
		}
		inline void operator() (const IDX* idices, size_t size) {
			FOREACHRAWPTR(pIdxVertex, idices, size)
				operator()(*pIdxVertex);
		}
		const Mesh::VertexFacesArr& vertexFaces;
		CameraFaces& cameraFaces;
	};
	typedef TOctree<Mesh::VertexArr,float,3> Octree;
	const Octree octree(scene.mesh.vertices);
	#if 0 && !defined(_RELEASE)
	Octree::DEBUGINFO_TYPE info;
	octree.GetDebugInfo(&info);
	Octree::LogDebugInfo(info);
	#endif
	FOREACH(ID, images) {
		const Image& imageData = images[ID];
		if (!imageData.IsValid())
			continue;
		typedef TFrustum<float,5> Frustum;
		FacesInserter inserter(scene.mesh.vertexFaces, arrCameraFaces[ID]);
		const Frustum frustum(Frustum::MATRIX3x4(((PMatrix::CEMatMap)imageData.camera.P).cast<float>()), (float)imageData.width, (float)imageData.height);
		octree.Traverse(frustum, inserter);
	}
	}

	// project mesh to each camera plane
	reportCudaError(vertices.Reset(scene.mesh.vertices));
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (imageData.IsValid())
			ProjectMesh(arrCameraFaces[idxImage], imageData.camera, views[idxImage].size, idxImage);
	}
}

// compute for each face the projection area as the maximum area in both images of a pair
// (make sure ListCameraFaces() was called before)
void MeshRefineCUDA::ListFaceAreas(Mesh::AreaArr& maxAreas)
{
	ASSERT(maxAreas.IsEmpty());
	// for each image, compute the projection area of visible faces
	typedef cList<Mesh::AreaArr> ImageAreaArr;
	ImageAreaArr viewAreas(images.GetSize());
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		Mesh::AreaArr& areas = viewAreas[idxImage];
		areas.Resize(scene.mesh.faces.GetSize());
		areas.Memset(0);
		// get faceMap from the GPU memory
		TImage<FIndex> faceMap(imageData.height, imageData.width);
		views[idxImage].faceMap.GetData(faceMap);
		// compute area covered by all vertices (incident faces) viewed by this image
		for (int j=0; j<faceMap.rows; ++j) {
			for (int i=0; i<faceMap.cols; ++i) {
				const FIndex idxFace(faceMap(j,i));
				if (idxFace == NO_ID)
					continue;
				++areas[idxFace];
			}
		}
	}
	// for each pair, mark the faces that have big projection areas in both images
	maxAreas.Resize(scene.mesh.faces.GetSize());
	maxAreas.Memset(0);
	FOREACHPTR(pPair, pairs) {
		const Mesh::AreaArr& areasA = viewAreas[pPair->i];
		const Mesh::AreaArr& areasB = viewAreas[pPair->j];
		ASSERT(areasA.GetSize() == areasB.GetSize());
		FOREACH(f, areasA) {
			const uint16_t minArea(MINF(areasA[f], areasB[f]));
			uint16_t& maxArea = maxAreas[f];
			if (maxArea < minArea)
				maxArea = minArea;
		}
	}
}

// decimate or subdivide mesh such that for each face there is no image pair in which
// its projection area is bigger than the given number of pixels in both images
void MeshRefineCUDA::SubdivideMesh(uint32_t maxArea, float fDecimate, unsigned nCloseHoles, unsigned nEnsureEdgeSize)
{
	Mesh::AreaArr maxAreas;

	// first decimate if necessary
	const bool bNoDecimation(fDecimate >= 1.f);
	const bool bNoSimplification(maxArea == 0);
	if (!bNoDecimation) {
		if (fDecimate > 0.f) {
			// decimate to the desired resolution
			scene.mesh.Clean(fDecimate, 0.f, false, nCloseHoles, 0, false);
			scene.mesh.Clean(1.f, 0.f, false, nCloseHoles, 0, true);

			#ifdef MESHOPT_ENSUREEDGESIZE
			// make sure there are no edges too small or too long
			if (nEnsureEdgeSize > 0 && bNoSimplification) {
				scene.mesh.EnsureEdgeSize();
				scene.mesh.Clean(1.f, 0.f, false, nCloseHoles, 0, true);
			}
			#endif

			// re-map vertex and camera faces
			ListVertexFacesPre();
		} else {
			// extract array of faces viewed by each camera
			ListCameraFaces();

			// estimate the faces' area that have big projection areas in both images of a pair
			ListFaceAreas(maxAreas);
			ASSERT(!maxAreas.IsEmpty());

			const float maxArea((float)(maxArea > 0 ? maxArea : 64));
			const float medianArea(6.f*(float)Mesh::AreaArr(maxAreas).GetMedian());
			if (medianArea < maxArea) {
				maxAreas.Empty();

				// decimate to the auto detected resolution
				scene.mesh.Clean(MAXF(0.1f, medianArea/maxArea), 0.f, false, nCloseHoles, 0, false);
				scene.mesh.Clean(1.f, 0.f, false, nCloseHoles, 0, true);

				#ifdef MESHOPT_ENSUREEDGESIZE
				// make sure there are no edges too small or too long
				if (nEnsureEdgeSize > 0 && bNoSimplification) {
					scene.mesh.EnsureEdgeSize();
					scene.mesh.Clean(1.f, 0.f, false, nCloseHoles, 0, true);
				}
				#endif

				// re-map vertex and camera faces
				ListVertexFacesPre();
			}
		}
	}
	if (bNoSimplification)
		return;

	if (maxAreas.IsEmpty()) {
		// extract array of faces viewed by each camera
		ListCameraFaces();

		// estimate the faces' area that have big projection areas in both images of a pair
		ListFaceAreas(maxAreas);
	}

	// subdivide mesh faces if its projection area is bigger than the given number of pixels
	const size_t numVertsOld(scene.mesh.vertices.GetSize());
	const size_t numFacesOld(scene.mesh.faces.GetSize());
	scene.mesh.Subdivide(maxAreas, maxArea);

	#ifdef MESHOPT_ENSUREEDGESIZE
	// make sure there are no edges too small or too long
	#if MESHOPT_ENSUREEDGESIZE==1
	if ((nEnsureEdgeSize == 1 && !bNoDecimation) || nEnsureEdgeSize > 1)
	#endif
	{
		scene.mesh.EnsureEdgeSize();
		scene.mesh.Clean(1.f, 0.f, false, nCloseHoles, 0, true);
	}
	#endif

	// re-map vertex and camera faces
	ListVertexFacesPre();

	DEBUG_EXTRA("Mesh subdivided: %u/%u -> %u/%u vertices/faces", numVertsOld, numFacesOld, scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize());

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 3)
		scene.mesh.Save(MAKE_PATH("MeshSubdivided.ply"));
	#endif
}


// compute face normals
void MeshRefineCUDA::ComputeNormalFaces()
{
	const FIndex numFaces(scene.mesh.faces.GetSize());
	reportCudaError(faceNormals.Reset(sizeof(Point3f)*numFaces));
	reportCudaError(Mesh::kernelComputeFaceNormal((int)numFaces,
		vertices,
		faces,
		faceNormals,
		numFaces
	));
}


// score mesh using photo-consistency
// and compute vertices gradient using analytical method
void MeshRefineCUDA::ScoreMesh(float* gradients)
{
	// extract array of faces viewed by each camera
	ListCameraFaces();

	// compute face normals
	ComputeNormalFaces();

	// init memory
	const VIndex numVertices(scene.mesh.vertices.GetSize());
	reportCudaError(cuMemsetD32(photoGrad, 0, numVertices*3));
	reportCudaError(cuMemsetD32(photoGradNorm, 0, numVertices));

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		switch (nAlternatePair) {
		case 1: {
			const PairIdx pair(iteration%2 ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
			ProcessPair(pair.i, pair.j);
			break; }
		case 2: {
			ProcessPair(pPair->i, pPair->j);
			break; }
		case 3: {
			ProcessPair(pPair->j, pPair->i);
			break; }
		default:
			for (int ip=0; ip<2; ++ip) {
				const PairIdx pair(ip ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
				ProcessPair(pair.i, pair.j);
			}
		}
	}

	// loop through all vertices and compute the smoothing score
	ComputeSmoothnessGradient(numVertices);

	// set the final gradient as the combination of photometric and smoothness gradients
	CombineGradients(numVertices);
	reportCudaError(photoGrad.GetData(gradients, sizeof(Point3f)*numVertices));
}


// project mesh to the given camera plane
void MeshRefineCUDA::ProjectMesh(
	const CameraFaces& cameraFaces,
	const Camera& camera, const Image8U::Size& size, uint32_t idxImage)
{
	View& view = views[idxImage];
	// init depth-map
	const float fltMax(FLT_MAX);
	reportCudaError(cuMemsetD32(view.depthMap, (uint32_t&)fltMax, size.area()));
	// fetch only the faces viewed by this camera
	Mesh::FaceIdxArr faceIDsView(0, (FIndex)cameraFaces.size());
	for (auto idxFace : cameraFaces)
		faceIDsView.Insert(idxFace);
	// project mesh
	reportCudaError(kernelProjectMesh((int)faceIDsView.GetSize(),
		vertices,
		faces,
		faceIDsView,
		view.depthMap,
		view.faceMap,
		view.baryMap,
		CameraCUDA(camera, size),
		faceIDsView.GetSize()
	));
	kernelProjectMesh.Reset();
	// cross-check valid depth and face index
	reportCudaError(kernelCrossCheckProjection(size,
		view.depthMap,
		view.faceMap,
		size.width, size.height
	));
	#if 0
	// debug view
	DepthMap depthMap(size);
	TImage<FIndex> faceMap(size);
	TImage<Point3hf> baryMap(size);
	view.depthMap.GetData(depthMap);
	view.faceMap.GetData(faceMap);
	view.baryMap.GetData(baryMap);
	TImage<Point3f> _baryMap(cvtImage<Point3hf,Point3f>(baryMap));
	#endif
}

void MeshRefineCUDA::ProcessPair(uint32_t idxImageA, uint32_t idxImageB)
{
	// fetch view A data
	const Image& imageDataA = images[idxImageA];
	ASSERT(imageDataA.IsValid());
	const Camera& cameraA = imageDataA.camera;
	const Image8U::Size& sizeA(views[idxImageA].size);
	// fetch view B data
	const Image& imageDataB = images[idxImageB];
	ASSERT(imageDataB.IsValid());
	const Camera& cameraB = imageDataB.camera;
	// warp imageB to imageA using the mesh
	ImageMeshWarp(cameraA, cameraB, sizeA, idxImageA, idxImageB);
	// init vertex textures
	ComputeLocalVariance(imageAB, sizeA, imageMeanAB, imageVarAB);
	ComputeLocalVariance(views[idxImageA].image, sizeA, imageMeanA, imageVarA);
	ComputeLocalZNCC(sizeA);
	const float RegularizationScale((float)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(cameraA.GetFocalLength()*cameraB.GetFocalLength())));
	ComputePhotometricGradient(cameraA, cameraB, sizeA, idxImageA, idxImageB, scene.mesh.vertices.GetSize(), RegularizationScale);
}

// project image from view B to view A through the mesh;
// the projected image is stored in imageA
void MeshRefineCUDA::ImageMeshWarp(
	const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB)
{
	// set image texture
	surfImageRef.Bind(views[idxImageA].image);
	texImageRef.Bind(views[idxImageB].image);
	// project image
	reportCudaError(kernelImageMeshWarp(size,
		views[idxImageA].depthMap,
		views[idxImageB].depthMap,
		mask,
		CameraCUDA(cameraA, size),
		CameraCUDA(cameraB, size)
	));
	#if 0
	// debug view
	Image16F _imageAB(size);
	Image8U _mask(size);
	imageAB.GetData(_imageAB);
	mask.GetData(_mask);
	Image32F __imageAB(cvtImage<hfloat,float>(_imageAB));
	#endif
}

// compute local variance for each image pixel
void MeshRefineCUDA::ComputeLocalVariance(const CUDA::ArrayRT16F& image, const Image8U::Size& size,
	CUDA::MemDevice& imageMean, CUDA::MemDevice& imageVar)
{
	surfImageRef.Bind(image);
	reportCudaError(kernelComputeImageMean(size,
		mask,
		imageMean,
		size.width, size.height,
		HalfSize
	));
	reportCudaError(kernelComputeImageVar(size,
		imageMean,
		mask,
		imageVar,
		size.width, size.height,
		HalfSize
	));
	#if 0
	// debug view
	Image32F mean(size);
	Image32F var(size);
	imageMean.GetData(mean);
	imageVar.GetData(var);
	#endif
}

// compute local ZNCC and its gradient for each image pixel
void MeshRefineCUDA::ComputeLocalZNCC(const Image8U::Size& size)
{
	reportCudaError(kernelComputeImageCov(size,
		imageMeanA,
		imageMeanAB,
		mask,
		imageCov,
		size.width, size.height,
		HalfSize
	));
	reportCudaError(kernelComputeImageZNCC(size,
		imageCov,
		imageVarA,
		imageVarAB,
		mask,
		imageZNCC,
		size.width, size.height,
		HalfSize
	));
	reportCudaError(kernelComputeImageDZNCC(size,
		imageMeanA,
		imageMeanAB,
		imageVarA,
		imageVarAB,
		imageZNCC,
		mask,
		imageDZNCC,
		size.width, size.height,
		HalfSize
	));
	#if 0
	// debug view
	Image32F _imageZNCC(size);
	Image32F _imageDZNCC(size);
	imageZNCC.GetData(_imageZNCC);
	imageDZNCC.GetData(_imageDZNCC);
	#endif
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefineCUDA::ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale)
{
	// compute photometric gradient for all visible vertices
	reportCudaError(cuMemsetD32(photoGradPixels, 0, numVertices));
	reportCudaError(kernelComputePhotometricGradient(size,
		faces, faceNormals,
		views[idxImageA].depthMap,
		views[idxImageA].faceMap,
		views[idxImageA].baryMap,
		imageDZNCC,
		mask,
		photoGrad, photoGradPixels,
		CameraCUDA(cameraA, size),
		CameraCUDA(cameraB, size),
		RegularizationScale
	));
	// update photometric gradient norm for all visible vertices
	reportCudaError(kernelUpdatePhotoGradNorm(numVertices,
		photoGradNorm, photoGradPixels,
		numVertices
	));
	#if 0
	// debug view
	Point3fArr _photoGrad(numVertices);
	FloatArr _photoGradPixels(numVertices);
	FloatArr _photoGradNorm(numVertices);
	photoGrad.GetData(_photoGrad);
	photoGradPixels.GetData(_photoGradPixels);
	photoGradNorm.GetData(_photoGradNorm);
	#endif
}

void MeshRefineCUDA::ComputeSmoothnessGradient(uint32_t numVertices)
{
	// compute smoothness gradient for all vertices
	reportCudaError(kernelComputeSmoothnessGradient((int)numVertices,
		vertices,
		vertexVerticesCont,
		vertexVerticesSizes,
		vertexVerticesPointers,
		smoothGrad1,
		numVertices,
		uint8_t(0)
	));
	reportCudaError(kernelComputeSmoothnessGradient((int)numVertices,
		smoothGrad1,
		vertexVerticesCont,
		vertexVerticesSizes,
		vertexVerticesPointers,
		smoothGrad2,
		numVertices,
		uint8_t(1)
	));
	#if 0		
	// debug view
	Point3fArr _smoothGrad1(numVertices);
	Point3fArr _smoothGrad2(numVertices);
	smoothGrad1.GetData(_smoothGrad1);
	smoothGrad2.GetData(_smoothGrad2);
	#endif
}

void MeshRefineCUDA::CombineGradients(uint32_t numVertices)
{
	// compute smoothness gradient for all vertices
	if (ratioRigidityElasticity >= 1.f) {
		reportCudaError(kernelCombineGradients((int)numVertices,
			photoGrad,
			photoGradNorm,
			smoothGrad2,
			numVertices,
			weightRegularity
		));
	} else {
		// compute smoothing gradient as a combination of level 1 and 2 of the Laplacian operator;
		// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
		const float rigidity((1.f-ratioRigidityElasticity)*weightRegularity);
		const float elasticity(ratioRigidityElasticity*weightRegularity);
		reportCudaError(kernelCombineAllGradients((int)numVertices,
			photoGrad,
			photoGradNorm,
			smoothGrad1,
			smoothGrad2,
			numVertices,
			rigidity,
			elasticity
		));
	}
	#if 0		
	// debug view
	Point3fArr _photoGrad(numVertices);
	photoGrad.GetData(_photoGrad);
	#endif
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

// optimize mesh using photo-consistency
bool Scene::RefineMeshCUDA(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews,
						   float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea,
						   unsigned nScales, float fScaleStep, unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fGradientStep)
{
	MeshRefineCUDA refine(*this, nAlternatePair, fRegularityWeight, fRatioRigidityElasticity, nResolutionLevel, nMinResolution, nMaxViews);
	if (!refine.IsValid())
		return false;

	// run the mesh optimization on multiple scales (coarse to fine)
	for (unsigned nScale=0; nScale<nScales; ++nScale) {
		// init images
		const float scale(POWI(fScaleStep, (int)(nScales-nScale-1)));
		const float step(POWI(2.f, (int)(nScales-nScale)));
		DEBUG_ULTIMATE("Refine mesh at: %.2f image scale", scale);
		if (!refine.InitImages(scale, 0.12f*step+0.2f))
			return false;

		// extract array of triangles incident to each vertex
		refine.ListVertexFacesPre();

		// automatic mesh subdivision
		refine.SubdivideMesh(nMaxFaceArea, nScale == 0 ? fDecimateMesh : 1.f, nCloseHoles, nEnsureEdgeSize);

		// extract array of triangle normals
		refine.ListVertexFacesPost();

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefine%u.ply", nScales-nScale-1)));
		#endif

		// loop a constant number of iterations and apply the gradient
		int iters(25);
		float gstep(0.05f);
		if (fGradientStep > 1) {
			iters = FLOOR2INT(fGradientStep);
			gstep = (fGradientStep-(float)iters)*10;
		}
		iters = MAXF(iters/(int)(nScale+1),8);
		const int iterStop(iters*7/10);
		Eigen::Matrix<float,Eigen::Dynamic,3,Eigen::RowMajor> gradients(mesh.vertices.GetSize(),3);
		Util::Progress progress(_T("Processed iterations"), iters);
		GET_LOGCONSOLE().Pause();
		for (int iter=0; iter<iters; ++iter) {
			refine.iteration = (unsigned)iter;
			refine.nAlternatePair = (iter+1 < iters ? nAlternatePair : 0);
			refine.ratioRigidityElasticity = (iter <= iterStop ? fRatioRigidityElasticity : 1.f);
			// evaluate residuals and gradients
			refine.ScoreMesh(gradients.data());
			// apply gradients
			float gv(0);
			FOREACH(v, mesh.vertices) {
				Vertex& vert = mesh.vertices[v];
				const Point3f grad(gradients.row(v));
				if (!ISFINITE(grad))
					continue;
				vert -= Vertex(grad*gstep);
				gv += norm(grad);
			}
			DEBUG_EXTRA("\t%2d. g: %.5f (%.3e - %.3e)\ts: %.3f", iter+1, gradients.norm(), gradients.norm()/mesh.vertices.GetSize(), gv/mesh.vertices.GetSize(), gstep);
			gstep *= 0.98f;
			progress.display(iter);
		}
		GET_LOGCONSOLE().Play();
		progress.close();

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-nScale-1)));
		#endif
	}

	return true;
} // RefineMeshCUDA
/*----------------------------------------------------------------*/

#endif // _USE_CUDA
