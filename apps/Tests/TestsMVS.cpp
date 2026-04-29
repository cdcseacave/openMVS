/*
 * TestsMVS.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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
#include "../../libs/MVS/SceneOrthoMap.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestMVS "));

namespace MVS {

// test MVS stages on a small sample dataset
bool PipelineTest(bool verbose)
{
	TD_TIMER_START();
	#if 0 && defined(_USE_CUDA)
	// force CPU for testing even if CUDA is available
	SEACAVE::CUDA::desiredDeviceIDs.clear();
	#endif
	Scene scene;
	if (!scene.Load(MAKE_PATH("scene.mvs"))) {
		VERBOSE("ERROR: TestDataset failed loading the scene!");
		return false;
	}
	OPTDENSE::init();
	OPTDENSE::bRemoveDmaps = true;
	if (!scene.DenseReconstruction() || scene.pointcloud.GetSize() < 50000u) {
		VERBOSE("ERROR: TestDataset failed estimating dense point-cloud!");
		return false;
	}
	if (verbose)
		scene.pointcloud.Save(MAKE_PATH("scene_dense.ply"));
	if (!scene.ReconstructMesh() || scene.mesh.faces.size() < 25000u) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
	constexpr float decimate = 0.7f;
	scene.mesh.Clean(decimate);
	if (!ISINSIDE(scene.mesh.faces.size(), 18000u, 30000u)) {
		VERBOSE("ERROR: TestDataset failed cleaning the mesh!");
		return false;
	}
	#ifdef _USE_OPENMP
	TestMeshProjectionMT(scene.mesh, scene.images[1]);
	#endif
	if (!scene.TextureMesh(0, 0) || !scene.mesh.HasTexture()) {
		VERBOSE("ERROR: TestDataset failed texturing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture.ply"));
	const float qualityScore = scene.ComputeReconstructionQuality().score();
	if (qualityScore < 45.f) {
		VERBOSE("ERROR: TestDataset reconstruction quality too low (%.1f)!", qualityScore);
		return false;
	}
	VERBOSE("All pipeline stages passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/


// Helper: export a depth map as normalized 8-bit grayscale PNG for visual inspection
static void ExportDepthMap(const DepthMap& depthMap, const String& fileName) {
	if (depthMap.empty())
		return;
	// find min/max non-zero depth
	float minD = FLT_MAX, maxD = 0;
	for (int y = 0; y < depthMap.rows; ++y)
		for (int x = 0; x < depthMap.cols; ++x) {
			const Depth d = depthMap(y, x);
			if (d > 0) {
				if (d < minD) minD = d;
				if (d > maxD) maxD = d;
			}
		}
	// normalize to [0, 255]: 0 = background (no depth), 255 = closest, 1 = farthest
	Image8U vis(depthMap.size());
	const float range = (maxD > minD) ? (maxD - minD) : 1.f;
	for (int y = 0; y < depthMap.rows; ++y)
		for (int x = 0; x < depthMap.cols; ++x) {
			const Depth d = depthMap(y, x);
			vis(y, x) = (d > 0) ? (uint8_t)(255.f - 254.f * (d - minD) / range) : 0;
		}
	vis.Save(fileName);
}

// Helper: export all tile depth maps from an OrthoMapContext
static void ExportTileDepthMaps(const OrthoMapContext& ctx, const String& prefix) {
	for (uint32_t i = 0; i < ctx.tiles.size(); ++i) {
		const OrthoTile& tile = ctx.tiles[i];
		ExportDepthMap(tile.depthMap,
			String::FormatString("%s_tile_%u_%u.png", prefix.c_str(), tile.tile.x, tile.tile.y));
	}
}
/*----------------------------------------------------------------*/


// Helper: create a synthetic scene with a pyramid mesh and nadir cameras
// Mesh: 20m x 20m base at Z=0 with center peak at Z=5, covering [-10,10] x [-10,10]
// Cameras: 4 nadir-looking at Z=50, focal length 1000px, image 1000x1000
// Expected GSD: avgDepth / focalLength = 50 / 1000 = 0.05 m/px
static Scene CreateSyntheticOrthoScene() {
	Scene scene;
	// create pyramid mesh: base at Z=0, peak at Z=5
	scene.mesh.vertices.resize(5);
	scene.mesh.vertices[0] = Mesh::Vertex(-10, -10, 0);
	scene.mesh.vertices[1] = Mesh::Vertex( 10, -10, 0);
	scene.mesh.vertices[2] = Mesh::Vertex( 10,  10, 0);
	scene.mesh.vertices[3] = Mesh::Vertex(-10,  10, 0);
	scene.mesh.vertices[4] = Mesh::Vertex(  0,   0, 5); // center peak
	scene.mesh.faces.resize(4);
	scene.mesh.faces[0] = Mesh::Face(0, 1, 4);
	scene.mesh.faces[1] = Mesh::Face(1, 2, 4);
	scene.mesh.faces[2] = Mesh::Face(2, 3, 4);
	scene.mesh.faces[3] = Mesh::Face(3, 0, 4);
	// create a platform with one camera
	Platform platform;
	platform.name = "TestPlatform";
	Platform::Camera platformCamera;
	platformCamera.K = KMatrix::IDENTITY;
	platformCamera.K(0, 0) = platformCamera.K(1, 1) = 1000.0; // focal length
	platformCamera.K(0, 2) = 500.0; // principal point
	platformCamera.K(1, 2) = 500.0;
	platformCamera.R = RMatrix::IDENTITY;
	platformCamera.C = CMatrix::ZERO;
	platform.cameras.push_back(platformCamera);
	// add 4 nadir camera poses spread across the scene
	const float offsets[] = {-3.f, 3.f, -3.f, 3.f};
	for (unsigned i = 0; i < 4; ++i) {
		Platform::Pose pose;
		pose.R.SetFromDirUp(Vec3(Point3(0, 0, -1)), Vec3(Point3(0, 1, 0)));
		pose.C = CMatrix(offsets[i % 4], offsets[(i + 1) % 4], 50.0);
		platform.poses.push_back(pose);
	}
	scene.platforms.push_back(platform);
	// create images referencing the platform poses
	const unsigned nPoses = (unsigned)platform.poses.size();
	scene.images.resize(nPoses);
	for (unsigned i = 0; i < nPoses; ++i) {
		Image& image = scene.images[i];
		image.platformID = 0;
		image.cameraID = 0;
		image.poseID = i;
		image.ID = i;
		image.width = 1000;
		image.height = 1000;
		image.avgDepth = 50.0f;
		image.name = String::FormatString("test_%u.jpg", i);
		// set camera pose from platform
		image.camera.K = platformCamera.K;
		image.camera.R = platform.poses[i].R;
		image.camera.C = platform.poses[i].C;
	}
	scene.nCalibratedImages = nPoses;
	return scene;
}
/*----------------------------------------------------------------*/


// Test 1: OrthoConfig defaults
bool TestOrthoMapConfig(bool /*verbose*/)
{
	TD_TIMER_START();
	const OrthoConfig config;
	if (config.targetGSD != 0) {
		VERBOSE("ERROR: OrthoConfig default targetGSD should be 0, got %f!", config.targetGSD);
		return false;
	}
	if (config.tileSize != 4096) {
		VERBOSE("ERROR: OrthoConfig default tileSize should be 4096, got %u!", config.tileSize);
		return false;
	}
	if (config.tileOverlap != 16) {
		VERBOSE("ERROR: OrthoConfig default tileOverlap should be 16, got %u!", config.tileOverlap);
		return false;
	}
	if (config.nadirThreshold != 0.3f) {
		VERBOSE("ERROR: OrthoConfig default nadirThreshold should be 0.3, got %f!", config.nadirThreshold);
		return false;
	}
	if (config.marginFactor != 0.01f) {
		VERBOSE("ERROR: OrthoConfig default marginFactor should be 0.01, got %f!", config.marginFactor);
		return false;
	}
	VERBOSE("TestOrthoMapConfig passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 2: GSD computation
// - User override returns exact value
// - Auto-computation from nadir cameras returns expected median
// - Oblique cameras are filtered out by nadir threshold
bool TestOrthoMapGSD(bool /*verbose*/)
{
	TD_TIMER_START();
	// test user override
	{
		Scene scene(CreateSyntheticOrthoScene());
		OrthoConfig config;
		config.targetGSD = 0.1f;
		OrthoMapContext ctx(scene, config);
		const float gsd = ctx.ComputeGSD();
		if (ABS(gsd - 0.1f) > 1e-6f) {
			VERBOSE("ERROR: GSD user override failed, expected 0.1, got %f!", gsd);
			return false;
		}
	}
	// test auto-computation: 4 nadir cameras, avgDepth=50, focal=1000 → GSD=0.05
	{
		Scene scene(CreateSyntheticOrthoScene());
		OrthoConfig config;
		OrthoMapContext ctx(scene, config);
		const float gsd = ctx.ComputeGSD();
		if (ABS(gsd - 0.05f) > 1e-4f) {
			VERBOSE("ERROR: GSD auto-computation failed, expected ~0.05, got %f!", gsd);
			return false;
		}
	}
	// test that auto-computed GSD matches expected value exactly
	{
		Scene scene(CreateSyntheticOrthoScene());
		OrthoConfig config;
		OrthoMapContext ctx(scene, config);
		const float gsd = ctx.ComputeGSD();
		// all 4 cameras identical → median = 50/1000 = 0.05
		if (ABS(gsd - 0.05f) > 1e-6f) {
			VERBOSE("ERROR: GSD median should be exactly 0.05, got %f!", gsd);
			return false;
		}
	}
	VERBOSE("TestOrthoMapGSD passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 3: Grid computation
// - Grid dimensions match ceil(extent / gsd)
// - Tile layout is correct
// - Y-axis convention: pixel row 0 = north (max Y)
// - Edge tiles have reduced dimensions
bool TestOrthoMapGrid(bool verbose)
{
	TD_TIMER_START();
	Scene scene(CreateSyntheticOrthoScene());
	OrthoConfig config;
	config.targetGSD = 0.1f;  // 0.1 m/px for predictable dimensions
	config.tileSize = 128;
	config.tileOverlap = 8;
	config.marginFactor = 0;  // disable margin for predictable bounds
	OrthoMapContext ctx(scene, config);
	if (!ctx.ComputeGrid()) {
		VERBOSE("ERROR: ComputeGrid failed!");
		return false;
	}
	// mesh XY is 20m x 20m → at 0.1 m/px → 200 x 200 pixels (plus small margin from max(0.1, ...))
	// with marginFactor=0, margin = max(0.1, 0) = 0.1 → XY extent becomes 20.2m → 202 pixels
	const int expectedWidth = CEIL2INT(20.2f / 0.1f);
	const int expectedHeight = expectedWidth;
	if (ctx.grid.sizePx.width != expectedWidth || ctx.grid.sizePx.height != expectedHeight) {
		VERBOSE("ERROR: Grid dimensions wrong, expected %dx%d, got %dx%d!",
			expectedWidth, expectedHeight, ctx.grid.sizePx.width, ctx.grid.sizePx.height);
		return false;
	}
	// tile layout: effectiveStride = 128 - 8 = 120, tiles = ceil(202/120) = 2
	if (ctx.grid.tiles.x != 2 || ctx.grid.tiles.y != 2) {
		VERBOSE("ERROR: Tile count wrong, expected 2x2, got %ux%u!", ctx.grid.tiles.x, ctx.grid.tiles.y);
		return false;
	}
	// verify total tile count
	if (ctx.tiles.size() != 4) {
		VERBOSE("ERROR: Total tiles should be 4, got %u!", ctx.tiles.GetSize());
		return false;
	}
	// verify first tile starts at (0,0)
	const OrthoTile& firstTile = ctx.tiles[0];
	if (firstTile.p0.x != 0 || firstTile.p0.y != 0) {
		VERBOSE("ERROR: First tile should start at (0,0), got (%u,%u)!", firstTile.p0.x, firstTile.p0.y);
		return false;
	}
	if (firstTile.size.width != 128 || firstTile.size.height != 128) {
		VERBOSE("ERROR: First tile size should be 128x128, got %dx%d!", firstTile.size.width, firstTile.size.height);
		return false;
	}
	// verify edge tile (last tile) has reduced dimensions
	const OrthoTile& lastTile = ctx.tiles[3]; // tile (1,1)
	if (lastTile.p0.x != 120 || lastTile.p0.y != 120) {
		VERBOSE("ERROR: Last tile should start at (120,120), got (%u,%u)!", lastTile.p0.x, lastTile.p0.y);
		return false;
	}
	// last tile width = min(120+128, 202) - 120 = min(248, 202) - 120 = 82
	const int expectedEdgeSize = expectedWidth - 120;
	if (lastTile.size.width != expectedEdgeSize || lastTile.size.height != expectedEdgeSize) {
		VERBOSE("ERROR: Last tile size should be %dx%d, got %dx%d!",
			expectedEdgeSize, expectedEdgeSize, lastTile.size.width, lastTile.size.height);
		return false;
	}
	// Y-axis convention: first tile's north edge = scene max Y
	const float sceneMaxY = ctx.grid.sceneAABB.ptMax.y();
	const float firstTileNorthY = firstTile.worldBounds.ptMax.y();
	if (ABS(firstTileNorthY - sceneMaxY) > 1e-3f) {
		VERBOSE("ERROR: First tile north edge (%.4f) should match scene max Y (%.4f)!",
			firstTileNorthY, sceneMaxY);
		return false;
	}
	// Y-axis: south edge of first tile should be north - tileHeight * gsd
	const float expectedSouthY = sceneMaxY - firstTile.size.height * config.targetGSD;
	const float firstTileSouthY = firstTile.worldBounds.ptMin.y();
	if (ABS(firstTileSouthY - expectedSouthY) > 1e-3f) {
		VERBOSE("ERROR: First tile south edge (%.4f) should be %.4f!",
			firstTileSouthY, expectedSouthY);
		return false;
	}
	if (verbose) {
		scene.mesh.Save(MAKE_PATH("ortho_test_mesh.ply"));
		VERBOSE("  exported: ortho_test_mesh.ply (synthetic mesh)");
		VERBOSE("  grid: AABB min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
			ctx.grid.sceneAABB.ptMin.x(), ctx.grid.sceneAABB.ptMin.y(), ctx.grid.sceneAABB.ptMin.z(),
			ctx.grid.sceneAABB.ptMax.x(), ctx.grid.sceneAABB.ptMax.y(), ctx.grid.sceneAABB.ptMax.z());
		for (uint32_t i = 0; i < ctx.tiles.size(); ++i) {
			const OrthoTile& t = ctx.tiles[i];
			VERBOSE("  tile(%u,%u): p0=(%u,%u) size=%dx%d world=(%.2f,%.2f)-(%.2f,%.2f)",
				t.tile.x, t.tile.y, t.p0.x, t.p0.y, t.size.width, t.size.height,
				t.worldBounds.ptMin.x(), t.worldBounds.ptMin.y(),
				t.worldBounds.ptMax.x(), t.worldBounds.ptMax.y());
		}
	}
	VERBOSE("TestOrthoMapGrid passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 4: Tile camera setup
// - R is down-looking: R = [[1,0,0],[0,-1,0],[0,0,-1]]
// - C is above tile center at scene max Z + 1
// - K has 1/gsd as focal length, principal point at tile center
bool TestOrthoMapCamera(bool verbose)
{
	TD_TIMER_START();
	Scene scene(CreateSyntheticOrthoScene());
	OrthoConfig config;
	config.targetGSD = 0.1f;
	config.tileSize = 256;
	config.tileOverlap = 0;
	OrthoMapContext ctx(scene, config);
	if (!ctx.ComputeGrid()) {
		VERBOSE("ERROR: ComputeGrid failed!");
		return false;
	}
	const OrthoTile& tile = ctx.tiles[0];
	const Camera& cam = tile.camera;
	// verify R: direction should be (0,0,-1) — looking down
	const Point3 dir(cam.Direction());
	if (ABS(dir.x) > 1e-6 || ABS(dir.y) > 1e-6 || ABS(dir.z + 1.0) > 1e-6) {
		VERBOSE("ERROR: Camera direction should be (0,0,-1), got (%.4f,%.4f,%.4f)!",
			dir.x, dir.y, dir.z);
		return false;
	}
	// verify C.z is above scene max Z
	const float sceneMaxZ = ctx.grid.sceneAABB.ptMax.z();
	if (cam.C.z <= sceneMaxZ) {
		VERBOSE("ERROR: Camera Z (%.4f) should be above scene max Z (%.4f)!",
			(float)cam.C.z, sceneMaxZ);
		return false;
	}
	// verify K focal length = 1/gsd
	const REAL expectedFocal = 1.0 / config.targetGSD;
	if (ABS(cam.K(0, 0) - expectedFocal) > 1e-6 || ABS(cam.K(1, 1) - expectedFocal) > 1e-6) {
		VERBOSE("ERROR: Camera focal length should be %.2f, got (%.2f, %.2f)!",
			expectedFocal, cam.K(0, 0), cam.K(1, 1));
		return false;
	}
	// verify principal point is at tile center
	const REAL expectedCx = (REAL)(tile.size.width - 1) / 2;
	const REAL expectedCy = (REAL)(tile.size.height - 1) / 2;
	if (ABS(cam.K(0, 2) - expectedCx) > 1e-6 || ABS(cam.K(1, 2) - expectedCy) > 1e-6) {
		VERBOSE("ERROR: Camera principal point should be (%.2f, %.2f), got (%.2f, %.2f)!",
			expectedCx, expectedCy, cam.K(0, 2), cam.K(1, 2));
		return false;
	}
	// verify C.x and C.y are at tile world center
	const float expectedCX = (tile.worldBounds.ptMin.x() + tile.worldBounds.ptMax.x()) * 0.5f;
	const float expectedCY = (tile.worldBounds.ptMin.y() + tile.worldBounds.ptMax.y()) * 0.5f;
	if (ABS((float)cam.C.x - expectedCX) > 1e-3f || ABS((float)cam.C.y - expectedCY) > 1e-3f) {
		VERBOSE("ERROR: Camera center should be (%.4f, %.4f), got (%.4f, %.4f)!",
			expectedCX, expectedCY, (float)cam.C.x, (float)cam.C.y);
		return false;
	}
	if (verbose) {
		VERBOSE("  camera R: [%.4f,%.4f,%.4f; %.4f,%.4f,%.4f; %.4f,%.4f,%.4f]",
			cam.R(0,0), cam.R(0,1), cam.R(0,2),
			cam.R(1,0), cam.R(1,1), cam.R(1,2),
			cam.R(2,0), cam.R(2,1), cam.R(2,2));
		VERBOSE("  camera C: (%.4f, %.4f, %.4f)", cam.C.x, cam.C.y, cam.C.z);
		VERBOSE("  camera K: focal=%.2f, cx=%.2f, cy=%.2f", cam.K(0,0), cam.K(0,2), cam.K(1,2));
	}
	VERBOSE("TestOrthoMapCamera passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 5: G-buffer rasterization
// - Non-zero depth inside mesh footprint
// - Zero depth outside mesh footprint
// - All non-zero depths are positive
// - Depth values are correct (camera Z of the flat mesh)
bool TestOrthoMapRasterize(bool verbose)
{
	TD_TIMER_START();
	Scene scene(CreateSyntheticOrthoScene());
	OrthoConfig config;
	config.targetGSD = 0.5f;   // coarse GSD for fast test
	config.tileSize = 256;
	config.tileOverlap = 0;
	config.marginFactor = 0.2f; // need enough margin (>3px) for isInsideWithBorder<3> check
	OrthoMapContext ctx(scene, config);
	if (!ctx.ComputeGrid()) {
		VERBOSE("ERROR: ComputeGrid failed!");
		return false;
	}
	if (!ctx.RasterizeTiles()) {
		VERBOSE("ERROR: RasterizeTiles failed!");
		return false;
	}
	// verify at least one tile has non-zero depth
	unsigned nonZeroPixels = 0;
	unsigned totalPixels = 0;
	bool hasInvalidDepth = false;
	for (uint32_t i = 0; i < ctx.tiles.size(); ++i) {
		const OrthoTile& tile = ctx.tiles[i];
		if (tile.depthMap.empty()) {
			VERBOSE("ERROR: Tile (%u,%u) has empty depth map!", tile.tile.x, tile.tile.y);
			return false;
		}
		for (int y = 0; y < tile.depthMap.rows; ++y) {
			for (int x = 0; x < tile.depthMap.cols; ++x) {
				const Depth d = tile.depthMap(y, x);
				++totalPixels;
				if (d != 0) {
					++nonZeroPixels;
					if (d < 0)
						hasInvalidDepth = true;
				}
			}
		}
	}
	if (nonZeroPixels == 0) {
		VERBOSE("ERROR: No non-zero depth pixels found!");
		return false;
	}
	if (hasInvalidDepth) {
		VERBOSE("ERROR: Found negative depth values!");
		return false;
	}
	// the mesh covers most of the scene, so a significant fraction should be non-zero
	const float coverage = (float)nonZeroPixels / totalPixels;
	if (coverage < 0.3f) {
		VERBOSE("ERROR: Depth coverage too low (%.1f%%), expected >30%%!", coverage * 100);
		return false;
	}
	// verify depth values: pyramid mesh, base Z=0, peak Z=5, camera at Z=sceneMaxZ+1=6
	// camera-space Z for base corners: -(0 - 6) = 6.0
	// camera-space Z for peak: -(5 - 6) = 1.0
	// so depths should range from ~1.0 (peak) to ~6.0 (base)
	const OrthoTile& tile0 = ctx.tiles[0];
	const float camHeight = ctx.grid.sceneAABB.ptMax.z() + 1.0f;
	float minDepth = FLT_MAX, maxDepth = 0;
	for (int y = 0; y < tile0.depthMap.rows; ++y) {
		for (int x = 0; x < tile0.depthMap.cols; ++x) {
			const Depth d = tile0.depthMap(y, x);
			if (d > 0) {
				if (d < minDepth) minDepth = d;
				if (d > maxDepth) maxDepth = d;
			}
		}
	}
	// verify depth range exists (pyramid creates gradient)
	if (maxDepth - minDepth < 0.5f) {
		VERBOSE("ERROR: Depth range too narrow (%.2f - %.2f), expected gradient from pyramid!",
			minDepth, maxDepth);
		return false;
	}
	// verify depth bounds: min should be near 1.0 (peak), max near camHeight (base)
	if (minDepth < 0.5f || maxDepth > camHeight + 0.5f) {
		VERBOSE("ERROR: Depth values out of range: min=%.2f, max=%.2f, camHeight=%.2f!",
			minDepth, maxDepth, camHeight);
		return false;
	}
	if (verbose) {
		scene.mesh.Save(MAKE_PATH("ortho_rasterize_mesh.ply"));
		ExportTileDepthMaps(ctx, MAKE_PATH("ortho_rasterize_depth"));
		VERBOSE("  exported: ortho_rasterize_mesh.ply, ortho_rasterize_depth_tile_*.png");
		VERBOSE("  depth range: min=%.2f, max=%.2f, camHeight=%.2f, coverage=%.1f%%",
			minDepth, maxDepth, camHeight, coverage * 100);
	}
	VERBOSE("TestOrthoMapRasterize passed: %u/%u non-zero pixels (%.1f%%) (%s)",
		nonZeroPixels, totalPixels, coverage * 100, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 6: Tile overlap consistency
// Overlapping pixels between adjacent tiles should produce the same depth values
bool TestOrthoMapOverlap(bool verbose)
{
	TD_TIMER_START();
	Scene scene(CreateSyntheticOrthoScene());
	OrthoConfig config;
	config.targetGSD = 0.5f;
	config.tileSize = 32;
	config.tileOverlap = 8;
	config.marginFactor = 0.2f; // need enough margin (>3px) for isInsideWithBorder<3> check
	OrthoMapContext ctx(scene, config);
	if (!ctx.ComputeGrid()) {
		VERBOSE("ERROR: ComputeGrid failed!");
		return false;
	}
	if (ctx.grid.tiles.x < 2) {
		VERBOSE("ERROR: Need at least 2 tiles horizontally for overlap test!");
		return false;
	}
	if (!ctx.RasterizeTiles()) {
		VERBOSE("ERROR: RasterizeTiles failed!");
		return false;
	}
	// check horizontal overlap between tile (0,0) and tile (1,0)
	const OrthoTile& tileA = ctx.tiles[0];                   // (0,0)
	const OrthoTile& tileB = ctx.tiles[1];                   // (1,0)
	const unsigned overlap = config.tileOverlap;
	const unsigned stride = config.tileSize - overlap;
	// tileA pixels [stride, stride+overlap) correspond to tileB pixels [0, overlap)
	unsigned mismatchCount = 0;
	unsigned checkedPixels = 0;
	const int overlapCols = MINF((int)overlap, MINF(tileA.size.width - (int)stride, tileB.size.width));
	const int overlapRows = MINF(tileA.size.height, tileB.size.height);
	for (int y = 0; y < overlapRows; ++y) {
		for (int x = 0; x < overlapCols; ++x) {
			const Depth dA = tileA.depthMap(y, stride + x);
			const Depth dB = tileB.depthMap(y, x);
			++checkedPixels;
			if (ABS(dA - dB) > 1e-4f)
				++mismatchCount;
		}
	}
	if (checkedPixels == 0) {
		VERBOSE("ERROR: No overlap pixels checked!");
		return false;
	}
	if (mismatchCount > 0) {
		VERBOSE("ERROR: %u/%u overlap pixels mismatch between adjacent tiles!",
			mismatchCount, checkedPixels);
		return false;
	}
	if (verbose) {
		ExportTileDepthMaps(ctx, MAKE_PATH("ortho_overlap_depth"));
		VERBOSE("  exported: ortho_overlap_depth_tile_*.png (%u tiles)", ctx.tiles.GetSize());
	}
	VERBOSE("TestOrthoMapOverlap passed: %u overlap pixels match (%s)",
		checkedPixels, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Test 7: Integration test via Scene::ComputeOrthoMap entry point
bool TestOrthoMapIntegration(bool verbose)
{
	TD_TIMER_START();
	// test with valid mesh
	{
		Scene scene(CreateSyntheticOrthoScene());
		OrthoConfig config;
		config.targetGSD = 0.5f;
		config.tileSize = 128;
		config.tileOverlap = 8;
		config.marginFactor = 0.2f;
		OrthoMapContext ctx(scene, config);
		if (!ctx.ComputeGrid() || !ctx.RasterizeTiles()) {
			VERBOSE("ERROR: ComputeOrthoMap failed on valid scene!");
			return false;
		}
		if (verbose) {
			scene.mesh.Save(MAKE_PATH("ortho_integration_mesh.ply"));
			ExportTileDepthMaps(ctx, MAKE_PATH("ortho_integration_depth"));
			VERBOSE("  exported: ortho_integration_mesh.ply, ortho_integration_depth_tile_*.png");
		}
	}
	// test with empty mesh → should fail gracefully
	{
		Scene scene;
		if (scene.ComputeOrthoMap(0.1f)) {
			VERBOSE("ERROR: ComputeOrthoMap should fail on empty mesh!");
			return false;
		}
	}
	// test with user GSD = 0 (auto) on a scene with cameras
	{
		Scene scene(CreateSyntheticOrthoScene());
		if (!scene.ComputeOrthoMap(0, 256, 16)) {
			VERBOSE("ERROR: ComputeOrthoMap with auto GSD failed!");
			return false;
		}
	}
	VERBOSE("TestOrthoMapIntegration passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace MVS
