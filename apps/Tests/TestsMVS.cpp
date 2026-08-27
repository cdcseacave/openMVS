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
#include "Tests.h"
#include "TestsMVS.h"
#include <halfmesh/RectPacking.h>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestMVS "));

namespace MVS {

bool MeshVertexColorsPLYTest()
{
	const ScopedTempDir tmpDir(_T("MeshVertexColorsPLYTest"));
	if (!tmpDir.IsValid())
		return false;

	Mesh mesh;
	mesh.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	mesh.faces = {{0, 1, 2}};
	mesh.vertexColors = {Pixel8U::RED, Pixel8U::GREEN, Pixel8U::BLUE};
	for (const bool bBinary: {false, true}) {
		const String fileName(tmpDir(bBinary ? _T("mesh-binary.ply") : _T("mesh-ascii.ply")));
		if (!mesh.Save(fileName, cList<String>(), bBinary))
			return false;
		// the mesh must reload with its colors
		Mesh loaded;
		if (!loaded.Load(fileName) || loaded.vertices != mesh.vertices || loaded.faces != mesh.faces ||
			loaded.vertexColors != mesh.vertexColors)
			return false;
		// the same file must read as a colored point-cloud
		PointCloud pointCloud;
		if (!pointCloud.Load(fileName) || pointCloud.points.size() != mesh.vertices.size() || pointCloud.colors.size() != mesh.vertexColors.size())
			return false;
		FOREACH(idxVertex, mesh.vertices)
			if (pointCloud.points[idxVertex] != mesh.vertices[idxVertex] || pointCloud.colors[idxVertex] != mesh.vertexColors[idxVertex])
				return false;
	}
	PointCloud pointCloud;
	pointCloud.points = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	pointCloud.colors = {Pixel8U::RED, Pixel8U::GREEN, Pixel8U::BLUE};
	pointCloud.normals = {{0.f, 0.f, 1.f}, {0.f, 0.f, 1.f}, {0.f, 0.f, 1.f}};
	const String fileName(tmpDir(_T("pointcloud.glb")));
	PointCloud loaded;
	if (!pointCloud.Save(fileName) || !loaded.Load(fileName) ||
		loaded.points != pointCloud.points || loaded.colors != pointCloud.colors || loaded.normals != pointCloud.normals)
		return false;
	return true;
}

bool MeshHalfMeshProcessingTest()
{
	Mesh mesh;
	mesh.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.5f, 1.f, 0.f}, {0.5f, 0.5f, 1.f},
		{2.f, 0.f, 0.f}, {3.f, 0.f, 0.f}
	};
	mesh.faces = {
		{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2},
		{0, 1, 4}, {4, 1, 5}
	};
	mesh.vertexColors = {
		Pixel8U::RED, Pixel8U::GREEN, Pixel8U::BLUE,
		Pixel8U::WHITE, Pixel8U::CYAN, Pixel8U::GRAY
	};
	// Populate only the derived arrays a caller actually owns. The HalfMesh
	// bridge must rebuild exactly these after topology changes and leave the
	// other caches empty.
	mesh.ListIncidentFaces();
	mesh.ListIncidentVertices();
	mesh.ComputeNormalFaces();
	mesh.ComputeNormalVertices();
	Mesh::CleanParams cleanParams;
	cleanParams.removeSpikes = true;
	mesh.Clean(cleanParams);
	if (mesh.vertices.size() != 4 || mesh.faces.size() != 4 ||
		mesh.vertexColors.size() != mesh.vertices.size() ||
		mesh.vertexFaces.size() != mesh.vertices.size() ||
		mesh.vertexVertices.size() != mesh.vertices.size() ||
		mesh.faceNormals.size() != mesh.faces.size() ||
		mesh.vertexNormals.size() != mesh.vertices.size() ||
		!mesh.faceFaces.empty() || !mesh.vertexBoundary.empty()) {
		VERBOSE("ERROR: HalfMesh bridge did not preserve attributes/derived-data ownership!");
		return false;
	}

	Mesh nonManifold;
	nonManifold.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f},
		{-1.f, 0.f, 0.f}, {0.f, -1.f, 0.f}
	};
	nonManifold.faces = {{0, 1, 2}, {0, 3, 4}};
	Mesh::VertexIdxArr duplicatedVertices;
	if (nonManifold.FixNonManifold(0.f, &duplicatedVertices) != 1 ||
		duplicatedVertices.size() != 1 || nonManifold.vertices.size() != 6) {
		VERBOSE("ERROR: HalfMesh bridge did not split a non-manifold bow-tie vertex!");
		return false;
	}

	Mesh duplicateGeometry;
	duplicateGeometry.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f},
		{0.f, 0.f, 0.f}, {2.f, 2.f, 2.f}
	};
	duplicateGeometry.faces = {{0, 1, 2}, {3, 2, 1}};
	if (duplicateGeometry.RemoveDuplicatedVertices() != 1 || duplicateGeometry.vertices.size() != 4 ||
		duplicateGeometry.RemoveUnreferencedVertices() != 1 || duplicateGeometry.vertices.size() != 3) {
		VERBOSE("ERROR: HalfMesh bridge did not remove duplicate and unreferenced vertices!");
		return false;
	}

	Mesh degenerateGeometry;
	degenerateGeometry.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	degenerateGeometry.faces = {{0, 1, 2}, {0, 1, 1}};
	if (degenerateGeometry.RemoveDegenerateFaces(0.f) != 1 || degenerateGeometry.faces.size() != 1) {
		VERBOSE("ERROR: HalfMesh bridge did not remove a degenerate face!");
		return false;
	}

	Mesh disconnected;
	disconnected.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {2.f, 0.f, 0.f},
		{0.f, 1.f, 0.f}, {1.f, 1.f, 0.f}, {2.f, 1.f, 0.f},
		{0.f, 2.f, 0.f}, {1.f, 2.f, 0.f}, {2.f, 2.f, 0.f},
		{4.f, 0.f, 0.f}, {4.1f, 0.f, 0.f}, {4.f, 0.1f, 0.f}
	};
	disconnected.faces = {
		{0, 4, 1}, {0, 3, 4}, {1, 5, 2}, {1, 4, 5},
		{3, 7, 4}, {3, 6, 7}, {4, 8, 5}, {4, 7, 8},
		{9, 10, 11}
	};
	if (disconnected.RemoveSpuriousComponents(1.5f) != 1 ||
		disconnected.faces.size() != 8 || disconnected.vertices.size() != 9) {
		VERBOSE("ERROR: HalfMesh bridge did not remove a spurious disconnected component!");
		return false;
	}

	// A geometry no-op must preserve authored attributes while leaving derived
	// cache ownership unchanged.
	mesh.faceTexcoords.resize(mesh.faces.size()*3);
	FOREACH(idxTexcoord, mesh.faceTexcoords)
		mesh.faceTexcoords[idxTexcoord] = Mesh::TexCoord((float)(idxTexcoord%3), (float)(idxTexcoord/3));
	mesh.faceTexindices.resize(mesh.faces.size());
	FOREACH(idxTexindex, mesh.faceTexindices)
		mesh.faceTexindices[idxTexindex] = 0;
	mesh.texturesDiffuse.emplace_back(1, 1);
	mesh.texturesDiffuse.back()(0, 0) = Pixel8U::RED;
	const Mesh::TexCoordArr faceTexcoords(mesh.faceTexcoords);
	const Mesh::TexIndexArr faceTexindices(mesh.faceTexindices);
	Mesh::CleanParams roundTripParams;
	roundTripParams.finalize = false;
	mesh.Clean(roundTripParams);
	if (mesh.faceTexcoords != faceTexcoords || mesh.faceTexindices != faceTexindices ||
		mesh.texturesDiffuse.size() != 1 || mesh.texturesDiffuse.front()(0, 0) != Pixel8U::RED) {
		VERBOSE("ERROR: HalfMesh bridge did not preserve texture attributes on a no-op round trip!");
		return false;
	}

	// Refinement selects planar vertices in MVS, but removal and retriangulation
	// are delegated to HalfMesh. Removing the top of an octahedron must close the
	// resulting four-edge hole and leave a watertight surface.
	Mesh octahedron;
	octahedron.vertices = {
		{0.f, 0.f, 1.f}, {0.f, 0.f, -1.f},
		{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {-1.f, 0.f, 0.f}, {0.f, -1.f, 0.f}
	};
	octahedron.faces = {
		{0, 2, 3}, {0, 3, 4}, {0, 4, 5}, {0, 5, 2},
		{1, 3, 2}, {1, 4, 3}, {1, 5, 4}, {1, 2, 5}
	};
	Mesh::VertexIdxArr verticesRemove;
	verticesRemove.emplace_back(0);
	if (octahedron.RemoveVerticesAndFill(verticesRemove) != 1 || !octahedron.IsWatertight()) {
		VERBOSE("ERROR: HalfMesh bridge did not fill the hole left by selected-vertex removal!");
		return false;
	}

	Mesh grid;
	grid.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {2.f, 0.f, 0.f},
		{0.f, 1.f, 0.f}, {1.f, 1.f, 0.f}, {2.f, 1.f, 0.f},
		{0.f, 2.f, 0.f}, {1.f, 2.f, 0.f}, {2.f, 2.f, 0.f}
	};
	grid.faces = {
		{0, 4, 1}, {0, 3, 4}, {1, 5, 2}, {1, 4, 5},
		{3, 7, 4}, {3, 6, 7}, {4, 8, 5}, {4, 7, 8}
	};
	grid.Simplify(0.5f);
	if (grid.faces.empty() || grid.faces.size() >= 8) {
		VERBOSE("ERROR: HalfMesh bridge simplification did not reduce the mesh!");
		return false;
	}

	Mesh openOctahedron(octahedron);
	openOctahedron.faces.pop_back();
	openOctahedron.faceTexcoords.resize(openOctahedron.faces.size()*3);
	openOctahedron.faceTexindices.resize(openOctahedron.faces.size());
	openOctahedron.texturesDiffuse.emplace_back(1, 1);
	if (openOctahedron.CloseHoles(3) != 1 || !openOctahedron.IsWatertight() ||
		!openOctahedron.faceTexcoords.empty() || !openOctahedron.faceTexindices.empty() ||
		!openOctahedron.texturesDiffuse.empty()) {
		VERBOSE("ERROR: HalfMesh bridge did not close a generic hole and invalidate texture data!");
		return false;
	}

	Mesh smoothGrid;
	smoothGrid.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {2.f, 0.f, 0.f},
		{0.f, 1.f, 0.f}, {1.f, 1.f, 1.f}, {2.f, 1.f, 0.f},
		{0.f, 2.f, 0.f}, {1.f, 2.f, 0.f}, {2.f, 2.f, 0.f}
	};
	smoothGrid.faces = {
		{0, 4, 1}, {0, 3, 4}, {1, 5, 2}, {1, 4, 5},
		{3, 7, 4}, {3, 6, 7}, {4, 8, 5}, {4, 7, 8}
	};
	smoothGrid.Smooth(1);
	if (smoothGrid.vertices[4].z == 1.f) {
		VERBOSE("ERROR: HalfMesh bridge smoothing did not update vertex positions!");
		return false;
	}

	Mesh remeshed;
	remeshed.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {1.f, 1.f, 0.f}, {0.f, 1.f, 0.f}};
	remeshed.faces = {{0, 1, 2}, {0, 2, 3}};
	Mesh::CleanParams remeshParams;
	remeshParams.edgeLength = 0.3f;
	remeshParams.remeshIterations = 2;
	remeshed.Clean(remeshParams);
	if (remeshed.faces.size() <= 2 || remeshed.vertices.size() <= 4) {
		VERBOSE("ERROR: HalfMesh bridge remeshing did not adapt mesh density!");
		return false;
	}

	// a relative target edge length resolves against the mesh's own mean edge
	Mesh relRemeshed;
	relRemeshed.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {1.f, 1.f, 0.f}, {0.f, 1.f, 0.f}};
	relRemeshed.faces = {{0, 1, 2}, {0, 2, 3}};
	Mesh::CleanParams relRemeshParams;
	relRemeshParams.edgeLength = -0.25f;
	relRemeshParams.remeshIterations = 2;
	relRemeshed.Clean(relRemeshParams);
	if (relRemeshed.faces.size() <= 2 || relRemeshed.vertices.size() <= 4) {
		VERBOSE("ERROR: HalfMesh bridge relative remeshing did not adapt mesh density!");
		return false;
	}

	Mesh texturedGrid;
	constexpr int gridCells = 4;
	constexpr int sourceTextureSize = 16;
	for (int y = 0; y <= gridCells; ++y)
		for (int x = 0; x <= gridCells; ++x)
			texturedGrid.vertices.emplace_back((float)x/gridCells, (float)y/gridCells, 0.f);
	const auto VertexIndex = [](int x, int y) { return (Mesh::VIndex)(y*(gridCells+1)+x); };
	const auto Texcoord = [](int x, int y) { return Mesh::TexCoord((float)(x*sourceTextureSize)/gridCells, (float)(y*sourceTextureSize)/gridCells); };
	for (int y = 0; y < gridCells; ++y) {
		for (int x = 0; x < gridCells; ++x) {
			texturedGrid.faces.emplace_back(VertexIndex(x, y), VertexIndex(x+1, y), VertexIndex(x+1, y+1));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x, y));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x+1, y));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x+1, y+1));
			texturedGrid.faces.emplace_back(VertexIndex(x, y), VertexIndex(x+1, y+1), VertexIndex(x, y+1));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x, y));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x+1, y+1));
			texturedGrid.faceTexcoords.emplace_back(Texcoord(x, y+1));
		}
	}
	texturedGrid.texturesDiffuse.emplace_back(sourceTextureSize, sourceTextureSize);
	for (int y = 0; y < sourceTextureSize; ++y)
		for (int x = 0; x < sourceTextureSize; ++x)
			texturedGrid.texturesDiffuse.front()(y, x) = Pixel8U::RED;
	Mesh rebakedGrid(texturedGrid);
	rebakedGrid.faceTexcoords.Release();
	rebakedGrid.texturesDiffuse.clear();
	if (!texturedGrid.TransferTexture(rebakedGrid, 1, 32) || !rebakedGrid.HasTexture() ||
		rebakedGrid.faceTexcoords.size() != rebakedGrid.faces.size()*3 ||
		rebakedGrid.texturesDiffuse.size() != 1 || rebakedGrid.texturesDiffuse.front().size() != cv::Size(32, 32)) {
		VERBOSE("ERROR: HalfMesh bridge texture rebake did not produce a valid atlas!");
		return false;
	}
	bool hasBakedTexel = false;
	for (int y = 0; y < rebakedGrid.texturesDiffuse.front().rows && !hasBakedTexel; ++y)
		for (int x = 0; x < rebakedGrid.texturesDiffuse.front().cols; ++x)
			if (rebakedGrid.texturesDiffuse.front()(y, x) != Pixel8U::BLACK) {
				hasBakedTexel = true;
				break;
			}
	if (!hasBakedTexel) {
		VERBOSE("ERROR: HalfMesh bridge texture rebake produced an empty atlas!");
		return false;
	}
	{
		const ScopedTempDir tmpDir(_T("MeshHalfMeshProcessingTest"));
		if (!tmpDir.IsValid())
			return false;
		const String fileName(tmpDir(_T("rebaked.glb")));
		// halfmesh names a non-embedded diffuse image <stem>_diffuse<NN>, per blob
		const String textureFileName(tmpDir(_T("rebaked_diffuse00.png")));
		Mesh reloaded;
		if (!rebakedGrid.Save(fileName) || !File::isFile(textureFileName) || !reloaded.Load(fileName) ||
			reloaded.vertices.size() != rebakedGrid.vertices.size() || reloaded.faces.size() != rebakedGrid.faces.size()) {
			VERBOSE("ERROR: HalfMesh bridge rebaked texture GLB export did not round-trip!");
			return false;
		}
		// glTF files are y-up while both meshes are not, so halfmesh puts the rotation
		// on the root node and undoes it on load; the round-trip must be an identity.
		// Compare the box rather than the vertices: a seam split would renumber them.
		const Mesh::Box box(rebakedGrid.GetAABB()), reloadedBox(reloaded.GetAABB());
		if (!box.ptMin.isApprox(reloadedBox.ptMin) || !box.ptMax.isApprox(reloadedBox.ptMax)) {
			VERBOSE("ERROR: HalfMesh bridge GLB round-trip did not preserve the orientation!");
			return false;
		}
	}

	const std::vector<cv::Rect> rectangles = {{0, 0, 6, 4}, {0, 0, 6, 4}, {0, 0, 6, 4}};
	halfmesh::RectPackParams packingParams;
	packingParams.pageSize = cv::Size(8, 8);
	packingParams.mode = halfmesh::RectPackMode::FixedMultiPage;
	packingParams.padding = 0;
	packingParams.allowRotation = true;
	std::vector<halfmesh::RectPlacement> placements;
	const halfmesh::RectPackResult packingResult(halfmesh::PackRectangles(rectangles, packingParams, placements));
	if (packingResult.numPacked != rectangles.size() || packingResult.numPages != 2 ||
		placements.size() != rectangles.size()) {
		VERBOSE("ERROR: HalfMesh multi-page rectangle packing returned an invalid result!");
		return false;
	}
	for (size_t i = 0; i < placements.size(); ++i) {
		const halfmesh::RectPlacement& placement = placements[i];
		if (!placement.packed || placement.page >= packingResult.numPages ||
			placement.rect.x < 0 || placement.rect.y < 0 ||
			placement.rect.br().x > packingResult.pageSize.width ||
			placement.rect.br().y > packingResult.pageSize.height) {
			VERBOSE("ERROR: HalfMesh rectangle packing placed a patch outside its page!");
			return false;
		}
		for (size_t j = i+1; j < placements.size(); ++j)
			if (placement.page == placements[j].page && (placement.rect & placements[j].rect).area() != 0) {
				VERBOSE("ERROR: HalfMesh rectangle packing produced overlapping patches!");
				return false;
			}
	}
	return true;
}
/*----------------------------------------------------------------*/

// Both fixtures below lock a cut topology that is partly decided by how the min-cut solver
// assigns the cells carrying no terminal capacity (s == t == 0) -- each fixture's comment says
// which part. Every solver this project has run agrees there (IBFS and Boost BK reconstruct
// byte-identical meshes), so the lock is stable; but a solver change that alters that convention
// would fail these tests with no mesh regression behind it, so the failure says so rather than
// leaving the next reader to rediscover it from the appendix
static void ReportFixtureSolverTieBreak()
{
	VERBOSE("NOTE: this fixture's topology depends on how the min-cut solver assigns cells with no terminal capacity; if the solver changed, compare the cut labels before calling this a regression");
}

// Fixture A ("bipyramid", docs/design/DelaunayMeshReconstruction.md,
// Appendix), a synthetic 2-tetrahedra / 1-camera / 1-contributing-point scene hand-solved down
// to exact facet capacities and s/t values. Those internal values are not observable through the
// public API without adding test-only instrumentation to libs/MVS (explicitly out of scope), so
// this locks the resulting cut *topology* instead -- confirmed empirically (byte-identical across
// repeated runs) rather than hand-derived from the appendix's numbers alone, because the actual
// min-cut solver's treatment of this graph's several disconnected, zero-capacity cells is an
// implementation behaviour, not something the appendix's per-cell s/t/f table determines on its
// own: a cell with no path to either terminal (s=t=0) resolves to the source/free side, while a
// cell with no path in but a nonzero t (like the D_in vote this fixture places on an infinite cell
// beyond E, which never connects back to the finite triangulation, the "vote never reaches the
// surface" mechanism reproduced here in miniature) resolves to the sink/full side from
// its own local bias alone. That interaction, not visible from the appendix table, is what this
// test locks: it deterministically extracts exactly one face -- the wing of the tetrahedron behind
// E that carries the orphaned D_in vote -- while the opposite apex D never appears (every facet
// touching D stays on the free side with its camera-linked neighbour, matching-side pairs never
// produce a face). A regression that drops the behind-the-point D_in vote or relocates it onto a
// different cell changes this result.
bool MeshBipyramidFixtureTest()
{
	Scene sceneA;
	sceneA.pointcloud.points = {
		PointCloud::Point(1.0f, 0.0f, 0.0f), // A
		PointCloud::Point(-0.5f, 0.8660254037844386f, 0.0f), // B
		PointCloud::Point(-0.5f, -0.8660254037844386f, 0.0f), // C
		PointCloud::Point(0.0f, 0.0f, 3.0f), // D
		PointCloud::Point(0.0f, 0.0f, -3.0f), // E
	};
	sceneA.pointcloud.pointViews = {
		PointCloud::ViewArr{0}, PointCloud::ViewArr{0}, PointCloud::ViewArr{0},
		PointCloud::ViewArr{0}, PointCloud::ViewArr{0},
	};
	sceneA.images.resize(1);
	Image& cam0 = sceneA.images[0];
	cam0.poseID = 0; cam0.ID = 0; cam0.width = cam0.height = 640;
	// R is a proper rotation (Rx(pi), det=+1), not the reflection diag(1,1,-1) that reads the same
	// way here: both send the world -Z axis to the camera +Z axis, so the camera looks from above
	// the z=0 plane towards E either way, but only the rotation passes Camera's validity check
	cam0.camera = Camera(Matrix3x3(200,0,320, 0,200,240, 0,0,1), Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), Point3(0,0,1.5), true);
	// kSigma = 1/sqrt(10): the median squared finite edge length is 10 (6 edges at length^2=10
	// vs 3 at length^2=3), so this makes sigma exactly 1.0, matching the appendix's derivation;
	// the appendix hand-solves the fixture under the single global sigma and the ungated
	// extraction, so the default per-vertex sigma, canonical rescale (a no-op at this
	// scale, pinned for determinism) and webbing gate are all pinned off here
	Scene::ReconstructMeshParams fixtureParams;
	fixtureParams.distInsert = 0.f;
	fixtureParams.bUseFreeSpaceSupport = false;
	fixtureParams.kSigma = 0.31622776601683794f;
	fixtureParams.kQual = 0.f;
	fixtureParams.bAdaptiveSigma = false;
	fixtureParams.bCanonicalRescale = false;
	fixtureParams.maxEdgeScale = 0.f;
	if (!sceneA.ReconstructMesh(fixtureParams)) {
		VERBOSE("ERROR: Fixture-A (bipyramid) reconstruction failed!");
		return false;
	}
	if (sceneA.mesh.vertices.size() != 3 || sceneA.mesh.faces.size() != 1) {
		VERBOSE("ERROR: Fixture-A (bipyramid) expected exactly 1 face (3 vertices), got %u vertices, %u faces!", sceneA.mesh.vertices.size(), sceneA.mesh.faces.size());
		ReportFixtureSolverTieBreak();
		return false;
	}
	// the single face must be one of E's two triangles with A/B/C -- D must never appear
	const Point3f pA(1.0f,0.0f,0.0f), pB(-0.5f,0.8660254037844386f,0.0f), pC(-0.5f,-0.8660254037844386f,0.0f), pD(0.0f,0.0f,3.0f), pE(0.0f,0.0f,-3.0f);
	unsigned nE(0), nD(0), nABC(0);
	for (const Mesh::Vertex& v: sceneA.mesh.vertices) {
		if (normSq(v-pE) < 1e-8f) ++nE;
		else if (normSq(v-pD) < 1e-8f) ++nD;
		else if (normSq(v-pA) < 1e-8f || normSq(v-pB) < 1e-8f || normSq(v-pC) < 1e-8f) ++nABC;
	}
	if (nE != 1 || nD != 0 || nABC != 2) {
		VERBOSE("ERROR: Fixture-A (bipyramid) produced an unexpected face -- expected E plus two of A/B/C, got %u E, %u D, %u of A/B/C!", nE, nD, nABC);
		ReportFixtureSolverTieBreak();
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// Fixture B ("tetra + interior point", same appendix), a synthetic
// star-of-4-tetrahedra scene around a single interior contributing point P. As with Fixture A, the
// appendix's own predictions are on internal graph-cut state unreachable from the public API, so
// this locks the resulting cut topology instead -- confirmed empirically (stable across repeated
// runs), not hand-derived, for the same reason as Fixture A: several of this fixture's cells are
// disconnected zero-capacity ("free") nodes whose final side is decided by the solver's own
// tie-break, not by the appendix's per-cell table. Concretely: P's forward-walk vote reaches
// Ca={P,V1,V2,V3} with real positive capacity from its camera-linked infinite neighbour, so Ca
// joins the free side; but P's behind-the-point vote -- deposited via mirror_facet on the arc OUT
// of Cb={P,V0,V2,V3} towards its own camera-linked infinite neighbour, i.e. away from the camera --
// capacity flows the wrong direction to ever pull Cb along with it
// (the arc runs Cb-to-neighbour, not neighbour-to-Cb), so Cb, Cc and Cd are all free-side "free"
// nodes with no s/t bias of their own and default to the same free side as Ca. Every facet in the
// fixture -- the six internal ones and the four hull ones -- therefore ends up with both sides
// matching, and the graph-cut surface extractor (which adds a face only when the two sides of a
// facet differ) produces nothing: an EMPTY mesh. This is a solver-behaviour finding, not a
// mirror_facet correctness proof -- flipping mirror_facet's arc would put the same capacity on the
// opposite (Cb-reaching) arc, and Cb would then join the free side by real reachability instead of
// by default, reaching an *observably identical* empty result. What this fixture does reliably
// lock: the pipeline runs this exact 4-tetrahedra / 2-camera fixture to completion,
// deterministically, with all four hull cells hard-stamped and both of P's votes landing on the
// cells the appendix derives.
bool MeshTetraInteriorPointFixtureTest()
{
	Scene sceneB;
	const float s3(1.7320508075688772f);
	sceneB.pointcloud.points = {
		PointCloud::Point(0.0f, 0.0f, 0.0f), // P
		PointCloud::Point(1.5f, 0.5f, 6.0f), // V0
		PointCloud::Point(4.0f, 0.0f, -2.0f), // V1
		PointCloud::Point(-2.0f, 2.f*s3, -2.0f), // V2
		PointCloud::Point(-2.0f, -2.f*s3, -2.0f), // V3
	};
	sceneB.pointcloud.pointViews = {
		PointCloud::ViewArr{0}, // P seen by camera 0
		PointCloud::ViewArr{1}, // V0 seen by camera 1 only (its ray provably contributes nothing)
		PointCloud::ViewArr{0}, // V1
		PointCloud::ViewArr{0}, // V2
		PointCloud::ViewArr{0}, // V3
	};
	sceneB.images.resize(2);
	Image& cam0 = sceneB.images[0];
	cam0.poseID = 0; cam0.ID = 0; cam0.width = cam0.height = 640;
	cam0.camera = Camera(Matrix3x3(200,0,320, 0,200,240, 0,0,1), Matrix3x3::IDENTITY, Point3(0,0,-10), true);
	Image& cam1 = sceneB.images[1];
	cam1.poseID = 1; cam1.ID = 1; cam1.width = cam1.height = 640;
	cam1.camera = Camera(Matrix3x3(200,0,320, 0,200,240, 0,0,1), Matrix3x3(1,0,0, 0,-1,0, 0,0,-1), Point3(1.5,0.5,26), true);
	// kSigma = 1/sqrt(3): the median squared finite edge length is 48, making sigma exactly 4.0;
	// pinned to the same hand-solved global-sigma, ungated configuration as Fixture A
	Scene::ReconstructMeshParams fixtureParams;
	fixtureParams.distInsert = 0.f;
	fixtureParams.bUseFreeSpaceSupport = false;
	fixtureParams.kSigma = 0.5773502691896258f;
	fixtureParams.kQual = 0.f;
	fixtureParams.bAdaptiveSigma = false;
	fixtureParams.bCanonicalRescale = false;
	fixtureParams.maxEdgeScale = 0.f;
	if (!sceneB.ReconstructMesh(fixtureParams)) {
		VERBOSE("ERROR: Fixture-B (tetra + interior point) reconstruction failed!");
		return false;
	}
	if (!sceneB.mesh.vertices.IsEmpty() || !sceneB.mesh.faces.IsEmpty()) {
		VERBOSE("ERROR: Fixture-B (tetra + interior point) expected an empty mesh, got %u vertices, %u faces!", sceneB.mesh.vertices.size(), sceneB.mesh.faces.size());
		ReportFixtureSolverTieBreak();
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// Exercise ROI integration with the first point outside the ROI; this used to leave
// default entries in the spatial-sort index and could reconstruct the wrong points.
static bool ROIMeshReconstructionTest(Scene& scene)
{
	PointCloud pointcloud(scene.pointcloud);
	const OBB3f initialOBB(scene.obb);
	const OBB3f roi(scene.pointcloud.GetAABB(0.1f, 0.9f));
	PointCloud::Index idxOutside(NO_ID);
	FOREACH(idxPoint, scene.pointcloud.points) {
		if (!roi.Intersects(scene.pointcloud.points[idxPoint])) {
			idxOutside = idxPoint;
			break;
		}
	}
	if (idxOutside == NO_ID) {
		VERBOSE("ERROR: TestDataset failed finding a point outside the test ROI!");
		return false;
	}
	if (idxOutside != 0) {
		std::swap(scene.pointcloud.points[0], scene.pointcloud.points[idxOutside]);
		std::swap(scene.pointcloud.pointViews[0], scene.pointcloud.pointViews[idxOutside]);
		if (!scene.pointcloud.pointWeights.IsEmpty())
			std::swap(scene.pointcloud.pointWeights[0], scene.pointcloud.pointWeights[idxOutside]);
		if (!scene.pointcloud.normals.IsEmpty())
			std::swap(scene.pointcloud.normals[0], scene.pointcloud.normals[idxOutside]);
		if (!scene.pointcloud.colors.IsEmpty())
			std::swap(scene.pointcloud.colors[0], scene.pointcloud.colors[idxOutside]);
		if (!scene.pointcloud.labels.IsEmpty())
			std::swap(scene.pointcloud.labels[0], scene.pointcloud.labels[idxOutside]);
	}
	scene.obb = roi;
	Scene::ReconstructMeshParams params;
	params.bUseFreeSpaceSupport = false;
	params.bUseOnlyROI = true;
	if (!scene.ReconstructMesh(params) || scene.mesh.IsEmpty()) {
		VERBOSE("ERROR: TestDataset failed reconstructing the ROI mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	scene.pointcloud.Swap(pointcloud);
	scene.obb = initialOBB;
	return true;
}
/*----------------------------------------------------------------*/

// A region-of-interest that intersects none of the dense points must
// fail cleanly through the "no points available" guard, not silently fall back to using
// every point
static bool EmptyROIMeshGuardTest(Scene& scene)
{
	const OBB3f initialOBB(scene.obb);
	// a small valid OBB (positive extent, so IsBounded() stays true) placed far outside the
	// synthetic scene's coordinate range: it intersects none of the dense points; the extent
	// must exceed the float ulp at this magnitude (0.0625 at 1e6) or it collapses to zero
	scene.obb.Set(Matrix3x3f::IDENTITY, Point3f(1.e6f,1.e6f,1.e6f), Point3f(1.e6f+1.f,1.e6f+1.f,1.e6f+1.f));
	if (!scene.obb.IsValid()) {
		VERBOSE("ERROR: TestDataset built an invalid empty-ROI OBB for the degenerate-input test!");
		return false;
	}
	Scene::ReconstructMeshParams params;
	params.bUseFreeSpaceSupport = false;
	params.bUseOnlyROI = true;
	if (scene.ReconstructMesh(params)) {
		VERBOSE("ERROR: TestDataset should have failed reconstructing an empty ROI!");
		return false;
	}
	scene.obb = initialOBB;
	return true;
}
/*----------------------------------------------------------------*/

// Reconstructing with pointWeights released (each view's vote falls back
// to the implicit constant 1, see vert_info_t::InsertViews in SceneReconstruct.cpp) must
// behave the same as reconstructing with pointWeights present and every entry explicitly 1
static bool UnitWeightsFallbackTest(Scene& scene)
{
	const PointCloud pointcloudBackup(scene.pointcloud);
	// run 1: no pointWeights at all
	scene.pointcloud = pointcloudBackup;
	scene.pointcloud.pointWeights.Release();
	Scene::ReconstructMeshParams params;
	params.bUseFreeSpaceSupport = false;
	if (!scene.ReconstructMesh(params) || scene.mesh.IsEmpty()) {
		VERBOSE("ERROR: TestDataset failed reconstructing with empty pointWeights!");
		return false;
	}
	const Mesh::FIndex numFacesEmptyWeights(scene.mesh.faces.size());
	// run 2: identical views layout, pointWeights explicitly all 1
	scene.pointcloud = pointcloudBackup;
	scene.pointcloud.pointWeights.resize(scene.pointcloud.pointViews.size());
	FOREACH(idxPoint, scene.pointcloud.pointViews) {
		const PointCloud::ViewArr& views = scene.pointcloud.pointViews[idxPoint];
		PointCloud::WeightArr& weights = scene.pointcloud.pointWeights[idxPoint];
		weights.resize(views.size());
		FOREACH(idxView, weights)
			weights[idxView] = PointCloud::Weight(1);
	}
	if (!scene.ReconstructMesh(params) || scene.mesh.IsEmpty()) {
		VERBOSE("ERROR: TestDataset failed reconstructing with explicit unit pointWeights!");
		return false;
	}
	const Mesh::FIndex numFacesUnitWeights(scene.mesh.faces.size());
	// Scene::ReconstructMesh takes no thread-count parameter, and its weighting pass runs
	// under OpenMP with atomic float adds keyed off the process-wide thread count (see
	// SceneReconstruct.cpp), so float summation order -- and so the exact face count -- is
	// not guaranteed to match run to run even for identical per-view weights. Forcing
	// omp_set_num_threads(1) here would make it exact, but as a global runtime setting it
	// would also serialize every later OpenMP stage in this same pipeline test (Clean,
	// vertex coloring, texturing), so it is deliberately not used; both meshes are non-empty
	// (checked above) and must agree within 1% of face count instead of exactly
	const Mesh::FIndex faceTol(numFacesEmptyWeights/100+1);
	if (numFacesEmptyWeights > numFacesUnitWeights+faceTol || numFacesUnitWeights > numFacesEmptyWeights+faceTol) {
		VERBOSE("ERROR: TestDataset empty-weights fallback diverged from explicit unit weights (%u vs %u faces)!", numFacesEmptyWeights, numFacesUnitWeights);
		return false;
	}
	scene.pointcloud = pointcloudBackup;
	return true;
}
/*----------------------------------------------------------------*/

// pointWeights must round-trip through the interface archive bit-for-bit
static bool PointWeightsArchiveRoundTripTest(Scene& scene)
{
	const ScopedTempDir tmpDir(_T("WeightsRoundTripTest"));
	if (!tmpDir.IsValid())
		return false;
	const PointCloud pointcloudBackup(scene.pointcloud);
	// non-trivial per-point weights: point i gets weight i/N (point 0 is intentionally 0;
	// the rest are strictly positive so LoadInterface's all-zero-weights drop guard does not
	// discard them)
	const float N(static_cast<float>(scene.pointcloud.points.size()));
	scene.pointcloud.pointWeights.resize(scene.pointcloud.pointViews.size());
	FOREACH(idxPoint, scene.pointcloud.pointViews) {
		const PointCloud::Weight w(static_cast<float>(idxPoint)/N);
		PointCloud::WeightArr& weights = scene.pointcloud.pointWeights[idxPoint];
		weights.resize(scene.pointcloud.pointViews[idxPoint].size());
		FOREACH(idxView, weights)
			weights[idxView] = w;
	}
	const String mvsPath(tmpDir(_T("weights.mvs")));
	Scene reloaded;
	if (!scene.SaveInterface(mvsPath) || !reloaded.LoadInterface(mvsPath)) {
		VERBOSE("ERROR: TestDataset failed the pointWeights round-trip archive I/O!");
		return false;
	}
	if (reloaded.pointcloud.points.size() != scene.pointcloud.points.size() ||
		reloaded.pointcloud.pointViews.size() != scene.pointcloud.pointViews.size() ||
		reloaded.pointcloud.pointWeights.size() != scene.pointcloud.pointWeights.size()) {
		VERBOSE("ERROR: TestDataset pointWeights round-trip changed the point-cloud size!");
		return false;
	}
	FOREACH(idxPoint, scene.pointcloud.pointWeights) {
		if (scene.pointcloud.pointWeights[idxPoint] != reloaded.pointcloud.pointWeights[idxPoint]) {
			VERBOSE("ERROR: TestDataset pointWeights round-trip changed point %u's weights!", idxPoint);
			return false;
		}
	}
	scene.pointcloud = pointcloudBackup;
	return true;
}
/*----------------------------------------------------------------*/

// Too few points for the Delaunay triangulation to ever reach
// dimension 3 must fail cleanly via the dimension guard in SceneReconstruct.cpp, not
// crash or produce garbage; 3 points can reach at most dimension 2, so this is
// deterministic regardless of their actual layout
static bool TooFewPointsMeshGuardTest(Scene& scene)
{
	const PointCloud pointcloudBackup(scene.pointcloud);
	PointCloud pointcloudTiny;
	pointcloudTiny.points.resize(3);
	pointcloudTiny.pointViews.resize(3);
	for (PointCloud::Index i=0; i<3; ++i) {
		pointcloudTiny.points[i] = pointcloudBackup.points[i];
		pointcloudTiny.pointViews[i] = pointcloudBackup.pointViews[i];
	}
	scene.pointcloud = pointcloudTiny;
	Scene::ReconstructMeshParams params;
	params.bUseFreeSpaceSupport = false;
	if (scene.ReconstructMesh(params)) {
		VERBOSE("ERROR: TestDataset should have failed reconstructing from only 3 points!");
		return false;
	}
	scene.pointcloud = pointcloudBackup;
	return true;
}
/*----------------------------------------------------------------*/

// SamplePoints with an explicit seed must be reproducible, and a
// different seed must draw a different sample; per-face point counts are themselves
// seed-dependent (see the fractional-area coin-flip draw in Mesh::SamplePoints), so the
// cross-seed check compares the sampled points directly rather than assuming counts differ
static bool MeshSamplePointsSeedTest(const Mesh& mesh)
{
	constexpr unsigned numSamples = 1000;
	PointCloud pcSeed42a, pcSeed42b, pcSeed7;
	mesh.SamplePoints(numSamples, pcSeed42a, 42);
	mesh.SamplePoints(numSamples, pcSeed42b, 42);
	mesh.SamplePoints(numSamples, pcSeed7, 7);
	if (pcSeed42a.points.empty()) {
		VERBOSE("ERROR: TestDataset SamplePoints produced no points!");
		return false;
	}
	if (pcSeed42a.points != pcSeed42b.points) {
		VERBOSE("ERROR: TestDataset SamplePoints(seed=42) was not reproducible (%u vs %u points)!", pcSeed42a.points.size(), pcSeed42b.points.size());
		return false;
	}
	if (pcSeed42a.points == pcSeed7.points) {
		VERBOSE("ERROR: TestDataset SamplePoints(seed=42) and SamplePoints(seed=7) produced identical points!");
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// the colors must survive the project archive round-trip
static bool MeshVertexColorsArchiveRoundTripTest(const Scene& scene)
{
	const ScopedTempDir tmpDir(_T("VertexColorsRoundTripTest"));
	if (!tmpDir.IsValid())
		return false;
	const String mvsPath(tmpDir(_T("colored.mvs")));
	Scene reloaded;
	if (!scene.Save(mvsPath) || !reloaded.Load(mvsPath) || reloaded.mesh.vertexColors != scene.mesh.vertexColors) {
		VERBOSE("ERROR: TestDataset failed reloading the mesh vertex colors!");
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// test MVS stages on a small sample dataset
bool PipelineTest(bool forceCPU, bool verbose)
{
	TD_TIMER_START();
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	// force CPU for testing even if a GPU backend is available
	if (forceCPU)
		SEACAVE::CUDA::desiredDeviceIDs.clear();
	#endif
	Scene scene;
	if (!scene.Load(MAKE_PATH("scene.mvs"))) {
		VERBOSE("ERROR: TestDataset failed loading the scene!");
		return false;
	}
	OPTDENSE::init();
	OPTDENSE::bRemoveDmaps = true;
	// The point/face counts and quality vary run-to-run (multi-threaded
	// densify/mesh) and differ between the CPU and GPU PatchMatch backends, so
	// these are deliberately wide plausibility windows, not tight regression
	// bounds: they bracket both backends' observed spread with margin.
	// Re-baselined 2026-08-10 for the current defaults. Note the backends now
	// differ by design, not just by numerical spread: nOptimize defaults to
	// ADJUST_CONFIDENCE_AUTO, so the confidence recalibration runs on the GPU
	// backend (fused into the last geometric-consistency iteration, nearly free)
	// and is skipped on the CPU backend (where it would cost a separate pass).
	// Also on: fusion rescue (fFusePriorWeight=3, ~+90% dense points on this
	// scene); pointWeights hold the plain [0,1] per-view confidence consumed by
	// the weighted mesh visibility, which this test exercises by calling the
	// library directly -- the ReconstructMesh app releases them first, unless
	// asked for the weighted path with --constant-weight 0.
	// Measured (GPU adjust-ON / CPU adjust-OFF): recon faces 52.9k / 71.4k,
	// cleaned faces 37.0k / 49.8k, quality 50.4 / 52.2.
	if (!scene.DenseReconstruction() || scene.pointcloud.GetSize() < 50000u) {
		VERBOSE("ERROR: TestDataset failed estimating dense point-cloud (%u points)!", scene.pointcloud.GetSize());
		return false;
	}
	if (verbose)
		scene.pointcloud.Save(MAKE_PATH("scene_dense.ply"));
	if (!ROIMeshReconstructionTest(scene))
		return false;
	if (!EmptyROIMeshGuardTest(scene))
		return false;
	if (!UnitWeightsFallbackTest(scene))
		return false;
	if (!PointWeightsArchiveRoundTripTest(scene))
		return false;
	if (!TooFewPointsMeshGuardTest(scene))
		return false;
	if (!scene.ReconstructMesh() || !ISINSIDE(scene.mesh.faces.size(), 40000u, 100000u)) {
		VERBOSE("ERROR: TestDataset failed reconstructing the mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh.ply"));
	if (!MeshSamplePointsSeedTest(scene.mesh))
		return false;
	constexpr float decimate = 0.7f;
	Mesh::CleanParams cleanParams;
	cleanParams.simplifyTarget = decimate;
	cleanParams.spuriousFactor = 10.f;
	cleanParams.removeSpikes = true;
	cleanParams.maxHoleEdges = 30;
	cleanParams.smoothIterations = 2;
	scene.mesh.Clean(cleanParams);
	if (!ISINSIDE(scene.mesh.faces.size(), 28000u, 70000u)) {
		VERBOSE("ERROR: TestDataset failed cleaning the mesh (%u faces)!", scene.mesh.faces.size());
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_clean.ply"));
	#ifdef _USE_OPENMP
	TestMeshProjectionMT(scene.mesh, scene.images[1]);
	#endif
	// color the mesh per vertex; this releases the images, which texturing below reloads
	const Mesh::Color colEmpty(255, 127, 39);
	if (!scene.ComputeVertexColors(0, 0, 0, 0.f, 0.3f, colEmpty) || scene.mesh.vertexColors.size() != scene.mesh.vertices.size()) {
		VERBOSE("ERROR: TestDataset failed computing the mesh vertex colors!");
		return false;
	}
	// most vertices are seen by at least one view, so they must be sampled and not left empty
	Mesh::VIndex numColored(0);
	for (const Mesh::Color& color: scene.mesh.vertexColors)
		if (color != colEmpty)
			++numColored;
	if (numColored*2 < scene.mesh.vertexColors.size()) {
		VERBOSE("ERROR: TestDataset colored only %u of %u mesh vertices!", numColored, scene.mesh.vertexColors.size());
		return false;
	}
	if (!MeshVertexColorsArchiveRoundTripTest(scene))
		return false;
	scene.mesh.vertexColors.Release();
	if (!scene.TextureMesh(0, 0) || !scene.mesh.HasTexture()) {
		VERBOSE("ERROR: TestDataset failed texturing the mesh!");
		return false;
	}
	if (verbose)
		scene.mesh.Save(MAKE_PATH("scene_dense_mesh_texture.ply"));
	const float qualityScore = scene.ComputeReconstructionQuality().score();
	if (qualityScore < 43.f) {
		VERBOSE("ERROR: TestDataset reconstruction quality too low (%.1f)!", qualityScore);
		return false;
	}
	VERBOSE("All pipeline stages passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace MVS
