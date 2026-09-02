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
#include "../../libs/MVS/SceneRefineCommon.h"
#include "../../libs/MVS/SceneRefineStep.h"
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

	// every vertex of a faceless mesh is incident to no face and so qualifies as a
	// spike; the bridge has to leave such a mesh alone rather than empty it
	Mesh verticesOnly;
	verticesOnly.vertices = {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
	if (verticesOnly.RemoveSpikes() != 0 || verticesOnly.vertices.size() != 3) {
		VERBOSE("ERROR: HalfMesh bridge removed the vertices of a mesh that has no faces!");
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

	// Authored per-vertex normals are attribute data, not a derived cache: an
	// operation that only renumbers vertices has to hand them back unchanged,
	// while one that moves a vertex has to invalidate them so they are recomputed
	// rather than returned stale.
	Mesh authoredNormals;
	authoredNormals.vertices = {
		{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f},
		{0.f, 0.f, 0.f}, {2.f, 2.f, 2.f}
	};
	authoredNormals.faces = {{0, 1, 2}, {3, 2, 1}};
	// tag each vertex with a normal no geometric computation would produce
	FOREACH(idxVertex, authoredNormals.vertices)
		authoredNormals.vertexNormals.emplace_back(0.f, 0.f, idxVertex+1.f);
	if (authoredNormals.RemoveDuplicatedVertices() != 1 ||
		authoredNormals.vertexNormals.size() != authoredNormals.vertices.size() ||
		authoredNormals.vertexNormals[0].z != 1.f) {
		VERBOSE("ERROR: HalfMesh bridge did not carry authored vertex normals through a vertex weld!");
		return false;
	}
	{
		// smoothing moves every vertex, so the authored values must not come back
		Mesh movedNormals(authoredNormals);
		movedNormals.Smooth(1);
		if (movedNormals.vertexNormals.size() != movedNormals.vertices.size()) {
			VERBOSE("ERROR: HalfMesh bridge did not rebuild vertex normals after smoothing!");
			return false;
		}
		bool anyStale(false);
		FOREACH(idxVertex, movedNormals.vertexNormals)
			if (movedNormals.vertexNormals[idxVertex].z == idxVertex+1.f)
				anyStale = true;
		if (anyStale) {
			VERBOSE("ERROR: HalfMesh bridge returned stale authored vertex normals after smoothing!");
			return false;
		}
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
		// A face subset is expressed against the UV-map the target already carries:
		// an index past its faces, or a target whose atlas would have to be generated
		// from scratch, has to be reported instead of baking something else.
		Mesh::FaceIdxArr outOfRangeSubset;
		outOfRangeSubset.emplace_back(rebakedGrid.faces.size());
		if (texturedGrid.TransferTexture(rebakedGrid, 1, 32, outOfRangeSubset)) {
			VERBOSE("ERROR: HalfMesh bridge accepted an out-of-range texture-transfer face subset!");
			return false;
		}
		Mesh unmappedGrid(texturedGrid);
		unmappedGrid.faceTexcoords.Release();
		unmappedGrid.texturesDiffuse.clear();
		Mesh::FaceIdxArr faceSubset;
		faceSubset.emplace_back(0);
		if (texturedGrid.TransferTexture(unmappedGrid, 1, 32, faceSubset) || unmappedGrid.HasTexture()) {
			VERBOSE("ERROR: HalfMesh bridge rebaked the whole mesh for a caller that asked for a face subset!");
			return false;
		}
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
// which part. Every solver this project has run agrees there (TetraFlow, IBFS and Boost BK reconstruct
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

// MeshRefineStep unit tests (SceneRefineStep.h): a pure unit test of the pixel-unit
// bold-driver optimizer against hand-built Terms arrays -- no images, no scene, so every
// evaluation is exact and reproducible. Two helpers mirror the class's own math bit-for-bit so
// the tests can predict what it will do without exposing any internal state:
//   RefineStepExpectedMedian mirrors MeshRefineStep::ComputeMedianScale
//   RefineStepExpectedDelta  mirrors the per-vertex move inside MeshRefineStep::Evaluate

// mirrors ComputeMedianScale() exactly: the per-scale normalizer m, derived from the
// gamma_v = |g_v|/s_v of every vertex with photoCount >= 2
static float RefineStepExpectedMedian(const MeshRefineStep::Terms& terms)
{
	FloatArr scratch;
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		if (terms.photoCount[v] < 2)
			continue;
		const MeshRefineStep::Grad g(terms.photoGrad[v]/terms.photoCount[v]);
		scratch.Insert(norm(g)/terms.footprint[v]);
	}
	return scratch.IsEmpty() ? 0.f : scratch.GetMedian();
}

// mirrors the per-vertex move inside Evaluate() exactly, given the eta (stats.step) and the
// scale's held median that the real call just used
static MeshRefineStep::Grad RefineStepExpectedDelta(const MeshRefineStep::Terms& terms, uint32_t v, float step, float median)
{
	typedef MeshRefineStep::Grad Grad;
	const float footprint(terms.footprint[v]);
	Grad photoDelta(Grad::ZERO);
	const float scale(MeshRefineStep::Kappa*median);
	if (terms.photoCount[v] >= 2 && scale > 0)
		photoDelta = terms.photoGrad[v]/(terms.photoCount[v]*scale);
	const Grad regular(terms.bilap[v]*terms.rigidity - terms.lap[v]*(1.f-terms.rigidity));
	return (photoDelta + regular*terms.regularityWeight)*-step;
}

// checks every vertex moved by exactly the formula above, and that stats.numMoved/medianPx
// match what that implies; reused after every APPLY across the subtests below
static bool RefineStepVerifyMove(const MeshRefineStep::Terms& terms, const Mesh::VertexArr& before,
	const Mesh::VertexArr& after, const MeshRefineStep::Stats& stats, float median, const char* label)
{
	typedef MeshRefineStep::Grad Grad;
	Mesh::VIndex numMoved(0);
	FloatArr appliedPx;
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		const Grad expected(RefineStepExpectedDelta(terms, v, stats.step, median));
		// compare POSITIONS, not the difference of positions: Evaluate() applies the step as
		// vertices[v] += delta, and recovering delta as after-before loses precision to
		// cancellation whenever the vertex does not sit at the origin (1.0f + 0.00132f - 1.0f is
		// not 0.00132f), which would fail this check on the implementation being correct
		if (after[v] != before[v]+Mesh::Vertex(expected)) {
			VERBOSE("ERROR: %s vertex %u moved by an unexpected delta!", label, v);
			return false;
		}
		const float len(norm(expected));
		if (len > 0) {
			++numMoved;
			if (terms.footprint[v] > 0)
				appliedPx.Insert(len/terms.footprint[v]);
		}
	}
	if (numMoved != stats.numMoved) {
		VERBOSE("ERROR: %s numMoved mismatch (expected %u, got %u)!", label, numMoved, stats.numMoved);
		return false;
	}
	const float expectedMedianPx(appliedPx.IsEmpty() ? 0.f : appliedPx.GetMedian());
	if (expectedMedianPx != stats.medianPx) {
		VERBOSE("ERROR: %s medianPx mismatch (expected %g, got %g)!", label, expectedMedianPx, stats.medianPx);
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// element-wise copies scaled by a constant factor, used only by the scale-invariance test below
static MeshRefineStep::GradArr RefineStepScaleGrads(const MeshRefineStep::GradArr& src, float factor)
{
	MeshRefineStep::GradArr dst;
	dst.Reserve(src.GetSize());
	FOREACH(i, src)
		dst.Insert(src[i]*factor);
	return dst;
}
static FloatArr RefineStepScaleFloats(const FloatArr& src, float factor)
{
	FloatArr dst;
	dst.Reserve(src.GetSize());
	FOREACH(i, src)
		dst.Insert(src[i]*factor);
	return dst;
}
static Mesh::VertexArr RefineStepScaleVertices(const Mesh::VertexArr& src, float factor)
{
	Mesh::VertexArr dst;
	dst.Reserve(src.GetSize());
	FOREACH(i, src)
		dst.Insert(src[i]*factor);
	return dst;
}
/*----------------------------------------------------------------*/

// 4-8 hand-built vertices spanning photoCount 0 (unseen), 1 (single-direction, smoothing-only)
// and >=2 (photometric+smoothing), footprint > 0 exactly where photoCount > 0, and boundary-like
// vertices with zero lap/bilap; S is irrelevant here beyond being valid, since the first
// evaluation of a scale is always accepted
static bool RefineStepAcceptMoveTest()
{
	typedef MeshRefineStep::Grad Grad;
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;

	// every vertex starts at the origin: Evaluate() only ever ADDS a delta to whatever position
	// it is given, so the stepper's decisions are independent of the starting position, and
	// starting at (0,0,0) means the applied delta (after[v]-before[v]) reconstructs bit-exactly
	// -- floating-point addition/subtraction of an exact zero never rounds, whereas a nonzero
	// base (e.g. vertex 1 at x=1) combined with a small delta would round on the way in and not
	// reconstruct bit-exactly on the way back out, breaking the exact-equality checks below
	Mesh::VertexArr vertices = {
		{0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}
	};
	const FloatArr photoCount = {0.f, 1.f, 2.f, 4.f, 2.f, 0.f};
	const FloatArr footprint  = {0.f, 1.5f, 2.f, 1.f, 0.5f, 0.f};
	const MeshRefineStep::GradArr photoGrad = {
		{0.f,0.f,0.f}, {3.f,0.f,0.f}, {4.f,0.f,0.f}, {8.f,0.f,0.f}, {1.f,0.f,0.f}, {0.f,0.f,0.f}
	};
	const MeshRefineStep::GradArr lap = {
		{0.1f,0.f,0.f}, {0.2f,0.f,0.f}, {0.f,0.f,0.f}, {0.1f,0.1f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}
	};
	const MeshRefineStep::GradArr bilap = {
		{0.05f,0.f,0.f}, {0.1f,0.f,0.f}, {0.f,0.f,0.f}, {0.05f,0.05f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}
	};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.S = 1.f;
	terms.rigidity = 0.6f;
	terms.regularityWeight = 0.2f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.3f, true);
	const Mesh::VertexArr before(vertices);
	Stats stats;
	const MeshRefineStep::Action action(stepper.Evaluate(terms, vertices, stats));
	if (action != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepAcceptMoveTest expected APPLY on the first evaluation, got %d!", (int)action);
		return false;
	}
	const float median(RefineStepExpectedMedian(terms));
	if (!RefineStepVerifyMove(terms, before, vertices, stats, median, "RefineStepAcceptMoveTest"))
		return false;
	// vertices 0 and 1 (photoCount < 2) must move by smoothing alone: their own gradient is
	// ignored, so only the regularization term applies
	for (uint32_t v=0; v<2; ++v) {
		const Grad regular(bilap[v]*terms.rigidity - lap[v]*(1.f-terms.rigidity));
		const Grad expectedSmoothOnly((regular*terms.regularityWeight)*-stats.step);
		if (vertices[v] != before[v]+Mesh::Vertex(expectedSmoothOnly)) {
			VERBOSE("ERROR: RefineStepAcceptMoveTest vertex %u (photoCount < 2) did not move by smoothing alone!", v);
			return false;
		}
	}
	// vertex 5 (photoCount 0, footprint 0, zero lap/bilap) must not move at all
	if (vertices[5] != before[5]) {
		VERBOSE("ERROR: RefineStepAcceptMoveTest vertex 5 moved despite a zero gradient and zero smoothing!");
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// two vertices identical except a 1:10 photoGrad ratio: isolating the photometric part of the
// applied delta (delta + eta*w*R, subtracting the identical regularization contribution back
// out per the header's own derivation) must show the same 1:10 ratio, proving the per-vertex
// step is proportional to the raw gradient with no per-vertex clamp
static bool RefineStepProportionalityTest()
{
	typedef MeshRefineStep::Grad Grad;
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;

	// both vertices start at the origin so that the applied step can be read back exactly (the
	// stepper never reads a vertex position, only adds to it, so the position is free to choose;
	// starting away from the origin would lose the small delta to cancellation)
	Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {0.f,0.f,0.f}};
	const FloatArr photoCount = {2.f, 2.f};
	const FloatArr footprint  = {1.f, 1.f};
	const MeshRefineStep::GradArr photoGrad = {{2.f,0.f,0.f}, {20.f,0.f,0.f}}; // ratio 1:10
	const MeshRefineStep::GradArr lap   = {{0.3f,0.f,0.f}, {0.3f,0.f,0.f}}; // identical on both
	const MeshRefineStep::GradArr bilap = {{0.1f,0.f,0.f}, {0.1f,0.f,0.f}}; // identical on both

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.S = 1.f;
	terms.rigidity = 0.5f;
	terms.regularityWeight = 0.4f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.4f, true);
	const Mesh::VertexArr before(vertices);
	Stats stats;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepProportionalityTest expected APPLY on the first evaluation!");
		return false;
	}
	const float median(RefineStepExpectedMedian(terms));
	if (!RefineStepVerifyMove(terms, before, vertices, stats, median, "RefineStepProportionalityTest"))
		return false;

	// isolate the photometric part: delta = (photoDelta + regular*w)*-step, so
	// delta + eta*w*R == -eta*photoDelta
	const Grad regularTerm((bilap[0]*terms.rigidity - lap[0]*(1.f-terms.rigidity))*terms.regularityWeight);
	const Grad photoPart0(Grad(vertices[0]-before[0]) + regularTerm*stats.step);
	const Grad photoPart1(Grad(vertices[1]-before[1]) + regularTerm*stats.step);
	if (photoPart0.x == 0.f) {
		VERBOSE("ERROR: RefineStepProportionalityTest degenerate fixture (zero photometric part)!");
		return false;
	}
	const float ratio(photoPart1.x/photoPart0.x);
	if (ABS(ratio-10.f) > 1e-4f) {
		VERBOSE("ERROR: RefineStepProportionalityTest photometric parts not in the expected 1:10 ratio (got %g)!", ratio);
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// bold-driver REJECT/undo mechanics: a worse S undoes half the last step and halves eta; the
// reference, accepted count and CONSECUTIVE reject count are untouched by a REJECT (so a later
// evaluation is still judged against the pre-reject S, not the rejected one), while the TOTAL
// reject count keeps accumulating for the log line; 4 consecutive rejections stop the scale
static bool RefineStepRejectUndoTest()
{
	typedef MeshRefineStep::Grad Grad;
	typedef MeshRefineStep::GradArr GradArr;
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	// every vertex starts at the origin so after[v]-before[v] reconstructs the applied delta
	// bit-exactly (see RefineStepAcceptMoveTest for why a nonzero base position would not)
	Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}};
	const FloatArr photoCount = {2.f, 3.f, 4.f};
	const FloatArr footprint  = {1.f, 2.f, 0.5f};
	const GradArr photoGrad = {{1.f,0.f,0.f}, {2.f,1.f,0.f}, {0.f,3.f,0.f}};
	const GradArr lap   = {{0.05f,0.f,0.02f}, {0.f,0.f,0.f}, {0.01f,0.f,0.f}};
	const GradArr bilap = {{0.02f,0.f,0.01f}, {0.f,0.f,0.f}, {0.005f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.5f;
	terms.regularityWeight = 0.3f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.25f, true);
	GradArr shadowStepPrev;
	shadowStepPrev.Resize(terms.numVertices);
	shadowStepPrev.Memset(0);
	Stats stats;

	// A: first evaluation, unconditionally accepted
	terms.S = 1.0f;
	Mesh::VertexArr expected(vertices);
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation A was not APPLY!");
		return false;
	}
	const float median(RefineStepExpectedMedian(terms));
	if (!RefineStepVerifyMove(terms, expected, vertices, stats, median, "RefineStepRejectUndoTest/A"))
		return false;
	for (uint32_t v=0; v<terms.numVertices; ++v)
		shadowStepPrev[v] = RefineStepExpectedDelta(terms, v, stats.step, median);
	expected = vertices;
	if (stats.numRejected != 0) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation A reported a reject!");
		return false;
	}

	// B: worse S -> REJECT #1 (consecutive and total both 1)
	terms.S = 1.2f;
	Action action(stepper.Evaluate(terms, vertices, stats));
	if (action != MeshRefineStep::REJECT) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation B was not REJECT!");
		return false;
	}
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		Grad& half = shadowStepPrev[v];
		half *= 0.5f;
		expected[v] -= half;
	}
	if (vertices != expected) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation B did not undo to v_prev + delta_prev/2!");
		return false;
	}
	if (stats.numRejected != 1 || stats.numAccepted != 1) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation B reject/accept counters wrong (rejected=%u, accepted=%u)!", stats.numRejected, stats.numAccepted);
		return false;
	}

	// C: still worse than the ORIGINAL reference (1.0), even though it is better than B's S
	// (1.2) -- if REJECT had updated the reference to 1.2 this would wrongly ACCEPT
	terms.S = 1.15f;
	action = stepper.Evaluate(terms, vertices, stats);
	if (action != MeshRefineStep::REJECT) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation C was not REJECT (the reference must still be A's S, not B's)!");
		return false;
	}
	for (uint32_t v=0; v<terms.numVertices; ++v) {
		Grad& half = shadowStepPrev[v];
		half *= 0.5f;
		expected[v] -= half;
	}
	if (vertices != expected) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation C did not undo to v_prev + delta_prev/2!");
		return false;
	}
	if (stats.numRejected != 2 || stats.numAccepted != 1) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation C reject/accept counters wrong (rejected=%u, accepted=%u)!", stats.numRejected, stats.numAccepted);
		return false;
	}

	// D: better than the original reference -> ACCEPT; the reference now beats the pre-reject
	// S (1.0), and the TOTAL reject count (2, from B and C) is carried, not reset
	terms.S = 0.9f;
	action = stepper.Evaluate(terms, vertices, stats);
	if (action != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation D was not APPLY!");
		return false;
	}
	if (!RefineStepVerifyMove(terms, expected, vertices, stats, median, "RefineStepRejectUndoTest/D"))
		return false;
	if (stats.numRejected != 2 || stats.numAccepted != 2) {
		VERBOSE("ERROR: RefineStepRejectUndoTest evaluation D counters wrong (rejected total=%u, accepted=%u)!", stats.numRejected, stats.numAccepted);
		return false;
	}
	for (uint32_t v=0; v<terms.numVertices; ++v)
		shadowStepPrev[v] = RefineStepExpectedDelta(terms, v, stats.step, median);
	expected = vertices;

	// E..H: four consecutive rejections must STOP on the fourth, with the total reject count
	// still accumulating past the 4 that ends the scale (it counted 2 already, from B and C)
	static const Action kExpectedAction[] = {MeshRefineStep::REJECT, MeshRefineStep::REJECT, MeshRefineStep::REJECT, MeshRefineStep::STOP};
	static const unsigned kExpectedTotal[] = {3, 4, 5, 6};
	for (unsigned i=0; i<4; ++i) {
		terms.S = 2.f;
		action = stepper.Evaluate(terms, vertices, stats);
		if (action != kExpectedAction[i]) {
			VERBOSE("ERROR: RefineStepRejectUndoTest reject-streak step %u had action %d, expected %d!", i, (int)action, (int)kExpectedAction[i]);
			return false;
		}
		for (uint32_t v=0; v<terms.numVertices; ++v) {
			Grad& half = shadowStepPrev[v];
			half *= 0.5f;
			expected[v] -= half;
		}
		if (vertices != expected) {
			VERBOSE("ERROR: RefineStepRejectUndoTest reject-streak step %u did not undo to v_prev + delta_prev/2!", i);
			return false;
		}
		if (stats.numRejected != kExpectedTotal[i]) {
			VERBOSE("ERROR: RefineStepRejectUndoTest reject-streak step %u total-reject count wrong (got %u, expected %u)!", i, stats.numRejected, kExpectedTotal[i]);
			return false;
		}
	}
	return true;
}
/*----------------------------------------------------------------*/

// STOP rule (a): Patience -- once MinIters accepted evaluations have run, 3 consecutive stalled
// ACCEPTs (relative S decrease < 1e-3, including a worsening S) STOP the scale BEFORE moving
// any vertex -- the stall check runs before the per-vertex loop in Evaluate()
static bool RefineStepPatienceStopTest()
{
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	Mesh::VertexArr vertices = {{0.f,0.f,0.f}};
	const FloatArr photoCount = {2.f};
	const FloatArr footprint  = {1.f};
	const MeshRefineStep::GradArr photoGrad = {{0.01f,0.f,0.f}};
	const MeshRefineStep::GradArr lap   = {{0.f,0.f,0.f}};
	const MeshRefineStep::GradArr bilap = {{0.f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.f;
	terms.regularityWeight = 0.f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.1f, true);

	// each S decreases by well under 0.1% relative to the previous accepted one
	static const float kS[] = {1.0f, 0.9998f, 0.9997f, 0.9996f};
	static const Action kExpectedAction[] = {
		MeshRefineStep::APPLY, MeshRefineStep::APPLY, MeshRefineStep::APPLY, MeshRefineStep::STOP
	};
	Stats stats;
	Mesh::VertexArr before(vertices);
	for (unsigned i=0; i<4; ++i) {
		terms.S = kS[i];
		before = vertices;
		const Action action(stepper.Evaluate(terms, vertices, stats));
		if (action != kExpectedAction[i]) {
			VERBOSE("ERROR: RefineStepPatienceStopTest step %u had action %d, expected %d!", i, (int)action, (int)kExpectedAction[i]);
			return false;
		}
	}
	// the terminal STOP must fire before touching the vertex or the move stats: it is a
	// stalled-progress judgement, not a converged-step judgement (contrast with the
	// full-step-median rule below, which stops AFTER applying its last, tiny move)
	if (vertices != before || stats.numMoved != 0 || stats.medianPx != 0.f) {
		VERBOSE("ERROR: RefineStepPatienceStopTest STOP evaluation moved vertices or reported a move!");
		return false;
	}
	if (stats.numAccepted != 4) {
		VERBOSE("ERROR: RefineStepPatienceStopTest expected 4 accepted evaluations at STOP, got %u!", stats.numAccepted);
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// STOP rule (b): full-step median -- once eta stays pinned at StepMax, medianPx IS the
// full-step value; feeding a gradient much smaller than the one that fixed the scale's median
// (held from the first evaluation) drives it under StepStop once MinIters accepted evaluations
// have run, and unlike the patience rule this STOP fires AFTER applying that last (tiny) move
static bool RefineStepConvergenceStopTest()
{
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	Mesh::VertexArr vertices = {{0.f,0.f,0.f}};
	MeshRefineStep::GradArr photoGrad = {{0.02f,0.f,0.f}};
	const FloatArr photoCount = {2.f};
	const FloatArr footprint  = {1.f};
	const MeshRefineStep::GradArr lap   = {{0.f,0.f,0.f}};
	const MeshRefineStep::GradArr bilap = {{0.f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.f;
	terms.regularityWeight = 0.f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	// eta starts and stays at StepMax (1.1x of 1.0 clamps back to 1.0), so medianAtFullStep
	// equals medianPx exactly and the test can drive it directly through the gradient
	stepper.Reset(terms.numVertices, MeshRefineStep::StepMax, true);

	// evaluation 1: sets the scale's median from a normal-sized gradient
	terms.S = 1.0f;
	Stats stats;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepConvergenceStopTest evaluation 1 was not APPLY!");
		return false;
	}
	if (stats.step != MeshRefineStep::StepMax) {
		VERBOSE("ERROR: RefineStepConvergenceStopTest eta did not stay clamped at StepMax!");
		return false;
	}

	// evaluations 2 and 3: a gradient 100x smaller than the one that fixed the median, so the
	// resulting per-vertex step is a small fraction of a pixel while S keeps improving by a
	// large margin (so the patience rule never fires instead); medianPx is already under
	// StepStop by evaluation 2, but MinIters (3 accepted evaluations) still gates the STOP to
	// evaluation 3
	photoGrad[0] = MeshRefineStep::Grad(0.0002f, 0.f, 0.f);
	static const float kS[] = {0.5f, 0.3f};
	static const Action kExpectedAction[] = {MeshRefineStep::APPLY, MeshRefineStep::STOP};
	for (unsigned i=0; i<2; ++i) {
		terms.S = kS[i];
		const Mesh::VertexArr before(vertices);
		const Action action(stepper.Evaluate(terms, vertices, stats));
		if (action != kExpectedAction[i]) {
			VERBOSE("ERROR: RefineStepConvergenceStopTest step %u had action %d, expected %d!", i, (int)action, (int)kExpectedAction[i]);
			return false;
		}
		if (vertices == before) {
			VERBOSE("ERROR: RefineStepConvergenceStopTest step %u did not move the vertex (the full-step rule stops AFTER its last move)!", i);
			return false;
		}
		if (stats.medianPx >= MeshRefineStep::StepStop) {
			VERBOSE("ERROR: RefineStepConvergenceStopTest step %u medianPx %g did not fall under StepStop!", i, stats.medianPx);
			return false;
		}
	}
	if (stats.numAccepted != 3) {
		VERBOSE("ERROR: RefineStepConvergenceStopTest expected 3 accepted evaluations at STOP, got %u!", stats.numAccepted);
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// the fixed-step control arm (boldDriver=false) never rejects and never changes eta, even fed
// the exact same worsening-then-improving S sequence that made RefineStepRejectUndoTest reject
// four times in a row; the stall/converged STOP rules are NOT gated on boldDriver, so this loop
// may still stop as soon as either fires -- what it must never do is REJECT or move eta
static bool RefineStepFixedArmTest()
{
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	// every vertex starts at the origin so after[v]-before[v] reconstructs the applied delta
	// bit-exactly (see RefineStepAcceptMoveTest for why a nonzero base position would not)
	Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}};
	const FloatArr photoCount = {2.f, 3.f, 4.f};
	const FloatArr footprint  = {1.f, 2.f, 0.5f};
	const MeshRefineStep::GradArr photoGrad = {{1.f,0.f,0.f}, {2.f,1.f,0.f}, {0.f,3.f,0.f}};
	const MeshRefineStep::GradArr lap   = {{0.05f,0.f,0.02f}, {0.f,0.f,0.f}, {0.01f,0.f,0.f}};
	const MeshRefineStep::GradArr bilap = {{0.02f,0.f,0.01f}, {0.f,0.f,0.f}, {0.005f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.5f;
	terms.regularityWeight = 0.3f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	constexpr float stepInit = 0.25f;
	stepper.Reset(terms.numVertices, stepInit, false); // boldDriver = false

	static const float kS[] = {1.0f, 1.2f, 1.15f, 0.9f, 2.0f, 2.0f, 2.0f, 2.0f};
	float median(-1.f);
	Stats stats;
	for (float s : kS) {
		terms.S = s;
		const Mesh::VertexArr before(vertices);
		const Action action(stepper.Evaluate(terms, vertices, stats));
		if (action == MeshRefineStep::REJECT) {
			VERBOSE("ERROR: RefineStepFixedArmTest the fixed-step arm rejected an evaluation!");
			return false;
		}
		if (stats.step != stepInit) {
			VERBOSE("ERROR: RefineStepFixedArmTest eta changed under the fixed-step arm (%g != %g)!", stats.step, stepInit);
			return false;
		}
		if (action == MeshRefineStep::APPLY) {
			if (median < 0.f)
				median = RefineStepExpectedMedian(terms);
			if (!RefineStepVerifyMove(terms, before, vertices, stats, median, "RefineStepFixedArmTest"))
				return false;
		}
		if (action == MeshRefineStep::STOP)
			break;
	}
	return true;
}
/*----------------------------------------------------------------*/

// scale invariance: an identical evaluation script run twice, the second time with footprint,
// photoGrad, lap, bilap and the vertex positions all multiplied by an exact power of two (128),
// must produce the identical Action sequence, the identical eta after every call, and identical
// per-vertex pixel steps -- scaling by a power of two is exact under IEEE-754 (only the exponent
// shifts), so every intermediate (multiply, divide, sqrt, add) stays bit-exact and the two runs
// can be compared exactly rather than within a tolerance; photoCount, S, rigidity and
// regularityWeight are pixel/ZNCC-space quantities and are deliberately left unscaled
static bool RefineStepScaleInvarianceTest()
{
	typedef MeshRefineStep::Grad Grad;
	typedef MeshRefineStep::GradArr GradArr;
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	const Mesh::VertexArr vertices1base = {{0.f,0.f,0.f}, {1.f,0.f,0.f}, {2.f,1.f,0.f}, {0.f,2.f,0.f}};
	const FloatArr photoCount = {0.f, 2.f, 3.f, 1.f};
	const FloatArr footprint1 = {0.f, 1.5f, 2.f, 0.8f};
	const GradArr photoGrad1 = {{0.f,0.f,0.f}, {4.f,0.f,0.f}, {3.f,4.f,0.f}, {2.f,0.f,0.f}};
	const GradArr lap1   = {{0.1f,0.f,0.f}, {0.05f,0.05f,0.f}, {0.f,0.f,0.f}, {0.02f,0.f,0.f}};
	const GradArr bilap1 = {{0.02f,0.f,0.f}, {0.01f,0.01f,0.f}, {0.f,0.f,0.f}, {0.005f,0.f,0.f}};
	constexpr float rigidity = 0.6f, regularityWeight = 0.25f, stepInit = 0.3f, kScale = 128.f;

	Mesh::VertexArr vertices1(vertices1base);
	Mesh::VertexArr vertices2(RefineStepScaleVertices(vertices1base, kScale));
	const FloatArr footprint2(RefineStepScaleFloats(footprint1, kScale));
	const GradArr photoGrad2(RefineStepScaleGrads(photoGrad1, kScale));
	const GradArr lap2(RefineStepScaleGrads(lap1, kScale));
	const GradArr bilap2(RefineStepScaleGrads(bilap1, kScale));

	static const float kS[] = {1.0f, 1.3f, 0.85f, 0.7f, 0.6f, 0.55f, 0.52f};
	const uint32_t numVertices((uint32_t)vertices1.GetSize());

	MeshRefineStep stepper1, stepper2;
	stepper1.Reset(numVertices, stepInit, true);
	stepper2.Reset(numVertices, stepInit, true);

	for (float s : kS) {
		Terms terms1;
		terms1.photoGrad = photoGrad1.Begin();
		terms1.photoCount = photoCount.Begin();
		terms1.footprint = footprint1.Begin();
		terms1.lap = lap1.Begin();
		terms1.bilap = bilap1.Begin();
		terms1.S = s;
		terms1.rigidity = rigidity;
		terms1.regularityWeight = regularityWeight;
		terms1.numVertices = numVertices;
		terms1.alternating = false;

		Terms terms2(terms1);
		terms2.photoGrad = photoGrad2.Begin();
		terms2.footprint = footprint2.Begin();
		terms2.lap = lap2.Begin();
		terms2.bilap = bilap2.Begin();

		const Mesh::VertexArr before1(vertices1), before2(vertices2);
		Stats stats1, stats2;
		const Action action1(stepper1.Evaluate(terms1, vertices1, stats1));
		const Action action2(stepper2.Evaluate(terms2, vertices2, stats2));
		if (action1 != action2) {
			VERBOSE("ERROR: RefineStepScaleInvarianceTest action diverged at S=%g (%d vs %d)!", s, (int)action1, (int)action2);
			return false;
		}
		if (stats1.step != stats2.step) {
			VERBOSE("ERROR: RefineStepScaleInvarianceTest eta diverged at S=%g (%g vs %g)!", s, stats1.step, stats2.step);
			return false;
		}
		for (uint32_t v=0; v<numVertices; ++v) {
			const Grad delta1(vertices1[v]-before1[v]), delta2(vertices2[v]-before2[v]);
			const float fp1(footprint1[v]), fp2(footprint2[v]);
			const float px1(fp1 > 0.f ? norm(delta1)/fp1 : 0.f);
			const float px2(fp2 > 0.f ? norm(delta2)/fp2 : 0.f);
			if (px1 != px2) {
				VERBOSE("ERROR: RefineStepScaleInvarianceTest per-vertex pixel step diverged at S=%g vertex %u!", s, v);
				return false;
			}
			if (vertices2[v] != Grad(vertices1[v])*kScale) {
				VERBOSE("ERROR: RefineStepScaleInvarianceTest vertex %u lost exact x128 correspondence at S=%g!", v, s);
				return false;
			}
		}
		if (action1 == MeshRefineStep::STOP)
			break;
	}
	return true;
}
/*----------------------------------------------------------------*/

// after TopologyChanged(), the next evaluation is accepted unconditionally even though its S is
// worse than the current reference -- exactly like the very first evaluation of a scale
static bool RefineStepTopologyChangedTest()
{
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	// every vertex starts at the origin so after[v]-before[v] reconstructs the applied delta
	// bit-exactly (see RefineStepAcceptMoveTest for why a nonzero base position would not)
	Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}};
	const FloatArr photoCount = {2.f, 3.f, 0.f};
	const FloatArr footprint  = {1.f, 0.5f, 0.f};
	const MeshRefineStep::GradArr photoGrad = {{2.f,0.f,0.f}, {1.f,2.f,0.f}, {0.f,0.f,0.f}};
	const MeshRefineStep::GradArr lap   = {{0.f,0.f,0.f}, {0.1f,0.f,0.f}, {0.05f,0.f,0.f}};
	const MeshRefineStep::GradArr bilap = {{0.f,0.f,0.f}, {0.02f,0.f,0.f}, {0.01f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.4f;
	terms.regularityWeight = 0.2f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = false;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.2f, true);
	Stats stats;

	terms.S = 1.0f;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepTopologyChangedTest evaluation 1 was not APPLY!");
		return false;
	}
	const float median(RefineStepExpectedMedian(terms));

	terms.S = 0.5f; // improves the reference to 0.5
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepTopologyChangedTest evaluation 2 was not APPLY!");
		return false;
	}

	// the topology changed under the caller (e.g. a planar-vertex removal): the undo buffer no
	// longer indexes the same vertices and the next S cannot be judged against 0.5
	stepper.TopologyChanged(terms.numVertices);

	terms.S = 2.0f; // far worse than the current reference (0.5) -- would REJECT without the call above
	const Mesh::VertexArr before(vertices);
	const Action action(stepper.Evaluate(terms, vertices, stats));
	if (action != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepTopologyChangedTest a worse S was not unconditionally accepted after TopologyChanged!");
		return false;
	}
	if (!RefineStepVerifyMove(terms, before, vertices, stats, median, "RefineStepTopologyChangedTest"))
		return false;
	return true;
}
/*----------------------------------------------------------------*/

// alternating runs compare S against the reference of the same evaluation-index parity: an S
// that would REJECT against the OTHER parity's reference must still ACCEPT when it beats its
// OWN parity's reference, and the two references stay independent afterwards
static bool RefineStepAlternatingTest()
{
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {1.f,0.f,0.f}};
	const FloatArr photoCount = {2.f, 2.f};
	const FloatArr footprint  = {1.f, 1.f};
	const MeshRefineStep::GradArr photoGrad = {{1.f,0.f,0.f}, {2.f,0.f,0.f}};
	const MeshRefineStep::GradArr lap   = {{0.f,0.f,0.f}, {0.f,0.f,0.f}};
	const MeshRefineStep::GradArr bilap = {{0.f,0.f,0.f}, {0.f,0.f,0.f}};

	Terms terms;
	terms.photoGrad = photoGrad.Begin();
	terms.photoCount = photoCount.Begin();
	terms.footprint = footprint.Begin();
	terms.lap = lap.Begin();
	terms.bilap = bilap.Begin();
	terms.rigidity = 0.5f;
	terms.regularityWeight = 0.2f;
	terms.numVertices = (uint32_t)vertices.GetSize();
	terms.alternating = true;

	MeshRefineStep stepper;
	stepper.Reset(terms.numVertices, 0.2f, true);
	Stats stats;

	// index 0 (even parity): first ever evaluation, unconditionally accepted; scoreRef = 1.0
	terms.S = 1.0f;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepAlternatingTest evaluation 0 was not APPLY!");
		return false;
	}
	// index 1 (odd parity): scoreRefAlt starts at FLT_MAX, so this is unconditionally accepted
	// too -- deliberately made a WORSE S than the even parity's reference to prove the two
	// references are compared independently, not against a single shared one
	terms.S = 0.5f;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepAlternatingTest evaluation 1 was not APPLY!");
		return false;
	}
	// index 2 (even parity): 0.7 is WORSE than the odd parity's reference (0.5) but still
	// BETTER than the even parity's own reference (1.0) -- must ACCEPT
	terms.S = 0.7f;
	if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
		VERBOSE("ERROR: RefineStepAlternatingTest evaluation 2 (worse than the other parity's reference) was not ACCEPTed!");
		return false;
	}
	// index 3 (odd parity): 0.6 is WORSE than the odd parity's own reference (0.5), unaffected
	// by evaluation 2's even-parity accept -- must REJECT
	terms.S = 0.6f;
	const Action action(stepper.Evaluate(terms, vertices, stats));
	if (action != MeshRefineStep::REJECT) {
		VERBOSE("ERROR: RefineStepAlternatingTest evaluation 3 was not REJECTed against its own (untouched) parity reference!");
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

// ResetStall(): a caller running a second phase within the same scale must get a fresh stall
// count, not one the first phase's tail already primed to STOP after a single evaluation -- but
// the consecutive-reject streak is untouched by the call, so a phase that starts right where the
// first one gave up on its rejections keeps giving up immediately (see ResetStall()'s comment:
// resetting the streak too measured -0.0048 mean F1 on the T&T set)
static bool RefineStepResetStallTest()
{
	typedef MeshRefineStep::GradArr GradArr;
	typedef MeshRefineStep::Terms Terms;
	typedef MeshRefineStep::Stats Stats;
	typedef MeshRefineStep::Action Action;

	// part 1: the consecutive-reject streak survives ResetStall()
	{
		Mesh::VertexArr vertices = {{0.f,0.f,0.f}, {0.f,0.f,0.f}, {0.f,0.f,0.f}};
		const FloatArr photoCount = {2.f, 3.f, 4.f};
		const FloatArr footprint  = {1.f, 2.f, 0.5f};
		const GradArr photoGrad = {{1.f,0.f,0.f}, {2.f,1.f,0.f}, {0.f,3.f,0.f}};
		const GradArr lap   = {{0.05f,0.f,0.02f}, {0.f,0.f,0.f}, {0.01f,0.f,0.f}};
		const GradArr bilap = {{0.02f,0.f,0.01f}, {0.f,0.f,0.f}, {0.005f,0.f,0.f}};

		Terms terms;
		terms.photoGrad = photoGrad.Begin();
		terms.photoCount = photoCount.Begin();
		terms.footprint = footprint.Begin();
		terms.lap = lap.Begin();
		terms.bilap = bilap.Begin();
		terms.rigidity = 0.5f;
		terms.regularityWeight = 0.3f;
		terms.numVertices = (uint32_t)vertices.GetSize();
		terms.alternating = false;

		MeshRefineStep stepper;
		stepper.Reset(terms.numVertices, 0.25f, true);
		Stats stats;

		// one accepted evaluation, then a reject streak that ends the scale on MaxRejects
		terms.S = 1.f;
		if (stepper.Evaluate(terms, vertices, stats) != MeshRefineStep::APPLY) {
			VERBOSE("ERROR: RefineStepResetStallTest first evaluation was not APPLY!");
			return false;
		}
		terms.S = 2.f;
		for (unsigned i=0; i<MeshRefineStep::MaxRejects; ++i) {
			const Action action(stepper.Evaluate(terms, vertices, stats));
			const Action expected(i+1 < MeshRefineStep::MaxRejects ? MeshRefineStep::REJECT : MeshRefineStep::STOP);
			if (action != expected) {
				VERBOSE("ERROR: RefineStepResetStallTest reject-streak step %u had action %d, expected %d!", i, (int)action, (int)expected);
				return false;
			}
		}

		// ResetStall() leaves the reject streak alone: it is still at MaxRejects, so the very
		// next rejected evaluation STOPs immediately instead of getting a fresh REJECT budget
		stepper.ResetStall();
		const Action action(stepper.Evaluate(terms, vertices, stats));
		if (action != MeshRefineStep::STOP) {
			VERBOSE("ERROR: RefineStepResetStallTest ResetStall() cleared the reject streak (expected it to carry over and STOP)!");
			return false;
		}
	}

	// part 2: ResetStall() clears a stall count already primed to STOP -- same single-vertex
	// setup and S sequence as RefineStepPatienceStopTest, which reaches the Patience STOP on the
	// 4th evaluation
	{
		Mesh::VertexArr vertices = {{0.f,0.f,0.f}};
		const FloatArr photoCount = {2.f};
		const FloatArr footprint  = {1.f};
		const GradArr photoGrad = {{0.01f,0.f,0.f}};
		const GradArr lap   = {{0.f,0.f,0.f}};
		const GradArr bilap = {{0.f,0.f,0.f}};

		Terms terms;
		terms.photoGrad = photoGrad.Begin();
		terms.photoCount = photoCount.Begin();
		terms.footprint = footprint.Begin();
		terms.lap = lap.Begin();
		terms.bilap = bilap.Begin();
		terms.rigidity = 0.f;
		terms.regularityWeight = 0.f;
		terms.numVertices = (uint32_t)vertices.GetSize();
		terms.alternating = false;

		MeshRefineStep stepper;
		stepper.Reset(terms.numVertices, 0.1f, true);
		Stats stats;

		// each S decreases by well under 0.1% relative to the previous accepted one
		static const float kS[] = {1.0f, 0.9998f, 0.9997f, 0.9996f};
		for (unsigned i=0; i<4; ++i) {
			terms.S = kS[i];
			const Action action(stepper.Evaluate(terms, vertices, stats));
			const Action expected(i < 3 ? MeshRefineStep::APPLY : MeshRefineStep::STOP);
			if (action != expected) {
				VERBOSE("ERROR: RefineStepResetStallTest stall-priming step %u had action %d, expected %d!", i, (int)action, (int)expected);
				return false;
			}
		}

		// ResetStall() clears the stall count: one more equally-stalled evaluation only reaches
		// numStalled == 1, well under Patience, so it applies instead of stopping again
		stepper.ResetStall();
		terms.S = 0.9995f;
		const Action action(stepper.Evaluate(terms, vertices, stats));
		if (action != MeshRefineStep::APPLY) {
			VERBOSE("ERROR: RefineStepResetStallTest ResetStall() did not clear a stall count primed to STOP!");
			return false;
		}
	}
	return true;
}
/*----------------------------------------------------------------*/

// pure unit test of MeshRefineStep against hand-built Terms arrays: no images, no scene,
// milliseconds to run. Each helper above/below isolates one documented invariant from
// SceneRefineStep.h.
bool MeshRefineStepTest()
{
	if (!RefineStepAcceptMoveTest())
		return false;
	if (!RefineStepProportionalityTest())
		return false;
	if (!RefineStepRejectUndoTest())
		return false;
	if (!RefineStepPatienceStopTest())
		return false;
	if (!RefineStepConvergenceStopTest())
		return false;
	if (!RefineStepFixedArmTest())
		return false;
	if (!RefineStepScaleInvarianceTest())
		return false;
	if (!RefineStepTopologyChangedTest())
		return false;
	if (!RefineStepAlternatingTest())
		return false;
	if (!RefineStepResetStallTest())
		return false;
	return true;
}
/*----------------------------------------------------------------*/

// MeshRefineWindowStats unit tests (SceneRefineCommon.h): the two pure scalar reducers both
// backends compute identically -- the six masked window sums reduced to means/variances/covariance
// with the two rejection gates, and the per-pixel ZNCC with its derivative. No scene, no images.
// Every expectation is derived independently of the production expressions: closed-form identities
// of an affine window (B = 2A+1 gives muB = 2muA+1, varB = 4varA, cov = 2varA, hence ZNCC = 1),
// the documented floor and gate boundaries, and a central finite difference of ZNCC itself.

// the six masked sums of one window, accumulated in double and handed over as the floats the
// production reducer takes; this is the input side of WindowStatsFromSums, not its formula
struct RefineWindowSums {
	float n, sA, sB, sAA, sBB, sAB;
};
static RefineWindowSums RefineComputeSums(const FloatArr& A, const FloatArr& B)
{
	ASSERT(A.GetSize() == B.GetSize());
	double n(0), sA(0), sB(0), sAA(0), sBB(0), sAB(0);
	FOREACH(i, A) {
		const double a(A[i]), b(B[i]);
		n += 1; sA += a; sB += b; sAA += a*a; sBB += b*b; sAB += a*b;
	}
	return RefineWindowSums{(float)n, (float)sA, (float)sB, (float)sAA, (float)sBB, (float)sAB};
}
static bool RefineReduceSums(const RefineWindowSums& s, float gateMeanDiff, float gateVarRatio, Refine::WindowStats& stats)
{
	return Refine::WindowStatsFromSums(s.n, s.sA, s.sB, s.sAA, s.sBB, s.sAB, gateMeanDiff, gateVarRatio, stats);
}
// deterministic values in [0,1): a tiny LCG, so the fixtures below are the same on every platform
// and compiler without pulling in a random engine whose sequence is implementation-defined
static void RefineFillRandom(FloatArr& arr, uint32_t& state, unsigned count)
{
	arr.Empty();
	arr.Reserve(count);
	while (count-- > 0) {
		state = state*1664525u + 1013904223u;
		arr.Insert((float)(state>>8)/(float)(1u<<24));
	}
}
static bool RefineCloseRel(float got, float expected, float tol, const char* label)
{
	if (ABS(got-expected) <= tol*ABS(expected))
		return true;
	VERBOSE("ERROR: %s expected %g, got %g!", label, expected, got);
	return false;
}

bool MeshRefineWindowStatsTest()
{
	typedef Refine::WindowStats WindowStats;

	// A takes 1/8 steps in [0,1], every value and product exactly representable, so the sums carry
	// no accumulation error of their own; B = 2A+1 is the affine twin
	FloatArr A(0, Refine::WindowArea), B(0, Refine::WindowArea);
	for (int i=0; i<Refine::WindowArea; ++i) {
		const float a((float)((i*7)%9)*0.125f);
		A.Insert(a);
		B.Insert(2.f*a+1.f);
	}
	const RefineWindowSums sums(RefineComputeSums(A, B));
	WindowStats s;

	// (1) MinWindowCount: a window with fewer valid samples is rejected outright
	{
		RefineWindowSums few(sums);
		few.n = (float)(Refine::MinWindowCount-1);
		if (RefineReduceSums(few, 0.f, 0.f, s)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest accepted a window below MinWindowCount!");
			return false;
		}
		few.n = (float)Refine::MinWindowCount;
		if (!RefineReduceSums(few, 0.f, 0.f, s)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest rejected a window at exactly MinWindowCount!");
			return false;
		}
	}

	// (2) affine window identities
	if (!RefineReduceSums(sums, 0.f, 0.f, s)) {
		VERBOSE("ERROR: MeshRefineWindowStatsTest rejected the affine window!");
		return false;
	}
	if (!RefineCloseRel(s.muB, 2.f*s.muA+1.f, 1e-5f, "affine muB"))
		return false;
	if (!RefineCloseRel(s.varB, 4.f*s.varA, 1e-5f, "affine varB"))
		return false;
	if (!RefineCloseRel(s.cov, 2.f*s.varA, 1e-5f, "affine cov"))
		return false;

	// (3) the variance floor: a constant window has zero variance on both sides
	{
		FloatArr C(0, Refine::WindowArea);
		for (int i=0; i<Refine::WindowArea; ++i)
			C.Insert(0.5f);
		WindowStats sc;
		if (!RefineReduceSums(RefineComputeSums(C, C), 0.f, 0.f, sc)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest rejected the constant window!");
			return false;
		}
		if (sc.varA != 1e-4f || sc.varB != 1e-4f) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest constant window variances not floored (varA=%g, varB=%g)!", sc.varA, sc.varB);
			return false;
		}
	}

	// (4) the mean gate is a strict "greater than": it passes at exact equality and rejects for
	// any smaller gate (muA-muB is 1+muA on the affine window, so the difference is large)
	{
		const float meanDiff(ABS(s.muA-s.muB));
		WindowStats sg;
		if (!RefineReduceSums(sums, meanDiff, 0.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest mean gate rejected at exact equality!");
			return false;
		}
		if (RefineReduceSums(sums, meanDiff*0.5f, 0.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest mean gate accepted a window past the gate!");
			return false;
		}
	}

	// (5) the variance-ratio gate, in the varA > gate*varB direction: swapping the two sides of
	// the affine window makes varA exactly 4x varB
	{
		const RefineWindowSums swapped{sums.n, sums.sB, sums.sA, sums.sBB, sums.sAA, sums.sAB};
		WindowStats sg;
		if (!RefineReduceSums(swapped, 0.f, 0.f, sg) || !RefineCloseRel(sg.varA, 4.f*sg.varB, 1e-5f, "swapped varA"))
			return false;
		if (RefineReduceSums(swapped, 0.f, 2.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest variance gate accepted varA > 2*varB!");
			return false;
		}
		if (!RefineReduceSums(swapped, 0.f, 8.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest variance gate rejected varA < 8*varB!");
			return false;
		}
		// both gates disabled at <= 0, on the very window that fails both when they are on
		if (RefineReduceSums(swapped, 0.4f, 2.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest gates did not reject the window they were sized for!");
			return false;
		}
		if (!RefineReduceSums(swapped, 0.f, 0.f, sg) || !RefineReduceSums(swapped, -1.f, -1.f, sg)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest gates were not disabled at <= 0!");
			return false;
		}
	}

	// (6) ZNCC of the affine window is 1, and of the negated one -1
	{
		float zncc, dzncc, conf;
		Refine::ZnccAndDerivative(s, sums.n, A[24], B[24], zncc, dzncc, conf);
		if (ABS(zncc-1.f) > 1e-5f) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest ZNCC of B = 2A+1 is %g, expected 1!", zncc);
			return false;
		}
		FloatArr N(0, Refine::WindowArea);
		FOREACH(i, A)
			N.Insert(-A[i]);
		WindowStats sn;
		if (!RefineReduceSums(RefineComputeSums(A, N), 0.f, 0.f, sn)) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest rejected the negated window!");
			return false;
		}
		Refine::ZnccAndDerivative(sn, sums.n, A[24], N[24], zncc, dzncc, conf);
		if (ABS(zncc+1.f) > 1e-5f) {
			VERBOSE("ERROR: MeshRefineWindowStatsTest ZNCC of B = -A is %g, expected -1!", zncc);
			return false;
		}
	}

	// (7) random windows: |ZNCC| <= 1 despite the variance floor (which can only grow the
	// denominator), the WindowArea/n factor, and the derivative against a central difference
	{
		uint32_t state(12345u);
		for (unsigned trial=0; trial<3; ++trial) {
			FloatArr R, Q;
			RefineFillRandom(R, state, Refine::WindowArea);
			RefineFillRandom(Q, state, Refine::WindowArea);
			const RefineWindowSums sr(RefineComputeSums(R, Q));
			WindowStats sq;
			if (!RefineReduceSums(sr, 0.f, 0.f, sq)) {
				VERBOSE("ERROR: MeshRefineWindowStatsTest rejected random window %u!", trial);
				return false;
			}
			const int centre(Refine::WindowArea/2);
			float zncc, dzncc, conf;
			Refine::ZnccAndDerivative(sq, sr.n, R[centre], Q[centre], zncc, dzncc, conf);
			if (!(ABS(zncc) <= 1.f+1e-5f)) {
				VERBOSE("ERROR: MeshRefineWindowStatsTest random window %u gave |ZNCC| = %g > 1!", trial, ABS(zncc));
				return false;
			}
			// the same statistics with fewer valid samples restore the magnitude a full window
			// would have produced: dzncc scales exactly as WindowArea/n
			float znccPartial, dznccPartial, confPartial;
			Refine::ZnccAndDerivative(sq, 30.f, R[centre], Q[centre], znccPartial, dznccPartial, confPartial);
			if (!RefineCloseRel(dznccPartial, dzncc*((float)Refine::WindowArea/30.f), 1e-6f, "partial-window dZNCC"))
				return false;
			// dZNCC/dB_p = [(A_p-muA)/sqrt(varA varB) - ZNCC (B_p-muB)/varB]/n (the mean's
			// dependence on B_p cancels because sum(A-muA) = 0), and dzncc is that times
			// -conf*WindowArea/n, so -dzncc/(conf*WindowArea) is the derivative itself
			const float analytic(-dzncc/(conf*(float)Refine::WindowArea));
			const float h(1e-3f);
			FloatArr Qp(Q), Qm(Q);
			Qp[centre] += h;
			Qm[centre] -= h;
			WindowStats sp, sm;
			const RefineWindowSums srp(RefineComputeSums(R, Qp)), srm(RefineComputeSums(R, Qm));
			if (!RefineReduceSums(srp, 0.f, 0.f, sp) || !RefineReduceSums(srm, 0.f, 0.f, sm)) {
				VERBOSE("ERROR: MeshRefineWindowStatsTest rejected a perturbed random window %u!", trial);
				return false;
			}
			float znccP, znccM, dummyD, dummyC;
			Refine::ZnccAndDerivative(sp, srp.n, R[centre], Qp[centre], znccP, dummyD, dummyC);
			Refine::ZnccAndDerivative(sm, srm.n, R[centre], Qm[centre], znccM, dummyD, dummyC);
			const float fd((znccP-znccM)/(2.f*h));
			// relative to 1e-3, plus an absolute floor: ZNCC is an O(1) float, so the central
			// difference of two of them divided by 2e-3 carries a few 1e-4 of cancellation noise
			// no matter how right the derivative is
			if (ABS(analytic-fd) > 1e-3f*ABS(fd)+1e-3f) {
				VERBOSE("ERROR: MeshRefineWindowStatsTest random window %u derivative %g disagrees with the finite difference %g!", trial, analytic, fd);
				return false;
			}
		}
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
	// every DEFVAR_OPTION starts at zero and only init()/update() install the declared defaults,
	// so the refiner's configuration space needs the same treatment as OPTDENSE
	OPTREFINE::init();
	OPTREFINE::update();
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


// Synthetic end-to-end test of the mesh-refinement photometric pipeline: builds a textured flat
// plane as ground truth, renders it into 4 known cameras to produce the "photographs"
// Scene::RefineMesh/RefineMeshCUDA read from disk, perturbs a coarse copy of the same plane to
// serve as the input mesh, and asserts the refiner recovers the flat plane to within a fraction
// of a pixel's footprint. Unlike MeshRefineStepTest/MeshRefineWindowStatsTest above (hand-built
// fixtures, no images), this exercises the rasterize/warp/window-stats/ZNCC-gradient/smoothing/
// stepper chain end to end, the same way PipelineTest exercises densify/reconstruct/texture.

// fixture geometry: a textured GT plane spanning [-sceneScale,sceneScale]^2 at z=0, rendered
// into 4 pinhole cameras above it, looking down at the origin
constexpr unsigned RefineSynthGTGridN = 40; // ground-truth mesh grid resolution (vertices/axis)
constexpr unsigned RefineSynthInputGridN = 10; // pre-refine (input) mesh grid resolution
constexpr unsigned RefineSynthTexSize = 512; // texture image size, both axes
constexpr unsigned RefineSynthImgWidth = 640, RefineSynthImgHeight = 480;
constexpr float RefineSynthFocal = 600.f; // pinhole focal length, pixels
constexpr float RefineSynthCamHeight = 3.f; // camera height above the plane, x sceneScale
constexpr float RefineSynthCamOffset = 0.4f; // camera (x,y) offset from the plane centre, x sceneScale

// deterministic LCG in [0,1), the same tiny generator MeshRefineWindowStatsTest's
// RefineFillRandom above uses: no <random> engine, whose sequence is implementation-defined, so
// the fixture is byte-identical on every platform and compiler
static inline float RefineSynthNextUniform(uint32_t& state)
{
	state = state*1664525u + 1013904223u;
	return (float)(state>>8)/(float)(1u<<24);
}
// Irwin-Hall approximation of a zero-mean, unit-variance Gaussian: the sum of 12 uniforms in
// [0,1) has mean 6 and variance 1
static float RefineSynthGaussian(uint32_t& state, float sigma)
{
	float sum(0.f);
	for (int k=0; k<12; ++k)
		sum += RefineSynthNextUniform(state);
	return (sum-6.f)*sigma;
}

// deterministic seeded texture: blurred noise plus a small-period checkerboard, so every
// Refine::WindowArea (7x7) window has non-zero variance regardless of where it lands
static void GenerateRefineSyntheticTexture(Image8U3& texture, unsigned texSize)
{
	FloatArr noise; noise.resize(texSize*texSize);
	uint32_t state(2463534242u);
	FOREACH(k, noise)
		noise[k] = RefineSynthNextUniform(state);
	const auto at = [texSize](const FloatArr& a, int x, int y) -> float {
		x = CLAMP(x, 0, (int)texSize-1);
		y = CLAMP(y, 0, (int)texSize-1);
		return a[(unsigned)y*texSize+(unsigned)x];
	};
	// separable 3-tap box blur, applied twice (no OpenCV dependency, fully deterministic)
	FloatArr blurX; blurX.resize(texSize*texSize);
	for (unsigned y=0; y<texSize; ++y)
		for (unsigned x=0; x<texSize; ++x)
			blurX[y*texSize+x] = (at(noise,(int)x-1,(int)y)+at(noise,(int)x,(int)y)+at(noise,(int)x+1,(int)y))*(1.f/3.f);
	FloatArr blur; blur.resize(texSize*texSize);
	for (unsigned y=0; y<texSize; ++y)
		for (unsigned x=0; x<texSize; ++x)
			blur[y*texSize+x] = (at(blurX,(int)x,(int)y-1)+at(blurX,(int)x,(int)y)+at(blurX,(int)x,(int)y+1))*(1.f/3.f);
	constexpr unsigned checkerSize = 4; // < Refine::WindowSize, so every window straddles an edge
	texture.create((int)texSize, (int)texSize);
	for (unsigned y=0; y<texSize; ++y) {
		for (unsigned x=0; x<texSize; ++x) {
			const bool tile(((x/checkerSize)+(y/checkerSize))%2 == 0);
			const float base(tile ? 170.f : 85.f);
			const float n(blur[y*texSize+x]*60.f-30.f);
			const uint8_t v((uint8_t)CLAMP(base+n, 0.f, 255.f));
			texture((int)y, (int)x) = Pixel8U(v, v, v);
		}
	}
}

// a flat NxN grid mesh spanning [-halfExtent,halfExtent]^2 at z=0; when texSize>0 also fills
// faceTexcoords with the linear (x,y)->pixel mapping GenerateRefineSyntheticTexture renders with
static void BuildRefineSyntheticPlaneMesh(Mesh& mesh, unsigned gridN, float halfExtent, unsigned texSize)
{
	ASSERT(gridN >= 2);
	mesh.Release();
	mesh.vertices.resize(gridN*gridN);
	Mesh::TexCoordArr vertexTex;
	if (texSize > 0)
		vertexTex.resize(gridN*gridN);
	for (unsigned j=0; j<gridN; ++j) {
		for (unsigned i=0; i<gridN; ++i) {
			const float fx(-halfExtent + 2.f*halfExtent*(float)i/(float)(gridN-1));
			const float fy(-halfExtent + 2.f*halfExtent*(float)j/(float)(gridN-1));
			mesh.vertices[j*gridN+i] = Mesh::Vertex(fx, fy, 0.f);
			if (texSize > 0)
				vertexTex[j*gridN+i] = Mesh::TexCoord((float)texSize*(float)i/(float)(gridN-1), (float)texSize*(float)j/(float)(gridN-1));
		}
	}
	mesh.faces.Reserve((gridN-1)*(gridN-1)*2);
	if (texSize > 0)
		mesh.faceTexcoords.Reserve((gridN-1)*(gridN-1)*2*3);
	for (unsigned j=0; j+1<gridN; ++j) {
		for (unsigned i=0; i+1<gridN; ++i) {
			const Mesh::VIndex v00(j*gridN+i), v10(j*gridN+i+1), v01((j+1)*gridN+i), v11((j+1)*gridN+i+1);
			mesh.faces.AddConstruct(v00, v10, v11);
			mesh.faces.AddConstruct(v00, v11, v01);
			if (texSize > 0) {
				mesh.faceTexcoords.Insert(vertexTex[v00]);
				mesh.faceTexcoords.Insert(vertexTex[v10]);
				mesh.faceTexcoords.Insert(vertexTex[v11]);
				mesh.faceTexcoords.Insert(vertexTex[v00]);
				mesh.faceTexcoords.Insert(vertexTex[v11]);
				mesh.faceTexcoords.Insert(vertexTex[v01]);
			}
		}
	}
}

// perturb a coarse copy of the plane along z: seeded Gaussian noise plus a smooth two-lobe bump,
// so the refiner has both high-frequency error to denoise and a low-frequency bias to correct;
// footprint is the nominal pixel footprint (world units/pixel) computed by the caller
static void PerturbRefineSyntheticInputMesh(Mesh& mesh, float sceneScale, float footprint)
{
	uint32_t state(0x9E3779B9u);
	const float sigmaZ(3.f*footprint), ampSin(10.f*footprint);
	const float period((float)PI/sceneScale); // 2*pi/(2*sceneScale)
	FOREACH(idx, mesh.vertices) {
		Mesh::Vertex& v = mesh.vertices[idx];
		const float noiseZ(RefineSynthGaussian(state, sigmaZ));
		const float bump(ampSin*(float)SIN(period*v.x)*(float)SIN(period*v.y));
		v.z = noiseZ + bump;
	}
}

// build the complete synthetic fixture: a textured GT plane rendered into 4 pinhole cameras
// (images written to disk under dir, since the refiner reloads them from Image::name at every
// scale) and a noisy, coarse input mesh (scene.mesh) for RefineMesh to start from
static void BuildRefineSyntheticScene(Scene& scene, Mesh& gtMesh, const String& dir, float sceneScale)
{
	scene.Release();
	Util::ensureFolder(dir);

	// ground-truth textured plane
	BuildRefineSyntheticPlaneMesh(gtMesh, RefineSynthGTGridN, sceneScale, RefineSynthTexSize);
	Image8U3 texture;
	GenerateRefineSyntheticTexture(texture, RefineSynthTexSize);
	gtMesh.texturesDiffuse.Insert(texture);

	// one platform, one relative camera mounted at the platform origin (identity pose), and one
	// world pose per camera position
	Platform& platform = scene.platforms.AddEmpty();
	Platform::Camera& relCamera = platform.cameras.AddEmpty();
	relCamera.R = Matrix3x3::IDENTITY;
	relCamera.C = Point3(0, 0, 0);
	Matrix3x3 K(Matrix3x3::IDENTITY);
	const float focalNorm(RefineSynthFocal/(float)MAXF(RefineSynthImgWidth,RefineSynthImgHeight));
	K(0,0) = K(1,1) = focalNorm;
	relCamera.K = K;

	static const float offsets[4][2] = {
		{-RefineSynthCamOffset,-RefineSynthCamOffset}, {-RefineSynthCamOffset, RefineSynthCamOffset},
		{ RefineSynthCamOffset,-RefineSynthCamOffset}, { RefineSynthCamOffset, RefineSynthCamOffset}
	};
	// SelectNeighborViews() (reached from RefineMesh through SampleMeshWithVisibility +
	// SelectRefineNeighbors, since the fixture never fills Image::neighbors itself) clamps its
	// required-neighbor count to nCalibratedImages-1: left at the Scene default (never set by a
	// caller that builds platforms/images directly instead of going through LoadInterface),
	// every image would fail that check silently and MeshRefine::IsValid() would report an empty
	// pair list instead of the real cause
	scene.nCalibratedImages = 4;
	for (unsigned i=0; i<4; ++i) {
		Platform::Pose& pose = platform.poses.AddEmpty();
		pose.C = Point3(offsets[i][0]*sceneScale, offsets[i][1]*sceneScale, RefineSynthCamHeight*sceneScale);
		pose.R.LookAt(pose.C, Point3(0,0,0), Point3(0,1,0));

		Image& image = scene.images.AddEmpty();
		image.ID = i;
		image.platformID = 0;
		image.cameraID = 0;
		image.poseID = i;
		image.width = RefineSynthImgWidth;
		image.height = RefineSynthImgHeight;
		image.scale = 1.f;
		image.name = dir + String::FormatString("cam%u.png", i);
		image.UpdateCamera(scene.platforms);

		// render the GT mesh's texture into this camera and save the photograph to disk
		DepthMap depthMap(cv::Size((int)RefineSynthImgWidth, (int)RefineSynthImgHeight));
		Image8U3 photo;
		gtMesh.Project(image.camera, depthMap, photo);
		photo.Save(image.name);
	}

	// noisy, coarse input mesh RefineMesh starts from
	const float footprint(RefineSynthCamHeight*sceneScale/RefineSynthFocal);
	BuildRefineSyntheticPlaneMesh(scene.mesh, RefineSynthInputGridN, sceneScale, 0);
	PerturbRefineSyntheticInputMesh(scene.mesh, sceneScale, footprint);
}

// RMS distance to the GT plane z=0 of the vertices whose (x,y) lies inside the plane extent
// minus one input-grid cell (avoids boundary effects at the mesh/background silhouette); also
// rejects (returns false for) a mesh with any non-finite vertex, anywhere, not only inside the
// RMS window
static bool RefineSynthComputeRMS(const Mesh::VertexArr& vertices, float sceneScale, float margin, float& rms)
{
	double sumSq(0);
	unsigned count(0);
	for (const Mesh::Vertex& v: vertices) {
		if (!ISFINITE(v.x) || !ISFINITE(v.y) || !ISFINITE(v.z))
			return false;
		if (ABS(v.x) > sceneScale-margin || ABS(v.y) > sceneScale-margin)
			continue;
		sumSq += SQUARE((double)v.z);
		++count;
	}
	if (count == 0)
		return false;
	rms = (float)SQRT(sumSq/(double)count);
	return true;
}

// synthetic end-to-end test of the mesh-refinement photometric pipeline (see the declaration in
// TestsMVS.h for the full description)
bool MeshRefineSyntheticTest(bool forceCPU, bool verbose)
{
	OPTREFINE::init();
	OPTREFINE::update();

	const String dir(MAKE_PATH("refine_synth/"));
	// removes the fixture directory -- and anything the refiner might drop next to the images --
	// on every exit path, including an early return on a failed assertion
	struct DirCleanup {
		String dir;
		~DirCleanup() {
			std::error_code ec;
			std::filesystem::remove_all(std::filesystem::path(dir.c_str()), ec);
		}
	} dirCleanup{dir};
	// Scene::RefineMesh logs its per-iteration line at -v 2; bump the ambient level for the
	// duration of this test only if the caller asked for verbose output, and always restore it
	const int prevVerbosity(g_nVerbosityLevel);
	if (verbose && g_nVerbosityLevel < 2)
		g_nVerbosityLevel = 2;
	struct VerbosityRestore {
		int prev;
		~VerbosityRestore() { g_nVerbosityLevel = prev; }
	} verbosityRestore{prevVerbosity};

	constexpr float sceneScaleUnit(1.f);
	const float footprintUnit(RefineSynthCamHeight*sceneScaleUnit/RefineSynthFocal);
	const float marginUnit(2.f*sceneScaleUnit/(float)(RefineSynthInputGridN-1)); // one input-grid cell

	// --- CPU run, unit scale ---
	Scene sceneCPU(1);
	Mesh gtMeshUnit;
	BuildRefineSyntheticScene(sceneCPU, gtMeshUnit, dir, sceneScaleUnit);
	float rmsBefore;
	if (!RefineSynthComputeRMS(sceneCPU.mesh.vertices, sceneScaleUnit, marginUnit, rmsBefore)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest input mesh has a non-finite vertex or an empty RMS window!");
		return false;
	}
	TD_TIMER_START();
	if (!sceneCPU.RefineMesh(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 0, 0.2f, 0.9f, 45.05f, 0.f)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest CPU RefineMesh failed!");
		return false;
	}
	const double elapsedCPU(TD_TIMER_GET()); // milliseconds
	float rmsAfterCPU;
	if (!RefineSynthComputeRMS(sceneCPU.mesh.vertices, sceneScaleUnit, marginUnit, rmsAfterCPU)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest CPU-refined mesh has a non-finite vertex or an empty RMS window!");
		return false;
	}
	const Mesh::VIndex numVertsUnit(sceneCPU.mesh.vertices.size());
	if (elapsedCPU >= 60000.0) {
		VERBOSE("ERROR: MeshRefineSyntheticTest CPU refinement took %.1f s (limit 60 s)!", elapsedCPU/1000.0);
		return false;
	}
	if (!(rmsAfterCPU < 0.25f*rmsBefore)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest CPU RMS did not converge enough (before %g, after %g, limit %g)!", rmsBefore, rmsAfterCPU, 0.25f*rmsBefore);
		return false;
	}
	if (!(rmsAfterCPU < 0.5f*footprintUnit)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest CPU RMS %g exceeds 0.5x pixel-footprint %g!", rmsAfterCPU, 0.5f*footprintUnit);
		return false;
	}
	VERBOSE("MeshRefineSyntheticTest CPU: RMS %g -> %g (footprint %g), %u verts (%s)", rmsBefore, rmsAfterCPU, footprintUnit, numVertsUnit, TD_TIMER_GET_FMT().c_str());

	#ifdef _USE_CERES
	{
		// --- Ceres arm, unit scale, fresh copy of the same fixture: the L-BFGS line search on the
		// exact energy has to end on a finite surface much closer to the plane than the input;
		// the bar is looser than the stepper's because the arm minimizes the pure thin-plate
		// energy, not the stepper's rigidity/elasticity blend
		Scene sceneCeres(1);
		Mesh gtMeshCeres;
		BuildRefineSyntheticScene(sceneCeres, gtMeshCeres, dir, sceneScaleUnit);
		// the arm refuses what it cannot honour, before touching the mesh: a planar-vertex
		// removal (its parameter count is fixed) and an alternating pair schedule (its energy
		// would change every iteration)
		if (sceneCeres.RefineMesh(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 0, 0.2f, 0.9f, 0.f, 0.001f)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest Ceres accepted --planar-vertex-ratio > 0!");
			return false;
		}
		if (sceneCeres.RefineMesh(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 1, 0.2f, 0.9f, 0.f, 0.f)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest Ceres accepted --alternate-pair 1!");
			return false;
		}
		TD_TIMER_START();
		if (!sceneCeres.RefineMesh(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 0, 0.2f, 0.9f, 0.f, 0.f)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest Ceres RefineMesh failed!");
			return false;
		}
		float rmsAfterCeres;
		if (!RefineSynthComputeRMS(sceneCeres.mesh.vertices, sceneScaleUnit, marginUnit, rmsAfterCeres)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest Ceres-refined mesh has a non-finite vertex or an empty RMS window!");
			return false;
		}
		if (rmsAfterCeres > 0.5f*rmsBefore) {
			VERBOSE("ERROR: MeshRefineSyntheticTest Ceres RMS did not converge enough (before %g, after %g, limit %g)!", rmsBefore, rmsAfterCeres, 0.5f*rmsBefore);
			return false;
		}
		VERBOSE("MeshRefineSyntheticTest Ceres: RMS %g -> %g (footprint %g), %u verts (%s)", rmsBefore, rmsAfterCeres, footprintUnit, sceneCeres.mesh.vertices.size(), TD_TIMER_GET_FMT().c_str());
	}
	#endif

	// --- CUDA run, unit scale, fresh copy of the same fixture ---
	#ifdef _USE_CUDA
	if (!forceCPU && SEACAVE::CUDA::isEnabled()) {
		Scene sceneCUDA(1);
		Mesh gtMeshCUDA;
		BuildRefineSyntheticScene(sceneCUDA, gtMeshCUDA, dir, sceneScaleUnit);
		if (!sceneCUDA.RefineMeshCUDA(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 0, 0.2f, 0.9f, 45.05f, 0.f)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest CUDA RefineMeshCUDA failed!");
			return false;
		}
		float rmsAfterCUDA;
		if (!RefineSynthComputeRMS(sceneCUDA.mesh.vertices, sceneScaleUnit, marginUnit, rmsAfterCUDA)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest CUDA-refined mesh has a non-finite vertex or an empty RMS window!");
			return false;
		}
		const float ratio(rmsAfterCUDA/rmsAfterCPU);
		if (ABS(ratio-1.f) > 0.1f) {
			VERBOSE("ERROR: MeshRefineSyntheticTest CUDA RMS %g diverges from CPU RMS %g (ratio %g, limit +-10%%)!", rmsAfterCUDA, rmsAfterCPU, ratio);
			return false;
		}
		if (!(rmsAfterCUDA < 0.5f*footprintUnit)) {
			VERBOSE("ERROR: MeshRefineSyntheticTest CUDA RMS %g exceeds 0.5x pixel-footprint %g!", rmsAfterCUDA, 0.5f*footprintUnit);
			return false;
		}
		VERBOSE("MeshRefineSyntheticTest CUDA: RMS -> %g (CPU ratio %g)", rmsAfterCUDA, ratio);
	}
	#endif

	// --- scale invariance: same fixture at 100x, CPU only ---
	constexpr float sceneScale100(100.f);
	const float marginScale100(2.f*sceneScale100/(float)(RefineSynthInputGridN-1));
	Scene scene100(1);
	Mesh gtMesh100;
	BuildRefineSyntheticScene(scene100, gtMesh100, dir, sceneScale100);
	if (!scene100.RefineMesh(0, 640, 8, 0.f, 30, 1, 16, 2, 0.5f, 0, 0.2f, 0.9f, 45.05f, 0.f)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest scale-100 RefineMesh failed!");
		return false;
	}
	float rmsAfterScale100;
	if (!RefineSynthComputeRMS(scene100.mesh.vertices, sceneScale100, marginScale100, rmsAfterScale100)) {
		VERBOSE("ERROR: MeshRefineSyntheticTest scale-100 refined mesh has a non-finite vertex or an empty RMS window!");
		return false;
	}
	if (scene100.mesh.vertices.size() != numVertsUnit) {
		VERBOSE("ERROR: MeshRefineSyntheticTest scale-100 refined vertex count %u differs from the unit-scale run's %u!", scene100.mesh.vertices.size(), numVertsUnit);
		return false;
	}
	const float scaledRMS(rmsAfterScale100/sceneScale100);
	// The pixel-space computation (camera K/R, projections, ZNCC) is exactly scale-invariant in
	// real arithmetic (proven: a uniform 100x rescale of both the scene and every camera centre
	// cancels in the projection formula, and LookAt's direction is scale-invariant by
	// construction), and the vertex/face counts at every subdivision stage above are confirmed
	// bit-identical between the two runs. But 100 is not a power of two, so the rescale is not
	// exact in IEEE double/float arithmetic: every downstream computation picks up sub-ULP
	// rounding differences, and the bold-driver stepper's discrete accept/reject decisions (~74
	// iterations across 2 subdivision stages) can amplify that into a measurable path-dependent
	// difference in the converged vertex positions. Empirically this is ~1.3% on this fixture --
	// both RMS values already sit ~7x below the 0.5x-footprint pass bar, so this is convergence
	// noise around a near-zero residual, not evidence of scale-dependent bias; 2% keeps a margin
	// over that measurement while staying two orders of magnitude tighter than the pass bar.
	if (ABS(scaledRMS-rmsAfterCPU) > 0.02f*rmsAfterCPU) {
		VERBOSE("ERROR: MeshRefineSyntheticTest scale-100 RMS/100 %g diverges from the unit-scale RMS %g!", scaledRMS, rmsAfterCPU);
		return false;
	}
	VERBOSE("MeshRefineSyntheticTest scale-100: RMS -> %g (/100 = %g)", rmsAfterScale100, scaledRMS);

	return true;
}
/*----------------------------------------------------------------*/


// Finite-difference consistency gate for the energy the Ceres arm of Scene::RefineMesh
// (--gradient-step 0) minimizes (see the declaration in TestsMVS.h). Reuses
// BuildRefineSyntheticScene above, so the surface, the cameras and the photographs are the same
// fixture MeshRefineSyntheticTest runs the whole refinement on.
bool MeshRefineEnergyGradientTest(bool verbose)
{
	OPTREFINE::init();
	OPTREFINE::update();

	const String dir(MAKE_PATH("refine_energy/"));
	// removes the fixture directory on every exit path, including an early return
	struct DirCleanup {
		String dir;
		~DirCleanup() {
			std::error_code ec;
			std::filesystem::remove_all(std::filesystem::path(dir.c_str()), ec);
		}
	} dirCleanup{dir};
	const int prevVerbosity(g_nVerbosityLevel);
	if (verbose && g_nVerbosityLevel < 2)
		g_nVerbosityLevel = 2;
	struct VerbosityRestore {
		int prev;
		~VerbosityRestore() { g_nVerbosityLevel = prev; }
	} verbosityRestore{prevVerbosity};

	constexpr float sceneScale(1.f);
	const float footprint(RefineSynthCamHeight*sceneScale/RefineSynthFocal); // scene units per pixel

	// the three terms the gate must certify separately: the photometric energy alone (no
	// regularization), the thin-plate energy alone (no image pair scored) and their sum -- an
	// error in either one would otherwise be able to hide inside the other's finite difference
	struct Arm {
		const char* name;
		float regularityWeight;
		bool photometric;
		float perturbPx; // displace the input surface by a bump of this many pixels before probing
	};
	// The perturbed arms probe from a surface a few pixels off the plane, where 1-ZNCC is O(0.1):
	// the terms of the gradient proportional to 1-ZNCC (the reliability weight's own derivative,
	// the domain's) are invisible on the near-perfect fixture, and a gate that only ever probed
	// there certified a pair that disagreed on every real scene
	static const Arm arms[] = {
		{ "photometric", 0.f, true, 0.f },
		{ "smoothness", 1.f, false, 0.f },
		{ "combined", 1.f, true, 0.f },
		{ "photometric-perturbed", 0.f, true, 2.5f },
		{ "combined-perturbed", 1.f, true, 2.5f },
	};
	// Offsets in +-pairs, as a fraction of one pixel's footprint (the probe's direction field has
	// unit length at its largest vertex, so an offset IS the largest per-vertex displacement in
	// scene units). Both magnitudes are small enough for the quadratic term of the expansion and
	// large enough for the energy difference to clear the rounding of the float vertex array.
	//
	// The gate compares the CENTRAL difference against t*<grad E,u>. The one-sided difference the
	// same numbers also report sits at 5-12% here and cannot be driven lower by shrinking t: this
	// energy carries an always-positive term proportional to |t| -- a step of either sign moves
	// warp samples across the depth-similarity test, and dropping a matching sample from a window
	// always raises its 1-ZNCC -- so the forward difference measures the gradient plus a
	// step-size-independent 8% offset that no gradient can carry. The central difference cancels
	// that even term and leaves exactly what this gate is for.
	const float steps[] = { 2e-1f*footprint, -2e-1f*footprint, 1e-1f*footprint, -1e-1f*footprint };
	constexpr unsigned numSteps(sizeof(steps)/sizeof(steps[0]));
	constexpr double maxRatio(0.05);
	bool bFailed(false);

	for (const Arm& arm: arms) {
		Scene scene(1); // single-threaded: the per-vertex sums are then completion-order independent
		Mesh gtMesh;
		BuildRefineSyntheticScene(scene, gtMesh, dir, sceneScale);
		if (arm.perturbPx > 0) {
			// one low-frequency bump across the plane, band-limited like the probe's own direction
			AABB3f aabb(scene.mesh.vertices.Begin(), scene.mesh.vertices.GetSize());
			const Point3f center(aabb.GetCenter()), extent(aabb.GetSize()*0.5f);
			for (Mesh::Vertex& v: scene.mesh.vertices)
				v.z += arm.perturbPx*footprint*sinf((float)M_PI*(v.x-center.x)/extent.x)*cosf((float)M_PI*0.5f*(v.y-center.y)/extent.y);
		}

		Scene::RefineEnergyProbe probe;
		probe.regularityWeight = arm.regularityWeight;
		probe.photometric = arm.photometric;
		probe.seed = 0x9E3779B9u;
		probe.steps.Resize(numSteps);
		FOREACH(i, probe.steps)
			probe.steps[i] = steps[i];
		if (!scene.RefineMeshEnergyProbe(0, 640, 8, probe)) {
			VERBOSE("ERROR: MeshRefineEnergyGradientTest %s probe failed!", arm.name);
			return false;
		}
		if (!ISFINITE(probe.energy) || !ISFINITE(probe.dirDerivative)) {
			VERBOSE("ERROR: MeshRefineEnergyGradientTest %s energy %g or directional derivative %g is not finite!",
				arm.name, probe.energy, probe.dirDerivative);
			return false;
		}
		if (probe.dirDerivative == 0) {
			VERBOSE("ERROR: MeshRefineEnergyGradientTest %s directional derivative is exactly zero:"
				" the finite difference below would compare nothing!", arm.name);
			return false;
		}
		for (unsigned i=0; i+1<numSteps; i+=2) {
			const double t((double)steps[i]);
			const double predicted(t*probe.dirDerivative);
			const double forward(probe.steppedEnergies[i]-probe.energy);
			const double central((probe.steppedEnergies[i]-probe.steppedEnergies[i+1])/2.0);
			const double ratio(ABS(central-predicted)/ABS(predicted));
			VERBOSE("MeshRefineEnergyGradientTest %s: t %.3g\tE %.9g\tt<g,u> %.6g\tcentral %.6g (%.3f%%)\tforward %.6g (%.3f%%)",
				arm.name, t, probe.energy, predicted, central, 100.0*ratio,
				forward, 100.0*ABS(forward-predicted)/ABS(predicted));
			if (!(ratio <= maxRatio)) {
				VERBOSE("ERROR: MeshRefineEnergyGradientTest %s gradient disagrees with the energy at t %g:"
					" central difference %g vs predicted %g (%.2f%%, limit %.0f%%)!",
					arm.name, t, central, predicted, 100.0*ratio, 100.0*maxRatio);
				bFailed = true;
			}
		}
	}
	return !bFailed;
}
/*----------------------------------------------------------------*/

} // namespace MVS
