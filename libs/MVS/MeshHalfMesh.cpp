/*
* MeshHalfMesh.cpp
*
* Copyright (c) 2026 SEACAVE
*
* Generic mesh processing delegated to halfmesh.
*/

#include "Common.h"
#include "Mesh.h"
#include <halfmesh/InteropOpenMVS.h>
#include <halfmesh/TextureBake.h>

using namespace MVS;

namespace {

// Which of the optional arrays the caller owned before the operation; halfmesh
// does not carry the adjacency caches, so they are rebuilt afterwards - and only
// those, so a caller that never asked for a cache does not start paying for it
// here. vertexNormals is not a cache: halfmesh transports it, keeping the
// caller's authored values through the operations that only renumber vertices
// and clearing it in the ones that move them. It is tracked here only to know
// whether recomputing is the right fallback once halfmesh has dropped it.
struct DerivedData
{
	bool vertexVertices;
	bool vertexFaces;
	bool vertexBoundary;
	bool faceFaces;
	bool faceNormals;
	bool vertexNormals;

	explicit DerivedData(const Mesh& mesh)
		: vertexVertices(!mesh.vertexVertices.empty())
		, vertexFaces(!mesh.vertexFaces.empty())
		, vertexBoundary(!mesh.vertexBoundary.empty())
		, faceFaces(!mesh.faceFaces.empty())
		, faceNormals(!mesh.faceNormals.empty())
		, vertexNormals(!mesh.vertexNormals.empty())
	{}
};

static halfmesh::Mesh ImportMesh(const Mesh& mesh)
{
	halfmesh::Mesh halfMesh;
	halfmesh::ConvertMesh(mesh, halfMesh);
	// Normals are derived and become stale after any geometry operation.
	halfMesh.faceNormals.clear();
	return halfMesh;
}

// Consuming import, for the in-place operations: every source array is freed
// as soon as it has been copied, so a large mesh is never resident in both
// representations at once. Only safe because each of those callers hands over
// a mesh it then unconditionally overwrites through ExportMesh().
static halfmesh::Mesh ImportMesh(Mesh&& mesh)
{
	halfmesh::Mesh halfMesh;
	halfmesh::ConvertMesh(std::move(mesh), halfMesh);
	halfMesh.faceNormals.clear();
	return halfMesh;
}

// halfmesh's contract for the optional per-element arrays is "empty or exactly
// sized"; drop whatever an operation left inconsistent with the new topology.
static void SanitizeAttributes(halfmesh::Mesh& halfMesh)
{
	if (halfMesh.vertexColors.size() != halfMesh.vertices.size())
		halfMesh.vertexColors.clear();
	if (halfMesh.vertexNormals.size() != halfMesh.vertices.size())
		halfMesh.vertexNormals.clear();
	if (halfMesh.faceNormals.size() != halfMesh.faces.size())
		halfMesh.faceNormals.clear();
	if (!halfMesh.faceTexcoords.empty() &&
		halfMesh.faceTexcoords.size() != halfMesh.faces.size()*3 &&
		halfMesh.faceTexcoords.size() != halfMesh.vertices.size())
		halfMesh.faceTexcoords.clear();
	if (!halfMesh.faceTexblobs.empty() && halfMesh.faceTexblobs.size() != halfMesh.faces.size())
		halfMesh.faceTexblobs.clear();
	if (halfMesh.faceTexcoords.empty()) {
		halfMesh.faceTexblobs.clear();
		halfMesh.texturesDiffuse.clear();
	}
}

static void ExportMesh(halfmesh::Mesh& halfMesh, Mesh& mesh, const DerivedData& derived)
{
	SanitizeAttributes(halfMesh);
	// consuming conversion: it frees each halfmesh array as it is copied and drops
	// the half-edge structure and the incident-face cache, which together usually
	// outweigh the geometry. No caller reads halfMesh after this, so the mesh is
	// never resident in both representations at once - the same reason the import
	// side moves.
	halfmesh::ConvertMesh(std::move(halfMesh), mesh);
	// vertexFaces is scaffolding for the face-adjacency and boundary caches, so it
	// has to exist while those are built even when the caller never owned it
	const bool scaffoldVertexFaces(!derived.vertexFaces && (derived.faceFaces || derived.vertexBoundary));
	if (derived.vertexFaces || scaffoldVertexFaces)
		mesh.ListIncidentFaces();
	if (derived.vertexVertices)
		mesh.ListIncidentVertices();
	if (derived.faceFaces)
		mesh.ListIncidentFaceFaces();
	if (derived.vertexBoundary)
		mesh.ListBoundaryVertices();
	if (scaffoldVertexFaces)
		mesh.vertexFaces.Release();
	// faceNormals first: the active ComputeNormalVertices() derives them from the
	// faces directly, but the angle-weighted variant behind it reads faceNormals
	if (derived.faceNormals)
		mesh.ComputeNormalFaces();
	if (derived.vertexNormals && mesh.vertexNormals.empty())
		mesh.ComputeNormalVertices();
}

// Target edge length for isotropic remeshing: positive is an absolute length,
// negative is that multiple of the mesh's current mean edge length.
static float ResolveEdgeLength(halfmesh::Mesh& halfMesh, float edgeLength)
{
	ASSERT(edgeLength != 0.f);
	return edgeLength > 0.f ? edgeLength : -edgeLength * halfMesh.ComputeMeanEdgeLength();
}

static void RemeshIsotropic(halfmesh::Mesh& halfMesh, float edgeLength, int iterations)
{
	edgeLength = ResolveEdgeLength(halfMesh, edgeLength);
	if (edgeLength <= 0.f)
		return; // no edge to measure
	halfmesh::Mesh::RemeshParams params;
	params.SetEdgeLength(edgeLength);
	params.iterations = MAXF(iterations, 1);
	halfMesh.RemeshIsotropic(params);
}

} // anonymous namespace

unsigned Mesh::FixNonManifold(float magDisplacementDuplicateVertices, VertexIdxArr* duplicatedVertices)
{
	if (vertices.empty() || faces.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	std::vector<halfmesh::Mesh::VIndex> duplicated;
	const unsigned count = halfMesh.FixNonManifold(
		magDisplacementDuplicateVertices, duplicatedVertices ? &duplicated : NULL);
	if (duplicatedVertices) {
		duplicatedVertices->resize((VIndex)duplicated.size());
		std::copy(duplicated.begin(), duplicated.end(), duplicatedVertices->begin());
	}
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveSpuriousComponents(float factor)
{
	if (vertices.empty() || faces.empty() || factor <= 0.f)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const FIndex count = halfMesh.RemoveSpuriousComponents(factor);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveSpikes(unsigned maxIterations)
{
	if (vertices.empty() || maxIterations == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const VIndex count = halfMesh.RemoveSpikes(maxIterations);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

void Mesh::Simplify(float target, float minEdgeLength, float aggressiveness)
{
	if (vertices.empty() || faces.empty())
		return;
	ASSERT(target > 0.f);
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	halfMesh.Simplify(target, minEdgeLength, aggressiveness);
	ExportMesh(halfMesh, *this, derived);
}

unsigned Mesh::CloseHoles(unsigned maxHoleEdges)
{
	if (vertices.empty() || faces.empty() || maxHoleEdges == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const unsigned count = halfMesh.CloseHoles(maxHoleEdges);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

void Mesh::Smooth(int iterations)
{
	if (vertices.empty() || faces.empty() || iterations <= 0)
		return;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	halfMesh.SmoothTaubin(iterations);
	ExportMesh(halfMesh, *this, derived);
}

bool Mesh::TransferTexture(Mesh& mesh, unsigned borderSize, unsigned textureSize, const FaceIdxArr& faceSubsetIndices)
{
	if (!HasTexture() || faceTexcoords.size() != faces.size()*3 || faces.empty() || mesh.faces.empty())
		return false;
	// the subset indexes the target's faces and reaches here straight from a user
	// file, so validate it before anything is converted: an out-of-range index
	// means the caller paired the wrong indices with this mesh, which is worth
	// reporting rather than silently dropping
	for (FIndex idxFace : faceSubsetIndices) {
		if (idxFace >= mesh.faces.size()) {
			DEBUG("error: face subset index %u is out of range for the target mesh (%u faces)",
				idxFace, mesh.faces.size());
			return false;
		}
	}
	const DerivedData derived(mesh);
	// A target that already carries a UV-map gets baked onto that layout: an
	// artist's atlas, or one an earlier tool fixed up, is exactly what a caller
	// asking for a texture transfer wants preserved, and it is the only layout a
	// face subset can be expressed against. Only a target without UVs gets a
	// freshly generated atlas. halfmesh bakes into square pages, so a target whose
	// texture is not square has to go the generated route too.
	const bool targetHasUVs(mesh.HasTextureCoordinates() && mesh.faceTexcoords.size() == mesh.faces.size()*3);
	int targetPageSize(-1);
	if (targetHasUVs) {
		if (mesh.texturesDiffuse.empty()) {
			// no texture yet: the UV-map is normalized, scale it into the pixel
			// space of the page we are about to bake, as this used to do
			targetPageSize = (int)textureSize;
		} else {
			// halfmesh bakes into N pages of one square size, so every page of the
			// target has to already be that same square - otherwise baking would
			// silently resize the odd ones out
			const Image8U3& page0 = mesh.texturesDiffuse.front();
			bool uniformSquare(page0.rows == page0.cols);
			for (const Image8U3& page : mesh.texturesDiffuse)
				uniformSquare = uniformSquare && page.rows == page0.rows && page.cols == page0.cols;
			if (uniformSquare)
				targetPageSize = page0.rows;
			else
				// texturing sizes each atlas page from its own leftovers, so a mesh
				// OpenMVS textured into more than one page usually lands here: the
				// trailing page holds a handful of patches and is smaller than the rest
				DEBUG("warning: the target's %u texture pages are not all the same square (the first is %dx%d), generating a new UV-map instead",
					(unsigned)mesh.texturesDiffuse.size(), page0.cols, page0.rows);
		}
	}
	// a face subset is expressed against the layout the target already carries, so
	// it means nothing once that layout is thrown away: a caller that asked for a
	// partial edit must not silently get the whole mesh rebaked instead
	if (!faceSubsetIndices.empty() && targetPageSize <= 0) {
		DEBUG("error: a face subset needs a target UV-map that can be baked onto, but this target needs a generated one");
		return false;
	}
	// both imports copy: a bake that reports nothing has to leave the caller's
	// target mesh exactly as it found it
	halfmesh::Mesh source = ImportMesh(*this);
	halfmesh::Mesh target = ImportMesh(mesh);
	halfmesh::BakeParams params;
	params.resolution = targetPageSize > 0 ? (unsigned)targetPageSize : textureSize;
	params.padding = borderSize;
	params.correspondence = halfmesh::Correspondence::Nearest;
	halfmesh::BakeResult result;
	if (targetPageSize > 0) {
		if (target.texturesDiffuse.empty()) {
			// A UV-map with no texture behind it is normally normalized, and has to
			// be scaled into the pixel space of the page about to be baked. Decide
			// it from the coordinates rather than assuming, as this used to: a mesh
			// carrying pixel-space UVs without an image would otherwise be scaled a
			// second time and land entirely off the page.
			bool normalized(true);
			for (const halfmesh::Mesh::TexCoord& uv : target.faceTexcoords)
				normalized = normalized && uv.x() <= 1.f && uv.y() <= 1.f;
			if (normalized) {
				const float scale((float)targetPageSize);
				for (halfmesh::Mesh::TexCoord& uv : target.faceTexcoords) {
					uv.x() *= scale;
					uv.y() *= scale;
				}
			}
		}
		std::vector<bool> faceMask;
		if (!faceSubsetIndices.empty()) {
			faceMask.assign(target.faces.size(), false);
			for (FIndex idxFace : faceSubsetIndices) {
				ASSERT(idxFace < faceMask.size()); // range-checked against the target above
				faceMask[idxFace] = true;
			}
			params.faceMask = &faceMask;
		}
		result = halfmesh::BakeOntoAtlas(source, target, params);
	} else {
		ASSERT(faceSubsetIndices.empty()); // rejected above, there is no layout to express it against
		result = halfmesh::RebakeTexture(source, target, params);
	}
	if (result.numPages == 0)
		return false;
	ExportMesh(target, mesh, derived);
	return mesh.HasTexture();
}

void Mesh::Clean(const CleanParams& params)
{
	if (vertices.empty() || faces.empty())
		return;
	TD_TIMER_STARTD();
	// the whole pipeline runs on a single halfmesh instance: one conversion in,
	// one out, no matter how many stages are enabled
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	if (params.spuriousFactor > 0.f)
		halfMesh.RemoveSpuriousComponents(params.spuriousFactor);
	if (params.removeSpikes)
		halfMesh.RemoveSpikes(params.maxSpikeIterations);
	// halfmesh reads the target by magnitude, so a ratio and an absolute face
	// count share one field; non-positive is not a target at all and disables the
	// stage, matching the "0 - auto" the apps resolve before they get here
	if (params.simplifyTarget > 0.f && params.simplifyTarget != 1.f)
		halfMesh.Simplify(params.simplifyTarget);
	if (params.maxHoleEdges > 0)
		halfMesh.CloseHoles(params.maxHoleEdges);
	if (params.smoothIterations > 0)
		halfMesh.SmoothTaubin(params.smoothIterations);
	if (params.edgeLength != 0.f)
		RemeshIsotropic(halfMesh, params.edgeLength, params.remeshIterations);
	if (params.finalize) {
		halfMesh.RemoveDegenerateFaces(10, 1e-10f);
		halfMesh.RemoveUnreferencedVertices();
		halfMesh.FixNonManifold();
	}
	ExportMesh(halfMesh, *this, derived);
	DEBUG("Cleaned mesh: %u vertices, %u faces (%s)",
		vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
}

unsigned Mesh::RemoveVerticesAndFill(const VertexIdxArr& verticesRemove)
{
	if (vertices.empty() || faces.empty() || verticesRemove.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	std::vector<halfmesh::Mesh::VIndex> removed(verticesRemove.begin(), verticesRemove.end());
	const unsigned count = halfMesh.RemoveVerticesAndFill(std::move(removed));
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveDegenerateFaces(Type thArea)
{
	if (faces.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const FIndex count = halfMesh.RemoveDegenerateFaces(thArea);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveDegenerateFaces(unsigned maxIterations, Type thArea)
{
	if (faces.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const FIndex count = halfMesh.RemoveDegenerateFaces(maxIterations, thArea);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveDuplicatedVertices()
{
	if (vertices.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const VIndex count = halfMesh.RemoveDuplicateVertices();
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveUnreferencedVertices()
{
	if (vertices.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(std::move(*this));
	const VIndex count = halfMesh.RemoveUnreferencedVertices();
	ExportMesh(halfMesh, *this, derived);
	return count;
}

// glTF import/export. halfmesh owns the only tinygltf implementation in the
// build (its TinyGLTFImpl.cpp), so this stage was already linking against it;
// delegating the whole codec keeps the two sides of the round-trip together.
// Both meshes hold faceTexcoords in absolute pixels, so the interop copy needs
// no rescaling: halfmesh normalizes on write and un-normalizes on read, exactly
// as FaceTexcoordsNormalize()/Unnormalize() used to here.
//
// glTF is y-up by specification. halfmesh states that in the file, as a rotation
// on the root node rather than baked into the vertex buffer, and undoes it when
// reading; the two matrices are signed permutations, so a mesh written here
// reloads bit-identical. The one casualty is glTF this stage wrote before the
// delegation: it carried no node transform, and nothing in such a file marks it
// as z-up, so it now reads back rotated and has to be re-exported.
bool Mesh::LoadGLTF(const String& fileName)
{
	ASSERT(!fileName.empty());
	Release(); // as LoadPLY/LoadOBJ do, so a failed Load() never leaves the old mesh
	halfmesh::Mesh halfMesh;
	if (!halfMesh.LoadGLTF(fileName))
		return false;
	// the loader flattens the node hierarchy into world space and concatenates
	// every triangle primitive, so one mesh comes back however the file was split
	halfmesh::ConvertMesh(std::move(halfMesh), *this);
	if (faces.empty()) {
		// a file carrying only point/line primitives is not a mesh; report the
		// failure with nothing half-populated left behind, as LoadPLY does
		DEBUG_EXTRA("error: invalid glTF mesh file");
		Release();
		return false;
	}
	return true;
}

bool Mesh::SaveGLTF(const String& fileName, bool bBinary, bool bTexLossless) const
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	halfmesh::Mesh halfMesh;
	halfmesh::ConvertMesh(*this, halfMesh);
	// textures are written beside the file rather than embedded, as before
	return halfMesh.SaveGLTF(fileName, bBinary,
		bTexLossless ? halfmesh::Mesh::ImageFormat::PNG : halfmesh::Mesh::ImageFormat::JPG,
		false);
}
