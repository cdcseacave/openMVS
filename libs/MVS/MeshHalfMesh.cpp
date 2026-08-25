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

// Which of the derived caches the caller owned before the operation; halfmesh
// does not carry them, so they are rebuilt afterwards - and only those, so a
// caller that never asked for a cache does not start paying for it here.
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

// halfmesh's contract for the optional per-element arrays is "empty or exactly
// sized"; drop whatever an operation left inconsistent with the new topology.
static void SanitizeAttributes(halfmesh::Mesh& halfMesh)
{
	if (halfMesh.vertexColors.size() != halfMesh.vertices.size())
		halfMesh.vertexColors.clear();
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
	halfmesh::ConvertMesh(halfMesh, mesh);
	if (derived.vertexFaces || derived.faceFaces || derived.vertexNormals || derived.vertexBoundary)
		mesh.ListIncidentFaces();
	if (derived.vertexVertices)
		mesh.ListIncidentVertices();
	if (derived.faceFaces)
		mesh.ListIncidentFaceFaces();
	if (derived.vertexBoundary)
		mesh.ListBoundaryVertices();
	if (derived.faceNormals || derived.vertexNormals)
		mesh.ComputeNormalFaces();
	if (derived.vertexNormals)
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
	halfmesh::Mesh halfMesh = ImportMesh(*this);
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
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const FIndex count = halfMesh.RemoveSpuriousComponents(factor);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveSpikes(unsigned maxIterations)
{
	if (vertices.empty() || maxIterations == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
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
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	halfMesh.Simplify(target, minEdgeLength, aggressiveness);
	ExportMesh(halfMesh, *this, derived);
}

unsigned Mesh::CloseHoles(unsigned maxHoleEdges)
{
	if (vertices.empty() || faces.empty() || maxHoleEdges == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const unsigned count = halfMesh.CloseHoles(maxHoleEdges);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

void Mesh::Smooth(int iterations)
{
	if (vertices.empty() || faces.empty() || iterations <= 0)
		return;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	halfMesh.SmoothTaubin(iterations);
	ExportMesh(halfMesh, *this, derived);
}

bool Mesh::TransferTexture(Mesh& mesh, unsigned borderSize, unsigned textureSize)
{
	if (!HasTexture() || faceTexcoords.size() != faces.size()*3 || faces.empty() || mesh.faces.empty())
		return false;
	const DerivedData derived(mesh);
	halfmesh::Mesh source = ImportMesh(*this);
	halfmesh::Mesh target = ImportMesh(mesh);
	halfmesh::BakeParams params;
	params.resolution = textureSize;
	params.padding = borderSize;
	params.correspondence = halfmesh::Correspondence::Nearest;
	const halfmesh::BakeResult result(halfmesh::RebakeTexture(source, target, params));
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
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	if (params.spuriousFactor > 0.f)
		halfMesh.RemoveSpuriousComponents(params.spuriousFactor);
	if (params.removeSpikes)
		halfMesh.RemoveSpikes(params.maxSpikeIterations);
	if (params.simplifyTarget != 1.f) {
		ASSERT(params.simplifyTarget > 0.f);
		halfMesh.Simplify(params.simplifyTarget);
	}
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

unsigned Mesh::RemoveVerticesAndFill(VertexIdxArr& verticesRemove)
{
	if (vertices.empty() || faces.empty() || verticesRemove.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
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
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const FIndex count = halfMesh.RemoveDegenerateFaces(thArea);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveDegenerateFaces(unsigned maxIterations, Type thArea)
{
	if (faces.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const FIndex count = halfMesh.RemoveDegenerateFaces(maxIterations, thArea);
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveDuplicatedVertices()
{
	if (vertices.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const VIndex count = halfMesh.RemoveDuplicateVertices();
	ExportMesh(halfMesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveUnreferencedVertices()
{
	if (vertices.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh halfMesh = ImportMesh(*this);
	const VIndex count = halfMesh.RemoveUnreferencedVertices();
	ExportMesh(halfMesh, *this, derived);
	return count;
}
