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

static void SanitizeAttributes(halfmesh::Mesh& mesh)
{
	if (mesh.vertexColors.size() != mesh.vertices.size())
		mesh.vertexColors.clear();
	if (mesh.faceNormals.size() != mesh.faces.size())
		mesh.faceNormals.clear();
	if (!mesh.faceTexcoords.empty() &&
		mesh.faceTexcoords.size() != mesh.faces.size()*3 &&
		mesh.faceTexcoords.size() != mesh.vertices.size())
		mesh.faceTexcoords.clear();
	if (!mesh.faceTexblobs.empty() && mesh.faceTexblobs.size() != mesh.faces.size())
		mesh.faceTexblobs.clear();
	if (mesh.faceTexcoords.empty()) {
		mesh.faceTexblobs.clear();
		mesh.texturesDiffuse.clear();
	}
}

static halfmesh::Mesh ImportMesh(const Mesh& mesh)
{
	halfmesh::Mesh halfMesh;
	halfmesh::ConvertMesh(mesh, halfMesh);
	// Normals are derived and become stale after any geometry operation.
	halfMesh.faceNormals.clear();
	return halfMesh;
}

static void RebuildDerivedData(Mesh& mesh, const DerivedData& derived)
{
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

static void ExportMesh(halfmesh::Mesh& halfMesh, Mesh& mesh, const DerivedData& derived)
{
	SanitizeAttributes(halfMesh);
	halfmesh::ConvertMesh(halfMesh, mesh);
	RebuildDerivedData(mesh, derived);
}

} // anonymous namespace

unsigned Mesh::FixNonManifold(float magDisplacementDuplicateVertices, VertexIdxArr* duplicatedVertices)
{
	if (vertices.empty() || faces.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	std::vector<halfmesh::Mesh::VIndex> duplicated;
	const unsigned count = mesh.FixNonManifold(
		magDisplacementDuplicateVertices, duplicatedVertices ? &duplicated : NULL);
	if (duplicatedVertices) {
		duplicatedVertices->resize((VIndex)duplicated.size());
		std::copy(duplicated.begin(), duplicated.end(), duplicatedVertices->begin());
	}
	ExportMesh(mesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveSpuriousComponents(float factor)
{
	if (vertices.empty() || faces.empty() || factor <= 0.f)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const FIndex count = mesh.RemoveSpuriousComponents(factor);
	ExportMesh(mesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveSpikes(unsigned maxIterations)
{
	if (vertices.empty() || maxIterations == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const VIndex count = mesh.RemoveSpikes(maxIterations);
	ExportMesh(mesh, *this, derived);
	return count;
}

void Mesh::Simplify(float target, float minEdgeLength, float aggressiveness)
{
	if (vertices.empty() || faces.empty())
		return;
	ASSERT(target > 0.f);
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	mesh.Simplify(target, minEdgeLength, aggressiveness);
	ExportMesh(mesh, *this, derived);
}

unsigned Mesh::CloseHoles(unsigned maxHoles)
{
	if (vertices.empty() || faces.empty() || maxHoles == 0)
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const unsigned count = mesh.CloseHoles(maxHoles);
	ExportMesh(mesh, *this, derived);
	return count;
}

void Mesh::SmoothHCLaplacian(int iterations)
{
	if (vertices.empty() || faces.empty() || iterations <= 0)
		return;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	mesh.SmoothHCLaplacian(iterations);
	ExportMesh(mesh, *this, derived);
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
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	mesh.RemoveSpuriousComponents(params.spuriousFactor);
	if (params.removeSpikes)
		mesh.RemoveSpikes(params.maxSpikeIterations);
	if (params.simplifyTarget != 1.f) {
		ASSERT(params.simplifyTarget > 0.f);
		mesh.Simplify(params.simplifyTarget);
	}
	if (params.maxHoles > 0)
		mesh.CloseHoles(params.maxHoles);
	if (params.smoothIterations > 0)
		mesh.SmoothHCLaplacian(params.smoothIterations);
	if (params.edgeLength > 0) {
		halfmesh::Mesh::RemeshParams remeshParams;
		remeshParams.SetEdgeLength(params.edgeLength);
		remeshParams.iterations = MAXF(params.remeshIterations, 1);
		mesh.RemeshIsotropic(remeshParams);
	}
	if (params.finalize) {
		mesh.RemoveDegenerateFaces(10, 1e-10f);
		mesh.RemoveUnreferencedVertices();
		mesh.FixNonManifold();
	}
	ExportMesh(mesh, *this, derived);
	DEBUG("Cleaned mesh: %u vertices, %u faces (%s)",
		vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
}

unsigned Mesh::RemoveVerticesAndFill(VertexIdxArr& verticesRemove)
{
	if (vertices.empty() || faces.empty() || verticesRemove.empty())
		return 0;
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	std::vector<halfmesh::Mesh::VIndex> removed(verticesRemove.begin(), verticesRemove.end());
	const unsigned count = mesh.RemoveVerticesAndFill(std::move(removed));
	ExportMesh(mesh, *this, derived);
	return count;
}

void Mesh::EnsureEdgeSize(float edgeLength, int iterations)
{
	if (vertices.empty() || faces.empty())
		return;
	TD_TIMER_STARTD();
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	if (edgeLength <= 0) {
		mesh.ListHalfEdgesSafe();
		double sumLength = 0;
		for (halfmesh::Mesh::EIndex edge = 0; edge < mesh.halfMesh.ESize(); ++edge) {
			const auto verts = mesh.halfMesh.EVertices(edge);
			sumLength += (mesh.vertices[verts.first]-mesh.vertices[verts.second]).norm();
		}
		if (mesh.halfMesh.ESize() == 0)
			return;
		edgeLength = (float)(sumLength/mesh.halfMesh.ESize());
	}
	halfmesh::Mesh::RemeshParams params;
	params.SetEdgeLength(edgeLength);
	params.iterations = MAXF(iterations, 1);
	mesh.RemeshIsotropic(params);
	ExportMesh(mesh, *this, derived);
	DEBUG("Ensured edge size around %g (%s)", edgeLength, TD_TIMER_GET_FMT().c_str());
}

Mesh::FIndex Mesh::RemoveDegenerateFaces(Type thArea)
{
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const FIndex count = mesh.RemoveDegenerateFaces(thArea);
	ExportMesh(mesh, *this, derived);
	return count;
}

Mesh::FIndex Mesh::RemoveDegenerateFaces(unsigned maxIterations, Type thArea)
{
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const FIndex count = mesh.RemoveDegenerateFaces(maxIterations, thArea);
	ExportMesh(mesh, *this, derived);
	return count;
}

Mesh::VIndex Mesh::RemoveDuplicatedVertices()
{
	const VIndex oldSize = vertices.size();
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const VIndex count = mesh.RemoveDuplicateVertices();
	ExportMesh(mesh, *this, derived);
	ASSERT(oldSize >= vertices.size());
	return count;
}

Mesh::VIndex Mesh::RemoveUnreferencedVertices()
{
	const DerivedData derived(*this);
	halfmesh::Mesh mesh = ImportMesh(*this);
	const VIndex count = mesh.RemoveUnreferencedVertices();
	ExportMesh(mesh, *this, derived);
	return count;
}
