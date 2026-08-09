/*
 * Renderer.cpp
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

#include "Common.h"
#include "Renderer.h"
#include "Scene.h"
#include "BoundingBoxEdit.h"

using namespace VIEWER;

static std::array<Point3f, 4> ComputeCameraFrustumCorners(const MVS::Image& imageData, float depth);
static uint32_t CreateCameraFrustumGeometry(
	const MVS::Image& imageData,
	float depth,
	bool showLookAt,
	const Pixel32F& centerColor,
	const Pixel32F& frustumColor,
	std::vector<float>& vertices,
	std::vector<float>& colors,
	std::vector<uint32_t>& indices,
	size_t baseIndex);

Renderer::Renderer()
	: pointCount(0)
	, pointNormalCount(0)
	, cameraPointIndexCount(0)
	, cameraLineIndexCount(0)
	, ellipsoidIndexCount(0)
	, imageOverlayIndexCount(0)
	, selectionPrimitiveCount(0)
	, neighborSelectionPrimitiveCount(0)
	, selectionOverlayVertexCount(0)
	, boundsPrimitiveCount(0)
	, pickFBO(0)
	, pickIDTex(0)
	, pickDepthRBO(0)
{
}

Renderer::~Renderer() {
}

bool Renderer::Initialize() {
	try {
		// Create uniform buffer objects first (binding points 0 and 1)
		viewProjectionUBO = std::make_unique<UBO>(0);
		lightingUBO = std::make_unique<UBO>(1);

		// Create shaders
		CreateShaders();

		// Create buffer objects
		CreateBuffers();

		// Set default lighting
		SetLighting(Eigen::Vector3f(0.f, 0.f, 1.f), 1.f, Eigen::Vector3f(1.f, 1.f, 1.f));

		// Enable point size and line width control
		GL_CHECK(glEnable(GL_PROGRAM_POINT_SIZE));

		// Enable depth testing
		GL_CHECK(glEnable(GL_DEPTH_TEST));
		GL_CHECK(glDepthFunc(GL_LESS));

		// Disable blending for transparency
		GL_CHECK(glDisable(GL_BLEND));
		GL_CHECK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

		// Disable face culling
		GL_CHECK(glDisable(GL_CULL_FACE));
		GL_CHECK(glFrontFace(GL_CCW));
		return true;
	}
	catch (const std::exception& e) {
		DEBUG("Renderer initialization failed: %s", e.what());
		return false;
	}
}

void Renderer::Release() {
	Reset();
}

void Renderer::Reset() {
	// Reset scene-dependent resources for loading a new scene.
	// Clears all uploaded geometry data (point clouds, meshes, cameras, etc.)
	// while preserving scene-independent UI elements (gizmos, axes).

	// Reset scene-dependent primitive counts
	pointCount = 0;
	pointNormalCount = 0;
	cameraPointIndexCount = 0;
	cameraLineIndexCount = 0;
	ellipsoidIndexCount = 0;
	imageOverlayIndexCount = 0;
	selectionPrimitiveCount = 0;
	neighborSelectionPrimitiveCount = 0;
	boundsPrimitiveCount = 0;

	// Clear mesh-related data
	pointLayerRanges.clear();
	meshFaceCounts.clear();
	meshTextures.clear();
	meshTextureIndices.clear();
	meshSubMeshLayerIDs.clear();
	meshLayerFaceMaps.clear();
	faceRefs.clear();
	globalFaceSubMeshIndices.clear();
	cameraLayerRanges.clear();
	ellipsoidCenters.clear();
	ellipsoidLayerIDs.clear();
	ellipsoidDrawOrder.clear();
	layerPassFilter.clear();

	// Clear scene-dependent geometry buffers by allocating empty data
	ReleasePickerBuffers();

	if (pointCloudVBO)
		pointCloudVBO->AllocateBuffer(0);
	if (pointCloudColorVBO)
		pointCloudColorVBO->AllocateBuffer(0);
	if (pointCloudNormalsVBO)
		pointCloudNormalsVBO->AllocateBuffer(0);

	if (meshVBO)
		meshVBO->AllocateBuffer(0);
	if (meshEBO)
		meshEBO->AllocateBuffer(0);
	if (meshNormalVBO)
		meshNormalVBO->AllocateBuffer(0);
	if (meshTexCoordVBO)
		meshTexCoordVBO->AllocateBuffer(0);

	if (cameraVBO)
		cameraVBO->AllocateBuffer(0);
	if (cameraEBO)
		cameraEBO->AllocateBuffer(0);
	if (cameraColorVBO)
		cameraColorVBO->AllocateBuffer(0);

	if (imageOverlayVBO)
		imageOverlayVBO->AllocateBuffer(0);
	if (imageOverlayEBO)
		imageOverlayEBO->AllocateBuffer(0);

	if (selectionVBO)
		selectionVBO->AllocateBuffer(0);

	if (boundsVBO)
		boundsVBO->AllocateBuffer(0);
}

const Renderer::LayerIndexRange* Renderer::FindPointLayerRange(uint32_t layerID) const
{
	for (const LayerIndexRange& range : pointLayerRanges)
		if (range.layerID == layerID)
			return &range;
	return nullptr;
}

const Renderer::CameraLayerRange* Renderer::FindCameraLayerRange(uint32_t layerID) const
{
	for (const CameraLayerRange& range : cameraLayerRanges)
		if (range.layerID == layerID)
			return &range;
	return nullptr;
}

const Renderer::LayerFaceMap* Renderer::FindMeshLayerMap(uint32_t layerID) const
{
	for (const LayerFaceMap& map : meshLayerFaceMaps)
		if (map.layerID == layerID)
			return &map;
	return nullptr;
}

bool Renderer::MapGlobalPoint(size_t globalIndex, uint32_t& layerID, uint32_t& localIndex) const
{
	for (const LayerIndexRange& range : pointLayerRanges) {
		if (globalIndex < range.offset || globalIndex >= range.offset + range.count)
			continue;
		layerID = range.layerID;
		localIndex = static_cast<uint32_t>(globalIndex - range.offset);
		return true;
	}
	return false;
}

bool Renderer::MapGlobalFace(size_t globalIndex, uint32_t& layerID, uint32_t& localIndex) const
{
	if (globalIndex >= faceRefs.size())
		return false;
	layerID = faceRefs[globalIndex].layerID;
	localIndex = faceRefs[globalIndex].localIndex;
	return layerID != NO_ID && localIndex != NO_ID;
}

bool Renderer::MapLocalFace(uint32_t layerID, uint32_t localIndex, uint32_t& globalIndex) const
{
	const LayerFaceMap* faceMap = FindMeshLayerMap(layerID);
	if (faceMap == nullptr || localIndex >= faceMap->localToGlobalFace.size())
		return false;
	globalIndex = faceMap->localToGlobalFace[localIndex];
	return globalIndex != NO_ID;
}

void Renderer::UploadLayers(const Scene& sceneController, const Window& window)
{
	UploadPointClouds(sceneController, window.pointNormalLength);
	UploadMeshes(sceneController);
	UploadCameras(window);
}

void Renderer::UploadPointClouds(const Scene& sceneController, float normalLength)
{
	pointCount = 0;
	pointNormalCount = 0;
	pointLayerRanges.clear();

	// Aggregate visible point clouds.
	for (const Scene::Layer& layer : sceneController.GetLayers()) {
		if (!layer.visible || layer.scene.pointcloud.IsEmpty())
			continue;
		const MVS::PointCloud& pointcloud = layer.scene.pointcloud;
		LayerIndexRange range;
		range.layerID = layer.id;
		range.offset = pointCount;
		range.count = pointcloud.points.size();
		if (pointcloud.normals.size() == pointcloud.points.size()) {
			range.normalOffset = pointNormalCount;
			range.normalCount = pointcloud.normals.size() * 2;
		}
		pointLayerRanges.push_back(range);
		pointCount += range.count;
		pointNormalCount += range.normalCount;
	}
	if (pointCount) {
		pointCloudVBO->AllocateBuffer(pointCount * 3 * sizeof(float));
		pointCloudColorVBO->AllocateBuffer(pointCount * 3 * sizeof(float));
		if (pointNormalCount)
			pointCloudNormalsVBO->AllocateBuffer(pointNormalCount * 3 * sizeof(float));
		size_t pointOffset = 0;
		size_t normalOffset = 0;
		for (const Scene::Layer& layer : sceneController.GetLayers()) {
			if (!layer.visible || layer.scene.pointcloud.IsEmpty())
				continue;
			const MVS::PointCloud& pointcloud = layer.scene.pointcloud;
			pointCloudVBO->SetSubData(pointcloud.points[0].ptr(), pointcloud.points.size() * 3, pointOffset * 3);

			std::vector<float> colors;
			colors.reserve(pointcloud.points.size() * 3);
			if (layer.usePointSolidColor) {
				for (size_t i = 0; i < pointcloud.points.size(); ++i) {
					colors.push_back(layer.pointColor.x);
					colors.push_back(layer.pointColor.y);
					colors.push_back(layer.pointColor.z);
				}
			} else if (pointcloud.colors.size() == pointcloud.points.size()) {
				for (const Pixel8U& color : pointcloud.colors) {
					colors.push_back(color.r / 255.f);
					colors.push_back(color.g / 255.f);
					colors.push_back(color.b / 255.f);
				}
			} else {
				colors.resize(pointcloud.points.size() * 3, 1.f);
			}
			pointCloudColorVBO->SetSubData(colors, pointOffset * 3);

			if (pointcloud.normals.size() == pointcloud.points.size()) {
				std::vector<float> normalLines;
				normalLines.reserve(pointcloud.normals.size() * 6);
				for (size_t i = 0; i < pointcloud.points.size(); ++i) {
					const MVS::PointCloud::Point& point = pointcloud.points[i];
					const MVS::PointCloud::Normal& normal = pointcloud.normals[i];
					normalLines.push_back(point.x);
					normalLines.push_back(point.y);
					normalLines.push_back(point.z);
					normalLines.push_back(point.x + normal.x * normalLength);
					normalLines.push_back(point.y + normal.y * normalLength);
					normalLines.push_back(point.z + normal.z * normalLength);
				}
				pointCloudNormalsVBO->SetSubData(normalLines, normalOffset * 3);
				normalOffset += pointcloud.normals.size() * 2;
			}
			pointOffset += pointcloud.points.size();
		}
		ASSERT(pointOffset == pointCount && normalOffset == pointNormalCount);
	}
}

void Renderer::UploadMeshes(const Scene& sceneController)
{
	meshFaceCounts.clear();
	meshTextures.clear();
	meshTextureIndices.clear();
	meshSubMeshLayerIDs.clear();
	meshLayerFaceMaps.clear();
	faceRefs.clear();
	globalFaceSubMeshIndices.clear();

	// Aggregate visible meshes into a combined GPU upload.
	struct PreparedLayerMesh
	{
		uint32_t layerID{NO_ID};
		size_t originalFaceCount{0};
		const MVS::Mesh* directMesh{nullptr};
		std::vector<MVS::Mesh> submeshes;
		MVS::Mesh::FaceIdxArr faceSubsetIndices;
		std::vector<uint32_t> faceSubmeshIndices;

		size_t GetSubmeshCount() const { return directMesh == nullptr ? submeshes.size() : 1; }
		const MVS::Mesh& GetSubmesh(size_t idx) const
		{
			ASSERT(idx < GetSubmeshCount());
			return directMesh == nullptr ? submeshes[idx] : *directMesh;
		}
	};
	std::vector<PreparedLayerMesh> preparedMeshes;
	size_t totalVertices = 0;
	size_t totalIndices = 0;
	size_t totalTexCoords = 0;
	size_t totalFaces = 0;
	for (const Scene::Layer& layer : sceneController.GetLayers()) {
		if (!layer.visible || layer.scene.mesh.IsEmpty() || layer.scene.mesh.faces.empty())
			continue;
		PreparedLayerMesh prepared;
		prepared.layerID = layer.id;
		prepared.originalFaceCount = layer.scene.mesh.faces.size();
		if (layer.scene.mesh.HasTexture()) {
			if (layer.scene.mesh.texturesDiffuse.size() > 1) {
				std::vector<MVS::Mesh> textureSubmeshes(layer.scene.mesh.SplitMeshPerTextureBlob(&prepared.faceSubsetIndices));
				std::vector<uint32_t> textureToSubmesh(textureSubmeshes.size(), NO_ID);
				for (size_t textureIdx = 0; textureIdx < textureSubmeshes.size(); ++textureIdx) {
					MVS::Mesh& submesh(textureSubmeshes[textureIdx]);
					if (submesh.IsEmpty())
						continue;
					MVS::Mesh convertedMesh;
					submesh.ConvertTexturePerVertex(convertedMesh);
					if (convertedMesh.vertexNormals.size() != convertedMesh.vertices.size())
						convertedMesh.ComputeNormalVertices();
					textureToSubmesh[textureIdx] = (uint32_t)prepared.submeshes.size();
					prepared.submeshes.emplace_back(std::move(convertedMesh));
				}
				prepared.faceSubmeshIndices.resize(layer.scene.mesh.faces.size());
				FOREACH(faceIdx, layer.scene.mesh.faces) {
					const uint32_t textureIdx(layer.scene.mesh.GetFaceTextureIndex(faceIdx));
					ASSERT(textureIdx < textureToSubmesh.size() && textureToSubmesh[textureIdx] != NO_ID);
					prepared.faceSubmeshIndices[faceIdx] = textureToSubmesh[textureIdx];
				}
			} else {
				MVS::Mesh convertedMesh;
				layer.scene.mesh.ConvertTexturePerVertex(convertedMesh);
				if (convertedMesh.vertexNormals.size() != convertedMesh.vertices.size())
					convertedMesh.ComputeNormalVertices();
				prepared.submeshes.emplace_back(std::move(convertedMesh));
			}
		} else {
			if (layer.scene.mesh.vertexNormals.size() != layer.scene.mesh.vertices.size()) {
				prepared.submeshes.emplace_back(layer.scene.mesh);
				prepared.submeshes.back().ComputeNormalVertices();
			} else {
				prepared.directMesh = &layer.scene.mesh;
			}
		}
		for (size_t submeshIdx = 0; submeshIdx < prepared.GetSubmeshCount(); ++submeshIdx) {
			const MVS::Mesh& submesh(prepared.GetSubmesh(submeshIdx));
			totalVertices += submesh.vertices.size();
			totalIndices += submesh.faces.size() * 3;
			totalTexCoords += submesh.vertices.size();
			totalFaces += submesh.faces.size();
		}
		preparedMeshes.emplace_back(std::move(prepared));
	}
	if (!preparedMeshes.empty()) {
		meshVBO->AllocateBuffer(totalVertices * 3 * sizeof(float));
		meshNormalVBO->AllocateBuffer(totalVertices * 3 * sizeof(float));
		meshTexCoordVBO->AllocateBuffer(totalTexCoords * 2 * sizeof(float));
		meshEBO->AllocateBuffer(totalIndices * sizeof(uint32_t));
		faceRefs.assign(totalFaces, {});
		globalFaceSubMeshIndices.assign(totalFaces, NO_ID);

		uint32_t vertexOffset = 0;
		uint32_t globalFaceOffset = 0;
		for (const PreparedLayerMesh& prepared : preparedMeshes) {
			LayerFaceMap layerFaceMap;
			layerFaceMap.layerID = prepared.layerID;
			layerFaceMap.localToGlobalFace.assign(prepared.originalFaceCount, NO_ID);
			const uint32_t layerFaceBase = globalFaceOffset;
			std::vector<uint32_t> layerSubmeshFaceOffsets(prepared.GetSubmeshCount(), 0);
			for (size_t submeshIdx = 1; submeshIdx < prepared.GetSubmeshCount(); ++submeshIdx)
				layerSubmeshFaceOffsets[submeshIdx] = layerSubmeshFaceOffsets[submeshIdx - 1] + prepared.GetSubmesh(submeshIdx - 1).faces.size();
			const uint32_t globalSubmeshBase = meshFaceCounts.size();

			for (size_t submeshIdx = 0; submeshIdx < prepared.GetSubmeshCount(); ++submeshIdx) {
				const MVS::Mesh& submesh(prepared.GetSubmesh(submeshIdx));
				MVS::Mesh::TexCoordArr normFaceTexcoords;
				if (!submesh.faceTexcoords.empty())
					submesh.FaceTexcoordsNormalize(normFaceTexcoords, false);
				else
					normFaceTexcoords.resize(submesh.vertices.size());
				ASSERT(submesh.vertexNormals.size() == submesh.vertices.size());

				std::vector<uint32_t> adjustedIndices;
				adjustedIndices.reserve(submesh.faces.size() * 3);
				for (const MVS::Mesh::Face& face : submesh.faces) {
					adjustedIndices.push_back(vertexOffset + face.x);
					adjustedIndices.push_back(vertexOffset + face.y);
					adjustedIndices.push_back(vertexOffset + face.z);
				}

				meshVBO->SetSubData(&submesh.vertices[0].x, submesh.vertices.size() * 3, vertexOffset * 3);
				meshNormalVBO->SetSubData(&submesh.vertexNormals[0].x, submesh.vertexNormals.size() * 3, vertexOffset * 3);
				meshTexCoordVBO->SetSubData(&normFaceTexcoords[0].x, normFaceTexcoords.size() * 2, vertexOffset * 2);

				const MVS::Mesh::FIndex faceCountPrev = meshFaceCounts.empty() ? 0 : meshFaceCounts.back();
				meshEBO->SetSubData(adjustedIndices, faceCountPrev * 3);
				uint32_t textureIndex = NO_ID;
				if (submesh.HasTexture()) {
					ASSERT(submesh.texturesDiffuse.size() == 1);
					textureIndex = (uint32_t)meshTextures.size();
					Image& image = meshTextures.emplace_back((MVS::IIndex)textureIndex);
					image.SetImageLoading();
					image.AssignImage(submesh.texturesDiffuse.front());
					image.TransferImage();
				}
				meshTextureIndices.emplace_back(textureIndex);
				meshSubMeshLayerIDs.emplace_back(prepared.layerID);
				meshFaceCounts.emplace_back(faceCountPrev + submesh.faces.size());

				for (size_t localFace = 0; localFace < submesh.faces.size(); ++localFace)
					globalFaceSubMeshIndices[globalFaceOffset + localFace] = globalSubmeshBase + submeshIdx;

				globalFaceOffset += submesh.faces.size();
				vertexOffset += submesh.vertices.size();
			}

			if (!prepared.faceSubsetIndices.empty()) {
				FOREACH(faceIdx, prepared.faceSubsetIndices) {
					const uint32_t submeshIdx = prepared.faceSubmeshIndices[faceIdx];
					const uint32_t globalFaceIndex = layerFaceBase + layerSubmeshFaceOffsets[submeshIdx] + prepared.faceSubsetIndices[faceIdx];
					layerFaceMap.localToGlobalFace[faceIdx] = globalFaceIndex;
					faceRefs[globalFaceIndex] = {prepared.layerID, (uint32_t)faceIdx};
				}
			} else {
				for (uint32_t faceIdx = 0; faceIdx < prepared.originalFaceCount; ++faceIdx) {
					const uint32_t globalFaceIndex = layerFaceBase + faceIdx;
					layerFaceMap.localToGlobalFace[faceIdx] = globalFaceIndex;
					faceRefs[globalFaceIndex] = {prepared.layerID, faceIdx};
				}
			}
			meshLayerFaceMaps.emplace_back(std::move(layerFaceMap));
		}
		ASSERT(vertexOffset == totalVertices && globalFaceOffset == totalFaces);
		ASSERT(meshFaceCounts.size() == meshTextureIndices.size());
		ASSERT(meshFaceCounts.size() == meshSubMeshLayerIDs.size());
	}
}

void Renderer::CreateShaders() {
	// Point cloud shader
	pointCloudShader = std::make_unique<Shader>(
		#include "shaders/pointcloud.vert"
		,
		#include "shaders/pointcloud.frag"
	);

	// Point cloud normals shader
	pointCloudNormalsShader = std::make_unique<Shader>(
		#include "shaders/pointcloudnormals.vert"
		,
		#include "shaders/pointcloudnormals.frag"
	);

	// Mesh shader
	meshShader = std::make_unique<Shader>(
		#include "shaders/mesh.vert"
		,
		#include "shaders/mesh.frag"
	);

	// Mesh textured shader
	meshTexturedShader = std::make_unique<Shader>(
		#include "shaders/meshtextured.vert"
		,
		#include "shaders/meshtextured.frag"
	);

	// Geometry selection highlighting shader (for SelectionController)
	geometrySelectionShader = std::make_unique<Shader>(
		#include "shaders/geometryselection.vert"
		,
		#include "shaders/geometryselection.frag"
	);

	// Camera frustum shader
	cameraShader = std::make_unique<Shader>(
		#include "shaders/camera.vert"
		,
		#include "shaders/camera.frag"
	);

	// Pose-uncertainty ellipsoid shader (lit, per-vertex color, translucent solid surface)
	ellipsoidShader = std::make_unique<Shader>(
		#include "shaders/ellipsoid.vert"
		,
		#include "shaders/ellipsoid.frag"
	);

	// 3D Image overlay shader (renders textured quad in 3D world space)
	imageOverlayShader = std::make_unique<Shader>(
		#include "shaders/imageoverlay.vert"
		,
		#include "shaders/imageoverlay.frag"
	);

	// Selection shader (simple colored lines/points)
	selectionShader = std::make_unique<Shader>(
		#include "shaders/selection.vert"
		,
		#include "shaders/selection.frag"
		,
		#include "shaders/selection.geom"
	);

	// 2D overlay shader for SelectionController
	selectionOverlayShader = std::make_unique<Shader>(
		#include "shaders/selectionoverlay.vert"
		,
		#include "shaders/selectionoverlay.frag"
	);

	// Picker shaders (ID-only rendering) - separate for mesh and points
	pickerMeshShader = std::make_unique<Shader>(
		#include "shaders/picker_mesh.vert"
		,
		#include "shaders/picker_mesh.frag"
	);
	pickerPointsShader = std::make_unique<Shader>(
		#include "shaders/picker_points.vert"
		,
		#include "shaders/picker_points.frag"
	);

	// Bounds shader
	boundsShader = std::make_unique<Shader>(
		#include "shaders/bounds.vert"
		,
		#include "shaders/bounds.frag"
	);

	// Coordinate axes shader
	axesShader = std::make_unique<Shader>(
		#include "shaders/axes.vert"
		,
		#include "shaders/axes.frag"
	);

	// Arcball gizmo shader
	gizmoShader = std::make_unique<Shader>(
		#include "shaders/gizmo.vert"
		,
		#include "shaders/gizmo.frag"
	);

	// Bind uniform buffer objects to shaders
	viewProjectionUBO->BindToShader(*pointCloudShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*pointCloudNormalsShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*meshShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*meshTexturedShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*geometrySelectionShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*cameraShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*ellipsoidShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*imageOverlayShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*selectionShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*boundsShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*gizmoShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*pickerMeshShader, "ViewProjection");
	viewProjectionUBO->BindToShader(*pickerPointsShader, "ViewProjection");

	lightingUBO->BindToShader(*meshShader, "Lighting");
}

void Renderer::CreateBuffers() {
	SetupPointCloudBuffers();
	SetupPointCloudNormalsBuffers();
	SetupMeshBuffers();
	SetupCameraBuffers();
	SetupEllipsoidBuffers();
	SetupImageOverlayBuffers();
	SetupSelectionBuffers();
	SetupSelectionOverlayBuffers();
	SetupBoundsBuffers();
	SetupBBoxHandleBuffers();
	SetupAxesBuffers();
	SetupGizmoBuffers();
}

void Renderer::SetupPointCloudBuffers() {
	pointCloudVAO = std::make_unique<VAO>();
	pointCloudVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	pointCloudColorVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	pointCloudVAO->Bind();

	// Position attribute (location 0)
	pointCloudVBO->Bind();
	pointCloudVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Color attribute (location 1)
	pointCloudColorVBO->Bind();
	pointCloudVAO->EnableAttribute(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	pointCloudVAO->Unbind();
}

void Renderer::SetupPointCloudNormalsBuffers() {
	pointCloudNormalsVAO = std::make_unique<VAO>();
	pointCloudNormalsVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	pointCloudNormalsVAO->Bind();

	// Position attribute (location 0) - contains both start and end points of normal lines
	pointCloudNormalsVBO->Bind();
	pointCloudNormalsVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	pointCloudNormalsVAO->Unbind();
}

void Renderer::SetupMeshBuffers() {
	meshVAO = std::make_unique<VAO>();
	meshVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	meshEBO = std::make_unique<VBO>(GL_ELEMENT_ARRAY_BUFFER);
	meshNormalVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	meshTexCoordVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	meshVAO->Bind();

	// Position attribute (location 0)
	meshVBO->Bind();
	meshVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Normal attribute (location 1)
	meshNormalVBO->Bind();
	meshVAO->EnableAttribute(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Texture coordinate attribute (location 2)
	meshTexCoordVBO->Bind();
	meshVAO->EnableAttribute(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

	meshVAO->Unbind();
}

void Renderer::SetupCameraBuffers() {
	cameraVAO = std::make_unique<VAO>();
	cameraVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	cameraEBO = std::make_unique<VBO>(GL_ELEMENT_ARRAY_BUFFER);
	cameraColorVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	cameraVAO->Bind();

	// Position attribute (location 0)
	cameraVBO->Bind();
	cameraVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Color attribute (location 1)
	cameraColorVBO->Bind();
	cameraVAO->EnableAttribute(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	cameraVAO->Unbind();
}

void Renderer::SetupEllipsoidBuffers() {
	ellipsoidVAO = std::make_unique<VAO>();
	ellipsoidVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	ellipsoidNormalVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	ellipsoidEBO = std::make_unique<VBO>(GL_ELEMENT_ARRAY_BUFFER);
	ellipsoidColorVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	ellipsoidVAO->Bind();

	// Position attribute (location 0)
	ellipsoidVBO->Bind();
	ellipsoidVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Normal attribute (location 1)
	ellipsoidNormalVBO->Bind();
	ellipsoidVAO->EnableAttribute(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Color attribute (location 2)
	ellipsoidColorVBO->Bind();
	ellipsoidVAO->EnableAttribute(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	ellipsoidVAO->Unbind();
}

void Renderer::SetupSelectionBuffers() {
	selectionVAO = std::make_unique<VAO>();
	selectionVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	selectionVAO->Bind();

	// Position attribute (location 0)
	selectionVBO->Bind();
	selectionVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	selectionVAO->Unbind();
}

void Renderer::SetupSelectionOverlayBuffers() {
	// Setup 2D overlay buffers for SelectionController
	selectionOverlayVAO = std::make_unique<VAO>();
	selectionOverlayVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	selectionOverlayVAO->Bind();
	selectionOverlayVBO->Bind();
	selectionOverlayVAO->EnableAttribute(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	selectionOverlayVAO->Unbind();

	selectionOverlayVertexCount = 0;
}

void Renderer::SetupBoundsBuffers() {
	boundsVAO = std::make_unique<VAO>();
	boundsVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	boundsVAO->Bind();

	// Position attribute (location 0)
	boundsVBO->Bind();
	boundsVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	boundsVAO->Unbind();
}

// Small scratch buffer for the 8 corner + 6 face-center position markers used
// during bounding-box edit mode. Vertex data is refreshed per-frame via
// SetData() from the currently-edited OBB.
void Renderer::SetupBBoxHandleBuffers() {
	bboxHandleVAO = std::make_unique<VAO>();
	bboxHandleVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	bboxHandleVAO->Bind();

	bboxHandleVBO->Bind();
	bboxHandleVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	bboxHandleVAO->Unbind();
}

void Renderer::SetupAxesBuffers() {
	axesVAO = std::make_unique<VAO>();
	axesVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	axesColorVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);

	axesVAO->Bind();

	// Position attribute (location 0)
	axesVBO->Bind();
	axesVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	// Create coordinate axes data
	std::vector<float> axesVertices {
		// X axis (red)
		0.f, 0.f, 0.f,
		1.f, 0.f, 0.f,
		// Y axis (green)
		0.f, 0.f, 0.f,
		0.f, 1.f, 0.f,
		// Z axis (blue)
		0.f, 0.f, 0.f,
		0.f, 0.f, 1.f
	};
	axesVBO->SetData(axesVertices);

	// Color attribute (location 1)
	axesColorVBO->Bind();
	axesVAO->EnableAttribute(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	std::vector<float> axesColors {
		// X axis (red)
		1.f, 0.f, 0.f,
		1.f, 0.f, 0.f,
		// Y axis (green)
		0.f, 1.f, 0.f,
		0.f, 1.f, 0.f,
		// Z axis (blue)
		0.f, 0.f, 1.f,
		0.f, 0.f, 1.f
	};
	axesColorVBO->SetData(axesColors);

	axesVAO->Unbind();
}

void Renderer::SetupImageOverlayBuffers() {
	// Setup 3D image overlay buffers
	imageOverlayVAO = std::make_unique<VAO>();
	imageOverlayVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	imageOverlayEBO = std::make_unique<VBO>(GL_ELEMENT_ARRAY_BUFFER);

	imageOverlayVAO->Bind();

	// Bind the VBO before setting up attributes
	imageOverlayVBO->Bind();

	// Position attribute (location 0) - 3D world space coordinates
	imageOverlayVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	// Texture coordinate attribute (location 1)
	imageOverlayVAO->EnableAttribute(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

	imageOverlayVAO->Unbind();
}

// Reusable circle line-segment builder (see Renderer.h for contract).
void Renderer::BuildCircleLineSegments(int numSegments, float radius,
                                       std::vector<float>& vertices,
                                       std::vector<uint32_t>& indices,
                                       uint32_t baseIndex)
{
	// Vertices for a closed unit circle in the local XY plane.
	for (int i = 0; i <= numSegments; ++i) {
		const float angle = FTWO_PI * i / numSegments;
		vertices.push_back(COS(angle) * radius);
		vertices.push_back(SIN(angle) * radius);
		vertices.push_back(0.f);
	}
	// Line-pair indices forming the loop.
	for (int i = 0; i < numSegments; ++i) {
		indices.push_back(baseIndex + i);
		indices.push_back(baseIndex + i + 1);
	}
}

void Renderer::SetupGizmoBuffers() {
	// Setup combined gizmo buffers for both circles and center axes
	gizmoVAO = std::make_unique<VAO>();
	gizmoVBO = std::make_unique<VBO>(GL_ARRAY_BUFFER);
	gizmoEBO = std::make_unique<VBO>(GL_ELEMENT_ARRAY_BUFFER);

	gizmoVAO->Bind();

	// Position attribute (location 0)
	gizmoVBO->Bind();
	gizmoVAO->EnableAttribute(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// Generate unit-circle geometry for trackball gizmos via the shared helper.
	const int numSegments = 64;
	const float radius = 1.f;

	std::vector<float> vertices;
	std::vector<uint32_t> indices;
	BuildCircleLineSegments(numSegments, radius, vertices, indices, 0);

	// Store index count for circles
	gizmoCircleIndexCount = indices.size();

	// Add center axes geometry (append to the same buffers)
	size_t centerAxesBaseVertex = vertices.size() / 3;
	std::vector<float> axesVertices {
		// X axis
		0.f, 0.f, 0.f,
		1.f, 0.f, 0.f,
		// Y axis
		0.f, 0.f, 0.f,
		0.f, 1.f, 0.f,
		// Z axis
		0.f, 0.f, 0.f,
		0.f, 0.f, 1.f
	};
	vertices.insert(vertices.end(), axesVertices.begin(), axesVertices.end());

	// Store starting vertex for center axes (for rendering)
	gizmoCenterAxesBaseVertex = centerAxesBaseVertex;
	gizmoCenterAxesVertexCount = 6; // 3 axes, 2 vertices each

	// Upload combined geometry
	gizmoVBO->SetData(vertices);
	gizmoEBO->Bind();
	gizmoEBO->SetData(indices.data(), indices.size() * sizeof(uint32_t), GL_STATIC_DRAW);

	gizmoVAO->Unbind();
}

// Helper function to compute camera frustum corners in world space
// This function correctly accounts for the principal point by using image coordinates
// and TransformPointI2W instead of assuming the principal point is at the image center
static std::array<Point3f, 4> ComputeCameraFrustumCorners(const MVS::Image& imageData, float depth) {
	// Define the 4 corners of the image in image coordinates
	// This correctly handles cases where the principal point is not at the image center
	Point3 imageCorners[4] = {
		Point3(0, 0, depth),                                    // top-left
		Point3(imageData.width, 0, depth),                      // top-right
		Point3(imageData.width, imageData.height, depth),       // bottom-right
		Point3(0, imageData.height, depth)                      // bottom-left
	};

	// Transform corners from image space to world space
	// This automatically accounts for the principal point position
	std::array<Point3f, 4> worldCorners;
	for (int i = 0; i < 4; ++i)
		worldCorners[i] = imageData.camera.TransformPointI2W(imageCorners[i]);
	return worldCorners;
}

// Helper function to create camera frustum geometry for a single camera;
// generate the vertices, colors, and indices for the camera wireframe and
// returns the number of indices added
static uint32_t CreateCameraFrustumGeometry(
	const MVS::Image& imageData,
	float depth,
	bool showLookAt,
	const Pixel32F& centerColor,
	const Pixel32F& frustumColor,
	std::vector<float>& vertices,
	std::vector<float>& colors,
	std::vector<uint32_t>& indices,
	size_t baseIndex) {
	// Camera center (apex of the pyramid)
	const Point3f center = imageData.camera.C;
	vertices.insert(vertices.end(), {center.x, center.y, center.z});
	colors.insert(colors.end(), {centerColor.c2, centerColor.c1, centerColor.c0});

	// Get frustum corners using the helper function,
	// add the 4 corners to vertices and colors
	for (const Point3f& worldCorner : ComputeCameraFrustumCorners(imageData, depth)) {
		vertices.insert(vertices.end(), {worldCorner.x, worldCorner.y, worldCorner.z});
		colors.insert(colors.end(), {frustumColor.c2, frustumColor.c1, frustumColor.c0});
	}

	// Create indices for wireframe lines,
	// lines from camera center to each corner (4 lines)
	for (int j = 0; j < 4; ++j) {
		indices.push_back(baseIndex);           // camera center
		indices.push_back(baseIndex + 1 + j);   // corner j
	}

	// Rectangle connecting the four corners (4 lines)
	for (int j = 0; j < 4; ++j) {
		indices.push_back(baseIndex + 1 + j);             // current corner
		indices.push_back(baseIndex + 1 + ((j + 1) % 4)); // next corner
	}
	if (!showLookAt)
		return 16; // 4 lines from center + 4 lines for rectangle = 8 lines = 16 indices

	// Add principal center point (green) - point on the image plane at the principal point
	const Point2 pp = imageData.camera.GetPrincipalPoint();
	const Point3f worldPrincipalPoint = imageData.camera.TransformPointI2W(Point3(pp.x, pp.y, depth));
	vertices.insert(vertices.end(), {worldPrincipalPoint.x, worldPrincipalPoint.y, worldPrincipalPoint.z});
	colors.insert(colors.end(), {0.f, 1.f, 0.f}); // Green

	// Add upwards direction indicator (blue) - line showing camera's up direction
	const Point3f worldUpPoint = imageData.camera.TransformPointI2W(Point3(pp.x, pp.y - imageData.height * 0.25f, depth)); // Quarter way up from center
	vertices.insert(vertices.end(), {worldUpPoint.x, worldUpPoint.y, worldUpPoint.z});
	colors.insert(colors.end(), {0.f, 0.f, 1.f}); // Blue

	// Line from camera center to principal point (look-at indicator).
	indices.push_back(baseIndex);     // camera center
	indices.push_back(baseIndex + 5); // principal point (index 5)

	// Line from principal point to upwards direction indicator.
	indices.push_back(baseIndex + 5); // principal point (index 5)
	indices.push_back(baseIndex + 6); // upwards direction indicator (index 6)

	return 20; // 4 lines from center + 4 lines for rectangle + 2 lines for look-at = 10 lines = 20 indices
}

void Renderer::UploadCameras(const Window& window) {
	cameraPointIndexCount = cameraLineIndexCount = imageOverlayIndexCount = 0;
	cameraLayerRanges.clear();

	const Scene& sceneController = window.GetScene();
	size_t imageCount = 0;
	for (const Scene::Layer& layer : sceneController.GetLayers()) {
		if (!layer.visible)
			continue;
		imageCount += layer.images.size();
	}
	if (imageCount == 0)
		return;

	const float depth = window.GetCamera().GetSceneDistance() * window.cameraSize;
	const bool displayDots = window.cameraDisplayType == Window::CAMERA_DISPLAY_DOT;
	const bool showLookAt = window.showCameraLookAt;

	std::vector<float> cameraVertices;
	std::vector<float> cameraColors;
	std::vector<uint32_t> cameraPointIndices;
	std::vector<uint32_t> cameraLineIndices;
	std::vector<float> allVertices;
	std::vector<uint32_t> allIndices;

	size_t globalCameraOffset = 0;
	for (const Scene::Layer& layer : sceneController.GetLayers()) {
		if (!layer.visible || layer.images.empty())
			continue;
		CameraLayerRange range;
		range.layerID = layer.id;
		range.offset = globalCameraOffset;
		range.count = layer.images.size();
		range.pointIndexOffset = cameraPointIndices.size();
		range.lineIndexOffset = cameraLineIndices.size();
		size_t layerCameraIdx = 0;
		for (const Image& image : layer.images) {
			const MVS::Image& imageData = layer.scene.images[image.idx];
			ASSERT(imageData.IsValid());
			const float colorValue = layer.useCameraJetColor
			                             ? (layer.images.size() > 1 ? ((float)layerCameraIdx / (float)(layer.images.size() - 1)) : 0.5f)
			                             : 0.5f;
			const Pixel32F cameraColor = layer.useCameraJetColor ? Pixel32F::gray2color(colorValue) : Pixel32F(layer.cameraColor.x, layer.cameraColor.y, layer.cameraColor.z);

			if (displayDots) {
				const uint32_t baseIndex = (uint32_t)(cameraVertices.size() / 3);
				const Point3f center = imageData.camera.C;
				cameraVertices.insert(cameraVertices.end(), {center.x, center.y, center.z});
				cameraColors.insert(cameraColors.end(), {cameraColor.c2, cameraColor.c1, cameraColor.c0});
				cameraPointIndices.push_back(baseIndex);
				++cameraPointIndexCount;
				if (showLookAt) {
					const Point2 pp = imageData.camera.GetPrincipalPoint();
					const Point3f worldPrincipalPoint = imageData.camera.TransformPointI2W(Point3(pp.x, pp.y, depth));
					cameraVertices.insert(cameraVertices.end(), {worldPrincipalPoint.x, worldPrincipalPoint.y, worldPrincipalPoint.z});
					cameraColors.insert(cameraColors.end(), {0.f, 1.f, 0.f});
					cameraLineIndices.push_back(baseIndex);
					cameraLineIndices.push_back(baseIndex + 1);
					cameraLineIndexCount += 2;
				}
			} else {
				const size_t baseIndex = cameraVertices.size() / 3;
				cameraLineIndexCount += CreateCameraFrustumGeometry(
				    imageData,
				    depth,
				    showLookAt,
				    cameraColor,
				    cameraColor,
				    cameraVertices,
				    cameraColors,
				    cameraLineIndices,
				    baseIndex);
			}

			std::array<Point3f, 4> worldCorners = ComputeCameraFrustumCorners(imageData, depth);
			const uint32_t baseVertex = (uint32_t)(allVertices.size() / 5);
			for (int i = 0; i < 4; ++i) {
				const Point3f& worldCorner = worldCorners[i];
				allVertices.push_back(worldCorner.x);
				allVertices.push_back(worldCorner.y);
				allVertices.push_back(worldCorner.z);
				switch (i) {
				case 0:
					allVertices.push_back(0.f);
					allVertices.push_back(0.f);
					break;
				case 1:
					allVertices.push_back(1.f);
					allVertices.push_back(0.f);
					break;
				case 2:
					allVertices.push_back(1.f);
					allVertices.push_back(1.f);
					break;
				default:
					allVertices.push_back(0.f);
					allVertices.push_back(1.f);
					break;
				}
			}
			allIndices.push_back(baseVertex + 0);
			allIndices.push_back(baseVertex + 1);
			allIndices.push_back(baseVertex + 2);
			allIndices.push_back(baseVertex + 0);
			allIndices.push_back(baseVertex + 2);
			allIndices.push_back(baseVertex + 3);
			++layerCameraIdx;
		}
		range.pointIndexCount = cameraPointIndices.size() - range.pointIndexOffset;
		range.lineIndexCount = cameraLineIndices.size() - range.lineIndexOffset;
		cameraLayerRanges.push_back(range);
		globalCameraOffset += layer.images.size();
	}

	std::vector<uint32_t> cameraIndices;
	cameraIndices.reserve(cameraPointIndices.size() + cameraLineIndices.size());
	cameraIndices.insert(cameraIndices.end(), cameraPointIndices.begin(), cameraPointIndices.end());
	cameraIndices.insert(cameraIndices.end(), cameraLineIndices.begin(), cameraLineIndices.end());

	if (!cameraIndices.empty()) {
		cameraVBO->SetData(cameraVertices);
		cameraColorVBO->SetData(cameraColors);
		cameraEBO->SetData(cameraIndices);
	}

	if (!allIndices.empty()) {
		imageOverlayIndexCount = allIndices.size();
		imageOverlayVBO->SetData(allVertices);
		imageOverlayEBO->SetData(allIndices);
	}
}

void Renderer::UploadUncertaintyEllipsoids(const Window& window) {
	ellipsoidIndexCount = 0;
	ellipsoidCenters.clear();
	ellipsoidLayerIDs.clear();
	const Scene& sceneController = window.GetScene();
	if (!sceneController.HasCameraUncertainty())
		return;

	// Solid unit-sphere template (UV sphere): a shared vertex grid + triangle list that is
	// transformed per camera into an oriented, shaded error ellipsoid. Per-vertex unit-sphere
	// positions double as the object-space directions used to derive the surface normals.
	constexpr int STACKS = 10, SLICES = 16;
	std::vector<Point3f> unitVertices;
	std::vector<uint32_t> unitIndices;
	unitVertices.reserve((STACKS + 1) * (SLICES + 1));
	for (int s = 0; s <= STACKS; ++s) {
		const float phi = FPI * s / STACKS; // 0 = north pole .. PI = south pole
		const float sinPhi = SIN(phi), cosPhi = COS(phi);
		for (int l = 0; l <= SLICES; ++l) {
			const float theta = 2.f * FPI * l / SLICES;
			unitVertices.emplace_back(sinPhi * COS(theta), sinPhi * SIN(theta), cosPhi);
		}
	}
	const int stride = SLICES + 1;
	for (int s = 0; s < STACKS; ++s) {
		for (int l = 0; l < SLICES; ++l) {
			const uint32_t i0 = (uint32_t)(s * stride + l), i1 = i0 + 1;
			const uint32_t i2 = i0 + stride, i3 = i2 + 1;
			unitIndices.insert(unitIndices.end(), { i0, i2, i1, i1, i2, i3 }); // outward winding
		}
	}

	std::vector<float> vertices, normals, colors;
	std::vector<uint32_t> indices;
	for (const Scene::Layer& layer : sceneController.GetLayers()) {
		if (!layer.visible || layer.cameraUncertainty.empty())
			continue;
		const MVS::ImageArr& images(layer.scene.images);
		const ImageArr& viewerImages(layer.images);
		ASSERT(layer.cameraUncertainty.size() == viewerImages.size());
		const float norm = layer.cameraUncertaintyNorm > 0.f ? layer.cameraUncertaintyNorm : 1.f;
		// Effective radius scale = per-layer auto-fit x the global user magnification slider.
		const float scale = window.uncertaintyEllipsoidScale * layer.cameraUncertaintyAutoScale;
		FOREACH(cameraIdx, viewerImages) {
			const Scene::CameraUncertainty& u = layer.cameraUncertainty[cameraIdx];
			if (!u.IsComputed())
				continue;
			// Oriented world-frame error ellipsoid: eigen-decompose the position covariance
			const Matrix3x3f& cov = u.posCov;
			Eigen::Matrix3f ecov;
			ecov << cov(0, 0), cov(0, 1), cov(0, 2),
			    cov(1, 0), cov(1, 1), cov(1, 2),
			    cov(2, 0), cov(2, 1), cov(2, 2);
			const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(ecov);
			if (es.info() != Eigen::Success)
				continue;
			const Eigen::Vector3f radii = es.eigenvalues().cwiseMax(0.f).cwiseSqrt() * scale;
			if (radii.maxCoeff() <= 0.f)
				continue; // gauge datum (or degenerate): nothing to draw
			const Eigen::Matrix3f R = es.eigenvectors();
			// Ellipsoid surface normal is R * diag(1/radii) * u (gradient of the implicit form);
			// clamp the reciprocal so a near-degenerate (thin) axis does not blow up the normal.
			const Eigen::Vector3f invRadii = radii.cwiseMax(1e-9f).cwiseInverse();
			const MVS::Image& imageData = images[viewerImages[cameraIdx].idx];
			const Point3f center = imageData.camera.C;
			const Eigen::Vector3f C(center.x, center.y, center.z);
			ellipsoidCenters.push_back(C); // one per accepted ellipsoid, aligned with the EBO slot order
			ellipsoidLayerIDs.push_back(layer.id);
			// gray2color maps 0 = red, 1 = blue: invert so blue = best localized, red = worst
			const float colorValue = MINF(u.MaxPosSigma() / norm, 1.f);
			const Pixel32F color = Pixel32F::gray2color(1.f - colorValue);
			const uint32_t baseIndex = (uint32_t)(vertices.size() / 3);
			for (const Point3f& v : unitVertices) {
				const Eigen::Vector3f dir(v.x, v.y, v.z);
				const Eigen::Vector3f p = C + R * radii.cwiseProduct(dir);
				const Eigen::Vector3f n = (R * invRadii.cwiseProduct(dir)).normalized();
				vertices.insert(vertices.end(), {p.x(), p.y(), p.z()});
				normals.insert(normals.end(), {n.x(), n.y(), n.z()});
				colors.insert(colors.end(), {color.c2, color.c1, color.c0});
			}
			for (const uint32_t idx : unitIndices)
				indices.push_back(baseIndex + idx);
		}
	}
	if (indices.empty())
		return;
	ellipsoidVBO->SetData(vertices);
	ellipsoidNormalVBO->SetData(normals);
	ellipsoidColorVBO->SetData(colors);
	ellipsoidEBO->SetData(indices);
	ellipsoidIndexCount = indices.size();
}

void Renderer::RenderUncertaintyEllipsoids(const Window& window) {
	if (ellipsoidIndexCount == 0)
		return;

	// Translucent shaded solids: depth-tested against the opaque scene but not writing depth,
	// so the camera frustum sitting at each ellipsoid center and any overlapping ellipsoids
	// remain visible through the surface.
	GL_CHECK(glEnable(GL_BLEND));
	GL_CHECK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
	GL_CHECK(glDepthMask(GL_FALSE));

	ellipsoidShader->Use();
	// Fairly opaque so thin/needle-shaped covariances stay clearly visible (a translucent
	// needle seen edge-on nearly disappears); depth-write is off so the camera at the center
	// and neighbouring ellipsoids show through.
	ellipsoidShader->SetFloat("alpha", 0.6f);

	ellipsoidVAO->Bind();
	ellipsoidEBO->Bind();
	GL_CHECK(glEnable(GL_CULL_FACE));
	// Every ellipsoid shares the same fixed sphere topology, so slot k owns the constant-size
	// EBO block [k*per, (k+1)*per). For correct translucency, sort the slots farthest-first from
	// the eye and draw them back-to-front; within each convex ellipsoid draw the far (back) faces
	// then the near (front) faces so its own shell is not inside-out. Without the inter-ellipsoid
	// ordering, overlapping ellipsoids blend in buffer order rather than by depth.
	const Eigen::Vector3f eye = window.GetCamera().GetPosition().cast<float>();
	const size_t n = ellipsoidCenters.size();
	const GLsizei per = (GLsizei)(ellipsoidIndexCount / n); // constant per-ellipsoid stride, exact
	ellipsoidDrawOrder.resize(n);
	std::iota(ellipsoidDrawOrder.begin(), ellipsoidDrawOrder.end(), 0u);
	std::sort(ellipsoidDrawOrder.begin(), ellipsoidDrawOrder.end(),
		[&](uint32_t a, uint32_t b) {
			return (ellipsoidCenters[a] - eye).squaredNorm() > (ellipsoidCenters[b] - eye).squaredNorm();
		});
	for (const uint32_t k : ellipsoidDrawOrder) {
		if (!IsLayerInPass(ellipsoidLayerIDs[k]))
			continue;
		const void* off = reinterpret_cast<const void*>((size_t)k * per * sizeof(uint32_t));
		GL_CHECK(glCullFace(GL_FRONT));
		GL_CHECK(glDrawElements(GL_TRIANGLES, per, GL_UNSIGNED_INT, off)); // far (back) faces
		GL_CHECK(glCullFace(GL_BACK));
		GL_CHECK(glDrawElements(GL_TRIANGLES, per, GL_UNSIGNED_INT, off)); // near (front) faces
	}
	GL_CHECK(glDisable(GL_CULL_FACE));
	ellipsoidVAO->Unbind();

	GL_CHECK(glDepthMask(GL_TRUE));
	GL_CHECK(glDisable(GL_BLEND));
}

void Renderer::UploadSelection(const Window& window) {
	selectionPrimitiveCount = 0;
	neighborSelectionPrimitiveCount = 0;
	if (window.selectionType == Window::SEL_NA)
		return;
	const bool requiresSelectionIndex = window.selectionType == Window::SEL_POINT || window.selectionType == Window::SEL_CAMERA;
	if (requiresSelectionIndex && !window.HasSelectionIds())
		return;
	const IDX primarySelectionIdx = window.GetSelectionId();

	// Handle point selection with valid pointViews
	std::vector<float> selectionVertices;
	const MVS::Scene& scene = window.GetScene().GetScene();
	if (window.selectionType == Window::SEL_POINT && scene.IsValid() && scene.pointcloud.IsValid()) {
		if (primarySelectionIdx >= scene.pointcloud.points.size() || primarySelectionIdx >= scene.pointcloud.pointViews.size())
			return;
		// Create line geometry from each camera seeing this point to the point
		const MVS::PointCloud::Point& selectedPoint = scene.pointcloud.points[primarySelectionIdx];
		const MVS::PointCloud::ViewArr& pointViews = scene.pointcloud.pointViews[primarySelectionIdx];
		selectionVertices.reserve(pointViews.size() * 6); // 2 points per line, 3 coordinates per point
		for (const MVS::PointCloud::View& viewIdx : pointViews) {
			ASSERT(viewIdx < scene.images.size());
			const MVS::Image& imageData = scene.images[viewIdx];
			ASSERT(imageData.IsValid());
			// add line from camera center to the selected point
			const Point3f& cameraCenter = imageData.camera.C;
			// first vertex: camera center
			selectionVertices.insert(selectionVertices.end(), {
				cameraCenter.x, cameraCenter.y, cameraCenter.z
			});
			// second vertex: selected point
			selectionVertices.insert(selectionVertices.end(), {
				selectedPoint.x, selectedPoint.y, selectedPoint.z
			});
		}
	}
	// Handle triangle selection
	else if (window.selectionType == Window::SEL_TRIANGLE) {
		const MVS::Mesh& mesh = scene.mesh;
		if (mesh.IsEmpty())
			return;
		selectionVertices.reserve(window.GetSelectionCount() * 18);
		for (IDX selectedFaceIdx : window.GetSelectionIds()) {
			if (selectedFaceIdx >= mesh.faces.size())
				continue;
			const MVS::Mesh::Face& face = mesh.faces[selectedFaceIdx];
			const Point3f& v0 = mesh.vertices[face[0]];
			const Point3f& v1 = mesh.vertices[face[1]];
			const Point3f& v2 = mesh.vertices[face[2]];
			// Line v0-v1
			selectionVertices.insert(selectionVertices.end(), { v0.x, v0.y, v0.z });
			selectionVertices.insert(selectionVertices.end(), { v1.x, v1.y, v1.z });
			// Line v1-v2
			selectionVertices.insert(selectionVertices.end(), { v1.x, v1.y, v1.z });
			selectionVertices.insert(selectionVertices.end(), { v2.x, v2.y, v2.z });
			// Line v2-v0
			selectionVertices.insert(selectionVertices.end(), { v2.x, v2.y, v2.z });
			selectionVertices.insert(selectionVertices.end(), { v0.x, v0.y, v0.z });
		}
		if (selectionVertices.empty())
			return;
	}
	// Handle camera selection
	else if (window.selectionType == Window::SEL_CAMERA) {
		const ImageArr& viewerImages = window.GetScene().GetImages();
		const float depth = window.GetCamera().GetSceneDistance() * window.cameraSize * 10.f;
		bool hasValidCamera = false;
		for (IDX cameraIdx : window.GetSelectionIds()) {
			if (cameraIdx >= viewerImages.size())
				continue;
			const Image& image = viewerImages[cameraIdx];
			const MVS::Image& selectedImage = scene.images[image.idx];
			if (!selectedImage.IsValid())
				continue;
			std::array<Point3f, 4> worldCorners = ComputeCameraFrustumCorners(selectedImage, depth);
			// Reserve space for lines: 4 (center to corners) + 4 (corner rectangle) = 8 lines × 2 vertices × 3 coordinates = 48 floats
			selectionVertices.reserve(selectionVertices.size() + 48);
			// Lines from camera center to each corner (4 lines)
			const Point3f center = selectedImage.camera.C;
			for (int j = 0; j < 4; ++j) {
				selectionVertices.insert(selectionVertices.end(), {center.x, center.y, center.z});
				selectionVertices.insert(selectionVertices.end(), {worldCorners[j].x, worldCorners[j].y, worldCorners[j].z});
			}
			// Rectangle connecting the four corners (4 lines)
			for (int j = 0; j < 4; ++j) {
				const Point3f& corner1 = worldCorners[j];
				const Point3f& corner2 = worldCorners[(j + 1) % 4];
				selectionVertices.insert(selectionVertices.end(), {corner1.x, corner1.y, corner1.z});
				selectionVertices.insert(selectionVertices.end(), {corner2.x, corner2.y, corner2.z});
			}
			hasValidCamera = true;
		}
		if (!hasValidCamera)
			return;
	}

	// Set the primitive count (number of vertices)
	selectionPrimitiveCount = selectionVertices.size() / 3;

	// Add neighbor camera geometry if selected
	if (window.selectedNeighborCamera != NO_ID) {
		const size_t neighborVertexOffset = selectionVertices.size() / 3;
		const Image& image = window.GetScene().GetImages()[window.selectedNeighborCamera];
		const MVS::Image& neighborImage = scene.images[image.idx];
		ASSERT(neighborImage.IsValid());
		const float depth = window.GetCamera().GetSceneDistance() * window.cameraSize * 10.f;
		// Get frustum corners for the neighbor camera
		std::array<Point3f, 4> worldCorners = ComputeCameraFrustumCorners(neighborImage, depth);
		// Lines from camera center to each corner (4 lines)
		const Point3f center = neighborImage.camera.C;
		for (int j = 0; j < 4; ++j) {
			// Line from center to corner j
			selectionVertices.insert(selectionVertices.end(), {center.x, center.y, center.z});
			selectionVertices.insert(selectionVertices.end(), {worldCorners[j].x, worldCorners[j].y, worldCorners[j].z});
		}
		// Rectangle connecting the four corners (4 lines)
		for (int j = 0; j < 4; ++j) {
			// Line from corner j to corner (j+1)%4
			const Point3f& corner1 = worldCorners[j];
			const Point3f& corner2 = worldCorners[(j + 1) % 4];
			selectionVertices.insert(selectionVertices.end(), {corner1.x, corner1.y, corner1.z});
			selectionVertices.insert(selectionVertices.end(), {corner2.x, corner2.y, corner2.z});
		}
		neighborSelectionPrimitiveCount = selectionVertices.size() / 3 - neighborVertexOffset;
	}

	// Upload all selection geometry to GPU if we have any
	if (!selectionVertices.empty())
		selectionVBO->SetData(selectionVertices);
}

void Renderer::UploadBounds(const MVS::Scene& scene) {
	boundsPrimitiveCount = 0;
	if (!scene.IsBounded())
		return;
	Point3f::EVec corners[8];
	scene.obb.GetCorners(corners);

	// Create wireframe lines for the bounding box
	// Each line needs 2 vertices, so we'll have 12 lines * 2 vertices = 24 vertices
	boundsPrimitiveCount = 24;
	std::vector<float> wireframeVertices;
	wireframeVertices.reserve(boundsPrimitiveCount * 3);

	// Define the 12 edges of a cube by vertex indices
	// Each edge connects two corners that differ by exactly one bit (one axis)
	// Bit pattern: corner i = (bit2=z, bit1=y, bit0=x) where 0=min, 1=max
	const int edges[12][2] = {
		// X-axis edges (differ in bit 0)
		{0,1}, {2,3}, {4,5}, {6,7},
		// Y-axis edges (differ in bit 1)
		{0,2}, {1,3}, {4,6}, {5,7},
		// Z-axis edges (differ in bit 2)
		{0,4}, {1,5}, {2,6}, {3,7}
	};

	// Generate line segments for each edge
	for (int i = 0; i < 12; ++i) {
		// First vertex of the line
		const Point3f::EVec& p1 = corners[edges[i][0]];
		wireframeVertices.push_back(p1.x());
		wireframeVertices.push_back(p1.y());
		wireframeVertices.push_back(p1.z());
		// Second vertex of the line
		const Point3f::EVec& p2 = corners[edges[i][1]];
		wireframeVertices.push_back(p2.x());
		wireframeVertices.push_back(p2.y());
		wireframeVertices.push_back(p2.z());
	}
	boundsVBO->SetData(wireframeVertices);
}

void Renderer::BeginFrame(const Camera& camera, const Eigen::Vector4f& clearColor) {
	// Set clear color and clear buffers
	GL_CHECK(glClearColor(clearColor.x(), clearColor.y(), clearColor.z(), clearColor.w()));
	GL_CHECK(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

	// Update view-projection matrices
	UpdateViewProjection(camera);
}

void Renderer::UpdateViewProjection(const Camera& camera) {
	// Convert from double to float matrices
	Eigen::Matrix4d viewMatrix = camera.GetViewMatrix();
	Eigen::Matrix4d projMatrix = camera.GetProjectionMatrix();
	Eigen::Matrix4d vpMatrix = projMatrix * viewMatrix;

	ViewProjectionData vpData;
	vpData.view = viewMatrix.cast<float>();
	vpData.projection = projMatrix.cast<float>();
	vpData.viewProjection = vpMatrix.cast<float>();
	vpData.cameraPos = camera.GetPosition().cast<float>();

	viewProjectionUBO->SetData(vpData);
}

void Renderer::SetLighting(const Eigen::Vector3f& direction, float intensity, const Eigen::Vector3f& color) {
	LightingData lightData;
	lightData.lightDirection = direction.normalized();
	lightData.lightIntensity = intensity;
	lightData.lightColor = color;
	lightData.ambientStrength = 0.1f;
	lightData.ambientColor = Eigen::Vector3f(1.f, 1.f, 1.f);

	lightingUBO->SetData(lightData);
}

void Renderer::RenderPointCloud(const Window& window) {
	if (pointCount == 0) return;

	// Use the point cloud shader
	pointCloudShader->Use();

	// Set uniforms based on window settings
	pointCloudShader->SetFloat("pointSize", window.pointSize);

	pointCloudVAO->Bind();

	if (layerPassFilter.empty()) {
		GL_CHECK(glDrawArrays(GL_POINTS, 0, pointCount));
	} else {
		for (const LayerIndexRange& range : pointLayerRanges)
			if (IsLayerInPass(range.layerID))
				GL_CHECK(glDrawArrays(GL_POINTS, (GLint)range.offset, (GLsizei)range.count));
	}

	pointCloudVAO->Unbind();
}

void Renderer::RenderPointCloudNormals(const Window& window) {
	if (pointNormalCount == 0) return;

	// Use the point cloud normals shader
	pointCloudNormalsShader->Use();

	// Set normal color (cyan for good visibility)
	pointCloudNormalsShader->SetVector3("normalColor", Eigen::Vector3f(0.f, 1.f, 1.f));

	pointCloudNormalsVAO->Bind();

	if (layerPassFilter.empty()) {
		GL_CHECK(glDrawArrays(GL_LINES, 0, pointNormalCount));
	} else {
		for (const LayerIndexRange& range : pointLayerRanges)
			if (range.normalCount > 0 && IsLayerInPass(range.layerID))
				GL_CHECK(glDrawArrays(GL_LINES, (GLint)range.normalOffset, (GLsizei)range.normalCount));
	}

	pointCloudNormalsVAO->Unbind();
}

void Renderer::UpdateLighting() {
	// Implementation for updating lighting UBO if necessary, currently handled by SetLighting
}

void Renderer::RenderMesh(const Window& window) {
	if (meshFaceCounts.empty())
		return;

	const bool isWireframe = window.showMeshWireframe;
	const bool texturesEnabled = window.showMeshTextured;
	if (isWireframe)
		GL_CHECK(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
	else
		GL_CHECK(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));

	meshVAO->Bind();
	meshEBO->Bind();

	// Render each sub-mesh
	FOREACH(i, meshFaceCounts) {
		// check if this sub-mesh should be rendered
		if (!window.meshSubMeshVisible.empty() && !window.meshSubMeshVisible[i])
			continue;
		if (!IsLayerInPass(GetMeshSubMeshLayerID(i)))
			continue;
		const uint32_t textureIndex = i < meshTextureIndices.size() ? meshTextureIndices[i] : NO_ID;
		const bool textureValid = textureIndex < meshTextures.size() && meshTextures[textureIndex].IsValid();
		// check if this sub-mesh has a valid texture
		const bool hasTexture = texturesEnabled && textureValid;
		// select the appropriate shader based on texture availability for this sub-mesh
		Shader* currentMeshShader = hasTexture ? meshTexturedShader.get() : meshShader.get();
		currentMeshShader->Use();
		// set uniforms
		currentMeshShader->SetBool("wireframe", isWireframe);
		if (hasTexture) {
			GL_CHECK(glActiveTexture(GL_TEXTURE0));
			GL_CHECK(glBindTexture(GL_TEXTURE_2D, meshTextures[textureIndex].GetID()));
			currentMeshShader->SetInt("diffuseTexture", 0);
		} else {
			currentMeshShader->SetVector3("meshColor", Eigen::Vector3f(0.8f, 0.8f, 0.8f));
		}
		// draw this sub-mesh
		const MVS::Mesh::FIndex faceCountOffset = i > 0 ? meshFaceCounts[i - 1] : 0u;
		const MVS::Mesh::FIndex faceCountTotal = meshFaceCounts[i];
		const MVS::Mesh::FIndex faceCount = faceCountTotal - faceCountOffset;
		const void* indexPtr = reinterpret_cast<const void*>(faceCountOffset * 3 * sizeof(uint32_t));
		GL_CHECK(glDrawElements(GL_TRIANGLES, faceCount * 3, GL_UNSIGNED_INT, indexPtr));
	}

	meshVAO->Unbind();

	// Reset polygon mode
	GL_CHECK(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
}

void Renderer::RenderCameras(const Window& window) {
	if (cameraPointIndexCount == 0 && cameraLineIndexCount == 0)
		return;

	cameraShader->Use();

	cameraVAO->Bind();
	cameraEBO->Bind();

	// The camera EBO stores all point indices first, then all line indices; each layer owns a
	// contiguous sub-range in both blocks, so a pass filter reduces to per-layer draw calls.
	const bool displayDots = window.cameraDisplayType == Window::CAMERA_DISPLAY_DOT;
	if (layerPassFilter.empty()) {
		if (displayDots) {
			if (cameraPointIndexCount > 0)
				GL_CHECK(glDrawElements(GL_POINTS, static_cast<GLsizei>(cameraPointIndexCount), GL_UNSIGNED_INT, 0));
			if (window.showCameraLookAt && cameraLineIndexCount > 0) {
				const void* lineOffset = reinterpret_cast<const void*>(cameraPointIndexCount * sizeof(uint32_t));
				GL_CHECK(glDrawElements(GL_LINES, static_cast<GLsizei>(cameraLineIndexCount), GL_UNSIGNED_INT, lineOffset));
			}
		} else {
			if (cameraLineIndexCount > 0)
				GL_CHECK(glDrawElements(GL_LINES, static_cast<GLsizei>(cameraLineIndexCount), GL_UNSIGNED_INT, 0));
		}
	} else {
		for (const CameraLayerRange& range : cameraLayerRanges) {
			if (!IsLayerInPass(range.layerID))
				continue;
			if (displayDots && range.pointIndexCount > 0) {
				const void* pointOffset = reinterpret_cast<const void*>(range.pointIndexOffset * sizeof(uint32_t));
				GL_CHECK(glDrawElements(GL_POINTS, static_cast<GLsizei>(range.pointIndexCount), GL_UNSIGNED_INT, pointOffset));
			}
			if ((!displayDots || window.showCameraLookAt) && range.lineIndexCount > 0) {
				const void* lineOffset = reinterpret_cast<const void*>((cameraPointIndexCount + range.lineIndexOffset) * sizeof(uint32_t));
				GL_CHECK(glDrawElements(GL_LINES, static_cast<GLsizei>(range.lineIndexCount), GL_UNSIGNED_INT, lineOffset));
			}
		}
	}

	cameraVAO->Unbind();
}

void Renderer::RenderImageOverlays(const Window& window) {
	const Camera& camera(window.GetCamera());
	if (imageOverlayIndexCount == 0 || !camera.IsCameraViewMode())
		return;
	const Scene& sceneController(window.GetScene());
	const Scene::Layer* layer(sceneController.GetActiveLayer());
	if (layer == nullptr || !layer->visible)
		return;
	const CameraLayerRange* range(FindCameraLayerRange(layer->id));
	const MVS::IIndex cameraID(camera.GetCurrentCamID());
	if (range == nullptr || cameraID >= layer->images.size())
		return;
	Image& image(const_cast<Image&>(layer->images[cameraID]));
	if (!image.IsValid()) {
		if (!image.IsImageValid())
			return;
		image.TransferImage();
	}

	// Set up for 3D rendering with special handling for transparency
	GL_CHECK(glDisable(GL_DEPTH_TEST)); // Temporarily disable depth testing to ensure visibility
	GL_CHECK(glEnable(GL_BLEND));
	GL_CHECK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

	// Use the 3D overlay shader
	imageOverlayShader->Use();

	// Set opacity
	imageOverlayShader->SetFloat("opacity", window.imageOverlayOpacity);
	imageOverlayShader->SetInt("overlayTexture", 0);

	// Render the specific overlay for this camera
	imageOverlayVAO->Bind();
	imageOverlayEBO->Bind();

	GL_CHECK(glActiveTexture(GL_TEXTURE0));
	image.Bind();
	const void* indexOffset = reinterpret_cast<const void*>((range->offset + cameraID) * 6 * sizeof(uint32_t));
	GL_CHECK(glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, indexOffset));

	imageOverlayVAO->Unbind();

	// Restore previous depth test state
	GL_CHECK(glDisable(GL_BLEND));
	GL_CHECK(glEnable(GL_DEPTH_TEST));
}

void Renderer::RenderSelection(const Window& window) {
	// Highlight selected point in point cloud if applicable
	if (window.showPointCloud && window.selectionType == Window::SEL_POINT && pointCount > 0 && window.HasSelectionIds()) {
		const Scene::Layer* activeLayer = window.GetScene().GetActiveLayer();
		const LayerIndexRange* pointRange = activeLayer != nullptr ? FindPointLayerRange(activeLayer->id) : nullptr;
		// Use the geometry selection shader for highlighting
		geometrySelectionShader->Use();
		geometrySelectionShader->SetBool("useHighlight", true);
		geometrySelectionShader->SetFloat("highlightOpacity", 0.8f);

		// Set highlight size and color for points (red)
		geometrySelectionShader->SetVector3("highlightColor", Eigen::Vector3f(1.f, 0.f, 0.f));
		geometrySelectionShader->SetFloat("pointSize", window.pointSize * 3.f);

		// We need access to the actual point cloud data to extract selected point
		pointCloudVAO->Bind();

		// Render each selected point individually using glDrawArrays with offset
		for (IDX selectedIdx : window.GetSelectionIds()) {
			if (pointRange == nullptr || selectedIdx >= pointRange->count)
				continue;
			GL_CHECK(glDrawArrays(GL_POINTS, static_cast<GLint>(pointRange->offset + selectedIdx), 1));
		}

		pointCloudVAO->Unbind();
	}

	// Only render if we have selection geometry
	if (selectionPrimitiveCount == 0)
		return;

	// Render selection lines
	GL_CHECK(glDisable(GL_DEPTH_TEST));

	selectionShader->Use();
	GLint viewport[4] = { 0, 0, 1, 1 };
	GL_CHECK(glGetIntegerv(GL_VIEWPORT, viewport));
	selectionShader->SetVector2("viewportSize", Eigen::Vector2f((float)viewport[2], (float)viewport[3]));
	selectionVAO->Bind();

	// Use different colors for different selection types
	if (window.selectionType == Window::SEL_POINT) {
		selectionShader->SetFloat("lineWidth", MAXF(window.pointSize*0.5f, 1.f)); // Line width based on point size
		selectionShader->SetVector3("selectionColor", Eigen::Vector3f(1.f, 0.f, 0.f)); // Red lines for points
	} else if (window.selectionType == Window::SEL_TRIANGLE) {
		selectionShader->SetFloat("lineWidth", 2.f);
		selectionShader->SetVector3("selectionColor", Eigen::Vector3f(1.f, 0.f, 0.f)); // Red lines for triangles
	} else if (window.selectionType == Window::SEL_CAMERA) {
		selectionShader->SetFloat("lineWidth", 1.f);
		selectionShader->SetVector3("selectionColor", Eigen::Vector3f(0.f, 1.f, 1.f)); // Cyan lines for cameras
	} else {
		selectionShader->SetFloat("lineWidth", 1.f);
		selectionShader->SetVector3("selectionColor", Eigen::Vector3f(1.f, 1.f, 0.f)); // Yellow for other selections
	}

	// Render primary selection geometry as lines
	GL_CHECK(glDrawArrays(GL_LINES, 0, selectionPrimitiveCount));

	// Render neighbor camera with different color
	if (window.selectedNeighborCamera != NO_ID && neighborSelectionPrimitiveCount > 0) {
		selectionShader->SetFloat("lineWidth", 1.f);
		selectionShader->SetVector3("selectionColor", Eigen::Vector3f(1.f, 0.f, 1.f)); // Magenta for neighbor camera
		// Render neighbor camera geometry as lines (starting after primary selection vertices)
		GL_CHECK(glDrawArrays(GL_LINES, selectionPrimitiveCount, neighborSelectionPrimitiveCount));
	}

	selectionVAO->Unbind();

	GL_CHECK(glEnable(GL_DEPTH_TEST));
}

void Renderer::RenderBounds() {
	if (boundsPrimitiveCount == 0)
		return;

	boundsShader->Use();
	boundsShader->SetVector3("boundsColor", Eigen::Vector3f(0.f, 1.f, 0.f)); // Green

	boundsVAO->Bind();

	// Render as lines (each pair of vertices forms a line)
	GL_CHECK(glDrawArrays(GL_LINES, 0, boundsPrimitiveCount));

	boundsVAO->Unbind();
}

// Render the interactive bounding-box edit gizmos.
// Drawn only while the bounding-box edit control mode is active; this routine
// owns no state - every call re-uploads the (cheap) handle positions from
// the provided OBB. Corners and face centers are rendered as GL_POINTS using
// the existing boundsShader; rotation rings reuse the arcball gizmoShader/VBO
// (unit circle) with per-axis modelMatrix transforms.
void Renderer::RenderBoundingBoxGizmos(const OBB3f& obb,
                                       int hoverCornerIdx,
                                       int hoverFaceIdx,
                                       int hoverAxisIdx)
{
	if (!obb.IsValid() || !boundsShader || !bboxHandleVAO || !bboxHandleVBO)
		return;

	// Gather handle world positions via the shared BoxHandleInteraction helpers
	// so all three consumers (picking, rendering, dragging) agree on geometry.
	Eigen::Vector3f corners[8];
	BoxHandleInteraction::GetCornerWorldPositions(obb, corners);
	Eigen::Vector3f faceCenters[6];
	BoxHandleInteraction::GetFaceCenterWorldPositions(obb, faceCenters);

	// Upload the 14 handle positions as a flat float array (corners 0..7
	// followed by face centers 0..5). The layout matches the draw-time indexing.
	std::vector<float> handleVerts;
	handleVerts.reserve(14 * 3);
	for (int i = 0; i < 8; ++i) {
		handleVerts.push_back(corners[i].x());
		handleVerts.push_back(corners[i].y());
		handleVerts.push_back(corners[i].z());
	}
	for (int i = 0; i < 6; ++i) {
		handleVerts.push_back(faceCenters[i].x());
		handleVerts.push_back(faceCenters[i].y());
		handleVerts.push_back(faceCenters[i].z());
	}

	bboxHandleVBO->Bind();
	bboxHandleVBO->SetData(handleVerts);

	// Depth test stays enabled so handles occlude correctly against geometry,
	// but we disable it just for the draw so handles remain visible even when
	// they sit inside the bounding box wireframe. This mirrors the arcball
	// gizmo convention (always visible on top).
	GLboolean depthWasEnabled;
	GL_CHECK(glGetBooleanv(GL_DEPTH_TEST, &depthWasEnabled));
	GL_CHECK(glDisable(GL_DEPTH_TEST));

	boundsShader->Use();
	bboxHandleVAO->Bind();

	// --- Corner handles ---
	{
		const Eigen::Vector3f cornerColor(1.0f, 0.85f, 0.2f);     // warm yellow
		const Eigen::Vector3f hoverColor (1.0f, 0.4f,  0.1f);     // bright orange
		const float basePointSize  = 12.0f;
		const float hoverPointSize = 18.0f;

		GL_CHECK(glPointSize(basePointSize));
		boundsShader->SetVector3("boundsColor", cornerColor);
		GL_CHECK(glDrawArrays(GL_POINTS, 0, 8));

		if (hoverCornerIdx >= 0 && hoverCornerIdx < 8) {
			GL_CHECK(glPointSize(hoverPointSize));
			boundsShader->SetVector3("boundsColor", hoverColor);
			GL_CHECK(glDrawArrays(GL_POINTS, hoverCornerIdx, 1));
		}
	}

	// --- Face-center handles ---
	{
		const Eigen::Vector3f faceColor (0.3f, 0.9f, 1.0f);       // cyan
		const Eigen::Vector3f hoverColor(1.0f, 0.4f, 0.1f);       // bright orange
		const float basePointSize  = 10.0f;
		const float hoverPointSize = 16.0f;

		GL_CHECK(glPointSize(basePointSize));
		boundsShader->SetVector3("boundsColor", faceColor);
		GL_CHECK(glDrawArrays(GL_POINTS, 8, 6));

		if (hoverFaceIdx >= 0 && hoverFaceIdx < 6) {
			GL_CHECK(glPointSize(hoverPointSize));
			boundsShader->SetVector3("boundsColor", hoverColor);
			GL_CHECK(glDrawArrays(GL_POINTS, 8 + hoverFaceIdx, 1));
		}
	}

	bboxHandleVAO->Unbind();
	GL_CHECK(glPointSize(1.0f));

	// --- Rotation rings (3 per-axis circles) ---
	// Reuse the arcball gizmo VAO/VBO which already holds a unit circle.
	if (gizmoShader && gizmoVAO) {
		gizmoVAO->Bind();
		gizmoShader->Use();

		const float ringRadius = BoxHandleInteraction::GetRotationRingRadius(obb);
		const Eigen::Vector3f center = obb.m_pos;

		// Local axes in world coordinates - row(k) of m_rot (world->local).
		const Eigen::Matrix3f rot = obb.m_rot;

		const Eigen::Vector3f baseColors[3] = {
			Eigen::Vector3f(1.0f, 0.35f, 0.35f), // X - red
			Eigen::Vector3f(0.35f, 1.0f, 0.35f), // Y - green
			Eigen::Vector3f(0.35f, 0.35f, 1.0f), // Z - blue
		};
		const Eigen::Vector3f hoverColor(1.0f, 0.9f, 0.2f);

		for (int axis = 0; axis < 3; ++axis) {
			// Target: a circle in world space whose plane is perpendicular to
			// the local axis 'axis'. The unit circle geometry lies in XY (z=0),
			// so we build a frame {e1, e2, n} where n = axis direction in world,
			// and e1, e2 span the ring plane.
			Eigen::Vector3f n = rot.row(axis).normalized();
			Eigen::Vector3f e1;
			// Pick a stable helper vector to derive e1.
			if (std::abs(n.x()) < 0.9f)
				e1 = Eigen::Vector3f::UnitX().cross(n);
			else
				e1 = Eigen::Vector3f::UnitY().cross(n);
			if (e1.norm() < 1e-6f) {
				// Degenerate; skip this ring.
				continue;
			}
			e1.normalize();
			Eigen::Vector3f e2 = n.cross(e1);

			// Build the 4x4 model matrix: columns [e1*r, e2*r, n*r, center]
			// with homogeneous row [0 0 0 1].
			Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
			model.block<3, 1>(0, 0) = e1 * ringRadius;
			model.block<3, 1>(0, 1) = e2 * ringRadius;
			model.block<3, 1>(0, 2) = n  * ringRadius;
			model.block<3, 1>(0, 3) = center;

			const bool isHovered = (hoverAxisIdx == axis);
			gizmoShader->SetMatrix4("modelMatrix", model);
			gizmoShader->SetVector3("gizmoColor", isHovered ? hoverColor : baseColors[axis]);
			gizmoShader->SetFloat("opacity", isHovered ? 1.0f : 0.85f);

			GL_CHECK(glDrawElements(GL_LINES, gizmoCircleIndexCount, GL_UNSIGNED_INT, 0));
		}

		gizmoVAO->Unbind();
	}

	if (depthWasEnabled)
		GL_CHECK(glEnable(GL_DEPTH_TEST));
}

void Renderer::RenderCoordinateAxes(const Camera& camera) {
	if (!axesShader || !axesVAO)
		return;

	// Save current viewport and depth test state
	GLint oldViewport[4];
	GL_CHECK(glGetIntegerv(GL_VIEWPORT, oldViewport));

	// Set up a small viewport in the bottom right corner
	const int axesSize = 100; // Size of the axes widget
	const int margin = 10;	// Margin from screen edges

	GL_CHECK(glViewport(
		oldViewport[2] - axesSize - margin,  // x: right side minus size and margin
		margin,							     // y: bottom with margin
		axesSize,							 // width
		axesSize							 // height
	));

	// Disable depth testing
	GL_CHECK(glDisable(GL_DEPTH_TEST));

	axesShader->Use();

	// Create an orthographic projection matrix that maps [-1,1] to the widget viewport
	Eigen::Matrix4f orthoProj = Eigen::Matrix4f::Identity();
	orthoProj(0,0) = 1.5f;  // Scale X to fit nicely in widget
	orthoProj(1,1) = 1.5f;  // Scale Y to fit nicely in widget
	orthoProj(2,2) = -0.1f; // Small Z range for orthographic

	// Get only the rotation part of the view matrix (no translation)
	Eigen::Matrix4d viewMatrix = camera.GetViewMatrix();
	Eigen::Matrix4f rotationOnlyView = Eigen::Matrix4f::Identity();
	rotationOnlyView.topLeftCorner<3, 3>() = viewMatrix.topLeftCorner<3, 3>().cast<float>();

	// Combine projection and rotation-only view
	Eigen::Matrix4f axesViewProj = orthoProj * rotationOnlyView;

	// Set the axes-specific view-projection matrix
	axesShader->SetMatrix4("viewProjection", axesViewProj);

	axesVAO->Bind();

	// Render as lines
	GL_CHECK(glDrawArrays(GL_LINES, 0, 6)); // 3 axes, 2 vertices each

	axesVAO->Unbind();

	// Restore original viewport and depth test state
	GL_CHECK(glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]));
	GL_CHECK(glEnable(GL_DEPTH_TEST));
}

void Renderer::RenderArcballGizmos(const Camera& camera, const class ArcballControls& controls) {
	if (!gizmoShader || !gizmoVAO || !controls.getEnableGizmos())
		return;

	gizmoVAO->Bind();

	// Get the trackball center (target) and radius from the controls
	Eigen::Vector3d target = camera.GetTarget();

	// Calculate gizmo size based on camera distance and viewport
	// This mimics the three.js trackball radius calculation
	double distance = (camera.GetPosition() - target).norm();
	float gizmoRadius;

	if (camera.IsOrthographic()) {
		// For orthographic camera, use a fixed size relative to viewport
		float minSide = MINF(camera.GetSize().width, camera.GetSize().height);
		gizmoRadius = minSide * 0.67f / (2.f * 1.f); // Assume zoom = 1.0 for now
	} else {
		// For perspective camera, calculate based on FOV and distance
		float fov = D2R(camera.GetFOV());
		float minSide = MINF(camera.GetSize().width, camera.GetSize().height);
		gizmoRadius = distance * TAN(fov / 2.f) * 0.67f * minSide / camera.GetSize().height;
	}

	// Set transparency based on active state
	float opacity = controls.getGizmosActive() ? 1.f : 0.6f;

	// Colors for X, Y, Z axes (red, green, blue)
	Eigen::Vector3f colors[3] = {
		Eigen::Vector3f(1.f, 0.5f, 0.5f), // X - red
		Eigen::Vector3f(0.5f, 1.f, 0.5f), // Y - green
		Eigen::Vector3f(0.5f, 0.5f, 1.f)  // Z - blue
	};

	// Render three circles for X, Y, Z axes using the gizmo shader
	gizmoShader->Use();

	for (int axis = 0; axis < 3; ++axis) {
		// Create transformation matrix for each circle
		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

		// Translate to target position
		transform.col(3).head<3>() = target.cast<float>();

		// Scale to gizmo radius
		transform.topLeftCorner<3, 3>() *= gizmoRadius;

		// Rotate circle to align with axis
		if (axis == 0) {
			// X-axis: rotate 90 degrees around Y-axis
			transform.topLeftCorner<3, 3>() *= Eigen::AngleAxisf(FHALF_PI, Eigen::Vector3f::UnitY()).toRotationMatrix();
		} else if (axis == 2) {
			// Z-axis: rotate 90 degrees around X-axis
			transform.topLeftCorner<3, 3>() *= Eigen::AngleAxisf(FHALF_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();
		}
		// Y-axis uses default circle orientation (no additional rotation needed)

		// Set uniforms
		gizmoShader->SetMatrix4("modelMatrix", transform);
		gizmoShader->SetVector3("gizmoColor", colors[axis]);
		gizmoShader->SetFloat("opacity", opacity);

		// Render circle as lines
		GL_CHECK(glDrawElements(GL_LINES, gizmoCircleIndexCount, GL_UNSIGNED_INT, 0));
	}

	// Render gizmo center axes if enabled
	if (controls.getEnableGizmosCenter()) {
		// Continue using the same gizmo shader for consistency
		// Render each axis with its corresponding color
		for (int axis = 0; axis < 3; ++axis) {
			// Create transformation matrix for the center axes
			Eigen::Matrix4f centerTransform = Eigen::Matrix4f::Identity();

			// Translate to target position
			centerTransform.col(3).head<3>() = target.cast<float>();

			// Scale to a smaller size (relative to gizmo radius)
			float centerScale = gizmoRadius * 0.15f; // 15% of gizmo radius
			centerTransform.topLeftCorner<3, 3>() *= centerScale;

			// Set uniforms
			gizmoShader->SetMatrix4("modelMatrix", centerTransform);
			gizmoShader->SetVector3("gizmoColor", colors[axis]); // Use same colors as circles
			gizmoShader->SetFloat("opacity", opacity);

			// Calculate vertex range for this axis (2 vertices per axis)
			int axisBaseVertex = gizmoCenterAxesBaseVertex + (axis * 2);

			// Render this axis as lines
			GL_CHECK(glDrawArrays(GL_LINES, axisBaseVertex, 2));
		}
	}

	gizmoVAO->Unbind();
}

void Renderer::RenderSelectionOverlay(const Window& window) {
	// Only render overlay if in selection mode
	if (window.GetControlMode() != Window::CONTROL_SELECTION)
		return;
	SelectionController& selectionController = window.GetSelectionController();
	// Only render if selecting or has a selection
	if (!selectionController.isSelecting() && !selectionController.hasSelection())
		return;
	// Safety check: ensure all required objects are initialized
	if (!selectionOverlayShader || !selectionOverlayVAO || !selectionOverlayVBO)
		return;
	// Disable depth testing for 2D overlay
	GL_CHECK(glDisable(GL_DEPTH_TEST));

	selectionOverlayShader->Use();
	selectionOverlayShader->SetVector3("overlayColor", Eigen::Vector3f(1.f, 1.f, 0.f)); // Yellow
	selectionOverlayShader->SetFloat("overlayOpacity", 0.8f);

	selectionOverlayVAO->Bind();

	if (selectionController.getSelectionMode() == SelectionController::MODE_BOX) {
		// Render box selection if active
		if (selectionController.isSelecting()) {
			const auto& start = selectionController.getSelectionStart();
			const auto& end = selectionController.getSelectionEnd();
			// SelectionController coordinates are already normalized [-1, 1]
			float x1 = static_cast<float>(start.x());
			float y1 = static_cast<float>(start.y());
			float x2 = static_cast<float>(end.x());
			float y2 = static_cast<float>(end.y());
			std::vector<float> boxVertices {
				x1, y1,
				x2, y1,
				x2, y2,
				x1, y2,
				x1, y1
			};
			selectionOverlayVBO->SetData(boxVertices);
			GL_CHECK(glDrawArrays(GL_LINE_STRIP, 0, 5));
		}
	} else {
		// Render lasso/circle selection path
		const auto& path = selectionController.getCurrentSelectionPath();
		if (!path.empty()) {
			std::vector<float> pathVertices;
			pathVertices.reserve(path.size() * 2);
			for (const auto& point : path) {
				// SelectionController coordinates are already normalized [-1, 1]
				float x = static_cast<float>(point.x());
				float y = static_cast<float>(point.y());
				pathVertices.push_back(x);
				pathVertices.push_back(y);
			}
			if (!pathVertices.empty()) {
				selectionOverlayVBO->SetData(pathVertices);
				GL_CHECK(glDrawArrays(GL_LINE_STRIP, 0, pathVertices.size() / 2));
			}
		}
	}

	selectionOverlayVAO->Unbind();

	// Restore OpenGL state
	GL_CHECK(glEnable(GL_DEPTH_TEST));
}

void Renderer::RenderSelectedGeometry(const Window& window) {
	// Render selected geometry regardless of control mode, as long as we have selections
	const SelectionController& selectionController = window.GetSelectionController();
	if (!selectionController.hasSelection())
		return;
	const Scene::Layer* activeLayer = window.GetScene().GetActiveLayer();
	if (activeLayer == nullptr)
		return;

	// Enable blending for highlighting effect
	GL_CHECK(glEnable(GL_BLEND));
	GL_CHECK(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

	// Use the geometry selection shader for highlighting
	geometrySelectionShader->Use();
	geometrySelectionShader->SetBool("useHighlight", true);
	geometrySelectionShader->SetFloat("highlightOpacity", 0.8f);

	// Render selected points with highlighting
	const auto& selectedPointIndices = selectionController.getSelectedPointIndices();
	if (window.showPointCloud && !selectedPointIndices.empty() && pointCount > 0) {
		const LayerIndexRange* pointRange = FindPointLayerRange(activeLayer->id);
		// set highlight size and color for points (red)
		geometrySelectionShader->SetVector3("highlightColor", Eigen::Vector3f(1.f, 0.f, 0.f));
		geometrySelectionShader->SetFloat("pointSize", window.pointSize * 2.5f);
		// render each selected point individually using glDrawArrays with offset
		pointCloudVAO->Bind();
		for (const auto& pointIdx : selectedPointIndices) {
			if (pointRange == nullptr || pointIdx >= pointRange->count)
				continue;
			GL_CHECK(glDrawArrays(GL_POINTS, pointRange->offset + pointIdx, 1));
		}
		pointCloudVAO->Unbind();
	}

	// Render selected faces with highlighting (wireframe overlay)
	const auto& selectedFaceIndices = selectionController.getSelectedFaceIndices();
	if (window.showMesh && !selectedFaceIndices.empty() && !meshFaceCounts.empty()) {
		// set highlight color for faces (red)
		geometrySelectionShader->SetVector3("highlightColor", Eigen::Vector3f(1.f, 0.f, 0.f));
		// render as wireframe overlay to show selection
		GL_CHECK(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
		// enable polygon offset to render selection on top of existing mesh
		GL_CHECK(glEnable(GL_POLYGON_OFFSET_LINE));
		GL_CHECK(glPolygonOffset(-1.f, -1.f)); // more aggressive offset
		meshVAO->Bind();
		meshEBO->Bind();
		for (const auto& faceIdx : selectedFaceIndices) {
			uint32_t globalFaceIdx;
			if (!MapLocalFace(activeLayer->id, faceIdx, globalFaceIdx))
				continue;
			if (globalFaceIdx >= globalFaceSubMeshIndices.size())
				continue;
			const uint32_t submeshIdx = globalFaceSubMeshIndices[globalFaceIdx];
			if (submeshIdx >= meshFaceCounts.size())
				continue;
			// check if this submesh is visible
			if (!window.meshSubMeshVisible.empty() && !window.meshSubMeshVisible[submeshIdx])
				continue;
			const MVS::Mesh::FIndex faceCountOffset = submeshIdx > 0 ? meshFaceCounts[submeshIdx - 1] : 0u;
			const MVS::Mesh::FIndex faceIdxInSubmesh = globalFaceIdx - faceCountOffset;
			const void* indexPtr = reinterpret_cast<const void*>((faceCountOffset + faceIdxInSubmesh) * 3 * sizeof(uint32_t));
			// render this single face (3 indices)
			GL_CHECK(glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, indexPtr));
		}
		meshVAO->Unbind();
		// restore rendering state
		GL_CHECK(glDisable(GL_POLYGON_OFFSET_LINE));
		GL_CHECK(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
	}

	// Reset shader state
	geometrySelectionShader->SetBool("useHighlight", false);

	// Restore OpenGL state
	GL_CHECK(glDisable(GL_BLEND));
}

void Renderer::EndFrame() {
	// Swap buffers is handled by GLFW in the Window class
	// This method can be used for cleanup or final operations if needed
}

void Renderer::ReleasePickerBuffers() {
	if (pickIDTex) { GL_CHECK(glDeleteTextures(1, &pickIDTex)); pickIDTex = 0; }
	if (pickDepthRBO) { GL_CHECK(glDeleteRenderbuffers(1, &pickDepthRBO)); pickDepthRBO = 0; }
	if (pickFBO) { GL_CHECK(glDeleteFramebuffers(1, &pickFBO)); pickFBO = 0; }
	pickFBOSize = cv::Size(0, 0);
}

void Renderer::EnsurePickFBOSize(int width, int height) {
	if (pickFBO != 0 && pickFBOSize.width == width && pickFBOSize.height == height)
		return;

	// Delete previous resources if any
	ReleasePickerBuffers();
	pickFBOSize = cv::Size(width, height);

	// Create integer ID texture
	GL_CHECK(glGenTextures(1, &pickIDTex));
	GL_CHECK(glBindTexture(GL_TEXTURE_2D, pickIDTex));
	GL_CHECK(glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, width, height, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr));
	GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
	GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
	GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));

	// Depth renderbuffer
	GL_CHECK(glGenRenderbuffers(1, &pickDepthRBO));
	GL_CHECK(glBindRenderbuffer(GL_RENDERBUFFER, pickDepthRBO));
	GL_CHECK(glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height));
	GL_CHECK(glBindRenderbuffer(GL_RENDERBUFFER, 0));

	// Framebuffer
	GL_CHECK(glGenFramebuffers(1, &pickFBO));
	GL_CHECK(glBindFramebuffer(GL_FRAMEBUFFER, pickFBO));
	GL_CHECK(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pickIDTex, 0));
	GL_CHECK(glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, pickDepthRBO));
	GL_CHECK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
}

// Perform a GPU pick around screen pixel position with given radius (pixels);
// if a primitive is found returns valid PickResult, where
// pick.idx is the primitive index (point index or face index) depending on isPoint
Renderer::PickResult Renderer::PickPrimitiveAt(const Point2f& screenPos, int radius, const Window& window) {
	// Ensure FBO matches the window framebuffer size
	const cv::Size& vpSize = window.GetSize();
	EnsurePickFBOSize(vpSize.width, vpSize.height);

	// Bind pick FBO
	GL_CHECK(glBindFramebuffer(GL_FRAMEBUFFER, pickFBO));

	// Clear ID attachment (-1 = no hit) and depth
	const GLuint clearID = NO_ID;
	GL_CHECK(glClearBufferuiv(GL_COLOR, 0, &clearID));
	GL_CHECK(glClear(GL_DEPTH_BUFFER_BIT));

	// Limit rasterization to small rectangle around cursor to reduce work
	const int half = MAXF(1, radius);
	// screenPos is in framebuffer pixel coordinates with origin at top-left (from GLFW),
	// while GL scissor/readpixels use a lower-left origin. Convert Y accordingly.
	const int centerX = ROUND2INT(screenPos.x);
	const int centerY = vpSize.height - 1 - ROUND2INT(screenPos.y);
	const int minX = CLAMP(centerX - half, 0, vpSize.width - 1);
	const int minY = CLAMP(centerY - half, 0, vpSize.height - 1);
	const int w = CLAMP(2 * half + 1, 1, vpSize.width - minX);
	const int h = CLAMP(2 * half + 1, 1, vpSize.height - minY);

	GL_CHECK(glEnable(GL_SCISSOR_TEST));
	GL_CHECK(glScissor(minX, minY, w, h));

	// In compare view each layer is displayed only on its assigned side, so restrict
	// the pick pass to the layers actually shown under the cursor and rasterize it
	// with that side's camera and viewport, matching the on-screen rendering.
	std::vector<uint32_t> cursorSideLayers;
	if (window.compareMode) {
		const int cursorSide = screenPos.x >= (float)window.GetCompareSplitX() ? 1 : 0;
		for (const Scene::Layer& layer : window.GetScene().GetLayers())
			if (layer.visible && layer.compareRight == (cursorSide == 1))
				cursorSideLayers.push_back(layer.id);
		const cv::Rect viewport = window.GetCompareViewport(cursorSide);
		GL_CHECK(glViewport(viewport.x, viewport.y, viewport.width, viewport.height));
		UpdateViewProjection(window.GetSideCamera(cursorSide));
	}
	const auto layerAtCursor = [&](uint32_t layerID) {
		return !window.compareMode ||
			std::find(cursorSideLayers.begin(), cursorSideLayers.end(), layerID) != cursorSideLayers.end();
	};

	// Render mesh (triangles) into pick FBO only if mesh rendering is enabled and we have mesh data
	unsigned baseFace = 0;
	if (window.showMesh && !meshFaceCounts.empty()) {
		pickerMeshShader->Use();
		meshVAO->Bind();
		meshEBO->Bind();
		FOREACH(i, meshFaceCounts) {
			// skip invisible submeshes if window indicates it
			if (!window.meshSubMeshVisible.empty() && !window.meshSubMeshVisible[i])
				continue;
			if (!layerAtCursor(GetMeshSubMeshLayerID(i)))
				continue;
			const MVS::Mesh::FIndex faceCountOffset = i > 0 ? meshFaceCounts[i - 1] : 0u;
			const MVS::Mesh::FIndex faceCountTotal = meshFaceCounts[i];
			const MVS::Mesh::FIndex faceCount = faceCountTotal - faceCountOffset;
			pickerMeshShader->SetUInt("uBaseID", faceCountOffset);
			const void* indexPtr = reinterpret_cast<const void*>(faceCountOffset * 3 * sizeof(uint32_t));
			GL_CHECK(glDrawElements(GL_TRIANGLES, faceCount * 3, GL_UNSIGNED_INT, indexPtr));
		}
		meshVAO->Unbind();
		baseFace = meshFaceCounts.back();
	}

	// Render points into pick FBO only if point cloud rendering is enabled and we have points
	if (window.showPointCloud && pointCount > 0) {
		pickerPointsShader->Use();
		pickerPointsShader->SetUInt("uBaseID", baseFace);
		pointCloudVAO->Bind();
		if (window.compareMode) {
			for (const LayerIndexRange& range : pointLayerRanges)
				if (layerAtCursor(range.layerID))
					GL_CHECK(glDrawArrays(GL_POINTS, (GLint)range.offset, (GLsizei)range.count));
		} else {
			GL_CHECK(glDrawArrays(GL_POINTS, 0, pointCount));
		}
		pointCloudVAO->Unbind();
	}

	// Read back ID and depth for the small rectangle
	const size_t numPixels = (size_t)w * (size_t)h;
	std::vector<GLuint> idBuf(numPixels);
	std::vector<float> depthBuf(numPixels);
	GL_CHECK(glPixelStorei(GL_PACK_ALIGNMENT, 1));
	GL_CHECK(glReadBuffer(GL_COLOR_ATTACHMENT0));
	// Read integer ID buffer
	GL_CHECK(glReadPixels(minX, minY, w, h, GL_RED_INTEGER, GL_UNSIGNED_INT, idBuf.data()));
	// Read depth buffer
	GL_CHECK(glReadPixels(minX, minY, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuf.data()));

	// Unbind and restore state
	GL_CHECK(glDisable(GL_SCISSOR_TEST));
	GL_CHECK(glBindFramebuffer(GL_FRAMEBUFFER, 0));
	if (window.compareMode) {
		GL_CHECK(glViewport(0, 0, vpSize.width, vpSize.height));
		UpdateViewProjection(window.GetCamera());
	}

	// Find nearest non-zero id (smallest depth)
	float bestDepth = FLT_MAX;
	GLuint bestID;
	for (size_t i = 0; i < numPixels; ++i) {
		const GLuint idVal = idBuf[i];
		if (idVal == NO_ID)
			continue;
		// depth 1.0 is far plane, prefer smaller values
		const float d = depthBuf[i];
		if (d < bestDepth) {
			bestDepth = d;
			bestID = idVal;
		}
	}
	if (bestDepth >= FLT_MAX)
		return {};

	// Determine if we hit face or point
	PickResult result;
	if (bestID < baseFace) {
		// hit a mesh face
		result.isPoint = false;
		if (!MapGlobalFace(bestID, result.layerID, result.index))
			return {};
		ASSERT(meshEBO && meshVBO);
		MVS::Mesh::Face face;
		meshEBO->GetSubData<uint32_t>(face.ptr(), 3, static_cast<size_t>(bestID) * 3);
		meshVBO->GetSubData<float>(result.points[0].ptr(), 3, static_cast<size_t>(face[0]) * 3);
		meshVBO->GetSubData<float>(result.points[1].ptr(), 3, static_cast<size_t>(face[1]) * 3);
		meshVBO->GetSubData<float>(result.points[2].ptr(), 3, static_cast<size_t>(face[2]) * 3);
	} else {
		// hit a point
		result.isPoint = true;
		if (!MapGlobalPoint(bestID - baseFace, result.layerID, result.index))
			return {};
		ASSERT(pointCloudVBO);
		pointCloudVBO->GetSubData<float>(result.points[0].ptr(), 3, static_cast<size_t>(bestID - baseFace) * 3);
	}
	return result;
}
/*----------------------------------------------------------------*/
