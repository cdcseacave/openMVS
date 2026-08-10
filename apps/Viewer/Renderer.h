/*
 * Renderer.h
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

#pragma once

#include "Camera.h"
#include "Image.h"
#include "Shader.h"
#include "BufferObjects.h"

namespace VIEWER {

// Forward declarations
class Window;
class Scene;

struct ViewProjectionData {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Matrix4f view;
	Eigen::Matrix4f projection;
	Eigen::Matrix4f viewProjection;
	Eigen::Vector3f cameraPos;
	float padding; // Alignment
};

struct LightingData {
	Eigen::Vector3f lightDirection;
	float lightIntensity;
	Eigen::Vector3f lightColor;
	float ambientStrength;
	Eigen::Vector3f ambientColor;
	float padding; // Alignment
};

class Renderer {
private:
	// Uniform Buffer Objects
	std::unique_ptr<UBO> viewProjectionUBO;
	std::unique_ptr<UBO> lightingUBO;

	// Point cloud rendering
	std::unique_ptr<Shader> pointCloudShader;
	std::unique_ptr<Shader> pointCloudNormalsShader;
	std::unique_ptr<VAO> pointCloudVAO;
	std::unique_ptr<VBO> pointCloudVBO, pointCloudColorVBO;
	std::unique_ptr<VAO> pointCloudNormalsVAO;
	std::unique_ptr<VBO> pointCloudNormalsVBO;
	size_t pointCount;
	size_t pointNormalCount;
	struct LayerPrimitiveRef
	{
		uint32_t layerID{NO_ID};
		uint32_t localIndex{NO_ID};
	};
	struct LayerIndexRange
	{
		uint32_t layerID{NO_ID};
		size_t offset{0};
		size_t count{0};
		size_t normalOffset{0}; // normal-line vertex sub-range (points only)
		size_t normalCount{0};
	};
	std::vector<LayerIndexRange> pointLayerRanges;
	// Compare split view: scene passes draw only these layers (empty = draw all)
	std::vector<uint32_t> layerPassFilter;

	// Mesh rendering
	std::unique_ptr<Shader> meshShader;
	std::unique_ptr<Shader> meshTexturedShader;
	std::unique_ptr<VAO> meshVAO;
	std::unique_ptr<VBO> meshVBO, meshEBO, meshNormalVBO, meshTexCoordVBO;
	std::vector<unsigned> meshFaceCounts; // number of faces till each sub-mesh (subtract the previous to get count per sub-mesh)
	ImageArr meshTextures;
	std::vector<uint32_t> meshTextureIndices; // texture index per sub-mesh (NO_ID for untextured sub-meshes)
	std::vector<uint32_t> meshSubMeshLayerIDs; // owning layer ID per sub-mesh
	struct LayerFaceMap
	{
		uint32_t layerID{NO_ID};
		std::vector<uint32_t> localToGlobalFace;
	};
	std::vector<LayerFaceMap> meshLayerFaceMaps;
	std::vector<LayerPrimitiveRef> faceRefs;
	std::vector<uint32_t> globalFaceSubMeshIndices;

	// Geometry selection highlighting (for SelectionController)
	std::unique_ptr<Shader> geometrySelectionShader;

	// Camera frustum rendering
	std::unique_ptr<Shader> cameraShader;
	std::unique_ptr<VAO> cameraVAO;
	std::unique_ptr<VBO> cameraVBO, cameraEBO, cameraColorVBO;
	size_t cameraPointIndexCount;
	size_t cameraLineIndexCount;
	struct CameraLayerRange
	{
		uint32_t layerID{NO_ID};
		size_t offset{0}; // first camera slot (indexes the image-overlay quads)
		size_t count{0};
		size_t pointIndexOffset{0}; // sub-range in the point block of the camera EBO
		size_t pointIndexCount{0};
		size_t lineIndexOffset{0}; // sub-range in the line block of the camera EBO
		size_t lineIndexCount{0};
	};
	std::vector<CameraLayerRange> cameraLayerRanges;

	// Pose-uncertainty ellipsoid rendering (translucent shaded solids, lit + per-vertex color)
	std::unique_ptr<Shader> ellipsoidShader;
	std::unique_ptr<VAO> ellipsoidVAO;
	std::unique_ptr<VBO> ellipsoidVBO, ellipsoidNormalVBO, ellipsoidEBO, ellipsoidColorVBO;
	size_t ellipsoidIndexCount;
	std::vector<Eigen::Vector3f> ellipsoidCenters; // world-space center per accepted ellipsoid, aligned with the EBO slots
	std::vector<uint32_t> ellipsoidLayerIDs; // owning layer per accepted ellipsoid, aligned with ellipsoidCenters
	std::vector<uint32_t> ellipsoidDrawOrder; // reused scratch for the per-frame back-to-front sort

	// 3D image overlay rendering (pre-computed for all images with valid textures)
	std::unique_ptr<Shader> imageOverlayShader;
	std::unique_ptr<VAO> imageOverlayVAO;
	std::unique_ptr<VBO> imageOverlayVBO;
	std::unique_ptr<VBO> imageOverlayEBO;
	size_t imageOverlayIndexCount;

	// Selection rendering
	std::unique_ptr<Shader> selectionShader;
	std::unique_ptr<VAO> selectionVAO;
	std::unique_ptr<VBO> selectionVBO;
	size_t selectionPrimitiveCount;
	size_t neighborSelectionPrimitiveCount;

	// Selection overlay rendering (2D screen space)
	std::unique_ptr<Shader> selectionOverlayShader;
	std::unique_ptr<VAO> selectionOverlayVAO;
	std::unique_ptr<VBO> selectionOverlayVBO;
	size_t selectionOverlayVertexCount;

	// Bounds rendering
	std::unique_ptr<Shader> boundsShader;
	std::unique_ptr<VAO> boundsVAO;
	std::unique_ptr<VBO> boundsVBO;
	size_t boundsPrimitiveCount;

	// Bounding-box edit handles (8 corner points; reuses boundsShader)
	std::unique_ptr<VAO> bboxHandleVAO;
	std::unique_ptr<VBO> bboxHandleVBO;

	// Coordinate axes
	std::unique_ptr<Shader> axesShader;
	std::unique_ptr<VAO> axesVAO;
	std::unique_ptr<VBO> axesVBO, axesColorVBO;

	// Arcball gizmos (combined buffer for circles and center axes)
	std::unique_ptr<Shader> gizmoShader;
	std::unique_ptr<VAO> gizmoVAO;
	std::unique_ptr<VBO> gizmoVBO, gizmoEBO;
	size_t gizmoCircleIndexCount; // Circle rendering indices
	size_t gizmoCenterAxesBaseVertex; // Starting vertex for center axes
	size_t gizmoCenterAxesVertexCount; // Number of center axes vertices

	// Picker FBO (ID-only rendering)
	std::unique_ptr<Shader> pickerMeshShader;
	std::unique_ptr<Shader> pickerPointsShader;
	GLuint pickFBO;
	GLuint pickIDTex; // GL_R32UI texture storing primitive ids
	GLuint pickDepthRBO; // depth renderbuffer
	cv::Size pickFBOSize;

public:
	Renderer();
	~Renderer();

	bool Initialize();
	void Release();
	void Reset();

	// Data upload
	void UploadLayers(const Scene& sceneController, const Window& window);
	void UploadPointClouds(const Scene& sceneController, float normalLength);
	void UploadCameras(const Window& window);
	void UploadUncertaintyEllipsoids(const Window& window);
	void UploadSelection(const Window& window);
	void UploadBounds(const MVS::Scene& scene);

	// Rendering
	void BeginFrame(const Camera& camera, const Eigen::Vector4f& clearColor);
	// Re-prime the cached view-projection matrices; called by the compare view to
	// render each side with its own camera (BeginFrame primes the main camera)
	void UpdateViewProjection(const Camera& camera);
	void SetLighting(const Eigen::Vector3f& direction, float intensity, const Eigen::Vector3f& color);

	void RenderPointCloud(const Window& window);
	void RenderPointCloudNormals(const Window& window);
	void RenderMesh(const Window& window);
	void RenderCameras(const Window& window);
	void RenderUncertaintyEllipsoids(const Window& window);
	void RenderImageOverlays(const Window& window);
	void RenderSelection(const Window& window);
	void RenderSelectionOverlay(const Window& window);
	void RenderSelectedGeometry(const Window& window);
	void RenderBounds();
	// Render the interactive bounding-box edit gizmos: 8 corner point handles,
	// 6 face center handles and 3 rotation rings around the OBB center.
	// hoverAxisIdx < 0 means no rotation ring is hovered; hoverCornerIdx / hoverFaceIdx < 0 means no corner/face hover.
	void RenderBoundingBoxGizmos(const OBB3f& obb,
	                             int hoverCornerIdx = -1,
	                             int hoverFaceIdx = -1,
	                             int hoverAxisIdx = -1);
	void RenderCoordinateAxes(const Camera& camera);
	void RenderArcballGizmos(const Camera& camera, const class ArcballControls& controls);

	struct PickResult {
		uint32_t index{NO_ID};
		uint32_t layerID{NO_ID};
		Point3f points[3];
		bool isPoint;
		bool IsValid() const { return index != NO_ID && layerID != NO_ID; }
	};
	PickResult PickPrimitiveAt(const Point2f& screenPos, int radius, const Window& window);

	void EndFrame();

	// Getters
	size_t GetMeshSubMeshCount() const { return meshFaceCounts.size(); }
	uint32_t GetMeshSubMeshLayerID(size_t submeshIdx) const
	{
		return submeshIdx < meshSubMeshLayerIDs.size() ? meshSubMeshLayerIDs[submeshIdx] : NO_ID;
	}

	// Compare split view: restrict the scene render passes to a subset of layers (empty = all)
	void SetLayerPassFilter(std::vector<uint32_t> layerIDs) { layerPassFilter = std::move(layerIDs); }
	void ClearLayerPassFilter() { layerPassFilter.clear(); }
	bool IsLayerInPass(uint32_t layerID) const
	{
		return layerPassFilter.empty() ||
			std::find(layerPassFilter.begin(), layerPassFilter.end(), layerID) != layerPassFilter.end();
	}

private:
	void CreateShaders();
	void CreateBuffers();
	void UpdateLighting();
	void UploadMeshes(const Scene& sceneController);

	// Utility methods
	void SetupPointCloudBuffers();
	void SetupPointCloudNormalsBuffers();
	void SetupMeshBuffers();
	void SetupCameraBuffers();
	void SetupEllipsoidBuffers();
	void SetupImageOverlayBuffers();
	void SetupSelectionBuffers();
	void SetupSelectionOverlayBuffers();
	void SetupBoundsBuffers();
	void SetupBBoxHandleBuffers();
	void SetupAxesBuffers();
	void SetupGizmoBuffers();

	// Reusable unit-circle line-list builder. Generates XY-plane vertices for a
	// closed line-loop of 'numSegments' segments with given radius, and matching
	// line-pair indices. Used by gizmo circles (arcball + OBB rotation rings).
	// Vertices are appended as {x,y,z} triples. Indices are offset by baseIndex
	// so callers can append multiple shapes into the same buffer.
	static void BuildCircleLineSegments(int numSegments, float radius,
	                                    std::vector<float>& vertices,
	                                    std::vector<uint32_t>& indices,
	                                    uint32_t baseIndex = 0);

	// Ensure pick FBO matches requested size (creates or recreates textures/renderbuffers)
	void EnsurePickFBOSize(int width, int height);
	// Release picker buffers (textures, renderbuffers, FBO)
	void ReleasePickerBuffers();

	const LayerIndexRange* FindPointLayerRange(uint32_t layerID) const;
	const CameraLayerRange* FindCameraLayerRange(uint32_t layerID) const;
	const LayerFaceMap* FindMeshLayerMap(uint32_t layerID) const;
	bool MapGlobalPoint(size_t globalIndex, uint32_t& layerID, uint32_t& localIndex) const;
	bool MapGlobalFace(size_t globalIndex, uint32_t& layerID, uint32_t& localIndex) const;
	bool MapLocalFace(uint32_t layerID, uint32_t localIndex, uint32_t& globalIndex) const;
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
