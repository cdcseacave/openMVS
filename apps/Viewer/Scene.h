/*
 * Scene.h
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

#ifndef _VIEWER_SCENE_H_
#define _VIEWER_SCENE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Window.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace VIEWER {

class Scene
{
public:
	typedef MVS::PointCloud::Octree OctreePoints;
	typedef MVS::Mesh::Octree OctreeMesh;

public:
	ARCHIVE_TYPE nArchiveType;
	String name;

	String sceneName;
	String geometryName;
	bool geometryMesh;
	MVS::Scene scene;
	Window window;
	ImageArr images; // scene photos
	ImageArr textures; // mesh textures

	OctreePoints octPoints;
	OctreeMesh octMesh;
	Point3fArr obbPoints;

	GLuint listPointCloud;
	CLISTDEF0IDX(GLuint,MVS::Mesh::TexIndex) listMeshes;

	// multi-threading
	static SEACAVE::EventQueue events; // internal events queue (processed by the working threads)
	static SEACAVE::Thread thread; // worker thread

public:
	explicit Scene(ARCHIVE_TYPE _nArchiveType = ARCHIVE_MVS);
	~Scene();

	void Empty();
	void Release();
	void ReleasePointCloud();
	void ReleaseMesh();
	inline bool IsValid() const { return window.IsValid(); }
	inline bool IsOpen() const { return IsValid() && !scene.IsEmpty(); }
	inline bool IsOctreeValid() const { return !octPoints.IsEmpty() || !octMesh.IsEmpty(); }

	bool Init(const cv::Size&, LPCTSTR windowName, LPCTSTR fileName=NULL, LPCTSTR geometryFileName=NULL);
	bool Open(LPCTSTR fileName, LPCTSTR geometryFileName=NULL);
	bool Save(LPCTSTR fileName=NULL, bool bRescaleImages=false);
	bool Export(LPCTSTR fileName, LPCTSTR exportType=NULL) const;
	void CompilePointCloud();
	void CompileMesh();
	void CompileBounds();
	void CropToBounds();

	void Draw();
	void Loop();

	void Center();
	void TogleSceneBox();
	void CastRay(const Ray3&, int);
protected:
	static void* ThreadWorker(void*);
};
/*----------------------------------------------------------------*/

} // namespace VIEWER

#endif // _VIEWER_SCENE_H_
