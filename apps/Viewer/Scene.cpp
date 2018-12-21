/*
 * Scene.cpp
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

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////

#define IMAGE_MAX_RESOLUTION 1024


// S T R U C T S ///////////////////////////////////////////////////

template <typename OCTREE>
struct LeafAvgSize {
	typedef typename OCTREE::IDX_TYPE IDX;
	const IDX maxNumItems;
	double radius;
	size_t count;
	LeafAvgSize(IDX cellNumItems) : maxNumItems(cellNumItems/2), radius(0), count(0) {}
	void operator () (const typename OCTREE::CELL_TYPE& cell, typename OCTREE::Type r) {
		if (cell.GetNumItems() > maxNumItems) {
			radius += r;
			count++;
		}
	}
	operator float () const {
		return (float)(radius / (double)count);
	}
};

template <typename SCENE, typename OCTREE>
struct TIntersectRay {
	typedef SCENE Scene;
	typedef OCTREE Octree;
	typedef typename OCTREE::IDX_TYPE IDX;

	struct IndexDist {
		IDX idx;
		REAL dist;

		inline IndexDist() : dist(REAL(FLT_MAX)) {}
		inline bool IsValid() const { return dist < REAL(FLT_MAX); }
	};

	const Scene& scene;
	const Octree& octree;
	const Ray3& ray;
	IndexDist pick;

	TIntersectRay(const Scene& _scene, const Octree& _octree, const Ray3& _ray)
		:
		scene(_scene),
		octree(_octree),
		ray(_ray)
	{
	}

	inline bool Intersects(const typename Octree::POINT_TYPE& center, typename Octree::Type radius) const {
		return ray.Intersects(AABB3f(center, radius));
	}
};

struct IntersectRayPoints : public TIntersectRay<MVS::PointCloud,Scene::OctreePoints> {
	using typename TIntersectRay::Scene;
	using typename TIntersectRay::Octree;
	using typename TIntersectRay::IDX;

	const REAL pointSize;
	const unsigned minViews;

	IntersectRayPoints(const Scene& _scene, const Octree& _octree, REAL _pointSize, unsigned _minViews, const Ray3& _ray)
		: TIntersectRay(_scene, _octree, _ray), pointSize(_pointSize), minViews(_minViews)
	{
		octree.Collect(*this, *this);
	}

	void operator () (const IDX* idices, IDX size) {
		// test ray-point intersection and keep the closest
		FOREACHRAWPTR(pIdx, idices, size) {
			const MVS::PointCloud::Index idx(*pIdx);
			if (!scene.pointViews.IsEmpty() && scene.pointViews[idx].size() < minViews)
				continue;
			const MVS::PointCloud::Point& X = scene.points[idx];
			REAL dist;
			if (ray.Intersects(Sphere3(Cast<REAL>(X), pointSize), dist)) {
				ASSERT(dist >= 0);
				if (pick.dist > dist) {
					pick.dist = dist;
					pick.idx = idx;
				}
			}
		}
	}
};

struct IntersectRayMesh : public TIntersectRay<MVS::Mesh,Scene::OctreeMesh> {
	using typename TIntersectRay::Scene;
	using typename TIntersectRay::Octree;
	using typename TIntersectRay::IDX;

	IntersectRayMesh(const Scene& _scene, const Octree& _octree, const Ray3& _ray)
		: TIntersectRay(_scene, _octree, _ray)
	{
		octree.Collect(*this, *this);
	}

	void operator () (const IDX* idices, IDX size) {
		// store all intersected faces only once
		typedef std::unordered_set<MVS::Mesh::FIndex> FaceSet;
		FaceSet set;
		FOREACHRAWPTR(pIdx, idices, size) {
			const MVS::Mesh::VIndex idxVertex((MVS::Mesh::VIndex)*pIdx);
			const MVS::Mesh::FaceIdxArr& faces = scene.vertexFaces[idxVertex];
			set.insert(faces.begin(), faces.end());
		}
		// test face intersection and keep the closest
		for (MVS::Mesh::FIndex idxFace: set) {
			const MVS::Mesh::Face& face = scene.faces[idxFace];
			REAL dist;
			if (ray.Intersects<true>(Triangle3(Cast<REAL>(scene.vertices[face[0]]), Cast<REAL>(scene.vertices[face[1]]), Cast<REAL>(scene.vertices[face[2]])), &dist)) {
				ASSERT(dist >= 0);
				if (pick.dist > dist) {
					pick.dist = dist;
					pick.idx = idxFace;
				}
			}
		}
	}
};
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

enum EVENT_TYPE {
	EVT_JOB = 0,
	EVT_CLOSE,
};

class EVTClose : public Event
{
public:
	EVTClose() : Event(EVT_CLOSE) {}
};
class EVTLoadImage : public Event
{
public:
	Scene* pScene;
	MVS::IIndex idx;
	unsigned nMaxResolution;
	bool Run(void*) {
		Image& image = pScene->images[idx];
		ASSERT(image.idx != NO_ID);
		MVS::Image& imageData = pScene->scene.images[image.idx];
		ASSERT(imageData.IsValid());
		if (imageData.image.empty() && !imageData.ReloadImage(nMaxResolution))
			return false;
		imageData.UpdateCamera(pScene->scene.platforms);
		image.AssignImage(imageData.image);
		imageData.ReleaseImage();
		glfwPostEmptyEvent();
		return true;
	}
	EVTLoadImage(Scene* _pScene, MVS::IIndex _idx, unsigned _nMaxResolution=0)
		: Event(EVT_JOB), pScene(_pScene), idx(_idx), nMaxResolution(_nMaxResolution) {}
};
class EVTComputeOctree : public Event
{
public:
	Scene* pScene;
	bool Run(void*) {
		MVS::Scene& scene = pScene->scene;
		if (!scene.mesh.IsEmpty()) {
			Scene::OctreeMesh octMesh(scene.mesh.vertices);
			LeafAvgSize<Scene::OctreeMesh> size(128);
			octMesh.ParseCells(size);
			scene.mesh.ListIncidenteFaces();
			pScene->octMesh.Swap(octMesh);
			pScene->avgScale = size;
		} else
		if (!scene.pointcloud.IsEmpty()) {
			Scene::OctreePoints octPoints(scene.pointcloud.points);
			LeafAvgSize<Scene::OctreePoints> size(256);
			octPoints.ParseCells(size);
			pScene->octPoints.Swap(octPoints);
			pScene->avgScale = size;
		}
		return true;
	}
	EVTComputeOctree(Scene* _pScene)
		: Event(EVT_JOB), pScene(_pScene) {}
};

void* Scene::ThreadWorker(void*) {
	while (true) {
		CAutoPtr<Event> evt(events.GetEvent());
		switch (evt->GetID()) {
		case EVT_JOB:
			evt->Run();
			break;
		case EVT_CLOSE:
			return NULL;
		default:
			ASSERT("Should not happen!" == NULL);
		}
	}
	return NULL;
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

SEACAVE::EventQueue Scene::events;
SEACAVE::Thread Scene::thread;

Scene::Scene()
	:
	listPointCloud(0),
	listMesh(0)
{
}
Scene::~Scene()
{
	Release();
}

void Scene::Empty()
{
	ReleasePointCloud();
	ReleaseMesh();
	textures.Release();
	images.Release();
	scene.Release();
	sceneName.clear();
	avgScale = 0;
}
void Scene::Release()
{
	if (!thread.isRunning()) {
		events.AddEvent(new EVTClose());
		thread.join();
	}
	Empty();
	window.Release();
	glfwTerminate();
}
void Scene::ReleasePointCloud()
{
	if (listPointCloud) {
		glDeleteLists(listPointCloud, 1);
		listPointCloud = 0;
	}
}
void Scene::ReleaseMesh()
{
	if (listMesh) {
		glDeleteLists(listMesh, 1);
		listMesh = 0;
	}
}

bool Scene::Init(int width, int height, LPCTSTR windowName, LPCTSTR fileName, LPCTSTR meshFileName)
{
	ASSERT(scene.IsEmpty());

	// init window
	if (glfwInit() == GL_FALSE)
		return false;
	if (!window.Init(width, height, windowName))
		return false;
	if (glewInit() != GLEW_OK)
		return false;
	name = windowName;
	window.clbkOpenScene = DELEGATEBINDCLASS(Window::ClbkOpenScene, &Scene::Open, this);
	window.clbkExportScene = DELEGATEBINDCLASS(Window::ClbkExportScene, &Scene::Export, this);
	window.clbkRayScene = DELEGATEBINDCLASS(Window::ClbkRayScene, &Scene::CastRay, this);
	window.clbkCompilePointCloud = DELEGATEBINDCLASS(Window::ClbkCompilePointCloud, &Scene::CompilePointCloud, this);
	window.clbkCompileMesh = DELEGATEBINDCLASS(Window::ClbkCompileMesh, &Scene::CompileMesh, this);

	// init OpenGL
	glPolygonMode(GL_FRONT, GL_FILL);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.f, 0.5f, 0.9f, 1.f);

	static const float light0_ambient[] = {0.1f, 0.1f, 0.1f, 1.0f};
	static const float light0_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	static const float light0_position[] = {0.0f, 0.0f, 1000.0f, 0.0f};
	static const float light0_specular[] = {0.4f, 0.4f, 0.4f, 1.0f};

	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// init working thread
	thread.start(ThreadWorker);

	// open scene or init empty scene
	if (fileName == NULL || !Open(fileName, meshFileName))
		window.SetCamera(CameraPtr(new Camera()));

	window.SetVisible(true);
	return true;
}
bool Scene::Open(LPCTSTR fileName, LPCTSTR meshFileName)
{
	ASSERT(fileName);
	DEBUG_EXTRA("Loading: '%s'", Util::getFileNameExt(fileName).c_str());
	Empty();
	sceneName = fileName;

	// load the scene
	WORKING_FOLDER = Util::getFilePath(fileName);
	INIT_WORKING_FOLDER;
	if (!scene.Load(fileName, true))
		return false;
	if (meshFileName) {
		// load given mesh
		scene.mesh.Load(meshFileName);
	}
	if (scene.IsEmpty())
		return false;

	#if 1
	// create octree structure used to accelerate selection functionality
	events.AddEvent(new EVTComputeOctree(this));
	#endif

	// init scene
	AABB3d bounds(true);
	if (!scene.pointcloud.IsEmpty()) {
		bounds = scene.pointcloud.GetAABB(3);
		if (bounds.IsEmpty())
			bounds = scene.pointcloud.GetAABB();
	}
	if (!scene.mesh.IsEmpty()) {
		scene.mesh.ComputeNormalFaces();
		bounds.Insert(scene.mesh.GetAABB());
	}

	// init images
	images.Reserve(scene.images.size());
	FOREACH(idxImage, scene.images) {
		const MVS::Image& imageData = scene.images[idxImage];
		if (!imageData.IsValid())
			continue;
		images.AddConstruct(idxImage);
	}

	// init and load texture
	if (scene.mesh.HasTexture()) {
		Image& image = textures.AddEmpty();
		ASSERT(image.idx == NO_ID);
		#if 0
		cv::flip(scene.mesh.textureDiffuse, scene.mesh.textureDiffuse, 0);
		image.SetImage(scene.mesh.textureDiffuse);
		scene.mesh.textureDiffuse.release();
		#else // preserve texture, used only to be able to export the mesh
		Image8U3 textureDiffuse;
		cv::flip(scene.mesh.textureDiffuse, textureDiffuse, 0);
		image.SetImage(textureDiffuse);
		#endif
		image.GenerateMipmap();
	}

	// init display lists
	// compile point-cloud
	CompilePointCloud();
	// compile mesh
	CompileMesh();

	// init camera
	window.SetCamera(CameraPtr(new Camera(bounds)));
	window.camera->maxCamID = images.size();
	window.SetName(String::FormatString((name + _T(": %s")).c_str(), Util::getFileName(fileName).c_str()));
	window.Reset();
	return true;
}

// export the scene
bool Scene::Export(LPCTSTR _fileName, LPCTSTR exportType, bool losslessTexture) const
{
	if (!IsOpen())
		return false;
	ASSERT(!sceneName.IsEmpty());
	String lastFileName;
	const String fileName(_fileName != NULL ? String(_fileName) : sceneName);
	const String baseFileName(Util::getFileFullName(fileName));
	const bool bPoints(scene.pointcloud.Save(lastFileName=(baseFileName+_T("_pointcloud.ply"))));
	const bool bMesh(scene.mesh.Save(lastFileName=(baseFileName+_T("_mesh")+(exportType?exportType:(Util::getFileExt(fileName)==_T(".obj")?_T(".obj"):_T(".ply")))), true, losslessTexture));
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2 && (bPoints || bMesh))
		scene.ExportCamerasMLP(Util::getFileFullName(lastFileName)+_T(".mlp"), lastFileName);
	#endif
	return (bPoints || bMesh);
}

void Scene::CompilePointCloud()
{
	if (scene.pointcloud.IsEmpty())
		return;
	ReleasePointCloud();
	listPointCloud = glGenLists(1);
	glNewList(listPointCloud, GL_COMPILE);
	ASSERT((window.sparseType&(Window::SPR_POINTS|Window::SPR_LINES)) != 0);
	// compile point-cloud
	if (!scene.pointcloud.IsEmpty() && (window.sparseType&Window::SPR_POINTS) != 0) {
		ASSERT_ARE_SAME_TYPE(float, MVS::PointCloud::Point::Type);
		glBegin(GL_POINTS);
		glColor3f(1.f,1.f,1.f);
		FOREACH(i, scene.pointcloud.points) {
			if (!scene.pointcloud.pointViews.IsEmpty() &&
				scene.pointcloud.pointViews[i].size() < window.minViews)
				continue;
			if (!scene.pointcloud.colors.IsEmpty()) {
				const MVS::PointCloud::Color& c = scene.pointcloud.colors[i];
				glColor3ub(c.r,c.g,c.b);
			}
			const MVS::PointCloud::Point& X = scene.pointcloud.points[i];
			glVertex3fv(X.ptr());
		}
		glEnd();
	}
	glEndList();
}

void Scene::CompileMesh()
{
	if (scene.mesh.IsEmpty())
		return;
	ReleaseMesh();
	listMesh = glGenLists(1);
	glNewList(listMesh, GL_COMPILE);
	// compile mesh
	ASSERT_ARE_SAME_TYPE(float, MVS::Mesh::Vertex::Type);
	ASSERT_ARE_SAME_TYPE(float, MVS::Mesh::Normal::Type);
	ASSERT_ARE_SAME_TYPE(float, MVS::Mesh::TexCoord::Type);
	glColor3f(1.f, 1.f, 1.f);
	glBegin(GL_TRIANGLES);
	FOREACH(i, scene.mesh.faces) {
		const MVS::Mesh::Face& face = scene.mesh.faces[i];
		const MVS::Mesh::Normal& n = scene.mesh.faceNormals[i];
		glNormal3fv(n.ptr());
		for (int j = 0; j < 3; ++j) {
			if (!scene.mesh.faceTexcoords.IsEmpty() && window.bRenderTexture) {
				const MVS::Mesh::TexCoord& t = scene.mesh.faceTexcoords[i * 3 + j];
				glTexCoord2fv(t.ptr());
			}
			const MVS::Mesh::Vertex& p = scene.mesh.vertices[face[j]];
			glVertex3fv(p.ptr());
		}
	}
	glEnd();
	glEndList();
}

void Scene::Draw()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(window.pointSize);

	window.UpdateView(images, scene.images);

	// render point-cloud
	if (listPointCloud) {
		glDisable(GL_TEXTURE_2D);
		glCallList(listPointCloud);
	}
	// render mesh
	if (listMesh) {
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		if (!scene.mesh.faceTexcoords.IsEmpty() && window.bRenderTexture) {
			glEnable(GL_TEXTURE_2D);
			textures.First().Bind();
			glCallList(listMesh);
			glDisable(GL_TEXTURE_2D);
		} else {
			glEnable(GL_LIGHTING);
			glCallList(listMesh);
			glDisable(GL_LIGHTING);
		}
	}
	// render cameras
	if (window.bRenderCameras) {
		glDisable(GL_CULL_FACE);
		FOREACH(idx, images) {
			Image& image = images[idx];
			const MVS::Image& imageData = scene.images[image.idx];
			const MVS::Camera& camera = imageData.camera;
			// change coordinates system to the camera space
			glPushMatrix();
			glMultMatrixd((GLdouble*)TransL2W((const Matrix3x3::EMat)camera.R, -(const Point3::EVec)camera.C).data());
			glPointSize(window.pointSize+1.f);
			glDisable(GL_TEXTURE_2D);
			// draw camera position and image center
			const double scaleFocal(window.camera->scaleF);
			glBegin(GL_POINTS);
			glColor3f(1,0,0); glVertex3f(0,0,0); // camera position
			glColor3f(0,1,0); glVertex3f(0,0,(float)scaleFocal); // image center
			glEnd();
			// cache image corner coordinates
			const Point2d pp(camera.GetPrincipalPoint());
			const double focal(camera.GetFocalLength()/scaleFocal);
			const double cx(-pp.x/focal);
			const double cy(-pp.y/focal);
			const double px((double)imageData.width/focal+cx);
			const double py((double)imageData.height/focal+cy);
			const Point3d ic1(cx, cy, scaleFocal);
			const Point3d ic2(cx, py, scaleFocal);
			const Point3d ic3(px, py, scaleFocal);
			const Point3d ic4(px, cy, scaleFocal);
			// draw image thumbnail
			const bool bSelectedImage(idx == window.camera->currentCamID);
			if (bSelectedImage) {
				if (image.IsValid()) {
					// render image
					glEnable(GL_TEXTURE_2D);
					image.Bind();
					glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
					glEnable(GL_BLEND);
					glDisable(GL_DEPTH_TEST);
					glColor4f(1,1,1,window.cameraBlend);
					glBegin(GL_QUADS);
					glTexCoord2d(0,0); glVertex3dv(ic1.ptr());
					glTexCoord2d(0,1); glVertex3dv(ic2.ptr());
					glTexCoord2d(1,1); glVertex3dv(ic3.ptr());
					glTexCoord2d(1,0); glVertex3dv(ic4.ptr());
					glEnd();
					glDisable(GL_TEXTURE_2D);
					glDisable(GL_BLEND);
					glEnable(GL_DEPTH_TEST);
				} else {
					// start and wait to load the image
					if (image.IsImageEmpty()) {
						// start loading
						image.SetImageLoading();
						events.AddEvent(new EVTLoadImage(this, idx, IMAGE_MAX_RESOLUTION));
					} else {
						// check if the image is available and set it
						image.TransferImage();
					}
				}
			}
			// draw camera frame
			glColor3f(bSelectedImage ? 0.f : 1.f, 1.f, 0.f);
			glBegin(GL_LINES);
			glVertex3d(0,0,0); glVertex3dv(ic1.ptr());
			glVertex3d(0,0,0); glVertex3dv(ic2.ptr());
			glVertex3d(0,0,0); glVertex3dv(ic3.ptr());
			glVertex3d(0,0,0); glVertex3dv(ic4.ptr());
			glVertex3dv(ic1.ptr()); glVertex3dv(ic2.ptr());
			glVertex3dv(ic2.ptr()); glVertex3dv(ic3.ptr());
			glVertex3dv(ic3.ptr()); glVertex3dv(ic4.ptr());
			glVertex3dv(ic4.ptr()); glVertex3dv(ic1.ptr());
			glEnd();
			// restore coordinate system
			glPopMatrix();
		}
	}
	if (window.selectionType != Window::SEL_NA) {
		glPointSize(window.pointSize+4);
		glDisable(GL_DEPTH_TEST);
		glBegin(GL_POINTS);
		glColor3f(1,0,0); glVertex3fv(window.selectionPoints[0].ptr());
		if (window.selectionType == Window::SEL_TRIANGLE) {
		glColor3f(0,1,0); glVertex3fv(window.selectionPoints[1].ptr());
		glColor3f(0,0,1); glVertex3fv(window.selectionPoints[2].ptr());
		}
		glEnd();
		glEnable(GL_DEPTH_TEST);
		glPointSize(window.pointSize);
	}
	glfwSwapBuffers(window.GetWindow());
}

void Scene::ProcessEvents()
{
	glfwWaitEvents();
	window.UpdateMousePosition();
	if (glfwGetMouseButton(window.GetWindow(), GLFW_MOUSE_BUTTON_1) != GLFW_RELEASE)
		window.camera->Rotate(window.pos, window.prevPos);
}

void Scene::Loop()
{
	while (!glfwWindowShouldClose(window.GetWindow())) {
		ProcessEvents();
		Draw();
	}
}


void Scene::CastRay(const Ray3& ray, int action)
{
	if (!IsOctreeValid())
		return;
	const double timeClick(0.2);
	const double timeDblClick(0.3);
	const double now(glfwGetTime());
	switch (action) {
	case GLFW_PRESS: {
		// remember when the click action started
		window.selectionTimeClick = now;
	break; }
	case GLFW_RELEASE: {
	if (now-window.selectionTimeClick > timeClick) {
		// this is a long click, ignore it
		break;
	} else
	if (window.selectionType != Window::SEL_NA &&
		now-window.selectionTime < timeDblClick) {
		// this is a double click, center scene at the selected point
		window.camera->center = Point3d(window.selectionPoints[3]);
		window.camera->dist *= 0.7;
		window.selectionTime = now;
	} else
	if (!octMesh.IsEmpty()) {
		// find ray intersection with the mesh
		const IntersectRayMesh intRay(scene.mesh, octMesh, ray);
		if (intRay.pick.IsValid()) {
			const MVS::Mesh::Face& face = scene.mesh.faces[(MVS::Mesh::FIndex)intRay.pick.idx];
			window.selectionPoints[0] = scene.mesh.vertices[face[0]];
			window.selectionPoints[1] = scene.mesh.vertices[face[1]];
			window.selectionPoints[2] = scene.mesh.vertices[face[2]];
			window.selectionPoints[3] = (ray.m_pOrig + ray.m_vDir*intRay.pick.dist).cast<float>();
			window.selectionType = Window::SEL_TRIANGLE;
			window.selectionTime = now;
			DEBUG("Face selected:\n\tindex: %u\n\tvertex 1: %u (%g %g %g)\n\tvertex 2: %u (%g %g %g)\n\tvertex 3: %u (%g %g %g)",
				intRay.pick.idx,
				face[0], window.selectionPoints[0].x, window.selectionPoints[0].y, window.selectionPoints[0].z,
				face[1], window.selectionPoints[1].x, window.selectionPoints[1].y, window.selectionPoints[1].z,
				face[2], window.selectionPoints[2].x, window.selectionPoints[2].y, window.selectionPoints[2].z
			);
		} else {
			window.selectionType = Window::SEL_NA;
		}
	} else
	if (!octPoints.IsEmpty()) {
		// find ray intersection with the points
		const IntersectRayPoints intRay(scene.pointcloud, octPoints, avgScale*0.1f, window.minViews, ray);
		if (intRay.pick.IsValid()) {
			window.selectionPoints[0] = window.selectionPoints[3] = scene.pointcloud.points[intRay.pick.idx];
			window.selectionType = Window::SEL_POINT;
			window.selectionTime = now;
			DEBUG("Point selected:\n\tindex: %u (%g %g %g)%s",
				intRay.pick.idx,
				window.selectionPoints[0].x, window.selectionPoints[0].y, window.selectionPoints[0].z,
				[&]() {
					if (scene.pointcloud.pointViews.empty())
						return String();
					const MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[intRay.pick.idx];
					ASSERT(!views.empty());
					String strViews(String::FormatString("\n\tviews: %u", views.size()));
					for (MVS::PointCloud::View idxImage: views) {
						const MVS::Image& imageData = scene.images[idxImage];
						const Point2 x(imageData.camera.TransformPointW2I(Cast<REAL>(window.selectionPoints[0])));
						strViews += String::FormatString("\n\t\t%s (%.2f %.2f)", Util::getFileNameExt(imageData.name).c_str(), x.x, x.y);
					}
					return strViews;
				}().c_str()
			);
		} else {
			window.selectionType = Window::SEL_POINT;
		}
	}
	break; }
	}
}
/*----------------------------------------------------------------*/
