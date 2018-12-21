/*
 * Window.cpp
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
#include "Window.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Window::WindowsMap Window::g_mapWindows;

Window::Window()
	:
	window(NULL),
	pos(Eigen::Vector2d::Zero()),
	prevPos(Eigen::Vector2d::Zero())
{
}
Window::~Window()
{
	Release();
}

void Window::Release()
{
	if (IsValid()) {
		glfwDestroyWindow(window);
		window = NULL;
	}
	clbkOpenScene.reset();
	clbkExportScene.reset();
	clbkRayScene.reset();
	clbkCompilePointCloud.reset();
	clbkCompileMesh.reset();
}

bool Window::Init(int width, int height, LPCTSTR name)
{
	glfwDefaultWindowHints();
	glfwWindowHint(GLFW_VISIBLE, 0);
	window = glfwCreateWindow(width, height, name, NULL, NULL);
	if (!window)
		return false;
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, Window::Resize);
	glfwSetKeyCallback(window, Window::Key);
	glfwSetMouseButtonCallback(window, Window::MouseButton);
	glfwSetScrollCallback(window, Window::Scroll);
	glfwSetDropCallback(window, Window::Drop);
	g_mapWindows[window] = this;
	Reset();
	return true;
}
void Window::SetCamera(CameraPtr cam)
{
	camera = cam;
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	Resize(width, height);
}
void Window::SetName(LPCTSTR name)
{
	glfwSetWindowTitle(window, name);
}
void Window::SetVisible(bool v)
{
	if (v)
		glfwShowWindow(window);
	else
		glfwHideWindow(window);
}
void Window::Reset()
{
	if (camera)
		camera->Reset();
	sparseType = SPR_ALL;
	minViews = 2;
	pointSize = 2.f;
	cameraBlend = 0.5f;
	bRenderCameras = true;
	bRenderSolid = true;
	bRenderTexture = true;
	selectionType = SEL_NA;
	if (clbkCompilePointCloud != NULL)
		clbkCompilePointCloud();
	if (clbkCompileMesh != NULL)
		clbkCompileMesh();
	glfwPostEmptyEvent();
}

void Window::UpdateView(const ImageArr& images, const MVS::ImageArr& sceneImages)
{
	glMatrixMode(GL_MODELVIEW);
	if (camera->prevCamID != camera->currentCamID && camera->currentCamID != NO_ID) {
		// enable camera view mode
		// apply current camera transform
		const Image& image = images[camera->currentCamID];
		const MVS::Image& imageData = sceneImages[image.idx];
		const MVS::Camera& camera = imageData.camera;
		const Eigen::Matrix4d trans(TransW2L((const Matrix3x3::EMat)camera.R, camera.GetT()));
		glLoadMatrixf((GLfloat*)gs_convert);
		glMultMatrixd((GLdouble*)trans.data());
	} else {
		// apply view point transform
		const Eigen::Matrix4d trans(camera->GetLookAt());
		glLoadMatrixd((GLdouble*)trans.data());
	}
}

void Window::UpdateMousePosition()
{
	prevPos = pos;
	// get current position
	glfwGetCursorPos(window, &pos.x(), &pos.y());
	// normalize position to [-1:1] range
	const int w(camera->width);
	const int h(camera->height);
	pos.x() = (2.0 * pos.x() - w) / w;
	pos.y() = (h - 2.0 * pos.y()) / h;
}

void Window::Resize(int width, int height)
{
	glfwMakeContextCurrent(window);
	glViewport(0, 0, (GLint)width, (GLint)height);
	camera->Resize(width, height);
}
void Window::Resize(GLFWwindow* window, int width, int height)
{
	g_mapWindows[window]->Resize(width, height);
}

void Window::Key(int k, int /*scancode*/, int action, int mod)
{
	switch (k) {
	case GLFW_KEY_ESCAPE:
		if (action == GLFW_RELEASE)
			glfwSetWindowShouldClose(window, 1);
		break;
	case GLFW_KEY_DOWN:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_SHIFT) {
				if (minViews > 2) {
					minViews--;
					if (clbkCompilePointCloud != NULL)
						clbkCompilePointCloud();
				}
			} else {
				pointSize = MAXF(pointSize-0.5f, 0.5f);
			}
		}
		break;
	case GLFW_KEY_UP:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_SHIFT) {
				minViews++;
				if (clbkCompilePointCloud != NULL)
					clbkCompilePointCloud();
			} else {
				pointSize += 0.5f;
			}
		}
		break;
	case GLFW_KEY_LEFT:
		if (action != GLFW_RELEASE) {
			camera->prevCamID = camera->currentCamID;
			camera->currentCamID--;
			if (camera->currentCamID < NO_ID && camera->currentCamID >= camera->maxCamID)
				camera->currentCamID = camera->maxCamID-1;
		}
		break;
	case GLFW_KEY_RIGHT:
		if (action != GLFW_RELEASE) {
			camera->prevCamID = camera->currentCamID;
			camera->currentCamID++;
			if (camera->currentCamID >= camera->maxCamID)
				camera->currentCamID = NO_ID;
		}
		break;
	case GLFW_KEY_E:
		if (action == GLFW_RELEASE && clbkExportScene != NULL)
			clbkExportScene(NULL, NULL, false);
		break;
	case GLFW_KEY_R:
		if (action == GLFW_RELEASE)
			Reset();
		break;
	case GLFW_KEY_C:
		if (action == GLFW_RELEASE)
			bRenderCameras = !bRenderCameras;
		break;
	case GLFW_KEY_W:
		if (action == GLFW_RELEASE) {
			if (bRenderSolid) {
				bRenderSolid = false;
				glPolygonMode(GL_FRONT, GL_LINE);
			} else {
				bRenderSolid = true;
				glPolygonMode(GL_FRONT, GL_FILL);
			}
		}
		break;
	case GLFW_KEY_T:
		if (action == GLFW_RELEASE) {
			bRenderTexture = !bRenderTexture;
			if (clbkCompileMesh != NULL)
				clbkCompileMesh();
		}
		break;
	case GLFW_KEY_P:
		switch (sparseType) {
		case SPR_POINTS: sparseType = SPR_LINES; break;
		case SPR_LINES: sparseType = SPR_ALL; break;
		case SPR_ALL: sparseType = SPR_POINTS; break;
		}
		if (clbkCompilePointCloud != NULL)
			clbkCompilePointCloud();
		break;
	case GLFW_KEY_KP_SUBTRACT:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_CONTROL)
				camera->SetFOV(MAXF(camera->fov-5, 5.0));
			else if (mod & GLFW_MOD_SHIFT)
				camera->scaleF *= 0.9f;
			else
				cameraBlend = MAXF(cameraBlend-0.1f, 0.f);
		}
		break;
	case GLFW_KEY_KP_ADD:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_CONTROL)
				camera->SetFOV(camera->fov+5);
			else if (mod & GLFW_MOD_SHIFT)
				camera->scaleF *= 1.11f;
			else
				cameraBlend += MAXF(cameraBlend-0.1f, 0.f);
		}
		break;
	}
}
void Window::Key(GLFWwindow* window, int k, int scancode, int action, int mod)
{
	g_mapWindows[window]->Key(k, scancode, action, mod);
}

void Window::MouseButton(int button, int action, int /*mods*/)
{
	if (clbkRayScene != NULL && button == GLFW_MOUSE_BUTTON_LEFT) {
		typedef Eigen::Matrix<double,4,4,Eigen::ColMajor> Mat4;
		Mat4 P, V;
		glGetDoublev(GL_MODELVIEW_MATRIX, V.data());
		glGetDoublev(GL_PROJECTION_MATRIX, P.data());
		// 4d Homogeneous Clip Coordinates
		const Eigen::Vector4d ray_clip(pos.x(), pos.y(), -1.0, 1.0);
		// 4d Eye (Camera) Coordinates
		Eigen::Vector4d ray_eye(P.inverse()*ray_clip);
		ray_eye.z() = -1.0;
		ray_eye.w() = 0.0;
		// 4d World Coordinates
		const Mat4 invV(V.inverse());
		ASSERT(ISEQUAL(invV(3,3),1.0));
		Eigen::Vector3d start(invV.topRightCorner<3,1>());
		const Eigen::Vector4d ray_wor(invV*ray_eye);
		const Eigen::Vector3d dir(ray_wor.topRows<3>().normalized());
		clbkRayScene(Ray3d(start, dir), action);
	}
}
void Window::MouseButton(GLFWwindow* window, int button, int action, int mods)
{
	g_mapWindows[window]->MouseButton(button, action, mods);
}

void Window::Scroll(double /*xoffset*/, double yoffset)
{
	camera->dist *= (yoffset>0 ? POW(1.11,yoffset) : POW(0.9,-yoffset));
}
void Window::Scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	g_mapWindows[window]->Scroll(xoffset, yoffset);
}

void Window::Drop(int count, const char** paths)
{
	if (clbkOpenScene && count > 0) {
		SetVisible(false);
		String fileName(paths[0]);
		Util::ensureUnifySlash(fileName);
		if (count > 1) {
			String meshFileName(paths[1]);
			Util::ensureUnifySlash(meshFileName);
			clbkOpenScene(fileName, meshFileName);
		} else {
			clbkOpenScene(fileName, NULL);
		}
		SetVisible(true);
	}
}
void Window::Drop(GLFWwindow* window, int count, const char** paths)
{
	g_mapWindows[window]->Drop(count, paths);
}

bool Window::IsShiftKeyPressed() const
{
	return
		glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}
bool Window::IsCtrlKeyPressed() const
{
	return
		glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}
bool Window::IsAltKeyPressed() const
{
	return
		glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
}
/*----------------------------------------------------------------*/
