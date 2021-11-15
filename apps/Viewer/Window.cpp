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
		#ifdef _USE_NUKLEAR
		nk_glfw3_shutdown();
		#endif
		glfwDestroyWindow(window);
		window = NULL;
	}
	clbkOpenScene.reset();
	ReleaseClbk();
}

void Window::ReleaseClbk()
{
	clbkExportScene.reset();
	clbkRayScene.reset();
	clbkCompilePointCloud.reset();
	clbkCompileMesh.reset();
	clbkCompileBounds.reset();
}

bool Window::Init(const cv::Size& _size, LPCTSTR name)
{
	sizeScale = 1;
	size = _size;

	glfwDefaultWindowHints();
	glfwWindowHint(GLFW_VISIBLE, 0);
	window = glfwCreateWindow(size.width, size.height, name, NULL, NULL);
	if (!window)
		return false;
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, Window::Resize);
	glfwSetKeyCallback(window, Window::Key);
	glfwSetMouseButtonCallback(window, Window::MouseButton);
	glfwSetCursorPosCallback(window, Window::MouseMove);
	glfwSetScrollCallback(window, Window::Scroll);
	glfwSetDropCallback(window, Window::Drop);
	g_mapWindows[window] = this;

	Reset();
	return true;
}
void Window::SetCamera(const Camera& cam)
{
	camera = cam;
	cv::Size _size;
	glfwGetFramebufferSize(window, &_size.width, &_size.height);
	Resize(_size);
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
bool Window::IsVisible() const
{
	return glfwGetWindowAttrib(window, GLFW_VISIBLE) != 0;
}
void Window::Reset(SPARSE _sparseType, unsigned _minViews)
{
	camera.Reset();
	inputType = INP_NA;
	sparseType = _sparseType;
	minViews = _minViews;
	pointSize = 2.f;
	cameraBlend = 0.5f;
	bRenderCameras = true;
	bRenderCameraTrajectory = true;
	bRenderImageVisibility = false;
	bRenderViews = true;
	bRenderSolid = true;
	bRenderTexture = true;
	bRenderBounds = false;
	selectionType = SEL_NA;
	selectionIdx = NO_IDX;
	if (clbkCompilePointCloud != NULL)
		clbkCompilePointCloud();
	if (clbkCompileMesh != NULL)
		clbkCompileMesh();
	glfwPostEmptyEvent();
}


void Window::CenterCamera(const Point3& pos)
{
	camera.center = pos;
	camera.dist *= 0.7;
}


void Window::UpdateView(const ImageArr& images, const MVS::ImageArr& sceneImagesMVS)
{
	if (camera.IsCameraViewMode()) {
		// enable camera view mode and apply current camera transform
		const Image& image = images[camera.currentCamID];
		const MVS::Camera& camera = sceneImagesMVS[image.idx].camera;
		UpdateView((const Matrix3x3::EMat)camera.R, camera.GetT());
	} else {
		// apply view point transform
		glMatrixMode(GL_MODELVIEW);
		const Eigen::Matrix4d trans(camera.GetLookAt());
		glLoadMatrixd((GLdouble*)trans.data());
	}
}

void Window::UpdateView(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
	glMatrixMode(GL_MODELVIEW);
	transform = gs_convert * TransW2L(R, t);
	glLoadMatrixd((GLdouble*)transform.data());
}

void Window::UpdateMousePosition(double xpos, double ypos)
{
	prevPos = pos;
	pos.x() = xpos;
	pos.y() = ypos;
	// normalize position to [-1:1] range
	const int w(camera.size.width);
	const int h(camera.size.height);
	pos.x() = (2.0 * pos.x() - w) / w;
	pos.y() = (h - 2.0 * pos.y()) / h;
}


void Window::GetFrame(Image8U3& image) const
{
	image.create(GetSize());
	glReadPixels(0, 0, image.width(), image.height(), GL_BGR_EXT, GL_UNSIGNED_BYTE, image.ptr());
	cv::flip(image, image, 0);
}


cv::Size Window::GetSize() const
{
	cv::Size _size;
	glfwGetWindowSize(window, &_size.width, &_size.height);
	return _size;
}
void Window::Resize(const cv::Size& _size)
{
	// detect scaled window
	sizeScale = (double)GetSize().width/_size.width;
	size = _size;
	// update resolution
	glfwMakeContextCurrent(window);
	glViewport(0, 0, size.width, size.height);
	camera.Resize(cv::Size(ROUND2INT(size.width*sizeScale), ROUND2INT(size.height*sizeScale)));
}
void Window::Resize(GLFWwindow* window, int width, int height)
{
	g_mapWindows[window]->Resize(cv::Size(width, height));
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
			camera.prevCamID = camera.currentCamID;
			camera.currentCamID--;
			if (camera.currentCamID < NO_ID && camera.currentCamID >= camera.maxCamID)
				camera.currentCamID = camera.maxCamID-1;
		}
		break;
	case GLFW_KEY_RIGHT:
		if (action != GLFW_RELEASE) {
			camera.prevCamID = camera.currentCamID;
			camera.currentCamID++;
			if (camera.currentCamID >= camera.maxCamID)
				camera.currentCamID = NO_ID;
		}
		break;
	case GLFW_KEY_B:
		if (action == GLFW_RELEASE)
			if (clbkCompileBounds != NULL)
				clbkCompileBounds();
		break;
	case GLFW_KEY_C:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_SHIFT) {
				bRenderCameraTrajectory = !bRenderCameraTrajectory;
			} else {
				bRenderCameras = !bRenderCameras;
			}
		}
		break;
	case GLFW_KEY_E:
		if (action == GLFW_RELEASE && clbkExportScene != NULL)
			clbkExportScene(NULL, NULL, false);
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
	case GLFW_KEY_R:
		if (action == GLFW_RELEASE)
			Reset();
		break;
	case GLFW_KEY_T:
		if (action == GLFW_RELEASE) {
			bRenderTexture = !bRenderTexture;
			if (clbkCompileMesh != NULL)
				clbkCompileMesh();
		}
		break;
	case GLFW_KEY_V:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_SHIFT) {
				bRenderImageVisibility = !bRenderImageVisibility;
			} else {
				bRenderViews = !bRenderViews;
			}
		}
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
	case GLFW_KEY_KP_SUBTRACT:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_CONTROL)
				camera.SetFOV(camera.fov-5.f);
			else if (mod & GLFW_MOD_SHIFT)
				camera.scaleF *= 0.9f;
			else
				cameraBlend = MAXF(cameraBlend-0.1f, 0.f);
		}
		break;
	case GLFW_KEY_KP_ADD:
		if (action == GLFW_RELEASE) {
			if (mod & GLFW_MOD_CONTROL)
				camera.SetFOV(camera.fov+5.f);
			else if (mod & GLFW_MOD_SHIFT)
				camera.scaleF *= 1.1111f;
			else
				cameraBlend = MINF(cameraBlend+0.1f, 1.f);
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
	switch (button) {
	case GLFW_MOUSE_BUTTON_LEFT: {
		if (action == GLFW_PRESS) {
			inputType.set(INP_MOUSE_LEFT);
		} else
		if (action == GLFW_RELEASE) {
			inputType.unset(INP_MOUSE_LEFT);
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
		if (clbkRayScene != NULL) {
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
			const Eigen::Vector3d start(invV.topRightCorner<3,1>());
			const Eigen::Vector4d ray_wor(invV*ray_eye);
			const Eigen::Vector3d dir(ray_wor.topRows<3>().normalized());
			clbkRayScene(Ray3d(start, dir), action);
		}
	} break;
	case GLFW_MOUSE_BUTTON_MIDDLE: {
		if (action == GLFW_PRESS) {
			inputType.set(INP_MOUSE_MIDDLE);
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		} else
		if (action == GLFW_RELEASE) {
			inputType.unset(INP_MOUSE_MIDDLE);
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
	} break;
	case GLFW_MOUSE_BUTTON_RIGHT: {
		if (action == GLFW_PRESS) {
			inputType.set(INP_MOUSE_RIGHT);
		} else
		if (action == GLFW_RELEASE) {
			inputType.unset(INP_MOUSE_RIGHT);
		}
	}
	}
}
void Window::MouseButton(GLFWwindow* window, int button, int action, int mods)
{
	g_mapWindows[window]->MouseButton(button, action, mods);
}

void Window::MouseMove(double xpos, double ypos)
{
	UpdateMousePosition(xpos, ypos);
	if (inputType.isSet(INP_MOUSE_LEFT)) {
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		camera.Rotate(pos, prevPos);
	} else
	if (inputType.isSet(INP_MOUSE_MIDDLE)) {
		camera.Translate(pos, prevPos);
	}
}
void Window::MouseMove(GLFWwindow* window, double xpos, double ypos)
{
	g_mapWindows[window]->MouseMove(xpos, ypos);
}

void Window::Scroll(double /*xoffset*/, double yoffset)
{
	camera.dist *= (yoffset>0 ? POW(1.11,yoffset) : POW(0.9,-yoffset));
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
