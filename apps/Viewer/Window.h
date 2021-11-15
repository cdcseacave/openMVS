/*
 * Window.h
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

#ifndef _VIEWER_WINDOW_H_
#define _VIEWER_WINDOW_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "Image.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace VIEWER {

class Window
{
public:
	GLFWwindow* window; // window handle
	Camera camera; // current camera
	Eigen::Vector2d pos, prevPos; // current and previous mouse position (normalized)
	Eigen::Matrix4d transform; // view matrix corresponding to the currently selected image
	cv::Size size; // resolution in pixels, sometimes not equal to window resolution, ex. on Retina display
	double sizeScale; // window/screen resolution scale

	enum INPUT : unsigned {
		INP_NA = 0,
		INP_MOUSE_LEFT    = (1 << 0),
		INP_MOUSE_MIDDLE  = (1 << 1),
		INP_MOUSE_RIGHT   = (1 << 2),
	};
	Flags inputType;

	enum SPARSE {
		SPR_NONE   = 0,
		SPR_POINTS = (1 << 0),
		SPR_LINES  = (1 << 1),
		SPR_ALL    = SPR_POINTS|SPR_LINES
	};
	SPARSE sparseType;
	unsigned minViews;
	float pointSize;
	float cameraBlend;
	bool bRenderCameras;
	bool bRenderCameraTrajectory;
	bool bRenderImageVisibility;
	bool bRenderViews;
	bool bRenderSolid;
	bool bRenderTexture;
	bool bRenderBounds;

	enum SELECTION {
		SEL_NA = 0,
		SEL_POINT,
		SEL_TRIANGLE
	};
	SELECTION selectionType;
	Point3f selectionPoints[4];
	double selectionTimeClick, selectionTime;
	IDX selectionIdx;

	typedef DELEGATE<bool (LPCTSTR, LPCTSTR)> ClbkOpenScene;
	ClbkOpenScene clbkOpenScene;
	typedef DELEGATE<bool (LPCTSTR, LPCTSTR, bool)> ClbkExportScene;
	ClbkExportScene clbkExportScene;
	typedef DELEGATE<void (const Ray3&, int)> ClbkRayScene;
	ClbkRayScene clbkRayScene;
	typedef DELEGATE<void (void)> ClbkCompilePointCloud;
	ClbkCompilePointCloud clbkCompilePointCloud;
	typedef DELEGATE<void (void)> ClbkCompileMesh;
	ClbkCompileMesh clbkCompileMesh;
	typedef DELEGATE<void (void)> ClbkCompileBounds;
	ClbkCompileBounds clbkCompileBounds;

	typedef std::unordered_map<GLFWwindow*,Window*> WindowsMap;
	static WindowsMap g_mapWindows;

public:
	Window();
	~Window();

	void Release();
	void ReleaseClbk();
	inline bool IsValid() const { return window != NULL; }

	inline GLFWwindow* GetWindow() { return window; }

	bool Init(const cv::Size&, LPCTSTR name);
	void SetCamera(const Camera&);
	void SetName(LPCTSTR);
	void SetVisible(bool);
	bool IsVisible() const;
	void Reset(SPARSE sparseType=SPR_ALL, unsigned minViews=2);

	void CenterCamera(const Point3&);

	void UpdateView(const ImageArr&, const MVS::ImageArr&);
	void UpdateView(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
	void UpdateMousePosition(double xpos, double ypos);

	void GetFrame(Image8U3&) const;

	cv::Size GetSize() const;
	void Resize(const cv::Size&);
	static void Resize(GLFWwindow* window, int width, int height);
	void Key(int k, int scancode, int action, int mod);
	static void Key(GLFWwindow* window, int k, int scancode, int action, int mod);
	void MouseButton(int button, int action, int mods);
	static void MouseButton(GLFWwindow* window, int button, int action, int mods);
	void MouseMove(double xpos, double ypos);
	static void MouseMove(GLFWwindow* window, double xpos, double ypos);
	void Scroll(double xoffset, double yoffset);
	static void Scroll(GLFWwindow* window, double xoffset, double yoffset);
	void Drop(int count, const char** paths);
	static void Drop(GLFWwindow* window, int count, const char** paths);

protected:
	bool IsShiftKeyPressed() const;
	bool IsCtrlKeyPressed() const;
	bool IsAltKeyPressed() const;
};
/*----------------------------------------------------------------*/

} // namespace VIEWER

#endif // _VIEWER_WINDOW_H_
