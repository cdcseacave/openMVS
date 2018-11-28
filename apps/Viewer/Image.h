/*
 * Image.h
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

#ifndef _VIEWER_IMAGE_H_
#define _VIEWER_IMAGE_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace VIEWER {

class Image
{
public:
	typedef CLISTDEFIDX(Image,uint32_t) ImageArr;
	enum {
		IMG_NULL = 0,
		IMG_LOADING,
		IMG_VALID
	};
	union ImagePtrInt {
		cv::Mat* pImage;
		int_t ptr;
		inline ImagePtrInt() : ptr(IMG_NULL) {}
		inline ImagePtrInt(cv::Mat* p) : pImage(p) {}
		inline operator cv::Mat* () const { return pImage; }
		inline operator cv::Mat*& () { return pImage; }
	};

public:
	MVS::IIndex idx; // image index in the current scene
	int width, height;
	GLuint texture;
	double opacity;
	ImagePtrInt pImage;

public:
	Image(MVS::IIndex = NO_ID);
	~Image();

	void Release();
	void ReleaseImage();
	inline bool IsValid() const { return texture > 0; }
	inline bool IsImageEmpty() const { return pImage.ptr == IMG_NULL; }
	inline bool IsImageLoading() const { return pImage.ptr == IMG_LOADING; }
	inline bool IsImageValid() const { return pImage.ptr >= IMG_VALID; }

	void SetImageLoading();
	void AssignImage(cv::InputArray);
	bool TransferImage();

	void SetImage(cv::InputArray);
	void GenerateMipmap() const;
	void Bind() const;

protected:
};
typedef Image::ImageArr ImageArr;
/*----------------------------------------------------------------*/

} // namespace VIEWER

#endif // _VIEWER_IMAGE_H_
