/*
 * Image.cpp
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
#include "Image.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Image::Image(MVS::IIndex _idx)
	:
	idx(_idx),
	texture(0)
{
}
Image::~Image()
{
	Release();
}

void Image::Release()
{
	if (IsValid()) {
		glDeleteTextures(1, &texture);
		texture = 0;
	}
	ReleaseImage();
}
void Image::ReleaseImage()
{
	if (IsImageValid()) {
		cv::Mat* const p(pImage);
		Thread::safeExchange(pImage.ptr, (int_t)IMG_NULL);
		delete p;
	}
}

void Image::SetImageLoading()
{
	ASSERT(IsImageEmpty());
	Thread::safeExchange(pImage.ptr, (int_t)IMG_LOADING);
}
void Image::AssignImage(cv::InputArray img)
{
	ASSERT(IsImageLoading());
	ImagePtrInt pImg(new cv::Mat(img.getMat()));
	if (pImg.pImage->cols%4 != 0) {
		// make sure the width is multiple of 4 (seems to be an OpenGL limitation)
		cv::resize(*pImg.pImage, *pImg.pImage, cv::Size((pImg.pImage->cols/4)*4, pImg.pImage->rows), 0, 0, cv::INTER_AREA);
	}
	Thread::safeExchange(pImage.ptr, pImg.ptr);
}
bool Image::TransferImage()
{
	if (!IsImageValid())
		return false;
	SetImage(*pImage);
	glfwPostEmptyEvent();
	ReleaseImage();
	return true;
}

void Image::SetImage(cv::InputArray img)
{
	cv::Mat image(img.getMat());
	glEnable(GL_TEXTURE_2D);
	// create texture
	glGenTextures(1, &texture);
	// select our current texture
	glBindTexture(GL_TEXTURE_2D, texture);
	// load texture
	width = image.cols;
	height = image.rows;
	ASSERT(image.channels() == 1 || image.channels() == 3);
	ASSERT(image.isContinuous());
	glTexImage2D(GL_TEXTURE_2D,
				 0, image.channels(),
				 width, height,
				 0, (image.channels() == 1) ? GL_LUMINANCE : GL_BGR,
				 GL_UNSIGNED_BYTE, image.ptr<uint8_t>());
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
}
void Image::GenerateMipmap() const {
	glBindTexture(GL_TEXTURE_2D, texture);
	glGenerateMipmap(GL_TEXTURE_2D);
}
void Image::Bind() const {
	glBindTexture(GL_TEXTURE_2D, texture);
}
/*----------------------------------------------------------------*/
