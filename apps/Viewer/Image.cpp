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
#include "Window.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Image::Image(MVS::IIndex _idx)
	:
	idx(_idx)
{
}
Image::~Image()
{
	Release();
}

Image::Image(Image &&other) noexcept
	:
	Texture(std::move(other)),
	idx(other.idx),
	opacity(other.opacity),
	pImage(other.pImage)
{
}

Image &Image::operator=(Image &&other) noexcept
{
	if (this != &other) {
		Texture::operator=(std::move(other));
		idx = other.idx;
		opacity = other.opacity;
		pImage = other.pImage;
	}
	return *this;
}

void Image::Release()
{
	Texture::Release();
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
void Image::CancelImageLoading()
{
	if (IsImageLoading())
		Thread::safeExchange(pImage.ptr, (int_t)IMG_NULL);
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
	Thread::safeExchange(pImage.ptr, pImg.ptr);
}
bool Image::TransferImage()
{
	if (!IsImageValid())
		return false;
	Create(*pImage, true);
	ReleaseImage();
	Window::RequestRedraw();
	return true;
}
/*----------------------------------------------------------------*/
