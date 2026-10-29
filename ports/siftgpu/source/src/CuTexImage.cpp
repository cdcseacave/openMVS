////////////////////////////////////////////////////////////////////////////
//	File:		CuTexImage.cpp
//	Author:		Changchang Wu
//	Description : implementation of the CuTexImage class.
//
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty.
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////

#if defined(CUDA_SIFTGPU_ENABLED)

#include <glad/glad.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <math.h>
#include <cstring>
using namespace std;


#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>

#include "GlobalUtil.h"
#include "GLTexImage.h"
#include "CuTexImage.h"
#include "ProgramCU.h"

cudaTextureObject_t CuTexImage::GetTexture(cudaResourceType type, const cudaTextureDesc& textureDesc,
										   const cudaChannelFormatDesc& channelFmtDesc)
{
	const int width = type == cudaResourceTypePitch2D ? _imgWidth : 0;
	const int height = type == cudaResourceTypePitch2D ? _imgHeight : 0;
	for(const TexObj& tex : _texObjs)
	{
		if(tex.type == type && tex.readMode == textureDesc.readMode && tex.width == width && tex.height == height &&
		   tex.format.x == channelFmtDesc.x && tex.format.y == channelFmtDesc.y && tex.format.z == channelFmtDesc.z &&
		   tex.format.w == channelFmtDesc.w && tex.format.f == channelFmtDesc.f)
			return tex.handle;
	}

	cudaResourceDesc resourceDesc;
	memset(&resourceDesc, 0, sizeof(resourceDesc));
	resourceDesc.resType = type;
	if(type == cudaResourceTypePitch2D)
	{
		resourceDesc.res.pitch2D.devPtr = _cuData;
		resourceDesc.res.pitch2D.width = _imgWidth;
		resourceDesc.res.pitch2D.height = _imgHeight;
		resourceDesc.res.pitch2D.pitchInBytes = _imgWidth * _numChannel * sizeof(float);
		resourceDesc.res.pitch2D.desc = channelFmtDesc;
	}else
	{
		resourceDesc.res.linear.devPtr = _cuData;
		resourceDesc.res.linear.desc = channelFmtDesc;
		resourceDesc.res.linear.sizeInBytes = _numBytes;
	}
	TexObj tex = {type, channelFmtDesc, textureDesc.readMode, width, height, 0};
	cudaCreateTextureObject(&tex.handle, &resourceDesc, &textureDesc, nullptr);
	ProgramCU::CheckErrorCUDA("CuTexImage::GetTexture");
	_texObjs.push_back(tex);
	return tex.handle;
}

void CuTexImage::ReleaseTextures()
{
	for(const TexObj& tex : _texObjs)
		cudaDestroyTextureObject(tex.handle);
	_texObjs.clear();
}

cudaTextureObject_t CuTexImage::BindTexture(const cudaTextureDesc& textureDesc,
											const cudaChannelFormatDesc& channelFmtDesc)
{
	return GetTexture(cudaResourceTypeLinear, textureDesc, channelFmtDesc);
}

cudaTextureObject_t CuTexImage::BindTexture2D(const cudaTextureDesc& textureDesc,
											  const cudaChannelFormatDesc& channelFmtDesc)
{
	return GetTexture(cudaResourceTypePitch2D, textureDesc, channelFmtDesc);
}

CuTexImage::CuTexImage()
{
	_cuData = NULL;
	_fromPBO = 0;
	_numChannel = _numBytes = 0;
	_imgWidth = _imgHeight = 0;
}

CuTexImage::CuTexImage(int width, int height, int nchannel, GLuint pbo)
{
	_cuData = NULL;

	//check size of pbo
	GLint bsize, esize = width * height * nchannel * sizeof(float);
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
	glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
	if(bsize < esize)
	{
		glBufferData(GL_PIXEL_PACK_BUFFER_ARB, esize,	NULL, GL_STATIC_DRAW_ARB);
		glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
	}
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
	if(bsize >=esize)
	{

		cudaGLRegisterBufferObject(pbo);
		cudaGLMapBufferObject(&_cuData, pbo);
		ProgramCU::CheckErrorCUDA("cudaGLMapBufferObject");
		_fromPBO = pbo;
	}else
	{
		_cuData = NULL;
		_fromPBO = 0;
	}
	if(_cuData)
	{
		_numBytes = bsize;
		_imgWidth = width;
		_imgHeight = height;
		_numChannel = nchannel;
	}else
	{
		_numBytes = 0;
		_imgWidth = 0;
		_imgHeight = 0;
		_numChannel = 0;
	}
}

CuTexImage::~CuTexImage()
{
	if(_fromPBO)
	{
		// the kernels using the mapped buffer must finish before it is unmapped
		cudaDeviceSynchronize();
		ReleaseTextures();
		cudaGLUnmapBufferObject(_fromPBO);
		cudaGLUnregisterBufferObject(_fromPBO);
	}else if(_cuData)
	{
		cudaFree(_cuData);
	}
	ReleaseTextures();
}

void CuTexImage::SetImageSize(int width, int height)
{
	_imgWidth = width;
	_imgHeight = height;
}

bool CuTexImage::InitTexture(int width, int height, int nchannel)
{
	_imgWidth = width;
	_imgHeight = height;
	_numChannel = min(max(nchannel, 1), 4);

	const size_t size = width * height * _numChannel * sizeof(float);

  if (size < 0) {
    return false;
  }

  // SiftGPU uses int for all indexes and
  // this ensures that all elements can be accessed.
  if (size >= INT_MAX * sizeof(float)) {
    return false;
  }

	if(size <= _numBytes) return true;

	// cudaFree waits for the kernels still using the buffer and its textures
	if(_cuData) cudaFree(_cuData);
	ReleaseTextures();

	//allocate the array data
	const cudaError_t status = cudaMalloc(&_cuData, _numBytes = size);

  if (status != cudaSuccess) {
    _cuData = NULL;
    _numBytes = 0;
    return false;
  }

  return true;
}

void CuTexImage::CopyFromHost(const void * buf)
{
	if(_cuData == NULL) return;
	cudaMemcpy( _cuData, buf, _imgWidth * _imgHeight * _numChannel * sizeof(float), cudaMemcpyHostToDevice);
}

void CuTexImage::CopyToHost(void * buf)
{
	if(_cuData == NULL) return;
	cudaMemcpy(buf, _cuData, _imgWidth * _imgHeight * _numChannel * sizeof(float), cudaMemcpyDeviceToHost);
}

void CuTexImage::CopyFromPBO(int width, int height, GLuint pbo)
{
	void* pbuf =NULL;
	GLint esize = width * height * sizeof(float);
	cudaGLRegisterBufferObject(pbo);
	cudaGLMapBufferObject(&pbuf, pbo);

	cudaMemcpy(_cuData, pbuf, esize, cudaMemcpyDeviceToDevice);

	cudaGLUnmapBufferObject(pbo);
	cudaGLUnregisterBufferObject(pbo);
}

int CuTexImage::CopyToPBO(GLuint pbo)
{
	void* pbuf =NULL;
	GLint bsize, esize = _imgWidth * _imgHeight * sizeof(float) * _numChannel;
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo);
	glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
	if(bsize < esize)
	{
		glBufferData(GL_PIXEL_PACK_BUFFER_ARB, esize*3/2,	NULL, GL_STATIC_DRAW_ARB);
		glGetBufferParameteriv(GL_PIXEL_PACK_BUFFER_ARB, GL_BUFFER_SIZE, &bsize);
	}
	glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

	if(bsize >= esize)
	{
		cudaGLRegisterBufferObject(pbo);
		cudaGLMapBufferObject(&pbuf, pbo);
		cudaMemcpy(pbuf, _cuData, esize, cudaMemcpyDeviceToDevice);
		cudaGLUnmapBufferObject(pbo);
		cudaGLUnregisterBufferObject(pbo);
		return 1;
	}else
	{
		return 0;
	}
}

#endif
