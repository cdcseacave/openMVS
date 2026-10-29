////////////////////////////////////////////////////////////////////////////
//	File:		CuTexImage.h
//	Author:		Changchang Wu
//	Description :	interface for the CuTexImage class.
//					class for storing data in CUDA.
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


#ifndef CU_TEX_IMAGE_H
#define CU_TEX_IMAGE_H

#include <cuda_runtime.h>
#include <vector>

class GLTexImage;

class CuTexImage
{
protected:
	void*		_cuData;
	int			_numChannel;
	size_t			_numBytes;
	int			_imgWidth;
	int			_imgHeight;
	GLuint		_fromPBO;
	// Texture objects created on the buffer, reused by later bindings with the same format and
	// view size; they are destroyed when the buffer is reallocated or released (only then can no
	// kernel launched with them still be running).
	struct TexObj
	{
		cudaResourceType type;
		cudaChannelFormatDesc format;
		cudaTextureReadMode readMode;
		int width, height;
		cudaTextureObject_t handle;
	};
	std::vector<TexObj> _texObjs;
	cudaTextureObject_t GetTexture(cudaResourceType type, const cudaTextureDesc& textureDesc,
								   const cudaChannelFormatDesc& channelFmtDesc);
	void ReleaseTextures();
public:
	virtual void SetImageSize(int width, int height);
	virtual bool InitTexture(int width, int height, int nchannel = 1);
	// texture of the whole buffer (1D fetches) / of the image (2D fetches, pitch-linear)
	cudaTextureObject_t BindTexture(const cudaTextureDesc& textureDesc, const cudaChannelFormatDesc& channelFmtDesc);
	cudaTextureObject_t BindTexture2D(const cudaTextureDesc& textureDesc, const cudaChannelFormatDesc& channelFmtDesc);
	void CopyToHost(void* buf);
	void CopyFromHost(const void* buf);
	int  CopyToPBO(GLuint pbo);
	void CopyFromPBO(int width, int height, GLuint pbo);
public:
	inline int GetImgWidth(){return _imgWidth;}
	inline int GetImgHeight(){return _imgHeight;}
	inline int GetDataSize(){return _numBytes;}
public:
	CuTexImage();
	CuTexImage(int width, int height, int nchannel, GLuint pbo);
	CuTexImage(const CuTexImage&) = delete;
	CuTexImage& operator=(const CuTexImage&) = delete;
	virtual ~CuTexImage();
	friend class ProgramCU;
	friend class PyramidCU;
};

#endif // !defined(CU_TEX_IMAGE_H)
