////////////////////////////////////////////////////////////////////////////
//	File:		PyramidCU.h
//	Author:		Changchang Wu
//	Description : interface for the PyramidCU
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



#ifndef _PYRAMID_CU_H
#define _PYRAMID_CU_H
#if defined(CUDA_SIFTGPU_ENABLED)

class GLTexImage;
class CuTexImage;
class SiftPyramid;
class PyramidCU:public SiftPyramid
{
	CuTexImage* _inputTex;
	CuTexImage* _allPyramid;
	// keypoint bit masks (32 pixels per word) and per-row keypoint counts of all levels
	CuTexImage* _keyMaskTex;
	CuTexImage* _keyRowCountTex;
	CuTexImage* _keyRowStartTex;
	// multi-orientation expansion: orientation counts and output offsets of the feature blocks,
	// expanded features of all levels
	CuTexImage* _orientationCountTex;
	CuTexImage* _orientationStartTex;
	CuTexImage* _featureAllTex;
	std::vector<int> _keyMaskOffset, _keyRowOffset;
	std::vector<int> _keyRowCount, _keyRowStart;
	CuTexImage* _featureTex;
	CuTexImage* _descriptorTex;
	CuTexImage* _orientationTex;
	GLuint		_bufferPBO;
    GLTexImage* _bufferTEX;
public:
	virtual void GetFeatureDescriptors();
	virtual void GenerateFeatureListTex();
	virtual void ReshapeFeatureListCPU();
	virtual void GenerateFeatureDisplayVBO();
	virtual void DestroySharedData();
	virtual void DestroyPerLevelData();
	virtual void DestroyPyramidData();
	virtual void DownloadKeypoints();
	virtual void GenerateFeatureListCPU();
	virtual void GenerateFeatureList();
	virtual GLTexImage* GetLevelTexture(int octave, int level);
	virtual GLTexImage* GetLevelTexture(int octave, int level, int dataName);
	virtual void BuildPyramid(GLTexInput * input);
	virtual void DetectKeypointsEX();
	virtual void ComputeGradient();
	virtual void GetFeatureOrientations();
	virtual void GetSimplifiedOrientation();
	virtual void InitPyramid(int w, int h, int ds = 0);
	virtual void ResizePyramid(int w, int h);
    virtual int  IsUsingRectDescription(){return _existing_keypoints & SIFT_RECT_DESCRIPTION; }	
	//////////
	void FitPyramid(int w, int h);

    void InitializeContext();
	int ResizeFeatureStorage();
	void SetLevelFeatureNum(int idx, int fcount);
	void ConvertInputToCU(GLTexInput* input);
	GLTexImage* ConvertTexCU2GL(CuTexImage* tex, int dataName);
	CuTexImage* GetBaseLevel(int octave, int dataName = DATA_GAUSSIAN);
    void TruncateWidth(int& w) { w = GLTexInput::TruncateWidthCU(w); }
    //////////////////////////
    static int CheckCudaDevice(int device);
public:
	PyramidCU(SiftParam& sp);
	virtual ~PyramidCU();
};



#endif
#endif
