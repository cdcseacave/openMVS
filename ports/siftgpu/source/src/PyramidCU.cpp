////////////////////////////////////////////////////////////////////////////
//	File:		PyramidCU.cpp
//	Author:		Changchang Wu
//	Description : implementation of the PyramidCU class.
//				CUDA-based implementation of SiftPyramid
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
#include <string.h>
#include <math.h>
using namespace std;

#include "GlobalUtil.h"
#include "GLTexImage.h"
#include "CuTexImage.h" 
#include "SiftGPU.h"
#include "SiftPyramid.h"
#include "ProgramCU.h"
#include "PyramidCU.h"


//#include "imdebug/imdebuggl.h"
//#pragma comment (lib, "../lib/imdebug.lib")



#define USE_TIMING()		double t, t0, tt;
#define OCTAVE_START()		if(GlobalUtil::_timingO){	t = t0 = CLOCK();	cout<<"#"<<i+_down_sample_factor<<"\t";	}
#define LEVEL_FINISH()		if(GlobalUtil::_timingL){	ProgramCU::FinishCUDA();	tt = CLOCK();cout<<(tt-t)<<"\t";	t = CLOCK();}
#define OCTAVE_FINISH()		if(GlobalUtil::_timingO)cout<<"|\t"<<(CLOCK()-t0)<<endl;


PyramidCU::PyramidCU(SiftParam& sp) : SiftPyramid(sp)
{
	_allPyramid = NULL;
	_featureTex = NULL;
	_keyMaskTex = new CuTexImage();
	_keyRowCountTex = new CuTexImage();
	_keyRowStartTex = new CuTexImage();
	_orientationCountTex = new CuTexImage();
	_orientationStartTex = new CuTexImage();
	_featureAllTex = new CuTexImage();
	_descriptorTex = NULL;
	_orientationTex = NULL;
	_bufferPBO = 0;
    _bufferTEX = NULL;
	_inputTex = new CuTexImage();

    /////////////////////////
    InitializeContext();
}

PyramidCU::~PyramidCU()
{
	DestroyPerLevelData();
	DestroySharedData();
	DestroyPyramidData();
	if(_inputTex) delete _inputTex;
	delete _keyMaskTex;
	delete _keyRowCountTex;
	delete _keyRowStartTex;
	delete _orientationCountTex;
	delete _orientationStartTex;
	delete _featureAllTex;
    if(_bufferPBO) glDeleteBuffers(1, &_bufferPBO);
    if(_bufferTEX) delete _bufferTEX;
}

void PyramidCU::InitializeContext()
{
    GlobalUtil::InitGLParam(1);
    GlobalUtil::_GoodOpenGL = max(GlobalUtil::_GoodOpenGL, 1); 
}

void PyramidCU::InitPyramid(int w, int h, int ds)
{
	int wp, hp, toobig = 0;
	if(ds == 0)
	{
		//
		TruncateWidth(w);
		////
		_down_sample_factor = 0;
		if(GlobalUtil::_octave_min_default>=0)
		{
			wp = w >> _octave_min_default;
			hp = h >> _octave_min_default;
		}else
		{
			//can't upsample by more than 8
			_octave_min_default = max(-3, _octave_min_default);
			//
			wp = w << (-_octave_min_default);
			hp = h << (-_octave_min_default);
		}
		_octave_min = _octave_min_default;
	}else
	{
		//must use 0 as _octave_min; 
		_octave_min = 0;
		_down_sample_factor = ds;
		w >>= ds;
		h >>= ds;
		/////

		TruncateWidth(w);

		wp = w;
		hp = h; 

	}

	while(wp > GlobalUtil::_texMaxDim  || hp > GlobalUtil::_texMaxDim )
	{
		_octave_min ++;
		wp >>= 1;
		hp >>= 1;
		toobig = 1;
	}

	while(GlobalUtil::_MemCapGPU > 0 && GlobalUtil::_FitMemoryCap &&  (wp >_pyramid_width || hp > _pyramid_height)&& 
		max(max(wp, hp), max(_pyramid_width, _pyramid_height)) >  1024 * sqrt(GlobalUtil::_MemCapGPU / 110.0))
	{
		_octave_min ++;
		wp >>= 1;
		hp >>= 1;
		toobig = 2;
	}


	if(toobig && GlobalUtil::_verbose && _octave_min > 0)
	{
		std::cout<<(toobig == 2 ? "[**SKIP OCTAVES**]:\tExceeding Memory Cap (-nomc)\n" :
					"[**SKIP OCTAVES**]:\tReaching the dimension limit(-maxd)!\n");
	}
	//ResizePyramid(wp, hp);
	if( wp == _pyramid_width && hp == _pyramid_height && _allocated )
	{
		FitPyramid(wp, hp);
	}else if(GlobalUtil::_ForceTightPyramid || _allocated ==0)
	{
		ResizePyramid(wp, hp);
	}
	else if( wp > _pyramid_width || hp > _pyramid_height )
	{
		ResizePyramid(max(wp, _pyramid_width), max(hp, _pyramid_height));
		if(wp < _pyramid_width || hp < _pyramid_height)  FitPyramid(wp, hp);
	}
	else
	{
		//try use the pyramid allocated for large image on small input images
		FitPyramid(wp, hp);
	}
}

void PyramidCU::ResizePyramid(int w, int h)
{
	//
	unsigned int totalkb = 0;
	int _octave_num_new, input_sz, i, j;
	//

	if(_pyramid_width == w && _pyramid_height == h && _allocated) return;

	if(w > GlobalUtil::_texMaxDim || h > GlobalUtil::_texMaxDim) return ;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<w<<"x"<<h<<endl;
	//first octave does not change
	_pyramid_octave_first = 0;

	
	//compute # of octaves

	input_sz = min(w,h) ;
	_pyramid_width =  w;
	_pyramid_height =  h;



	//reset to preset parameters

	_octave_num_new  = GlobalUtil::_octave_num_default;

	if(_octave_num_new < 1) 
	{
		_octave_num_new = (int) floor (log ( double(input_sz))/log(2.0)) -3 ;
		if(_octave_num_new<1 ) _octave_num_new = 1;
	}

	if(_pyramid_octave_num != _octave_num_new)
	{
		//destroy the original pyramid if the # of octave changes
		if(_octave_num >0) 
		{
			DestroyPerLevelData();
			DestroyPyramidData();
		}
		_pyramid_octave_num = _octave_num_new;
	}

	_octave_num = _pyramid_octave_num;

	int noct = _octave_num;
	int nlev = param._level_num;

	//	//initialize the pyramid
	if(_allPyramid==NULL)	_allPyramid = new CuTexImage[ noct* nlev * DATA_NUM];

	CuTexImage * gus =  GetBaseLevel(_octave_min, DATA_GAUSSIAN);
	CuTexImage * dog =  GetBaseLevel(_octave_min, DATA_DOG);
	CuTexImage * got =  GetBaseLevel(_octave_min, DATA_GRAD);
	CuTexImage * key =  GetBaseLevel(_octave_min, DATA_KEYPOINT);

	////////////there could be "out of memory" happening during the allocation

	for(i = 0; i< noct; i++)
	{
		int wa = ((w + 3) / 4) * 4;

		totalkb += ((nlev *8 -19)* (wa * h) * 4 / 1024);
		for( j = 0; j< nlev; j++, gus++, dog++, got++, key++)
		{
			gus->InitTexture(wa, h); //nlev
			if(j==0)continue;
			dog->InitTexture(wa, h);  //nlev -1
			if(	j >= 1 && j < 1 + param._dog_level_num)
			{
				got->InitTexture(wa, h, 2); //2 * nlev - 6
			}
			if(j > 1 && j < nlev -1)	key->InitTexture(wa, h, 4); // nlev -3 ; 4 * nlev - 12
		}
		w>>=1;
		h>>=1;
	}

	totalkb += ResizeFeatureStorage();

	if(ProgramCU::CheckErrorCUDA("ResizePyramid")) SetFailStatus(); 

	_allocated = 1;

	if(GlobalUtil::_verbose && GlobalUtil::_timingS) std::cout<<"[Allocate Pyramid]:\t" <<(totalkb/1024)<<"MB\n";

}

void PyramidCU::FitPyramid(int w, int h)
{
	_pyramid_octave_first = 0;
	//
	_octave_num  = GlobalUtil::_octave_num_default;

	int _octave_num_max = max(1, (int) floor (log ( double(min(w, h)))/log(2.0))  -3 );

	if(_octave_num < 1 || _octave_num > _octave_num_max) 
	{
		_octave_num = _octave_num_max;
	}


	int pw = _pyramid_width>>1, ph = _pyramid_height>>1;
	while(_pyramid_octave_first + _octave_num < _pyramid_octave_num &&  
		pw >= w && ph >= h)
	{
		_pyramid_octave_first++;
		pw >>= 1;
		ph >>= 1;
	}

	//////////////////
	int nlev = param._level_num;
	CuTexImage * gus =  GetBaseLevel(_octave_min, DATA_GAUSSIAN);
	CuTexImage * dog =  GetBaseLevel(_octave_min, DATA_DOG);
	CuTexImage * got =  GetBaseLevel(_octave_min, DATA_GRAD);
	CuTexImage * key =  GetBaseLevel(_octave_min, DATA_KEYPOINT);
	for(int i = 0; i< _octave_num; i++)
	{
		int wa = ((w + 3) / 4) * 4;

		for(int j = 0; j< nlev; j++, gus++, dog++, got++, key++)
		{
			gus->InitTexture(wa, h); //nlev
			if(j==0)continue;
			dog->InitTexture(wa, h);  //nlev -1
			if(	j >= 1 && j < 1 + param._dog_level_num)
			{
				got->InitTexture(wa, h, 2); //2 * nlev - 6
			}
			if(j > 1 && j < nlev -1)	key->InitTexture(wa, h, 4); // nlev -3 ; 4 * nlev - 12
		}
		w>>=1;
		h>>=1;
	}
}

int PyramidCU::CheckCudaDevice(int device)
{
    return ProgramCU::CheckCudaDevice(device);
}

void PyramidCU::SetLevelFeatureNum(int idx, int fcount)
{
	_featureTex[idx].InitTexture(fcount, 1, 4);
	_levelFeatureNum[idx] = fcount;
}

int PyramidCU::ResizeFeatureStorage()
{
	int totalkb = 0;
	if(_levelFeatureNum==NULL)	_levelFeatureNum = new int[_octave_num * param._dog_level_num];
	std::fill(_levelFeatureNum, _levelFeatureNum+_octave_num * param._dog_level_num, 0); 

	int i;

	//initialize the feature texture
	int idx = 0, n = _octave_num * param._dog_level_num;
	if(_featureTex==NULL)	_featureTex = new CuTexImage[n];
	if(GlobalUtil::_MaxOrientation >1 && GlobalUtil::_OrientationPack2==0 && _orientationTex== NULL)
		_orientationTex = new CuTexImage[n];


	for(i = 0; i < _octave_num; i++)
	{
		CuTexImage * tex = GetBaseLevel(i+_octave_min);
		int fmax = int(tex->GetImgWidth() * tex->GetImgHeight()*GlobalUtil::_MaxFeaturePercent);
		//
		if(fmax > GlobalUtil::_MaxLevelFeatureNum) fmax = GlobalUtil::_MaxLevelFeatureNum;
		else if(fmax < 32) fmax = 32;	//give it at least a space of 32 feature

		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			_featureTex[idx].InitTexture(fmax, 1, 4);
			totalkb += fmax * 16 /1024;
			//
			if(GlobalUtil::_MaxOrientation>1 && GlobalUtil::_OrientationPack2 == 0)
			{
				_orientationTex[idx].InitTexture(fmax, 1, 4);
				totalkb += fmax * 16 /1024;
			}
		}
	}


	//this just need be initialized once
	if(_descriptorTex==NULL)
	{
		//initialize feature texture pyramid
		int fmax = _featureTex->GetImgWidth();
		_descriptorTex = new CuTexImage;
		totalkb += ( fmax /2);
		_descriptorTex->InitTexture(fmax *128, 1, 1);
	}else
	{
		totalkb +=  _descriptorTex->GetDataSize()/1024;
	}
	return totalkb;
}

void PyramidCU::GetFeatureDescriptors() 
{
	//descriptors...
	float* pd =  &_descriptor_buffer[0];
	vector<float> descriptor_buffer2;

	//use another buffer if we need to re-order the descriptors
	if(_keypoint_index.size() > 0)
	{
		descriptor_buffer2.resize(_descriptor_buffer.size());
		pd = &descriptor_buffer2[0];
	}

	// the descriptors of all levels in one buffer, downloaded at once
	int num = 0;
	for(int idx = 0; idx < _octave_num * param._dog_level_num; idx++) num += _levelFeatureNum[idx];
	_descriptorTex->InitTexture(num * 128, 1, 1);
	float* descriptors = (float*) _descriptorTex->_cuData;
	CuTexImage * got, * ftex= _featureTex;
	for(int i = 0, idx = 0; i < _octave_num; i++)
	{
		got = GetBaseLevel(i + _octave_min, DATA_GRAD) + 1;
		for(int j = 0; j < param._dog_level_num; j++, ftex++, idx++, got++)
		{
			if(_levelFeatureNum[idx]==0) continue;
			ProgramCU::ComputeDescriptor(ftex, got, descriptors, IsUsingRectDescription());
			descriptors += 128*_levelFeatureNum[idx];
		}
	}
	if(num > 0) _descriptorTex->CopyToHost(pd);

	if(GlobalUtil::_timingS) ProgramCU::FinishCUDA();

	if(_keypoint_index.size() > 0)
	{
	    //put the descriptor back to the original order for keypoint list.
		for(int i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_descriptor_buffer[index*128], &descriptor_buffer2[i*128], 128 * sizeof(float));
		}
	}

	if(ProgramCU::CheckErrorCUDA("PyramidCU::GetFeatureDescriptors")) SetFailStatus(); 
}

void PyramidCU::GenerateFeatureListTex() 
{

	vector<float> list;
	int idx = 0;
	const double twopi = 2.0*3.14159265358979323846;
	float sigma_half_step = powf(2.0f, 0.5f / param._dog_level_num);
	float octave_sigma = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f; 
	if(_down_sample_factor>0) octave_sigma *= float(1<<_down_sample_factor); 

	_keypoint_index.resize(0); // should already be 0
	for(int i = 0; i < _octave_num; i++, octave_sigma*= 2.0f)
	{
		for(int j = 0; j < param._dog_level_num; j++, idx++)
		{
			list.resize(0);
			float level_sigma = param.GetLevelSigma(j + param._level_min + 1) * octave_sigma;
			float sigma_min = level_sigma / sigma_half_step;
			float sigma_max = level_sigma * sigma_half_step;
			int fcount = 0 ;
			for(int k = 0; k < _featureNum; k++)
			{
				float * key = &_keypoint_buffer[k*4];
                float sigmak = key[2]; 
                //////////////////////////////////////
                if(IsUsingRectDescription()) sigmak = min(key[2], key[3]) / 12.0f; 

				if(   (sigmak >= sigma_min && sigmak < sigma_max)
					||(sigmak < sigma_min && i ==0 && j == 0)
					||(sigmak > sigma_max && i == _octave_num -1 && j == param._dog_level_num - 1))
				{
					//add this keypoint to the list
					list.push_back((key[0] - offset) / octave_sigma + 0.5f);
					list.push_back((key[1] - offset) / octave_sigma + 0.5f);
                    if(IsUsingRectDescription())
                    {
                        list.push_back(key[2] / octave_sigma);
                        list.push_back(key[3] / octave_sigma);
                    }else
                    {
					    list.push_back(key[2] / octave_sigma);
					    list.push_back((float)fmod(twopi-key[3], twopi));
                    }
					fcount ++;
					//save the index of keypoints
					_keypoint_index.push_back(k);
				}

			}

			_levelFeatureNum[idx] = fcount;
			if(fcount==0)continue;
			CuTexImage * ftex = _featureTex+idx;

			SetLevelFeatureNum(idx, fcount);
			ftex->CopyFromHost(&list[0]);
		}
	}

	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}

}

// Expands the features with two orientations into two features, on the GPU: the orientation
// counts of each block of features of all levels come back in one transfer, the output offset of
// each block is the sum of the counts before it, and the expanded features of all levels are
// written into one buffer, copied back to the level lists and downloaded at once.
void PyramidCU::ReshapeFeatureListCPU()
{
	const int n = param._dog_level_num*_octave_num;
	std::vector<int> blockOffset(n + 1, 0);
	for(int i = 0; i < n; i++)
		blockOffset[i + 1] = blockOffset[i] + ProgramCU::GetOrientationBlockNum(_levelFeatureNum[i]);
	const int nblock = blockOffset[n];
	if(nblock == 0)
	{
		_featureNum = 0;
		_keypoint_buffer.resize(0);
		return;
	}
	_orientationCountTex->InitTexture(nblock, 1);
	_orientationStartTex->InitTexture(nblock, 1);
	int* blockCount = (int*) _orientationCountTex->_cuData;
	for(int i = 0; i < n; i++)
		if(_levelFeatureNum[i] > 0) ProgramCU::CountOrientations(_featureTex + i, _levelFeatureNum[i], blockCount + blockOffset[i]);

	std::vector<int> count(nblock), start(nblock);
	std::vector<int> levelCount(n, 0), levelStart(n, 0);
	_orientationCountTex->CopyToHost(&count[0]);
	_featureNum = 0;
	for(int i = 0; i < n; i++)
	{
		levelStart[i] = _featureNum;
		for(int b = blockOffset[i]; b < blockOffset[i + 1]; ++b)
		{
			start[b] = _featureNum;
			_featureNum += count[b];
		}
		levelCount[i] = _featureNum - levelStart[i];
	}
	_orientationStartTex->CopyFromHost(&start[0]);
	_featureAllTex->InitTexture(_featureNum, 1, 4);
	float4* all = (float4*) _featureAllTex->_cuData;
	for(int i = 0; i < n; i++)
		if(_levelFeatureNum[i] > 0) ProgramCU::ExpandOrientations(_featureTex + i, _levelFeatureNum[i], (const int*) _orientationStartTex->_cuData + blockOffset[i], all);
	for(int i = 0; i < n; i++)
	{
		// a reallocation of the list waits for the expansion that reads it
		SetLevelFeatureNum(i, levelCount[i]);
		if(levelCount[i] > 0)
			cudaMemcpyAsync(_featureTex[i]._cuData, all + levelStart[i], levelCount[i] * sizeof(float4), cudaMemcpyDeviceToDevice);
	}

#ifdef NO_DUPLICATE_DOWNLOAD
	// the keypoints in image coordinates, so DownloadKeypoints is not needed
	std::vector<float> features(4 * _featureNum);
	if(_featureNum > 0) _featureAllTex->CopyToHost(&features[0]);
	const double twopi = 2.0*3.14159265358979323846;
	float os = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	if(_down_sample_factor>0) os *= float(1<<_down_sample_factor);
	const float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f;
	_keypoint_buffer.resize(_featureNum * 4);
	for(int i = 0; i < n; i++)
	{
		const float oss = os * (1 << (i / param._dog_level_num));
		float* ds = &_keypoint_buffer[0] + levelStart[i] * 4;
		const float* fs = &features[0] + levelStart[i] * 4;
		for(int k = 0; k < levelCount[i]; k++, ds+=4, fs+=4)
		{
			ds[0] = oss*(fs[0]-0.5f) + offset;	//x
			ds[1] = oss*(fs[1]-0.5f) + offset;	//y
			ds[2] = oss*fs[2];  //scale
			ds[3] = (float)fmod(twopi-fs[3], twopi);	//orientation, mirrored
		}
	}
#endif
	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features MO:\t"<<_featureNum<<endl;
	}
}

void PyramidCU::GenerateFeatureDisplayVBO() 
{
	//it is weird that this part is very slow.
	//use a big VBO to save all the SIFT box vertices
	int nvbo = _octave_num * param._dog_level_num;
	if(_featureDisplayVBO==NULL)
	{
		//initialize the vbos
		_featureDisplayVBO = new GLuint[nvbo];
		_featurePointVBO = new GLuint[nvbo];
		glGenBuffers(nvbo, _featureDisplayVBO);	
		glGenBuffers(nvbo, _featurePointVBO);
	}
	for(int i = 0; i < nvbo; i++)
	{
		if(_levelFeatureNum[i]<=0)continue;
		CuTexImage * ftex  = _featureTex + i;
		CuTexImage texPBO1( _levelFeatureNum[i]* 10, 1, 4, _featureDisplayVBO[i]);
		CuTexImage texPBO2(_levelFeatureNum[i], 1, 4, _featurePointVBO[i]);
		ProgramCU::DisplayKeyBox(ftex, &texPBO1);
		ProgramCU::DisplayKeyPoint(ftex, &texPBO2);	
	}
}

void PyramidCU::DestroySharedData() 
{
	//descriptor storage shared by all levels
	if(_descriptorTex)
	{
		delete _descriptorTex;
		_descriptorTex = NULL;
	}
	//cpu reduction buffer.
	if(_histo_buffer)
	{
		delete[] _histo_buffer;
		_histo_buffer = 0;
	}
}

void PyramidCU::DestroyPerLevelData() 
{
	//integers vector to store the feature numbers.
	if(_levelFeatureNum)
	{
		delete [] _levelFeatureNum;
		_levelFeatureNum = NULL;
	}
	//texture used to store features
	if(	_featureTex)
	{
		delete [] _featureTex;
		_featureTex =	NULL;
	}
	//texture used for multi-orientation 
	if(_orientationTex)
	{
		delete [] _orientationTex;
		_orientationTex = NULL;
	}
	int no = _octave_num* param._dog_level_num;

	//two sets of vbos used to display the features
	if(_featureDisplayVBO)
	{
		glDeleteBuffers(no, _featureDisplayVBO);
		delete [] _featureDisplayVBO;
		_featureDisplayVBO = NULL;
	}
	if( _featurePointVBO)
	{
		glDeleteBuffers(no, _featurePointVBO);
		delete [] _featurePointVBO;
		_featurePointVBO = NULL;
	}
}

void PyramidCU::DestroyPyramidData()
{
	if(_allPyramid)
	{
		delete [] _allPyramid;
		_allPyramid = NULL;
	}
}

void PyramidCU::DownloadKeypoints() 
{
	const double twopi = 2.0*3.14159265358979323846;
	int idx = 0;
	float * buffer = &_keypoint_buffer[0];
	vector<float> keypoint_buffer2;
	//use a different keypoint buffer when processing with an exisint features list
	//without orientation information. 
	if(_keypoint_index.size() > 0)
	{
		keypoint_buffer2.resize(_keypoint_buffer.size());
		buffer = &keypoint_buffer2[0];
	}
	float * p = buffer, *ps;
	CuTexImage * ftex = _featureTex;
	/////////////////////
	float os = _octave_min>=0? float(1<<_octave_min): 1.0f/(1<<(-_octave_min));
	if(_down_sample_factor>0) os *= float(1<<_down_sample_factor); 
	float offset = GlobalUtil::_LoweOrigin? 0 : 0.5f;
	/////////////////////
	for(int i = 0; i < _octave_num; i++, os *= 2.0f)
	{
	
		for(int j = 0; j  < param._dog_level_num; j++, idx++, ftex++)
		{

			if(_levelFeatureNum[idx]>0)
			{	
				ftex->CopyToHost(ps = p);
				for(int k = 0;  k < _levelFeatureNum[idx]; k++, ps+=4)
				{
					ps[0] = os*(ps[0]-0.5f) + offset;	//x
					ps[1] = os*(ps[1]-0.5f) + offset;	//y
					ps[2] = os*ps[2]; 
					ps[3] = (float)fmod(twopi-ps[3], twopi);	//orientation, mirrored
				}
				p+= 4* _levelFeatureNum[idx];
			}
		}
	}

	//put the feature into their original order for existing keypoint 
	if(_keypoint_index.size() > 0)
	{
		for(int i = 0; i < _featureNum; ++i)
		{
			int index = _keypoint_index[i];
			memcpy(&_keypoint_buffer[index*4], &keypoint_buffer2[i*4], 4 * sizeof(float));
		}
	}
}

void PyramidCU::GenerateFeatureListCPU()
{
	//no cpu version provided
	GenerateFeatureList();
}

// The keypoint detection left a bit mask of the keypoints and the keypoint count of every row
// of every level (see ProgramCU::ComputeKEY). One transfer brings the row counts of all levels
// back, the list offset of each row is the sum of the counts before it, and one kernel per level
// writes the keypoint positions in row-major order.
void PyramidCU::GenerateFeatureList()
{
	const int reverse = (GlobalUtil::_TruncateMethod == 1);
	const int nlevel = _octave_num * param._dog_level_num;
	const int nrow = _keyRowOffset[nlevel];
	_keyRowCount.resize(nrow);
	_keyRowStart.assign(nrow, 0);
	_keyRowCountTex->CopyToHost(&_keyRowCount[0]);

	_featureNum = 0;
	FOR_EACH_OCTAVE(i, reverse)
	{
		FOR_EACH_LEVEL(j, reverse)
		{
			const int idx = i * param._dog_level_num + j;
			// (mgprt 20/06/2018) _levelFeatureNum can still contain old
			// values for these levels, so if we do not reset them the sum
			// of level features will not match the absolte number of features.
			if (GlobalUtil::_TruncateMethod && GlobalUtil::_FeatureCountThreshold > 0 && _featureNum > GlobalUtil::_FeatureCountThreshold) {
				_levelFeatureNum[idx] = 0;
				continue;
			}
			int fcount = 0;
			for(int r = _keyRowOffset[idx]; r < _keyRowOffset[idx + 1]; ++r)
			{
				_keyRowStart[r] = fcount;
				fcount += _keyRowCount[r];
			}
			SetLevelFeatureNum(idx, fcount);
			_featureNum += fcount;
		}
	}
	_keyRowStartTex->CopyFromHost(&_keyRowStart[0]);

	const unsigned int* mask = (const unsigned int*) _keyMaskTex->_cuData;
	const int* rowCount = (const int*) _keyRowCountTex->_cuData;
	const int* rowStart = (const int*) _keyRowStartTex->_cuData;
	for(int i = 0, idx = 0; i < _octave_num; i++)
	{
		CuTexImage* key = GetBaseLevel(_octave_min + i, DATA_KEYPOINT) + 2;
		for(int j = 0; j < param._dog_level_num; j++, idx++, key++)
		{
			if(GlobalUtil::_timingO) std::cout << _levelFeatureNum[idx] << (j + 1 < param._dog_level_num ? "\t" : "\n");
			if(_levelFeatureNum[idx] == 0) continue;
			const int row = _keyRowOffset[idx];
			ProgramCU::ListKeys(_featureTex + idx, mask + _keyMaskOffset[idx], ProgramCU::GetKeyMaskWidth(key->GetImgWidth()),
				rowCount + row, rowStart + row, key->GetImgHeight());
		}
	}
	if(GlobalUtil::_timingS)ProgramCU::FinishCUDA();

	if(GlobalUtil::_verbose)
	{
		std::cout<<"#Features:\t"<<_featureNum<<"\n";
	}

	if(ProgramCU::CheckErrorCUDA("PyramidCU::GenerateFeatureList")) SetFailStatus();
}

GLTexImage* PyramidCU::GetLevelTexture(int octave, int level)
{
	return GetLevelTexture(octave, level, DATA_GAUSSIAN);
}

GLTexImage* PyramidCU::ConvertTexCU2GL(CuTexImage* tex, int dataName)
{
	
	GLenum format = GL_LUMINANCE;
	int convert_done = 1;
    if(_bufferPBO == 0) glGenBuffers(1, &_bufferPBO);
    if(_bufferTEX == NULL) _bufferTEX = new GLTexImage;
	switch(dataName)
	{
	case DATA_GAUSSIAN:
		{
			convert_done = tex->CopyToPBO(_bufferPBO);
			break;
		}
	case DATA_DOG:
		{
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 1, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertDOG(tex, &texPBO);
			break;
		}
	case DATA_GRAD:
		{
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 1, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertGRD(tex, &texPBO);
			break;
		}
	case DATA_KEYPOINT:
		{
			CuTexImage * dog = tex - param._level_num * _pyramid_octave_num;
			format = GL_RGBA;
			CuTexImage texPBO(tex->GetImgWidth(), tex->GetImgHeight(), 4, _bufferPBO);
			if(texPBO._cuData == 0 || tex->_cuData == NULL) convert_done = 0;
			else ProgramCU::DisplayConvertKEY(tex, dog, &texPBO);
			break;
		}
	default:
			convert_done = 0;
			break;
	}

	if(convert_done)
	{
		_bufferTEX->InitTexture(max(_bufferTEX->GetTexWidth(), tex->GetImgWidth()), max(_bufferTEX->GetTexHeight(), tex->GetImgHeight()));
		_bufferTEX->CopyFromPBO(_bufferPBO, tex->GetImgWidth(), tex->GetImgHeight(), format);
	}else
	{
		_bufferTEX->SetImageSize(0, 0);
	}

	return _bufferTEX;
}

GLTexImage* PyramidCU::GetLevelTexture(int octave, int level, int dataName) 
{
	CuTexImage* tex = GetBaseLevel(octave, dataName) + (level - param._level_min);
	//CuTexImage* gus = GetBaseLevel(octave, DATA_GAUSSIAN) + (level - param._level_min); 
	return ConvertTexCU2GL(tex, dataName);
}

void PyramidCU::ConvertInputToCU(GLTexInput* input)
{
	int ws = input->GetImgWidth(), hs = input->GetImgHeight();
	TruncateWidth(ws);
	//copy the input image to pixel buffer object
    if(input->_pixel_data)
    {
        _inputTex->InitTexture(ws, hs, 1);
        _inputTex->CopyFromHost(input->_pixel_data); 
    }else
    {
        if(_bufferPBO == 0) glGenBuffers(1, &_bufferPBO);
        if(input->_rgb_converted && input->CopyToPBO(_bufferPBO, ws, hs, GL_LUMINANCE))
        {
		    _inputTex->InitTexture(ws, hs, 1);
            _inputTex->CopyFromPBO(ws, hs, _bufferPBO); 
        }else if(input->CopyToPBO(_bufferPBO, ws, hs))
	    {
		    CuTexImage texPBO(ws, hs, 4, _bufferPBO);
		    _inputTex->InitTexture(ws, hs, 1);
		    ProgramCU::ReduceToSingleChannel(_inputTex, &texPBO, !input->_rgb_converted);
	    }else
	    {
		    std::cerr<< "Unable To Convert Input\n";
	    }
    }
}

void PyramidCU::BuildPyramid(GLTexInput * input)
{

	USE_TIMING();

	int i, j;
	
	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{

		float* filter_sigma = param._sigma;
		CuTexImage *tex = GetBaseLevel(i);
		CuTexImage *buf = GetBaseLevel(i, DATA_KEYPOINT) +2;
		j = param._level_min + 1;

		OCTAVE_START();

		if( i == _octave_min )
		{	
			ConvertInputToCU(input);

			if(i == 0)
			{
				ProgramCU::FilterImage(tex, _inputTex, buf, 
                    param.GetInitialSmoothSigma(_octave_min + _down_sample_factor));
			}else
			{
				if(i < 0)	ProgramCU::SampleImageU(tex, _inputTex, -i);			
				else		ProgramCU::SampleImageD(tex, _inputTex, i);
				ProgramCU::FilterImage(tex, tex, buf, 
                    param.GetInitialSmoothSigma(_octave_min + _down_sample_factor));
			}
		}else
		{
			ProgramCU::SampleImageD(tex, GetBaseLevel(i - 1) + param._level_ds - param._level_min); 
			if(param._sigma_skip1 > 0)
			{
				ProgramCU::FilterImage(tex, tex, buf, param._sigma_skip1);
			}
		}
		LEVEL_FINISH();
		for( ; j <=  param._level_max ; j++, tex++, filter_sigma++)
		{
			// filtering
			ProgramCU::FilterImage(tex + 1, tex, buf, *filter_sigma);
			LEVEL_FINISH();
		}
		OCTAVE_FINISH();
	}
	if(GlobalUtil::_timingS) ProgramCU::FinishCUDA();

	if(ProgramCU::CheckErrorCUDA("PyramidCU::BuildPyramid")) SetFailStatus();
}

void PyramidCU::DetectKeypointsEX()
{


	int i, j;
	double t0, t, ts, t1, t2;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = CLOCK();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		CuTexImage * gus = GetBaseLevel(i) + 1;
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) + 1;
		CuTexImage * got = GetBaseLevel(i, DATA_GRAD) + 1;
		//compute the gradient
		for(j = param._level_min +1; j <=  param._level_max ; j++, gus++, dog++, got++)
		{
			//input: gus and gus -1
			//output: gradient, dog, orientation
			ProgramCU::ComputeDOG(gus, dog, got);
		}
	}
	if(GlobalUtil::_timingS && GlobalUtil::_verbose)
	{
		ProgramCU::FinishCUDA();
		t1 = CLOCK();
	}

	// keypoint bit masks and per-row keypoint counts of all levels (see GenerateFeatureList)
	const int nlevel = _octave_num * param._dog_level_num;
	_keyMaskOffset.resize(nlevel);
	_keyRowOffset.resize(nlevel + 1);
	int nmask = 0, nrow = 0;
	for(i = 0; i < _octave_num; i++)
	{
		CuTexImage * dog = GetBaseLevel(i + _octave_min, DATA_DOG) + 2;
		for(j = 0; j < param._dog_level_num; j++, dog++)
		{
			const int idx = i * param._dog_level_num + j;
			_keyMaskOffset[idx] = nmask;
			_keyRowOffset[idx] = nrow;
			nmask += ProgramCU::GetKeyMaskWidth(dog->GetImgWidth()) * dog->GetImgHeight();
			nrow += dog->GetImgHeight();
		}
	}
	_keyRowOffset[nlevel] = nrow;
	_keyMaskTex->InitTexture(nmask, 1);
	_keyRowCountTex->InitTexture(nrow, 1);
	_keyRowStartTex->InitTexture(nrow, 1);
	cudaMemsetAsync(_keyRowCountTex->_cuData, 0, nrow * sizeof(int));

	for ( i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		if(GlobalUtil::_timingO)
		{
			t0 = CLOCK();
			std::cout<<"#"<<(i + _down_sample_factor)<<"\t";
		}
		const int level0 = (i - _octave_min) * param._dog_level_num;
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) + 2;
		CuTexImage * key = GetBaseLevel(i, DATA_KEYPOINT) +2;
		CuTexImage * gus = GetBaseLevel(i) + 2; // Gaussian level of the DoG, for darkness adaptation


		for( j = param._level_min +2; j <  param._level_max ; j++, dog++, key++, gus++)
		{
			const int idx = level0 + j - (param._level_min + 2);
			if(GlobalUtil::_timingL)t = CLOCK();
			//input, dog, dog + 1, dog -1
			//output, key
			// sigma of the keypoints found on this level (see GetFeatureOrientations)
			ProgramCU::ComputeKEY(dog, key, gus, (unsigned int*) _keyMaskTex->_cuData + _keyMaskOffset[idx],
				(int*) _keyRowCountTex->_cuData + _keyRowOffset[idx],
				param._dog_threshold, param._edge_threshold, param.GetLevelSigma(j - 1));
			if(GlobalUtil::_timingL)
			{
				std::cout<<(CLOCK()-t)<<"\t";
			}
		}
		if(GlobalUtil::_timingO)
		{
			std::cout<<"|\t"<<(CLOCK()-t0)<<"\n";
		}
	}

	if(GlobalUtil::_timingS)
	{
		ProgramCU::FinishCUDA();
		if(GlobalUtil::_verbose) 
		{	
			t2 = CLOCK();
			std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)<<"\n"
						<<"<Get Keypoints  >\t"<<(t2-t1)<<"\n";
		}				
	}
}

void PyramidCU::ComputeGradient() 
{

	int i, j;
	double ts, t1;

	if(GlobalUtil::_timingS && GlobalUtil::_verbose)ts = CLOCK();

	for(i = _octave_min; i < _octave_min + _octave_num; i++)
	{
		CuTexImage * gus = GetBaseLevel(i) +  1;
		CuTexImage * dog = GetBaseLevel(i, DATA_DOG) +  1;
		CuTexImage * got = GetBaseLevel(i, DATA_GRAD) +  1;

		//compute the gradient
		for(j = 0; j <  param._dog_level_num ; j++, gus++, dog++, got++)
		{
			ProgramCU::ComputeDOG(gus, dog, got);
		}
	}
	if(GlobalUtil::_timingS)
	{
		ProgramCU::FinishCUDA();
		if(GlobalUtil::_verbose)
		{
			t1 = CLOCK();
			std::cout	<<"<Gradient, DOG  >\t"<<(t1-ts)<<"\n";
		}
	}
}

void PyramidCU::GetFeatureOrientations() 
{

	CuTexImage * ftex = _featureTex;
	int * count	 = _levelFeatureNum;
	float sigma, sigma_step = powf(2.0f, 1.0f/param._dog_level_num);

	for(int i = 0; i < _octave_num; i++)
	{
		CuTexImage* got = GetBaseLevel(i + _octave_min, DATA_GRAD) + 1;
		CuTexImage* key = GetBaseLevel(i + _octave_min, DATA_KEYPOINT) + 2;

		for(int j = 0; j < param._dog_level_num; j++, ftex++, count++, got++, key++)
		{
			if(*count<=0)continue;

			//if(ftex->GetImgWidth() < *count) ftex->InitTexture(*count, 1, 4);

			sigma = param.GetLevelSigma(j+param._level_min+1);

			ProgramCU::ComputeOrientation(ftex, got, key, sigma, sigma_step, _existing_keypoints);		
		}
	}

	if(GlobalUtil::_timingS)ProgramCU::FinishCUDA();
	if(ProgramCU::CheckErrorCUDA("PyramidCU::GetFeatureOrientations")) SetFailStatus();

}

void PyramidCU::GetSimplifiedOrientation() 
{
	//no simplified orientation
	GetFeatureOrientations();
}

CuTexImage* PyramidCU::GetBaseLevel(int octave, int dataName)
{
	if(octave <_octave_min || octave > _octave_min + _octave_num) return NULL;
	int offset = (_pyramid_octave_first + octave - _octave_min) * param._level_num;
	int num = param._level_num * _pyramid_octave_num;
	if (dataName == DATA_ROT) dataName = DATA_GRAD;
	return _allPyramid + num * dataName + offset;
}

#endif
