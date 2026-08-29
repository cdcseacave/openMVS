////////////////////////////////////////////////////////////////////
// SFM.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_H_
#define _SFM_H_


// I N C L U D E S /////////////////////////////////////////////////

// SFM library - Structure from Motion

// Core components
#include "SFM/Common.h"
#include "SFM/Pose.h"
#include "SFM/Camera.h"
#include "SFM/View.h"
#include "SFM/Image.h"
#include "SFM/ImagePair.h"
#include "SFM/Scene.h"

// Keyframe extraction
#include "SFM/KeyframeExtractor.h"

// Image matching and pairing
#include "SFM/VocabularyTree.h"
#include "SFM/PairsMatcher.h"
#include "SFM/PairsWeighting.h"
#include "SFM/RelativePoseRefine.h"
#include "SFM/MatchROMA2.h"
#include "SFM/ROMA2Warp.h"

// Track building and triangulation
#include "SFM/Track.h"
#include "SFM/Triangulation.h"

// Scene clustering
#include "SFM/SceneCluster.h"

// Incremental reconstruction
#include "SFM/StarInitializer.h"
#include "SFM/Resection.h"
#include "SFM/BundleAdjustment.h"

// Sub-scene alignment
#include "SFM/SimilarityTransform.h"
#include "SFM/GlobalAlignment.h"

// Interface to external formats/tools
#include "SFM/InterfaceMVS.h"
#include "SFM/PoseIO.h"
#include "SFM/ImportCOLMAP.h"

// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

#endif // _SFM_H_
