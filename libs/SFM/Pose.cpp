////////////////////////////////////////////////////////////////////
// Pose.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Pose.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

PMatrix Pose3D::GetPfromRC() const
{
	PMatrix P;
	AssembleProjectionMatrix(R, C, P);
	return P;
}
/*----------------------------------------------------------------*/
