#ifndef __INSTANCES_H__
#define __INSTANCES_H__


#if defined(_MSC_VER)

// C4661: '...' : no suitable definition provided for explicit template instantiation request
#pragma warning(disable: 4661)

#endif

#include "typeBinary.h"
#include "typeBinaryFast.h"
#include "typePotts.h"
#include "typeGeneral.h"
#include "typeTruncatedLinear.h"
#include "typeTruncatedQuadratic.h"
#include "typeTruncatedLinear2D.h"
#include "typeTruncatedQuadratic2D.h"


#endif
