/*
* Common.h
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

#ifndef _MVS_COMMON_H_
#define _MVS_COMMON_H_


// I N C L U D E S /////////////////////////////////////////////////

#if defined(MVS_EXPORTS) && !defined(Common_EXPORTS)
#define Common_EXPORTS
#endif

#include "../Common/Common.h"
#include "../IO/Common.h"
#include "../Math/Common.h"

#ifndef MVS_API
#define MVS_API GENERAL_API
#endif
#ifndef MVS_TPL
#define MVS_TPL GENERAL_TPL
#endif


// D E F I N E S ///////////////////////////////////////////////////


// P R O T O T Y P E S /////////////////////////////////////////////

using namespace SEACAVE;

namespace MVS {

/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_COMMON_H_
