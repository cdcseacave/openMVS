/*
 * Tests.cpp
 *
 * Copyright (c) 2014-2021 SEACAVE
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

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// test MVS functionality
int main(int argc, LPCTSTR* argv)
{
	OPEN_LOG();
	OPEN_LOGCONSOLE();
	Util::Init();
	TD_TIMER_START();
	if (!SEACAVE::cListTest<true>(100)) {
		VERBOSE("ERROR: cListTest failed!");
		return EXIT_FAILURE;
	}
	if (!SEACAVE::OctreeTest<double,2>(100)) {
		VERBOSE("ERROR: OctreeTest<double,2> failed!");
		return EXIT_FAILURE;
	}
	if (!SEACAVE::OctreeTest<float,3>(100)) {
		VERBOSE("ERROR: OctreeTest<float,3> failed!");
		return EXIT_FAILURE;
	}
	if (!SEACAVE::TestRayTriangleIntersection<float>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<float> failed!");
		return EXIT_FAILURE;
	}
	if (!SEACAVE::TestRayTriangleIntersection<double>(1000)) {
		VERBOSE("ERROR: TestRayTriangleIntersection<double> failed!");
		return EXIT_FAILURE;
	}
	VERBOSE("All tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
