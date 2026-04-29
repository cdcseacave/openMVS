/*
 * TestsMVS.h
 *
 * Copyright (c) 2014-2025 SEACAVE
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


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// test MVS stages on a small sample dataset
bool PipelineTest(bool verbose = false);

// test OrthoMap Step 1: Grid Initialization & DEM Generation
bool TestOrthoMapConfig(bool verbose = false);
bool TestOrthoMapGSD(bool verbose = false);
bool TestOrthoMapGrid(bool verbose = false);
bool TestOrthoMapCamera(bool verbose = false);
bool TestOrthoMapRasterize(bool verbose = false);
bool TestOrthoMapOverlap(bool verbose = false);
bool TestOrthoMapIntegration(bool verbose = false);
/*----------------------------------------------------------------*/

} // namespace MVS
