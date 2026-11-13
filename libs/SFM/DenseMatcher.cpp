/*
 * DenseMatcher.cpp
 *
 * Copyright (c) 2014-2026 SEACAVE
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
 */

#include "Common.h"
#include "DenseMatcher.h"
#include "MatchROMA2.h"
#include "RoMa2Matcher.h"
#include "FlexGlueMatcher.h"

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

bool DenseMatcher::IsAvailable()
{
	return RoMa2Onnx::IsAvailable();
}

std::unique_ptr<DenseMatcher> DenseMatcher::Create(const ROMA2Config& config)
{
	ASSERT(config.matcher == "roma2" || config.matcher == "flexglue");
	ASSERT(IsAvailable());
	#ifdef _USE_ONNXRUNTIME
	if (config.matcher == "flexglue")
		return std::make_unique<FlexGlueMatcher>();
	#endif
	return std::make_unique<RoMa2Onnx>();
}

unsigned DenseMatcher::ExpectedRetrievalSize(const ROMA2Config& config)
{
	ASSERT(config.matcher == "roma2" || config.matcher == "flexglue");
	String modelDir(config.ResolveModelPath());
	if (modelDir.empty())
		return 0;
	Util::ensureFolderSlash(modelDir);
	if (config.matcher == "flexglue") {
		FlexGlueManifest manifest;
		return manifest.Load(modelDir + "flexglue_" + config.setting + ".json") ? manifest.retrievalDim : 0;
	}
	RoMa2Manifest manifest;
	return manifest.Load(modelDir + "roma_" + config.setting + ".json") ? (unsigned)manifest.facetsDim : 0;
}
/*----------------------------------------------------------------*/
