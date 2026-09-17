/*
 * GlobalDescriptors.cpp
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

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "GlobalDescriptors.h"
#include "Scene.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

bool GlobalDescriptors::Build(const Scene& scene)
{
	descriptors.resize(0, 0);
	imageIDs.clear();
	if (scene.images.size() < 2) {
		VERBOSE("error: at least 2 images are needed to build a global-descriptor retrieval index (scene has %u)", scene.images.size());
		return false;
	}
	const int dim = scene.images[0].globalDescriptor.cols;
	descriptors.resize(scene.images.size(), dim);
	imageIDs.resize(scene.images.size());
	FOREACH(i, scene.images) {
		const Image& img = scene.images[i];
		if (!img.HasGlobalDescriptor() || img.globalDescriptor.rows != 1 ||
			img.globalDescriptor.cols != dim || img.globalDescriptor.type() != CV_32F) {
			VERBOSE("error: image %u has no 1x%d CV_32F global descriptor", img.ID, dim);
			descriptors.resize(0, 0);
			imageIDs.clear();
			return false;
		}
		const Eigen::Map<const Eigen::RowVectorXf> row(img.globalDescriptor.ptr<float>(), dim);
		descriptors.row(i) = row.normalized(); // defensive re-normalization: the graph already emits unit vectors
		imageIDs[i] = img.ID;
	}
	return true;
}

std::vector<std::pair<uint32_t, float>> GlobalDescriptors::Query(IIndex idx, unsigned maxResults) const
{
	ASSERT(IsValid() && idx < Size());
	const Eigen::VectorXf sims = descriptors * descriptors.row(idx).transpose();
	std::vector<std::pair<uint32_t, float>> ranked;
	ranked.reserve(Size() - 1);
	for (Eigen::Index r = 0; r < sims.size(); ++r)
		if ((IIndex)r != idx)
			ranked.emplace_back(imageIDs[(IIndex)r], sims[r]); // self is the only exclusion
	const size_t numTaken = MINF((size_t)maxResults, ranked.size());
	// total order, so that two queries of the same index return the identical prefix
	std::partial_sort(ranked.begin(), ranked.begin() + numTaken, ranked.end(),
		[](const auto& a, const auto& b) {
			return a.second != b.second ? a.second > b.second : a.first < b.first;
		});
	ranked.resize(numTaken);
	return ranked;
}
/*----------------------------------------------------------------*/


#pragma pop_macro("VERBOSE")
