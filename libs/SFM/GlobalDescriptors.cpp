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

namespace {

// Generalized-mean pooling (p=3) of one [numPatches x channels] slice, L2-normalized and
// appended to out; the cube costs precision, so it is accumulated in double
void GemSlice(const float* slice, unsigned numPatches, unsigned channels, std::vector<float>& out)
{
	std::vector<double> sums(channels, 0.0);
	for (unsigned p = 0; p < numPatches; ++p) {
		const float* const v = slice + (size_t)p * channels;
		for (unsigned c = 0; c < channels; ++c) {
			const double x = MAXF((double)v[c], 1e-6);
			sums[c] += x * x * x;
		}
	}
	double norm2 = 0.0;
	const size_t offset = out.size();
	out.resize(offset + channels);
	for (unsigned c = 0; c < channels; ++c) {
		const double pooled = std::cbrt(sums[c] / numPatches);
		out[offset + c] = (float)pooled;
		norm2 += pooled * pooled;
	}
	if (norm2 > 0.0) {
		const float inv = (float)(1.0 / std::sqrt(norm2));
		for (unsigned c = 0; c < channels; ++c)
			out[offset + c] *= inv;
	}
}

// L2-normalize the given descriptor in place (no-op if it is all zeros)
void NormalizeL2(std::vector<float>& v)
{
	double norm2 = 0.0;
	for (float x : v)
		norm2 += (double)x * x;
	if (norm2 > 0.0) {
		const float inv = (float)(1.0 / std::sqrt(norm2));
		for (float& x : v)
			x *= inv;
	}
}

} // unnamed namespace

void SFM::PoolRetrievalDescriptor(const float* tensor, unsigned numSlices, unsigned numPatches, unsigned channels, RetrievalRecipe recipe, float power, std::vector<float>& descriptor)
{
	ASSERT(numSlices >= 1 && numPatches > 0 && channels > 0);
	descriptor.clear();
	if (recipe == RetrievalRecipe::LAYERS) {
		// the shipped reference recipe: GeM of the deepest slice alone, L2-normalized
		GemSlice(tensor + (size_t)(numSlices - 1) * numPatches * channels, numPatches, channels, descriptor);
		return;
	}
	// the requested design: per-slice GeM + L2, concatenated, L2, signed power, L2
	for (unsigned s = 0; s < numSlices; ++s)
		GemSlice(tensor + (size_t)s * numPatches * channels, numPatches, channels, descriptor);
	NormalizeL2(descriptor);
	if (power > 0.f && power != 1.f) {
		for (float& x : descriptor)
			x = (x < 0.f ? -1.f : 1.f) * std::pow(ABS(x), power);
		NormalizeL2(descriptor);
	}
}
/*----------------------------------------------------------------*/


bool GlobalDescriptors::Build(const Scene& scene)
{
	descriptors.resize(0, 0);
	imageIDs.clear();
	if (scene.images.size() < 2)
		return false;
	const int dim = scene.images[0].globalDescriptor.cols;
	descriptors.resize(scene.images.size(), dim);
	imageIDs.resize(scene.images.size());
	FOREACH(i, scene.images) {
		const Image& img = scene.images[i];
		if (!img.HasGlobalDescriptor() || img.globalDescriptor.cols != dim || img.globalDescriptor.type() != CV_32F) {
			VERBOSE("error: image %u has no %d-D global descriptor", img.ID, dim);
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


bool SFM::ExportRetrievalRankingsCSV(const Scene& scene, const String& fileName, unsigned maxRank)
{
	GlobalDescriptors index;
	if (!index.Build(scene))
		return false;
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	// the rows name both endpoints by image ID and by image-file stem
	std::unordered_map<IIndex, IIndex> idxFromID;
	idxFromID.reserve(scene.images.size());
	FOREACH(i, scene.images)
		idxFromID.emplace(scene.images[i].ID, i);
	ofs << "idxA,idxB,similarity,imageA,imageB\n";
	unsigned numRows = 0;
	FOREACH(i, scene.images) {
		const String stemA = Util::getFileName(scene.images[i].fileName);
		for (const auto& [imageID, similarity] : index.Query(i, maxRank)) {
			const auto it = idxFromID.find(imageID);
			ASSERT(it != idxFromID.end());
			ofs << scene.images[i].ID << "," << imageID << "," << similarity << ","
				<< stemA << "," << Util::getFileName(scene.images[it->second].fileName) << "\n";
			++numRows;
		}
	}
	ofs.close();
	VERBOSE("Exported the top-%u retrieval rankings of %u images (%u rows) to '%s'",
		maxRank, scene.images.size(), numRows, fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
