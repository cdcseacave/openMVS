/*
 * GlobalDescriptors.h
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

#ifndef _SFM_GLOBALDESCRIPTORS_H_
#define _SFM_GLOBALDESCRIPTORS_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "MatchROMA2.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

// Pool one image's ROMAv2 descriptor-graph output into its global retrieval descriptor:
// tensor is a [numSlices, numPatches, channels] channels-last fp32 buffer (the `value_facets`
// or the `layers` output of the describe graph, batch dimension dropped), recipe selects
// between the request's design (FACETS, numSlices*channels-d) and the shipped reference
// (LAYERS, channels-d, pooled from the deepest slice only), and power is the exponent of the
// signed power normalization FACETS ends with (<=0 or 1 disables it).
// The result is L2-normalized and accumulated in double, matching the Python reference.
SFM_API void PoolRetrievalDescriptor(
	const float* tensor,
	unsigned numSlices,
	unsigned numPatches,
	unsigned channels,
	RetrievalRecipe recipe,
	float power,
	std::vector<float>& descriptor);
/*----------------------------------------------------------------*/

// Cosine retrieval index over the per-image global descriptors of a scene: the alternative
// to the vocabulary tree as the source of the per-image ranked candidate lists the pair
// selection consumes. All descriptors share one dimension and are held as unit rows, so a
// query is a single matrix-vector product.
class SFM_API GlobalDescriptors
{
public:
	// Index every image of the scene by its global descriptor.
	// Returns false if the scene has less than two images or any image is missing a
	// descriptor of the common dimension.
	bool Build(const Scene& scene);

	// Check if the index is ready to be queried
	inline bool IsValid() const { return !imageIDs.empty() && descriptors.cols() > 0; }

	// Dimension of the indexed descriptors
	inline int Dim() const { return (int)descriptors.cols(); }

	// Number of indexed images
	inline unsigned Size() const { return (unsigned)imageIDs.size(); }

	// Rank the images most similar to the given one (as an index in the scene image array).
	// Returns at most maxResults (imageID, cosine similarity) pairs, the query image
	// excluded, sorted by decreasing similarity and, on ties, by increasing image ID.
	std::vector<std::pair<uint32_t, float>> Query(IIndex idx, unsigned maxResults) const;

private:
	// one unit-norm descriptor per scene image, row-major so a row maps a contiguous descriptor
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors;
	std::vector<uint32_t> imageIDs; // image ID of every descriptor row
};
/*----------------------------------------------------------------*/

// Export the global-descriptor retrieval rankings of a scene to a CSV file:
// header `idxA,idxB,similarity,imageA,imageB`, one row per retrieved candidate, ordered by
// increasing query image, then by decreasing similarity, then by increasing candidate image;
// maxRank is the number of candidates kept per image.
// Returns false if the index cannot be built or the file cannot be written.
SFM_API bool ExportRetrievalRankingsCSV(const Scene& scene, const String& fileName, unsigned maxRank = 50);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBALDESCRIPTORS_H_
