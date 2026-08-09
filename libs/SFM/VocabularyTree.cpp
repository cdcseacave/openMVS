/*
 * VocabularyTree.cpp
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
 */

#include "Common.h"
#include "VocabularyTree.h"
#include "Scene.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define VOCABTREE_USE_OPENMP
#endif

#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("VocabTre"));

using EigenVectoru = Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>;
using EigenVectorf = Eigen::VectorXf;
using EigenMatrixf = Eigen::MatrixXf;

struct VocabularyTree::Impl
{
	enum DescType { BINARY = 0, QFLOAT = 1 };

	struct Node
	{
		// Centroid representation for this node:
		// - For quantized float descriptors (RootSIFT-like): `centroid` as unit-range floats.
		// - For binary descriptors (ORB/AKAZE): `centroidBytes` as majority-bit vector.
		EigenVectorf centroid;              // QFLOAT centroid
		std::vector<uint8_t> centroidBytes; // BINARY centroid

		// Children indices in `nodes`; empty means leaf.
		std::vector<int> children;

		// Leaf visual word id (>=0 for leaves, -1 for internal nodes).
		int wordId = -1;

		bool isLeaf() const { return children.empty(); }
	};

	// Config/state
	int K = 10;
	int L = 6;
	unsigned maxKMeansIters = 10;
	unsigned seed = 42;
	int dimBytes = 0;
	DescType dtype = QFLOAT;
	int softAssignmentK = 3;
	unsigned queryExpansionImages = 5;
	unsigned maxDescriptorsPerImage = 1000;

	// Tree
	std::vector<Node> nodes; // nodes[0] is root
	std::vector<int> leafNodeIdx; // wordId -> node index

	// Weights + DB
	std::vector<float> idf; // per word
	std::vector<std::vector<std::pair<uint32_t, float>>> postings; // word -> (img, tf)
	std::vector<float> imageNorm; // per image L2 norm of tf-idf vector
	uint32_t numImages = 0;

	// Cache for top-N descriptors per image
	mutable std::mutex cacheMutex;
	mutable std::unordered_map<IIndex, cv::Mat> descriptorsCache;
	// Images whose descriptors were selected (misses) and served from the cache (hits)
	mutable CacheHitStats hitStats;

	const cv::Mat& getDescriptors(const Image& img) const {
		ASSERT(!img.descriptors.empty());
		std::lock_guard<std::mutex> lock(cacheMutex);
		auto it = descriptorsCache.find(img.ID);
		if (it != descriptorsCache.end()) {
			hitStats.Hit();
			return it->second;
		}
		hitStats.Miss();
		// Cache descriptors with sampling if needed
		cv::Mat& cached = descriptorsCache[img.ID];
		const int rows = img.descriptors.rows;
		if (maxDescriptorsPerImage > 0 && rows > (int)maxDescriptorsPerImage) {
			// Select top N keypoints and copy their descriptors
			UnsignedArr indices = img.SelectTopKeypoints(maxDescriptorsPerImage);
			cached.create((int)indices.size(), img.descriptors.cols, CV_8U);
			for (size_t i = 0; i < indices.size(); ++i) {
				const uint8_t* src = img.descriptors.ptr<uint8_t>((int)indices[i]);
				uint8_t* dst = cached.ptr<uint8_t>((int)i);
				memcpy(dst, src, img.descriptors.cols);
			}
		} else {
			// Let's clone for consistency and ownership in cache.
			cached = img.descriptors.clone(); 
		}
		return cached;
	}

	// --- Utils ---
	/*
	 * Computes Hamming distance between two descriptor byte arrays.
	 * Optimized: counts bits via PopCnt on 32-bit blocks, then tail bytes.
	 */
	static inline int hamming(const uint8_t* a, const uint8_t* b, int nBytes)
	{
		int d = 0;
		const uint32_t* pa = (const uint32_t*)a;
		const uint32_t* pb = (const uint32_t*)b;
		int n = nBytes / 4;
		for (int i = 0; i < n; ++i)
			d += PopCnt(pa[i] ^ pb[i]);
		for (int i = n * 4; i < nBytes; ++i)
			d += (a[i] ^ b[i]) ? 1 : 0;
		return d;
	}

	/*
	 * K-means++-like seeding for float descriptors: picks K initial centers
	 * proportional to squared distance from the last chosen center to spread seeds.
	 */
	void initCentersQ(const std::vector<const uint8_t*>& descs, int Ksel, std::vector<int>& centerIdx, std::mt19937& rng) const
	{
		std::uniform_int_distribution<int> unif(0, (int)descs.size() - 1);
		centerIdx.clear();
		centerIdx.reserve(Ksel);
		centerIdx.push_back(unif(rng));
		std::vector<float> dist2(descs.size(), std::numeric_limits<float>::max());
		while ((int)centerIdx.size() < Ksel) {
			int last = centerIdx.back();
			const uint8_t* c = descs[last];
			for (size_t i = 0; i < descs.size(); ++i) {
				float d = ((Eigen::Map<const EigenVectoru>(descs[i], dimBytes).template cast<float>() - Eigen::Map<const EigenVectoru>(c, dimBytes).template cast<float>()) * (1.f / 255.f)).squaredNorm();
				if (d < dist2[i])
					dist2[i] = d;
			}
			float sum = std::accumulate(dist2.begin(), dist2.end(), 0.f);
			if (sum <= 0.f) {
				centerIdx.push_back(unif(rng));
				continue;
			}
			std::uniform_real_distribution<float> ur(0.f, sum);
			float r = ur(rng);
			float acc = 0.f;
			size_t idx = 0;
			for (; idx < dist2.size(); ++idx) {
				acc += dist2[idx];
				if (acc >= r)
					break;
			}
			if (idx >= descs.size())
				idx = descs.size() - 1;
			centerIdx.push_back((int)idx);
		}
	}
	/*
	 * Random seeding without replacement for binary descriptors (ids refer to subset entries).
	 */
	void initCentersB(const std::vector<int>& ids, int Ksel, std::vector<int>& centerIdx, std::mt19937& rng) const
	{
		centerIdx.clear();
		centerIdx.reserve(Ksel);
		std::unordered_set<int> used;
		std::uniform_int_distribution<int> unif(0, (int)ids.size() - 1);
		while ((int)centerIdx.size() < Ksel) {
			int rid = ids[unif(rng)];
			if (!used.insert(rid).second)
				continue;
			centerIdx.push_back(rid);
		}
	}

	/*
	 * Computes the centroid for a node given a subset of descriptor indices.
	 * - QFLOAT: arithmetic mean in [0,1] space
	 * - BINARY: majority vote per bit across all descriptors
	 */
	template <typename Accessor>
	void computeCentroid(const Accessor& getDesc, const std::vector<int>& subset, Node& node) const
	{
		if (dtype == QFLOAT) {
			node.centroid = EigenVectorf::Zero(dimBytes);
			for (int id : subset) {
				const uint8_t* d = getDesc(id);
				node.centroid += Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f);
			}
			node.centroid /= MAXF<size_t>(1, subset.size());
		} else {
			node.centroidBytes.assign(dimBytes, 0);
			std::vector<int> cnt(dimBytes * 8, 0);
			int N = (int)subset.size();
			for (int id : subset) {
				const uint8_t* d = getDesc(id);
				for (int b = 0; b < dimBytes; ++b) {
					uint8_t v = d[b];
					for (int bit = 0; bit < 8; ++bit)
						if (v & (1u << bit))
							++cnt[b * 8 + bit];
				}
			}
			for (int b = 0; b < dimBytes; ++b) {
				uint8_t out = 0;
				for (int bit = 0; bit < 8; ++bit)
					if (cnt[b * 8 + bit] > N / 2)
						out |= (1u << bit);
				node.centroidBytes[b] = out;
			}
		}
	}

	/*
	 * Recursively builds a hierarchical K-means/medoids tree level by level.
	 * Main blocks:
	 *  - Stop condition: if `depth>=L` or subset size <= K, make leaf word.
	 *  - Initialization: choose K centers (K-means++ for QFLOAT, random for BINARY).
	 *  - Assignment: assign each descriptor to nearest center (L2/Hamming).
	 *  - Update: recompute centers (mean for QFLOAT, majority bits for BINARY).
	 *  - Split: recurse for each cluster; compute parent centroid as average of child centers.
	 */
	template <typename Accessor>
	int buildNode(const Accessor& getDesc, const std::vector<int>& subset, int depth, std::mt19937& rng)
	{
		int nodeIdx = (int)nodes.size();
		nodes.emplace_back();
		Node& node = nodes.back();
		if (depth >= L || (int)subset.size() <= K) {
			node.wordId = (int)leafNodeIdx.size();
			leafNodeIdx.push_back(nodeIdx);
			computeCentroid(getDesc, subset, node);
			return nodeIdx;
		}
		const int Ksel = MINF(K, (int)subset.size());
		std::vector<int> centerPick;
		if (dtype == QFLOAT) {
			std::vector<const uint8_t*> descs;
			descs.reserve(subset.size());
			for (int id : subset)
				descs.push_back(getDesc(id));
			initCentersQ(descs, Ksel, centerPick, rng);
		} else {
			initCentersB(subset, Ksel, centerPick, rng);
		}
		std::vector<EigenVectorf> centersQ;
		std::vector<std::vector<uint8_t>> centersB;
		if (dtype == QFLOAT) {
			centersQ.reserve(Ksel);
			for (int c = 0; c < Ksel; ++c) {
				const uint8_t* d = getDesc(centerPick[c]);
				centersQ.emplace_back(Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f));
			}
		} else {
			centersB.reserve(Ksel);
			for (int c = 0; c < Ksel; ++c) {
				const uint8_t* d = getDesc(centerPick[c]);
				centersB.emplace_back(d, d + dimBytes);
			}
		}
		std::vector<int> assign(subset.size(), -1);
		for (unsigned it = 0; it < maxKMeansIters; ++it) {
			bool changed = false;
			FOREACH(i, subset) {
				const uint8_t* d = getDesc(subset[i]);
				int best = -1;
				float bd = std::numeric_limits<float>::max();
				int bdH = INT_MAX;
				for (int c = 0; c < Ksel; ++c) {
					if (dtype == QFLOAT) {
						float s = (Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f) - centersQ[c]).squaredNorm();
						if (s < bd) {
							bd = s;
							best = c;
						}
					} else {
						int hd = hamming(d, centersB[c].data(), dimBytes);
						if (hd < bdH) {
							bdH = hd;
							best = c;
						}
					}
				}
				if (assign[i] != best) {
					assign[i] = best;
					changed = true;
				}
			}
			if (!changed)
				break;
			if (dtype == QFLOAT) {
				std::vector<EigenVectorf> sum(Ksel, EigenVectorf::Zero(dimBytes));
				std::vector<int> cnt(Ksel, 0);
				for (size_t i = 0; i < subset.size(); ++i) {
					int a = assign[i];
					const uint8_t* d = getDesc(subset[i]);
					sum[a] += Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f);
					++cnt[a];
				}
				for (int c = 0; c < Ksel; ++c) {
					if (cnt[c] > 0)
						centersQ[c] = sum[c] / (float)cnt[c];
				}
			} else {
				std::vector<std::vector<int>> bitCount(Ksel, std::vector<int>(dimBytes * 8, 0));
				std::vector<int> cnt(Ksel, 0);
				for (size_t i = 0; i < subset.size(); ++i) {
					int a = assign[i];
					const uint8_t* d = getDesc(subset[i]);
					++cnt[a];
					for (int b = 0; b < dimBytes; ++b) {
						uint8_t val = d[b];
						for (int bit = 0; bit < 8; ++bit) {
							if (val & (1u << bit))
								++bitCount[a][b * 8 + bit];
						}
					}
				}
				for (int c = 0; c < Ksel; ++c) {
					if (cnt[c] == 0)
						continue;
					centersB[c].assign(dimBytes, 0);
					for (int b = 0; b < dimBytes; ++b) {
						uint8_t out = 0;
						for (int bit = 0; bit < 8; ++bit) {
							if (bitCount[c][b * 8 + bit] > cnt[c] / 2)
								out |= (1u << bit);
						}
						centersB[c][b] = out;
					}
				}
			}
		}
		std::vector<std::vector<int>> sub(Ksel);
		for (size_t i = 0; i < subset.size(); ++i) {
			int a = assign[i];
			if (a < 0)
				a = 0;
			sub[a].push_back(subset[i]);
		}
		for (int c = 0; c < Ksel; ++c)
			if (sub[c].empty()) {
				int maxc = 0;
				for (int t = 1; t < Ksel; ++t)
					if (sub[t].size() > sub[maxc].size())
						maxc = t;
				if (!sub[maxc].empty()) {
					sub[c].push_back(sub[maxc].back());
					sub[maxc].pop_back();
				}
			}
		if (dtype == QFLOAT) {
			nodes[nodeIdx].centroid = EigenVectorf::Zero(dimBytes);
			for (int c = 0; c < Ksel; ++c)
				nodes[nodeIdx].centroid += centersQ[c];
			nodes[nodeIdx].centroid /= (float)Ksel;
		} else {
			nodes[nodeIdx].centroidBytes.assign(dimBytes, 0);
			for (int b = 0; b < dimBytes; ++b) {
				int sum = 0;
				for (int c = 0; c < Ksel; ++c)
					sum += centersB[c][b];
				nodes[nodeIdx].centroidBytes[b] = ROUND2INT<uint8_t>((float)sum / (float)Ksel);
			}
		}
		nodes[nodeIdx].children.reserve(Ksel);
		for (int c = 0; c < Ksel; ++c) {
			if (sub[c].empty())
				continue;
			int ch = buildNode(getDesc, sub[c], depth + 1, rng);
			nodes[nodeIdx].children.push_back(ch);
		}
		if (nodes[nodeIdx].children.empty()) {
			nodes[nodeIdx].wordId = (int)leafNodeIdx.size();
			leafNodeIdx.push_back(nodeIdx);
		}
		return nodeIdx;
	}

	/*
	 * Greedy tree traversal to assign a descriptor to a single visual word.
	 * At each level, selects the closest child centroid (L2/Hamming) and descends.
	 * Returns the leaf `wordId`.
	 */
	int quantize(const uint8_t* d) const
	{
		int idx = 0;
		for (;;) {
			const Node& n = nodes[idx];
			if (n.children.empty())
				return n.wordId;
			int best = -1;
			float bd = std::numeric_limits<float>::max();
			int bdH = INT_MAX;
			for (int ch : n.children) {
				const Node& c = nodes[ch];
				if (dtype == QFLOAT) {
					float s = (Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f) - c.centroid).squaredNorm();
					if (s < bd) {
						bd = s;
						best = ch;
					}
				} else {
					int hd = hamming(d, c.centroidBytes.data(), dimBytes);
					if (hd < bdH) {
						bdH = hd;
						best = ch;
					}
				}
			}
			if (best < 0)
				return n.wordId >= 0 ? n.wordId : 0;
			idx = best;
		}
	}

	/*
	 * Soft Assignment Algorithm:
	 *  - Beam search maintains top-k candidates at each tree level (nodeIdx + cumulative dist).
	 *  - Prunes candidates to k×K per level for efficiency.
	 *  - Gaussian weighting: weight = exp(-distance²/(2σ²)), σ = median of final distances.
	 *  - Normalizes weights to sum to 1.0 across selected leaves.
	 * Returns k-best (wordId, weight) pairs.
	 */
	std::vector<std::pair<int, float>> quantizeSoft(const uint8_t* d, int k) const
	{
		if (k <= 0 || nodes.empty())
			return {};
		if (k == 1) {
			int w = quantize(d);
			return {{w, 1.0f}};
		}

		// Beam search: track (nodeIdx, cumulativeDistance) pairs
		struct Candidate {
			int nodeIdx;
			float dist;
			bool operator<(const Candidate& o) const { return dist > o.dist; } // min-heap
		};
		std::priority_queue<Candidate> beam;
		beam.push({0, 0.0f}); // Start at root with distance 0

		// Traverse tree level by level
		for (int level = 0; level < L; ++level) {
			std::priority_queue<Candidate> nextBeam;
			while (!beam.empty()) {
				Candidate curr = beam.top();
				beam.pop();
				// If leaf, add to next beam to be collected at the end
				const Node& n = nodes[curr.nodeIdx];
				if (n.children.empty()) {
					nextBeam.push(curr);
					continue;
				}

				// Expand children
				for (int ch : n.children) {
					const Node& c = nodes[ch];
					float childDist;
					if (dtype == QFLOAT) {
						childDist = (Eigen::Map<const EigenVectoru>(d, dimBytes).template cast<float>() * (1.f / 255.f) - c.centroid).squaredNorm();
					} else {
						childDist = (float)hamming(d, c.centroidBytes.data(), dimBytes);
					}
					nextBeam.push({ch, curr.dist + childDist});
				}
			}

			// Keep only top-k candidates for next level
			if ((int)nextBeam.size() > k * K) { // Prune aggressively
				std::priority_queue<Candidate> pruned;
				for (int i = 0; i < k * K && !nextBeam.empty(); ++i) {
					pruned.push(nextBeam.top());
					nextBeam.pop();
				}
				beam = std::move(pruned);
			} else {
				beam = std::move(nextBeam);
			}
		}

		// Collect k-best leaves
		std::vector<std::pair<int, float>> results;
		results.reserve(k);
		std::vector<float> distances;
		distances.reserve(k);
		while (!beam.empty() && (int)results.size() < k) {
			Candidate c = beam.top();
			beam.pop();
			const Node& n = nodes[c.nodeIdx];
			if (n.wordId >= 0) {
				results.push_back({n.wordId, c.dist});
				distances.push_back(c.dist);
			}
		}

		if (results.empty())
			return {};

		// Convert distances to weights using Gaussian kernel
		// sigma = median distance (robust to outliers)
		std::vector<float> distCopy = distances;
		std::nth_element(distCopy.begin(), distCopy.begin() + distCopy.size() / 2, distCopy.end());
		float sigma = MAXF(distCopy[distCopy.size() / 2], 1e-6f);
		float denom = 2.0f * sigma * sigma;

		float sumWeights = 0.0f;
		for (size_t i = 0; i < results.size(); ++i) {
			float weight = std::exp(-results[i].second / denom);
			results[i].second = weight;
			sumWeights += weight;
		}

		// Normalize weights to sum to 1.0
		if (sumWeights > 0.0f) {
			for (auto& [w, weight] : results)
				weight /= sumWeights;
		}

		return results;
	}

	/*
	 * Description:
	 *  Builds the scene-dependent inverted index (postings per visual word) and TF-IDF
	 *  normalization terms used during retrieval. Applies the same descriptor sampling
	 *  policy as training for consistency and optionally uses soft-assignment when enabled.
	 *
	 * What it does:
	 *  - Iterates images and collects per-image term frequencies (TF) for visual words.
	 *  - Computes document frequency (DF) per word and IDF = log(N/DF).
	 *  - Stores postings as (imageId, tf) with tf being float (hard=1.0, soft=sum(weights)).
	 *  - Precomputes per-image L2 norm of TF-IDF vector to enable cosine similarity.
	 *
	 * Technical Details:
	 *  - Burstiness handling: uses sqrt(TF) weighting (Jégou 2009) when forming TF-IDF norms.
	 *  - Sampling: uses `SelectTopKeypoints(maxDescriptorsPerImage)` when limit > 0, otherwise all.
	 *  - Soft assignment: if `softAssignmentK>0`, each descriptor contributes to multiple words
	 *    with Gaussian weights from `quantizeSoft()`; otherwise hard assignment to single word.
	 */
	void buildDatabase(const Scene& scene)
	{
		numImages = (uint32_t)scene.images.size();
		const int nWords = (int)leafNodeIdx.size();
		postings.assign(nWords, {});
		imageNorm.assign(numImages, 0.f);

		std::vector<std::unordered_map<int, float>> imgTF(numImages);
		std::vector<uint32_t> df(nWords, 0);
		#ifdef VOCABTREE_USE_OPENMP
		#pragma omp parallel for schedule(dynamic)
		for (int_t _i = 0; _i < (int_t)numImages; ++_i) {
			const Image& img = scene.images[_i];
		#else
		for (const Image& img : scene.images) {
		#endif
			if (img.descriptors.empty())
				continue;
			auto& tfm = imgTF[img.ID];
			// Apply consistent descriptor sampling (same strategy as Build)
			const cv::Mat& descriptors = getDescriptors(img);
			for (int idx = 0; idx < descriptors.rows; ++idx) {
				const uint8_t* d = descriptors.ptr<uint8_t>(idx);
				if (softAssignmentK > 0) {
					auto words = quantizeSoft(d, softAssignmentK);
					for (const auto& kv : words)
						tfm[kv.first] += kv.second;
				} else {
					int w = quantize(d);
					tfm[w] += 1.f;
				}
			}
			// Increment document frequency for each word present in this image
			#ifdef VOCABTREE_USE_OPENMP
			#pragma omp critical
			#endif
			{
				for (const auto& kv : tfm)
					df[kv.first]++;
			}
		}

		// Standard IDF: log(N/df) with smoothing to avoid division by zero
		// and log(0) for words that appear in all images.
		// Words with df=0 get idf=0 (won't contribute to scoring)
		idf.assign(nWords, 0.f);
		for (int w = 0; w < nWords; ++w)
			if (df[w] > 0)
				idf[w] = LOGN((float)numImages / (float)df[w]);
		for (uint32_t i = 0; i < numImages; ++i) {
			float norm2 = 0.f;
			for (const auto& kv : imgTF[i]) {
				int w = kv.first;
				float tf = kv.second;
				postings[w].emplace_back(i, tf);
				// Do not square TF to reduce burstiness from repetitive features
				norm2 += tf * SQUARE(idf[w]);
			}
			imageNorm[i] = SQRT(MAXF(norm2, 1e-12f));
		}
	}
};
/*----------------------------------------------------------------*/


VocabularyTree::VocabularyTree() : pImpl(nullptr) {}
VocabularyTree::~VocabularyTree() { Release(); }

void VocabularyTree::Release()
{
	if (pImpl) {
		ClearDescriptorsCache();
		delete pImpl;
		pImpl = nullptr;
	}
}

bool VocabularyTree::Build(const Scene& scene, const Config& cfg, const String& vocabFile)
{
	/*
	 * Description:
	 *  Trains a new vocabulary tree from the scene's descriptors or loads a pre-trained
	 *  tree from disk, then builds the scene-specific inverted index. Applies consistent
	 *  descriptor sampling across training and indexing.
	 *
	 * Main blocks:
	 *  - Pre-trained path: Load topology + centroids + IDF and `Index(scene)`.
	 *  - Training path: Copy config, collect descriptors with sampling, build tree via
	 *    recursive K-means, then compute DB via `buildDatabase(scene)`.
	 */
	// If a pre-trained vocabulary file is provided, load it first and then index the scene.
	if (!vocabFile.empty() && File::isFile(vocabFile)) {
		if (!Load(vocabFile)) {
			VERBOSE("VocabularyTree: failed to load '%s'", vocabFile.c_str());
			return false;
		}
		// Build scene-specific inverted index and TF-IDF using the loaded tree
		return Index(scene);
	}

	// Create Impl and initialize from config for training a new vocabulary
	if (!pImpl)
		pImpl = new Impl();
	Impl& I = *pImpl;
	I.K = cfg.K;
	I.L = cfg.L;
	I.maxKMeansIters = MAXF(1u, cfg.maxKMeansIters);
	I.seed = cfg.randomSeed;
	I.dtype = cfg.descriptorsAreBinary ? Impl::BINARY : Impl::QFLOAT;
	I.softAssignmentK = cfg.softAssignmentK;
	I.queryExpansionImages = cfg.queryExpansionImages;
	I.maxDescriptorsPerImage = cfg.maxDescriptorsPerImage;

	// Collect descriptors (training pool) with consistent sampling policy
	std::vector<const uint8_t*> pool;
	pool.reserve(scene.images.size() * MAXF(1000u, I.maxDescriptorsPerImage));
	I.dimBytes = 0;
	for (const Image& img : scene.images) {
		if (img.descriptors.empty())
			continue;
		ASSERT(img.descriptors.type() == CV_8U);
		if (I.dimBytes == 0)
			I.dimBytes = img.descriptors.cols;
		ASSERT(I.dimBytes == img.descriptors.cols);
		// Apply top-N filtering if configured using grid-based selection
		const cv::Mat& descriptors = I.getDescriptors(img);
		const int rows = descriptors.rows;
		for (int r = 0; r < rows; ++r)
			pool.push_back(descriptors.ptr<uint8_t>(r));
	}
	if (pool.empty()) {
		VERBOSE("VocabularyTree: no descriptors in scene");
		return false;
	}
	I.nodes.reserve(1024);
	I.leafNodeIdx.clear();
	std::vector<int> indices(pool.size());
	std::iota(indices.begin(), indices.end(), 0);
	auto accessor = [&](int idx) -> const uint8_t* { return pool[idx]; };
	std::mt19937 rng(I.seed);
	I.buildNode(accessor, indices, 0, rng);
	I.buildDatabase(scene);
	return true;
}

std::vector<std::pair<uint32_t, float>> VocabularyTree::Query(const Image& image, unsigned maxResults, float minScore) const
{
	/*
	 * Description:
	 *  Computes the query BoW vector (hard or soft TF), applies sqrt(TF)*IDF weighting,
	 *  and retrieves similar images via inverted index with cosine similarity. Optionally
	 *  performs a query expansion re-query using top-N results.
	 *
	 * Technical Details:
	 *  - Sampling: applies `SelectTopKeypoints()` when descriptor limit is set.
	 *  - Soft assignment: uses `quantizeSoft()` to distribute TF across k words.
	 *  - Weighting: sqrt(TF) * IDF, cosine similarity normalization.
	 *  - Query Expansion Strategy:
	 *      * Weighted by rank: weight = 1.0 / (i + 2) for result i
	 *      * Rebuilds query vector with expanded TF counts
	 *      * Second query pass with expanded representation
	 */
	std::vector<std::pair<uint32_t, float>> outEmpty;
	if (!pImpl || image.descriptors.empty())
		return outEmpty;
	const Impl& I = *pImpl;
	const int nWords = (int)I.leafNodeIdx.size();
	if (nWords == 0)
		return outEmpty;

	// Apply consistent descriptor sampling (same strategy as Build and buildDatabase)
	const cv::Mat& descriptors = I.getDescriptors(image);
	const int rows = descriptors.rows;

	std::unordered_map<int, float> qTF;
	if (I.softAssignmentK > 0) {
		// Soft assignment: each descriptor contributes to k-best visual words
		for (int idx = 0; idx < rows; ++idx) {
			const uint8_t* d = descriptors.ptr<uint8_t>(idx);
			auto words = I.quantizeSoft(d, I.softAssignmentK);
			for (auto& [w, weight] : words) {
				auto it = qTF.find(w);
				if (it == qTF.end())
					qTF.emplace(w, weight);
				else
					it->second += weight;
			}
		}
	} else {
		// Hard assignment: each descriptor maps to single best visual word
		std::unordered_map<int, uint32_t> qTFcount;
		for (int idx = 0; idx < rows; ++idx) {
			const uint8_t* d = descriptors.ptr<uint8_t>(idx);
			int w = I.quantize(d);
			auto it = qTFcount.find(w);
			if (it == qTFcount.end())
				qTFcount.emplace(w, 1);
			else
				++it->second;
		}
		// Convert to float TF for consistency with soft assignment path
		for (auto& [w, count] : qTFcount)
			qTF.emplace(w, (float)count);
	}
	if (qTF.empty())
		return outEmpty;

	// Compute query vector with sqrt(TF) * IDF weighting (consistent with database)
	float qnorm2 = 0.f;
	std::unordered_map<int, float> qW;
	qW.reserve(qTF.size());
	for (auto& kv : qTF) {
		int w = kv.first;
		// Use sqrt(TF) weighting to match database indexing (burstiness reduction)
		float wgt = SQRT(kv.second) * (w < (int)I.idf.size() ? I.idf[w] : 0.f);
		if (wgt > 0.f) {
			qW.emplace(w, wgt);
			qnorm2 += wgt * wgt;
		}
	}
	float qnorm = SQRT(MAXF(qnorm2, 1e-12f));

	// Accumulate scores for candidate images using inverted index
	std::unordered_map<uint32_t, float> acc;
	acc.reserve(qW.size() * 8);
	for (auto& kv : qW) {
		int w = kv.first;
		float qw = kv.second;
		if (w < 0 || w >= (int)I.postings.size())
			continue;
		for (auto& p : I.postings[w]) {
			uint32_t img = p.first;
			// Database uses sqrt(TF) * IDF for image weights
			float iw = SQRT((float)p.second) * I.idf[w];
			acc[img] += qw * iw;
		}
	}

	// Compute final cosine similarity scores
	std::vector<std::pair<uint32_t, float>> out;
	out.reserve(acc.size());
	for (auto& kv : acc) {
		uint32_t img = kv.first;
		float dot = kv.second;
		float denom = qnorm * (img < I.imageNorm.size() ? I.imageNorm[img] : 1.f);
		float s = (denom > 0.f) ? (dot / denom) : 0.f;
		if (s >= minScore)
			out.emplace_back(img, s);
	}
	std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.second > b.second; });

	// Query expansion: average top-N results with query and re-query
	if (I.queryExpansionImages > 0 && out.size() > I.queryExpansionImages) {
		// Build expanded query by averaging top-N BoW vectors with original query
		std::unordered_map<int, float> expandedTF = qTF;
		for (unsigned i = 0; i < I.queryExpansionImages && i < out.size(); ++i) {
			uint32_t imgID = out[i].first;
			// Weight by rank (top result gets full weight, decreases linearly)
			float rankWeight = 1.0f / (float)(i + 2);
			// Add this image's BoW representation to expanded query
			for (int w = 0; w < (int)I.postings.size(); ++w) {
				for (auto& p : I.postings[w]) {
					if (p.first == imgID) {
						auto it = expandedTF.find(w);
						if (it == expandedTF.end())
							expandedTF.emplace(w, (float)p.second * rankWeight);
						else
							it->second += (float)p.second * rankWeight;
						break;
					}
				}
			}
		}

		// Re-query with expanded vector
		qW.clear();
		qnorm2 = 0.f;
		for (auto& kv : expandedTF) {
			int w = kv.first;
			float wgt = SQRT(kv.second) * (w < (int)I.idf.size() ? I.idf[w] : 0.f);
			if (wgt > 0.f) {
				qW.emplace(w, wgt);
				qnorm2 += wgt * wgt;
			}
		}
		qnorm = SQRT(MAXF(qnorm2, 1e-12f));

		acc.clear();
		for (auto& kv : qW) {
			int w = kv.first;
			float qw = kv.second;
			if (w < 0 || w >= (int)I.postings.size())
				continue;
			for (auto& p : I.postings[w]) {
				uint32_t img = p.first;
				float iw = SQRT((float)p.second) * I.idf[w];
				acc[img] += qw * iw;
			}
		}

		out.clear();
		out.reserve(acc.size());
		for (auto& kv : acc) {
			uint32_t img = kv.first;
			float dot = kv.second;
			float denom = qnorm * (img < I.imageNorm.size() ? I.imageNorm[img] : 1.f);
			float s = (denom > 0.f) ? (dot / denom) : 0.f;
			if (s >= minScore)
				out.emplace_back(img, s);
		}
		std::sort(out.begin(), out.end(), [](auto& a, auto& b) { return a.second > b.second; });
	}

	if (out.size() > maxResults)
		out.resize(maxResults);
	return out;
}

namespace {
struct VocabArchive
{
	int K, L, dimBytes, dtype;
	int softAssignmentK;
	unsigned queryExpansionImages;
	unsigned maxDescriptorsPerImage;
	std::vector<uint32_t> nodeChildOffsets;
	std::vector<int> nodeChildren;
	std::vector<int> nodeWordId;
	std::vector<float> centroidsQ;
	std::vector<uint8_t> centroidsB;
	std::vector<float> idf;
};
} // namespace

bool VocabularyTree::Save(const String& path) const
{
	if (!pImpl)
		return false;
	const Impl& I = *pImpl;
	VocabArchive A;
	A.K = I.K;
	A.L = I.L;
	A.dimBytes = I.dimBytes;
	A.dtype = (int)I.dtype;
	A.softAssignmentK = I.softAssignmentK;
	A.queryExpansionImages = I.queryExpansionImages;
	A.maxDescriptorsPerImage = I.maxDescriptorsPerImage;
	const size_t N = I.nodes.size();
	A.nodeChildOffsets.resize(N + 1, 0);
	for (size_t i = 0; i < N; ++i)
		A.nodeChildOffsets[i + 1] = A.nodeChildOffsets[i] + (uint32_t)I.nodes[i].children.size();
	A.nodeChildren.reserve(A.nodeChildOffsets.back());
	A.nodeWordId.resize(N, -1);
	for (size_t i = 0; i < N; ++i) {
		A.nodeChildren.insert(A.nodeChildren.end(), I.nodes[i].children.begin(), I.nodes[i].children.end());
		A.nodeWordId[i] = I.nodes[i].wordId;
		if (I.dtype == Impl::QFLOAT) {
			A.centroidsQ.insert(A.centroidsQ.end(), I.nodes[i].centroid.data(), I.nodes[i].centroid.data() + I.nodes[i].centroid.size());
		} else {
			A.centroidsB.insert(A.centroidsB.end(), I.nodes[i].centroidBytes.begin(), I.nodes[i].centroidBytes.end());
		}
	}
	A.idf = I.idf;
	std::ofstream fs(path.c_str(), std::ios::binary);
	if (!fs)
		return false;
	auto writeVec = [&](auto& v) { uint64_t n=v.size(); fs.write((char*)&n,sizeof(n)); if(n) fs.write((char*)v.data(), sizeof(v[0])*n); };
	fs.write((char*)&A.K, sizeof(A.K));
	fs.write((char*)&A.L, sizeof(A.L));
	fs.write((char*)&A.dimBytes, sizeof(A.dimBytes));
	fs.write((char*)&A.dtype, sizeof(A.dtype));
	fs.write((char*)&A.softAssignmentK, sizeof(A.softAssignmentK));
	fs.write((char*)&A.queryExpansionImages, sizeof(A.queryExpansionImages));
	fs.write((char*)&A.maxDescriptorsPerImage, sizeof(A.maxDescriptorsPerImage));
	writeVec(A.nodeChildOffsets);
	writeVec(A.nodeChildren);
	writeVec(A.nodeWordId);
	writeVec(A.centroidsQ);
	writeVec(A.centroidsB);
	writeVec(A.idf);
	return (bool)fs;
}

bool VocabularyTree::Load(const String& path)
{
	Release();
	pImpl = new Impl();
	Impl& I = *pImpl;
	VocabArchive A;
	std::ifstream fs(path.c_str(), std::ios::binary);
	if (!fs)
		return false;
	auto readVec = [&](auto& v) { uint64_t n=0; fs.read((char*)&n,sizeof(n)); v.resize(n); if(n) fs.read((char*)v.data(), sizeof(v[0])*n); };
	fs.read((char*)&A.K, sizeof(A.K));
	fs.read((char*)&A.L, sizeof(A.L));
	fs.read((char*)&A.dimBytes, sizeof(A.dimBytes));
	fs.read((char*)&A.dtype, sizeof(A.dtype));
	fs.read((char*)&A.softAssignmentK, sizeof(A.softAssignmentK));
	fs.read((char*)&A.queryExpansionImages, sizeof(A.queryExpansionImages));
	fs.read((char*)&A.maxDescriptorsPerImage, sizeof(A.maxDescriptorsPerImage));
	readVec(A.nodeChildOffsets);
	readVec(A.nodeChildren);
	readVec(A.nodeWordId);
	readVec(A.centroidsQ);
	readVec(A.centroidsB);
	readVec(A.idf);
	I.K = A.K;
	I.L = A.L;
	I.dimBytes = A.dimBytes;
	I.dtype = (A.dtype == 0 ? Impl::BINARY : Impl::QFLOAT);
	I.softAssignmentK = A.softAssignmentK;
	I.queryExpansionImages = A.queryExpansionImages;
	I.maxDescriptorsPerImage = A.maxDescriptorsPerImage;
	I.nodes.clear();
	I.leafNodeIdx.clear();
	const size_t N = A.nodeWordId.size();
	I.nodes.resize(N);
	size_t offQ = 0, offB = 0;
	for (size_t i = 0; i < N; ++i) {
		Impl::Node& n = I.nodes[i];
		n.wordId = A.nodeWordId[i];
		uint32_t b = A.nodeChildOffsets[i], e = A.nodeChildOffsets[i + 1];
		n.children.assign(A.nodeChildren.begin() + b, A.nodeChildren.begin() + e);
		if (I.dtype == Impl::QFLOAT) {
			n.centroid.resize(I.dimBytes);
			if (!A.centroidsQ.empty())
				std::memcpy(n.centroid.data(), A.centroidsQ.data() + offQ, sizeof(float) * I.dimBytes);
			offQ += I.dimBytes;
		} else {
			n.centroidBytes.assign(I.dimBytes, 0);
			if (!A.centroidsB.empty())
				std::memcpy(n.centroidBytes.data(), A.centroidsB.data() + offB, I.dimBytes);
			offB += I.dimBytes;
		}
		if (n.wordId >= 0) {
			if ((int)I.leafNodeIdx.size() <= n.wordId)
				I.leafNodeIdx.resize(n.wordId + 1, -1);
			I.leafNodeIdx[n.wordId] = (int)i;
		}
	}
	I.idf = std::move(A.idf);
	I.postings.clear();
	I.imageNorm.clear();
	I.numImages = 0; // scene dependent
	return true;
}

bool VocabularyTree::Index(const Scene& scene)
{
	// Build scene-specific inverted index and TF-IDF using the loaded tree
	ASSERT(pImpl);
	pImpl->buildDatabase(scene);
	return true;
}

const cv::Mat& VocabularyTree::GetTopDescriptors(const Image& image) const
{
	// Returns the (cached) top-N descriptors for an image
	ASSERT(pImpl);
	return pImpl->getDescriptors(image);
}

void VocabularyTree::ClearDescriptorsCache()
{
	if (pImpl) {
		std::lock_guard<std::mutex> lock(pImpl->cacheMutex);
		// Report how well the cache did before forgetting it
		REPORT_CACHE_HIT_STATS(pImpl->hitStats, "Descriptors");
		pImpl->hitStats.Reset();
		pImpl->descriptorsCache.clear();
	}
}

unsigned VocabularyTree::GetMaxDescriptors() const
{
	ASSERT(pImpl);
	return pImpl->maxDescriptorsPerImage;
}
/*----------------------------------------------------------------*/
