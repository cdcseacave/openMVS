/*
 * FeaturesExtractor.h
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

#ifndef _SFM_FEATURESEXTRACTOR_H_
#define _SFM_FEATURESEXTRACTOR_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Image;
class SFM_API Scene;

enum class FeatureType : uint8_t {
	NONE = 0,
	AKAZE = 1,
	ORB = 2,
	SIFT = 3,
	SIFTGPU = 4
};


inline bool IsBinaryDescriptor(FeatureType type) {
	return (type == FeatureType::AKAZE || type == FeatureType::ORB);
}

inline String FeatureTypeToString(FeatureType type) {
	switch (type) {
		case FeatureType::AKAZE: return "AKAZE";
		case FeatureType::ORB: return "ORB";
		case FeatureType::SIFT: return "SIFT";
		case FeatureType::SIFTGPU: return "SIFTGPU";
		default: return "NONE";
	}
}

inline FeatureType FeatureTypeFromString(const String& str) {
	if (str == "AKAZE") return FeatureType::AKAZE;
	if (str == "ORB") return FeatureType::ORB;
	if (str == "SIFT") return FeatureType::SIFT;
	if (str == "SIFTGPU") return FeatureType::SIFTGPU;
	return FeatureType::NONE;
}
/*----------------------------------------------------------------*/


/**
 * @brief Configuration for feature extraction
 */
struct SFM_API FeatureExtractionConfig {
	FeatureType detectorType = FeatureType::AKAZE; // feature detector: AKAZE/ORB/SIFT/SIFTGPU
	int maxFeaturesPerCell = 3000; // maximum features per grid cell (3x3 grid)
	int minFeaturesPerCell = 500;  // minimum features per cell before adjusting sensitivity
	bool releaseImagePixels = true; // release image pixel data after feature extraction to save memory
	bool useCUDA = true; // use CUDA for SiftGPU if available (otherwise OpenGL)
	String importOpenMVGDir; // directory to import OpenMVG features from (optional)
	String exportOpenMVGDir; // directory to export OpenMVG features to (optional)

	// Spherical / cube-map feature extraction (applied per-image when the
	// image's camera is a SphericalCamera). See SphereCubeMap for the set
	// of supported face counts.
	int   cubemapFaces = 6;             // 4 | 6 | 8 | 12 | 20
	int   cubemapFaceSize = 0;          // 0 = auto: max(1024, equirect_width/4)
	float cubemapDedupAngleDeg = 0.25f; // angular-NMS threshold across face seams

	int GetMaxNumFeatures() const {
		// Total max features per image = maxFeaturesPerCell * 3 * 3 grid
		return maxFeaturesPerCell * 3 * 3; // 3x3 grid
	}
	void SetMaxNumFeatures(int maxNumFeatures) {
		maxFeaturesPerCell = maxNumFeatures / 9; // 3x3 grid
		if (maxFeaturesPerCell < minFeaturesPerCell)
			minFeaturesPerCell = maxFeaturesPerCell;
	}
};

/**
 * @brief Feature extraction class for images and scenes
 *
 * Handles extraction of keypoints and descriptors from images using various
 * feature detectors (AKAZE, ORB, SIFT). Uses a 3x3 grid-based extraction
 * strategy to ensure spatially distributed features.
 */
class SFM_API FeaturesExtractor
{
public:
	FeaturesExtractor(Scene& _scene, const FeatureExtractionConfig& _config);
	~FeaturesExtractor();

	// Access scene
	const Scene& GetScene() const { return scene; }
	Scene& GetScene() { return scene; }

	// Access configuration
	const FeatureExtractionConfig& GetConfig() const { return config; }
	FeatureExtractionConfig& GetConfig() { return config; }

	/**
	 * @brief Extract features from all images in the scene
	 * @return Number of total features extracted across all images
	 */
	size_t Extract();

	/**
	 * @brief Extract features from a single image
	 * @param image Image to extract features from
	 * @param detector Feature detector to use (nullptr to create internally)
	 * @param skipIO If true, skip OpenMVG import/export (used for internal calls on cube-map faces)
	 * @return true if features were extracted successfully
	 */
	bool ExtractImage(Image& image, cv::Ptr<cv::Feature2D>& detector, bool skipIO = false);

	/**
	 * @brief Convert SIFT descriptors to RootSIFT and quantize to uint8_t
	 * @param siftDesc Input CV_32F SIFT descriptors (rows x 128)
	 * @return CV_8U RootSIFT descriptors (rows x 128)
	 */
	static cv::Mat ConvertToRootSIFT(const cv::Mat& siftDesc);

	/**
	 * @brief Export keypoints and descriptors to OpenMVG feature/descriptor files.
	 * @param outputDir destination directory
	 * @param image source image containing keypoints/descriptors
	 * @return true on success, false otherwise
	 */
	static bool ExportFeaturesOpenMVG(const String& outputDir, const Image& image);

	/**
	 * @brief Import keypoints and descriptors from OpenMVG feature/descriptor files.
	 * @param inputDir source directory containing <stem>.feat and optional <stem>.desc
	 * @param image destination image to populate
	 * @return true on success, false otherwise
	 */
	static bool ImportFeaturesOpenMVG(const String& inputDir, Image& image);

private:
	// Spherical path: render N tangent faces via SphereCubeMap::SphericalToTangentialFaces,
	// recursively run the pinhole ExtractImage on each face with skipIO=true,
	// then reproject + angular-NMS-dedup back to equirectangular coordinates.
	bool ExtractImageSpherical(Image& image, cv::Ptr<cv::Feature2D>& detector);

	Scene& scene;
	FeatureExtractionConfig config;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_FEATURESEXTRACTOR_H_
