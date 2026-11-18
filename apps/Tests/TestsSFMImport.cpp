/*
 * TestsSFMImport.cpp
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

#include "../../libs/SFM.h"
#include "../../libs/SFM/GlobalRotationAveraging.h"
#include "../../libs/SFM/GlobalScaleAveraging.h"
#include "../../libs/SFM/GlobalTranslationAveraging.h"
#include "../../libs/Math/DisjointSet.h"
#include "../../libs/SFM/RobustAveraging.h"
#include "../../libs/SFM/PairsWeighting.h"
#include "../../libs/SFM/ViewGraphCalibrator.h"
#include "../../libs/SFM/BundleAdjustment.h"
#include "../../libs/SFM/SceneCluster.h"
#include "../../libs/SFM/SceneBlocks.h"
#include "../../libs/SFM/GlobalAlignment.h"
#include "../../libs/SFM/PoseLink.h"
#include "../../libs/SFM/Link2D3D.h"
#include "../../libs/SFM/Triangulation.h"
#include "../../libs/SFM/MatchGeometric.h"
#include "../../libs/SFM/LensFit.h"
#include "../../libs/SFM/RoMa2Matcher.h"
#include "../../libs/SFM/FlexGlueMatcher.h"
#include "../../libs/SFM/SphereCubeMap.h"
#include "../../libs/SFM/InterfaceMVS.h"
#include "../../libs/MVS.h"
#include <nlohmann/json.hpp>
#include "Tests.h"
#include "TestsSFMFramework.h"
#include <filesystem>
#include <chrono>
#include <random>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

#ifdef _IMAGE_HEIF
// HEIF/HEIC integration at the SFM layer; see the declaration for the coverage list.
// There are no HEIF-only fixtures: two of the four pipeline images are HEIC, so RealImagesPipelineTest
// and the MVS PipelineTest decode them for real on every run -- which is what covers pixel
// content as a whole, far more sharply than a mean-absolute-difference check could:
//  - 00001.heic: decodes 640x479, no container rotation, carrying a genuine fully opaque alpha
//                channel like a real iPhone photo -- locks the alpha-instead-of-luminance bug
//  - 00002.heic: stored landscape but carrying a container 'irot' for 90deg CCW, so it DECODES
//                portrait 479x640, AND a stored EXIF Orientation=8 naming the same rotation --
//                locks the "don't rotate twice" guard. SFM turns it back into the landscape
//                working raster (View::ToWorkingOrientation rotates 90deg CW), so the whole
//                rotate-back path runs inside RealImagesPipelineTest. MVS has no EXIF-rotation concept,
//                so scene.mvs gives this one image its own portrait camera (K with fx/fy swapped
//                and the principal point mapped by (cx,cy) -> (cy, W-1-cx), rotation Rz(-90)
//                relative to the platform), which describes exactly the same rays.
// Both carry ExifIFD FocalLengthIn35mmFilm=39, grafted verbatim from the JPGs they replaced, and
// a synthetic GPS fix, so the container EXIF blob is covered for IFD0, ExifIFD and the GPS IFD.
bool HEIFMetadataTest()
{
	TD_TIMER_START();

	const String pathAlpha = MAKE_PATH("images/00001.heic");
	const String pathRotated = MAKE_PATH("images/00002.heic");
	const String pathJpg = MAKE_PATH("images/00000.jpg");

	// 1) The MVS-layer read must deliver exactly the resolution scene.mvs pairs each image with.
	// MVS derives width/height from the decoded pixels (Image::ReloadImage -> ResizeImage), so a
	// decode that padded 479 up to an even height, or applied the rotation to the wrong one of
	// the two, would silently pair the wrong camera with the image rather than fail.
	{
		const std::pair<const String&, cv::Size> expected[] = {
			{pathAlpha, cv::Size(640, 479)},   // camera 0, landscape
			{pathRotated, cv::Size(479, 640)}, // camera 1, the portrait 'rotated' camera
		};
		for (const auto& [path, size] : expected) {
			Image8U3 heicImg;
			if (MVS::Image::ReadImage(path, heicImg) == NULL) {
				VERBOSE("ERROR: HEIFMetadataTest: failed to read '%s'", path.c_str());
				return false;
			}
			if (heicImg.cols != size.width || heicImg.rows != size.height) {
				VERBOSE("ERROR: HEIFMetadataTest: '%s' decoded %dx%d, expected %dx%d (the "
					"resolution scene.mvs pairs it with)", path.c_str(), heicImg.cols, heicImg.rows,
					size.width, size.height);
				return false;
			}
		}
	}

	// 2) EXIF bridge + the orientation double-rotation guard.
	{
		SFM::Image imgAlpha(0, pathAlpha), imgRotated(1, pathRotated), imgJpg(2, pathJpg);
		if (!imgAlpha.LoadMetadata() || !imgRotated.LoadMetadata() || !imgJpg.LoadMetadata()) {
			VERBOSE("ERROR: HEIFMetadataTest: LoadMetadata failed on one of the images");
			return false;
		}
		if (!imgAlpha.HasCamera() || !imgJpg.HasCamera()) {
			VERBOSE("ERROR: HEIFMetadataTest: expected a valid camera on both 00001.heic and 00000.jpg");
			return false;
		}
		// 00001.heic was re-encoded from a sibling house JPG, so it has the same 640x479 raster and
		// the same grafted FocalLengthIn35mmFilm=39, and neither carries FocalPlaneResolution
		// tags: both take the identical 35mm-equivalent ladder branch on identical dimensions, so
		// the focal in pixels must come out *equal*, not merely close. Any drift here means the
		// container EXIF bridge disagrees with the classic stream scan.
		const REAL focalHeic = imgAlpha.pCamera->GetFocalLength();
		const REAL focalJpg = imgJpg.pCamera->GetFocalLength();
		if (!ISEQUAL(focalHeic, focalJpg)) {
			VERBOSE("ERROR: HEIFMetadataTest: focal mismatch between the container EXIF bridge and "
				"the stream scan: HEIC %.6f vs JPG %.6f", focalHeic, focalJpg);
			return false;
		}
		// The whole point of 00002.heic: libheif already applied the container 'irot' at decode
		// time (so ReadHeader reports 479x640) AND the file carries an EXIF Orientation=8 for the
		// same rotation. Honoring the tag on top would rotate the pixels a second time, desyncing
		// the derived 'rotated' flag from the actual pixel layout -- which feeds the known-poses
		// rotated-image handling, a silent pose-import breaker, not a cosmetic bug.
		if (imgRotated.metadata.orientation != 1) {
			VERBOSE("ERROR: HEIFMetadataTest: 00002.heic orientation not normalized: expected 1, got %u",
				(unsigned)imgRotated.metadata.orientation);
			return false;
		}
		// 'rotated' must match the decoded pixel dimensions: 00002.heic decodes portrait so it must
		// be true, 00001.heic decodes landscape so it must be false. Note SFM::Image declares its
		// own nested 'metadata' (holding 'orientation') which shadows View::metadata (holding
		// 'rotated'), hence the explicit qualification.
		if (!imgRotated.View::metadata.rotated || imgAlpha.View::metadata.rotated) {
			VERBOSE("ERROR: HEIFMetadataTest: rotated flags wrong: 00002.heic=%d (expected 1), "
				"00001.heic=%d (expected 0)",
				(int)imgRotated.View::metadata.rotated, (int)imgAlpha.View::metadata.rotated);
			return false;
		}
		// GPS EXIF path, i.e. the GPS sub-IFD surviving the container blob: both HEICs carry
		// synthetic but well-formed coordinates (the house JPGs have no GPS tags at all to compare
		// against), so only presence is checked.
		if (!imgRotated.View::metadata.HasGPS() || !imgAlpha.View::metadata.HasGPS()) {
			VERBOSE("ERROR: HEIFMetadataTest: GPS metadata not parsed (00001.heic=%d, 00002.heic=%d)",
				(int)imgAlpha.View::metadata.HasGPS(), (int)imgRotated.View::metadata.HasGPS());
			return false;
		}
	}

	// 3) The LoadPixels fallback: cv::imread has no HEIF codec, so this exercises the CImage
	// branch of the IO LoadImage(), in color and then in the gray mode feature extraction uses.
	// Done on the rotated image, so the rotate-back is covered too: the file decodes portrait
	// 479x640 and LoadPixels must hand back the landscape 640x479 working raster.
	{
		SFM::Image img(0, pathRotated);
		if (!img.LoadMetadata() || !img.LoadPixels() || !img.HasPixels()) {
			VERBOSE("ERROR: HEIFMetadataTest: LoadPixels failed for '%s'", pathRotated.c_str());
			return false;
		}
		if (img.pixels.cols != 640 || img.pixels.rows != 479) {
			VERBOSE("ERROR: HEIFMetadataTest: 00002.heic was not rotated back to the landscape "
				"working raster: got %dx%d, expected 640x479", img.pixels.cols, img.pixels.rows);
			return false;
		}
		// LoadPixels applies ToWorkingOrientation, so compare against the metadata-derived
		// working dims rather than a hard-coded size
		if (img.pixels.cols != img.GetWidth() || img.pixels.rows != img.GetHeight()) {
			VERBOSE("ERROR: HEIFMetadataTest: LoadPixels size mismatch: expected %dx%d, got %dx%d",
				img.GetWidth(), img.GetHeight(), img.pixels.cols, img.pixels.rows);
			return false;
		}
	}
	{
		// The alpha regression, end to end: real iPhone HEIFs carry an alpha channel, and a
		// reader advertising a 32-bit format sends this gray load through FilterFormat's
		// 32-bit->gray case, which copies the *alpha* byte instead of a luminance of R,G,B.
		// Alpha is constant on real photos, so the buffer came out flat -- 0 SIFT features on
		// every image, with correct dimensions and no error logged anywhere. Only a check on
		// the *spread* of the content catches it, which is why this asserts a stddev. Since
		// 00001.heic is a pipeline image, RealImagesPipelineTest gray-loads it on every run too.
		SFM::Image img(0, pathAlpha);
		if (!img.LoadMetadata() || !img.LoadPixels(true)) {
			VERBOSE("ERROR: HEIFMetadataTest: LoadPixels(gray) failed for '%s'", pathAlpha.c_str());
			return false;
		}
		if (img.pixels.channels() != 1) {
			VERBOSE("ERROR: HEIFMetadataTest: gray load of 00001.heic has %d channels, expected 1",
				img.pixels.channels());
			return false;
		}
		cv::Scalar mu, sigma;
		cv::meanStdDev(img.pixels, mu, sigma);
		// Measured 62.6 on this image; the bug yields a constant buffer (stddev ~= 0), so a
		// threshold far below the measured value is still decisive and cannot go flaky on
		// lossy-codec drift across libheif/vcpkg versions.
		constexpr double minGrayStdDev = 10.0;
		VERBOSE("HEIFMetadataTest: 00001.heic gray-load stddev = %.3f (threshold > %.1f)",
			sigma.val[0], minGrayStdDev);
		if (sigma.val[0] <= minGrayStdDev) {
			VERBOSE("ERROR: HEIFMetadataTest: gray load of 00001.heic looks constant "
				"(alpha-copy regression): stddev %.3f <= %.1f", sigma.val[0], minGrayStdDev);
			return false;
		}
	}

	VERBOSE("HEIFMetadataTest: All tests passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
} // HEIFMetadataTest
/*----------------------------------------------------------------*/
#endif // _IMAGE_HEIF

// Every field Scene::Import's camera clustering (step 2c) builds its key from, in one string, so
// that two imports grouping the images differently show up as a plain textual difference naming
// the image that moved.
static String ImportCameraSignature(const Camera& camera)
{
	String signature = CameraTypeToString(camera.GetType()) +
		String::FormatString("|%dx%d", camera.GetWidth(), camera.GetHeight());
	if (camera.GetType() == CameraType::PINHOLE) {
		const PinholeCamera& pinhole = static_cast<const PinholeCamera&>(camera);
		signature += String::FormatString("|%.9g|%.9g|%.9g|%.9g", pinhole.fx, pinhole.fy, pinhole.cx, pinhole.cy);
		signature += String::FormatString("|%.9g|%.9g|%.9g|%.9g|%.9g|%.9g", pinhole.k1, pinhole.k2, pinhole.k3, pinhole.p1, pinhole.p2, pinhole.k4);
		signature += String::FormatString("|%.9g|%.9g", pinhole.k5, pinhole.k6);
	}
	signature += camera.TrustIntrinsics() ? "|trusted" : "|untrusted";
	signature += "|" + camera.metadata.name + "|" + camera.metadata.model;
	signature += String::FormatString("|%.9g|%.9g", camera.metadata.sensorWidth, camera.metadata.sensorHeight);
	return signature;
}

// Everything Image::LoadMetadata derives from EXIF but does *not* put on the camera, in one
// string. These are the fields that tell a cleared EXIF record from an uncleared one whatever
// the focal ladder does with the focal-plane tags: TinyEXIF's clear() parks the geolocation at
// DBL_MAX, which is the sentinel hasAltitude()/hasOrientation()/hasAccuracy() read, and zeroes
// the exposure/ISO the caller copies out unconditionally -- so on a file that carries none of
// those tags an uncleared record reports them as present, with whatever value was on the stack.
static String ImportImageMetadataSignature(const SFM::Image& image)
{
	String signature = String::FormatString("rot %d|gps %d", (int)image.View::metadata.rotated,
		(int)image.View::metadata.HasGPS());
	signature += String::FormatString("|latlon %.9g,%.9g|alt %.9g", image.View::metadata.latitude,
		image.View::metadata.longitude, image.View::metadata.altitude);
	signature += String::FormatString("|acc %.9g,%.9g,%.9g", image.View::metadata.positionAccuracy,
		image.View::metadata.positionAccuracyZ, image.View::metadata.rotationAccuracy);
	signature += String::FormatString("|ypr %.9g,%.9g,%.9g", image.View::metadata.yawDeg,
		image.View::metadata.pitchDeg, image.View::metadata.rollDeg);
	signature += String::FormatString("|orient %u|iso %u|exp %.9g", (unsigned)image.metadata.orientation,
		(unsigned)image.metadata.ISO, image.metadata.exposureTime);
	signature += "|date " + image.metadata.dateTimeOriginal;
	return signature;
}

// Fill the stack just below the caller with a repeating 8-byte pattern, so that the next call
// made by the caller runs on known-dirty memory. An uninitialized read is invisible on a stack
// that happens to still hold zeros -- the value picked up has to look like a *plausible* field
// before the reader reacts to it -- which is exactly why the bug below only surfaced in a process
// that had done a lot of other work first. Both patterns used here therefore read as a valid EXIF
// FocalPlaneResolutionUnit code (2 = inch, 5 = um) taken as an integer, and as a small positive
// denormal taken as a double -- neither of which is any sentinel TinyEXIF's clear() installs.
// Out of line, and stored through a volatile array, on purpose: inlined, the slab would sit in
// the caller's own frame, i.e. *above* everything the next call pushes, and the stores into a
// buffer nobody reads would be dropped.
// Note this cannot work under ASan with detect_stack_use_after_return=1: the slab is then placed
// in a heap-allocated fake frame that the next call does not reuse, so the poisoned half of the
// test passes vacuously -- a green run of it under that option is not evidence.
#ifdef _MSC_VER
#define TESTS_NOINLINE __declspec(noinline)
#else
#define TESTS_NOINLINE __attribute__((noinline))
#endif
static TESTS_NOINLINE void PoisonStackBelow(uint64_t pattern)
{
	constexpr size_t numWords = 8*1024; // 64 KB, far deeper than any metadata reader goes
	uint64_t slab[numWords];
	volatile uint64_t* const words = slab;
	for (size_t i = 0; i < numWords; ++i)
		words[i] = pattern;
}

// The EXIF ladder of Image::LoadMetadata must end in usable numbers whatever the file carries:
// a non-finite or non-positive focal poisons every projection downstream, and a non-finite
// sensor size stays invisible until it changes the camera key. 'context' names the read.
static bool CheckImportedCameraValues(const Camera& camera, const String& fileName, const String& context)
{
	if (camera.GetType() == CameraType::PINHOLE) {
		const PinholeCamera& pinhole = static_cast<const PinholeCamera&>(camera);
		if (!ISFINITE(pinhole.fx) || !ISFINITE(pinhole.fy) || pinhole.fx <= 0 || pinhole.fy <= 0) {
			VERBOSE("ERROR: ImportMetadataDeterminismTest: %s: image '%s' has an unusable focal length: fx %g, fy %g",
				context.c_str(), Util::getFileName(fileName).c_str(), pinhole.fx, pinhole.fy);
			return false;
		}
	}
	if (!ISFINITE(camera.metadata.sensorWidth) || !ISFINITE(camera.metadata.sensorHeight) ||
		camera.metadata.sensorWidth < 0 || camera.metadata.sensorHeight < 0)
	{
		VERBOSE("ERROR: ImportMetadataDeterminismTest: %s: image '%s' has an unusable sensor size: %g x %g mm",
			context.c_str(), Util::getFileName(fileName).c_str(),
			camera.metadata.sensorWidth, camera.metadata.sensorHeight);
		return false;
	}
	return true;
}

// Import metadata determinism: Scene::Import reads every image's header and EXIF in step 2a and
// then groups the images into shared cameras by an exact-match key in step 2c, so anything the
// metadata reader picks up that does not come out of the file changes the *scene* rather than
// failing the import. That is what happened to the two bundled HEICs: the container-EXIF path
// left the EXIF fields the file does not carry uninitialized, and once the leftovers happened to
// look like focal-plane resolutions the derived sensor size came out inf, split the camera the
// four images otherwise share, and moved everything a later stage reads per camera (the
// view-graph calibrator first). So this checks two things on the bundled 2 JPG + 2 HEIC folder:
// repeated imports produce identical cameras, metadata and camera counts, and a single read
// produces the same of all three even on a stack deliberately filled with plausible-looking
// leftovers.
bool ImportMetadataDeterminismTest()
{
	TD_TIMER_START();

	// LoadMetadata reads headers and EXIF only, no pixels, so the whole loop stays well inside a
	// second; it is the repetition that would catch anything drifting over a long-lived process
	constexpr unsigned numIterations = 200;
	constexpr unsigned numBundledImages = 4; // 2 JPG + 2 HEIC in apps/Tests/data/images
	CLISTDEF2(String) referenceSignatures, referenceMetadata, referenceFileNames;
	IIndex referenceNumCameras = 0;
	for (unsigned iteration = 0; iteration < numIterations; ++iteration) {
		// more than one thread, so the import loop really runs in parallel: Scene's constructor
		// is what sizes the OpenMP team
		Scene scene(4);
		const ImportConfig importCfg;
		if (!scene.Import(MAKE_PATH("images"), importCfg)) {
			VERBOSE("ERROR: ImportMetadataDeterminismTest: Import failed at iteration %u", iteration);
			return false;
		}
		CLISTDEF2(String) signatures, metadata;
		FOREACH(idxImage, scene.images) {
			const SFM::Image& image = scene.images[idxImage];
			if (!image.HasCamera()) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: iteration %u: image %u ('%s') has no camera",
					iteration, idxImage, Util::getFileName(image.fileName).c_str());
				return false;
			}
			const Camera& camera = *image.pCamera;
			if (!CheckImportedCameraValues(camera, image.fileName, String::FormatString("iteration %u", iteration)))
				return false;
			signatures.emplace_back(ImportCameraSignature(camera));
			metadata.emplace_back(ImportImageMetadataSignature(image));
		}
		if (iteration == 0) {
			// a build that cannot read one of the formats, or a lost fixture, would otherwise
			// make every comparison below trivially true
			if (signatures.size() != numBundledImages) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: imported %u images with a camera, expected %u",
					(unsigned)signatures.size(), numBundledImages);
				return false;
			}
			referenceSignatures = signatures;
			referenceMetadata = metadata;
			referenceNumCameras = scene.cameras.size();
			FOREACH(idxImage, scene.images)
				referenceFileNames.emplace_back(scene.images[idxImage].fileName);
			continue;
		}
		if (signatures.size() != referenceSignatures.size()) {
			VERBOSE("ERROR: ImportMetadataDeterminismTest: iteration %u imported %u images, iteration 0 imported %u",
				iteration, (unsigned)signatures.size(), (unsigned)referenceSignatures.size());
			return false;
		}
		FOREACH(idxImage, signatures) {
			if (signatures[idxImage] != referenceSignatures[idxImage]) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: iteration %u: image %u camera changed:\n  %s\n  %s (iteration 0)",
					iteration, idxImage, signatures[idxImage].c_str(), referenceSignatures[idxImage].c_str());
				return false;
			}
			if (metadata[idxImage] != referenceMetadata[idxImage]) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: iteration %u: image %u metadata changed:\n  %s\n  %s (iteration 0)",
					iteration, idxImage, metadata[idxImage].c_str(), referenceMetadata[idxImage].c_str());
				return false;
			}
		}
		if (scene.cameras.size() != referenceNumCameras) {
			VERBOSE("ERROR: ImportMetadataDeterminismTest: iteration %u clustered %u cameras, iteration 0 clustered %u",
				iteration, (unsigned)scene.cameras.size(), (unsigned)referenceNumCameras);
			return false;
		}
	}

	// The loop above only sees the leftovers the import itself keeps putting on the stack, which
	// repeat; the original failure needed leftovers from *other* work, which is why it took a full
	// pipeline run to show. So read each file's metadata once more on a stack deliberately filled
	// with a plausible-looking pattern, twice with two different patterns: what a file yields must
	// depend on the file alone, so both must reproduce what the imports above agreed on.
	// Nothing may run between the poison and the read -- the reader's frames have to land on the
	// pattern -- hence the bare call sequence below.
	// Note which comparison is the load-bearing one here. The camera is *not*: the focal-plane
	// guard in LoadMetadata drops a branch that derived nothing usable, so even with the record
	// left uncleared the camera falls through to the 35mm-equivalent source and comes out
	// byte-identical. It is the metadata signature -- the geolocation presence sentinels, the
	// exposure/ISO copied out unconditionally -- that the guard does not and should not touch,
	// and that therefore goes red the moment the clear() is dropped.
	FOREACH(idxFile, referenceFileNames) {
		const String& fileName = referenceFileNames[idxFile];
		for (unsigned pattern = 0; pattern < 2; ++pattern) {
			SFM::Image probe(idxFile, fileName);
			PoisonStackBelow(pattern ? 0x0000000500000005ull : 0x0000000200000002ull);
			if (!probe.LoadMetadata() || !probe.HasCamera()) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: LoadMetadata failed for '%s' on a poisoned stack",
					Util::getFileName(fileName).c_str());
				return false;
			}
			if (!CheckImportedCameraValues(*probe.pCamera, fileName, String::FormatString("poisoned stack, pattern %u", pattern)))
				return false;
			// exact-match clustering means the shared camera carries the key of each image that
			// joined it, so the imported signature is what a single read must reproduce
			const String signature = ImportCameraSignature(*probe.pCamera);
			if (signature != referenceSignatures[idxFile]) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: '%s' read on a poisoned stack (pattern %u) "
					"gives a different camera:\n  %s\n  %s (imported)", Util::getFileName(fileName).c_str(),
					pattern, signature.c_str(), referenceSignatures[idxFile].c_str());
				return false;
			}
			const String metadata = ImportImageMetadataSignature(probe);
			if (metadata != referenceMetadata[idxFile]) {
				VERBOSE("ERROR: ImportMetadataDeterminismTest: '%s' read on a poisoned stack (pattern %u) "
					"gives different metadata:\n  %s\n  %s (imported)", Util::getFileName(fileName).c_str(),
					pattern, metadata.c_str(), referenceMetadata[idxFile].c_str());
				return false;
			}
		}
	}

	VERBOSE("ImportMetadataDeterminismTest: %u imports of %u images, %u cameras each (%s)",
		numIterations, (unsigned)referenceSignatures.size(), (unsigned)referenceNumCameras,
		TD_TIMER_GET_FMT().c_str());
	return true;
} // ImportMetadataDeterminismTest
/*----------------------------------------------------------------*/

// An EXIF 35 mm-equivalent focal (Image::LoadMetadata's second rung) seeds the camera's focal and leaves it
// untrusted: it describes the lens on the full sensor, which a video frame can crop (Tanks and Temples'
// DJI X5R frames, whose 24 mm tag gives a focal 40% short), so the lens fit fits the focal free with the lens
// from it. The two fixtures are 192x108 grey images carrying the EXIF block of a DJI X5R and a Sony A7SM2 frame
// of Tanks and Temples (FocalLength 12 and 21 mm, FocalLengthIn35mmFilm 24 and 21, no focal-plane
// resolution): the focal is the 35 mm formula on the decoded size
bool ExifFocalUntrustedTest()
{
	TD_TIMER_START();
	struct Case {
		const char* file;
		REAL focalLength35mm;
	};
	const Case cases[] = {
		{"exif_samples/dji_x5r_sample.jpg", REAL(24)},
		{"exif_samples/sony_a7sm2_sample.jpg", REAL(21)},
	};
	for (const Case& c : cases) {
		Image image(0, MAKE_PATH(c.file));
		if (!image.LoadMetadata() || !image.HasCamera() || image.GetCameraType() != CameraType::PINHOLE) {
			VERBOSE("ExifFocalUntrustedTest FAILED: '%s' did not import as a pinhole camera", c.file);
			return false;
		}
		const PinholeCamera& camera = static_cast<const PinholeCamera&>(*image.pCamera);
		const REAL expected = SQRT(SQUARE((REAL)camera.GetWidth()) + SQUARE((REAL)camera.GetHeight()))*c.focalLength35mm/REAL(43.27);
		if (camera.fx != expected || camera.fy != expected) {
			VERBOSE("ExifFocalUntrustedTest FAILED: '%s' focal %g %g, the 35 mm formula gives %g", c.file, camera.fx, camera.fy, expected);
			return false;
		}
		if (camera.TrustIntrinsics()) {
			VERBOSE("ExifFocalUntrustedTest FAILED: '%s' trusts its 35 mm-equivalent focal %g", c.file, camera.fx);
			return false;
		}
		VERBOSE("ExifFocalUntrustedTest: '%s' %dx%d, focal %g from the 35 mm-equivalent %g mm, untrusted", c.file,
			camera.GetWidth(), camera.GetHeight(), camera.fx, c.focalLength35mm);
	}
	VERBOSE("ExifFocalUntrustedTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// The camera model ImportConfig::cameraType gives every imported image. A panorama whose file carries no
// equirectangular projection tag (Holo360D's 2880x1440 JPEGs carry none) imports as a pinhole by default, the
// model the metadata gives; forced spherical, every image gets a SphericalCamera and the import shares one camera
// among them; forced pinhole, the same file stays a pinhole. The fixtures are two 64x32 grey JPEGs written
// here (the 2:1 shape an equirectangular image needs)
bool CameraTypeImportTest()
{
	TD_TIMER_START();
	const ScopedTempDir tmpDir(_T("CameraTypeImportTest"));
	if (!tmpDir.IsValid())
		return false;
	constexpr int width = 64, height = 32; // equirectangular: width = 2 x height
	for (unsigned i = 0; i < 2; ++i) {
		const cv::Mat grey(height, width, CV_8UC3, cv::Scalar::all(96 + 32 * i));
		if (!cv::imwrite(tmpDir(String::FormatString("pano%u.jpg", i)), grey)) {
			VERBOSE("CameraTypeImportTest FAILED: cannot write the fixture images");
			return false;
		}
	}
	struct Case {
		CameraType forced;
		CameraType expected;
	};
	const Case cases[] = {
		{CameraType::UNDEFINED, CameraType::PINHOLE},
		{CameraType::SPHERICAL, CameraType::SPHERICAL},
		{CameraType::PINHOLE, CameraType::PINHOLE},
	};
	for (const Case& c : cases) {
		Scene scene;
		ImportConfig importCfg;
		importCfg.cameraType = c.forced;
		if (!scene.Import(tmpDir.Path(), importCfg) || scene.images.size() != 2) {
			VERBOSE("CameraTypeImportTest FAILED: the import (camera type %s) did not load the two images",
				CameraTypeToString(c.forced).c_str());
			return false;
		}
		for (const Image& image : scene.images) {
			if (!image.HasCamera() || image.GetCameraType() != c.expected) {
				VERBOSE("CameraTypeImportTest FAILED: camera type %s gave '%s' a %s camera, %s expected",
					CameraTypeToString(c.forced).c_str(), Util::getFileName(image.fileName).c_str(),
					image.HasCamera() ? CameraTypeToString(image.GetCameraType()).c_str() : "no",
					CameraTypeToString(c.expected).c_str());
				return false;
			}
		}
		if (scene.cameras.size() != 1) {
			VERBOSE("CameraTypeImportTest FAILED: camera type %s left %u cameras for two images of one size",
				CameraTypeToString(c.forced).c_str(), (unsigned)scene.cameras.size());
			return false;
		}
		VERBOSE("CameraTypeImportTest: camera type %s -> %s", CameraTypeToString(c.forced).c_str(),
			CameraTypeToString(c.expected).c_str());
	}
	VERBOSE("CameraTypeImportTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// A spherical import of an image that is not 2:1 is a wrong input file: Image::LoadMetadata logs an error
// naming the image and fails, and Scene::Import treats it as it treats any image whose metadata fails (the
// image stays in the scene with no camera, the others import) instead of reaching the SphericalCamera
// constructor's contract. The fixtures are two 64x32 JPEGs and one 64x40 JPEG; forced spherical, the 64x40
// image holds no camera and the two 2:1 ones hold a spherical camera; the same three imported as pinholes
// (the 64x40 is a legitimate photo then) all hold a camera. The image-level failure is checked directly too
bool SphericalImageSizeImportTest()
{
	TD_TIMER_START();
	const ScopedTempDir tmpDir(_T("SphericalImageSizeImportTest"));
	if (!tmpDir.IsValid())
		return false;
	for (unsigned i = 0; i < 3; ++i) {
		const int height = i < 2 ? 32 : 40;
		const cv::Mat grey(height, 64, CV_8UC3, cv::Scalar::all(96 + 32 * i));
		if (!cv::imwrite(tmpDir(String::FormatString("pano%u.jpg", i)), grey)) {
			VERBOSE("SphericalImageSizeImportTest FAILED: cannot write the fixture images");
			return false;
		}
	}
	{
		Image image;
		image.fileName = tmpDir("pano2.jpg");
		if (image.LoadMetadata(1.2f, CameraType::SPHERICAL) || image.HasCamera()) {
			VERBOSE("SphericalImageSizeImportTest FAILED: LoadMetadata of a 64x40 image forced spherical did not fail");
			return false;
		}
	}
	for (const CameraType forced : {CameraType::SPHERICAL, CameraType::PINHOLE}) {
		Scene scene;
		ImportConfig importCfg;
		importCfg.cameraType = forced;
		if (!scene.Import(tmpDir.Path(), importCfg) || scene.images.size() != 3) {
			VERBOSE("SphericalImageSizeImportTest FAILED: the import (camera type %s) did not keep the three images",
				CameraTypeToString(forced).c_str());
			return false;
		}
		for (const Image& image : scene.images) {
			const bool bad = forced == CameraType::SPHERICAL && Util::getFileName(image.fileName) == "pano2";
			if (image.HasCamera() == bad || (image.HasCamera() && image.GetCameraType() != forced)) {
				VERBOSE("SphericalImageSizeImportTest FAILED: camera type %s: '%s' holds %s camera, expected %s",
					CameraTypeToString(forced).c_str(), Util::getFileName(image.fileName).c_str(),
					image.HasCamera() ? "a" : "no", bad ? "none" : "one of the forced type");
				return false;
			}
		}
	}
	VERBOSE("SphericalImageSizeImportTest PASSED: a 64x40 image forced spherical fails with an error and holds no camera, the 2:1 images import, and as a pinhole it imports (%s)",
		TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

// Pose-frame detection: a frames.json declares neither the camera axes it uses nor, for an
// EXIF-rotated image, how much in-plane rotation separates its camera frame from the working
// raster (1 quarter turn for the on-disk portrait raster, 2 for the sensor-native landscape one
// ARKit reports). Both must be recovered from the matched pairs, and since the two choices do not
// commute, every combination has to round-trip. A wrong turn count conjugates every rotation by a
// multiple of Rz(90), which preserves its angle but tilts its axis, so no axes flip can repair it
// -- before this was searched, such a capture showed a large error under *both* axes hypotheses
// and detection gave up.
bool FramesPoseFrameDetectionTest()
{
	// the two building blocks ImportFramesJSON composes: the in-plane rotation it applies to a
	// rotated image, and the ARKit<->OpenCV camera-axes flip
	const auto InPlane = [](int turns) { return Matrix3x3(RMatrix(0, 0, REAL(M_PI_2) * turns)); };
	const Matrix3x3 axesFlip(1, 0, 0, 0, -1, 0, 0, 0, -1);
	constexpr unsigned numImages = 5;

	// Forward-simulate an import instead of reusing the library's correction, so the test derives
	// the expected poses independently: for a rotated image the import always produces
	// `Rz(90) * D * c2w^T`, while the pose that is actually correct in the working frame carries
	// `turns` quarter turns and the flip only for ARKit axes.
	const auto RunCase = [&](FramesConvention axes, unsigned turns, bool rotated) -> bool {
		Scene scene;
		CLISTDEF0(Matrix3x3) imported(numImages);
		for (unsigned i = 0; i < numImages; ++i) {
			Image& img = scene.images.emplace_back(i, String::FormatString("/in/%u.heic", i));
			img.pCamera = new PinholeCamera(cv::Size(640, 480), 700, 700, 319.5, 239.5);
			img.cameraID = NO_ID; // the image owns its camera
			img.View::metadata.rotated = rotated;
			// the file's own world-to-camera rotation; the tilt keeps the rotation axes well away
			// from the optical axis, without which conjugating by Rz(90) would be a no-op and
			// there would be nothing to detect
			const REAL angle = REAL(0.4) * i;
			const Matrix3x3 fileR(RMatrix(REAL(0.25), angle, REAL(0.1) * i));
			// what is actually correct in the working frame, and what the import produces
			Matrix3x3 truth(fileR);
			if (axes == FramesConvention::ARKIT)
				truth = Matrix3x3(axesFlip * truth);
			if (rotated)
				truth = Matrix3x3(InPlane((int)turns) * truth);
			imported[i] = Matrix3x3(axesFlip * fileR);
			if (rotated)
				imported[i] = Matrix3x3(InPlane(1) * imported[i]);
			// hold the truth while the verified relative poses are built from it
			img.R = RMatrix(truth);
			img.C = CMatrix(3 * std::cos(angle), 3 * std::sin(angle), REAL(0.5) * i);
		}
		// the verified relative poses are what geometric matching recovers: ground truth
		for (unsigned i = 0; i + 1 < numImages; ++i) {
			for (unsigned j = i + 1; j < numImages; ++j) {
				ImagePair& pair = scene.pairs.emplace_back(i, j);
				pair.relativePose = scene.images[j] / scene.images[i];
				pair.matches.emplace_back(0, 0); // the detector only checks that matches exist
			}
		}
		// only now hand the scene the poses the import would have left behind
		FOREACH(i, scene.images)
			scene.images[i].R = RMatrix(imported[i]);

		const FramesPoseFrame detected = DetectFramesConvention(scene, FramesConvention::ARKIT);
		const FramesPoseFrame expected{axes, rotated ? turns : 0u};
		if (detected.convention != expected.convention || detected.inPlaneTurns != expected.inPlaneTurns) {
			VERBOSE("ERROR: FramesPoseFrameDetectionTest: rotated=%d, expected %s, detected %s",
				(int)rotated, FramesPoseFrameToString(expected).c_str(),
				FramesPoseFrameToString(detected).c_str());
			return false;
		}
		// the correction must actually restore the poses, not merely be named correctly
		ApplyFramesPoseFrame(scene, FramesConvention::ARKIT, detected);
		REAL maxError = 0;
		for (const ImagePair& pair : scene.pairs) {
			const Matrix3x3 relative(scene.images[pair.ID2].R * scene.images[pair.ID1].R.t());
			maxError = MAXF(maxError, ComputeAngleSO3<REAL>(relative, pair.relativePose->R));
		}
		if (R2D(maxError) > 1e-6) {
			VERBOSE("ERROR: FramesPoseFrameDetectionTest: %s corrected poses still differ from the "
				"verified relative rotations by %g deg", FramesPoseFrameToString(expected).c_str(),
				R2D(maxError));
			return false;
		}
		return true;
	};

	// every frame a rotated capture can be in: both axes conventions x all four quarter turns.
	// Turn 1 is what the import assumes and turn 2 is what a real ARKit capture needs, but 0 and 3
	// are searched too, so this pins the whole space rather than the two cases seen so far.
	for (const FramesConvention axes : {FramesConvention::ARKIT, FramesConvention::OPENCV})
		for (unsigned turns = 0; turns < FRAMES_IN_PLANE_TURNS; ++turns)
			if (!RunCase(axes, turns, true))
				return false;
	// and an unrotated capture, where every turn count collapses to the same correction and must
	// report 0 rather than split the evidence across four identical hypotheses
	for (const FramesConvention axes : {FramesConvention::ARKIT, FramesConvention::OPENCV})
		if (!RunCase(axes, 0, false))
			return false;

	VERBOSE("FramesPoseFrameDetectionTest: All tests passed");
	return true;
} // FramesPoseFrameDetectionTest
/*----------------------------------------------------------------*/

// External pose import test: frames.json name matching, intrinsics, duplicate rejection,
// and CSV validation without partial updates from an invalid pose row.
bool KnownPosesImportTest()
{
	const ScopedTempDir tmpDir(_T("KnownPosesImportTest"));
	if (!tmpDir.IsValid())
		return false;

	const auto AddImage = [](ImageArr& images, IIndex id, const String& fileName, REAL focal) {
		Image& image = images.emplace_back(id, fileName);
		image.pCamera = new PinholeCamera(cv::Size(640, 480), focal, focal, 319.5, 239.5);
		image.cameraID = NO_ID; // the image owns its pre-deduplication camera
	};

	Scene scene;
	AddImage(scene.images, 0, "/input/FrameA.jpg", 700);
	AddImage(scene.images, 1, "/input/frameB.png", 700);
	AddImage(scene.images, 2, "/input/framec.jpg", 700);
	const String jsonPath = tmpDir(_T("frames.json"));
	{
		std::ofstream os(jsonPath);
		// framea declares its intrinsics in the working (landscape) orientation, framec in the
		// transposed (portrait) one at half resolution: both describe the same 640x480 camera and
		// must land on the same intrinsics, since the orientation is a property of the
		// declaration rather than of the image (all three images here are unrotated)
		os << R"json([
  {"name":"framea.jpg","transform":[1,0,0,0,0,1,0,0,0,0,1,0,1,2,3,1],
   "params":{"camera_model":"OPENCV","w":640,"h":480,"fx":800,"fy":810,"cx":320,"cy":240,"k1":0.01,"k2":-0.02,"p1":0.001,"p2":-0.002}},
  {"name":"FRAMEB","transform":[1,0,0,0,0,1,0,0,0,0,1,0,4,5,6,1]},
  {"name":"framec.jpg","transform":[1,0,0,0,0,1,0,0,0,0,1,0,7,8,9,1],
   "params":{"camera_model":"OPENCV","w":240,"h":320,"fx":405,"fy":400,"cx":120,"cy":159.5,"k1":0.01,"k2":-0.02,"p1":0.001,"p2":-0.002}},
  {"name":"framea.jpg","transform":[1,0,0,0,0,1,0,0,0,0,1,0,9,9,9,1]}
])json";
		if (!os) {
			VERBOSE("KnownPosesImportTest FAILED: cannot write frames.json");
			return false;
		}
	}
	if (ImportFramesJSON(jsonPath, scene, PoseImportMode::POSES_INTRINSICS, FramesConvention::OPENCV) != 3) {
		VERBOSE("KnownPosesImportTest FAILED: frames.json did not import exactly three unique images");
		return false;
	}
	const PinholeCamera* const cameraA = static_cast<const PinholeCamera*>(scene.images[0].pCamera);
	const PinholeCamera* const cameraB = static_cast<const PinholeCamera*>(scene.images[1].pCamera);
	const PinholeCamera* const cameraC = static_cast<const PinholeCamera*>(scene.images[2].pCamera);
	if (norm(scene.images[0].C - Point3(1, 2, 3)) > REAL(1e-6) ||
		norm(scene.images[1].C - Point3(4, 5, 6)) > REAL(1e-6) ||
		norm(scene.images[2].C - Point3(7, 8, 9)) > REAL(1e-6) ||
		ABS(cameraA->fx - REAL(800)) > REAL(1e-6) || !cameraA->trustIntrinsics ||
		cameraB->trustIntrinsics)
	{
		VERBOSE("KnownPosesImportTest FAILED: imported pose/intrinsics mismatch");
		return false;
	}
	// framec declares the same physical camera transposed (portrait) and at half resolution, so
	// after rescaling and the 90-degree rotation it must land on exactly framea's intrinsics.
	// This is what keying the rotation off the *declared* aspect buys: framec's image carries no
	// EXIF rotation at all, so a check on img.IsRotated() would leave it unrotated (and its
	// portrait resolution would simply be rejected as not matching the image).
	if (ABS(cameraC->fx - cameraA->fx) > REAL(1e-6) || ABS(cameraC->fy - cameraA->fy) > REAL(1e-6) ||
		ABS(cameraC->cx - cameraA->cx) > REAL(1e-6) || ABS(cameraC->cy - cameraA->cy) > REAL(1e-6) ||
		!cameraC->trustIntrinsics)
	{
		VERBOSE("KnownPosesImportTest FAILED: portrait-declared intrinsics did not rotate onto the "
			"landscape ones: (%g,%g,%g,%g) vs (%g,%g,%g,%g)", cameraC->fx, cameraC->fy, cameraC->cx,
			cameraC->cy, cameraA->fx, cameraA->fy, cameraA->cx, cameraA->cy);
		return false;
	}
	// the tangential coefficients do rotate with the raster (the radial ones do not)
	if (ABS(cameraC->p1 - REAL(-0.002)) > REAL(1e-9) || ABS(cameraC->p2 - REAL(-0.001)) > REAL(1e-9) ||
		ABS(cameraC->k1 - cameraA->k1) > REAL(1e-9) || ABS(cameraC->k2 - cameraA->k2) > REAL(1e-9))
	{
		VERBOSE("KnownPosesImportTest FAILED: distortion not rotated as expected: p1=%g p2=%g k1=%g k2=%g",
			cameraC->p1, cameraC->p2, cameraC->k1, cameraC->k2);
		return false;
	}

	ImageArr csvImages;
	AddImage(csvImages, 0, "/input/framea.jpg", 700);
	AddImage(csvImages, 1, "/input/frameb.jpg", 700);
	AddImage(csvImages, 2, "/input/one/ambiguous.jpg", 700);
	AddImage(csvImages, 3, "/input/two/ambiguous.png", 700);
	const String csvPath = tmpDir(_T("poses.csv"));
	{
		std::ofstream os(csvPath);
		os << "filename,fx,fy,cx,cy,qx,qy,qz,qw,Cx,Cy,Cz,score\n";
		os << "framea,800,810,320,240,0,0,0,1,1,2,3,1\n";
		os << "frameb,900,900,320,240,0,0,0,0,4,5,6,1\n"; // invalid zero quaternion
		os << "ambiguous,900,900,320,240,0,0,0,1,7,8,9,1\n";
		if (!os) {
			VERBOSE("KnownPosesImportTest FAILED: cannot write pose CSV");
			return false;
		}
	}
	if (ImportPosesCSV(csvPath, csvImages, PoseImportMode::POSES_INTRINSICS) != 1) {
		VERBOSE("KnownPosesImportTest FAILED: pose CSV did not reject the invalid quaternion row");
		return false;
	}
	const PinholeCamera* const csvCameraA = static_cast<const PinholeCamera*>(csvImages[0].pCamera);
	const PinholeCamera* const csvCameraB = static_cast<const PinholeCamera*>(csvImages[1].pCamera);
	if (!csvImages[0].HasPose() || csvImages[1].HasPose() ||
		csvImages[2].HasPose() || csvImages[3].HasPose() ||
		!csvCameraA->trustIntrinsics || csvCameraB->trustIntrinsics ||
		ABS(csvCameraB->fx - REAL(700)) > REAL(1e-6))
	{
		VERBOSE("KnownPosesImportTest FAILED: invalid CSV row partially modified its image");
		return false;
	}

	VERBOSE("KnownPosesImportTest PASSED");
	return true;
}

} // namespace SFM
