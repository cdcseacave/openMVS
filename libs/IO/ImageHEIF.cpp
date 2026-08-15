////////////////////////////////////////////////////////////////////
// ImageHEIF.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include <vector>
#include <cstring>

#ifdef _IMAGE_HEIF
#include <libheif/heif.h>
#include "ImageHEIF.h"

namespace SEACAVE {

struct HeifState {
	std::vector<uint8_t> fileData; // whole compressed file; must outlive ctx (read_from_memory_without_copy)
	heif_context* ctx = nullptr;
	heif_image_handle* handle = nullptr; // primary image

	void Close() {
		if (handle) {
			heif_image_handle_release(handle);
			handle = nullptr;
		}
		if (ctx) {
			heif_context_free(ctx);
			ctx = nullptr;
		}
		fileData.clear();
	}

	bool ReadStreamChunk(IOSTREAMPTR& stream, size_t chunk_size = 1024*64) {
		ASSERT(stream);
		auto* in = stream->getInputStream();
		fileData.resize(fileData.size() + chunk_size);
		const size_t read = in->read(fileData.data() + fileData.size() - chunk_size, chunk_size);
		if (read == STREAM_ERROR) {
			// do not fold STREAM_ERROR ((size_t)-1) into the size arithmetic below: it would
			// wrap around and request an absurd allocation instead of reporting the read error
			fileData.resize(fileData.size() - chunk_size);
			return false;
		}
		fileData.resize(fileData.size() + read - chunk_size);
		if (read == 0)
			return false; // no more data to read
		return true;
	}
};

CImageHEIF::CImageHEIF() : m_state(NULL) {}
CImageHEIF::~CImageHEIF() { Close(); }

void CImageHEIF::Close() {
	if (m_state) {
		HeifState* const state = reinterpret_cast<HeifState*>(m_state);
		state->Close();
		// delete through the typed pointer: deleting a void* would skip ~HeifState() and leak
		// fileData's buffer (Close() only clear()s it, which does not release capacity), once
		// per decoded image
		delete state;
		m_state = NULL;
	}
	m_width = m_height = 0;
	CImage::Close();
}

bool CImageHEIF::ReadHeader() {
	if (!m_pStream)
		return false;
	HeifState*& state = reinterpret_cast<HeifState*&>(m_state);
	if (state)
		state->Close();
	else
		state = new HeifState();
	// read the whole compressed file into memory: HEIC files are a few MB,
	// so whole-file buffering is intentional and fine
	m_pStream->getInputStream()->setPos(0);
	while (state->ReadStreamChunk(m_pStream)) {}
	if (state->fileData.empty()) {
		Close();
		return false;
	}
	state->ctx = heif_context_alloc();
	if (!state->ctx) {
		Close();
		return false;
	}
	// the "without_copy" variant requires the memory to outlive the context,
	// which it does since fileData lives alongside ctx in HeifState
	heif_error err = heif_context_read_from_memory_without_copy(state->ctx, state->fileData.data(), state->fileData.size(), NULL);
	if (err.code != heif_error_Ok) {
		LOG(LT_IMAGE, "error: unable to parse HEIF file: %s", err.message);
		Close();
		return false;
	}
	// multi-image containers (bursts, Live Photos, thumbnails, aux depth maps)
	// correctly resolve to the primary image
	err = heif_context_get_primary_image_handle(state->ctx, &state->handle);
	if (err.code != heif_error_Ok) {
		LOG(LT_IMAGE, "error: unable to get HEIF primary image: %s", err.message);
		Close();
		return false;
	}
	// these dimensions already reflect the container irot/imir transforms
	// applied by libheif at decode time, so header dims and decoded dims are consistent
	m_width = (Size)heif_image_handle_get_width(state->handle);
	m_height = (Size)heif_image_handle_get_height(state->handle);
	if (m_width == 0 || m_height == 0) {
		LOG(LT_IMAGE, "error: unsupported HEIF image");
		Close();
		return false;
	}
	m_dataWidth = m_width;
	m_dataHeight = m_height;
	m_numLevels = 1;
	// Native format: report the alpha channel when the file actually has one, so callers can
	// ask for it; ReadData then only decodes it when the requested format wants it (see there).
	// CHANNEL ORDER: libheif's heif_chroma_interleaved_RGB/RGBA emit bytes in literal R,G,B[,A]
	// order. SEACAVE's PF_B8G8R8 constant -- despite its name -- is the one whose in-memory byte
	// order is R,G,B: the names list channels most- to least-significant bit (see PIXELFORMAT in
	// Image.h), so on a little-endian machine they read back reversed. The anchor is CImagePNG,
	// which calls libpng's png_set_bgr() precisely when a PF_B8G8R8 file is read into a
	// PF_R8G8B8 request, i.e. PF_R8G8B8 is the B,G,R (OpenCV) order every OpenMVS consumer asks
	// for and PF_B8G8R8 is the R,G,B order codecs natively emit; CImageJPG agrees, declaring
	// PF_B8G8R8 for libjpeg's JCS_RGB. FilterFormat then flips for us on the way out.
	m_format = heif_image_handle_has_alpha_channel(state->handle) ? PF_B8G8R8A8 : PF_B8G8R8;
	m_stride = FormatHasAlpha(m_format) ? 4 : 3;
	m_lineWidth = m_width * m_stride;
	// 10/12-bit HDR sources: libheif down-converts to 8-bit when 8-bit interleaved chroma is
	// requested (as we do in ReadData); acceptable for SfM/MVS, a 16-bit PF path is future work
	return true;
}

bool CImageHEIF::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth) {
	HeifState* state = (HeifState*)m_state;
	if (!state || !state->handle)
		return false;
	// Decode the alpha channel only when the caller asks for a format that has one. Real iPhone
	// HEIFs do carry alpha, and decoding it for a request that cannot hold it would route the
	// copy through FilterFormat's 32-bit conversions -- whose 8-bit-gray destination copies the
	// *alpha* byte rather than a luminance of RGB (see its PF_A8/PF_GRAY8 case), i.e. a constant
	// image and so zero features for the gray loads feature extraction performs. Dropping alpha
	// keeps such requests on exactly the same well-tested 24-bit path as CImageJPG.
	const bool wantAlpha = FormatHasAlpha(dataFormat) && FormatHasAlpha(m_format);
	const PIXELFORMAT srcFormat = wantAlpha ? PF_B8G8R8A8 : PF_B8G8R8;
	const Size srcPixelStride = wantAlpha ? 4 : 3;
	const heif_chroma chroma = wantAlpha ? heif_chroma_interleaved_RGBA : heif_chroma_interleaved_RGB;
	heif_image* img = NULL;
	// do NOT set ignore_transformations: we want libheif to apply irot/imir,
	// consistent with the dimensions already reported by ReadHeader
	heif_error err = heif_decode_image(state->handle, &img, heif_colorspace_RGB, chroma, NULL);
	if (err.code != heif_error_Ok) {
		LOG(LT_IMAGE, "error: unable to decode HEIF image: %s", err.message);
		return false;
	}
	int srcLineWidth(0);
	const uint8_t* src = heif_image_get_plane_readonly(img, heif_channel_interleaved, &srcLineWidth);
	if (src == NULL || srcLineWidth <= 0) {
		heif_image_release(img);
		return false;
	}
	// srcLineWidth may exceed m_width*srcPixelStride, so copy row-wise, never as one block
	uint8_t* dst = (uint8_t*)pData;
	bool ok(true);
	if (dataFormat == srcFormat && nStride == srcPixelStride) {
		// read image directly to the data buffer
		for (Size j = 0; j < m_height; ++j, dst += lineWidth, src += srcLineWidth)
			memcpy(dst, src, m_width * srcPixelStride);
	} else {
		// read image to a buffer and convert it
		for (Size j = 0; j < m_height; ++j, dst += lineWidth, src += srcLineWidth) {
			if (!FilterFormat(dst, dataFormat, nStride, src, srcFormat, srcPixelStride, m_width)) {
				ok = false;
				break;
			}
		}
	}
	heif_image_release(img);
	return ok;
}

bool CImageHEIF::WriteHeader(PIXELFORMAT /*imageFormat*/, Size /*width*/, Size /*height*/, BYTE /*numLevels*/) {
	LOG(LT_IMAGE, "error: HEIF/HEIC write is not supported (read-only)");
	return false;
}

bool CImageHEIF::WriteData(void* /*pData*/, PIXELFORMAT /*dataFormat*/, Size /*nStride*/, Size /*lineWidth*/) {
	LOG(LT_IMAGE, "error: HEIF/HEIC write is not supported (read-only)");
	return false;
}

// Fetch the raw EXIF blob attached to the primary image, normalized so it always starts with the
// 6 bytes "Exif\0\0" as required by TinyEXIF::parseFromEXIFSegment (see TinyEXIF.h).
// The item stored by libheif is a 4-byte big-endian exif_tiff_header_offset followed by the EXIF
// payload (usually "Exif\0\0"+TIFF header, but sometimes a bare TIFF header with no marker).
bool CImageHEIF::GetMetadataEXIF(std::vector<uint8_t>& blob) const {
	HeifState* state = (HeifState*)m_state;
	if (!state || !state->handle)
		return false;
	heif_item_id id(0);
	if (heif_image_handle_get_list_of_metadata_block_IDs(state->handle, "Exif", &id, 1) < 1)
		return false; // no EXIF block present
	const size_t size = heif_image_handle_get_metadata_size(state->handle, id);
	if (size <= 4)
		return false; // too short to hold anything beyond the offset prefix
	std::vector<uint8_t> raw(size);
	if (heif_image_handle_get_metadata(state->handle, id, raw.data()).code != heif_error_Ok)
		return false;
	// skip the 4-byte exif_tiff_header_offset prefix
	const uint8_t* payload = raw.data() + 4;
	const size_t payloadSize = size - 4;
	if (payloadSize < 8)
		return false; // too short to be a valid TIFF header
	static const uint8_t exifMarker[6] = {'E', 'x', 'i', 'f', 0, 0};
	blob.clear();
	if (memcmp(payload, exifMarker, 6) == 0) {
		// already starts with "Exif\0\0"
		blob.assign(payload, payload + payloadSize);
	} else {
		// bare TIFF header: prepend the marker TinyEXIF::parseFromEXIFSegment expects
		blob.reserve(6 + payloadSize);
		blob.insert(blob.end(), exifMarker, exifMarker + 6);
		blob.insert(blob.end(), payload, payload + payloadSize);
	}
	return true;
}


#ifdef _USE_TESTS

namespace {

// Read a whole fixture at the requested format into a tightly packed buffer
bool ReadFixture(const String& path, PIXELFORMAT format, CImage::Size stride,
	std::vector<uint8_t>& data, CImage::Size& width, CImage::Size& height)
{
	CAutoPtr<CImage> pImage(CImage::Create(path, CImage::READ));
	if (pImage == NULL || !pImage->ReadHeader())
		return false;
	width = pImage->GetWidth();
	height = pImage->GetHeight();
	data.resize((size_t)width * height * stride);
	return pImage->ReadData(data.data(), format, stride, width * stride);
}

// Peak-to-peak spread of one interleaved channel: tells a real picture from the constant
// image a leaked (uniformly opaque) alpha channel would produce
unsigned ChannelSpread(const std::vector<uint8_t>& data, CImage::Size stride, CImage::Size channel)
{
	uint8_t lo = 255, hi = 0;
	for (size_t i = channel; i < data.size(); i += stride) {
		lo = MINF(lo, data[i]);
		hi = MAXF(hi, data[i]);
	}
	return hi > lo ? unsigned(hi - lo) : 0u;
}

} // unnamed namespace

bool CImageHEIF::Test(const String& folder)
{
	String dir(folder);
	Util::ensureValidFolderPath(dir);
	// There are no HEIF-only fixtures: two of the four pipeline images are HEIC, so the SFM and
	// MVS tests decode them for real on every run, which is what covers pixel content as a whole.
	//  - 00001.heic: decodes 640x479, no container rotation, carries a genuine fully-opaque alpha
	//                channel like a real iPhone photo; feature extraction gray-loads it every run
	//  - 00002.heic: stored landscape but carrying a container 'irot' for 90deg CCW, so it DECODES
	//                portrait 479x640, AND a stored EXIF Orientation=8 naming the same rotation.
	//                Honoring the tag on top of the irot libheif already applied would rotate a
	//                second time. SFM turns it back into the landscape working raster
	//                (View::ToWorkingOrientation rotates 90deg CW); MVS, which has no
	//                EXIF-rotation concept, is given a matching portrait camera by scene.mvs.
	const String pathAlpha(dir + "00001.heic");
	const String pathRotated(dir + "00002.heic");
	// 00001.heic is brightest in its top-right corner, where red and blue differ by ~105 -- far
	// more than the lossy-HEVC tolerance, so comparing that one pixel catches a channel swap
	// decisively (the tolerance also absorbs decoder drift across libheif versions). The channel
	// order is a property of the reader, not of the file, so pinning it on one image is enough --
	// and it has to be this one: 00002.heic's corner is dark, with red and blue only ~14 apart.
	constexpr unsigned alphaR = 127, alphaG = 171, alphaB = 232;
	constexpr unsigned cornerTol = 12;
	// a real picture spreads far wider than this; a constant one does not spread at all
	constexpr unsigned minSpread = 10;

	// 1) The decoded dimensions must already include the container 'irot' libheif applies at
	// decode time, and the header-only read must agree with the full decode -- otherwise the
	// two disagree and a cached header mis-sizes the buffer the pixels are read into.
	const auto CheckHeader = [](const String& path, CImage::Size width, CImage::Size height,
		bool hasAlpha) -> bool
	{
		CAutoPtr<CImage> pImage(CImage::Create(path, CImage::READ));
		if (pImage == NULL || !pImage->ReadHeader()) {
			VERBOSE("error: CImageHEIF::Test: cannot read the header of '%s'", path.c_str());
			return false;
		}
		if (pImage->GetWidth() != width || pImage->GetHeight() != height) {
			VERBOSE("error: CImageHEIF::Test: '%s' header is %ux%u, expected %ux%u", path.c_str(),
				pImage->GetWidth(), pImage->GetHeight(), width, height);
			return false;
		}
		// the native format must report the alpha channel the file really has, so that a caller
		// can ask for it; ReadData still only decodes it on demand (checked below)
		if (pImage->FormatHasAlpha() != hasAlpha || pImage->GetStride() != (hasAlpha ? 4u : 3u)) {
			VERBOSE("error: CImageHEIF::Test: '%s' reports alpha=%d stride=%u, expected alpha=%d",
				path.c_str(), (int)pImage->FormatHasAlpha(), pImage->GetStride(), (int)hasAlpha);
			return false;
		}
		return true;
	};
	if (!CheckHeader(pathRotated, 479, 640, false) || // genuine irot: 90deg CCW of a 640x479 picture
		!CheckHeader(pathAlpha, 640, 479, true))
		return false;

	// 2) Absolute channel order. PF_* names list channels most- to least-significant bit, so on
	// a little-endian machine they read back reversed: PF_R8G8B8 must deliver B,G,R (the OpenCV
	// order every OpenMVS consumer requests) and PF_B8G8R8 the R,G,B order codecs emit. Checking
	// both directions locks the absolute order down, not merely the parity between the two.
	CImage::Size width = 0, height = 0;
	std::vector<uint8_t> bgr, rgb;
	if (!ReadFixture(pathAlpha, PF_R8G8B8, 3, bgr, width, height) ||
		!ReadFixture(pathAlpha, PF_B8G8R8, 3, rgb, width, height))
	{
		VERBOSE("error: CImageHEIF::Test: cannot decode '%s'", pathAlpha.c_str());
		return false;
	}
	const auto CheckCorner = [&](const char* what, const std::vector<uint8_t>& data,
		CImage::Size stride, unsigned e0, unsigned e1, unsigned e2) -> bool
	{
		const size_t corner = (size_t)(width - 1) * stride; // top-right pixel of the first row
		const unsigned c0 = data[corner], c1 = data[corner+1], c2 = data[corner+2];
		if (ABS((int)c0 - (int)e0) > (int)cornerTol || ABS((int)c1 - (int)e1) > (int)cornerTol ||
			ABS((int)c2 - (int)e2) > (int)cornerTol)
		{
			VERBOSE("error: CImageHEIF::Test: %s corner is (%u,%u,%u), expected (%u,%u,%u); "
				"a mismatch of the outer two channels is a red/blue swap", what, c0, c1, c2, e0, e1, e2);
			return false;
		}
		return true;
	};
	if (!CheckCorner("PF_R8G8B8", bgr, 3, alphaB, alphaG, alphaR) ||
		!CheckCorner("PF_B8G8R8", rgb, 3, alphaR, alphaG, alphaB))
		return false;

	// 3) Alpha is decoded only when the requested format can hold it. Real iPhone HEIFs carry an
	// alpha channel; decoding it for a 3-channel or gray request would route the copy through
	// FilterFormat's 32-bit conversions, whose gray destination copies the *alpha* byte instead
	// of a luminance of RGB -- a constant image, and so zero features, with nothing logged.
	std::vector<uint8_t> alphaAsBGR, alphaAsGray, alphaAsBGRA;
	if (!ReadFixture(pathAlpha, PF_R8G8B8, 3, alphaAsBGR, width, height) ||
		!ReadFixture(pathAlpha, PF_GRAY8, 1, alphaAsGray, width, height) ||
		!ReadFixture(pathAlpha, PF_R8G8B8A8, 4, alphaAsBGRA, width, height))
	{
		VERBOSE("error: CImageHEIF::Test: cannot decode '%s'", pathAlpha.c_str());
		return false;
	}
	// dropping the alpha must leave the picture itself untouched
	if (!CheckCorner("alpha PF_R8G8B8", alphaAsBGR, 3, alphaB, alphaG, alphaR))
		return false;
	const unsigned graySpread = ChannelSpread(alphaAsGray, 1, 0);
	if (graySpread < minSpread) {
		VERBOSE("error: CImageHEIF::Test: gray read of '%s' spreads only %u levels: the alpha "
			"channel leaked into the luminance conversion", pathAlpha.c_str(), graySpread);
		return false;
	}
	// asking for a format that does hold alpha must still deliver it, last channel, fully opaque
	const unsigned alphaSpread = ChannelSpread(alphaAsBGRA, 4, 3);
	if (alphaSpread != 0 || alphaAsBGRA[3] != 255) {
		VERBOSE("error: CImageHEIF::Test: alpha channel of '%s' is not uniformly opaque "
			"(first %u, spread %u)", pathAlpha.c_str(), (unsigned)alphaAsBGRA[3], alphaSpread);
		return false;
	}
	if (!CheckCorner("alpha PF_R8G8B8A8", alphaAsBGRA, 4, alphaB, alphaG, alphaR))
		return false;

	// 4) The EXIF bridge must hand back a blob TinyEXIF can parse, i.e. one starting with the
	// 6-byte "Exif\0\0" marker (libheif stores a 4-byte offset prefix, and sometimes no marker).
	{
		CAutoPtr<CImage> pImage(CImage::Create(pathRotated, CImage::READ));
		std::vector<uint8_t> exif;
		if (pImage == NULL || !pImage->ReadHeader() || !pImage->GetMetadataEXIF(exif)) {
			VERBOSE("error: CImageHEIF::Test: no EXIF blob in '%s'", pathRotated.c_str());
			return false;
		}
		static const uint8_t marker[6] = {'E', 'x', 'i', 'f', 0, 0};
		if (exif.size() <= sizeof(marker) || memcmp(exif.data(), marker, sizeof(marker)) != 0) {
			VERBOSE("error: CImageHEIF::Test: EXIF blob of '%s' (%u bytes) is not prefixed with "
				"the \"Exif\\0\\0\" marker TinyEXIF requires", pathRotated.c_str(), (unsigned)exif.size());
			return false;
		}
	}

	// 5) Write is not supported and must be refused rather than emitting a broken file
	{
		CImageHEIF writer;
		if (writer.WriteHeader(PF_B8G8R8, 16, 16, 1) || writer.WriteData(NULL, PF_B8G8R8, 3, 48)) {
			VERBOSE("error: CImageHEIF::Test: write must be refused");
			return false;
		}
	}
	return true;
} // Test

#endif // _USE_TESTS

} // namespace SEACAVE

#endif // _IMAGE_HEIF
