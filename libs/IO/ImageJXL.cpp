////////////////////////////////////////////////////////////////////
// ImageJXL.cpp
//
// Copyright 2024 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include <vector>
#include <cstring>

#ifdef _IMAGE_JXL
#include <jxl/decode.h>
#include <jxl/encode.h>
#include "ImageJXL.h"

namespace SEACAVE {

struct JxlState {
	std::vector<uint8_t> compressed;
	JxlBasicInfo info;
	bool got_info = false;
	JxlDecoder* decoder = nullptr; // Hold decoder for incremental reading

	void Close() {
		if (decoder) {
			JxlDecoderDestroy(decoder);
			decoder = nullptr;
		}
		compressed.clear();
		got_info = false;
		info = {};
	}

	bool ReadStreamChunk(IOSTREAMPTR& stream, size_t chunk_size = 1024*64) {
		ASSERT(stream);
		auto* in = stream->getInputStream();
		compressed.resize(compressed.size() + chunk_size);
		const size_t read = in->read(compressed.data() + compressed.size() - chunk_size, chunk_size);
		if (read == STREAM_ERROR) {
			// do not fold STREAM_ERROR ((size_t)-1) into the size arithmetic below: it would
			// wrap around and request an absurd allocation instead of reporting the read error
			compressed.resize(compressed.size() - chunk_size);
			return false;
		}
		compressed.resize(compressed.size() + read - chunk_size);
		if (read == 0)
			return false; // no more data to read
		return true;
	}

	bool HandleNeedMoreInput(IOSTREAMPTR& stream) {
		const size_t remaining_size = JxlDecoderReleaseInput(decoder);
		ASSERT(remaining_size <= compressed.size());
		if (remaining_size > 0)
			memmove(compressed.data(), compressed.data() + compressed.size() - remaining_size, remaining_size);
		compressed.resize(remaining_size);
		if (!ReadStreamChunk(stream) && compressed.empty())
			return false;
		JxlDecoderSetInput(decoder, compressed.data(), compressed.size());
		return true;
	}
};

CImageJXL::CImageJXL() : m_state(NULL) {}
CImageJXL::~CImageJXL() { Close(); }

void CImageJXL::Close() {
	if (m_state) {
		JxlState* const state = reinterpret_cast<JxlState*>(m_state);
		state->Close();
		// delete through the typed pointer: deleting a void* would skip ~JxlState() and leak
		// compressed's buffer (Close() only clear()s it, which does not release capacity), once
		// per decoded image
		delete state;
		m_state = NULL;
	}
	m_width = m_height = 0;
	CImage::Close();
}

bool CImageJXL::ReadHeader() {
	if (!m_pStream)
		return false;
	JxlState*& state = reinterpret_cast<JxlState*&>(m_state);
	if (state)
		state->Close();
	else
		state = new JxlState();
	state->decoder = JxlDecoderCreate(NULL);
	if (!state->decoder)
		return false;
	JxlDecoderSubscribeEvents(state->decoder, JXL_DEC_BASIC_INFO | JXL_DEC_COLOR_ENCODING | JXL_DEC_FULL_IMAGE);
	// Read initial chunk
	m_pStream->getInputStream()->setPos(0);
	if (!state->ReadStreamChunk(m_pStream))
		return false;
	JxlDecoderSetInput(state->decoder, state->compressed.data(), state->compressed.size());
	for (;;) {
		JxlDecoderStatus status = JxlDecoderProcessInput(state->decoder);
		if (status == JXL_DEC_ERROR)
			break;
		if (status == JXL_DEC_NEED_MORE_INPUT) {
			if (!state->HandleNeedMoreInput(m_pStream))
				break;
			continue;
		}
		if (status == JXL_DEC_BASIC_INFO) {
			if (JxlDecoderGetBasicInfo(state->decoder, &state->info) == JXL_DEC_SUCCESS) {
				m_width = state->info.xsize;
				m_height = state->info.ysize;
				m_dataWidth = m_width;
				m_dataHeight = m_height;
				m_numLevels = 1;
				if (state->info.num_color_channels == 1) {
					if (state->info.bits_per_sample == 8) {
						m_format = PF_GRAY8;
						m_stride = 1;
					} else if (state->info.bits_per_sample == 32) {
						m_format = PF_GRAY32F;
						m_stride = 4;
					} else {
						Close();
						return false; // Unsupported format
					}
				} else if (state->info.num_color_channels == 4) {
					m_format = PF_B8G8R8A8;
					m_stride = 4;
				} else if (state->info.num_color_channels == 3) {
					m_format = PF_B8G8R8;
					m_stride = 3;
				} else {
					Close();
					return false; // Unsupported format
				}
				m_lineWidth = m_width * m_stride;
				state->got_info = true;
			}
			continue;
		}
		if (status == JXL_DEC_COLOR_ENCODING || status == JXL_DEC_FRAME || status == JXL_DEC_SUCCESS)
			break;
	}
	// Do NOT destroy decoder here; keep it for ReadData
	return state->got_info ? true : false;
}

bool CImageJXL::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth) {
	JxlState* state = (JxlState*)m_state;
	if (!state || !state->got_info || !state->decoder)
		return false;
	JxlPixelFormat format = {};
	format.num_channels = state->info.num_color_channels;
	if (dataFormat == PF_GRAY32F) {
		format.data_type = (JxlDataType)JXL_TYPE_FLOAT;
	} else {
		format.data_type = (JxlDataType)JXL_TYPE_UINT8;
	}
	format.endianness = (JxlEndianness)JXL_NATIVE_ENDIAN;
	format.align = 0u;
	uint8_t* buffer;
	size_t buffer_size = 0;
	std::vector<uint8_t> pixels_u8;
	if (dataFormat == m_format && nStride == m_stride) {
		// read image directly to the data buffer
		buffer_size = m_height * lineWidth;
		buffer = reinterpret_cast<uint8_t*>(pData);
	} else {
		// read image to a buffer and convert it
		buffer_size = m_height * m_width * m_stride;
		pixels_u8.resize(buffer_size);
		buffer = pixels_u8.data();
	}
	for (;;) {
		JxlDecoderStatus status = JxlDecoderProcessInput(state->decoder);
		if (status == JXL_DEC_ERROR)
			break;
		if (status == JXL_DEC_NEED_MORE_INPUT) {
			if (!state->HandleNeedMoreInput(m_pStream))
				break;
			continue;
		}
		if (status == JXL_DEC_NEED_IMAGE_OUT_BUFFER) {
			if (JxlDecoderSetImageOutBuffer(state->decoder, &format, buffer, buffer_size) != JXL_DEC_SUCCESS)
				break;
			continue;
		}
		if (status == JXL_DEC_FULL_IMAGE) {
			if (buffer == pixels_u8.data()) {
				uint8_t* dst = (uint8_t*)pData;
				uint8_t* src = buffer;
				for (Size j = 0; j < m_height; ++j, dst += lineWidth, src += m_width * m_stride)
					if (!FilterFormat(dst, dataFormat, nStride, src, m_format, m_stride, m_width))
						return false;
			}
			continue;
		}
		if (status == JXL_DEC_SUCCESS)
			return true;
	}
	JxlDecoderDestroy(state->decoder);
	state->decoder = nullptr;
	return false;
}

bool CImageJXL::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels) {
	m_width = width;
	m_height = height;
	m_dataWidth = width;
	m_dataHeight = height;
	m_numLevels = numLevels;
	m_format = imageFormat;
	if (imageFormat == PF_B8G8R8A8 || imageFormat == PF_GRAY32F)
		m_stride = 4;
	else
		m_stride = 3;
	m_lineWidth = m_width * m_stride;
	return true;
}

bool CImageJXL::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth) {
	if (!m_pStream)
		return false;
	JxlEncoder* enc = JxlEncoderCreate(NULL);
	if (!enc)
		return false;
	JxlEncoderFrameSettings* frame_settings = JxlEncoderFrameSettingsCreate(enc, NULL);
	JxlBasicInfo info;
	JxlEncoderInitBasicInfo(&info);
	info.xsize = m_width;
	info.ysize = m_height;
	info.bits_per_sample = (m_format == PF_GRAY32F) ? 32 : 8;
	info.exponent_bits_per_sample = 0;
	info.num_color_channels = m_stride;
	info.num_extra_channels = 0;
	JxlEncoderSetBasicInfo(enc, &info);
	JxlColorEncoding color_encoding;
	JxlColorEncodingSetToSRGB(&color_encoding, /*is_gray=*/m_stride < 3);
	JxlEncoderSetColorEncoding(enc, &color_encoding);
	JxlPixelFormat format;
	format.num_channels = nStride;
	format.data_type = (dataFormat == PF_GRAY32F) ? (JxlDataType)JXL_TYPE_FLOAT : (JxlDataType)JXL_TYPE_UINT8;
	format.endianness = (JxlEndianness)JXL_NATIVE_ENDIAN;
	format.align = 0u;
	if (JxlEncoderAddImageFrame(frame_settings, &format, pData, m_height * lineWidth) != JXL_ENC_SUCCESS) {
		JxlEncoderDestroy(enc);
		return false;
	}
	JxlEncoderCloseInput(enc);
	std::vector<uint8_t> compressed(4096);
	uint8_t* next_out = compressed.data();
	size_t avail_out = compressed.size();
	for (;;) {
		JxlEncoderStatus status = JxlEncoderProcessOutput(enc, &next_out, &avail_out);
		if (status == JXL_ENC_ERROR) {
			JxlEncoderDestroy(enc);
			return false;
		}
		if (status == JXL_ENC_NEED_MORE_OUTPUT) {
			size_t offset = (size_t)(next_out - compressed.data());
			compressed.resize(compressed.size() * 2);
			next_out = compressed.data() + offset;
			avail_out = compressed.size() - offset;
			continue;
		}
		if (status == JXL_ENC_SUCCESS)
			break;
	}
	size_t out_size = (size_t)(next_out - compressed.data());
	m_pStream->getOutputStream()->setPos(0);
	m_pStream->getOutputStream()->write(compressed.data(), out_size);
	JxlEncoderDestroy(enc);
	return true;
}

} // namespace SEACAVE

#endif // _IMAGE_JXL
