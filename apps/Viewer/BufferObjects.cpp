/*
 * BufferObjects.cpp
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

#include "BufferObjects.h"
#include "Shader.h"
#include "Renderer.h"

using namespace VIEWER;

// VBO Implementation
VBO::VBO(GLenum target) : target(target) {
	GL_CHECK(glGenBuffers(1, &id));
}

VBO::~VBO() {
	if (id != 0) {
		GL_CHECK(glDeleteBuffers(1, &id));
	}
}

void VBO::Bind() const {
	GL_CHECK(glBindBuffer(target, id));
}

void VBO::Unbind() const {
	GL_CHECK(glBindBuffer(target, 0));
}

template<typename T>
void VBO::SetData(const std::vector<T>& data, GLenum usage) {
	SetData(data.data(), data.size(), usage);
}

template<typename T>
void VBO::SetData(const T* data, size_t count, GLenum usage) {
	Bind();
	GL_CHECK(glBufferData(target, count * sizeof(T), data, usage));
}

void VBO::SetData(const void* data, size_t size, GLenum usage) {
	Bind();
	GL_CHECK(glBufferData(target, size, data, usage));
}

void VBO::AllocateBuffer(size_t size, GLenum usage) {
	Bind();
	GL_CHECK(glBufferData(target, size, nullptr, usage));
}

template<typename T>
void VBO::SetSubData(const std::vector<T>& data, size_t offset) {
	SetSubData(data.data(), data.size(), offset);
}

template<typename T>
void VBO::SetSubData(const T* data, size_t count, size_t offset) {
	Bind();
	GL_CHECK(glBufferSubData(target, offset * sizeof(T), count * sizeof(T), data));
}

void VBO::SetSubData(const void* data, size_t size, size_t offset) {
	Bind();
	GL_CHECK(glBufferSubData(target, offset, size, data));
}

// Readback implementations
template<typename T>
void VBO::GetData(T* out, size_t count) {
	ASSERT(count > 0);
	Bind();
	GL_CHECK(glGetBufferSubData(target, 0, count * sizeof(T), out));
}

template<typename T>
void VBO::GetData(std::vector<T>& out) {
	GetData(out.data(), out.size());
}

template<typename T>
void VBO::GetSubData(T* out, size_t count, size_t offset) {
	Bind();
	GL_CHECK(glGetBufferSubData(target, offset * sizeof(T), count * sizeof(T), out));
}

template<typename T>
void VBO::GetSubData(std::vector<T>& out, size_t offset) {
	if (out.empty()) return;
	GetSubData(out.data(), out.size(), offset);
}

// Explicit template instantiations
template void VBO::SetData<float>(const std::vector<float>&, GLenum);
template void VBO::SetData<uint32_t>(const std::vector<uint32_t>&, GLenum);
template void VBO::SetData<uint8_t>(const std::vector<uint8_t>&, GLenum);
template void VBO::SetData<float>(const float*, size_t, GLenum);
template void VBO::SetData<uint32_t>(const uint32_t*, size_t, GLenum);
template void VBO::SetData<uint8_t>(const uint8_t*, size_t, GLenum);

template void VBO::SetSubData<float>(const std::vector<float>&, size_t);
template void VBO::SetSubData<uint32_t>(const std::vector<uint32_t>&, size_t);
template void VBO::SetSubData<uint8_t>(const std::vector<uint8_t>&, size_t);
template void VBO::SetSubData<float>(const float*, size_t, size_t);
template void VBO::SetSubData<uint32_t>(const uint32_t*, size_t, size_t);
template void VBO::SetSubData<uint8_t>(const uint8_t*, size_t, size_t);

// Explicit template instantiations for readback
template void VBO::GetData<float>(float*, size_t);
template void VBO::GetData<uint32_t>(uint32_t*, size_t);
template void VBO::GetData<uint8_t>(uint8_t*, size_t);
template void VBO::GetData<float>(std::vector<float>&);
template void VBO::GetData<uint32_t>(std::vector<uint32_t>&);
template void VBO::GetData<uint8_t>(std::vector<uint8_t>&);

template void VBO::GetSubData<float>(float*, size_t, size_t);
template void VBO::GetSubData<uint32_t>(uint32_t*, size_t, size_t);
template void VBO::GetSubData<uint8_t>(uint8_t*, size_t, size_t);
template void VBO::GetSubData<float>(std::vector<float>&, size_t);
template void VBO::GetSubData<uint32_t>(std::vector<uint32_t>&, size_t);
template void VBO::GetSubData<uint8_t>(std::vector<uint8_t>&, size_t);

// VAO Implementation
VAO::VAO() {
	GL_CHECK(glGenVertexArrays(1, &id));
}

VAO::~VAO() {
	if (id != 0) {
		GL_CHECK(glDeleteVertexArrays(1, &id));
	}
}

void VAO::Bind() const {
	GL_CHECK(glBindVertexArray(id));
}

void VAO::Unbind() const {
	GL_CHECK(glBindVertexArray(0));
}

void VAO::EnableAttribute(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer) {
	GL_CHECK(glEnableVertexAttribArray(index));
	GL_CHECK(glVertexAttribPointer(index, size, type, normalized, stride, pointer));
}

void VAO::DisableAttribute(GLuint index) {
	GL_CHECK(glDisableVertexAttribArray(index));
}

// UBO Implementation
UBO::UBO(GLuint bindingPoint) : bindingPoint(bindingPoint) {
	GL_CHECK(glGenBuffers(1, &id));
}

UBO::~UBO() {
	if (id != 0) {
		GL_CHECK(glDeleteBuffers(1, &id));
	}
}

void UBO::Bind() const {
	GL_CHECK(glBindBuffer(GL_UNIFORM_BUFFER, id));
	GL_CHECK(glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, id));
}

void UBO::BindToShader(const Shader& shader, const std::string& blockName) {
	GLuint blockIndex = glGetUniformBlockIndex(shader.GetProgram(), blockName.c_str());
	if (blockIndex != GL_INVALID_INDEX) {
		GL_CHECK(glUniformBlockBinding(shader.GetProgram(), blockIndex, bindingPoint));
	}
}

template<typename T>
void UBO::SetData(const T& data, GLenum usage) {
	Bind();
	GL_CHECK(glBufferData(GL_UNIFORM_BUFFER, sizeof(T), &data, usage));
}

void UBO::SetSubData(const void* data, size_t offset, size_t size) {
	Bind();
	GL_CHECK(glBufferSubData(GL_UNIFORM_BUFFER, offset, size, data));
}

// Explicit template instantiations for common uniform buffer types
template void UBO::SetData<ViewProjectionData>(const ViewProjectionData&, GLenum);
template void UBO::SetData<LightingData>(const LightingData&, GLenum);
/*----------------------------------------------------------------*/
