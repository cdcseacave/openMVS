/*
 * BufferObjects.h
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

#pragma once

#include <glad/glad.h>

namespace VIEWER {

// Forward declarations
class Shader;
struct ViewProjectionData;
struct LightingData;

/**
 * @brief Vertex Buffer Object (VBO) wrapper class
 * 
 * Manages OpenGL vertex buffer objects which store vertex data (positions, normals, 
 * texture coordinates, etc.) in GPU memory. VBOs are used to efficiently transfer 
 * vertex data from CPU to GPU and provide fast access during rendering.
 */
class VBO {
private:
	GLuint id;
	GLenum target;
	
public:
	VBO(GLenum target = GL_ARRAY_BUFFER);
	~VBO();

	// Non-copyable
	VBO(const VBO&) = delete;
	VBO& operator=(const VBO&) = delete;

	void Bind() const;
	void Unbind() const;

	// the typed data functions take counts and offsets in elements of T, never in bytes
	// (only AllocateBuffer takes a byte size)
	template<typename T>
	void SetData(const std::vector<T>& data, GLenum usage = GL_STATIC_DRAW);

	template<typename T>
	void SetData(const T* data, size_t count, GLenum usage = GL_STATIC_DRAW);

	// Buffer allocation and sub-data functions for multi-mesh support
	void AllocateBuffer(size_t size, GLenum usage = GL_STATIC_DRAW);

	template<typename T>
	void SetSubData(const std::vector<T>& data, size_t offset);

	template<typename T>
	void SetSubData(const T* data, size_t count, size_t offset);

	// Read back buffer data
	template<typename T>
	void GetData(T* out, size_t count);

	template<typename T>
	void GetData(std::vector<T>& out);

	template<typename T>
	void GetSubData(T* out, size_t count, size_t offset);

	template<typename T>
	void GetSubData(std::vector<T>& out, size_t offset);

	GLuint GetID() const { return id; }
};

/**
 * @brief Vertex Array Object (VAO) wrapper class
 * 
 * Manages OpenGL vertex array objects which store vertex attribute configuration.
 * VAOs remember the vertex attribute setup (which VBOs are bound, how data is 
 * interpreted, etc.) allowing for efficient switching between different vertex 
 * data layouts without reconfiguring attributes each time.
 */
class VAO {
private:
	GLuint id;
	
public:
	VAO();
	~VAO();

	// Non-copyable
	VAO(const VAO&) = delete;
	VAO& operator=(const VAO&) = delete;

	void Bind() const;
	void Unbind() const;

	void EnableAttribute(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer);
	void DisableAttribute(GLuint index);

	GLuint GetID() const { return id; }
};

/**
 * @brief Uniform Buffer Object (UBO) wrapper class
 * 
 * Manages OpenGL uniform buffer objects which store uniform data shared across 
 * multiple shaders. UBOs are more efficient than individual uniforms when dealing 
 * with large amounts of uniform data (like transformation matrices, lighting data, 
 * material properties) and allow for better memory management and performance.
 */
class UBO {
private:
	GLuint id;
	GLuint bindingPoint;
	
public:
	UBO(GLuint bindingPoint);
	~UBO();

	// Non-copyable
	UBO(const UBO&) = delete;
	UBO& operator=(const UBO&) = delete;

	void Bind() const;
	void BindToShader(const Shader& shader, const std::string& blockName);

	template<typename T>
	void SetData(const T& data, GLenum usage = GL_DYNAMIC_DRAW);

	void SetSubData(const void* data, size_t offset, size_t size);
	GLuint GetID() const { return id; }

	template<typename T>
	void GetData(T& data) const {
		glBindBuffer(GL_UNIFORM_BUFFER, id);
		void* ptr = glMapBuffer(GL_UNIFORM_BUFFER, GL_READ_ONLY);
		if (ptr) {
			memcpy(&data, ptr, sizeof(T));
			glUnmapBuffer(GL_UNIFORM_BUFFER);
		}
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
	}
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
