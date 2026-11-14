R"glsl(
#version 330 core

layout (location = 0) in vec3 position;  // 3D world position
layout (location = 1) in vec2 texCoord;  // Texture coordinates

layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
};

uniform mat4 model; // identity for the image quads (world positions), the image pose and size for the panorama sphere

out vec2 TexCoord;

void main() {
    gl_Position = viewProjection * model * vec4(position, 1.0);
    TexCoord = texCoord;
}
)glsl"
