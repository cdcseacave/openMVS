R"glsl(
#version 330 core

layout (location = 0) in vec4 aPos; // xyz line end, w quantized confidence of its point

uniform vec2 confidenceWindow; // normals of points with a confidence outside [x, y] are hidden

// Uniform Block for ViewProjection
layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
    float padding;
};

void main() {
    // a hidden line has both ends moved outside the clip volume, so it is culled
    gl_Position = aPos.w >= confidenceWindow.x && aPos.w <= confidenceWindow.y ? viewProjection * vec4(aPos.xyz, 1.0) : vec4(2.0, 2.0, 2.0, 1.0);
}
)glsl"
