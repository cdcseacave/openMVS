R"glsl(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor; // only a (quantized confidence) is used
flat out uint vVertexID;

uniform uint uBaseID;
uniform vec2 confidenceWindow; // points with a confidence outside [x, y] are hidden

layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
};

void main() {
    gl_Position = aColor.a >= confidenceWindow.x && aColor.a <= confidenceWindow.y ? viewProjection * vec4(aPos, 1.0) : vec4(2.0, 2.0, 2.0, 1.0);
    vVertexID = uBaseID + uint(gl_VertexID);
}
)glsl"
