R"glsl(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor; // rgb color, a quantized confidence

out vec3 FragColor;

// Uniforms
uniform float pointSize;
uniform vec2 confidenceWindow; // points with a confidence outside [x, y] are hidden

// Uniform Block for ViewProjection
layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
    float padding;
};

void main() {
    // a hidden point is moved outside the clip volume, so it is culled
    gl_Position = aColor.a >= confidenceWindow.x && aColor.a <= confidenceWindow.y ? viewProjection * vec4(aPos, 1.0) : vec4(2.0, 2.0, 2.0, 1.0);
    FragColor = aColor.rgb;
    gl_PointSize = pointSize;
}
)glsl"
