R"glsl(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor; // points: rgb color, a quantized confidence; faces: a = 1

out vec3 FragColor;

uniform vec3 highlightColor = vec3(1.0, 0.0, 0.0);
uniform bool useHighlight = false;
uniform float pointSize = 5.0;
uniform vec2 confidenceWindow; // primitives with a confidence outside [x, y] are hidden

layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
};

void main() {
    gl_Position = aColor.a >= confidenceWindow.x && aColor.a <= confidenceWindow.y ? viewProjection * vec4(aPos, 1.0) : vec4(2.0, 2.0, 2.0, 1.0);
    FragColor = useHighlight ? highlightColor : aColor.rgb;
    gl_PointSize = pointSize;
}
)glsl"
