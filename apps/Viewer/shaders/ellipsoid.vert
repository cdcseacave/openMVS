R"glsl(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;

layout (std140) uniform ViewProjection {
    mat4 view;
    mat4 projection;
    mat4 viewProjection;
    vec3 cameraPos;
};

out vec3 normal;
out vec3 viewDir;
out vec3 vColor;

void main() {
    gl_Position = viewProjection * vec4(aPos, 1.0);
    normal = aNormal;
    viewDir = normalize(cameraPos - aPos);
    vColor = aColor;
}
)glsl"
