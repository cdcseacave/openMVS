R"glsl(
#version 330 core

in vec3 normal;
in vec3 viewDir;
in vec3 vColor;

out vec4 FragColor;

uniform float alpha = 0.6;

void main() {
    // Two-sided head-light shading: the normal is flipped toward the viewer so both the near
    // and far translucent hemispheres are lit, giving the surface a clear 3D (ellipsoidal) shape
    // regardless of the covariance orientation.
    vec3 norm = normalize(normal);
    if (dot(norm, viewDir) < 0.0)
        norm = -norm;
    float diff = max(dot(norm, viewDir), 0.0);
    float ambient = 0.35;
    vec3 color = vColor * (ambient + 0.75 * diff);
    FragColor = vec4(color, alpha);
}
)glsl"
