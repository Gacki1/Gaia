#version 450

layout(location = 0) in vec2 vUV;
layout(location = 1) in vec4 vColor;

layout(set = 0, binding = 6) uniform sampler2D uFont;

layout(location = 0) out vec4 outColor;

vec3 srgbToLinear(vec3 c) {
    return mix(c / 12.92,
               pow((c + 0.055) / 1.055, vec3(2.4)),
               step(vec3(0.04045), c));
}

void main() {
    float cover = texture(uFont, vUV).r;

    outColor = vec4(srgbToLinear(vColor.rgb), vColor.a * cover);
}
