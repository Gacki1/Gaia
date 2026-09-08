#version 450

layout(location = 0) in vec2 inPos;
layout(location = 1) in vec2 inUV;
layout(location = 2) in vec4 inColor;

layout(push_constant) uniform Overlay {
    vec4 viewport;
} o;

layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;

void main() {

    vec2 ndc = inPos / o.viewport.xy * 2.0 - 1.0;
    gl_Position = vec4(ndc, 0.0, 1.0);
    vUV    = inUV;
    vColor = inColor;
}
