#version 450

layout(location = 0) in vec3 inPos;

layout(location = 1) in vec2 inNormalOct;
layout(location = 2) in vec4 inColor;
layout(location = 3) in vec3 inCoarse;
layout(location = 4) in vec2 inCoarseNOct;
layout(location = 5) in vec4 inMat0;
layout(location = 6) in vec4 inMat1;

layout(set = 0, binding = 0) uniform Frame {
    mat4 viewProj;
    vec4 sunDir;
    vec4 params;
    vec4 camRight;
    vec4 camUp;
    vec4 camFwd;
    vec4 camGeo;
    vec4 betaR;
    vec4 atmoA;
    vec4 atmoB;
    vec4 sunDisc;
    vec4 marker;
    vec4 terrain;
    vec4 terrain2;
} f;

layout(push_constant) uniform Object {
    vec4 offset;

    vec4 rot;

    vec4 shade;

    vec4 texOrigin;

} o;

layout(location = 0) out vec3  vNormal;
layout(location = 1) out vec3  vColor;
layout(location = 2) out float vAmbient;

layout(location = 3) out vec4  vTexPos;
layout(location = 4) out vec4  vMat0;
layout(location = 5) out vec4  vMat1;

const vec3 kHighlightTint = vec3(1.9, 1.6, 0.7);

vec3 qrot(vec4 q, vec3 v) {
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

float morphFactor(vec3 camRelative) {
    if (o.shade.w <= 0.0) return 0.0;
    float d = length(camRelative);
    return smoothstep(o.shade.w * f.terrain.x, o.shade.w, d);
}

vec3 octDecode(vec2 e) {
    vec3 v = vec3(e.x, e.y, 1.0 - abs(e.x) - abs(e.y));
    if (v.z < 0.0) v.xy = (1.0 - abs(v.yx)) * vec2(v.x >= 0.0 ? 1.0 : -1.0,
                                                   v.y >= 0.0 ? 1.0 : -1.0);
    return normalize(v);
}

void main() {
    vec3 inNormal  = octDecode(inNormalOct);
    vec3 inCoarseN = octDecode(inCoarseNOct);

    vec3 fine   = inPos * o.shade.z;
    vec3 coarse = inCoarse * o.shade.z;
    float m     = morphFactor(qrot(o.rot, fine) + o.offset.xyz);
    vec3 local  = mix(fine, coarse, m);

    gl_Position = f.viewProj * vec4(qrot(o.rot, local) + o.offset.xyz, 1.0);

    vNormal = qrot(o.rot, normalize(mix(inNormal, inCoarseN, m)));

    vTexPos = vec4(qrot(o.rot, local) + o.texOrigin.xyz,
                   length(qrot(o.rot, local) + o.offset.xyz));
    vMat0   = inMat0;
    vMat1   = inMat1;

    vColor   = inColor.rgb * mix(vec3(1.0), kHighlightTint, o.shade.y);
    vAmbient = o.shade.x;
}
