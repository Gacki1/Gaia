#version 450

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
} f;

layout(set = 0, binding = 1) uniform sampler2D uSceneColor;
layout(set = 0, binding = 2) uniform sampler2D uTransmittance;
layout(set = 0, binding = 3) uniform sampler2D uDepth;

layout(location = 0) in  vec2 vUV;
layout(location = 0) out vec4 outColor;

const float kPi      = 3.14159265358979;
const float kFarAway = 1e9;

const int   kSteps   = 16;

vec3 aces(vec3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 viewRay() {
    vec2 ndc = vUV * 2.0 - 1.0;
    return normalize(f.camFwd.xyz + ndc.x * f.camRight.xyz - ndc.y * f.camUp.xyz);
}

float rayLength(vec3 dir) {
    float depth = texture(uDepth, vUV).r;
    if (depth <= 0.0) return kFarAway;
    return (f.camFwd.w / depth) / dot(dir, f.camFwd.xyz);
}

vec2 shellHit(float a, float mu, float s) {
    float b = (1.0 + a) * mu;
    float c = (a - s) * (a + s + 2.0);
    float disc = b * b - c;
    if (disc < 0.0) return vec2(1.0, -1.0);
    float sq = sqrt(disc);
    return vec2(-b - sq, -b + sq);
}

float altAt(float a, float mu, float t) {
    float q = a * (a + 2.0) + t * (t + 2.0 * (1.0 + a) * mu);
    return q / (sqrt(1.0 + q) + 1.0);
}

bool hitsGround(float a, float mu) {
    if (mu >= 0.0) return false;
    float b = (1.0 + a) * mu;
    return b * b - a * (a + 2.0) >= 0.0;
}

float rayleighPhase(float c) { return (3.0 / (16.0 * kPi)) * (1.0 + c * c); }

float miePhase(float c, float g) {
    float g2 = g * g;
    float d  = 1.0 + g2 - 2.0 * g * c;
    return (1.0 - g2) / (4.0 * kPi * d * sqrt(max(d, 1e-8)));
}

vec3 sunTransmittance(float a, float mu, float top) {
    if (hitsGround(a, mu)) return vec3(0.0);

    float span = sqrt(top * (top + 2.0));
    float rho  = sqrt(max(a * (a + 2.0), 0.0));
    float dMin = top - a;
    float dMax = rho + span;
    float d    = shellHit(a, mu, top).y;

    vec2 x  = vec2(clamp((d - dMin) / max(dMax - dMin, 1e-9), 0.0, 1.0),
                   clamp(rho / span, 0.0, 1.0));
    vec2 sz = vec2(textureSize(uTransmittance, 0));
    return texture(uTransmittance, (0.5 + x * (sz - 1.0)) / sz).rgb;
}

vec3 sunDiscRadiance(vec3 dir, float top) {
    vec3  sunL = -normalize(f.sunDir.xyz);
    float cosR = f.sunDisc.x;

    float w    = max(f.sunDisc.z * sqrt(max(1.0 - cosR * cosR, 0.0)), 1e-9);
    float edge = smoothstep(cosR - w, cosR + w, dot(dir, sunL));
    if (edge <= 0.0) return vec3(0.0);

    float a  = f.camGeo.w / f.atmoB.x;
    float mu = dot(f.camGeo.xyz, dir);
    if (hitsGround(a, mu)) return vec3(0.0);

    return sunTransmittance(a, mu, top) * (f.sunDisc.y * edge);
}

vec3 scatter(vec3 dir, float tEndMetres, out vec3 viewT) {
    viewT = vec3(1.0);
    float R   = f.atmoB.x;
    float top = f.atmoB.y / R;
    float a   = f.camGeo.w / R;
    vec3  up  = f.camGeo.xyz;
    float mu  = dot(up, dir);

    float t0, t1;
    if (a <= top) {
        t0 = 0.0;
        t1 = shellHit(a, mu, top).y;
    } else {
        vec2 h = shellHit(a, mu, top);
        if (h.x > h.y || h.y <= 0.0) return vec3(0.0);
        t0 = max(h.x, 0.0);
        t1 = h.y;
    }
    if (hitsGround(a, mu)) t1 = min(t1, shellHit(a, mu, 0.0).x);
    t1 = min(t1, tEndMetres / R);
    if (t1 <= t0) return vec3(0.0);

    vec3  sunL  = -normalize(f.sunDir.xyz);
    float cosT  = dot(dir, sunL);
    float phR   = rayleighPhase(cosT);
    float phM   = miePhase(cosT, f.atmoA.w);

    float ds  = (t1 - t0) / float(kSteps);
    float dsM = ds * R;

    float odR = 0.0, odM = 0.0;
    vec3  sumR = vec3(0.0), sumM = vec3(0.0);

    for (int i = 0; i < kSteps; ++i) {
        float t   = t0 + (float(i) + 0.5) * ds;
        float alt = altAt(a, mu, t);
        float hM  = alt * R;
        float dR  = exp(-hM / f.atmoA.x);
        float dM  = exp(-hM / f.atmoA.y);

        odR += 0.5 * dR * dsM;
        odM += 0.5 * dM * dsM;
        vec3 tView = exp(-(f.betaR.rgb * odR + vec3(f.atmoA.z * odM)));
        odR += 0.5 * dR * dsM;
        odM += 0.5 * dM * dsM;

        vec3  pos = (1.0 + a) * up + t * dir;
        float rr  = length(pos);
        vec3  tSun = sunTransmittance(alt, dot(pos, sunL) / rr, top);

        sumR += tView * tSun * dR * dsM;
        sumM += tView * tSun * dM * dsM;
    }

    viewT = exp(-(f.betaR.rgb * odR + vec3(f.atmoA.z * odM)));

    return f.atmoB.z * (f.betaR.rgb * (phR * sumR) + f.betaR.w * (phM * sumM));
}

vec2 projectDirection(vec3 w, out bool behind, out bool offscreen) {
    float z = dot(w, f.camFwd.xyz);
    vec2 ndc;
    behind = z <= 1e-4;
    if (!behind) {
        vec3 wp = w / z;
        ndc = vec2( dot(wp, f.camRight.xyz) / dot(f.camRight.xyz, f.camRight.xyz),
                   -dot(wp, f.camUp.xyz)    / dot(f.camUp.xyz,    f.camUp.xyz));
    } else {

        vec2 planar = vec2(dot(w, f.camRight.xyz), -dot(w, f.camUp.xyz));
        ndc = (length(planar) > 1e-9) ? normalize(planar) * 4.0 : vec2(0.0, -4.0);
    }
    offscreen = behind || any(greaterThan(abs(ndc), vec2(0.92)));
    if (offscreen) ndc /= max(max(abs(ndc.x), abs(ndc.y)), 1e-6) / 0.92;
    return ndc;
}

void main() {
    vec3  dir = viewRay();

    if (f.params.y > 1.5 && f.params.y < 2.5) {
        float d = rayLength(dir);
        if (d >= kFarAway) { outColor = vec4(0.0, 0.0, 0.0, 1.0); return; }
        float decade = log(d) / log(10.0);
        vec3  tint   = vec3(decade < 2.0 ? 1.0 : 0.2,
                            decade < 4.0 ? 1.0 : 0.2,
                            decade < 3.0 ? 0.2 : 1.0);
        outColor = vec4(tint * (0.25 + 0.75 * fract(decade)), 1.0);
        return;
    }

    if (f.params.y > 2.5) {
        vec3 vt;
        vec3 ins = scatter(dir, f.atmoB.w, vt);
        outColor = vec4(aces((vec3(1.0) * vt + ins) * f.params.w), 1.0);
        return;
    }

    if (f.params.y > 0.5) {
        outColor = vec4(texture(uTransmittance, vUV).rgb, 1.0);
        return;
    }

    float dist = rayLength(dir);
    vec3  viewT;

    vec3  inscatter = scatter(dir, dist, viewT);
    vec3  behind    = (dist >= kFarAway) ? vec3(0.0)
                                         : texture(uSceneColor, vUV).rgb;

    if (dist >= kFarAway) inscatter += sunDiscRadiance(dir, f.atmoB.y / f.atmoB.x);

    outColor = vec4(aces((behind * viewT + inscatter) * f.params.w), 1.0);

    if (f.marker.w > 0.5) {
        bool isBehind, isOff;
        vec2 ndc = projectDirection(normalize(f.marker.xyz), isBehind, isOff);
        vec2 sz  = vec2(textureSize(uDepth, 0));

        vec2  p = ((vUV * 2.0 - 1.0) - ndc) * vec2(sz.x / sz.y, 1.0);
        float r = length(p);

        float radius = isOff ? 0.020 : 0.032;
        float edge   = 2.0 / sz.y;
        float ring   = smoothstep(radius + 0.004 + edge, radius + 0.004, r) *
                       smoothstep(radius - edge, radius, r);

        vec3 tint = (f.marker.w > 1.5) ? vec3(1.00, 0.62, 0.20)
                                       : vec3(0.35, 0.85, 1.00);
        outColor.rgb = mix(outColor.rgb, tint, ring * 0.9);
    }
}
