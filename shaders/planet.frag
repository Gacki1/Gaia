#version 450

layout(location = 0) in vec3  vNormal;
layout(location = 1) in vec3  vColor;
layout(location = 2) in float vAmbient;
layout(location = 3) in vec4  vTexPos;

layout(location = 4) in vec4  vMat0;
layout(location = 5) in vec4  vMat1;

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

layout(set = 0, binding = 4) uniform sampler2DArray uGround;
layout(set = 0, binding = 5) uniform sampler2DArray uGroundN;

layout(location = 0) out vec4 outColor;

float triWeightExp = 4.0;

vec3 triWeights(vec3 n) {
    vec3 w = pow(abs(n), vec3(triWeightExp));
    return w / (w.x + w.y + w.z);
}

vec4 triplanar(vec3 p, vec3 n, float layer, float metres) {
    vec3 w   = triWeights(n);
    vec3 uvw = p / metres;

    return w.x * texture(uGround, vec3(uvw.zy, layer))
         + w.y * texture(uGround, vec3(uvw.xz, layer))
         + w.z * texture(uGround, vec3(uvw.xy, layer));
}

vec3 unpackNormal(vec2 rg, float strength) {
    vec2 xy = (rg * 2.0 - 1.0) * strength;
    return vec3(xy, sqrt(max(0.0, 1.0 - dot(xy, xy))));
}

vec3 triplanarNormal(vec3 p, vec3 n, float layer, float metres, float strength) {
    vec3 w   = triWeights(n);
    vec3 uvw = p / metres;

    vec3 tx = unpackNormal(texture(uGroundN, vec3(uvw.zy, layer)).rg, strength);
    vec3 ty = unpackNormal(texture(uGroundN, vec3(uvw.xz, layer)).rg, strength);
    vec3 tz = unpackNormal(texture(uGroundN, vec3(uvw.xy, layer)).rg, strength);

    tx = vec3(tx.xy + n.zy, abs(tx.z) * n.x);
    ty = vec3(ty.xy + n.xz, abs(ty.z) * n.y);
    tz = vec3(tz.xy + n.xy, abs(tz.z) * n.z);

    return normalize(tx.zyx * w.x + ty.xzy * w.y + tz.xyz * w.z);
}

void main() {
    vec3  N = normalize(vNormal);
    vec3  L = normalize(-f.sunDir.xyz);

    float mw[8] = float[8](vMat0.x, vMat0.y, vMat0.z, vMat0.w,
                           vMat1.x, vMat1.y, vMat1.z, vMat1.w);
    int   nLayer = int(f.terrain2.y);
    float onGround = 0.0;
    for (int i = 0; i < nLayer; ++i) onGround += mw[i];

    float texFade = 1.0 - smoothstep(f.terrain2.z, f.terrain2.w, vTexPos.w);

    vec3  albedo = vColor;
    float occl   = 1.0;

    if (onGround > 0.001 && texFade > 0.002) {
        vec3  mat = vec3(0.0);
        vec3  nrm = vec3(0.0);
        float ao  = 0.0;
        for (int i = 0; i < nLayer; ++i) {

            if (mw[i] <= 0.004) continue;
            vec4 fine  = triplanar(vTexPos.xyz, N, float(i), f.terrain.y);
            vec4 macro = triplanar(vTexPos.xyz, N, float(i), f.terrain.z);
            vec4 c     = mix(fine, macro, 0.4);
            mat += mw[i] * c.rgb;
            ao  += mw[i] * c.a;
            nrm += mw[i] * triplanarNormal(vTexPos.xyz, N, float(i),
                                           f.terrain.y, f.terrain2.x);
        }
        mat /= onGround;
        ao  /= onGround;

        const float g = onGround * texFade;
        N = normalize(mix(N, nrm, g));

        albedo = mix(mat, vColor, mix(1.0, f.terrain.w * onGround, texFade));
        occl   = mix(1.0, ao, g);
    }

    float ambient = max(f.params.x, vAmbient);
    float diffuse = max(dot(N, L), 0.0);

    float lit = ambient * occl + (1.0 - ambient) * diffuse;

    outColor = vec4(albedo * (f.params.z * lit), 1.0);
}
