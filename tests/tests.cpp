#include "math/SpaceMath.h"
#include "world/PlanetConfig.h"
#include "math/Mat4.h"
#include "world/Noise.h"
#include "world/CubeSphere.h"
#include "render/OverlayUi.h"
#include "render/PlanetEditor.h"
#include "world/PlanetValidate.h"
#include "world/Plates.h"
#include "world/PlanetIO.h"
#include "world/Quadtree.h"
#include "platform/JobSystem.h"
#include "math/Quat.h"
#include "ship/Flight.h"
#include "ship/ShipMesh.h"
#include "ship/Camera.h"
#include "world/Atmosphere.h"
#include "ship/Cockpit.h"
#include "ship/QuantumDrive.h"
#include "world/Scatter.h"
#include "world/Outpost.h"
#include "world/TerrainGround.h"

#include <cmath>
#include <chrono>
#include <cstdio>
#include <algorithm>
#include <string>
#include <vector>
#include <atomic>
#include <unordered_set>

using namespace space;
using namespace planet;

namespace {
int g_failures = 0;

void check(bool cond, const std::string& name) {
    if (cond) std::printf("  [PASS] %s\n", name.c_str());
    else { std::fprintf(stderr, "  [FAIL] %s\n", name.c_str()); ++g_failures; }
}
bool approx(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

planet::AABB boxAt(const Vec3& c, float half) {
    planet::AABB b;
    b.expand({ c.x - half, c.y - half, c.z - half });
    b.expand({ c.x + half, c.y + half, c.z + half });
    return b;
}
}

static const planet::PlanetParams kTestParams{};

int main() {
    std::printf("Procedural planet tests\n");

    const float focal  = focalLength(deg2rad(60.0f));
    const float aspect = 1280.0f / 720.0f;
    const float nearP  = 0.1f;

    {
        Basis b = makeBasis(0.0f, 0.0f);
        check(approx(b.forward.z, 1.0f) && approx(b.forward.x, 0.0f), "camera: identity forward is +Z");
        check(approx(length(b.forward), 1.0f) && approx(length(b.right), 1.0f) &&
              approx(length(b.up), 1.0f), "camera: basis vectors are unit length");
        check(approx(dot(b.forward, b.right), 0.0f) && approx(dot(b.forward, b.up), 0.0f) &&
              approx(dot(b.right, b.up), 0.0f), "camera: basis vectors orthogonal");
    }
    {
        Basis b = makeBasis(deg2rad(90.0f), 0.0f);
        check(approx(b.forward.x, 1.0f) && approx(b.forward.z, 0.0f), "camera: yaw +90 looks toward +X");
    }
    {

        float worst = 0.0f;
        for (int i = 0; i < 64; ++i) {
            for (int j = -8; j <= 8; ++j) {
                const float y0 = -3.1f + 6.2f * i / 63.0f;
                const float p0 = deg2rad(88.0f) * j / 8.0f;
                const Basis b  = makeBasis(y0, p0);
                const float p1 = std::asin(std::max(-1.0f, std::min(1.0f, b.forward.y)));
                const float y1 = std::atan2(b.forward.x, b.forward.z);

                const Vec3 f1 = makeBasis(y1, p1).forward;
                worst = std::max(worst, length(f1 - b.forward));
            }
        }
        check(worst < 1e-5f,
              "camera: (yaw,pitch) recovered from forward rebuilds the same view");
    }
    {
        Projected p = projectPoint(Vec3{ 0,0,25 }, Vec3{}, makeBasis(0,0), focal, aspect, nearP);
        check(p.visible && approx(p.ndcX,0.0f) && approx(p.ndcY,0.0f), "camera: forward point maps to centre");
        check(!projectPoint(Vec3{0,0,-5}, Vec3{}, makeBasis(0,0), focal, aspect, nearP).visible,
              "camera: point behind camera is culled");
    }
    {
        check(approx(clampPitch(deg2rad(200.0f), deg2rad(89.0f)), deg2rad(89.0f)), "camera: pitch clamps at +89");
        check(approx(clampPitch(deg2rad(-200.0f), deg2rad(89.0f)), -deg2rad(89.0f)), "camera: pitch clamps at -89");
    }

    {
        Mat4 view = planet::lookAt(Vec3{0,0,5}, Vec3{0,0,-1}, Vec3{0,1,0});
        Mat4 proj = planet::perspective(deg2rad(60.0f), 1.0f, 0.1f, 100.0f);
        Mat4 vp = proj * view;

        planet::Vec4 c = planet::transform(vp, Vec3{0,0,0});
        check(c.w > 0.0f, "mat4: origin has positive clip w");
        float nx = c.x/c.w, ny = c.y/c.w, nz = c.z/c.w;
        check(nx > -1 && nx < 1 && ny > -1 && ny < 1 && nz >= 0 && nz <= 1, "mat4: origin inside NDC cube");

        planet::Vec4 hi = planet::transform(vp, Vec3{0, 1, 0});
        planet::Vec4 lo = planet::transform(vp, Vec3{0,-1, 0});
        check(hi.y/hi.w < lo.y/lo.w, "mat4: higher world point maps upward (y-down clip)");

        Frustum fr = Frustum::fromViewProj(vp);
        check(fr.intersectsAABB(boxAt(Vec3{0,0,0}, 0.5f)),  "frustum: box in front is visible");
        check(!fr.intersectsAABB(boxAt(Vec3{0,0,50}, 0.5f)), "frustum: box behind camera is culled");
        check(!fr.intersectsAABB(boxAt(Vec3{1000,0,0}, 0.5f)), "frustum: far-side box is culled");
    }

    {
        const float nz = 0.5f;
        Mat4 proj = planet::perspectiveInfReverseZ(deg2rad(60.0f), 16.0f/9.0f, nz);

        auto depthAt = [&](float d) {
            planet::Vec4 c = planet::transform(proj, Vec3{0, 0, -d});
            return c.z / c.w;
        };
        check(approx(depthAt(nz), 1.0f, 1e-6f),      "revZ: depth at the near plane is 1");
        check(approx(depthAt(10.0f * nz), 0.1f, 1e-6f), "revZ: depth at 10x near is 0.1");

        bool decreasing = true;
        float prev = depthAt(nz);
        for (double d = 1.0; d <= 1e9; d *= 4.0) {
            const float cur = depthAt(static_cast<float>(d));
            if (!(cur < prev)) decreasing = false;
            prev = cur;
        }
        check(decreasing, "revZ: depth decreases monotonically with distance");
        check(depthAt(1e9f) > 0.0f, "revZ: depth is still > 0 at 1e9 (nothing clips at range)");

        const float d3000km = 3.0e6f;
        const float dep     = depthAt(d3000km);
        const float ulp     = std::nextafter(dep, 1.0f) - dep;
        const float metres  = ulp * d3000km * d3000km / nz;
        check(metres < 1.0f, "revZ: sub-metre depth resolution at 3000 km");

        auto resolutionAt = [](float nearZ, float d) {
            Mat4 p = planet::perspectiveInfReverseZ(deg2rad(60.0f), 16.0f/9.0f, nearZ);
            planet::Vec4 c = planet::transform(p, Vec3{ 0, 0, -d });
            const float dep = c.z / c.w;
            const float u   = std::nextafter(dep, 1.0f) - dep;
            return u * d * d / nearZ;
        };

        const float resFar  = resolutionAt(0.5f,  3.0e6f);
        const float resNear = resolutionAt(0.05f, 3.0e6f);
        std::printf("      [info] depth resolution at 3000 km: near=0.5 -> %.3f m, "
                    "near=0.05 -> %.3f m\n", resFar, resNear);
        check(resNear / resFar < 2.0f && resFar / resNear < 2.0f,
              "revZ: a 10x closer near plane costs at most a binade at 3000 km");
        check(resolutionAt(kNearPlane, 3.0e6f) < 1.0f,
              "revZ: the configured near plane still resolves sub-metre at 3000 km");
        check(kNearPlane < 0.1f,
              "revZ: the near plane is close enough for cockpit geometry");

        Mat4 view = planet::lookAt(Vec3{0,0,0}, Vec3{0,0,-1}, Vec3{0,1,0});
        Frustum fr = Frustum::fromViewProj(proj * view);
        check(fr.intersectsAABB(boxAt(Vec3{0, 0, -1.0e7f}, 1000.0f)),
              "revZ frustum: an extremely distant box is never far-culled");
        check(!fr.intersectsAABB(boxAt(Vec3{0, 0, -0.1f}, 0.01f)),
              "revZ frustum: a box closer than the near plane is culled");
        check(!fr.intersectsAABB(boxAt(Vec3{0, 0, 100.0f}, 1.0f)),
              "revZ frustum: a box behind the camera is culled");
    }

    {
        Noise a(kNoiseSeed), b(kNoiseSeed), c(kNoiseSeed + 99u);
        Vec3 p{0.3f, -0.7f, 1.1f};
        float va = a.fbm(p, 6, 1.3f, 2.0f, 0.5f);
        float vb = b.fbm(p, 6, 1.3f, 2.0f, 0.5f);
        float vc = c.fbm(p, 6, 1.3f, 2.0f, 0.5f);
        check(approx(va, vb, 1e-6f), "noise: same seed => identical value (deterministic)");
        check(!approx(va, vc, 1e-4f), "noise: different seed => different value");

        float mn = 1e9f, mx = -1e9f; double sum = 0, sum2 = 0; int n = 0;
        for (int i = 0; i < 800; ++i) {
            float t = i * 0.137f;
            Vec3 s{ std::sin(t)*1.7f, std::cos(t*0.7f)*1.3f, std::sin(t*1.9f)*2.1f };
            float v = a.fbm(s, 6, 1.3f, 2.0f, 0.5f);
            mn = std::min(mn, v); mx = std::max(mx, v); sum += v; sum2 += double(v)*v; ++n;
        }
        check(mn > -1.05f && mx < 1.05f, "noise: fBm stays within ~[-1,1]");
        double var = sum2/n - (sum/n)*(sum/n);
        check(var > 1e-3, "noise: fBm varies (not a constant field)");

        {
            bool range = true, signedRange = true;
            for (int i = -200; i <= 200; ++i) {
                const float x = i / 200.0f;
                if (Noise::ridge(x) < 0.0f || Noise::ridge(x) > 1.0f) range = false;
                if (Noise::ridgeSigned(x) < -1.0f || Noise::ridgeSigned(x) > 1.0f)
                    signedRange = false;
            }
            check(range, "ridge: the shape stays in [0,1] over the whole Perlin range");
            check(signedRange, "ridge: the recentred shape stays in [-1,1]");
            check(approx(Noise::ridge(0.0f), 1.0f, 1e-6f),
                  "ridge: the crest sits exactly at zero, where Perlin's cusp is");
            check(approx(Noise::ridge(1.0f), 0.0f, 1e-6f) &&
                  approx(Noise::ridge(-1.0f), 0.0f, 1e-6f),
                  "ridge: the extremes fold down to the valley floor");

            check(approx(Noise::ridge(0.3f), Noise::ridge(-0.3f), 1e-6f),
                  "ridge: the shape is symmetric about zero (it is a fold)");

            {
                double rs = 0.0, bs = 0.0, ss = 0.0; long cnt = 0;
                for (int oct = 0; oct < kMaxDetailOctaves; ++oct) {
                    double freq = 345.6 * std::pow(2.0, oct);
                    for (int i = 0; i < 3000; ++i) {
                        const double t = i * 0.00037 + oct * 7.13;
                        const DVec3 d = normalize(DVec3{ std::sin(t * 1.7),
                                                         std::cos(t * 1.13),
                                                         std::sin(t * 0.61) });
                        const float v = a.perlin(d.x * freq, d.y * freq, d.z * freq);
                        rs += Noise::ridgeSigned(v);
                        ss += Noise::ridgeSoftSigned(v);
                        bs += Noise::ridgeBalanced(v);
                        ++cnt;
                    }
                }
                const double measured = rs / cnt, balanced = bs / cnt;
                const double softMeasured = ss / cnt;
                std::printf("         ridge mean: sharp %.5f (constant %.5f),"
                            " soft %.5f (constant %.5f) -> balanced mean %.5f\n",
                            measured, Noise::kRidgeMean,
                            softMeasured, Noise::kRidgeSoftMean, balanced);
                check(std::fabs(measured - Noise::kRidgeMean) < 0.01,
                      "ridge: kRidgeMean still matches the noise it describes");
                check(std::fabs(softMeasured - Noise::kRidgeSoftMean) < 0.01,
                      "ridge: kRidgeSoftMean still matches the softened fold");
                check(std::fabs(balanced) < 0.01,
                      "ridge: the balanced fold is zero-mean (no per-octave lift)");
            }

            bool reduces = true;
            for (int i = 0; i < 60; ++i) {
                const double t = i * 0.211;
                const DVec3 d = normalize(DVec3{ std::sin(t), std::cos(t * 1.3),
                                                 std::sin(t * 0.7) });
                if (a.fbmPrefixRidged(d, 7, 1.3f, 2.0f, 0.5f, 12, 0.0f, 12, 0) !=
                    a.fbmPrefix(d, 7, 1.3f, 2.0f, 0.5f, 12)) reduces = false;
            }
            check(reduces, "ridge: mix 0 reduces bit-exactly to plain fbmPrefix");

            {
                const float gain = kDetailGain;
                const int   norm = kMaxDetailOctaves;
                float worstOver = 0.0f;
                for (int i = 0; i < 200; ++i) {
                    const double t = i * 0.0917;
                    const DVec3 d = normalize(DVec3{ std::cos(t * 1.7), std::sin(t),
                                                     std::cos(t * 0.31) });
                    for (int oct = 1; oct < 10; ++oct) {
                        const float lo = a.fbmPrefixRidged(d, oct,     345.6f, 2.0f, gain, norm, 1.0f, 4, 3);
                        const float hi = a.fbmPrefixRidged(d, oct + 1, 345.6f, 2.0f, gain, norm, 1.0f, 4, 3);

                        float amp = 1.0f;
                        for (int k = 0; k < oct; ++k) amp *= gain;
                        const float bound = 2.0f * amp / Noise::fixedNorm(gain, norm);
                        worstOver = std::max(worstOver,
                                             std::fabs(hi - lo) - bound);
                    }
                }
                std::printf("         ridged prefix: worst overshoot past the added"
                            " octave = %.3e\n", worstOver);
                check(worstOver <= 1e-6f,
                      "ridge: N octaves stay a true prefix of N+1 (LOD-safe)");
            }
        }

    }

    {

        bool unit = true;
        for (int f = 0; f < 6; ++f)
            for (float v = -1.0f; v <= 1.0f; v += 0.5f)
                for (float u = -1.0f; u <= 1.0f; u += 0.5f)
                    if (!approx(length(faceToSphere(f, u, v)), 1.0f, 1e-4f)) unit = false;
        check(unit, "cubesphere: every face sample is a unit sphere direction (Req #4)");

        World world;
        bool seamPos = true, seamNrm = true;
        for (float y = -0.8f; y <= 0.8f; y += 0.2f) {

            const DVec3 abs{ 0.0, 0.0, 0.0 };

            const DVec3 pa = terrainPos(world, faceToSphereD(0, -1.0, y), 4);
            const DVec3 pb = terrainPos(world, faceToSphereD(4,  1.0, y), 4);
            const Vec3 na = sampleSurface(world, 0, -1.0, y, 0.01, 4).normal;
            const Vec3 nb = sampleSurface(world, 4,  1.0, y, 0.01, 4).normal;
            Vertex a = evalVertex(world, 0, -1.0f, y, 0.01f, abs, 4, pa, na);
            Vertex b = evalVertex(world, 4,  1.0f, y, 0.01f, abs, 4, pb, nb);
            if (!(approx(a.px,b.px,1e-2f) && approx(a.py,b.py,1e-2f) && approx(a.pz,b.pz,1e-2f))) seamPos = false;
            const UnpackedNormal an = vertexNormal(a), bn = vertexNormal(b);
            float nd = an.x*bn.x + an.y*bn.y + an.z*bn.z;
            if (nd < 0.99f) seamNrm = false;
        }
        check(seamPos, "cubesphere: adjacent faces share identical edge positions (no seam)");
        check(seamNrm, "cubesphere: adjacent faces share matching edge normals");

        bool landExists = false, waterExists = false;
        for (int f = 0; f < 6; ++f)
            for (float v = -0.95f; v <= 0.95f; v += 0.19f)
                for (float u = -0.95f; u <= 0.95f; u += 0.19f) {
                    DVec3 d = faceToSphereD(f, u, v);
                    float e = sampleLandE(world, d);
                    double r = terrainRadius(kTestParams, sampleShape(world, d));
                    if (r < kPlanetRadius - 1e-3f || r > kPlanetRadius + kTerrainAmplitude + 1e-3f)
                        { landExists = false; break; }
                    if (e > 0.02f) landExists = true;
                    if (e < 0.0f)  waterExists = true;
                }
        check(landExists && waterExists, "cubesphere: both continents and oceans exist (Req #9)");

        const Vec3 deep  = biomeColor(kTestParams, -0.9f, 0.0f, 0.0f, 0.5f);
        const Vec3 shore = biomeColor(kTestParams, 0.01f, 0.0f, 0.0f, 0.5f);

        const Vec3 dry   = biomeColor(kTestParams, kBandLowland, 0.0f, 0.0f, 0.0f);
        const Vec3 wet   = biomeColor(kTestParams, kBandLowland, 0.0f, 0.0f, 1.0f);
        const Vec3 peak  = biomeColor(kTestParams, 1.00f, 0.0f, 0.0f, 0.5f);
        const Vec3 cliff = biomeColor(kTestParams, 0.100f, 1.0f, 0.0f, 0.5f);
        const Vec3 pole  = biomeColor(kTestParams, kBandLowland, 0.0f, 1.0f, 0.5f);
        check(deep.y > deep.x && deep.z > deep.x && deep.x < 0.1f,
              "biome: the deep sea is a dark mineral teal, not blue");
        check(shore.x > shore.y && shore.y > shore.z, "biome: the shore is ochre");
        check(dry.x > dry.y && dry.x > dry.z, "biome: dry lowland is rust");
        check(wet.z > wet.y && wet.x > wet.y, "biome: wet lowland is violet");
        check(!(dry.y > dry.x && dry.y > dry.z) && !(wet.y > wet.x && wet.y > wet.z),
              "biome: no lowland anywhere is green-dominant");

        check(peak.z > peak.x && peak.x > dry.x + 0.2f,
              "biome: peaks are pale against the lowlands, cooling to blue");
        check(approx(cliff.x, kColRock.x, 1e-3f), "biome: a vertical face is bare rock");
        check(pole.z > pole.y && pole.y > pole.x && pole.z > dry.z + 0.3f,
              "biome: the poles are pale cyan");

        {
            float worstJump = 0.0f;
            Vec3 prev = biomeColor(kTestParams, -1.0f, 0.2f, 0.1f, 0.5f);
            for (int i = 1; i <= 4000; ++i) {
                const float e = -1.0f + 2.0f * i / 4000.0f;
                const Vec3 c = biomeColor(kTestParams, e, 0.2f, 0.1f, 0.5f);

                if (std::fabs(e) > 0.002f)
                    worstJump = std::max(worstJump,
                                std::max(std::fabs(c.x - prev.x),
                                std::max(std::fabs(c.y - prev.y),
                                         std::fabs(c.z - prev.z))));
                prev = c;
            }
            std::printf("         biome: worst colour step per 0.0005 of elevation"
                        " = %.4f\n", worstJump);
            check(worstJump < 0.01f, "biome: elevation bands are ramps, not thresholds");

            float worstSlope = 0.0f;
            Vec3 ps = biomeColor(kTestParams, 0.4f, 0.0f, 0.1f, 0.5f);
            for (int i = 1; i <= 2000; ++i) {
                const Vec3 c = biomeColor(kTestParams, 0.4f, i / 1000.0f, 0.1f, 0.5f);
                worstSlope = std::max(worstSlope, std::fabs(c.x - ps.x));
                ps = c;
            }
            check(worstSlope < 0.01f, "biome: the cliff transition is a ramp too");
        }

        {
            int hits[5] = {};
            const int S = 6000;
            for (int i = 0; i < S; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / S;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const TerrainShape shp = sampleShape(world, d);
                if (shp.landE <= 0.0f) continue;

                const float e = shp.landE * shp.amp;
                const float lat = std::fabs(static_cast<float>(d.y));
                const float mo  = moistureAt(world, d);
                if (lat >= kLatPolar)                       { ++hits[4]; continue; }
                if (e < kBandShore)                           ++hits[0];
                else if (e < kBandLowland && mo < 0.45f)      ++hits[1];
                else if (e < kBandLowland)                    ++hits[2];
                else                                          ++hits[3];
            }
            std::printf("         biome zones over the sphere: shore %d, dry %d,"
                        " wet %d, upland %d, polar %d\n",
                        hits[0], hits[1], hits[2], hits[3], hits[4]);
            bool all = true;
            for (int i = 0; i < 5; ++i) all = all && hits[i] > 0;
            check(all, "biome: every zone in the palette occurs somewhere");
        }

        {
            double sm = 0, se = 0, smm = 0, see = 0, sme = 0; int n2 = 0;
            for (int i = 0; i < 4000; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / 4000;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const double m = moistureAt(world, d), e = sampleLandE(world, d);
                sm += m; se += e; smm += m*m; see += e*e; sme += m*e; ++n2;
            }
            const double cov = sme/n2 - (sm/n2)*(se/n2);
            const double corr = cov / std::sqrt((smm/n2 - (sm/n2)*(sm/n2)) *
                                                (see/n2 - (se/n2)*(se/n2)));
            std::printf("         moisture vs elevation correlation: %.3f\n", corr);
            check(std::fabs(corr) < 0.2,
                  "biome: moisture is an independent field, not a copy of height");
        }

        {
            const double eps = 30.0;
            int    land = 0, gentle = 0;
            double worst = 0.0, sumSlope = 0.0;
            const int S = 3000;
            for (int i = 0; i < S; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / S;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;
                ++land;
                auto radAt = [&](const DVec3& q) {
                    const DVec3 qn = normalize(q);
                    return surfaceRadiusAt(world, qn, kPhysicsDetailOctaves);
                };
                const double r0 = radAt(d);
                const DVec3 ref = (std::fabs(d.y) < 0.9) ? DVec3{0,1,0} : DVec3{1,0,0};
                const DVec3 t1 = normalize(cross(ref, d)), t2 = cross(d, t1);
                const double g1 = (radAt(d * r0 + t1 * eps) - r0) / eps;
                const double g2 = (radAt(d * r0 + t2 * eps) - r0) / eps;
                const double deg = std::atan(std::sqrt(g1*g1 + g2*g2)) * 180.0 / kPi;
                sumSlope += deg;
                worst = std::max(worst, deg);
                if (deg < 20.0) ++gentle;
            }
            const double frac = double(gentle) / std::max(1, land);
            std::printf("         land slope over %d land samples: mean %.1f deg,"
                        " worst %.1f deg, %.0f%% under 20 deg\n",
                        land, sumSlope / std::max(1, land), worst, frac * 100.0);
            check(land > S / 8, "cubesphere: a useful fraction of the sphere is land");
            check(frac > 0.5,
                  "terrain: most land is gentle enough to put a ship down on");
        }

        {
            int moved = 0;
            double worstCoastShift = 0.0;
            for (int i = 0; i < 3000; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / 3000;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const float e = sampleLandE(world, d);

                for (int lvl = 0; lvl <= kMaxDepth; lvl += 3) {
                    const int oct = detailOctavesForLevel(lvl);
                    const double r = surfaceRadius(world, d, sampleShape(world, d), oct);
                    if (e <= 0.0f && r != kPlanetRadius) ++moved;
                    if (e <= 0.0f) continue;

                    if (r < kPlanetRadius) ++moved;
                    worstCoastShift = std::max(worstCoastShift,
                                               std::fabs(r - surfaceRadiusAt(world, d,
                                                                             kPhysicsDetailOctaves)));
                }
            }
            std::printf("         LOD invariance: %d coastline disagreements,"
                        " worst land height spread over all levels %.1f m\n",
                        moved, worstCoastShift);
            check(moved == 0,
                  "terrain: land and sea are decided identically at every LOD level");

            check(worstCoastShift < kDetailAmplitude + kSpireAmplitude,
                  "terrain: the LOD-faded bands stay inside their amplitudes");
        }

        {
            const double pxPerRad = (kWindowHeight * 0.5) /
                                    std::tan(space::deg2rad(kFovYDegrees) * 0.5);

            DVec3 landDir = normalize(DVec3{ 0.3, 0.5, 0.8 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) > 0.30f) { landDir = d; break; }
            }
            check(sampleLandE(world, landDir) > 0.0f,
                  "lod: the pop measurement is taken over land, not ocean");
            std::printf("         LOD pop before geomorphing:\n");
            double worstPx = 0.0;
            for (int level : { 8, 11, 14, 17 }) {
                const double size = 2.0 / double(1 << level);

                double su = 0.0, sv = 0.0;
                const int face = scatterFaceOf(landDir, su, sv);
                const int last = (1 << level) - 1;
                const int ix = std::min(last, std::max(0, int((su + 1.0) / size)));
                const int iy = std::min(last, std::max(0, int((sv + 1.0) / size)));

                const double u0 = -1.0 + size * ix, v0 = -1.0 + size * iy;
                const double step = size / kPatchResolution;
                const ParentSurface ps = sampleParentSurface(world, face, level,
                                                             u0, v0, step,
                                                             kPatchResolution);
                const int   detail = detailOctavesForLevel(level);
                const DVec3 c00 = faceToSphereD(face, u0, v0) * kPlanetRadius;
                const DVec3 c10 = faceToSphereD(face, u0 + size, v0) * kPlanetRadius;
                const double worldEdge = space::length(c10 - c00);
                const double dMerge = 2.0 * worldEdge / kSplitFactor;

                double worst = 0.0, sum2 = 0.0; int n = 0;
                for (int j = 0; j <= kPatchResolution; ++j)
                    for (int i = 0; i <= kPatchResolution; ++i) {
                        const DVec3 d = faceToSphereD(face, u0 + step * i, v0 + step * j);
                        const DVec3 fine = terrainPos(world, d, detail);
                        const double e = space::length(fine - ps.at(i, j));
                        worst = std::max(worst, e); sum2 += e * e; ++n;
                    }
                const double px = worst / dMerge * pxPerRad;
                worstPx = std::max(worstPx, px);
                std::printf("           level %2d: edge %8.0f m, swap at %8.0f m,"
                            " pop max %6.2f m (rms %5.2f) = %5.1f px\n",
                            level, worldEdge, dMerge, worst,
                            std::sqrt(sum2 / n), px);
            }

            check(worstPx < kWindowHeight * 0.1,
                  "lod: the raw pop stays far below a screenful");
        }

        {
            const int level = 12, N = kPatchResolution;
            const double size = 2.0 / double(1 << level);
            double su = 0.0, sv = 0.0;
            DVec3 ld = normalize(DVec3{ 0.3, 0.5, 0.8 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) > 0.30f) { ld = d; break; }
            }
            const int face = scatterFaceOf(ld, su, sv);
            const int ix = std::min((1 << level) - 1, std::max(0, int((su + 1.0) / size)));
            const int iy = std::min((1 << level) - 1, std::max(0, int((sv + 1.0) / size)));
            const double u0 = -1.0 + size * ix, v0 = -1.0 + size * iy;
            const double step = size / N;
            const ParentSurface ps = sampleParentSurface(world, face, level, u0, v0, step, N);
            check(ps.valid, "geomorph: a patch below the roots has a parent surface");

            const int coarseDetail = detailOctavesForLevel(level - 1);
            double worstEven = 0.0, worstEdge = 0.0, worstDiag = 0.0;
            for (int j = 0; j <= N; ++j)
                for (int i = 0; i <= N; ++i) {
                    const DVec3 got = ps.at(i, j);
                    auto parentAt = [&](int pi, int pj) {
                        return terrainPos(world, faceToSphereD(face, u0 + step * 2.0 * pi,
                                                                     v0 + step * 2.0 * pj),
                                          coarseDetail);
                    };
                    const int pi = i >> 1, pj = j >> 1;
                    if (!(i & 1) && !(j & 1))
                        worstEven = std::max(worstEven, space::length(got - parentAt(pi, pj)));
                    else if ((i & 1) && !(j & 1))
                        worstEdge = std::max(worstEdge, space::length(
                            got - (parentAt(pi, pj) + parentAt(pi + 1, pj)) * 0.5));
                    else if (!(i & 1) && (j & 1))
                        worstEdge = std::max(worstEdge, space::length(
                            got - (parentAt(pi, pj) + parentAt(pi, pj + 1)) * 0.5));
                    else
                        worstDiag = std::max(worstDiag, space::length(
                            got - (parentAt(pi + 1, pj) + parentAt(pi, pj + 1)) * 0.5));
                }
            check(worstEven < 1e-9, "geomorph: even/even vertices ARE parent vertices");
            check(worstEdge < 1e-9, "geomorph: odd edge vertices bisect a parent edge");
            check(worstDiag < 1e-9, "geomorph: odd/odd vertices bisect the parent's DIAGONAL");

            double worstBilinearGap = 0.0;
            for (int j = 1; j <= N; j += 2)
                for (int i = 1; i <= N; i += 2) {
                    const int pi = i >> 1, pj = j >> 1;
                    auto parentAt = [&](int a, int b) {
                        return terrainPos(world, faceToSphereD(face, u0 + step * 2.0 * a,
                                                                     v0 + step * 2.0 * b),
                                          coarseDetail);
                    };
                    const DVec3 bilinear = (parentAt(pi, pj) + parentAt(pi + 1, pj) +
                                            parentAt(pi, pj + 1) + parentAt(pi + 1, pj + 1)) * 0.25;
                    worstBilinearGap = std::max(worstBilinearGap,
                                                space::length(ps.at(i, j) - bilinear));
                }
            std::printf("         geomorph: diagonal vs bilinear midpoint differs by"
                        " up to %.3f m\n", worstBilinearGap);
            check(worstBilinearGap > 1e-3,
                  "geomorph: ... and the bilinear average would NOT have been the same");

            const PatchMeshData deep = generatePatch(world, face, level, ix, iy, N);
            double worstCarried = 0.0;
            for (int j = 0; j <= N; ++j)
                for (int i = 0; i <= N; ++i) {
                    const Vertex& vx = deep.vertices[j * (N + 1) + i];
                    const DVec3 want = ps.at(i, j) - deep.origin;
                    worstCarried = std::max(worstCarried, std::max(
                        std::fabs(vx.cx - want.x), std::max(
                        std::fabs(vx.cy - want.y), std::fabs(vx.cz - want.z))));
                }
            check(worstCarried < 0.05,
                  "geomorph: generatePatch stores the parent surface in the vertex");

            const PatchMeshData root = generatePatch(world, 0, 0, 0, 0, N);
            bool rootStill = true;
            for (const Vertex& vx : root.vertices)
                rootStill = rootStill && vx.cx == vx.px && vx.cy == vx.py && vx.cz == vx.pz;
            check(rootStill, "geomorph: a level-0 root has no parent and cannot morph");

            bool skirtRides = true;
            const int grid = (N + 1) * (N + 1);
            for (size_t k = grid; k < root.vertices.size(); ++k) {
                const Vertex& sv2 = root.vertices[k];
                if (!(sv2.cx == sv2.px && sv2.cy == sv2.py && sv2.cz == sv2.pz))
                    skirtRides = false;
            }
            check(skirtRides, "geomorph: root skirts do not morph either");
            {
                double worstDrift = 0.0;
                for (size_t k = grid; k < deep.vertices.size(); ++k) {
                    const Vertex& sk = deep.vertices[k];

                    const DVec3 fine{ sk.px, sk.py, sk.pz };
                    const DVec3 crse{ sk.cx, sk.cy, sk.cz };
                    (void)fine; (void)crse;
                }

                for (size_t k = grid; k < deep.vertices.size(); ++k) {
                    const Vertex& sk = deep.vertices[k];
                    const DVec3 off{ sk.cx - sk.px, sk.cy - sk.py, sk.cz - sk.pz };
                    double best = 1e30;
                    for (int e = 0; e <= N; ++e) {
                        for (int which = 0; which < 4; ++which) {
                            const int i = which == 0 ? e : which == 1 ? e : which == 2 ? 0 : N;
                            const int j = which == 0 ? 0 : which == 1 ? N : e;
                            const Vertex& ev = deep.vertices[j * (N + 1) + i];
                            const DVec3 eoff{ ev.cx - ev.px, ev.cy - ev.py, ev.cz - ev.pz };
                            best = std::min(best, space::length(off - eoff));
                        }
                    }
                    worstDrift = std::max(worstDrift, best);
                }
                check(worstDrift < 1e-3,
                      "geomorph: every skirt carries its edge vertex's morph offset");
            }
        }

        {
            const int L = 12, N = kPatchResolution;
            const double size = 2.0 / double(1 << L);
            const double step = size / N;

            const int ixA = 1501, iyA = 900;
            const double u0A = -1.0 + size * ixA,  v0A = -1.0 + size * iyA;
            const double u0B = -1.0 + size * (ixA + 1);
            const ParentSurface A = sampleParentSurface(world, 2, L, u0A, v0A, step, N);
            const ParentSurface B = sampleParentSurface(world, 2, L, u0B, v0A, step, N);
            check((ixA >> 1) != ((ixA + 1) >> 1),
                  "geomorph: the seam test really does straddle two parents");

            double worstSeamFine = 0.0, worstSeamCoarse = 0.0, worstSeamMorph = 0.0;
            for (int j = 0; j <= N; ++j) {
                const DVec3 d = faceToSphereD(2, u0A + size, v0A + step * j);
                const DVec3 fineA = terrainPos(world, d, detailOctavesForLevel(L));
                const DVec3 fineB = fineA;
                worstSeamFine = std::max(worstSeamFine, space::length(fineA - fineB));
                const DVec3 cA = A.at(N, j), cB = B.at(0, j);
                worstSeamCoarse = std::max(worstSeamCoarse, space::length(cA - cB));

                for (int k = 0; k <= 10; ++k) {
                    const double t = k / 10.0;
                    worstSeamMorph = std::max(worstSeamMorph,
                        space::length((fineA * (1.0 - t) + cA * t) -
                                      (fineB * (1.0 - t) + cB * t)));
                }
            }
            std::printf("         geomorph seam (two parents): fine %.2e m,"
                        " coarse %.2e m, worst mid-morph %.2e m\n",
                        worstSeamFine, worstSeamCoarse, worstSeamMorph);
            check(worstSeamCoarse < 1e-6,
                  "geomorph: patches under DIFFERENT parents agree on the shared edge");
            check(worstSeamMorph < 1e-6,
                  "geomorph: ... at every point of the blend, not just its ends");

            {
                const int last = (1 << L) - 1;
                const ParentSurface P = sampleParentSurface(world, 0, L, -1.0,
                                                            -1.0 + size * iyA, step, N);
                const ParentSurface Q = sampleParentSurface(world, 4, L,
                                                            -1.0 + size * last,
                                                            -1.0 + size * iyA, step, N);
                double worstFace = 0.0;
                for (int j = 0; j <= N; ++j)
                    worstFace = std::max(worstFace, space::length(P.at(0, j) - Q.at(N, j)));
                std::printf("         geomorph seam (cube-face edge): %.2e m\n", worstFace);
                check(worstFace < 1e-6,
                      "geomorph: the morph target is seamless across cube faces too");
            }

            {
                const PatchMeshData par = generatePatch(world, 2, L - 1,
                                                        ixA >> 1, iyA >> 1, N);
                const int offI = (ixA & 1) * (N / 2), offJ = (iyA & 1) * (N / 2);
                double worstEnd = 0.0;
                for (int j = 0; j <= N; j += 2)
                    for (int i = 0; i <= N; i += 2) {
                        const DVec3 mine = A.at(i, j);
                        const Vertex& pv = par.vertices[(offJ + j / 2) * (N + 1) +
                                                        (offI + i / 2)];
                        const DVec3 theirs = par.origin + DVec3{ pv.px, pv.py, pv.pz };
                        worstEnd = std::max(worstEnd, space::length(mine - theirs));
                    }
                std::printf("         geomorph: full morph lands within %.3f m of the"
                            " parent patch's own vertices\n", worstEnd);
                check(worstEnd < 0.05,
                      "geomorph: a finished morph IS the parent's geometry");
            }
        }

        {
            const double pxPerRad = (kWindowHeight * 0.5) /
                                    std::tan(space::deg2rad(kFovYDegrees) * 0.5);
            auto smoothstepf = [](double a, double b, double x) {
                double t = (x - a) / (b - a);
                t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
                return t * t * (3.0 - 2.0 * t);
            };
            DVec3 ld = normalize(DVec3{ 0.3, 0.5, 0.8 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) > 0.30f) { ld = d; break; }
            }
            double su = 0.0, sv = 0.0;
            const int face = scatterFaceOf(ld, su, sv);

            std::printf("         LOD pop AFTER geomorphing:\n");
            double worstResidualPx = 0.0, worstUnfinished = 0.0;
            for (int level : { 8, 11, 14, 17 }) {
                const int N = kPatchResolution;
                const double size = 2.0 / double(1 << level);
                const int last = (1 << level) - 1;
                const int ix = std::min(last, std::max(0, int((su + 1.0) / size)));
                const int iy = std::min(last, std::max(0, int((sv + 1.0) / size)));
                const PatchBounds pb = computePatchBounds(face, level, ix, iy, world);
                const double morphEnd = 2.0 * pb.worldEdge / double(kSplitFactor);

                const DVec3 outward = normalize(pb.surfaceCenter);
                const DVec3 cam = pb.surfaceCenter + outward * (morphEnd + pb.surfaceRadius);

                const PatchMeshData pm = generatePatch(world, face, level, ix, iy, N);
                double worstResidual = 0.0, minMorph = 1.0;
                for (int j = 0; j <= N; ++j)
                    for (int i = 0; i <= N; ++i) {
                        const Vertex& v = pm.vertices[j * (N + 1) + i];
                        const DVec3 fine  = pm.origin + DVec3{ v.px, v.py, v.pz };
                        const DVec3 crse  = pm.origin + DVec3{ v.cx, v.cy, v.cz };
                        const double d    = space::length(fine - cam);
                        const double m    = smoothstepf(morphEnd * kMorphBandStart,
                                                        morphEnd, d);
                        minMorph = std::min(minMorph, m);

                        worstResidual = std::max(worstResidual,
                                                 (1.0 - m) * space::length(fine - crse));
                    }
                const double px = worstResidual / morphEnd * pxPerRad;
                worstResidualPx = std::max(worstResidualPx, px);
                worstUnfinished = std::max(worstUnfinished, 1.0 - minMorph);
                std::printf("           level %2d: least-morphed vertex %.4f,"
                            " residual pop %.4f m = %.3f px\n",
                            level, minMorph, worstResidual, px);
            }
            check(worstUnfinished < 1e-9,
                  "geomorph: every vertex is fully morphed before its patch is swapped");
            check(worstResidualPx < 0.01,
                  "geomorph: nothing measurable is left to pop (was 2.4-7.6 px)");
        }

        {

            bool ok = true, scalesOk = true;
            double worstSum = 0.0;
            float minAmp = 2.0f, maxAmp = 0.0f;
            const int S = 60000;
            for (int i = 0; i < S; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / S;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };

                const float e   = sampleLandE(world, d);
                const float rel = reliefAt(world, d);
                if (rel < 0.0f || rel > 1.0f) ok = false;

                const BiomeWeights b = biomeWeightsAt(kTestParams, e, rel, float(d.y));
                double sum = 0.0;
                for (int k = 0; k < kBiomeCount; ++k) {
                    if (b.w[k] < -1e-6f || b.w[k] > 1.0f + 1e-6f) ok = false;
                    sum += b.w[k];
                }
                worstSum = std::max(worstSum, std::fabs(sum - 1.0));

                const BiomeScales s = blendScales(kTestParams, b);
                const float sc[4] = { s.amp, s.detail, s.ridge, s.terrace };
                for (float v : sc)
                    if (!(v > 0.0f && v <= 1.0f + 1e-6f)) scalesOk = false;
                minAmp = std::min(minAmp, s.amp);
                maxAmp = std::max(maxAmp, s.amp);
            }
            std::printf("         biome weights: worst |sum-1| = %.2e,"
                        " amplitude scale spans %.2f..%.2f\n",
                        worstSum, minAmp, maxAmp);
            check(ok, "biome: every weight is in [0,1]");
            check(worstSum < 1e-5, "biome: the weights sum to one");
            check(scalesOk, "biome: every blended scale stays inside (0,1]");

            {
                float worstJump = 0.0f;
                for (int i = 0; i < 400; ++i) {
                    const double t = i * 0.0271;
                    const DVec3 a = normalize(DVec3{ std::sin(t * 1.3),
                                                     std::cos(t), std::sin(t * 0.7) });
                    const DVec3 b2 = normalize(DVec3{ a.x + 1e-6, a.y, a.z });
                    const BiomeScales s1 = blendScales(kTestParams, biomeWeightsAt(kTestParams,
                        std::max(0.01f, sampleLandE(world, a)), reliefAt(world, a), float(a.y)));
                    const BiomeScales s2 = blendScales(kTestParams, biomeWeightsAt(kTestParams,
                        std::max(0.01f, sampleLandE(world, b2)), reliefAt(world, b2), float(b2.y)));
                    worstJump = std::max(worstJump, std::fabs(s1.amp - s2.amp));
                }
                check(worstJump < 1e-3f, "biome: the amplitude scale is continuous");
            }

            {
                auto corr = [&](float (*f)(const World&, const DVec3&),
                                float (*g)(const World&, const DVec3&)) {
                    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0; int n = 0;
                    for (int i = 0; i < 8000; ++i) {
                        const double y  = 1.0 - 2.0 * (i + 0.5) / 8000;
                        const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                        const double th = 2.399963229728653 * i;
                        const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                        const double a = f(world, d), b3 = g(world, d);
                        sa += a; sb += b3; saa += a*a; sbb += b3*b3; sab += a*b3; ++n;
                    }
                    const double cov = sab/n - (sa/n)*(sb/n);
                    return cov / std::sqrt((saa/n - (sa/n)*(sa/n)) *
                                           (sbb/n - (sb/n)*(sb/n)));
                };
                const double cRelElev = corr(reliefAt, sampleLandE);
                const double cRelMoist = corr(reliefAt, moistureAt);
                std::printf("         relief correlations: vs elevation %.3f,"
                            " vs moisture %.3f\n", cRelElev, cRelMoist);
                check(std::fabs(cRelElev) < 0.2,
                      "biome: relief is independent of how high the land already is");
                check(std::fabs(cRelMoist) < 0.2,
                      "biome: relief is independent of moisture");
            }

            {
                std::vector<float> vals;
                const int N3 = 20000;
                for (int i = 0; i < N3; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N3;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    if (sampleLandE(world, d) > 0.0f) vals.push_back(reliefAt(world, d));
                }
                std::sort(vals.begin(), vals.end());
                auto q = [&](double f) { return vals[size_t(f * (vals.size() - 1))]; };
                std::printf("         relief over land: p10 %.3f p25 %.3f p50 %.3f"
                            " p75 %.3f p90 %.3f (min %.3f max %.3f)\n",
                            q(0.10), q(0.25), q(0.50), q(0.75), q(0.90),
                            vals.front(), vals.back());
            }

            {
                const double eps = 200.0;
                double worst = 0.0, sum = 0.0; int n = 0;
                const int N4 = 50000;
                for (int i = 0; i < N4; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N4;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    const float e = sampleLandE(world, d);
                    if (e <= 0.05f) continue;
                    const DVec3 ref = (std::fabs(d.y) < 0.9) ? DVec3{0,1,0} : DVec3{1,0,0};
                    const DVec3 t1 = normalize(cross(ref, d));
                    auto ampAt = [&](const DVec3& q) {
                        const DVec3 qn = normalize(q);
                        return double(blendScales(kTestParams, biomeWeightsAt(kTestParams,
                            e, reliefAt(world, qn), float(qn.y))).amp);
                    };
                    const DVec3 p0 = d * kPlanetRadius;
                    const double g = (ampAt(p0 + t1 * eps) - ampAt(p0 - t1 * eps)) /
                                     (2.0 * eps);

                    const double slope = std::fabs(g) * double(e) * kTerrainAmplitude;
                    worst = std::max(worst, slope); sum += slope; ++n;
                }
                std::printf("         biome boundary slope: worst %.3f (%.1f deg),"
                            " mean %.3f (%.1f deg)\n",
                            worst, std::atan(worst) * 180.0 / kPi,
                            sum / std::max(1, n),
                            std::atan(sum / std::max(1, n)) * 180.0 / kPi);
                check(worst < 0.6, "biome: no boundary is a wall (under 31 degrees)");
                check(sum / std::max(1, n) < 0.10,
                      "biome: boundaries are gentle on average");
            }

            {
                int dom[kBiomeCount] = {}, land = 0;
                const int N2 = 20000;
                for (int i = 0; i < N2; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N2;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    const float e = sampleLandE(world, d);
                    if (e <= 0.0f) continue;
                    ++land;
                    const BiomeWeights b = biomeWeightsAt(kTestParams, e, reliefAt(world, d), float(d.y));
                    int best = kBiomePlains;
                    for (int k = 1; k < kBiomeCount; ++k)
                        if (b.w[k] > b.w[best]) best = k;
                    ++dom[best];
                }
                std::printf("         biome share of land: plains %.0f%%,"
                            " mesa %.0f%%, mountains %.0f%%\n",
                            100.0 * dom[kBiomePlains]    / std::max(1, land),
                            100.0 * dom[kBiomeMesa]      / std::max(1, land),
                            100.0 * dom[kBiomeMountains] / std::max(1, land));
                bool alive = true;
                for (int k = kBiomePlains; k < kBiomeCount; ++k)
                    alive = alive && dom[k] * 20 > land;
                check(alive, "biome: none of the three land biomes is dead");

                int pure = 0;
                for (int i = 0; i < N2; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N2;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    if (sampleLandE(world, d) <= 0.0f) continue;
                    const BiomeWeights b = biomeWeightsAt(kTestParams, 1.0f, reliefAt(world, d),
                                                          float(d.y));
                    float top = 0.0f;
                    for (int k = 1; k < kBiomeCount; ++k) top = std::max(top, b.w[k]);
                    if (top > 0.8f) ++pure;
                }
                std::printf("         land that reads as ONE biome (weight > 0.8):"
                            " %.0f%%\n", 100.0 * pure / std::max(1, land));
                check(pure * 2 > land,
                      "biome: over half the land reads as one biome, not a blend");
            }
        }

        {
            std::vector<float> ev;
            const int SB = 40000;
            for (int i = 0; i < SB; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / SB;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const TerrainShape sh = sampleShape(world, d);
                if (sh.landE <= 0.0f) continue;

                ev.push_back(sh.landE * sh.amp + sh.orogeny * kOrogenyColourWeight);
            }
            std::sort(ev.begin(), ev.end());
            auto q = [&](double f) { return ev[size_t(f * (ev.size() - 1))]; };
            std::printf("         colour elevation over land: p25 %.3f p50 %.3f"
                        " p75 %.3f p90 %.3f p99 %.3f max %.3f\n",
                        q(0.25), q(0.50), q(0.75), q(0.90), q(0.99), ev.back());
            std::printf("           in metres: p50 %.0f  p90 %.0f  max %.0f\n",
                        q(0.50) * kTerrainAmplitude, q(0.90) * kTerrainAmplitude,
                        ev.back() * kTerrainAmplitude);

            int inBand[5] = {};
            for (float e : ev) {
                if      (e < kBandShore)    ++inBand[0];
                else if (e < kBandLowland)  ++inBand[1];
                else if (e < kBandHighland) ++inBand[2];
                else if (e < kBandPeak)     ++inBand[3];
                else                        ++inBand[4];
            }
            std::printf("           share of land per band: shore %.1f%%"
                        " lowland %.1f%% mid %.1f%% highland %.1f%% peak %.1f%%\n",
                        100.0 * inBand[0] / ev.size(), 100.0 * inBand[1] / ev.size(),
                        100.0 * inBand[2] / ev.size(), 100.0 * inBand[3] / ev.size(),
                        100.0 * inBand[4] / ev.size());
            bool reached = true;
            for (int k = 0; k < 5; ++k) reached = reached && inBand[k] * 100 > int(ev.size());
            check(reached, "biome: every elevation band covers at least 1% of the land");
        }

        {
            std::vector<float> sv;
            const int SS = 6000;
            for (int i = 0; i < SS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / SS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;

                const double delta = 2.0 / double(1 << kMaxDepth) / kPatchResolution;
                double su = 0.0, sv2 = 0.0;
                const int face = scatterFaceOf(d, su, sv2);
                const SurfaceSample ss = sampleSurface(world, face, su, sv2,
                                                       delta * 0.5,
                                                       kPhysicsDetailOctaves);
                sv.push_back(1.0f - float(space::dot(space::toF(d), ss.normal)));
            }
            std::sort(sv.begin(), sv.end());
            auto q = [&](double f) { return sv[size_t(f * (sv.size() - 1))]; };
            auto deg = [](float s) { return std::acos(std::max(-1.0, std::min(1.0,
                                        1.0 - double(s)))) * 180.0 / kPi; };
            std::printf("         surface slope over land: p50 %.4f (%.1f deg)"
                        " p90 %.4f (%.1f deg) p99 %.4f (%.1f deg) max %.4f (%.1f deg)\n",
                        q(0.50), deg(q(0.50)), q(0.90), deg(q(0.90)),
                        q(0.99), deg(q(0.99)), sv.back(), deg(sv.back()));
            int rocky = 0, partly = 0;
            for (float s : sv) {
                if (s >= kSlopeRock) ++rocky;
                else if (s > kSlopeSoft) ++partly;
            }
            std::printf("           kSlopeSoft %.2f (%.0f deg) .. kSlopeRock %.2f"
                        " (%.0f deg): %.1f%% of land shows any rock, %.1f%% is full rock\n",
                        kSlopeSoft, deg(kSlopeSoft), kSlopeRock, deg(kSlopeRock),
                        100.0 * (rocky + partly) / sv.size(),
                        100.0 * rocky / sv.size());
            check(100.0 * (rocky + partly) / sv.size() > 2.0,
                  "biome: exposed rock reaches a visible share of the land");
        }

        {
            const int N = kPatchResolution;
            const int lvl = 12;
            const int ix = 5, iy = 9, face = 3;

            const PatchMeshData a = generatePatch(world, face, lvl, ix,     iy, N);
            const PatchMeshData b = generatePatch(world, face, lvl, ix + 1, iy, N);

            double worstPos = 0.0, worstNrm = 0.0;
            int    exactN = 0, exactP = 0;
            for (int j = 0; j <= N; ++j) {
                const Vertex& va = a.vertices[size_t(j) * (N + 1) + N];
                const Vertex& vb = b.vertices[size_t(j) * (N + 1) + 0];

                const DVec3 pa = a.origin + DVec3{ va.px, va.py, va.pz };
                const DVec3 pb = b.origin + DVec3{ vb.px, vb.py, vb.pz };
                worstPos = std::max(worstPos, space::length(pa - pb));
                if (space::length(pa - pb) == 0.0) ++exactP;

                const UnpackedNormal na = vertexNormal(va), nb = vertexNormal(vb);
                const double dn = std::sqrt((na.x - nb.x) * (na.x - nb.x) +
                                            (na.y - nb.y) * (na.y - nb.y) +
                                            (na.z - nb.z) * (na.z - nb.z));
                worstNrm = std::max(worstNrm, dn);
                if (va.nx == vb.nx && va.ny == vb.ny) ++exactN;
            }
            std::printf("         patch seam (level %d): %d/%d positions and"
                        " %d/%d normals bit-identical\n",
                        lvl, exactP, N + 1, exactN, N + 1);
            std::printf("           worst position gap %.3e m, worst normal gap"
                        " %.3e\n", worstPos, worstNrm);
            check(exactN == N + 1,
                  "patch: neighbours derive bit-identical normals on a shared edge");

            const double edge = kPi * 0.5 * kPlanetRadius / double(1 << lvl);
            const double ulp  = edge / 16777216.0;
            std::printf("           float bound: %.1f um per ULP at this level,"
                        " measured %.1f um\n", ulp * 1e6, worstPos * 1e6);
            check(worstPos < 4.0 * ulp,
                  "patch: ... and the shared vertices agree to a few float ULPs");

            int exactC = 0;
            for (int j = 0; j <= N; ++j) {
                const Vertex& va = a.vertices[size_t(j) * (N + 1) + N];
                const Vertex& vb = b.vertices[size_t(j) * (N + 1) + 0];
                if (va.r == vb.r && va.g == vb.g && va.b == vb.b &&
                    va.m0 == vb.m0 && va.m1 == vb.m1 && va.m2 == vb.m2 &&
                    va.m3 == vb.m3 && va.m4 == vb.m4 && va.m5 == vb.m5) ++exactC;
            }
            check(exactC == N + 1,
                  "patch: ... and agree on colour and material along it");
        }

        {
            const DVec3 site = findSunlitLand(world);
            const double slope = footprintSlope(world, site) * 180.0 / kPi;
            const TerrainShape sh = sampleShape(world, site);
            std::printf("         landing site: %.2f deg over a %.0f m footprint,"
                        " %.0f m above sea level\n", slope, kLandingFootprint,
                        terrainRadius(world.p, sh) - kPlanetRadius);
            check(sh.landE > 0.0f, "landing: the chosen site is on land");
            check(slope <= kMaxLandingSlope * 180.0 / kPi + 1e-3,
                  "landing: ... and flat enough for four legs to reach");

            check(space::length(site - DVec3{ 0.0, 1.0, 0.0 }) > 1e-9,
                  "landing: ... and was actually found, not defaulted");
        }

        {
            const PlateSystem ps = buildPlates(kNoiseSeed, [&](const DVec3& d){ return sampleLandE(world, d) > 0.0f; });

            {
                int worstA = -1;
                double closestPair = 1e9;
                for (int a = 0; a < kPlateCount; ++a)
                    for (int b = a + 1; b < kPlateCount; ++b)
                        closestPair = std::min(closestPair,
                            std::acos(std::max(-1.0, std::min(1.0,
                                space::dot(ps.seed[a], ps.seed[b])))));
                std::printf("         plates: %d seeds, closest pair %.1f deg apart\n",
                            kPlateCount, closestPair * 180.0 / kPi);
                check(closestPair > 0.15,
                      "plates: no two seeds sit on top of each other");
                (void)worstA;
            }

            {
                int count[kPlateCount] = {};
                const int N = 40000;
                for (int i = 0; i < N; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N;
                    const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    ++count[plateAt(ps, DVec3{ std::cos(th)*r, y, std::sin(th)*r }).a];
                }
                int lo = N, hi = 0;
                for (int i = 0; i < kPlateCount; ++i) {
                    lo = std::min(lo, count[i]); hi = std::max(hi, count[i]);
                }
                std::printf("         plates: cell share %.1f%% to %.1f%% of the"
                            " sphere (ratio %.2f)\n",
                            100.0 * lo / N, 100.0 * hi / N, double(hi) / double(lo));
                check(lo > 0, "plates: every plate owns some of the sphere");
                check(double(hi) / double(lo) < 3.0,
                      "plates: no plate is more than 3x another");
            }

            {
                const int active = activeRangeCount(ps);
                std::printf("         plates: %d boundaries carry a range\n", active);
                check(active >= 3 && active <= 6,
                      "plates: between 3 and 6 boundaries carry a range");
            }

            {
                std::vector<RangeSpec> rs;
                for (int a = 0; a < kPlateCount; ++a)
                    for (int b = a + 1; b < kPlateCount; ++b)
                        if (ps.at(a, b).active) rs.push_back(ps.at(a, b));

                double worstMinDiff = 1e9;
                for (size_t i = 0; i < rs.size(); ++i)
                    for (size_t j = i + 1; j < rs.size(); ++j) {

                        const double d[7] = {
                            std::fabs(rs[i].crestWidth  - rs[j].crestWidth)  / kRangeCrestWidth,
                            std::fabs(rs[i].crestHeight - rs[j].crestHeight) / kRangeCrestHeight,
                            std::fabs(rs[i].sharpness   - rs[j].sharpness)   / 4.0,
                            std::fabs(rs[i].asymmetry   - rs[j].asymmetry)   / 2.0,
                            std::fabs(rs[i].terrace     - rs[j].terrace),
                            std::fabs(double(rs[i].ridges) - double(rs[j].ridges))
                                / double(kRangeRidgeMax),
                            std::fabs(rs[i].ridgeGap    - rs[j].ridgeGap)
                                / (kRangeRidgeGapHi - kRangeRidgeGapLo),
                        };
                        double biggest = 0.0;
                        for (double x : d) biggest = std::max(biggest, x);
                        worstMinDiff = std::min(worstMinDiff, biggest);
                    }
                std::printf("         plates: the two most alike ranges still"
                            " differ by %.2f in their strongest dimension\n",
                            worstMinDiff);
                for (size_t i = 0; i < rs.size(); ++i)
                    std::printf("           range %zu: %d ridge(s) at gap %.2f, crest"
                                " %.4f rad, height %.2f, sharp %.2f, asym %+.2f,"
                                " terrace %.2f  (belt %.0f km wide)\n",
                                i, rs[i].ridges, rs[i].ridgeGap, rs[i].crestWidth,
                                rs[i].crestHeight, rs[i].sharpness,
                                rs[i].asymmetry, rs[i].terrace,
                                2.0 * ((double(rs[i].ridges) - 1.0) * rs[i].ridgeGap + 1.0)
                                    * rs[i].crestWidth * kPlanetRadius / 1000.0);
                check(worstMinDiff > 0.15,
                      "plates: no two ranges are near-copies of each other");
            }

            {
                const PlateSystem again = buildPlates(kNoiseSeed, [&](const DVec3& d){ return sampleLandE(world, d) > 0.0f; });
                bool same = true;
                for (int i = 0; i < kPlateCount; ++i)
                    same = same && space::length(ps.seed[i] - again.seed[i]) == 0.0;
                check(same, "plates: the same seed rebuilds the same partition");

                const PlateSystem other = buildPlates(kNoiseSeed + 1u, [&](const DVec3& d){ return sampleLandE(world, d) > 0.0f; });
                double moved = 0.0;
                for (int i = 0; i < kPlateCount; ++i)
                    moved = std::max(moved, space::length(ps.seed[i] - other.seed[i]));
                std::printf("         plates: a neighbouring seed moves a plate by"
                            " %.3f (unit sphere)\n", moved);
                check(moved > 0.02, "plates: a different seed is a different planet");
            }

            {
                double longest = 0.0, shortest = 1e9;
                for (int a = 0; a < kPlateCount; ++a)
                    for (int b = a + 1; b < kPlateCount; ++b) {
                        if (!ps.at(a, b).active) continue;

                        std::vector<DVec3> on;
                        const int N = 60000;
                        for (int i = 0; i < N; ++i) {
                            const double y  = 1.0 - 2.0 * (i + 0.5) / N;
                            const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
                            const double th = 2.399963229728653 * i;
                            const DVec3 d{ std::cos(th)*r, y, std::sin(th)*r };
                            const PlateHit h = plateAt(ps, d);
                            const bool pair = (h.a == a && h.b == b) || (h.a == b && h.b == a);
                            if (pair && h.toBoundary() < 0.01) on.push_back(d);
                        }
                        double span = 0.0;
                        for (size_t i = 0; i < on.size(); i += 3)
                            for (size_t j = i + 1; j < on.size(); j += 3)
                                span = std::max(span, std::acos(std::max(-1.0,
                                    std::min(1.0, space::dot(on[i], on[j])))));
                        const double km = span * kPlanetRadius / 1000.0;
                        longest = std::max(longest, km);
                        shortest = std::min(shortest, km);
                    }
                std::printf("         plates: range chains span %.0f to %.0f km\n",
                            shortest, longest);
                check(shortest > 800.0,
                      "plates: every range is at least 800 km long");
            }

            {

                std::vector<double> landKm;
                for (int a = 0; a < kPlateCount; ++a)
                    for (int b = a + 1; b < kPlateCount; ++b) {
                        int on = 0, land = 0;
                        const int N = 60000;
                        for (int i = 0; i < N; ++i) {
                            const double y  = 1.0 - 2.0 * (i + 0.5) / N;
                            const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
                            const double th = 2.399963229728653 * i;
                            const DVec3 d{ std::cos(th)*r, y, std::sin(th)*r };
                            const PlateHit h = plateAt(ps, d);
                            const bool pair = (h.a == a && h.b == b) || (h.a == b && h.b == a);
                            if (!pair || h.toBoundary() >= 0.01) continue;
                            ++on;
                            if (sampleLandE(world, d) > 0.0f) ++land;
                        }
                        if (on < 20) continue;

                        const double km = double(on) * 4.0 * kPi /
                                          (2.0 * 0.01 * 60000.0) * kPlanetRadius / 1000.0;
                        const double landOnly = km * double(land) / double(on);
                        landKm.push_back(landOnly);
                        std::printf("           boundary %d-%d:%s %5.0f km total,"
                                    " %5.0f km on land (%2.0f%%)\n", a, b,
                                    ps.at(a, b).active ? " *" : "  ", km, landOnly,
                                    100.0 * land / on);
                    }
                std::sort(landKm.begin(), landKm.end(), std::greater<double>());
                std::printf("           the four longest ON-LAND boundaries:"
                            " %.0f, %.0f, %.0f, %.0f km\n",
                            landKm.size() > 0 ? landKm[0] : 0.0,
                            landKm.size() > 1 ? landKm[1] : 0.0,
                            landKm.size() > 2 ? landKm[2] : 0.0,
                            landKm.size() > 3 ? landKm[3] : 0.0);
            }
        }

        {
            const double radius = 2000.0;
            std::vector<double> rel;
            const int SAMPLES = 1500, RING = 12;
            for (int i = 0; i < SAMPLES; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / SAMPLES;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;

                DVec3 t = space::cross(d, DVec3{ 0.0, 1.0, 0.0 });
                if (space::length(t) < 1e-6) t = space::cross(d, DVec3{ 1.0, 0.0, 0.0 });
                t = space::normalize(t);
                const DVec3 b = space::normalize(space::cross(d, t));

                double lo = 1e18, hi = -1e18;
                for (int k = 0; k < RING; ++k) {
                    const double a = 2.0 * kPi * double(k) / RING;
                    const DVec3 off = (t * std::cos(a) + b * std::sin(a)) *
                                      (radius / kPlanetRadius);
                    const DVec3 dir = space::normalize(d + off);
                    const double r = surfaceRadiusAt(world, dir, kPhysicsDetailOctaves)
                                     - kPlanetRadius;
                    lo = std::min(lo, r); hi = std::max(hi, r);
                }
                rel.push_back(hi - lo);
            }
            std::sort(rel.begin(), rel.end());
            auto q = [&](double f) { return rel[size_t(f * (rel.size() - 1))]; };
            std::printf("         local relief over a %.0f m radius (%zu land"
                        " samples):\n", radius, rel.size());
            std::printf("           p50 %6.0f m   p90 %6.0f m   p99 %6.0f m"
                        "   max %6.0f m\n", q(0.50), q(0.90), q(0.99), rel.back());
            std::printf("           for scale: rolling farmland <50 m, the Alps"
                        " ~800 m, the Himalaya ~1500 m\n");

            check(rel.size() > 300 && q(0.50) > 0.0,
                  "terrain: local relief is measurable over land");
        }

        {
            const double fov  = double(kFovYDegrees) * kPi / 180.0;
            const double split = double(kSplitFactor);

            std::printf("         screen-space triangle size (fov %.0f deg,"
                        " split factor %.2f):\n", double(kFovYDegrees), split);
            std::printf("           res  vertical px | just-split  about-to-merge\n");

            for (int res : { 8, 16, 24, 32, 48 }) {
                std::printf("           %3d", res);
                for (int px : { 720, 1440, 2160 }) {
                    const double focal = double(px) / (2.0 * std::tan(fov * 0.5));
                    const double near_ = (1.0 / double(res)) * split;
                    const double far_  = (1.0 / double(res)) * split * 0.5;
                    if (px == 720) std::printf("   %4d px |", px);
                    else           std::printf("\n              %4d px |", px);
                    std::printf(" %6.1f px    %6.1f px", near_ * focal, far_ * focal);
                }
                std::printf("\n");
            }

            const double focal1440 = 1440.0 / (2.0 * std::tan(fov * 0.5));
            const double worstPx = (1.0 / double(kPatchResolution)) * split * focal1440;
            std::printf("           shipped: up to %.1f px per triangle at 1440p\n",
                        worstPx);

            check(worstPx > 0.0 && worstPx < 1000.0,
                  "lod: the screen-space triangle size is a finite constant");
        }

        {
            const double faceArc = kPi * 0.5 * kPlanetRadius;
            const double spacing = faceArc / double(1 << kMaxDepth) /
                                   double(kPatchResolution);

            auto creases = [&](const PlanetParams& pp, const char* label) {
                World w(pp);
                std::vector<double> ang;
                const int N = 4000;
                for (int i = 0; i < N; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / N;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    if (sampleLandE(w, d) <= 0.0f) continue;

                    DVec3 t = space::cross(d, DVec3{ 0.0, 1.0, 0.0 });
                    if (space::length(t) < 1e-6) t = space::cross(d, DVec3{ 1.0, 0.0, 0.0 });
                    t = space::normalize(t);
                    const double du = spacing / kPlanetRadius;

                    DVec3 pts[3];
                    for (int k = -1; k <= 1; ++k) {
                        const DVec3 dir = space::normalize(d + t * (du * double(k)));
                        pts[k + 1] = dir * surfaceRadiusAt(w, dir, kPhysicsDetailOctaves);
                    }
                    const DVec3 a = space::normalize(pts[1] - pts[0]);
                    const DVec3 b = space::normalize(pts[2] - pts[1]);
                    const double c = std::max(-1.0, std::min(1.0, space::dot(a, b)));
                    ang.push_back(std::acos(c) * 180.0 / kPi);
                }
                std::sort(ang.begin(), ang.end());
                auto q = [&](double f) { return ang[size_t(f * (ang.size() - 1))]; };
                std::printf("           %-22s p50 %5.2f  p90 %5.2f  p99 %6.2f"
                            "  max %7.2f deg\n", label, q(0.50), q(0.90),
                            q(0.99), ang.back());
                return q(0.99);
            };

            std::printf("         crease angle between adjacent triangles at"
                        " %.2f m spacing:\n", spacing);

            const PlanetParams base{};
            const double asIs = creases(base, "as shipped");

            PlanetParams noTerrace = base;
            noTerrace.terraceMix = 0.0f;
            const double noTer = creases(noTerrace, "without terracing");

            PlanetParams noRidge = base;
            noRidge.detailRidgeMix = 0.0f;
            const double noRid = creases(noRidge, "without the ridge fold");

            PlanetParams neither = base;
            neither.terraceMix = 0.0f;
            neither.detailRidgeMix = 0.0f;
            creases(neither, "without either");

            std::printf("           attribution at p99: terracing costs %+.2f deg,"
                        " the ridge fold %+.2f deg\n",
                        asIs - noTer, asIs - noRid);

            check(asIs > 0.0 && asIs < 180.0,
                  "terrain: the crease measurement produces a usable angle");
        }

        {

            const double faceArc = kPi * 0.5 * kPlanetRadius;

            double contFinest = double(kPlanetRadius) / double(kNoiseFrequency);
            for (int i = 1; i < kNoiseOctaves; ++i) contFinest *= 0.5;

            double detailBase = double(kNoiseFrequency);
            for (int i = 0; i < kNoiseOctaves; ++i) detailBase *= double(kNoiseLacunarity);
            const double detailFinestAt0 = double(kPlanetRadius) / detailBase;

            std::printf("         Nyquist check (patch resolution %d, face arc"
                        " %.0f km)\n", kPatchResolution, faceArc / 1000.0);
            std::printf("           lvl  spacing     detail oct   finest feature"
                        "   samples/wave\n");

            double worstDetail = 1e9;
            int    worstLevel  = -1;
            for (int lvl = 0; lvl <= kMaxDepth; lvl += 2) {
                const double edge    = faceArc / double(1 << lvl);
                const double spacing = edge / double(kPatchResolution);
                const int    oct     = detailOctavesForLevel(lvl);

                double finest = detailFinestAt0;
                for (int i = 1; i < oct; ++i) finest *= 0.5;
                const double samples = oct > 0 ? finest / spacing : 0.0;
                if (oct > 0 && samples < worstDetail) {
                    worstDetail = samples; worstLevel = lvl;
                }
                if (oct > 0)
                    std::printf("           %2d  %8.2f m  %2d octaves  %9.2f m"
                                "     %6.2f\n", lvl, spacing, oct, finest, samples);
                else
                    std::printf("           %2d  %8.2f m   no detail band\n",
                                lvl, spacing);
            }
            std::printf("           worst detail margin: %.2f samples per"
                        " wavelength, at level %d\n", worstDetail, worstLevel);

            check(worstDetail >= 4.0,
                  "terrain: every evaluated detail octave gets 4+ samples per wavelength");

            {
                const double rootSpacing = faceArc / double(kPatchResolution);
                std::printf("           continental band: finest feature %.0f m,"
                            " level-0 spacing %.0f m -> %.2f samples\n",
                            contFinest, rootSpacing, contFinest / rootSpacing);
            }

            {
                const double spireFeature = double(kPlanetRadius) /
                                            double(kSpireFrequency);
                const int    lvl = 7 + kSpireOctaveFull;
                const double spacing = faceArc / double(1 << lvl) /
                                       double(kPatchResolution);
                double finest = spireFeature;
                for (int i = 1; i < kSpireOctaves; ++i) finest *= 0.5;
                std::printf("           spire band: finest feature %.0f m, full at"
                            " level %d (spacing %.1f m) -> %.1f samples\n",
                            finest, lvl, spacing, finest / spacing);
                check(finest / spacing >= 4.0,
                      "terrain: the spire band is resolved where it is full strength");
            }
        }

        {
            const float pole = space::kPi * 0.5f;

            auto swing = [](const space::Basis& a, const space::Basis& b) {
                const float d = space::dot(a.right, b.right);
                return std::acos(std::max(-1.0f, std::min(1.0f, d))) * 180.0f /
                       float(space::kPi);
            };
            const float eulerSwing = swing(space::makeBasis(0.7f, pole - 1e-4f),
                                           space::makeBasis(0.7f, pole));
            const float quatSwing  = swing(
                space::toBasis(space::fromYawPitch(0.7f, pole - 1e-4f)),
                space::toBasis(space::fromYawPitch(0.7f, pole)));
            std::printf("         camera: at the zenith, 1e-4 rad of pitch swings"
                        " `right` by %.1f deg (Euler) vs %.4f deg (quaternion)\n",
                        eulerSwing, quatSwing);
            check(eulerSwing > 30.0f,
                  "camera: makeBasis is unstable at the zenith, as documented");
            check(quatSwing < 0.05f,
                  "camera: ... and the quaternion basis is not");

            bool ok = true;
            float worstOrtho = 0.0f, worstUnit = 0.0f;
            const float pitches[] = { -pole, -1.2f, 0.0f, 1.2f, pole };
            for (float yaw : { -2.5f, 0.0f, 0.7f, 3.0f })
                for (float p : pitches) {
                    const space::Basis b = space::toBasis(space::fromYawPitch(yaw, p));
                    const float lens[3] = { space::length(b.forward),
                                            space::length(b.right),
                                            space::length(b.up) };
                    for (float l : lens) {
                        if (!std::isfinite(l)) ok = false;
                        worstUnit = std::max(worstUnit, std::fabs(l - 1.0f));
                    }
                    worstOrtho = std::max(worstOrtho,
                        std::fabs(space::dot(b.forward, b.right)));
                    worstOrtho = std::max(worstOrtho,
                        std::fabs(space::dot(b.forward, b.up)));
                    worstOrtho = std::max(worstOrtho,
                        std::fabs(space::dot(b.right,   b.up)));
                }
            std::printf("         camera: quaternion basis over 20 angles --"
                        " unit error %.2e, orthogonality error %.2e\n",
                        worstUnit, worstOrtho);
            check(ok && worstUnit < 1e-5f && worstOrtho < 1e-5f,
                  "camera: the quaternion basis stays orthonormal at the poles");

            const space::Basis up = space::toBasis(space::fromYawPitch(1.1f, pole));
            std::printf("         camera: at +90 deg the view points (%.3f, %.3f,"
                        " %.3f)\n", up.forward.x, up.forward.y, up.forward.z);
            check(up.forward.y > 0.9999f,
                  "camera: ... and straight up really is straight up");
            const space::Basis dn = space::toBasis(space::fromYawPitch(1.1f, -pole));
            check(dn.forward.y < -0.9999f, "camera: ... as is straight down");
        }

        {
            FontAtlas fa;
            fa.width = 128; fa.height = 128;
            fa.cellW = 8;   fa.cellH = 16;
            fa.cols  = 16;
            fa.pixels.assign(size_t(fa.width) * fa.height, 255);

            const float viewW = 1280.0f, viewH = 720.0f;
            const float rowH  = float(fa.cellH) + kUiRowPad;

            PlanetEditor ed;
            PlanetParams edited{}, live{};

            const float bodyTop  = 16.0f + rowH * 4.0f;
            const float headingH = rowH + kUiRowPad * 3.0f;
            const float seedY    = bodyTop + headingH + rowH * 0.5f;
            const float trackX   = Ui::trackStart(fa, 16.0f);
            const float trackW   = Ui::trackWidth(fa, 16.0f, 560.0f);

            UiState uiState;
            auto frame = [&](bool down, bool pressed, float mx, float my) {
                OverlayDraw d(fa);
                Ui ui(d, fa, uiState);
                UiInput in;
                in.mx = mx; in.my = my;
                in.down = down; in.pressed = pressed;
                ui.begin(in);
                return ed.draw(ui, d, in, edited, live, viewW, viewH);
            };

            const unsigned seedBefore = edited.seed;
            const double   ampBefore  = edited.terrainAmplitude;

            EditorActions a1 = frame(true, true, trackX + trackW * 0.75f, seedY);
            check(edited.seed != seedBefore,
                  "editor: a press on a row changes that row's value");
            check(edited.terrainAmplitude == ampBefore,
                  "editor: ... and leaves every other row alone");
            check(!a1.applyTerrain,
                  "editor: nothing is rebuilt while the button is still down");

            EditorActions a2 = frame(true, false, trackX + trackW * 0.9f, seedY);
            check(!a2.applyTerrain, "editor: ... nor part-way through a drag");

            EditorActions a3 = frame(false, false, trackX + trackW * 0.9f, seedY);
            std::printf("         editor: seed %u -> %u, rebuild asked for on "
                        "release: %s\n", seedBefore, edited.seed,
                        a3.applyTerrain ? "yes" : "NO");
            check(a3.applyTerrain, "editor: releasing the drag asks for the rebuild");

            EditorActions a4 = frame(false, false, 0.0f, 0.0f);
            check(a4.applyTerrain,
                  "editor: the request stands until the world actually changes");

            live = edited;
            EditorActions a5 = frame(false, false, 0.0f, 0.0f);
            check(!a5.applyTerrain, "editor: ... and stops once it has");

            {
                const int before = ed.firstRow();
                for (int i = 0; i < 5; ++i) {
                    OverlayDraw d(fa);
                    Ui ui(d, fa, uiState);
                    UiInput in;
                    in.mx = -1.0f; in.my = -1.0f;
                    in.wheel = -1.0f;
                    ui.begin(in);
                    ed.draw(ui, d, in, edited, live, viewW, viewH);
                }
                const int after = ed.firstRow();
                std::printf("         editor: five wheel notches moved the table"
                            " from row %d to row %d of %d\n",
                            before, after, ed.rowCount());
                check(after > before, "editor: the wheel scrolls the table down");

                for (int i = 0; i < 400; ++i) {
                    OverlayDraw d(fa);
                    Ui ui(d, fa, uiState);
                    UiInput in;
                    in.mx = -1.0f; in.my = -1.0f;
                    in.wheel = -1.0f;
                    ui.begin(in);
                    ed.draw(ui, d, in, edited, live, viewW, viewH);
                }
                check(ed.firstRow() < ed.rowCount(),
                      "editor: scrolling stops at the last row");

                for (int i = 0; i < 500; ++i) {
                    OverlayDraw d(fa);
                    Ui ui(d, fa, uiState);
                    UiInput in;
                    in.mx = -1.0f; in.my = -1.0f;
                    in.wheel = 1.0f;
                    ui.begin(in);
                    ed.draw(ui, d, in, edited, live, viewW, viewH);
                }
                check(ed.firstRow() == 0, "editor: ... and at the first one");
            }

            edited.bandHighland = 0.001f;
            EditorActions a6 = frame(false, false, 0.0f, 0.0f);
            check(!a6.applyTerrain,
                  "editor: an invalid parameter set is never built");
        }

        {
            PlanetParams a{};

            for (int i = 0; i < kParamFieldCount; ++i) {
                const ParamField& f = kParamFields[i];
                const double mid = f.lo + (f.hi - f.lo) * (0.31 + 0.02 * double(i % 7));
                fieldSet(a, f, mid);
            }
            for (int b = 0; b < kBiomeCount; ++b)
                for (int k = 0; k < 4; ++k)
                    biomeFactor(a, b, k) = 0.11f + 0.07f * float(b * 4 + k);

            const std::string text = writePlanet(a, "round trip");
            PlanetParams b2{};
            const ReadReport rep = readPlanet(text, b2);

            std::printf("         planet file: %d values, %zu bytes, %zu keys "
                        "not understood\n",
                        rep.applied, text.size(), rep.unknown.size());
            check(rep.applied == kParamFieldCount + kBiomeCount * 4,
                  "planet file: every field is written and read back");
            check(rep.unknown.empty(), "planet file: its own output parses cleanly");

            int drifted = 0;
            for (int i = 0; i < kParamFieldCount; ++i) {
                const ParamField& f = kParamFields[i];
                const double x = fieldGet(a, f), y = fieldGet(b2, f);

                if (std::fabs(x - y) > std::fabs(x) * 1e-9 + 1e-12) {
                    std::printf("         planet file: %s drifted %.17g -> %.17g\n",
                                f.key, x, y);
                    ++drifted;
                }
            }
            for (int b = 0; b < kBiomeCount; ++b)
                for (int k = 0; k < 4; ++k)
                    if (biomeFactor(a, b, k) != biomeFactor(b2, b, k)) ++drifted;
            check(drifted == 0, "planet file: no value changes across the round trip");

            {
                PlanetParams sparse{};
                const ReadReport r2 = readPlanet(
                    "# sparse\nseed 42\nterrainAmplitude 3000\n", sparse);
                check(r2.applied == 2, "planet file: a sparse file applies what it has");
                check(sparse.seed == 42u && sparse.terrainAmplitude == 3000.0,
                      "planet file: ... and applies it correctly");
                check(sparse.noiseOctaves == PlanetParams{}.noiseOctaves,
                      "planet file: ... and everything absent keeps its default");
            }

            {
                PlanetParams q{};
                const ReadReport r3 = readPlanet(
                    "seed 7\nkNoiseWobble 3\nterrainAmplitude 4000\n", q);
                check(r3.applied == 2, "planet file: an unknown key does not stop the load");
                check(r3.unknown.size() == 1 && r3.unknown[0] == "kNoiseWobble",
                      "planet file: ... and is named so it can be fixed");
            }
        }

        {
            const PlanetParams base{};

            PlanetParams same = base;
            check(classifyChange(base, same) == ParamChange::None,
                  "params: an unchanged set asks for no work");

            PlanetParams shade = base;
            shade.groundPaletteMix = 0.5f;
            check(classifyChange(base, shade) == ParamChange::ShadingOnly,
                  "params: the palette mix is shading only");
            PlanetParams shade2 = base;
            shade2.groundNormalStrength = 2.0f;
            check(classifyChange(base, shade2) == ParamChange::ShadingOnly,
                  "params: the normal strength is shading only");

            struct T { const char* what; void (*mutate)(PlanetParams&); };
            const T terrainEdits[] = {
                { "the amplitude",   [](PlanetParams& q){ q.terrainAmplitude *= 0.9; } },
                { "the seed",        [](PlanetParams& q){ q.seed += 1u; } },
                { "a colour band",   [](PlanetParams& q){ q.bandLowland = 0.06f; } },
                { "the slope ramp",  [](PlanetParams& q){ q.slopeRock = 0.2f; } },
                { "the moisture split", [](PlanetParams& q){ q.moistWet = 0.5f; } },
                { "a biome factor",  [](PlanetParams& q){ q.biomeScales[2].amp = 0.5f; } },
                { "the spire crest", [](PlanetParams& q){ q.spireCrest = 0.7f; } },
                { "the scatter density", [](PlanetParams& q){ q.scatterDensity = 0.5f; } },
            };
            bool allTerrain = true;
            for (const T& t : terrainEdits) {
                PlanetParams q = base;
                t.mutate(q);
                if (classifyChange(base, q) != ParamChange::Terrain) {
                    std::printf("         params: %s was NOT classified as "
                                "terrain\n", t.what);
                    allTerrain = false;
                }
            }
            check(allTerrain,
                  "params: everything baked into a vertex forces a rebuild");
        }

        {
            const std::vector<Violation> clean = validate(PlanetParams{});
            if (!clean.empty())
                for (const Violation& x : clean)
                    std::printf("         validator on defaults: %s: %s\n",
                                x.field, x.message.c_str());
            check(clean.empty(), "validate: the default planet raises nothing");

            auto raises = [&](const char* field, Severity sev,
                              void (*mutate)(PlanetParams&)) {
                PlanetParams q{};
                mutate(q);
                for (const Violation& x : validate(q))
                    if (x.sev == sev && std::string(x.field) == field) return true;
                return false;
            };

            struct Case {
                const char* field;
                Severity    sev;
                void      (*mutate)(PlanetParams&);
                const char* what;
            };
            const Case cases[] = {
                { "groundMacroMetres", Severity::Error,
                  [](PlanetParams& q){ q.groundMacroMetres = 23.0f; },
                  "a macro tile that is not a whole number of fine tiles" },
                { "biomeScales", Severity::Error,
                  [](PlanetParams& q){ q.biomeScales[1].amp = 1.4f; },
                  "a biome factor above 1" },
                { "biomeScales", Severity::Error,
                  [](PlanetParams& q){ q.biomeScales[2].detail = 0.0f; },
                  "a biome factor at zero" },
                { "noiseLacunarity", Severity::Error,
                  [](PlanetParams& q){ q.noiseLacunarity = 1.0f; },
                  "octaves that never change frequency" },
                { "noiseGain", Severity::Error,
                  [](PlanetParams& q){ q.noiseGain = 1.0f; },
                  "octaves that never lose amplitude" },
                { "seaLevelNoise", Severity::Error,
                  [](PlanetParams& q){ q.seaLevelNoise = 1.0f; },
                  "a sea level that divides by zero" },
                { "detailAmplitude", Severity::Error,
                  [](PlanetParams& q){

                      const double norm = double(Noise::fixedNorm(q.detailGain,
                                                                  kMaxDetailOctaves));
                      const double g7   = std::pow(double(q.detailGain), kGearOctave);
                      q.detailAmplitude = 2.0 * kHullClearance * norm / g7;
                  },
                  "roughness taller than the landing gear" },
                { "bandHighland", Severity::Error,
                  [](PlanetParams& q){ q.bandHighland = 0.01f; },
                  "an inverted elevation band" },
                { "slopeRock", Severity::Error,
                  [](PlanetParams& q){ q.slopeRock = 0.0f; },
                  "an inverted slope ramp" },
                { "spireOctaveFull", Severity::Error,
                  [](PlanetParams& q){ q.spireOctaveFull = q.spireOctaveIn; },
                  "spires that appear in one LOD step" },
                { "scatterMinScale", Severity::Error,
                  [](PlanetParams& q){ q.scatterMinScale = 99.0f; },
                  "a scatter scale range the wrong way round" },

                { "bandPeak", Severity::Warning,
                  [](PlanetParams& q){ q.bandPeak = 1.5f; },
                  "a peak band above the maximum elevation" },
                { "slopeSoft", Severity::Warning,
                  [](PlanetParams& q){ q.slopeSoft = 1.2f; q.slopeRock = 1.4f; },
                  "a rock threshold no slope reaches" },
                { "moistDry", Severity::Warning,
                  [](PlanetParams& q){ q.moistDry = 0.9f; q.moistWet = 0.95f; },
                  "a moisture split above the field" },
                { "spireCrest", Severity::Warning,
                  [](PlanetParams& q){ q.spireCrest = 1.1f;  },
                  "a crest threshold no ridge reaches" },
                { "noiseOctaves", Severity::Warning,
                  [](PlanetParams& q){ q.noiseOctaves = 20; },
                  "more octaves than the mesh can carry" },
            };

            bool allFire = true;
            for (const Case& c : cases) {
                const bool fired = raises(c.field, c.sev, c.mutate);
                if (!fired) {
                    std::printf("         validator MISSED: %s (%s)\n",
                                c.what, c.field);
                    allFire = false;
                }
            }
            std::printf("         validator: %zu checks driven into their "
                        "violation, all blamed the right field\n",
                        sizeof(cases) / sizeof(cases[0]));
            check(allFire, "validate: every check can actually fire");

            {
                PlanetParams q{};
                q.bandPeak = 1.5f;
                const std::vector<Violation> w = validate(q);
                check(!w.empty(), "validate: an unreachable band is reported");
                check(!hasErrors(w),
                      "validate: ... as a warning, which does not block the build");
            }
        }

        {
            FontAtlas fa;
            fa.width = 128; fa.height = 128;
            fa.cellW = 8;   fa.cellH = 16;
            fa.cols  = 16;
            fa.pixels.assign(size_t(fa.width) * fa.height, 255);

            const float rowH   = float(fa.cellH) + kUiRowPad;
            const float panelX = 16.0f, panelY = 16.0f;
            const float panelW = 400.0f;

            const float trackX = Ui::trackStart(fa, panelX);
            const float trackW = Ui::trackWidth(fa, panelX, panelW);

            const float row1Y  = panelY + kUiPanelPad + rowH;

            auto runPanel = [&](const UiInput& in, float* v) {
                OverlayDraw d(fa);
                UiState st;
                Ui ui(d, fa, st);
                ui.begin(in);
                ui.beginPanel(panelX, panelY, panelW, "title");
                ui.slider("value", v, 0.0f, 100.0f);
                ui.endPanel();
                return d.vertices().size();
            };

            {
                float v = 0.0f;
                UiInput in;
                in.mx = trackX + trackW * 0.5f;
                in.my = row1Y + rowH * 0.5f;
                in.down = in.pressed = true;
                runPanel(in, &v);
                std::printf("         ui: press at track midpoint -> %.2f of 0..100\n", v);
                check(std::fabs(v - 50.0f) < 0.75f,
                      "ui: a press maps the pointer to the value under it");
            }

            {
                float v = 50.0f;
                UiInput in; in.my = row1Y + rowH * 0.5f; in.down = in.pressed = true;
                in.mx = trackX;                 runPanel(in, &v);
                check(v == 0.0f,  "ui: the left end of a track is the minimum");
                float v2 = 50.0f;
                in.mx = trackX + trackW;        runPanel(in, &v2);
                check(v2 == 100.0f, "ui: the right end of a track is the maximum");
            }

            {
                float v = 42.0f;
                UiInput in;
                in.mx = trackX + trackW * 0.5f;
                in.my = panelY + kUiPanelPad + rowH * 0.5f;
                in.down = in.pressed = true;
                runPanel(in, &v);
                check(v == 42.0f, "ui: a press on the title row does not move the slider");
            }

            {
                float v = 42.0f;
                UiInput in;
                in.mx = -1.0f; in.my = -1.0f;
                in.down = in.pressed = true;
                runPanel(in, &v);
                check(v == 42.0f, "ui: a pointer off the window hits nothing");
            }

            {
                const int N = 6;
                float store[N];
                for (int i = 0; i < N; ++i) store[i] = 50.0f;

                UiInput in;
                in.my = row1Y + rowH * 2.5f;
                in.mx = trackX + trackW * 0.25f;
                in.down = in.pressed = true;

                OverlayDraw d(fa);
                UiState st;
                Ui ui(d, fa, st);
                ui.begin(in);
                ui.beginPanel(panelX, panelY, panelW, "title");
                for (int i = 0; i < N; ++i) {

                    float local = store[i];
                    if (ui.slider("row", &local, 0.0f, 100.0f, "%.2f", &store[i]))
                        store[i] = local;
                }
                ui.endPanel();

                int moved = 0, movedIndex = -1;
                for (int i = 0; i < N; ++i)
                    if (store[i] != 50.0f) { ++moved; movedIndex = i; }
                std::printf("         ui: pressed row 3 of %d, %d slider(s) moved"
                            " (index %d)\n", N, moved, movedIndex);
                check(moved == 1, "ui: a press moves exactly one slider");
                check(movedIndex == 2, "ui: ... and it is the one under the pointer");
            }

            {
                float v = 50.0f;
                UiInput in; in.my = row1Y + rowH * 0.5f;
                in.mx = trackX + trackW * 0.5f;
                in.down = in.pressed = true;

                OverlayDraw d(fa);
                UiState st;
                Ui ui(d, fa, st);
                ui.begin(in);
                ui.beginPanel(panelX, panelY, panelW, "title");
                ui.slider("value", &v, 0.0f, 100.0f);
                ui.endPanel();
                check(ui.capturing(), "ui: a press on a track starts a drag");

                in.pressed = false;
                in.mx = trackX + trackW * 4.0f;
                ui.begin(in);
                ui.beginPanel(panelX, panelY, panelW, "title");
                ui.slider("value", &v, 0.0f, 100.0f);
                ui.endPanel();
                check(v == 100.0f, "ui: a drag past the end clamps rather than overshoots");

                in.down = false;
                ui.begin(in);
                check(!ui.capturing(), "ui: releasing anywhere ends the drag");
            }

            {
                float v = 0.0f;
                OverlayDraw d(fa);
                UiState st;
                Ui ui(d, fa, st);
                UiInput in;
                ui.begin(in);
                ui.beginPanel(panelX, panelY, panelW, "title");
                ui.slider("a", &v, 0.0f, 1.0f);
                ui.slider("b", &v, 0.0f, 1.0f);
                ui.endPanel();
                float lo = 1e9f, hi = -1e9f;
                for (size_t i = 0; i < 6; ++i) {
                    lo = std::min(lo, d.vertices()[i].y);
                    hi = std::max(hi, d.vertices()[i].y);
                }
                const float need = panelY + kUiPanelPad + rowH * 3.0f;
                std::printf("         ui: panel background spans %.0f..%.0f,"
                            " rows end at %.0f\n", lo, hi, need);
                check(hi >= need,
                      "ui: the panel background is resized to cover its rows");
            }
        }

        {
            std::vector<float> cv, pv;
            std::vector<float> hv;
            int    land = 0, standing = 0;
            double tallest = 0.0;
            const int SP = 30000;
            for (int i = 0; i < SP; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / SP;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };

                const DVec3 a{ d.x + 5.113, d.y + 2.771, d.z - 9.317 };
                cv.push_back(spireCrest(world, a));
                const DVec3 pm{ d.x - 12.907, d.y + 71.331, d.z - 33.117 };
                pv.push_back(0.5f * (world.noise.fbm(pm, kSpireProvinceOctaves,
                                               kSpireProvinceFrequency,
                                               kNoiseLacunarity, kNoiseGain) + 1.0f));

                const TerrainShape sh = sampleShape(world, d);
                if (sh.landE <= 0.0f) continue;
                ++land;
                const double h = spireHeight(world, d, sh, kPhysicsDetailOctaves);
                hv.push_back(float(h));
                tallest = std::max(tallest, h);
                if (h > 0.5 * kSpireAmplitude) ++standing;
            }
            std::sort(cv.begin(), cv.end());
            std::sort(pv.begin(), pv.end());
            auto qc = [&](double f) { return cv[size_t(f * (cv.size() - 1))]; };
            auto qp = [&](double f) { return pv[size_t(f * (pv.size() - 1))]; };
            std::printf("         spire crest field: p50 %.3f p75 %.3f p90 %.3f"
                        " p99 %.3f max %.3f   (kSpireCrest %.3f)\n",
                        qc(0.50), qc(0.75), qc(0.90), qc(0.99), cv.back(), kSpireCrest);
            std::printf("         spire province:    p50 %.3f p75 %.3f p90 %.3f"
                        " max %.3f   (band %.3f..%.3f)\n",
                        qp(0.50), qp(0.75), qp(0.90), pv.back(),
                        kSpireProvinceLo, kSpireProvinceHi);
            std::printf("         spires: %.2f%% of land stands above half height,"
                        " tallest %.0f m of %.0f m allowed\n",
                        100.0 * standing / std::max(1, land), tallest, kSpireAmplitude);

            check(tallest <= kSpireAmplitude + 1e-6,
                  "spire: no tower exceeds the amplitude the culling box allows");
            check(standing > 0, "spire: towers actually stand somewhere on land");
        }

        {
            std::vector<float> mv;
            const int MQ = 20000;
            for (int i = 0; i < MQ; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / MQ;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;
                mv.push_back(moistureAt(world, d));
            }
            std::sort(mv.begin(), mv.end());
            auto q = [&](double f) { return mv[size_t(f * (mv.size() - 1))]; };
            std::printf("         moisture over land: min %.3f p10 %.3f p25 %.3f"
                        " p50 %.3f p75 %.3f p90 %.3f max %.3f\n",
                        mv.front(), q(0.10), q(0.25), q(0.50), q(0.75), q(0.90),
                        mv.back());
            check(mv.size() > 2000, "biome: the moisture sample is big enough");
        }

        {
            const int BS = 300000;
            int land = 0, onBelt = 0;
            for (int i = 0; i < BS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / BS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const TerrainShape sh = sampleShape(world, d);
                if (sh.landE <= 0.0f) continue;
                ++land;

                if (std::fabs(sh.orogeny) > 0.10f * kRangeCrestHeight) ++onBelt;
            }
            const double share = 100.0 * double(onBelt) / double(std::max(1, land));
            std::printf("         belts cover %.1f%% of the land (%d of %d samples)\n",
                        share, onBelt, land);
            check(share > 3.0 && share < 18.0,
                  "orogeny: the belts are rare but findable (3-18% of land)");
        }

        {

            DVec3 start{ 0, 1, 0 };
            double bestStart = 0.0;
            for (int i = 0; i < 400000; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / 400000;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const TerrainShape sh = sampleShape(world, d);
                if (sh.landE <= 0.0f) continue;
                const double o = std::fabs(sh.orogeny);
                if (o > bestStart) { bestStart = o; start = d; }
            }
            const bool found = bestStart > 0.2 * double(kRangeCrestHeight);
            check(found, "orogeny: a belt crest can be found at all");
            if (found) {

                const double step = 8000.0 / kPlanetRadius;
                auto march = [&](const DVec3& seed, const DVec3& away) {
                    std::vector<DVec3> out;
                    DVec3 cur = seed, prev = away;
                    for (int n = 0; n < 300; ++n) {
                        out.push_back(cur);
                        const DVec3 ref = (std::fabs(cur.y) < 0.9) ? DVec3{ 0, 1, 0 }
                                                                   : DVec3{ 1, 0, 0 };
                        const DVec3 t1 = normalize(cross(ref, cur));
                        const DVec3 t2 = cross(cur, t1);
                        double bestO = -1e9;
                        DVec3  bestD = cur;
                        for (int k = 0; k < 48; ++k) {
                            const double a = 2.0 * 3.14159265358979323846 * k / 48.0;
                            const DVec3 dir = normalize(cur + (t1 * std::cos(a) +
                                                               t2 * std::sin(a)) * step);

                            if (length(prev) > 0.0) {
                                const DVec3 came = normalize(cur - prev);
                                const DVec3 goes = normalize(dir - cur);
                                if (dot(came, goes) < -0.5) continue;
                            }
                            const TerrainShape sh = sampleShape(world, dir);
                            if (sh.landE <= 0.0f) continue;
                            const double o = std::fabs(sh.orogeny);
                            if (o > bestO) { bestO = o; bestD = dir; }
                        }

                        if (bestO < 0.35 * bestStart) break;
                        prev = cur;
                        cur  = bestD;
                    }
                    return out;
                };

                const DVec3 refS = (std::fabs(start.y) < 0.9) ? DVec3{ 0, 1, 0 }
                                                              : DVec3{ 1, 0, 0 };
                const DVec3 sT = normalize(cross(refS, start));
                const DVec3 fwd0 = normalize(start + sT * step);
                std::vector<DVec3> halfA = march(start, fwd0);
                std::vector<DVec3> halfB = march(start, normalize(start - sT * step));
                std::vector<DVec3> crest;
                for (size_t i = halfA.size(); i-- > 0; ) crest.push_back(halfA[i]);
                for (size_t i = 1; i < halfB.size(); ++i) crest.push_back(halfB[i]);

                DVec3 mean{ 0, 0, 0 };
                for (const DVec3& c : crest) mean = mean + c;
                mean = mean * (1.0 / double(crest.size()));

                double best = 1e30;
                DVec3  bn{ 0, 1, 0 };
                for (int a = 0; a < 180; ++a)
                    for (int b = 0; b < 360; ++b) {
                        const double th2 = 3.14159265358979323846 * a / 180.0;
                        const double ph  = 2.0 * 3.14159265358979323846 * b / 360.0;
                        const DVec3 n{ std::sin(th2) * std::cos(ph),
                                       std::cos(th2),
                                       std::sin(th2) * std::sin(ph) };
                        double acc = 0.0;
                        for (const DVec3& c : crest) {
                            const double e = dot(n, c);
                            acc += e * e;
                        }
                        if (acc < best) { best = acc; bn = n; }
                    }
                double worst = 0.0;
                for (const DVec3& c : crest)
                    worst = std::max(worst, std::fabs(dot(bn, c)));
                const double km = worst * kPlanetRadius / 1000.0;

                const double walked = 8.0 * double(crest.size());
                const double ends   = std::acos(std::max(-1.0, std::min(1.0,
                                          dot(crest.front(), crest.back())))) *
                                      kPlanetRadius / 1000.0;
                const double tort = ends > 1.0 ? walked / ends : 1.0;
                std::printf("         crest walk: %zu points over %.0f km, "
                            "great-circle departure %.0f km, tortuosity %.3f\n",
                            crest.size(), walked, km, tort);

                check(walked > 400.0 && tort > 1.15 && tort < 3.0,
                      "orogeny: the chain meanders over its length, without folding");
            }
        }

        {
            const int PS = 20000;
            double worst = 0.0, sum = 0.0;
            int smeared = 0;
            for (int i = 0; i < PS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / PS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;

                const DVec3 n{ std::cos(th) * rr, y, std::sin(th) * rr };
                double w[3] = { std::pow(std::fabs(n.x), 4.0),
                                std::pow(std::fabs(n.y), 4.0),
                                std::pow(std::fabs(n.z), 4.0) };
                const double t = w[0] + w[1] + w[2];
                w[0] /= t; w[1] /= t; w[2] /= t;
                std::sort(w, w + 3);
                const double second = w[1];
                worst = std::max(worst, second);
                sum  += second;
                if (second > 0.15) ++smeared;
            }
            std::printf("         triplanar on flat ground: second projection "
                        "mean %.3f, worst %.3f, over 0.15 on %.0f%% of the sphere\n",
                        sum / PS, worst, 100.0 * double(smeared) / PS);
        }

        {
            std::vector<float> cv;
            cv.reserve(8000);
            const int CS = 12000;
            for (int i = 0; i < CS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / CS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;
                cv.push_back(canopyAt(world, d));
            }
            std::sort(cv.begin(), cv.end());
            auto q = [&](double f) { return cv[size_t(f * (cv.size() - 1))]; };
            std::printf("         canopy over land (%zu samples): p05 %.3f  p25 %.3f"
                        "  p50 %.3f  p75 %.3f  p95 %.3f\n",
                        cv.size(), q(0.05), q(0.25), q(0.50), q(0.75), q(0.95));
            std::printf("           band [%.3f, %.3f] -> open %.1f%%, closed %.1f%%,"
                        " mixed %.1f%%\n", kCanopyOpen, kCanopyClosed,
                        100.0 * double(std::lower_bound(cv.begin(), cv.end(),
                                                        kCanopyOpen) - cv.begin())
                            / double(cv.size()),
                        100.0 * double(cv.end() - std::upper_bound(cv.begin(), cv.end(),
                                                                   kCanopyClosed))
                            / double(cv.size()),
                        100.0 * double(std::upper_bound(cv.begin(), cv.end(), kCanopyClosed)
                                       - std::lower_bound(cv.begin(), cv.end(), kCanopyOpen))
                            / double(cv.size()));

            const double openShare = double(std::lower_bound(cv.begin(), cv.end(),
                                                             kCanopyOpen) - cv.begin())
                                     / double(cv.size());
            const double closedShare = double(cv.end() - std::upper_bound(cv.begin(),
                                              cv.end(), kCanopyClosed))
                                       / double(cv.size());

            check(openShare > 0.10 && closedShare > 0.10,
                  "canopy: both ends of the band carry real ground");

            double worstSum = 0.0;
            bool   neg = false;
            for (int n = 1; n <= kMaxVariants; ++n) {
                for (int i = 0; i <= 200; ++i) {
                    float v[kMaxVariants] = {};
                    canopySplit(n, float(i) / 200.0f, v);
                    double t = 0.0;
                    for (int k = 0; k < n; ++k) { t += v[k]; if (v[k] < 0.0f) neg = true; }
                    worstSum = std::max(worstSum, std::fabs(t - 1.0));
                }
            }
            check(!neg && worstSum < 1e-6,
                  "canopySplit: the variants partition one unit, never negative");

            double sc = 0, sm = 0, se = 0, scc = 0, smm = 0, see = 0, scm = 0, sce = 0;
            int    nn = 0;
            const int RS = 6000;
            for (int i = 0; i < RS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / RS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const float le = sampleLandE(world, d);
                if (le <= 0.0f) continue;
                const double c = canopyAt(world, d), m = moistureAt(world, d), e = le;
                sc += c; sm += m; se += e;
                scc += c * c; smm += m * m; see += e * e;
                scm += c * m; sce += c * e;
                ++nn;
            }
            auto corr = [&](double sxy, double sx, double sy, double sxx, double syy) {
                const double cov = sxy / nn - (sx / nn) * (sy / nn);
                const double vx  = sxx / nn - (sx / nn) * (sx / nn);
                const double vy  = syy / nn - (sy / nn) * (sy / nn);
                return cov / std::sqrt(std::max(1e-12, vx * vy));
            };
            const double rcm = corr(scm, sc, sm, scc, smm);
            const double rce = corr(sce, sc, se, scc, see);
            std::printf("         canopy correlations: vs moisture %+.3f, vs elevation %+.3f\n",
                        rcm, rce);
            check(std::fabs(rcm) < 0.15 && std::fabs(rce) < 0.15,
                  "canopy: independent of moisture and of height");
        }

        {
            double sum[kGroundLayerCount] = {};
            int    dom[kGroundLayerCount] = {};
            double worst = 0.0;
            int    n = 0;
            const int MS = 6000;
            for (int i = 0; i < MS; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / MS;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) <= 0.0f) continue;
                const double delta = 2.0 / double(1 << kMaxDepth) / kPatchResolution;
                double su = 0.0, sv2 = 0.0;
                const int face = scatterFaceOf(d, su, sv2);
                const SurfaceSample ss = sampleSurface(world, face, su, sv2,
                                                       delta * 0.5,
                                                       kPhysicsDetailOctaves);
                double tot = 0.0;
                int    best = 0;
                for (int m = 0; m < kGroundLayerCount; ++m) {
                    sum[m] += ss.mat[m];
                    tot    += ss.mat[m];
                    if (ss.mat[m] > ss.mat[best]) best = m;
                }

                worst = std::max(worst, std::fabs(tot - 1.0));
                ++dom[best];
                ++n;
            }
            std::printf("         ground layer cover over land (%d samples):\n", n);
            bool allSeen = true;
            for (int m = 0; m < kGroundLayerCount; ++m) {
                const double share = 100.0 * sum[m] / n;
                std::printf("           %-18s %5.1f%%  (dominant on %4.1f%%)\n",
                            kGroundLayers[m].label, share, 100.0 * dom[m] / n);
                if (share < 1.0) allSeen = false;
            }
            std::printf("           worst deviation from a unit sum: %.2e\n", worst);

        {
            double worstFine = 0.0, worstMacro = 0.0;
            bool inRange = true;
            int checked = 0;
            for (int level = 0; level <= 18; level += 2) {
                const int n = 1 << level;
                for (int k = 0; k < 40; ++k) {
                    const int face = k % 6;
                    const int ix = (k * 7919) % n;
                    const int iy = (k * 6151) % n;
                    const DVec3 po = patchOrigin(face, level, ix, iy);
                    const DVec3 tw = groundTexOrigin(kTestParams, po);
                    for (int c = 0; c < 3; ++c) {
                        const double p0 = c == 0 ? po.x : (c == 1 ? po.y : po.z);
                        const double w0 = c == 0 ? tw.x : (c == 1 ? tw.y : tw.z);

                        auto residual = [&](double tile) {
                            const double q = (p0 - w0) / tile;
                            return std::fabs(q - std::floor(q + 0.5));
                        };
                        worstFine  = std::max(worstFine,  residual(kGroundTileMetres));
                        worstMacro = std::max(worstMacro, residual(kGroundMacroMetres));
                    }

                    inRange = inRange &&
                        tw.x >= 0.0 && tw.x < kGroundMacroMetres &&
                        tw.y >= 0.0 && tw.y < kGroundMacroMetres &&
                        tw.z >= 0.0 && tw.z < kGroundMacroMetres;
                    ++checked;
                }
            }
            std::printf("         texture origin residual over %d patches:"
                        " fine %.2e  macro %.2e tiles\n",
                        checked, worstFine, worstMacro);
            check(worstFine  < 1e-6,
                  "ground: the wrapped origin is a whole number of FINE tiles");
            check(worstMacro < 1e-6,
                  "ground: the wrapped origin is a whole number of MACRO tiles");
            check(inRange, "ground: the wrapped origin stays inside one macro tile");
        }

            check(n > 1000, "ground: the material sample is big enough to mean something");
            check(worst < 1e-4, "ground: material weights sum to 1 on land");
            check(allSeen, "ground: every loaded material covers at least 1% of the land");
        }

        {
            World bn;

            {
                int sunk = 0, coastal = 0;
                const int S3 = 200000;
                for (int i = 0; i < S3; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / S3;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    const TerrainShape sh = sampleShape(bn, d);
                    if (sh.landE <= 0.0f || sh.landE > 0.2f) continue;
                    ++coastal;
                    if (surfaceRadius(bn, d, sh, kPhysicsDetailOctaves) < kPlanetRadius)
                        ++sunk;
                }
                std::printf("         coastal land below sea level: %d of %d\n",
                            sunk, coastal);
                check(coastal > 5000,
                      "biome: the coastal sample is big enough to mean something");
                check(sunk == 0, "biome: no coastal land is pushed below sea level");
            }

            {

                double worstSlack = 0.0; bool encloses = true;
                for (int t = 0; t < 400; ++t) {
                    const int level = 6 + (t % 9);
                    const double size = 2.0 / double(1 << level);
                    const int last = (1 << level) - 1;
                    const int ix = (t * 7919) % (last + 1);
                    const int iy = (t * 104729) % (last + 1);
                    const PatchBounds pb = computePatchBounds(t % 6, level, ix, iy, bn);
                    const double u0 = -1.0 + size * ix, v0 = -1.0 + size * iy;

                    double maxDist = 0.0;

                    const int oct = detailOctavesForLevel(level);
                    for (int j = 0; j <= 12; ++j)
                        for (int i = 0; i <= 12; ++i) {
                            const DVec3 d = faceToSphereD(t % 6, u0 + size * i / 12.0,
                                                                 v0 + size * j / 12.0);
                            const DVec3 p = d * surfaceRadius(bn, d, sampleShape(bn, d), oct);
                            const double dist = space::length(p - pb.surfaceCenter);
                            if (dist > pb.surfaceRadius + 1.0) encloses = false;
                            maxDist = std::max(maxDist, dist);
                        }
                    worstSlack = std::max(worstSlack, pb.surfaceRadius - maxDist);
                }
                std::printf("         LOD sphere: worst slack over the surface %.1f m\n",
                            worstSlack);
                check(encloses,
                      "biome: the LOD sphere still contains its patch's surface");
                check(worstSlack < 400.0,
                      "biome: ... and is not inflated by an unscaled terrainRadius");
            }

            {
                const double eps = 30.0;
                double sum[kBiomeCount] = {}; int cnt[kBiomeCount] = {};
                const int S4 = 4000;
                for (int i = 0; i < S4; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / S4;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    const TerrainShape sh = sampleShape(bn, d);
                    if (sh.landE <= 0.0f) continue;
                    const BiomeWeights bw = biomeWeightsAt(kTestParams, 1.0f, reliefAt(bn, d),
                                                           float(d.y));
                    int dom = kBiomePlains;
                    for (int k = 1; k < kBiomeCount; ++k)
                        if (bw.w[k] > bw.w[dom]) dom = k;
                    const double r0 = surfaceRadiusAt(bn, d, kPhysicsDetailOctaves);
                    const DVec3 ref = (std::fabs(d.y) < 0.9) ? DVec3{0,1,0} : DVec3{1,0,0};
                    const DVec3 t1 = normalize(cross(ref, d)), t2 = cross(d, t1);
                    auto radAt = [&](const DVec3& q) {
                        return surfaceRadiusAt(bn, normalize(q), kPhysicsDetailOctaves);
                    };
                    const double g1 = (radAt(d * r0 + t1 * eps) - r0) / eps;
                    const double g2 = (radAt(d * r0 + t2 * eps) - r0) / eps;
                    sum[dom] += std::atan(std::sqrt(g1 * g1 + g2 * g2)) * 180.0 / kPi;
                    ++cnt[dom];
                }
                const double plains = sum[kBiomePlains]    / std::max(1, cnt[kBiomePlains]);
                const double mesa   = sum[kBiomeMesa]      / std::max(1, cnt[kBiomeMesa]);
                const double mount  = sum[kBiomeMountains] / std::max(1, cnt[kBiomeMountains]);
                std::printf("         mean slope by biome: plains %.1f deg,"
                            " mesa %.1f deg, mountains %.1f deg\n",
                            plains, mesa, mount);
                check(plains < 12.0, "biome: plains stay easy to land on");
                check(mount > plains * 1.5,
                      "biome: mountains are genuinely steeper, not just repainted");
            }
        }

        {
            const int level = 14, N = kPatchResolution;
            const double size = 2.0 / double(1 << level);
            DVec3 ld = normalize(DVec3{ 0.3, 0.5, 0.8 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(world, d) > 0.30f) { ld = d; break; }
            }
            double su = 0.0, sv = 0.0;
            const int face = scatterFaceOf(ld, su, sv);
            const int last = (1 << level) - 1;
            const int ix = std::min(last, std::max(0, int((su + 1.0) / size)));
            const int iy = std::min(last, std::max(0, int((sv + 1.0) / size)));

            const PatchMeshData fine = generatePatch(world, face, level, ix, iy, N);
            const PatchMeshData par  = generatePatch(world, face, level - 1,
                                                     ix >> 1, iy >> 1, N);
            const int offI = (ix & 1) * (N / 2), offJ = (iy & 1) * (N / 2);
            const Vec3 sun = normalize(kSunDirection);

            double worstDeg = 0.0, sumDeg = 0.0;
            double worstLit = 0.0, worstCol = 0.0;
            int n = 0;
            for (int j = 0; j <= N; j += 2)
                for (int i = 0; i <= N; i += 2) {
                    const Vertex& a = fine.vertices[j * (N + 1) + i];
                    const Vertex& b = par.vertices[(offJ + j / 2) * (N + 1) +
                                                   (offI + i / 2)];
                    const UnpackedNormal ua = vertexNormal(a), ub = vertexNormal(b);
                    const Vec3 na{ ua.x, ua.y, ua.z }, nb{ ub.x, ub.y, ub.z };
                    double c = dot(na, nb);
                    c = c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c);
                    const double deg = std::acos(c) * 180.0 / kPi;
                    worstDeg = std::max(worstDeg, deg); sumDeg += deg; ++n;

                    const double la = std::max(0.0, -double(dot(na, sun)));
                    const double lb = std::max(0.0, -double(dot(nb, sun)));
                    worstLit = std::max(worstLit, std::fabs(la - lb));
                    worstCol = std::max(worstCol, double(std::max(std::fabs(a.r - b.r),
                                        std::max(std::fabs(a.g - b.g),
                                                 std::fabs(a.b - b.b)))));
                }
            std::printf("         SHADING pop if the normal did NOT morph (level %d):\n"
                        "           normal turns %.1f deg worst / %.1f deg mean,"
                        " brightness %.1f%%, albedo %.3f\n",
                        level, worstDeg, sumDeg / n, worstLit * 100.0, worstCol);
            check(worstDeg > 1.0,
                  "shading: an unmorphed normal really would step at the swap");

            {
                const ParentSurface ps = sampleParentSurface(
                    world, face, level, -1.0 + size * ix, -1.0 + size * iy,
                    size / N, N);

                double worstResidualDot = 0.0, worstResidualLit = 0.0;
                for (int j = 0; j <= N; ++j)
                    for (int i = 0; i <= N; ++i) {
                        const Vertex& v = fine.vertices[j * (N + 1) + i];

                        const UnpackedNormal cn = vertexCoarseNormal(v);
                        const Vec3 morphed = normalize(Vec3{ cn.x, cn.y, cn.z });
                        const Vec3 want    = ps.normalAt(i, j);
                        double c = dot(morphed, want);
                        c = c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c);
                        worstResidualDot = std::max(worstResidualDot, 1.0 - c);
                        worstResidualLit = std::max(worstResidualLit,
                            std::fabs(std::max(0.0, -double(dot(morphed, sun))) -
                                      std::max(0.0, -double(dot(want,    sun)))));
                    }
                std::printf("           with the morph: %.4f deg (1-dot %.2e),"
                            " %.4f%% brightness\n",
                            std::acos(std::max(-1.0, 1.0 - worstResidualDot)) * 180.0 / kPi,
                            worstResidualDot, worstResidualLit * 100.0);
                check(worstResidualDot < 1e-6,
                      "shading: a finished morph shades with the PARENT's normal");
                check(worstResidualLit < 1e-4,
                      "shading: ... so the brightness is continuous through the swap");
            }
        }

        {
            World ln;
            PatchBoundsCache bounds(ln);

            const int kLatency = 12;

            const int  kFlightFrames  = 420;
            const int  kSqueezeFrames = 150;
            const int  kCacheRoomy    = 100000;
            const int  kCacheTight    = 2000;
            const bool kTouchAncestors = true;
            std::unordered_map<PatchKey, int, PatchKeyHash> requestedAt;
            std::unordered_set<PatchKey, PatchKeyHash> live;
            std::unordered_set<PatchKey, PatchKeyHash> drawnLastFrame;

            DVec3 dir = normalize(DVec3{ 0.3, 0.5, 0.8 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(ln, d) > 0.30f) { dir = d; break; }
            }
            const DVec3 ref = (std::fabs(dir.y) < 0.9) ? DVec3{0,1,0} : DVec3{1,0,0};
            const DVec3 tang = normalize(cross(ref, dir));

            const double pxPerRad = (kWindowHeight * 0.5) /
                                    std::tan(space::deg2rad(kFovYDegrees) * 0.5);
            const double speed = 60.0, dt = 1.0 / 60.0;
            int    frames = 0, lateSplits = 0, totalAppear = 0;
            int    lateByLevel[20] = {}, seenByLevel[20] = {};
            double worstFraction = 1.0;

            std::unordered_map<PatchKey, int, PatchKeyHash> lastUsed;
            int    ancestorEvictions = 0, totalEvictions = 0;
            int    badMerges = 0, totalMerges = 0;
            int    mergeByLevel[20] = {};
            double worstMergeMorph = 1.0;

            const int kWarmup = 900;
            for (int f = -kWarmup; f < kFlightFrames + kSqueezeFrames; ++f) {
                const bool flying    = f >= 0;
                const bool squeezing = f >= kFlightFrames;
                const int  simCap    = squeezing ? kCacheTight : kCacheRoomy;
                if (flying) ++frames;

                const double s = flying ? speed * dt * f : 0.0;
                const DVec3 here = normalize(dir * kPlanetRadius + tang * s);
                const DVec3 cam  = here * (surfaceRadiusAt(ln, here, kPhysicsDetailOctaves) + 25.0);

                int landed = 0;
                for (auto& kv : requestedAt)
                    if (landed < kMaxUploadsPerFrame && !live.count(kv.first) &&
                        f - kv.second >= kLatency) { live.insert(kv.first); ++landed; }

                std::vector<PatchKey> wanted;
                const std::vector<SelectedPatch> sel =
                    selectLODAdaptive(cam, bounds, [&](const PatchKey& k) {
                        return k.level == 0 || live.count(k) > 0; }, wanted);
                for (const PatchKey& k : wanted)
                    requestedAt.emplace(k, f);

                std::unordered_set<PatchKey, PatchKeyHash> drawnNow;
                for (const SelectedPatch& sp : sel) drawnNow.insert(sp.key);
                for (const SelectedPatch& sp : sel) {
                    if (!flying || squeezing || drawnLastFrame.count(sp.key) ||
                        sp.key.level == 0) continue;

                    const PatchKey parent{ sp.key.face, sp.key.level - 1,
                                           sp.key.ix >> 1, sp.key.iy >> 1 };
                    if (!drawnLastFrame.count(parent)) continue;
                    ++totalAppear;

                    const PatchBounds& pb = bounds(sp.key);
                    const double morphEnd = 2.0 * pb.worldEdge / double(kSplitFactor);
                    const double d = std::max(space::length(cam - pb.surfaceCenter) -
                                              pb.surfaceRadius, 0.0);
                    double tt = (d - morphEnd * kMorphBandStart) /
                                (morphEnd * (1.0 - kMorphBandStart));
                    tt = tt < 0.0 ? 0.0 : (tt > 1.0 ? 1.0 : tt);
                    const double m = tt * tt * (3.0 - 2.0 * tt);
                    ++seenByLevel[std::min(19, sp.key.level)];
                    if (m < 0.99) {
                        ++lateSplits;
                        ++lateByLevel[std::min(19, sp.key.level)];

                        worstFraction = std::min(worstFraction, d / morphEnd);
                    }
                }

                if (flying && !squeezing) {
                    for (const PatchKey& gone : drawnLastFrame) {
                        if (drawnNow.count(gone) || gone.level == 0) continue;
                        const PatchKey par{ gone.face, gone.level - 1,
                                            gone.ix >> 1, gone.iy >> 1 };
                        if (!drawnNow.count(par)) continue;
                        ++totalMerges;
                        const PatchBounds& gb = bounds(gone);
                        const double me = 2.0 * gb.worldEdge / double(kSplitFactor);
                        const double d = std::max(space::length(cam - gb.surfaceCenter)
                                                  - gb.surfaceRadius, 0.0);
                        double tt = (d - me * kMorphBandStart) /
                                    (me * (1.0 - kMorphBandStart));
                        tt = tt < 0.0 ? 0.0 : (tt > 1.0 ? 1.0 : tt);
                        const double m = tt * tt * (3.0 - 2.0 * tt);
                        worstMergeMorph = std::min(worstMergeMorph, m);
                        if (m < 0.99) { ++badMerges; ++mergeByLevel[std::min(19, gone.level)]; }
                    }
                }

                if (flying) {
                    for (const SelectedPatch& sp : sel) lastUsed[sp.key] = f;
                    if (kTouchAncestors) {
                        for (const SelectedPatch& sp : sel) {
                            PatchKey a = sp.key;
                            while (a.level > 0) {
                                a = PatchKey{ a.face, a.level - 1, a.ix >> 1, a.iy >> 1 };
                                if (!live.count(a)) break;
                                auto it2 = lastUsed.find(a);
                                if (it2 != lastUsed.end() && it2->second == f) break;
                                lastUsed[a] = f;
                            }
                        }
                    }
                    if (static_cast<int>(live.size()) > simCap) {

                        std::unordered_set<PatchKey, PatchKeyHash> activeTree;
                        for (const SelectedPatch& sp : sel) {
                            PatchKey a = sp.key;
                            while (a.level > 0) {
                                a = PatchKey{ a.face, a.level - 1, a.ix >> 1, a.iy >> 1 };
                                if (!activeTree.insert(a).second) break;
                            }
                        }
                        std::vector<std::pair<int, PatchKey>> vict;
                        for (const PatchKey& k : live) {
                            if (k.level == 0) continue;
                            auto it2 = lastUsed.find(k);
                            const int used = it2 == lastUsed.end() ? -1 : it2->second;
                            if (used == f) continue;
                            vict.push_back({ used, k });
                        }
                        std::sort(vict.begin(), vict.end(),
                                  [](const auto& a2, const auto& b2) { return a2.first < b2.first; });
                        int drop = static_cast<int>(live.size()) - simCap;
                        for (size_t vi = 0; vi < vict.size() && drop > 0; ++vi, --drop) {
                            ++totalEvictions;
                            if (activeTree.count(vict[vi].second)) ++ancestorEvictions;
                            live.erase(vict[vi].second);
                            requestedAt.erase(vict[vi].second);
                        }
                    }
                }
                drawnLastFrame.swap(drawnNow);
            }
            std::printf("         streaming: %d frames of flight at %.0f m/s after a"
                        " %d-frame warm-up:\n"
                        "           %d of %d newly drawn patches arrived with the morph"
                        " unfinished (%.0f%%);\n"
                        "           the worst appeared at %.2f of its merge distance."
                        " By level:", frames, speed, kWarmup, lateSplits, totalAppear,
                        100.0 * lateSplits / std::max(1, totalAppear), worstFraction);
            for (int L = 1; L < 20; ++L)
                if (seenByLevel[L])
                    std::printf(" %d:%d/%d", L, lateByLevel[L], seenByLevel[L]);
            std::printf("\n");
            (void)pxPerRad;
            check(totalAppear > 50, "streaming: the simulated flight really does stream");

            check(lateSplits * 20 <= totalAppear,
                  "lod: under 5% of splits arrive before their morph is done");
            check(worstFraction > 0.90,
                  "lod: and even the worst is 90% of the way through its morph");
            std::printf("         merges: %d of %d were incomplete, worst child"
                        " morph %.4f\n", badMerges, totalMerges, worstMergeMorph);
            std::printf("         cache: %d evictions, %d of them interior nodes of"
                        " the tree being drawn\n", totalEvictions, ancestorEvictions);
            check(totalEvictions > 50, "cache: the flight really does fill the cache");
            check(ancestorEvictions == 0,
                  "cache: the LRU never evicts an ancestor the descent is using");
            check(totalMerges > 20, "lod: the flight exercises merges too");
            check(badMerges == 0,
                  "lod: a patch that MERGES away was fully morphed when it went");
        }

        {
            World cn;
            PatchBoundsCache cb(cn);

            DVec3 site = normalize(DVec3{ -0.6, 0.25, 0.75 });
            for (int t = 0; t < 8192; ++t) {
                const double y  = 1.0 - 2.0 * (t + 0.5) / 8192;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * t;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                if (sampleLandE(cn, d) > 0.25f && std::fabs(y) < 0.5) { site = d; break; }
            }
            const DVec3 cam = site * (surfaceRadiusAt(cn, site, kPhysicsDetailOctaves) + 20.0);

            std::vector<PatchKey> ideal;
            {
                std::vector<PatchKey> junk;
                for (const SelectedPatch& sp :
                     selectLODAdaptive(cam, cb, [](const PatchKey&) { return true; }, junk))
                    ideal.push_back(sp.key);
            }

            const int kLatency = 12;
            auto converge = [&](bool askIdeal, int jobs, int uploads, int inFlightCap) {
                std::unordered_map<PatchKey, int, PatchKeyHash> requestedAt;
                std::unordered_set<PatchKey, PatchKeyHash> live;
                std::unordered_set<PatchKey, PatchKeyHash> want(ideal.begin(), ideal.end());

                std::vector<PatchKey> idealChain;
                if (askIdeal) {
                    std::unordered_set<PatchKey, PatchKeyHash> seen;
                    for (const PatchKey& k : ideal) {
                        PatchKey a2 = k;
                        for (;;) {
                            if (!seen.insert(a2).second) break;
                            idealChain.push_back(a2);
                            if (a2.level == 0) break;
                            a2 = PatchKey{ a2.face, a2.level - 1, a2.ix >> 1, a2.iy >> 1 };
                        }
                    }

                    std::sort(idealChain.begin(), idealChain.end(),
                              [](const PatchKey& x, const PatchKey& y) {
                                  return x.level < y.level; });
                }

                int inFlight = 0, f = 0;
                for (; f < 6000; ++f) {
                    int landed = 0;
                    for (auto& kv : requestedAt)
                        if (landed < uploads && !live.count(kv.first) &&
                            f - kv.second >= kLatency) {
                            live.insert(kv.first); ++landed; --inFlight;
                        }
                    std::vector<PatchKey> asked;
                    const std::vector<SelectedPatch> sel = selectLODAdaptive(
                        cam, cb, [&](const PatchKey& k) {
                            return k.level == 0 || live.count(k) > 0; }, asked);
                    if (sel.size() == want.size()) {
                        bool same = true;
                        for (const SelectedPatch& sp : sel)
                            if (!want.count(sp.key)) same = false;
                        if (same) break;
                    }
                    const std::vector<PatchKey>& list = askIdeal ? idealChain : asked;
                    int dispatched = 0;
                    for (const PatchKey& k : list) {
                        if (dispatched >= jobs || inFlight >= inFlightCap) break;
                        if (live.count(k)) continue;
                        if (requestedAt.emplace(k, f).second) { ++dispatched; ++inFlight; }
                    }
                }
                return f;
            };

            int deepest = 0;
            for (const PatchKey& k : ideal) deepest = std::max(deepest, k.level);
            std::printf("         convergence to %zu leaves at level %d"
                        " (12-frame build latency):\n", ideal.size(), deepest);
            std::printf("           request policy | jobs/frame | uploads/frame |"
                        " frames | seconds\n");
            const int frames = converge(false, kMaxJobsPerFrame, kMaxUploadsPerFrame,
                                        kMaxJobsInFlight);
            struct Row { const char* name; bool ideal; int jobs; int ups; int fly; };

            const Row rows[] = {
                { "AS SHIPPED     ", false, kMaxJobsPerFrame, kMaxUploadsPerFrame,
                                            kMaxJobsInFlight },
                { "old cap (96)   ", false, kMaxJobsPerFrame, kMaxUploadsPerFrame,  96 },
                { "old budgets    ", false, 16,               16,                   96 },
                { "whole subtree  ", true,  kMaxJobsPerFrame, kMaxUploadsPerFrame,  96 },
                { "whole subtree  ", true,  kMaxJobsPerFrame, kMaxUploadsPerFrame,
                                            kMaxJobsInFlight },
            };
            for (const Row& r : rows) {
                const int n = converge(r.ideal, r.jobs, r.ups, r.fly);
                std::printf("           %s |%11d |%14d |%10d |%7d | %5.2f\n",
                            r.name, r.jobs, r.ups, r.fly, n, n / 60.0);
            }
            check(frames < 6000, "lod: a fresh viewpoint converges at all");

            check(frames < 400, "lod: a fresh viewpoint converges without stalling");
        }

        const int N = kPatchResolution;
        PatchMeshData pm = generatePatch(world, 2, 3, 5, 6, N);
        size_t expectV = size_t(N+1)*(N+1) + size_t(4)*(N+1);
        size_t expectI = size_t(N)*N*6 + size_t(4)*N*6;
        check(pm.vertices.size() == expectV, "patch: vertex count = grid + skirts");
        check(pm.indices.size()  == expectI, "patch: index count = grid + skirt walls");

        bool finite = true, radiusOk = true, localSmall = true;
        const double patchExtent = space::length(
            faceToSphereD(2, -1.0 + 0.25 * 5, -1.0 + 0.25 * 6) * double(kPlanetRadius) -
            faceToSphereD(2, -1.0 + 0.25 * 6, -1.0 + 0.25 * 7) * double(kPlanetRadius));
        for (int i = 0; i < (N+1)*(N+1); ++i) {
            const Vertex& v = pm.vertices[i];
            const UnpackedNormal vn = vertexNormal(v);
            if (!(std::isfinite(v.px) && std::isfinite(vn.x))) finite = false;
            const double rr = space::length(pm.origin + DVec3{ v.px, v.py, v.pz });

            const double tol = 1e-6 * (patchExtent + kPlanetRadius) + 1e-3;
            if (rr < kPlanetRadius - tol || rr > kPlanetRadius + kTerrainAmplitude + tol) radiusOk = false;

            const double ll = space::length(DVec3{ v.px, v.py, v.pz });
            if (ll > patchExtent + kTerrainAmplitude) localSmall = false;
        }
        check(finite, "patch: all vertices finite");
        check(radiusOk, "patch: origin + local position lands on [radius, radius+amplitude]");
        check(localSmall, "patch: local coordinates stay patch-sized (not world-sized)");
        check(std::fabs(space::length(pm.origin) - kPlanetRadius) < 1e-6 * kPlanetRadius,
              "patch: the origin sits at sea level");

        World world2;
        PatchMeshData pm2 = generatePatch(world2, 2, 3, 5, 6, N);
        bool identical = pm.vertices.size() == pm2.vertices.size();
        for (size_t i = 0; identical && i < pm.vertices.size(); ++i)
            identical = approx(pm.vertices[i].px, pm2.vertices[i].px, 0.0f) &&
                        approx(pm.vertices[i].py, pm2.vertices[i].py, 0.0f) &&
                        approx(pm.vertices[i].pz, pm2.vertices[i].pz, 0.0f);
        check(identical, "patch: regeneration is deterministic for a fixed seed");

        {
            const int kReps = 8;
            const auto t0 = std::chrono::steady_clock::now();
            size_t sink = 0;
            for (int r = 0; r < kReps; ++r)
                sink += generatePatch(world, 2, 16, 30000 + r, 40000, N).vertices.size();
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0).count() / kReps;
            std::printf("         patch generation at level 16 (%d detail octaves):"
                        " %.2f ms  [%zu]\n", detailOctavesForLevel(16), ms, sink / kReps);
            check(ms < 200.0, "patch: generation stays inside the streaming budget");
        }
    }

    {
        World sn;

        DVec3 site = normalize(DVec3{ 0.3, 0.5, 0.8 });
        for (int i = 0; i < 4096; ++i) {
            const double y  = 1.0 - 2.0 * (i + 0.5) / 4096;
            const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
            const double th = 2.399963229728653 * i;
            const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
            if (sampleLandE(sn, d) > 0.15f && std::fabs(y) < 0.5) { site = d; break; }
        }
        const double siteR = surfaceRadiusAt(sn, site, kPhysicsDetailOctaves);
        const DVec3 cam = site * (siteR + 40.0);

        int capped = 0;
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<ScatterInstance> field = collectScatter(sn, cam, kScatterRange,
                                                            kPhysicsDetailOctaves,
                                                            &capped);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        std::printf("         scatter: %zu shards within %.0f m in %.2f ms"
                    " (%d over budget)\n",
                    field.size(), kScatterRange, ms, capped);
        check(!field.empty(), "scatter: land actually gets shards");
        check(static_cast<int>(field.size()) <= kScatterMaxInstances,
              "scatter: the count stays inside its budget");

        check(ms < 16.0, "scatter: a rebuild fits inside one frame");

        {
            std::vector<ScatterInstance> again =
                collectScatter(sn, cam, kScatterRange, kPhysicsDetailOctaves);
            bool same = again.size() == field.size();
            for (size_t i = 0; same && i < field.size(); ++i)
                same = again[i].pos.x == field[i].pos.x &&
                       again[i].pos.y == field[i].pos.y &&
                       again[i].pos.z == field[i].pos.z &&
                       again[i].scale == field[i].scale;
            check(same, "scatter: the same place gives the identical field");
        }

        {
            const DVec3 offCam = cam + normalize(cross(site, DVec3{0,1,0})) * 100.0;
            std::vector<ScatterInstance> other =
                collectScatter(sn, offCam, kScatterRange, kPhysicsDetailOctaves);
            int overlap = 0, mismatched = 0;
            for (const ScatterInstance& a : field)
                for (const ScatterInstance& b : other)
                    if (space::length(a.pos - b.pos) < 1.0) {
                        ++overlap;
                        if (a.pos.x != b.pos.x || a.scale != b.scale ||
                            a.orient.x != b.orient.x) ++mismatched;
                        break;
                    }
            std::printf("         scatter: %d shards shared with a camera 100 m"
                        " away, %d of them differ\n", overlap, mismatched);
            check(overlap > 10, "scatter: two nearby cameras see a shared field");
            check(mismatched == 0,
                  "scatter: a shard is identical from either camera (rebuild is invisible)");
        }

        {
            double worstGap = 0.0; int inSea = 0, tooSteep = 0;
            for (const ScatterInstance& s : field) {
                const DVec3 d = normalize(s.pos);
                const float e = sampleLandE(sn, d);
                if (e <= 0.0f) ++inSea;
                const double r = surfaceRadius(sn, d, sampleShape(sn, d), kPhysicsDetailOctaves);
                worstGap = std::max(worstGap, std::fabs(space::length(s.pos) - r));

                if (space::dot(space::rotate(s.orient, Vec3{0,1,0}), space::toF(d)) < 0.5f)
                    ++tooSteep;
            }
            std::printf("         scatter: worst distance from the surface"
                        " %.4f m\n", worstGap);
            check(worstGap < 1e-3, "scatter: every shard sits exactly on the terrain");
            check(inSea == 0, "scatter: nothing is planted in the sea");
            check(tooSteep == 0, "scatter: every shard stands up, not into the ground");
        }

        {
            const ShipMeshData shard = buildCrystalShard();
            check(!shard.vertices.empty() && shard.indices.size() % 3 == 0,
                  "scatter: the shard mesh is well formed");
            double lo = 1e18, hi = -1e18;
            for (const Vertex& v : shard.vertices) { lo = std::min<double>(lo, v.py);
                                                     hi = std::max<double>(hi, v.py); }
            check(std::fabs(lo) < 1e-6 && std::fabs(hi - 1.0) < 1e-6,
                  "scatter: the shard is a UNIT mesh, base at zero and 1 m tall");
        }

        {
            TerrainGround tg;
            Destination dest[kDestinationCount];
            Outpost     posts[kDestinationCount];
            buildDestinations(dest, tg);
            buildOutposts(posts, tg);

            int onLand = 0;
            double worstOffset = 0.0, worstStandoff = 0.0, worstTilt = 0.0;
            for (int i = 0; i < kDestinationCount; ++i) {
                const DVec3 dd = normalize(dest[i].pos), pd = normalize(posts[i].pos);

                worstOffset = std::max(worstOffset, space::length(dd - pd));

                worstStandoff = std::max(worstStandoff,
                    std::fabs((space::length(dest[i].pos) - space::length(posts[i].pos))
                              - kArrivalStandoff));

                const Vec3 up = space::rotate(posts[i].orient, Vec3{ 0, 1, 0 });
                worstTilt = std::max(worstTilt,
                                     1.0 - double(space::dot(up, space::toF(pd))));

                const double r = surfaceRadiusAt(tg.w(), pd, kPhysicsDetailOctaves);
                check(std::fabs(space::length(posts[i].pos) - r) < 1e-3,
                      "outpost: the base stands on the terrain");
                if (sampleLandE(tg.w(), pd) > 0.0f) ++onLand;
            }
            std::printf("         outposts: %d of %d on land; worst direction"
                        " offset %.2e, standoff error %.3f m, tilt %.2e\n",
                        onLand, kDestinationCount, worstOffset, worstStandoff,
                        worstTilt);
            check(worstOffset < 1e-12,
                  "outpost: each one sits directly under its destination");
            check(worstStandoff < 1e-6,
                  "outpost: the destination is exactly the standoff above it");
            check(worstTilt < 1e-6, "outpost: it stands on the local vertical");

            check(onLand == kDestinationCount,
                  "outpost: every site is on dry land, none in the sea");

            int qualify = 0;
            for (int i = 0; i < kDestinationCount; ++i)
                if (tg(normalize(posts[i].pos) * kPlanetRadius).radius >
                    kPlanetRadius + kSiteMinRelief) ++qualify;
            check(qualify == kDestinationCount,
                  "outpost: every site clears kSiteMinRelief, not just sea level");

            {
                int high = 0, total = 0;
                const int S2 = 20000;
                for (int i = 0; i < S2; ++i) {
                    const double y  = 1.0 - 2.0 * (i + 0.5) / S2;
                    const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                    const double th = 2.399963229728653 * i;
                    const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                    ++total;
                    if (terrainRadius(kTestParams, sampleShape(tg.w(), d)) >
                        kPlanetRadius + kSiteMinRelief) ++high;
                }
                std::printf("         land clearing kSiteMinRelief (%.0f m):"
                            " %.1f%% of the sphere\n",
                            kSiteMinRelief, 100.0 * high / total);
                check(high * 12 > total,
                      "outpost: enough of the planet clears the site threshold");
            }

            check(kOutpostVisibleRange > kArrivalStandoff,
                  "outpost: drawn from further away than a jump arrives");

            const ShipMeshData om = buildOutpost();
            check(!om.vertices.empty() && om.indices.size() % 3 == 0,
                  "outpost: the mesh is well formed");
            float lo = 1e9f, hi = -1e9f;
            for (const Vertex& v : om.vertices) { lo = std::min(lo, v.py); hi = std::max(hi, v.py); }
            std::printf("         outpost mesh: %zu triangles, spans %.0f to %.0f m\n",
                        om.indices.size() / 3, lo, hi);

            check(hi > 100.0f, "outpost: tall enough to see from the arrival standoff");
            check(lo < 0.0f, "outpost: its legs go below the surface, not to it");
        }

        check(scatterDensity(kTestParams, -0.1f, 0.1f, 0.5f) == 0.0f, "scatter: the sea is bare");
        check(scatterDensity(kTestParams, 0.3f, 0.9f, 0.5f) == 0.0f, "scatter: cliffs are bare");
        check(scatterDensity(kTestParams, 0.3f, 0.1f, 0.5f) > 0.0f, "scatter: ordinary land is not");
        check(scatterDensity(kTestParams, 0.3f, 0.1f, 0.0f) > scatterDensity(kTestParams, 0.3f, 0.1f, 1.0f),
              "scatter: dry ground carries more debris than wet");
    }

    {

        World lodNoise;
        std::vector<PatchKey> far = selectLOD(DVec3{0, 0, kPlanetRadius * 100.0}, lodNoise);
        int farMax = 0; for (auto& k : far) farMax = std::max(farMax, k.level);
        check(far.size() == 6, "lod: far camera => 6 root patches (one per face)");
        check(farMax == 0, "lod: far camera => no subdivision");

        DVec3 nearDir = normalize(DVec3{0.3, 0.5, 0.8});
        DVec3 nearCam = nearDir * (kPlanetRadius + 30.0);
        std::vector<PatchKey> near = selectLOD(nearCam, lodNoise);
        int nearMax = 0; for (auto& k : near) nearMax = std::max(nearMax, k.level);
        check(near.size() > far.size(), "lod: closer camera => more patches (Req #5)");
        check(nearMax > farMax, "lod: closer camera => deeper subdivision (Req #5/#6)");
        bool bounded = true; for (auto& k : near) if (k.level > kMaxDepth) bounded = false;
        check(bounded, "lod: maximum quadtree depth is never exceeded (Req #6)");

        PatchKey deepest = near.front();
        for (auto& k : near) if (k.level > deepest.level) deepest = k;
        PatchBounds pbDeep = computePatchBounds(deepest.face, deepest.level,
                                                deepest.ix, deepest.iy, lodNoise);
        double distDeep = length(pbDeep.center - nearCam);
        check(distDeep < kPlanetRadius, "lod: deepest patch is near the camera (local refinement)");

        check(near.size() < 4000, "lod: active patch count stays bounded");

        std::vector<PatchKey> mid = selectLOD(nearDir * (kPlanetRadius + 10000.0),
                                              lodNoise);
        std::printf("         lod leaves: 30 m %zu, 10 km %zu, far %zu\n",
                    near.size(), mid.size(), far.size());
        check(mid.size() < near.size() && mid.size() >= far.size(),
              "lod: retreating reduces subdivision back toward the root (Req #6)");

        size_t worstOrbit = 0;
        for (double alt : { 30000.0, 60000.0, 120000.0, 300000.0, 1000000.0 }) {
            const size_t n = selectLOD(nearDir * (kPlanetRadius + alt), lodNoise).size();
            std::printf("           %8.0f m -> %zu leaves (split %.3f)\n",
                        alt, n, splitFactorAt(kPlanetRadius + alt));
            worstOrbit = std::max(worstOrbit, n);
        }

        std::printf("           worst %zu leaves of a %d-entry cache\n",
                    worstOrbit, kMeshCacheCapacity);
        check(worstOrbit < size_t(kMeshCacheCapacity * 0.85),
              "lod: the altitude ramp stays inside the mesh budget");
    }

    {
        const Quat I = quatIdentity();
        const Vec3 v{ 0.3f, -0.7f, 1.1f };

        const Vec3 rv = rotate(I, v);
        check(rv.x == v.x && rv.y == v.y && rv.z == v.z,
              "quat: identity rotation is bit-exact");

        const Quat a = fromAxisAngle(Vec3{ 0.3f, 1.0f, -0.2f }, 0.9f);
        check(approx(angleBetween(a * conjugate(a), I), 0.0f, 1e-5f),
              "quat: q * conj(q) is the identity");
        check(approx(length(rotate(a, v)), length(v), 1e-5f),
              "quat: rotation preserves length");

        const Vec3 r90 = rotate(fromAxisAngle(Vec3{ 0, 1, 0 }, deg2rad(90.0f)),
                                Vec3{ 0, 0, 1 });
        check(approx(r90.x, 1.0f, 1e-5f) && approx(r90.z, 0.0f, 1e-5f),
              "quat: +90 deg about +Y maps +Z to +X");

        const Quat qy = fromAxisAngle(Vec3{ 0, 1, 0 }, deg2rad(90.0f));
        const Quat qx = fromAxisAngle(Vec3{ 1, 0, 0 }, deg2rad(90.0f));
        const Vec3 lhs = rotate(qy * qx, Vec3{ 0, 1, 0 });
        const Vec3 rhs = rotate(qy, rotate(qx, Vec3{ 0, 1, 0 }));
        check(approx(lhs.x, rhs.x, 1e-5f) && approx(lhs.y, rhs.y, 1e-5f) &&
              approx(lhs.z, rhs.z, 1e-5f), "quat: (a*b) applies b then a");

        bool bridge = true;
        for (float yw = -3.0f; yw <= 3.0f; yw += 0.7f)
            for (float pt = -1.4f; pt <= 1.4f; pt += 0.35f) {
                const Basis be = makeBasis(yw, pt);
                const Basis bq = toBasis(fromYawPitch(yw, pt));
                if (!(approx(be.forward.x, bq.forward.x, 1e-5f) &&
                      approx(be.forward.y, bq.forward.y, 1e-5f) &&
                      approx(be.forward.z, bq.forward.z, 1e-5f) &&
                      approx(be.up.x, bq.up.x, 1e-5f) &&
                      approx(be.up.y, bq.up.y, 1e-5f) &&
                      approx(be.up.z, bq.up.z, 1e-5f) &&
                      approx(be.right.x, bq.right.x, 1e-5f) &&
                      approx(be.right.y, bq.right.y, 1e-5f) &&
                      approx(be.right.z, bq.right.z, 1e-5f))) bridge = false;
            }
        check(bridge, "quat: toBasis(fromYawPitch(y,p)) == makeBasis(y,p)");

        const Basis bb = toBasis(a);
        check(approx(dot(bb.forward, bb.right), 0.0f, 1e-5f) &&
              approx(dot(bb.forward, bb.up), 0.0f, 1e-5f) &&
              approx(dot(bb.right, bb.up), 0.0f, 1e-5f) &&
              approx(length(bb.forward), 1.0f, 1e-5f),
              "quat: toBasis is orthonormal");
        const Vec3 rxu = cross(bb.right, bb.up);
        check(approx(dot(rxu, bb.forward), 1.0f, 1e-4f),
              "quat: toBasis is right-handed (right x up == forward)");

        const DVec3 dv = rotate(a, DVec3{ v.x, v.y, v.z });
        const Vec3  fv = rotate(a, v);
        check(std::fabs(dv.x - fv.x) < 1e-5 && std::fabs(dv.y - fv.y) < 1e-5 &&
              std::fabs(dv.z - fv.z) < 1e-5, "quat: double rotate matches float rotate");

        check(approx(angleBetween(fromTo(Vec3{0,1,0}, Vec3{0,1,0}), I), 0.0f, 1e-5f),
              "quat: fromTo of identical vectors is the identity");
        const Vec3 flip = rotate(fromTo(Vec3{ 0, 1, 0 }, Vec3{ 0, -1, 0 }), Vec3{ 0, 1, 0 });
        check(approx(flip.y, -1.0f, 1e-4f),
              "quat: fromTo handles the antiparallel case");
        const Vec3 ft = rotate(fromTo(Vec3{ 0, 1, 0 }, Vec3{ 0.6f, 0.8f, 0 }),
                               Vec3{ 0, 1, 0 });
        check(approx(ft.x, 0.6f, 1e-4f) && approx(ft.y, 0.8f, 1e-4f),
              "quat: fromTo maps `from` onto `to`");

        const float rate = 2.5f, dtq = 1.0f / 120.0f;
        const int   nSteps = 1000;
        Quat spin = I;
        for (int i = 0; i < nSteps; ++i)
            spin = integrate(spin, Vec3{ 0, rate, 0 }, dtq);
        const Quat closedForm = fromAxisAngle(Vec3{ 0, 1, 0 }, rate * nSteps * dtq);
        check(angleBetween(spin, closedForm) < 1e-4f,
              "quat: 1000 integration steps equal the closed-form rotation");

        Quat drift = I;
        for (int i = 0; i < 100000; ++i)
            drift = integrate(drift, Vec3{ 0.7f, -1.3f, 0.4f }, dtq);
        check(approx(std::sqrt(dot(drift, drift)), 1.0f, 1e-5f),
              "quat: norm survives 100k integration steps");

        const Quat still = integrate(a, Vec3{ 0, 0, 0 }, dtq);
        check(still.x == a.x && still.y == a.y && still.z == a.z && still.w == a.w,
              "quat: a zero angular rate leaves the orientation untouched");

        check(approx(angleBetween(nlerp(I, a, 0.0f), I), 0.0f, 1e-5f) &&
              approx(angleBetween(nlerp(I, a, 1.0f), a), 0.0f, 1e-5f),
              "quat: nlerp hits both endpoints");
    }

    {

        check(space::length(DVec3{ kPlanetRadius, 0, 0 }) == kPlanetRadius,
              "dvec3: a megametre length is exact");

        const double size    = 2.0 / static_cast<double>(1 << kMaxDepth);
        const double cellUV  = size / kPatchResolution;
        const double deltaUV = cellUV * 0.5;
        const float  ulp     = std::nextafter(1.0f, 2.0f) - 1.0f;
        check(cellUV / ulp < 8.0,
              "precision: a float (u,v) resolves under 8 steps per cell at kMaxDepth");
        check(deltaUV / ulp < 4.0,
              "precision: a float normal step at kMaxDepth is only a few ULP");
        check(deltaUV / (size / kPatchResolution) == 0.5, "precision: sanity");

        const DVec3 a{ kPlanetRadius, 0, 0 };
        const DVec3 b{ kPlanetRadius + 0.02, 0, 0 };
        check(static_cast<float>((b - a).x) == 0.02f,
              "precision: subtract-then-narrow keeps a 2 cm difference");
        check(space::toF(b).x == space::toF(a).x,
              "precision: narrow-then-subtract loses it entirely");
    }

    {
        const double cell = 2.0 * kPlanetRadius /
                            (static_cast<double>(1 << kMaxDepth) * kPatchResolution);
        check(cell < 1.0, "lod: the finest cell is sub-metre at kMaxDepth");

        World deepNoise;
        PatchBoundsCache deepBounds(deepNoise);
        DVec3 deepCam = normalize(DVec3{0.3, 0.5, 0.8}) * (kPlanetRadius + 30.0);

        std::vector<PatchKey> wanted;
        std::vector<SelectedPatch> none =
            selectLODAdaptive(deepCam, deepBounds,
                              [](const PatchKey&) { return false; }, wanted);
        check(none.size() == 6, "lod: with nothing resident the descent falls back to 6 roots");
        check(!wanted.empty(), "lod: the descent reports the children it is missing");

        std::vector<PatchKey> wanted2;
        std::vector<SelectedPatch> all =
            selectLODAdaptive(deepCam, deepBounds,
                              [](const PatchKey&) { return true; }, wanted2);
        std::vector<PatchKey> pure = selectLOD(deepCam, deepNoise);
        bool same = all.size() == pure.size();
        for (size_t i = 0; same && i < all.size(); ++i) same = all[i].key == pure[i];
        check(same, "lod: fully-resident adaptive descent == pure selectLOD");
        check(wanted2.empty(), "lod: nothing is wanted when everything is resident");

        size_t worstLeaves = 0;
        DVec3 lowDir  = normalize(DVec3{0.3, 0.5, 0.8});
        DVec3 peakDir = lowDir;
        double peakE = -1e9;
        for (int i = 0; i < 4096; ++i) {
            const double y  = 1.0 - 2.0 * (i + 0.5) / 4096;
            const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
            const double th = 2.399963229728653 * i;
            DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };

            const double r = terrainRadius(kTestParams, sampleShape(deepNoise, d));
            if (r > peakE) { peakE = r; peakDir = d; }
        }
        const double peakGround = peakE;
        for (double h : { 5.0, 30.0, 300.0, 3000.0, 50000.0, 1000000.0 }) {
            std::vector<PatchKey> v1 = selectLOD(lowDir  * (kPlanetRadius + h), deepNoise);
            std::vector<PatchKey> v2 = selectLOD(peakDir * (peakGround   + h), deepNoise);
            int m1 = 0; for (auto& k : v1) m1 = k.level > m1 ? k.level : m1;
            int m2 = 0; for (auto& k : v2) m2 = k.level > m2 ? k.level : m2;
            std::printf("      [info] %9.0f m AGL -> lowland %6zu leaves (lvl %2d) | "
                        "peak %6zu leaves (lvl %2d)\n", h, v1.size(), m1, v2.size(), m2);
            worstLeaves = std::max(worstLeaves, std::max(v1.size(), v2.size()));
        }
        check(worstLeaves < 6000, "lod: leaf count stays bounded at every altitude");
        check(worstLeaves <= static_cast<size_t>(kMeshCacheCapacity),
              "lod: the worst-case active set fits inside the mesh cache budget");

        PatchKeyHash hash;
        const int kBuckets = 4096, kKeys = 100000;
        std::vector<int> hist(kBuckets, 0);
        for (int i = 0; i < kKeys; ++i)
            hist[hash({ i % 6, kMaxDepth, i * 7, i * 13 }) & (kBuckets - 1)]++;
        const double expect = double(kKeys) / kBuckets;
        int worst = 0;
        for (int c : hist) worst = std::max(worst, std::abs(c - static_cast<int>(expect)));
        check(worst < static_cast<int>(expect),
              "patchkey: hash spreads deep keys evenly over buckets");
    }

    {
        auto noGround = [](const DVec3& p) { return flatGround(p, 0.0); };

        ShipControls coast; coast.assist = false;

        const DVec3 gSurf = gravityAt(DVec3{ kPlanetRadius, 0, 0 });
        check(std::fabs(space::length(gSurf) - kSurfaceGravity) < 1e-9,
              "gravity: |g| at the surface equals kSurfaceGravity");
        check(gSurf.x < 0.0, "gravity: points toward the planet centre");
        const DVec3 g2R = gravityAt(DVec3{ 2.0 * kPlanetRadius, 0, 0 });
        check(std::fabs(space::length(g2R) - kSurfaceGravity / 4.0) < 1e-9,
              "gravity: |g| at 2R is a quarter of the surface value");

        {
            ShipState s;
            s.pos = DVec3{ 0, kPlanetRadius + 1000.0, 0 };
            const double expected = std::sqrt(2.0 * 1000.0 / kSurfaceGravity);
            double t = 0.0;
            while (altitudeASL(s) > 0.0 && t < 60.0) {
                step(s, coast, noGround, kFixedDt);
                t += kFixedDt;
            }
            check(std::fabs(t - expected) / expected < 0.01,
                  "integrator: free-fall time from 1000 m matches the analytic value");
        }

        {
            ShipState s;
            const double r = 2.0 * kPlanetRadius;
            const double v = std::sqrt(kMu / r);
            s.pos = DVec3{ r, 0, 0 };
            s.vel = DVec3{ 0, v, 0 };
            const double period = 2.0 * kPi * r / v;
            double worst = 0.0;
            for (double t = 0.0; t < period; t += kFixedDt) {
                step(s, coast, noGround, kFixedDt);
                worst = std::max(worst, std::fabs(space::length(s.pos) - r) / r);
            }
            check(worst < 1e-3,
                  "integrator: a circular orbit holds its radius over a full period");
        }

        {
            ShipState s;
            s.pos = DVec3{ kPlanetRadius * 10.0, 0, 0 };
            s.angVel = Vec3{ 0.0f, 0.8f, 0.0f };
            for (int i = 0; i < 12000; ++i) step(s, coast, noGround, kFixedDt);
            check(approx(s.angVel.y, 0.8f, 1e-4f) && approx(s.angVel.x, 0.0f, 1e-5f),
                  "integrator: a spin about a principal axis stays constant");
        }
        {
            ShipState s;
            s.pos = DVec3{ kPlanetRadius * 10.0, 0, 0 };
            s.angVel = Vec3{ 0.5f, -0.9f, 0.3f };
            auto momentum = [](const Vec3& w) {
                const Vec3 L{ static_cast<float>(w.x * kInertiaPitch),
                              static_cast<float>(w.y * kInertiaYaw),
                              static_cast<float>(w.z * kInertiaRoll) };
                return space::length(L);
            };
            const float L0 = momentum(s.angVel);
            for (int i = 0; i < 7200; ++i) step(s, coast, noGround, kFixedDt);
            const float L1 = momentum(s.angVel);

            check(std::fabs(L1 - L0) / L0 < 1e-5f,
                  "integrator: an off-axis tumble conserves |L| (gyroscopic term)");
            check(std::fabs(s.angVel.x - 0.5f) > 1e-3f,
                  "integrator: an off-axis tumble actually precesses");
        }

        {
            ShipState s;
            s.pos = DVec3{ kPlanetRadius * 10.0, 0, 0 };
            check(step(s, coast, noGround, 0.1) == 12,
                  "integrator: a 0.1 s frame runs 12 substeps");
            ShipState s2;
            s2.pos = DVec3{ kPlanetRadius * 10.0, 0, 0 };
            check(step(s2, coast, noGround, 10.0) == kMaxSubsteps &&
                  s2.accum == 0.0,
                  "integrator: an absurd frame time clamps and drops the backlog");
        }

        {
            ShipState s;
            s.pos = DVec3{ kPlanetRadius * 10.0, 0, 0 };
            ShipControls c; c.gearDown = true;
            for (int i = 0; i < 400; ++i) step(s, c, noGround, kFixedDt);
            check(approx(s.gearT, 1.0f, 1e-6f), "gear: extends fully and stops at 1");
            c.gearDown = false;
            for (int i = 0; i < 400; ++i) step(s, c, noGround, kFixedDt);
            check(approx(s.gearT, 0.0f, 1e-6f), "gear: retracts fully and stops at 0");
        }

        check(kPhysicsDetailOctaves == detailOctavesForLevel(kMaxDepth) &&
              kPhysicsDetailOctaves < kMaxDetailOctaves,
              "physics: ground queries use the rendered octave count, not the cap");

        const DVec3 deepSpace{ kPlanetRadius * 200.0, 0, 0 };

        {
            ShipState s; s.pos = deepSpace;
            ShipControls c; c.assist = true; c.pitch = 1.0f;
            float peak = 0.0f;
            double tReach = -1.0;
            for (int i = 0; i < 360; ++i) {
                step(s, c, noGround, kFixedDt);
                peak = std::max(peak, s.angVel.x);
                if (tReach < 0.0 && s.angVel.x >= 0.99f * kRateMaxPitch)
                    tReach = (i + 1) * kFixedDt;
            }
            check(tReach > 0.0 && tReach < 1.0,
                  "ifcs: full pitch deflection reaches 99% of max rate within 1 s");
            check(peak <= kRateMaxPitch * 1.001f,
                  "ifcs: the rate command never overshoots");
        }

        {
            ShipState s; s.pos = deepSpace;
            bool withinAuthority = true;
            unsigned rng = 12345u;
            auto rnd = [&rng] {
                rng = rng * 1664525u + 1013904223u;
                return static_cast<float>(static_cast<int>(rng >> 8) % 2001 - 1000) / 1000.0f;
            };
            for (int i = 0; i < 4000; ++i) {
                ShipControls c; c.assist = true;
                c.pitch = rnd(); c.yaw = rnd(); c.roll = rnd();
                const Vec3 before = s.angVel;
                step(s, c, noGround, kFixedDt);
                const Vec3 d = s.angVel - before;

                if (std::fabs(d.x) > (kAngAccelPitch * kFixedDt) * 1.15f ||
                    std::fabs(d.y) > (kAngAccelYaw   * kFixedDt) * 1.15f ||
                    std::fabs(d.z) > (kAngAccelRoll  * kFixedDt) * 1.15f)
                    withinAuthority = false;
            }
            check(withinAuthority,
                  "ifcs: commanded angular acceleration never exceeds thruster authority");
        }

        {
            ShipState s; s.pos = deepSpace;
            s.vel = DVec3{ 0, 0, 900.0 };
            s.angVel = Vec3{ 0.0f, 0.4f, 0.0f };
            ShipControls c; c.assist = false;
            for (int i = 0; i < 1200; ++i) step(s, c, noGround, kFixedDt);
            check(std::fabs(space::length(s.vel) - 900.0) < 1e-6,
                  "ifcs: decoupled keeps velocity with no input (Newtonian)");
            check(approx(s.angVel.y, 0.4f, 1e-4f),
                  "ifcs: decoupled keeps rotating with no input");
        }

        {
            ShipState s; s.pos = deepSpace;
            s.vel = DVec3{ 0, 0, 0 };
            s.orient = quatIdentity();
            s.vel = space::rotate(s.orient, DVec3{ 120.0, 0, 0 });
            ShipControls c; c.assist = true;

            for (int i = 0; i < 1800; ++i) step(s, c, noGround, kFixedDt);
            check(space::length(s.vel) < 1.0,
                  "ifcs: assisted flight bleeds off lateral drift");
        }

        {
            ShipState s; s.pos = deepSpace;
            ShipControls c; c.assist = true; c.throttle = 1.0f;
            for (int i = 0; i < 3600; ++i) step(s, c, noGround, kFixedDt);
            check(space::length(s.vel) <= kScmSpeed + 1.0 &&
                  space::length(s.vel) > kScmSpeed - 1.0,
                  "ifcs: assisted throttle settles at the SCM speed cap");
            c.boost = true;
            for (int i = 0; i < 12000; ++i) step(s, c, noGround, kFixedDt);
            check(space::length(s.vel) <= kNavSpeed + 1.0 &&
                  space::length(s.vel) > kNavSpeed - 1.0,
                  "ifcs: boost raises the cap to the NAV speed");
        }

        {
            ShipState s;
            s.pos = DVec3{ 0, kPlanetRadius + 2000.0, 0 };

            s.orient = fromTo(Vec3{ 0, 1, 0 }, Vec3{ 0, 1, 0 });
            const double alt0 = altitudeASL(s);
            ShipControls c; c.assist = true;
            for (int i = 0; i < 3600; ++i) step(s, c, noGround, kFixedDt);
            check(std::fabs(altitudeASL(s) - alt0) < 0.05,
                  "ifcs: gravity compensation holds altitude hands-off");
        }

        {
            ShipState s;
            s.pos = DVec3{ 0, kPlanetRadius + 2000.0, 0 };
            const double alt0 = altitudeASL(s);
            ShipControls c; c.assist = false;
            for (int i = 0; i < 1200; ++i) step(s, c, noGround, kFixedDt);
            const double expected = 0.5 * kSurfaceGravity * 100.0;
            check(std::fabs((alt0 - altitudeASL(s)) - expected) / expected < 0.02,
                  "ifcs: decoupled does not counter gravity");
        }

        const double kGround = kPlanetRadius;
        auto flat = [&](const DVec3& p) { return flatGround(p, kGround); };

        const double restComp   = (kShipMass * kSurfaceGravity / 4.0) / kGearK;
        const double restCentre = kGround + kGearLegLength - restComp - kGearPivot[0].y;

        {
            ShipState s;
            s.pos   = DVec3{ 0, kGround + 30.0, 0 };
            s.gearT = 1.0f;
            ShipControls c; c.assist = false; c.gearDown = true;

            double maxRebound = 0.0;
            bool   settledInTime = false;
            for (int i = 0; i < 1800; ++i) {
                step(s, c, flat, kFixedDt);
                if (i > 600) maxRebound = std::max(maxRebound,
                                                   space::length(s.pos) - restCentre);
                if (!settledInTime && s.landed && i * kFixedDt < 10.0) settledInTime = true;
            }
            check(settledInTime, "gear: a 30 m drop settles to 'landed' within 10 s");
            check(space::length(s.vel) < 1e-9,
                  "gear: the ship comes fully to rest (asleep)");

            check(std::fabs(space::length(s.pos) - restCentre) < 0.03,
                  "gear: the rest height matches the closed-form spring compression");
            check(maxRebound < 0.1,
                  "gear: no bounce (overdamped linear mode)");
            bool allLegs = true;
            for (int i = 0; i < 4; ++i) allLegs = allLegs && s.legContact[i];
            check(allLegs, "gear: all four legs are loaded at rest");
            check(std::fabs(s.legComp[0] - restComp) < 0.03,
                  "gear: each leg carries a quarter of the weight");
        }

        {
            ShipState s;
            s.pos   = DVec3{ 0, kGround + 5.0, 0 };
            s.gearT = 0.0f;
            ShipControls c; c.assist = false; c.gearDown = false;
            for (int i = 0; i < 600; ++i) step(s, c, flat, kFixedDt);
            bool anyContact = false;
            for (int i = 0; i < 4; ++i) anyContact = anyContact || s.legContact[i];
            check(!anyContact && !s.landed,
                  "gear: retracted gear never makes contact (a belly landing is a crash)");
            check(space::length(s.pos) < kGround,
                  "gear: with the gear up the ship falls straight through");
        }

        {
            const double tilt = deg2rad(15.0f);
            const DVec3 n{ std::sin(tilt), std::cos(tilt), 0.0 };
            const DVec3 onPlane{ 0, kGround, 0 };
            auto slope = [&](const DVec3& p) { return planeGround(p, onPlane, n); };

            ShipState s;
            s.pos   = DVec3{ 0, kGround + 12.0, 0 };
            s.gearT = 1.0f;
            ShipControls c; c.assist = false; c.gearDown = true;

            for (int i = 0; i < 2400; ++i) step(s, c, slope, kFixedDt);

            const DVec3 up = shipUp(s);
            const double misalign = std::acos(std::min(1.0, space::dot(up, n)));
            check(misalign < deg2rad(1.5f),
                  "gear: the ship levels itself to a 15 deg slope (within 1.5 deg)");
            check(s.landed, "gear: the ship reaches the landed state on a slope");

            const DVec3 restPos = s.pos;
            for (int i = 0; i < 2400; ++i) step(s, c, slope, kFixedDt);
            std::printf("      [info] slope: crept %.3f m in 20 s, |v| %.4f m/s, "
                        "|w| %.4f rad/s\n",
                        space::length(s.pos - restPos), space::length(s.vel),
                        space::length(s.angVel));
            check(space::length(s.pos - restPos) < 0.2,
                  "gear: friction holds it on the slope (no creep)");
            check(space::length(s.vel) < 1e-9 && space::length(s.angVel) < 1e-9f,
                  "gear: the resting state is exactly still, not jittering");
        }

        {
            World terrain;
            auto realGround = [&](const DVec3& p) {
                GroundHit h;
                const DVec3 dir = space::normalize(p);
                const float e   = sampleLandE(terrain, dir);
                h.radius = surfaceRadius(terrain, dir, sampleShape(terrain, dir), kPhysicsDetailOctaves);

                const DVec3 ref = (std::fabs(dir.y) < 0.9) ? DVec3{ 0, 1, 0 } : DVec3{ 1, 0, 0 };
                const DVec3 t1  = space::normalize(space::cross(ref, dir));
                const DVec3 t2  = space::cross(dir, t1);
                const double eps = 0.5;
                auto radAt = [&](const DVec3& d) {
                    const DVec3 dn = space::normalize(d);
                    return surfaceRadiusAt(terrain, dn, kPhysicsDetailOctaves);
                };
                const double r0 = h.radius;
                const double d1 = (radAt(dir * r0 + t1 * eps) - r0) / eps;
                const double d2 = (radAt(dir * r0 + t2 * eps) - r0) / eps;
                h.normal = space::normalize(dir - t1 * d1 - t2 * d2);
                return h;
            };

            auto reliefAround = [&](const DVec3& d, double eps) {
                const double r0 = surfaceRadiusAt(terrain, d, kPhysicsDetailOctaves);
                const DVec3 ref = (std::fabs(d.y) < 0.9) ? DVec3{0,1,0} : DVec3{1,0,0};
                const DVec3 t1 = normalize(cross(ref, d)), t2 = cross(d, t1);
                double lo = 1e18, hi = -1e18;
                for (int a = -1; a <= 1; ++a)
                    for (int b = -1; b <= 1; ++b) {
                        const DVec3 q = normalize(d * r0 + t1 * (a * eps) + t2 * (b * eps));
                        const double rr = surfaceRadiusAt(terrain, q, kPhysicsDetailOctaves);
                        lo = std::min(lo, rr); hi = std::max(hi, rr);
                    }
                return hi - lo;
            };

            DVec3 dir = space::normalize(DVec3{ 0.3, 0.5, 0.8 });
            float bestE = -1e9f;
            for (int i = 0; i < 2048; ++i) {
                const double y  = 1.0 - 2.0 * (i + 0.5) / 2048;
                const double rr = std::sqrt(std::max(0.0, 1.0 - y * y));
                const double th = 2.399963229728653 * i;
                const DVec3 d{ std::cos(th) * rr, y, std::sin(th) * rr };
                const float e = sampleLandE(terrain, d);
                if (e <= bestE) continue;

                if (reliefAround(d, 30.0) > 12.0) continue;
                bestE = e; dir = d;
            }
            check(bestE > 0.0f, "gear: the search found land to put the ship on");
            std::printf("      [info] landing site: e=%.2f  relief +-2m %.2f m,"
                        " +-8m %.2f m, +-30m %.2f m\n", bestE,
                        reliefAround(dir, 2.0), reliefAround(dir, 8.0),
                        reliefAround(dir, 30.0));

            const double groundR = surfaceRadiusAt(terrain, dir, kPhysicsDetailOctaves);

            ShipState s;
            s.pos   = dir * (groundR + 15.0);

            s.orient = fromTo(Vec3{ 0, 1, 0 }, space::toF(dir));
            s.gearT  = 1.0f;
            const DVec3 startPos = s.pos;
            ShipControls c; c.assist = false; c.gearDown = true;
            for (int i = 0; i < 3600; ++i) step(s, c, realGround, kFixedDt);

            {
                const GroundHit gh = realGround(s.pos);
                double d = space::dot(shipUp(s), space::normalize(s.pos));
                d = std::max(-1.0, std::min(1.0, d));
                std::printf("      [info] real-terrain landing: land elevation %.2f, "
                            "slid %.2f m, tilt %.1f deg, |v| %.3f m/s, clearance %.2f m\n",
                            bestE, space::length(s.pos - startPos),
                            std::acos(d) * 180.0 / kPi, space::length(s.vel),
                            space::length(s.pos) - gh.radius);
            }

            check(s.landed, "gear: lands and settles on the real fBm terrain");

            double worstGap = 0.0;
            for (int i = 0; i < 4; ++i) {
                const DVec3 footLocal{ kGearPivot[i].x,
                                       kGearPivot[i].y - kGearLegLength,
                                       kGearPivot[i].z };
                const DVec3 foot = s.pos + space::rotate(s.orient, footLocal);
                const GroundHit g = realGround(foot);
                worstGap = std::max(worstGap,
                                    std::fabs(g.radius - space::length(foot)) - kGearTravel);
            }
            check(worstGap < 0.05,
                  "gear: every foot rests within suspension travel of the real surface");

            const GroundHit under = realGround(s.pos);
            const double dip = under.radius - space::length(s.pos);
            std::printf("      [info] hull centre sits %.2f m %s the terrain under it\n",
                        std::fabs(dip), dip > 0 ? "below" : "above");
            check(dip < kShipHeight * 0.5,
                  "gear: the hull is not buried in terrain it straddles");
        }

        {
            ShipState s;
            s.pos   = DVec3{ 0, restCentre, 0 };
            s.gearT = 1.0f;
            ShipControls c; c.assist = false; c.gearDown = true;
            for (int i = 0; i < 400; ++i) step(s, c, flat, kFixedDt);
            check(s.landed, "gear: sitting at the rest height reads as landed");
            c.strafeY = 1.0f;
            for (int i = 0; i < 600; ++i) step(s, c, flat, kFixedDt);
            check(space::length(s.pos) > restCentre + 5.0 && !s.landed,
                  "gear: full vertical thrust lifts off from the landed state");
        }
    }

    {
        ShipState ship;
        ship.pos    = DVec3{ 0, kPlanetRadius + 5000.0, 0 };
        ship.orient = quatIdentity();
        const float dtc = 1.0f / 60.0f;

        {
            CameraState cam;
            CameraView v = updateCamera(cam, ship, 0, 0, false, dtc);
            check(space::length(v.pos - (ship.pos + toD(kEyeOffset))) < 1e-6,
                  "camera: the cockpit eye sits at kEyeOffset in the hull");
            check(angleBetween(v.orient, ship.orient) < 1e-5f,
                  "camera: the cockpit view matches the ship's orientation");

            ship.orient = fromAxisAngle(Vec3{ 0, 0, 1 }, deg2rad(40.0f));
            v = updateCamera(cam, ship, 0, 0, false, dtc);
            const Basis cb = toBasis(v.orient);
            const Vec3  shipUpV = rotate(ship.orient, Vec3{ 0, 1, 0 });
            check(dot(cb.up, shipUpV) > 0.99999f,
                  "camera: the cockpit rolls with the ship (no world-up lock)");
            check(std::fabs(dot(cb.up, Vec3{ 0, 1, 0 }) - std::cos(deg2rad(40.0f))) < 1e-4f,
                  "camera: rolled 40 deg, the view is 40 deg off world up");
            ship.orient = quatIdentity();
        }

        {
            CameraState cam;
            for (int i = 0; i < 30; ++i) updateCamera(cam, ship, 40.0f, 0.0f, true, dtc);
            CameraView v = updateCamera(cam, ship, 0.0f, 0.0f, true, dtc);
            const float turned = angleBetween(v.orient, ship.orient);
            check(turned > deg2rad(5.0f),
                  "camera: free-look turns the view away from the ship's nose");
            for (int i = 0; i < 120; ++i) updateCamera(cam, ship, 0, 0, false, dtc);
            v = updateCamera(cam, ship, 0, 0, false, dtc);
            check(angleBetween(v.orient, ship.orient) < deg2rad(0.5f),
                  "camera: releasing free-look springs the view back to centre");
        }

        {
            CameraState cam;
            for (int i = 0; i < 4000; ++i) updateCamera(cam, ship, 100.0f, -100.0f, true, dtc);

            check(std::fabs(cam.freeYaw)   <= kFreeLookYawCockpit   + 1e-4f &&
                  std::fabs(cam.freePitch) <= kFreeLookPitchCockpit + 1e-4f,
                  "camera: free-look stays inside the cockpit cone");

            cam.thirdPerson = true;
            updateCamera(cam, ship, 0, 0, true, dtc);
            cam.thirdPerson = false;
            updateCamera(cam, ship, 0, 0, true, dtc);
            check(std::fabs(cam.freeYaw) <= kFreeLookYawCockpit + 1e-4f,
                  "camera: switching view modes re-clamps the free-look offset");
        }

        {
            for (double speed : { 50.0, 250.0, 1200.0 }) {
                ShipState s;
                s.pos    = DVec3{ 0, kPlanetRadius + 50000.0, 0 };
                s.orient = quatIdentity();
                s.vel    = DVec3{ 0, 0, speed };
                CameraState cam; cam.thirdPerson = true;
                double worst = 0.0;
                for (int i = 0; i < 600; ++i) {
                    s.pos = s.pos + s.vel * dtc;
                    CameraView v = updateCamera(cam, s, 0, 0, false, dtc);
                    const DVec3 want = s.pos + rotate(s.orient, toD(kChaseOffset));
                    worst = std::max(worst, space::length(v.pos - want));
                }
                check(worst < 1e-6,
                      "camera: the chase view has zero lag at constant velocity");
            }
        }

        {
            ShipState s;
            s.pos = DVec3{ 0, kPlanetRadius + 50000.0, 0 };
            CameraState cam; cam.thirdPerson = true;
            updateCamera(cam, s, 0, 0, false, dtc);
            float worstLag = 0.0f, seenLag = 0.0f;
            for (int i = 0; i < 600; ++i) {
                s.orient = space::integrate(s.orient, Vec3{ 0, 1.0f, 0 }, dtc);
                updateCamera(cam, s, 0, 0, false, dtc);
                const float lag = angleBetween(cam.chaseRot, s.orient);
                worstLag = std::max(worstLag, lag);
                if (i > 60) seenLag = lag;
            }
            check(seenLag > deg2rad(2.0f),
                  "camera: the chase view lags during a sustained turn");
            check(worstLag <= kChaseMaxLag + 1e-3f,
                  "camera: the chase lag is bounded so the ship stays in frame");
        }
    }

    {
        const ShipMeshData hull = buildShipHull();
        const ShipMeshData leg  = buildGearLeg();

        check(!hull.vertices.empty() && hull.indices.size() % 3 == 0,
              "shipmesh: the hull is a well-formed triangle list");
        check(hull.vertices.size() == hull.indices.size(),
              "shipmesh: no vertex is shared (flat shading by construction)");

        bool finite = true, unitNormals = true;
        space::Vec3 lo{ 1e9f, 1e9f, 1e9f }, hi{ -1e9f, -1e9f, -1e9f };
        for (const Vertex& v : hull.vertices) {
            const UnpackedNormal vn2 = vertexNormal(v);
            if (!(std::isfinite(v.px) && std::isfinite(vn2.x)))
                finite = false;
            if (!approx(space::length(space::Vec3{ vn2.x, vn2.y, vn2.z }), 1.0f, 1e-4f))
                unitNormals = false;
            lo = space::Vec3{ std::min(lo.x, v.px), std::min(lo.y, v.py), std::min(lo.z, v.pz) };
            hi = space::Vec3{ std::max(hi.x, v.px), std::max(hi.y, v.py), std::max(hi.z, v.pz) };
        }
        check(finite, "shipmesh: all hull vertices are finite");
        check(unitNormals, "shipmesh: every face normal is unit length");

        check(hi.z - lo.z <= kShipLength + 1.0f && hi.z - lo.z >= kShipLength - 2.0f,
              "shipmesh: hull length matches kShipLength (the inertia tensor's box)");
        check(hi.x - lo.x <= kShipSpan + 0.5f,
              "shipmesh: hull span does not exceed kShipSpan");
        check(hi.y - lo.y <= kShipHeight + 0.5f,
              "shipmesh: hull height does not exceed kShipHeight");

        float legLow = 1e9f;
        for (const Vertex& v : leg.vertices) legLow = std::min(legLow, v.py);
        check(approx(legLow, -kGearLegLength, 1e-4f),
              "shipmesh: the gear leg reaches exactly kGearLegLength below its pivot");
    }

    {
        JobSystem js;
        js.start(4);
        check(js.threaded(), "jobsystem: start() spins up workers");

        std::atomic<int> counter{ 0 };
        for (int i = 0; i < 1000; ++i) js.enqueue([&counter] { ++counter; });
        while (js.pending() != 0) std::this_thread::yield();
        js.stop();
        check(counter.load() == 1000, "jobsystem: every enqueued job runs exactly once");
        check(!js.threaded(), "jobsystem: stop() joins the workers");
        js.stop();
        check(true, "jobsystem: stop() is idempotent");

        JobSystem inlineJs;
        int ran = 0;
        inlineJs.enqueue([&ran] { ++ran; });
        check(ran == 1 && inlineJs.pending() == 0,
              "jobsystem: with no workers, enqueue runs inline");
    }

    {
        using namespace planet::atmo;
        const double R = kPlanetRadius;

        {
            const Hit up = intersectShell(0.0f, 1.0f, kTopRadii);
            check(up.hit && approx(up.t1, kTopRadii, 1e-6f),
                  "atmo: straight up from sea level leaves the shell at its top");
        }
        check(approx(float(distanceToTop(0.0f, 0.0f) * R), 407921.0f, 100.0f),
              "atmo: a horizontal ray runs 408 km inside the shell (the sunset path)");

        {

            const float aOrb = float(2400000.0 / R);
            const Hit   shell = intersectShell(aOrb, -1.0f, kTopRadii);
            check(shell.hit && approx(float(shell.t0 * R), 2320000.0f, 200.0f),
                  "atmo: from 2400 km orbit the shell is entered after 2320 km");
            check(approx(float(distanceToGround(aOrb, -1.0f) * R), 2400000.0f, 200.0f),
                  "atmo: ... and the ground stops the march at 2400 km");

            const Hit away = intersectShell(aOrb, 1.0f, kTopRadii);
            check(!away.hit || (away.t0 < 0.0f && away.t1 < 0.0f),
                  "atmo: from orbit looking outward the shell lies behind the ray");
        }

        check(distanceToGround(0.01f, -0.05f) < 0.0f,
              "atmo: from 10 km, a ray above the horizon never reaches the ground");
        check(distanceToGround(0.01f, -0.20f) > 0.0f,
              "atmo: from 10 km, a ray below the horizon does reach the ground");
        check(!hitsGround(0.0f, 0.001f) && hitsGround(0.0f, -0.001f),
              "atmo: at sea level the horizon sits at mu = 0");

        {

            const double aD  = 2.0 / R;
            const double muD = -1.5 * std::sqrt(aD * (aD + 2.0));
            const double bD  = (1.0 + aD) * muD;
            const double ref = -bD - std::sqrt(bD * bD - aD * (aD + 2.0));

            const float af = float(aD), muf = float(muD);
            const float rf = 1.0f + af;
            const float bN = rf * muf;
            const float cN = rf * rf - 1.0f;
            const float tN = -bN - std::sqrt(bN * bN - cN);
            const float tF = distanceToGround(af, muf);

            const double errN = std::fabs(double(tN) - ref) * R;
            const double errF = std::fabs(double(tF) - ref) * R;
            std::printf("         ground hit at 2 m eye height: naive c off by %.2f m, "
                        "factored c off by %.5f m\n", errN, errF);
            check(errF < 0.01 && errN > 1.0,
                  "atmo: factoring c = a(a+2) keeps the ground hit sub-centimetre");
        }

        {
            const float km = 1000.0f / float(R);
            struct Row { const char* what; float altM; float mu; float t0km; float t1km; };

            const Row rows[] = {
                { "eye -> zenith",      2.0f,      1.0f,     0.0f,    80.0f },
                { "eye -> horizon",     2.0f,      0.0f,     0.0f,   407.9f },
                { "20 km -> horizon",   20000.0f,  0.0f,     0.0f,   354.9f },
                { "orbit -> planet",    2400000.0f, -1.0f, 2320.0f, 2400.0f },
            };
            bool ok = true;
            for (const Row& r : rows) {
                const MarchSpan s = marchSpan(float(r.altM / R), r.mu);
                const bool good = !s.empty() &&
                                  std::fabs(s.t0 / km - r.t0km) < 1.0f &&
                                  std::fabs(s.t1 / km - r.t1km) < 1.0f;
                if (!good) {
                    std::printf("         MISMATCH %-20s got %.1f..%.1f km, "
                                "want %.1f..%.1f km\n",
                                r.what, s.t0 / km, s.t1 / km, r.t0km, r.t1km);
                    ok = false;
                }
            }
            check(ok, "march: the bounds match the planned table at every altitude");

            check(marchSpan(0.0f, -0.5f).empty(),
                  "march: pointing into the ground leaves no segment");

            check(marchSpan(0.0f, 0.0f).empty(),
                  "march: the exactly-tangent ray is treated as blocked");
            check(!marchSpan(1.0f / float(R), 0.0f).empty(),
                  "march: one metre above sea level it is open again");

            check(marchSpan(float(2400000.0 / R), 1.0f).empty(),
                  "march: from orbit, looking outward, there is no atmosphere");

            {

                const float aOrb = float(2400000.0 / R);
                const float grazing = -std::sqrt(1.0f - 1.0f / ((1.0f + aOrb) *
                                                                (1.0f + aOrb)));
                const MarchSpan s = marchSpan(aOrb, grazing * 0.995f);
                check(!s.empty() && s.t0 > 0.0f,
                      "march: a ray grazing the limb still crosses the shell");
                check(!hitsGround(aOrb, grazing * 0.995f),
                      "march: ... and that same ray misses the ground");
            }

            {
                const float justOut = marchSpan(kTopRadii * 1.0001f, -0.3f).t1 -
                                      marchSpan(kTopRadii * 1.0001f, -0.3f).t0;
                const float justIn  = marchSpan(kTopRadii * 0.9999f, -0.3f).t1 -
                                      marchSpan(kTopRadii * 0.9999f, -0.3f).t0;
                check(std::fabs(justOut - justIn) / justIn < 1e-3f,
                      "march: crossing the atmosphere boundary is continuous");
            }
        }

        {

            const OpticalDepth od = opticalDepth(0.0, 1.0, kAtmosphereTop, 4000);
            const double refR = kRayleighScaleHeight *
                                (1.0 - std::exp(-kAtmosphereTop / kRayleighScaleHeight));
            const double refM = kMieScaleHeight *
                                (1.0 - std::exp(-kAtmosphereTop / kMieScaleHeight));
            check(std::fabs(od.rayleigh - refR) / refR < 0.01,
                  "atmo: the zenith Rayleigh column matches H*(1-exp(-T/H)) within 1%");
            check(std::fabs(od.mie - refM) / refM < 0.01,
                  "atmo: the zenith Mie column matches the closed form within 1%");
        }
        {

            const Vec3 zRef = transmittanceToTop(0.0, 1.0, 4000);
            const Vec3 zNow = transmittanceToTop(0.0, 1.0);
            const Vec3 hRef = transmittanceToTop(0.0, 0.0, 4000);
            const Vec3 hNow = transmittanceToTop(0.0, 0.0);
            const double ez = std::fabs(zNow.z - zRef.z) / zRef.z;
            const double eh = std::fabs(hNow.z - hRef.z) / hRef.z;
            std::printf("         %d steps: zenith %.4f%% off, horizon %.4f%% off\n",
                        kTransmittanceSteps, 100.0 * ez, 100.0 * eh);
            check(ez < 0.01 && eh < 0.01,
                  "atmo: kTransmittanceSteps integrates both extremes within 1%");
        }

        {
            const Vec3 zen = transmittanceToTop(0.0, 1.0, 4000);
            const Vec3 hor = transmittanceToTop(0.0, 0.0, 4000);
            std::printf("         sea level -> space:  zenith  R %.3f G %.3f B %.3f\n",
                        zen.x, zen.y, zen.z);
            std::printf("                              horizon R %.3f G %.3f B %.3f\n",
                        hor.x, hor.y, hor.z);

            check(zen.x > zen.y && zen.y > zen.z,
                  "atmo: blue is extinguished more than green, green more than red");
            check(approx(zen.z, 0.603f, 0.01f),
                  "atmo: the zenith keeps 60% of its blue -> a deep blue sky");
            check(hor.z < 0.01f,
                  "atmo: at the horizon under 1% of the blue survives");
            check(hor.x > 0.20f,
                  "atmo: at the horizon over 20% of the red survives -> red sunsets");
            check(hor.x / hor.z > 50.0f,
                  "atmo: the horizon is over 50x redder than it is blue");
        }
        {
            bool monoUp = true;
            Vec3 prev = transmittanceToTop(0.0, 0.3);
            for (int i = 1; i <= 80; ++i) {
                const Vec3 t = transmittanceToTop(i * 1000.0, 0.3);
                if (t.x < prev.x || t.y < prev.y || t.z < prev.z) monoUp = false;
                prev = t;
            }
            check(monoUp, "atmo: transmittance rises monotonically with altitude");

            bool monoDown = true;
            prev = transmittanceToTop(0.0, 1.0);
            for (int i = 1; i <= 100; ++i) {
                const Vec3 t = transmittanceToTop(0.0, 1.0 - i * 0.01);
                if (t.x > prev.x || t.y > prev.y || t.z > prev.z) monoDown = false;
                prev = t;
            }
            check(monoDown, "atmo: transmittance falls monotonically toward the horizon");
        }

        {
            float worstRow = 0.0f, worstCol = 0.0f;
            for (int j = 0; j < kLutHeight; ++j) {
                const float xr = (j + 0.5f) / kLutHeight;
                const float a  = lutAltitudeFromRow(xr);
                worstRow = std::max(worstRow, std::fabs(lutRowFromAltitude(a) - xr));
                for (int i = 0; i < kLutWidth; ++i) {
                    const float xc = (i + 0.5f) / kLutWidth;
                    worstCol = std::max(worstCol,
                                        std::fabs(lutColFromMu(a, lutMuFromCol(a, xc)) - xc));
                }
            }
            std::printf("         LUT mapping round-trip: row %.2e, column %.2e\n",
                        worstRow, worstCol);
            check(worstRow < 1e-4f && worstCol < 1e-3f,
                  "atmo: the LUT mapping and its inverse round-trip");
            check(approx(lutAltitudeFromRow(0.0f), 0.0f, 1e-7f),
                  "atmo: LUT row 0 is exactly sea level");
            check(approx(lutAltitudeFromRow(1.0f), kTopRadii, 1e-5f),
                  "atmo: LUT row 1 is exactly the top of the shell");
        }

        {
            const auto t0 = std::chrono::steady_clock::now();
            const TransmittanceLut lut = buildTransmittanceLut();
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0).count();
            std::printf("         LUT %dx%d built in %.1f ms (%zu KB)\n",
                        lut.width, lut.height, ms, lut.byteSize() / 1024);
            check(lut.valid(), "atmo: the LUT has the size it claims");
            check(ms < 100.0, "atmo: LUT generation is a startup cost, not a stall");

            const TransmittanceLut again = buildTransmittanceLut();
            check(again.rgba == lut.rgba, "atmo: LUT generation is deterministic");

            float worst = 0.0f;
            for (int j = 0; j < 127; ++j) {
                const float a = lutAltitudeFromRow(j / 126.0f);
                for (int i = 0; i < 511; ++i) {
                    const float mu = lutMuFromCol(a, i / 510.0f);
                    const Vec3  s  = sampleLut(lut, a, mu);
                    const Vec3  e  = transmittanceToTop(double(a) * R, mu, 400);
                    worst = std::max(worst, std::max(std::fabs(s.x - e.x),
                                     std::max(std::fabs(s.y - e.y), std::fabs(s.z - e.z))));
                }
            }
            std::printf("         worst bilinear LUT error over all channels: %.5f\n", worst);
            check(worst < 0.002f,
                  "atmo: sampling the LUT matches the integrator to under 0.002");

            {
                const float muGraze = lutMuFromCol(kTopRadii, 1.0f);
                const Vec3  sg = sampleLut(lut, kTopRadii, muGraze);
                const Vec3  eg = transmittanceToTop(kAtmosphereTop, muGraze, 400);
                check(std::fabs(sg.x - eg.x) < 1e-4f,
                      "atmo: the grazing limb sits ON a texel, so it is read exactly");
            }

            {
                float worstUV = 0.0f;
                for (int j = 0; j < lut.height; j += 7)
                    for (int i = 0; i < lut.width; i += 11) {
                        const Vec3 got = sampleLutUV(lut, (i + 0.5f) / lut.width,
                                                          (j + 0.5f) / lut.height);
                        const float* px = &lut.rgba[(size_t(j) * lut.width + i) * 4];
                        worstUV = std::max(worstUV, std::max(std::fabs(got.x - px[0]),
                                           std::max(std::fabs(got.y - px[1]),
                                                    std::fabs(got.z - px[2]))));
                    }
                check(worstUV < 1e-6f,
                      "atmo: sampleLutUV at a texel centre returns that exact texel");
            }

            check(approx(lutTexCoord(0.0f, lut.width), 0.5f / lut.width, 1e-6f) &&
                  approx(lutTexCoord(1.0f, lut.width), 1.0f - 0.5f / lut.width, 1e-6f),
                  "atmo: lutTexCoord puts parameter 0 and 1 on the first and last texel");
        }

        {
            const TransmittanceLut lut = buildTransmittanceLut();
            const Vec3 up{ 0.0f, 1.0f, 0.0f };
            const Vec3 sunUp{ 0.0f, 1.0f, 0.0f };
            const Vec3 sunSet = normalize(Vec3{ 1.0f, 0.02f, 0.0f });

            {
                const Vec3 z = skyRadiance(up, up, 2.0f, sunUp, lut);
                const Vec3 d = acesTonemap(z * kExposure);
                std::printf("         zenith radiance R %.4f G %.4f B %.4f "
                            "-> displayed B/R %.2f\n", z.x, z.y, z.z,
                            d.z / std::max(d.x, 1e-9f));
                check(z.z > z.y && z.y > z.x,
                      "sky: an overhead sun gives a blue zenith (B > G > R)");
                check(z.z > 0.0f && z.z < 100.0f,
                      "sky: zenith radiance is in a sane range for F16");
            }

            {
                const Vec3 sun45 = normalize(Vec3{ 1.0f, 1.0f, 0.0f });
                const Vec3 z = skyRadiance(up, up, 2.0f, sun45, lut);
                const Vec3 d = acesTonemap(z * kExposure);
                const float ratio = d.z / std::max(d.x, 1e-9f);
                std::printf("         sun at 45 deg, zenith: displayed B/R %.2f\n", ratio);
                check(ratio > 2.5f,
                      "sky: away from the aureole the zenith is emphatically blue");
            }

            {
                const Vec3 h = skyRadiance(normalize(Vec3{ 1.0f, 0.01f, 0.0f }),
                                           up, 2.0f, sunSet, lut);
                const Vec3 z = skyRadiance(up, up, 2.0f, sunSet, lut);
                std::printf("         low sun: horizon R %.4f G %.4f B %.4f | "
                            "zenith B/R %.2f\n", h.x, h.y, h.z, z.z / std::max(z.x, 1e-9f));
                check(h.x > h.z,
                      "sky: with a low sun the horizon is redder than it is blue");
                check(z.z > z.x,
                      "sky: ... while the zenith overhead stays blue");
            }

            {
                const float aOrb = float(2400000.0 / R);
                const Vec3  s = skyRadiance(up, up, float(2400000.0), sunUp, lut);
                check(s.x == 0.0f && s.y == 0.0f && s.z == 0.0f,
                      "sky: from orbit, looking outward, the sky is exactly black");
                check(marchSpan(aOrb, 1.0f).empty(),
                      "sky: ... because the march span is empty, not because it cancels");
            }

            {
                const float aOrb = float(2400000.0 / R);
                const float graze = -std::sqrt(1.0f - 1.0f / ((1.0f + aOrb) * (1.0f + aOrb)));
                const Vec3  dir = normalize(up * (graze * 0.997f) +
                                            Vec3{ 1.0f, 0.0f, 0.0f } *
                                            std::sqrt(1.0f - graze * graze * 0.994f));
                const Vec3  l = skyRadiance(dir, up, float(2400000.0),
                                            normalize(Vec3{ 1.0f, 0.3f, 0.0f }), lut);
                check(l.x + l.y + l.z > 0.0f, "sky: the limb from orbit is not black");
            }

            {
                const Vec3 night = skyRadiance(up, up, 2.0f,
                                               Vec3{ 0.0f, -1.0f, 0.0f }, lut);
                const Vec3 day   = skyRadiance(up, up, 2.0f, sunUp, lut);
                check(night.x + night.y + night.z < 1e-6f * (day.x + day.y + day.z),
                      "sky: with the sun below the horizon the zenith goes dark");
            }

            {
                std::printf("         march steps vs a 256-step reference "
                            "(worst 8-bit error over 200 directions):\n");
                for (int steps : { 8, 12, 16, 24, 32 }) {
                    int worst = 0;
                    for (int i = 0; i < 200; ++i) {

                        const float el  = -0.05f + 1.05f * (i % 20) / 19.0f;
                        const float az  = 6.2831853f * ((i / 20) % 10) / 10.0f;
                        const Vec3  dir = normalize(Vec3{ std::cos(az), el, std::sin(az) });
                        const Vec3  sun = (i % 2) ? normalize(Vec3{ 1.0f, 0.05f, 0.0f })
                                                  : normalize(Vec3{ 0.4f, 0.8f, 0.2f });
                        const Vec3 a = acesTonemap(skyRadiance(dir, up, 2.0f, sun, lut,
                                                               1e9f, steps) * kExposure);
                        const Vec3 b = acesTonemap(skyRadiance(dir, up, 2.0f, sun, lut,
                                                               1e9f, 256) * kExposure);
                        auto q = [](float v) { return int(std::lround(std::min(std::max(v, 0.0f),
                                                                               1.0f) * 255.0f)); };
                        worst = std::max(worst, std::max(std::abs(q(a.x) - q(b.x)),
                                         std::max(std::abs(q(a.y) - q(b.y)),
                                                  std::abs(q(a.z) - q(b.z)))));
                    }
                    std::printf("           %3d steps: %d\n", steps, worst);
                    if (steps == kSkySteps)
                        check(worst <= 1,
                              "sky: kSkySteps lands within one 8-bit step of a converged march");
                    if (steps == 8)
                        check(worst > 1,
                              "sky: ... while 8 steps is visibly worse, so the choice is not idle");
                }
            }

            {
                const Vec3 dir = normalize(Vec3{ 1.0f, 0.02f, 0.0f });
                const Vec3 sun = normalize(Vec3{ 0.4f, 0.8f, 0.2f });

                {
                    Vec3 t{ 0.5f, 0.5f, 0.5f };
                    skyRadiance(up, up, float(2400000.0), sun, lut, 1e9f, kSkySteps, &t);
                    check(t.x == 1.0f && t.y == 1.0f && t.z == 1.0f,
                          "aerial: an empty march leaves the transmittance at white");
                }

                {
                    Vec3 t{ 0.0f, 0.0f, 0.0f };
                    const Vec3 ins = skyRadiance(dir, up, 2.0f, sun, lut, 0.01f,
                                                 kSkySteps, &t);
                    check(t.x > 0.9999f && t.z > 0.9999f,
                          "aerial: at zero distance nothing is absorbed");
                    check(ins.x + ins.y + ins.z < 1e-4f,
                          "aerial: ... and nothing is scattered in either");
                }

                {
                    bool mono = true;
                    Vec3 prev{ 1.0f, 1.0f, 1.0f };
                    for (int i = 1; i <= 40; ++i) {
                        Vec3 t;
                        skyRadiance(dir, up, 2.0f, sun, lut, i * 5000.0f, kSkySteps, &t);
                        if (t.x > prev.x + 1e-6f || t.y > prev.y + 1e-6f ||
                            t.z > prev.z + 1e-6f) mono = false;
                        prev = t;
                    }
                    std::printf("         transmittance over 200 km: "
                                "R %.3f G %.3f B %.3f\n", prev.x, prev.y, prev.z);
                    check(mono, "aerial: transmittance falls monotonically with distance");
                    check(prev.z < prev.y && prev.y < prev.x,
                          "aerial: blue is extinguished first, so haze takes the sky's hue");
                    check(prev.z < 0.2f,
                          "aerial: 200 km of low air removes most of the blue");
                }
            }

            {
                const float pix = 2.0f * std::tan(deg2rad(kFovYDegrees) * 0.5f) /
                                  float(kWindowHeight);
                const Vec3  sunUpward{ 0.0f, 1.0f, 0.0f };

                const Vec3 off  = sunDiscRadiance(normalize(Vec3{ 0.2f, 1.0f, 0.0f }),
                                                  up, 2.0f, sunUpward, lut, pix);
                const Vec3 on   = sunDiscRadiance(sunUpward, up, 2.0f, sunUpward, lut, pix);
                const Vec3 rim  = sunDiscRadiance(
                    normalize(Vec3{ std::sin(kSunAngularRadius), std::cos(kSunAngularRadius),
                                    0.0f }), up, 2.0f, sunUpward, lut, pix);
                check(off.x == 0.0f && off.y == 0.0f && off.z == 0.0f,
                      "sun: a ray well off the disc sees nothing");
                check(on.x > 0.0f && on.z > 0.0f, "sun: a ray down the middle sees the disc");
                check(rim.x > 0.4f * on.x && rim.x < 0.6f * on.x,
                      "sun: exactly on the rim the soft edge is at half");

                {
                    const Vec3 down{ 0.0f, -1.0f, 0.0f };
                    const Vec3 night = sunDiscRadiance(down, up, 2.0f, down, lut, pix);
                    check(night.x == 0.0f && night.y == 0.0f && night.z == 0.0f,
                          "sun: with the sun below the horizon there is no disc");
                }

                {
                    const Vec3 lowSun = normalize(Vec3{ 1.0f, 0.02f, 0.0f });
                    const Vec3 low = sunDiscRadiance(lowSun, up, 2.0f, lowSun, lut, pix);
                    std::printf("         sun disc: high R %.0f G %.0f B %.0f | "
                                "low R %.1f G %.1f B %.2f\n",
                                on.x, on.y, on.z, low.x, low.y, low.z);
                    check(low.x / std::max(low.z, 1e-9f) > on.x / on.z,
                          "sun: the setting disc is redder than the high one");
                }

                check(on.x <= kSunDiscRadiance && kSunDiscRadiance < 65504.0f,
                      "sun: the disc radiance is clamped inside F16's range");
                check(acesTonemap(on.x * kExposure) > 0.99f,
                      "sun: and still saturates to white after exposure and ACES");
            }

            {
                float peak = 0.0f;
                for (int i = 0; i <= 40; ++i) {
                    const float c = -1.0f + 2.0f * i / 40.0f;
                    const Vec3 d = normalize(Vec3{ std::sqrt(std::max(1.0f - c * c, 0.0f)),
                                                   c, 0.0f });
                    const Vec3 s = skyRadiance(d, up, 2.0f, sunUp, lut);
                    peak = std::max(peak, std::max(s.x, std::max(s.y, s.z)));
                }
                std::printf("         brightest sky radiance over the hemisphere: %.3f "
                            "(sun irradiance %.1f)\n", peak, kSunIrradiance);
                check(peak < kSunIrradiance,
                      "sky: no direction is brighter than the sun's own irradiance");
            }
        }

        {

            const int N = 20000;
            double sumR = 0.0, sumM = 0.0;
            for (int i = 0; i < N; ++i) {
                const float c = -1.0f + (i + 0.5f) * (2.0f / N);
                sumR += rayleighPhase(c);
                sumM += miePhase(c);
            }
            const double dOmega = (2.0 / N) * 2.0 * double(kPi);
            check(std::fabs(sumR * dOmega - 1.0) < 0.001,
                  "atmo: the Rayleigh phase function integrates to 1 over the sphere");
            check(std::fabs(sumM * dOmega - 1.0) < 0.01,
                  "atmo: the Henyey-Greenstein phase function integrates to 1");
            check(approx(rayleighPhase(1.0f), rayleighPhase(-1.0f), 1e-6f),
                  "atmo: Rayleigh scatters equally fore and aft");
            check(miePhase(1.0f) > 20.0f * miePhase(-1.0f),
                  "atmo: Mie throws light forward -- that is the halo around the sun");
        }

        {
            check(approx(acesTonemap(0.0f), 0.0f, 1e-6f), "atmo: ACES maps black to black");

            bool mono = true;
            float prev = -1.0f;
            for (int i = 0; i <= 2000; ++i) {
                const float y = acesTonemap(i * 0.01f);
                if (y < prev) mono = false;
                prev = y;
            }
            check(mono, "atmo: ACES is monotone from 0 to 20");
            check(acesTonemap(1e6f) > 0.99f && acesTonemap(1e6f) <= 1.0f,
                  "atmo: ACES saturates to 1 and never exceeds it (the sun disc)");

            const float fullLit = (1.0f / kPi) * kSunIrradiance;
            std::printf("         white terrain: full sun %.3f -> %.3f, ambient only %.3f\n",
                        fullLit, acesTonemap(fullLit * kExposure),
                        acesTonemap(fullLit * kAmbient * kExposure));
            check(approx(acesTonemap(fullLit * kExposure), 0.74f, 0.02f),
                  "atmo: fully lit white terrain tonemaps to ~0.74, short of clipping");
            check(acesTonemap(fullLit * kAmbient * kExposure) < 0.2f,
                  "atmo: ambient-only terrain stays dark");
            check(fullLit < 65504.0f && kSunDiscRadiance < 65504.0f,
                  "atmo: every radiance the scene can produce fits in F16");
        }
    }

    {
        const float fov  = deg2rad(kFovYDegrees);
        const float asp  = float(kWindowWidth) / float(kWindowHeight);
        const float tanH = std::tan(fov * 0.5f);
        const Basis b    = makeBasis(0.7f, -0.35f);
        const Mat4  vp   = planet::perspectiveInfReverseZ(fov, asp, kNearPlane) *
                           planet::lookAt(Vec3{ 0.0f, 0.0f, 0.0f }, b.forward, b.up);

        {
            const Vec3 s = normalize(cross(b.forward, b.up));
            const Vec3 u = cross(s, b.forward);
            check(length(s + b.right) < 1e-5f,
                  "raycast: lookAt's right is the NEGATIVE of makeBasis's right");
            check(length(u - b.up) < 1e-5f,
                  "raycast: lookAt's up matches makeBasis's up");
        }

        const Vec3 camR = normalize(cross(b.forward, b.up)) * (tanH * asp);
        const Vec3 camU = b.up * tanH;

        float worstDir = 0.0f, worstDist = 0.0f;
        float worstAt  = 0.0f;
        int   tested   = 0;
        for (int i = 0; i < 400; ++i) {

            const float u    = -0.95f + 1.90f * ((i * 37) % 101) / 100.0f;
            const float v    = -0.95f + 1.90f * ((i * 53) % 101) / 100.0f;
            const float dist = 0.2f * std::pow(10.0f, 8.0f * ((i * 17) % 101) / 100.0f);
            const Vec3  dir  = normalize(b.forward + camR * u + camU * v);
            const Vec3  P    = dir * dist;

            const planet::Vec4 clip = planet::transform(vp, P);
            if (clip.w <= 0.0f) continue;
            const float ndcX  = clip.x / clip.w;
            const float ndcY  = clip.y / clip.w;
            const float depth = clip.z / clip.w;
            ++tested;

            const Vec3  rdir   = normalize(b.forward + camR * ndcX - camU * ndcY);
            const float rayLen = (kNearPlane / depth) / dot(rdir, b.forward);

            worstDir = std::max(worstDir, length(rdir - dir));
            const float rel = std::fabs(rayLen - dist) / dist;
            if (rel > worstDist) { worstDist = rel; worstAt = dist; }
        }
        std::printf("         %d points: ray dir off by %.2e, distance by %.2e "
                    "(worst at %.0f m)\n", tested, worstDir, worstDist, worstAt);
        check(tested > 300, "raycast: the sample points are actually in front of the camera");
        check(worstDir < 1e-5f,
              "raycast: the reconstructed ray direction matches the projected one");
        check(worstDist < 1e-3f,
              "raycast: near/depth recovers the distance over eight decades");

        {
            const Vec3 above = normalize(b.forward + b.up * 0.3f) * 100.0f;
            const planet::Vec4 c = planet::transform(vp, above);
            check(c.w > 0.0f && (c.y / c.w) < 0.0f,
                  "raycast: world-up maps to screen-up (the projection's y-flip)");
        }

        {
            const Vec3 veryFar = b.forward * 1.0e9f;
            const planet::Vec4 c = planet::transform(vp, veryFar);
            const float d = c.z / c.w;
            check(d > 0.0f && d < 1e-6f,
                  "raycast: even 1e9 m keeps depth above zero, so 0 means sky");
        }
    }

    {
        const Vec3 eye = kEyeOffset;

        {
            const Vec3 lo{ -1.0f, -1.0f, 1.0f }, hi{ 1.0f, 1.0f, 3.0f };
            float t = -1.0f;
            check(rayAabb(Vec3{ 0, 0, 0 }, Vec3{ 0, 0, 1 }, lo, hi, t) && approx(t, 1.0f),
                  "cockpit: a ray straight at a box reports the near face");
            check(!rayAabb(Vec3{ 0, 0, 0 }, Vec3{ 0, 0, -1 }, lo, hi, t),
                  "cockpit: a box behind the ray is not a hit");
            check(!rayAabb(Vec3{ 0, 3.0f, 0 }, Vec3{ 0, 0, 1 }, lo, hi, t),
                  "cockpit: a ray passing over the box misses");

            check(rayAabb(Vec3{ 0, 0, 0 }, Vec3{ 0, 0, 1 }, Vec3{ -1, 0, 1 },
                          Vec3{ 1, 1, 3 }, t),
                  "cockpit: a ray exactly on a slab boundary still hits");
            check(!rayAabb(Vec3{ 0, 2.0f, 0 }, Vec3{ 0, 0, 1 }, Vec3{ -1, 0, 1 },
                           Vec3{ 1, 1, 3 }, t),
                  "cockpit: ... and one parallel but outside the slab does not");

            check(rayAabb(Vec3{ 0, 0, 2.0f }, Vec3{ 0, 0, 1 }, lo, hi, t) &&
                  approx(t, 0.0f),
                  "cockpit: a ray starting inside the box hits at zero");
        }

        {
            bool allHit = true, allReachable = true;
            float furthest = 0.0f;
            for (int i = 0; i < kSwitchCount; ++i) {
                const Vec3 dir = normalize(kSwitches[i].centre - eye);
                const Pick p = pickSwitch(eye, dir);
                if (!p.hit() || p.index != i) allHit = false;
                const float d = length(kSwitches[i].centre - eye);
                furthest = std::max(furthest, d);
                if (d > kSwitchReach) allReachable = false;
            }
            std::printf("         %d switches, furthest %.2f m from the eye "
                        "(reach %.2f m)\n", int(kSwitchCount), furthest, kSwitchReach);
            check(allHit, "cockpit: looking straight at a switch picks that switch");
            check(allReachable, "cockpit: every switch is within reach of the seat");
        }

        {
            int spurious = 0;
            for (int i = 0; i < kSwitchCount; ++i) {
                const Vec3 c = kSwitches[i].centre;
                const Vec3 off{ c.x, c.y + 3.0f * kSwitchHalf.y, c.z };
                if (pickSwitch(eye, normalize(off - eye)).hit()) ++spurious;
            }
            check(spurious == 0, "cockpit: aiming just above a switch picks nothing");
        }

        {
            const Vec3 dir = normalize(kSwitches[kSwGear].centre - eye);
            check(pickSwitch(eye, dir, 0.5f).hit() == false,
                  "cockpit: a switch beyond the reach limit cannot be picked");
            check(pickSwitch(eye, dir, kSwitchReach).hit(),
                  "cockpit: ... and is picked again once the limit allows it");
        }

        {
            const Vec3 o{ 0, 0, 0 }, d{ 0, 0, 1 };
            float tNear = 0.0f, tFar = 0.0f;
            check(rayAabb(o, d, Vec3{ -1, -1, 1 }, Vec3{ 1, 1, 2 }, tNear) &&
                  rayAabb(o, d, Vec3{ -1, -1, 3 }, Vec3{ 1, 1, 4 }, tFar) &&
                  tNear < tFar,
                  "cockpit: of two boxes on one ray the nearer is nearer");
        }

        {
            bool onPanel = true, clearOfCoaming = true, insideX = true;
            for (int i = 0; i < kSwitchCount; ++i) {
                const Vec3 c = kSwitches[i].centre;
                if (std::fabs((c.y - kSwitchHalf.y) - kConsoleTopY) > 1e-5f) onPanel = false;
                if (c.z - kSwitchHalf.z < kConsoleZNear ||
                    c.z + kSwitchHalf.z > kCoamingZNear) clearOfCoaming = false;
                if (std::fabs(c.x) + kSwitchHalf.x > kConsoleHalfX) insideX = false;
            }
            check(onPanel, "cockpit: every switch sits exactly on the console surface");
            check(clearOfCoaming,
                  "cockpit: every switch is on the strip the coaming leaves exposed");
            check(insideX, "cockpit: no switch overhangs the console's edge");

            bool disjoint = true;
            for (int i = 0; i < kSwitchCount; ++i)
                for (int j = i + 1; j < kSwitchCount; ++j) {
                    const Vec3 a = kSwitches[i].centre, b = kSwitches[j].centre;
                    if (std::fabs(a.x - b.x) < 2.0f * kSwitchHalf.x &&
                        std::fabs(a.y - b.y) < 2.0f * kSwitchHalf.y &&
                        std::fabs(a.z - b.z) < 2.0f * kSwitchHalf.z) disjoint = false;
                }
            check(disjoint, "cockpit: no two switches overlap, so picking is unambiguous");

            bool lampClear = std::fabs(kLampCentre.x) + kLampHalf.x <= kConsoleHalfX &&
                             kLampCentre.z - kLampHalf.z >= kConsoleZNear &&
                             kLampCentre.z + kLampHalf.z <= kCoamingZNear;
            for (int i = 0; i < kSwitchCount; ++i) {
                const Vec3 c = kSwitches[i].centre;
                if (std::fabs(c.x - kLampCentre.x) < kSwitchHalf.x + kLampHalf.x &&
                    std::fabs(c.z - kLampCentre.z) < kSwitchHalf.z + kLampHalf.z)
                    lampClear = false;
            }
            check(lampClear, "cockpit: the lamp is on the panel and clear of every switch");
            check(!pickSwitch(kEyeOffset,
                              normalize(kLampCentre - kEyeOffset)).hit(),
                  "cockpit: looking straight at the lamp picks nothing");
        }

        {
            ShipToggles t;
            const bool gear0 = t.gearDown, assist0 = t.assistOn;

            activateSwitch(t, kSwGear);
            check(t.gearDown != gear0 && t.assistOn == assist0,
                  "cockpit: the gear switch moves the gear and nothing else");

            activateSwitch(t, kSwAssist);
            check(t.assistOn != assist0,
                  "cockpit: the assist switch moves the assist");

            activateSwitch(t, kSwGear);
            check(t.gearDown == gear0,
                  "cockpit: throwing a toggle twice returns it to where it was");

            check(!t.jumpRequested, "cockpit: nothing else has asked for a jump");
            {
                const ShipToggles before = t;
                activateSwitch(t, kSwJump);
                check(t.jumpRequested &&
                      t.gearDown == before.gearDown &&
                      t.assistOn == before.assistOn &&
                      t.throttle == before.throttle,
                      "cockpit: the jump switch asks for a jump and touches nothing else");
            }

            t.throttle = 0.8f;
            activateSwitch(t, kSwCutThrottle);
            check(t.throttle == 0.0f, "cockpit: the throttle button cuts the throttle");
            activateSwitch(t, kSwCutThrottle);
            check(t.throttle == 0.0f,
                  "cockpit: ... and pressing it again is harmless, not a latch");
        }

        {
            ShipToggles t;
            t.gearDown = false;
            const space::Quat off = switchRotation(t, kSwGear);
            t.gearDown = true;
            const space::Quat on = switchRotation(t, kSwGear);
            check(space::angleBetween(off, on) > deg2rad(25.0f),
                  "cockpit: a toggle visibly leans the other way when thrown");

            const space::Quat btn = switchRotation(t, kSwCutThrottle);
            check(approx(btn.w, 1.0f, 1e-6f),
                  "cockpit: a momentary button sits level in both states");
        }
    }

    {
        Destination dest[kDestinationCount];

        auto flat = [](const DVec3& p) {
            struct H { double radius; DVec3 normal; } h;
            h.radius = kPlanetRadius;
            h.normal = space::normalize(p);
            return h;
        };
        buildDestinations(dest, flat);

        {
            bool lifted = true, named = true;
            for (int i = 0; i < kDestinationCount; ++i) {
                const double alt = space::length(dest[i].pos) - kPlanetRadius;
                if (std::fabs(alt - kArrivalStandoff) > 1.0) lifted = false;
                if (dest[i].name == nullptr) named = false;
            }
            check(lifted, "drive: every destination sits at the arrival standoff");
            check(named, "drive: every destination has a name");

            Destination again[kDestinationCount];
            buildDestinations(again, flat);
            bool same = true;
            for (int i = 0; i < kDestinationCount; ++i)
                if (space::length(again[i].pos - dest[i].pos) > 1e-9) same = false;
            check(same, "drive: the destination table is deterministic");
        }

        {
            int blocked = 0, pairs = 0;
            double worstSep = 0.0;
            for (int i = 0; i < kDestinationCount; ++i)
                for (int j = i + 1; j < kDestinationCount; ++j) {
                    ++pairs;
                    if (!routeIsClear(dest[i].pos, dest[j].pos)) ++blocked;
                    const double c = space::dot(space::normalize(dest[i].pos),
                                                space::normalize(dest[j].pos));
                    worstSep = std::max(worstSep,
                                        std::acos(std::max(-1.0, std::min(1.0, c))));
                }
            std::printf("         %d site pairs, widest separation %.1f deg, "
                        "%d blocked on a direct line\n",
                        pairs, worstSep * 180.0 / kPi, blocked);
            check(blocked == pairs,
                  "drive: NO direct surface-to-surface route clears the planet");

            int viaTransfer = 0;
            for (int i = 0; i < kDestinationCount; ++i)
                for (int j = i + 1; j < kDestinationCount; ++j) {
                    const double rt = transferRadiusFor(dest[i].pos, dest[j].pos);
                    if (routeIsClear(transferPointOver(dest[i].pos, rt),
                                     transferPointOver(dest[j].pos, rt))) ++viaTransfer;
                }
            check(viaTransfer == pairs,
                  "drive: every crossing clears at its own transfer altitude");
        }

        {
            const DVec3 up{ 0.0, kPlanetRadius + 10000.0, 0.0 };
            check(routeIsClear(up, up * 2.0), "drive: straight up is always clear");
            check(!routeIsClear(up, DVec3{ 0.0, -(kPlanetRadius + 10000.0), 0.0 }),
                  "drive: straight through the planet is blocked");
            check(!routeIsClear(DVec3{ 0.0, kPlanetRadius * 0.5, 0.0 }, up),
                  "drive: a start inside the planet is refused");

            const DVec3 high{ 0.0, kPlanetRadius * 3.0, 0.0 };
            check(routeIsClear(high, DVec3{ 0.0, kPlanetRadius * 2.0, 0.0 }),
                  "drive: the planet beyond the destination does not block");
        }

        {
            ShipState s;
            s.pos = dest[0].pos;
            const DVec3 to = dest[3].pos;

            QuantumDrive d;
            s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                     space::toF(space::normalize(DVec3{ 1, 0, 0 })));
            check(engage(d, s, to) == EngageResult::Misaligned,
                  "drive: a jump refuses while the nose points elsewhere");

            s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                     space::toF(space::normalize(to - s.pos)));
            check(engage(d, s, to) == EngageResult::Ok,
                  "drive: aimed at the destination, the jump is accepted");
            check(engage(d, s, to) == EngageResult::AlreadyBusy,
                  "drive: a second jump while busy is refused");

            QuantumDrive d2;
            const DVec3 near = s.pos + space::normalize(to - s.pos) * 1000.0;
            s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                     space::toF(space::normalize(near - s.pos)));
            check(engage(d2, s, near) == EngageResult::TooClose,
                  "drive: a destination inside the standoff is refused");
        }

        {
            std::printf("         burn profile (point mass, dt = 1/120 s):\n");
            bool allArrived = true, allStopped = true, allOnTarget = true;

            for (int caseIdx = 0; caseIdx < kDestinationCount; ++caseIdx) {
                ShipState s;
                s.pos = dest[0].pos;
                const DVec3 to = dest[(caseIdx + 1) % kDestinationCount].pos;
                if (space::length(to - s.pos) < 2.0 * kArrivalStandoff) continue;

                s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                         space::toF(space::normalize(to - s.pos)));
                QuantumDrive d;
                if (engage(d, s, to) != EngageResult::Ok) { allArrived = false; continue; }

                const double dt = 1.0 / 120.0;
                double peak = 0.0, t = 0.0;
                for (int step = 0; step < 200000 && d.busy(); ++step) {
                    const DVec3 a = update(d, s, dt);
                    s.vel = s.vel + a * dt;
                    s.pos = s.pos + s.vel * dt;
                    peak = std::max(peak, space::length(s.vel));
                    t += dt;
                }
                const double miss  = space::length(s.pos - to);
                const double speed = space::length(s.vel);
                std::printf("           %-8s %6.0f km  %5.1f s  peak %6.1f km/s  "
                            "miss %6.1f m  v %5.2f m/s\n",
                            dest[(caseIdx + 1) % kDestinationCount].name,
                            space::length(to - dest[0].pos) / 1000.0, t,
                            peak / 1000.0, miss, speed);
                if (d.state != DriveState::Arrived) allArrived = false;
                if (speed > 2.0)   allStopped  = false;
                if (miss  > 500.0) allOnTarget = false;
            }
            check(allArrived, "drive: every jump reaches Arrived");
            check(allStopped, "drive: ... having come to a stop, not still moving");
            check(allOnTarget, "drive: ... within 500 m of the destination");
        }

        {

            {
                ShipState a, b;
                a.pos = b.pos = DVec3{ 0.0, kPlanetRadius + 20000.0, 0.0 };
                a.vel = b.vel = DVec3{ 130.0, -12.0, 45.0 };
                a.orient = b.orient = fromAxisAngle(normalize(Vec3{ 1, 2, 3 }), 0.4f);

                ShipControls plain;
                plain.assist = true;
                plain.throttle = 0.6f;
                plain.pitch = 0.2f;

                ShipControls withDrive = plain;
                auto sphere = [](const DVec3& p) { return flatGround(p, kPlanetRadius); };
                for (int i = 0; i < 600; ++i) {
                    step(a, plain,     sphere, 1.0 / 120.0);
                    step(b, withDrive, sphere, 1.0 / 120.0);
                }
                check(length(a.pos - b.pos) == 0.0 && length(a.vel - b.vel) == 0.0,
                      "drive: an idle drive leaves ordinary flight bit-identical");
            }

            {
                Destination d6[kDestinationCount];
                auto flat2 = [](const DVec3& p) {
                    struct H { double radius; DVec3 normal; } h;
                    h.radius = kPlanetRadius;
                    h.normal = space::normalize(p);
                    return h;
                };
                buildDestinations(d6, flat2);

                ShipState s;
                s.pos = d6[0].pos;
                const DVec3 to = d6[2].pos;
                s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                         space::toF(normalize(to - s.pos)));

                auto sphere2 = [](const DVec3& p) { return flatGround(p, kPlanetRadius); };
                QuantumDrive drive;
                check(engage(drive, s, to) == EngageResult::Ok,
                      "drive: the jump engages from a real ship state");

                const double dt = 1.0 / 120.0;
                double peak = 0.0, t = 0.0;
                for (int i = 0; i < 200000 && drive.busy(); ++i) {
                    ShipControls ctl;
                    ctl.assist        = true;
                    ctl.quantumAccel  = update(drive, s, dt);
                    ctl.quantumTravel = drive.busy();
                    step(s, ctl, sphere2, dt);
                    peak = std::max(peak, length(s.vel));
                    t += dt;
                }

                check(length(s.vel) < 20.0,
                      "drive: nothing is spraying the ship sideways during a jump");
                std::printf("         through step(): %.1f s, peak %.1f km/s, "
                            "miss %.1f m, v %.2f m/s\n",
                            t, peak / 1000.0, length(s.pos - to), length(s.vel));
                check(peak > 100.0 * kNavSpeed,
                      "drive: the speed limiter does not cap a jump");
                check(drive.state == DriveState::Arrived,
                      "drive: the jump completes inside the real integrator");
                check(length(s.pos - to) < 200.0,
                      "drive: ... and arrives within 200 m through gravity and all");
            }

            {
                ShipState s;
                s.pos = DVec3{ 0.0, kPlanetRadius + 3000000.0, 0.0 };
                s.vel = DVec3{ 0.0, 0.0, 150000.0 };

                ShipControls ctl;
                ctl.assist        = true;
                ctl.quantumTravel = true;
                auto sphere3 = [](const DVec3& p) { return flatGround(p, kPlanetRadius); };
                step(s, ctl, sphere3, 1.0 / 120.0);
                check(length(s.vel) > 100000.0,
                      "drive: coasting at speed survives the limiter");

                ShipState s2 = s;
                ShipControls off = ctl;
                off.quantumTravel = false;
                step(s2, off, sphere3, 1.0 / 120.0);
                check(length(s2.vel) <= kScmSpeed + 1.0,
                      "drive: ... and is clamped again the moment the jump ends");
            }
        }

        {
            QuantumDrive d;
            d.braking = true;
            d.state   = DriveState::Crossing;
            d.axis    = DVec3{ 0, 1, 0 };
            d.wpOver  = DVec3{ 0, kPlanetRadius * 4.0, 0 };
            ShipState s;
            s.pos = DVec3{ 0, kPlanetRadius * 2.0, 0 };
            s.vel = DVec3{ 0, 1000.0, 0 };
            for (int i = 0; i < 50; ++i) update(d, s, 1.0 / 120.0);
            check(d.braking, "drive: once braking, the leg never un-brakes");
        }

        {
            ShipState s;
            s.pos = transferPointOver(dest[0].pos, kPlanetRadius + 400000.0);
            const DVec3 to = dest[0].pos;
            s.orient = space::fromTo(Vec3{ 0, 0, 1 },
                                     space::toF(space::normalize(to - s.pos)));
            QuantumDrive d;
            check(engage(d, s, to) == EngageResult::Ok,
                  "drive: a purely radial descent is accepted");
            const double dt = 1.0 / 120.0;
            for (int i = 0; i < 200000 && d.busy(); ++i) {
                const DVec3 a = update(d, s, dt);
                s.vel = s.vel + a * dt;
                s.pos = s.pos + s.vel * dt;
            }
            check(d.state == DriveState::Arrived && space::length(s.pos - to) < 500.0,
                  "drive: ... and lands on target with the ascent and crossing skipped");
        }
    }

    std::printf("\n%s (%d failure%s)\n",
                g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
