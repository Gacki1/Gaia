#include "platform/Win32Window.h"
#include "render/VulkanRenderer.h"
#include "render/PlanetRenderer.h"
#include "render/ShipRenderer.h"
#include "render/ScatterRenderer.h"
#include "ship/Flight.h"
#include "world/TerrainGround.h"
#include "ship/Camera.h"
#include "world/Atmosphere.h"
#include "ship/Cockpit.h"
#include "ship/QuantumDrive.h"
#include "world/PlanetConfig.h"
#include "world/CubeSphere.h"
#include "math/SpaceMath.h"
#include "math/Mat4.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cmath>
#include <algorithm>
#include <fstream>
#include "render/GroundTextureSet.h"
#include "render/OverlayUi.h"
#include "render/PlanetEditor.h"
#include "world/PlanetIO.h"
#include <vector>

using namespace planet;
using space::Vec3;

namespace {

void initConsole() {
    HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    const bool haveOut = out != nullptr && out != INVALID_HANDLE_VALUE;
    if (!haveOut && AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE* f = nullptr;
        freopen_s(&f, "CONOUT$", "w", stdout);
        freopen_s(&f, "CONOUT$", "w", stderr);
    }
}

bool hasArg(const char* name) {
    for (int i = 1; i < __argc; ++i)
        if (std::strcmp(__argv[i], name) == 0) return true;
    return false;
}

int intArg(const char* name, int fallback) {
    for (int i = 1; i + 1 < __argc; ++i)
        if (std::strcmp(__argv[i], name) == 0) return std::atoi(__argv[i + 1]);
    return fallback;
}

double dblArg(const char* name, double fallback) {
    for (int i = 1; i + 1 < __argc; ++i)
        if (std::strcmp(__argv[i], name) == 0) return std::atof(__argv[i + 1]);
    return fallback;
}

const char* strArg(const char* name) {
    for (int i = 1; i + 1 < __argc; ++i)
        if (std::strcmp(__argv[i], name) == 0) return __argv[i + 1];
    return nullptr;
}

void scriptFlight(ShipControls& ctl, bool& gearDown, float& throttle,
                  const ShipState& ship, const TerrainGround& ground, int frame) {
    ctl = ShipControls{};
    ctl.assist   = true;
    ctl.gearDown = true;
    gearDown     = true;
    throttle     = 0.0f;

    if (frame < 150) { ctl.roll = 0.7f; return; }

    bool anyContact = false;
    for (int i = 0; i < 4; ++i) anyContact = anyContact || ship.legContact[i];
    if (anyContact) return;

    const space::DVec3 upWorld = space::normalize(ship.pos);
    const Vec3 u = toBody(ship.orient, space::toF(upWorld));
    const float k = 2.0f;
    ctl.roll  = std::max(-1.0f, std::min(1.0f, -k * u.x));
    ctl.pitch = std::max(-1.0f, std::min(1.0f,  k * u.z));

    if (u.y < 0.995f) return;

    const double agl = space::length(ship.pos) - ground(ship.pos).radius;
    ctl.strafeY = (agl > 100.0) ? -0.25f
                : (agl >  30.0) ? -0.08f
                : (agl >   8.0) ? -0.03f
                                : -0.015f;
}

void scriptTravel(ShipControls& ctl, QuantumDrive& drive, const Destination& dest,
                  const ShipState& ship, int frame) {
    ctl = ShipControls{};
    ctl.assist = true;

    if (drive.busy() || drive.state == DriveState::Arrived) return;

    const space::DVec3 delta = dest.pos - ship.pos;
    const double d = space::length(delta);
    if (d < 1.0) return;
    const Vec3 w = toBody(ship.orient, space::toF(delta * (1.0 / d)));

    const float k = 2.0f;
    ctl.yaw   = std::max(-1.0f, std::min(1.0f,  k * w.x));
    ctl.pitch = std::max(-1.0f, std::min(1.0f, -k * w.y));

    if (frame > 120 && alignmentError(ship, dest.pos) <= kAlignTolerance)
        engage(drive, ship, dest.pos);
}

bool dumpLut(const char* path) {
    const planet::atmo::TransmittanceLut lut = planet::atmo::buildTransmittanceLut();
    const int W = planet::kWindowWidth, H = planet::kWindowHeight;
    std::vector<unsigned char> rgb(size_t(W) * H * 3);

    auto encodeSrgb = [](float c) -> unsigned char {
        c = std::max(0.0f, std::min(1.0f, c));
        const float e = (c <= 0.0031308f) ? c * 12.92f
                                          : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
        return static_cast<unsigned char>(std::lround(e * 255.0f));
    };

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const space::Vec3 t = planet::atmo::sampleLutUV(lut, (x + 0.5f) / W,
                                                                 (y + 0.5f) / H);
            unsigned char* px = &rgb[(size_t(y) * W + x) * 3];
            px[0] = encodeSrgb(t.x); px[1] = encodeSrgb(t.y); px[2] = encodeSrgb(t.z);
        }
    }

    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return false;
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(rgb.data()), std::streamsize(rgb.size()));
    return f.good();
}

double nowSeconds() {
    static LARGE_INTEGER freq = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return f; }();
    LARGE_INTEGER c; QueryPerformanceCounter(&c);
    return static_cast<double>(c.QuadPart) / static_cast<double>(freq.QuadPart);
}

bool dumpLand(const char* path, const space::DVec3& centre, double span,
              bool showRidge) {
    const int W = planet::kWindowWidth, H = planet::kWindowHeight;
    std::vector<unsigned char> rgb(size_t(W) * H * 3, 0);
    planet::World world;

    const space::DVec3 fwd = space::normalize(centre);
    const space::DVec3 ref = (std::fabs(fwd.y) < 0.9) ? space::DVec3{ 0, 1, 0 }
                                                      : space::DVec3{ 1, 0, 0 };
    const space::DVec3 rgt = space::normalize(space::cross(ref, fwd));
    const space::DVec3 upv = space::cross(fwd, rgt);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {

            const double a = ((x + 0.5) / W * 2.0 - 1.0) * span * (double(W) / H);
            const double b = (1.0 - (y + 0.5) / H * 2.0) * span;
            const double q = a * a + b * b;
            if (q >= 1.0) continue;
            const space::DVec3 d = space::normalize(
                fwd * std::sqrt(1.0 - q) + rgt * a + upv * b);

            float e;
            if (showRidge) {
                const planet::Noise::DualFbm f = world.noise.fbmDual(
                    d, planet::kNoiseOctaves, planet::kNoiseFrequency,
                    planet::kNoiseLacunarity, planet::kNoiseGain,
                    planet::kRangeOctaves);
                e = (f.plain - planet::kSeaLevelNoise) < 0.0f
                        ? -0.5f
                        : std::pow(f.ridged, planet::kRangeSharpness);
            } else {
                e = planet::sampleLandE(world, d);
            }
            unsigned char* px = &rgb[(size_t(y) * W + x) * 3];

            if (std::fabs(e) < 0.004f) { px[0] = 255; px[1] = 40; px[2] = 40; continue; }
            if (e > 0.0f) {

                const float t = std::min(1.0f, e / 0.35f);
                const unsigned char v = static_cast<unsigned char>(
                    30 + std::lround(std::sqrt(t) * 225.0f));
                px[0] = v; px[1] = v; px[2] = static_cast<unsigned char>(v * 0.8);
            } else {
                px[2] = static_cast<unsigned char>(
                    70 + std::lround(std::min(1.0f, -e) * 100.0f));
                px[1] = px[2] / 3;
            }
        }
    }
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) return false;
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(rgb.data()), std::streamsize(rgb.size()));
    return f.good();
}

int run() {
    initConsole();

    if (const char* landPath = strArg("--dump-land")) {
        space::DVec3 c{ 0.3, 0.5, 0.8 };
        if (const char* s = strArg("--land-dir"))
            std::sscanf(s, "%lf,%lf,%lf", &c.x, &c.y, &c.z);
        const bool ok = dumpLand(landPath, c, dblArg("--land-span", 1.0),
                                 hasArg("--land-ridge"));
        std::printf("%s land map -> %s\n", ok ? "Wrote" : "FAILED to write", landPath);
        return ok ? 0 : 1;
    }

    if (const char* gPath = strArg("--dump-ground")) {
        const planet::GroundSet gs = planet::loadGroundSet(
            uint32_t(intArg("--dump-ground-size", 512)));
        bool ok = true;
        for (uint32_t m = 0; m < gs.layers; ++m) {
            for (int pass = 0; pass < 2; ++pass) {
                char name[512];
                std::snprintf(name, sizeof(name), "%s_%u_%s.ppm", gPath, m,
                              pass == 0 ? "color" : "normal");
                std::ofstream f(name, std::ios::binary);
                if (!f) { ok = false; continue; }
                f << "P6\n" << gs.size << " " << gs.size << "\n255\n";
                for (size_t i = 0; i < size_t(gs.size) * gs.size; ++i) {
                    unsigned char px[3];
                    if (pass == 0) {
                        const uint8_t* c =
                            &gs.rgba[(size_t(m) * gs.size * gs.size + i) * 4];
                        px[0] = c[0]; px[1] = c[1]; px[2] = c[2];
                    } else {
                        const uint8_t* n =
                            &gs.normalRG[(size_t(m) * gs.size * gs.size + i) * 2];

                        const double x = double(n[0]) / 127.5 - 1.0;
                        const double y = double(n[1]) / 127.5 - 1.0;
                        const double z = std::sqrt(std::max(0.0, 1.0 - x * x - y * y));
                        px[0] = n[0]; px[1] = n[1];
                        px[2] = (unsigned char)(z * 255.0 + 0.5);
                    }
                    f.write(reinterpret_cast<const char*>(px), 3);
                }
            }

            char name[512];
            std::snprintf(name, sizeof(name), "%s_%u_ao.ppm", gPath, m);
            std::ofstream f(name, std::ios::binary);
            if (!f) { ok = false; continue; }
            f << "P6\n" << gs.size << " " << gs.size << "\n255\n";
            for (size_t i = 0; i < size_t(gs.size) * gs.size; ++i) {
                const uint8_t a = gs.rgba[(size_t(m) * gs.size * gs.size + i) * 4 + 3];
                const unsigned char px[3] = { a, a, a };
                f.write(reinterpret_cast<const char*>(px), 3);
            }
        }
        std::printf("%s ground array -> %s_<layer>_{color,normal,ao}.ppm\n",
                    ok ? "Wrote" : "FAILED to write", gPath);
        return ok ? 0 : 1;
    }

    if (const char* lutPath = strArg("--dump-lut")) {
        const bool ok = dumpLut(lutPath);
        std::printf("%s transmittance LUT -> %s\n", ok ? "Wrote" : "FAILED to write", lutPath);
        return ok ? 0 : 1;
    }
    const bool  showLut     = hasArg("--show-lut");
    const bool  showDepth   = hasArg("--show-depth");
    const char* dumpSkyPath = strArg("--dump-sky");

    const double skyEnd     = dblArg("--sky-end", 0.0);

    const bool   lookSun    = hasArg("--look-sun");
    const double sunElev    = dblArg("--sun-elev", -1.0);
    const char* shotPath    = strArg("--screenshot");

    const bool  screenshot  = shotPath != nullptr || dumpSkyPath != nullptr;
    int         framesLimit = intArg("--frames", 0);
    if (screenshot && framesLimit == 0) framesLimit = 150;
    const double spawnAlt  = dblArg("--alt", -1.0);

    const int   flightTest = intArg("--flight-test", 0);

    const int   travelTest = intArg("--travel-test", 0);
    const bool  scripted   = flightTest > 0;
    const bool  travelling = travelTest > 0;
    if (scripted)   framesLimit = flightTest;
    if (travelling) framesLimit = travelTest;

    const bool  syncGen    = hasArg("--sync-gen") || screenshot;

    const bool  parked     = hasArg("--cockpit");

    const double lookDown  = dblArg("--look-down", 0.0);

    const int    pressAt   = intArg("--press-at", 0);

    const int    destArg   = intArg("--dest", -1);

    const bool  startFreecam = !scripted && !travelling && !parked &&
                               !hasArg("--creative-at") &&
                               (hasArg("--freecam") || screenshot ||
                                framesLimit > 0 || spawnAlt >= 0.0);

    bool creative = startFreecam;

    const int creativeAt = intArg("--creative-at", 0);

    const double skimSpeed = dblArg("--skim", 0.0);
    const double skimAlt   = dblArg("--skim-alt", 60.0);
    const bool  showShip   = hasArg("--show-ship");
    const bool  startChase = hasArg("--chase");

    const bool  startFull  = hasArg("--fullscreen");
    bool       wireframe   = hasArg("--wireframe");

    bool       showHud     = hasArg("--hud");

    bool  showEditor   = hasArg("--editor");

    PlanetParams  editedParams{};

    if (const char* planetFile = strArg("--planet")) {
        std::ifstream pf(planetFile, std::ios::binary);
        if (pf) {
            const std::string text((std::istreambuf_iterator<char>(pf)),
                                    std::istreambuf_iterator<char>());
            const ReadReport rep = readPlanet(text, editedParams);
            const std::vector<Violation> bad = validate(editedParams);
            if (hasErrors(bad)) {

                std::printf("planet %s REFUSED:\n", planetFile);
                for (const Violation& b : bad)
                    if (b.sev == Severity::Error)
                        std::printf("  %s: %s\n", b.field, b.message.c_str());
                return 1;
            }
            std::printf("planet %s: %d values loaded, %zu keys not understood\n",
                        planetFile, rep.applied, rep.unknown.size());
            for (const std::string& u : rep.unknown)
                std::printf("  unknown key: %s\n", u.c_str());
            for (const Violation& b : bad)
                std::printf("  warning: %s: %s\n", b.field, b.message.c_str());
        } else {
            std::printf("planet %s: cannot open\n", planetFile);
            return 1;
        }
    }
    PlanetEditor  editor;

    UiState       uiState;
    std::string   editorNote;
    const char*   kPlanetFile = "planet.gaia";

    const int  rebuildEvery   = intArg("--rebuild-test", 0);
    int        rebuilds       = 0;
    double     worstRebuildMs = 0.0;
    double     worstShadingMs = 0.0;
    uint32_t   minDrawnAfter  = 0xFFFFFFFFu;
    PlanetParams livingParams{};

#ifdef _DEBUG
    bool       validation  = !hasArg("--novalidation");
#else

    bool       validation  = hasArg("--validate") ||
                             (std::getenv("PLANET_VALIDATION") != nullptr) ||
                             framesLimit > 0;
    if (hasArg("--novalidation")) validation = false;
#endif

    Win32Window window;

    const bool background = framesLimit > 0 && !hasArg("--foreground");
    if (!window.create("Procedural Planet (Vulkan)", kWindowWidth, kWindowHeight,
                       background)) {
        std::fprintf(stderr, "Failed to create window\n");
        return 1;
    }

    VulkanRenderer renderer;
    try {
        renderer.init(window.hinstance(), window.hwnd(), kWindowWidth, kWindowHeight, validation);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        return 1;
    }
    if (startFull) window.setFullscreen(true);

    std::printf("GPU: %s | validation: %s | camera: %s\n", renderer.gpuName().c_str(),
                validation ? "on" : "off", creative ? "freecam" : "ship");
    std::fflush(stdout);

    PlanetRenderer planet;
    planet.init(renderer.device(), renderer.physicalDevice(),
                renderer.graphicsQueue(), renderer.graphicsFamily(),
                renderer.framesInFlight(), !syncGen);

    if (classifyChange(planet.world().p, editedParams) != ParamChange::None)
        planet.setParams(editedParams, 0);

    ScatterRenderer scatter;

    ShipRenderer shipRenderer;
    shipRenderer.init(renderer.device(), renderer.physicalDevice(),
                      renderer.graphicsQueue(), renderer.graphicsFamily());
    ShipState ship;

    space::DVec3 camPos{ 0.0, kPlanetRadius * 0.6, kPlanetRadius * 2.4 };
    float yaw   = space::kPi;
    float pitch = -0.24f;

    space::Basis spawnBasis;
    if (spawnAlt >= 0.0) {

        World probe;
        const space::DVec3 toSun = space::normalize(space::DVec3{
            -kSunDirection.x, -kSunDirection.y, -kSunDirection.z });
        space::DVec3 best{ 0.0, 1.0, 0.0 };
        double bestR = -1e9;
        float  bestE = 0.0f;

        const double latCap = dblArg("--spawn-lat", 1.1);

        const int destSpawn = intArg("--spawn-dest", -1);

        const int biomeSpawn = intArg("--spawn-biome", -1);
        const bool spireSpawn = hasArg("--spawn-spire");

        const bool rangeSpawn = hasArg("--spawn-range");

        const bool canopySpawn = hasArg("--spawn-canopy");

        const int kProbes = (rangeSpawn || spireSpawn || canopySpawn) ? 200000 : 8192;
        const double ga = space::kPi * (3.0 - std::sqrt(5.0));
        for (int i = 0; i < kProbes; ++i) {
            const double y = 1.0 - 2.0 * (i + 0.5) / kProbes;
            const double r = std::sqrt(std::max(0.0, 1.0 - y * y));
            const double th = ga * i;
            const space::DVec3 d{ std::cos(th) * r, y, std::sin(th) * r };
            if (std::fabs(y) > latCap) continue;
            const double sunCos = space::dot(d, toSun);
            if (sunElev >= 0.0) {

                if (std::fabs(sunCos - sunElev) > 0.02) continue;
            } else if (sunCos < 0.5) {
                continue;
            }

            if (canopySpawn) {
                const TerrainShape shp = sampleShape(probe, d);
                if (shp.landE <= 0.0f) continue;

                const double delta = 2.0 / double(1 << kMaxDepth) / kPatchResolution;
                double cu = 0.0, cv = 0.0;
                const int cf = scatterFaceOf(d, cu, cv);
                const SurfaceSample cs = sampleSurface(probe, cf, cu, cv, delta * 0.5,
                                                       kPhysicsDetailOctaves);
                if (cs.mat[kLayerDirt] < 0.25f || cs.mat[kLayerForestFloor] < 0.25f)
                    continue;
            }
            if (rangeSpawn) {
                const TerrainShape shp = sampleShape(probe, d);
                if (shp.orogeny < 0.5f * kRangeCrestHeight) continue;
            }
            if (spireSpawn) {
                const TerrainShape shp = sampleShape(probe, d);
                if (spireHeight(probe, d, shp, kPhysicsDetailOctaves) <
                    0.35 * kSpireAmplitude) continue;
            }
            if (biomeSpawn >= 0) {
                const BiomeWeights bw = biomeWeightsAt(probe.p, 1.0f, reliefAt(probe, d),
                                                       float(d.y));
                int dom = kBiomePlains;
                for (int k = 1; k < kBiomeCount; ++k)
                    if (bw.w[k] > bw.w[dom]) dom = k;
                if (dom != biomeSpawn) continue;
            }

            const TerrainShape shp = sampleShape(probe, d);
            if (shp.landE * shp.amp > kBandHighland) continue;
            const double r2 = terrainRadius(probe.p, shp);
            if (r2 > bestR) { bestR = r2; best = d; }
        }

        if (destSpawn >= 0 && destSpawn < kDestinationCount) {
            Destination sites[kDestinationCount];
            buildDestinations(sites, [&](const space::DVec3& p) {
                GroundHit h;
                const space::DVec3 d = space::normalize(p);
                h.radius = surfaceRadiusAt(probe, d, kPhysicsDetailOctaves);
                h.normal = d;
                return h;
            });
            best  = space::normalize(sites[destSpawn].pos);
        }

        bestE = sampleLandE(probe, best);
        const double groundR = surfaceRadiusAt(probe, best, kPhysicsDetailOctaves);
        camPos = best * (groundR + spawnAlt);

        const double back = dblArg("--spawn-back", 0.0);
        if (back != 0.0) {
            const space::DVec3 u0 = space::normalize(camPos);
            const space::DVec3 r0 = (std::fabs(u0.y) < 0.9) ? space::DVec3{0,1,0}
                                                            : space::DVec3{1,0,0};
            const space::DVec3 t0 = space::normalize(space::cross(u0, r0));
            const space::DVec3 shifted = space::normalize(camPos - t0 * back);
            camPos = shifted * (surfaceRadiusAt(probe, shifted,
                                                kPhysicsDetailOctaves) + spawnAlt);
        }

        const space::DVec3 upD = space::normalize(camPos);
        const space::DVec3 ref = (std::fabs(upD.y) < 0.9) ? space::DVec3{0,1,0}
                                                          : space::DVec3{1,0,0};
        const space::DVec3 tangent = space::normalize(space::cross(upD, ref));
        const double dip = std::acos(kPlanetRadius / (groundR + spawnAlt)) + 0.06;
        const space::DVec3 fwdD = space::normalize(tangent * std::cos(dip) +
                                                   upD * -std::sin(dip));

        const space::DVec3 aimD = lookSun ? toSun : fwdD;
        spawnBasis.forward = space::toF(aimD);
        spawnBasis.right   = space::toF(space::normalize(space::cross(aimD, upD)));
        spawnBasis.up      = space::toF(space::normalize(space::cross(spawnBasis.right,
                                                                      aimD)));
        std::printf("spawn: land elevation %.2f (ground %.0f m ASL), camera %.0f m AGL\n",
                    bestE, groundR - kPlanetRadius, spawnAlt);

        {
            double su = 0.0, sv = 0.0;
            const int face = scatterFaceOf(best, su, sv);
            const double delta = 2.0 / double(1 << kMaxDepth) / kPatchResolution;
            const SurfaceSample ss = sampleSurface(probe, face, su, sv, delta * 0.5,
                                                   kPhysicsDetailOctaves);
            std::printf("spawn: canopy %.3f, ground layers", canopyAt(probe, best));
            for (int m = 0; m < kGroundLayerCount; ++m)
                if (ss.mat[m] > 0.01f)
                    std::printf("  %s %.0f%%", kGroundLayers[m].label, 100.0f * ss.mat[m]);
            std::printf("\n");
        }
        std::fflush(stdout);
    }

    if (showShip) {
        const space::Basis b = (spawnAlt >= 0.0) ? spawnBasis
                                                 : space::makeBasis(yaw, pitch);
        const space::DVec3 up  = space::normalize(camPos);
        const space::DVec3 fwd = space::normalize(
            space::toD(b.forward) - up * space::dot(space::toD(b.forward), up));
        ship.pos = camPos + space::toD(b.forward) * 45.0;

        const space::Quat level = space::fromTo(Vec3{ 0, 1, 0 }, space::toF(up));
        const space::Vec3 lf    = space::rotate(level, Vec3{ 0, 0, 1 });

        const space::Quat turn  = space::fromAxisAngle(space::toF(up),
                                                       space::deg2rad(210.0f));
        ship.orient = turn * space::fromTo(lf, space::toF(fwd)) * level;
        ship.gearT  = 1.0f;
        std::printf("  --show-ship: hull parked 45 m ahead, %llu triangles\n",
                    static_cast<unsigned long long>(shipRenderer.triangleCount()));
        std::fflush(stdout);
    }

    TerrainGround ground(editedParams);

    scatter.init(renderer.device(), renderer.physicalDevice(),
                 renderer.graphicsQueue(), renderer.graphicsFamily(), ground);
    scatter.startJobs(!syncGen);
    CameraState   cam;
    cam.thirdPerson = startChase;
    bool  assistOn = true;
    bool  gearDown = false;
    float throttle = 0.0f;
    float stickX = 0.0f, stickY = 0.0f;
    int   hovered  = -1;
    LampState lamp = LampState::Cold;

    QuantumDrive drive;
    Destination  destinations[kDestinationCount];
    buildDestinations(destinations, ground);
    int selected = 0;

    {
        const space::DVec3 dir = findSunlitLand(ground.w());
        const double groundR = ground(dir * kPlanetRadius).radius;

        const double spawnH = scripted ? 300.0 : kShipSpawnAltitude;
        ship.pos = dir * (groundR + spawnH);

        ship.orient = uprightOn(dir, space::DVec3{ 0, 1, 0 });

        if (parked) {
            ship.pos    = dir * (groundR + kGearLegLength);
            ship.vel    = space::DVec3{ 0.0, 0.0, 0.0 };
            ship.angVel = Vec3{ 0.0f, 0.0f, 0.0f };
            ship.gearT  = 1.0f;
            gearDown    = true;
        }

        if (!creative) {
            camPos = ship.pos;
            std::printf("ship: spawned %.0f m above land at %.0f m ASL\n",
                        spawnH, groundR - kPlanetRadius);
            std::fflush(stdout);
        }
    }

    if (destArg >= 0) selected = destArg % kDestinationCount;
    else if (travelling) {
        double best = 1e30;
        for (int i = 0; i < kDestinationCount; ++i) {
            const double d = space::length(destinations[i].pos - ship.pos);
            if (d < best) { best = d; selected = i; }
        }
        std::printf("travel: heading for %s, %.0f km away\n",
                    destinations[selected].name, best / 1000.0);
        std::fflush(stdout);
    }

    const float kLookSens = 0.0022f;
    const float kMaxPitch = space::deg2rad(89.0f);

    const float kSpeedStep = 1.30f;
    const float kSpeedMin  = 1.0f / 256.0f;
    const float kSpeedMax  = 64.0f;
    float creativeSpeed    = 1.0f;

    const double kShipRevealRange = 30.0;

    space::Basis basis = space::makeBasis(yaw, pitch);

    const Vec3 sun = space::normalize(kSunDirection);

    const double startTime = nowSeconds();
    double lastTime   = startTime;
    double titleTimer = 0.0;
    double fpsAccum   = 0.0;

    std::vector<float> frameMs;
    int    fpsFrames  = 0;
    int    shownFps   = 0;
    uint64_t maxTriangles = 0;

    int  rendered   = 0;
    int  settledAt  = -1;

    float peakSelect = 0.0f, peakUpload = 0.0f;

    double recAccum = 0.0; int recFrames = 0; double peakRecord = 0.0;
    double selAccum = 0.0, upAccum = 0.0;
    uint32_t peakDraws = 0;
    int  arrivedFor = 0;
    long iterations = 0;
    const long maxIterations = framesLimit > 0 ? (long)framesLimit * 20 + 2000 : 0x7fffffff;

    while (window.pump()) {
        ++iterations;
        if (framesLimit > 0 && (rendered >= framesLimit || iterations > maxIterations)) break;

        if (travelling && drive.state == DriveState::Arrived && ++arrivedFor > 120) break;

        const double t  = nowSeconds();
        float dt = static_cast<float>(t - lastTime);
        lastTime = t;
        if (dt > 0.1f) dt = 0.1f;

        if (window.keyDown(VK_ESCAPE)) break;

        if (window.keyPressed(VK_F11) ||
            (window.keyDown(VK_MENU) && window.keyPressed(VK_RETURN)))
            window.setFullscreen(!window.fullscreen());
        if (window.keyPressed('F'))   wireframe = !wireframe;

        float mdx = 0.0f, mdy = 0.0f;
        window.consumeMouseDelta(mdx, mdy);
        if (framesLimit > 0) { mdx = 0.0f; mdy = 0.0f; }

        const float wheelRaw = window.consumeWheel();
        const float editorWheel = showEditor ? wheelRaw : 0.0f;
        const float wheel       = showEditor ? 0.0f     : wheelRaw;

        const bool toggleNow = window.keyPressed(VK_F2) ||
                               (creativeAt > 0 && rendered == creativeAt);
        if (!scripted && !travelling && toggleNow) {
            creative = !creative;
            if (creative) {
                pitch = std::asin(std::max(-1.0f, std::min(1.0f, basis.forward.y)));
                yaw   = std::atan2(basis.forward.x, basis.forward.z);
            }
            std::printf("%s\n", creative ? "creative mode: free flight (F2 to return"
                                           " to the ship, wheel changes speed)"
                                         : "creative mode off: back in the ship");
            std::fflush(stdout);
        }

        double alt = 0.0;

        if (creative) {

            yaw   -= mdx * kLookSens;
            pitch -= mdy * kLookSens;

            pitch  = space::clampPitch(pitch, space::kPi * 0.5f);
            basis  = (spawnAlt >= 0.0)
                   ? spawnBasis
                   : space::toBasis(space::fromYawPitch(yaw, pitch));

            const Vec3 strafeRight = space::normalize(space::cross(basis.forward, basis.up));

            Vec3 move{ 0, 0, 0 };
            if (window.keyDown('W')) move = move + basis.forward;
            if (window.keyDown('S')) move = move - basis.forward;
            if (window.keyDown('D')) move = move + strafeRight;
            if (window.keyDown('A')) move = move - strafeRight;
            if (window.keyDown(VK_SPACE)) move.y += 1.0f;
            if (window.keyDown(VK_SHIFT)) move.y -= 1.0f;

            alt = std::max(space::length(camPos) - ground(camPos).radius, 0.0);
            float speed = static_cast<float>(
                              std::min(std::max(alt, 10.0), 150000.0) * 1.1 + 15.0);
            if (window.keyDown(VK_CONTROL)) speed *= 6.0f;

            if (wheel != 0.0f) {
                creativeSpeed *= std::pow(kSpeedStep, wheel);
                creativeSpeed = std::max(kSpeedMin, std::min(kSpeedMax, creativeSpeed));
            }
            speed *= creativeSpeed;

            if (skimSpeed > 0.0 && framesLimit > 0) {
                const space::DVec3 up0 = space::normalize(camPos);
                const space::DVec3 ref0 = (std::fabs(up0.y) < 0.9)
                                          ? space::DVec3{ 0, 1, 0 }
                                          : space::DVec3{ 1, 0, 0 };
                const space::DVec3 fwd0 = space::normalize(space::cross(up0, ref0));
                const space::DVec3 next = space::normalize(camPos + fwd0 * (skimSpeed * dt));
                camPos = next * (surfaceRadiusAt(planet.world(), next,
                                                 kPhysicsDetailOctaves) + skimAlt);
                basis  = space::makeBasis(std::atan2(float(fwd0.x), float(fwd0.z)),
                                          -0.12f);
            } else if (framesLimit > 0 && !screenshot && spawnAlt < 0.0) {
                const float ph = static_cast<float>(rendered) / static_cast<float>(framesLimit);
                const double targetAlt = 2000.0 + 400000.0 * std::fabs(std::sin(ph * space::kPi * 2.0f));
                const space::DVec3 dir = space::normalize(camPos);
                camPos = dir * (static_cast<double>(kPlanetRadius) + targetAlt);
                yaw += 0.01f;
            } else if (space::length(move) > 1e-5f) {
                camPos = camPos + space::normalize(move) * (speed * dt);
            }
        } else {

            if (window.keyPressed('V'))     assistOn = !assistOn;
            if (window.keyPressed('G'))     gearDown = !gearDown;

            const bool clicked = window.keyPressed(VK_LBUTTON) ||
                                 (pressAt > 0 && rendered == pressAt);
            if (hovered >= 0 && !cam.thirdPerson && clicked) {
                ShipToggles t{ gearDown, assistOn, throttle };
                activateSwitch(t, hovered);
                gearDown = t.gearDown;
                assistOn = t.assistOn;
                throttle = t.throttle;

                if (t.jumpRequested) {
                    if (drive.state == DriveState::Spooling) abortDrive(drive);
                    else engage(drive, ship, destinations[selected].pos);
                }
            }
            if (window.keyPressed(VK_F4))   cam.thirdPerson = !cam.thirdPerson;
            const bool freeLook = window.keyDown('Y');

            if (!freeLook) {
                stickX += mdx * kStickGain;
                stickY += mdy * kStickGain;
            }

            const float centre = 1.0f - std::exp(-dt / kStickReturnTau);
            stickX -= stickX * centre;
            stickY -= stickY * centre;
            stickX = std::max(-1.0f, std::min(1.0f, stickX));
            stickY = std::max(-1.0f, std::min(1.0f, stickY));

            ShipControls ctl;
            ctl.assist   = assistOn;
            ctl.gearDown = gearDown;
            ctl.boost    = window.keyDown(VK_SHIFT);
            if (!freeLook) {

                ctl.pitch = stickY;

                ctl.yaw = -stickX;
            }
            if (window.keyDown('E')) ctl.roll += 1.0f;
            if (window.keyDown('Q')) ctl.roll -= 1.0f;

            if (window.keyDown('D')) ctl.strafeX -= 1.0f;
            if (window.keyDown('A')) ctl.strafeX += 1.0f;
            if (window.keyDown(VK_SPACE))   ctl.strafeY += 1.0f;
            if (window.keyDown(VK_CONTROL)) ctl.strafeY -= 1.0f;

            if (assistOn) {
                if (window.keyDown('W')) throttle += kThrottleRate * dt;
                if (window.keyDown('S')) throttle -= kThrottleRate * dt;
                throttle += wheel * kThrottleNotch;
                if (window.keyPressed('X')) throttle = 0.0f;
                throttle = std::max(0.0f, std::min(1.0f, throttle));
                ctl.throttle = throttle;
            } else {

                if (window.keyDown('W')) ctl.thrustZ += 1.0f;
                if (window.keyDown('S')) ctl.thrustZ -= 1.0f;
            }

            if (window.keyPressed('N') && !drive.busy())
                selected = (selected + 1) % kDestinationCount;
            if (window.keyPressed('B')) {
                if (drive.state == DriveState::Spooling) abortDrive(drive);
                else engage(drive, ship, destinations[selected].pos);
            }

            if (scripted)   scriptFlight(ctl, gearDown, throttle, ship, ground, rendered);
            if (travelling) scriptTravel(ctl, drive, destinations[selected], ship, rendered);

            ctl.quantumAccel  = update(drive, ship, dt);
            ctl.quantumTravel = drive.busy();

            if (!parked) step(ship, ctl, ground, dt);

            const bool holdLook = freeLook || lookDown > 0.0;
            if (lookDown > 0.0) cam.freePitch = -space::deg2rad(float(lookDown));
            const CameraView view = updateCamera(cam, ship, freeLook ? mdx : 0.0f,
                                                 freeLook ? mdy : 0.0f, holdLook, dt);
            camPos = view.pos;
            basis  = space::toBasis(view.orient);
            alt    = space::length(ship.pos) - ground(ship.pos).radius;

            if (drive.busy()) {
                lamp = LampState::Busy;
            } else {
                QuantumDrive probe;
                lamp = (engage(probe, ship, destinations[selected].pos) ==
                        EngageResult::Ok) ? LampState::Ready : LampState::Cold;
            }

            hovered = cam.thirdPerson
                    ? -1
                    : pickSwitch(kEyeOffset, toBody(ship.orient, basis.forward)).index;
        }

        const uint32_t w = std::max(window.width(), 1u);
        const uint32_t h = std::max(window.height(), 1u);
        const float aspect = static_cast<float>(w) / static_cast<float>(h);

        Mat4 view = planet::lookAt(Vec3{ 0.0f, 0.0f, 0.0f }, basis.forward, basis.up);
        Mat4 proj = planet::perspectiveInfReverseZ(space::deg2rad(kFovYDegrees),
                                                   aspect, kNearPlane);
        Mat4 viewProj = proj * view;
        Frustum frustum = Frustum::fromViewProj(viewProj);

        planet.update(camPos, renderer.frameNumber());

        scatter.update(planet.world(), camPos, syncGen);

        if (settledAt < 0 && planet.settled() && rendered > 2) settledAt = rendered;

        if (screenshot && rendered == framesLimit - 1) {

            uint64_t settleFrame = renderer.frameNumber();
            for (int guard = 0; guard < 10000 && !planet.settled(); ++guard)
                planet.update(camPos, ++settleFrame);

            if (shotPath) renderer.requestCapture(shotPath);
        }

        FrameUniforms frame{};
        for (int i = 0; i < 16; ++i) frame.viewProj[i] = viewProj.m[i];
        frame.sunDir[0] = sun.x; frame.sunDir[1] = sun.y; frame.sunDir[2] = sun.z;
        frame.params[0] = wireframe ? 0.55f : kAmbient;
        frame.params[1] = (skyEnd > 0.0) ? 3.0f
                        : showDepth ? 2.0f : (showLut ? 1.0f : 0.0f);

        frame.params[2] = kSunIrradiance / space::kPi;
        frame.params[3] = kExposure;
        frame.terrain[0] = kMorphBandStart;

        frame.terrain[1]  = planet.world().p.groundTileMetres;
        frame.terrain[2]  = planet.world().p.groundMacroMetres;
        frame.terrain[3]  = planet.world().p.groundPaletteMix;
        frame.terrain2[0] = planet.world().p.groundNormalStrength;

        frame.terrain2[1] = float(kGroundLayerCount);

        {
            const float radPerPixel = 2.0f * std::tan(space::deg2rad(kFovYDegrees) * 0.5f)
                                      / float(std::max(1u, renderer.extent().height));
            const float oneTexel = kGroundMacroMetres / radPerPixel;
            frame.terrain2[2] = oneTexel * 0.25f;
            frame.terrain2[3] = oneTexel * 2.0f;
        }

        const float tanHalf = std::tan(space::deg2rad(kFovYDegrees) * 0.5f);
        const Vec3  screenRight = space::normalize(space::cross(basis.forward, basis.up));
        const Vec3  camR = screenRight * (tanHalf * aspect);
        const Vec3  camU = basis.up    * tanHalf;
        frame.camRight[0] = camR.x; frame.camRight[1] = camR.y; frame.camRight[2] = camR.z;
        frame.camUp[0]    = camU.x; frame.camUp[1]    = camU.y; frame.camUp[2]    = camU.z;
        frame.camFwd[0] = basis.forward.x;
        frame.camFwd[1] = basis.forward.y;
        frame.camFwd[2] = basis.forward.z;
        frame.camFwd[3] = kNearPlane;

        const space::DVec3 upD = space::normalize(camPos);
        frame.camGeo[0] = static_cast<float>(upD.x);
        frame.camGeo[1] = static_cast<float>(upD.y);
        frame.camGeo[2] = static_cast<float>(upD.z);
        frame.camGeo[3] = static_cast<float>(space::length(camPos) - kPlanetRadius);

        frame.betaR[0] = static_cast<float>(kBetaRayleighR);
        frame.betaR[1] = static_cast<float>(kBetaRayleighG);
        frame.betaR[2] = static_cast<float>(kBetaRayleighB);
        frame.betaR[3] = static_cast<float>(kBetaMieScatter);
        frame.atmoA[0] = static_cast<float>(kRayleighScaleHeight);
        frame.atmoA[1] = static_cast<float>(kMieScaleHeight);
        frame.atmoA[2] = static_cast<float>(kBetaMieExtinct);
        frame.atmoA[3] = kMieG;
        frame.atmoB[0] = static_cast<float>(kPlanetRadius);
        frame.atmoB[1] = static_cast<float>(kAtmosphereTop);
        frame.atmoB[2] = kSunIrradiance;
        frame.atmoB[3] = static_cast<float>(skyEnd);

        frame.sunDisc[0] = std::cos(kSunAngularRadius);
        frame.sunDisc[1] = kSunDiscRadiance;
        frame.sunDisc[2] = 2.0f * tanHalf / static_cast<float>(h);
        frame.sunDisc[3] = 0.0f;

        {
            const space::DVec3 toDest = destinations[selected].pos - camPos;
            const double len = space::length(toDest);
            const space::DVec3 unit = (len > 1.0) ? toDest * (1.0 / len)
                                                  : space::DVec3{ 0, 1, 0 };
            frame.marker[0] = static_cast<float>(unit.x);
            frame.marker[1] = static_cast<float>(unit.y);
            frame.marker[2] = static_cast<float>(unit.z);
            frame.marker[3] = creative ? 0.0f : (drive.busy() ? 2.0f : 1.0f);
        }

        if (dumpSkyPath && rendered == framesLimit - 1) {
            const planet::atmo::TransmittanceLut lut = planet::atmo::buildTransmittanceLut();
            const int W = int(w), H = int(h);
            std::vector<unsigned char> rgb(size_t(W) * H * 3);

            const Vec3 camF{ frame.camFwd[0],   frame.camFwd[1],   frame.camFwd[2]   };
            const Vec3 camRv{ frame.camRight[0], frame.camRight[1], frame.camRight[2] };
            const Vec3 camUv{ frame.camUp[0],    frame.camUp[1],    frame.camUp[2]    };
            const Vec3 upv{ frame.camGeo[0], frame.camGeo[1], frame.camGeo[2] };
            const Vec3 sunToward = space::normalize(Vec3{ -frame.sunDir[0],
                                                          -frame.sunDir[1],
                                                          -frame.sunDir[2] });
            auto encodeSrgb = [](float c) -> unsigned char {
                c = std::max(0.0f, std::min(1.0f, c));
                const float e = (c <= 0.0031308f) ? c * 12.92f
                                                  : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
                return static_cast<unsigned char>(std::lround(e * 255.0f));
            };

            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    const float ndcX = ((x + 0.5f) / W) * 2.0f - 1.0f;
                    const float ndcY = ((y + 0.5f) / H) * 2.0f - 1.0f;
                    const Vec3  dir  = space::normalize(camF + camRv * ndcX - camUv * ndcY);

                    Vec3 vt{ 1.0f, 1.0f, 1.0f };
                    const float tEnd = (skyEnd > 0.0) ? float(skyEnd) : 1e9f;
                    const Vec3  ins  = planet::atmo::skyRadiance(dir, upv, frame.camGeo[3],
                                                                 sunToward, lut, tEnd,
                                                                 kSkySteps, &vt);
                    const Vec3  disc = (skyEnd > 0.0)
                        ? Vec3{ 0.0f, 0.0f, 0.0f }
                        : planet::atmo::sunDiscRadiance(dir, upv, frame.camGeo[3],
                                                        sunToward, lut,
                                                        frame.sunDisc[2]);
                    const Vec3  behind = (skyEnd > 0.0) ? vt : Vec3{ 0.0f, 0.0f, 0.0f };
                    const Vec3  t    = planet::atmo::acesTonemap((behind + ins + disc) * kExposure);
                    unsigned char* px = &rgb[(size_t(y) * W + x) * 3];
                    px[0] = encodeSrgb(t.x); px[1] = encodeSrgb(t.y); px[2] = encodeSrgb(t.z);
                }
            }
            std::ofstream fo(dumpSkyPath, std::ios::binary);
            bool ok = fo.is_open();
            if (ok) {
                fo << "P6\n" << W << " " << H << "\n255\n";
                fo.write(reinterpret_cast<const char*>(rgb.data()), std::streamsize(rgb.size()));
                ok = fo.good();
            }
            std::printf("%s CPU sky -> %s  (alt %.1f m)\n",
                        ok ? "Wrote" : "FAILED to write", dumpSkyPath,
                        double(frame.camGeo[3]));
            renderer.waitIdle();
            break;
        }

        if (rebuildEvery > 0 && rendered > 0 && rendered % rebuildEvery == 0) {

            const bool terrainEdit = (rebuilds % 2) == 0;
            if (terrainEdit)
                livingParams.terrainAmplitude =
                    (rebuilds % 4 == 0) ? kTerrainAmplitude * 0.92
                                        : kTerrainAmplitude;
            else
                livingParams.groundPaletteMix =
                    (rebuilds % 4 == 1) ? 0.5f : 0.0f;

            const double t0 = nowSeconds();
            planet.setParams(livingParams, renderer.frameNumber());
            const double ms = (nowSeconds() - t0) * 1000.0;
            if (terrainEdit) worstRebuildMs = std::max(worstRebuildMs, ms);
            else             worstShadingMs = std::max(worstShadingMs, ms);
            ++rebuilds;
        }

        const double tRec0 = nowSeconds();
        VkCommandBuffer cmd = renderer.beginFrame(frame, wireframe);
        PlanetStats stats{};
        if (cmd != VK_NULL_HANDLE) {
            planet.recordDraw(cmd, renderer.pipelineLayout(), frustum, stats);
            scatter.recordDraw(cmd, renderer.pipelineLayout(), camPos, stats.triangles);
            scatter.recordOutposts(cmd, renderer.pipelineLayout(), camPos, stats.triangles);

            const bool shipInView = showShip ||
                (creative && !startFreecam &&
                 space::length(ship.pos - camPos) > kShipRevealRange) ||
                (!creative && cam.thirdPerson);
            if (shipInView)
                shipRenderer.recordDraw(cmd, renderer.pipelineLayout(), camPos, ship,
                                        stats.triangles);
            else if (!creative)
                shipRenderer.recordCockpit(cmd, renderer.pipelineLayout(), camPos, ship,
                                           ShipToggles{ gearDown, assistOn, throttle },
                                           hovered, lamp, stats.triangles);

            if (window.keyPressed(VK_F3)) showHud = !showHud;
            if (window.keyPressed(VK_F4) && renderer.font().ok()) {
                showEditor = !showEditor;

                window.setMouseLook(!showEditor);
            }

            if (showEditor && renderer.font().ok()) {
                OverlayDraw od(renderer.font());
                Ui ui(od, renderer.font(), uiState);

                UiInput uin;
                window.mousePosition(uin.mx, uin.my);
                uin.down    = window.keyDown(VK_LBUTTON);
                uin.pressed = window.keyPressed(VK_LBUTTON);
                uin.coarse  = window.keyDown(VK_SHIFT);
                uin.wheel   = editorWheel;
                if (window.keyPressed(VK_LEFT))  uin.nudge = -1;
                if (window.keyPressed(VK_RIGHT)) uin.nudge = +1;
                ui.begin(uin);

                EditorActions act = editor.draw(ui, od, uin, editedParams,
                                                planet.world().p,
                                                float(window.width()),
                                                float(window.height()));
                if (window.keyPressed(VK_F5)) act.save = true;
                if (window.keyPressed(VK_F9)) act.load = true;

                const ParamChange ch = classifyChange(planet.world().p, editedParams);
                if (ch == ParamChange::ShadingOnly ||
                    (act.applyTerrain && ch == ParamChange::Terrain))
                    planet.setParams(editedParams, renderer.frameNumber());

                if (act.reset) { editedParams = PlanetParams{}; editorNote = "reset"; }
                if (act.save) {
                    std::ofstream f(kPlanetFile, std::ios::binary);
                    if (f) { f << writePlanet(editedParams, "gaia"); editorNote =
                                 std::string("saved ") + kPlanetFile; }
                    else editorNote = std::string("could NOT write ") + kPlanetFile;
                }
                if (act.load) {
                    std::ifstream f(kPlanetFile, std::ios::binary);
                    if (f) {
                        const std::string text((std::istreambuf_iterator<char>(f)),
                                                std::istreambuf_iterator<char>());
                        PlanetParams loaded{};
                        const ReadReport rep = readPlanet(text, loaded);
                        editedParams = loaded;
                        char note[128];
                        std::snprintf(note, sizeof(note),
                                      "loaded %d values, %zu unknown",
                                      rep.applied, rep.unknown.size());
                        editorNote = note;
                    } else {
                        editorNote = std::string("no ") + kPlanetFile + " to load";
                    }
                }
                if (!editorNote.empty())
                    od.text(24.0f, float(window.height()) - 24.0f, editorNote, kUiValue);

                renderer.setOverlay(od.vertices());
            } else if (showHud && renderer.font().ok()) {
                OverlayDraw hud(renderer.font());
                const float lh  = hud.lineHeight();
                const float pad = 8.0f;
                char line[160];

                std::vector<std::string> rows;
                std::snprintf(line, sizeof(line), "%d FPS   %llu tris", shownFps,
                              static_cast<unsigned long long>(stats.triangles));
                rows.push_back(line);
                std::snprintf(line, sizeof(line), "patches  %u active  %u drawn  %u culled",
                              stats.activePatches, stats.drawnPatches, stats.culledPatches);
                rows.push_back(line);
                std::snprintf(line, sizeof(line), "meshes   %u cached  %u pending",
                              stats.cachedMeshes, stats.pendingJobs);
                rows.push_back(line);
                std::snprintf(line, sizeof(line), "select   %.2f ms   upload %.2f ms",
                              stats.msSelect, stats.msUpload);
                rows.push_back(line);
                std::snprintf(line, sizeof(line), "altitude %.0f m", alt);
                rows.push_back(line);

                size_t widest = 0;
                for (const std::string& r : rows) widest = std::max(widest, r.size());
                const float w = hud.textWidth(widest) + pad * 2.0f;
                const float h = lh * float(rows.size()) + pad * 2.0f;
                hud.rect(pad, pad, w, h, kUiPanel);
                float y = pad * 2.0f;
                for (const std::string& r : rows) {
                    hud.text(pad * 2.0f, y, r, kUiText);
                    y += lh;
                }
                renderer.setOverlay(hud.vertices());
            } else {
                renderer.setOverlay({});
            }

            renderer.endFrame();
            const double recMs = (nowSeconds() - tRec0) * 1000.0;
            recAccum += recMs; ++recFrames;
            peakRecord = std::max(peakRecord, recMs);
            peakDraws  = std::max(peakDraws,
                                  stats.drawnPatches + uint32_t(scatter.count()));
            ++rendered;
            maxTriangles = std::max(maxTriangles, stats.triangles);

            if (rebuildEvery > 0 && rebuilds > 0)
                minDrawnAfter = std::min(minDrawnAfter, stats.drawnPatches);
            selAccum += stats.msSelect; upAccum += stats.msUpload;
            peakSelect = std::max(peakSelect, stats.msSelect);
            peakUpload = std::max(peakUpload, stats.msUpload);

            if (framesLimit > 0 && rendered % 500 == 0) {

                std::printf("  frame %d/%d | %llu tris | patches %u (drawn %u/culled %u)"
                            " | cache %u | shards %zu%s\n",
                            rendered, framesLimit,
                            static_cast<unsigned long long>(stats.triangles),
                            stats.activePatches, stats.drawnPatches, stats.culledPatches,
                            stats.cachedMeshes, scatter.count(),
                            scatter.droppedLastRebuild() > 0 ? " (OVER BUDGET)" : "");

                if (stats.evictedAncestors > 0)
                    std::printf("    LOD COLLAPSES so far: %u active-tree ancestors"
                                " evicted\n", stats.evictedAncestors);
                std::fflush(stdout);
            }
        }

        ++fpsFrames;
        frameMs.push_back(dt * 1000.0f);
        fpsAccum += dt;
        titleTimer += dt;
        if (fpsAccum >= 0.5) { shownFps = static_cast<int>(fpsFrames / fpsAccum + 0.5); fpsFrames = 0; fpsAccum = 0.0; }
        if (titleTimer >= 0.25) {
            titleTimer = 0.0;

            char altText[32];
            if (alt < 1000.0)        std::snprintf(altText, sizeof(altText), "%.0f m",  alt);
            else if (alt < 1.0e6)    std::snprintf(altText, sizeof(altText), "%.1f km", alt / 1000.0);
            else                     std::snprintf(altText, sizeof(altText), "%.0f km", alt / 1000.0);
            char title[320];
            if (creative) {

                const double baseSpeed =
                    std::min(std::max(alt, 10.0), 150000.0) * 1.1 + 15.0;
                double shownSpeed = baseSpeed * creativeSpeed;
                if (window.keyDown(VK_CONTROL)) shownSpeed *= 6.0;
                char spdText[48];
                if (shownSpeed < 1000.0)
                    std::snprintf(spdText, sizeof(spdText), "%.0f m/s", shownSpeed);
                else
                    std::snprintf(spdText, sizeof(spdText), "%.1f km/s", shownSpeed / 1000.0);
                std::snprintf(title, sizeof(title),
                    "CREATIVE (F2) | %llu tris | patches %u (drawn %u / culled %u) | "
                    "%d FPS | alt %s | %s (wheel, x%.2f) | %s",
                    static_cast<unsigned long long>(stats.triangles),
                    stats.activePatches, stats.drawnPatches, stats.culledPatches,
                    shownFps, altText, spdText, creativeSpeed,
                    wireframe ? "WIREFRAME (F)" : "solid (F)");
            } else {
                std::snprintf(title, sizeof(title),
                    "%s | %.0f m/s | AGL %s | thr %3.0f%% | %s | gear %s%s | %d FPS | %llu tris",
                    cam.thirdPerson ? "CHASE" : "COCKPIT",
                    space::length(ship.vel), altText, throttle * 100.0,
                    assistOn ? (window.keyDown(VK_SHIFT) ? "NAV" : "SCM") : "DECOUPLED",
                    gearDown ? "DOWN" : "UP",
                    ship.landed ? " | LANDED" : "",
                    shownFps, static_cast<unsigned long long>(stats.triangles));
            }
            window.setTitle(title);
        }
    }

    if (screenshot) {
        const bool ok = renderer.writeCapture();
        std::printf("screenshot %s -> %s\n", ok ? "saved" : "FAILED", shotPath);
        std::fflush(stdout);
    }

    renderer.waitIdle();
    shipRenderer.shutdown();
    scatter.shutdown();
    planet.shutdown();

    int scriptExit = 0;
    if (travelling) {
        const double miss  = space::length(ship.pos - destinations[selected].pos);
        const double speed = space::length(ship.vel);

        const space::DVec3 up = space::normalize(ship.pos);
        const double vUp   = space::dot(ship.vel, up);
        const double vSide = std::sqrt(std::max(0.0, speed * speed - vUp * vUp));
        std::printf("travel: residual %.1f m/s = %.1f vertical + %.1f lateral\n",
                    speed, vUp, vSide);
        if (drive.state == DriveState::Arrived && miss < 20000.0) {
            std::printf("TRAVEL TEST: ARRIVED at %s  miss %.0f m  |v| %.1f m/s\n",
                        destinations[selected].name, miss, speed);
        } else {
            std::fprintf(stderr,
                "TRAVEL TEST: FAILED  state %d  miss %.0f m  |v| %.1f m/s\n",
                int(drive.state), miss, speed);
            scriptExit = 3;
        }
    }
    if (scripted) {
        const double agl = space::length(ship.pos) - ground(ship.pos).radius;
        int legs = 0;
        for (int i = 0; i < 4; ++i) legs += ship.legContact[i] ? 1 : 0;
        const double tilt = std::acos(std::max(-1.0, std::min(1.0,
                                space::dot(shipUp(ship), ground(ship.pos).normal))))
                            * 180.0 / space::kPi;
        if (ship.landed && legs == 4) {
            std::printf("FLIGHT TEST: LANDED  |v| %.3f m/s  tilt %.1f deg  "
                        "AGL %.2f m  legs %d/4\n",
                        space::length(ship.vel), tilt, agl, legs);
        } else {
            std::fprintf(stderr,
                "FLIGHT TEST: FAILED to land  |v| %.3f m/s  AGL %.2f m  "
                "legs %d/4  landed=%d\n",
                space::length(ship.vel), agl, legs, ship.landed ? 1 : 0);
            scriptExit = 3;
        }
        std::fflush(stdout);
    }

    const uint32_t vErrors = renderer.validationErrors();
    const double   elapsed = nowSeconds() - startTime;
    const double   avgFps  = elapsed > 0.0 ? rendered / elapsed : 0.0;
    if (recFrames > 0)
        std::printf("main thread MEAN per frame: LOD select %.2f ms | uploads %.2f ms"
                    " | record %.2f ms   (peaks %.1f / %.1f / %.1f)\n",
                    selAccum / recFrames, upAccum / recFrames, recAccum / recFrames,
                    peakSelect, peakUpload, peakRecord);
    if (recFrames > 0)
        std::printf("draws: up to %u per frame\n", peakDraws);
    if (settledAt >= 0)
        std::printf("LOD converged after %d frames%s\n",
                    settledAt, syncGen ? " (sync-gen)" : "");
    if (rebuildEvery > 0)
        std::printf("REBUILD TEST: %d swaps | terrain edit worst %.1f ms |"
                    " shading edit worst %.3f ms | fewest patches drawn"
                    " afterwards %u\n",
                    rebuilds, worstRebuildMs, worstShadingMs,
                    minDrawnAfter == 0xFFFFFFFFu ? 0u : minDrawnAfter);
    std::printf("Exited cleanly. frames=%d elapsed=%.1fs avgFPS=%.0f maxTriangles=%llu validationErrors=%u\n",
                rendered, elapsed, avgFps,
                static_cast<unsigned long long>(maxTriangles), vErrors);
    std::fflush(stdout);

    renderer.shutdown();
    window.destroy();

    if (framesLimit > 0 && vErrors > 0) return 2;
    return scriptExit;
}

}

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    try {
        return run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        MessageBoxA(nullptr, e.what(), "Procedural Planet - fatal error", MB_ICONERROR);
        return 1;
    }
}
