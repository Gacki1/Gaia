#pragma once

#include "world/Noise.h"
#include "world/PlanetParams.h"
#include "world/Plates.h"

namespace planet {

struct World {
    PlanetParams p;
    Noise        noise;

    PlateSystem  plates;

    World() : World(PlanetParams{}) {}
    explicit World(const PlanetParams& params)
        : p(params), noise(params.seed),
          plates(buildPlates(params.seed, [this](const DVec3& d) {
              return plateIsLand(noise, p, d);
          })) {}

    void set(const PlanetParams& params) {
        const bool reseed = params.seed != p.seed;
        p = params;
        if (reseed) noise.reseed(p.seed);

        plates = buildPlates(p.seed, [this](const DVec3& d) {
            return plateIsLand(noise, p, d);
        });
    }
};

}
