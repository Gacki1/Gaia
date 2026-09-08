#pragma once

#include "world/PlanetConfig.h"

namespace planet {

struct PlanetParams {

    unsigned seed = kNoiseSeed;

    float noiseFrequency  = kNoiseFrequency;
    float noiseLacunarity = kNoiseLacunarity;
    float noiseGain       = kNoiseGain;
    int   noiseOctaves    = kNoiseOctaves;
    float seaLevelNoise   = kSeaLevelNoise;
    double terrainAmplitude = kTerrainAmplitude;

    double detailAmplitude = kDetailAmplitude;
    float  detailGain      = kDetailGain;
    float  detailFadeIn    = kDetailFadeIn;
    float  detailRidgeMix  = kDetailRidgeMix;
    int    detailRidgeFull = kDetailRidgeFull;
    int    detailRidgeFade = kDetailRidgeFade;

    float terraceSteps = kTerraceSteps;
    float terraceRiser = kTerraceRiser;
    float terraceMix   = kTerraceMix;

    int   moistureOctaves   = kMoistureOctaves;
    float moistureFrequency = kMoistureFrequency;
    int   reliefOctaves     = kReliefOctaves;
    float reliefFrequency   = kReliefFrequency;

    float reliefPlains   = kReliefPlains;
    float reliefMesa     = kReliefMesa;
    float reliefMountain = kReliefMountain;

    BiomeScales biomeScales[kBiomeCount] = {
        kBiomeScales[0], kBiomeScales[1], kBiomeScales[2], kBiomeScales[3] };

    float rangeSharpness = kRangeSharpness;
    float rangeBoost     = kRangeBoost;
    float rangeBlend     = kRangeBlend;
    int   rangeOctaves   = kRangeOctaves;
    float rangeCoastFade = kRangeCoastFade;

    double spireAmplitude       = kSpireAmplitude;
    float  spireFrequency       = kSpireFrequency;
    int    spireOctaves         = kSpireOctaves;
    float  spireCrest           = kSpireCrest;
    float  spireWall            = kSpireWall;
    int    spireProvinceOctaves = kSpireProvinceOctaves;
    float  spireProvinceFrequency = kSpireProvinceFrequency;
    float  spireProvinceLo      = kSpireProvinceLo;
    float  spireProvinceHi      = kSpireProvinceHi;
    int    spireOctaveIn        = kSpireOctaveIn;
    int    spireOctaveFull      = kSpireOctaveFull;

    float bandBeach    = kBandBeach;
    float bandShore    = kBandShore;
    float bandLowland  = kBandLowland;
    float bandHighland = kBandHighland;
    float bandPeak     = kBandPeak;
    float slopeSoft    = kSlopeSoft;
    float slopeRock    = kSlopeRock;
    float latTemperate = kLatTemperate;
    float latPolar     = kLatPolar;
    float moistDry     = kMoistDry;
    float moistWet     = kMoistWet;

    float groundTileMetres    = kGroundTileMetres;
    float groundMacroMetres   = kGroundMacroMetres;
    float groundPaletteMix    = kGroundPaletteMix;
    float groundNormalStrength = kGroundNormalStrength;

    float  scatterDensity      = kScatterDensity;
    float  scatterMinElevation = kScatterMinElevation;
    float  scatterMaxSlope     = kScatterMaxSlope;
    float  scatterMinScale     = kScatterMinScale;
    float  scatterMaxScale     = kScatterMaxScale;
    double scatterCeiling      = kScatterCeiling;
};

enum class ParamChange {
    None,
    ShadingOnly,
    Terrain,
};

inline bool sameTerrain(const PlanetParams& a, const PlanetParams& b) {
    static_assert(sizeof(PlanetParams) == 320,
                  "PlanetParams changed size -- a parameter was added or "
                  "removed. Classify it in sameTerrain(): terrain if it is "
                  "baked into a vertex or into the LOD bounds, shading if it "
                  "only travels in the frame uniform.");
    for (int i = 0; i < kBiomeCount; ++i) {
        const BiomeScales& x = a.biomeScales[i];
        const BiomeScales& y = b.biomeScales[i];
        if (!(x.amp == y.amp && x.detail == y.detail &&
              x.ridge == y.ridge && x.terrace == y.terrace)) return false;
    }
    return
        a.seed == b.seed &&
        a.noiseFrequency == b.noiseFrequency &&
        a.noiseLacunarity == b.noiseLacunarity &&
        a.noiseGain == b.noiseGain &&
        a.noiseOctaves == b.noiseOctaves &&
        a.seaLevelNoise == b.seaLevelNoise &&
        a.terrainAmplitude == b.terrainAmplitude &&
        a.detailAmplitude == b.detailAmplitude &&
        a.detailGain == b.detailGain &&
        a.detailFadeIn == b.detailFadeIn &&
        a.detailRidgeMix == b.detailRidgeMix &&
        a.detailRidgeFull == b.detailRidgeFull &&
        a.detailRidgeFade == b.detailRidgeFade &&
        a.terraceSteps == b.terraceSteps &&
        a.terraceRiser == b.terraceRiser &&
        a.terraceMix == b.terraceMix &&
        a.moistureOctaves == b.moistureOctaves &&
        a.moistureFrequency == b.moistureFrequency &&
        a.reliefOctaves == b.reliefOctaves &&
        a.reliefFrequency == b.reliefFrequency &&
        a.reliefPlains == b.reliefPlains &&
        a.reliefMesa == b.reliefMesa &&
        a.reliefMountain == b.reliefMountain &&
        a.rangeSharpness == b.rangeSharpness &&
        a.rangeBoost == b.rangeBoost &&
        a.rangeBlend == b.rangeBlend &&
        a.rangeOctaves == b.rangeOctaves &&
        a.rangeCoastFade == b.rangeCoastFade &&
        a.spireAmplitude == b.spireAmplitude &&
        a.spireFrequency == b.spireFrequency &&
        a.spireOctaves == b.spireOctaves &&
        a.spireCrest == b.spireCrest &&
        a.spireWall == b.spireWall &&
        a.spireProvinceOctaves == b.spireProvinceOctaves &&
        a.spireProvinceFrequency == b.spireProvinceFrequency &&
        a.spireProvinceLo == b.spireProvinceLo &&
        a.spireProvinceHi == b.spireProvinceHi &&
        a.spireOctaveIn == b.spireOctaveIn &&
        a.spireOctaveFull == b.spireOctaveFull &&
        a.bandBeach == b.bandBeach &&
        a.bandShore == b.bandShore &&
        a.bandLowland == b.bandLowland &&
        a.bandHighland == b.bandHighland &&
        a.bandPeak == b.bandPeak &&
        a.slopeSoft == b.slopeSoft &&
        a.slopeRock == b.slopeRock &&
        a.latTemperate == b.latTemperate &&
        a.latPolar == b.latPolar &&
        a.moistDry == b.moistDry &&
        a.moistWet == b.moistWet &&
        a.scatterDensity == b.scatterDensity &&
        a.scatterMinElevation == b.scatterMinElevation &&
        a.scatterMaxSlope == b.scatterMaxSlope &&
        a.scatterMinScale == b.scatterMinScale &&
        a.scatterMaxScale == b.scatterMaxScale &&
        a.scatterCeiling == b.scatterCeiling;
}

inline ParamChange classifyChange(const PlanetParams& a, const PlanetParams& b) {
    if (!sameTerrain(a, b)) return ParamChange::Terrain;
    const bool shadingSame =
        a.groundPaletteMix     == b.groundPaletteMix &&
        a.groundNormalStrength == b.groundNormalStrength &&
        a.groundTileMetres     == b.groundTileMetres &&
        a.groundMacroMetres    == b.groundMacroMetres;
    return shadingSame ? ParamChange::None : ParamChange::ShadingOnly;
}

}
