#pragma once

#include <cstdint>

namespace planet {

inline uint32_t hashMix(uint32_t h) {
    h ^= h >> 16; h *= 0x85EBCA6Bu;
    h ^= h >> 13; h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
}

inline uint32_t hashCombine(uint32_t h, uint32_t v) {
    return hashMix(h + v * 0x9E3779B9u);
}

inline float hashUnit(uint32_t h) {
    return static_cast<float>(h >> 8) / static_cast<float>(1 << 24);
}

}
