#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace planet {

struct LoadedImage {
    std::vector<uint8_t> rgba;
    uint32_t width  = 0;
    uint32_t height = 0;
    bool ok() const { return width > 0 && height > 0 && !rgba.empty(); }
};

LoadedImage loadImageRGBA(const std::string& path, uint32_t maxSize = 0);

}
