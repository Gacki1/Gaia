#pragma once

#include "render/VkUtil.h"
#include <vulkan/vulkan.h>
#include <cstdint>
#include <cstddef>

namespace planet {

class Texture2D {
public:
    Texture2D() = default;
    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;
    Texture2D(Texture2D&& o) noexcept { moveFrom(o); }
    Texture2D& operator=(Texture2D&& o) noexcept {
        if (this != &o) { destroy(); moveFrom(o); }
        return *this;
    }
    ~Texture2D() { destroy(); }

    static Texture2D create(VkDevice device, VkPhysicalDevice phys,
                            VkQueue queue, uint32_t queueFamily,
                            uint32_t width, uint32_t height, VkFormat format,
                            const void* pixels, size_t byteSize,
                            VkFilter filter);

    static Texture2D createArray(VkDevice device, VkPhysicalDevice phys,
                                 VkQueue queue, uint32_t queueFamily,
                                 uint32_t width, uint32_t height, uint32_t layers,
                                 VkFormat format, const void* pixels,
                                 size_t byteSize, float maxAnisotropy);

    static bool supportsLinearFilter(VkPhysicalDevice phys, VkFormat format);

    VkImageView view()    const { return view_; }
    VkSampler   sampler() const { return sampler_; }
    uint32_t    width()   const { return width_; }
    uint32_t    height()  const { return height_; }
    uint32_t    layers()  const { return layers_; }
    uint32_t    mipLevels() const { return mipLevels_; }
    bool        valid()   const { return image_ != VK_NULL_HANDLE; }

private:
    void destroy();
    void moveFrom(Texture2D& o);

    VkDevice       device_  = VK_NULL_HANDLE;
    VkImage        image_   = VK_NULL_HANDLE;
    VkDeviceMemory mem_     = VK_NULL_HANDLE;
    VkImageView    view_    = VK_NULL_HANDLE;
    VkSampler      sampler_ = VK_NULL_HANDLE;
    uint32_t       width_   = 0;
    uint32_t       height_  = 0;
    uint32_t       layers_    = 1;
    uint32_t       mipLevels_ = 1;
};

}
