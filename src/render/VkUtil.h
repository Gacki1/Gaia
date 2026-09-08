#pragma once

#include <vulkan/vulkan.h>
#include <stdexcept>
#include <string>

namespace planet {

inline void vkCheck(VkResult r, const char* what) {
    if (r != VK_SUCCESS) {
        throw std::runtime_error(std::string("Vulkan error: ") + what +
                                 " (VkResult " + std::to_string(static_cast<int>(r)) + ")");
    }
}

inline uint32_t findMemoryType(VkPhysicalDevice phys, uint32_t typeFilter,
                               VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProps{};
    vkGetPhysicalDeviceMemoryProperties(phys, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeFilter & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & properties) == properties) {
            return i;
        }
    }
    throw std::runtime_error("Vulkan error: no suitable memory type found");
}

}
