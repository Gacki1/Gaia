#include "render/Texture.h"
#include <cstring>
#include <vector>

namespace planet {

bool Texture2D::supportsLinearFilter(VkPhysicalDevice phys, VkFormat format) {
    VkFormatProperties fp{};
    vkGetPhysicalDeviceFormatProperties(phys, format, &fp);
    return (fp.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT) != 0;
}

Texture2D Texture2D::create(VkDevice device, VkPhysicalDevice phys,
                            VkQueue queue, uint32_t queueFamily,
                            uint32_t width, uint32_t height, VkFormat format,
                            const void* pixels, size_t byteSize,
                            VkFilter filter) {
    Texture2D t;
    t.device_ = device;
    t.width_  = width;
    t.height_ = height;

    VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = format;
    ici.extent        = { width, height, 1 };
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    vkCheck(vkCreateImage(device, &ici, nullptr, &t.image_), "vkCreateImage(texture)");

    VkMemoryRequirements mr{};
    vkGetImageMemoryRequirements(device, t.image_, &mr);
    VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    mai.allocationSize  = mr.size;
    mai.memoryTypeIndex = findMemoryType(phys, mr.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkCheck(vkAllocateMemory(device, &mai, nullptr, &t.mem_), "vkAllocateMemory(texture)");
    vkCheck(vkBindImageMemory(device, t.image_, t.mem_, 0), "vkBindImageMemory(texture)");

    VkBuffer       stageBuf = VK_NULL_HANDLE;
    VkDeviceMemory stageMem = VK_NULL_HANDLE;
    {
        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size        = byteSize;
        bi.usage       = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device, &bi, nullptr, &stageBuf), "vkCreateBuffer(tex staging)");

        VkMemoryRequirements smr{};
        vkGetBufferMemoryRequirements(device, stageBuf, &smr);
        VkMemoryAllocateInfo smai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        smai.allocationSize  = smr.size;
        smai.memoryTypeIndex = findMemoryType(phys, smr.memoryTypeBits,
                                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device, &smai, nullptr, &stageMem),
                "vkAllocateMemory(tex staging)");
        vkCheck(vkBindBufferMemory(device, stageBuf, stageMem, 0),
                "vkBindBufferMemory(tex staging)");

        void* mapped = nullptr;
        vkCheck(vkMapMemory(device, stageMem, 0, byteSize, 0, &mapped),
                "vkMapMemory(tex staging)");
        std::memcpy(mapped, pixels, byteSize);
        vkUnmapMemory(device, stageMem);
    }

    VkCommandPoolCreateInfo pci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pci.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pci.queueFamilyIndex = queueFamily;
    VkCommandPool pool = VK_NULL_HANDLE;
    vkCheck(vkCreateCommandPool(device, &pci, nullptr, &pool), "vkCreateCommandPool(texture)");

    VkCommandBufferAllocateInfo cai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cai.commandPool        = pool;
    cai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkCheck(vkAllocateCommandBuffers(device, &cai, &cmd), "vkAllocateCommandBuffers(texture)");

    VkCommandBufferBeginInfo bbi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkCheck(vkBeginCommandBuffer(cmd, &bbi), "vkBeginCommandBuffer(texture)");

    VkImageMemoryBarrier bar{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bar.image               = t.image_;
    bar.subresourceRange    = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };

    bar.oldLayout     = VK_IMAGE_LAYOUT_UNDEFINED;
    bar.newLayout     = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    bar.srcAccessMask = 0;
    bar.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
                         0, nullptr, 0, nullptr, 1, &bar);

    VkBufferImageCopy region{};
    region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
    region.imageExtent      = { width, height, 1 };
    vkCmdCopyBufferToImage(cmd, stageBuf, t.image_,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    bar.oldLayout     = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    bar.newLayout     = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    bar.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    bar.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
                         0, nullptr, 0, nullptr, 1, &bar);

    vkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(texture)");

    VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    VkFence fence = VK_NULL_HANDLE;
    vkCheck(vkCreateFence(device, &fci, nullptr, &fence), "vkCreateFence(texture)");

    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkCheck(vkQueueSubmit(queue, 1, &si, fence), "vkQueueSubmit(texture)");
    vkCheck(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX),
            "vkWaitForFences(texture)");

    vkDestroyFence(device, fence, nullptr);
    vkDestroyCommandPool(device, pool, nullptr);
    vkDestroyBuffer(device, stageBuf, nullptr);
    vkFreeMemory(device, stageMem, nullptr);

    VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    vci.image            = t.image_;
    vci.viewType         = VK_IMAGE_VIEW_TYPE_2D;
    vci.format           = format;
    vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
    vkCheck(vkCreateImageView(device, &vci, nullptr, &t.view_), "vkCreateImageView(texture)");

    VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    sci.magFilter    = filter;
    sci.minFilter    = filter;
    sci.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.borderColor  = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
    vkCheck(vkCreateSampler(device, &sci, nullptr, &t.sampler_), "vkCreateSampler(texture)");

    return t;
}

Texture2D Texture2D::createArray(VkDevice device, VkPhysicalDevice phys,
                                 VkQueue queue, uint32_t queueFamily,
                                 uint32_t width, uint32_t height, uint32_t layers,
                                 VkFormat format, const void* pixels,
                                 size_t byteSize, float maxAnisotropy) {
    Texture2D t;
    t.device_ = device;
    t.width_  = width;
    t.height_ = height;
    t.layers_ = layers;

    uint32_t mips = 1;
    for (uint32_t d = (width > height ? width : height); d > 1; d >>= 1) ++mips;
    t.mipLevels_ = mips;

    VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = format;
    ici.extent        = { width, height, 1 };
    ici.mipLevels     = mips;
    ici.arrayLayers   = layers;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;

    ici.usage         = VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                        VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                        VK_IMAGE_USAGE_SAMPLED_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    vkCheck(vkCreateImage(device, &ici, nullptr, &t.image_), "vkCreateImage(tex array)");

    VkMemoryRequirements mr{};
    vkGetImageMemoryRequirements(device, t.image_, &mr);
    VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    mai.allocationSize  = mr.size;
    mai.memoryTypeIndex = findMemoryType(phys, mr.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkCheck(vkAllocateMemory(device, &mai, nullptr, &t.mem_), "vkAllocateMemory(tex array)");
    vkCheck(vkBindImageMemory(device, t.image_, t.mem_, 0), "vkBindImageMemory(tex array)");

    VkBuffer       stageBuf = VK_NULL_HANDLE;
    VkDeviceMemory stageMem = VK_NULL_HANDLE;
    {
        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size        = byteSize;
        bi.usage       = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device, &bi, nullptr, &stageBuf), "vkCreateBuffer(array staging)");
        VkMemoryRequirements smr{};
        vkGetBufferMemoryRequirements(device, stageBuf, &smr);
        VkMemoryAllocateInfo smai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        smai.allocationSize  = smr.size;
        smai.memoryTypeIndex = findMemoryType(phys, smr.memoryTypeBits,
                                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device, &smai, nullptr, &stageMem),
                "vkAllocateMemory(array staging)");
        vkCheck(vkBindBufferMemory(device, stageBuf, stageMem, 0),
                "vkBindBufferMemory(array staging)");
        void* mapped = nullptr;
        vkCheck(vkMapMemory(device, stageMem, 0, byteSize, 0, &mapped),
                "vkMapMemory(array staging)");
        std::memcpy(mapped, pixels, byteSize);
        vkUnmapMemory(device, stageMem);
    }

    VkCommandPoolCreateInfo pci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pci.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pci.queueFamilyIndex = queueFamily;
    VkCommandPool pool = VK_NULL_HANDLE;
    vkCheck(vkCreateCommandPool(device, &pci, nullptr, &pool), "vkCreateCommandPool(array)");
    VkCommandBufferAllocateInfo cai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cai.commandPool        = pool;
    cai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkCheck(vkAllocateCommandBuffers(device, &cai, &cmd), "vkAllocateCommandBuffers(array)");
    VkCommandBufferBeginInfo bbi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkCheck(vkBeginCommandBuffer(cmd, &bbi), "vkBeginCommandBuffer(array)");

    VkImageMemoryBarrier bar{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bar.image               = t.image_;

    bar.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, layers };
    bar.oldLayout        = VK_IMAGE_LAYOUT_UNDEFINED;
    bar.newLayout        = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    bar.srcAccessMask    = 0;
    bar.dstAccessMask    = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
                         0, nullptr, 0, nullptr, 1, &bar);

    const VkDeviceSize layerBytes = VkDeviceSize(byteSize) / layers;
    std::vector<VkBufferImageCopy> regions(layers);
    for (uint32_t i = 0; i < layers; ++i) {
        regions[i] = {};
        regions[i].bufferOffset      = layerBytes * i;
        regions[i].imageSubresource  = { VK_IMAGE_ASPECT_COLOR_BIT, 0, i, 1 };
        regions[i].imageExtent       = { width, height, 1 };
    }
    vkCmdCopyBufferToImage(cmd, stageBuf, t.image_,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           layers, regions.data());

    int32_t mw = int32_t(width), mh = int32_t(height);
    for (uint32_t level = 1; level < mips; ++level) {
        bar.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 1, 0, layers };
        bar.oldLayout        = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        bar.newLayout        = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        bar.srcAccessMask    = VK_ACCESS_TRANSFER_WRITE_BIT;
        bar.dstAccessMask    = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
                             0, nullptr, 0, nullptr, 1, &bar);

        const int32_t nw = mw > 1 ? mw / 2 : 1;
        const int32_t nh = mh > 1 ? mh / 2 : 1;
        VkImageBlit blit{};
        blit.srcOffsets[1]  = { mw, mh, 1 };
        blit.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, layers };
        blit.dstOffsets[1]  = { nw, nh, 1 };
        blit.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, level,     0, layers };
        vkCmdBlitImage(cmd,
                       t.image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       t.image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       1, &blit, VK_FILTER_LINEAR);

        bar.oldLayout     = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        bar.newLayout     = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        bar.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        bar.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
                             0, nullptr, 0, nullptr, 1, &bar);
        mw = nw; mh = nh;
    }

    bar.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, mips - 1, 1, 0, layers };
    bar.oldLayout        = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    bar.newLayout        = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    bar.srcAccessMask    = VK_ACCESS_TRANSFER_WRITE_BIT;
    bar.dstAccessMask    = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
                         0, nullptr, 0, nullptr, 1, &bar);

    vkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(array)");
    VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    VkFence fence = VK_NULL_HANDLE;
    vkCheck(vkCreateFence(device, &fci, nullptr, &fence), "vkCreateFence(array)");
    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkCheck(vkQueueSubmit(queue, 1, &si, fence), "vkQueueSubmit(array)");
    vkCheck(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX),
            "vkWaitForFences(array)");
    vkDestroyFence(device, fence, nullptr);
    vkDestroyCommandPool(device, pool, nullptr);
    vkDestroyBuffer(device, stageBuf, nullptr);
    vkFreeMemory(device, stageMem, nullptr);

    VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    vci.image            = t.image_;
    vci.viewType         = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    vci.format           = format;
    vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, mips, 0, layers };
    vkCheck(vkCreateImageView(device, &vci, nullptr, &t.view_), "vkCreateImageView(array)");

    VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    sci.magFilter        = VK_FILTER_LINEAR;
    sci.minFilter        = VK_FILTER_LINEAR;
    sci.mipmapMode       = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sci.addressModeU     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.addressModeV     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.addressModeW     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.minLod           = 0.0f;
    sci.maxLod           = float(mips);
    sci.anisotropyEnable = maxAnisotropy > 1.0f ? VK_TRUE : VK_FALSE;
    sci.maxAnisotropy    = maxAnisotropy > 1.0f ? maxAnisotropy : 1.0f;
    sci.borderColor      = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
    vkCheck(vkCreateSampler(device, &sci, nullptr, &t.sampler_), "vkCreateSampler(array)");

    return t;
}

void Texture2D::destroy() {
    if (device_ == VK_NULL_HANDLE) return;
    if (sampler_) vkDestroySampler(device_, sampler_, nullptr);
    if (view_)    vkDestroyImageView(device_, view_, nullptr);
    if (image_)   vkDestroyImage(device_, image_, nullptr);
    if (mem_)     vkFreeMemory(device_, mem_, nullptr);
    sampler_ = VK_NULL_HANDLE; view_ = VK_NULL_HANDLE;
    image_   = VK_NULL_HANDLE; mem_  = VK_NULL_HANDLE;
    device_  = VK_NULL_HANDLE;
    width_ = height_ = 0;
}

void Texture2D::moveFrom(Texture2D& o) {
    device_ = o.device_; image_ = o.image_; mem_ = o.mem_;
    view_   = o.view_;   sampler_ = o.sampler_;
    width_  = o.width_;  height_  = o.height_;
    layers_ = o.layers_; mipLevels_ = o.mipLevels_;
    o.device_ = VK_NULL_HANDLE; o.image_ = VK_NULL_HANDLE; o.mem_ = VK_NULL_HANDLE;
    o.view_   = VK_NULL_HANDLE; o.sampler_ = VK_NULL_HANDLE;
    o.width_  = 0; o.height_ = 0; o.layers_ = 1; o.mipLevels_ = 1;
}

}
