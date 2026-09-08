#include "render/Mesh.h"
#include <cstring>

namespace planet {

namespace {

void makeBuffer(VkDevice device, VkPhysicalDevice phys, VkDeviceSize size,
                VkBufferUsageFlags usage, VkMemoryPropertyFlags props,
                VkBuffer& buf, VkDeviceMemory& mem) {
    VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bi.size        = size;
    bi.usage       = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCheck(vkCreateBuffer(device, &bi, nullptr, &buf), "vkCreateBuffer");

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(device, buf, &req);

    VkMemoryAllocateInfo ai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    ai.allocationSize  = req.size;
    ai.memoryTypeIndex = findMemoryType(phys, req.memoryTypeBits, props);
    vkCheck(vkAllocateMemory(device, &ai, nullptr, &mem), "vkAllocateMemory");
    vkCheck(vkBindBufferMemory(device, buf, mem, 0), "vkBindBufferMemory");
}
}

Mesh Mesh::create(VkDevice device, VkPhysicalDevice phys,
                  const std::vector<Vertex>& vertices,
                  const std::vector<uint16_t>& indices,
                  VkCommandBuffer cmd, Staging& staging) {
    Mesh m;
    m.device_      = device;
    m.indexCount_  = static_cast<uint32_t>(indices.size());
    m.vertexCount_ = static_cast<uint32_t>(vertices.size());
    if (vertices.empty() || indices.empty()) return m;

    vkCheck(vertices.size() <= 65536u ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED,
            "Mesh::create: more than 65536 vertices cannot be indexed by uint16");

    const VkDeviceSize vBytes = sizeof(Vertex)   * vertices.size();
    const VkDeviceSize iBytes = sizeof(uint16_t) * indices.size();

    const VkDeviceSize kIndexAlign = sizeof(uint32_t);
    m.indexOffset_ = (vBytes + kIndexAlign - 1) / kIndexAlign * kIndexAlign;
    const VkDeviceSize total = m.indexOffset_ + iBytes;

    staging.device_ = device;
    makeBuffer(device, phys, total, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
               VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               staging.buf_, staging.mem_);

    void* mapped = nullptr;
    vkCheck(vkMapMemory(device, staging.mem_, 0, total, 0, &mapped), "vkMapMemory");
    std::memcpy(static_cast<char*>(mapped), vertices.data(),
                static_cast<size_t>(vBytes));
    std::memcpy(static_cast<char*>(mapped) + m.indexOffset_, indices.data(),
                static_cast<size_t>(iBytes));
    vkUnmapMemory(device, staging.mem_);

    makeBuffer(device, phys, total,
               VK_BUFFER_USAGE_TRANSFER_DST_BIT |
               VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
               VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
               m.buf_, m.mem_);

    VkBufferCopy region{};
    region.size = total;
    vkCmdCopyBuffer(cmd, staging.buf_, m.buf_, 1, &region);
    return m;
}

Mesh Mesh::createFromArena(VkDevice device, VkPhysicalDevice phys,
                           const std::vector<Vertex>& vertices,
                           const std::vector<uint16_t>& indices,
                           VkCommandBuffer cmd,
                           VkBuffer arena, void* arenaMapped,
                           VkDeviceSize arenaSize, VkDeviceSize& arenaOffset) {
    Mesh m;
    m.device_      = device;
    m.indexCount_  = static_cast<uint32_t>(indices.size());
    m.vertexCount_ = static_cast<uint32_t>(vertices.size());
    if (vertices.empty() || indices.empty()) return m;
    vkCheck(vertices.size() <= 65536u ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED,
            "Mesh::createFromArena: more than 65536 vertices cannot be indexed by uint16");

    const VkDeviceSize vBytes = sizeof(Vertex)   * vertices.size();
    const VkDeviceSize iBytes = sizeof(uint16_t) * indices.size();
    const VkDeviceSize kIndexAlign = sizeof(uint32_t);
    m.indexOffset_ = (vBytes + kIndexAlign - 1) / kIndexAlign * kIndexAlign;
    const VkDeviceSize total = m.indexOffset_ + iBytes;

    const VkDeviceSize base = (arenaOffset + 15u) / 16u * 16u;
    if (base + total > arenaSize) {
        Mesh none;
        none.device_ = device;
        return none;
    }

    char* dst = static_cast<char*>(arenaMapped) + base;
    std::memcpy(dst, vertices.data(), static_cast<size_t>(vBytes));
    std::memcpy(dst + m.indexOffset_, indices.data(), static_cast<size_t>(iBytes));

    makeBuffer(device, phys, total,
               VK_BUFFER_USAGE_TRANSFER_DST_BIT |
               VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
               VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, m.buf_, m.mem_);

    VkBufferCopy region{};
    region.srcOffset = base;
    region.dstOffset = 0;
    region.size      = total;
    vkCmdCopyBuffer(cmd, arena, m.buf_, 1, &region);

    arenaOffset = base + total;
    return m;
}

Mesh Mesh::createImmediate(VkDevice device, VkPhysicalDevice phys,
                           VkQueue queue, uint32_t queueFamily,
                           const std::vector<Vertex>& vertices,
                           const std::vector<uint16_t>& indices) {
    VkCommandPoolCreateInfo pci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pci.flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    pci.queueFamilyIndex = queueFamily;
    VkCommandPool pool = VK_NULL_HANDLE;
    vkCheck(vkCreateCommandPool(device, &pci, nullptr, &pool),
            "vkCreateCommandPool(immediate)");

    VkCommandBufferAllocateInfo cai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cai.commandPool        = pool;
    cai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkCheck(vkAllocateCommandBuffers(device, &cai, &cmd),
            "vkAllocateCommandBuffers(immediate)");

    VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkCheck(vkBeginCommandBuffer(cmd, &bi), "vkBeginCommandBuffer(immediate)");

    Staging staging;
    Mesh m = create(device, phys, vertices, indices, cmd, staging);

    VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
    mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_VERTEX_INPUT_BIT, 0,
                         1, &mb, 0, nullptr, 0, nullptr);
    vkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(immediate)");

    VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    VkFence fence = VK_NULL_HANDLE;
    vkCheck(vkCreateFence(device, &fci, nullptr, &fence), "vkCreateFence(immediate)");

    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkCheck(vkQueueSubmit(queue, 1, &si, fence), "vkQueueSubmit(immediate)");
    vkCheck(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX),
            "vkWaitForFences(immediate)");

    vkDestroyFence(device, fence, nullptr);
    vkDestroyCommandPool(device, pool, nullptr);
    return m;
}

void Mesh::Staging::destroy() {
    if (device_ != VK_NULL_HANDLE) {
        if (buf_) vkDestroyBuffer(device_, buf_, nullptr);
        if (mem_) vkFreeMemory(device_, mem_, nullptr);
    }
    device_ = VK_NULL_HANDLE;
    buf_ = VK_NULL_HANDLE;
    mem_ = VK_NULL_HANDLE;
}

void Mesh::Staging::moveFrom(Staging& o) {
    device_ = o.device_; buf_ = o.buf_; mem_ = o.mem_;
    o.device_ = VK_NULL_HANDLE; o.buf_ = VK_NULL_HANDLE; o.mem_ = VK_NULL_HANDLE;
}

void Mesh::draw(VkCommandBuffer cmd) const {
    if (buf_ == VK_NULL_HANDLE || indexCount_ == 0) return;
    VkBuffer     bufs[] = { buf_ };
    VkDeviceSize offs[] = { 0 };
    vkCmdBindVertexBuffers(cmd, 0, 1, bufs, offs);
    vkCmdBindIndexBuffer(cmd, buf_, indexOffset_, VK_INDEX_TYPE_UINT16);
    vkCmdDrawIndexed(cmd, indexCount_, 1, 0, 0, 0);
}

void Mesh::destroy() {
    if (device_ != VK_NULL_HANDLE) {
        if (buf_) vkDestroyBuffer(device_, buf_, nullptr);
        if (mem_) vkFreeMemory(device_, mem_, nullptr);
    }
    device_ = VK_NULL_HANDLE;
    buf_ = VK_NULL_HANDLE;
    mem_ = VK_NULL_HANDLE;
    indexOffset_ = 0;
    indexCount_ = vertexCount_ = 0;
}

void Mesh::moveFrom(Mesh& o) {
    device_      = o.device_;
    buf_         = o.buf_;
    mem_         = o.mem_;
    indexOffset_ = o.indexOffset_;
    indexCount_  = o.indexCount_;
    vertexCount_ = o.vertexCount_;
    o.device_ = VK_NULL_HANDLE;
    o.buf_ = VK_NULL_HANDLE;
    o.mem_ = VK_NULL_HANDLE;
    o.indexOffset_ = 0;
    o.indexCount_ = o.vertexCount_ = 0;
}

VkVertexInputBindingDescription Mesh::bindingDescription() {
    VkVertexInputBindingDescription b{};
    b.binding   = 0;
    b.stride    = sizeof(Vertex);
    b.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return b;
}

std::array<VkVertexInputAttributeDescription, 7> Mesh::attributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 7> a{};

    for (auto& d : a) d.binding = 0;
    a[0].location = 0; a[0].format = VK_FORMAT_R32G32B32_SFLOAT; a[0].offset = offsetof(Vertex, px);
    a[1].location = 1; a[1].format = VK_FORMAT_R16G16_SNORM;     a[1].offset = offsetof(Vertex, nx);
    a[2].location = 2; a[2].format = VK_FORMAT_R8G8B8A8_UNORM;   a[2].offset = offsetof(Vertex, r);
    a[3].location = 3; a[3].format = VK_FORMAT_R32G32B32_SFLOAT; a[3].offset = offsetof(Vertex, cx);
    a[4].location = 4; a[4].format = VK_FORMAT_R16G16_SNORM;     a[4].offset = offsetof(Vertex, cnx);
    a[5].location = 5; a[5].format = VK_FORMAT_R8G8B8A8_UNORM;   a[5].offset = offsetof(Vertex, m0);
    a[6].location = 6; a[6].format = VK_FORMAT_R8G8B8A8_UNORM;   a[6].offset = offsetof(Vertex, m4);
    return a;
}

}
