#pragma once

#include "render/Vertex.h"
#include "render/VkUtil.h"
#include <vulkan/vulkan.h>
#include <vector>
#include <array>
#include <cstdint>

namespace planet {

class Mesh {
public:
    Mesh() = default;
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& o) noexcept { moveFrom(o); }
    Mesh& operator=(Mesh&& o) noexcept { if (this != &o) { destroy(); moveFrom(o); } return *this; }
    ~Mesh() { destroy(); }

    class Staging {
    public:
        Staging() = default;
        Staging(const Staging&) = delete;
        Staging& operator=(const Staging&) = delete;
        Staging(Staging&& o) noexcept { moveFrom(o); }
        Staging& operator=(Staging&& o) noexcept { if (this != &o) { destroy(); moveFrom(o); } return *this; }
        ~Staging() { destroy(); }
    private:
        friend class Mesh;
        void destroy();
        void moveFrom(Staging& o);
        VkDevice       device_ = VK_NULL_HANDLE;
        VkBuffer       buf_    = VK_NULL_HANDLE;
        VkDeviceMemory mem_    = VK_NULL_HANDLE;
    };

    static Mesh create(VkDevice device, VkPhysicalDevice phys,
                       const std::vector<Vertex>& vertices,
                       const std::vector<uint16_t>& indices,
                       VkCommandBuffer cmd, Staging& staging);

    static Mesh createFromArena(VkDevice device, VkPhysicalDevice phys,
                                const std::vector<Vertex>& vertices,
                                const std::vector<uint16_t>& indices,
                                VkCommandBuffer cmd,
                                VkBuffer arena, void* arenaMapped,
                                VkDeviceSize arenaSize, VkDeviceSize& arenaOffset);

    static Mesh createImmediate(VkDevice device, VkPhysicalDevice phys,
                                VkQueue queue, uint32_t queueFamily,
                                const std::vector<Vertex>& vertices,
                                const std::vector<uint16_t>& indices);

    void draw(VkCommandBuffer cmd) const;

    uint32_t indexCount()    const { return indexCount_; }
    uint32_t triangleCount() const { return indexCount_ / 3u; }
    uint32_t vertexCount()   const { return vertexCount_; }
    bool     valid()         const { return buf_ != VK_NULL_HANDLE; }

    static VkVertexInputBindingDescription bindingDescription();
    static std::array<VkVertexInputAttributeDescription, 7> attributeDescriptions();

private:
    void destroy();
    void moveFrom(Mesh& o);

    VkDevice        device_ = VK_NULL_HANDLE;
    VkBuffer        buf_    = VK_NULL_HANDLE;
    VkDeviceMemory  mem_    = VK_NULL_HANDLE;
    VkDeviceSize    indexOffset_ = 0;
    uint32_t        indexCount_ = 0;
    uint32_t        vertexCount_ = 0;
};

}
