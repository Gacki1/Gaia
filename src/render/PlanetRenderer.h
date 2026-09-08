#pragma once

#include "render/Mesh.h"
#include "world/World.h"
#include "world/Quadtree.h"
#include "math/Mat4.h"
#include "platform/JobSystem.h"
#include "world/CubeSphere.h"
#include <vulkan/vulkan.h>
#include <vector>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

namespace planet {

struct PlanetStats {
    uint32_t activePatches = 0;
    uint32_t drawnPatches  = 0;
    uint32_t culledPatches = 0;
    uint64_t triangles     = 0;
    uint32_t cachedMeshes  = 0;
    uint32_t pendingJobs   = 0;

    uint32_t evictedAncestors = 0;

    float    msSelect  = 0.0f;
    float    msUpload  = 0.0f;
};

class PlanetRenderer {
public:

    void init(VkDevice device, VkPhysicalDevice phys, VkQueue queue,
              uint32_t queueFamily, uint32_t framesInFlight, bool threaded = true);
    void shutdown();

    void update(const DVec3& cameraPos, uint64_t frameNumber);

    void recordDraw(VkCommandBuffer cmd, VkPipelineLayout layout,
                    const Frustum& frustum, PlanetStats& stats);

    bool settled() const;

    const World& world() const { return *world_; }

    void setParams(const PlanetParams& params, uint64_t frameNumber);
    uint32_t generation() const { return generation_; }

private:
    struct CacheEntry {
        Mesh     mesh;
        uint64_t lastUsedFrame = 0;
        bool     pinned = false;
    };
    struct Retired {
        Mesh     mesh;
        uint64_t retiredFrame = 0;
    };
    struct JobResult {
        PatchKey      key;
        PatchMeshData data;

        uint32_t      generation = 0;
    };

    void drainResults();
    void uploadReady(uint64_t frameNumber);
    void requestPatches(const std::vector<PatchKey>& wanted);
    void evictAndCollect(uint64_t frameNumber);
    void flushTransfers();
    bool anyTransferBusy() const;
    bool resident(const PatchKey& key) const { return cache_.count(key) != 0; }

    VkDevice         device_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_   = VK_NULL_HANDLE;
    uint32_t         framesInFlight_ = 2;

    struct TransferBatch {
        VkCommandBuffer cmd   = VK_NULL_HANDLE;
        VkFence         fence = VK_NULL_HANDLE;
        bool            busy  = false;
        VkDeviceSize    base  = 0;
        VkDeviceSize    size  = 0;
        std::vector<std::pair<PatchKey, Mesh>> built;
    };
    VkQueue         queue_        = VK_NULL_HANDLE;
    VkCommandPool   transferPool_ = VK_NULL_HANDLE;
    TransferBatch   batches_[kTransferBatches];
    int             nextBatch_    = 0;

    std::shared_ptr<const World> world_ = std::make_shared<const World>();

    uint32_t generation_ = 0;
    PatchBoundsCache bounds_{ *world_ };

    VkBuffer       staging_       = VK_NULL_HANDLE;
    VkDeviceMemory stagingMem_    = VK_NULL_HANDLE;
    void*          stagingMapped_ = nullptr;
    VkDeviceSize   stagingSize_   = 0;

    JobSystem  jobs_;

    mutable std::mutex     resultsMtx_;
    std::vector<JobResult> results_;

    std::deque<JobResult>                      pendingUploads_;
    std::unordered_set<PatchKey, PatchKeyHash> inFlight_;

    struct ActivePatch { PatchKey key; AABB aabb; Vec3 offset; float morphEnd;
                         Vec3 texOrigin; };
    std::vector<ActivePatch> active_;

    std::unordered_map<PatchKey, CacheEntry, PatchKeyHash> cache_;
    std::vector<Retired> retired_;

    DVec3    lastCamera_{ 0.0, 0.0, 0.0 };
    uint64_t frameNumber_ = 0;
    uint32_t evictedAncestors_ = 0;
    uint32_t staleResults_ = 0;
    float    msSelect_ = 0.0f;
    float    msUpload_ = 0.0f;
};

}
