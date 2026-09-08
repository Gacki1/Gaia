#include "render/PlanetRenderer.h"
#include <chrono>
#include "world/CubeSphere.h"
#include "world/PlanetConfig.h"
#include "render/VulkanRenderer.h"
#include <algorithm>

namespace planet {

void PlanetRenderer::init(VkDevice device, VkPhysicalDevice phys, VkQueue queue,
                          uint32_t queueFamily, uint32_t framesInFlight,
                          bool threaded) {
    device_ = device;
    phys_   = phys;
    queue_  = queue;
    framesInFlight_ = framesInFlight;
    world_ = std::make_shared<const World>();

    bounds_.reset(*world_);

    VkCommandPoolCreateInfo pci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pci.queueFamilyIndex = queueFamily;
    vkCheck(vkCreateCommandPool(device_, &pci, nullptr, &transferPool_),
            "vkCreateCommandPool(transfer)");

    for (TransferBatch& b : batches_) {
        VkCommandBufferAllocateInfo cai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        cai.commandPool        = transferPool_;
        cai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        vkCheck(vkAllocateCommandBuffers(device_, &cai, &b.cmd),
                "vkAllocateCommandBuffers(transfer)");
        VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        vkCheck(vkCreateFence(device_, &fci, nullptr, &b.fence),
                "vkCreateFence(transfer)");
    }

    {
        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size        = kStagingArenaBytes;
        bi.usage       = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device_, &bi, nullptr, &staging_),
                "vkCreateBuffer(staging arena)");

        VkMemoryRequirements req{};
        vkGetBufferMemoryRequirements(device_, staging_, &req);
        VkMemoryAllocateInfo ai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        ai.allocationSize  = req.size;
        ai.memoryTypeIndex = findMemoryType(phys_, req.memoryTypeBits,
                                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device_, &ai, nullptr, &stagingMem_),
                "vkAllocateMemory(staging arena)");
        vkCheck(vkBindBufferMemory(device_, staging_, stagingMem_, 0),
                "vkBindBufferMemory(staging arena)");

        vkCheck(vkMapMemory(device_, stagingMem_, 0, req.size, 0, &stagingMapped_),
                "vkMapMemory(staging arena)");
        stagingSize_ = req.size;

        const VkDeviceSize slice = (req.size / kTransferBatches) / 16u * 16u;
        for (int i = 0; i < kTransferBatches; ++i) {
            batches_[i].base = slice * VkDeviceSize(i);
            batches_[i].size = slice;
        }
    }

    if (threaded) jobs_.start();

    for (int f = 0; f < 6; ++f) {
        JobResult r;
        r.key  = { f, 0, 0, 0 };
        r.data = generatePatch(*world_, f, 0, 0, 0, kPatchResolution);
        pendingUploads_.push_back(std::move(r));
    }

    while (!pendingUploads_.empty() || anyTransferBusy()) {
        uploadReady(0);
        flushTransfers();
    }
    for (auto& kv : cache_) kv.second.pinned = true;
}

void PlanetRenderer::shutdown() {

    jobs_.stop();
    {
        std::lock_guard<std::mutex> lock(resultsMtx_);
        results_.clear();
    }
    pendingUploads_.clear();
    inFlight_.clear();

    cache_.clear();
    retired_.clear();
    active_.clear();

    if (device_ != VK_NULL_HANDLE) {
        if (stagingMapped_) { vkUnmapMemory(device_, stagingMem_); stagingMapped_ = nullptr; }
        if (staging_)    vkDestroyBuffer(device_, staging_, nullptr);
        if (stagingMem_) vkFreeMemory(device_, stagingMem_, nullptr);
        staging_ = VK_NULL_HANDLE; stagingMem_ = VK_NULL_HANDLE; stagingSize_ = 0;
        for (TransferBatch& b : batches_) {
            if (b.fence) vkDestroyFence(device_, b.fence, nullptr);
            b.fence = VK_NULL_HANDLE;
            b.built.clear();
        }
        if (transferPool_)  vkDestroyCommandPool(device_, transferPool_, nullptr);
    }
    transferPool_ = VK_NULL_HANDLE;
}

void PlanetRenderer::flushTransfers() {
    for (TransferBatch& b : batches_) {
        if (!b.busy) continue;
        vkCheck(vkWaitForFences(device_, 1, &b.fence, VK_TRUE, UINT64_MAX),
                "vkWaitForFences(transfer flush)");
    }
    uploadReady(0);
}

bool PlanetRenderer::anyTransferBusy() const {
    for (const TransferBatch& b : batches_) if (b.busy) return true;
    return false;
}

bool PlanetRenderer::settled() const {

    if (anyTransferBusy()) return false;
    if (jobs_.pending() != 0) return false;
    {
        std::lock_guard<std::mutex> lock(resultsMtx_);
        if (!results_.empty()) return false;
    }
    return pendingUploads_.empty() && inFlight_.empty();
}

void PlanetRenderer::drainResults() {
    std::vector<JobResult> fresh;
    {
        std::lock_guard<std::mutex> lock(resultsMtx_);
        fresh.swap(results_);
    }
    for (JobResult& r : fresh) {

        if (r.generation != generation_) { ++staleResults_; continue; }
        pendingUploads_.push_back(std::move(r));
    }
}

void PlanetRenderer::uploadReady(uint64_t frameNumber) {
    msUpload_ = 0.0f;
    const auto tUp = std::chrono::steady_clock::now();

    for (TransferBatch& b : batches_) {
        if (!b.busy) continue;
        if (vkGetFenceStatus(device_, b.fence) != VK_SUCCESS) continue;
        vkCheck(vkResetFences(device_, 1, &b.fence), "vkResetFences(transfer)");
        for (auto& kv : b.built) {
            inFlight_.erase(kv.first);
            if (cache_.count(kv.first)) continue;
            CacheEntry entry;
            entry.mesh          = std::move(kv.second);
            entry.lastUsedFrame = frameNumber;
            cache_.emplace(kv.first, std::move(entry));
        }
        b.built.clear();
        b.busy = false;
    }

    if (pendingUploads_.empty()) return;

    TransferBatch* slot = nullptr;
    for (int i = 0; i < kTransferBatches; ++i) {
        TransferBatch& b = batches_[(nextBatch_ + i) % kTransferBatches];
        if (!b.busy) { slot = &b; nextBatch_ = (nextBatch_ + i + 1) % kTransferBatches; break; }
    }
    if (!slot) return;

    vkCheck(vkResetCommandBuffer(slot->cmd, 0), "vkResetCommandBuffer(transfer)");
    VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkCheck(vkBeginCommandBuffer(slot->cmd, &bi), "vkBeginCommandBuffer(transfer)");

    VkDeviceSize arenaOffset = slot->base;
    const VkDeviceSize arenaEnd = slot->base + slot->size;

    int budget = kMaxUploadsPerFrame;
    while (budget > 0 && !pendingUploads_.empty()) {

        if (std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - tUp).count() >= kUploadBudgetMs)
            break;

        JobResult r = std::move(pendingUploads_.front());
        pendingUploads_.pop_front();
        if (cache_.count(r.key)) { inFlight_.erase(r.key); continue; }

        Mesh mesh = Mesh::createFromArena(device_, phys_, r.data.vertices,
                                          r.data.indices, slot->cmd,
                                          staging_, stagingMapped_, arenaEnd,
                                          arenaOffset);
        if (!mesh.valid()) {

            pendingUploads_.push_front(std::move(r));
            break;
        }
        slot->built.emplace_back(r.key, std::move(mesh));
        --budget;
    }

    if (slot->built.empty()) {
        vkCheck(vkEndCommandBuffer(slot->cmd), "vkEndCommandBuffer(transfer)");
        return;
    }

    VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
    mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
    vkCmdPipelineBarrier(slot->cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_VERTEX_INPUT_BIT, 0,
                         1, &mb, 0, nullptr, 0, nullptr);

    vkCheck(vkEndCommandBuffer(slot->cmd), "vkEndCommandBuffer(transfer)");

    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &slot->cmd;
    vkCheck(vkQueueSubmit(queue_, 1, &si, slot->fence), "vkQueueSubmit(transfer)");
    slot->busy = true;

    msUpload_ = float(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - tUp).count());
}

void PlanetRenderer::requestPatches(const std::vector<PatchKey>& wanted) {
    int budget = kMaxJobsPerFrame;
    for (const PatchKey& key : wanted) {
        if (budget <= 0) break;
        if (inFlight_.size() >= static_cast<size_t>(kMaxJobsInFlight)) break;
        if (cache_.count(key) || inFlight_.count(key)) continue;

        inFlight_.insert(key);
        --budget;

        auto snapshot = world_;
        const uint32_t gen = generation_;
        jobs_.enqueue([this, key, snapshot, gen] {

            JobResult r;
            r.key  = key;
            r.generation = gen;
            r.data = generatePatch(*snapshot, key.face, key.level, key.ix,
                                   key.iy, kPatchResolution);
            std::lock_guard<std::mutex> lock(resultsMtx_);
            results_.push_back(std::move(r));
        });
    }
}

void PlanetRenderer::setParams(const PlanetParams& params, uint64_t frameNumber) {
    const ParamChange change = classifyChange(world_->p, params);
    if (change == ParamChange::None) return;

    world_ = std::make_shared<const World>(params);

    if (change == ParamChange::ShadingOnly) {

        bounds_.repoint(*world_);
        return;
    }

    ++generation_;
    bounds_.reset(*world_);

    for (auto& kv : cache_)
        retired_.push_back({ std::move(kv.second.mesh), frameNumber });
    cache_.clear();
    active_.clear();

    pendingUploads_.clear();
    inFlight_.clear();

    for (int f = 0; f < 6; ++f) {
        JobResult r;
        r.key  = { f, 0, 0, 0 };
        r.generation = generation_;
        r.data = generatePatch(*world_, f, 0, 0, 0, kPatchResolution);
        pendingUploads_.push_back(std::move(r));
    }
    while (!pendingUploads_.empty() || anyTransferBusy()) {
        uploadReady(frameNumber);
        flushTransfers();
    }
    for (auto& kv : cache_) kv.second.pinned = true;

    update(lastCamera_, frameNumber);
}

void PlanetRenderer::evictAndCollect(uint64_t frameNumber) {

    const uint64_t safeAge = framesInFlight_ + 1;
    retired_.erase(std::remove_if(retired_.begin(), retired_.end(),
        [&](Retired& r) { return frameNumber - r.retiredFrame >= safeAge; }),
        retired_.end());

    if (cache_.size() <= static_cast<size_t>(kMeshCacheCapacity)) return;

    std::unordered_set<PatchKey, PatchKeyHash> activeTree;
    for (const ActivePatch& ap : active_) {
        PatchKey a = ap.key;
        while (a.level > 0) {
            a = PatchKey{ a.face, a.level - 1, a.ix >> 1, a.iy >> 1 };
            if (!activeTree.insert(a).second) break;
        }
    }

    std::vector<std::pair<uint64_t, PatchKey>> victims;
    victims.reserve(cache_.size());
    for (auto& kv : cache_)
        if (!kv.second.pinned && kv.second.lastUsedFrame != frameNumber)
            victims.push_back({ kv.second.lastUsedFrame, kv.first });
    std::sort(victims.begin(), victims.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    size_t toRemove = cache_.size() - static_cast<size_t>(kMeshCacheCapacity);
    for (size_t i = 0; i < victims.size() && toRemove > 0; ++i, --toRemove) {
        auto it = cache_.find(victims[i].second);
        if (it == cache_.end()) continue;
        if (activeTree.count(it->first)) ++evictedAncestors_;
        retired_.push_back({ std::move(it->second.mesh), frameNumber });
        cache_.erase(it);
    }
}

void PlanetRenderer::update(const DVec3& cameraPos, uint64_t frameNumber) {
    frameNumber_ = frameNumber;
    lastCamera_  = cameraPos;

    drainResults();
    uploadReady(frameNumber);

    std::vector<PatchKey> wanted;
    const auto tSel = std::chrono::steady_clock::now();
    const std::vector<SelectedPatch> selected = selectLODAdaptive(
        cameraPos,
        bounds_,
        [this](const PatchKey& k) { return resident(k); },
        wanted);

    msSelect_ = float(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - tSel).count());

    active_.clear();
    active_.reserve(selected.size());
    for (const SelectedPatch& s : selected) {
        auto it = cache_.find(s.key);
        if (it == cache_.end()) continue;
        it->second.lastUsedFrame = frameNumber;

        PatchKey a = s.key;
        while (a.level > 0) {
            a = PatchKey{ a.face, a.level - 1, a.ix >> 1, a.iy >> 1 };
            auto anc = cache_.find(a);
            if (anc == cache_.end()) break;
            if (anc->second.lastUsedFrame == frameNumber) break;
            anc->second.lastUsedFrame = frameNumber;
        }

        const Vec3 offset = space::toF(
            patchOrigin(s.key.face, s.key.level, s.key.ix, s.key.iy) - cameraPos);

        const float morphEnd = s.key.level > 0
            ? static_cast<float>(2.0 * s.worldEdge /
                                 double(splitFactorAt(space::length(cameraPos))))
            : 0.0f;

        const DVec3 po = patchOrigin(s.key.face, s.key.level, s.key.ix, s.key.iy);
        const Vec3 texOrigin = space::toF(groundTexOrigin(world_->p, po));
        active_.push_back({ s.key, relativeAABB(s.aabb, cameraPos), offset, morphEnd,
                            texOrigin });
    }

    requestPatches(wanted);
    evictAndCollect(frameNumber);
}

void PlanetRenderer::recordDraw(VkCommandBuffer cmd, VkPipelineLayout layout,
                                const Frustum& frustum, PlanetStats& stats) {
    stats = PlanetStats{};
    stats.activePatches = static_cast<uint32_t>(active_.size());
    stats.cachedMeshes  = static_cast<uint32_t>(cache_.size());
    stats.pendingJobs   = static_cast<uint32_t>(inFlight_.size());
    stats.evictedAncestors = evictedAncestors_;
    stats.msSelect = msSelect_;
    stats.msUpload = msUpload_;

    for (const ActivePatch& ap : active_) {
        if (!frustum.intersectsAABB(ap.aabb)) {
            ++stats.culledPatches;
            continue;
        }
        auto it = cache_.find(ap.key);
        if (it == cache_.end() || !it->second.mesh.valid()) continue;

        const float patchPush[16] = { ap.offset.x, ap.offset.y, ap.offset.z,
                                      static_cast<float>(ap.key.level),
                                      0.0f, 0.0f, 0.0f, 1.0f,
                                      0.0f, 0.0f, 1.0f, ap.morphEnd,
                                      ap.texOrigin.x, ap.texOrigin.y,
                                      ap.texOrigin.z, 0.0f };
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                           kPatchPushOffset, kPatchPushSize, patchPush);

        it->second.mesh.draw(cmd);
        ++stats.drawnPatches;
        stats.triangles += it->second.mesh.triangleCount();
    }
}

}
