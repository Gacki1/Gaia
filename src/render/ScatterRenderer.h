#pragma once

#include "render/Mesh.h"
#include "ship/ShipMesh.h"
#include "world/Scatter.h"
#include "world/Outpost.h"
#include "world/TerrainGround.h"
#include "ship/ShipConfig.h"
#include "world/World.h"
#include "math/WorldMath.h"
#include "platform/JobSystem.h"
#include <vulkan/vulkan.h>
#include <vector>
#include <mutex>

namespace planet {

class ScatterRenderer {
public:

    void init(VkDevice device, VkPhysicalDevice phys, VkQueue queue,
              uint32_t queueFamily, const TerrainGround& ground) {
        const ShipMeshData m = buildCrystalShard();
        shard_ = Mesh::createImmediate(device, phys, queue, queueFamily,
                                       m.vertices, m.indices);
        const ShipMeshData o = buildOutpost();
        outpost_ = Mesh::createImmediate(device, phys, queue, queueFamily,
                                         o.vertices, o.indices);
        buildOutposts(sites_, ground);
    }

    void startJobs(bool threaded) { if (threaded) jobs_.start(1); }

    void shutdown() { jobs_.stop(); shard_ = Mesh{}; outpost_ = Mesh{};
                      instances_.clear(); have_ = false; }

    void update(const World& w, const DVec3& camPos, bool force = false) {

        const double len = space::length(camPos);
        if (len < 1.0) return;
        const DVec3  dir = camPos * (1.0 / len);
        const double ground = surfaceRadiusAt(w, dir, kPhysicsDetailOctaves);
        const double alt = len - ground;
        if (alt > kScatterCeiling) {
            if (!instances_.empty()) { instances_.clear(); have_ = false; }
            return;
        }

        collectFinished();
        if (jobs_.pending() > 0) return;
        if (have_ && !force) {
            const DVec3 d = camPos - centre_;
            if (space::dot(d, d) < kScatterRebuild * kScatterRebuild) return;
        }
        centre_ = camPos;
        have_   = true;

        jobs_.enqueue([this, &w, camPos] {
            int dropped = 0;
            std::vector<ScatterInstance> built =
                collectScatter(w, camPos, kScatterRange,
                               kPhysicsDetailOctaves, &dropped);
            std::lock_guard<std::mutex> lock(mtx_);
            staged_     = std::move(built);
            stagedDrop_ = dropped;
            ready_      = true;
        });

        if (!jobs_.threaded()) collectFinished();
    }

    int droppedLastRebuild() const { return capped_; }
    size_t count() const { return instances_.size(); }

    void recordDraw(VkCommandBuffer cmd, VkPipelineLayout layout,
                    const DVec3& camPos, uint64_t& trianglesOut) const {
        if (!shard_.valid() || instances_.empty()) return;

        for (const ScatterInstance& s : instances_) {
            const Vec3 off = space::toF(s.pos - camPos);
            const float data[16] = { off.x, off.y, off.z, 0.0f,
                                     s.orient.x, s.orient.y, s.orient.z, s.orient.w,
                                     0.0f, 0.0f, s.scale, 0.0f,
                                     0.0f, 0.0f, 0.0f, 0.0f };
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                               0, sizeof(data), data);
            shard_.draw(cmd);
            trianglesOut += shard_.triangleCount();
        }
    }

    void recordOutposts(VkCommandBuffer cmd, VkPipelineLayout layout,
                        const DVec3& camPos, uint64_t& trianglesOut) const {
        if (!outpost_.valid()) return;
        for (const Outpost& o : sites_) {
            const DVec3 delta = o.pos - camPos;
            if (space::dot(delta, delta) >
                kOutpostVisibleRange * kOutpostVisibleRange) continue;
            const Vec3 off = space::toF(delta);
            const float data[16] = { off.x, off.y, off.z, 0.0f,
                                     o.orient.x, o.orient.y, o.orient.z, o.orient.w,
                                     kOutpostAmbient, 0.0f, 1.0f, 0.0f,
                                     0.0f, 0.0f, 0.0f, 0.0f };
            vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                               0, sizeof(data), data);
            outpost_.draw(cmd);
            trianglesOut += outpost_.triangleCount();
        }
    }

    const Outpost* sites() const { return sites_; }

private:

    void collectFinished() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!ready_) return;
        instances_ = std::move(staged_);
        capped_    = stagedDrop_;
        ready_     = false;
    }

    JobSystem                    jobs_;
    mutable std::mutex           mtx_;
    std::vector<ScatterInstance> staged_;
    int                          stagedDrop_ = 0;
    bool                         ready_      = false;

    Mesh                         shard_;
    Mesh                         outpost_;
    Outpost                      sites_[kDestinationCount]{};
    std::vector<ScatterInstance> instances_;
    DVec3                        centre_{};
    bool                         have_   = false;
    int                          capped_ = 0;
};

}
