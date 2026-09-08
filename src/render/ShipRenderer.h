#pragma once

#include "render/Mesh.h"
#include "ship/ShipMesh.h"
#include "ship/Cockpit.h"
#include "ship/Flight.h"
#include "math/Quat.h"
#include "math/WorldMath.h"
#include "render/VulkanRenderer.h"
#include <vulkan/vulkan.h>

namespace planet {

class ShipRenderer {
public:
    void init(VkDevice device, VkPhysicalDevice phys, VkQueue queue,
              uint32_t queueFamily) {
        const ShipMeshData hull = buildShipHull();
        const ShipMeshData leg  = buildGearLeg();
        const ShipMeshData in   = buildCockpitInterior();
        const ShipMeshData sw   = buildSwitchLever();
        const ShipMeshData lp   = buildLamp();
        hull_ = Mesh::createImmediate(device, phys, queue, queueFamily,
                                      hull.vertices, hull.indices);
        leg_  = Mesh::createImmediate(device, phys, queue, queueFamily,
                                      leg.vertices, leg.indices);
        interior_ = Mesh::createImmediate(device, phys, queue, queueFamily,
                                          in.vertices, in.indices);
        switch_   = Mesh::createImmediate(device, phys, queue, queueFamily,
                                          sw.vertices, sw.indices);
        lamp_     = Mesh::createImmediate(device, phys, queue, queueFamily,
                                          lp.vertices, lp.indices);
    }

    void shutdown() { hull_ = Mesh{}; leg_ = Mesh{}; interior_ = Mesh{};
                      switch_ = Mesh{}; lamp_ = Mesh{}; }

    uint64_t triangleCount() const {
        return hull_.triangleCount() + 4ull * leg_.triangleCount() +
               interior_.triangleCount() +
               uint64_t(kSwitchCount) * switch_.triangleCount() +
               lamp_.triangleCount();
    }

    void recordCockpit(VkCommandBuffer cmd, VkPipelineLayout layout,
                       const space::DVec3& camPos, const ShipState& s,
                       const ShipToggles& toggles, int hovered, LampState lamp,
                       uint64_t& trianglesOut) const {
        if (!interior_.valid()) return;
        const space::Vec3 shipOff = space::toF(s.pos - camPos);
        pushObject(cmd, layout, shipOff, s.orient, 0.0f, kCockpitAmbient);
        interior_.draw(cmd);
        trianglesOut += interior_.triangleCount();

        if (!switch_.valid()) return;

        for (int i = 0; i < kSwitchCount; ++i) {
            const space::Vec3& c = kSwitches[i].centre;
            const space::DVec3 hinge{ c.x, c.y - kSwitchHalf.y, c.z };
            const space::DVec3 world = s.pos + space::rotate(s.orient, hinge);
            const float amb = switchIsOn(toggles, i) ? kSwitchAmbientOn
                                                     : kSwitchAmbientOff;
            pushObject(cmd, layout, space::toF(world - camPos),
                       s.orient * switchRotation(toggles, i), 0.0f,
                       amb, (i == hovered) ? 1.0f : 0.0f);
            switch_.draw(cmd);
            trianglesOut += switch_.triangleCount();
        }

        if (lamp_.valid()) {
            const space::DVec3 base{ kLampCentre.x, kLampCentre.y - kLampHalf.y,
                                     kLampCentre.z };
            const space::DVec3 world = s.pos + space::rotate(s.orient, base);
            const float amb  = (lamp == LampState::Cold) ? 0.20f : 0.95f;
            const float warm = (lamp == LampState::Busy) ? 1.0f : 0.0f;
            pushObject(cmd, layout, space::toF(world - camPos), s.orient, 0.0f,
                       amb, warm);
            lamp_.draw(cmd);
            trianglesOut += lamp_.triangleCount();
        }
    }

    void recordDraw(VkCommandBuffer cmd, VkPipelineLayout layout,
                    const space::DVec3& camPos, const ShipState& s,
                    uint64_t& trianglesOut) const {
        if (!hull_.valid()) return;

        pushObject(cmd, layout, space::toF(s.pos - camPos), s.orient, 0.0f);
        hull_.draw(cmd);
        trianglesOut += hull_.triangleCount();

        if (s.gearT <= 0.001f || !leg_.valid()) return;

        const space::Quat fold =
            space::fromAxisAngle(space::Vec3{ 1.0f, 0.0f, 0.0f },
                                 (1.0f - s.gearT) * space::kPi * 0.5f);
        for (int i = 0; i < 4; ++i) {
            const space::DVec3 pivot{ kGearPivot[i].x, kGearPivot[i].y, kGearPivot[i].z };
            const space::DVec3 world = s.pos + space::rotate(s.orient, pivot);
            pushObject(cmd, layout, space::toF(world - camPos), s.orient * fold, 1.0f);
            leg_.draw(cmd);
            trianglesOut += leg_.triangleCount();
        }
    }

private:

    static void pushObject(VkCommandBuffer cmd, VkPipelineLayout layout,
                           const space::Vec3& offset, const space::Quat& rot,
                           float tag, float ambient = 0.0f, float highlight = 0.0f,
                           float scale = 1.0f) {

        const float data[16] = { offset.x, offset.y, offset.z, tag,
                                 rot.x, rot.y, rot.z, rot.w,
                                 ambient, highlight, scale, 0.0f,
                                 0.0f, 0.0f, 0.0f, 0.0f };
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT,
                           kPatchPushOffset, kPatchPushSize, data);
    }

    Mesh hull_;
    Mesh leg_;
    Mesh interior_;
    Mesh switch_;
    Mesh lamp_;
};

}
