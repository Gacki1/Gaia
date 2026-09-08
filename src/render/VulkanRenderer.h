#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#define VK_USE_PLATFORM_WIN32_KHR
#include <vulkan/vulkan.h>

#include "world/PlanetConfig.h"
#include "render/Texture.h"
#include "render/OverlayDraw.h"
#include <vector>
#include <array>
#include <string>
#include <cstdint>

namespace planet {

struct FrameUniforms {
    float viewProj[16];
    float sunDir[4];
    float params[4];

    float camRight[4];
    float camUp[4];
    float camFwd[4];

    float camGeo[4];

    float betaR[4];

    float atmoA[4];

    float atmoB[4];

    float sunDisc[4];

    float marker[4];

    float terrain[4];

    float terrain2[4];

};
static_assert(sizeof(FrameUniforms) == 272, "FrameUniforms must match the GLSL block");

struct ObjectPush {
    float offset[4];
    float rot[4];

    float shade[4];

    float texOrigin[4];
};
static_assert(sizeof(ObjectPush) == 64, "ObjectPush must match the push range");

inline constexpr uint32_t kPatchPushOffset = 0;
inline constexpr uint32_t kPatchPushSize   = 64;

class VulkanRenderer {
public:
    void init(HINSTANCE hinst, HWND hwnd, uint32_t width, uint32_t height,
              bool enableValidation);
    void shutdown();

    VkDevice         device()         const { return device_; }

    VkPipelineLayout pipelineLayout() const { return pipelineLayout_; }
    VkPhysicalDevice physicalDevice() const { return physical_; }

    VkQueue          graphicsQueue()  const { return graphicsQueue_; }

    float            maxAnisotropy()  const { return maxAnisotropy_; }

    void setOverlay(const std::vector<OverlayVertex>& verts);

    const FontAtlas& font() const { return fontAtlas_; }
    uint32_t         graphicsFamily() const { return graphicsFamily_; }
    VkExtent2D       extent()         const { return extent_; }

    VkCommandBuffer beginFrame(const FrameUniforms& frame, bool wireframe);
    void endFrame();

    void notifyResize(uint32_t width, uint32_t height);

    uint64_t frameNumber() const { return frameNumber_; }
    static constexpr uint32_t framesInFlight() { return static_cast<uint32_t>(kFramesInFlight); }

    uint32_t           validationErrors() const { return validationErrors_; }
    const std::string& gpuName()          const { return gpuName_; }
    void waitIdle() const { if (device_) vkDeviceWaitIdle(device_); }

    void requestCapture(const std::string& path);
    bool writeCapture();

private:
    void createInstance(bool enableValidation);
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createCommandPool();
    void createSyncObjects();
    void createRenderPass();
    void createPipelines();
    void createFrameResources();
    void createCompositeResources();

    void createSwapchain();
    void createSceneTargets();
    void createFramebuffers();
    void updateFrameSets();
    void destroySwapchainResources();
    void recreateSwapchain();

    VkShaderModule loadShader(const std::string& path);
    VkFormat pickDepthFormat() const;
    VkFormat pickHdrFormat() const;

    HINSTANCE hinst_ = nullptr;
    HWND      hwnd_   = nullptr;

    VkInstance               instance_  = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT messenger_ = VK_NULL_HANDLE;
    VkSurfaceKHR             surface_   = VK_NULL_HANDLE;
    VkPhysicalDevice         physical_  = VK_NULL_HANDLE;
    VkDevice                 device_    = VK_NULL_HANDLE;

    uint32_t graphicsFamily_ = UINT32_MAX;
    uint32_t presentFamily_  = UINT32_MAX;
    VkQueue  graphicsQueue_  = VK_NULL_HANDLE;
    VkQueue  presentQueue_   = VK_NULL_HANDLE;

    VkSwapchainKHR           swapchain_   = VK_NULL_HANDLE;
    VkFormat                 colorFormat_ = VK_FORMAT_UNDEFINED;
    VkExtent2D               extent_{};
    std::vector<VkImage>     images_;
    std::vector<VkImageView> imageViews_;

    VkFormat       depthFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat       hdrFormat_   = VK_FORMAT_UNDEFINED;
    std::array<VkImage,        kFramesInFlight> depthImages_{};
    std::array<VkDeviceMemory, kFramesInFlight> depthMems_{};
    std::array<VkImageView,    kFramesInFlight> depthViews_{};
    std::array<VkImage,        kFramesInFlight> hdrImages_{};
    std::array<VkDeviceMemory, kFramesInFlight> hdrMems_{};
    std::array<VkImageView,    kFramesInFlight> hdrViews_{};

    VkRenderPass sceneRenderPass_     = VK_NULL_HANDLE;
    VkRenderPass compositeRenderPass_ = VK_NULL_HANDLE;
    std::array<VkFramebuffer, kFramesInFlight> sceneFramebuffers_{};
    std::vector<VkFramebuffer>                 compositeFramebuffers_;

    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline       fillPipeline_   = VK_NULL_HANDLE;
    VkPipeline       wirePipeline_   = VK_NULL_HANDLE;

    VkSampler             sceneSampler_  = VK_NULL_HANDLE;
    VkDescriptorSetLayout frameSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool      descriptorPool_ = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, kFramesInFlight> frameSets_{};

    std::array<VkBuffer,       kFramesInFlight> frameUbos_{};
    std::array<VkDeviceMemory, kFramesInFlight> frameUboMems_{};
    std::array<void*,          kFramesInFlight> frameUboMapped_{};

    VkPipeline compositePipeline_ = VK_NULL_HANDLE;

    Texture2D transmittanceLut_;

    Texture2D      fontTex_;
    VkPipeline     overlayPipeline_ = VK_NULL_HANDLE;
    void           createOverlayPipeline();

    VkBuffer       overlayBuf_[kFramesInFlight]{};
    VkDeviceMemory overlayMem_[kFramesInFlight]{};
    void*          overlayMapped_[kFramesInFlight]{};
    VkDeviceSize   overlayCapacity_[kFramesInFlight]{};
    uint32_t       overlayVertexCount_ = 0;

    Texture2D groundTex_;
    Texture2D groundNrmTex_;
    float     maxAnisotropy_ = 1.0f;
    FontAtlas fontAtlas_;

    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    std::array<VkCommandBuffer, kFramesInFlight> commandBuffers_{};

    std::array<VkSemaphore, kFramesInFlight> imageAvailable_{};
    std::array<VkFence,     kFramesInFlight> inFlight_{};
    std::vector<VkSemaphore> renderFinished_;
    std::vector<VkFence>     imagesInFlight_;

    uint32_t currentFrame_ = 0;
    uint32_t imageIndex_   = 0;
    uint64_t frameNumber_  = 0;

    bool     resizePending_ = false;
    uint32_t pendingWidth_  = 0;
    uint32_t pendingHeight_ = 0;

    bool     validationEnabled_ = false;
    uint32_t validationErrors_  = 0;
    bool     canCapture_        = false;
    std::string gpuName_ = "unknown";

    bool           captureRequested_ = false;
    bool           captureHasData_   = false;
    std::string    capturePath_;
    VkBuffer       captureBuf_  = VK_NULL_HANDLE;
    VkDeviceMemory captureMem_  = VK_NULL_HANDLE;
    VkDeviceSize   captureSize_ = 0;
    VkExtent2D     captureExtent_{};
};

}
