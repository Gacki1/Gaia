#include "render/VulkanRenderer.h"
#include "platform/FontAtlas.h"
#include "render/GroundTextureSet.h"
#include "world/Atmosphere.h"
#include "render/Mesh.h"
#include "render/VkUtil.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <algorithm>
#include <array>

namespace planet {

namespace {

const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT severity,
        VkDebugUtilsMessageTypeFlagsEXT type,
        const VkDebugUtilsMessengerCallbackDataEXT* data,
        void* user) {

    const bool relevant = (type & (VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                   VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT)) != 0;
    if (!relevant) return VK_FALSE;

    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        std::fprintf(stderr, "[vulkan] %s\n", data->pMessage);
        std::fflush(stderr);
    }
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT && user) {
        *static_cast<uint32_t*>(user) += 1;
    }
    return VK_FALSE;
}

bool layerAvailable(const char* name) {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const auto& l : layers)
        if (std::strcmp(l.layerName, name) == 0) return true;
    return false;
}

std::string exeDir() {
    char buf[MAX_PATH] = {0};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string p(buf);
    size_t slash = p.find_last_of("\\/");
    return slash == std::string::npos ? std::string(".") : p.substr(0, slash);
}

VkDebugUtilsMessengerCreateInfoEXT makeMessengerInfo(uint32_t* counter) {
    VkDebugUtilsMessengerCreateInfoEXT ci{ VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
    ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                         VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType     = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                         VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                         VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = debugCallback;
    ci.pUserData       = counter;
    return ci;
}

}

void VulkanRenderer::init(HINSTANCE hinst, HWND hwnd, uint32_t width,
                          uint32_t height, bool enableValidation) {
    hinst_ = hinst;
    hwnd_  = hwnd;
    pendingWidth_  = width;
    pendingHeight_ = height;
    validationEnabled_ = enableValidation && layerAvailable(kValidationLayer);

    createInstance(validationEnabled_);
    setupDebugMessenger();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createCommandPool();
    createSyncObjects();
    createSwapchain();
    createSceneTargets();
    createRenderPass();
    createFrameResources();
    createPipelines();
    createCompositeResources();
    createOverlayPipeline();
    createFramebuffers();
    updateFrameSets();
}

void VulkanRenderer::createInstance(bool enableValidation) {
    VkApplicationInfo app{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    app.pApplicationName   = "Procedural Planet";
    app.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    app.pEngineName        = "PlanetEngine";
    app.engineVersion      = VK_MAKE_VERSION(1, 0, 0);
    app.apiVersion         = VK_API_VERSION_1_1;

    std::vector<const char*> exts = {
        VK_KHR_SURFACE_EXTENSION_NAME,
        VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
    };
    std::vector<const char*> layers;
    if (enableValidation) {
        exts.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        layers.push_back(kValidationLayer);
    }

    VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    ci.pApplicationInfo        = &app;
    ci.enabledExtensionCount   = static_cast<uint32_t>(exts.size());
    ci.ppEnabledExtensionNames = exts.data();
    ci.enabledLayerCount       = static_cast<uint32_t>(layers.size());
    ci.ppEnabledLayerNames     = layers.empty() ? nullptr : layers.data();

    VkDebugUtilsMessengerCreateInfoEXT dbg = makeMessengerInfo(&validationErrors_);
    if (enableValidation) ci.pNext = &dbg;

    vkCheck(vkCreateInstance(&ci, nullptr, &instance_), "vkCreateInstance");
}

void VulkanRenderer::setupDebugMessenger() {
    if (!validationEnabled_) return;
    auto fn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
    if (!fn) return;
    VkDebugUtilsMessengerCreateInfoEXT ci = makeMessengerInfo(&validationErrors_);
    fn(instance_, &ci, nullptr, &messenger_);
}

void VulkanRenderer::createSurface() {
    VkWin32SurfaceCreateInfoKHR ci{ VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR };
    ci.hinstance = hinst_;
    ci.hwnd      = hwnd_;
    vkCheck(vkCreateWin32SurfaceKHR(instance_, &ci, nullptr, &surface_),
            "vkCreateWin32SurfaceKHR");
}

void VulkanRenderer::pickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    if (count == 0) throw std::runtime_error("No Vulkan-capable GPU found");
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance_, &count, devices.data());

    auto suitable = [&](VkPhysicalDevice dev, uint32_t& gfx, uint32_t& present) {

        uint32_t qCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, nullptr);
        std::vector<VkQueueFamilyProperties> qprops(qCount);
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, qprops.data());
        gfx = present = UINT32_MAX;
        for (uint32_t i = 0; i < qCount; ++i) {
            if (qprops[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) gfx = i;
            VkBool32 sup = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface_, &sup);
            if (sup) present = i;
            if (gfx != UINT32_MAX && present != UINT32_MAX) break;
        }
        if (gfx == UINT32_MAX || present == UINT32_MAX) return false;

        uint32_t eCount = 0;
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &eCount, nullptr);
        std::vector<VkExtensionProperties> exts(eCount);
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &eCount, exts.data());
        bool hasSwap = false;
        for (const auto& e : exts)
            if (std::strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) hasSwap = true;
        if (!hasSwap) return false;

        VkPhysicalDeviceFeatures feats{};
        vkGetPhysicalDeviceFeatures(dev, &feats);
        return feats.fillModeNonSolid == VK_TRUE;
    };

    VkPhysicalDevice chosen = VK_NULL_HANDLE;
    uint32_t gfx = UINT32_MAX, present = UINT32_MAX;

    for (auto dev : devices) {
        uint32_t g, p;
        if (!suitable(dev, g, p)) continue;
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(dev, &props);
        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            chosen = dev; gfx = g; present = p; break;
        }
        if (chosen == VK_NULL_HANDLE) { chosen = dev; gfx = g; present = p; }
    }
    if (chosen == VK_NULL_HANDLE)
        throw std::runtime_error("No suitable GPU (needs swapchain + fillModeNonSolid)");

    physical_       = chosen;
    graphicsFamily_ = gfx;
    presentFamily_  = present;
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(physical_, &props);
    gpuName_ = props.deviceName;
}

void VulkanRenderer::createLogicalDevice() {
    std::set<uint32_t> families = { graphicsFamily_, presentFamily_ };
    std::vector<VkDeviceQueueCreateInfo> qcis;
    float prio = 1.0f;
    for (uint32_t f : families) {
        VkDeviceQueueCreateInfo qci{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
        qci.queueFamilyIndex = f;
        qci.queueCount       = 1;
        qci.pQueuePriorities = &prio;
        qcis.push_back(qci);
    }

    VkPhysicalDeviceFeatures avail{};
    vkGetPhysicalDeviceFeatures(physical_, &avail);
    VkPhysicalDeviceFeatures feats{};
    feats.fillModeNonSolid = VK_TRUE;
    feats.wideLines        = avail.wideLines;

    feats.samplerAnisotropy = avail.samplerAnisotropy;
    maxAnisotropy_ = VK_FALSE != avail.samplerAnisotropy ? 0.0f : 1.0f;

    const char* deviceExts[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };

    VkDeviceCreateInfo ci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    ci.queueCreateInfoCount    = static_cast<uint32_t>(qcis.size());
    ci.pQueueCreateInfos       = qcis.data();
    ci.enabledExtensionCount   = 1;
    ci.ppEnabledExtensionNames = deviceExts;
    ci.pEnabledFeatures        = &feats;

    vkCheck(vkCreateDevice(physical_, &ci, nullptr, &device_), "vkCreateDevice");

    if (maxAnisotropy_ == 0.0f) {
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(physical_, &props);
        maxAnisotropy_ = props.limits.maxSamplerAnisotropy;
    }

    vkGetDeviceQueue(device_, graphicsFamily_, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, presentFamily_, 0, &presentQueue_);
}

VkFormat VulkanRenderer::pickDepthFormat() const {

    const VkFormat candidates[] = { VK_FORMAT_D32_SFLOAT,
                                    VK_FORMAT_D32_SFLOAT_S8_UINT };
    for (VkFormat f : candidates) {
        VkFormatProperties props{};
        vkGetPhysicalDeviceFormatProperties(physical_, f, &props);
        if (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
            return f;
    }
    throw std::runtime_error("No supported float depth format (D32_SFLOAT required "
                             "for reversed-Z)");
}

VkExtent2D queryClientExtent(HWND hwnd) {
    RECT rc{};
    GetClientRect(hwnd, &rc);
    return { static_cast<uint32_t>(std::max<LONG>(rc.right - rc.left, 0)),
             static_cast<uint32_t>(std::max<LONG>(rc.bottom - rc.top, 0)) };
}

void VulkanRenderer::createSwapchain() {
    VkSurfaceCapabilitiesKHR caps{};
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_, surface_, &caps);

    uint32_t fmtCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_, surface_, &fmtCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fmtCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physical_, surface_, &fmtCount, formats.data());
    VkSurfaceFormatKHR sf = formats[0];
    for (const auto& f : formats)
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { sf = f; break; }
    colorFormat_ = sf.format;

    VkExtent2D ext = queryClientExtent(hwnd_);
    if (caps.currentExtent.width != UINT32_MAX) ext = caps.currentExtent;
    ext.width  = std::clamp(ext.width,  caps.minImageExtent.width,  caps.maxImageExtent.width);
    ext.height = std::clamp(ext.height, caps.minImageExtent.height, caps.maxImageExtent.height);
    extent_ = ext;

    uint32_t imageCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
        imageCount = caps.maxImageCount;

    VkSwapchainCreateInfoKHR sci{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
    sci.surface          = surface_;
    sci.minImageCount    = imageCount;
    sci.imageFormat      = sf.format;
    sci.imageColorSpace  = sf.colorSpace;
    sci.imageExtent      = extent_;
    sci.imageArrayLayers = 1;
    sci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    canCapture_ = (caps.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) != 0;
    if (canCapture_) sci.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    sci.preTransform     = caps.currentTransform;
    sci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    sci.presentMode      = VK_PRESENT_MODE_FIFO_KHR;
    sci.clipped          = VK_TRUE;

    uint32_t fam[] = { graphicsFamily_, presentFamily_ };
    if (graphicsFamily_ != presentFamily_) {
        sci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
        sci.queueFamilyIndexCount = 2;
        sci.pQueueFamilyIndices   = fam;
    } else {
        sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }
    vkCheck(vkCreateSwapchainKHR(device_, &sci, nullptr, &swapchain_), "vkCreateSwapchainKHR");

    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, nullptr);
    images_.resize(imageCount);
    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, images_.data());

    imageViews_.resize(imageCount);
    for (uint32_t i = 0; i < imageCount; ++i) {
        VkImageViewCreateInfo iv{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        iv.image    = images_[i];
        iv.viewType = VK_IMAGE_VIEW_TYPE_2D;
        iv.format   = colorFormat_;
        iv.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        vkCheck(vkCreateImageView(device_, &iv, nullptr, &imageViews_[i]),
                "vkCreateImageView(color)");
    }

    renderFinished_.resize(imageCount);
    for (auto& s : renderFinished_) {
        VkSemaphoreCreateInfo si{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        vkCheck(vkCreateSemaphore(device_, &si, nullptr, &s), "vkCreateSemaphore(renderFinished)");
    }
    imagesInFlight_.assign(imageCount, VK_NULL_HANDLE);
}

namespace {

void makeTarget(VkDevice device, VkPhysicalDevice phys, VkExtent2D extent,
                VkFormat format, VkImageUsageFlags usage, VkImageAspectFlags aspect,
                VkImage& image, VkDeviceMemory& mem, VkImageView& view,
                const char* what) {
    VkImageCreateInfo ii{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    ii.imageType   = VK_IMAGE_TYPE_2D;
    ii.extent      = { extent.width, extent.height, 1 };
    ii.mipLevels   = 1;
    ii.arrayLayers = 1;
    ii.format      = format;
    ii.tiling      = VK_IMAGE_TILING_OPTIMAL;
    ii.usage       = usage;
    ii.samples     = VK_SAMPLE_COUNT_1_BIT;
    ii.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCheck(vkCreateImage(device, &ii, nullptr, &image), what);

    VkMemoryRequirements mr{};
    vkGetImageMemoryRequirements(device, image, &mr);
    VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    mai.allocationSize  = mr.size;
    mai.memoryTypeIndex = findMemoryType(phys, mr.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkCheck(vkAllocateMemory(device, &mai, nullptr, &mem), what);
    vkCheck(vkBindImageMemory(device, image, mem, 0), what);

    VkImageViewCreateInfo vi{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    vi.image            = image;
    vi.viewType         = VK_IMAGE_VIEW_TYPE_2D;
    vi.format           = format;
    vi.subresourceRange = { aspect, 0, 1, 0, 1 };
    vkCheck(vkCreateImageView(device, &vi, nullptr, &view), what);
}
}

void VulkanRenderer::createSceneTargets() {
    depthFormat_ = pickDepthFormat();
    hdrFormat_   = pickHdrFormat();
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        makeTarget(device_, physical_, extent_, depthFormat_,
                   VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
                   VK_IMAGE_USAGE_SAMPLED_BIT,
                   VK_IMAGE_ASPECT_DEPTH_BIT,
                   depthImages_[i], depthMems_[i], depthViews_[i], "depth target");
        makeTarget(device_, physical_, extent_, hdrFormat_,
                   VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                   VK_IMAGE_ASPECT_COLOR_BIT,
                   hdrImages_[i], hdrMems_[i], hdrViews_[i], "HDR target");
    }
}

void VulkanRenderer::createFramebuffers() {

    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        VkImageView attachments[] = { hdrViews_[i], depthViews_[i] };
        VkFramebufferCreateInfo fi{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        fi.renderPass      = sceneRenderPass_;
        fi.attachmentCount = 2;
        fi.pAttachments    = attachments;
        fi.width           = extent_.width;
        fi.height          = extent_.height;
        fi.layers          = 1;
        vkCheck(vkCreateFramebuffer(device_, &fi, nullptr, &sceneFramebuffers_[i]),
                "vkCreateFramebuffer(scene)");
    }

    compositeFramebuffers_.resize(imageViews_.size());
    for (size_t i = 0; i < imageViews_.size(); ++i) {
        VkFramebufferCreateInfo fi{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        fi.renderPass      = compositeRenderPass_;
        fi.attachmentCount = 1;
        fi.pAttachments    = &imageViews_[i];
        fi.width           = extent_.width;
        fi.height          = extent_.height;
        fi.layers          = 1;
        vkCheck(vkCreateFramebuffer(device_, &fi, nullptr, &compositeFramebuffers_[i]),
                "vkCreateFramebuffer(composite)");
    }
}

void VulkanRenderer::updateFrameSets() {
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        VkDescriptorImageInfo img{};
        img.sampler     = sceneSampler_;
        img.imageView   = hdrViews_[i];
        img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo dep{};
        dep.sampler     = sceneSampler_;
        dep.imageView   = depthViews_[i];
        dep.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet w[2]{};
        w[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[0].dstSet          = frameSets_[i];
        w[0].dstBinding      = 1;
        w[0].descriptorCount = 1;
        w[0].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[0].pImageInfo      = &img;
        w[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[1].dstSet          = frameSets_[i];
        w[1].dstBinding      = 3;
        w[1].descriptorCount = 1;
        w[1].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[1].pImageInfo      = &dep;
        vkUpdateDescriptorSets(device_, 2, w, 0, nullptr);
    }
}

void VulkanRenderer::recreateSwapchain() {
    vkDeviceWaitIdle(device_);
    destroySwapchainResources();
    createSwapchain();
    createSceneTargets();
    createFramebuffers();
    updateFrameSets();
}

void VulkanRenderer::destroySwapchainResources() {
    for (auto fb : compositeFramebuffers_) vkDestroyFramebuffer(device_, fb, nullptr);
    compositeFramebuffers_.clear();
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        if (sceneFramebuffers_[i]) vkDestroyFramebuffer(device_, sceneFramebuffers_[i], nullptr);
        sceneFramebuffers_[i] = VK_NULL_HANDLE;
        if (depthViews_[i])  vkDestroyImageView(device_, depthViews_[i], nullptr);
        if (depthImages_[i]) vkDestroyImage(device_, depthImages_[i], nullptr);
        if (depthMems_[i])   vkFreeMemory(device_, depthMems_[i], nullptr);
        depthViews_[i] = VK_NULL_HANDLE; depthImages_[i] = VK_NULL_HANDLE;
        depthMems_[i]  = VK_NULL_HANDLE;
        if (hdrViews_[i])  vkDestroyImageView(device_, hdrViews_[i], nullptr);
        if (hdrImages_[i]) vkDestroyImage(device_, hdrImages_[i], nullptr);
        if (hdrMems_[i])   vkFreeMemory(device_, hdrMems_[i], nullptr);
        hdrViews_[i] = VK_NULL_HANDLE; hdrImages_[i] = VK_NULL_HANDLE;
        hdrMems_[i]  = VK_NULL_HANDLE;
    }
    for (auto v : imageViews_) vkDestroyImageView(device_, v, nullptr);
    imageViews_.clear();
    for (auto s : renderFinished_) vkDestroySemaphore(device_, s, nullptr);
    renderFinished_.clear();
    imagesInFlight_.clear();
    if (swapchain_) vkDestroySwapchainKHR(device_, swapchain_, nullptr);
    swapchain_ = VK_NULL_HANDLE;
}

void VulkanRenderer::createFrameResources() {
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size        = sizeof(FrameUniforms);
        bi.usage       = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device_, &bi, nullptr, &frameUbos_[i]), "vkCreateBuffer(ubo)");

        VkMemoryRequirements mr{};
        vkGetBufferMemoryRequirements(device_, frameUbos_[i], &mr);
        VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        mai.allocationSize  = mr.size;
        mai.memoryTypeIndex = findMemoryType(physical_, mr.memoryTypeBits,
                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                             VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device_, &mai, nullptr, &frameUboMems_[i]),
                "vkAllocateMemory(ubo)");
        vkCheck(vkBindBufferMemory(device_, frameUbos_[i], frameUboMems_[i], 0),
                "vkBindBufferMemory(ubo)");

        vkCheck(vkMapMemory(device_, frameUboMems_[i], 0, sizeof(FrameUniforms), 0,
                            &frameUboMapped_[i]), "vkMapMemory(ubo)");
    }

    VkDescriptorSetLayoutBinding b[7]{};
    b[0].binding         = 0;
    b[0].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    b[0].descriptorCount = 1;
    b[0].stageFlags      = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    b[1].binding         = 1;
    b[1].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[1].descriptorCount = 1;
    b[1].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    b[2].binding         = 2;
    b[2].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[2].descriptorCount = 1;
    b[2].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    b[3].binding         = 3;
    b[3].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[3].descriptorCount = 1;
    b[3].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    b[4].binding         = 4;
    b[4].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[4].descriptorCount = 1;
    b[4].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    b[5].binding         = 5;
    b[5].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[5].descriptorCount = 1;
    b[5].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    b[6].binding         = 6;
    b[6].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[6].descriptorCount = 1;
    b[6].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo li{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    li.bindingCount = 7;
    li.pBindings    = b;
    vkCheck(vkCreateDescriptorSetLayout(device_, &li, nullptr, &frameSetLayout_),
            "vkCreateDescriptorSetLayout");

    VkDescriptorPoolSize ps[2] = {
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         kFramesInFlight },
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kFramesInFlight * 6 },
    };
    VkDescriptorPoolCreateInfo pi{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    pi.maxSets       = kFramesInFlight;
    pi.poolSizeCount = 2;
    pi.pPoolSizes    = ps;
    vkCheck(vkCreateDescriptorPool(device_, &pi, nullptr, &descriptorPool_),
            "vkCreateDescriptorPool");

    std::array<VkDescriptorSetLayout, kFramesInFlight> layouts{};
    layouts.fill(frameSetLayout_);
    VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    ai.descriptorPool     = descriptorPool_;
    ai.descriptorSetCount = kFramesInFlight;
    ai.pSetLayouts        = layouts.data();
    vkCheck(vkAllocateDescriptorSets(device_, &ai, frameSets_.data()),
            "vkAllocateDescriptorSets");

    {
        const VkFormat lutFormat = VK_FORMAT_R32G32B32A32_SFLOAT;
        if (!Texture2D::supportsLinearFilter(physical_, lutFormat)) {
            throw std::runtime_error(
                "Vulkan error: R32G32B32A32_SFLOAT cannot be linearly filtered on this GPU; "
                "the transmittance LUT needs it");
        }
        const atmo::TransmittanceLut lut = atmo::buildTransmittanceLut();
        transmittanceLut_ = Texture2D::create(device_, physical_, graphicsQueue_,
                                              graphicsFamily_,
                                              uint32_t(lut.width), uint32_t(lut.height),
                                              lutFormat, lut.rgba.data(), lut.byteSize(),
                                              VK_FILTER_LINEAR);
    }

    {
        fontAtlas_ = buildFontAtlas("Consolas", 16);
        if (fontAtlas_.ok()) {
            fontTex_ = Texture2D::create(device_, physical_, graphicsQueue_,
                                         graphicsFamily_,
                                         fontAtlas_.width, fontAtlas_.height,
                                         VK_FORMAT_R8_UNORM,
                                         fontAtlas_.pixels.data(),
                                         fontAtlas_.pixels.size(),
                                         VK_FILTER_LINEAR);
            std::printf("interface font: %ux%u atlas, %ux%u cell\n",
                        fontAtlas_.width, fontAtlas_.height,
                        fontAtlas_.cellW, fontAtlas_.cellH);
        } else {

            std::printf("interface font: FAILED to rasterise -- overlay disabled\n");
        }
    }

    {
        const GroundSet gs = loadGroundSet();

        groundTex_ = Texture2D::createArray(device_, physical_, graphicsQueue_,
                                            graphicsFamily_,
                                            gs.size, gs.size, gs.layers,

                                            VK_FORMAT_R8G8B8A8_SRGB,
                                            gs.rgba.data(), gs.rgba.size(),
                                            maxAnisotropy_);

        groundNrmTex_ = Texture2D::createArray(device_, physical_, graphicsQueue_,
                                               graphicsFamily_,
                                               gs.size, gs.size, gs.layers,
                                               VK_FORMAT_R8G8_UNORM,
                                               gs.normalRG.data(),
                                               gs.normalRG.size(), maxAnisotropy_);
        std::printf("ground materials: %u layers of %ux%u, %u mips, aniso %.0fx"
                    " (%d of %u colour, %d of %u normal maps from disk)\n",
                    gs.layers, gs.size, gs.size, groundTex_.mipLevels(),
                    maxAnisotropy_, gs.loaded, gs.layers,
                    gs.normalsLoaded, gs.layers);
    }

    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        VkDescriptorBufferInfo buf{ frameUbos_[i], 0, sizeof(FrameUniforms) };
        VkDescriptorImageInfo  lutImg{ transmittanceLut_.sampler(),
                                       transmittanceLut_.view(),
                                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo  gndImg{ groundTex_.sampler(), groundTex_.view(),
                                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo  gnrImg{ groundNrmTex_.sampler(), groundNrmTex_.view(),
                                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

        VkDescriptorImageInfo  fntImg{ fontTex_.valid() ? fontTex_.sampler()
                                                        : transmittanceLut_.sampler(),
                                       fontTex_.valid() ? fontTex_.view()
                                                        : transmittanceLut_.view(),
                                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkWriteDescriptorSet w[5]{};
        w[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[0].dstSet          = frameSets_[i];
        w[0].dstBinding      = 0;
        w[0].descriptorCount = 1;
        w[0].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w[0].pBufferInfo     = &buf;
        w[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[1].dstSet          = frameSets_[i];
        w[1].dstBinding      = 2;
        w[1].descriptorCount = 1;
        w[1].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[1].pImageInfo      = &lutImg;
        w[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[2].dstSet          = frameSets_[i];
        w[2].dstBinding      = 4;
        w[2].descriptorCount = 1;
        w[2].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[2].pImageInfo      = &gndImg;
        w[3] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[3].dstSet          = frameSets_[i];
        w[3].dstBinding      = 5;
        w[3].descriptorCount = 1;
        w[3].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[3].pImageInfo      = &gnrImg;
        w[4] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w[4].dstSet          = frameSets_[i];
        w[4].dstBinding      = 6;
        w[4].descriptorCount = 1;
        w[4].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[4].pImageInfo      = &fntImg;
        vkUpdateDescriptorSets(device_, 5, w, 0, nullptr);
    }
}

void VulkanRenderer::createCompositeResources() {

    VkSamplerCreateInfo si{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    si.magFilter    = VK_FILTER_NEAREST;
    si.minFilter    = VK_FILTER_NEAREST;
    si.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    si.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    si.borderColor  = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
    vkCheck(vkCreateSampler(device_, &si, nullptr, &sceneSampler_), "vkCreateSampler");

    VkShaderModule vs = loadShader("shaders/composite.vert.spv");
    VkShaderModule fs = loadShader("shaders/composite.frag.spv");
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0] = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;   stages[0].module = vs; stages[0].pName = "main";
    stages[1] = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module = fs; stages[1].pName = "main";

    VkPipelineVertexInputStateCreateInfo vi{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    VkPipelineInputAssemblyStateCreateInfo ia{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    vp.viewportCount = 1;
    vp.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    cb.attachmentCount = 1;
    cb.pAttachments    = &cba;

    VkDynamicState dyn[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dsc{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dsc.dynamicStateCount = 2;
    dsc.pDynamicStates    = dyn;

    VkGraphicsPipelineCreateInfo gp{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    gp.stageCount          = 2;
    gp.pStages             = stages;
    gp.pVertexInputState   = &vi;
    gp.pInputAssemblyState = &ia;
    gp.pViewportState      = &vp;
    gp.pRasterizationState = &rs;
    gp.pMultisampleState   = &ms;
    gp.pColorBlendState    = &cb;
    gp.pDynamicState       = &dsc;
    gp.layout              = pipelineLayout_;
    gp.renderPass          = compositeRenderPass_;
    gp.subpass             = 0;
    vkCheck(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr,
                                      &compositePipeline_),
            "vkCreateGraphicsPipelines(composite)");

    vkDestroyShaderModule(device_, fs, nullptr);
    vkDestroyShaderModule(device_, vs, nullptr);
}

void VulkanRenderer::createOverlayPipeline() {
    if (!fontTex_.valid()) return;

    VkShaderModule vs = loadShader("shaders/overlay.vert.spv");
    VkShaderModule fs = loadShader("shaders/overlay.frag.spv");
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0] = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;   stages[0].module = vs; stages[0].pName = "main";
    stages[1] = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module = fs; stages[1].pName = "main";

    VkVertexInputBindingDescription bind{};
    bind.binding   = 0;
    bind.stride    = sizeof(OverlayVertex);
    bind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription attr[3]{};
    attr[0].location = 0; attr[0].format = VK_FORMAT_R32G32_SFLOAT;
    attr[0].offset = offsetof(OverlayVertex, x);
    attr[1].location = 1; attr[1].format = VK_FORMAT_R32G32_SFLOAT;
    attr[1].offset = offsetof(OverlayVertex, u);

    attr[2].location = 2; attr[2].format = VK_FORMAT_R8G8B8A8_UNORM;
    attr[2].offset = offsetof(OverlayVertex, rgba);

    VkPipelineVertexInputStateCreateInfo vi{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vi.vertexBindingDescriptionCount   = 1;
    vi.pVertexBindingDescriptions      = &bind;
    vi.vertexAttributeDescriptionCount = 3;
    vi.pVertexAttributeDescriptions    = attr;

    VkPipelineInputAssemblyStateCreateInfo ia{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    vp.viewportCount = 1; vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rs.polygonMode = VK_POLYGON_MODE_FILL;

    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState cba{};
    cba.blendEnable         = VK_TRUE;
    cba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    cba.colorBlendOp        = VK_BLEND_OP_ADD;
    cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    cba.alphaBlendOp        = VK_BLEND_OP_ADD;
    cba.colorWriteMask      = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                              VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    cb.attachmentCount = 1;
    cb.pAttachments    = &cba;

    VkDynamicState dyn[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dsc{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dsc.dynamicStateCount = 2;
    dsc.pDynamicStates    = dyn;

    VkGraphicsPipelineCreateInfo gp{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    gp.stageCount          = 2;
    gp.pStages             = stages;
    gp.pVertexInputState   = &vi;
    gp.pInputAssemblyState = &ia;
    gp.pViewportState      = &vp;
    gp.pRasterizationState = &rs;
    gp.pMultisampleState   = &ms;
    gp.pColorBlendState    = &cb;
    gp.pDynamicState       = &dsc;
    gp.layout              = pipelineLayout_;
    gp.renderPass          = compositeRenderPass_;
    gp.subpass             = 0;
    vkCheck(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr,
                                      &overlayPipeline_),
            "vkCreateGraphicsPipelines(overlay)");

    vkDestroyShaderModule(device_, fs, nullptr);
    vkDestroyShaderModule(device_, vs, nullptr);
}

void VulkanRenderer::setOverlay(const std::vector<OverlayVertex>& verts) {
    overlayVertexCount_ = 0;
    if (verts.empty() || !overlayPipeline_) return;

    const uint32_t f = currentFrame_;
    const VkDeviceSize need = VkDeviceSize(verts.size() * sizeof(OverlayVertex));

    if (need > overlayCapacity_[f]) {

        vkDeviceWaitIdle(device_);
        if (overlayBuf_[f]) vkDestroyBuffer(device_, overlayBuf_[f], nullptr);
        if (overlayMem_[f]) vkFreeMemory(device_, overlayMem_[f], nullptr);
        VkDeviceSize cap = overlayCapacity_[f] ? overlayCapacity_[f] : 4096;
        while (cap < need) cap *= 2;

        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size        = cap;
        bi.usage       = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device_, &bi, nullptr, &overlayBuf_[f]),
                "vkCreateBuffer(overlay)");
        VkMemoryRequirements mr{};
        vkGetBufferMemoryRequirements(device_, overlayBuf_[f], &mr);
        VkMemoryAllocateInfo mai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        mai.allocationSize  = mr.size;
        mai.memoryTypeIndex = findMemoryType(physical_, mr.memoryTypeBits,
                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                             VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device_, &mai, nullptr, &overlayMem_[f]),
                "vkAllocateMemory(overlay)");
        vkCheck(vkBindBufferMemory(device_, overlayBuf_[f], overlayMem_[f], 0),
                "vkBindBufferMemory(overlay)");
        vkCheck(vkMapMemory(device_, overlayMem_[f], 0, cap, 0, &overlayMapped_[f]),
                "vkMapMemory(overlay)");
        overlayCapacity_[f] = cap;
    }

    std::memcpy(overlayMapped_[f], verts.data(), size_t(need));
    overlayVertexCount_ = uint32_t(verts.size());
}

void VulkanRenderer::createRenderPass() {

    VkAttachmentDescription color{};
    color.format         = hdrFormat_;
    color.samples        = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;

    color.finalLayout    = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription depth{};
    depth.format         = depthFormat_;
    depth.samples        = VK_SAMPLE_COUNT_1_BIT;
    depth.loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;

    depth.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    depth.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;

    depth.finalLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    VkAttachmentReference colorRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    VkAttachmentReference depthRef{ 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

    VkSubpassDescription sub{};
    sub.pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sub.colorAttachmentCount    = 1;
    sub.pColorAttachments       = &colorRef;
    sub.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass    = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass    = 0;
    dep.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                        VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    VkSubpassDependency depOut{};
    depOut.srcSubpass    = 0;
    depOut.dstSubpass    = VK_SUBPASS_EXTERNAL;
    depOut.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                           VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    depOut.dstStageMask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    depOut.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                           VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    depOut.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    VkAttachmentDescription attachments[] = { color, depth };
    VkSubpassDependency     deps[]        = { dep, depOut };
    VkRenderPassCreateInfo rp{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    rp.attachmentCount = 2;
    rp.pAttachments    = attachments;
    rp.subpassCount    = 1;
    rp.pSubpasses      = &sub;
    rp.dependencyCount = 2;
    rp.pDependencies   = deps;
    vkCheck(vkCreateRenderPass(device_, &rp, nullptr, &sceneRenderPass_),
            "vkCreateRenderPass(scene)");

    VkAttachmentDescription present{};
    present.format         = colorFormat_;
    present.samples        = VK_SAMPLE_COUNT_1_BIT;
    present.loadOp         = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    present.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    present.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    present.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    present.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    present.finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference presentRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    VkSubpassDescription csub{};
    csub.pipelineBindPoint    = VK_PIPELINE_BIND_POINT_GRAPHICS;
    csub.colorAttachmentCount = 1;
    csub.pColorAttachments    = &presentRef;

    VkSubpassDependency cdep{};
    cdep.srcSubpass    = VK_SUBPASS_EXTERNAL;
    cdep.dstSubpass    = 0;
    cdep.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    cdep.dstStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    cdep.srcAccessMask = 0;
    cdep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo crp{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    crp.attachmentCount = 1;
    crp.pAttachments    = &present;
    crp.subpassCount    = 1;
    crp.pSubpasses      = &csub;
    crp.dependencyCount = 1;
    crp.pDependencies   = &cdep;
    vkCheck(vkCreateRenderPass(device_, &crp, nullptr, &compositeRenderPass_),
            "vkCreateRenderPass(composite)");
}

VkFormat VulkanRenderer::pickHdrFormat() const {

    const VkFormat candidates[] = { VK_FORMAT_R16G16B16A16_SFLOAT,
                                    VK_FORMAT_R32G32B32A32_SFLOAT };
    for (VkFormat f : candidates) {
        VkFormatProperties props{};
        vkGetPhysicalDeviceFormatProperties(physical_, f, &props);
        const VkFormatFeatureFlags need = VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT |
                                          VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT;
        if ((props.optimalTilingFeatures & need) == need) return f;
    }
    throw std::runtime_error("No supported float colour format for the HDR target");
}

static std::string exeDirectory() {
    char buf[MAX_PATH]{};
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    const std::string full(buf, n);
    const size_t cut = full.find_last_of("\\/");
    return (cut == std::string::npos) ? std::string{} : full.substr(0, cut + 1);
}

VkShaderModule VulkanRenderer::loadShader(const std::string& path) {

    const std::string beside = exeDirectory() + path;
    std::ifstream f(beside, std::ios::ate | std::ios::binary);
    if (!f.is_open()) f.open(path, std::ios::ate | std::ios::binary);
    if (!f.is_open())
        throw std::runtime_error("Cannot open shader: " + path +
                                 " (looked in " + beside + " and the working directory)");
    size_t size = static_cast<size_t>(f.tellg());
    std::vector<char> code(size);
    f.seekg(0);
    f.read(code.data(), size);
    f.close();

    VkShaderModuleCreateInfo ci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    ci.codeSize = code.size();
    ci.pCode    = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule module = VK_NULL_HANDLE;
    vkCheck(vkCreateShaderModule(device_, &ci, nullptr, &module), "vkCreateShaderModule");
    return module;
}

void VulkanRenderer::createPipelines() {
    const std::string dir = exeDir();
    VkShaderModule vert = loadShader(dir + "/shaders/planet.vert.spv");
    VkShaderModule frag = loadShader(dir + "/shaders/planet.frag.spv");

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag;
    stages[1].pName  = "main";

    auto binding = Mesh::bindingDescription();
    auto attrs   = Mesh::attributeDescriptions();
    VkPipelineVertexInputStateCreateInfo vi{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vi.vertexBindingDescriptionCount   = 1;
    vi.pVertexBindingDescriptions      = &binding;
    vi.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
    vi.pVertexAttributeDescriptions    = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo ia{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    vp.viewportCount = 1;
    vp.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
    ds.depthTestEnable  = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp   = VK_COMPARE_OP_GREATER;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    cb.attachmentCount = 1;
    cb.pAttachments    = &cba;

    VkDynamicState dyn[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dsc{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dsc.dynamicStateCount = 2;
    dsc.pDynamicStates    = dyn;

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pcr.offset     = 0;
    pcr.size       = sizeof(ObjectPush);
    VkPipelineLayoutCreateInfo pl{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pl.setLayoutCount         = 1;
    pl.pSetLayouts            = &frameSetLayout_;
    pl.pushConstantRangeCount = 1;
    pl.pPushConstantRanges    = &pcr;
    vkCheck(vkCreatePipelineLayout(device_, &pl, nullptr, &pipelineLayout_), "vkCreatePipelineLayout");

    VkGraphicsPipelineCreateInfo gp{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    gp.stageCount          = 2;
    gp.pStages             = stages;
    gp.pVertexInputState   = &vi;
    gp.pInputAssemblyState = &ia;
    gp.pViewportState      = &vp;
    gp.pRasterizationState = &rs;
    gp.pMultisampleState   = &ms;
    gp.pDepthStencilState  = &ds;
    gp.pColorBlendState    = &cb;
    gp.pDynamicState       = &dsc;
    gp.layout              = pipelineLayout_;
    gp.renderPass          = sceneRenderPass_;
    gp.subpass             = 0;
    vkCheck(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr, &fillPipeline_),
            "vkCreateGraphicsPipelines(fill)");

    rs.polygonMode = VK_POLYGON_MODE_LINE;
    vkCheck(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &gp, nullptr, &wirePipeline_),
            "vkCreateGraphicsPipelines(wire)");

    vkDestroyShaderModule(device_, vert, nullptr);
    vkDestroyShaderModule(device_, frag, nullptr);
}

void VulkanRenderer::createCommandPool() {
    VkCommandPoolCreateInfo pi{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    pi.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pi.queueFamilyIndex = graphicsFamily_;
    vkCheck(vkCreateCommandPool(device_, &pi, nullptr, &commandPool_), "vkCreateCommandPool");

    VkCommandBufferAllocateInfo ai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    ai.commandPool        = commandPool_;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = kFramesInFlight;
    vkCheck(vkAllocateCommandBuffers(device_, &ai, commandBuffers_.data()),
            "vkAllocateCommandBuffers");
}

void VulkanRenderer::createSyncObjects() {
    for (int i = 0; i < kFramesInFlight; ++i) {
        VkSemaphoreCreateInfo si{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        vkCheck(vkCreateSemaphore(device_, &si, nullptr, &imageAvailable_[i]),
                "vkCreateSemaphore(imageAvailable)");
        VkFenceCreateInfo fi{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        vkCheck(vkCreateFence(device_, &fi, nullptr, &inFlight_[i]), "vkCreateFence(inFlight)");
    }
}

void VulkanRenderer::notifyResize(uint32_t width, uint32_t height) {
    resizePending_ = true;
    pendingWidth_  = width;
    pendingHeight_ = height;
}

VkCommandBuffer VulkanRenderer::beginFrame(const FrameUniforms& frame, bool wireframe) {

    VkExtent2D win = queryClientExtent(hwnd_);
    if (resizePending_ || win.width != extent_.width || win.height != extent_.height) {
        if (win.width == 0 || win.height == 0) return VK_NULL_HANDLE;
        recreateSwapchain();
        resizePending_ = false;
    }
    if (extent_.width == 0 || extent_.height == 0) return VK_NULL_HANDLE;

    vkWaitForFences(device_, 1, &inFlight_[currentFrame_], VK_TRUE, UINT64_MAX);

    VkResult acq = vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX,
                                         imageAvailable_[currentFrame_],
                                         VK_NULL_HANDLE, &imageIndex_);
    if (acq == VK_ERROR_OUT_OF_DATE_KHR) { resizePending_ = true; return VK_NULL_HANDLE; }
    if (acq != VK_SUCCESS && acq != VK_SUBOPTIMAL_KHR)
        throw std::runtime_error("vkAcquireNextImageKHR failed");

    if (imagesInFlight_[imageIndex_] != VK_NULL_HANDLE)
        vkWaitForFences(device_, 1, &imagesInFlight_[imageIndex_], VK_TRUE, UINT64_MAX);
    imagesInFlight_[imageIndex_] = inFlight_[currentFrame_];
    vkResetFences(device_, 1, &inFlight_[currentFrame_]);

    std::memcpy(frameUboMapped_[currentFrame_], &frame, sizeof(FrameUniforms));

    VkCommandBuffer cmd = commandBuffers_[currentFrame_];
    vkResetCommandBuffer(cmd, 0);
    VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    vkBeginCommandBuffer(cmd, &bi);

    VkClearValue clears[2];

    clears[0].color        = { { 0.0f, 0.0f, 0.0f, 1.0f } };
    clears[1].depthStencil = { 0.0f, 0 };

    VkRenderPassBeginInfo rp{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    rp.renderPass        = sceneRenderPass_;
    rp.framebuffer       = sceneFramebuffers_[currentFrame_];
    rp.renderArea.extent = extent_;
    rp.clearValueCount   = 2;
    rp.pClearValues      = clears;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{ 0.0f, 0.0f, static_cast<float>(extent_.width),
                         static_cast<float>(extent_.height), 0.0f, 1.0f };
    VkRect2D scissor{ { 0, 0 }, extent_ };
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                      wireframe ? wirePipeline_ : fillPipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                            0, 1, &frameSets_[currentFrame_], 0, nullptr);

    return cmd;
}

void VulkanRenderer::endFrame() {
    VkCommandBuffer cmd = commandBuffers_[currentFrame_];
    vkCmdEndRenderPass(cmd);

    {
        VkRenderPassBeginInfo crp{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
        crp.renderPass        = compositeRenderPass_;
        crp.framebuffer       = compositeFramebuffers_[imageIndex_];
        crp.renderArea.extent = extent_;
        crp.clearValueCount   = 0;
        vkCmdBeginRenderPass(cmd, &crp, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport{ 0.0f, 0.0f, static_cast<float>(extent_.width),
                             static_cast<float>(extent_.height), 0.0f, 1.0f };
        VkRect2D scissor{ { 0, 0 }, extent_ };
        vkCmdSetViewport(cmd, 0, 1, &viewport);
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline_);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                                0, 1, &frameSets_[currentFrame_], 0, nullptr);
        vkCmdDraw(cmd, 3, 1, 0, 0);

        if (overlayVertexCount_ > 0 && overlayPipeline_ &&
            overlayBuf_[currentFrame_]) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, overlayPipeline_);
            const float pushVp[4] = { float(extent_.width), float(extent_.height),
                                      0.0f, 0.0f };
            vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT,
                               0, sizeof(pushVp), pushVp);
            VkDeviceSize zero = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &overlayBuf_[currentFrame_], &zero);
            vkCmdDraw(cmd, overlayVertexCount_, 1, 0, 0);
        }
        vkCmdEndRenderPass(cmd);
    }

    if (captureRequested_ && canCapture_ && captureBuf_) {
        VkImage img = images_[imageIndex_];
        auto barrier = [&](VkImageLayout from, VkImageLayout to, VkAccessFlags sa, VkAccessFlags da) {
            VkImageMemoryBarrier b{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
            b.oldLayout = from; b.newLayout = to;
            b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.image = img; b.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            b.srcAccessMask = sa; b.dstAccessMask = da;
            vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
        };
        barrier(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        VkBufferImageCopy region{};
        region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        region.imageExtent = { extent_.width, extent_.height, 1 };
        vkCmdCopyImageToBuffer(cmd, img, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               captureBuf_, 1, &region);
        barrier(VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                VK_ACCESS_TRANSFER_READ_BIT, 0);
        captureExtent_    = extent_;
        captureRequested_ = false;
        captureHasData_   = true;
    }

    vkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.waitSemaphoreCount   = 1;
    si.pWaitSemaphores      = &imageAvailable_[currentFrame_];
    si.pWaitDstStageMask    = &waitStage;
    si.commandBufferCount   = 1;
    si.pCommandBuffers      = &cmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores    = &renderFinished_[imageIndex_];
    vkCheck(vkQueueSubmit(graphicsQueue_, 1, &si, inFlight_[currentFrame_]), "vkQueueSubmit");

    VkPresentInfoKHR pi{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores    = &renderFinished_[imageIndex_];
    pi.swapchainCount     = 1;
    pi.pSwapchains        = &swapchain_;
    pi.pImageIndices      = &imageIndex_;
    VkResult pr = vkQueuePresentKHR(presentQueue_, &pi);
    if (pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR) resizePending_ = true;
    else if (pr != VK_SUCCESS) throw std::runtime_error("vkQueuePresentKHR failed");

    currentFrame_ = (currentFrame_ + 1) % kFramesInFlight;
    ++frameNumber_;
}

void VulkanRenderer::requestCapture(const std::string& path) {
    if (!canCapture_) return;
    capturePath_ = path;
    const VkDeviceSize need = VkDeviceSize(extent_.width) * extent_.height * 4;
    if (need != captureSize_) {
        if (captureBuf_) vkDestroyBuffer(device_, captureBuf_, nullptr);
        if (captureMem_) vkFreeMemory(device_, captureMem_, nullptr);
        captureBuf_ = VK_NULL_HANDLE; captureMem_ = VK_NULL_HANDLE;
        VkBufferCreateInfo bi{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bi.size = need; bi.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        vkCheck(vkCreateBuffer(device_, &bi, nullptr, &captureBuf_), "vkCreateBuffer(capture)");
        VkMemoryRequirements req{}; vkGetBufferMemoryRequirements(device_, captureBuf_, &req);
        VkMemoryAllocateInfo ai{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = findMemoryType(physical_, req.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkCheck(vkAllocateMemory(device_, &ai, nullptr, &captureMem_), "vkAllocateMemory(capture)");
        vkBindBufferMemory(device_, captureBuf_, captureMem_, 0);
        captureSize_ = need;
    }
    captureRequested_ = true;
    captureHasData_   = false;
}

bool VulkanRenderer::writeCapture() {
    if (!captureHasData_ || !captureMem_) return false;
    vkDeviceWaitIdle(device_);

    void* mapped = nullptr;
    vkMapMemory(device_, captureMem_, 0, captureSize_, 0, &mapped);
    const uint8_t* px = static_cast<const uint8_t*>(mapped);
    const uint32_t w = captureExtent_.width, h = captureExtent_.height;
    std::vector<uint8_t> rgb(size_t(w) * h * 3);
    for (size_t i = 0; i < size_t(w) * h; ++i) {
        rgb[i*3+0] = px[i*4+2];
        rgb[i*3+1] = px[i*4+1];
        rgb[i*3+2] = px[i*4+0];
    }
    vkUnmapMemory(device_, captureMem_);

    bool ok = false;
    std::ofstream f(capturePath_, std::ios::binary);
    if (f.is_open()) {
        f << "P6\n" << w << " " << h << "\n255\n";
        f.write(reinterpret_cast<const char*>(rgb.data()), std::streamsize(rgb.size()));
        ok = f.good();
    }
    captureHasData_ = false;
    return ok;
}

void VulkanRenderer::shutdown() {
    if (device_) vkDeviceWaitIdle(device_);
    if (captureBuf_) vkDestroyBuffer(device_, captureBuf_, nullptr);
    if (captureMem_) vkFreeMemory(device_, captureMem_, nullptr);
    captureBuf_ = VK_NULL_HANDLE; captureMem_ = VK_NULL_HANDLE;
    destroySwapchainResources();

    if (fillPipeline_)      vkDestroyPipeline(device_, fillPipeline_, nullptr);
    if (wirePipeline_)      vkDestroyPipeline(device_, wirePipeline_, nullptr);
    if (compositePipeline_) vkDestroyPipeline(device_, compositePipeline_, nullptr);
    if (pipelineLayout_)    vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
    transmittanceLut_ = Texture2D{};
    groundTex_        = Texture2D{};
    groundNrmTex_     = Texture2D{};
    fontTex_          = Texture2D{};
    if (overlayPipeline_) vkDestroyPipeline(device_, overlayPipeline_, nullptr);
    overlayPipeline_ = VK_NULL_HANDLE;
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        if (overlayBuf_[i]) vkDestroyBuffer(device_, overlayBuf_[i], nullptr);
        if (overlayMem_[i]) vkFreeMemory(device_, overlayMem_[i], nullptr);
        overlayBuf_[i] = VK_NULL_HANDLE;
        overlayMem_[i] = VK_NULL_HANDLE;
        overlayMapped_[i] = nullptr;
        overlayCapacity_[i] = 0;
    }

    if (descriptorPool_) vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    if (frameSetLayout_) vkDestroyDescriptorSetLayout(device_, frameSetLayout_, nullptr);
    if (sceneSampler_)   vkDestroySampler(device_, sceneSampler_, nullptr);
    for (uint32_t i = 0; i < kFramesInFlight; ++i) {
        if (frameUboMapped_[i]) vkUnmapMemory(device_, frameUboMems_[i]);
        if (frameUbos_[i])      vkDestroyBuffer(device_, frameUbos_[i], nullptr);
        if (frameUboMems_[i])   vkFreeMemory(device_, frameUboMems_[i], nullptr);
        frameUboMapped_[i] = nullptr;
        frameUbos_[i]      = VK_NULL_HANDLE;
        frameUboMems_[i]   = VK_NULL_HANDLE;
    }
    if (sceneRenderPass_)     vkDestroyRenderPass(device_, sceneRenderPass_, nullptr);
    if (compositeRenderPass_) vkDestroyRenderPass(device_, compositeRenderPass_, nullptr);

    for (int i = 0; i < kFramesInFlight; ++i) {
        if (imageAvailable_[i]) vkDestroySemaphore(device_, imageAvailable_[i], nullptr);
        if (inFlight_[i])       vkDestroyFence(device_, inFlight_[i], nullptr);
    }
    if (commandPool_) vkDestroyCommandPool(device_, commandPool_, nullptr);
    if (device_)      vkDestroyDevice(device_, nullptr);

    if (messenger_) {
        auto fn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (fn) fn(instance_, messenger_, nullptr);
    }
    if (surface_)  vkDestroySurfaceKHR(instance_, surface_, nullptr);
    if (instance_) vkDestroyInstance(instance_, nullptr);

    device_ = VK_NULL_HANDLE;
    instance_ = VK_NULL_HANDLE;
}

}
