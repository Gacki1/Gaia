cmake_minimum_required(VERSION 3.16)
project(ProceduralPlanet CXX)

# ---------------------------------------------------------------------------
#  A native Vulkan cube-sphere planet renderer. Dependencies: only the Vulkan
#  SDK (headers + loader + glslc) and the Windows SDK -- windowing is raw Win32,
#  so no GLFW/SDL/vcpkg/network is required. Math is self-contained.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For single-config generators (e.g. "NMake Makefiles" in a VS dev prompt) the
# plain `cmake -S . -B build` command should still land the executable at
# build/Release/planet.exe (the documented path). Some CMake versions default
# CMAKE_BUILD_TYPE to Debug for these generators, so promote that default to
# Release. Multi-config generators (Ninja Multi-Config via build.bat, VS) are
# untouched and honour --config, so `build.bat Debug` still gives a debug build.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
endif()

# Always emit binaries to build/<Config>/ so the launch path is predictable:
#   build\Release\planet.exe   (and build\Release\planet_tests.exe)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>")

add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)

find_package(Vulkan REQUIRED)
# Patch generation runs on a worker pool (src/platform/JobSystem.h).
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
#  The planet application (Win32 subsystem -> WinMain, single window).
# ---------------------------------------------------------------------------
# Sources are GLOBbed, not listed. The hand-written list this replaced had
# already rotted: six headers added over the last two milestones were never in
# it, and nothing complained -- a missing header only costs you IDE navigation,
# so the list silently drifts. CONFIGURE_DEPENDS makes the build re-glob when a
# file appears or disappears.
file(GLOB_RECURSE _planet_sources CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

add_executable(planet WIN32 ${_planet_sources})

# Show the folders as folders in Visual Studio / Rider rather than as one flat
# "Source Files" bucket -- which is most of the point of having them.
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" FILES ${_planet_sources})

# src/ on the include path, so every #include is written from the SOURCE ROOT
# ("world/CubeSphere.h") instead of relative to the including file. Relative
# includes would mean the same header is spelled differently depending on where
# it is included from, and moving a file would then rewrite its dependents
# rather than just itself.
target_include_directories(planet PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Where the ground material folders live. Compiled in rather than copied next to
# the executable: the scans are tens of megabytes each and are downscaled at
# load time anyway, so duplicating them into the build tree would cost disk for
# nothing. The loader still checks an exe-relative "textures/" first, so a
# shipped build needs no rebuild.
target_compile_definitions(planet PRIVATE
    PLANET_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# gdi32: the font rasteriser (src/platform/FontAtlas.cpp). Same reasoning as
# windowscodecs below -- the Windows SDK is already a hard requirement, so a
# text overlay costs a library that is present rather than a vendored header.
# windowscodecs + ole32: the WIC image decoder (src/platform/ImageLoad.cpp).
# Part of the Windows SDK this build already requires -- no new dependency, and
# the reason there is no vendored image library in this repo.
target_link_libraries(planet PRIVATE Vulkan::Vulkan user32 gdi32 windowscodecs ole32
                                     Threads::Threads)

# ---------------------------------------------------------------------------
#  Compile GLSL shaders to SPIR-V next to the executable (Req: no manual step).
# ---------------------------------------------------------------------------
if(NOT Vulkan_GLSLC_EXECUTABLE)
    find_program(Vulkan_GLSLC_EXECUTABLE glslc
                 HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin")
endif()
if(NOT Vulkan_GLSLC_EXECUTABLE)
    message(FATAL_ERROR "glslc not found. Install the Vulkan SDK / set VULKAN_SDK.")
endif()

# Every .vert/.frag under shaders/ is compiled. A glob rather than a hardcoded
# list, because the shader count grows with each rendering feature and a
# forgotten entry shows up as a confusing "file not found" at runtime.
# -I lets shaders share constants through an include.
file(GLOB _shader_sources
     "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.vert"
     "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.frag")
set(_shader_dir "$<TARGET_FILE_DIR:planet>/shaders")
set(_shader_cmds COMMAND ${CMAKE_COMMAND} -E make_directory "${_shader_dir}")
foreach(_shader ${_shader_sources})
    get_filename_component(_name "${_shader}" NAME)
    list(APPEND _shader_cmds
         COMMAND ${Vulkan_GLSLC_EXECUTABLE}
                 -I "${CMAKE_CURRENT_SOURCE_DIR}/shaders"
                 "${_shader}" -o "${_shader_dir}/${_name}.spv")
endforeach()
add_custom_command(TARGET planet POST_BUILD
    ${_shader_cmds}
    COMMENT "Compiling GLSL shaders to SPIR-V"
    VERBATIM)

# ---------------------------------------------------------------------------
#  Headless unit tests (no GPU / window). Console subsystem.
# ---------------------------------------------------------------------------
enable_testing()
add_executable(planet_tests tests/tests.cpp)
target_include_directories(planet_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(planet_tests PRIVATE Threads::Threads)  # JobSystem tests
add_test(NAME PlanetTests COMMAND planet_tests)
