#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <array>
#include <cstdint>

namespace planet {

class Win32Window {
public:

    bool create(const char* title, int width, int height, bool background = false);
    void destroy();

    bool pump();

    bool keyDown(int vk) const { return vk >= 0 && vk < 256 && keys_[vk]; }

    bool keyPressed(int vk);

    void consumeMouseDelta(float& dx, float& dy);

    float consumeWheel();

    void mousePosition(float& x, float& y) const;

    void setMouseLook(bool on);
    bool mouseLook() const { return mouseLook_; }

    void setFullscreen(bool on);
    bool fullscreen() const { return fullscreen_; }

    bool     shouldClose() const { return closing_; }
    uint32_t width()  const { return width_; }
    uint32_t height() const { return height_; }
    void     setTitle(const char* title);

    HWND      hwnd()      const { return hwnd_; }
    HINSTANCE hinstance() const { return hinst_; }

private:
    static LRESULT CALLBACK wndProc(HWND, UINT, WPARAM, LPARAM);
    LRESULT handle(UINT msg, WPARAM w, LPARAM l);
    void setCursorCapture(bool capture);
    void setMouseButton(int vk, bool down);

    HINSTANCE hinst_ = nullptr;
    HWND      hwnd_  = nullptr;
    uint32_t  width_ = 0, height_ = 0;
    bool      closing_ = false;
    bool      focused_ = false;
    bool      fullscreen_ = false;
    WINDOWPLACEMENT placement_{};

    std::array<bool, 256> keys_{};
    std::array<bool, 256> pressedEdge_{};
    bool  background_ = false;
    bool  mouseLook_ = true;
    float mouseDx_ = 0.0f, mouseDy_ = 0.0f;
    float wheel_   = 0.0f;
};

}
