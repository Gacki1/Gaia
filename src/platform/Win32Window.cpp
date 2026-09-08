#include "platform/Win32Window.h"
#include <cstdio>

namespace planet {

static const char* kClassName = "ProceduralPlanetWindow";

#ifndef PLANET_DPI_PER_MONITOR_V2
#define PLANET_DPI_PER_MONITOR_V2 (reinterpret_cast<HANDLE>(-4))
#endif

bool Win32Window::create(const char* title, int width, int height,
                         bool background) {
    background_ = background;
    hinst_ = GetModuleHandleA(nullptr);

    using SetDpiCtxFn = BOOL(WINAPI*)(HANDLE);
    if (HMODULE user32 = GetModuleHandleA("user32.dll")) {
        if (auto setCtx = reinterpret_cast<SetDpiCtxFn>(
                GetProcAddress(user32, "SetProcessDpiAwarenessContext")))
            setCtx(PLANET_DPI_PER_MONITOR_V2);
    }

    WNDCLASSEXA wc{};
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wc.lpfnWndProc   = &Win32Window::wndProc;
    wc.hInstance     = hinst_;
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = kClassName;
    RegisterClassExA(&wc);

    RECT rc{ 0, 0, width, height };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);

    const DWORD exStyle = background ? WS_EX_NOACTIVATE : 0;
    hwnd_ = CreateWindowExA(exStyle, kClassName, title, WS_OVERLAPPEDWINDOW,
                            CW_USEDEFAULT, CW_USEDEFAULT,
                            rc.right - rc.left, rc.bottom - rc.top,
                            nullptr, nullptr, hinst_, this);
    if (!hwnd_) {
        std::fprintf(stderr, "CreateWindowExA failed (GetLastError=%lu)\n",
                     static_cast<unsigned long>(GetLastError()));
        return false;
    }

    width_  = static_cast<uint32_t>(width);
    height_ = static_cast<uint32_t>(height);

    RAWINPUTDEVICE rid{};
    rid.usUsagePage = 0x01;
    rid.usUsage     = 0x02;
    rid.dwFlags     = 0;
    rid.hwndTarget  = hwnd_;
    RegisterRawInputDevices(&rid, 1, sizeof(rid));

    ShowWindow(hwnd_, background ? SW_SHOWNOACTIVATE : SW_SHOW);
    UpdateWindow(hwnd_);
    return true;
}

void Win32Window::setFullscreen(bool on) {
    if (!hwnd_ || on == fullscreen_) return;

    if (on) {

        placement_.length = sizeof(placement_);
        GetWindowPlacement(hwnd_, &placement_);

        MONITORINFO mi{ sizeof(mi) };
        GetMonitorInfo(MonitorFromWindow(hwnd_, MONITOR_DEFAULTTONEAREST), &mi);

        SetWindowLongPtrA(hwnd_, GWL_STYLE, WS_POPUP | WS_VISIBLE);

        SetWindowPos(hwnd_, HWND_TOP,
                     mi.rcMonitor.left, mi.rcMonitor.top,
                     mi.rcMonitor.right  - mi.rcMonitor.left,
                     mi.rcMonitor.bottom - mi.rcMonitor.top,
                     SWP_FRAMECHANGED | SWP_NOOWNERZORDER);
    } else {
        SetWindowLongPtrA(hwnd_, GWL_STYLE, WS_OVERLAPPEDWINDOW | WS_VISIBLE);
        SetWindowPlacement(hwnd_, &placement_);
        SetWindowPos(hwnd_, nullptr, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER |
                     SWP_FRAMECHANGED | SWP_NOOWNERZORDER);
    }
    fullscreen_ = on;

    setCursorCapture(mouseLook_ && focused_);
}

void Win32Window::destroy() {
    setCursorCapture(false);
    if (hwnd_) { DestroyWindow(hwnd_); hwnd_ = nullptr; }
    UnregisterClassA(kClassName, hinst_);
}

LRESULT CALLBACK Win32Window::wndProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l) {
    if (msg == WM_NCCREATE) {
        auto* cs = reinterpret_cast<CREATESTRUCTA*>(l);
        SetWindowLongPtrA(hwnd, GWLP_USERDATA,
                          reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }
    auto* self = reinterpret_cast<Win32Window*>(GetWindowLongPtrA(hwnd, GWLP_USERDATA));
    if (self) {

        self->hwnd_ = hwnd;
        return self->handle(msg, w, l);
    }
    return DefWindowProcA(hwnd, msg, w, l);
}

LRESULT Win32Window::handle(UINT msg, WPARAM w, LPARAM l) {
    switch (msg) {
    case WM_CLOSE:
        closing_ = true;
        return 0;
    case WM_DESTROY:
        closing_ = true;
        PostQuitMessage(0);
        return 0;
    case WM_SIZE: {
        width_  = LOWORD(l);
        height_ = HIWORD(l);
        return 0;
    }
    case WM_SETFOCUS:
        focused_ = true;
        setCursorCapture(mouseLook_);
        return 0;
    case WM_KILLFOCUS:
        focused_ = false;
        setCursorCapture(false);

        keys_.fill(false);
        pressedEdge_.fill(false);
        mouseDx_ = mouseDy_ = 0.0f;
        wheel_   = 0.0f;
        return 0;
    case WM_KEYDOWN:
    case WM_SYSKEYDOWN: {
        if (w < 256) {
            if (!keys_[w] && !(l & (1 << 30))) pressedEdge_[w] = true;
            keys_[w] = true;
        }
        return 0;
    }
    case WM_KEYUP:
    case WM_SYSKEYUP:
        if (w < 256) keys_[w] = false;
        return 0;
    case WM_MOUSEWHEEL:
        wheel_ += static_cast<float>(GET_WHEEL_DELTA_WPARAM(w)) / WHEEL_DELTA;
        return 0;

    case WM_LBUTTONDOWN: setMouseButton(VK_LBUTTON, true);  return 0;
    case WM_LBUTTONUP:   setMouseButton(VK_LBUTTON, false); return 0;
    case WM_RBUTTONDOWN: setMouseButton(VK_RBUTTON, true);  return 0;
    case WM_RBUTTONUP:   setMouseButton(VK_RBUTTON, false); return 0;
    case WM_MBUTTONDOWN: setMouseButton(VK_MBUTTON, true);  return 0;
    case WM_MBUTTONUP:   setMouseButton(VK_MBUTTON, false); return 0;

    case WM_INPUT: {
        UINT size = 0;
        GetRawInputData(reinterpret_cast<HRAWINPUT>(l), RID_INPUT, nullptr, &size,
                        sizeof(RAWINPUTHEADER));
        if (size > 0 && size <= 128) {
            BYTE buf[128];
            if (GetRawInputData(reinterpret_cast<HRAWINPUT>(l), RID_INPUT, buf, &size,
                                sizeof(RAWINPUTHEADER)) == size) {
                auto* ri = reinterpret_cast<RAWINPUT*>(buf);
                if (ri->header.dwType == RIM_TYPEMOUSE &&
                    (ri->data.mouse.usFlags & MOUSE_MOVE_ABSOLUTE) == 0) {
                    mouseDx_ += static_cast<float>(ri->data.mouse.lLastX);
                    mouseDy_ += static_cast<float>(ri->data.mouse.lLastY);
                }
            }
        }
        return 0;
    }
    default:
        break;
    }
    return DefWindowProcA(hwnd_, msg, w, l);
}

void Win32Window::mousePosition(float& x, float& y) const {
    POINT p{};
    if (!hwnd_ || !GetCursorPos(&p) || !ScreenToClient(hwnd_, &p)) {
        x = y = -1.0f;
        return;
    }
    x = float(p.x);
    y = float(p.y);
}

void Win32Window::setMouseLook(bool on) {
    if (mouseLook_ == on) return;
    mouseLook_ = on;

    mouseDx_ = mouseDy_ = 0.0f;
    setCursorCapture(on && focused_);
}

void Win32Window::setCursorCapture(bool capture) {

    if (background_) capture = false;
    if (capture && hwnd_) {
        RECT rc; GetClientRect(hwnd_, &rc);
        POINT tl{ rc.left, rc.top }, br{ rc.right, rc.bottom };
        ClientToScreen(hwnd_, &tl); ClientToScreen(hwnd_, &br);
        RECT clip{ tl.x, tl.y, br.x, br.y };
        ClipCursor(&clip);
        while (ShowCursor(FALSE) >= 0) {}
    } else {
        ClipCursor(nullptr);
        while (ShowCursor(TRUE) < 0) {}
    }
}

bool Win32Window::pump() {
    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
        if (msg.message == WM_QUIT) closing_ = true;
    }
    return !closing_;
}

bool Win32Window::keyPressed(int vk) {
    if (vk < 0 || vk >= 256) return false;
    bool p = pressedEdge_[vk];
    pressedEdge_[vk] = false;
    return p;
}

float Win32Window::consumeWheel() {
    const float w = wheel_;
    wheel_ = 0.0f;
    return w;
}

void Win32Window::setMouseButton(int vk, bool down) {
    if (vk < 0 || vk >= 256) return;
    if (down && !keys_[vk]) pressedEdge_[vk] = true;
    keys_[vk] = down;
}

void Win32Window::consumeMouseDelta(float& dx, float& dy) {

    if (!mouseLook_) { dx = dy = 0.0f; mouseDx_ = mouseDy_ = 0.0f; return; }
    dx = mouseDx_; dy = mouseDy_;
    mouseDx_ = mouseDy_ = 0.0f;
}

void Win32Window::setTitle(const char* title) {
    if (hwnd_) SetWindowTextA(hwnd_, title);
}

}
