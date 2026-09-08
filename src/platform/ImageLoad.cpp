#include "platform/ImageLoad.h"

#include <windows.h>
#include <wincodec.h>
#include <cstdio>

namespace planet {
namespace {

bool ensureCom() {
    static const bool ok = [] {
        const HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED |
                                                   COINIT_DISABLE_OLE1DDE);

        return SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE;
    }();
    return ok;
}

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), w.data(), n);
    return w;
}

template <class T> void release(T*& p) { if (p) { p->Release(); p = nullptr; } }

}

LoadedImage loadImageRGBA(const std::string& path, uint32_t maxSize) {
    LoadedImage out;
    if (!ensureCom()) return out;

    IWICImagingFactory*    factory = nullptr;
    IWICBitmapDecoder*     decoder = nullptr;
    IWICBitmapFrameDecode* frame   = nullptr;
    IWICFormatConverter*   conv    = nullptr;
    IWICBitmapScaler*      scaler  = nullptr;

    auto cleanup = [&] {
        release(scaler); release(conv); release(frame);
        release(decoder); release(factory);
    };

    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                IID_PPV_ARGS(&factory)))) { cleanup(); return out; }

    const std::wstring wide = widen(path);
    if (FAILED(factory->CreateDecoderFromFilename(wide.c_str(), nullptr, GENERIC_READ,
                                                  WICDecodeMetadataCacheOnDemand,
                                                  &decoder))) { cleanup(); return out; }
    if (FAILED(decoder->GetFrame(0, &frame))) { cleanup(); return out; }

    UINT w = 0, h = 0;
    if (FAILED(frame->GetSize(&w, &h)) || w == 0 || h == 0) { cleanup(); return out; }

    IWICBitmapSource* src = frame;
    UINT dw = w, dh = h;
    if (maxSize > 0 && (w > maxSize || h > maxSize)) {

        const double s = double(maxSize) / double(w > h ? w : h);
        dw = UINT(double(w) * s + 0.5); if (dw < 1) dw = 1;
        dh = UINT(double(h) * s + 0.5); if (dh < 1) dh = 1;
        if (FAILED(factory->CreateBitmapScaler(&scaler))) { cleanup(); return out; }
        if (FAILED(scaler->Initialize(frame, dw, dh, WICBitmapInterpolationModeFant))) {
            cleanup(); return out;
        }
        src = scaler;
    }

    if (FAILED(factory->CreateFormatConverter(&conv))) { cleanup(); return out; }
    if (FAILED(conv->Initialize(src, GUID_WICPixelFormat32bppRGBA,
                                WICBitmapDitherTypeNone, nullptr, 0.0,
                                WICBitmapPaletteTypeCustom))) { cleanup(); return out; }
    src = conv;

    out.width  = dw;
    out.height = dh;
    out.rgba.resize(size_t(dw) * dh * 4);
    const UINT stride = dw * 4;
    if (FAILED(src->CopyPixels(nullptr, stride, UINT(out.rgba.size()), out.rgba.data()))) {
        out = LoadedImage{};
    }

    cleanup();
    return out;
}

}
