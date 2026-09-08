#include "platform/FontAtlas.h"

#include <windows.h>
#include <cstring>

namespace planet {

FontAtlas buildFontAtlas(const char* face, int pixelHeight) {
    FontAtlas atlas;

    HFONT font = CreateFontA(-pixelHeight, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                             DEFAULT_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
                             ANTIALIASED_QUALITY, FIXED_PITCH | FF_MODERN, face);
    if (!font) return atlas;

    HDC dc = CreateCompatibleDC(nullptr);
    if (!dc) { DeleteObject(font); return atlas; }
    HGDIOBJ oldFont = SelectObject(dc, font);

    TEXTMETRICA tm{};
    GetTextMetricsA(dc, &tm);

    uint32_t cw = uint32_t(tm.tmAveCharWidth);
    uint32_t ch = uint32_t(tm.tmHeight);
    if (cw == 0) cw = uint32_t(pixelHeight / 2 + 1);
    if (ch == 0) ch = uint32_t(pixelHeight);

    const uint32_t cols = 16;
    const uint32_t rows = (kFontCharCount + cols - 1) / cols + 1;
    const uint32_t W = cols * cw;
    const uint32_t H = rows * ch;

    BITMAPINFO bi{};
    bi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth       = int(W);
    bi.bmiHeader.biHeight      = -int(H);
    bi.bmiHeader.biPlanes      = 1;
    bi.bmiHeader.biBitCount    = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    void* bits = nullptr;
    HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
    if (!bmp || !bits) {
        SelectObject(dc, oldFont); DeleteDC(dc); DeleteObject(font);
        return atlas;
    }
    HGDIOBJ oldBmp = SelectObject(dc, bmp);

    RECT full{ 0, 0, int(W), int(H) };
    FillRect(dc, &full, HBRUSH(GetStockObject(BLACK_BRUSH)));
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB(255, 255, 255));

    for (int i = 0; i < kFontCharCount; ++i) {
        const char c = char(kFontFirstChar + i);
        const int cx = int((uint32_t(i) % cols) * cw);
        const int cy = int((uint32_t(i) / cols) * ch);
        TextOutA(dc, cx, cy, &c, 1);
    }

    const uint32_t whiteRow = rows - 1;
    RECT wr{ 0, int(whiteRow * ch), int(W), int(H) };
    FillRect(dc, &wr, HBRUSH(GetStockObject(WHITE_BRUSH)));

    GdiFlush();

    atlas.pixels.resize(size_t(W) * H);
    const uint8_t* src = static_cast<const uint8_t*>(bits);
    for (size_t i = 0; i < size_t(W) * H; ++i) atlas.pixels[i] = src[i * 4];

    atlas.width  = W;
    atlas.height = H;
    atlas.cellW  = cw;
    atlas.cellH  = ch;
    atlas.cols   = cols;
    atlas.ascent = tm.tmAscent;

    atlas.whiteU = 0.5f;
    atlas.whiteV = (float(whiteRow) + 0.5f) * float(ch) / float(H);

    SelectObject(dc, oldBmp);
    DeleteObject(bmp);
    SelectObject(dc, oldFont);
    DeleteDC(dc);
    DeleteObject(font);
    return atlas;
}

}
