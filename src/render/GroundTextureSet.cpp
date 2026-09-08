#include "render/GroundTextureSet.h"

#include <windows.h>
#include <algorithm>
#include <cctype>
#include <vector>

namespace planet {
namespace groundtex {
namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return char(std::tolower(c)); });
    return s;
}

bool dirExists(const std::string& p) {
    const DWORD a = GetFileAttributesA(p.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY);
}

}

static std::string findFolderIn(const std::string& root, const std::string& want,
                                int depth) {
    if (depth > 3 || !dirExists(root)) return {};
    WIN32_FIND_DATAA fd{};
    const HANDLE h = FindFirstFileA((root + "/*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return {};
    std::vector<std::string> subdirs;
    std::string found;
    do {
        if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) continue;
        const std::string name = lower(fd.cFileName);
        if (name == "." || name == "..") continue;

        if (found.empty() && name.find(want) != std::string::npos) {
            const std::string cand = root + "/" + fd.cFileName;
            if (!findMap(cand, "basecolor").empty()) found = cand;
        }
        subdirs.push_back(root + "/" + fd.cFileName);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    if (!found.empty()) return found;
    for (const std::string& sub : subdirs) {
        const std::string deeper = findFolderIn(sub, want, depth + 1);
        if (!deeper.empty()) return deeper;
    }
    return {};
}

std::string findFolder(const std::string& token) {
    const std::string want = lower(token);
    for (const std::string& root : searchRoots()) {
        const std::string found = findFolderIn(root, want, 0);
        if (!found.empty()) return found;
    }
    return {};
}

std::string findMap(const std::string& dir, const char* token) {
    const std::string want = lower(token);
    WIN32_FIND_DATAA fd{};
    const HANDLE h = FindFirstFileA((dir + "/*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return {};
    std::string found;
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        const std::string name = lower(fd.cFileName);

        if (name.find(want) == std::string::npos) continue;

        const size_t dot = name.rfind('.');
        if (dot == std::string::npos) continue;
        const std::string ext = name.substr(dot);
        if (ext != ".jpg" && ext != ".jpeg" && ext != ".png") continue;
        found = dir + "/" + fd.cFileName;
        break;
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    return found;
}

}
}
