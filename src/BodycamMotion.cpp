#include <EGL/egl.h>
#include <dlfcn.h>
#include <link.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <optional>

#include <pl/Config.hpp>
#include <pl/Input.hpp>
#include <pl/Mod.hpp>
#include <pl/memory/Hook.hpp>
#include <pl/modmenu.hpp>

#include "Motion.hpp"
#include "Spring.hpp"

namespace bodycam {

// ============================================================
// RUNTIME GATE + CONFIG PERSISTENCE
// ============================================================
std::atomic<bool> gEnabled{true};

namespace {
pl::config::ConfigFile<Tuning> gCfg;
bool gCfgLoaded = false;

void ensureConfig() { if (!gCfgLoaded) gCfgLoaded = gCfg.load(); }

void writeEnabled(bool on) {
    ensureConfig();
    Tuning t = gCfg.value();
    t.enabled = on;
    gCfg.set(t);
    gCfg.save();
}
} // anon

bool isEnabled() { return gEnabled.load(std::memory_order_relaxed); }

void setEnabled(bool on, bool persist = true) {
    gEnabled.store(on, std::memory_order_relaxed);
    if (!on) { /* springs reset on next update */ }
    if (persist) writeEnabled(on);
}

// ============================================================
// SIGNATURE SCANNER (no pl/memory/Memory.hpp needed)
// ============================================================
namespace sigscan {

struct Pattern {
    uint8_t bytes[128];
    uint8_t mask[128];
    size_t len = 0;
};

inline bool parse(Pattern &p, const char *sig) {
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return 0;
    };
    p.len = 0;
    while (*sig && p.len < sizeof(p.bytes)) {
        while (*sig == ' ') ++sig;
        if (!*sig) break;
        if (*sig == '?') {
            p.bytes[p.len] = 0; p.mask[p.len] = 0; ++p.len;
            while (*sig == '?') ++sig;
        } else {
            p.bytes[p.len] = static_cast<uint8_t>(hex(sig[0]) << 4 | hex(sig[1]));
            p.mask[p.len] = 1; ++p.len; sig += 2;
        }
    }
    return p.len > 0;
}

struct Ctx { const Pattern *pat; void *found; };

inline int callback(struct dl_phdr_info *info, size_t, void *data) {
    auto *ctx = static_cast<Ctx *>(data);
    if (!info->dlpi_name || !std::strstr(info->dlpi_name, "libminecraftpe.so")) return 0;
    const Pattern &pat = *ctx->pat;
    size_t anchor = 0;
    while (anchor < pat.len && !pat.mask[anchor]) ++anchor;
    if (anchor >= pat.len) return 0;

    for (int i = 0; i < static_cast<int>(info->dlpi_phnum); ++i) {
        const ElfW(Phdr) &ph = info->dlpi_phdr[i];
        if (ph.p_type != PT_LOAD || !(ph.p_flags & PF_X)) continue;
        const uint8_t *start = reinterpret_cast<const uint8_t *>(info->dlpi_addr + ph.p_vaddr);
        const size_t len = ph.p_memsz;
        if (len <= pat.len) continue;
        const size_t last = len - pat.len;
        size_t pos = anchor;
        while (pos < len) {
            const uint8_t *hit = static_cast<const uint8_t *>(
                std::memchr(start + pos, pat.bytes[anchor], len - pos));
            if (!hit) break;
            const size_t at = static_cast<size_t>(hit - start);
            const size_t off = at - anchor;
            if (off <= last) {
                size_t k = 0;
                for (; k < pat.len; ++k)
                    if (pat.mask[k] && start[off + k] != pat.bytes[k]) break;
                if (k == pat.len) {
                    ctx->found = const_cast<uint8_t *>(start + off);
                    return 1;
                }
            }
            pos = at + 1;
        }
    }
    return 0;
}

inline void *find(const Pattern &pat) {
    Ctx ctx{&pat, nullptr};
    dl_iterate_phdr(callback, &ctx);
    return ctx.found;
}

} // namespace sigscan

// ============================================================
// GLOBALS
// ============================================================
constexpr const char *kSigCameraUpdate =
    "? ? ? D1 ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? A9 ? ? ? F9 ? ? ? A9 ? ? ? A9 ? ? ? 91 "
    "55 D0 3B D5 F3 03 01 AA F4 03 00 AA ? ? ? F9 ? ? ? 91 ? ? ? 91";

constexpr uintptr_t kOutQuat = 0x28;
constexpr uintptr_t kOutPos  = 0x38;
constexpr int kQX = 2, kQY = 0, kQZ = 1, kQW = 3;

ll::mod::NativeMod *gSelf;
Tuning gTuning;
Motion gMotion;

using CameraUpdateFn = void (*)(void *, void *, float);
CameraUpdateFn gOrigUpdate = nullptr;
void *gUpdateTarget = nullptr;
std::atomic<bool> gHookInstalled{false};

using SwapFn = EGLBoolean (*)(EGLDisplay, EGLSurface);
SwapFn gSwap = nullptr;
void *gSwapTarget = nullptr;

std::atomic<bool> gActive{false};
std::atomic<int> gSurfW{0}, gSurfH{0};

// ============================================================
// QUATERNION MATH
// ============================================================
struct Quat { float x, y, z, w; };

inline Quat loadQuat(const float *m) {
    return {m[kQX], m[kQY], m[kQZ], m[kQW]};
}
inline void storeQuat(float *m, const Quat &q) {
    m[kQX] = q.x; m[kQY] = q.y; m[kQZ] = q.z; m[kQW] = q.w;
}
inline Quat qmul(const Quat &a, const Quat &b) {
    return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}
inline Quat qAxis(float ax, float ay, float az, float rad) {
    float s = std::sin(rad * 0.5f);
    return {ax*s, ay*s, az*s, std::cos(rad * 0.5f)};
}
inline void qRotate(const Quat &q, float vx, float vy, float vz, float *out) {
    float tx = 2.f*(q.y*vz - q.z*vy);
    float ty = 2.f*(q.z*vx - q.x*vz);
    float tz = 2.f*(q.x*vy - q.y*vx);
    out[0] = vx + q.w*tx + (q.y*tz - q.z*ty);
    out[1] = vy + q.w*ty + (q.z*tx - q.x*tz);
    out[2] = vz + q.w*tz + (q.x*ty - q.y*tx);
}

// ============================================================
// TOUCH INPUT
// ============================================================
struct Pointer {
    bool down = false, left = false;
    float startX = 0, startY = 0, lastX = 0, lastY = 0;
};
constexpr int kMaxPointers = 10;
std::mutex gTouchMutex;
Pointer gPointers[kMaxPointers];
float gTurnPx = 0.f;

bool onTouch(const pl::input::TouchEvent &e) {
    if (!gActive.load()) return false;
    int w = gSurfW.load(), h = gSurfH.load();
    if (w <= 0 || h <= 0 || e.pointerId < 0 || e.pointerId >= kMaxPointers) return false;
    int action = e.action & 0xFF;
    std::lock_guard<std::mutex> lock(gTouchMutex);
    Pointer &p = gPointers[e.pointerId];
    switch (action) {
    case 0: case 5: p = {true, e.x < w*0.5f, e.x, e.y, e.x, e.y}; break;
    case 2: if (p.down) { if (!p.left) gTurnPx += e.x - p.lastX; p.lastX = e.x; p.lastY = e.y; } break;
    case 1: case 3: case 6: p.down = false; break;
    }
    return false;
}

Input readInput(double dt, int w, int h) {
    Input in;
    std::lock_guard<std::mutex> lock(gTouchMutex);
    double radius = std::max(1.0, gTuning.joystickRadius * h);
    for (const Pointer &p : gPointers) {
        if (!p.down || !p.left) continue;
        in.moving = true;
        double dx = p.lastX - p.startX, dy = p.lastY - p.startY;
        in.strafe = std::clamp(dx / radius, -1.0, 1.0);
        in.forward = std::fabs(dy) < radius * 0.15 ? 1.0 : std::clamp(-dy / radius, -1.0, 1.0);
        break;
    }
    in.turnSpeed = dt > 1e-4 ? (gTurnPx / std::max(1, w)) / dt : 0.0;
    gTurnPx = 0.f;
    return in;
}

double nowSeconds() {
    timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// ============================================================
// CAMERA HOOK
// ============================================================
void hkCameraUpdate(void *out, void *state, float alpha) {
    gOrigUpdate(out, state, alpha);

    if (!gActive.load() || !isEnabled() || !out) return;

    static double last = 0.0;
    double now = nowSeconds();
    double dt = last > 0.0 ? now - last : 0.016;
    last = now;

    int w = gSurfW.load(), h = gSurfH.load();
    auto in = readInput(dt, w > 0 ? w : 16, h > 0 ? h : 9);
    auto o = gMotion.update(dt, in, (w > 0 && h > 0) ? double(w)/h : 16.0/9.0);

    float *qm = reinterpret_cast<float *>(static_cast<char *>(out) + kOutQuat);
    float *pm = reinterpret_cast<float *>(static_cast<char *>(out) + kOutPos);

    // 3-axis camera-space rotation: Yaw -> Pitch -> Roll
    Quat dq = qmul(qAxis(0,1,0, static_cast<float>(o.yawRad)),
                   qAxis(1,0,0, static_cast<float>(o.pitchRad)));
    dq = qmul(dq, qAxis(0,0,1, static_cast<float>(o.rollRad)));

    storeQuat(qm, qmul(loadQuat(qm), dq));

    // Position shift rotated into world space
    float wp[3];
    qRotate(loadQuat(qm),
            static_cast<float>(o.offX),
            static_cast<float>(o.offY),
            static_cast<float>(o.offZ), wp);
    pm[0] += wp[0]; pm[1] += wp[1]; pm[2] += wp[2];
}

// ============================================================
// LAZY HOOK INSTALL
// ============================================================
void ensureHooked() {
    if (gHookInstalled.load()) return;
    static sigscan::Pattern pat;
    static bool parsed = false;
    if (!parsed) { parsed = true; sigscan::parse(pat, kSigCameraUpdate); }
    void *target = sigscan::find(pat);
    if (!target) return;
    if (pl::memory::hook(target, reinterpret_cast<void *>(&hkCameraUpdate),
                         reinterpret_cast<void **>(&gOrigUpdate)) != 0) return;
    gUpdateTarget = target;
    gHookInstalled.store(true);
    gSelf->getLogger().info("Bodycam: hooked camera update at {}", target);
}

EGLBoolean swapHook(EGLDisplay dpy, EGLSurface surf) {
    ensureHooked();
    EGLint w = 0, h = 0;
    eglQuerySurface(dpy, surf, EGL_WIDTH, &w);
    eglQuerySurface(dpy, surf, EGL_HEIGHT, &h);
    if (w > 0 && h > 0) { gSurfW.store(w); gSurfH.store(h); }
    return gSwap(dpy, surf);
}

// ============================================================
// MOD MENU REGISTRATION
// ============================================================
namespace {

pl::modmenu::ModuleInfo makeMenuInfo() {
    pl::modmenu::ModuleInfo info{};
    info.name        = "Bodycam Motion";
    info.description = "Physics-based camera: step impacts, turn lag, idle sway";
    info.version     = "2.0";
    info.author      = "Satoshi442";

    pl::modmenu::ToggleOption toggle{};
    toggle.label    = "Enabled";
    toggle.initial  = isEnabled();
    toggle.onChange = [](bool on) { setEnabled(on, true); };
    info.options.push_back(toggle);
    return info;
}

} // anon

__attribute__((constructor)) static void initMenu() {
    ensureConfig();
    gEnabled.store(gCfgLoaded ? gCfg.value().enabled : Tuning{}.enabled,
                   std::memory_order_relaxed);
    pl::modmenu::registerModule(makeMenuInfo());
}

} // namespace bodycam

// ============================================================
// LEVILAMINA MOD CLASS
// ============================================================
class BodycamMotion {
public:
    static BodycamMotion &instance() { static BodycamMotion inst; return inst; }
    BodycamMotion() : mSelf(*ll::mod::NativeMod::current()) { bodycam::gSelf = &mSelf; }

    bool load() {
        bodycam::ensureConfig();
        if (!bodycam::gCfgLoaded)
            mSelf.getLogger().warn("Config load failed, using defaults");
        bodycam::gTuning = bodycam::gCfg.value();
        bodycam::gMotion.setTuning(bodycam::gTuning);
        bodycam::gEnabled.store(bodycam::gTuning.enabled);
        return true;
    }

    bool enable() {
        using namespace bodycam;
        gSwapTarget = dlsym(RTLD_DEFAULT, "eglSwapBuffers");
        if (!gSwapTarget ||
            pl::memory::hook(gSwapTarget, reinterpret_cast<void *>(&swapHook),
                             reinterpret_cast<void **>(&gSwap)) != 0) {
            mSelf.getLogger().error("Failed to hook eglSwapBuffers");
            return false;
        }
        if (!mInputRegistered) {
            pl::input::registerTouchCallback(&onTouch);
            mInputRegistered = true;
        }
        ensureHooked();
        gActive.store(true);
        mSelf.getLogger().info("Bodycam Motion v2.0 active (native camera-space)");
        return true;
    }

    bool disable() {
        using namespace bodycam;
        gActive.store(false);
        if (gSwapTarget && gSwap)
            pl::memory::unhook(gSwapTarget, reinterpret_cast<void *>(&swapHook));
        if (gUpdateTarget)
            pl::memory::unhook(gUpdateTarget, reinterpret_cast<void *>(&hkCameraUpdate));
        return true;
    }

private:
    ll::mod::NativeMod &mSelf;
    bool mInputRegistered = false;
};

PL_REGISTER_MOD(BodycamMotion, BodycamMotion::instance())
