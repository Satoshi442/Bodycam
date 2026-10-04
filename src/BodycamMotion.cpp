// BodycamMotion - bodycam-style camera movement, applied in CAMERA space.
//
// Hooks the game's render-camera update function (resolved at runtime by byte
// signature - the same pattern the reference CameraOverhaul mod uses, verified
// with Zaphkiel on this MC build at file VA 0x114d9e48):
//
//     void fn(void* out, void* state, float alpha)
//
// The function writes, per frame, into `out`:
//     out+0x28 .. 0x34  : camera orientation quaternion (4 floats)
//     out+0x38 .. 0x44  : camera world position (3 floats)
//     out+0x48          : fov-ish scalar
// We call the original first, then post-multiply the quaternion by a small
// camera-local delta (roll / pitch) and shift the position by a camera-local
// offset (sway / bob). The world moves with real parallax; HUD, menus and
// loading screens are untouched because they never pass through this struct.
//
// Movement input comes from touch events (Levi Input API): a touch on the left
// half = moving, drag on the right half = turning.
//
// Motion model ideas credit: CameraOverhaul by LENDS DZIN (used with permission).

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

#include "Motion.hpp"

namespace {

using bodycam::Tuning;

// --------------------------------------------------------------- config ----
// Verified resolving signature for the render-camera update function.
constexpr const char *kSigCameraUpdate =
    "? ? ? D1 ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? A9 ? ? ? F9 ? ? ? A9 ? ? ? A9 ? ? ? 91 "
    "55 D0 3B D5 F3 03 01 AA F4 03 00 AA ? ? ? F9 ? ? ? 91 ? ? ? 91";
// Reserved for later (LocalPlayer/gameplay tick @ 0xaac7dbc): real velocity input.
// constexpr const char *kSigPlayerTick =
//     "? ? ? FC ? ? ? A9 ? ? ? A9 ? ? ? A9 ? ? ? A9 ? ? ? A9 ? ? ? 91 ? ? ? D1 54 D0 3B D5 "
//     "F3 03 00 AA ? ? ? F9 ? ? ? F8 ? ? ? 39";

// Offsets inside the hooked function's output struct (read from its stores).
constexpr uintptr_t kOutQuat = 0x28;  // 4 floats
constexpr uintptr_t kOutPos  = 0x38;  // 3 floats

// Memory slot order of the quaternion components in `out` (derived from the
// euler->quat store sequence at 0x114da244..0x114da28c: [y, z, x, w]).
// If roll ever tilts around the wrong axis, permute kQX/kQY/kQZ.
constexpr int kQX = 2, kQY = 0, kQZ = 1, kQW = 3;

// Motion output -> camera-space gains.
constexpr float kRollGain  = 1.0f;    // out.rollRad -> radians of camera roll
constexpr float kPitchGain = 0.35f;   // out.offY    -> radians of camera pitch
constexpr float kShiftGain = 0.15f;   // out.offX/Y  -> blocks of camera shift

// ---------------------------------------------------------------- state ----
ll::mod::NativeMod *gSelf;
Tuning gTuning;
bodycam::Motion gMotion;

using CameraUpdateFn = void (*)(void *, void *, float);
CameraUpdateFn gOrigUpdate = nullptr;
void *gUpdateTarget = nullptr;
std::atomic<bool> gHookInstalled{false};

using SwapFn = EGLBoolean (*)(EGLDisplay, EGLSurface);
SwapFn gSwap = nullptr;
void *gSwapTarget = nullptr;

std::atomic<bool> gActive{false};
std::atomic<int> gSurfW{0}, gSurfH{0};

// ------------------------------------------------- signature scanner -------
namespace sigscan {

struct Pattern {
    uint8_t bytes[128];
    uint8_t mask[128];  // 1 = fixed, 0 = wildcard
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
            p.bytes[p.len] = 0;
            p.mask[p.len] = 0;
            ++p.len;
            while (*sig == '?') ++sig;
        } else {
            p.bytes[p.len] = static_cast<uint8_t>(hex(sig[0]) << 4 | hex(sig[1]));
            p.mask[p.len] = 1;
            ++p.len;
            sig += 2;
        }
    }
    return p.len > 0;
}

struct Ctx {
    const Pattern *pat;
    void *found;
};

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

}  // namespace sigscan

// ------------------------------------------------------------ quat math ----
struct Quat { float x, y, z, w; };

inline Quat loadQuat(const float *m) {
    Quat q;
    q.x = m[kQX]; q.y = m[kQY]; q.z = m[kQZ]; q.w = m[kQW];
    return q;
}
inline void storeQuat(float *m, const Quat &q) {
    m[kQX] = q.x; m[kQY] = q.y; m[kQZ] = q.z; m[kQW] = q.w;
}
inline Quat qmul(const Quat &a, const Quat &b) {  // a ⊗ b
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat qAxis(float ax, float ay, float az, float rad) {
    const float s = std::sin(rad * 0.5f);
    return {ax * s, ay * s, az * s, std::cos(rad * 0.5f)};
}
inline void qRotate(const Quat &q, float vx, float vy, float vz, float *out) {
    // v' = q v q*
    const float tx = 2.f * (q.y * vz - q.z * vy);
    const float ty = 2.f * (q.z * vx - q.x * vz);
    const float tz = 2.f * (q.x * vy - q.y * vx);
    out[0] = vx + q.w * tx + (q.y * tz - q.z * ty);
    out[1] = vy + q.w * ty + (q.z * tx - q.x * tz);
    out[2] = vz + q.w * tz + (q.x * ty - q.y * tx);
}

// ---------------------------------------------------------------- touch ----
struct Pointer {
    bool down = false;
    bool left = false;  // started on the left half
    float startX = 0, startY = 0, lastX = 0, lastY = 0;
};
constexpr int kMaxPointers = 10;

std::mutex gTouchMutex;
Pointer gPointers[kMaxPointers];
float gTurnPx = 0.0f;  // right-side horizontal drag accumulated since last frame

bool onTouch(const pl::input::TouchEvent &e) {
    if (!gActive.load()) return false;
    const int w = gSurfW.load(), h = gSurfH.load();
    if (w <= 0 || h <= 0 || e.pointerId < 0 || e.pointerId >= kMaxPointers) return false;

    const int action = e.action & 0xFF;  // strip pointer-index bits if present
    std::lock_guard<std::mutex> lock(gTouchMutex);
    Pointer &p = gPointers[e.pointerId];
    switch (action) {
    case 0:  // ACTION_DOWN
    case 5:  // ACTION_POINTER_DOWN
        p = {true, e.x < w * 0.5f, e.x, e.y, e.x, e.y};
        break;
    case 2:  // ACTION_MOVE
        if (p.down) {
            if (!p.left) gTurnPx += e.x - p.lastX;
            p.lastX = e.x;
            p.lastY = e.y;
        }
        break;
    case 1:  // ACTION_UP
    case 3:  // ACTION_CANCEL
    case 6:  // ACTION_POINTER_UP
        p.down = false;
        break;
    default:
        break;
    }
    return false;  // never consume: the game still gets every touch
}

bodycam::Input readInput(double dt, int w, int h) {
    bodycam::Input in;
    std::lock_guard<std::mutex> lock(gTouchMutex);
    const double radius = std::max(1.0, gTuning.joystickRadius * h);
    for (const Pointer &p : gPointers) {
        if (!p.down || !p.left) continue;
        in.moving = true;
        const double dx = p.lastX - p.startX, dy = p.lastY - p.startY;
        in.strafe = std::clamp(dx / radius, -1.0, 1.0);
        // Pressing without dragging counts as walking forward.
        in.forward = std::fabs(dy) < radius * 0.15 ? 1.0 : std::clamp(-dy / radius, -1.0, 1.0);
        break;
    }
    in.turnSpeed = dt > 1e-4 ? (gTurnPx / std::max(1, w)) / dt : 0.0;
    gTurnPx = 0.0f;
    return in;
}

double nowSeconds() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// ---------------------------------------------------------------- hooks ----
void hkCameraUpdate(void *out, void *state, float alpha) {
    gOrigUpdate(out, state, alpha);
    if (!gActive.load() || !gTuning.enabled || !out) return;

    static double last = 0.0;
    const double now = nowSeconds();
    const double dt = last > 0.0 ? now - last : 0.016;
    last = now;

    const int w = gSurfW.load(), h = gSurfH.load();
    const auto in = readInput(dt, w > 0 ? w : 16, h > 0 ? h : 9);
    const auto o = gMotion.update(dt, in, (w > 0 && h > 0) ? double(w) / h : 16.0 / 9.0);

    float *qm = reinterpret_cast<float *>(static_cast<char *>(out) + kOutQuat);
    float *pm = reinterpret_cast<float *>(static_cast<char *>(out) + kOutPos);

    // Camera-local delta rotation: roll about the view axis (Z), bob pitch about X.
    const Quat dq = qmul(qAxis(0.f, 0.f, 1.f, static_cast<float>(o.rollRad) * kRollGain),
                         qAxis(1.f, 0.f, 0.f, static_cast<float>(o.offY) * kPitchGain));
    const Quat res = qmul(loadQuat(qm), dq);  // post-multiply => camera space
    storeQuat(qm, res);

    // Sway / bob shift, rotated into world space by the final orientation.
    float wp[3];
    qRotate(res, static_cast<float>(o.offX) * kShiftGain,
                 static_cast<float>(o.offY) * kShiftGain * 0.5f, 0.f, wp);
    pm[0] += wp[0];
    pm[1] += wp[1];
    pm[2] += wp[2];
}

void ensureHooked() {
    if (gHookInstalled.load()) return;
    static sigscan::Pattern pat;
    static bool parsed = false;
    if (!parsed) {
        parsed = true;
        sigscan::parse(pat, kSigCameraUpdate);
    }
    void *target = sigscan::find(pat);
    if (!target) return;  // libminecraftpe.so not mapped yet - retry next frame
    if (pl::memory::hook(target, reinterpret_cast<void *>(&hkCameraUpdate),
                         reinterpret_cast<void **>(&gOrigUpdate)) != 0) {
        gSelf->getLogger().error("Bodycam: failed to hook camera update fn");
        return;
    }
    gUpdateTarget = target;
    gHookInstalled.store(true);
    gSelf->getLogger().info("Bodycam: camera update hooked at {}", target);
}

EGLBoolean swapHook(EGLDisplay dpy, EGLSurface surf) {  // size probe + lazy hook
    ensureHooked();
    EGLint w = 0, h = 0;
    eglQuerySurface(dpy, surf, EGL_WIDTH, &w);
    eglQuerySurface(dpy, surf, EGL_HEIGHT, &h);
    if (w > 0 && h > 0) {
        gSurfW.store(w);
        gSurfH.store(h);
    }
    return gSwap(dpy, surf);
}

}  // namespace

class BodycamMotion {
public:
    static BodycamMotion &instance() {
        static BodycamMotion inst;
        return inst;
    }

    BodycamMotion() : mSelf(*ll::mod::NativeMod::current()) { gSelf = &mSelf; }

    bool load() {
        mConfig.emplace();
        if (!mConfig->load()) mSelf.getLogger().warn("Config load failed, using defaults");
        gTuning = mConfig->value();
        gMotion.setTuning(gTuning);
        return true;
    }

    bool enable() {
        gSwapTarget = dlsym(RTLD_DEFAULT, "eglSwapBuffers");
        if (!gSwapTarget ||
            pl::memory::hook(gSwapTarget, reinterpret_cast<void *>(&swapHook),
                             reinterpret_cast<void **>(&gSwap)) != 0) {
            mSelf.getLogger().error("Failed to hook eglSwapBuffers");
            return false;
        }
        if (!mInputRegistered) {  // the Input API has no unregister
            pl::input::registerTouchCallback(&onTouch);
            mInputRegistered = true;
        }
        ensureHooked();  // usually succeeds immediately; swapHook retries otherwise
        gActive.store(true);
        mSelf.getLogger().info("Bodycam motion active (native camera-space)");
        return true;
    }

    bool disable() {
        gActive.store(false);
        if (gSwapTarget && gSwap) pl::memory::unhook(gSwapTarget, reinterpret_cast<void *>(&swapHook));
        if (gUpdateTarget) pl::memory::unhook(gUpdateTarget, reinterpret_cast<void *>(&hkCameraUpdate));
        return true;
    }

private:
    ll::mod::NativeMod &mSelf;
    std::optional<pl::config::ConfigFile<Tuning>> mConfig;
    bool mInputRegistered = false;
};

PL_REGISTER_MOD(BodycamMotion, BodycamMotion::instance())
