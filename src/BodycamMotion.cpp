// BodycamMotion - native camera-space bodycam movement.
// Hooks the game's render-camera update function (resolved by the same
// signature the reference CameraOverhaul mod uses) and injects roll /
// pitch / sway into the camera transform AFTER the game computes it.
// The world moves with real parallax; HUD, menus and loading screens
// never see it. Zero extra draw calls.

#include <EGL/egl.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <pl/Config.hpp>
#include <pl/Input.hpp>
#include <pl/Mod.hpp>
#include <pl/memory/Hook.hpp>
#include <pl/memory/Memory.hpp>

#include "Motion.hpp"

namespace {

using bodycam::Tuning;

// Verified resolving on this build (Zaphkiel sig scan, match @ 0x114d9e48).
constexpr std::string_view kSigCameraUpdate =
    "? ? ? D1 ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? 6D ? ? ? A9 ? ? ? F9 ? ? ? A9 ? ? ? A9 ? ? ? 91 "
    "55 D0 3B D5 F3 03 01 AA F4 03 00 AA ? ? ? F9 ? ? ? 91 ? ? ? 91";
// Banked for later (match @ 0xaac7dbc, LocalPlayer/gameplay tick):
// "? ? ? FC ? ? ? A9 ... ? ? ? 39"

// Offsets inside the hooked function's output struct, read from its own
// stores:  stp s4,s1,[x20,#0x28] + stp s0,s2,[x20,#0x30] -> quaternion
//          stp s1,s2,[x20,#0x38] + str s0,[x20,#0x40]     -> world position
constexpr uintptr_t kOutQuat = 0x28;
constexpr uintptr_t kOutPos  = 0x38;

// If roll ever spins around the wrong axis, the quat is stored (w,x,y,z):
constexpr bool kQuatIsXYZW = true;

// Motion output -> camera-space gains
constexpr float kRollGain  = 1.0f;    // out.rollRad -> radians of roll
constexpr float kPitchGain = 0.35f;   // out.offY    -> radians of pitch
constexpr float kShiftGain = 0.15f;   // out.offX/Y  -> blocks of sway/bob

ll::mod::NativeMod *gSelf;
Tuning gTuning;
bodycam::Motion gMotion;

using CameraUpdateFn = void (*)(void *, void *, float);
CameraUpdateFn gOrigUpdate = nullptr;
void *gUpdateTarget = nullptr;

using SwapFn = EGLBoolean (*)(EGLDisplay, EGLSurface);
SwapFn gSwap = nullptr;
void *gSwapTarget = nullptr;

std::atomic<bool> gActive{false};
std::atomic<int> gSurfW{0}, gSurfH{0};

// ---------------------------------------------------------------- touch ----
struct Pointer {
    bool down = false, left = false;
    float startX = 0, startY = 0, lastX = 0, lastY = 0;
};
constexpr int kMaxPointers = 10;
std::mutex gTouchMutex;
Pointer gPointers[kMaxPointers];
float gTurnPx = 0.0f;

bool onTouch(const pl::input::TouchEvent &e) {
    if (!gActive.load()) return false;
    const int w = gSurfW.load(), h = gSurfH.load();
    if (w <= 0 || h <= 0 || e.pointerId < 0 || e.pointerId >= kMaxPointers) return false;
    const int action = e.action & 0xFF;
    std::lock_guard<std::mutex> lock(gTouchMutex);
    Pointer &p = gPointers[e.pointerId];
    switch (action) {
    case 0: case 5: p = {true, e.x < w * 0.5f, e.x, e.y, e.x, e.y}; break;
    case 2:
        if (p.down) {
            if (!p.left) gTurnPx += e.x - p.lastX;
            p.lastX = e.x; p.lastY = e.y;
        }
        break;
    case 1: case 3: case 6: p.down = false; break;
    default: break;
    }
    return false;
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

// ------------------------------------------------------------ quat math ----
struct Quat { float x, y, z, w; };

Quat loadQuat(const float *m) {
    return kQuatIsXYZW ? Quat{m[0], m[1], m[2], m[3]} : Quat{m[1], m[2], m[3], m[0]};
}
void storeQuat(float *m, const Quat &q) {
    if (kQuatIsXYZW) { m[0] = q.x; m[1] = q.y; m[2] = q.z; m[3] = q.w; }
    else             { m[0] = q.w; m[1] = q.x; m[2] = q.y; m[3] = q.z; }
}
Quat qmul(const Quat &a, const Quat &b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
Quat qAxisAngle(float ax, float ay, float az, float rad) {
    const float s = std::sin(rad * 0.5f);
    return {ax * s, ay * s, az * s, std::cos(rad * 0.5f)};
}
void qRotate(const Quat &q, float vx, float vy, float vz, float *out) {
    // v' = q v q*
    const float tx = 2.f * (q.y * vz - q.z * vy);
    const float ty = 2.f * (q.z * vx - q.x * vz);
    const float tz = 2.f * (q.x * vy - q.y * vx);
    out[0] = vx + q.w * tx + (q.y * tz - q.z * ty);
    out[1] = vy + q.w * ty + (q.z * tx - q.x * tz);
    out[2] = vz + q.w * tz + (q.x * ty - q.y * tx);
}

// ---------------------------------------------------------------- hook -----
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

    // Camera-local delta rotation: roll about Z, bob pitch about X.
    const Quat dq = qmul(qAxisAngle(0.f, 0.f, 1.f, static_cast<float>(o.rollRad) * kRollGain),
                         qAxisAngle(1.f, 0.f, 0.f, static_cast<float>(o.offY) * kPitchGain));
    const Quat res = qmul(loadQuat(qm), dq);  // post-multiply => camera space
    storeQuat(qm, res);

    // Sway / bob shift, rotated into world space by the final orientation.
    float wp[3];
    qRotate(res, static_cast<float>(o.offX) * kShiftGain,
                 static_cast<float>(o.offY) * kShiftGain * 0.5f, 0.f, wp);
    pm[0] += wp[0]; pm[1] += wp[1]; pm[2] += wp[2];
}

EGLBoolean swapHook(EGLDisplay dpy, EGLSurface surf) {  // size probe only
    EGLint w = 0, h = 0;
    eglQuerySurface(dpy, surf, EGL_WIDTH, &w);
    eglQuerySurface(dpy, surf, EGL_HEIGHT, &h);
    if (w > 0 && h > 0) { gSurfW.store(w); gSurfH.store(h); }
    return gSwap(dpy, surf);
}

}  // namespace

class BodycamMotion {
public:
    static BodycamMotion &instance() { static BodycamMotion inst; return inst; }
    BodycamMotion() : mSelf(*ll::mod::NativeMod::current()) { gSelf = &mSelf; }

    bool load() {
        mConfig.emplace();
        if (!mConfig->load()) mSelf.getLogger().warn("Config load failed, using defaults");
        gTuning = mConfig->value();
        gMotion.setTuning(gTuning);
        return true;
    }

    bool enable() {
        std::vector<std::string> sigs{std::string(kSigCameraUpdate)};
        const auto addrs = pl::memory::resolveSignatures(sigs, "libminecraftpe.so");
        void *target = addrs.empty() ? nullptr : addrs[0];
        if (!target) {
            mSelf.getLogger().error("Camera signature did not resolve - MC version mismatch?");
            return false;
        }
        gUpdateTarget = target;
        if (pl::memory::hook(gUpdateTarget, reinterpret_cast<void *>(&hkCameraUpdate),
                             reinterpret_cast<void **>(&gOrigUpdate)) != 0) {
            mSelf.getLogger().error("Failed to hook camera update fn");
            return false;
        }
        mSelf.getLogger().info("Camera update hooked at {}", target);

        gSwapTarget = dlsym(RTLD_DEFAULT, "eglSwapBuffers");
        if (gSwapTarget)
            pl::memory::hook(gSwapTarget, reinterpret_cast<void *>(&swapHook),
                             reinterpret_cast<void **>(&gSwap));

        if (!mInputRegistered) {
            pl::input::registerTouchCallback(&onTouch);
            mInputRegistered = true;
        }
        gActive.store(true);
        mSelf.getLogger().info("Bodycam motion active (native camera-space)");
        return true;
    }

    bool disable() {
        gActive.store(false);
        if (gUpdateTarget) pl::memory::unhook(gUpdateTarget, reinterpret_cast<void *>(&hkCameraUpdate));
        if (gSwapTarget && gSwap) pl::memory::unhook(gSwapTarget, reinterpret_cast<void *>(&swapHook));
        return true;
    }

private:
    ll::mod::NativeMod &mSelf;
    std::optional<pl::config::ConfigFile<Tuning>> mConfig;
    bool mInputRegistered = false;
};

PL_REGISTER_MOD(BodycamMotion, BodycamMotion::instance())
