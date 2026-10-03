// BodycamMotion - bodycam-style camera movement, applied in CAMERA space.
//
// Instead of redrawing the finished frame (which shook the whole screen, HUD
// and menus), we post-multiply the game's view-projection matrix by a small
// camera-space transform R each frame:  M' = M * R  (column-major), which is
// exactly equivalent to moving the camera. The world rolls/bobs/sways with
// real parallax; 2D/ortho draws (HUD, loading screens, menus) are untouched.
//
// The view-proj uniform is found without any game offsets: per frame we watch
// glUniformMatrix4fv uploads; a (program, location) pair that receives >=2
// identical perspective matrices in one frame is the shared view-proj uniform.
// From the next frame on, every upload to that location is patched.
//
// Movement input comes from touch events (Levi Input API): a touch on the left
// half = moving, drag on the right half = turning.
//
// Motion model ideas credit: CameraOverhaul by LENDS DZIN (used with permission).

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <dlfcn.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <optional>
#include <unordered_map>

#include <pl/Config.hpp>
#include <pl/Input.hpp>
#include <pl/Mod.hpp>
#include <pl/memory/Hook.hpp>

#include "Motion.hpp"

namespace {

using bodycam::Tuning;

// Camera-space gains (how strongly the motion model drives the camera).
constexpr float kRollGain   = 1.0f;   // out.rollRad -> camera roll
constexpr float kBobPitch   = 0.35f;  // out.offY    -> camera pitch (rad scale)
constexpr float kShiftGain  = 0.25f;  // out.offX/Y  -> camera translation (units)
constexpr int   kMinRepeats = 2;      // identical uploads/frame to trust a uniform
constexpr int   kLearnFrames = 120;   // frames before "no shared VP" is reported

// ---------------------------------------------------------------- state ----
ll::mod::NativeMod *gSelf;
Tuning gTuning;
bodycam::Motion gMotion;

using SwapFn = EGLBoolean (*)(EGLDisplay, EGLSurface);
SwapFn gSwap;
void *gSwapTarget;

using UM4Fn = void (*)(GLint, GLsizei, GLboolean, const GLfloat *);
UM4Fn gUM4;
void *gUM4Target;

std::atomic<bool> gActive{false};
std::atomic<int> gSurfW{0}, gSurfH{0};

// Camera-space transform for the frame being rendered (column-major 4x4).
float gR[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// ------------------------------------------------------------- mat4 math ----
void mat4Mul(const float *a, const float *b, float *out) {  // out = a * b
    float t[16];
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            t[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] +
                           a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    std::memcpy(out, t, sizeof(t));
}

void buildCameraR(const bodycam::Output &o) {
    // R = Rz(roll) * Rx(pitch) * T(shift): roll outermost, translation innermost.
    const float cr = std::cos(o.rollRad * kRollGain), sr = std::sin(o.rollRad * kRollGain);
    const float cp = std::cos(o.offY * kBobPitch), sp = std::sin(o.offY * kBobPitch);
    const float tx = o.offX * kShiftGain, ty = o.offY * kShiftGain * 0.5f;

    float rz[16] = {cr, sr, 0, 0, -sr, cr, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float rx[16] = {1, 0, 0, 0, 0, cp, sp, 0, 0, -sp, cp, 0, 0, 0, 0, 1};
    float tr[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, tx, ty, 0, 1};

    float tmp[16];
    mat4Mul(rx, tr, tmp);
    mat4Mul(rz, tmp, gR);
}

// ------------------------------------------------------- VP uniform learn ----
struct UniformStats {
    int count = 0;
    bool allEqual = true;
    bool marked = false;
    float first[16] = {};
};
struct Key {
    GLuint prog;
    GLint loc;
    bool operator==(const Key &o) const { return prog == o.prog && loc == o.loc; }
};
struct KeyHash {
    size_t operator()(const Key &k) const {
        return std::hash<uint64_t>{}((uint64_t(k.prog) << 32) | uint32_t(k.loc));
    }
};
std::unordered_map<Key, UniformStats, KeyHash> gStats;
int gMarkedCount = 0;
int gFrames = 0;
bool gLearnReported = false;

inline bool isPerspectiveish(const float *m) {
    // Ortho/HUD matrices have w-row == (0,0,0,1); perspective combos don't.
    return !(std::fabs(m[12]) < 1e-6f && std::fabs(m[13]) < 1e-6f &&
             std::fabs(m[14]) < 1e-6f && std::fabs(m[15] - 1.0f) < 1e-6f);
}

inline bool matEq(const float *a, const float *b) {
    for (int i = 0; i < 16; ++i)
        if (std::fabs(a[i] - b[i]) > 1e-3f) return false;
    return true;
}

// Finalize learning for the frame that just finished; prepare per-frame stats.
void finalizeFrameLearning() {
    gMarkedCount = 0;
    for (auto &kv : gStats) {
        UniformStats &s = kv.second;
        if (!s.marked && s.count >= kMinRepeats && s.allEqual) {
            s.marked = true;
            gSelf->getLogger().info("View-proj uniform learned (program {}, location {})",
                                    kv.first.prog, kv.first.loc);
        } else if (s.marked && (s.count == 0 || !s.allEqual)) {
            s.marked = false;  // location reused for something else this frame
            gSelf->getLogger().warn("View-proj uniform unmarked (program {}, location {})",
                                    kv.first.prog, kv.first.loc);
        }
        if (s.marked) ++gMarkedCount;
        s.count = 0;
        s.allEqual = true;
    }
    if (gMarkedCount == 0 && ++gFrames > kLearnFrames && !gLearnReported) {
        gLearnReported = true;
        gSelf->getLogger().warn(
            "No shared view-proj uniform detected: this renderer uploads combined "
            "per-object matrices, camera-space mode cannot engage.");
    }
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

// ----------------------------------------------------------------- hooks ----
double nowSeconds() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

void GL_APIENTRY uniformMatrix4fvHook(GLint location, GLsizei count, GLboolean transpose,
                                      const GLfloat *value) {
    if (gActive.load() && gTuning.enabled && value && count == 1 && !transpose &&
        gMarkedCount >= 0 && isPerspectiveish(value)) {
        GLint prog = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
        UniformStats &s = gStats[Key{GLuint(prog), location}];
        if (s.marked) {
            float out[16];
            mat4Mul(value, gR, out);  // M' = M * R  -> camera-space transform
            gUM4(location, count, transpose, out);
            return;
        }
        // Learning: is this location receiving the same perspective matrix repeatedly?
        if (s.count == 0) {
            std::memcpy(s.first, value, sizeof(s.first));
        } else if (s.allEqual) {
            s.allEqual = matEq(s.first, value);
        }
        ++s.count;
    }
    gUM4(location, count, transpose, value);
}

EGLBoolean swapHook(EGLDisplay dpy, EGLSurface surf) {
    static double last = 0.0;
    if (gActive.load() && gTuning.enabled) {
        EGLint w = 0, h = 0;
        eglQuerySurface(dpy, surf, EGL_WIDTH, &w);
        eglQuerySurface(dpy, surf, EGL_HEIGHT, &h);
        if (w > 0 && h > 0) {
            gSurfW.store(w);
            gSurfH.store(h);
            const double now = nowSeconds();
            const double dt = last > 0.0 ? now - last : 0.016;
            last = now;
            const auto in = readInput(dt, w, h);
            const auto out = gMotion.update(dt, in, static_cast<double>(w) / h);
            buildCameraR(out);       // R used by all patched uploads next frame
            finalizeFrameLearning(); // close stats for the frame just rendered
        }
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
        gUM4Target = dlsym(RTLD_DEFAULT, "glUniformMatrix4fv");
        if (!gUM4Target ||
            pl::memory::hook(gUM4Target, reinterpret_cast<void *>(&uniformMatrix4fvHook),
                             reinterpret_cast<void **>(&gUM4)) != 0) {
            mSelf.getLogger().error("Failed to hook glUniformMatrix4fv");
            return false;
        }
        if (!mInputRegistered) {  // the Input API has no unregister
            pl::input::registerTouchCallback(&onTouch);
            mInputRegistered = true;
        }
        gActive.store(true);
        mSelf.getLogger().info("Bodycam motion active (camera-space)");
        return true;
    }

    bool disable() {
        gActive.store(false);
        if (gSwapTarget) pl::memory::unhook(gSwapTarget, reinterpret_cast<void *>(&swapHook));
        if (gUM4Target) pl::memory::unhook(gUM4Target, reinterpret_cast<void *>(&uniformMatrix4fvHook));
        return true;
    }

private:
    ll::mod::NativeMod &mSelf;
    std::optional<pl::config::ConfigFile<Tuning>> mConfig;
    bool mInputRegistered = false;
};

PL_REGISTER_MOD(BodycamMotion, BodycamMotion::instance())
