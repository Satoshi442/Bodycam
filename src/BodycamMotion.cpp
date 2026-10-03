// BodycamMotion - bodycam-style camera movement, applied in image space.
//
// Every frame, just before the game swaps buffers (eglSwapBuffers hook), the
// finished frame is copied to a texture and drawn back through a fullscreen
// pass that rotates / shifts / zooms it. The game's own camera is untouched,
// so this needs no game-function offsets and survives game updates.
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
#include <mutex>
#include <optional>

#include <pl/Config.hpp>
#include <pl/Input.hpp>
#include <pl/Mod.hpp>
#include <pl/memory/Hook.hpp>

#include "Motion.hpp"

namespace {

using bodycam::Tuning;

// ---------------------------------------------------------------- state ----
ll::mod::NativeMod *gSelf;
Tuning gTuning;
bodycam::Motion gMotion;

using SwapFn = EGLBoolean (*)(EGLDisplay, EGLSurface);
SwapFn gSwap;
void *gSwapTarget;
std::atomic<bool> gActive{false};
std::atomic<int> gSurfW{0}, gSurfH{0};

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

// ------------------------------------------------------------------- GL ----
const char *kVert = R"(#version 300 es
out vec2 vQ;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    vQ = p * 2.0 - 1.0;
    gl_Position = vec4(vQ, 0.0, 1.0);
})";

const char *kFrag = R"(#version 300 es
precision highp float;
in vec2 vQ;
uniform sampler2D uTex;
uniform float uAspect;
uniform float uCos;
uniform float uSin;
uniform vec2 uOff;
uniform float uZoom;
out vec4 oColor;
void main() {
    vec2 q = vec2(vQ.x * uAspect, vQ.y);
    vec2 s = vec2(uCos * q.x - uSin * q.y, uSin * q.x + uCos * q.y) / uZoom + uOff;
    vec2 uv = vec2(s.x / uAspect, s.y) * 0.5 + 0.5;
    oColor = vec4(texture(uTex, uv).rgb, 1.0);
})";

struct GlRes {
    EGLContext ctx = EGL_NO_CONTEXT;
    GLuint prog = 0, vao = 0, tex = 0;
    GLint uAspect = -1, uCos = -1, uSin = -1, uOff = -1, uZoom = -1, uTex = -1;
    int texW = 0, texH = 0;
};
GlRes gRes;
bool gGlFailed = false;

GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512] = {};
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        gSelf->getLogger().error("shader compile failed: {}", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

bool createResources() {
    GLuint vs = compile(GL_VERTEX_SHADER, kVert);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFrag);
    if (!vs || !fs) return false;
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        gSelf->getLogger().error("program link failed");
        glDeleteProgram(prog);
        return false;
    }
    gRes.prog = prog;
    gRes.uAspect = glGetUniformLocation(prog, "uAspect");
    gRes.uCos = glGetUniformLocation(prog, "uCos");
    gRes.uSin = glGetUniformLocation(prog, "uSin");
    gRes.uOff = glGetUniformLocation(prog, "uOff");
    gRes.uZoom = glGetUniformLocation(prog, "uZoom");
    gRes.uTex = glGetUniformLocation(prog, "uTex");
    glGenVertexArrays(1, &gRes.vao);
    glGenTextures(1, &gRes.tex);
    return true;
}

// Saves the GL state we touch and restores it on scope exit.
struct StateGuard {
    GLint prog, vao, activeTex, tex0, sampler0, drawFbo, readFbo, viewport[4], colorMask[4];
    GLboolean scissor, depth, blend, cull, stencil, dither, depthMask;
    StateGuard() {
        glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
        glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
        glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFbo);
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFbo);
        glGetIntegerv(GL_VIEWPORT, viewport);
        glGetBooleanv(GL_COLOR_WRITEMASK, reinterpret_cast<GLboolean *>(colorMask));
        glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
        scissor = glIsEnabled(GL_SCISSOR_TEST);
        depth = glIsEnabled(GL_DEPTH_TEST);
        blend = glIsEnabled(GL_BLEND);
        cull = glIsEnabled(GL_CULL_FACE);
        stencil = glIsEnabled(GL_STENCIL_TEST);
        dither = glIsEnabled(GL_DITHER);
        glActiveTexture(GL_TEXTURE0);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &tex0);
        glGetIntegerv(GL_SAMPLER_BINDING, &sampler0);
    }
    ~StateGuard() {
        glBindSampler(0, static_cast<GLuint>(sampler0));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(tex0));
        glActiveTexture(static_cast<GLenum>(activeTex));
        glBindVertexArray(static_cast<GLuint>(vao));
        glUseProgram(static_cast<GLuint>(prog));
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(drawFbo));
        glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(readFbo));
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        const GLboolean *m = reinterpret_cast<const GLboolean *>(colorMask);
        glColorMask(m[0], m[1], m[2], m[3]);
        glDepthMask(depthMask);
        auto set = [](GLenum cap, GLboolean on) { on ? glEnable(cap) : glDisable(cap); };
        set(GL_SCISSOR_TEST, scissor);
        set(GL_DEPTH_TEST, depth);
        set(GL_BLEND, blend);
        set(GL_CULL_FACE, cull);
        set(GL_STENCIL_TEST, stencil);
        set(GL_DITHER, dither);
    }
};

void drawPass(int w, int h, const bodycam::Output &o) {
    const EGLContext ctx = eglGetCurrentContext();
    if (ctx == EGL_NO_CONTEXT) return;
    if (ctx != gRes.ctx) {  // first frame, or the game recreated its context
        gRes = {};
        gRes.ctx = ctx;
        if (!createResources()) {
            gGlFailed = true;
            return;
        }
        gSelf->getLogger().info("Motion pass ready ({}x{})", w, h);
    }

    StateGuard guard;
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindSampler(0, 0);
    glBindTexture(GL_TEXTURE_2D, gRes.tex);
    if (gRes.texW != w || gRes.texH != h) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        gRes.texW = w;
        gRes.texH = h;
    }
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, w, h);  // frame -> texture

    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_DITHER);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glViewport(0, 0, w, h);

    glUseProgram(gRes.prog);
    glUniform1i(gRes.uTex, 0);
    glUniform1f(gRes.uAspect, static_cast<float>(w) / static_cast<float>(h));
    glUniform1f(gRes.uCos, static_cast<float>(std::cos(o.rollRad)));
    glUniform1f(gRes.uSin, static_cast<float>(std::sin(o.rollRad)));
    glUniform2f(gRes.uOff, static_cast<float>(o.offX), static_cast<float>(o.offY));
    glUniform1f(gRes.uZoom, static_cast<float>(o.zoom));
    glBindVertexArray(gRes.vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

// ----------------------------------------------------------------- hook ----
double nowSeconds() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

EGLBoolean swapHook(EGLDisplay dpy, EGLSurface surf) {
    static double last = 0.0;
    if (gActive.load() && gTuning.enabled && !gGlFailed) {
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
            drawPass(w, h, out);
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
        if (!mInputRegistered) {  // the Input API has no unregister
            pl::input::registerTouchCallback(&onTouch);
            mInputRegistered = true;
        }
        gActive.store(true);
        mSelf.getLogger().info("Bodycam motion active");
        return true;
    }

    bool disable() {
        gActive.store(false);
        if (gSwapTarget) pl::memory::unhook(gSwapTarget, reinterpret_cast<void *>(&swapHook));
        return true;
    }

private:
    ll::mod::NativeMod &mSelf;
    std::optional<pl::config::ConfigFile<Tuning>> mConfig;
    bool mInputRegistered = false;
};

PL_REGISTER_MOD(BodycamMotion, BodycamMotion::instance())
