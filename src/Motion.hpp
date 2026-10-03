// Pure motion model (no Android / GL / SDK dependencies, so it is testable on
// any host).
//
// Parameter ideas (pitch / roll / idle sway, with intensity + smoothing for
// each) follow CameraOverhaul by LENDS DZIN, used with the author's permission.
// This is our own image-space implementation of them.
//
// Units: screen height = 1.0 for offsets in Tuning. Internally the shader works
// in aspect-corrected coordinates where half the screen height = 1.0, so
// offsets are multiplied by 2 before they leave Motion::update().
#pragma once
#include <algorithm>
#include <cmath>

namespace bodycam {

// Public config (aggregate with `int version` first: required by pl::config).
struct Tuning {
    int version = 1;
    bool enabled = true;

    bool enableBob = true;
    bool enableRoll = true;
    bool enableSway = true;

    // Walking bob
    double bobAmount = 0.012;      // vertical, fraction of screen height
    double bobSideAmount = 0.006;  // horizontal, fraction of screen height
    double stepFrequency = 1.9;    // steps per second while moving
    double forwardPitch = 0.010;   // vertical lean while moving forward (can be negative)

    // Roll (degrees at full input)
    double turningRoll = 3.0;      // from fast turning; negative flips direction
    double strafingRoll = 2.0;     // from sideways movement; negative flips direction

    // Idle sway
    double swayIntensity = 0.35;   // roll degrees
    double swayOffset = 0.004;     // offset, fraction of screen height
    double swayFrequency = 0.35;   // Hz
    double swayFadeInDelay = 0.6;  // seconds standing still before sway starts
    double swayFadeInLength = 1.5;
    double swayFadeOutLength = 0.4;

    // Smoothing (time constants in seconds; larger = lazier camera)
    double moveSmoothing = 0.10;
    double rollSmoothing = 0.14;

    // Touch input mapping
    double joystickRadius = 0.12;  // fraction of screen height for full strafe/back input
    double maxTurnSpeed = 1.2;     // screen widths per second that counts as "full" turning
};

struct Input {
    bool moving = false;     // a movement touch is held
    double forward = 0.0;    // -1..1
    double strafe = 0.0;     // -1..1
    double turnSpeed = 0.0;  // signed screen widths / second
};

struct Output {
    double rollRad = 0.0;
    double offX = 0.0;  // aspect-corrected units (half height = 1)
    double offY = 0.0;
    double zoom = 1.0;
};

// Smallest zoom such that the rotated + offset + zoomed image still covers the
// whole screen (no empty borders). A = aspect (half width), B = 1 (half height).
inline double minZoom(double aspect, double rollRad, double ox, double oy) {
    const double A = aspect, B = 1.0;
    const double c = std::fabs(std::cos(rollRad)), s = std::fabs(std::sin(rollRad));
    const double availX = std::max(A - std::fabs(ox), 0.05);
    const double availY = std::max(B - std::fabs(oy), 0.05);
    const double zx = (A * c + B * s) / availX;
    const double zy = (A * s + B * c) / availY;
    return std::max({1.0, zx, zy});
}

class Motion {
public:
    explicit Motion(const Tuning &t = {}) : t_(t) {}
    void setTuning(const Tuning &t) { t_ = t; }

    Output update(double dt, const Input &in, double aspect) {
        dt = std::clamp(dt, 0.0, 0.1);  // a long hitch must not teleport the camera
        const double kMove = smoothing(dt, t_.moveSmoothing);
        const double kRoll = smoothing(dt, t_.rollSmoothing);

        const double moveMag = in.moving ? 1.0 : 0.0;
        move_ += (moveMag - move_) * kMove;
        fwd_ += (std::clamp(in.forward, -1.0, 1.0) * moveMag - fwd_) * kMove;
        strafe_ += (std::clamp(in.strafe, -1.0, 1.0) * moveMag - strafe_) * kRoll;
        turn_ += (std::clamp(in.turnSpeed / std::max(t_.maxTurnSpeed, 1e-6), -1.0, 1.0) - turn_) * kRoll;

        // Idle tracking: fade sway in after standing still, out when active.
        const bool active = in.moving || std::fabs(in.turnSpeed) > 0.05;
        if (active) idle_ = 0.0; else idle_ += dt;
        double target = 0.0;
        if (!active && idle_ > t_.swayFadeInDelay)
            target = 1.0;
        const double fadeLen = target > swayW_ ? std::max(t_.swayFadeInLength, 1e-3)
                                               : std::max(t_.swayFadeOutLength, 1e-3);
        swayW_ += std::clamp((target - swayW_), -dt / fadeLen, dt / fadeLen);

        phase_ += kTwoPi * t_.stepFrequency * dt * move_;
        phase_ = std::fmod(phase_, kTwoPi * 2.0);
        swayT_ += dt;

        double roll = 0.0, ox = 0.0, oy = 0.0;
        if (t_.enableBob) {
            oy += std::sin(phase_) * t_.bobAmount * move_;
            ox += std::sin(phase_ * 0.5) * t_.bobSideAmount * move_;
            oy += t_.forwardPitch * fwd_;
        }
        if (t_.enableRoll) {
            roll += t_.turningRoll * turn_ + t_.strafingRoll * strafe_;
        }
        if (t_.enableSway) {
            const double w = kTwoPi * t_.swayFrequency * swayT_;
            // two incommensurate sines so it never looks like a clean loop
            roll += t_.swayIntensity * swayW_ * (std::sin(w) + 0.5 * std::sin(w * 1.7 + 1.3)) / 1.5;
            ox += t_.swayOffset * swayW_ * std::sin(w * 0.8 + 0.7);
            oy += t_.swayOffset * swayW_ * std::sin(w * 1.3 + 2.1) * 0.7;
        }

        Output o;
        o.rollRad = roll * kPi / 180.0;
        o.offX = ox * 2.0;  // fraction of height -> half-height units
        o.offY = oy * 2.0;
        o.zoom = minZoom(aspect, o.rollRad, o.offX, o.offY);
        return o;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kTwoPi = 2.0 * kPi;
    static double smoothing(double dt, double tau) {
        return tau <= 1e-4 ? 1.0 : 1.0 - std::exp(-dt / tau);
    }

    Tuning t_;
    double move_ = 0, fwd_ = 0, strafe_ = 0, turn_ = 0;
    double idle_ = 0, swayW_ = 0, swayT_ = 0, phase_ = 0;
};

}  // namespace bodycam
