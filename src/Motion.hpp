#pragma once
#include <algorithm>
#include <cmath>
#include "Spring.hpp"

namespace bodycam {

struct Tuning {
    int version = 3;
    bool enabled = true;

    // Reference mod parameters (from strings.txt)
    double verticalPitchIntensity = 8.0;   // Degrees of pitch nod per step
    double turningRollIntensity = 6.0;     // Degrees of roll per turn unit
    double strafingRollIntensity = 4.0;    // Degrees of roll when strafing
    double swayIntensity = 3.0;            // Idle sway amplitude (degrees)
    double swayFrequency = 0.8;            // Hz
    double overallSmoothness = 0.12;       // Spring damping factor

    // Step detection
    double stepDistance = 1.8;             // Virtual blocks between steps
    double stepImpactY = -0.08;            // Vertical drop shock
    double stepImpactX = 0.04;             // Lateral shock

    // Movement lean
    double movePitchLean = -0.06;          // Pitch down when moving forward
    double strafeXLean = 0.03;             // Lateral shift when strafing
    double turnYawLag = 0.08;              // Camera trails behind crosshair

    // Touch input
    double joystickRadius = 0.12;
    double maxTurnSpeed = 1.2;
};

struct Input {
    bool moving = false;
    double forward = 0.0;
    double strafe = 0.0;
    double turnSpeed = 0.0;
};

struct Output {
    double pitchRad = 0.0;
    double yawRad = 0.0;
    double rollRad = 0.0;
    double offX = 0.0;
    double offY = 0.0;
    double offZ = 0.0;
};

class Motion {
public:
    explicit Motion(const Tuning &t = {}) : t_(t) {}
    void setTuning(const Tuning &t) { t_ = t; reset(); }
    void reset() {
        sPitch_.reset(); sYaw_.reset(); sRoll_.reset();
        sX_.reset(); sY_.reset(); sZ_.reset();
        dist_ = 0.f; steps_ = 0; idleT_ = 0.f; idleBlend_ = 0.f;
    }

    Output update(double dt, const Input &in, double aspect) {
        dt = std::clamp(dt, 0.0, 0.1);
        float fdt = static_cast<float>(dt);
        float smooth = static_cast<float>(t_.overallSmoothness);

        // --- Step Detection ---
        float speed = std::sqrt(in.forward * in.forward + in.strafe * in.strafe);
        if (in.moving) dist_ += speed * fdt * 5.0f;

        if (dist_ > t_.stepDistance) {
            dist_ -= t_.stepDistance;
            steps_++;
            float side = (steps_ % 2 == 0) ? 1.f : -1.f;
            float deg2rad = 0.01745329f;

            sPitch_.impulse(static_cast<float>(t_.verticalPitchIntensity) * deg2rad * 18.f);
            sRoll_.impulse(static_cast<float>(t_.strafingRollIntensity) * side * deg2rad * 12.f);
            sY_.impulse(static_cast<float>(t_.stepImpactY) * 25.f);
            sX_.impulse(static_cast<float>(t_.stepImpactX) * side * 25.f);
        }

        // --- Targets ---
        sPitch_.target = in.moving ? static_cast<float>(in.forward) * static_cast<float>(t_.movePitchLean) : 0.f;
        sRoll_.target  = in.moving ? static_cast<float>(in.strafe) * static_cast<float>(t_.strafingRollIntensity) * 0.01745329f : 0.f;
        sX_.target     = in.moving ? static_cast<float>(in.strafe) * static_cast<float>(t_.strafeXLean) : 0.f;
        sYaw_.target   = -static_cast<float>(in.turnSpeed) * 1.5f * static_cast<float>(t_.turnYawLag);

        // Turning roll (continuous, not just step-based)
        float turnRollTarget = -static_cast<float>(in.turnSpeed) * static_cast<float>(t_.turningRollIntensity) * 0.01745329f;
        sRoll_.target += turnRollTarget;

        // --- Spring Parameters ---
        float stiff = 80.f;
        float damp = 12.f / std::max(smooth, 0.01f);
        sPitch_.stiffness = stiff; sPitch_.damping = damp;
        sYaw_.stiffness   = stiff; sYaw_.damping   = damp;
        sRoll_.stiffness  = stiff; sRoll_.damping  = damp;
        sX_.stiffness     = 60.f;  sX_.damping     = damp;
        sY_.stiffness     = 60.f;  sY_.damping     = damp;
        sZ_.stiffness     = 60.f;  sZ_.damping     = damp;

        // --- Simulate ---
        sPitch_.update(fdt); sYaw_.update(fdt); sRoll_.update(fdt);
        sX_.update(fdt); sY_.update(fdt); sZ_.update(fdt);

        // --- Idle Sway & Breath ---
        idleT_ += fdt;
        float idleFactor = in.moving ? 0.f : 1.f;
        idleBlend_ += (idleFactor - idleBlend_) * fdt * 2.f;

        float sw = static_cast<float>(t_.swayIntensity) * 0.01745329f;
        float sf = static_cast<float>(t_.swayFrequency);
        float swayP = (std::sin(idleT_ * sf * 6.2832f) + 0.5f * std::sin(idleT_ * sf * 10.68f)) * sw * idleBlend_;
        float swayY = (std::sin(idleT_ * sf * 5.0265f) + 0.5f * std::cos(idleT_ * sf * 3.1416f)) * sw * idleBlend_;
        float swayR = std::sin(idleT_ * sf * 5.6549f) * sw * 0.5f * idleBlend_;
        float breath = std::sin(idleT_ * 1.5f) * 0.004f * idleBlend_;

        // --- Output ---
        Output o;
        o.pitchRad = sPitch_.value + swayP;
        o.yawRad   = sYaw_.value;
        o.rollRad  = sRoll_.value + swayR;
        o.offX     = sX_.value;
        o.offY     = sY_.value + breath;
        o.offZ     = sZ_.value;
        return o;
    }

private:
    Tuning t_;
    Spring sPitch_, sYaw_, sRoll_, sX_, sY_, sZ_;
    float dist_ = 0.f;
    int steps_ = 0;
    float idleT_ = 0.f, idleBlend_ = 0.f;
};

} // namespace bodycam
