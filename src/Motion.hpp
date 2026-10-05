#pragma once
#include <algorithm>
#include <cmath>
#include "Spring.hpp"

namespace bodycam {

struct Tuning {
    int version = 2;
    bool enabled = true;

    // Step / Footstep Impacts (The core of the Bodycam feel)
    double stepDistance = 2.2;      // Virtual blocks between steps
    double stepImpactY = -0.06;     // Vertical drop shock per step
    double stepImpactX = 0.03;      // Lateral sway shock per step
    double stepImpactPitch = 2.5;   // Degrees of "nod" shock per step
    double stepImpactYaw = 1.2;     // Degrees of "twist" shock per step
    double stepImpactRoll = 0.8;    // Micro-tilt shock per step

    // Spring Physics (Inertia & Lag)
    double pitchStiffness = 60.0;
    double pitchDamping = 9.0;
    double yawStiffness = 80.0;     // Higher = less turn lag
    double yawDamping = 12.0;
    double posStiffness = 50.0;
    double posDamping = 8.0;

    // Movement influence (Leaning)
    double movePitchLean = -0.04;   // Pitch down slightly when sprinting forward
    double strafeRollLean = 0.03;   // Roll slightly when strafing
    double strafeXLean = 0.02;      // Shift camera laterally when strafing
    double turnYawLag = 0.06;       // How much the camera trails behind the crosshair

    // Idle Sway & Breathing
    double idleSwayIntensity = 0.5;
    double idleBreathIntensity = 0.004;
    
    // Touch input mapping
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
    void setTuning(const Tuning &t) { t_ = t; }

    Output update(double dt, const Input &in, double aspect) {
        dt = std::clamp(dt, 0.0, 0.1);
        float fdt = static_cast<float>(dt);

        // 1. Virtual Velocity (Estimating distance traveled for footsteps)
        float speed = std::sqrt(in.forward * in.forward + in.strafe * in.strafe);
        if (in.moving) {
            distanceTraveled_ += speed * fdt * 4.5f; 
        }

        // 2. Step Impacts (Triggering physical shocks)
        if (distanceTraveled_ > t_.stepDistance) {
            distanceTraveled_ -= t_.stepDistance;
            stepCount_++;
            float side = (stepCount_ % 2 == 0) ? 1.0f : -1.0f; // Alternate left/right foot
            
            springY_.impulse(static_cast<float>(t_.stepImpactY) * 20.0f);
            springX_.impulse(static_cast<float>(t_.stepImpactX) * side * 20.0f);
            springPitch_.impulse(static_cast<float>(t_.stepImpactPitch) * 0.01745f * 15.0f);
            springYaw_.impulse(static_cast<float>(t_.stepImpactYaw) * side * 0.01745f * 15.0f);
            springRoll_.impulse(static_cast<float>(t_.stepImpactRoll) * side * 0.01745f * 10.0f);
        }

        // 3. Targets based on input (Leaning / Lag)
        springPitch_.target = in.moving ? static_cast<float>(in.forward) * static_cast<float>(t_.movePitchLean) : 0.f;
        springRoll_.target  = in.moving ? static_cast<float>(in.strafe) * static_cast<float>(t_.strafeRollLean) : 0.f;
        springX_.target     = in.moving ? static_cast<float>(in.strafe) * static_cast<float>(t_.strafeXLean) : 0.f;

        // Yaw lag: camera trails behind the crosshair when turning
        float turnRad = static_cast<float>(in.turnSpeed) * 1.5f; 
        springYaw_.target = -turnRad * static_cast<float>(t_.turnYawLag);

        // 4. Update Spring Parameters from Config
        springPitch_.stiffness = t_.pitchStiffness; springPitch_.damping = t_.pitchDamping;
        springYaw_.stiffness   = t_.yawStiffness;   springYaw_.damping   = t_.yawDamping;
        springRoll_.stiffness  = 60.f;              springRoll_.damping  = 8.f;
        springX_.stiffness     = t_.posStiffness;   springX_.damping     = t_.posDamping;
        springY_.stiffness     = t_.posStiffness;   springY_.damping     = t_.posDamping;
        springZ_.stiffness     = t_.posStiffness;   springZ_.damping     = t_.posDamping;

        // 5. Step Springs (Physics Simulation)
        springPitch_.update(fdt);
        springYaw_.update(fdt);
        springRoll_.update(fdt);
        springX_.update(fdt);
        springY_.update(fdt);
        springZ_.update(fdt);

        // 6. Idle Sway & Breathing (Micro-jitters when standing still)
        idleTime_ += fdt;
        float idleFactor = in.moving ? 0.0f : 1.0f;
        idleBlend_ += (idleFactor - idleBlend_) * fdt * 2.0f; // Smooth fade

        float swayPitch = std::sin(idleTime_ * 1.1f) * 0.002f + std::sin(idleTime_ * 0.7f) * 0.001f;
        float swayYaw   = std::sin(idleTime_ * 0.8f) * 0.002f + std::cos(idleTime_ * 0.5f) * 0.001f;
        float swayRoll  = std::sin(idleTime_ * 0.9f) * 0.001f;
        float breathY   = std::sin(idleTime_ * 1.5f) * 0.003f;

        // 7. Compile Output
        Output o;
        o.pitchRad = springPitch_.value + swayPitch * idleBlend_ * static_cast<float>(t_.idleSwayIntensity);
        o.yawRad   = springYaw_.value   + swayYaw   * idleBlend_ * static_cast<float>(t_.idleSwayIntensity);
        o.rollRad  = springRoll_.value  + swayRoll  * idleBlend_ * static_cast<float>(t_.idleSwayIntensity);
        
        o.offX = springX_.value;
        o.offY = springY_.value + breathY * idleBlend_ * static_cast<float>(t_.idleBreathIntensity);
        o.offZ = springZ_.value;

        return o;
    }

private:
    Tuning t_;
    Spring springPitch_, springYaw_, springRoll_;
    Spring springX_, springY_, springZ_;
    
    float distanceTraveled_ = 0.f;
    int stepCount_ = 0;
    float idleTime_ = 0.f;
    float idleBlend_ = 0.f;
};

} // namespace bodycam
