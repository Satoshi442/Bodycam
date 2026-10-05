#pragma once
#include <cmath>
#include <algorithm>

namespace bodycam {

// A critically-damped spring system. This gives the camera "weight" and inertia.
// Instead of instantly moving to a target, it trails behind, overshoots, and settles.
struct Spring {
    float value = 0.f;
    float velocity = 0.f;
    float target = 0.f;
    float stiffness = 100.f;
    float damping = 10.f;

    void update(float dt) {
        if (dt <= 0.f) return;
        dt = std::min(dt, 0.05f); // Prevent physics explosion on lag spikes
        float force = stiffness * (target - value) - damping * velocity;
        velocity += force * dt;
        value += velocity * dt;
    }
    
    // Apply a sudden physical shock (like a footstep hitting the ground)
    void impulse(float v) { 
        velocity += v; 
    }
    
    void reset() {
        value = 0.f;
        velocity = 0.f;
        target = 0.f;
    }
};

} // namespace bodycam
