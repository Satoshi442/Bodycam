#pragma once
#include <cmath>
#include <algorithm>

namespace bodycam {

struct Spring {
    float value = 0.f;
    float velocity = 0.f;
    float target = 0.f;
    float stiffness = 100.f;
    float damping = 10.f;

    void update(float dt) {
        if (dt <= 0.f) return;
        dt = std::min(dt, 0.05f);
        float force = stiffness * (target - value) - damping * velocity;
        velocity += force * dt;
        value += velocity * dt;
    }

    void impulse(float v) { velocity += v; }
    void reset() { value = 0.f; velocity = 0.f; target = 0.f; }
};

} // namespace bodycam
