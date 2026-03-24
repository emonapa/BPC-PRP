#pragma once

#include <iostream>
#include <chrono>

namespace algorithms {

    class Pid {
    public:
        Pid(float kp, float ki, float kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    float step(float error, float dt) {
            if (dt <= 0.0f) return 0.0f;

            // 1. Integrál počítáme jen při rozumné chybě (Deadband)
            if (std::abs(error) > 0.001f) {
                integral_ += error * dt;
            }

            // 2. Tvrdý limit (Saturace) - nikdy nenulovat, jen zastavit
            float max_integral = 50.0f; 
            integral_ = std::clamp(integral_, -max_integral, max_integral);

            float derivative = (error - prev_error_) / dt;
            float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

            prev_error_ = error;
            return output;
        }

        void reset() {
            prev_error_ = 0;
            integral_ = 0;
        }

    private:
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float integral_;
    };
}
