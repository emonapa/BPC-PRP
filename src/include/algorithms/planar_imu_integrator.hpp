#pragma once

#include <vector>
#include <numeric>

namespace algorithms {

    class PlanarImuIntegrator {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        // Aktualizuje odhad úhlu na základě rychlosti a uběhlého času (dt)
        void update(float gyro_z, double dt) {
            theta_ += (gyro_z - gyro_offset_) * dt;
        }

        // Vypočítá chybu senzoru z naměřených vzorků, když robot stál na místě
        void setCalibration(const std::vector<float>& gyro_samples) {
            if (gyro_samples.empty()) return;
            float sum = std::accumulate(gyro_samples.begin(), gyro_samples.end(), 0.0f);
            gyro_offset_ = sum / gyro_samples.size();
        }

        [[nodiscard]] float getYaw() const {
            return theta_;
        }

        void reset() {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

    private:
        float theta_;       // Zintegrovaný úhel v radiánech
        float gyro_offset_; // Chyba (bias) gyroskopu
    };
}
