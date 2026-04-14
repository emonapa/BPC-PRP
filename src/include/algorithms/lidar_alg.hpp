#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>

namespace algorithms {

    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFilter {
    public:
        // Zúžené zorné pole pro bludiště (klapky na oči proti rohům)
        static constexpr float SECTOR_HALF_WIDTH_DEG = 25.0f;

        LidarFilter() = default;

        LidarFilterResults apply_filter(const std::vector<float>& points, float angle_start, float angle_end) {

            std::vector<float> left;
            std::vector<float> right;
            std::vector<float> front;
            std::vector<float> back;

            auto angle_step = (angle_end - angle_start) / points.size();
            const float HW_RAD = SECTOR_HALF_WIDTH_DEG * M_PI / 180.0f;

            for (size_t i = 0; i < points.size(); ++i) {
                float angle = angle_start + i * angle_step;
                float value = points[i];

                if (std::isinf(value) || std::isnan(value) || value <= 0.01f) {
                    continue;
                }

                float norm_angle = std::atan2(std::sin(angle), std::cos(angle));

                // ZADEK: 0 stupňů (LiDAR má nulu namířenou dozadu)
                if (std::abs(norm_angle) <= HW_RAD) {
                    back.push_back(value);
                }
                // VPRAVO: +90 stupňů
                else if (std::abs(norm_angle - (M_PI / 2.0f)) <= HW_RAD) {
                    right.push_back(value);
                }
                // VLEVO: -90 stupňů
                else if (std::abs(norm_angle + (M_PI / 2.0f)) <= HW_RAD) {
                    left.push_back(value);
                }
                // PŘEDEK: 180 nebo -180 stupňů
                else if (std::abs(norm_angle) >= (M_PI - HW_RAD)) {
                    front.push_back(value);
                }
            }

            auto avg = [](const std::vector<float>& v) {
                if (v.empty()) return std::numeric_limits<float>::infinity();
                return std::accumulate(v.begin(), v.end(), 0.0f) / v.size();
            };

            return LidarFilterResults{
                .front = avg(front),
                .back  = avg(back),
                .left  = avg(left),
                .right = avg(right),
            };
        }
    };
}
