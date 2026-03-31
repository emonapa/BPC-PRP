#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>
#define FRONT_ANGLE 90
#define RIGHT_ANGLE
namespace algorithms {

    // Structure to store filtered average distances in key directions
    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFilter {
    public:
        LidarFilter() = default;

        LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI/8;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;
                float value = points[i];
                if (std::isinf(value) || value <= 0.01) {
                    continue;
                }
                // TODO: Skip invalid (infinite) readings

                  if (std::abs(angle) < angle_range) {
                    back.push_back(value);
                    }
                    // ✅ LEFT (~ +90°)
                    else if (angle > (M_PI/2 - angle_range) && angle < (M_PI/2 + angle_range)) {
                        right.push_back(value);
                    }
                    // ✅ RIGHT (~ -90°)
                    else if (angle > (-M_PI/2 - angle_range) && angle < (-M_PI/2 + angle_range)) {
                        left.push_back(value);
                    }
                    // ✅ BACK (~ ±180°)
                    else if (std::abs(std::abs(angle) - M_PI) < angle_range) {
                        front.push_back(value);
                    }
                // TODO: Sort the value into the correct directional bin based on angle
                
            }
            auto avg = [](const std::vector<float>& v) {
                if (v.empty()) return std::numeric_limits<float>::infinity();
                return std::accumulate(v.begin(), v.end(), 0.0f) / v.size();
            };
            // TODO: Return the average of each sector (basic mean filter)
            return LidarFilterResults{
                .front = avg(front),
                .back = avg(back),
                .left = avg(left),
                .right = avg(right),
            };
        }
    };
}