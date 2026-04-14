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
        LidarFilter() = default;

        LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {
            std::vector<float> left, right, front, back;

            // KONŠTANTA PRE PRESNOSŤ: 
            // Pôvodných M_PI/8 (22.5°) nahradíme cca 5° (M_PI/36).
            // Tým získaš priemer z viacerých bodov (stabilitu), 
            // ale len z tých, ktoré sú takmer kolmo na stenu.
            constexpr float angle_range = M_PI / 36.0f; 

            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;
                float value = points[i];

                if (!std::isfinite(value) || value <= 0.01f) continue;

                // 1. BACK (0 rad)
                if (std::abs(angle) < angle_range) {
                    back.push_back(value);
                }
                // 2. RIGHT (PI/2 rad)
                else if (std::abs(angle - M_PI/2.0f) < angle_range) {
                    right.push_back(value);
                }
                // 3. LEFT (-PI/2 rad)
                else if (std::abs(angle + M_PI/2.0f) < angle_range) {
                    left.push_back(value);
                }
                // 4. FRONT (PI rad)
                else if (std::abs(std::abs(angle) - M_PI) < angle_range) {
                    front.push_back(value);
                }
            }

            auto avg = [](const std::vector<float>& v) {
                if (v.empty()) return 0.5f; // Ak nič nevidí, vrátime bezpečnú vzdialenosť
                return std::accumulate(v.begin(), v.end(), 0.0f) / v.size();
            };

            return LidarFilterResults{
                .front = avg(front),
                .back = avg(back),
                .left = avg(left),
                .right = avg(right),
            };
        }
    };
}