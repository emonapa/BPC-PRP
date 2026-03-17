#include <algorithm>
#include <cstdint>

enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};

class LineEstimator {
public:
    // Konštanty pre kalibráciu (v reálnej praxi by mali byť konfigurovateľné)
    static constexpr uint16_t MIN_ADC = 35;  // Biela plocha
    static constexpr uint16_t MAX_ADC_LEFT = 550; // Čierna čiara
    static constexpr uint16_t MAX_ADC_RIGHT = 350; // Čierna čiara
    static constexpr float SENSOR_DISTANCE_M = 0.027; // Vzdialenosť senzorov 4cm

    static float normalize(uint16_t raw, bool is_right) {
    float max_adc = is_right ? static_cast<float>(MAX_ADC_RIGHT) : static_cast<float>(MAX_ADC_LEFT);
    float min_adc = static_cast<float>(MIN_ADC); // Ideálně is_right ? MIN_ADC_RIGHT : MIN_ADC_LEFT

    float raw_f = static_cast<float>(raw);

    float val = (raw_f - min_adc) / (max_adc - min_adc);
    return std::clamp(val, 0.0f, 1.0f);
    }

    static DiscreteLinePose estimate_discrete(uint16_t left_val, uint16_t right_val) {
        float l = normalize(left_val, 0);
        float r = normalize(right_val, 1);
        float threshold = 0.3f;

        if (l > threshold && r > threshold) return DiscreteLinePose::LineBoth;
        if (l > threshold) return DiscreteLinePose::LineOnLeft;
        if (r > threshold) return DiscreteLinePose::LineOnRight;
        return DiscreteLinePose::LineNone;
    }

    static float estimate_continuous(uint16_t left_val, uint16_t right_val) {
        float l = normalize(left_val, 0);
        float r = normalize(right_val, 1);

        // Diferenciálny výpočet: (L - R) nám dá smer odchýlky
        // Výsledok je v metroch od stredu
        return (l - r) * (SENSOR_DISTANCE_M / 2.0f);
    }
};
