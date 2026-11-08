#include "pure_uav/detection/input_quantization.hpp"

#include <algorithm>
#include <cmath>

namespace pure_uav::detection {
bool buildInputLut(InputElement element, InputQuantization quantization, float scale, int zero_point, int fixed_point, std::array<uint8_t, 256>& lut, std::string& error) {
    if (element == InputElement::fp32) {
        error.clear();
        return true;
    }
    if (element != InputElement::uint8 && element != InputElement::int8) {
        error = "Unsupported integer input type";
        return false;
    }
    if (quantization == InputQuantization::none && element == InputElement::int8) {
        error = "INT8 input without quantization is unsupported";
        return false;
    }
    if (quantization == InputQuantization::asymmetric && (!std::isfinite(scale) || scale <= 0)) {
        error = "Input quantization scale must be finite and positive";
        return false;
    }
    if (quantization == InputQuantization::dynamic_fixed && (fixed_point < -31 || fixed_point > 31)) {
        error = "Input dynamic fixed point must be in [-31,31]";
        return false;
    }
    if (quantization != InputQuantization::none && quantization != InputQuantization::asymmetric && quantization != InputQuantization::dynamic_fixed) {
        error = "Unsupported input quantization format";
        return false;
    }
    std::array<uint8_t, 256> next{};
    const bool signed_input = element == InputElement::int8;
    for (int pixel = 0; pixel < 256; ++pixel) {
        if (quantization == InputQuantization::none) {
            next[static_cast<size_t>(pixel)] = static_cast<uint8_t>(pixel);
            continue;
        }
        const float normalized = static_cast<float>(pixel) / 255.0f;
        const float value = quantization == InputQuantization::asymmetric ? normalized / scale + static_cast<float>(zero_point) : std::ldexp(normalized, fixed_point);
        const float bounded = std::clamp(std::round(value), signed_input ? -128.0f : 0.0f, signed_input ? 127.0f : 255.0f);
        next[static_cast<size_t>(pixel)] = static_cast<uint8_t>(static_cast<int>(bounded));
    }
    lut = next;
    error.clear();
    return true;
}
}  // namespace pure_uav::detection
